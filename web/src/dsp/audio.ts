// Browser-side audio pipeline. Owns the AudioContext, the AudioWorklet that
// plays queued PCM, and the demod+DSP chain. Consumes narrowband IQ blocks
// via `feed()`; produces playable audio on the default output.

import type { DspSettings } from '@/types';
import { decodeIq } from './iq';
import { makeDemod, type Demodulator } from './demod';
import { BiquadNotch, applyGain, rmsDb } from './chain';
import { RdsDecoder, type RdsState } from './rds';

const WORKLET_SRC = `
class PcmFeederProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.queue = [];
    this.queueLen = 0;
    this.MAX = 48000; // ~1s at 48kHz — drop if more
    this.port.onmessage = (ev) => {
      if (ev.data === 'flush') { this.queue = []; this.queueLen = 0; return; }
      const buf = ev.data;
      if (this.queueLen + buf.length > this.MAX) {
        // backpressure: drop oldest
        while (this.queue.length && this.queueLen + buf.length > this.MAX) {
          this.queueLen -= this.queue.shift().length;
        }
      }
      this.queue.push(buf);
      this.queueLen += buf.length;
    };
  }
  process(_inputs, outputs) {
    const out = outputs[0][0];
    if (!out) return true;
    let i = 0;
    while (i < out.length) {
      if (!this.queue.length) { out.fill(0, i); break; }
      const head = this.queue[0];
      const n = Math.min(head.length, out.length - i);
      out.set(head.subarray(0, n), i);
      if (n === head.length) this.queue.shift();
      else this.queue[0] = head.subarray(n);
      this.queueLen -= n;
      i += n;
    }
    return true;
  }
}
registerProcessor('pcm-feeder', PcmFeederProcessor);
`;

export class AudioPipeline {
  private ctx: AudioContext | null = null;
  private node: AudioWorkletNode | null = null;
  private demod: Demodulator | null = null;
  private notch = new BiquadNotch();
  private settings: DspSettings;
  private iqRate = 0;
  private lastNotchFreq = 0;
  private rds: RdsDecoder | null = null;
  private onRds: ((s: RdsState) => void) | null = null;

  constructor(initial: DspSettings) {
    this.settings = { ...initial };
  }

  async start() {
    if (this.ctx) return;
    const ctx = new AudioContext();
    const blob = new Blob([WORKLET_SRC], { type: 'application/javascript' });
    const url = URL.createObjectURL(blob);
    await ctx.audioWorklet.addModule(url);
    URL.revokeObjectURL(url);
    const node = new AudioWorkletNode(ctx, 'pcm-feeder', {
      numberOfInputs: 0,
      numberOfOutputs: 1,
      outputChannelCount: [1],
    });
    node.connect(ctx.destination);
    this.ctx = ctx;
    this.node = node;
  }

  async stop() {
    this.node?.disconnect();
    this.node = null;
    await this.ctx?.close();
    this.ctx = null;
    this.demod = null;
    this.rds = null;
  }

  setRdsHandler(h: ((s: RdsState) => void) | null) { this.onRds = h; }

  update(s: Partial<DspSettings>) {
    const prev = this.settings;
    this.settings = { ...prev, ...s };
    // Any of these invalidate demod state — rebuild on next block.
    if (
      (s.mode && s.mode !== prev.mode) ||
      (s.deemphasisUs !== undefined && s.deemphasisUs !== prev.deemphasisUs)
    ) this.demod = null;
  }

  /** Called when the server announces a new IQ stream rate. */
  setIqRate(rate: number) {
    if (rate !== this.iqRate) {
      this.iqRate = rate;
      this.demod = null;
      this.rds = null;  // sample-rate-dependent BPF coefficients
      this.node?.port.postMessage('flush');
    }
  }

  feed(buf: ArrayBuffer) {
    if (!this.node || !this.iqRate) return;
    if (!this.demod) {
      this.demod = makeDemod(this.settings.mode, this.iqRate, {
        deemphasisUs: this.settings.deemphasisUs,
      });
    }
    const iq = decodeIq(buf);
    const pcm = this.demod.process(iq);

    // RDS taps the demod's pre-decimation MPX (WBFM only). Needs ≥ 200 kHz
    // MPX rate to clear the 57 kHz subcarrier; the DDC output rate controls
    // this implicitly via the bandwidth the user selects.
    if (this.settings.mode === 'WBFM' && this.onRds) {
      const mpx = this.demod.lastMpx?.();
      if (mpx && this.iqRate >= 150000) {
        if (!this.rds) this.rds = new RdsDecoder(this.iqRate);
        this.rds.feed(mpx);
        this.onRds(this.rds.snapshot());
      }
    }

    if (!pcm.length) return;

    // Squelch: mute whole block when audio RMS sits below threshold
    const level = rmsDb(pcm);
    if (level < this.settings.squelchDb) {
      pcm.fill(0);
    } else {
      if (this.settings.notchHz > 0) {
        if (this.settings.notchHz !== this.lastNotchFreq) {
          this.notch.setup(this.settings.notchHz, this.demod.outputRate);
          this.lastNotchFreq = this.settings.notchHz;
        }
        this.notch.process(pcm);
      }
      applyGain(pcm, this.settings.volume);
    }
    this.node.port.postMessage(pcm);
  }
}
