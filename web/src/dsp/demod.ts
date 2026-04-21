import type { DemodMode } from '@/types';
import type { IqBlock } from './iq';

// Demod dispatcher. M2 implements WBFM (plus a passthrough stub for the other
// modes so the UI works end-to-end); M3 fills in NFM/AM/USB/LSB/DSB/CW.

export interface Demodulator {
  readonly inputRate: number;   // expected IQ sample rate (Hz)
  readonly outputRate: number;  // audio sample rate after internal decimation
  /** Return mono PCM (Float32, -1..1). Internal state is preserved across calls. */
  process(iq: IqBlock): Float32Array;
  /** Optional: stereo L/R (WBFM). Returns null if mode is mono-only. */
  processStereo?(iq: IqBlock): { l: Float32Array; r: Float32Array } | null;
  reset(): void;
}

export function makeDemod(mode: DemodMode, inputRate: number): Demodulator {
  switch (mode) {
    case 'WBFM': return new WbfmDemod(inputRate);
    default:
      // M3 will replace with real implementations. For M2 we route everything
      // through a WBFM demod so the audio path is exercisable end-to-end; a
      // mode switch in the UI just changes the label until M3 lands.
      return new WbfmDemod(inputRate);
  }
}

// ── WBFM ────────────────────────────────────────────────────────────
// Straightforward FM discriminator via phase differentiation, followed by a
// simple integer-ratio decimator down to the AudioContext rate.

class WbfmDemod implements Demodulator {
  readonly inputRate: number;
  readonly outputRate: number;
  private decim: number;
  private prevI = 0;
  private prevQ = 0;
  private accumIdx = 0;
  private accumSum = 0;
  // Simple 1-pole de-emphasis @ 50us (EU). Coefficient pre-computed against
  // the output rate.
  private deempState = 0;
  private deempAlpha: number;

  constructor(inputRate: number) {
    this.inputRate = inputRate;
    // Pick an integer decimation that lands near 48 kHz.
    const target = 48000;
    this.decim = Math.max(1, Math.round(inputRate / target));
    this.outputRate = inputRate / this.decim;
    const tau = 50e-6;
    const dt = 1 / this.outputRate;
    this.deempAlpha = dt / (tau + dt);
  }

  reset() {
    this.prevI = 0; this.prevQ = 0;
    this.accumIdx = 0; this.accumSum = 0;
    this.deempState = 0;
  }

  process(iq: IqBlock): Float32Array {
    const { i, q } = iq;
    const n = i.length;
    const outCap = Math.ceil(n / this.decim) + 1;
    const out = new Float32Array(outCap);
    let outIdx = 0;
    let pi = this.prevI, pq = this.prevQ;
    let acc = this.accumSum;
    let ai = this.accumIdx;
    const d = this.decim;
    // Gain: discriminator output is ~[-π, π]; scale to ±1 ≈ peak deviation.
    const gain = 1.0 / Math.PI;
    let deemp = this.deempState;
    const a = this.deempAlpha;
    for (let k = 0; k < n; k++) {
      const ii = i[k], qq = q[k];
      // atan2(Im(z_k * conj(z_{k-1})), Re(z_k * conj(z_{k-1})))
      const re = ii * pi + qq * pq;
      const im = qq * pi - ii * pq;
      const phase = Math.atan2(im, re) * gain;
      acc += phase;
      ai++;
      pi = ii; pq = qq;
      if (ai === d) {
        const sample = acc / d;
        // 1-pole de-emphasis
        deemp += a * (sample - deemp);
        out[outIdx++] = deemp;
        ai = 0;
        acc = 0;
      }
    }
    this.prevI = pi; this.prevQ = pq;
    this.accumIdx = ai; this.accumSum = acc;
    this.deempState = deemp;
    return out.subarray(0, outIdx);
  }
}
