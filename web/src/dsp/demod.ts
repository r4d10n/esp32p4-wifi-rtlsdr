import type { DemodMode } from '@/types';
import type { IqBlock } from './iq';

export interface Demodulator {
  readonly inputRate: number;
  readonly outputRate: number;
  process(iq: IqBlock): Float32Array;
  reset(): void;
  /** WBFM only: last block's pre-decimation MPX baseband (input-rate samples).
   *  Null for every other mode. Consumers like RDS decoders tap this. */
  lastMpx?(): Float32Array | null;
}

export interface DemodOpts {
  deemphasisUs?: number;   // WBFM / NFM; 0 disables
}

export function makeDemod(mode: DemodMode, inputRate: number, opts: DemodOpts = {}): Demodulator {
  switch (mode) {
    case 'WBFM': return new FmDemod(inputRate, opts.deemphasisUs ?? 50);
    case 'NFM':  return new FmDemod(inputRate, opts.deemphasisUs ?? 50, /* narrowband */ true);
    case 'AM':   return new AmDemod(inputRate);
    case 'DSB':  return new DsbDemod(inputRate);
    case 'USB':  return new SsbDemod(inputRate, +1);
    case 'LSB':  return new SsbDemod(inputRate, -1);
    case 'CW':   return new SsbDemod(inputRate, +1, /* bfo */ 700);
  }
}

// ── Shared helpers ──────────────────────────────────────────────────

function pickDecim(inputRate: number, target = 48000): number {
  return Math.max(1, Math.round(inputRate / target));
}

// 1-pole de-emphasis (exponential LPF with tau)
class Deemphasis {
  private state = 0;
  private alpha: number;
  constructor(sampleRate: number, tauUs: number) {
    if (tauUs <= 0) { this.alpha = 1; return; }
    const tau = tauUs * 1e-6;
    const dt = 1 / sampleRate;
    this.alpha = dt / (tau + dt);
  }
  apply(x: Float32Array) {
    if (this.alpha === 1) return;
    let s = this.state;
    const a = this.alpha;
    for (let i = 0; i < x.length; i++) { s += a * (x[i] - s); x[i] = s; }
    this.state = s;
  }
  reset() { this.state = 0; }
}

// Dead-simple first-order DC blocker (R = 0.995 at 48 kHz ≈ 38 Hz corner)
class DcBlock {
  private xPrev = 0; private yPrev = 0;
  private readonly R: number;
  constructor(sampleRate: number, cornerHz = 40) {
    this.R = Math.exp(-2 * Math.PI * cornerHz / sampleRate);
  }
  apply(x: Float32Array) {
    let xp = this.xPrev, yp = this.yPrev;
    const R = this.R;
    for (let i = 0; i < x.length; i++) {
      const xi = x[i];
      const yi = xi - xp + R * yp;
      x[i] = yi; xp = xi; yp = yi;
    }
    this.xPrev = xp; this.yPrev = yp;
  }
  reset() { this.xPrev = 0; this.yPrev = 0; }
}

// ── FM (WBFM/NFM) ───────────────────────────────────────────────────
// Phase-differentiation discriminator + integer-ratio decimation. The only
// practical difference between WBFM and NFM here is expected deviation and
// the caller's bandwidth choice — the server DDC filter already limits
// incoming BW; the discriminator itself is BW-agnostic.
class FmDemod implements Demodulator {
  readonly inputRate: number;
  readonly outputRate: number;
  private decim: number;
  private prevI = 0; private prevQ = 0;
  private accIdx = 0; private accSum = 0;
  private deemp: Deemphasis;
  private narrow: boolean;
  private mpxBuf: Float32Array | null = null;  // pre-decim discriminator output

  constructor(inputRate: number, deemphasisUs: number, narrow = false) {
    this.inputRate = inputRate;
    this.decim = pickDecim(inputRate);
    this.outputRate = inputRate / this.decim;
    this.deemp = new Deemphasis(this.outputRate, deemphasisUs);
    this.narrow = narrow;
  }

  reset() {
    this.prevI = 0; this.prevQ = 0; this.accIdx = 0; this.accSum = 0;
    this.deemp.reset(); this.mpxBuf = null;
  }

  lastMpx(): Float32Array | null { return this.narrow ? null : this.mpxBuf; }

  process({ i, q }: IqBlock): Float32Array {
    const n = i.length;
    const out = new Float32Array(Math.ceil(n / this.decim) + 1);
    // For WBFM we keep the full-rate discriminator output around so an RDS
    // decoder (running off the same pipeline block) can tap it without
    // re-running the expensive atan2.
    const mpx = !this.narrow ? new Float32Array(n) : null;
    let outIdx = 0;
    let pi = this.prevI, pq = this.prevQ;
    let acc = this.accSum, ai = this.accIdx;
    const d = this.decim;
    const gain = (this.narrow ? 3.0 : 1.0) / Math.PI;
    for (let k = 0; k < n; k++) {
      const ii = i[k], qq = q[k];
      const re = ii * pi + qq * pq;
      const im = qq * pi - ii * pq;
      const phase = Math.atan2(im, re) * gain;
      if (mpx) mpx[k] = phase;
      acc += phase; ai++;
      pi = ii; pq = qq;
      if (ai === d) { out[outIdx++] = acc / d; ai = 0; acc = 0; }
    }
    this.prevI = pi; this.prevQ = pq;
    this.accIdx = ai; this.accSum = acc;
    this.mpxBuf = mpx;
    const slice = out.subarray(0, outIdx);
    this.deemp.apply(slice);
    return slice;
  }
}

// ── AM (envelope) ───────────────────────────────────────────────────
class AmDemod implements Demodulator {
  readonly inputRate: number;
  readonly outputRate: number;
  private decim: number;
  private accIdx = 0; private accSum = 0;
  private dc: DcBlock;

  constructor(inputRate: number) {
    this.inputRate = inputRate;
    this.decim = pickDecim(inputRate);
    this.outputRate = inputRate / this.decim;
    this.dc = new DcBlock(this.outputRate);
  }
  reset() { this.accIdx = 0; this.accSum = 0; this.dc.reset(); }

  process({ i, q }: IqBlock): Float32Array {
    const n = i.length;
    const out = new Float32Array(Math.ceil(n / this.decim) + 1);
    let outIdx = 0;
    let acc = this.accSum, ai = this.accIdx;
    const d = this.decim;
    for (let k = 0; k < n; k++) {
      const mag = Math.sqrt(i[k] * i[k] + q[k] * q[k]);
      acc += mag; ai++;
      if (ai === d) { out[outIdx++] = acc / d; ai = 0; acc = 0; }
    }
    this.accIdx = ai; this.accSum = acc;
    const slice = out.subarray(0, outIdx);
    this.dc.apply(slice);
    return slice;
  }
}

// ── DSB (real part of baseband) ─────────────────────────────────────
class DsbDemod implements Demodulator {
  readonly inputRate: number;
  readonly outputRate: number;
  private decim: number;
  private accIdx = 0; private accSum = 0;
  constructor(inputRate: number) {
    this.inputRate = inputRate;
    this.decim = pickDecim(inputRate);
    this.outputRate = inputRate / this.decim;
  }
  reset() { this.accIdx = 0; this.accSum = 0; }
  process({ i }: IqBlock): Float32Array {
    const n = i.length;
    const out = new Float32Array(Math.ceil(n / this.decim) + 1);
    let outIdx = 0;
    let acc = this.accSum, ai = this.accIdx;
    const d = this.decim;
    for (let k = 0; k < n; k++) {
      acc += i[k]; ai++;
      if (ai === d) { out[outIdx++] = acc / d; ai = 0; acc = 0; }
    }
    this.accIdx = ai; this.accSum = acc;
    return out.subarray(0, outIdx);
  }
}

// ── SSB / CW (BFO + real projection) ────────────────────────────────
// Classic software-BFO approach: tune the DDC slightly off the signal, mix
// the complex baseband with a local oscillator at the BFO offset, take the
// real part. Sideband selection flips the LO rotation direction. For CW we
// use a fixed 700 Hz BFO tone so the Morse pitch is audible.
class SsbDemod implements Demodulator {
  readonly inputRate: number;
  readonly outputRate: number;
  private decim: number;
  private phase = 0;
  private dphase: number;
  private accIdx = 0; private accSum = 0;

  constructor(inputRate: number, sideband: 1 | -1, bfoHz = 1500) {
    this.inputRate = inputRate;
    this.decim = pickDecim(inputRate);
    this.outputRate = inputRate / this.decim;
    this.dphase = sideband * 2 * Math.PI * bfoHz / inputRate;
  }
  reset() { this.phase = 0; this.accIdx = 0; this.accSum = 0; }

  process({ i, q }: IqBlock): Float32Array {
    const n = i.length;
    const out = new Float32Array(Math.ceil(n / this.decim) + 1);
    let outIdx = 0;
    let ph = this.phase;
    const dp = this.dphase;
    let acc = this.accSum, ai = this.accIdx;
    const d = this.decim;
    for (let k = 0; k < n; k++) {
      // Re{ (I+jQ) * (cos(ph) + j sin(ph)) } = I*cos - Q*sin
      const c = Math.cos(ph), s = Math.sin(ph);
      const sample = i[k] * c - q[k] * s;
      acc += sample; ai++;
      ph += dp;
      if (ai === d) { out[outIdx++] = acc / d; ai = 0; acc = 0; }
    }
    // Keep phase bounded
    this.phase = ph - Math.floor(ph / (2 * Math.PI)) * 2 * Math.PI;
    this.accIdx = ai; this.accSum = acc;
    return out.subarray(0, outIdx);
  }
}
