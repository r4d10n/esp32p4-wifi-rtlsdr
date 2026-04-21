// Post-demod DSP chain: squelch → notch → volume. Noise reduction is left as
// a hook for M3.

export class BiquadNotch {
  private b0 = 1; private b1 = 0; private b2 = 0;
  private a1 = 0; private a2 = 0;
  private z1 = 0; private z2 = 0;

  setup(freqHz: number, sampleRate: number, q = 30) {
    const w0 = 2 * Math.PI * freqHz / sampleRate;
    const alpha = Math.sin(w0) / (2 * q);
    const cosw = Math.cos(w0);
    const a0 = 1 + alpha;
    this.b0 = 1 / a0;
    this.b1 = (-2 * cosw) / a0;
    this.b2 = 1 / a0;
    this.a1 = (-2 * cosw) / a0;
    this.a2 = (1 - alpha) / a0;
    this.z1 = 0; this.z2 = 0;
  }

  process(x: Float32Array) {
    const { b0, b1, b2, a1, a2 } = this;
    let z1 = this.z1, z2 = this.z2;
    for (let i = 0; i < x.length; i++) {
      const xi = x[i];
      const y = b0 * xi + z1;
      z1 = b1 * xi - a1 * y + z2;
      z2 = b2 * xi - a2 * y;
      x[i] = y;
    }
    this.z1 = z1; this.z2 = z2;
  }
}

export function rmsDb(x: Float32Array): number {
  let s = 0;
  for (let i = 0; i < x.length; i++) s += x[i] * x[i];
  const rms = Math.sqrt(s / x.length);
  if (rms <= 1e-9) return -120;
  return 20 * Math.log10(rms);
}

export function applyGain(x: Float32Array, g: number) {
  if (g === 1) return;
  for (let i = 0; i < x.length; i++) x[i] *= g;
}
