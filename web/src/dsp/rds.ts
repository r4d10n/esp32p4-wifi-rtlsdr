// RDS (Radio Data System) decoder.
//
// Input : WBFM MPX baseband at the DDC output rate (≥ 200 kHz recommended).
// Output: structured group data surfaced as RdsEvents — PI, PS, PTY, RT.
//
// The pipeline is:
//   MPX → 19 kHz BPF → PLL locked to pilot → derive 57 kHz as 3rd harmonic
//       → 57 kHz BPF + mix by pilot-locked LO (I & Q) → LPF
//       → axis-select (auto-pick stronger I/Q projection)
//       → symbol clock recovery → differential Manchester decode → bit stream
//       → 26-bit sliding window + CRC-10 syndrome vs. offset words A/B/C/C'/D
//       → group parser
//
// The 57 kHz subcarrier is defined by the RDS spec as phase-locked to the
// 19 kHz stereo pilot (3rd harmonic), so the only correct way to demodulate
// is to lock a PLL to the pilot and derive 57 kHz from it. A free-running
// oscillator drifts and the data bit energy leaks across I/Q unpredictably.
//
// One remaining simplification: block sync uses pure offset-word syndrome
// matching, not full (26,16) Meggitt error correction — single-bit errors
// inside a block are dropped rather than corrected. Fine for strong signals;
// can be upgraded later without touching the public surface.

export interface RdsState {
  pi: number;               // 16-bit program identification code, 0 = unknown
  ps: string;               // 8-char program service (station) name
  pty: number;              // 5-bit program type, 0 = unknown
  rt: string;               // up to 64-char radio text
  tp: boolean;              // traffic program flag
  lastUpdate: number;       // performance.now() ms of last successful group
  groupCount: number;
  blockCount: number;       // cumulative successfully-synced blocks
  pilotLocked: boolean;     // 19 kHz pilot PLL lock status
}

const BIT_RATE = 1187.5;

// Offset words per EN 50067, used to reveal block position after CRC.
// Syndrome = offset XOR when block aligned.
const OFFSET_A  = 0b0011111100;
const OFFSET_B  = 0b0110011000;
const OFFSET_C  = 0b0101101000;
const OFFSET_CP = 0b1101010000;
const OFFSET_D  = 0b0110110100;

function crc10(bits: number): number {
  // Divide the 16 info bits by g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1
  const poly = 0b11011011101; // 0x5B7 in 11-bit
  let shift = bits << 10;
  for (let i = 25; i >= 10; i--) {
    if ((shift >>> i) & 1) shift ^= poly << (i - 10);
  }
  return shift & 0x3ff;
}

export class RdsDecoder {
  private state: RdsState = {
    pi: 0, ps: '        ', pty: 0, rt: '                                                                ',
    tp: false, lastUpdate: 0, groupCount: 0, blockCount: 0, pilotLocked: false,
  };

  // Pilot PLL (19 kHz). The 57 kHz LO is derived from it as the 3rd harmonic.
  private pll: PilotPll;

  // Biquad BPF @ 57 kHz (Q ≈ 40) — narrow enough to reject audio + pilot
  private bpf: Biquad;
  // LPF after mixer (~2.4 kHz, enough for 1187.5 baud)
  private lpfI: Biquad; private lpfQ: Biquad;

  // Slow-leaking per-axis energy estimates for axis selection. The PLL has
  // a π-rad phase ambiguity (it can lock in or out of phase with the pilot),
  // and encoder conventions vary on which quadrature carries the data, so
  // we pick the axis with stronger signal.
  private eI = 0; private eQ = 0;

  // Symbol clock recovery — simple 2 samples-per-symbol Manchester sampler.
  private samplesPerHalfSymbol: number;
  private sampleIdx = 0;
  private halfBitToggle = false;
  private lastHalf = 0;
  private lastBit = 0;

  // Bit-window for block sync — 26 bits sliding
  private window: number = 0;
  private windowLen = 0;

  private groupBlocks: number[] = [];
  private expecting: 'A' | 'B' | 'C' | 'D' = 'A';

  constructor(sampleRate: number) {
    this.pll = new PilotPll(sampleRate, 19000);
    this.bpf = Biquad.bandpass(57000, sampleRate, 40);
    this.lpfI = Biquad.lowpass(2400, sampleRate);
    this.lpfQ = Biquad.lowpass(2400, sampleRate);
    this.samplesPerHalfSymbol = sampleRate / (BIT_RATE * 2);
  }

  snapshot(): RdsState { return { ...this.state, ps: this.state.ps, rt: this.state.rt }; }

  feed(mpx: Float32Array) {
    for (let k = 0; k < mpx.length; k++) {
      const x = mpx[k];
      // Step the pilot PLL first so the 57 kHz LO is phase-aligned with the
      // pilot on every sample.
      const [c57, s57] = this.pll.step(x);

      const bp = this.bpf.step(x);
      const iVal = this.lpfI.step(bp * c57);
      const qVal = this.lpfQ.step(bp * s57);

      // Slow-leaking per-axis energy estimate (~0.1 s time constant).
      const a = 1e-4;
      this.eI += a * (iVal * iVal - this.eI);
      this.eQ += a * (qVal * qVal - this.eQ);
      const sig = this.eI >= this.eQ ? iVal : -qVal;

      this.sampleIdx++;
      if (this.sampleIdx >= this.samplesPerHalfSymbol) {
        this.sampleIdx -= this.samplesPerHalfSymbol;
        this.processHalfSymbol(sig);
      }
    }
    this.state.pilotLocked = this.pll.locked;
  }

  private processHalfSymbol(x: number) {
    const half = x > 0 ? 1 : 0;
    // Manchester: each data bit = two opposite half-symbols. Pair them and
    // XOR to recover the differentially-encoded bit.
    if (!this.halfBitToggle) {
      this.lastHalf = half;
      this.halfBitToggle = true;
      return;
    }
    this.halfBitToggle = false;
    const diff = this.lastHalf ^ half;  // data '1' = transition
    const bit = diff ^ this.lastBit;    // differential decode
    this.lastBit = diff;
    this.pushBit(bit);
  }

  private pushBit(b: number) {
    this.window = ((this.window << 1) | b) & 0x3ffffff; // 26 bits
    if (this.windowLen < 26) { this.windowLen++; return; }
    this.state.blockCount++;

    const info = (this.window >>> 10) & 0xffff;
    const check = this.window & 0x3ff;
    const syndrome = crc10(info) ^ check;

    const match =
      syndrome === OFFSET_A ? 'A' :
      syndrome === OFFSET_B ? 'B' :
      syndrome === OFFSET_C ? 'C' :
      syndrome === OFFSET_CP ? 'C' :
      syndrome === OFFSET_D ? 'D' : null;

    if (!match) return;

    // Sync handling — only accept blocks in order
    if (this.expecting === 'A' && match !== 'A') return;
    if (this.expecting !== match) {
      // Out-of-order block → abandon and resync
      this.groupBlocks = [];
      this.expecting = 'A';
      if (match === 'A') this.pushInfo('A', info);
      return;
    }
    this.pushInfo(match, info);
  }

  private pushInfo(kind: 'A' | 'B' | 'C' | 'D', info: number) {
    this.groupBlocks.push(info);
    this.expecting = kind === 'A' ? 'B' : kind === 'B' ? 'C' : kind === 'C' ? 'D' : 'A';
    if (kind === 'D' && this.groupBlocks.length === 4) {
      this.parseGroup(this.groupBlocks);
      this.groupBlocks = [];
      this.expecting = 'A';
      this.state.groupCount++;
      this.state.lastUpdate = performance.now();
    }
  }

  private parseGroup(b: number[]) {
    const [a, bB, c, d] = b;
    this.state.pi = a;
    const groupType = (bB >> 12) & 0x0f;
    const gBVar = (bB >> 11) & 0x01;  // 0 = version A, 1 = version B
    this.state.pty = (bB >> 5) & 0x1f;
    this.state.tp = !!((bB >> 10) & 0x01);

    if (groupType === 0) {
      // 0A/0B: PS name. Address is the low 2 bits of block B; block D has two PS chars.
      const addr = bB & 0x03;
      const ch1 = (d >> 8) & 0xff;
      const ch2 = d & 0xff;
      this.writePsChar(addr * 2,     ch1);
      this.writePsChar(addr * 2 + 1, ch2);
      void gBVar; void c; // block C is alt freq / PI repeat — unused here
    } else if (groupType === 2) {
      // 2A/2B: RadioText. Address is low 4 bits of block B.
      const addr = bB & 0x0f;
      if (gBVar === 0) {
        // 2A — 4 chars in blocks C and D
        this.writeRtChar(addr * 4,     (c >> 8) & 0xff);
        this.writeRtChar(addr * 4 + 1, c & 0xff);
        this.writeRtChar(addr * 4 + 2, (d >> 8) & 0xff);
        this.writeRtChar(addr * 4 + 3, d & 0xff);
      } else {
        // 2B — 2 chars in block D
        this.writeRtChar(addr * 2,     (d >> 8) & 0xff);
        this.writeRtChar(addr * 2 + 1, d & 0xff);
      }
    }
  }

  private writePsChar(i: number, ch: number) {
    if (i < 0 || i >= 8) return;
    const c = (ch >= 32 && ch < 127) ? String.fromCharCode(ch) : ' ';
    const arr = this.state.ps.split('');
    arr[i] = c;
    this.state.ps = arr.join('');
  }

  private writeRtChar(i: number, ch: number) {
    if (i < 0 || i >= 64) return;
    if (ch === 0x0d) {
      // RT terminator — pad rest with spaces
      const head = this.state.rt.slice(0, i);
      this.state.rt = head.padEnd(64, ' ');
      return;
    }
    const c = (ch >= 32 && ch < 127) ? String.fromCharCode(ch) : ' ';
    const arr = this.state.rt.split('');
    arr[i] = c;
    this.state.rt = arr.join('');
  }
}

// ── Pilot PLL ───────────────────────────────────────────────────────
// Second-order PLL locked to the 19 kHz stereo pilot. On every sample it
// produces cos/sin of the current VCO phase; callers that want the 57 kHz
// subcarrier phase use the 3rd-harmonic (triple-angle) identities.
//
// Loop design: damping ζ = 0.707 (critically-damped-ish), natural frequency
// ω_n chosen for ~10 Hz loop bandwidth. Converted to per-sample gains via
//   α = 2ζω_n / fs
//   β = ω_n² / fs²
// The nominal center frequency is baked into the integrator so the NCO
// freewheels at 19 kHz when no pilot is present; the PLL only steers
// deviations from that.
class PilotPll {
  private phase = 0;
  private freqDev = 0;             // phase error accumulator (β integrator)
  private readonly nominal: number; // nominal phase increment per sample
  private readonly alpha: number;
  private readonly beta: number;
  private readonly bpf: Biquad;    // tight BPF around 19 kHz for the phase detector
  private lockEma = 0;             // slow-leaking cos(phase_error) estimate
  locked = false;

  constructor(sampleRate: number, pilotHz: number, loopBwHz = 10) {
    this.nominal = 2 * Math.PI * pilotHz / sampleRate;
    const zeta = Math.SQRT1_2;
    const wn = 2 * Math.PI * loopBwHz / Math.sqrt(1 - 2 * zeta * zeta + Math.sqrt(2 + 4 * zeta * zeta * (zeta * zeta - 1)));
    this.alpha = 2 * zeta * wn / sampleRate;
    this.beta = (wn / sampleRate) ** 2;
    this.bpf = Biquad.bandpass(pilotHz, sampleRate, 200);
  }

  /** Advance one sample; returns [cos(3φ), sin(3φ)] — the 57 kHz LO. */
  step(mpxSample: number): [number, number] {
    const pilot = this.bpf.step(mpxSample);
    const c = Math.cos(this.phase);
    const s = Math.sin(this.phase);
    // Phase detector: multiply pilot by the in-phase VCO reference. For a
    // pure tone cos(ωt) and VCO cos(ωt + φ_err), the low-passed product is
    // 0.5 cos(φ_err); simpler to use the quadrature product which is
    // 0.5 sin(φ_err) ≈ φ_err for small errors.
    const err = pilot * (-s);

    // Advance loop
    this.freqDev += this.beta * err;
    this.phase += this.nominal + this.freqDev + this.alpha * err;
    // Keep phase bounded to avoid float precision decay over long runs
    if (this.phase > Math.PI) this.phase -= 2 * Math.PI;
    else if (this.phase < -Math.PI) this.phase += 2 * Math.PI;

    // Lock indicator: when the in-phase projection is consistently positive
    // and sizable, we're locked. EMA τ ≈ 0.1 s worth of samples.
    this.lockEma += 2e-4 * (pilot * c - this.lockEma);
    this.locked = Math.abs(this.lockEma) > 0.02;

    // Triple-angle identities — cheaper and phase-exact vs. a second NCO:
    //   cos(3φ) = 4c³ − 3c
    //   sin(3φ) = 3s − 4s³
    const c3 = 4 * c * c * c - 3 * c;
    const s3 = 3 * s - 4 * s * s * s;
    return [c3, s3];
  }
}

// ── Tiny biquad helper (kept local to avoid circular deps with chain.ts) ─
class Biquad {
  private b0 = 1; private b1 = 0; private b2 = 0;
  private a1 = 0; private a2 = 0;
  private z1 = 0; private z2 = 0;
  static bandpass(freqHz: number, sampleRate: number, q: number): Biquad {
    const b = new Biquad();
    const w0 = 2 * Math.PI * freqHz / sampleRate;
    const alpha = Math.sin(w0) / (2 * q);
    const cosw = Math.cos(w0);
    const a0 = 1 + alpha;
    b.b0 = alpha / a0;
    b.b1 = 0;
    b.b2 = -alpha / a0;
    b.a1 = -2 * cosw / a0;
    b.a2 = (1 - alpha) / a0;
    return b;
  }
  static lowpass(freqHz: number, sampleRate: number, q = 0.707): Biquad {
    const b = new Biquad();
    const w0 = 2 * Math.PI * freqHz / sampleRate;
    const alpha = Math.sin(w0) / (2 * q);
    const cosw = Math.cos(w0);
    const a0 = 1 + alpha;
    b.b0 = (1 - cosw) / 2 / a0;
    b.b1 = (1 - cosw) / a0;
    b.b2 = (1 - cosw) / 2 / a0;
    b.a1 = -2 * cosw / a0;
    b.a2 = (1 - alpha) / a0;
    return b;
  }
  step(x: number): number {
    const y = this.b0 * x + this.z1;
    this.z1 = this.b1 * x - this.a1 * y + this.z2;
    this.z2 = this.b2 * x - this.a2 * y;
    return y;
  }
}
