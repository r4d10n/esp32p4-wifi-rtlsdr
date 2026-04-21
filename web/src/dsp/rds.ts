// RDS (Radio Data System) decoder.
//
// Input : WBFM MPX baseband at the DDC output rate (≥ 200 kHz recommended).
// Output: structured group data surfaced as RdsEvents — PI, PS, PTY, RT.
//
// The pipeline is:
//   MPX → 57 kHz BPF → mix by 57 kHz LO → LPF → symbol clock recovery
//       → differential Manchester decode → bit stream → block sync (offset
//         word A/B/C/C'/D correlation) → group parser
//
// This is a pragmatic clean-room implementation. Two known simplifications,
// flagged for future work:
//   1. The 57 kHz LO is fixed-frequency — a Costas / pilot-locked PLL would
//      give better performance on weak or off-tuned signals.
//   2. Block sync uses pure offset-word matching against the CRC syndrome,
//      not full (26,16) Meggitt error correction — single-bit errors inside
//      a block are currently dropped rather than corrected.
// Both can be layered in later without changing the external surface.

export interface RdsState {
  pi: number;               // 16-bit program identification code, 0 = unknown
  ps: string;               // 8-char program service (station) name
  pty: number;              // 5-bit program type, 0 = unknown
  rt: string;               // up to 64-char radio text
  tp: boolean;              // traffic program flag
  lastUpdate: number;       // performance.now() ms of last successful group
  groupCount: number;
  blockCount: number;       // cumulative successfully-synced blocks
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
    tp: false, lastUpdate: 0, groupCount: 0, blockCount: 0,
  };

  // Local oscillator phase for 57 kHz mixer
  private lo = 0;
  private loStep: number;

  // Biquad BPF @ 57 kHz (Q ≈ 6) — one-pass on MPX input
  private bpf: Biquad;
  // LPF after mixer (~2 kHz, enough for 1187.5 baud)
  private lpfI: Biquad; private lpfQ: Biquad;

  // Symbol clock recovery (simple Gardner-like approach)
  private samplesPerSymbol: number;
  private symPhase = 0;   // fractional-sample accumulator
  private lastSym = 0;    // previous symbol for differential decoding
  private lastBit = 0;    // previous differential bit (for Manchester)
  private halfBitToggle = false;

  // Bit-window for block sync — 26 bits sliding
  private window: number = 0;
  private windowLen = 0;

  // Current in-flight group: 4 blocks of 16 info bits
  private groupBlocks: number[] = [];
  private expecting: 'A' | 'B' | 'C' | 'D' = 'A';

  constructor(sampleRate: number) {
    this.loStep = 2 * Math.PI * 57000 / sampleRate;
    this.bpf = Biquad.bandpass(57000, sampleRate, 6);
    this.lpfI = Biquad.lowpass(2400, sampleRate);
    this.lpfQ = Biquad.lowpass(2400, sampleRate);
    this.samplesPerSymbol = sampleRate / (BIT_RATE * 2); // Manchester → 2x baud
  }

  snapshot(): RdsState { return { ...this.state, ps: this.state.ps, rt: this.state.rt }; }

  feed(mpx: Float32Array) {
    for (let k = 0; k < mpx.length; k++) {
      const bp = this.bpf.step(mpx[k]);
      // Mix to baseband: bp is real → split into I/Q with cos/sin LO
      const c = Math.cos(this.lo), s = Math.sin(this.lo);
      this.lo += this.loStep;
      if (this.lo > 2 * Math.PI) this.lo -= 2 * Math.PI;
      const iVal = this.lpfI.step(bp * c);
      const qVal = this.lpfQ.step(bp * s);
      // BPSK → use the axis with larger energy. Without a PLL we don't know
      // which axis holds the data a priori; pick per-block based on magnitude.
      // For simplicity, use I for now — quality depends on LO phase alignment.
      void qVal;
      this.processSample(iVal);
    }
  }

  private processSample(x: number) {
    this.symPhase += 1;
    if (this.symPhase >= this.samplesPerSymbol) {
      this.symPhase -= this.samplesPerSymbol;
      const sym = x > 0 ? 1 : 0;
      // Manchester: a data bit is encoded as two half-bit opposite transitions.
      // Accumulate two half-bit symbols, XOR to recover the differential bit.
      if (!this.halfBitToggle) {
        this.lastSym = sym;
        this.halfBitToggle = true;
        return;
      }
      this.halfBitToggle = false;
      const diff = this.lastSym ^ sym;  // 1 if transition = data '1'
      // Differential decode against previous bit
      const bit = diff ^ this.lastBit;
      this.lastBit = diff;
      this.pushBit(bit);
    }
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
