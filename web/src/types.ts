// Protocol types mirrored by hand from components/websdr/websdr.c.
// When the C side adds a new cmd/type, update this file in the same commit.

// ── Browser → ESP (text JSON) ───────────────────────────────────────
export type ClientCmd =
  | { cmd: 'freq'; value: number }
  | { cmd: 'gain'; value: number }                // tuner gain tenths-dB, or 0 = auto
  | { cmd: 'rate' | 'sample_rate'; value: number }
  | { cmd: 'fft_size'; value: number }
  | { cmd: 'db_range'; min: number; max: number }
  | { cmd: 'subscribe_iq'; offset: number; bw: number }
  | { cmd: 'unsubscribe_iq' };

// ── ESP → Browser (text JSON) ───────────────────────────────────────
export type ServerMsg =
  | { type: 'info'; freq: number; rate: number; gain: number; [k: string]: unknown }
  | { type: 'config'; fft_size: number; sample_rate: number; [k: string]: unknown }
  | { type: 'freq'; value: number }
  | { type: 'iq_start'; offset: number; bw: number; rate: number }
  | { type: 'iq_stop' };

// ── ESP → Browser (binary) ──────────────────────────────────────────
// Binary routing is implicit: the IQ subscription state tells us the payload.
// - No IQ subscription: payload is FFT power spectrum as uint8 (0..255),
//   length == fft_size, mapped via db_range to the dB scale.
// - IQ subscribed: payload is narrowband uint8 interleaved IQ (I0,Q0,I1,Q1,…)
//   at the DDC output rate (reported in the `iq_start` message).
//   The browser demodulates this stream — C only channelizes.
export type BinaryFrameKind = 'fft' | 'iq';

// ── Demodulation ────────────────────────────────────────────────────
export type DemodMode = 'WBFM' | 'NFM' | 'AM' | 'USB' | 'LSB' | 'DSB' | 'CW';

export interface DspSettings {
  mode: DemodMode;
  bwHz: number;        // channel bandwidth (passed to DDC via subscribe_iq)
  squelchDb: number;   // mute threshold in dB (RMS below = muted); -120 = off
  noiseReduction: number;  // 0..1, spectral smoothing amount
  notchHz: number;     // 0 = off, else tone frequency to notch
  deemphasisUs: number; // WBFM only: 50 or 75 microseconds, 0 = off
  stereo: boolean;     // WBFM only
  volume: number;      // 0..1
}
