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
// Binary frame classification happens by inspecting the first byte tag.
// Current protocol uses raw float32/int16 arrays; the first-byte tag is
// a forward-compatible extension — for M1 we just treat binary as FFT bins
// when IQ is not subscribed, else as audio PCM. The existing sdr.js does
// the same implicit routing; we preserve it.
export type BinaryFrameKind = 'fft' | 'audio';
