#!/usr/bin/env python3
"""
FM Radio Transmitter via PlutoSDR (no GnuRadio dependency)

Streams a WAV file as WBFM on a specified frequency using pyadi-iio.
Processes audio in chunks to avoid memory issues with long files.

Matches the working GnuRadio flowgraph parameters:
  - Quadrature rate: 2,048,000 SPS
  - Peak deviation: 75 kHz
  - Pre-emphasis: 75µs at quadrature rate
  - PlutoSDR BW: 2,048,000

Usage:
  python3 fm_tx_pluto.py [--freq 106.8e6] [--wav Ezhilam.wav] [--gain -10]

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import sys
import time
import wave

import numpy as np

try:
    import adi
except ImportError:
    print("ERROR: pip install pyadi-iio"); sys.exit(1)

PLUTO_RATE = 2048000   # Match GnuRadio quadrature_rate
FM_DEVIATION = 75000   # 75 kHz WBFM standard
PREEMPH_TAU = 75e-6    # 75µs US pre-emphasis
CHUNK_SEC = 10         # Process in 10-second chunks for memory efficiency


def load_wav_mono(path):
    """Load WAV, convert to mono float32 [-1, 1]."""
    with wave.open(path, 'r') as w:
        ch = w.getnchannels()
        rate = w.getframerate()
        n = w.getnframes()
        raw = w.readframes(n)
        dtype = np.int16 if w.getsampwidth() == 2 else np.int32
        data = np.frombuffer(raw, dtype=dtype).astype(np.float32) / np.iinfo(dtype).max
        if ch == 2:
            data = (data[0::2] + data[1::2]) * 0.5
    print(f"[WAV] {path}")
    print(f"      {ch}ch {rate}Hz {n} frames ({n/rate:.1f}s)")
    return data, rate


def resample(audio, src_rate, dst_rate):
    """Resample audio from src_rate to dst_rate using linear interpolation."""
    if src_rate == dst_rate:
        return audio
    ratio = dst_rate / src_rate
    n_out = int(len(audio) * ratio)
    indices = np.arange(n_out) * (src_rate / dst_rate)
    lo = np.clip(indices.astype(int), 0, len(audio) - 1)
    hi = np.clip(lo + 1, 0, len(audio) - 1)
    frac = (indices - lo).astype(np.float32)
    return audio[lo] * (1 - frac) + audio[hi] * frac


def preemphasis(audio, tau, fs):
    """Apply FM pre-emphasis using scipy.signal.lfilter or numpy fallback."""
    try:
        from scipy.signal import lfilter
        # Bilinear transform of H(s) = 1 + s*tau
        w = 2 * fs
        a1 = (w * tau - 1) / (w * tau + 1)
        b0 = w * tau / (w * tau + 1)
        return lfilter([b0, b0], [1.0, a1], audio).astype(np.float32)
    except ImportError:
        # Numpy vectorized approximation
        alpha = np.exp(-1.0 / (tau * fs))
        out = np.empty_like(audio)
        out[0] = audio[0]
        out[1:] = audio[1:] - alpha * audio[:-1]
        return out


def fm_modulate_chunk(audio_chunk, phase_acc, rf_rate, deviation):
    """FM modulate a chunk of audio, returns (IQ, new_phase_accumulator)."""
    # Instantaneous frequency offset = deviation * audio
    freq_offset = deviation * audio_chunk
    # Phase is integral of frequency
    phase_inc = 2.0 * np.pi * freq_offset / rf_rate
    phase = phase_acc + np.cumsum(phase_inc)
    # Keep phase bounded to prevent float precision loss
    phase_acc_new = phase[-1] % (2 * np.pi)
    phase = phase % (2 * np.pi)
    iq = np.exp(1j * phase).astype(np.complex64)
    return iq, phase_acc_new


def main():
    parser = argparse.ArgumentParser(description="FM TX via PlutoSDR")
    parser.add_argument("--freq", type=float, default=106.8e6,
                        help="TX frequency in Hz (default: 106.8 MHz)")
    parser.add_argument("--wav", default="/opt1/workshop/MAPCON/Demos/1 - FM Radio RX-TX/Ezhilam.wav")
    parser.add_argument("--gain", type=int, default=-10,
                        help="TX gain in dB (-89 to 0)")
    parser.add_argument("--pluto-ip", default="192.168.2.1")
    parser.add_argument("--loop-sec", type=float, default=5,
                        help="Seconds of audio for cyclic buffer (default: 5, max ~8 for PlutoSDR DMA)")
    args = parser.parse_args()

    # Load audio
    audio, wav_rate = load_wav_mono(args.wav)

    # Trim to loop_sec for manageable cyclic buffer
    max_samples = int(args.loop_sec * wav_rate)
    if len(audio) > max_samples:
        audio = audio[:max_samples]
        print(f"[DSP] Trimmed to {args.loop_sec:.0f}s ({max_samples} samples)")

    # Resample to quadrature rate
    print(f"[DSP] Resampling {wav_rate}→{PLUTO_RATE}Hz...")
    audio_q = resample(audio, wav_rate, PLUTO_RATE)
    print(f"      {len(audio_q)} samples at quadrature rate")

    # Pre-emphasis at quadrature rate (matching GnuRadio: analog.fm_preemph)
    print(f"[DSP] Pre-emphasis (75µs at {PLUTO_RATE}Hz)...")
    audio_q = preemphasis(audio_q, PREEMPH_TAU, PLUTO_RATE)

    # Normalize
    peak = np.max(np.abs(audio_q))
    if peak > 0:
        audio_q *= 0.8 / peak

    # FM modulate
    print(f"[DSP] FM modulating: deviation={FM_DEVIATION/1000:.0f}kHz...")
    iq, _ = fm_modulate_chunk(audio_q, 0.0, PLUTO_RATE, FM_DEVIATION)

    # Phase-continuous looping: correct end phase to match start (0)
    phase_end = np.angle(iq[-1])
    correction = np.exp(-1j * np.linspace(0, phase_end, len(iq)))
    iq *= correction

    print(f"[DSP] IQ buffer: {len(iq)} samples ({len(iq)/PLUTO_RATE:.1f}s, "
          f"{len(iq)*8/1024/1024:.0f}MB)")

    # Connect PlutoSDR
    freq_mhz = args.freq / 1e6
    print(f"\n[SDR] Connecting to PlutoSDR at {args.pluto_ip}...")
    sdr = adi.Pluto(f"ip:{args.pluto_ip}")
    # Destroy any stale TX buffer first
    try:
        sdr.tx_destroy_buffer()
    except Exception:
        pass

    # Configure BEFORE creating buffer
    sdr.sample_rate = PLUTO_RATE
    sdr.tx_rf_bandwidth = PLUTO_RATE
    sdr.tx_lo = int(args.freq)
    sdr.tx_hardwaregain_chan0 = args.gain
    sdr.tx_cyclic_buffer = True

    # Verify LO is set correctly
    import iio
    ctx = iio.Context(f"ip:{args.pluto_ip}")
    phy = ctx.find_device('ad9361-phy')
    actual_lo = int(phy.find_channel('altvoltage1', True).attrs['frequency'].value)
    print(f"[SDR] TX LO verified: {actual_lo/1e6:.3f} MHz")
    if abs(actual_lo - int(args.freq)) > 1000:
        print(f"[SDR] WARNING: LO mismatch! Expected {args.freq/1e6:.3f} MHz")

    # Transmit
    sdr.tx(iq * 2**14)

    # Re-verify LO after tx() in case it got reset
    actual_lo2 = int(phy.find_channel('altvoltage1', True).attrs['frequency'].value)
    if abs(actual_lo2 - int(args.freq)) > 1000:
        print(f"[SDR] LO shifted after tx()! Forcing back to {args.freq/1e6:.3f} MHz")
        sdr.tx_lo = int(args.freq)

    print(f"\n[TX] ♪ Broadcasting WBFM on {freq_mhz:.1f} MHz")
    print(f"     Gain: {args.gain} dB")
    print(f"     Audio: {args.loop_sec:.0f}s loop ({len(iq)/PLUTO_RATE:.1f}s IQ)")
    print(f"     Rate: {PLUTO_RATE/1e6:.3f} MSPS")
    print(f"     Deviation: ±{FM_DEVIATION/1000:.0f} kHz")
    print(f"     Press Ctrl+C to stop\n")

    try:
        t0 = time.time()
        while True:
            elapsed = int(time.time() - t0)
            m, s = divmod(elapsed, 60)
            sys.stdout.write(f"\r[TX] {freq_mhz:.1f} MHz | {m:02d}:{s:02d} elapsed")
            sys.stdout.flush()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n[TX] Stopping...")
        sdr.tx_destroy_buffer()
        print("[TX] Done.")


if __name__ == "__main__":
    main()
