#!/usr/bin/env python3
"""
DTMF RF Loopback Test: PlutoSDR TX → ESP32-P4 WebSDR RX

Transmits FM-modulated DTMF tones via PlutoSDR (192.168.2.1),
receives via WebSDR WebSocket, demodulates FM, and detects
DTMF using Goertzel algorithm. Iterates across frequency bands
and optimizes until detection is pristine.

Requirements:
  pip install pyadi-iio websocket-client numpy

Usage:
  python test_dtmf_loopback.py [--pluto-ip 192.168.2.1] [--sdr-ip 192.168.1.232]

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import sys
import threading
import time
import wave
from collections import deque
from dataclasses import dataclass, field

import numpy as np

try:
    import adi
except ImportError:
    print("ERROR: pip install pyadi-iio")
    sys.exit(1)

try:
    import websocket
except ImportError:
    print("ERROR: pip install websocket-client")
    sys.exit(1)

# ─────────────────────── DTMF Constants ───────────────────────

DTMF_LOW  = [697, 770, 852, 941]
DTMF_HIGH = [1209, 1336, 1477, 1633]

DTMF_MAP = {
    '1': (697, 1209), '2': (697, 1336), '3': (697, 1477), 'A': (697, 1633),
    '4': (770, 1209), '5': (770, 1336), '6': (770, 1477), 'B': (770, 1633),
    '7': (852, 1209), '8': (852, 1336), '9': (852, 1477), 'C': (852, 1633),
    '*': (941, 1209), '0': (941, 1336), '#': (941, 1477), 'D': (941, 1633),
}

DTMF_SEQUENCE = "123A456B789C*0#D"

# Test frequencies (MHz) — PlutoSDR TX range 47-6000, RTL-SDR RX range 24-1766
TEST_FREQS_MHZ = [88.0, 108.0, 145.0, 174.0, 290.0, 435.0, 915.0, 1090.0, 1420.0, 1700.0]

# FM parameters
FM_DEVIATION = 5000       # 5 kHz NFM
AUDIO_RATE = 48000
TONE_DURATION = 0.15      # 150ms per tone
SILENCE_DURATION = 0.10   # 100ms between tones
PLUTO_SAMPLE_RATE = 1000000  # 1 MSPS


# ─────────────────────── Goertzel Algorithm ───────────────────────

def goertzel_mag(samples, freq, sample_rate):
    """Compute magnitude of a single DFT bin using Goertzel algorithm.
    O(N) per frequency — much faster than FFT for sparse frequency detection."""
    N = len(samples)
    k = int(0.5 + N * freq / sample_rate)
    w = 2.0 * np.pi * k / N
    coeff = 2.0 * np.cos(w)
    s0 = s1 = s2 = 0.0
    for x in samples:
        s0 = x + coeff * s1 - s2
        s2 = s1
        s1 = s0
    power = s1 * s1 + s2 * s2 - coeff * s1 * s2
    return np.sqrt(abs(power)) / N


def goertzel_mag_vectorized(samples, freq, sample_rate):
    """Vectorized Goertzel for better performance on longer blocks."""
    N = len(samples)
    k = int(0.5 + N * freq / sample_rate)
    w = 2.0 * np.pi * k / N
    coeff = 2.0 * np.cos(w)
    s1, s2 = 0.0, 0.0
    for x in samples:
        s0 = float(x) + coeff * s1 - s2
        s2 = s1
        s1 = s0
    power = s1 * s1 + s2 * s2 - coeff * s1 * s2
    return np.sqrt(abs(power)) / N


def detect_dtmf(audio, sample_rate, block_ms=40):
    """Detect DTMF tones in audio using Goertzel.
    Returns list of (time_ms, detected_char, low_freq, high_freq, low_mag, high_mag, snr_db)."""
    block_size = int(sample_rate * block_ms / 1000)
    all_freqs = DTMF_LOW + DTMF_HIGH
    results = []

    for start in range(0, len(audio) - block_size, block_size // 2):  # 50% overlap
        block = audio[start:start + block_size]
        time_ms = start * 1000 / sample_rate

        # Compute Goertzel magnitude for all 8 DTMF frequencies
        mags = {}
        for f in all_freqs:
            mags[f] = goertzel_mag_vectorized(block, f, sample_rate)

        # Find strongest low and high
        low_mags = [(f, mags[f]) for f in DTMF_LOW]
        high_mags = [(f, mags[f]) for f in DTMF_HIGH]

        best_low = max(low_mags, key=lambda x: x[1])
        best_high = max(high_mags, key=lambda x: x[1])

        # SNR: best vs second-best in each group
        low_sorted = sorted(low_mags, key=lambda x: x[1], reverse=True)
        high_sorted = sorted(high_mags, key=lambda x: x[1], reverse=True)

        low_snr = 20 * np.log10(low_sorted[0][1] / max(low_sorted[1][1], 1e-10))
        high_snr = 20 * np.log10(high_sorted[0][1] / max(high_sorted[1][1], 1e-10))

        # Threshold: both must be above noise floor and have good SNR
        noise_floor = np.mean([m for _, m in low_mags + high_mags]) * 0.5
        min_snr = 6.0  # dB

        if (best_low[1] > noise_floor and best_high[1] > noise_floor
                and low_snr > min_snr and high_snr > min_snr):
            # Look up DTMF character
            char = '?'
            for c, (fl, fh) in DTMF_MAP.items():
                if fl == best_low[0] and fh == best_high[0]:
                    char = c
                    break
            snr = min(low_snr, high_snr)
            results.append((time_ms, char, best_low[0], best_high[0],
                            best_low[1], best_high[1], snr))

    return results


def consolidate_detections(detections, min_gap_ms=50, block_ms=30):
    """Merge consecutive detections of same character into single events.
    Returns list of (char, snr, low_mag, high_mag, hit_count, start_ms, duration_ms)."""
    if not detections:
        return []
    merged = []
    prev_char = detections[0][1]
    start_time = detections[0][0]
    prev_time = detections[0][0]
    best_snr = detections[0][6]
    best_low_mag = detections[0][4]
    best_high_mag = detections[0][5]
    count = 1

    for det in detections[1:]:
        t, c, fl, fh, ml, mh, snr = det
        if c == prev_char and (t - prev_time) < 200:
            best_snr = max(best_snr, snr)
            best_low_mag = max(best_low_mag, ml)
            best_high_mag = max(best_high_mag, mh)
            count += 1
            prev_time = t
        else:
            duration = prev_time - start_time + block_ms
            merged.append((prev_char, best_snr, best_low_mag, best_high_mag,
                           count, start_time, duration))
            prev_char = c
            start_time = t
            prev_time = t
            best_snr = snr
            best_low_mag = ml
            best_high_mag = mh
            count = 1

    duration = prev_time - start_time + block_ms
    merged.append((prev_char, best_snr, best_low_mag, best_high_mag,
                   count, start_time, duration))
    return merged


# ─────────────────────── DTMF Tone Generation ───────────────────────

def generate_dtmf_audio(sequence, audio_rate, tone_dur, silence_dur):
    """Generate DTMF audio waveform for a sequence of characters."""
    samples = []
    tone_samples = int(audio_rate * tone_dur)
    silence_samples = int(audio_rate * silence_dur)

    # Leading silence
    samples.extend([0.0] * silence_samples)

    for char in sequence:
        if char not in DTMF_MAP:
            continue
        f_low, f_high = DTMF_MAP[char]
        t = np.arange(tone_samples) / audio_rate

        # Equal amplitude tones with slight windowing to reduce clicks
        window = np.ones(tone_samples)
        ramp = min(int(audio_rate * 0.005), tone_samples // 4)  # 5ms ramp
        window[:ramp] = np.linspace(0, 1, ramp)
        window[-ramp:] = np.linspace(1, 0, ramp)

        tone = (np.sin(2 * np.pi * f_low * t) + np.sin(2 * np.pi * f_high * t)) * 0.4 * window
        samples.extend(tone.tolist())
        samples.extend([0.0] * silence_samples)

    # Trailing silence
    samples.extend([0.0] * silence_samples)
    return np.array(samples, dtype=np.float32)


def fm_modulate(audio, audio_rate, rf_rate, deviation):
    """FM modulate audio to complex baseband IQ at rf_rate."""
    # Resample audio to RF rate
    ratio = rf_rate / audio_rate
    n_out = int(len(audio) * ratio)
    indices = np.arange(n_out) * (audio_rate / rf_rate)
    lo = indices.astype(int)
    frac = indices - lo
    lo = np.clip(lo, 0, len(audio) - 1)
    hi = np.clip(lo + 1, 0, len(audio) - 1)
    audio_resampled = audio[lo] * (1.0 - frac) + audio[hi] * frac

    # FM modulate: phase = integral of frequency deviation
    phase = 2 * np.pi * deviation * np.cumsum(audio_resampled) / rf_rate
    iq = np.exp(1j * phase).astype(np.complex64)
    return iq


# ─────────────────────── PlutoSDR TX ───────────────────────

class PlutoTx:
    """Manage PlutoSDR TX lifecycle — reuse device object across gain changes."""
    def __init__(self, pluto_ip):
        self.sdr = adi.Pluto(f"ip:{pluto_ip}")
        self.active = False

    def start(self, center_freq_hz, iq_data, sample_rate, gain_db=-10):
        if self.active:
            self.stop()
        self.sdr.sample_rate = int(sample_rate)
        self.sdr.tx_rf_bandwidth = int(sample_rate)
        self.sdr.tx_lo = int(center_freq_hz)
        self.sdr.tx_hardwaregain_chan0 = gain_db
        self.sdr.tx_cyclic_buffer = True
        iq_scaled = iq_data * 2**14
        self.sdr.tx(iq_scaled)
        self.active = True

    def stop(self):
        if self.active:
            try:
                self.sdr.tx_destroy_buffer()
            except Exception:
                pass
            self.active = False


# ─────────────────────── WebSDR RX (WebSocket) ───────────────────────

@dataclass
class RxState:
    iq_rate: int = 0
    iq_queue: deque = field(default_factory=lambda: deque(maxlen=500))
    connected: bool = False
    info_received: bool = False
    iq_started: bool = False
    running: bool = True


def demod_fm(iq_uint8, state_prev):
    """FM discriminator returning (audio, new_prev_iq)."""
    iq_float = (iq_uint8.astype(np.float32) - 127.5) / 127.5
    i_s = iq_float[0::2]
    q_s = iq_float[1::2]
    n = len(i_s)
    if n < 2:
        return np.array([], dtype=np.float32), state_prev

    prev_i, prev_q = state_prev
    audio = np.empty(n, dtype=np.float32)
    for k in range(n):
        ci, cq = i_s[k], q_s[k]
        re = ci * prev_i + cq * prev_q
        im = cq * prev_i - ci * prev_q
        audio[k] = np.arctan2(im, re) / np.pi
        prev_i, prev_q = ci, cq

    return audio, (prev_i, prev_q)


def resample_linear(audio, src_rate, dst_rate):
    if src_rate == dst_rate or src_rate <= 0 or len(audio) == 0:
        return audio
    ratio = src_rate / dst_rate
    n_out = int(len(audio) / ratio)
    if n_out <= 0:
        return np.array([], dtype=np.float32)
    indices = np.arange(n_out) * ratio
    lo = np.clip(indices.astype(int), 0, len(audio) - 1)
    hi = np.clip(lo + 1, 0, len(audio) - 1)
    frac = indices - indices.astype(int)
    return (audio[lo] * (1.0 - frac) + audio[hi] * frac).astype(np.float32)


def create_ws_receiver(sdr_ip, sdr_port, rx_state):
    """Create WebSocket receiver for IQ data."""

    def on_message(ws, message):
        if isinstance(message, str):
            try:
                msg = json.loads(message)
            except json.JSONDecodeError:
                return
            if msg.get("type") == "info":
                rx_state.info_received = True
            elif msg.get("type") == "iq_start":
                rx_state.iq_rate = msg.get("rate", 0)
                rx_state.iq_started = True
        elif len(message) >= 2:
            if message[0] == 0x02:
                rx_state.iq_queue.append(np.frombuffer(message[1:], dtype=np.uint8))

    def on_open(ws):
        rx_state.connected = True

    def on_close(ws, *args):
        rx_state.connected = False

    def on_error(ws, err):
        pass

    url = f"ws://{sdr_ip}:{sdr_port}/ws"
    ws = websocket.WebSocketApp(url, on_open=on_open, on_message=on_message,
                                 on_error=on_error, on_close=on_close)
    threading.Thread(target=ws.run_forever, daemon=True).start()
    return ws


def capture_audio(rx_state, duration_s):
    """Capture and demodulate audio from WebSDR for given duration."""
    audio_chunks = []
    prev_iq = (0.0, 0.0)
    t0 = time.time()

    while time.time() - t0 < duration_s and rx_state.running:
        if len(rx_state.iq_queue) == 0:
            time.sleep(0.002)
            continue
        try:
            iq_data = rx_state.iq_queue.popleft()
        except IndexError:
            continue

        audio, prev_iq = demod_fm(iq_data, prev_iq)
        if len(audio) > 0 and rx_state.iq_rate > 0:
            audio = resample_linear(audio, rx_state.iq_rate, AUDIO_RATE)
            if len(audio) > 0:
                audio_chunks.append(audio)

    if audio_chunks:
        return np.concatenate(audio_chunks)
    return np.array([], dtype=np.float32)


# ─────────────────────── Metrics ───────────────────────

def circular_match(expected, detected):
    """Find best circular alignment of detected within repeated expected.
    TX is cyclic, so detected may start at any offset in the sequence."""
    if not detected or not expected:
        return 0, 0
    doubled = expected * 3  # repeat to handle wrap-around
    best_correct = 0
    best_offset = 0
    for offset in range(len(expected)):
        correct = 0
        for i, c in enumerate(detected):
            if offset + i < len(doubled) and doubled[offset + i] == c:
                correct += 1
            else:
                break  # stop at first mismatch for contiguous match
        if correct > best_correct:
            best_correct = correct
            best_offset = offset
    return best_correct, best_offset


def compute_metrics(expected_seq, detected_chars, all_detections):
    """Compute detection quality metrics with circular alignment."""
    detected_str = ''.join([d[0] for d in detected_chars])

    # Circular match — TX is cyclic, capture can start at any point
    correct, offset = circular_match(expected_seq, detected_str)

    # Accuracy = fraction of expected sequence length matched contiguously
    accuracy = min(correct / len(expected_seq) * 100, 100) if expected_seq else 0
    total_detected = len(detected_str)

    # SNR stats
    snrs = [d[1] for d in detected_chars] if detected_chars else [0]
    mags_low = [d[2] for d in detected_chars] if detected_chars else [0]
    mags_high = [d[3] for d in detected_chars] if detected_chars else [0]

    # Timing analysis: tone duration vs expected 150ms, gap vs expected 100ms
    expected_tone_ms = TONE_DURATION * 1000   # 150ms
    expected_gap_ms = SILENCE_DURATION * 1000  # 100ms

    tone_durations = [d[6] for d in detected_chars] if detected_chars else []
    tone_errors = [abs(dur - expected_tone_ms) for dur in tone_durations]

    # Compute gaps between consecutive detections (end of one to start of next)
    gap_errors = []
    gaps = []
    for i in range(1, len(detected_chars)):
        prev_end = detected_chars[i - 1][5] + detected_chars[i - 1][6]
        curr_start = detected_chars[i][5]
        gap = curr_start - prev_end
        if gap >= 0:
            gaps.append(gap)
            gap_errors.append(abs(gap - expected_gap_ms))

    timing_error_ms = float(np.mean(tone_errors)) if tone_errors else 0.0
    gap_error_ms = float(np.mean(gap_errors)) if gap_errors else 0.0

    return {
        'expected': expected_seq,
        'detected': detected_str,
        'accuracy': accuracy,
        'correct': correct,
        'total_expected': len(expected_seq),
        'total_detected': total_detected,
        'snr_min': min(snrs),
        'snr_max': max(snrs),
        'snr_mean': np.mean(snrs),
        'mag_low_mean': np.mean(mags_low),
        'mag_high_mean': np.mean(mags_high),
        'timing_error_ms': timing_error_ms,
        'gap_error_ms': gap_error_ms,
        'tone_durations': tone_durations,
        'gaps': gaps,
    }


def print_metrics(metrics, freq_mhz, iteration):
    """Print formatted metrics table."""
    print(f"\n{'─'*70}")
    print(f"  Freq: {freq_mhz:.1f} MHz | Iteration: {iteration}")
    print(f"{'─'*70}")
    print(f"  Expected:  {metrics['expected']}")
    print(f"  Detected:  {metrics['detected']}")
    print(f"  Accuracy:  {metrics['accuracy']:.0f}% "
          f"({metrics['correct']}/{metrics['total_expected']})")
    print(f"  SNR:       min={metrics['snr_min']:.1f}dB "
          f"max={metrics['snr_max']:.1f}dB "
          f"mean={metrics['snr_mean']:.1f}dB")
    print(f"  Magnitude: low={metrics['mag_low_mean']:.4f} "
          f"high={metrics['mag_high_mean']:.4f}")
    print(f"  Timing:    tone_err={metrics['timing_error_ms']:.1f}ms "
          f"gap_err={metrics['gap_error_ms']:.1f}ms "
          f"(expected: tone={TONE_DURATION*1000:.0f}ms gap={SILENCE_DURATION*1000:.0f}ms)")

    # Per-character detail
    if metrics['detected']:
        print(f"\n  {'Char':>4} {'SNR':>7} {'LowMag':>8} {'HighMag':>8} "
              f"{'Hits':>5} {'Dur ms':>7} {'TErr':>6}")
        print(f"  {'─'*55}")


def print_detection_detail(detected_chars):
    """Print per-character detection detail with timing."""
    expected_tone_ms = TONE_DURATION * 1000
    for char, snr, ml, mh, count, start_ms, dur_ms in detected_chars:
        tone_err = dur_ms - expected_tone_ms
        status = "✓" if snr > 10 else "~" if snr > 6 else "✗"
        print(f"  {status} {char:>3} {snr:>6.1f}dB {ml:>8.4f} {mh:>8.4f} "
              f"{count:>5} {dur_ms:>6.0f} {tone_err:>+5.0f}")


def save_debug_wav(audio, filename):
    """Save audio to WAV for offline analysis."""
    if len(audio) == 0:
        return
    peak = np.max(np.abs(audio))
    if peak > 0:
        audio = audio / peak * 0.9
    int16_data = (audio * 32767).astype(np.int16)
    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(AUDIO_RATE)
        wf.writeframes(int16_data.tobytes())


# ─────────────────────── Main Test Loop ───────────────────────

def connect_websdr(sdr_ip, sdr_port):
    """Create fresh WebSocket connection to WebSDR. Returns (ws, rx_state)."""
    rx_state = RxState()
    ws = create_ws_receiver(sdr_ip, sdr_port, rx_state)
    t0 = time.time()
    while not rx_state.connected and time.time() - t0 < 5:
        time.sleep(0.1)
    return ws, rx_state


def ws_send_safe(ws, rx_state, msg, sdr_ip, sdr_port):
    """Send WS message, reconnect if needed. Returns (ws, rx_state)."""
    try:
        ws.send(msg)
        return ws, rx_state
    except Exception:
        print("[RX] Reconnecting WebSocket...")
        try:
            ws.close()
        except Exception:
            pass
        time.sleep(1)
        ws, rx_state = connect_websdr(sdr_ip, sdr_port)
        if rx_state.connected:
            ws.send(msg)
        return ws, rx_state


def run_test(args):
    print("="*70)
    print("  DTMF RF Loopback Test")
    print(f"  PlutoSDR TX: {args.pluto_ip}")
    print(f"  WebSDR  RX:  {args.sdr_ip}:{args.sdr_port}")
    print("="*70)

    # Generate DTMF audio
    dtmf_audio = generate_dtmf_audio(DTMF_SEQUENCE, AUDIO_RATE,
                                      TONE_DURATION, SILENCE_DURATION)
    total_audio_dur = len(dtmf_audio) / AUDIO_RATE
    print(f"\n[TX] DTMF sequence: {DTMF_SEQUENCE}")
    print(f"[TX] Audio duration: {total_audio_dur:.2f}s "
          f"(tone={TONE_DURATION*1000:.0f}ms gap={SILENCE_DURATION*1000:.0f}ms)")

    # Initialize PlutoSDR (reuse across iterations)
    print(f"\n[TX] Connecting to PlutoSDR at {args.pluto_ip}...")
    try:
        pluto = PlutoTx(args.pluto_ip)
    except Exception as e:
        print(f"[TX ERROR] Cannot connect to PlutoSDR: {e}")
        return
    print(f"[TX] PlutoSDR connected")

    # Connect to WebSDR
    ws, rx_state = connect_websdr(args.sdr_ip, args.sdr_port)
    if not rx_state.connected:
        print("[ERROR] Cannot connect to WebSDR")
        return
    print(f"[RX] WebSDR connected")

    # Test frequency list
    freqs = [f * 1e6 for f in TEST_FREQS_MHZ]
    if args.freq:
        freqs = [args.freq * 1e6]

    best_overall = {'accuracy': 0, 'freq': 0, 'gain': 0, 'snr': 0}
    all_results = []

    for freq_hz in freqs:
        freq_mhz = freq_hz / 1e6
        print(f"\n{'═'*70}")
        print(f"  Testing {freq_mhz:.1f} MHz")
        print(f"{'═'*70}")

        # FM modulate DTMF audio
        iq_data = fm_modulate(dtmf_audio, AUDIO_RATE, PLUTO_SAMPLE_RATE, FM_DEVIATION)

        # Iterate TX gain to find best audio quality
        gains = [args.tx_gain] if args.tx_gain != -10 else [-10, -5, -15, -20, 0]

        for gain_db in gains:
            print(f"\n[TX] Starting PlutoSDR: freq={freq_mhz:.1f}MHz gain={gain_db}dB")

            try:
                pluto.start(freq_hz, iq_data, PLUTO_SAMPLE_RATE, gain_db)
            except Exception as e:
                print(f"[TX ERROR] {e}")
                continue

            time.sleep(0.5)  # Let TX stabilize

            # Tune WebSDR to same frequency (reconnect-safe)
            ws, rx_state = ws_send_safe(ws, rx_state,
                json.dumps({"cmd": "freq", "value": int(freq_hz)}),
                args.sdr_ip, args.sdr_port)
            time.sleep(0.3)

            # Subscribe IQ with NFM bandwidth
            ws, rx_state = ws_send_safe(ws, rx_state,
                json.dumps({"cmd": "subscribe_iq", "offset": 0, "bw": 25000}),
                args.sdr_ip, args.sdr_port)
            time.sleep(0.5)

            if not rx_state.iq_started:
                print("[RX] Waiting for IQ stream...")
                t0 = time.time()
                while not rx_state.iq_started and time.time() - t0 < 3:
                    time.sleep(0.1)

            if rx_state.iq_rate > 0:
                print(f"[RX] DDC rate: {rx_state.iq_rate}Hz")
            else:
                print("[RX] No DDC rate received, using default")

            # Drain stale IQ data
            rx_state.iq_queue.clear()
            time.sleep(0.2)

            # Capture audio for 2 full DTMF cycles + margin (cyclic TX)
            capture_dur = total_audio_dur * 1.5 + 1.0
            print(f"[RX] Capturing {capture_dur:.1f}s of audio...")
            audio = capture_audio(rx_state, capture_dur)

            # Stop TX
            pluto.stop()

            if len(audio) < AUDIO_RATE * 0.5:
                print(f"[RX] Too few audio samples: {len(audio)}")
                continue

            # Save debug WAV
            wav_name = f"test/dtmf_{freq_mhz:.0f}mhz_{gain_db}db.wav"
            save_debug_wav(audio, wav_name)
            rms = np.sqrt(np.mean(audio**2))
            print(f"[RX] Audio: {len(audio)} samples ({len(audio)/AUDIO_RATE:.1f}s) "
                  f"RMS={rms:.4f} → {wav_name}")

            # DTMF detection
            raw_detections = detect_dtmf(audio, AUDIO_RATE, block_ms=30)
            detected_chars = consolidate_detections(raw_detections)
            metrics = compute_metrics(DTMF_SEQUENCE, detected_chars, raw_detections)
            metrics['freq_mhz'] = freq_mhz
            metrics['gain_db'] = gain_db

            print_metrics(metrics, freq_mhz, gains.index(gain_db) + 1)
            print_detection_detail(detected_chars)

            all_results.append(metrics)

            # Track best
            if metrics['accuracy'] > best_overall['accuracy']:
                best_overall = {
                    'accuracy': metrics['accuracy'],
                    'freq': freq_mhz,
                    'gain': gain_db,
                    'snr': metrics['snr_mean'],
                    'detected': metrics['detected'],
                }

            # Quality gate: accuracy AND timing must be good
            timing_ok = (metrics['timing_error_ms'] <= 0.2 * TONE_DURATION * 1000)
            quality_perfect = (metrics['accuracy'] >= 100 and timing_ok)

            if quality_perfect:
                print(f"\n  ★ PERFECT detection at {freq_mhz:.1f}MHz gain={gain_db}dB!"
                      f" (timing_err={metrics['timing_error_ms']:.1f}ms"
                      f" gap_err={metrics['gap_error_ms']:.1f}ms)")
                break

            # If good enough (>75% and timing < 20%), try one more gain step
            if metrics['accuracy'] >= 75 and timing_ok and gains.index(gain_db) >= 2:
                break

    # ─────────────── Final Summary ───────────────

    print(f"\n{'═'*70}")
    print(f"  FINAL RESULTS")
    print(f"{'═'*70}")
    print(f"\n  {'Freq':>8} {'Gain':>6} {'Accuracy':>8} {'SNR':>8} "
          f"{'ToneErr':>8} {'GapErr':>8} {'Detected':>20}")
    print(f"  {'─'*74}")

    for r in all_results:
        status = "★" if r['accuracy'] >= 100 else "✓" if r['accuracy'] >= 75 else "✗"
        print(f"  {status} {r['freq_mhz']:>6.1f} {r['gain_db']:>5}dB "
              f"{r['accuracy']:>6.0f}% {r['snr_mean']:>6.1f}dB "
              f"{r['timing_error_ms']:>6.1f}ms {r['gap_error_ms']:>6.1f}ms "
              f" {r['detected']:>18}")

    print(f"\n  Best: {best_overall['accuracy']:.0f}% at {best_overall['freq']:.1f}MHz "
          f"gain={best_overall['gain']}dB SNR={best_overall['snr']:.1f}dB")
    print(f"  Detected: {best_overall.get('detected', 'none')}")

    # Quality verdict
    if best_overall['accuracy'] >= 100:
        print(f"\n  ✅ PRISTINE — all {len(DTMF_SEQUENCE)} DTMF tones detected perfectly")
    elif best_overall['accuracy'] >= 75:
        print(f"\n  ⚠️  GOOD — most tones detected, some dropouts")
    else:
        print(f"\n  ❌ POOR — significant detection failures")
        print(f"     Check: TX power, antenna, frequency, WebSDR audio pipeline")

    # Cleanup
    pluto.stop()
    rx_state.running = False
    try:
        ws.close()
    except Exception:
        pass


# ─────────────────────── Entry Point ───────────────────────

def main():
    parser = argparse.ArgumentParser(description="DTMF RF Loopback Test")
    parser.add_argument("--pluto-ip", default="192.168.2.1", help="PlutoSDR IP")
    parser.add_argument("--sdr-ip", default="192.168.1.232", help="WebSDR IP")
    parser.add_argument("--sdr-port", type=int, default=8080, help="WebSDR port")
    parser.add_argument("--freq", type=float, default=0,
                        help="Single test frequency in MHz (0=scan all)")
    parser.add_argument("--tx-gain", type=int, default=-10,
                        help="PlutoSDR TX gain in dB (-89 to 0)")
    parser.add_argument("--sequence", default=None,
                        help="DTMF sequence to transmit")
    args = parser.parse_args()

    if args.sequence:
        global DTMF_SEQUENCE
        DTMF_SEQUENCE = args.sequence

    try:
        run_test(args)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
