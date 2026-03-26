#!/usr/bin/env python3
"""
ESP32-P4 FM Radio — Browser Audio Streaming Test & Diagnosis

Connects to the standalone FM radio via WebSocket (same protocol as index.html),
subscribes to PCM audio, validates the stream, plays through speakers, and saves WAV.

The ESP32-P4 sends already-demodulated int16 PCM @ 48 kHz (mono or interleaved
stereo) as binary WebSocket frames.  This script is the Python equivalent of the
AudioWorklet PCM player in index.html.

Usage:
    pip install websocket-client numpy sounddevice
    python test_browser_audio.py --host 192.168.1.232 --freq 100.0

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import ssl
import struct
import sys
import threading
import time
import wave
from collections import deque

import numpy as np

try:
    import websocket
except ImportError:
    print("ERROR: pip install websocket-client")
    sys.exit(1)

try:
    import sounddevice as sd
except ImportError:
    sd = None
    print("WARNING: pip install sounddevice  (audio playback disabled)")


# ─────────────────────── Configuration ───────────────────────

AUDIO_RATE = 48000
RING_SIZE  = AUDIO_RATE * 2   # 2 seconds ring buffer


# ─────────────────────── Global State ───────────────────────

running         = True
lock            = threading.Lock()

# Counters
text_msgs       = 0
binary_msgs     = 0
binary_bytes    = 0
pcm_samples     = 0
underruns       = 0
overruns        = 0

# Signal strength from JSON pushes
signal_strength = 0
stereo          = False

# Ring buffer for audio playback (same design as the AudioWorklet)
ring            = np.zeros(RING_SIZE, dtype=np.float32)
ring_w          = 0      # write pointer
ring_r          = 0      # read pointer
ring_count      = 0      # samples available
ring_lock       = threading.Lock()

# WAV recording
wav_chunks      = []

# Stereo detection
channels_detected = 1


# ─────────────────────── Ring Buffer Ops ───────────────────────

def ring_write(samples):
    """Write float32 samples into the ring buffer (same as AudioWorklet)."""
    global ring_w, ring_count, overruns
    n = len(samples)
    with ring_lock:
        for i in range(n):
            ring[ring_w] = samples[i]
            ring_w = (ring_w + 1) % RING_SIZE
            if ring_count < RING_SIZE:
                ring_count += 1
            else:
                overruns += 1


def ring_read(frames):
    """Read frames from ring buffer, return float32 array."""
    global ring_r, ring_count, underruns
    out = np.zeros(frames, dtype=np.float32)
    with ring_lock:
        for i in range(frames):
            if ring_count > 0:
                out[i] = ring[ring_r]
                ring_r = (ring_r + 1) % RING_SIZE
                ring_count -= 1
            else:
                underruns += 1
    return out


# ─────────────────────── WebSocket Handlers ───────────────────────

def on_message(ws, message):
    global text_msgs, binary_msgs, binary_bytes, pcm_samples
    global signal_strength, stereo, channels_detected

    if isinstance(message, str):
        with lock:
            text_msgs += 1
        try:
            d = json.loads(message)
            if 'signal_strength' in d:
                signal_strength = d['signal_strength']
            if 'stereo' in d:
                stereo = d['stereo']
        except json.JSONDecodeError:
            pass
        return

    # Binary frame = raw PCM int16
    raw = message
    n_bytes = len(raw)
    if n_bytes < 2:
        return

    with lock:
        binary_msgs += 1
        binary_bytes += n_bytes

    # Decode int16 PCM
    n_samples = n_bytes // 2
    s16 = np.frombuffer(raw, dtype=np.int16, count=n_samples)
    f32 = s16.astype(np.float32) / 32768.0

    # Detect stereo: if sample count is always even and channels alternate,
    # we get interleaved L/R.  For playback simplicity, mix to mono.
    # (The ESP sends mono for NBFM, interleaved stereo for WBFM stereo)
    if stereo and n_samples % 2 == 0:
        channels_detected = 2
        # Downmix to mono: (L + R) / 2
        left  = f32[0::2]
        right = f32[1::2]
        f32 = (left + right) * 0.5
        n_samples = len(f32)

    with lock:
        pcm_samples += n_samples

    # Feed ring buffer for playback
    ring_write(f32)

    # Record for WAV
    wav_chunks.append(f32.copy())


def on_error(ws, error):
    print(f"\n[WS ERROR] {error}")


def on_close(ws, code, msg):
    print(f"\n[WS CLOSED] code={code} msg={msg}")


def on_open(ws):
    print("[WS] Connected")


# ─────────────────────── Audio Playback ───────────────────────

def audio_playback_thread():
    """Play audio via sounddevice using the ring buffer."""
    if sd is None:
        return

    def callback(outdata, frames, time_info, status):
        data = ring_read(frames)
        outdata[:, 0] = data

    try:
        stream = sd.OutputStream(
            samplerate=AUDIO_RATE,
            channels=1,
            blocksize=1024,
            dtype='float32',
            callback=callback,
        )
        stream.start()
        print(f"[AUDIO] Playback started @ {AUDIO_RATE} Hz")
        while running:
            time.sleep(0.1)
        stream.stop()
        stream.close()
    except Exception as e:
        print(f"[AUDIO] Playback error: {e}")


# ─────────────────────── Stats Printer ───────────────────────

def stats_thread():
    prev_samples = 0
    prev_time = time.time()

    while running:
        time.sleep(2.0)
        now = time.time()
        dt = now - prev_time

        with lock:
            cur_samples = pcm_samples
            cur_binary = binary_msgs
            cur_bytes = binary_bytes
            cur_underruns = underruns
            cur_overruns = overruns

        with ring_lock:
            buf_fill = ring_count

        d_samples = cur_samples - prev_samples
        rate = d_samples / dt if dt > 0 else 0
        fill_pct = rate / AUDIO_RATE * 100
        buf_ms = buf_fill * 1000 / AUDIO_RATE

        sig_pct = signal_strength * 100 / 32767 if signal_strength > 0 else 0

        stereo_str = "STEREO" if stereo else "MONO"
        ch_str = f"{channels_detected}ch"

        print(f"[STATS] PCM:{rate:.0f}sps ({fill_pct:.0f}% of 48k) | "
              f"buf:{buf_ms:.0f}ms | WS:{cur_binary}frames {cur_bytes/1024:.0f}KB | "
              f"sig:{sig_pct:.0f}% {stereo_str} {ch_str} | "
              f"under:{cur_underruns} over:{cur_overruns}")

        prev_samples = cur_samples
        prev_time = now


# ─────────────────────── API Helper ───────────────────────

def api_post(host, port, path, body):
    """POST JSON to the REST API (HTTPS, self-signed cert)."""
    import urllib.request
    url = f"https://{host}:{port}/api/{path}"
    data = json.dumps(body).encode()
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    req = urllib.request.Request(url, data=data, method='POST',
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, context=ctx, timeout=5) as r:
            return json.loads(r.read())
    except Exception as e:
        print(f"[API] {path} failed: {e}")
        return None


def api_get(host, port, path):
    """GET from the REST API."""
    import urllib.request
    url = f"https://{host}:{port}/api/{path}"
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    req = urllib.request.Request(url)
    try:
        with urllib.request.urlopen(req, context=ctx, timeout=5) as r:
            return json.loads(r.read())
    except Exception as e:
        print(f"[API] {path} failed: {e}")
        return None


# ─────────────────────── Save WAV ───────────────────────

def save_wav(filename, duration_limit=None):
    if not wav_chunks:
        print("[WAV] No audio recorded")
        return

    audio = np.concatenate(wav_chunks)
    if duration_limit:
        max_samples = int(duration_limit * AUDIO_RATE)
        audio = audio[:max_samples]

    dur = len(audio) / AUDIO_RATE
    rms = np.sqrt(np.mean(audio ** 2))
    peak = np.max(np.abs(audio))

    print(f"\n[WAV] {len(audio)} samples ({dur:.1f}s) RMS={rms:.4f} Peak={peak:.4f}")

    # Write int16 WAV
    int16 = (np.clip(audio, -1.0, 1.0) * 32767).astype(np.int16)
    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(AUDIO_RATE)
        wf.writeframes(int16.tobytes())

    print(f"[WAV] Saved → {filename}")

    if rms < 0.001:
        print("[WAV] WARNING: Very low level — likely silence")
    elif rms < 0.01:
        print("[WAV] WARNING: Low level — weak signal or squelch closed")
    else:
        print("[WAV] OK: Audio level looks good")

    return rms, peak, dur


# ─────────────────────── Test Sequence ───────────────────────

def run_tests(host, port, freq_mhz, gain, duration, play, wav_file):
    global running

    print("=" * 64)
    print("  ESP32-P4 FM Radio — Browser Audio Streaming Test")
    print("=" * 64)

    # ── Step 1: Check device status ──
    print(f"\n[1] Fetching device status from https://{host}:{port}/api/status ...")
    status = api_get(host, port, 'status')
    if status:
        cur_freq = status.get('freq', 0) / 1e6
        cur_mode = 'WBFM' if status.get('mode', 0) == 0 else 'NBFM'
        cur_vol  = status.get('volume', 0)
        cur_gain = status.get('gain', 0)
        print(f"    Freq: {cur_freq:.3f} MHz  Mode: {cur_mode}  "
              f"Vol: {cur_vol}  Gain: {cur_gain}")
    else:
        print("    WARNING: Could not reach API (will try WS anyway)")

    # ── Step 2: Set frequency and gain ──
    target_freq = int(freq_mhz * 1e6)
    print(f"\n[2] Tuning to {freq_mhz:.3f} MHz, gain={gain} ...")
    api_post(host, port, 'freq', {'value': target_freq})
    if gain >= 0:
        api_post(host, port, 'gain', {'value': gain})
    # Ensure WBFM mode for stereo/RDS
    api_post(host, port, 'mode', {'value': 0})
    time.sleep(0.3)

    # ── Step 3: Connect WebSocket ──
    print(f"\n[3] Connecting WebSocket wss://{host}:{port}/ws ...")
    sslopt = {"cert_reqs": ssl.CERT_NONE}
    ws = websocket.WebSocketApp(
        f"wss://{host}:{port}/ws",
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )
    ws_thread = threading.Thread(target=ws.run_forever,
                                  kwargs={"sslopt": sslopt},
                                  daemon=True)
    ws_thread.start()

    # Wait for connection
    t0 = time.time()
    while time.time() - t0 < 5:
        if ws.sock and ws.sock.connected:
            break
        time.sleep(0.1)
    else:
        print("    FAIL: WebSocket connection timeout")
        running = False
        return False

    # ── Step 4: Check signal strength pushes ──
    print("\n[4] Waiting for signal strength data ...")
    t0 = time.time()
    while text_msgs == 0 and time.time() - t0 < 3:
        time.sleep(0.1)
    if text_msgs > 0:
        sig_pct = signal_strength * 100 / 32767
        print(f"    OK: {text_msgs} status messages, signal={sig_pct:.0f}%")
    else:
        print("    WARNING: No signal strength received (timer may not be running)")

    # ── Step 5: Subscribe to audio ──
    print("\n[5] Sending audio_on command ...")
    ws.send(json.dumps({"cmd": "audio_on"}))
    time.sleep(0.5)

    # Wait for first binary frame
    t0 = time.time()
    while binary_msgs == 0 and time.time() - t0 < 5:
        time.sleep(0.1)

    if binary_msgs == 0:
        print("    FAIL: No audio frames received after 5s")
        print("    Possible causes:")
        print("      - audio_on command not handled by firmware")
        print("      - fm_pipeline_task not calling web_radio_push_audio()")
        print("      - WebSocket client not in s_ws_audio[] table")
        running = False
        ws.send(json.dumps({"cmd": "audio_off"}))
        ws.close()
        return False

    print(f"    OK: First audio frame received ({binary_bytes} bytes)")

    # ── Step 6: Start playback + stats ──
    if play and sd:
        threading.Thread(target=audio_playback_thread, daemon=True).start()
    threading.Thread(target=stats_thread, daemon=True).start()

    # ── Step 7: Record for duration ──
    print(f"\n[6] Recording {duration}s of audio ...")
    if play and sd:
        print("    Audio playing through speakers")
    print("    (Ctrl+C to stop early)\n")

    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    # ── Step 8: Unsubscribe and close ──
    print("\n[7] Stopping ...")
    try:
        ws.send(json.dumps({"cmd": "audio_off"}))
    except Exception:
        pass
    running = False
    time.sleep(0.5)
    try:
        ws.close()
    except Exception:
        pass

    # ── Step 9: Check RDS ──
    rds = api_get(host, port, 'rds')
    if rds:
        ps = rds.get('ps_name', '').strip()
        rt = rds.get('radio_text', '').strip()
        pi = rds.get('pi_code', '')
        pty = rds.get('pty', 0)
        rds_stereo = rds.get('stereo', False)
        print(f"\n[RDS] PS=\"{ps}\" RT=\"{rt}\"")
        print(f"      PI={pi} PTY={pty} Stereo={rds_stereo}")
    else:
        print("\n[RDS] Could not fetch RDS data")

    # ── Step 10: Save WAV and summarize ──
    rms_peak_dur = save_wav(wav_file, duration_limit=duration)

    # Summary
    with lock:
        total_samples = pcm_samples
        total_frames = binary_msgs
        total_bytes = binary_bytes

    actual_dur = total_samples / AUDIO_RATE if total_samples > 0 else 0
    avg_rate = total_samples / duration if duration > 0 else 0
    fill_pct = avg_rate / AUDIO_RATE * 100

    print(f"\n{'=' * 64}")
    print(f"  RESULTS")
    print(f"{'=' * 64}")
    print(f"  Frequency:     {freq_mhz:.3f} MHz")
    print(f"  WS frames:     {total_frames}")
    print(f"  WS bytes:      {total_bytes / 1024:.0f} KB")
    print(f"  PCM samples:   {total_samples} ({actual_dur:.1f}s)")
    print(f"  Avg fill rate: {avg_rate:.0f} sps ({fill_pct:.0f}% of 48kHz)")
    print(f"  Channels:      {channels_detected} ({'stereo detected' if channels_detected == 2 else 'mono'})")
    print(f"  Underruns:     {underruns}")
    print(f"  Overruns:      {overruns}")

    # Diagnosis
    if total_samples == 0:
        print(f"\n  DIAGNOSIS: Zero audio — web_radio_push_audio() not wired or squelch closed")
        return False
    elif fill_pct < 50:
        print(f"\n  DIAGNOSIS: Audio underrun ({fill_pct:.0f}% fill)")
        print(f"    - Check fm_pipeline_task is running and calling web_radio_push_audio()")
        print(f"    - Check WebSocket bandwidth (TLS overhead on ESP32-P4)")
        print(f"    - Try reducing volume or disabling stereo to reduce frame size")
        return False
    elif fill_pct < 90:
        print(f"\n  WARNING: Slight underrun ({fill_pct:.0f}% fill) — may hear occasional glitches")
        return True
    else:
        print(f"\n  OK: Audio stream healthy ({fill_pct:.0f}% fill)")
        return True


# ─────────────────────── Main ───────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ESP32-P4 FM Radio Browser Audio Test",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default="192.168.1.232",
                        help="ESP32-P4 IP address")
    parser.add_argument("--port", type=int, default=8080,
                        help="HTTPS port")
    parser.add_argument("--freq", type=float, default=100.0,
                        help="FM frequency in MHz")
    parser.add_argument("--gain", type=int, default=496,
                        help="Tuner gain (0=auto, 9-496=manual tenths dB)")
    parser.add_argument("--duration", type=int, default=15,
                        help="Recording duration in seconds")
    parser.add_argument("--no-play", action="store_true",
                        help="Disable speaker playback")
    parser.add_argument("--wav", default="fm_audio_test.wav",
                        help="Output WAV filename")
    args = parser.parse_args()

    print(f"Target: {args.host}:{args.port}  Freq: {args.freq:.3f} MHz  "
          f"Gain: {args.gain}  Duration: {args.duration}s")
    print()

    success = run_tests(
        host=args.host,
        port=args.port,
        freq_mhz=args.freq,
        gain=args.gain,
        duration=args.duration,
        play=not args.no_play,
        wav_file=args.wav,
    )

    print(f"\nPlay saved audio:  aplay {args.wav}")
    print(f"Analyze spectrum:  python3 -c \"import scipy.io.wavfile as w; "
          f"import matplotlib.pyplot as plt; r,d = w.read('{args.wav}'); "
          f"plt.specgram(d, Fs=r); plt.show()\"")

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
