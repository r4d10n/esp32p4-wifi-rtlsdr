#!/usr/bin/env python3
"""
ESP32-P4 FM Radio — Browser Audio Streaming Test & Diagnosis

Connects to the standalone FM radio via WebSocket (same protocol as index.html),
subscribes to PCM audio, validates the stream, plays through speakers, and saves WAV.

The ESP32-P4 sends already-demodulated int16 PCM @ 48 kHz (mono or interleaved
stereo) as binary WebSocket frames.  This script is the Python equivalent of the
AudioWorklet PCM player in index.html.

Handles TLS WebSocket reconnection — the ESP32-P4's lightweight TLS stack may
drop connections under sustained binary frame throughput.

Usage:
    pip install websocket-client numpy sounddevice
    python test_browser_audio.py --host 192.168.1.233 --freq 100.0

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import ssl
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


# ─────────────────────── Global State ───────────────────────

running         = True
lock            = threading.Lock()

# Counters
text_msgs       = 0
binary_msgs     = 0
binary_bytes    = 0
pcm_samples     = 0
underruns       = 0
ws_reconnects   = 0

# Signal strength from JSON pushes
signal_strength = 0
stereo          = False

# Audio chunk queue — much faster than per-sample ring buffer
# Each element is a float32 numpy array of decoded PCM samples
audio_queue     = deque(maxlen=500)   # ~500 chunks = several seconds

# Playback buffer (fed from audio_queue in callback)
play_buf        = np.zeros(0, dtype=np.float32)

# WAV recording
wav_chunks      = []

# Stereo detection
channels_detected = 1


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

    # Decode int16 PCM → float32 (vectorized, no Python loops)
    n_samples = n_bytes // 2
    s16 = np.frombuffer(raw, dtype=np.int16, count=n_samples)
    f32 = s16.astype(np.float32) * (1.0 / 32768.0)

    # Track total samples received (before downmix) for fill rate calc
    with lock:
        pcm_samples += n_samples

    # Detect stereo: interleaved L/R → downmix to mono for playback
    if stereo and n_samples >= 2 and n_samples % 2 == 0:
        channels_detected = 2
        f32 = (f32[0::2] + f32[1::2]) * 0.5

    # Queue for playback (lock-free deque append is thread-safe)
    audio_queue.append(f32)

    # Record for WAV
    wav_chunks.append(f32.copy())


def on_error(ws, error):
    err_str = str(error)
    # Suppress noisy SSL errors during reconnect
    if 'RECORD_LAYER' in err_str or 'EOF' in err_str:
        return
    print(f"\n[WS ERROR] {error}")


def on_close(ws, code, msg):
    pass  # Reconnect logic handles this


def on_open(ws):
    # Re-subscribe to audio on every (re)connect
    try:
        ws.send(json.dumps({"cmd": "audio_on"}))
    except Exception:
        pass


# ─────────────────────── WebSocket with Auto-Reconnect ───────────────────────

class ReconnectingWS:
    """WebSocket wrapper that auto-reconnects on TLS/connection failures."""

    def __init__(self, url, sslopt):
        self.url = url
        self.sslopt = sslopt
        self.ws = None
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self):
        global ws_reconnects
        backoff = 0.5
        while running:
            try:
                self.ws = websocket.WebSocketApp(
                    self.url,
                    on_open=on_open,
                    on_message=on_message,
                    on_error=on_error,
                    on_close=on_close,
                )
                self.ws.run_forever(sslopt=self.sslopt, ping_interval=5,
                                     ping_timeout=3)
            except Exception:
                pass

            if not running:
                break

            with lock:
                ws_reconnects += 1
            # Brief backoff, then reconnect
            time.sleep(min(backoff, 3.0))
            backoff = min(backoff * 1.5, 3.0)

    @property
    def connected(self):
        return (self.ws and self.ws.sock and
                hasattr(self.ws.sock, 'connected') and self.ws.sock.connected)

    def send(self, data):
        if self.connected:
            try:
                self.ws.send(data)
            except Exception:
                pass

    def close(self):
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass


# ─────────────────────── Audio Playback ───────────────────────

def audio_playback_thread():
    """Play audio via sounddevice, pulling from the chunk queue."""
    global play_buf, underruns

    if sd is None:
        return

    def callback(outdata, frames, time_info, status):
        global play_buf, underruns

        # Drain chunks into contiguous buffer
        while len(play_buf) < frames:
            try:
                chunk = audio_queue.popleft()
                play_buf = np.concatenate([play_buf, chunk])
            except IndexError:
                break

        n = min(len(play_buf), frames)
        if n > 0:
            outdata[:n, 0] = play_buf[:n]
            play_buf = play_buf[n:]
        if n < frames:
            outdata[n:, 0] = 0.0
            underruns += (frames - n)

    try:
        stream = sd.OutputStream(
            samplerate=AUDIO_RATE,
            channels=1,
            blocksize=2048,
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
            cur_reconnects = ws_reconnects

        d_samples = cur_samples - prev_samples
        rate = d_samples / dt if dt > 0 else 0
        fill_pct = rate / AUDIO_RATE * 100
        q_chunks = len(audio_queue)

        sig_pct = signal_strength * 100 / 32767 if signal_strength > 0 else 0
        stereo_str = "ST" if stereo else "MO"
        ch_str = f"{channels_detected}ch"

        print(f"[STATS] {rate:.0f}sps ({fill_pct:.0f}%) | "
              f"q:{q_chunks} | ws:{cur_binary}f {cur_bytes/1024:.0f}KB "
              f"reconn:{cur_reconnects} | "
              f"sig:{sig_pct:.0f}% {stereo_str} {ch_str} | "
              f"under:{cur_underruns}")

        prev_samples = cur_samples
        prev_time = now


# ─────────────────────── API Helper ───────────────────────

def _ssl_ctx():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx

def api_post(host, port, path, body):
    import urllib.request
    url = f"https://{host}:{port}/api/{path}"
    data = json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method='POST',
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, context=_ssl_ctx(), timeout=5) as r:
            return json.loads(r.read())
    except Exception as e:
        print(f"[API] {path} failed: {e}")
        return None

def api_get(host, port, path):
    import urllib.request
    url = f"https://{host}:{port}/api/{path}"
    req = urllib.request.Request(url)
    try:
        with urllib.request.urlopen(req, context=_ssl_ctx(), timeout=5) as r:
            return json.loads(r.read())
    except Exception as e:
        print(f"[API] {path} failed: {e}")
        return None


# ─────────────────────── Save WAV ───────────────────────

def save_wav(filename, duration_limit=None):
    if not wav_chunks:
        print("[WAV] No audio recorded")
        return None

    audio = np.concatenate(wav_chunks)
    if duration_limit:
        max_samples = int(duration_limit * AUDIO_RATE)
        audio = audio[:max_samples]

    dur = len(audio) / AUDIO_RATE
    rms = np.sqrt(np.mean(audio ** 2))
    peak = np.max(np.abs(audio)) if len(audio) > 0 else 0

    print(f"\n[WAV] {len(audio)} samples ({dur:.1f}s) RMS={rms:.4f} Peak={peak:.4f}")

    # Normalize if peak is very low (boost weak signals for audibility)
    if 0 < peak < 0.1:
        gain = 0.8 / peak
        print(f"[WAV] Boosting by {gain:.1f}x (weak signal normalization)")
        audio_norm = audio * gain
    else:
        audio_norm = audio

    int16 = (np.clip(audio_norm, -1.0, 1.0) * 32767).astype(np.int16)
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
    print(f"\n[1] Fetching status from https://{host}:{port} ...")
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

    # ── Step 2: Tune to target ──
    target_freq = int(freq_mhz * 1e6)
    print(f"\n[2] Tuning to {freq_mhz:.3f} MHz, gain={gain} ...")
    api_post(host, port, 'freq', {'value': target_freq})
    if gain >= 0:
        api_post(host, port, 'gain', {'value': gain})
    api_post(host, port, 'mode', {'value': 0})  # WBFM
    time.sleep(0.5)

    # ── Step 3: Connect WebSocket with auto-reconnect ──
    print(f"\n[3] Connecting WebSocket wss://{host}:{port}/ws ...")
    sslopt = {"cert_reqs": ssl.CERT_NONE}
    rws = ReconnectingWS(f"wss://{host}:{port}/ws", sslopt)
    rws.start()

    # Wait for first connection
    t0 = time.time()
    while time.time() - t0 < 8:
        if rws.connected:
            break
        time.sleep(0.2)
    else:
        print("    FAIL: WebSocket connection timeout")
        running = False
        return False

    print("    Connected (auto-reconnect enabled)")

    # ── Step 4: Wait for signal strength ──
    print("\n[4] Waiting for signal strength ...")
    t0 = time.time()
    while text_msgs == 0 and time.time() - t0 < 3:
        time.sleep(0.1)
    if text_msgs > 0:
        sig_pct = signal_strength * 100 / 32767
        print(f"    OK: signal={sig_pct:.0f}%")
    else:
        print("    WARNING: No signal strength received yet")

    # ── Step 5: Wait for audio frames ──
    print("\n[5] Waiting for audio frames ...")
    t0 = time.time()
    while binary_msgs == 0 and time.time() - t0 < 5:
        time.sleep(0.1)

    if binary_msgs == 0:
        print("    FAIL: No audio frames received after 5s")
        print("    - web_radio_push_audio() may not be wired")
        print("    - Squelch may be closing the audio gate")
        running = False
        rws.close()
        return False

    with lock:
        first_bytes = binary_bytes
    print(f"    OK: Audio streaming ({first_bytes} bytes, {binary_msgs} frames)")

    # ── Step 6: Playback + recording ──
    if play and sd:
        threading.Thread(target=audio_playback_thread, daemon=True).start()
    threading.Thread(target=stats_thread, daemon=True).start()

    print(f"\n[6] Recording {duration}s ...")
    if play and sd:
        print("    Playing through speakers")
    print("    (Ctrl+C to stop early)\n")

    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    # ── Step 7: Stop ──
    print("\n[7] Stopping ...")
    rws.send(json.dumps({"cmd": "audio_off"}))
    running = False
    time.sleep(0.5)
    rws.close()

    # ── Step 8: RDS ──
    rds = api_get(host, port, 'rds')
    if rds:
        ps = rds.get('ps_name', '').strip()
        rt = rds.get('radio_text', '').strip()
        pi = rds.get('pi_code', '')
        pty = rds.get('pty', 0)
        rds_st = rds.get('stereo', False)
        print(f"\n[RDS] PS=\"{ps}\" RT=\"{rt}\"")
        print(f"      PI={pi} PTY={pty} Stereo={rds_st}")

    # ── Step 9: Save WAV ──
    result = save_wav(wav_file, duration_limit=duration)

    # ── Summary ──
    with lock:
        total_samples = pcm_samples
        total_frames = binary_msgs
        total_bytes = binary_bytes
        total_reconnects = ws_reconnects

    actual_dur = total_samples / AUDIO_RATE if total_samples > 0 else 0
    avg_rate = total_samples / duration if duration > 0 else 0
    fill_pct = avg_rate / AUDIO_RATE * 100

    print(f"\n{'=' * 64}")
    print(f"  RESULTS")
    print(f"{'=' * 64}")
    print(f"  Frequency:     {freq_mhz:.3f} MHz")
    print(f"  WS frames:     {total_frames}")
    print(f"  WS data:       {total_bytes / 1024:.0f} KB")
    print(f"  WS reconnects: {total_reconnects}")
    print(f"  PCM samples:   {total_samples} ({actual_dur:.1f}s)")
    print(f"  Avg fill rate: {avg_rate:.0f} sps ({fill_pct:.0f}% of 48kHz)")
    print(f"  Channels:      {channels_detected} ({'stereo' if channels_detected == 2 else 'mono'})")
    print(f"  Underruns:     {underruns}")

    if total_samples == 0:
        print(f"\n  DIAGNOSIS: Zero audio — push_audio not wired or squelch closed")
        return False
    elif fill_pct < 50:
        print(f"\n  DIAGNOSIS: Audio underrun ({fill_pct:.0f}% fill)")
        if total_reconnects > 5:
            print(f"    TLS reconnects ({total_reconnects}) are the bottleneck.")
            print(f"    Each reconnect loses ~1-2s of audio.")
            print(f"    Consider: larger WS frame batching or plain HTTP mode.")
        return False
    elif fill_pct < 90:
        print(f"\n  WARNING: Slight underrun ({fill_pct:.0f}%) — occasional glitches")
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
    parser.add_argument("--host", default="192.168.1.233",
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
          f"Gain: {args.gain}  Duration: {args.duration}s\n")

    success = run_tests(
        host=args.host,
        port=args.port,
        freq_mhz=args.freq,
        gain=args.gain,
        duration=args.duration,
        play=not args.no_play,
        wav_file=args.wav,
    )

    print(f"\nPlay:    aplay {args.wav}")
    print(f"Analyze: python3 -c \"import scipy.io.wavfile as w; "
          f"import matplotlib.pyplot as plt; r,d = w.read('{args.wav}'); "
          f"plt.specgram(d, Fs=r); plt.show()\"")

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
