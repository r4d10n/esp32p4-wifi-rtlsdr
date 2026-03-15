#!/usr/bin/env python3
"""
WebSDR Audio Pipeline Test & Diagnosis

Connects to ESP32-P4 WebSDR via WebSocket, subscribes to IQ,
demodulates FM/AM, plays audio, and saves WAV for analysis.

Tests each stage independently with sample counters (not queue depth).

Usage:
  pip install websocket-client numpy sounddevice
  python test_audio_ws.py [--host 192.168.1.232] [--port 8080] [--mode fm] [--no-play]

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import sys
import threading
import time
import traceback
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
    print("WARNING: sounddevice not available, audio playback disabled")


# ─────────────────────── Global State ───────────────────────

device_info = {}
fft_count = 0
iq_frame_count = 0
iq_bytes_total = 0
iq_rate = 0
audio_rate = 48000
audio_samples_produced = 0
audio_samples_played = 0

iq_queue = deque(maxlen=500)
audio_queue = deque(maxlen=500)

# FM demodulator state
prev_i = 0.0
prev_q = 0.0

# WAV recording
wav_samples = []

stats_lock = threading.Lock()
running = True


# ─────────────────────── Demodulation ───────────────────────

def demod_fm(iq_uint8):
    global prev_i, prev_q
    iq_float = (iq_uint8.astype(np.float32) - 127.5) / 127.5
    i_s = iq_float[0::2]
    q_s = iq_float[1::2]
    n = len(i_s)
    if n < 2:
        return np.array([], dtype=np.float32)
    audio = np.empty(n, dtype=np.float32)
    for k in range(n):
        ci, cq = i_s[k], q_s[k]
        re = ci * prev_i + cq * prev_q
        im = cq * prev_i - ci * prev_q
        audio[k] = np.arctan2(im, re) / np.pi
        prev_i, prev_q = ci, cq
    return audio


def demod_am(iq_uint8):
    iq_float = (iq_uint8.astype(np.float32) - 127.5) / 127.5
    i_s = iq_float[0::2]
    q_s = iq_float[1::2]
    env = np.sqrt(i_s**2 + q_s**2)
    return (env - np.mean(env)).astype(np.float32)


def resample_linear(audio, src_rate, dst_rate):
    if src_rate == dst_rate or src_rate <= 0 or len(audio) == 0:
        return audio
    ratio = src_rate / dst_rate
    n_out = int(len(audio) / ratio)
    if n_out <= 0:
        return np.array([], dtype=np.float32)
    indices = np.arange(n_out) * ratio
    lo = indices.astype(int)
    frac = indices - lo
    lo = np.clip(lo, 0, len(audio) - 1)
    hi = np.clip(lo + 1, 0, len(audio) - 1)
    return (audio[lo] * (1.0 - frac) + audio[hi] * frac).astype(np.float32)


# ─────────────────────── WebSocket Handlers ───────────────────────

def on_message(ws, message):
    global fft_count, iq_frame_count, iq_bytes_total, iq_rate, device_info

    if isinstance(message, str):
        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            return
        msg_type = msg.get("type", "")
        if msg_type == "info":
            device_info = msg
            print(f"\n[INFO] freq={msg.get('freq')}Hz rate={msg.get('rate')}Hz "
                  f"gain={msg.get('gain')} fft={msg.get('fft_size')} "
                  f"dB=[{msg.get('db_min')}, {msg.get('db_max')}]")
        elif msg_type == "iq_start":
            iq_rate = msg.get("rate", 0)
            print(f"[IQ_START] offset={msg.get('offset')}Hz bw={msg.get('bw')}Hz "
                  f"DDC_rate={iq_rate}Hz")
        elif msg_type == "config":
            print(f"[CONFIG] {msg}")
        elif msg_type == "freq":
            print(f"[FREQ] {msg.get('value')}Hz")
        return

    if len(message) < 2:
        return
    msg_type = message[0]
    payload = message[1:]

    if msg_type == 0x01:
        with stats_lock:
            fft_count += 1
    elif msg_type == 0x02:
        with stats_lock:
            iq_frame_count += 1
            iq_bytes_total += len(payload)
        iq_queue.append(np.frombuffer(payload, dtype=np.uint8))


def on_error(ws, error):
    print(f"[WS ERROR] {error}")

def on_close(ws, close_status, close_msg):
    print(f"[WS CLOSED] status={close_status}")

def on_open(ws):
    print("[WS CONNECTED]")


# ─────────────────────── Audio Processing Thread ───────────────────────

def audio_processor(mode, record=True):
    global running, audio_samples_produced
    demod_fn = demod_fm if mode in ("fm", "wbfm") else demod_am

    while running:
        if len(iq_queue) == 0:
            time.sleep(0.002)
            continue
        try:
            iq_data = iq_queue.popleft()
        except IndexError:
            continue

        audio = demod_fn(iq_data)
        if len(audio) == 0:
            continue

        if iq_rate > 0 and iq_rate != audio_rate:
            audio = resample_linear(audio, iq_rate, audio_rate)

        if len(audio) > 0:
            with stats_lock:
                audio_samples_produced += len(audio)
            audio_queue.append(audio)
            if record:
                wav_samples.append(audio.copy())


# ─────────────────────── Audio Playback ───────────────────────

def audio_playback():
    global running, audio_samples_played
    if sd is None:
        return

    buf = np.zeros(0, dtype=np.float32)

    def callback(outdata, frames, time_info, status):
        nonlocal buf
        while len(buf) < frames and len(audio_queue) > 0:
            try:
                buf = np.concatenate([buf, audio_queue.popleft()])
            except IndexError:
                break
        n = min(len(buf), frames)
        if n > 0:
            outdata[:n, 0] = buf[:n] * 0.8
            buf = buf[n:]
        outdata[n:, 0] = 0.0

    try:
        stream = sd.OutputStream(samplerate=audio_rate, channels=1,
                                  blocksize=2048, dtype='float32',
                                  callback=callback)
        stream.start()
        print(f"[AUDIO] Playback started at {audio_rate}Hz")
        while running:
            time.sleep(0.1)
        stream.stop()
        stream.close()
    except Exception as e:
        print(f"[AUDIO ERROR] {e}")


# ─────────────────────── Stats Printer ───────────────────────

def stats_printer():
    global running
    prev = {"fft": 0, "iq": 0, "iq_bytes": 0, "audio": 0, "t": time.time()}

    while running:
        time.sleep(2.0)
        now = time.time()
        dt = now - prev["t"]
        with stats_lock:
            d_fft = fft_count - prev["fft"]
            d_iq = iq_frame_count - prev["iq"]
            d_bytes = iq_bytes_total - prev["iq_bytes"]
            d_audio = audio_samples_produced - prev["audio"]
            prev.update({"fft": fft_count, "iq": iq_frame_count,
                         "iq_bytes": iq_bytes_total, "audio": audio_samples_produced,
                         "t": now})

        fft_hz = d_fft / dt
        iq_hz = d_iq / dt
        iq_kbps = d_bytes * 8 / 1000 / dt
        audio_hz = d_audio / dt

        # Compute expected vs actual
        expected_audio = audio_rate  # 48000 sps needed
        fill_pct = (audio_hz / expected_audio * 100) if expected_audio > 0 else 0

        print(f"[STATS] FFT:{fft_hz:.0f}Hz IQ:{iq_hz:.0f}fr/s({iq_kbps:.0f}kbps) "
              f"Audio:{audio_hz:.0f}sps/{expected_audio}sps ({fill_pct:.0f}% fill) "
              f"Q:iq={len(iq_queue)} aud={len(audio_queue)}")


# ─────────────────────── Save WAV ───────────────────────

def save_wav(filename):
    if not wav_samples:
        print(f"[WAV] No audio recorded")
        return
    all_audio = np.concatenate(wav_samples)
    print(f"[WAV] Saving {len(all_audio)} samples ({len(all_audio)/audio_rate:.1f}s) → {filename}")

    # Normalize to int16
    peak = np.max(np.abs(all_audio))
    if peak > 0:
        all_audio = all_audio / peak * 0.9
    int16_data = (all_audio * 32767).astype(np.int16)

    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(audio_rate)
        wf.writeframes(int16_data.tobytes())

    # Analyze
    rms = np.sqrt(np.mean(all_audio**2))
    print(f"[WAV] RMS={rms:.4f} Peak={peak:.4f} Duration={len(all_audio)/audio_rate:.1f}s")
    if rms < 0.001:
        print(f"[WAV] ⚠ VERY LOW LEVEL — likely silence or near-DC signal")
    elif rms < 0.01:
        print(f"[WAV] ⚠ Low level — signal may be weak")
    else:
        print(f"[WAV] ✓ Audio level OK")


# ─────────────────────── Test Sequence ───────────────────────

def run_tests(ws, mode, bw, duration):
    global running

    print("\n" + "="*60)
    print("  ESP32-P4 WebSDR Audio Pipeline Test")
    print("="*60)

    # Test 1: Device info
    print("\n[TEST 1] Device info...")
    t0 = time.time()
    while not device_info and time.time() - t0 < 5:
        time.sleep(0.1)
    if device_info:
        print(f"  ✓ freq={device_info.get('freq')}Hz rate={device_info.get('rate')}Hz")
    else:
        print(f"  ✗ No device info")
        return

    # Test 2: FFT frames
    print("\n[TEST 2] FFT frames...")
    t0 = time.time()
    while fft_count == 0 and time.time() - t0 < 5:
        time.sleep(0.1)
    print(f"  {'✓' if fft_count else '✗'} FFT frames: {fft_count}")

    # Test 3: Subscribe IQ
    bw_val = bw or {"fm": 25000, "am": 10000, "wbfm": 200000}.get(mode, 25000)
    print(f"\n[TEST 3] Subscribe IQ (mode={mode} bw={bw_val}Hz)...")
    ws.send(json.dumps({"cmd": "subscribe_iq", "offset": 0, "bw": bw_val}))
    t0 = time.time()
    while iq_frame_count == 0 and time.time() - t0 < 5:
        time.sleep(0.1)
    if iq_frame_count:
        print(f"  ✓ IQ frames arriving, DDC rate={iq_rate}Hz")
        print(f"    Expected audio fill: {iq_rate}/{audio_rate} = "
              f"{iq_rate/audio_rate*100:.0f}% (before demod overhead)")
    else:
        print(f"  ✗ No IQ frames")
        return

    # Test 4: Demodulation check
    print(f"\n[TEST 4] Demodulation ({mode.upper()})...")
    time.sleep(2.0)
    with stats_lock:
        produced = audio_samples_produced
    if produced > 0:
        rate = produced / 2.0
        print(f"  ✓ {produced} audio samples in 2s ({rate:.0f} sps)")
        print(f"    Fill rate: {rate/audio_rate*100:.0f}% of 48kHz")
        if rate < audio_rate * 0.5:
            print(f"  ⚠ Under 50% fill — audio will be choppy")
            print(f"    Root cause: server DDC only outputs {iq_rate}Hz, "
                  f"need more IQ throughput")
    else:
        print(f"  ✗ Zero audio samples produced!")
        print(f"    IQ frames received: {iq_frame_count}")
        print(f"    IQ queue: {len(iq_queue)}")
        return

    # Test 5: Listen + Record
    print(f"\n[TEST 5] Recording {duration}s of audio...")
    if sd is not None:
        print(f"  Audio playing through speakers")
    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        pass

    # Summary
    with stats_lock:
        total_audio = audio_samples_produced

    print(f"\n{'='*60}")
    print(f"  RESULTS")
    print(f"{'='*60}")
    print(f"  FFT frames:    {fft_count}")
    print(f"  IQ frames:     {iq_frame_count} ({iq_bytes_total/1024:.0f} KB)")
    print(f"  DDC rate:      {iq_rate} Hz")
    print(f"  Audio produced:{total_audio} samples ({total_audio/audio_rate:.1f}s)")
    print(f"  Fill rate:     {total_audio/audio_rate/(duration+2)*100:.0f}%")

    if total_audio / audio_rate < (duration + 2) * 0.3:
        print(f"\n  ⚠ DIAGNOSIS: Audio underrun — server sends too few IQ frames")
        print(f"    The DDC in fft_process_task only runs at FFT rate (20Hz)")
        print(f"    and reads limited data per cycle.")
        print(f"    FIX: Decouple DDC from FFT rate, or increase task frequency.")


# ─────────────────────── Main ───────────────────────

def main():
    global running
    parser = argparse.ArgumentParser(description="WebSDR Audio Pipeline Test")
    parser.add_argument("--host", default="192.168.1.232")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--mode", choices=["fm", "am", "wbfm"], default="fm")
    parser.add_argument("--bw", type=int, default=0)
    parser.add_argument("--duration", type=int, default=10)
    parser.add_argument("--no-play", action="store_true")
    parser.add_argument("--wav", default="test_audio.wav")
    args = parser.parse_args()

    url = f"ws://{args.host}:{args.port}/ws"
    print(f"Connecting to {url} ...")

    ws = websocket.WebSocketApp(url, on_open=on_open, on_message=on_message,
                                 on_error=on_error, on_close=on_close)
    threading.Thread(target=ws.run_forever, daemon=True).start()
    time.sleep(1.0)

    if not ws.sock or not ws.sock.connected:
        print("ERROR: WebSocket connection failed")
        sys.exit(1)

    # Start threads
    threading.Thread(target=audio_processor, args=(args.mode,), daemon=True).start()
    if not args.no_play and sd:
        threading.Thread(target=audio_playback, daemon=True).start()
    threading.Thread(target=stats_printer, daemon=True).start()

    try:
        run_tests(ws, args.mode, args.bw, args.duration)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
    finally:
        running = False
        ws.close()
        time.sleep(0.3)

    save_wav(args.wav)
    print("\nDone. Play with: aplay test_audio.wav")


if __name__ == "__main__":
    main()
