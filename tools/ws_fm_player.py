#!/usr/bin/env python3
"""
WebSocket FM Audio Player for ESP32-P4 WebSDR — v5

Vectorized numpy FM demodulator with 15 kHz audio LPF, proper gain structure,
direct int16 IQ, fractional resampling, and robust audio ring buffer.

Usage:
    python3 tools/ws_fm_player.py [freq_mhz] [host]

Requirements: pip install websocket-client numpy sounddevice scipy

SPDX-License-Identifier: GPL-2.0-or-later
"""

import sys, json, ssl, time, threading
import numpy as np
import sounddevice as sd
import websocket
from scipy.signal import firwin, lfilter

# ── Configuration ──
AUDIO_RATE = 48000
IQ_BW = 150000
SAMPLE_RATE = 300000
VOLUME = 0.5          # Output volume (0.0 - 1.0)
DE_EMPH_TAU = 75e-6   # US: 75us, EU: 50us

# ── Audio ring buffer ──
RING_SECS = 2.0
ring = np.zeros(int(AUDIO_RATE * RING_SECS), dtype=np.float32)
ring_wr = 0; ring_rd = 0; ring_sz = len(ring)
ring_lock = threading.Lock()

def ring_avail():
    return (ring_wr - ring_rd) % ring_sz

def ring_push(s):
    global ring_wr
    n = len(s)
    if n == 0: return
    with ring_lock:
        w = ring_wr; e = w + n
        if e <= ring_sz: ring[w:e] = s
        else:
            f = ring_sz - w; ring[w:] = s[:f]; ring[:n-f] = s[f:]
        ring_wr = e % ring_sz

def ring_pull(n):
    global ring_rd
    with ring_lock:
        a = ring_avail(); t = min(n, a)
        if t == 0: return np.zeros(n, dtype=np.float32)
        r = ring_rd; e = r + t
        if e <= ring_sz: out = ring[r:e].copy()
        else:
            f = ring_sz - r; out = np.concatenate([ring[r:], ring[:t-f]])
        ring_rd = e % ring_sz
        return np.concatenate([out, np.zeros(n-t, dtype=np.float32)]) if t < n else out


class FMDemod:
    """WBFM demodulator: discriminator → de-emphasis → 15kHz LPF → fractional resample."""

    def __init__(self, iq_rate, audio_rate, tau=75e-6):
        self.iq_rate = iq_rate
        self.audio_rate = audio_rate
        self.prev_iq = 0.0 + 0.0j

        # FM normalization: ±75kHz deviation → ±1.0
        self.fm_norm = iq_rate / (2.0 * np.pi * 75000.0)

        # De-emphasis IIR
        self.de_alpha = 1.0 / (tau * iq_rate + 1.0)
        self.de_state = 0.0

        # 15 kHz audio LPF (anti-alias before decimation)
        self.lpf_taps = firwin(63, 15000, fs=iq_rate)
        self.lpf_state = np.zeros(len(self.lpf_taps) - 1)

        # Fractional resampler
        self.dec_ratio = iq_rate / audio_rate
        self.resamp_phase = 0.0

    def process(self, iq_float):
        n = len(iq_float) // 2
        if n < 2:
            return np.array([], dtype=np.float32)

        iq = iq_float[0::2] + 1j * iq_float[1::2]
        ext = np.concatenate([[self.prev_iq], iq])
        self.prev_iq = iq[-1]

        # FM discriminator
        pd = np.angle(ext[1:] * np.conj(ext[:-1])) * self.fm_norm

        # De-emphasis IIR
        a, b = self.de_alpha, 1.0 - self.de_alpha
        de, zi = lfilter([a], [1, -b], pd, zi=[self.de_state * b])
        self.de_state = float(zi[0]) / b if b != 0 else 0

        # 15 kHz audio LPF
        filtered, self.lpf_state = lfilter(self.lpf_taps, 1.0, de, zi=self.lpf_state)

        # Fractional resample
        r = self.dec_ratio; ph = self.resamp_phase; L = len(filtered)
        pos = np.arange(0, L - 1, r) + ph
        pos = pos[pos < L - 1]
        if len(pos) == 0:
            return np.array([], dtype=np.float32)
        idx = pos.astype(int); frac = (pos - idx).astype(np.float32)
        audio = filtered[idx] * (1 - frac) + filtered[idx + 1] * frac
        self.resamp_phase = (pos[-1] + r) - L

        return (audio * VOLUME).astype(np.float32)


# ── Globals ──
demod = None
iq_rate = IQ_BW
running = True
stats = {'blocks': 0, 'secs': 0.0, 'underruns': 0}

def on_message(ws, msg):
    global iq_rate, demod
    if not running: return
    if isinstance(msg, str):
        m = json.loads(msg)
        t = m.get('type', '')
        if t == 'iq_start':
            iq_rate = m.get('rate', IQ_BW)
            demod = FMDemod(iq_rate, AUDIO_RATE, DE_EMPH_TAU)
            print(f"  IQ stream: rate={iq_rate}, dec={iq_rate/AUDIO_RATE:.2f}:1")
        elif t == 'info':
            print(f"  Device: {m.get('freq',0)/1e6:.3f} MHz, {m.get('rate',0)/1e3:.0f} kSPS")
        elif t == 'error':
            print(f"  Error: {m.get('msg','?')}")
        return

    d = np.frombuffer(msg, dtype=np.uint8)
    if len(d) < 2: return
    mt = d[0]
    if mt == 3:
        p = d[1:]
        if len(p) % 2: p = p[:-1]
        iq = np.frombuffer(p.tobytes(), dtype=np.int16).astype(np.float32) / 32768.0
    elif mt == 2:
        iq = (d[1:].astype(np.float32) - 127.5) / 127.5
    else:
        return

    if demod is None:
        demod = FMDemod(iq_rate, AUDIO_RATE, DE_EMPH_TAU)

    audio = demod.process(iq)
    if len(audio) > 0:
        ring_push(audio)
        stats['blocks'] += 1
        stats['secs'] += len(audio) / AUDIO_RATE

def audio_cb(outdata, frames, ti, status):
    if status: stats['underruns'] += 1
    outdata[:, 0] = ring_pull(frames)

def on_open(ws):
    fhz = int(freq_mhz * 1e6)
    print(f"  Connected! Setting up {freq_mhz:.1f} MHz...")
    ws.send(json.dumps({"cmd": "sample_rate", "value": SAMPLE_RATE}))
    time.sleep(1.0)
    ws.send(json.dumps({"cmd": "freq", "value": fhz}))
    time.sleep(0.5)
    ws.send(json.dumps({"cmd": "gain", "value": 400}))
    time.sleep(0.3)
    ws.send(json.dumps({"cmd": "subscribe_iq", "offset": 0, "bw": IQ_BW}))

def on_error(ws, e): print(f"  WS Error: {e}")
def on_close(ws, *a): print("  Disconnected.")

def reporter():
    while running:
        time.sleep(5)
        buf_ms = ring_avail() / AUDIO_RATE * 1000
        print(f"  [Stats] blk={stats['blocks']}, audio={stats['secs']:.1f}s, "
              f"buf={buf_ms:.0f}ms, underruns={stats['underruns']}")

def main():
    global freq_mhz, running
    freq_mhz = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    host = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.233'

    print(f"  ESP32-P4 WebSDR FM Player v5")
    print(f"  {freq_mhz} MHz | {host} | Ctrl+C to stop")

    stream = sd.OutputStream(samplerate=AUDIO_RATE, channels=1, dtype='float32',
                              callback=audio_cb, blocksize=2048, latency='high')
    stream.start()
    threading.Thread(target=reporter, daemon=True).start()

    ws = websocket.WebSocketApp(f"wss://{host}/ws",
        on_open=on_open, on_message=on_message, on_error=on_error, on_close=on_close)
    try:
        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
    except KeyboardInterrupt:
        pass
    finally:
        running = False; stream.stop(); stream.close()
        print(f"\n  Done: {stats['blocks']} blk, {stats['secs']:.1f}s, {stats['underruns']} underruns")

if __name__ == '__main__':
    main()
