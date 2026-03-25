#!/usr/bin/env python3
"""
RTL-TCP FM Client — connects to rtl_tcp server, demodulates WBFM, plays audio.

Usage: python3 tools/rtltcp_fm_client.py [freq_mhz] [host] [port] [gain_tenth_db]

Compatible with: rtl_tcp, ESP32-P4 RTL-TCP server, any rtl_tcp-compatible server.

SPDX-License-Identifier: GPL-2.0-or-later
"""
import sys, struct, socket, time, threading
import numpy as np
import sounddevice as sd
from scipy.signal import firwin, lfilter

HOST = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.233'
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 1234
FREQ_MHZ = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
GAIN = int(sys.argv[4]) if len(sys.argv) > 4 else 400  # tenths of dB
IQ_RATE = 250000
AUDIO_RATE = 48000
VOLUME = 0.5

# ── Audio ring buffer ──
ring = np.zeros(int(AUDIO_RATE * 2), dtype=np.float32)
ring_wr = 0; ring_rd = 0; ring_sz = len(ring); ring_lock = threading.Lock()
def ring_avail(): return (ring_wr - ring_rd) % ring_sz
def ring_push(s):
    global ring_wr
    n = len(s)
    if n == 0: return
    with ring_lock:
        w = ring_wr; e = w + n
        if e <= ring_sz: ring[w:e] = s
        else: f = ring_sz - w; ring[w:] = s[:f]; ring[:n-f] = s[f:]
        ring_wr = e % ring_sz
def ring_pull(n):
    global ring_rd
    with ring_lock:
        a = ring_avail(); t = min(n, a)
        if t == 0: return np.zeros(n, dtype=np.float32)
        r = ring_rd; e = r + t
        if e <= ring_sz: out = ring[r:e].copy()
        else: f = ring_sz - r; out = np.concatenate([ring[r:], ring[:t-f]])
        ring_rd = e % ring_sz
        return np.concatenate([out, np.zeros(n-t, dtype=np.float32)]) if t < n else out

# ── FM Demodulator ──
class FMDemod:
    def __init__(self, iq_rate):
        self.prev_iq = 0.0+0.0j
        self.fm_norm = iq_rate / (2*np.pi*75000)
        a = 1/(75e-6*iq_rate+1); self.de_a = a; self.de_s = 0.0
        self.lpf = firwin(63, 15000, fs=iq_rate); self.lpf_s = np.zeros(62)
        self.ratio = iq_rate / AUDIO_RATE; self.phase = 0.0
    def process(self, iq_float):
        n = len(iq_float)//2
        if n < 2: return np.array([], dtype=np.float32)
        iq = iq_float[0::2]+1j*iq_float[1::2]
        ext = np.concatenate([[self.prev_iq], iq]); self.prev_iq = iq[-1]
        pd = np.angle(ext[1:]*np.conj(ext[:-1]))*self.fm_norm
        de, zi = lfilter([self.de_a], [1, -(1-self.de_a)], pd, zi=[self.de_s*(1-self.de_a)])
        self.de_s = float(zi[0])/(1-self.de_a) if self.de_a < 1 else 0
        filt, self.lpf_s = lfilter(self.lpf, 1, de, zi=self.lpf_s)
        L = len(filt); pos = np.arange(0, L-1, self.ratio)+self.phase; pos = pos[pos<L-1]
        if len(pos)==0: return np.array([],dtype=np.float32)
        idx = pos.astype(int); frac = (pos-idx).astype(np.float32)
        audio = filt[idx]*(1-frac)+filt[idx+1]*frac
        self.phase = (pos[-1]+self.ratio)-L
        return (audio*VOLUME).astype(np.float32)

running = True; stats = {'bytes':0, 'audio':0.0, 'ur':0}

def audio_cb(out, frames, ti, status):
    if status: stats['ur'] += 1
    out[:, 0] = ring_pull(frames)

def receiver(sock, demod):
    global running
    while running:
        try:
            data = sock.recv(16384)
            if not data: break
            stats['bytes'] += len(data)
            iq = (np.frombuffer(data, dtype=np.uint8).astype(np.float32)-127.5)/127.5
            audio = demod.process(iq)
            if len(audio) > 0:
                ring_push(audio); stats['audio'] += len(audio)/AUDIO_RATE
        except: break
    running = False

print(f"  RTL-TCP FM Client | {FREQ_MHZ} MHz | {HOST}:{PORT} | gain={GAIN/10:.1f}dB")
sock = socket.create_connection((HOST, PORT), timeout=10)
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

# DongleInfo
info = sock.recv(12)
if len(info)==12:
    m,t,g = struct.unpack('>III', info)
    print(f"  DongleInfo: tuner={t}, gains={g}")

# Commands
def cmd(c, p): sock.sendall(struct.pack('>BI', c, p))
cmd(0x02, IQ_RATE);  time.sleep(0.5)
cmd(0x01, int(FREQ_MHZ*1e6)); time.sleep(0.3)
cmd(0x03, 1); cmd(0x04, GAIN)

stream = sd.OutputStream(samplerate=AUDIO_RATE, channels=1, dtype='float32',
                          callback=audio_cb, blocksize=2048, latency='high')
stream.start()

demod = FMDemod(IQ_RATE)
t = threading.Thread(target=receiver, args=(sock, demod), daemon=True); t.start()
print(f"  Streaming — Ctrl+C to stop")

try:
    while running:
        time.sleep(5)
        print(f"  [Stats] {stats['bytes']/1024:.0f}KB, {stats['audio']:.1f}s audio, "
              f"buf={ring_avail()/AUDIO_RATE*1000:.0f}ms, ur={stats['ur']}")
except KeyboardInterrupt: pass
finally:
    running = False; stream.stop(); stream.close(); sock.close()
    print(f"\n  Done: {stats['audio']:.1f}s audio, {stats['ur']} underruns")
