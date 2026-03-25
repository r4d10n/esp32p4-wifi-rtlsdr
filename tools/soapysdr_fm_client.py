#!/usr/bin/env python3
"""
SoapySDR Remote FM Client — connects via RPC, receives IQ over UDP, demodulates WBFM.

Usage: python3 tools/soapysdr_fm_client.py [freq_mhz] [host] [port] [gain_db]

SPDX-License-Identifier: GPL-2.0-or-later
"""
import sys, struct, socket, time, threading
import numpy as np
import sounddevice as sd
from scipy.signal import firwin, lfilter

HOST = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.233'
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 55132
FREQ_MHZ = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
GAIN_DB = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0
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

# ── SoapySDR RPC ──
class SoapyRPC:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=10)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    def recv_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk: raise ConnectionError("disconnected")
            buf += chunk
        return buf
    def rpc(self, cmd, payload=b''):
        self.sock.sendall(struct.pack('<II', cmd, len(payload)) + payload)
        hdr = self.recv_exact(8)
        resp_cmd, resp_len = struct.unpack('<II', hdr)
        body = self.recv_exact(resp_len) if resp_len > 0 else b''
        return resp_cmd, body
    def close(self):
        try: self.sock.close()
        except: pass

running = True; stats = {'pkts':0, 'bytes':0, 'audio':0.0, 'ur':0}

def audio_cb(out, frames, ti, status):
    if status: stats['ur'] += 1
    out[:, 0] = ring_pull(frames)

print(f"  SoapySDR Remote FM Client | {FREQ_MHZ} MHz | {HOST}:{PORT} | gain={GAIN_DB}dB")

rpc = SoapyRPC(HOST, PORT)
print(f"  Connected")

# Discover
_, body = rpc.rpc(0x00); print(f"  Discover: {body.decode('utf-8','replace').rstrip(chr(0))}")
# Make device
rpc.rpc(0x01); print(f"  Device created")
# Hardware key
_, body = rpc.rpc(0x10); print(f"  HW: {body.decode('utf-8','replace').rstrip(chr(0))}")
# Set frequency
rpc.rpc(0x20, struct.pack('<d', FREQ_MHZ*1e6)); print(f"  Freq: {FREQ_MHZ} MHz")
# Set sample rate
rpc.rpc(0x30, struct.pack('<d', float(IQ_RATE))); print(f"  Rate: {IQ_RATE/1e3:.0f} kSPS")
# Set gain mode manual + gain
rpc.rpc(0x43, struct.pack('<I', 1))
rpc.rpc(0x40, struct.pack('<d', GAIN_DB)); print(f"  Gain: {GAIN_DB} dB")

# Bind UDP socket
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.settimeout(2.0)
udp_sock.bind(('0.0.0.0', 0))
local_port = udp_sock.getsockname()[1]

# Setup stream with our port
_, body = rpc.rpc(0x50, struct.pack('<H', local_port))
srv_port = struct.unpack('<I', body[:4])[0] if len(body)>=4 else 0
print(f"  Stream: local_udp={local_port}, server_resp={srv_port}")

# Activate
rpc.rpc(0x51); print(f"  Streaming activated")

stream = sd.OutputStream(samplerate=AUDIO_RATE, channels=1, dtype='float32',
                          callback=audio_cb, blocksize=2048, latency='high')
stream.start()
demod = FMDemod(IQ_RATE)
print(f"  Playing — Ctrl+C to stop")

try:
    while running:
        try:
            data, _ = udp_sock.recvfrom(8192)
            stats['pkts'] += 1
            iq_data = data[4:] if len(data)>4 else data
            stats['bytes'] += len(iq_data)
            iq = (np.frombuffer(iq_data, dtype=np.uint8).astype(np.float32)-127.5)/127.5
            audio = demod.process(iq)
            if len(audio)>0: ring_push(audio); stats['audio'] += len(audio)/AUDIO_RATE
            if stats['pkts'] % 500 == 0:
                print(f"  [Stats] {stats['pkts']} pkts, {stats['bytes']/1024:.0f}KB, "
                      f"{stats['audio']:.1f}s, ur={stats['ur']}")
        except socket.timeout:
            if stats['pkts']==0: print(f"  No UDP data — check server"); break
except KeyboardInterrupt: pass
finally:
    running = False; stream.stop(); stream.close()
    try: rpc.rpc(0x52)  # deactivate
    except: pass
    try: rpc.rpc(0x53)  # close stream
    except: pass
    rpc.close(); udp_sock.close()
    print(f"\n  Done: {stats['pkts']} pkts, {stats['audio']:.1f}s, ur={stats['ur']}")
