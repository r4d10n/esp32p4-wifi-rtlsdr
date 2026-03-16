#!/usr/bin/env python3
"""
Continuous WBFM Streaming Player via ESP32-P4 WebSDR

Receives IQ via WebSocket, demodulates WBFM, plays audio continuously.
Proven working parameters: 300kSPS, 150kHz DDC BW, scipy de-emphasis.

Usage: python3 fm_rx_stream.py [--freq 106.8e6] [--host 192.168.1.232]
"""

import argparse, json, sys, time, threading, queue
import numpy as np
import sounddevice as sd
import websocket
from scipy.signal import lfilter

AUDIO_RATE = 48000

class WBFMReceiver:
    def __init__(self, host, port, freq, gain, bw):
        self.host = host
        self.port = port
        self.freq = int(freq)
        self.gain = gain
        self.bw = bw
        self.iq_rate = 0
        self.prevI = 0.0
        self.prevQ = 0.0
        self.running = True
        self.audio_q = queue.Queue(maxsize=200)
        # De-emphasis coefficients
        tau = 75e-6
        dt = 1.0 / AUDIO_RATE
        self.de_alpha = dt / (tau + dt)
        self.de_b = [self.de_alpha]
        self.de_a = [1, -(1 - self.de_alpha)]

    def demod(self, raw):
        iq = (raw.astype(np.float32) - 127.5) / 127.5
        I, Q = iq[0::2], iq[1::2]
        if len(I) < 2:
            return np.array([], dtype=np.float32)
        Ip = np.empty_like(I); Ip[0] = self.prevI; Ip[1:] = I[:-1]
        Qp = np.empty_like(Q); Qp[0] = self.prevQ; Qp[1:] = Q[:-1]
        audio = np.arctan2(Q*Ip - I*Qp, I*Ip + Q*Qp).astype(np.float32) / np.float32(np.pi)
        self.prevI = float(I[-1])
        self.prevQ = float(Q[-1])
        # Resample to 48kHz
        if self.iq_rate > 0 and self.iq_rate != AUDIO_RATE:
            r = self.iq_rate / AUDIO_RATE
            n = max(1, int(len(audio) / r))
            idx = np.arange(n) * r
            lo = np.clip(idx.astype(int), 0, len(audio) - 1)
            hi = np.clip(lo + 1, 0, len(audio) - 1)
            f = (idx - lo).astype(np.float32)
            audio = audio[lo] * (1 - f) + audio[hi] * f
        # De-emphasis
        audio = lfilter(self.de_b, self.de_a, audio).astype(np.float32)
        return audio

    def ws_thread(self):
        while self.running:
            try:
                ws = websocket.create_connection(
                    f'ws://{self.host}:{self.port}/ws', timeout=5)
                ws.recv()  # device info
                ws.send(json.dumps({'cmd': 'sample_rate', 'value': 300000}))
                time.sleep(0.3)
                ws.send(json.dumps({'cmd': 'freq', 'value': self.freq}))
                ws.send(json.dumps({'cmd': 'gain', 'value': self.gain}))
                time.sleep(0.2)
                ws.send(json.dumps({'cmd': 'subscribe_iq',
                                     'offset': 0, 'bw': self.bw}))
                time.sleep(0.5)

                # Get DDC rate
                ws.settimeout(2)
                try:
                    for _ in range(50):
                        d = ws.recv()
                        if isinstance(d, str):
                            m = json.loads(d)
                            if m.get('type') == 'iq_start':
                                self.iq_rate = m['rate']
                                print(f'[RX] DDC rate: {self.iq_rate}Hz')
                                break
                except:
                    pass
                if self.iq_rate == 0:
                    self.iq_rate = 150000

                ws.settimeout(0.5)
                print(f'[RX] Streaming from {self.freq/1e6:.3f} MHz...')

                while self.running:
                    try:
                        d = ws.recv()
                    except:
                        continue
                    if not isinstance(d, bytes) or len(d) < 3 or d[0] != 2:
                        continue
                    audio = self.demod(np.frombuffer(d[1:], dtype=np.uint8))
                    if len(audio) > 0:
                        try:
                            self.audio_q.put_nowait(audio)
                        except queue.Full:
                            self.audio_q.get()  # drop oldest
                            self.audio_q.put_nowait(audio)

            except Exception as e:
                if self.running:
                    print(f'[RX] Reconnecting... ({e})')
                    time.sleep(2)

    def audio_callback(self, outdata, frames, time_info, status):
        buf = np.zeros(frames, dtype=np.float32)
        pos = 0
        while pos < frames:
            try:
                chunk = self.audio_q.get_nowait()
                n = min(len(chunk), frames - pos)
                buf[pos:pos+n] = chunk[:n] * 0.7
                if n < len(chunk):
                    # Put remainder back
                    try:
                        self.audio_q.put_nowait(chunk[n:])
                    except:
                        pass
                pos += n
            except queue.Empty:
                break
        outdata[:, 0] = buf

    def run(self):
        freq_mhz = self.freq / 1e6
        print(f'[WBFM] {freq_mhz:.1f} MHz | gain={self.gain} | bw={self.bw/1e3:.0f}kHz')

        # Start WS receiver thread
        t = threading.Thread(target=self.ws_thread, daemon=True)
        t.start()

        # Start audio output
        stream = sd.OutputStream(
            samplerate=AUDIO_RATE, channels=1, blocksize=2048,
            dtype='float32', callback=self.audio_callback)
        stream.start()
        print(f'[AUDIO] Output started on default device')
        print(f'[AUDIO] Press Ctrl+C to stop\n')

        try:
            t0 = time.time()
            while True:
                time.sleep(1)
                e = int(time.time() - t0)
                m, s = divmod(e, 60)
                qs = self.audio_q.qsize()
                sys.stdout.write(
                    f'\r[RX] {freq_mhz:.1f} MHz | {m:02d}:{s:02d} | '
                    f'buf:{qs} | rate:{self.iq_rate}Hz')
                sys.stdout.flush()
        except KeyboardInterrupt:
            print('\n\nStopping...')
            self.running = False
            stream.stop()
            stream.close()
            print('Done.')

def main():
    p = argparse.ArgumentParser(description='WBFM Streaming Player')
    p.add_argument('--freq', type=float, default=106.8e6)
    p.add_argument('--host', default='192.168.1.232')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--gain', type=int, default=400)
    p.add_argument('--bw', type=int, default=150000)
    a = p.parse_args()
    WBFMReceiver(a.host, a.port, a.freq, a.gain, a.bw).run()

if __name__ == '__main__':
    main()
