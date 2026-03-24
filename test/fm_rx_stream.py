#!/usr/bin/env python3
"""
Continuous WBFM Streaming Player via ESP32-P4 WebSDR

Receives IQ via WebSocket, demodulates WBFM with proper de-emphasis,
plays audio continuously via aplay in overlapping chunks.

Proven working parameters:
  - 300kSPS sample rate, 150kHz DDC BW
  - scipy lfilter de-emphasis with persistent state
  - 15kHz FIR lowpass for hiss removal
  - tanh soft limiter with fixed gain (no per-chunk normalization)
  - 2s chunk playback via aplay double-buffer

Usage: python3 fm_rx_stream.py [--freq 106.8e6] [--gain 200]
"""
import argparse, websocket, json, time, numpy as np, wave, subprocess, sys
from scipy.signal import lfilter, firwin

AUDIO_RATE = 48000

def main():
    p = argparse.ArgumentParser(description='WBFM Streaming Player')
    p.add_argument('--freq', type=float, default=106.8e6)
    p.add_argument('--host', default='192.168.1.232')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--gain', type=int, default=200)
    p.add_argument('--bw', type=int, default=150000)
    p.add_argument('--chunk', type=float, default=2.0, help='Chunk duration in seconds')
    p.add_argument('--volume', type=float, default=0.35)
    a = p.parse_args()

    ws = websocket.create_connection(f'ws://{a.host}:{a.port}/ws', timeout=5)
    ws.recv()
    ws.send(json.dumps({'cmd': 'sample_rate', 'value': 300000}))
    time.sleep(0.5)
    ws.send(json.dumps({'cmd': 'freq', 'value': int(a.freq)}))
    ws.send(json.dumps({'cmd': 'gain', 'value': a.gain}))
    time.sleep(0.3)
    ws.send(json.dumps({'cmd': 'subscribe_iq', 'offset': 0, 'bw': a.bw}))
    time.sleep(1)

    ws.settimeout(2)
    iq_rate = a.bw
    try:
        for _ in range(50):
            d = ws.recv()
            if isinstance(d, str):
                m = json.loads(d)
                if m.get('type') == 'iq_start':
                    iq_rate = m['rate']
                    break
    except:
        pass
    print(f'DDC: {iq_rate}Hz  Gain: {a.gain}')

    AR = AUDIO_RATE
    prevI, prevQ = 0.0, 0.0
    tau = 75e-6
    da = 1.0 / (tau * AR + 1)
    de_zi = np.array([0.0])
    lpf = firwin(15, 15000, fs=AR).astype(np.float32)

    freq_mhz = a.freq / 1e6
    print(f'Playing WBFM {freq_mhz:.1f} MHz ({a.chunk:.0f}s chunks)... Ctrl+C to stop')
    ws.settimeout(0.5)
    n = 0

    try:
        while True:
            all_audio = []
            t0 = time.time()
            while time.time() - t0 < a.chunk:
                try:
                    d = ws.recv()
                except:
                    continue
                if not isinstance(d, bytes) or len(d) < 3 or d[0] != 2:
                    continue
                raw = np.frombuffer(d[1:], dtype=np.uint8)
                iq = (raw.astype(np.float32) - 127.5) / 127.5
                I, Q = iq[0::2], iq[1::2]
                if len(I) < 2:
                    continue
                Ip = np.empty_like(I); Ip[0] = prevI; Ip[1:] = I[:-1]
                Qp = np.empty_like(Q); Qp[0] = prevQ; Qp[1:] = Q[:-1]
                fm = np.arctan2(Q*Ip - I*Qp, I*Ip + Q*Qp, dtype=np.float32) / np.float32(np.pi)
                prevI, prevQ = float(I[-1]), float(Q[-1])
                if iq_rate != AR:
                    r = iq_rate / AR
                    nn = max(1, int(len(fm) / r))
                    idx = np.clip((np.arange(nn) * r).astype(int), 0, len(fm) - 1)
                    fm = fm[idx]
                fm, de_zi = lfilter([da], [1, -(1 - da)], fm, zi=de_zi)
                all_audio.append(fm.astype(np.float32))

            if not all_audio:
                continue
            audio = np.concatenate(all_audio)
            audio = np.convolve(audio, lpf, mode='same').astype(np.float32)
            audio = np.tanh(audio * 2.0) * a.volume

            fname = f'/tmp/fmchunk_{n % 2}.wav'
            d16 = (audio * 32767).astype(np.int16)
            with wave.open(fname, 'w') as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(AR)
                w.writeframes(d16.tobytes())
            subprocess.Popen(['aplay', '-q', fname])
            n += 1
            pk = np.max(np.abs(audio))
            sys.stdout.write(f'\r  chunk {n}: {len(audio)/AR:.1f}s pk={pk:.3f}')
            sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    try:
        ws.send(json.dumps({'cmd': 'unsubscribe_iq'}))
        ws.close()
    except:
        pass
    print('Done.')

if __name__ == '__main__':
    main()
