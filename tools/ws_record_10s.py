#!/usr/bin/env python3
"""Record 10s of FM audio from WebSDR — optimized demod for quality analysis."""
import json, ssl, time, wave, threading
import numpy as np
import websocket

HOST = '192.168.1.233'
FREQ_MHZ = 100.0
IQ_BW = 150000
SAMPLE_RATE = 300000
AUDIO_RATE = 48000
RECORD_SECS = 10
DE_EMPH_TAU = 75e-6

# FM Demod state
prev_sample = 0.0 + 0.0j
de_state = 0.0
resamp_phase = 0.0
iq_rate = IQ_BW
audio_samples = []
start_time = None
done = False

def fm_demod(iq_float):
    """Vectorized FM discriminator + de-emphasis + fractional resampling."""
    global prev_sample, de_state, resamp_phase
    n = len(iq_float) // 2
    if n < 2:
        return np.array([], dtype=np.float32)

    iq = iq_float[0::2] + 1j * iq_float[1::2]

    # Phase difference discriminator
    iq_ext = np.concatenate([[prev_sample], iq])
    prev_sample = iq[-1]
    product = iq_ext[1:] * np.conj(iq_ext[:-1])
    phase_diff = np.angle(product) / np.pi

    # De-emphasis IIR
    alpha = 1.0 / (DE_EMPH_TAU * iq_rate + 1.0)
    beta = 1.0 - alpha
    deemph = np.empty_like(phase_diff)
    state = de_state
    for i in range(len(phase_diff)):
        state = alpha * phase_diff[i] + beta * state
        deemph[i] = state
    de_state = state

    # Fractional resampling with linear interpolation
    ratio = iq_rate / AUDIO_RATE
    out = []
    phase = resamp_phase
    src_len = len(deemph)
    while phase < src_len - 1:
        idx = int(phase)
        frac = phase - idx
        out.append(deemph[idx] * (1.0 - frac) + deemph[idx + 1] * frac)
        phase += ratio
    resamp_phase = phase - src_len

    if not out:
        return np.array([], dtype=np.float32)
    return np.tanh(np.array(out, dtype=np.float32) * 2.5) * 0.7

def on_message(ws, msg):
    global start_time, done, iq_rate
    if done:
        return
    if isinstance(msg, str):
        m = json.loads(msg)
        if m.get('type') == 'iq_start':
            iq_rate = m.get('rate', IQ_BW)
            print(f"  IQ rate: {iq_rate} Hz, dec ratio: {iq_rate/AUDIO_RATE:.2f}:1")
        elif m.get('type') == 'info':
            print(f"  Device: {m.get('freq',0)/1e6:.1f} MHz, {m.get('rate',0)/1e3:.0f} kSPS")
        return

    d = np.frombuffer(msg, dtype=np.uint8)
    if len(d) < 2:
        return
    t = d[0]

    # Direct int16 or uint8 to float32
    if t == 3:
        payload = d[1:]
        if len(payload) % 2:
            payload = payload[:-1]
        i16 = np.frombuffer(payload.tobytes(), dtype=np.int16)
        iq_float = i16.astype(np.float32) / 32768.0
    elif t == 2:
        iq_float = (d[1:].astype(np.float32) - 127.5) / 127.5
    else:
        return

    if start_time is None:
        start_time = time.time()
        print(f"  Recording started...")

    audio = fm_demod(iq_float)
    audio_samples.extend(audio.tolist())

    elapsed = time.time() - start_time
    if int(elapsed) > int(elapsed - 0.01) and int(elapsed) % 2 == 0:
        print(f"  {elapsed:.0f}s: {len(audio_samples)} samples ({len(audio_samples)/AUDIO_RATE:.1f}s audio)")

    if elapsed >= RECORD_SECS:
        done = True
        ws.close()

def on_open(ws):
    print(f"  Connected. Setting up...")
    # Set sample rate first and wait for it to take effect
    ws.send(json.dumps({"cmd": "sample_rate", "value": SAMPLE_RATE}))
    time.sleep(1.0)  # Wait for RTL-SDR sample rate change to complete
    ws.send(json.dumps({"cmd": "freq", "value": int(FREQ_MHZ * 1e6)}))
    time.sleep(0.5)
    ws.send(json.dumps({"cmd": "gain", "value": 0}))
    time.sleep(0.3)
    print(f"  Tuned to {FREQ_MHZ} MHz @ {SAMPLE_RATE/1e3:.0f} kSPS. Subscribing IQ...")
    ws.send(json.dumps({"cmd": "subscribe_iq", "offset": 0, "bw": IQ_BW}))

def on_close(ws, *a):
    print(f"  Disconnected. {len(audio_samples)} samples ({len(audio_samples)/AUDIO_RATE:.1f}s)")

print(f"Recording {RECORD_SECS}s FM from wss://{HOST}/ws at {FREQ_MHZ} MHz...")
ws = websocket.WebSocketApp(f"wss://{HOST}/ws",
    on_open=on_open, on_message=on_message, on_close=on_close)
ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

if audio_samples:
    out_path = '/tmp/websdr_fm_10s.wav'
    samples = np.array(audio_samples, dtype=np.float32)
    s16 = (samples * 32767).astype(np.int16)
    with wave.open(out_path, 'w') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(AUDIO_RATE)
        w.writeframes(s16.tobytes())
    print(f"  Saved: {out_path}")

    # Quality analysis
    print(f"\n  === Quality Analysis ===")
    print(f"  Duration: {len(s16)/AUDIO_RATE:.1f}s")
    print(f"  Peak: {np.max(np.abs(samples)):.4f}")
    print(f"  RMS: {np.sqrt(np.mean(samples**2)):.4f}")

    diffs = np.abs(np.diff(samples))
    print(f"  Large jumps (>0.3): {np.sum(diffs > 0.3)}")

    # SNR
    from numpy.fft import rfft
    if len(samples) >= AUDIO_RATE:
        fft = np.abs(rfft(samples[:AUDIO_RATE]))
        freqs = np.arange(len(fft)) * AUDIO_RATE / (2 * len(fft))
        ab = fft[(freqs > 300) & (freqs < 5000)]
        nb = fft[(freqs > 15000) & (freqs < 20000)]
        snr = 20 * np.log10(np.mean(ab) / (np.mean(nb) + 1e-10))
        print(f"  SNR (300-5kHz vs 15-20kHz): {snr:.1f} dB")

    # Gaps
    zero_run = 0
    gaps = 0
    for s in samples:
        if abs(s) < 0.001:
            zero_run += 1
        else:
            if zero_run > AUDIO_RATE * 0.01:
                gaps += 1
            zero_run = 0
    print(f"  Audio gaps (>10ms): {gaps}")

    # Envelope modulation
    env = np.abs(samples[:AUDIO_RATE])
    env_fft = np.abs(rfft(env))
    env_freqs = np.arange(len(env_fft)) * AUDIO_RATE / (2 * len(env_fft))
    low = env_fft[(env_freqs > 1) & (env_freqs < 50)]
    low_f = env_freqs[(env_freqs > 1) & (env_freqs < 50)]
    if len(low) > 0:
        pk = np.argmax(low)
        print(f"  Envelope modulation peak: {low_f[pk]:.1f} Hz (amp: {low[pk]:.2f})")
