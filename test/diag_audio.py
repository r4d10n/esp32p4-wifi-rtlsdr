#!/usr/bin/env python3
"""
ESP32-P4 FM Radio Audio Diagnostic Script
Connects to device at 192.168.1.233:8080 and captures diagnostic data
across multiple gain settings.
"""

import asyncio
import ssl
import json
import time
import struct
import urllib.request
import urllib.error
import numpy as np
from scipy import signal as scipy_signal
import websockets

DEVICE_HOST = "192.168.1.233"
DEVICE_PORT = 8080
BASE_URL = f"https://{DEVICE_HOST}:{DEVICE_PORT}"
WSS_URL = f"wss://{DEVICE_HOST}:{DEVICE_PORT}/ws"
GAIN_VALUES = [9, 77, 229, 496]
SAMPLE_RATE = 48000
CAPTURE_SECONDS = 2
STABILIZE_SECONDS = 2


def make_ssl_context():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


def http_get(path):
    url = f"{BASE_URL}{path}"
    req = urllib.request.Request(url)
    ctx = make_ssl_context()
    try:
        with urllib.request.urlopen(req, context=ctx, timeout=10) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        body = e.read().decode()
        raise RuntimeError(f"HTTP {e.code} on GET {path}: {body}")


def http_post(path, payload):
    url = f"{BASE_URL}{path}"
    data = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data,
                                  headers={"Content-Type": "application/json"},
                                  method="POST")
    ctx = make_ssl_context()
    try:
        with urllib.request.urlopen(req, context=ctx, timeout=10) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        body = e.read().decode()
        raise RuntimeError(f"HTTP {e.code} on POST {path}: {body}")


def decode_pcm_frames(raw_bytes):
    """Convert raw bytes to int16 numpy array."""
    n_samples = len(raw_bytes) // 2
    if n_samples == 0:
        return np.array([], dtype=np.int16)
    samples = np.frombuffer(raw_bytes[:n_samples * 2], dtype=np.int16)
    return samples


def compute_rms(samples):
    if len(samples) == 0:
        return 0.0
    return float(np.sqrt(np.mean(samples.astype(np.float64) ** 2)))


def compute_peak(samples):
    if len(samples) == 0:
        return 0
    return int(np.max(np.abs(samples)))


def compute_zero_crossing_rate(samples):
    if len(samples) < 2:
        return 0.0
    signs = np.sign(samples.astype(np.float64))
    # treat zero as positive
    signs[signs == 0] = 1
    crossings = np.sum(np.diff(signs) != 0)
    return float(crossings) / len(samples)


def compute_autocorrelation_lag1(samples):
    """Normalized autocorrelation at lag 1. White noise -> ~0, structured audio -> high."""
    if len(samples) < 2:
        return 0.0
    x = samples.astype(np.float64)
    x -= x.mean()
    var = np.var(x)
    if var < 1e-10:
        return 0.0
    return float(np.corrcoef(x[:-1], x[1:])[0, 1])


def compute_spectral_content(samples, sr=SAMPLE_RATE, top_n=8):
    """FFT and return top_n dominant frequency components with amplitudes."""
    if len(samples) < 256:
        return []
    x = samples.astype(np.float64)
    # Use a window to reduce spectral leakage
    window = np.hanning(len(x))
    x_windowed = x * window
    fft_vals = np.fft.rfft(x_windowed)
    freqs = np.fft.rfftfreq(len(x), d=1.0 / sr)
    magnitudes = np.abs(fft_vals)
    # Get top N peaks
    peak_indices = np.argsort(magnitudes)[::-1][:top_n]
    result = []
    for idx in sorted(peak_indices, key=lambda i: freqs[i]):
        result.append((float(freqs[idx]), float(magnitudes[idx])))
    return result


def classify_noise(samples, autocorr_lag1):
    """Heuristic classification of the captured audio."""
    rms = compute_rms(samples)
    zcr = compute_zero_crossing_rate(samples)

    if rms < 10:
        return "SILENCE (near-zero signal)"
    if abs(autocorr_lag1) < 0.05 and zcr > 0.3:
        return "WHITE NOISE (uncorrelated, flat spectrum)"
    if abs(autocorr_lag1) > 0.5:
        return "STRUCTURED/CORRELATED (likely audio or tonal signal)"
    if abs(autocorr_lag1) > 0.2:
        return "COLORED NOISE (some structure, possibly FM quieting noise)"
    return f"AMBIGUOUS (autocorr={autocorr_lag1:.3f}, zcr={zcr:.3f})"


async def capture_audio_ws(gain_value):
    """Open WSS WebSocket, send audio_on, capture 2 seconds of PCM, send audio_off."""
    ssl_ctx = make_ssl_context()
    raw_bytes = bytearray()
    frame_count = 0
    capture_deadline = CAPTURE_SECONDS + 0.5  # extra buffer

    print(f"    Connecting to {WSS_URL} ...")

    try:
        async with websockets.connect(
            WSS_URL,
            ssl=ssl_ctx,
            open_timeout=10,
            ping_interval=None,
        ) as ws:
            print(f"    WebSocket connected. Sending audio_on ...")
            await ws.send(json.dumps({"cmd": "audio_on"}))

            start_time = asyncio.get_event_loop().time()
            deadline = start_time + capture_deadline

            while True:
                remaining = deadline - asyncio.get_event_loop().time()
                if remaining <= 0:
                    break
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=min(remaining, 1.0))
                    if isinstance(msg, bytes):
                        raw_bytes.extend(msg)
                        frame_count += 1
                    elif isinstance(msg, str):
                        # Text frame — status/control message
                        try:
                            parsed = json.loads(msg)
                            print(f"    [WS text] {parsed}")
                        except Exception:
                            print(f"    [WS text] {msg[:120]}")
                except asyncio.TimeoutError:
                    break

            print(f"    Sending audio_off ...")
            await ws.send(json.dumps({"cmd": "audio_off"}))
            # Brief drain
            try:
                await asyncio.wait_for(ws.recv(), timeout=0.5)
            except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
                pass

    except websockets.exceptions.WebSocketException as e:
        print(f"    [WS ERROR] {e}")
        return np.array([], dtype=np.int16), 0

    samples = decode_pcm_frames(bytes(raw_bytes))
    print(f"    Captured {frame_count} frames, {len(raw_bytes)} bytes -> {len(samples)} samples "
          f"({len(samples)/SAMPLE_RATE:.2f}s @ {SAMPLE_RATE}Hz)")
    return samples, frame_count


def set_gain(gain_value):
    """POST gain setting to device."""
    try:
        result = http_post("/api/gain", {"gain": gain_value})
        print(f"    Set gain={gain_value}: {result}")
    except RuntimeError as e:
        # Try alternate endpoint
        try:
            result = http_post("/api/config", {"gain": gain_value})
            print(f"    Set gain={gain_value} via /api/config: {result}")
        except RuntimeError as e2:
            print(f"    [WARN] Could not set gain={gain_value}: {e} / {e2}")


def get_status():
    """GET /api/status and return the parsed JSON."""
    try:
        return http_get("/api/status")
    except RuntimeError as e:
        print(f"    [WARN] GET /api/status failed: {e}")
        return {}


def print_section(title):
    width = 70
    print()
    print("=" * width)
    print(f"  {title}")
    print("=" * width)


def print_analysis(gain, status, samples):
    if len(samples) == 0:
        print("  [NO DATA] No PCM samples captured.")
        return

    rms = compute_rms(samples)
    peak = compute_peak(samples)
    zcr = compute_zero_crossing_rate(samples)
    autocorr = compute_autocorrelation_lag1(samples)
    classification = classify_noise(samples, autocorr)
    spectral = compute_spectral_content(samples)

    signal_strength = status.get("signal_strength", status.get("rssi", "N/A"))
    freq = status.get("frequency", status.get("freq", "N/A"))

    print(f"  Gain setting   : {gain}")
    print(f"  Signal strength: {signal_strength}")
    print(f"  Frequency      : {freq}")
    print(f"  Samples        : {len(samples)} ({len(samples)/SAMPLE_RATE:.2f}s)")
    print(f"  RMS amplitude  : {rms:.1f}  (int16 range: 0-32767)")
    print(f"  Peak amplitude : {peak}  ({peak/32767*100:.1f}% of full scale)")
    print(f"  Zero-cross rate: {zcr:.4f}  (white noise ~0.5, audio ~0.1-0.3)")
    print(f"  Autocorr lag-1 : {autocorr:.4f}  (white ~0, structured audio >0.3)")
    print(f"  Classification : {classification}")

    if spectral:
        print(f"  Top spectral components:")
        for freq_hz, mag in spectral[:8]:
            bar = "#" * min(40, int(mag / (max(m for _, m in spectral) + 1e-9) * 40))
            print(f"    {freq_hz:8.1f} Hz  |{bar}")
    else:
        print("  Spectral analysis: insufficient data")


async def run_diagnostics():
    print_section("ESP32-P4 FM Radio Audio Diagnostic")
    print(f"  Target  : {BASE_URL}")
    print(f"  WS      : {WSS_URL}")
    print(f"  Gains   : {GAIN_VALUES}")
    print(f"  Capture : {CAPTURE_SECONDS}s per gain @ {SAMPLE_RATE}Hz")

    # Initial connectivity check
    print()
    print("Checking device connectivity...")
    initial_status = get_status()
    if initial_status:
        print(f"  Device status: {json.dumps(initial_status, indent=2)}")
    else:
        print("  [WARN] No initial status — device may be unreachable or API differs")

    results = {}

    for gain in GAIN_VALUES:
        print_section(f"Gain = {gain}")

        print(f"  Setting gain to {gain}...")
        set_gain(gain)

        print(f"  Waiting {STABILIZE_SECONDS}s for signal to stabilize...")
        await asyncio.sleep(STABILIZE_SECONDS)

        print(f"  Getting status...")
        status = get_status()
        if status:
            print(f"  Status: {json.dumps(status)}")

        print(f"  Capturing audio...")
        samples, frame_count = await capture_audio_ws(gain)

        print()
        print_analysis(gain, status, samples)

        results[gain] = {
            "status": status,
            "samples": samples,
            "rms": compute_rms(samples),
            "peak": compute_peak(samples),
            "autocorr_lag1": compute_autocorrelation_lag1(samples),
            "zcr": compute_zero_crossing_rate(samples),
            "frame_count": frame_count,
        }

    # Cross-gain summary
    print_section("Cross-Gain Analysis Summary")
    print(f"  {'Gain':>6}  {'RMS':>8}  {'Peak':>6}  {'Autocorr':>9}  {'ZCR':>7}  {'Frames':>6}")
    print(f"  {'-'*6}  {'-'*8}  {'-'*6}  {'-'*9}  {'-'*7}  {'-'*6}")
    for gain in GAIN_VALUES:
        r = results[gain]
        print(f"  {gain:>6}  {r['rms']:>8.1f}  {r['peak']:>6}  "
              f"{r['autocorr_lag1']:>9.4f}  {r['zcr']:>7.4f}  {r['frame_count']:>6}")

    # Diagnosis
    print()
    print("  --- Diagnosis ---")
    rms_values = [results[g]["rms"] for g in GAIN_VALUES]
    valid_rms = [r for r in rms_values if r > 0]
    if not valid_rms or max(valid_rms) < 1.0:
        print("  RESULT: Audio path appears BROKEN — all RMS near zero.")
        print("          No PCM data arriving from the ESP32 audio pipeline.")
    elif max(valid_rms) - min(valid_rms) < 5.0:
        print("  RESULT: RMS does NOT change with gain — audio path may be DISCONNECTED")
        print("          from the RF/gain control, OR gain values have no effect on output.")
    else:
        rms_trend = "increasing" if rms_values[-1] > rms_values[0] else "decreasing"
        print(f"  RESULT: RMS is {rms_trend} with gain — gain control IS affecting audio level.")

    autocorr_values = [results[g]["autocorr_lag1"] for g in GAIN_VALUES]
    mean_autocorr = np.mean([abs(a) for a in autocorr_values])
    if mean_autocorr < 0.05:
        print("  NOISE:  White/uncorrelated noise — no structured FM audio detected.")
    elif mean_autocorr > 0.4:
        print("  NOISE:  Structured/correlated signal — FM demodulation may be working.")
    else:
        print(f"  NOISE:  Intermediate correlation ({mean_autocorr:.3f}) — colored noise or weak FM.")

    print()
    print("Diagnostic complete.")


if __name__ == "__main__":
    asyncio.run(run_diagnostics())
