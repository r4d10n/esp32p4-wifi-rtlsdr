#!/usr/bin/env python3
"""
Spectral profile analysis for RTL-SDR IQ captures.
Creates reference profiles from direct USB captures and compares
with WiFi-bridged captures.

Usage:
  python3 analyze_spectrum.py reference <iq_file> [--freq 936e6] [--rate 1024000]
  python3 analyze_spectrum.py compare <reference.json> <test_file>
  python3 analyze_spectrum.py capture_wifi <output_file> [--host 192.168.1.232] [--port 1234]
"""

import numpy as np
import json
import sys
import os

def load_iq(filename, count=-1):
    """Load uint8 IQ file, return complex float centered at 0."""
    raw = np.fromfile(filename, dtype=np.uint8, count=count)
    # Interleaved I,Q uint8 (128 = 0.0)
    i = raw[0::2].astype(np.float32) - 127.5
    q = raw[1::2].astype(np.float32) - 127.5
    return i + 1j * q

def compute_spectrum(iq, fft_size=1024, sample_rate=1024000, center_freq=936e6):
    """Compute averaged power spectrum in dB."""
    n_segments = len(iq) // fft_size
    if n_segments == 0:
        return None, None

    # Reshape into segments
    iq_segments = iq[:n_segments * fft_size].reshape(n_segments, fft_size)

    # Apply window
    window = np.hanning(fft_size)
    windowed = iq_segments * window

    # FFT and average power
    spectra = np.fft.fftshift(np.fft.fft(windowed, axis=1), axes=1)
    power = np.mean(np.abs(spectra) ** 2, axis=0)
    power_db = 10 * np.log10(power + 1e-12)

    # Frequency axis
    freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, 1.0 / sample_rate))
    freqs_hz = freqs + center_freq

    return freqs_hz, power_db

def find_peaks(freqs, power_db, threshold_db=-60, min_width_hz=10000):
    """Find spectral peaks above threshold."""
    peaks = []
    above = power_db > threshold_db
    in_peak = False
    start_idx = 0

    for i in range(len(above)):
        if above[i] and not in_peak:
            in_peak = True
            start_idx = i
        elif not above[i] and in_peak:
            in_peak = False
            width_hz = freqs[i-1] - freqs[start_idx]
            if width_hz >= min_width_hz:
                peak_idx = start_idx + np.argmax(power_db[start_idx:i])
                peaks.append({
                    'freq_hz': float(freqs[peak_idx]),
                    'power_db': float(power_db[peak_idx]),
                    'start_hz': float(freqs[start_idx]),
                    'end_hz': float(freqs[i-1]),
                    'width_hz': float(width_hz),
                    'center_hz': float((freqs[start_idx] + freqs[i-1]) / 2),
                })

    return peaks

def create_profile(filename, sample_rate=1024000, center_freq=936e6, fft_size=1024):
    """Create spectral profile from IQ file."""
    print(f"Loading {filename}...")
    iq = load_iq(filename)
    print(f"  Loaded {len(iq)} IQ samples ({len(iq)/sample_rate:.2f}s)")

    # Basic IQ statistics
    i_mean = float(np.mean(iq.real))
    q_mean = float(np.mean(iq.imag))
    i_std = float(np.std(iq.real))
    q_std = float(np.std(iq.imag))
    dc_offset = float(np.abs(np.mean(iq)))
    print(f"  I: mean={i_mean:.2f} std={i_std:.2f}")
    print(f"  Q: mean={q_mean:.2f} std={q_std:.2f}")
    print(f"  DC offset: {dc_offset:.2f}")

    # Compute spectrum
    freqs, power_db = compute_spectrum(iq, fft_size, sample_rate, center_freq)
    print(f"  Spectrum: {len(freqs)} bins, {fft_size}-point FFT, {len(iq)//fft_size} averages")

    # Overall power stats
    total_power_db = float(10 * np.log10(np.sum(10 ** (power_db / 10))))
    mean_power_db = float(np.mean(power_db))
    peak_power_db = float(np.max(power_db))
    noise_floor_db = float(np.percentile(power_db, 10))
    print(f"  Total power: {total_power_db:.1f} dB")
    print(f"  Mean power:  {mean_power_db:.1f} dB")
    print(f"  Peak power:  {peak_power_db:.1f} dB")
    print(f"  Noise floor: {noise_floor_db:.1f} dB (10th percentile)")

    # Find peaks (GSM channels ~200 kHz wide)
    peaks = find_peaks(freqs, power_db, threshold_db=noise_floor_db + 10, min_width_hz=50000)
    print(f"  Found {len(peaks)} spectral peaks:")
    for i, p in enumerate(peaks):
        print(f"    Peak {i}: {p['center_hz']/1e6:.3f} MHz, "
              f"power={p['power_db']:.1f} dB, width={p['width_hz']/1e3:.0f} kHz")

    # Spectral shape signature (normalized power in 10 bands)
    n_bands = 20
    band_power = []
    band_size = len(power_db) // n_bands
    for i in range(n_bands):
        bp = float(np.mean(power_db[i*band_size:(i+1)*band_size]))
        band_power.append(bp)
    band_power_normalized = [bp - noise_floor_db for bp in band_power]

    profile = {
        'source': os.path.basename(filename),
        'capture_type': 'direct_usb',
        'center_freq_hz': center_freq,
        'sample_rate_hz': sample_rate,
        'fft_size': fft_size,
        'n_samples': len(iq),
        'duration_s': float(len(iq) / sample_rate),
        'iq_stats': {
            'i_mean': i_mean, 'q_mean': q_mean,
            'i_std': i_std, 'q_std': q_std,
            'dc_offset': dc_offset,
        },
        'power_stats': {
            'total_db': total_power_db,
            'mean_db': mean_power_db,
            'peak_db': peak_power_db,
            'noise_floor_db': noise_floor_db,
            'dynamic_range_db': float(peak_power_db - noise_floor_db),
        },
        'peaks': peaks,
        'spectral_shape': {
            'n_bands': n_bands,
            'band_power_db': band_power,
            'band_power_above_noise': band_power_normalized,
        },
        'spectrum': {
            'freqs_mhz': [float(f/1e6) for f in freqs],
            'power_db': [float(p) for p in power_db],
        },
    }

    return profile

def compare_profiles(ref_profile, test_profile):
    """Compare test profile against reference."""
    print("\n" + "="*60)
    print("SPECTRAL COMPARISON")
    print("="*60)
    print(f"Reference: {ref_profile['source']} ({ref_profile['capture_type']})")
    print(f"Test:      {test_profile['source']} ({test_profile['capture_type']})")
    print()

    # Power level comparison
    ref_p = ref_profile['power_stats']
    test_p = test_profile['power_stats']
    print("Power Stats:")
    print(f"  {'Metric':<20} {'Reference':>10} {'Test':>10} {'Diff':>10}")
    print(f"  {'─'*20} {'─'*10} {'─'*10} {'─'*10}")
    for key in ['mean_db', 'peak_db', 'noise_floor_db', 'dynamic_range_db']:
        diff = test_p[key] - ref_p[key]
        print(f"  {key:<20} {ref_p[key]:>10.1f} {test_p[key]:>10.1f} {diff:>+10.1f}")

    # Spectral shape correlation
    ref_shape = np.array(ref_profile['spectral_shape']['band_power_above_noise'])
    test_shape = np.array(test_profile['spectral_shape']['band_power_above_noise'])
    correlation = float(np.corrcoef(ref_shape, test_shape)[0, 1])
    rmse = float(np.sqrt(np.mean((ref_shape - test_shape) ** 2)))
    print(f"\nSpectral Shape:")
    print(f"  Correlation: {correlation:.4f} (1.0 = perfect match)")
    print(f"  RMSE:        {rmse:.2f} dB")

    # Peak comparison
    ref_peaks = ref_profile['peaks']
    test_peaks = test_profile['peaks']
    print(f"\nPeaks: reference={len(ref_peaks)}, test={len(test_peaks)}")

    # Match peaks by frequency
    matched = 0
    for rp in ref_peaks:
        for tp in test_peaks:
            if abs(rp['center_hz'] - tp['center_hz']) < 50000:  # 50 kHz tolerance
                matched += 1
                freq_diff = tp['center_hz'] - rp['center_hz']
                power_diff = tp['power_db'] - rp['power_db']
                print(f"  Matched: {rp['center_hz']/1e6:.3f} MHz "
                      f"(freq_diff={freq_diff/1e3:+.1f} kHz, "
                      f"power_diff={power_diff:+.1f} dB)")
                break

    # Overall score
    score = correlation * 100
    if rmse > 5:
        score -= (rmse - 5) * 5
    if matched < len(ref_peaks):
        score -= (len(ref_peaks) - matched) * 10

    print(f"\n{'='*60}")
    print(f"MATCH SCORE: {score:.1f}/100")
    if score > 90:
        print("EXCELLENT - Signals match closely")
    elif score > 75:
        print("GOOD - Signals are similar with minor differences")
    elif score > 50:
        print("FAIR - Signals partially match")
    else:
        print("POOR - Significant differences detected")
    print(f"{'='*60}")

    return {
        'correlation': correlation,
        'rmse_db': rmse,
        'power_diff_mean_db': float(test_p['mean_db'] - ref_p['mean_db']),
        'peaks_matched': matched,
        'peaks_total_ref': len(ref_peaks),
        'score': score,
    }

def capture_wifi(output_file, host='192.168.1.232', port=1234,
                 n_samples=2048000, freq=936000000, gain=496, rate=1024000):
    """Capture IQ samples from WiFi RTL-TCP bridge."""
    import socket, struct

    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(15)
    sock.connect((host, port))

    # Read DongleInfo
    header = sock.recv(12)
    tuner = struct.unpack('>I', header[4:8])[0]
    print(f"Connected (tuner={tuner})")

    # Send configuration commands
    def send_cmd(cmd, param):
        sock.send(struct.pack('>BI', cmd, param))

    send_cmd(0x01, freq)         # Set frequency
    send_cmd(0x02, rate)         # Set sample rate
    send_cmd(0x03, 1)            # Manual gain
    send_cmd(0x04, gain)         # Set gain (tenths of dB)
    send_cmd(0x05, 0)            # Freq correction 0
    import time; time.sleep(0.5) # Let settings apply

    # Discard initial burst (settling time)
    discard = 0
    while discard < rate:  # Discard ~1 second
        chunk = sock.recv(65536)
        if chunk:
            discard += len(chunk) // 2

    # Capture
    print(f"Capturing {n_samples} samples at {freq/1e6:.3f} MHz, gain={gain/10:.1f} dB...")
    data = bytearray()
    target_bytes = n_samples * 2
    while len(data) < target_bytes:
        chunk = sock.recv(min(65536, target_bytes - len(data)))
        if not chunk:
            break
        data.extend(chunk)

    sock.close()

    # Save
    with open(output_file, 'wb') as f:
        f.write(bytes(data[:target_bytes]))
    print(f"Saved {len(data)} bytes to {output_file}")

    return output_file

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    cmd = sys.argv[1]

    if cmd == 'reference':
        filename = sys.argv[2]
        freq = float(sys.argv[sys.argv.index('--freq') + 1]) if '--freq' in sys.argv else 936e6
        rate = int(sys.argv[sys.argv.index('--rate') + 1]) if '--rate' in sys.argv else 1024000
        profile = create_profile(filename, sample_rate=rate, center_freq=freq)

        out_json = filename.rsplit('.', 1)[0] + '_profile.json'
        with open(out_json, 'w') as f:
            json.dump(profile, f, indent=2)
        print(f"\nProfile saved to {out_json}")

    elif cmd == 'compare':
        ref_json = sys.argv[2]
        test_file = sys.argv[3]
        freq = float(sys.argv[sys.argv.index('--freq') + 1]) if '--freq' in sys.argv else 936e6
        rate = int(sys.argv[sys.argv.index('--rate') + 1]) if '--rate' in sys.argv else 1024000

        with open(ref_json) as f:
            ref_profile = json.load(f)

        test_profile = create_profile(test_file, sample_rate=rate, center_freq=freq)
        test_profile['capture_type'] = 'wifi_bridge'

        result = compare_profiles(ref_profile, test_profile)

        out_json = test_file.rsplit('.', 1)[0] + '_comparison.json'
        with open(out_json, 'w') as f:
            json.dump(result, f, indent=2)
        print(f"\nComparison saved to {out_json}")

    elif cmd == 'capture_wifi':
        output = sys.argv[2]
        host = sys.argv[sys.argv.index('--host') + 1] if '--host' in sys.argv else '192.168.1.232'
        port = int(sys.argv[sys.argv.index('--port') + 1]) if '--port' in sys.argv else 1234
        capture_wifi(output, host=host, port=port)

    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
        sys.exit(1)
