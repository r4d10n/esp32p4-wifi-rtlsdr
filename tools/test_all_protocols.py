#!/usr/bin/env python3
"""
Test all ESP32-P4 SDR streaming protocols: RTL-TCP, RTL-UDP, SpyServer, SoapySDR Remote.
Records 5s of IQ from each, demodulates WBFM, measures SNR, and reports.

Usage: python3 tools/test_all_protocols.py [freq_mhz] [host]

SPDX-License-Identifier: GPL-2.0-or-later
"""

import sys, struct, socket, time, wave, json
import numpy as np
from scipy.signal import firwin, lfilter

HOST = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.233'
FREQ_MHZ = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
FREQ_HZ = int(FREQ_MHZ * 1e6)
RECORD_SECS = 5
AUDIO_RATE = 48000

# ── FM Demodulator (shared) ──
class FMDemod:
    def __init__(self, iq_rate):
        self.iq_rate = iq_rate
        self.prev_iq = 0.0 + 0.0j
        self.fm_norm = iq_rate / (2.0 * np.pi * 75000.0)
        alpha = 1.0 / (75e-6 * iq_rate + 1.0)
        self.de_alpha = alpha
        self.de_state = 0.0
        self.lpf_taps = firwin(63, 15000, fs=iq_rate)
        self.lpf_state = np.zeros(len(self.lpf_taps) - 1)
        self.dec_ratio = iq_rate / AUDIO_RATE
        self.resamp_phase = 0.0

    def process(self, iq_float):
        n = len(iq_float) // 2
        if n < 2: return np.array([], dtype=np.float32)
        iq = iq_float[0::2] + 1j * iq_float[1::2]
        ext = np.concatenate([[self.prev_iq], iq])
        self.prev_iq = iq[-1]
        pd = np.angle(ext[1:] * np.conj(ext[:-1])) * self.fm_norm
        a, b = self.de_alpha, 1.0 - self.de_alpha
        de, zi = lfilter([a], [1, -b], pd, zi=[self.de_state * b])
        self.de_state = float(zi[0]) / b if b != 0 else 0
        filtered, self.lpf_state = lfilter(self.lpf_taps, 1.0, de, zi=self.lpf_state)
        r = self.dec_ratio; ph = self.resamp_phase; L = len(filtered)
        pos = np.arange(0, L - 1, r) + ph
        pos = pos[pos < L - 1]
        if len(pos) == 0: return np.array([], dtype=np.float32)
        idx = pos.astype(int); frac = (pos - idx).astype(np.float32)
        audio = filtered[idx] * (1 - frac) + filtered[idx + 1] * frac
        self.resamp_phase = (pos[-1] + r) - L
        return (audio * 0.5).astype(np.float32)


def analyze_audio(samples, label):
    """Compute SNR and quality metrics."""
    if len(samples) < AUDIO_RATE:
        print(f"  {label}: Only {len(samples)} samples ({len(samples)/AUDIO_RATE:.1f}s) — too short")
        return
    s = np.array(samples, dtype=np.float32)
    rms = np.sqrt(np.mean(s**2))
    peak = np.max(np.abs(s))
    jumps = np.sum(np.abs(np.diff(s)) > 0.3)
    fft = np.abs(np.fft.rfft(s[:AUDIO_RATE]))
    f = np.arange(len(fft)) * AUDIO_RATE / (2 * len(fft))
    ab = fft[(f > 300) & (f < 5000)]
    nb = fft[(f > 15000) & (f < 20000)]
    snr = 20 * np.log10(np.mean(ab) / (np.mean(nb) + 1e-10))
    print(f"  {label}: {len(s)/AUDIO_RATE:.1f}s, RMS={rms:.4f}, Peak={peak:.4f}, "
          f"Jumps>0.3={jumps}, SNR={snr:.1f} dB")
    return snr


# ════════════════════════════════════════════════
#  1. RTL-TCP Client
# ════════════════════════════════════════════════
def test_rtltcp(host, port=1234):
    print(f"\n{'='*60}")
    print(f"  RTL-TCP @ {host}:{port}")
    print(f"{'='*60}")

    sock = socket.create_connection((host, port), timeout=10)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # Read DongleInfo (12 bytes)
    info = sock.recv(12)
    if len(info) == 12:
        magic, tuner, gains = struct.unpack('>III', info)
        magic_str = struct.pack('>I', magic).decode('ascii', errors='replace')
        print(f"  DongleInfo: magic='{magic_str}', tuner={tuner}, gains={gains}")
    else:
        print(f"  WARNING: DongleInfo only {len(info)} bytes")

    # Send commands: set freq, gain, sample rate
    def send_cmd(cmd, param):
        sock.sendall(struct.pack('>BI', cmd, param))

    send_cmd(0x02, 250000)     # sample rate
    time.sleep(0.5)
    send_cmd(0x01, FREQ_HZ)    # frequency
    time.sleep(0.3)
    send_cmd(0x03, 1)          # manual gain
    send_cmd(0x04, 400)        # gain 40 dB

    # Receive IQ data
    demod = FMDemod(250000)
    audio_out = []
    t0 = time.time()
    iq_bytes = 0

    while time.time() - t0 < RECORD_SECS:
        try:
            data = sock.recv(16384)
            if not data: break
            iq_bytes += len(data)
            iq = (np.frombuffer(data, dtype=np.uint8).astype(np.float32) - 127.5) / 127.5
            audio = demod.process(iq)
            audio_out.extend(audio.tolist())
        except socket.timeout:
            break

    elapsed = time.time() - t0
    print(f"  Received: {iq_bytes/1024:.0f} KB in {elapsed:.1f}s ({iq_bytes/elapsed/1024:.0f} KB/s)")
    sock.close()

    snr = analyze_audio(audio_out, "RTL-TCP FM")
    return snr


# ════════════════════════════════════════════════
#  2. RTL-UDP Client
# ════════════════════════════════════════════════
def test_rtludp(host, port=1235):
    print(f"\n{'='*60}")
    print(f"  RTL-UDP @ {host}:{port}")
    print(f"{'='*60}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    sock.bind(('0.0.0.0', 0))

    # Subscribe by sending any packet
    sock.sendto(b'\x00', (host, port))
    print(f"  Subscribed (sent trigger packet)")

    # Send commands via 5-byte rtl_tcp format
    def send_cmd(cmd, param):
        sock.sendto(struct.pack('>BI', cmd, param), (host, port))

    send_cmd(0x01, FREQ_HZ)    # frequency
    send_cmd(0x03, 1)          # manual gain
    send_cmd(0x04, 400)        # gain 40 dB
    time.sleep(0.3)

    # Receive IQ packets
    demod = FMDemod(250000)
    audio_out = []
    t0 = time.time()
    pkt_count = 0
    total_bytes = 0
    seq_errs = 0
    last_seq = None

    while time.time() - t0 < RECORD_SECS:
        try:
            data, addr = sock.recvfrom(2048)
            pkt_count += 1
            total_bytes += len(data)

            # Header: seq(4) + timestamp_us(4) + IQ payload
            if len(data) > 8:
                seq, ts = struct.unpack('<II', data[:8])
                if last_seq is not None and seq != last_seq + 1:
                    seq_errs += 1
                last_seq = seq
                iq_data = data[8:]
                iq = (np.frombuffer(iq_data, dtype=np.uint8).astype(np.float32) - 127.5) / 127.5
                audio = demod.process(iq)
                audio_out.extend(audio.tolist())
        except socket.timeout:
            if pkt_count == 0:
                print(f"  No packets received — device may not be streaming")
                break

    elapsed = time.time() - t0
    print(f"  Received: {pkt_count} packets, {total_bytes/1024:.0f} KB in {elapsed:.1f}s "
          f"({total_bytes/elapsed/1024:.0f} KB/s, seq_errs={seq_errs})")
    sock.close()

    snr = analyze_audio(audio_out, "RTL-UDP FM")
    return snr


# ════════════════════════════════════════════════
#  3. SpyServer Client
# ════════════════════════════════════════════════
def test_spyserver(host, port=5555):
    print(f"\n{'='*60}")
    print(f"  SpyServer @ {host}:{port}")
    print(f"{'='*60}")

    sock = socket.create_connection((host, port), timeout=10)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def recv_exact(n):
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk: raise ConnectionError("disconnected")
            buf += chunk
        return buf

    def send_cmd(cmd_type, body=b''):
        sock.sendall(struct.pack('<II', cmd_type, len(body)) + body)

    def set_setting(setting, value):
        send_cmd(2, struct.pack('<II', setting, value))  # CMD_SET_SETTING=2

    # HELLO
    name = b'TestClient'
    send_cmd(0, struct.pack('<I', (2 << 24) | (0 << 16) | 1700) + name)  # CMD_HELLO=0

    # Read DeviceInfo + ClientSync
    for _ in range(2):
        hdr = recv_exact(20)
        proto, msg_type, stream_type, seq, body_size = struct.unpack('<5I', hdr)
        body = recv_exact(body_size) if body_size > 0 else b''
        mt = msg_type & 0xFFFF
        if mt == 0:  # DEVICE_INFO
            fields = struct.unpack('<12I', body[:48])
            dev_types = {0:'Invalid', 1:'Airspy One', 2:'Airspy HF', 3:'RTL-SDR'}
            print(f"  DeviceInfo: {dev_types.get(fields[0],'?')}, rate={fields[2]/1e3:.0f}kSPS, "
                  f"gains={fields[6]}, freq={fields[7]/1e6:.0f}-{fields[8]/1e6:.0f}MHz")
        elif mt == 1:  # CLIENT_SYNC
            fields = struct.unpack('<9I', body[:36])
            print(f"  ClientSync: control={fields[0]}, freq={fields[2]/1e6:.3f}MHz")

    # Configure
    set_setting(101, FREQ_HZ)   # IQ_FREQUENCY
    time.sleep(0.1)
    set_setting(2, 18)          # GAIN (index)
    set_setting(100, 1)         # IQ_FORMAT = UINT8
    set_setting(0, 1)           # STREAMING_MODE = IQ
    set_setting(1, 1)           # STREAMING_ENABLED

    # Receive IQ
    demod = FMDemod(250000)
    audio_out = []
    t0 = time.time()
    iq_msgs = 0
    total_bytes = 0

    sock.settimeout(2.0)
    while time.time() - t0 < RECORD_SECS:
        try:
            hdr = recv_exact(20)
            proto, msg_type, stream_type, seq, body_size = struct.unpack('<5I', hdr)
            body = recv_exact(body_size) if body_size > 0 else b''
            mt = msg_type & 0xFFFF

            if mt == 100:  # UINT8_IQ
                iq_msgs += 1
                total_bytes += len(body)
                iq = (np.frombuffer(body, dtype=np.uint8).astype(np.float32) - 127.5) / 127.5
                audio = demod.process(iq)
                audio_out.extend(audio.tolist())
            elif mt == 1:  # CLIENT_SYNC (after settings change)
                pass
        except (socket.timeout, ConnectionError):
            break

    elapsed = time.time() - t0
    print(f"  Received: {iq_msgs} IQ msgs, {total_bytes/1024:.0f} KB in {elapsed:.1f}s "
          f"({total_bytes/elapsed/1024:.0f} KB/s)")

    set_setting(1, 0)  # disable streaming
    sock.close()

    snr = analyze_audio(audio_out, "SpyServer FM")
    return snr


# ════════════════════════════════════════════════
#  4. SoapySDR Remote Client
# ════════════════════════════════════════════════
def test_soapyremote(host, port=55132):
    print(f"\n{'='*60}")
    print(f"  SoapySDR Remote @ {host}:{port}")
    print(f"{'='*60}")

    try:
        sock = socket.create_connection((host, port), timeout=5)
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  SKIP: Cannot connect ({e})")
        return None

    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def recv_exact(n):
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk: raise ConnectionError("disconnected")
            buf += chunk
        return buf

    def rpc(cmd, payload=b''):
        sock.sendall(struct.pack('<II', cmd, len(payload)) + payload)
        hdr = recv_exact(8)
        resp_cmd, resp_len = struct.unpack('<II', hdr)
        body = recv_exact(resp_len) if resp_len > 0 else b''
        return resp_cmd, body

    # Discover
    cmd, body = rpc(0x00)  # CMD_DISCOVER
    print(f"  Discover: cmd=0x{cmd:02x}, {len(body)} bytes response")

    # Make device
    cmd, body = rpc(0x01)  # CMD_MAKE_DEVICE
    print(f"  MakeDevice: cmd=0x{cmd:02x}")

    # Get hardware key
    cmd, body = rpc(0x10)  # CMD_GET_HARDWARE_KEY
    hw_key = body.decode('utf-8', errors='replace').rstrip('\x00')
    print(f"  HardwareKey: '{hw_key}'")

    # Set frequency
    cmd, body = rpc(0x20, struct.pack('<d', float(FREQ_HZ)))  # CMD_SET_FREQUENCY
    print(f"  SetFreq: {FREQ_MHZ} MHz → cmd=0x{cmd:02x}")

    # Set sample rate
    cmd, body = rpc(0x30, struct.pack('<d', 250000.0))  # CMD_SET_SAMPLE_RATE
    print(f"  SetRate: 250 kSPS → cmd=0x{cmd:02x}")

    # Set gain
    cmd, body = rpc(0x40, struct.pack('<d', 40.0))  # CMD_SET_GAIN
    print(f"  SetGain: 40 dB → cmd=0x{cmd:02x}")

    # Bind UDP socket first, then tell server the port
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.settimeout(2.0)
    udp_sock.bind(('0.0.0.0', 0))  # OS assigns a free port
    local_udp_port = udp_sock.getsockname()[1]

    # Setup stream — tell server our UDP port
    cmd, body = rpc(0x50, struct.pack('<H', local_udp_port))  # CMD_SETUP_STREAM
    srv_port = 0
    if len(body) >= 4:
        srv_port = struct.unpack('<I', body[:4])[0]
    print(f"  SetupStream: told server our port={local_udp_port}, server responded port={srv_port}")

    # Activate stream
    cmd, body = rpc(0x51)  # CMD_ACTIVATE_STREAM
    print(f"  ActivateStream: cmd=0x{cmd:02x}")

        demod = FMDemod(250000)
        audio_out = []
        t0 = time.time()
        pkt_count = 0
        total_bytes = 0

        while time.time() - t0 < RECORD_SECS:
            try:
                data, _ = udp_sock.recvfrom(8192)
                pkt_count += 1
                # Skip 4-byte seq header
                iq_data = data[4:] if len(data) > 4 else data
                total_bytes += len(iq_data)
                iq = (np.frombuffer(iq_data, dtype=np.uint8).astype(np.float32) - 127.5) / 127.5
                audio = demod.process(iq)
                audio_out.extend(audio.tolist())
            except socket.timeout:
                if pkt_count == 0:
                    print(f"  No UDP packets received")
                break

        elapsed = time.time() - t0
        print(f"  Received: {pkt_count} UDP packets, {total_bytes/1024:.0f} KB in {elapsed:.1f}s")
        udp_sock.close()

        # Deactivate + close
        rpc(0x52)  # CMD_DEACTIVATE_STREAM
        rpc(0x53)  # CMD_CLOSE_STREAM
        sock.close()

        snr = analyze_audio(audio_out, "SoapySDR FM")
        return snr
    else:
        print(f"  No UDP port returned — stream not available")
        sock.close()
        return None


# ════════════════════════════════════════════════
#  Main
# ════════════════════════════════════════════════
if __name__ == '__main__':
    print(f"ESP32-P4 SDR Protocol Test Suite")
    print(f"Host: {HOST} | Freq: {FREQ_MHZ} MHz | Record: {RECORD_SECS}s")

    results = {}

    # Test each protocol
    try:
        results['rtl_tcp'] = test_rtltcp(HOST)
    except Exception as e:
        print(f"  RTL-TCP FAILED: {e}")
        results['rtl_tcp'] = None

    try:
        results['rtl_udp'] = test_rtludp(HOST)
    except Exception as e:
        print(f"  RTL-UDP FAILED: {e}")
        results['rtl_udp'] = None

    try:
        results['spyserver'] = test_spyserver(HOST)
    except Exception as e:
        print(f"  SpyServer FAILED: {e}")
        results['spyserver'] = None

    try:
        results['soapysdr'] = test_soapyremote(HOST)
    except Exception as e:
        print(f"  SoapySDR FAILED: {e}")
        results['soapysdr'] = None

    # Summary
    print(f"\n{'='*60}")
    print(f"  SUMMARY — FM demod SNR at {FREQ_MHZ} MHz")
    print(f"{'='*60}")
    for name, snr in results.items():
        status = f"{snr:.1f} dB" if snr is not None else "FAILED"
        bar = '█' * int(snr / 2) if snr and snr > 0 else ''
        print(f"  {name:15s} {status:>10s}  {bar}")
    print()
