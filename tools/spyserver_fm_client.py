#!/usr/bin/env python3
"""
SpyServer Client — WBFM decoder for ESP32-P4

Connects to the SpyServer on ESP32-P4, tunes to an FM station,
receives IQ data, demodulates WBFM, and plays audio.

Usage:
    python3 tools/spyserver_fm_client.py [freq_mhz] [host] [port]

Reference: github.com/miweber67/spyserver_client
           github.com/AlexandreRouma/SDRPlusPlus

SPDX-License-Identifier: GPL-2.0-or-later
"""

import sys, struct, socket, time, threading
import numpy as np
import sounddevice as sd
from scipy.signal import firwin, lfilter

# ── SpyServer Protocol Constants ──
SPYSERVER_PROTOCOL_VERSION = (2 << 24) | (0 << 16) | 1700
SPYSERVER_PROTOCOL_ID = 0x53595053  # 'SPYS'

# Commands
CMD_HELLO = 0
CMD_SET_SETTING = 2
CMD_PING = 3

# Settings
SETTING_STREAMING_MODE = 0
SETTING_STREAMING_ENABLED = 1
SETTING_GAIN = 2
SETTING_IQ_FORMAT = 100
SETTING_IQ_FREQUENCY = 101
SETTING_IQ_DECIMATION = 102
SETTING_FFT_FORMAT = 200
SETTING_FFT_FREQUENCY = 201
SETTING_FFT_DECIMATION = 202
SETTING_FFT_DB_OFFSET = 203
SETTING_FFT_DB_RANGE = 204
SETTING_FFT_DISPLAY_PIXELS = 205

# Stream types
STREAM_TYPE_STATUS = 0
STREAM_TYPE_IQ = 1
STREAM_TYPE_AF = 2
STREAM_TYPE_FFT = 4

# Stream formats
STREAM_FORMAT_UINT8 = 1
STREAM_FORMAT_INT16 = 2

# Message types
MSG_TYPE_DEVICE_INFO = 0
MSG_TYPE_CLIENT_SYNC = 1
MSG_TYPE_PONG = 2
MSG_TYPE_UINT8_IQ = 100
MSG_TYPE_INT16_IQ = 101
MSG_TYPE_UINT8_FFT = 301

# ── Configuration ──
AUDIO_RATE = 48000
IQ_RATE = 250000  # Must match SpyServer device rate
DE_EMPH_TAU = 75e-6
VOLUME = 0.5

# ── Audio Ring Buffer ──
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
        else: f = ring_sz - r; out = np.concatenate([ring[r:], ring[:t-f]])
        ring_rd = e % ring_sz
        return np.concatenate([out, np.zeros(n-t, dtype=np.float32)]) if t < n else out


# ── FM Demodulator ──
class FMDemod:
    def __init__(self, iq_rate, audio_rate, tau=75e-6):
        self.iq_rate = iq_rate
        self.audio_rate = audio_rate
        self.prev_iq = 0.0 + 0.0j
        self.fm_norm = iq_rate / (2.0 * np.pi * 75000.0)
        self.de_alpha = 1.0 / (tau * iq_rate + 1.0)
        self.de_state = 0.0
        self.lpf_taps = firwin(63, 15000, fs=iq_rate)
        self.lpf_state = np.zeros(len(self.lpf_taps) - 1)
        self.dec_ratio = iq_rate / audio_rate
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
        return (audio * VOLUME).astype(np.float32)


# ── SpyServer Client ──
class SpyServerClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.dev_info = None
        self.running = False
        self.demod = None
        self.stats = {'iq_msgs': 0, 'audio_secs': 0.0, 'underruns': 0}

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=10)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"  Connected to {self.host}:{self.port}")

    def send_command(self, cmd_type, body=b''):
        hdr = struct.pack('<II', cmd_type, len(body))
        self.sock.sendall(hdr + body)

    def send_hello(self):
        # Handshake: protocol version + client name
        name = b'ESP32P4-SpyClient'
        body = struct.pack('<I', SPYSERVER_PROTOCOL_VERSION) + name
        self.send_command(CMD_HELLO, body)

    def set_setting(self, setting, value):
        body = struct.pack('<II', setting, value)
        self.send_command(CMD_SET_SETTING, body)

    def recv_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("SpyServer disconnected")
            buf += chunk
        return buf

    def recv_message(self):
        """Receive one SpyServer message (header + body)."""
        hdr_data = self.recv_exact(20)  # 5 x uint32
        proto_id, msg_type_raw, stream_type, seq, body_size = struct.unpack('<5I', hdr_data)

        msg_type = msg_type_raw & 0xFFFF
        msg_flags = (msg_type_raw >> 16) & 0xFFFF

        body = b''
        if body_size > 0:
            body = self.recv_exact(body_size)

        return msg_type, msg_flags, stream_type, seq, body

    def parse_device_info(self, body):
        if len(body) < 48:
            print(f"  DeviceInfo too short: {len(body)} bytes")
            return
        fields = struct.unpack('<12I', body[:48])
        self.dev_info = {
            'device_type': fields[0],
            'device_serial': fields[1],
            'max_sample_rate': fields[2],
            'max_bandwidth': fields[3],
            'decimation_stages': fields[4],
            'gain_stages': fields[5],
            'max_gain_index': fields[6],
            'min_freq': fields[7],
            'max_freq': fields[8],
            'resolution': fields[9],
            'min_iq_decimation': fields[10],
            'forced_iq_format': fields[11],
        }
        dev_names = {0: 'Invalid', 1: 'Airspy One', 2: 'Airspy HF', 3: 'RTL-SDR'}
        print(f"  Device: {dev_names.get(self.dev_info['device_type'], '?')}")
        print(f"  Max rate: {self.dev_info['max_sample_rate']/1e6:.1f} MSPS")
        print(f"  Freq range: {self.dev_info['min_freq']/1e6:.0f} - {self.dev_info['max_freq']/1e6:.0f} MHz")
        print(f"  Gain steps: {self.dev_info['max_gain_index']}")
        print(f"  Resolution: {self.dev_info['resolution']} bits")

    def parse_client_sync(self, body):
        if len(body) < 36:
            return
        fields = struct.unpack('<9I', body[:36])
        print(f"  Sync: control={fields[0]}, gain={fields[1]}, "
              f"dev_freq={fields[2]/1e6:.3f}MHz, iq_freq={fields[3]/1e6:.3f}MHz")

    def process_iq_uint8(self, body, flags):
        """Process uint8 IQ data through FM demodulator."""
        if self.demod is None:
            return
        iq = (np.frombuffer(body, dtype=np.uint8).astype(np.float32) - 127.5) / 127.5
        audio = self.demod.process(iq)
        if len(audio) > 0:
            ring_push(audio)
            self.stats['iq_msgs'] += 1
            self.stats['audio_secs'] += len(audio) / AUDIO_RATE

    def process_iq_int16(self, body, flags):
        """Process int16 IQ data through FM demodulator."""
        if self.demod is None:
            return
        iq = np.frombuffer(body, dtype=np.int16).astype(np.float32) / 32768.0
        audio = self.demod.process(iq)
        if len(audio) > 0:
            ring_push(audio)
            self.stats['iq_msgs'] += 1
            self.stats['audio_secs'] += len(audio) / AUDIO_RATE

    def receiver_loop(self):
        """Main receive loop — runs in a thread."""
        self.running = True
        while self.running:
            try:
                msg_type, flags, stream_type, seq, body = self.recv_message()

                if msg_type == MSG_TYPE_DEVICE_INFO:
                    self.parse_device_info(body)
                elif msg_type == MSG_TYPE_CLIENT_SYNC:
                    self.parse_client_sync(body)
                elif msg_type == MSG_TYPE_PONG:
                    pass
                elif msg_type == MSG_TYPE_UINT8_IQ:
                    self.process_iq_uint8(body, flags)
                elif msg_type == MSG_TYPE_INT16_IQ:
                    self.process_iq_int16(body, flags)
                elif msg_type == MSG_TYPE_UINT8_FFT:
                    pass  # FFT data (not used for audio)
                else:
                    pass  # Unknown message type

            except (ConnectionError, OSError) as e:
                print(f"\n  Connection lost: {e}")
                self.running = False
                break

    def start_fm(self, freq_hz, gain_index=18):
        """Configure SpyServer for FM reception and start streaming."""
        # Initialize demodulator
        self.demod = FMDemod(IQ_RATE, AUDIO_RATE, DE_EMPH_TAU)

        # Send HELLO
        self.send_hello()
        time.sleep(0.5)

        # Wait for DeviceInfo
        msg_type, _, _, _, body = self.recv_message()
        if msg_type == MSG_TYPE_DEVICE_INFO:
            self.parse_device_info(body)
        # Read ClientSync
        msg_type, _, _, _, body = self.recv_message()
        if msg_type == MSG_TYPE_CLIENT_SYNC:
            self.parse_client_sync(body)

        # Configure
        print(f"  Setting frequency: {freq_hz/1e6:.3f} MHz")
        self.set_setting(SETTING_IQ_FREQUENCY, freq_hz)
        time.sleep(0.1)

        print(f"  Setting gain index: {gain_index}")
        self.set_setting(SETTING_GAIN, gain_index)
        time.sleep(0.1)

        # Set IQ format to uint8
        self.set_setting(SETTING_IQ_FORMAT, STREAM_FORMAT_UINT8)
        time.sleep(0.1)

        # Set streaming mode to IQ only
        self.set_setting(SETTING_STREAMING_MODE, STREAM_TYPE_IQ)
        time.sleep(0.1)

        # Enable streaming
        print("  Enabling IQ streaming...")
        self.set_setting(SETTING_STREAMING_ENABLED, 1)

        # Start receiver thread
        rx_thread = threading.Thread(target=self.receiver_loop, daemon=True)
        rx_thread.start()
        return rx_thread

    def stop(self):
        self.running = False
        try:
            self.set_setting(SETTING_STREAMING_ENABLED, 0)
        except:
            pass
        try:
            self.sock.close()
        except:
            pass


def audio_cb(outdata, frames, ti, status):
    if status:
        client.stats['underruns'] += 1
    outdata[:, 0] = ring_pull(frames)


def reporter(client):
    while client.running:
        time.sleep(5)
        buf_ms = ring_avail() / AUDIO_RATE * 1000
        print(f"  [Stats] iq={client.stats['iq_msgs']}, audio={client.stats['audio_secs']:.1f}s, "
              f"buf={buf_ms:.0f}ms, ur={client.stats['underruns']}")


# ── Main ──
if __name__ == '__main__':
    freq_mhz = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    host = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.233'
    port = int(sys.argv[3]) if len(sys.argv) > 3 else 5555
    gain = int(sys.argv[4]) if len(sys.argv) > 4 else 18

    freq_hz = int(freq_mhz * 1e6)

    print(f"  ESP32-P4 SpyServer FM Client")
    print(f"  {freq_mhz} MHz | {host}:{port} | gain idx={gain}")
    print(f"  IQ rate: {IQ_RATE/1e3:.0f} kSPS | Audio: {AUDIO_RATE} Hz")
    print()

    client = SpyServerClient(host, port)
    client.connect()

    # Start audio output
    stream = sd.OutputStream(samplerate=AUDIO_RATE, channels=1, dtype='float32',
                              callback=audio_cb, blocksize=2048, latency='high')
    stream.start()

    # Start stats reporter
    threading.Thread(target=reporter, args=(client,), daemon=True).start()

    # Start FM reception
    rx_thread = client.start_fm(freq_hz, gain)

    print(f"\n  Streaming WBFM at {freq_mhz} MHz — Ctrl+C to stop\n")

    try:
        while client.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        stream.stop()
        stream.close()
        print(f"\n  Done: {client.stats['iq_msgs']} IQ msgs, "
              f"{client.stats['audio_secs']:.1f}s audio, "
              f"{client.stats['underruns']} underruns")
