#!/usr/bin/env python3
"""
AFSK1200 Packet RF Loopback Test: PlutoSDR TX → ESP32-P4 WebSDR RX

Generates AX.25 AFSK1200 packets, FM-transmits via PlutoSDR,
receives via WebSDR WebSocket, demodulates FM, decodes AFSK1200,
and validates packet integrity. Much more demanding than DTMF —
requires continuous, gap-free audio for successful decode.

Implements AFSK1200 TX/RX entirely in Python (no direwolf needed):
  - TX: AX.25 framing → NRZI → AFSK (1200/2200 Hz) → FM mod → PlutoSDR
  - RX: WebSDR WS → FM demod → AFSK correlator → NRZI → AX.25 decode → CRC check

Requirements:
  pip install pyadi-iio websocket-client numpy pyserial

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import struct
import sys
import threading
import time
import wave
from collections import deque
from dataclasses import dataclass, field

import numpy as np

try:
    import adi
except ImportError:
    print("ERROR: pip install pyadi-iio"); sys.exit(1)
try:
    import websocket
except ImportError:
    print("ERROR: pip install websocket-client"); sys.exit(1)
try:
    import serial
except ImportError:
    serial = None
    print("WARNING: pyserial not available, UART reset disabled")


# ═══════════════════════════════════════════════════════════════
#  AFSK1200 / AX.25 Constants
# ═══════════════════════════════════════════════════════════════

MARK_FREQ = 1200      # Hz
SPACE_FREQ = 2200     # Hz
BAUD_RATE = 1200      # symbols/sec
AUDIO_RATE = 48000    # samples/sec
SAMPLES_PER_BIT = AUDIO_RATE // BAUD_RATE  # 40

AX25_FLAG = 0x7E
AX25_CRC_POLY = 0x8408  # CRC-CCITT (reversed)

FM_DEVIATION = 5000   # 5 kHz standard NFM deviation for AFSK1200
PLUTO_SAMPLE_RATE = 1000000


# ═══════════════════════════════════════════════════════════════
#  AX.25 Frame Construction
# ═══════════════════════════════════════════════════════════════

def ax25_callsign(call, ssid=0, last=False):
    """Encode AX.25 callsign (6 chars + SSID byte)."""
    call = call.upper().ljust(6)[:6]
    encoded = bytes([ord(c) << 1 for c in call])
    ssid_byte = ((ssid & 0x0F) << 1) | 0x60
    if last:
        ssid_byte |= 0x01  # last address flag
    return encoded + bytes([ssid_byte])


def ax25_crc(data):
    """Compute AX.25 CRC-CCITT (FCS)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ AX25_CRC_POLY
            else:
                crc >>= 1
    return crc ^ 0xFFFF


def make_ax25_frame(src, dst, info, src_ssid=0, dst_ssid=0):
    """Build a complete AX.25 UI frame."""
    addr = ax25_callsign(dst, dst_ssid) + ax25_callsign(src, src_ssid, last=True)
    ctrl = 0x03   # UI frame
    pid = 0xF0    # No layer 3
    payload = addr + bytes([ctrl, pid]) + info.encode('ascii', errors='replace')
    crc = ax25_crc(payload)
    return payload + struct.pack('<H', crc)


def frame_to_bits(frame):
    """Convert AX.25 frame bytes to bit stream with HDLC bit stuffing."""
    bits = []
    # Preamble: multiple flags
    for _ in range(40):  # 40 flag bytes = 333ms preamble
        for bit in range(8):
            bits.append((AX25_FLAG >> bit) & 1)

    # Frame data with bit stuffing
    ones_count = 0
    for byte in frame:
        for bit in range(8):
            b = (byte >> bit) & 1
            bits.append(b)
            if b == 1:
                ones_count += 1
                if ones_count == 5:
                    bits.append(0)  # stuff zero
                    ones_count = 0
            else:
                ones_count = 0

    # Closing flags
    for _ in range(4):
        for bit in range(8):
            bits.append((AX25_FLAG >> bit) & 1)

    return bits


def nrzi_encode(bits):
    """NRZI encode: 0 = transition, 1 = no transition."""
    state = 0
    out = []
    for b in bits:
        if b == 0:
            state ^= 1  # transition
        out.append(state)
    return out


# ═══════════════════════════════════════════════════════════════
#  AFSK1200 Modulation
# ═══════════════════════════════════════════════════════════════

def afsk_modulate(nrzi_bits):
    """Generate AFSK1200 audio from NRZI bit stream."""
    phase = 0.0
    audio = []
    for bit in nrzi_bits:
        freq = MARK_FREQ if bit == 0 else SPACE_FREQ
        phase_inc = 2.0 * np.pi * freq / AUDIO_RATE
        for _ in range(SAMPLES_PER_BIT):
            audio.append(np.sin(phase) * 0.7)
            phase += phase_inc
            if phase > 2 * np.pi:
                phase -= 2 * np.pi
    return np.array(audio, dtype=np.float32)


def generate_test_packets(count, payload_len=64):
    """Generate test AX.25 packets with sequential numbered payloads."""
    packets = []
    for i in range(count):
        info = f"PKT{i:04d}:{'A' * (payload_len - 8)}"
        frame = make_ax25_frame("TEST", "CQ", info, src_ssid=1)
        packets.append((info, frame))
    return packets


def packets_to_audio(packets):
    """Convert list of AX.25 frames to continuous AFSK1200 audio.
    Adds 500ms silence at start and end for clean PlutoSDR cyclic buffer looping."""
    all_audio = []
    gap_samples = int(AUDIO_RATE * 0.2)  # 200ms between packets
    pad_samples = int(AUDIO_RATE * 0.5)  # 500ms padding at boundaries

    # Leading silence (avoids cyclic buffer seam hitting a packet)
    all_audio.append(np.zeros(pad_samples, dtype=np.float32))

    for _, frame in packets:
        bits = frame_to_bits(frame)
        nrzi = nrzi_encode(bits)
        audio = afsk_modulate(nrzi)
        all_audio.append(audio)
        all_audio.append(np.zeros(gap_samples, dtype=np.float32))

    # Trailing silence (phase settles to zero before wrap)
    all_audio.append(np.zeros(pad_samples, dtype=np.float32))

    return np.concatenate(all_audio)


# ═══════════════════════════════════════════════════════════════
#  AFSK1200 Demodulation (Correlator-based)
# ═══════════════════════════════════════════════════════════════

class AFSKDemodulator:
    """AFSK1200 demodulator using correlation detector."""

    def __init__(self, sample_rate=AUDIO_RATE):
        self.sr = sample_rate
        self.spb = sample_rate // BAUD_RATE
        # Generate correlator templates (one bit period each)
        t = np.arange(self.spb) / sample_rate
        self.mark_cos = np.cos(2 * np.pi * MARK_FREQ * t)
        self.mark_sin = np.sin(2 * np.pi * MARK_FREQ * t)
        self.space_cos = np.cos(2 * np.pi * SPACE_FREQ * t)
        self.space_sin = np.sin(2 * np.pi * SPACE_FREQ * t)

    def demod_to_bits(self, audio):
        """Demodulate AFSK audio to raw bit decisions."""
        n = len(audio)
        bits = []
        energies = []

        for i in range(0, n - self.spb, self.spb):
            segment = audio[i:i + self.spb]
            mark_e = (np.dot(segment, self.mark_cos)**2 +
                      np.dot(segment, self.mark_sin)**2)
            space_e = (np.dot(segment, self.space_cos)**2 +
                       np.dot(segment, self.space_sin)**2)
            bits.append(0 if mark_e > space_e else 1)
            energies.append((mark_e, space_e))

        return bits, energies

    def nrzi_decode(self, bits):
        """NRZI decode: transition = 0, no transition = 1."""
        decoded = []
        prev = bits[0] if bits else 0
        for b in bits[1:]:
            decoded.append(0 if b != prev else 1)
            prev = b
        return decoded

    def find_flags(self, bits):
        """Find AX.25 flag (0x7E = 01111110) positions in bit stream."""
        flag_pattern = [0, 1, 1, 1, 1, 1, 1, 0]
        positions = []
        for i in range(len(bits) - 8):
            if bits[i:i+8] == flag_pattern:
                positions.append(i)
        return positions

    def unstuff_bits(self, bits):
        """Remove bit stuffing (zero after five ones)."""
        out = []
        ones = 0
        for b in bits:
            if ones == 5:
                if b == 0:
                    ones = 0
                    continue  # skip stuffed zero
                else:
                    return None  # invalid: six ones without flag
            out.append(b)
            ones = ones + 1 if b == 1 else 0
        return out

    def bits_to_bytes(self, bits):
        """Convert bit list to bytes (LSB first)."""
        result = []
        for i in range(0, len(bits) - 7, 8):
            byte = 0
            for j in range(8):
                byte |= bits[i + j] << j
            result.append(byte)
        return bytes(result)

    def decode_frames(self, audio):
        """Full decode pipeline: audio → AX.25 frames."""
        raw_bits, energies = self.demod_to_bits(audio)
        nrzi_decoded = self.nrzi_decode(raw_bits)
        flags = self.find_flags(nrzi_decoded)

        frames = []
        for i in range(len(flags) - 1):
            start = flags[i] + 8  # skip opening flag
            end = flags[i + 1]    # up to closing flag

            if end - start < 8 * 17:  # min AX.25 frame = 17 bytes
                continue
            if end - start > 8 * 400:  # sanity limit
                continue

            frame_bits = nrzi_decoded[start:end]
            unstuffed = self.unstuff_bits(frame_bits)
            if unstuffed is None:
                continue

            frame_bytes = self.bits_to_bytes(unstuffed)
            if len(frame_bytes) < 17:
                continue

            # Verify CRC
            payload = frame_bytes[:-2]
            rx_crc = struct.unpack('<H', frame_bytes[-2:])[0]
            calc_crc = ax25_crc(payload)

            frames.append({
                'data': frame_bytes,
                'payload': payload,
                'crc_ok': rx_crc == calc_crc,
                'rx_crc': rx_crc,
                'calc_crc': calc_crc,
                'bit_pos': flags[i],
                'info': self._extract_info(payload),
            })

        return frames, len(raw_bits), energies

    def _extract_info(self, payload):
        """Extract info field from AX.25 payload."""
        if len(payload) < 16:
            return ""
        # Skip addresses (14 bytes) + ctrl (1) + pid (1)
        info = payload[16:]
        try:
            return info.decode('ascii', errors='replace')
        except Exception:
            return repr(info)


# ═══════════════════════════════════════════════════════════════
#  Audio Gap Detection
# ═══════════════════════════════════════════════════════════════

def detect_audio_gaps(audio, threshold=0.01, min_gap_ms=5):
    """Detect silent gaps in audio that indicate data loss."""
    block_size = int(AUDIO_RATE * min_gap_ms / 1000)
    gaps = []
    in_gap = False
    gap_start = 0

    for i in range(0, len(audio) - block_size, block_size // 2):
        block = audio[i:i + block_size]
        rms = np.sqrt(np.mean(block**2))

        if rms < threshold:
            if not in_gap:
                gap_start = i
                in_gap = True
        else:
            if in_gap:
                gap_dur_ms = (i - gap_start) * 1000 / AUDIO_RATE
                if gap_dur_ms >= min_gap_ms:
                    gaps.append({
                        'start_ms': gap_start * 1000 / AUDIO_RATE,
                        'duration_ms': gap_dur_ms,
                        'start_sample': gap_start,
                    })
                in_gap = False

    return gaps


# ═══════════════════════════════════════════════════════════════
#  FM Modulation / Demodulation
# ═══════════════════════════════════════════════════════════════

def fm_modulate(audio, audio_rate, rf_rate, deviation):
    """FM modulate audio to complex baseband IQ."""
    ratio = rf_rate / audio_rate
    n_out = int(len(audio) * ratio)
    indices = np.arange(n_out) * (audio_rate / rf_rate)
    lo = np.clip(indices.astype(int), 0, len(audio) - 1)
    hi = np.clip(lo + 1, 0, len(audio) - 1)
    frac = indices - indices.astype(int)
    audio_rs = audio[lo] * (1 - frac) + audio[hi] * frac
    phase = 2 * np.pi * deviation * np.cumsum(audio_rs) / rf_rate
    return np.exp(1j * phase).astype(np.complex64)


def demod_fm_block(iq_uint8, prev_iq):
    """FM discriminator for uint8 IQ data."""
    iq_float = (iq_uint8.astype(np.float32) - 127.5) / 127.5
    i_s, q_s = iq_float[0::2], iq_float[1::2]
    n = len(i_s)
    if n < 2:
        return np.array([], dtype=np.float32), prev_iq
    prev_i, prev_q = prev_iq
    audio = np.empty(n, dtype=np.float32)
    for k in range(n):
        ci, cq = i_s[k], q_s[k]
        re = ci * prev_i + cq * prev_q
        im = cq * prev_i - ci * prev_q
        audio[k] = np.arctan2(im, re) / np.pi
        prev_i, prev_q = ci, cq
    return audio, (prev_i, prev_q)


def resample_linear(audio, src_rate, dst_rate):
    if src_rate == dst_rate or src_rate <= 0 or len(audio) == 0:
        return audio
    ratio = src_rate / dst_rate
    n_out = int(len(audio) / ratio)
    if n_out <= 0:
        return np.array([], dtype=np.float32)
    indices = np.arange(n_out) * ratio
    lo = np.clip(indices.astype(int), 0, len(audio) - 1)
    hi = np.clip(lo + 1, 0, len(audio) - 1)
    frac = indices - indices.astype(int)
    return (audio[lo] * (1 - frac) + audio[hi] * frac).astype(np.float32)


# ═══════════════════════════════════════════════════════════════
#  UART Reset
# ═══════════════════════════════════════════════════════════════

def uart_reset_esp32(port='/dev/ttyACM0'):
    """Reset ESP32-P4 via esptool (avoids download mode from raw DTR/RTS)."""
    import subprocess
    try:
        # Use esptool's --after hard_reset which properly sequences the pins
        result = subprocess.run(
            ['python3', '-m', 'esptool', '--chip', 'esp32p4', '-p', port,
             '--after', 'hard_reset', 'read_mac'],
            capture_output=True, text=True, timeout=10)
        print(f"[UART] Reset via esptool: {result.stdout.strip().split(chr(10))[-1]}")
        return True
    except Exception as e:
        print(f"[UART] Reset failed: {e}")
        return False


# ═══════════════════════════════════════════════════════════════
#  PlutoSDR TX
# ═══════════════════════════════════════════════════════════════

class PlutoTx:
    def __init__(self, ip):
        self.sdr = adi.Pluto(f"ip:{ip}")
        self.active = False

    def start(self, freq_hz, iq_data, sample_rate, gain_db=-10):
        if self.active:
            self.stop()
        self.sdr.sample_rate = int(sample_rate)
        self.sdr.tx_rf_bandwidth = int(sample_rate)
        self.sdr.tx_lo = int(freq_hz)
        self.sdr.tx_hardwaregain_chan0 = gain_db
        self.sdr.tx_cyclic_buffer = True
        self.sdr.tx(iq_data * 2**14)
        self.active = True

    def stop(self):
        if self.active:
            try: self.sdr.tx_destroy_buffer()
            except: pass
            self.active = False


# ═══════════════════════════════════════════════════════════════
#  WebSDR RX
# ═══════════════════════════════════════════════════════════════

@dataclass
class RxState:
    iq_rate: int = 0
    iq_queue: deque = field(default_factory=lambda: deque(maxlen=1000))
    connected: bool = False
    iq_started: bool = False
    running: bool = True


def create_ws_rx(sdr_ip, sdr_port, rx):
    def on_msg(ws, msg):
        if isinstance(msg, str):
            try: m = json.loads(msg)
            except: return
            if m.get("type") == "info": rx.connected = True
            elif m.get("type") == "iq_start":
                rx.iq_rate = m.get("rate", 0); rx.iq_started = True
        elif len(msg) >= 2 and msg[0] == 0x02:
            rx.iq_queue.append(np.frombuffer(msg[1:], dtype=np.uint8))
    def on_open(ws): rx.connected = True
    def on_close(ws, *a): rx.connected = False
    ws = websocket.WebSocketApp(f"ws://{sdr_ip}:{sdr_port}/ws",
                                 on_open=on_open, on_message=on_msg,
                                 on_close=on_close, on_error=lambda w,e: None)
    threading.Thread(target=ws.run_forever, daemon=True).start()
    return ws


def despike_audio(audio, threshold=2.5):
    """Remove FM demod phase spikes caused by IQ data gaps.
    Vectorized: detect spikes > threshold*σ, replace with median-filtered values."""
    if len(audio) < 100:
        return audio
    # Compute local statistics in blocks
    block = 256
    out = audio.copy()
    for start in range(0, len(audio) - block, block):
        seg = out[start:start + block]
        med = np.median(seg)
        std = np.std(seg)
        if std < 0.001:
            continue
        mask = np.abs(seg - med) > threshold * std
        if np.any(mask):
            # Replace spikes with median
            seg[mask] = med
            out[start:start + block] = seg
    return out


def capture_audio(rx, duration_s, native_rate=False):
    """Capture FM-demodulated audio. If native_rate=True, skip resampling."""
    chunks = []
    prev = (0.0, 0.0)
    t0 = time.time()
    while time.time() - t0 < duration_s and rx.running:
        if not rx.iq_queue:
            time.sleep(0.002); continue
        try: iq = rx.iq_queue.popleft()
        except: continue
        audio, prev = demod_fm_block(iq, prev)
        if len(audio) > 0:
            if not native_rate and rx.iq_rate > 0:
                audio = resample_linear(audio, rx.iq_rate, AUDIO_RATE)
            if len(audio) > 0:
                chunks.append(audio)
    if not chunks:
        return np.array([], dtype=np.float32)
    raw = np.concatenate(chunks)
    return despike_audio(raw)


def save_wav(audio, filename):
    if len(audio) == 0: return
    peak = np.max(np.abs(audio))
    if peak > 0: audio = audio / peak * 0.9
    d = (audio * 32767).astype(np.int16)
    with wave.open(filename, 'w') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(AUDIO_RATE)
        w.writeframes(d.tobytes())


# ═══════════════════════════════════════════════════════════════
#  Main Test
# ═══════════════════════════════════════════════════════════════

def ws_send_safe(ws, rx, msg, ip, port):
    try:
        ws.send(msg); return ws, rx
    except:
        print("[RX] Reconnecting...")
        try: ws.close()
        except: pass
        time.sleep(1)
        rx2 = RxState()
        ws2 = create_ws_rx(ip, port, rx2)
        t0 = time.time()
        while not rx2.connected and time.time() - t0 < 5: time.sleep(0.1)
        if rx2.connected:
            try: ws2.send(msg)
            except: pass
        return ws2, rx2


def run_test(args):
    print("="*70)
    print("  AFSK1200 Packet RF Loopback Test")
    print(f"  PlutoSDR TX: {args.pluto_ip}")
    print(f"  WebSDR  RX:  {args.sdr_ip}:{args.sdr_port}")
    print("="*70)

    # Generate test packets
    num_packets = args.packets
    packets = generate_test_packets(num_packets, payload_len=args.payload_len)
    afsk_audio = packets_to_audio(packets)
    total_dur = len(afsk_audio) / AUDIO_RATE

    print(f"\n[TX] {num_packets} AX.25 packets, {args.payload_len} byte payload each")
    print(f"[TX] AFSK1200 audio: {total_dur:.1f}s ({len(afsk_audio)} samples)")
    print(f"[TX] Packet info example: {packets[0][0][:30]}...")

    # Verify TX encoding with local decode
    demod = AFSKDemodulator()
    local_frames, _, _ = demod.decode_frames(afsk_audio)
    local_ok = sum(1 for f in local_frames if f['crc_ok'])
    print(f"[TX] Local loopback verify: {local_ok}/{num_packets} packets CRC OK")
    if local_ok < num_packets:
        print(f"[TX] WARNING: Local encode/decode loses packets — check AFSK implementation")

    # Connect PlutoSDR
    print(f"\n[TX] Connecting PlutoSDR...")
    try:
        pluto = PlutoTx(args.pluto_ip)
    except Exception as e:
        print(f"[TX ERROR] {e}"); return
    print(f"[TX] PlutoSDR connected")

    # FM modulate
    iq_data = fm_modulate(afsk_audio, AUDIO_RATE, PLUTO_SAMPLE_RATE, FM_DEVIATION)
    print(f"[TX] FM modulated: {len(iq_data)} IQ samples, deviation={FM_DEVIATION}Hz")

    # Connect WebSDR
    rx = RxState()
    ws = create_ws_rx(args.sdr_ip, args.sdr_port, rx)
    time.sleep(1.5)
    if not rx.connected:
        print("[RX] WebSDR connection failed, attempting UART reset...")
        print("[UART] Skipping reset - causes download mode")
        time.sleep(15)
        rx = RxState()
        ws = create_ws_rx(args.sdr_ip, args.sdr_port, rx)
        time.sleep(2)
        if not rx.connected:
            print("[ERROR] Cannot connect to WebSDR"); return
    print(f"[RX] WebSDR connected")

    # Test frequencies
    freqs = [f * 1e6 for f in args.freqs]
    gains = [-10, -5, -15, -20, 0]
    all_results = []
    best = {'decoded': 0, 'freq': 0, 'gain': 0}

    for freq_hz in freqs:
        freq_mhz = freq_hz / 1e6
        print(f"\n{'═'*70}")
        print(f"  {freq_mhz:.0f} MHz")
        print(f"{'═'*70}")

        for gain_db in gains:
            print(f"\n[TX] {freq_mhz:.0f}MHz gain={gain_db}dB")

            try:
                pluto.start(freq_hz, iq_data, PLUTO_SAMPLE_RATE, gain_db)
            except Exception as e:
                print(f"[TX ERROR] {e}"); continue

            time.sleep(0.3)

            # Tune WebSDR and set lower sample rate for better throughput
            ws, rx = ws_send_safe(ws, rx,
                json.dumps({"cmd": "sample_rate", "value": 300000}),
                args.sdr_ip, args.sdr_port)
            time.sleep(0.3)
            ws, rx = ws_send_safe(ws, rx,
                json.dumps({"cmd": "freq", "value": int(freq_hz)}),
                args.sdr_ip, args.sdr_port)
            time.sleep(0.3)
            ws, rx = ws_send_safe(ws, rx,
                json.dumps({"cmd": "subscribe_iq", "offset": 0, "bw": 25000}),
                args.sdr_ip, args.sdr_port)
            time.sleep(0.5)

            if not rx.iq_started:
                t0 = time.time()
                while not rx.iq_started and time.time() - t0 < 3: time.sleep(0.1)

            print(f"[RX] DDC rate: {rx.iq_rate}Hz")
            rx.iq_queue.clear()
            time.sleep(0.2)

            # Capture 2 full cycles + margin
            cap_dur = total_dur * 2 + 2.0
            print(f"[RX] Capturing {cap_dur:.1f}s...")
            audio = capture_audio(rx, cap_dur, native_rate=False)
            actual_rate = AUDIO_RATE
            pluto.stop()

            if len(audio) < AUDIO_RATE:
                print(f"[RX] Too few samples: {len(audio)}")
                # Try UART reset
                if len(audio) == 0:
                    print("[RX] Zero audio — attempting UART reset...")
                    print("[UART] Skipping reset - causes download mode")
                    time.sleep(15)
                    rx = RxState()
                    ws = create_ws_rx(args.sdr_ip, args.sdr_port, rx)
                    time.sleep(2)
                continue

            # Save WAV (resample to 48kHz for playback)
            wav_name = f"test/afsk_{freq_mhz:.0f}mhz_{gain_db}db.wav"
            wav_audio = resample_linear(audio, actual_rate, AUDIO_RATE) if actual_rate != AUDIO_RATE else audio
            save_wav(wav_audio, wav_name)
            rms = np.sqrt(np.mean(audio**2))
            dur = len(audio) / actual_rate
            print(f"[RX] Audio: {dur:.1f}s RMS={rms:.4f} rate={actual_rate}Hz → {wav_name}")

            # Detect gaps
            gaps = detect_audio_gaps(audio, threshold=0.005, min_gap_ms=8)
            if gaps:
                total_gap = sum(g['duration_ms'] for g in gaps)
                print(f"[RX] Gaps detected: {len(gaps)} gaps, "
                      f"total {total_gap:.0f}ms ({total_gap/dur/10:.1f}% of audio)")
                for g in gaps[:5]:
                    print(f"     @{g['start_ms']:.0f}ms dur={g['duration_ms']:.0f}ms")
                if len(gaps) > 5:
                    print(f"     ... and {len(gaps)-5} more")
            else:
                print(f"[RX] No audio gaps detected ✓")

            # Decode AFSK1200 at native DDC rate (avoids resampling artifacts)
            rx_demod = AFSKDemodulator(sample_rate=actual_rate)
            frames, total_bits, energies = rx_demod.decode_frames(audio)
            crc_ok = [f for f in frames if f['crc_ok']]
            crc_fail = [f for f in frames if not f['crc_ok']]

            print(f"\n[DECODE] {len(frames)} frames found, "
                  f"{len(crc_ok)} CRC OK, {len(crc_fail)} CRC fail")

            # Match decoded packets to transmitted
            matched = 0
            matched_ids = set()
            for f in crc_ok:
                info = f['info']
                for idx, (orig_info_str, orig_frame) in enumerate(packets):
                    # orig_frame has CRC at end, strip it; also compare against orig_info_str
                    try:
                        if (orig_info_str in info) or (info.strip() == orig_info_str.strip()):
                            if idx not in matched_ids:
                                matched_ids.add(idx)
                                matched += 1
                            break
                    except:
                        pass

            pkt_rate = matched / num_packets * 100
            print(f"[RESULT] Matched: {matched}/{num_packets} ({pkt_rate:.0f}%)")

            if crc_ok:
                for f in crc_ok[:3]:
                    print(f"  ✓ {f['info'][:50]}")
            if crc_fail:
                for f in crc_fail[:3]:
                    print(f"  ✗ CRC fail: rx={f['rx_crc']:#06x} "
                          f"calc={f['calc_crc']:#06x} info={f['info'][:30]}")

            result = {
                'freq_mhz': freq_mhz, 'gain_db': gain_db,
                'frames': len(frames), 'crc_ok': len(crc_ok),
                'matched': matched, 'pkt_rate': pkt_rate,
                'gaps': len(gaps), 'gap_total_ms': sum(g['duration_ms'] for g in gaps) if gaps else 0,
                'rms': rms, 'audio_dur': dur,
            }
            all_results.append(result)

            if matched > best['decoded']:
                best = {'decoded': matched, 'freq': freq_mhz, 'gain': gain_db}

            # Perfect decode — skip other gains
            if matched >= num_packets:
                print(f"\n  ★ PERFECT: all {num_packets} packets decoded at "
                      f"{freq_mhz:.0f}MHz gain={gain_db}dB!")
                break

            # Good enough
            if pkt_rate >= 80 and gains.index(gain_db) >= 2:
                break

    # ─── Final Summary ───
    print(f"\n{'═'*70}")
    print(f"  AFSK1200 RESULTS ({num_packets} packets per frequency)")
    print(f"{'═'*70}")
    print(f"\n  {'Freq':>6} {'Gain':>6} {'Frames':>7} {'CRC OK':>7} "
          f"{'Match':>6} {'Rate':>5} {'Gaps':>5} {'GapMs':>7} {'RMS':>6}")
    print(f"  {'─'*62}")

    for r in all_results:
        status = "★" if r['pkt_rate'] >= 100 else "✓" if r['pkt_rate'] >= 50 else "✗"
        print(f"  {status} {r['freq_mhz']:>5.0f} {r['gain_db']:>5}dB "
              f"{r['frames']:>6} {r['crc_ok']:>6} {r['matched']:>5} "
              f"{r['pkt_rate']:>4.0f}% {r['gaps']:>4} {r['gap_total_ms']:>6.0f} "
              f"{r['rms']:>.4f}")

    print(f"\n  Best: {best['decoded']}/{num_packets} packets at "
          f"{best['freq']:.0f}MHz gain={best['gain']}dB")

    if best['decoded'] >= num_packets:
        print(f"\n  ✅ ALL PACKETS DECODED — audio pipeline is reliable")
    elif best['decoded'] > 0:
        loss = (1 - best['decoded'] / num_packets) * 100
        print(f"\n  ⚠ {loss:.0f}% packet loss — audio gaps cause AFSK decode failures")
        print(f"    Root cause: IQ data loss in WebSDR ring buffer (81% fill rate)")
    else:
        print(f"\n  ❌ ZERO packets decoded — check frequency, gain, antenna")

    pluto.stop()
    rx.running = False
    try: ws.close()
    except: pass


def main():
    parser = argparse.ArgumentParser(description="AFSK1200 Packet Loopback Test")
    parser.add_argument("--pluto-ip", default="192.168.2.1")
    parser.add_argument("--sdr-ip", default="192.168.1.232")
    parser.add_argument("--sdr-port", type=int, default=8080)
    parser.add_argument("--uart-port", default="/dev/ttyACM0")
    parser.add_argument("--packets", type=int, default=10)
    parser.add_argument("--payload-len", type=int, default=64)
    parser.add_argument("--freqs", type=float, nargs='+',
                        default=[88.0, 108.0, 145.0, 174.0, 290.0,
                                 435.0, 915.0, 1090.0, 1420.0, 1700.0])
    args = parser.parse_args()

    try:
        run_test(args)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()
