#!/usr/bin/env python3
"""
FM Stereo + RDS Transmitter for PlutoSDR

Generates a complete FM broadcast signal with:
  - Stereo audio (L+R mono, L-R on 38 kHz DSB-SC subcarrier)
  - 19 kHz pilot tone
  - RDS data on 57 kHz subcarrier (Group 0A: PS name, Group 2A: RadioText)

Audio source: test tones (default) or MP3/M4A/WAV/FLAC file via --audio.

Requirements:
  pip install pyadi-iio numpy
  ffmpeg (in PATH) — required for --audio file decoding

Usage:
  python fm_stereo_rds_tx.py --freq 100.0 --ps "TEST FM " --rt "Hello RDS!"
  python fm_stereo_rds_tx.py --freq 100.0 --audio music.mp3
  python fm_stereo_rds_tx.py --freq 100.0 --audio song.m4a --ps "MY RADIO"
  python fm_stereo_rds_tx.py --save test.iq --audio music.mp3
"""

import numpy as np
import argparse
import subprocess
import sys
import time
import threading
import queue

# ═══════════════════════════════════════════════════════════════════════════
# RDS Encoder (IEC 62106)
# ═══════════════════════════════════════════════════════════════════════════

RDS_SYMBOL_RATE = 2375.0  # biphase symbols/sec (2 x 1187.5 bps)

# Offset words XORed with CRC for each block position
RDS_OFFSET = {'A': 0x0FC, 'B': 0x198, 'C': 0x168, 'Cp': 0x350, 'D': 0x1B4}


def rds_crc(data_word):
    """Compute 10-bit RDS CRC for 16-bit data word.
    Generator polynomial: x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1"""
    reg = 0
    for i in range(15, -1, -1):
        bit = (data_word >> i) & 1
        fb = bit ^ ((reg >> 9) & 1)
        reg = (reg << 1) & 0x3FF
        if fb:
            reg ^= 0x1B9
    return reg


def rds_block(data, offset_key):
    """Encode one 26-bit RDS block: 16 data + 10 check (CRC XOR offset)."""
    check = rds_crc(data) ^ RDS_OFFSET[offset_key]
    return (data << 10) | check


def rds_group_0a(pi, pty, ps, segment, tp=1, ta=0, ms=1, di=0):
    """Group 0A — Basic tuning info + Programme Service name (2 chars/group)."""
    seg = segment & 0x3
    blk_a = pi
    blk_b = ((0x0 << 12) | (0 << 11) | (tp << 10) | ((pty & 0x1F) << 5) |
              (ta << 4) | (ms << 3) | (((di >> (3 - seg)) & 1) << 2) | seg)
    blk_c = 0xE000  # no AF
    idx = seg * 2
    c1 = ord(ps[idx]) if idx < len(ps) else 0x20
    c2 = ord(ps[idx + 1]) if idx + 1 < len(ps) else 0x20
    blk_d = (c1 << 8) | c2
    return [rds_block(blk_a, 'A'), rds_block(blk_b, 'B'),
            rds_block(blk_c, 'C'), rds_block(blk_d, 'D')]


def rds_group_2a(pi, pty, text, segment, ab_flag=0, tp=1):
    """Group 2A — RadioText (4 chars/group, up to 64 chars)."""
    seg = segment & 0xF
    blk_a = pi
    blk_b = ((0x2 << 12) | (0 << 11) | (tp << 10) | ((pty & 0x1F) << 5) |
              (ab_flag << 4) | seg)
    idx = seg * 4
    cc = [ord(text[idx + i]) if idx + i < len(text) else 0x20 for i in range(4)]
    blk_c = (cc[0] << 8) | cc[1]
    blk_d = (cc[2] << 8) | cc[3]
    return [rds_block(blk_a, 'A'), rds_block(blk_b, 'B'),
            rds_block(blk_c, 'C'), rds_block(blk_d, 'D')]


class RDSEncoder:
    """Generates a continuous stream of RDS groups (0A + 2A)."""

    def __init__(self, pi=0x1234, pty=0, ps="TEST FM ", rt="PlutoSDR RDS"):
        self.pi = pi
        self.pty = pty
        self.ps = ps.ljust(8)[:8]
        self.rt = rt.ljust(64)[:64]
        self.ps_seg = 0
        self.rt_seg = 0
        self.rt_segs = max((len(rt.rstrip()) + 3) // 4, 1)
        self.group_n = 0

    def next_group(self):
        """Return next RDS group, cycling 0A/0A/2A pattern."""
        if self.group_n % 3 < 2:
            g = rds_group_0a(self.pi, self.pty, self.ps, self.ps_seg)
            self.ps_seg = (self.ps_seg + 1) % 4
        else:
            g = rds_group_2a(self.pi, self.pty, self.rt, self.rt_seg)
            self.rt_seg = (self.rt_seg + 1) % self.rt_segs
        self.group_n += 1
        return g

    def group_to_symbols(self, group):
        """Convert 4-block group -> differential biphase symbols (+-1.0).
        104 data bits -> 104 diff bits -> 208 biphase symbols."""
        bits = []
        for block in group:
            for i in range(25, -1, -1):
                bits.append((block >> i) & 1)

        # Differential encoding: 0->toggle, 1->hold
        diff = []
        prev = 0
        for b in bits:
            if b == 0:
                prev = 1 - prev
            diff.append(prev)

        # Biphase (Manchester): each diff bit -> two symbols
        syms = np.empty(len(diff) * 2, dtype=np.float32)
        for i, d in enumerate(diff):
            if d:
                syms[2 * i] = 1.0
                syms[2 * i + 1] = -1.0
            else:
                syms[2 * i] = -1.0
                syms[2 * i + 1] = 1.0
        return syms


# ═══════════════════════════════════════════════════════════════════════════
# Audio File Loader
# ═══════════════════════════════════════════════════════════════════════════

def load_audio_file(path, fs=228000, duration=None):
    """Decode audio file to stereo float32 at target sample rate using ffmpeg.

    Supports MP3, M4A/AAC, WAV, FLAC, OGG, and anything else ffmpeg handles.

    Returns:
        (left, right): Tuple of float32 numpy arrays at target sample rate.
    """
    probe_cmd = [
        'ffprobe', '-v', 'quiet', '-print_format', 'json',
        '-show_format', '-show_streams', path
    ]
    try:
        probe = subprocess.run(probe_cmd, capture_output=True, text=True)
        if probe.returncode == 0:
            import json
            info = json.loads(probe.stdout)
            file_dur = float(info.get('format', {}).get('duration', 0))
            channels = 2
            for s in info.get('streams', []):
                if s.get('codec_type') == 'audio':
                    channels = int(s.get('channels', 2))
                    break
            print(f"      File: {path}")
            print(f"      Duration: {file_dur:.1f}s, channels: {channels}")
    except FileNotFoundError:
        print("ERROR: ffprobe not found. Install ffmpeg.")
        sys.exit(1)

    cmd = ['ffmpeg', '-v', 'quiet', '-i', path]
    if duration is not None and duration > 0:
        cmd += ['-t', str(duration)]
    cmd += ['-f', 'f32le', '-acodec', 'pcm_f32le', '-ac', '2', '-ar', str(fs), '-']

    try:
        proc = subprocess.run(cmd, capture_output=True, timeout=300)
    except FileNotFoundError:
        print("ERROR: ffmpeg not found in PATH.")
        sys.exit(1)

    if proc.returncode != 0 or len(proc.stdout) == 0:
        print(f"ERROR: ffmpeg failed decoding {path}")
        sys.exit(1)

    samples = np.frombuffer(proc.stdout, dtype=np.float32).reshape(-1, 2)
    left = samples[:, 0].copy()
    right = samples[:, 1].copy()

    peak = max(np.max(np.abs(left)), np.max(np.abs(right)), 1e-6)
    left /= peak
    right /= peak

    print(f"      Decoded: {len(left)} samples @ {fs} Hz ({len(left)/fs:.2f}s stereo)")
    return left, right


def generate_test_tones(fs, duration_s, left_freq=1000.0, right_freq=400.0):
    """Generate stereo test tones."""
    n = int(duration_s * fs)
    t = np.arange(n, dtype=np.float64) / fs
    left = np.sin(2.0 * np.pi * left_freq * t).astype(np.float32)
    right = np.sin(2.0 * np.pi * right_freq * t).astype(np.float32)
    return left, right


# ═══════════════════════════════════════════════════════════════════════════
# Pre-emphasis
# ═══════════════════════════════════════════════════════════════════════════

def apply_preemphasis(left, right, fs, tau):
    """Apply first-order pre-emphasis to full L/R arrays.

    Args:
        tau: Time constant in seconds (50e-6 EU, 75e-6 US, 0 = off).
    Returns:
        (left_pe, right_pe): Pre-emphasized and re-normalized arrays.
    """
    if tau <= 0:
        return left.copy(), right.copy()

    alpha = 1.0 / (fs * tau)
    l = _preemphasis_filter(left, alpha)
    r = _preemphasis_filter(right, alpha)

    peak = max(np.max(np.abs(l)), np.max(np.abs(r)), 1e-6)
    l /= peak
    r /= peak
    return l, r


def _preemphasis_filter(x, alpha):
    """y[n] = x[n] + alpha * (x[n] - x[n-1])  — vectorized."""
    d = np.empty_like(x)
    d[0] = 0.0
    d[1:] = x[1:] - x[:-1]
    return x + alpha * d


# ═══════════════════════════════════════════════════════════════════════════
# FM Stereo Multiplex Generator
# ═══════════════════════════════════════════════════════════════════════════

def generate_fm_baseband(left, right, fs_bb, rds_enc,
                         audio_amp=0.4, pilot_amp=0.1,
                         stereo_amp=0.4, rds_amp=0.04,
                         t_offset=0.0):
    """Generate composite FM stereo baseband with RDS.

    Args:
        left, right: Audio arrays at fs_bb, pre-emphasis already applied.
        t_offset: Time offset in seconds for phase-continuous subcarriers.
    """
    n = len(left)
    t = (np.arange(n, dtype=np.float64) / fs_bb) + t_offset

    mono = audio_amp * 0.5 * (left + right)
    diff = stereo_amp * 0.5 * (left - right)

    # 19 kHz pilot — phase-continuous via t_offset
    pilot = pilot_amp * np.sin(2.0 * np.pi * 19000.0 * t).astype(np.float32)

    # 38 kHz DSB-SC stereo subcarrier
    sc38 = np.sin(2.0 * np.pi * 38000.0 * t).astype(np.float32)
    stereo_sub = diff * sc38

    # RDS on 57 kHz subcarrier — vectorized symbol expansion
    sps = int(fs_bb / RDS_SYMBOL_RATE)  # 228000/2375 = 96
    n_syms_needed = (n + sps - 1) // sps
    sym_list = []
    while len(sym_list) < n_syms_needed:
        sym_list.append(rds_enc.group_to_symbols(rds_enc.next_group()))
    all_syms = np.concatenate(sym_list)[:n_syms_needed]
    rds_bb = np.repeat(all_syms, sps)[:n]

    sc57 = np.sin(2.0 * np.pi * 57000.0 * t).astype(np.float32)
    rds_mod = rds_amp * rds_bb * sc57

    composite = mono + pilot + stereo_sub + rds_mod
    return composite


# ═══════════════════════════════════════════════════════════════════════════
# FM Modulator (stateful for phase-continuous chunk processing)
# ═══════════════════════════════════════════════════════════════════════════

def fm_modulate(baseband, fs_bb, fs_out, deviation=75000.0,
                phase_acc=0.0, **_kwargs):
    """FM-modulate baseband with phase continuity across calls.

    Interpolates the FM phase (real-valued) to the output rate, then
    computes exp(j*phase). This avoids expensive complex-signal convolution
    and is mathematically exact for FM.

    Args:
        phase_acc: Starting phase (radians). Pass the returned value
                   to the next call for gapless streaming.

    Returns:
        (iq, new_phase_acc): Complex IQ samples and updated phase.
    """
    # Phase integration with carry-over
    dphi = (2.0 * np.pi * deviation / fs_bb) * baseband.astype(np.float64)
    phase_bb = np.cumsum(dphi) + phase_acc
    new_phase_acc = float(phase_bb[-1]) if len(phase_bb) > 0 else phase_acc

    ratio = int(round(fs_out / fs_bb))
    if ratio <= 1:
        fm = np.exp(1j * phase_bb).astype(np.complex64)
        return fm, new_phase_acc

    # Interpolate phase (real signal) to output rate — linear interp is
    # sufficient for 10x and introduces < 0.01 dB error at 100 kHz BW
    n_bb = len(phase_bb)
    n_out = n_bb * ratio
    idx_bb = np.arange(n_bb, dtype=np.float64)
    idx_out = np.arange(n_out, dtype=np.float64) / ratio
    phase_out = np.interp(idx_out, idx_bb, phase_bb)

    fm = np.exp(1j * phase_out).astype(np.complex64)
    return fm, new_phase_acc


# ═══════════════════════════════════════════════════════════════════════════
# PlutoSDR Transmitter
# ═══════════════════════════════════════════════════════════════════════════

def get_pluto(uri, freq_hz, fs_out, attn_db, cyclic):
    """Connect and configure PlutoSDR."""
    try:
        import adi
    except ImportError:
        print("ERROR: pyadi-iio not found.")
        print("  Install: pip install pyadi-iio")
        print("  Or save to file with --save and use other TX tool.")
        sys.exit(1)

    print(f"Connecting to PlutoSDR at {uri}...")
    sdr = adi.Pluto(uri=uri)
    sdr.sample_rate = int(fs_out)
    sdr.tx_rf_bandwidth = int(200000)
    sdr.tx_lo = int(freq_hz)
    sdr.tx_hardwaregain_chan0 = -abs(attn_db)
    sdr.tx_cyclic_buffer = cyclic
    return sdr


def transmit_pluto_cyclic(iq, uri, freq_hz, fs_out, attn_db=10.0):
    """Send IQ buffer to PlutoSDR in cyclic (looping) mode."""
    sdr = get_pluto(uri, freq_hz, fs_out, attn_db, cyclic=True)

    peak = np.max(np.abs(iq))
    if peak > 0:
        iq = iq / peak

    print(f"TX: {freq_hz / 1e6:.3f} MHz | {fs_out / 1e6:.3f} MSPS | "
          f"atten {abs(attn_db):.1f} dB | {len(iq)} samples")
    sdr.tx(iq * (2**14))

    print("Cyclic buffer active — Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sdr.tx_destroy_buffer()
        print("\nTransmitter stopped.")


def stream_pluto(left, right, fs_bb, fs_out, rds_enc, uri, freq_hz,
                 attn_db=10.0, deviation=75000.0, chunk_sec=1.0,
                 loop=False):
    """Stream audio to PlutoSDR with threaded pre-buffering.

    Uses a producer thread that generates IQ chunks ahead of time into a
    queue, while the main thread feeds the SDR. This eliminates gaps caused
    by real-time processing delays and maintains phase continuity across
    all chunk boundaries.
    """
    sdr = get_pluto(uri, freq_hz, fs_out, attn_db, cyclic=False)

    chunk_bb = int(chunk_sec * fs_bb)
    total = len(left)
    duration = total / fs_bb
    interp_ratio = int(round(fs_out / fs_bb))

    # Pre-buffer queue: producer fills ahead, consumer (TX) drains
    PRE_BUFFER = 4
    iq_queue = queue.Queue(maxsize=PRE_BUFFER + 1)
    stop_event = threading.Event()
    error_box = [None]  # mutable container for thread error reporting

    def producer():
        """Generate IQ chunks with continuous phase and subcarrier timing."""
        try:
            phase_acc = 0.0
            while not stop_event.is_set():
                t_offset = 0.0
                for start in range(0, total, chunk_bb):
                    if stop_event.is_set():
                        return
                    end = min(start + chunk_bb, total)
                    l_chunk = left[start:end]
                    r_chunk = right[start:end]

                    bb = generate_fm_baseband(
                        l_chunk, r_chunk, fs_bb, rds_enc,
                        t_offset=t_offset)

                    iq, phase_acc = fm_modulate(
                        bb, fs_bb, fs_out, deviation,
                        phase_acc=phase_acc)

                    # Pad last chunk to standard size (pyadi-iio requires
                    # constant buffer length across all tx() calls)
                    expected_len = chunk_bb * interp_ratio
                    if len(iq) < expected_len:
                        iq = np.pad(iq, (0, expected_len - len(iq)))

                    # Normalize per-chunk to DAC range
                    peak = np.max(np.abs(iq))
                    if peak > 0:
                        iq = iq / peak
                    scaled = (iq * (2**14)).astype(np.complex64)

                    t_offset += len(l_chunk) / fs_bb

                    # Put into queue; blocks if queue full (backpressure)
                    while not stop_event.is_set():
                        try:
                            iq_queue.put(scaled, timeout=0.5)
                            break
                        except queue.Full:
                            continue

                if not loop:
                    # Signal end-of-stream
                    while not stop_event.is_set():
                        try:
                            iq_queue.put(None, timeout=0.5)
                            break
                        except queue.Full:
                            continue
                    return
        except Exception as e:
            error_box[0] = e
            try:
                iq_queue.put(None, timeout=1)
            except queue.Full:
                pass

    # Start producer thread
    prod_thread = threading.Thread(target=producer, daemon=True)
    prod_thread.start()

    # Wait for pre-buffer to fill before starting TX
    print(f"TX: {freq_hz / 1e6:.3f} MHz | {fs_out / 1e6:.3f} MSPS | "
          f"atten {abs(attn_db):.1f} dB")
    print(f"Streaming {duration:.1f}s audio in {chunk_sec:.1f}s chunks "
          f"(pre-buffer: {PRE_BUFFER})"
          f"{' [loop]' if loop else ''}...")

    # Let producer fill a few chunks before we start sending
    pre_filled = 0
    while pre_filled < PRE_BUFFER and not stop_event.is_set():
        if iq_queue.qsize() >= min(PRE_BUFFER, (total + chunk_bb - 1) // chunk_bb):
            break
        time.sleep(0.05)
        pre_filled = iq_queue.qsize()
    print(f"Pre-buffered {iq_queue.qsize()} chunks. Transmitting...")
    print("Ctrl+C to stop.\n")

    chunk_count = 0
    chunks_per_pass = (total + chunk_bb - 1) // chunk_bb

    try:
        while True:
            try:
                chunk = iq_queue.get(timeout=2.0)
            except queue.Empty:
                if error_box[0]:
                    print(f"\nProducer error: {error_box[0]}")
                    break
                continue

            if chunk is None:
                break

            sdr.tx(chunk)
            chunk_count += 1

            pos = min(chunk_count % chunks_per_pass, chunks_per_pass) * chunk_sec
            total_pos = chunk_count * chunk_sec
            sys.stdout.write(
                f"\r  [{pos:6.1f}s / {duration:.1f}s]  "
                f"total: {total_pos:.0f}s  buf: {iq_queue.qsize()}")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        sdr.tx_destroy_buffer()
        prod_thread.join(timeout=3)
        print("\nTransmitter stopped.")


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description='FM Stereo + RDS Transmitter for PlutoSDR',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  %(prog)s --freq 100.0 --ps "MY RADIO" --rt "Now playing test tones"
  %(prog)s --freq 100.0 --audio music.mp3 --ps "MY RADIO"
  %(prog)s --freq 100.0 --audio song.m4a --rt "Artist - Title"
  %(prog)s --freq 88.1 --left-freq 440 --right-freq 880 --attn 20
  %(prog)s --save fm_test.iq --audio music.mp3
  %(prog)s --save fm_test.iq --save-format cf32
""")
    ag = p.add_argument_group('audio source (default: test tones)')
    ag.add_argument('--audio', type=str, default=None, metavar='FILE',
                    help='Audio file (MP3, M4A, WAV, FLAC, OGG, ...)')
    ag.add_argument('--left-freq', type=float, default=1000.0,
                    help='Left test tone Hz (default: 1000, ignored with --audio)')
    ag.add_argument('--right-freq', type=float, default=400.0,
                    help='Right test tone Hz (default: 400, ignored with --audio)')

    rg = p.add_argument_group('RF settings')
    rg.add_argument('--freq', type=float, default=100.0,
                    help='TX frequency MHz (default: 100.0)')
    rg.add_argument('--uri', default='ip:192.168.2.1',
                    help='PlutoSDR URI (default: ip:192.168.2.1)')
    rg.add_argument('--attn', type=float, default=10.0,
                    help='TX attenuation dB (default: 10)')
    rg.add_argument('--deviation', type=float, default=75000.0,
                    help='FM deviation Hz (default: 75000)')

    rdg = p.add_argument_group('RDS settings')
    rdg.add_argument('--ps', default='TEST FM ',
                     help='Programme Service name, 8 chars (default: "TEST FM ")')
    rdg.add_argument('--rt', default='Hello from PlutoSDR FM Stereo + RDS!',
                     help='RadioText, max 64 chars')
    rdg.add_argument('--pi', type=lambda x: int(x, 0), default=0x1234,
                     help='PI code hex (default: 0x1234)')
    rdg.add_argument('--pty', type=int, default=0,
                     help='PTY code 0-31 (default: 0)')

    og = p.add_argument_group('output')
    og.add_argument('--duration', type=float, default=0,
                    help='Clip duration seconds (0 = full file or 2s for tones)')
    og.add_argument('--loop', action='store_true',
                    help='Loop audio playback (default for test tones)')
    og.add_argument('--preemphasis', type=float, default=50e-6, metavar='TAU',
                    help='Pre-emphasis time constant (50e-6 EU, 75e-6 US, 0=off)')
    og.add_argument('--chunk', type=float, default=1.0,
                    help='Streaming chunk size seconds (default: 1.0)')
    og.add_argument('--save', type=str, default=None,
                    help='Save IQ to file instead of transmitting')
    og.add_argument('--save-format', choices=['cs16', 'cf32'], default='cs16',
                    help='IQ file format: cs16 (int16) or cf32 (float32)')
    args = p.parse_args()

    fs_bb = 228000
    fs_out = 2280000

    freq_hz = int(args.freq * 1e6)
    using_audio = args.audio is not None

    if args.duration > 0:
        duration = args.duration
    elif using_audio:
        duration = 0  # entire file
    else:
        duration = 2.0

    # --- Header ---
    print("╔══════════════════════════════════════════════╗")
    print("║   FM Stereo + RDS Transmitter (PlutoSDR)     ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║  Frequency  : {args.freq:>8.3f} MHz                  ║")
    print(f"║  Deviation  : {args.deviation/1e3:>5.0f} kHz                     ║")
    if using_audio:
        fname = args.audio if len(args.audio) <= 28 else '...' + args.audio[-25:]
        print(f"║  Audio      : {fname:<28s}   ║")
    else:
        print(f"║  L tone     : {args.left_freq:>6.0f} Hz                     ║")
        print(f"║  R tone     : {args.right_freq:>6.0f} Hz                     ║")
    print(f"║  PS Name    : {args.ps:<8s}                      ║")
    print(f"║  RadioText  : {args.rt[:28]:<28s}   ║")
    print(f"║  PI / PTY   : 0x{args.pi:04X} / {args.pty:<2d}                    ║")
    pe_label = f"{args.preemphasis*1e6:.0f}us" if args.preemphasis > 0 else "off"
    print(f"║  Pre-emph   : {pe_label:<6s}                        ║")
    print(f"║  Baseband   : {fs_bb/1e3:.0f} kHz -> {fs_out/1e6:.2f} MSPS          ║")
    print("╚══════════════════════════════════════════════╝")
    print()

    rds = RDSEncoder(pi=args.pi, pty=args.pty, ps=args.ps, rt=args.rt)

    # --- Load / generate audio ---
    if using_audio:
        print("[1/3] Loading audio file...")
        left, right = load_audio_file(args.audio, fs=fs_bb,
                                      duration=duration if duration > 0 else None)
    else:
        print("[1/3] Generating test tones...")
        left, right = generate_test_tones(fs_bb, duration,
                                          args.left_freq, args.right_freq)
        print(f"      {len(left)} samples @ {fs_bb} Hz ({duration:.2f}s)")

    # --- Apply pre-emphasis to full audio (avoids filter state across chunks) ---
    if args.preemphasis > 0:
        print("      Applying pre-emphasis...")
        left, right = apply_preemphasis(left, right, fs_bb, args.preemphasis)

    # --- Process and output ---
    if args.save or not using_audio:
        # Batch mode: process entire buffer at once
        print("[2/3] Generating FM stereo multiplex + RDS...")
        baseband = generate_fm_baseband(left, right, fs_bb, rds)
        print(f"      {len(baseband)} baseband samples")

        print("[3/3] FM modulating + interpolating...")
        iq, _ = fm_modulate(baseband, fs_bb, fs_out, deviation=args.deviation)
        print(f"      {len(iq)} IQ samples @ {fs_out} Hz ({len(iq)/fs_out:.2f}s)")

        if args.save:
            print(f"      Saving to {args.save} ({args.save_format})...")
            if args.save_format == 'cf32':
                iq_norm = iq / np.max(np.abs(iq))
                iq_norm.tofile(args.save)
            else:
                peak = np.max(np.abs(iq))
                if peak > 0:
                    iq = iq / peak
                out = np.empty(len(iq) * 2, dtype=np.int16)
                out[0::2] = (np.real(iq) * 32767).astype(np.int16)
                out[1::2] = (np.imag(iq) * 32767).astype(np.int16)
                out.tofile(args.save)
            sz_mb = len(iq) * (8 if args.save_format == 'cf32' else 4) / 1e6
            print(f"      Done ({sz_mb:.1f} MB)")
        else:
            print("      Transmitting (cyclic)...")
            transmit_pluto_cyclic(iq, args.uri, freq_hz, fs_out,
                                  attn_db=args.attn)
    else:
        # Streaming mode: threaded producer/consumer for gapless audio
        print("[2/3] Streaming mode — threaded pre-buffered pipeline")
        print("[3/3] Transmitting...")
        stream_pluto(left, right, fs_bb, fs_out, rds, args.uri, freq_hz,
                     attn_db=args.attn, deviation=args.deviation,
                     chunk_sec=args.chunk, loop=args.loop)


if __name__ == '__main__':
    main()
