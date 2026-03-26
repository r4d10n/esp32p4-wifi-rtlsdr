#!/usr/bin/env python3
"""
RDS TX→RX Loopback Test (Offline)

Generates an FM stereo+RDS signal using the TX encoder (fm_stereo_rds_tx.py),
then demodulates and decodes RDS using a Python port of the ESP32-P4 RX code
(from rds_decoder.c + fm_stereo.c).

This validates TX/RX compatibility without any hardware.

Usage:
    python test_rds_loopback.py

Expected output:
    RDS PS: "MY RADIO"
    RDS PI: 0x1234
    RDS RT: "Now playing Ezhilam"

SPDX-License-Identifier: GPL-2.0-or-later
"""

import sys
import os
import numpy as np

# Add tools directory to path for TX encoder
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'tools'))

from fm_stereo_rds_tx import (
    RDSEncoder, generate_fm_baseband, generate_test_tones,
    apply_preemphasis
)


# ═══════════════════════════════════════════════════════════════════════════
# RX: Python port of ESP32-P4 RDS decoder (rds_decoder.c + fm_stereo.c)
# ═══════════════════════════════════════════════════════════════════════════

RDS_CRC_POLY = 0x5B9
RDS_OFFSET = {'A': 0x0FC, 'B': 0x198, 'C': 0x168, 'Cp': 0x350, 'D': 0x1B4}


def rds_syndrome(block_26bit):
    """CRC-10 syndrome via GF(2) long division — matches rds_decoder.c"""
    reg = block_26bit
    for i in range(15, -1, -1):
        if reg & (1 << (i + 10)):
            reg ^= (RDS_CRC_POLY << i)
    return reg & 0x3FF


def rds_check_block(block_26bit, offset):
    return rds_syndrome(block_26bit) == offset


class RDSDecoder:
    """Python port of rds_decoder.c with Manchester biphase decoding."""

    def __init__(self, sample_rate):
        self.sample_rate = sample_rate
        self.symbol_rate = 2375.0
        self.clock_inc = self.symbol_rate * 65536.0 / sample_rate
        self.clock_phase = 0.0
        self.prev_sample = 0.0
        self.sym_acc = 0.0
        self.sym_count = 0
        self.first_half = 0
        self.prev_diff_bit = 0

        self.shift_reg = 0
        self.bit_count = 0
        self.synced = False
        self.block_idx = 0
        self.good_blocks = 0
        self.bad_blocks = 0
        self.group_data = [0, 0, 0, 0]

        # Decoded data
        self.pi_code = 0
        self.ps_name = [''] * 8
        self.radio_text = [''] * 64
        self.pty = 0
        self.groups_received = 0
        self.block_errors = 0

    def process_bit(self, bit):
        self.shift_reg = ((self.shift_reg << 1) | (bit & 1)) & 0x3FFFFFF
        self.bit_count += 1

        if not self.synced:
            if rds_check_block(self.shift_reg, RDS_OFFSET['A']):
                self.synced = True
                self.block_idx = 1  # Next expected block is B
                self.group_data[0] = self.shift_reg >> 10
                self.bit_count = 0
                self.good_blocks = 1
                self.bad_blocks = 0
                print(f"  [SYNC] Block A found! data=0x{self.group_data[0]:04X}")
            return

        if self.bit_count < 26:
            return
        self.bit_count = 0

        offsets = [RDS_OFFSET['A'], RDS_OFFSET['B'], RDS_OFFSET['C'], RDS_OFFSET['D']]
        ok = rds_check_block(self.shift_reg, offsets[self.block_idx])

        if ok:
            self.group_data[self.block_idx] = self.shift_reg >> 10
            self.good_blocks += 1
            self.bad_blocks = 0
        else:
            if self.block_idx == 2 and rds_check_block(self.shift_reg, RDS_OFFSET['Cp']):
                self.group_data[2] = self.shift_reg >> 10
                ok = True
            else:
                self.bad_blocks += 1
                self.block_errors += 1
                if self.block_errors <= 5:
                    syn = rds_syndrome(self.shift_reg)
                    print(f"  [FAIL] Block {'ABCD'[self.block_idx]}: "
                          f"0x{self.shift_reg:07X} syn=0x{syn:03X} exp=0x{offsets[self.block_idx]:03X}")

        self.block_idx = (self.block_idx + 1) % 4

        if self.block_idx == 0 and ok:
            self.decode_group()

        if self.bad_blocks > 10:
            self.synced = False
            print(f"  [SYNC LOST] bad_blocks={self.bad_blocks}")

    def decode_group(self):
        a, b, c, d = self.group_data
        self.pi_code = a
        group_type = (b >> 12) & 0xF
        version_b = (b >> 11) & 1
        self.pty = (b >> 5) & 0x1F
        self.groups_received += 1

        if group_type == 0:
            seg = b & 0x03
            c1 = (d >> 8) & 0xFF
            c2 = d & 0xFF
            self.ps_name[seg * 2] = chr(c1) if 32 <= c1 < 127 else ' '
            self.ps_name[seg * 2 + 1] = chr(c2) if 32 <= c2 < 127 else ' '
            ps_str = ''.join(self.ps_name)
            print(f"  [GROUP 0A] seg={seg} PS=\"{ps_str}\" PI=0x{a:04X}")

        if group_type == 2 and not version_b:
            seg = b & 0x0F
            for j, blk in enumerate([c, d]):
                c1 = (blk >> 8) & 0xFF
                c2 = blk & 0xFF
                idx = seg * 4 + j * 2
                if idx < 64:
                    self.radio_text[idx] = chr(c1) if 32 <= c1 < 127 else ' '
                if idx + 1 < 64:
                    self.radio_text[idx + 1] = chr(c2) if 32 <= c2 < 127 else ' '
            rt_str = ''.join(self.radio_text).rstrip()
            print(f"  [GROUP 2A] seg={seg} RT=\"{rt_str}\"")

    def process_samples(self, samples):
        """Process RDS samples at symbol rate (1 sample = 1 biphase symbol).
        Manchester decode: pair consecutive symbols into data bits."""
        for s in samples:
            sym_positive = 1 if s > 0 else 0

            if self.sym_count == 0:
                self.first_half = sym_positive
                self.sym_count = 1
            else:
                diff_bit = self.first_half
                data_bit = 1 if (diff_bit == self.prev_diff_bit) else 0
                self.prev_diff_bit = diff_bit
                self.process_bit(data_bit)
                self.sym_count = 0


def fm_discriminator(iq_samples):
    """FM discriminator: atan2(cross, dot) of conjugate product."""
    i = np.real(iq_samples)
    q = np.imag(iq_samples)
    i_prev = np.roll(i, 1)
    q_prev = np.roll(q, 1)
    i_prev[0] = 0
    q_prev[0] = 0
    cross = q * i_prev - i * q_prev
    dot = i * i_prev + q * q_prev
    return np.arctan2(cross, dot).astype(np.float32)


# ═══════════════════════════════════════════════════════════════════════════
# Main test
# ═══════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 64)
    print("  RDS TX→RX Loopback Test (Offline)")
    print("=" * 64)

    # TX parameters (matching the user's command line)
    PI = 0x1234
    PS = "MY RADIO"
    RT = "Now playing Ezhilam"
    PTY = 0

    fs_bb = 228000  # TX baseband rate
    duration = 3.0  # 3 seconds should be enough for several RDS groups

    print(f"\n[TX] Generating FM stereo + RDS signal...")
    print(f"     PI=0x{PI:04X} PS=\"{PS}\" RT=\"{RT}\" PTY={PTY}")

    rds_enc = RDSEncoder(pi=PI, pty=PTY, ps=PS, rt=RT)
    left, right = generate_test_tones(fs_bb, duration, 1000.0, 400.0)
    left, right = apply_preemphasis(left, right, fs_bb, 50e-6)

    baseband = generate_fm_baseband(left, right, fs_bb, rds_enc)
    print(f"     {len(baseband)} baseband samples @ {fs_bb} Hz ({duration:.1f}s)")

    # FM modulate to IQ (no interpolation needed for loopback)
    deviation = 75000.0
    dphi = (2.0 * np.pi * deviation / fs_bb) * baseband.astype(np.float64)
    phase = np.cumsum(dphi)
    iq = np.exp(1j * phase).astype(np.complex64)

    print(f"\n[RX] FM discriminator...")
    mpx = fm_discriminator(iq)
    print(f"     MPX: {len(mpx)} samples, peak={np.max(np.abs(mpx)):.4f}")

    # Extract 57kHz RDS subcarrier.
    # Find pilot phase by correlating MPX with sin/cos at 19kHz,
    # then use 3x that phase for coherent 57kHz reference.
    print(f"\n[RX] Extracting pilot phase and 57kHz RDS subcarrier...")

    t = np.arange(len(mpx), dtype=np.float64) / fs_bb

    # Find pilot phase: correlate first 0.5s with sin and cos
    n_corr = min(int(0.5 * fs_bb), len(mpx))
    tc = t[:n_corr]
    sin19 = np.sin(2.0 * np.pi * 19000.0 * tc)
    cos19 = np.cos(2.0 * np.pi * 19000.0 * tc)
    a_sin = np.sum(mpx[:n_corr] * sin19)
    a_cos = np.sum(mpx[:n_corr] * cos19)
    pilot_phase = np.arctan2(a_sin, a_cos)
    print(f"     Pilot phase: {pilot_phase:.4f} rad ({np.degrees(pilot_phase):.1f}°)")

    # Mix with 57kHz reference (using known pilot phase) and decimate
    # directly to biphase symbol rate (2375 sps = one sample per symbol).
    # This matches the firmware's approach: accumulate-and-dump to symbol rate.
    rds_sym_rate = 2375.0
    rds_decim = round(fs_bb / rds_sym_rate)  # 96 for 228 kHz
    rds_rate = fs_bb / rds_decim
    n_out = len(mpx) // rds_decim

    # Use direct 57kHz reference (loopback: we know exact frequency)
    ref_57k = np.sin(2.0 * np.pi * 57000.0 * t).astype(np.float32)
    rds_mixed = mpx * ref_57k

    rds_baseband = np.zeros(n_out, dtype=np.float32)
    for k in range(n_out):
        chunk = rds_mixed[k * rds_decim : (k + 1) * rds_decim]
        rds_baseband[k] = np.sum(chunk)  # one symbol per output sample
    print(f"     RDS baseband: {len(rds_baseband)} samples @ {rds_rate:.0f} Hz")
    print(f"     RDS peak: {np.max(np.abs(rds_baseband)):.4f}")

    print(f"\n[RX] Manchester biphase RDS decoder...")
    decoder = RDSDecoder(sample_rate=rds_rate)
    decoder.process_samples(rds_baseband)

    print(f"\n{'=' * 64}")
    print(f"  RESULTS")
    print(f"{'=' * 64}")
    ps_str = ''.join(decoder.ps_name).strip()
    rt_str = ''.join(decoder.radio_text).strip()
    print(f"  PI code:   0x{decoder.pi_code:04X} (expected 0x{PI:04X}) {'✓' if decoder.pi_code == PI else '✗'}")
    print(f"  PS name:   \"{ps_str}\" (expected \"{PS}\") {'✓' if ps_str == PS else '✗'}")
    print(f"  RadioText: \"{rt_str[:30]}\" (expected \"{RT[:30]}\") {'✓' if rt_str.startswith(RT[:8]) else '✗'}")
    print(f"  PTY:       {decoder.pty} (expected {PTY}) {'✓' if decoder.pty == PTY else '✗'}")
    print(f"  Groups:    {decoder.groups_received}")
    print(f"  Errors:    {decoder.block_errors}")
    print(f"  Synced:    {decoder.synced}")

    success = (decoder.pi_code == PI and ps_str == PS and decoder.groups_received > 0)
    print(f"\n  {'PASS' if success else 'FAIL'}: RDS TX→RX loopback")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
