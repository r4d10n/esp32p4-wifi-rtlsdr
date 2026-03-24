/*
 * PIE SIMD Kernel Functions for ESP32-P4
 *
 * Optimized signal processing kernels using PIE 128-bit SIMD.
 * Each function has a C reference implementation and (future) assembly fast path.
 * All buffers must be 16-byte aligned. Lengths must be multiples of 8 (int16).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── FFT Pipeline Kernels ── */

/**
 * Initialize heap-allocated scratch buffer for power spectrum accumulation.
 * Must be called before pie_power_spectrum_accumulate when using PIE assembly.
 * Call with the FFT size. Safe to call multiple times (reallocs only if needed).
 */
void pie_pwr_scratch_init(int fft_n);

/**
 * Free the power spectrum scratch buffer.
 */
void pie_pwr_scratch_free(void);

/**
 * Convert uint8 IQ pairs to windowed int16 in a single pass.
 *
 * For each IQ pair: output = ((uint8 - 128) << 8) * window_q15 >> 15
 * Produces interleaved int16 I,Q suitable for dsps_fft2r_sc16.
 *
 * @param src       uint8 interleaved IQ data (I0,Q0,I1,Q1,...) [16-byte aligned]
 * @param window    Q15 Hann window coefficients [16-byte aligned, fft_n entries]
 * @param dst       int16 interleaved IQ output [16-byte aligned, fft_n*2 entries]
 * @param iq_pairs  Number of IQ pairs to process (must be multiple of 8)
 */
void pie_u8iq_to_s16_windowed(const uint8_t *src, const int16_t *window,
                               int16_t *dst, int iq_pairs);

/**
 * Accumulate power spectrum from int16 FFT output into int64 buffer.
 *
 * For each bin k: accum[k] += re[k]*re[k] + im[k]*im[k]
 * Input is interleaved [re0,im0,re1,im1,...] from dsps_fft2r_sc16.
 *
 * @param fft_out   int16 interleaved FFT output [16-byte aligned]
 * @param accum     int64 power accumulator [16-byte aligned, fft_n entries]
 * @param fft_n     FFT size (number of bins, must be multiple of 8)
 */
void pie_power_spectrum_accumulate(const int16_t *fft_out, int64_t *accum, int fft_n);

/**
 * Convert int64 power spectrum to uint8 dB with FFT shift.
 *
 * Uses fast_log10f approximation (IEEE 754 bit manipulation) for speed.
 * Output is FFT-shifted: second half first, then first half.
 *
 * @param power     int64 accumulated power [16-byte aligned, fft_n entries]
 * @param db_out    uint8 dB output (0-255) [fft_n entries]
 * @param fft_n     FFT size
 * @param avg_count Number of frames averaged (for normalization)
 * @param db_min    Minimum dB value (maps to 0)
 * @param db_max    Maximum dB value (maps to 255)
 */
void pie_power_to_db_u8(const int64_t *power, uint8_t *db_out, int fft_n,
                         int avg_count, float db_min, float db_max);

/* ── DDC Pipeline Kernels ── */

/**
 * NCO (Numerically Controlled Oscillator) with 32-bit phase accumulator.
 * Uses a fixed-size lookup table with phase accumulator for zero discontinuity.
 * Stores interleaved [cos, -sin] pairs as Q15 int16.
 */
typedef struct {
    int16_t    *table;       /* Interleaved [cos,-sin,cos,-sin,...] Q15 */
    uint32_t    table_len;   /* Number of complex entries (power of 2) */
    uint32_t    phase_acc;   /* 32-bit phase accumulator (wraps naturally) */
    uint32_t    phase_inc;   /* Phase increment per sample */
} pie_nco_t;

/**
 * Create NCO with 32-bit phase accumulator for given offset frequency.
 * Phase wraps naturally at 2^32 with no discontinuity regardless of frequency.
 *
 * @param sample_rate  Input sample rate in Hz
 * @param offset_hz    Frequency offset from center (signed)
 * @return NCO handle, or NULL on failure. Free with pie_nco_free().
 */
pie_nco_t *pie_nco_create(uint32_t sample_rate, int32_t offset_hz);

/**
 * Free NCO table.
 */
void pie_nco_free(pie_nco_t *nco);

/**
 * Convert uint8 IQ to int16 with bias removal.
 *
 * output[i] = ((int16_t)input[i] - 128) << 8
 * Gives full Q15 range [-32768, +32512].
 *
 * @param src       uint8 IQ data [16-byte aligned]
 * @param dst       int16 output [16-byte aligned]
 * @param count     Number of uint8 samples (must be multiple of 16)
 */
void pie_u8_to_s16_bias(const uint8_t *src, int16_t *dst, int count);

/**
 * Complex multiply int16 IQ stream with NCO (frequency shift).
 * Uses 32-bit phase accumulator for zero-discontinuity phase tracking.
 *
 * @param iq_in     int16 interleaved IQ input [16-byte aligned]
 * @param nco       NCO handle (phase auto-advanced)
 * @param iq_out    int16 interleaved IQ output [16-byte aligned]
 * @param iq_pairs  Number of IQ pairs (must be multiple of 8)
 */
void pie_nco_mix_s16(const int16_t *iq_in, pie_nco_t *nco,
                      int16_t *iq_out, int iq_pairs);

/**
 * 3rd-order CIC decimation on int16 interleaved IQ.
 *
 * Implements 3 integrator stages (at input rate) followed by
 * 3 comb stages (at output rate) for -39 dB alias rejection.
 * Uses int64 internal accumulators to handle bit growth.
 *
 * @param iq_in       int16 interleaved IQ input
 * @param in_pairs    Number of input IQ pairs
 * @param iq_out      int16 interleaved IQ output
 * @param out_pairs   On entry: max output pairs. On exit: actual output pairs.
 * @param decim_ratio Decimation ratio (must be power of 2)
 * @param accum       Persistent state: int32[26] (integrators + combs + counter)
 */
void pie_cic_decimate_s16(const int16_t *iq_in, int in_pairs,
                           int16_t *iq_out, int *out_pairs,
                           int decim_ratio, int32_t *accum);

/**
 * Convert int16 IQ to uint8 IQ.
 *
 * output[i] = clamp((input[i] >> 8) + 128, 0, 255)
 *
 * @param src       int16 IQ data [16-byte aligned]
 * @param dst       uint8 output
 * @param count     Number of int16 samples (must be multiple of 16)
 */
void pie_s16_to_u8(const int16_t *src, uint8_t *dst, int count);

#ifdef __cplusplus
}
#endif
