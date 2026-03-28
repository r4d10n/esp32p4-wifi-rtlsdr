/*
 * WBFM Demodulator with Stereo — Plain C, Floating Point
 *
 * Mono:   U8 IQ → FM discriminator → de-emphasis → FIR LPF → resample
 * Stereo: FM discriminator → Goertzel pilot → PLL → L-R demod → matrix
 *
 * No SIMD, no Q15, no assembly. Just clean float math.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "rds_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fm_demod_simple fm_demod_simple_t;

/**
 * Create a new FM demodulator instance.
 *
 * @param sample_rate  Input IQ sample rate in Hz (e.g. 256000)
 * @param audio_rate   Output audio sample rate in Hz (e.g. 48000)
 * @return Handle, or NULL on failure.
 */
fm_demod_simple_t *fm_demod_simple_create(uint32_t sample_rate, uint32_t audio_rate);

void fm_demod_simple_free(fm_demod_simple_t *d);

/**
 * Process uint8 IQ input, produce interleaved stereo int16 output.
 *
 * When stereo pilot is detected, output is interleaved [L,R,L,R,...].
 * When mono, output is duplicated [M,M,M,M,...].
 *
 * @param d          Demodulator handle
 * @param iq_u8      Input IQ data (interleaved I,Q uint8 pairs)
 * @param iq_bytes   Number of input bytes (must be even)
 * @param audio_out  Output buffer for int16 stereo pairs
 * @param max_pairs  Maximum number of stereo pairs (frames)
 * @return Number of stereo pairs produced
 */
int fm_demod_simple_process(fm_demod_simple_t *d, const uint8_t *iq_u8, int iq_bytes,
                            int16_t *audio_out, int max_pairs);

float fm_demod_simple_get_signal_strength(fm_demod_simple_t *d);
bool  fm_demod_simple_is_stereo(fm_demod_simple_t *d);
void  fm_demod_simple_get_rds(fm_demod_simple_t *d, rds_data_t *out);

#define MPX_SPECTRUM_BINS 128

/** Get MPX baseband power spectrum (128 bins, 0-128kHz, 1kHz resolution).
 *  Output: uint8 log-magnitude (0=-80dB, 255=0dB). */
void fm_demod_simple_get_mpx_spectrum(fm_demod_simple_t *d, uint8_t *bins_out, int *num_bins);

#ifdef __cplusplus
}
#endif
