/*
 * Simple WBFM Demodulator — Plain C, Floating Point
 *
 * Full pipeline: U8 IQ → CIC decimate → FM discriminator → de-emphasis →
 * FIR LPF → resample → int16 audio output.
 *
 * No SIMD, no Q15, no assembly. Just clean float math.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fm_demod_simple fm_demod_simple_t;

/**
 * Create a new FM demodulator instance.
 *
 * @param sample_rate  Input IQ sample rate in Hz (e.g. 1024000)
 * @param audio_rate   Output audio sample rate in Hz (e.g. 48000)
 * @return Handle, or NULL on failure.
 */
fm_demod_simple_t *fm_demod_simple_create(uint32_t sample_rate, uint32_t audio_rate);

/**
 * Free a demodulator instance.
 */
void fm_demod_simple_free(fm_demod_simple_t *d);

/**
 * Process uint8 IQ input, produce float audio output.
 *
 * @param d          Demodulator handle
 * @param iq_u8      Input IQ data (interleaved I,Q uint8 pairs)
 * @param iq_bytes   Number of input bytes (must be even)
 * @param audio_out  Output buffer for float audio samples [-1.0, 1.0]
 * @param audio_max  Maximum number of output samples
 * @return Number of audio samples produced
 */
int fm_demod_simple_process(fm_demod_simple_t *d, const uint8_t *iq_u8, int iq_bytes,
                            float *audio_out, int audio_max);

/**
 * Get current signal strength (RMS of recent IQ samples).
 */
float fm_demod_simple_get_signal_strength(fm_demod_simple_t *d);

#ifdef __cplusplus
}
#endif
