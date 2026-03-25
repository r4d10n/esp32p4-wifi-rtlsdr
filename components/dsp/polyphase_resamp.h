/*
 * Polyphase FIR Resampler
 *
 * Efficient rational rate conversion using polyphase decomposition.
 * Prototype filter uses Nuttall window for -93 dB stopband.
 * Sub-filter MACs use PIE SIMD when available.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct polyphase_resamp polyphase_resamp_t;

/**
 * Create polyphase resampler for given rate conversion.
 * @param in_rate    Input sample rate (Hz)
 * @param out_rate   Output sample rate (Hz)
 * @param taps_per_phase  Taps per polyphase branch (8-32, default 16)
 * @return Handle, or NULL on failure
 */
polyphase_resamp_t *polyphase_resamp_create(uint32_t in_rate, uint32_t out_rate,
                                              int taps_per_phase);

/**
 * Free resampler.
 */
void polyphase_resamp_free(polyphase_resamp_t *r);

/**
 * Process samples through resampler.
 * @param r         Resampler handle
 * @param input     Input int16 samples
 * @param in_count  Number of input samples
 * @param output    Output int16 samples
 * @param out_max   Max output capacity
 * @return Number of output samples produced
 */
int polyphase_resamp_process(polyphase_resamp_t *r, const int16_t *input,
                              int in_count, int16_t *output, int out_max);

/**
 * Reset resampler state.
 */
void polyphase_resamp_reset(polyphase_resamp_t *r);

/**
 * Get the actual output rate (may differ slightly from requested due to rational approximation).
 */
uint32_t polyphase_resamp_get_out_rate(polyphase_resamp_t *r);

#ifdef __cplusplus
}
#endif
