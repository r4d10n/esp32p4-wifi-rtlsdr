/*
 * PIE SIMD FM Discriminator
 *
 * Two selectable methods for FM phase discrimination:
 * Method A (POLY_CORRECTED): esp.cmul.s16 conjugate + polynomial atan2, <0.3% THD
 * Method B (FAST_LINEAR): esp.cmul.s16 conjugate + linear approx + correction, ~1% THD
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FM_DISC_POLY_CORRECTED = 0,  /* Best quality: polynomial atan2, <0.3% THD, ~6 cyc/sample */
    FM_DISC_FAST_LINEAR = 1,     /* Fast: linear approx + correction, ~1% THD, ~4 cyc/sample */
} fm_disc_mode_t;

typedef struct {
    fm_disc_mode_t mode;
    int16_t prev_i;      /* Previous sample I (state between calls) */
    int16_t prev_q;      /* Previous sample Q */
    int16_t fm_scale;    /* Normalization scale factor */
} fm_disc_state_t;

/**
 * Initialize discriminator state.
 * @param state     State struct to initialize
 * @param mode      Discriminator method
 * @param sample_rate Input sample rate in Hz
 * @param deviation  FM deviation in Hz (75000 WBFM, 5000 NBFM)
 */
void fm_disc_init(fm_disc_state_t *state, fm_disc_mode_t mode,
                  uint32_t sample_rate, uint32_t deviation);

/**
 * Reset state (call on frequency change).
 */
void fm_disc_reset(fm_disc_state_t *state);

/**
 * Process a block of IQ samples through the FM discriminator.
 *
 * @param state     Discriminator state (prev_i/prev_q updated)
 * @param iq_in     int16 interleaved IQ [I0,Q0,I1,Q1,...] (16-byte aligned)
 * @param audio_out int16 demodulated audio output (16-byte aligned)
 * @param n_pairs   Number of IQ pairs (must be multiple of 4 for SIMD path)
 */
void fm_disc_process(fm_disc_state_t *state, const int16_t *iq_in,
                     int16_t *audio_out, int n_pairs);

#ifdef __cplusplus
}
#endif
