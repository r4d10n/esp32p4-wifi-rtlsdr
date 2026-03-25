/*
 * FM Demodulator for ESP32-P4
 *
 * Fixed-point (Q15) WBFM/NBFM demodulator with:
 * - Fast atan2 discriminator (no trig, no division)
 * - Single-pole IIR de-emphasis (75us US / 50us EU)
 * - FIR audio low-pass filter (63 taps)
 * - Fractional resampler to 48 kHz audio output
 *
 * All processing in int16 Q15 for PIE SIMD compatibility.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FM modes */
typedef enum {
    FM_DEMOD_WBFM = 0,     /* Wideband FM: ±75 kHz deviation, 15 kHz audio */
    FM_DEMOD_NBFM = 1,     /* Narrowband FM: ±5 kHz deviation, 3 kHz audio */
} fm_demod_mode_t;

/* Demodulator configuration */
typedef struct {
    fm_demod_mode_t mode;           /* WBFM or NBFM */
    uint32_t        sample_rate;    /* Input IQ sample rate in Hz */
    uint32_t        audio_rate;     /* Output audio rate (typically 48000) */
    float           de_emphasis_tau; /* De-emphasis time constant (75e-6 or 50e-6) */
    uint32_t        deviation;      /* FM deviation in Hz (75000 WBFM, 5000 NBFM) */
    uint32_t        audio_lpf_cutoff; /* Audio LPF cutoff in Hz */
} fm_demod_config_t;

#define FM_DEMOD_CONFIG_WBFM() { \
    .mode = FM_DEMOD_WBFM, \
    .sample_rate = 256000, \
    .audio_rate = 48000, \
    .de_emphasis_tau = 75e-6f, \
    .deviation = 75000, \
    .audio_lpf_cutoff = 15000, \
}

#define FM_DEMOD_CONFIG_NBFM() { \
    .mode = FM_DEMOD_NBFM, \
    .sample_rate = 32000, \
    .audio_rate = 48000, \
    .de_emphasis_tau = 75e-6f, \
    .deviation = 5000, \
    .audio_lpf_cutoff = 3000, \
}

typedef struct fm_demod fm_demod_t;

/**
 * Create FM demodulator instance.
 * Allocates FIR filter taps and internal buffers.
 */
fm_demod_t *fm_demod_create(const fm_demod_config_t *config);

/**
 * Free demodulator and all internal buffers.
 */
void fm_demod_free(fm_demod_t *demod);

/**
 * Process IQ samples through the demodulator.
 *
 * @param demod     Demodulator handle
 * @param iq_in     int16 interleaved IQ input [I0,Q0,I1,Q1,...]
 * @param iq_pairs  Number of IQ pairs in input
 * @param audio_out int16 mono audio output buffer
 * @param audio_max Maximum number of audio samples that fit in audio_out
 * @return Number of audio samples produced (0 if not enough input)
 */
int fm_demod_process(fm_demod_t *demod, const int16_t *iq_in, int iq_pairs,
                     int16_t *audio_out, int audio_max);

/**
 * Run only the discriminator stage (noise blanker + FM discriminator).
 * Returns raw MPX output at sample_rate, pre-de-emphasis.
 * Use this when feeding the output to fm_stereo for stereo decoding.
 *
 * @param demod     Demodulator handle
 * @param iq_in     int16 interleaved IQ input [I0,Q0,I1,Q1,...]
 * @param iq_pairs  Number of IQ pairs in input
 * @param mpx_out   Raw discriminator output (int16, at sample_rate)
 * @param mpx_max   Maximum number of samples that fit in mpx_out
 * @return Number of MPX samples produced (== iq_pairs)
 */
int fm_demod_discriminate(fm_demod_t *demod, const int16_t *iq_in, int iq_pairs,
                           int16_t *mpx_out, int mpx_max);

/**
 * Apply volume control to audio buffer in-place.
 * @param audio     int16 audio samples
 * @param count     Number of samples
 * @param volume    Volume level 0-100
 */
void fm_demod_apply_volume(int16_t *audio, int count, int volume);

/**
 * Reset demodulator state (call on frequency change).
 */
void fm_demod_reset(fm_demod_t *demod);

/**
 * Reconfigure demodulator mode (WBFM/NBFM).
 * Reallocates FIR taps if cutoff frequency changes.
 */
esp_err_t fm_demod_set_mode(fm_demod_t *demod, fm_demod_mode_t mode);

/**
 * Get approximate signal strength from last processed block.
 * Returns RMS power in arbitrary units (0-32767).
 */
int16_t fm_demod_get_signal_strength(fm_demod_t *demod);

/**
 * Set squelch threshold (0=off, 1-100 range).
 * Audio is muted when signal strength falls below threshold.
 */
void fm_demod_set_squelch(fm_demod_t *demod, uint8_t threshold);

/**
 * Check if squelch is currently open (signal above threshold).
 */
bool fm_demod_squelch_open(fm_demod_t *demod);

/**
 * Configure noise blanker.
 * @param demod     Demodulator handle
 * @param enabled   Enable/disable noise blanker
 * @param threshold Blanking threshold (1-10, higher = less blanking)
 */
void fm_demod_set_noise_blanker(fm_demod_t *demod, bool enabled, uint8_t threshold);

#ifdef __cplusplus
}
#endif
