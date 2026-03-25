/*
 * FM Stereo MPX Decoder
 *
 * Decodes stereo from the FM MPX baseband signal using:
 * - Goertzel detector for 19 kHz pilot presence
 * - Second-order PLL for pilot phase tracking
 * - 38 kHz reference generation (2x pilot)
 * - L-R extraction via mixing + LPF
 * - L/R matrix: L=(L+R)+(L-R), R=(L+R)-(L-R)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include "sdkconfig.h"

#ifdef CONFIG_FM_STEREO_ENABLE

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fm_stereo fm_stereo_t;

typedef struct {
    uint32_t    sample_rate;    /* MPX input rate (typically 256000) */
    uint32_t    audio_rate;     /* Output audio rate (48000) */
    float       de_emphasis_tau; /* De-emphasis time constant */
} fm_stereo_config_t;

#define FM_STEREO_CONFIG_DEFAULT() { \
    .sample_rate = 256000, \
    .audio_rate = 48000, \
    .de_emphasis_tau = 75e-6f, \
}

fm_stereo_t *fm_stereo_create(const fm_stereo_config_t *config);
void fm_stereo_free(fm_stereo_t *st);
void fm_stereo_reset(fm_stereo_t *st);

/**
 * Process MPX samples (raw discriminator output, pre-de-emphasis).
 * Outputs interleaved stereo [L0,R0,L1,R1,...] at audio_rate.
 *
 * @param st        Stereo decoder handle
 * @param mpx_in    Raw discriminator output int16 at sample_rate
 * @param n_samples Number of MPX samples
 * @param stereo_out Interleaved L/R int16 output buffer
 * @param out_max   Max output PAIRS (each pair = L+R = 2 int16)
 * @return Number of output stereo pairs produced
 */
int fm_stereo_process(fm_stereo_t *st, const int16_t *mpx_in, int n_samples,
                       int16_t *stereo_out, int out_max);

bool fm_stereo_is_locked(fm_stereo_t *st);
bool fm_stereo_is_stereo(fm_stereo_t *st);

#include "rds_decoder.h"

/**
 * Get current decoded RDS data from the stereo decoder's RDS sub-decoder.
 * @param st   Stereo decoder handle
 * @param out  Receives a snapshot of decoded RDS data
 */
void fm_stereo_get_rds(fm_stereo_t *st, rds_data_t *out);

#ifdef __cplusplus
}
#endif
#endif /* CONFIG_FM_STEREO_ENABLE */
