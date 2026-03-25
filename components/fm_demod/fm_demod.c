/*
 * FM Demodulator — Fixed-Point (Q15) Implementation
 *
 * Signal chain: IQ → discriminator → de-emphasis → LPF → resample → audio
 *
 * The discriminator uses a fast atan2 approximation based on:
 *   cross = I[n]*Q[n-1] - Q[n]*I[n-1]
 *   dot   = I[n]*I[n-1] + Q[n]*Q[n-1]
 *   angle ≈ cross / (|cross| + |dot|)   (scaled to [-1,1])
 *
 * This avoids trig entirely and produces <1% error for FM audio.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "fm_demod.h"
#include "pie_fm_discriminator.h"
#include "pie_kernels.h"

static const char *TAG = "fm_demod";

/* ── FIR Filter Taps ── */

#define FIR_MAX_TAPS    255     /* Max FIR taps (odd, for symmetric filter) */
#define RESAMP_FRAC_BITS 16     /* Fractional resampler precision */

typedef enum {
    FIR_WINDOW_HAMMING  = 0,
    FIR_WINDOW_BLACKMAN = 1,
    FIR_WINDOW_NUTTALL  = 2,
} fir_window_t;

static float window_func(int n, int N, fir_window_t type)
{
    float x = 2.0f * M_PI * n / (N - 1);
    switch (type) {
    case FIR_WINDOW_HAMMING:
        return 0.54f - 0.46f * cosf(x);
    case FIR_WINDOW_BLACKMAN:
        return 0.42f - 0.5f * cosf(x) + 0.08f * cosf(2.0f * x);
    case FIR_WINDOW_NUTTALL:
        return 0.3635819f - 0.4891775f * cosf(x)
             + 0.1365995f * cosf(2.0f * x)
             - 0.0106411f * cosf(3.0f * x);
    default:
        return 0.54f - 0.46f * cosf(x); /* Hamming fallback */
    }
}

struct fm_demod {
    fm_demod_config_t   config;

    /* Discriminator state (PIE SIMD) */
    fm_disc_state_t     disc_state;

    /* De-emphasis IIR state (Q15 fixed-point) */
    int16_t             de_alpha;       /* Q15: alpha = 1/(tau*fs+1) */
    int32_t             de_state;       /* Q15 accumulator */

    /* Audio LPF (FIR via PIE SIMD) */
    pie_fir_state_t    *fir;
    int                 fir_len;        /* Actual tap count (for redesign) */

    /* Fractional resampler */
    uint32_t            resamp_inc;     /* Fixed-point increment (input/output ratio) */
    uint32_t            resamp_phase;   /* Current phase accumulator */

    /* Signal strength */
    int64_t             rms_accum;
    int                 rms_count;
    int16_t             signal_strength;

    /* Scratch buffer for discriminator output (before LPF) */
    int16_t            *disc_buf;
    int                 disc_buf_size;

    /* Squelch */
    uint8_t     squelch_threshold;  /* 0=off, 1-100 */
    bool        squelch_open;       /* Current gate state */
    int16_t     squelch_open_level; /* Open at this level */
    int16_t     squelch_close_level; /* Close at this level (hysteresis) */

    /* Noise blanker */
    bool        nb_enabled;
    uint8_t     nb_threshold;       /* 1-10 (multiplier for average magnitude) */
    int32_t     nb_avg_mag;         /* Running average magnitude (Q15 << 8 for smoothing) */
};

/* ── Windowed sinc FIR design ── */

static void design_lpf_fir(int16_t *taps_q15, int ntaps, float cutoff_hz,
                            float sample_rate, fir_window_t window_type)
{
    float fc = cutoff_hz / sample_rate;     /* Normalized cutoff */
    int M = ntaps - 1;
    float sum = 0.0f;
    float *ftaps = malloc(ntaps * sizeof(float));
    if (!ftaps) return;

    for (int n = 0; n < ntaps; n++) {
        float x = n - M / 2.0f;
        float sinc;
        if (fabsf(x) < 1e-6f) {
            sinc = 2.0f * fc;
        } else {
            sinc = sinf(2.0f * M_PI * fc * x) / (M_PI * x);
        }
        ftaps[n] = sinc * window_func(n, ntaps, window_type);
        sum += ftaps[n];
    }

    /* Normalize and convert to Q15 */
    for (int n = 0; n < ntaps; n++) {
        float normalized = ftaps[n] / sum;
        int32_t q15 = (int32_t)(normalized * 32767.0f + 0.5f);
        if (q15 > 32767) q15 = 32767;
        if (q15 < -32768) q15 = -32768;
        taps_q15[n] = (int16_t)q15;
    }

    free(ftaps);
}

/* ── Create / Free ── */

fm_demod_t *fm_demod_create(const fm_demod_config_t *config)
{
    fm_demod_t *d = calloc(1, sizeof(fm_demod_t));
    if (!d) return NULL;

    d->config = *config;

    /* FIR tap count from Kconfig (configurable, clamped to valid range) */
    d->fir_len = CONFIG_FM_FIR_TAPS;
    if (d->fir_len > FIR_MAX_TAPS) d->fir_len = FIR_MAX_TAPS;
    /* Ensure odd */
    if ((d->fir_len & 1) == 0) d->fir_len++;

    /* Design the audio LPF taps into a temporary buffer, then create PIE FIR */
    {
        int16_t *tmp_taps = malloc(d->fir_len * sizeof(int16_t));
        if (!tmp_taps) {
            ESP_LOGE(TAG, "FIR temp taps allocation failed");
            fm_demod_free(d);
            return NULL;
        }
        fir_window_t win = (fir_window_t)CONFIG_FM_FIR_WINDOW;
        design_lpf_fir(tmp_taps, d->fir_len, config->audio_lpf_cutoff, config->sample_rate, win);
        d->fir = pie_fir_create(tmp_taps, d->fir_len);
        free(tmp_taps);
        if (!d->fir) {
            ESP_LOGE(TAG, "FIR filter creation failed");
            fm_demod_free(d);
            return NULL;
        }
    }

    /* De-emphasis IIR: alpha = 1 / (tau * fs + 1) in Q15 */
    float alpha = 1.0f / (config->de_emphasis_tau * config->sample_rate + 1.0f);
    d->de_alpha = (int16_t)(alpha * 32767.0f);
    d->de_state = 0;

    /* Fractional resampler: increment = input_rate / output_rate in fixed-point */
    d->resamp_inc = (uint32_t)(((uint64_t)config->sample_rate << RESAMP_FRAC_BITS) / config->audio_rate);
    d->resamp_phase = 0;

    /* Initialize PIE FM discriminator */
    fm_disc_init(&d->disc_state, FM_DISC_POLY_CORRECTED,
                 config->sample_rate, config->deviation);

    /* Scratch buffer — sized for 3x: IQ copy (2x) + discriminator output (1x) */
    d->disc_buf_size = 4096;
    d->disc_buf = malloc(d->disc_buf_size * 3 * sizeof(int16_t));
    if (!d->disc_buf) {
        ESP_LOGE(TAG, "Discriminator buffer allocation failed");
        fm_demod_free(d);
        return NULL;
    }

    /* Noise blanker */
#ifdef CONFIG_FM_NOISE_BLANKER
    d->nb_enabled   = true;
    d->nb_threshold = CONFIG_FM_NOISE_BLANKER_THRESHOLD;
#else
    d->nb_enabled   = false;
    d->nb_threshold = 5;
#endif
    d->nb_avg_mag = 0;

    ESP_LOGI(TAG, "FM demod created: mode=%s rate=%lu→%lu dev=%luHz lpf=%luHz fir=%d taps",
             config->mode == FM_DEMOD_WBFM ? "WBFM" : "NBFM",
             (unsigned long)config->sample_rate, (unsigned long)config->audio_rate,
             (unsigned long)config->deviation, (unsigned long)config->audio_lpf_cutoff,
             d->fir_len);
    ESP_LOGI(TAG, "  de-emph alpha=%d/32767 resamp_inc=0x%08lx",
             d->de_alpha, (unsigned long)d->resamp_inc);

    return d;
}

void fm_demod_free(fm_demod_t *d)
{
    if (!d) return;
    pie_fir_free(d->fir);
    free(d->disc_buf);
    free(d);
}

void fm_demod_reset(fm_demod_t *d)
{
    if (!d) return;
    fm_disc_reset(&d->disc_state);
    d->de_state = 0;
    d->resamp_phase = 0;
    d->rms_accum = 0;
    d->rms_count = 0;
    pie_fir_reset(d->fir);
}

/* ── Noise Blanker ── */

/**
 * Noise blanker: detect and blank impulse noise in IQ domain.
 * Algorithm:
 * 1. Compute instantaneous magnitude: mag = |I| + |Q| (fast approximation)
 * 2. Update running average: avg = avg * 0.999 + mag * 0.001
 * 3. If mag > threshold * avg: blank sample (replace with previous good sample)
 * 4. Use linear interpolation for blanked regions (look-ahead 1 sample)
 */
static void noise_blanker_process(fm_demod_t *d, int16_t *iq, int n_pairs)
{
    if (!d->nb_enabled) return;

    int32_t avg = d->nb_avg_mag;
    int32_t thresh_mult = (int32_t)d->nb_threshold * 4;  /* 1→4x, 5→20x, 10→40x */
    int16_t prev_i = iq[0], prev_q = iq[1];

    for (int k = 0; k < n_pairs; k++) {
        int16_t si = iq[k * 2];
        int16_t sq = iq[k * 2 + 1];

        /* Fast magnitude: |I| + |Q| (within 1 dB of true magnitude for FM) */
        int32_t mag = (si < 0 ? -si : si) + (sq < 0 ? -sq : sq);

        /* Update running average (exponential, alpha ~= 0.001 → shift by 10) */
        avg = avg - (avg >> 10) + (mag >> 10);

        /* Check if spike exceeds threshold */
        if (mag > (avg >> 8) * thresh_mult) {
            /* Blank: replace with previous good sample */
            iq[k * 2]     = prev_i;
            iq[k * 2 + 1] = prev_q;
        } else {
            prev_i = si;
            prev_q = sq;
        }
    }

    d->nb_avg_mag = avg;
}

/* ── Main processing ── */

int fm_demod_process(fm_demod_t *d, const int16_t *iq_in, int iq_pairs,
                     int16_t *audio_out, int audio_max)
{
    if (!d || !iq_in || !audio_out || iq_pairs <= 0) return 0;

    int audio_pos = 0;

    /* Process in chunks that fit in disc_buf */
    int offset = 0;
    while (offset < iq_pairs && audio_pos < audio_max) {
        int chunk = iq_pairs - offset;
        if (chunk > d->disc_buf_size) chunk = d->disc_buf_size;

        /* Stage 0: Copy IQ to scratch for noise blanker (avoid const-cast).
         * Layout: disc_buf[0..chunk*2-1] = blanked IQ, disc_buf[chunk*2..] = disc output */
        int16_t *blanked_iq = d->disc_buf;
        int16_t *disc_out   = d->disc_buf + chunk * 2;
        memcpy(blanked_iq, &iq_in[offset * 2], chunk * 2 * sizeof(int16_t));

        /* Stage 0b: Noise blanker — operates in-place on copied IQ */
        noise_blanker_process(d, blanked_iq, chunk);

        /* Stage 1: FM discriminator (PIE SIMD) */
        fm_disc_process(&d->disc_state, blanked_iq, disc_out, chunk);

        /* RMS accumulator for signal strength */
        for (int k = 0; k < chunk; k++) {
            int16_t i_cur = blanked_iq[k * 2];
            int16_t q_cur = blanked_iq[k * 2 + 1];
            d->rms_accum += (int32_t)i_cur * i_cur + (int32_t)q_cur * q_cur;
            d->rms_count++;
        }

        /* Stage 2: De-emphasis IIR + Stage 3: FIR LPF + Stage 4: Fractional resample */
        for (int k = 0; k < chunk && audio_pos < audio_max; k++) {
            /* De-emphasis: y = alpha*x + (1-alpha)*y_prev */
            int32_t de_in = disc_out[k];
            d->de_state = (d->de_alpha * de_in + (32767 - d->de_alpha) * (d->de_state >> 15));
            int16_t de_out = (int16_t)(d->de_state >> 15);

            /* FIR low-pass filter (PIE SIMD accelerated) */
            int16_t filtered;
            pie_fir_process(d->fir, &de_out, &filtered, 1);

            /* Fractional resampler: output sample when phase accumulator wraps */
            d->resamp_phase += (1 << RESAMP_FRAC_BITS); /* Advance by 1 input sample */

            while (d->resamp_phase >= d->resamp_inc && audio_pos < audio_max) {
                d->resamp_phase -= d->resamp_inc;
                audio_out[audio_pos++] = filtered;
            }
        }

        offset += chunk;
    }

    /* Update signal strength periodically */
    if (d->rms_count >= 1024) {
        int64_t avg = d->rms_accum / d->rms_count;
        /* Approximate sqrt via bit shifting (good enough for signal meter) */
        int16_t rms = 0;
        if (avg > 0) {
            int shift = 0;
            int64_t tmp = avg;
            while (tmp > 0x7FFFFFFF) { tmp >>= 2; shift++; }
            /* Integer sqrt approximation */
            uint32_t v = (uint32_t)tmp;
            uint32_t r = 0, bit = 1u << 30;
            while (bit > v) bit >>= 2;
            while (bit != 0) {
                if (v >= r + bit) {
                    v -= r + bit;
                    r = (r >> 1) + bit;
                } else {
                    r >>= 1;
                }
                bit >>= 2;
            }
            rms = (int16_t)(r << shift);
        }
        d->signal_strength = rms;
        d->rms_accum = 0;
        d->rms_count = 0;
    }

    return audio_pos;
}

/* ── Discriminator-only processing ── */

int fm_demod_discriminate(fm_demod_t *d, const int16_t *iq_in, int iq_pairs,
                           int16_t *mpx_out, int mpx_max)
{
    if (!d || !iq_in || !mpx_out || iq_pairs <= 0) return 0;

    int total = 0;
    int offset = 0;

    while (offset < iq_pairs && total < mpx_max) {
        int chunk = iq_pairs - offset;
        if (chunk > d->disc_buf_size) chunk = d->disc_buf_size;
        if (chunk > mpx_max - total) chunk = mpx_max - total;

        /* Stage 0: Copy IQ to disc_buf for noise blanker (avoid const-cast) */
        int16_t *blanked_iq = d->disc_buf;
        memcpy(blanked_iq, &iq_in[offset * 2], chunk * 2 * sizeof(int16_t));

        /* Stage 0b: Noise blanker — operates in-place on copied IQ */
        noise_blanker_process(d, blanked_iq, chunk);

        /* Stage 1: FM discriminator (PIE SIMD) */
        fm_disc_process(&d->disc_state, blanked_iq, &mpx_out[total], chunk);

        /* RMS accumulator for signal strength */
        for (int k = 0; k < chunk; k++) {
            int16_t i_cur = blanked_iq[k * 2];
            int16_t q_cur = blanked_iq[k * 2 + 1];
            d->rms_accum += (int32_t)i_cur * i_cur + (int32_t)q_cur * q_cur;
            d->rms_count++;
        }

        /* Update signal strength periodically */
        if (d->rms_count >= 1024) {
            int64_t avg = d->rms_accum / d->rms_count;
            int16_t rms = 0;
            if (avg > 0) {
                int shift = 0;
                int64_t tmp = avg;
                while (tmp > 0x7FFFFFFF) { tmp >>= 2; shift++; }
                uint32_t v = (uint32_t)tmp;
                uint32_t r = 0, bit = 1u << 30;
                while (bit > v) bit >>= 2;
                while (bit != 0) {
                    if (v >= r + bit) {
                        v -= r + bit;
                        r = (r >> 1) + bit;
                    } else {
                        r >>= 1;
                    }
                    bit >>= 2;
                }
                rms = (int16_t)(r << shift);
            }
            d->signal_strength = rms;
            d->rms_accum = 0;
            d->rms_count = 0;
        }

        total += chunk;
        offset += chunk;
    }

    return total;
}

/* ── Volume control ── */

void fm_demod_apply_volume(int16_t *audio, int count, int volume)
{
    if (volume >= 100) return; /* No attenuation needed */
    if (volume <= 0) {
        memset(audio, 0, count * sizeof(int16_t));
        return;
    }

    /* Volume as Q15: 0→0, 50→16384, 100→32767 */
    int16_t vol_q15 = (int16_t)((int32_t)volume * 32767 / 100);

    for (int i = 0; i < count; i++) {
        audio[i] = (int16_t)(((int32_t)audio[i] * vol_q15) >> 15);
    }
}

/* ── Mode change ── */

esp_err_t fm_demod_set_mode(fm_demod_t *d, fm_demod_mode_t mode)
{
    if (!d) return ESP_ERR_INVALID_ARG;

    if (mode == FM_DEMOD_WBFM) {
        d->config.mode = FM_DEMOD_WBFM;
        d->config.deviation = 75000;
        d->config.audio_lpf_cutoff = 15000;
    } else {
        d->config.mode = FM_DEMOD_NBFM;
        d->config.deviation = 5000;
        d->config.audio_lpf_cutoff = 3000;
    }
    /* fir_len stays at the value set during create (CONFIG_FM_FIR_TAPS) */

    /* Redesign FIR: create new taps and rebuild pie_fir_state */
    {
        pie_fir_state_t *old_fir = d->fir;
        int16_t *tmp_taps = malloc(d->fir_len * sizeof(int16_t));
        if (tmp_taps) {
            design_lpf_fir(tmp_taps, d->fir_len, d->config.audio_lpf_cutoff,
                           d->config.sample_rate, (fir_window_t)CONFIG_FM_FIR_WINDOW);
            d->fir = pie_fir_create(tmp_taps, d->fir_len);
            free(tmp_taps);
            if (d->fir) {
                pie_fir_free(old_fir);
            } else {
                d->fir = old_fir; /* Restore on failure */
                ESP_LOGE(TAG, "FIR redesign failed, keeping old filter");
            }
        }
    }

    /* Reconfigure discriminator */
    fm_disc_init(&d->disc_state, FM_DISC_POLY_CORRECTED,
                 d->config.sample_rate, d->config.deviation);

    fm_demod_reset(d);

    ESP_LOGI(TAG, "Mode changed to %s: dev=%luHz lpf=%luHz fir=%d",
             mode == FM_DEMOD_WBFM ? "WBFM" : "NBFM",
             (unsigned long)d->config.deviation,
             (unsigned long)d->config.audio_lpf_cutoff,
             d->fir_len);

    return ESP_OK;
}

int16_t fm_demod_get_signal_strength(fm_demod_t *d)
{
    return d ? d->signal_strength : 0;
}

void fm_demod_set_noise_blanker(fm_demod_t *d, bool enabled, uint8_t threshold)
{
    if (!d) return;
    d->nb_enabled   = enabled;
    d->nb_threshold = (threshold < 1) ? 1 : (threshold > 10 ? 10 : threshold);
    ESP_LOGI(TAG, "Noise blanker %s (threshold=%d)", enabled ? "ON" : "OFF", d->nb_threshold);
}

void fm_demod_set_squelch(fm_demod_t *d, uint8_t threshold)
{
    if (!d) return;
    d->squelch_threshold = threshold;
    if (threshold == 0) {
        d->squelch_open = true;
        d->squelch_open_level = 0;
        d->squelch_close_level = 0;
    } else {
        d->squelch_open_level  = (int16_t)((int32_t)threshold * 327);
        d->squelch_close_level = (int16_t)((int32_t)(threshold > 10 ? threshold - 10 : 0) * 327);
    }
    ESP_LOGI(TAG, "Squelch %s (threshold=%d, open=%d, close=%d)",
             threshold ? "ON" : "OFF", threshold,
             d->squelch_open_level, d->squelch_close_level);
}

bool fm_demod_squelch_open(fm_demod_t *d)
{
    if (!d) return true;
    if (d->squelch_threshold == 0) return true;

    if (d->squelch_open) {
        if (d->signal_strength < d->squelch_close_level) {
            d->squelch_open = false;
        }
    } else {
        if (d->signal_strength > d->squelch_open_level) {
            d->squelch_open = true;
        }
    }
    return d->squelch_open;
}
