/*
 * FM Stereo MPX Decoder — Fixed-Point (Q15) Implementation
 *
 * Signal chain: MPX (256kSPS) -> Goertzel pilot detect -> PLL lock
 *               -> 38kHz ref -> L-R mix+LPF -> de-emphasis
 *               -> stereo matrix -> resample -> interleaved L/R (48kHz)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#ifdef CONFIG_FM_STEREO_ENABLE

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "fm_stereo.h"
#include "rds_decoder.h"
#include "pie_kernels.h"
#include "polyphase_resamp.h"

static const char *TAG = "fm_stereo";

/* ── Constants ── */

/* Goertzel for 19 kHz at 256 kSPS: k = round(1024 * 19000 / 256000) = 76
 * coeff = 2 * cos(2*pi*76/1024) in Q14 = 29128 */
#define GOERTZEL_N          1024
#define GOERTZEL_COEFF_Q14  29128

/* PLL nominal freq word: 19000 * 2^32 / 256000 = 316,669,952 */
#define PLL_NOMINAL_FREQ    316669952U
#define PLL_KP              200
#define PLL_KI              2
#define PLL_LUT_SIZE        256

/* Pilot detection threshold (empirical: power level indicating pilot present) */
#define PILOT_DETECT_THRESH     2000000LL
#define PILOT_DETECT_HYST       500000LL

/* PLL lock threshold (exponential average of |pd_i|, in Q8) */
#define PLL_LOCK_THRESH     500

/* Audio LPF: 31 taps for 15 kHz cutoff at 256 kSPS */
#define AUDIO_LPF_TAPS      31

/* ── Goertzel State ── */

typedef struct {
    int32_t     s1;
    int32_t     s2;
    int         count;
    bool        pilot_detected;
    int64_t     last_power;
} goertzel_state_t;

/* ── PLL State ── */

typedef struct {
    uint32_t    phase_acc;
    int32_t     freq_word;
    int32_t     integrator;
    int16_t     sin_lut[PLL_LUT_SIZE];
    int16_t     cos_lut[PLL_LUT_SIZE];
    int32_t     lock_i;
    bool        locked;
} stereo_pll_t;

/* ── De-emphasis IIR State ── */

typedef struct {
    int16_t     alpha;      /* Q15 */
    int32_t     state;      /* Q15 accumulator */
} deemph_state_t;

/* ── Main Stereo Decoder Struct ── */

struct fm_stereo {
    fm_stereo_config_t  config;

    /* Goertzel pilot detector */
    goertzel_state_t    goertzel;

    /* PLL */
    stereo_pll_t        pll;

    /* L+R path: LPF + de-emphasis */
    pie_fir_state_t    *fir_lpr;        /* L+R 15kHz LPF */
    deemph_state_t      deemph_lpr;     /* L+R de-emphasis */

    /* L-R path: LPF + de-emphasis */
    pie_fir_state_t    *fir_lmr;        /* L-R 15kHz LPF */
    deemph_state_t      deemph_lmr;     /* L-R de-emphasis */

    /* Polyphase resamplers: 256kSPS -> 48kHz */
    polyphase_resamp_t *resamp_l;
    polyphase_resamp_t *resamp_r;

    /* Scratch buffers */
    int16_t            *lpr_buf;        /* L+R filtered samples (block) */
    int16_t            *lmr_buf;        /* L-R demodulated+filtered samples (block) */
    int16_t            *left_buf;       /* Left channel pre-resample */
    int16_t            *right_buf;      /* Right channel pre-resample */
    int16_t            *left_rs;        /* Left channel post-resample */
    int16_t            *right_rs;       /* Right channel post-resample */
    int                 scratch_size;

    /* RDS decoder */
    rds_decoder_t      *rds;
    int16_t            *rds_buf;        /* 57kHz mixed RDS baseband */
    int                 rds_decim_phase; /* Decimation phase counter */

    /* Stereo blend */
    int16_t     blend_ratio;        /* 0=mono, 32767=full stereo (Q15) */
    int16_t     blend_threshold;    /* Signal strength for full mono */
    int16_t     blend_range;        /* Range above threshold for full stereo */
};

/* ── FIR Design Helper ── */

static void design_lpf_fir(int16_t *taps_q15, int ntaps, float cutoff_hz, float sample_rate)
{
    float fc = cutoff_hz / sample_rate;
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
            sinc = sinf(2.0f * (float)M_PI * fc * x) / ((float)M_PI * x);
        }
        /* Nuttall window */
        float w = 2.0f * (float)M_PI * n / (ntaps - 1);
        float win = 0.3635819f - 0.4891775f * cosf(w)
                  + 0.1365995f * cosf(2.0f * w)
                  - 0.0106411f * cosf(3.0f * w);
        ftaps[n] = sinc * win;
        sum += ftaps[n];
    }

    for (int n = 0; n < ntaps; n++) {
        float normalized = ftaps[n] / sum;
        int32_t q15 = (int32_t)(normalized * 32767.0f + 0.5f);
        if (q15 > 32767) q15 = 32767;
        if (q15 < -32768) q15 = -32768;
        taps_q15[n] = (int16_t)q15;
    }

    free(ftaps);
}

/* ── PLL LUT Init ── */

static void pll_init_lut(stereo_pll_t *pll)
{
    for (int i = 0; i < PLL_LUT_SIZE; i++) {
        double angle = 2.0 * M_PI * i / PLL_LUT_SIZE;
        pll->sin_lut[i] = (int16_t)(sin(angle) * 32767.0);
        pll->cos_lut[i] = (int16_t)(cos(angle) * 32767.0);
    }
    pll->phase_acc = 0;
    pll->freq_word = PLL_NOMINAL_FREQ;
    pll->integrator = 0;
    pll->lock_i = 0;
    pll->locked = false;
}

/* ── Goertzel Single-Sample Update ── */

static inline void goertzel_update(goertzel_state_t *g, int16_t sample)
{
    /* s0 = (coeff * s1 >> 14) - s2 + x */
    int32_t s0 = ((int64_t)GOERTZEL_COEFF_Q14 * g->s1 >> 14) - g->s2 + sample;
    g->s2 = g->s1;
    g->s1 = s0;
    g->count++;

    if (g->count >= GOERTZEL_N) {
        /* Compute power: s1*s1 + s2*s2 - coeff*s1*s2 >> 14 */
        int64_t power = (int64_t)g->s1 * g->s1
                      + (int64_t)g->s2 * g->s2
                      - ((int64_t)GOERTZEL_COEFF_Q14 * g->s1 >> 14) * g->s2;
        g->last_power = power;

        /* Hysteresis-based stereo detection */
        if (g->pilot_detected) {
            if (power < PILOT_DETECT_THRESH - PILOT_DETECT_HYST) {
                g->pilot_detected = false;
            }
        } else {
            if (power > PILOT_DETECT_THRESH + PILOT_DETECT_HYST) {
                g->pilot_detected = true;
            }
        }

        /* Reset for next block */
        g->s1 = 0;
        g->s2 = 0;
        g->count = 0;
    }
}

/* ── PLL Single-Sample Update ── */

static inline void pll_update(stereo_pll_t *pll, int16_t mpx_sample)
{
    /* NCO output from LUT */
    uint8_t idx = pll->phase_acc >> 24;
    int16_t nco_sin = pll->sin_lut[idx];
    int16_t nco_cos = pll->cos_lut[idx];

    /* Phase detector: multiply input by NCO quadrature outputs */
    int32_t pd_q = ((int32_t)mpx_sample * nco_sin) >> 15;
    int32_t pd_i = ((int32_t)mpx_sample * nco_cos) >> 15;

    /* PI loop filter */
    pll->integrator += PLL_KI * pd_q;
    int32_t correction = (PLL_KP * pd_q + pll->integrator) >> 8;

    /* Update NCO frequency and advance phase */
    pll->freq_word = (int32_t)PLL_NOMINAL_FREQ + correction;
    pll->phase_acc += (uint32_t)pll->freq_word;

    /* Lock detector: exponential average of |pd_i| */
    int32_t abs_pd_i = pd_i > 0 ? pd_i : -pd_i;
    pll->lock_i = pll->lock_i - (pll->lock_i >> 8) + abs_pd_i;
    pll->locked = (pll->lock_i >> 8) > PLL_LOCK_THRESH;
}

/* ── De-emphasis Single-Sample ── */

static inline int16_t deemph_process(deemph_state_t *d, int16_t input)
{
    /* y = alpha*x + (1-alpha)*y_prev */
    d->state = (int32_t)d->alpha * input + (32767 - d->alpha) * (d->state >> 15);
    return (int16_t)(d->state >> 15);
}

/* ── Saturation Helper ── */

static inline int16_t sat16(int32_t x)
{
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

/* ── Create / Free ── */

fm_stereo_t *fm_stereo_create(const fm_stereo_config_t *config)
{
    fm_stereo_t *st = calloc(1, sizeof(fm_stereo_t));
    if (!st) return NULL;

    st->config = *config;

    /* Init PLL sin/cos LUT */
    pll_init_lut(&st->pll);

    /* Init Goertzel */
    memset(&st->goertzel, 0, sizeof(st->goertzel));

    /* De-emphasis: alpha = 1 / (tau * fs + 1) in Q15 */
    float alpha = 1.0f / (config->de_emphasis_tau * config->sample_rate + 1.0f);
    int16_t alpha_q15 = (int16_t)(alpha * 32767.0f);
    st->deemph_lpr.alpha = alpha_q15;
    st->deemph_lpr.state = 0;
    st->deemph_lmr.alpha = alpha_q15;
    st->deemph_lmr.state = 0;

    /* Design 15 kHz audio LPF (31 taps, Nuttall window) */
    int16_t *tmp_taps = malloc(AUDIO_LPF_TAPS * sizeof(int16_t));
    if (!tmp_taps) {
        ESP_LOGE(TAG, "LPF taps allocation failed");
        fm_stereo_free(st);
        return NULL;
    }
    design_lpf_fir(tmp_taps, AUDIO_LPF_TAPS, 15000.0f, (float)config->sample_rate);

    /* Create two FIR filter instances: one for L+R, one for L-R */
    st->fir_lpr = pie_fir_create(tmp_taps, AUDIO_LPF_TAPS);
    st->fir_lmr = pie_fir_create(tmp_taps, AUDIO_LPF_TAPS);
    free(tmp_taps);

    if (!st->fir_lpr || !st->fir_lmr) {
        ESP_LOGE(TAG, "FIR filter creation failed");
        fm_stereo_free(st);
        return NULL;
    }

    /* Create polyphase resamplers: sample_rate -> audio_rate */
    st->resamp_l = polyphase_resamp_create(config->sample_rate, config->audio_rate, 16);
    st->resamp_r = polyphase_resamp_create(config->sample_rate, config->audio_rate, 16);
    if (!st->resamp_l || !st->resamp_r) {
        ESP_LOGE(TAG, "Resampler creation failed");
        fm_stereo_free(st);
        return NULL;
    }

    /* Scratch buffers — sized for max block of 4096 input samples */
    st->scratch_size = 4096;
    st->lpr_buf   = heap_caps_aligned_alloc(16, st->scratch_size * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    st->lmr_buf   = heap_caps_aligned_alloc(16, st->scratch_size * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    st->left_buf  = heap_caps_aligned_alloc(16, st->scratch_size * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    st->right_buf = heap_caps_aligned_alloc(16, st->scratch_size * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    /* Resample output: max output = input * (audio_rate / sample_rate) + margin */
    int rs_max = (st->scratch_size * config->audio_rate / config->sample_rate) + 64;
    st->left_rs   = heap_caps_aligned_alloc(16, rs_max * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    st->right_rs  = heap_caps_aligned_alloc(16, rs_max * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    if (!st->lpr_buf || !st->lmr_buf || !st->left_buf || !st->right_buf ||
        !st->left_rs || !st->right_rs) {
        ESP_LOGE(TAG, "Scratch buffer allocation failed");
        fm_stereo_free(st);
        return NULL;
    }

    /* RDS decoder: 57kHz subcarrier mixed down, decimated to ~5kHz
     * Decimation ratio: sample_rate / 5333 ≈ 48 for 256kSPS */
    {
        uint32_t rds_rate = 5333;
        st->rds = rds_decoder_create(rds_rate);
        if (st->rds) {
            st->rds_buf = heap_caps_aligned_alloc(16, st->scratch_size * sizeof(int16_t), MALLOC_CAP_DEFAULT);
            if (!st->rds_buf) {
                ESP_LOGW(TAG, "RDS buffer alloc failed, RDS disabled");
                rds_decoder_free(st->rds);
                st->rds = NULL;
            } else {
                ESP_LOGI(TAG, "RDS decoder attached (decim %lu->%lu)",
                         (unsigned long)config->sample_rate, (unsigned long)rds_rate);
            }
        } else {
            ESP_LOGW(TAG, "RDS decoder creation failed, continuing without RDS");
        }
        st->rds_decim_phase = 0;
    }

    /* Init stereo blend */
    st->blend_threshold = CONFIG_FM_STEREO_BLEND_THRESHOLD * 327; /* Scale 0-100 to 0-32700 */
    st->blend_range = 10 * 327; /* 10 units of hysteresis */
    st->blend_ratio = 0;

    ESP_LOGI(TAG, "FM stereo decoder created: %lu->%lu Hz, de-emph alpha=%d",
             (unsigned long)config->sample_rate, (unsigned long)config->audio_rate,
             alpha_q15);

    return st;
}

void fm_stereo_free(fm_stereo_t *st)
{
    if (!st) return;
    pie_fir_free(st->fir_lpr);
    pie_fir_free(st->fir_lmr);
    polyphase_resamp_free(st->resamp_l);
    polyphase_resamp_free(st->resamp_r);
    heap_caps_free(st->lpr_buf);
    heap_caps_free(st->lmr_buf);
    heap_caps_free(st->left_buf);
    heap_caps_free(st->right_buf);
    heap_caps_free(st->left_rs);
    heap_caps_free(st->right_rs);
    rds_decoder_free(st->rds);
    heap_caps_free(st->rds_buf);
    free(st);
}

void fm_stereo_reset(fm_stereo_t *st)
{
    if (!st) return;
    memset(&st->goertzel, 0, sizeof(st->goertzel));
    pll_init_lut(&st->pll);
    st->deemph_lpr.state = 0;
    st->deemph_lmr.state = 0;
    pie_fir_reset(st->fir_lpr);
    pie_fir_reset(st->fir_lmr);
    polyphase_resamp_reset(st->resamp_l);
    polyphase_resamp_reset(st->resamp_r);
}

/* ── Main Processing ── */

int fm_stereo_process(fm_stereo_t *st, const int16_t *mpx_in, int n_samples,
                       int16_t *stereo_out, int out_max)
{
    if (!st || !mpx_in || !stereo_out || n_samples <= 0) return 0;

    /* Clamp to scratch size */
    if (n_samples > st->scratch_size) {
        n_samples = st->scratch_size;
    }

    /* Per-sample processing: PLL update, Goertzel, L+R/L-R extraction */
    for (int i = 0; i < n_samples; i++) {
        int16_t mpx = mpx_in[i];

        /* Update Goertzel pilot detector */
        goertzel_update(&st->goertzel, mpx);

        /* Update PLL (always runs, even if pilot not yet detected) */
        pll_update(&st->pll, mpx);

        /* L+R: the MPX signal below 15 kHz IS the L+R signal.
         * We just need to LPF it (done below via FIR).
         * Store raw MPX for L+R LPF input. */
        st->lpr_buf[i] = mpx;

        /* L-R: multiply MPX by 38 kHz reference from PLL (2x pilot phase) */
        uint32_t phase_38k = st->pll.phase_acc << 1;  /* 2x frequency */
        uint8_t idx_38 = phase_38k >> 24;
        int16_t ref_38k = st->pll.sin_lut[idx_38];
        /* Demodulate L-R: mix and scale by 2 (DSB-SC produces half amplitude) */
        int32_t mixed = ((int32_t)mpx * ref_38k) >> 14;  /* >> 15 for Q15 multiply, << 1 for 2x gain */
        st->lmr_buf[i] = sat16(mixed);
    }

    /* RDS: mix MPX with 57kHz reference (3x pilot), decimate to ~5kHz, feed decoder */
    if (st->rds && st->rds_buf) {
        /* Decimation ratio: sample_rate / 5333 ≈ 48 for 256kSPS */
        int rds_decim = st->config.sample_rate / 5333;
        if (rds_decim < 1) rds_decim = 1;
        int rds_count = 0;

        for (int i = 0; i < n_samples; i++) {
            /* 57kHz = 3x 19kHz pilot phase */
            uint32_t phase_57k = st->pll.phase_acc * 3;
            uint8_t idx_57 = phase_57k >> 24;
            int16_t ref_57k = st->pll.sin_lut[idx_57];

            /* Mix MPX with 57kHz reference */
            int32_t rds_mixed = ((int32_t)mpx_in[i] * ref_57k) >> 15;

            /* Decimate */
            st->rds_decim_phase++;
            if (st->rds_decim_phase >= rds_decim) {
                st->rds_decim_phase = 0;
                st->rds_buf[rds_count++] = sat16(rds_mixed);
            }
        }

        if (rds_count > 0) {
            rds_decoder_process(st->rds, st->rds_buf, rds_count);
        }
    }

    /* Apply 15 kHz LPF to both L+R and L-R paths */
    int16_t lpr_filtered[n_samples];
    int16_t lmr_filtered[n_samples];
    pie_fir_process(st->fir_lpr, st->lpr_buf, lpr_filtered, n_samples);
    pie_fir_process(st->fir_lmr, st->lmr_buf, lmr_filtered, n_samples);

    /* De-emphasis + stereo matrix */
    bool is_stereo = st->goertzel.pilot_detected && st->pll.locked;

    /* Update blend ratio based on pilot power as signal strength proxy */
    {
        int16_t pilot_mag = (int16_t)(st->goertzel.last_power >> 16);
        if (st->blend_threshold == 0) {
            st->blend_ratio = 32767; /* Always full stereo */
        } else if (!is_stereo) {
            st->blend_ratio = 0;
        } else if (pilot_mag > st->blend_threshold + st->blend_range) {
            st->blend_ratio = 32767;
        } else if (pilot_mag < st->blend_threshold) {
            st->blend_ratio = 0;
        } else {
            /* Linear interpolation in blend range */
            st->blend_ratio = (int16_t)(((int32_t)(pilot_mag - st->blend_threshold) * 32767)
                                        / st->blend_range);
        }
    }

    for (int i = 0; i < n_samples; i++) {
        int16_t lpr_de = deemph_process(&st->deemph_lpr, lpr_filtered[i]);
        int16_t lmr_de = deemph_process(&st->deemph_lmr, lmr_filtered[i]);

        /* Stereo matrix: L = (L+R) + (L-R), R = (L+R) - (L-R) */
        int16_t L = sat16(((int32_t)lpr_de + lmr_de) >> 1);
        int16_t R = sat16(((int32_t)lpr_de - lmr_de) >> 1);
        int16_t mono = lpr_de;

        /* Apply blend: out = mono*(1-blend) + stereo*blend */
        st->left_buf[i]  = (int16_t)(((int32_t)mono * (32767 - st->blend_ratio)
                                      + (int32_t)L * st->blend_ratio) >> 15);
        st->right_buf[i] = (int16_t)(((int32_t)mono * (32767 - st->blend_ratio)
                                      + (int32_t)R * st->blend_ratio) >> 15);
    }

    /* Resample both channels from sample_rate to audio_rate */
    int rs_max = (n_samples * st->config.audio_rate / st->config.sample_rate) + 64;
    if (rs_max > out_max) rs_max = out_max;

    int n_left  = polyphase_resamp_process(st->resamp_l, st->left_buf, n_samples,
                                            st->left_rs, rs_max);
    int n_right = polyphase_resamp_process(st->resamp_r, st->right_buf, n_samples,
                                            st->right_rs, rs_max);

    /* Interleave L/R into output */
    int n_out = n_left < n_right ? n_left : n_right;
    if (n_out > out_max) n_out = out_max;

    for (int i = 0; i < n_out; i++) {
        stereo_out[i * 2]     = st->left_rs[i];
        stereo_out[i * 2 + 1] = st->right_rs[i];
    }

    return n_out;
}

bool fm_stereo_is_locked(fm_stereo_t *st)
{
    return st ? st->pll.locked : false;
}

bool fm_stereo_is_stereo(fm_stereo_t *st)
{
    return st ? (st->goertzel.pilot_detected && st->pll.locked) : false;
}

void fm_stereo_get_rds(fm_stereo_t *st, rds_data_t *out)
{
    if (!st || !out) return;
    if (st->rds) {
        rds_decoder_get_data(st->rds, out);
    } else {
        memset(out, 0, sizeof(*out));
    }
}

int fm_stereo_get_blend_ratio(fm_stereo_t *st)
{
    if (!st) return 0;
    /* Convert Q15 (0-32767) to percentage (0-100) */
    return (int)((int32_t)st->blend_ratio * 100 / 32767);
}

#endif /* CONFIG_FM_STEREO_ENABLE */
