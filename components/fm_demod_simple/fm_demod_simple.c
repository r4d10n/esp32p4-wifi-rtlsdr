/*
 * WBFM Demodulator with Stereo — Plain C, Floating Point
 *
 * Pipeline:
 *   U8 IQ @ 256kSPS → atan2f discriminator → MPX baseband
 *   MPX → Goertzel 19kHz pilot detect → PLL lock
 *   MPX → 15kHz LPF → de-emphasis → L+R (mono)
 *   MPX × 38kHz ref → 15kHz LPF → de-emphasis → L-R
 *   L = (L+R + L-R) / 2,  R = (L+R - L-R) / 2
 *   Resample 256k→48k → int16 stereo [L,R,L,R,...]
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fm_demod_simple.h"
#include "rds_decoder.h"

/* ── Configuration ── */

#define FIR_TAPS        31
#define GOERTZEL_N      1024
#define PLL_LUT_SIZE    256
#define DEEMPH_TAU_US   75      /* 75us NA/Japan, 50us Europe */
#define FM_DEVIATION    75000.0f

/* ── FIR LPF Design ── */

static void design_lpf(float *taps, int ntaps, float cutoff_hz, float sample_rate)
{
    float fc = cutoff_hz / sample_rate;
    int mid = ntaps / 2;
    float sum = 0.0f;

    for (int i = 0; i < ntaps; i++) {
        int n = i - mid;
        float sinc = (n == 0) ? 2.0f * fc
                     : sinf(2.0f * (float)M_PI * fc * (float)n) / ((float)M_PI * (float)n);
        /* Nuttall window */
        float w = 2.0f * (float)M_PI * (float)i / (float)(ntaps - 1);
        float win = 0.355768f - 0.487396f * cosf(w) + 0.144232f * cosf(2.0f * w)
                  - 0.012604f * cosf(3.0f * w);
        taps[i] = sinc * win;
        sum += taps[i];
    }
    for (int i = 0; i < ntaps; i++) taps[i] /= sum;
}

/* ── FIR Filter State ── */

typedef struct {
    float taps[FIR_TAPS];
    float delay[FIR_TAPS];
    int   pos;
} fir_state_t;

static float fir_process_sample(fir_state_t *f, float in)
{
    f->delay[f->pos] = in;
    f->pos = (f->pos + 1) % FIR_TAPS;
    float acc = 0.0f;
    int idx = f->pos;
    for (int t = 0; t < FIR_TAPS; t++) {
        idx--;
        if (idx < 0) idx = FIR_TAPS - 1;
        acc += f->delay[idx] * f->taps[t];
    }
    return acc;
}

/* ── Demodulator State ── */

struct fm_demod_simple {
    uint32_t sample_rate;
    uint32_t audio_rate;
    float    fm_scale;          /* 1/max_angle for normalization */
    float    resample_ratio;    /* sample_rate / audio_rate */
    float    resample_phase;
    int      volume;            /* 0-100, applied as float multiply */

    /* FM discriminator */
    float prev_i, prev_q;

    /* L+R path: LPF + de-emphasis */
    fir_state_t fir_lpr;
    float       deemph_lpr;
    float       deemph_alpha;

    /* L-R path: LPF + de-emphasis */
    fir_state_t fir_lmr;
    float       deemph_lmr;

    /* Goertzel 19kHz pilot detector */
    float goertzel_s1, goertzel_s2;
    float goertzel_coeff;       /* 2*cos(2*pi*19000/sample_rate) */
    int   goertzel_count;
    float goertzel_power;
    bool  pilot_detected;

    /* PLL for 19kHz pilot */
    float pll_phase;            /* radians */
    float pll_freq;             /* radians/sample (nominal 19kHz) */
    float pll_integrator;
    bool  pll_locked;

    /* Stereo blend */
    float blend_ratio;          /* 0=mono, 1=full stereo */

    /* RDS decoder (57kHz subcarrier) */
    rds_decoder_t *rds;
    float *rds_buf;             /* Accumulate-and-dump output buffer */
    int    rds_buf_size;
    float  rds_acc;             /* A&D accumulator */
    int    rds_decim_phase;     /* A&D sample counter */
    int    rds_decim;           /* Decimation ratio: sample_rate / ~2375 */

    /* Signal strength */
    float signal_rms;
    float signal_acc;
    int   signal_count;

    /* Pre-allocated work buffers (avoid malloc/free per block) */
    int    work_buf_size;
    float *mpx_buf;
    float *lpr_buf;
    float *lmr_buf;
};

fm_demod_simple_t *fm_demod_simple_create(uint32_t sample_rate, uint32_t audio_rate)
{
    fm_demod_simple_t *d = calloc(1, sizeof(fm_demod_simple_t));
    if (!d) return NULL;

    d->sample_rate = sample_rate;
    d->audio_rate = audio_rate;
    d->volume = 90;

    /* FM discriminator scale */
    float max_angle = 2.0f * (float)M_PI * FM_DEVIATION / (float)sample_rate;
    d->fm_scale = 1.0f / max_angle;

    /* De-emphasis */
    float tau = (float)DEEMPH_TAU_US * 1e-6f;
    d->deemph_alpha = 1.0f - expf(-1.0f / (tau * (float)sample_rate));

    /* Design 15kHz LPF for both L+R and L-R paths */
    design_lpf(d->fir_lpr.taps, FIR_TAPS, 15000.0f, (float)sample_rate);
    memcpy(d->fir_lmr.taps, d->fir_lpr.taps, sizeof(d->fir_lpr.taps));

    /* Goertzel: 2*cos(2*pi*19000/sample_rate) */
    d->goertzel_coeff = 2.0f * cosf(2.0f * (float)M_PI * 19000.0f / (float)sample_rate);

    /* PLL nominal frequency */
    d->pll_freq = 2.0f * (float)M_PI * 19000.0f / (float)sample_rate;

    /* Resampler */
    d->resample_ratio = (float)sample_rate / (float)audio_rate;
    d->resample_phase = 0.0f;

    /* Pre-allocate work buffers for max block size (16384 bytes = 8192 IQ pairs) */
    d->work_buf_size = 8192 + 1;
    d->mpx_buf = calloc(d->work_buf_size, sizeof(float));
    d->lpr_buf = calloc(d->work_buf_size, sizeof(float));
    d->lmr_buf = calloc(d->work_buf_size, sizeof(float));
    if (!d->mpx_buf || !d->lpr_buf || !d->lmr_buf) {
        free(d->mpx_buf); free(d->lpr_buf); free(d->lmr_buf); free(d);
        return NULL;
    }

    /* RDS decoder: 57kHz mixing + accumulate-and-dump to symbol rate */
    d->rds_decim = (int)(sample_rate / 2375.0f);
    if (d->rds_decim < 1) d->rds_decim = 1;
    d->rds_buf_size = d->work_buf_size / d->rds_decim + 2;
    d->rds_buf = calloc(d->rds_buf_size, sizeof(float));
    float rds_rate = sample_rate / (float)d->rds_decim;
    d->rds = rds_decoder_create(rds_rate);
    if (!d->rds || !d->rds_buf) {
        free(d->rds_buf);
        rds_decoder_free(d->rds);
        d->rds = NULL;
        d->rds_buf = NULL;
    }

    return d;
}

void fm_demod_simple_free(fm_demod_simple_t *d) {
    if (d) { free(d->mpx_buf); free(d->lpr_buf); free(d->lmr_buf); free(d); }
}

/* ── Processing ── */

int fm_demod_simple_process(fm_demod_simple_t *d, const uint8_t *iq_u8, int iq_bytes,
                            int16_t *audio_out, int max_pairs)
{
    if (!d || !iq_u8 || !audio_out || iq_bytes < 2) return 0;

    int iq_pairs = iq_bytes / 2;
    int audio_count = 0;

    /* Use pre-allocated work buffers */
    if (iq_pairs >= d->work_buf_size) iq_pairs = d->work_buf_size - 1;
    float *mpx_buf = d->mpx_buf;
    float *lpr_buf = d->lpr_buf;
    float *lmr_buf = d->lmr_buf;
    int mpx_count = 0;
    int rds_count = 0;

    /* Step 1: U8→float IQ + FM discriminator → MPX */
    for (int i = 0; i < iq_pairs; i++) {
        float fi = ((float)iq_u8[i * 2]     - 127.5f) / 127.5f;
        float fq = ((float)iq_u8[i * 2 + 1] - 127.5f) / 127.5f;

        /* Signal strength */
        d->signal_acc += fi * fi + fq * fq;
        d->signal_count++;

        /* FM discriminator: atan2(cross, dot) */
        float dot   = fi * d->prev_i + fq * d->prev_q;
        float cross = fq * d->prev_i - fi * d->prev_q;
        d->prev_i = fi;
        d->prev_q = fq;

        float mpx = atan2f(cross, dot) * d->fm_scale;
        mpx_buf[mpx_count++] = mpx;
    }

    /* Step 2: Per-MPX-sample processing: Goertzel + PLL + L+R/L-R extraction */
    for (int i = 0; i < mpx_count; i++) {
        float mpx = mpx_buf[i];

        /* Goertzel pilot detector */
        float s0 = d->goertzel_coeff * d->goertzel_s1 - d->goertzel_s2 + mpx;
        d->goertzel_s2 = d->goertzel_s1;
        d->goertzel_s1 = s0;
        d->goertzel_count++;

        if (d->goertzel_count >= GOERTZEL_N) {
            float power = d->goertzel_s1 * d->goertzel_s1
                        + d->goertzel_s2 * d->goertzel_s2
                        - d->goertzel_coeff * d->goertzel_s1 * d->goertzel_s2;
            d->goertzel_power = power;
            /* Threshold: empirically tuned for 256kSPS FM signal */
            float thresh = 0.001f;
            d->pilot_detected = (power > thresh);
            d->goertzel_s1 = 0;
            d->goertzel_s2 = 0;
            d->goertzel_count = 0;
        }

        /* PLL tracking 19kHz pilot */
        float nco_sin = sinf(d->pll_phase);

        /* Phase detector: quadrature error */
        float pd_q = mpx * nco_sin;

        /* PI loop filter */
        d->pll_integrator += 0.0001f * pd_q;
        float correction = 0.01f * pd_q + d->pll_integrator;

        d->pll_phase += d->pll_freq + correction;
        /* Keep phase in [0, 2*pi) */
        if (d->pll_phase > 2.0f * (float)M_PI) d->pll_phase -= 2.0f * (float)M_PI;
        if (d->pll_phase < 0) d->pll_phase += 2.0f * (float)M_PI;

        /* PLL lock: |pd_q| should be small when locked */
        d->pll_locked = d->pilot_detected;

        /* L+R: MPX directly (below 15kHz is L+R) → LPF → de-emphasis */
        float lpr_filt = fir_process_sample(&d->fir_lpr, mpx);
        d->deemph_lpr = d->deemph_alpha * lpr_filt + (1.0f - d->deemph_alpha) * d->deemph_lpr;
        lpr_buf[i] = d->deemph_lpr;

        /* L-R: multiply MPX by 38kHz reference (2× pilot phase) */
        float ref_38k = sinf(2.0f * d->pll_phase);
        float lmr_raw = mpx * ref_38k * 2.0f;  /* ×2 for DSB-SC amplitude recovery */
        float lmr_filt = fir_process_sample(&d->fir_lmr, lmr_raw);
        d->deemph_lmr = d->deemph_alpha * lmr_filt + (1.0f - d->deemph_alpha) * d->deemph_lmr;
        lmr_buf[i] = d->deemph_lmr;

        /* RDS: mix MPX with 57kHz (3× pilot phase), accumulate-and-dump */
        if (d->rds) {
            float ref_57k = sinf(3.0f * d->pll_phase);
            float rds_mixed = mpx * ref_57k;
            d->rds_acc += rds_mixed;
            d->rds_decim_phase++;
            if (d->rds_decim_phase >= d->rds_decim) {
                if (rds_count < d->rds_buf_size) {
                    d->rds_buf[rds_count++] = d->rds_acc / (float)(d->rds_decim / 8 + 1);
                }
                d->rds_acc = 0;
                d->rds_decim_phase = 0;
            }
        }
    }

    /* Feed accumulated RDS samples to decoder */
    if (d->rds && rds_count > 0) {
        rds_decoder_process(d->rds, d->rds_buf, rds_count);
    }

    /* Step 3: Stereo blend */
    bool is_stereo = d->pilot_detected && d->pll_locked;
    if (is_stereo) {
        /* Ramp blend toward 1.0 */
        d->blend_ratio += 0.01f;
        if (d->blend_ratio > 1.0f) d->blend_ratio = 1.0f;
    } else {
        /* Ramp blend toward 0.0 (mono) */
        d->blend_ratio -= 0.01f;
        if (d->blend_ratio < 0.0f) d->blend_ratio = 0.0f;
    }

    /* Step 4: Resample 256k→48k + stereo matrix → int16 output */
    float vol = (float)d->volume / 100.0f;

    while (d->resample_phase < (float)mpx_count && audio_count < max_pairs) {
        int idx0 = (int)d->resample_phase;
        float frac = d->resample_phase - (float)idx0;
        int idx1 = (idx0 + 1 < mpx_count) ? idx0 + 1 : idx0;

        /* Interpolate L+R and L-R */
        float lpr = lpr_buf[idx0] + frac * (lpr_buf[idx1] - lpr_buf[idx0]);
        float lmr = lmr_buf[idx0] + frac * (lmr_buf[idx1] - lmr_buf[idx0]);

        /* Stereo matrix with blend */
        float L, R;
        if (d->blend_ratio > 0.01f) {
            float stereo_L = (lpr + lmr) * 0.5f;
            float stereo_R = (lpr - lmr) * 0.5f;
            L = lpr * (1.0f - d->blend_ratio) + stereo_L * d->blend_ratio;
            R = lpr * (1.0f - d->blend_ratio) + stereo_R * d->blend_ratio;
        } else {
            L = lpr;
            R = lpr;
        }

        /* Volume + clip + convert to int16 */
        L *= vol * 32000.0f;
        R *= vol * 32000.0f;
        if (L > 32767.0f) L = 32767.0f;
        if (L < -32768.0f) L = -32768.0f;
        if (R > 32767.0f) R = 32767.0f;
        if (R < -32768.0f) R = -32768.0f;

        audio_out[audio_count * 2]     = (int16_t)L;
        audio_out[audio_count * 2 + 1] = (int16_t)R;
        audio_count++;

        d->resample_phase += d->resample_ratio;
    }

    d->resample_phase -= (float)mpx_count;
    if (d->resample_phase < 0.0f) d->resample_phase = 0.0f;

    /* Update signal strength */
    if (d->signal_count > 0) {
        d->signal_rms = sqrtf(d->signal_acc / (float)d->signal_count);
        d->signal_acc = 0.0f;
        d->signal_count = 0;
    }

    return audio_count;
}

float fm_demod_simple_get_signal_strength(fm_demod_simple_t *d)
{
    return d ? d->signal_rms : 0.0f;
}

bool fm_demod_simple_is_stereo(fm_demod_simple_t *d)
{
    return d ? (d->pilot_detected && d->pll_locked) : false;
}

void fm_demod_simple_get_rds(fm_demod_simple_t *d, rds_data_t *out)
{
    if (d && d->rds && out) {
        rds_decoder_get_data(d->rds, out);
    } else if (out) {
        memset(out, 0, sizeof(*out));
    }
}
