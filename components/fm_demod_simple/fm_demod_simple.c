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

#define FIR_TAPS        15     /* 15 taps: ~50% less CPU than 31. Wider transition but adequate for mono+stereo */
#define GOERTZEL_N      1024
#define SIN_LUT_SIZE    1024    /* Must be power of 2 */
#define SIN_LUT_MASK    (SIN_LUT_SIZE - 1)
#define DEEMPH_TAU_US   75      /* 75us NA/Japan, 50us Europe */
#define FM_DEVIATION    75000.0f

/* ── Fast Math: Sin/Cos LUT ── */

static float sin_lut[SIN_LUT_SIZE];
static int sin_lut_initialized = 0;

static void init_sin_lut(void)
{
    if (sin_lut_initialized) return;
    for (int i = 0; i < SIN_LUT_SIZE; i++) {
        sin_lut[i] = sinf(2.0f * (float)M_PI * (float)i / (float)SIN_LUT_SIZE);
    }
    sin_lut_initialized = 1;
}

/* Phase in radians → sin via LUT with linear interpolation. ~8 cycles vs ~85 for sinf */
static inline float fast_sinf(float phase)
{
    /* Normalize phase to [0, 2*pi) */
    const float TWO_PI = 2.0f * (float)M_PI;
    const float INV_TWO_PI = 1.0f / TWO_PI;
    float norm = phase * INV_TWO_PI;
    norm -= (float)(int)norm;
    if (norm < 0) norm += 1.0f;

    float fidx = norm * (float)SIN_LUT_SIZE;
    int idx0 = (int)fidx & SIN_LUT_MASK;
    int idx1 = (idx0 + 1) & SIN_LUT_MASK;
    float frac = fidx - (float)(int)fidx;
    return sin_lut[idx0] + frac * (sin_lut[idx1] - sin_lut[idx0]);
}

static inline float fast_cosf(float phase)
{
    return fast_sinf(phase + (float)M_PI * 0.5f);
}

/* Ultra-fast sin/cos from uint32 phase accumulator — zero division.
 * Phase: 0..2^32 maps to 0..2π. LUT index = phase >> (32 - LUT_BITS).
 * Linear interpolation using fractional bits. ~4 cycles. */
#define LUT_SHIFT   (32 - 10)   /* 10 = log2(SIN_LUT_SIZE=1024) */
#define LUT_FRAC_BITS (LUT_SHIFT)
#define LUT_FRAC_SCALE (1.0f / (float)(1u << LUT_FRAC_BITS))

static inline float sin_u32(uint32_t phase)
{
    uint32_t idx0 = phase >> LUT_SHIFT;
    uint32_t idx1 = (idx0 + 1) & SIN_LUT_MASK;
    float frac = (float)(phase & ((1u << LUT_SHIFT) - 1)) * LUT_FRAC_SCALE;
    return sin_lut[idx0] + frac * (sin_lut[idx1] - sin_lut[idx0]);
}

static inline float cos_u32(uint32_t phase)
{
    return sin_u32(phase + (1u << 30));  /* +π/2 = +2^30 */
}

/* ── Fast Math: Polynomial atan2 ── */

/* 9th-order minimax polynomial atan. Max error 0.001° (0.00001 rad).
 * THD < 0.001% vs 0.17% for 3rd-order. Only 2 extra multiplies.
 * Coefficients from Abramowitz & Stegun, optimized for |r|<=1. */
static inline float fast_atan_poly(float r)
{
    float r2 = r * r;
    return r * (0.9998660f + r2 * (-0.3302995f + r2 * (0.1801410f
              + r2 * (-0.0851330f + r2 * 0.0208351f))));
}

static inline float fast_atan2f(float y, float x)
{
    float abs_x = fabsf(x);
    float abs_y = fabsf(y);

    if (abs_x < 1e-20f && abs_y < 1e-20f) return 0.0f;

    float angle;
    if (abs_x >= abs_y) {
        angle = fast_atan_poly(y / x);
        if (x < 0.0f) {
            angle += (y >= 0.0f) ? (float)M_PI : -(float)M_PI;
        }
    } else {
        angle = (y > 0.0f ? (float)M_PI * 0.5f : -(float)M_PI * 0.5f)
              - fast_atan_poly(x / y);
    }
    return angle;
}

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

    /* 19kHz pilot notch filter (2nd-order IIR, Direct Form I) */
    float notch_x1, notch_x2;  /* Input history */
    float notch_y1, notch_y2;  /* Output history */

    /* L-R path: LPF + de-emphasis */
    fir_state_t fir_lmr;
    float       deemph_lmr;

    /* Goertzel 19kHz pilot detector */
    float goertzel_s1, goertzel_s2;
    float goertzel_coeff;       /* 2*cos(2*pi*19000/sample_rate) */
    int   goertzel_count;
    float goertzel_power;
    bool  pilot_detected;

    /* PLL for 19kHz pilot — uses uint32 phase accumulator for fast LUT access.
     * Phase wraps naturally at 2^32 = one full cycle.
     * LUT index = phase >> (32 - LUT_BITS). No division needed. */
    uint32_t pll_phase;         /* 0..2^32 = 0..2π */
    uint32_t pll_freq;          /* Phase increment per sample */
    float    pll_integrator;
    bool     pll_locked;

    /* Stereo blend */
    float blend_ratio;          /* 0=mono, 1=full stereo */

    /* Audio AGC (slow auto-gain to maximize output level) */
    float agc_gain;             /* Current gain multiplier */
    float agc_peak;             /* Running peak tracker */

    /* RDS decoder (57kHz subcarrier) */
    rds_decoder_t *rds;
    float *rds_buf;             /* Accumulate-and-dump output buffer */
    int    rds_buf_size;
    float  rds_acc;             /* A&D accumulator */
    float  rds_phase;           /* Fractional phase accumulator [0, 1) */
    float  rds_phase_inc;       /* Phase increment per sample: 2375/sample_rate */
    int    rds_acc_count;       /* Samples in current accumulation */
    int    rds_decim;           /* Nominal decimation ratio (for buffer sizing) */

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
    init_sin_lut();

    fm_demod_simple_t *d = calloc(1, sizeof(fm_demod_simple_t));
    if (!d) return NULL;

    d->sample_rate = sample_rate;
    d->audio_rate = audio_rate;
    d->volume = 90;
    d->agc_gain = 4.0f;    /* Start with moderate gain */
    d->agc_peak = 0.1f;    /* Initial peak estimate */

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

    /* PLL: uint32 phase accumulator. 2^32 = one full cycle.
     * freq_inc = 19000 / sample_rate * 2^32 */
    d->pll_freq = (uint32_t)(19000.0 / (double)sample_rate * 4294967296.0);

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

    /* RDS decoder: 57kHz mixing + fractional accumulate-and-dump.
     * Use fractional phase accumulator to avoid integer rate mismatch.
     * Target: exactly 2375 symbols/sec from 256000 sps.
     * Phase inc = 2375.0 / sample_rate. Dump when phase >= 1.0. */
    d->rds_decim = (int)(sample_rate / 2375.0f);
    if (d->rds_decim < 1) d->rds_decim = 1;
    d->rds_buf_size = d->work_buf_size / d->rds_decim + 2;
    d->rds_buf = calloc(d->rds_buf_size, sizeof(float));
    d->rds_phase_inc = 2375.0f / (float)sample_rate;  /* Exact fractional increment */
    float rds_rate = 2375.0f;
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

        float mpx = fast_atan2f(cross, dot) * d->fm_scale;
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
        /* PLL: use uint32 phase accumulator for fast LUT access */
        float nco_sin = sin_u32(d->pll_phase);

        /* Phase detector: quadrature error */
        float pd_q = mpx * nco_sin;

        /* PI loop filter → phase correction as uint32 increment */
        d->pll_integrator += 0.0001f * pd_q;
        float correction = 0.01f * pd_q + d->pll_integrator;
        /* Convert float correction (radians) to uint32 phase units.
         * Precomputed: (1<<30)/PI = 341782637.8 */
        int32_t corr_u32 = (int32_t)(correction * 341782638.0f);

        d->pll_phase += d->pll_freq + (uint32_t)corr_u32;
        /* uint32 wraps naturally — no range check needed */

        d->pll_locked = d->pilot_detected;

        /* L+R: MPX → LPF → 19kHz notch → de-emphasis */
        float lpr_filt = fir_process_sample(&d->fir_lpr, mpx);

        /* 19kHz pilot notch (2nd-order IIR, r=0.95).
         * H(z) = (1 + b1·z⁻¹ + z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)
         * b1 = -2cos(w0) = -1.7865, a1 = -2r·cos(w0) = -1.6971, a2 = r² = 0.9025
         * Gives -inf dB at 19kHz, <0.6dB effect on 0-15kHz audio */
        {
            const float b1 = -1.78645f, a1 = -1.69713f, a2 = 0.90250f;
            float y = lpr_filt + b1 * d->notch_x1 + d->notch_x2
                    - a1 * d->notch_y1 - a2 * d->notch_y2;
            d->notch_x2 = d->notch_x1; d->notch_x1 = lpr_filt;
            d->notch_y2 = d->notch_y1; d->notch_y1 = y;
            lpr_filt = y;
        }

        d->deemph_lpr = d->deemph_alpha * lpr_filt + (1.0f - d->deemph_alpha) * d->deemph_lpr;
        lpr_buf[i] = d->deemph_lpr;

        /* L-R: only compute when stereo is active (saves ~30% CPU in mono) */
        if (d->blend_ratio > 0.001f || d->pilot_detected) {
            float ref_38k = sin_u32(d->pll_phase << 1);
            float lmr_raw = mpx * ref_38k * 2.0f;
            float lmr_filt = fir_process_sample(&d->fir_lmr, lmr_raw);
            d->deemph_lmr = d->deemph_alpha * lmr_filt + (1.0f - d->deemph_alpha) * d->deemph_lmr;
            lmr_buf[i] = d->deemph_lmr;
        } else {
            lmr_buf[i] = 0.0f;
        }

        /* RDS: mix MPX with 57kHz (3× pilot phase), fractional A&D */
        if (d->rds) {
            float ref_57k = sin_u32(d->pll_phase * 3);
            float rds_mixed = mpx * ref_57k;
            d->rds_acc += rds_mixed;
            d->rds_acc_count++;
            d->rds_phase += d->rds_phase_inc;
            if (d->rds_phase >= 1.0f) {
                d->rds_phase -= 1.0f;
                if (rds_count < d->rds_buf_size && d->rds_acc_count > 0) {
                    /* Normalize by sample count for consistent amplitude */
                    d->rds_buf[rds_count++] = d->rds_acc / (float)d->rds_acc_count;
                }
                d->rds_acc = 0;
                d->rds_acc_count = 0;
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

    /* Step 4: Resample 256k→48k + stereo matrix + AGC → int16 output */
    float vol = (float)d->volume / 100.0f;

    /* AGC: track peak and adjust gain slowly.
     * Target: 70% of full scale (22937 out of 32767).
     * Attack: fast (immediate peak update). Release: slow (0.9995 decay). */
    float agc_target = 0.7f;

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

        /* AGC: update peak tracker and compute gain */
        float peak = fabsf(L);
        float peak_r = fabsf(R);
        if (peak_r > peak) peak = peak_r;
        if (peak > d->agc_peak) {
            d->agc_peak = peak;                           /* Fast attack */
        } else {
            d->agc_peak = d->agc_peak * 0.9999f + peak * 0.0001f;  /* Slow release */
        }
        if (d->agc_peak > 0.001f) {
            d->agc_gain = agc_target / d->agc_peak;
            if (d->agc_gain > 20.0f) d->agc_gain = 20.0f;   /* Max 26dB gain */
            if (d->agc_gain < 0.5f) d->agc_gain = 0.5f;     /* Min -6dB */
        }

        /* Apply AGC + volume + convert to int16 */
        L *= d->agc_gain * vol * 32767.0f;
        R *= d->agc_gain * vol * 32767.0f;
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
