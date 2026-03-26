/*
 * Simple WBFM Demodulator — Plain C, Floating Point
 *
 * Pipeline: U8 IQ -> float IQ -> CIC decimate x4 -> FM discriminator ->
 *           de-emphasis -> FIR LPF 15kHz -> resample to 48kHz -> float output
 *
 * Input:  1024 kSPS uint8 IQ
 * After CIC: 256 kSPS float IQ
 * After demod: 256 kSPS float mono
 * After resample: 48 kHz float mono
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fm_demod_simple.h"

/* ── FIR LPF Design ── */

#define FIR_TAPS    63
#define CIC_DECIMATION  4
#define DEEMPH_TAU_US   75  /* 75us for NA/Japan, 50us for Europe */

/* Nuttall window coefficients */
static void nuttall_window(float *w, int n)
{
    const float a0 = 0.355768f;
    const float a1 = 0.487396f;
    const float a2 = 0.144232f;
    const float a3 = 0.012604f;

    for (int i = 0; i < n; i++) {
        float x = 2.0f * (float)M_PI * (float)i / (float)(n - 1);
        w[i] = a0 - a1 * cosf(x) + a2 * cosf(2.0f * x) - a3 * cosf(3.0f * x);
    }
}

/* Design a windowed sinc LPF */
static void design_lpf(float *taps, int ntaps, float cutoff_hz, float sample_rate)
{
    float fc = cutoff_hz / sample_rate;
    float window[FIR_TAPS];
    nuttall_window(window, ntaps);

    int mid = ntaps / 2;
    float sum = 0.0f;

    for (int i = 0; i < ntaps; i++) {
        int n = i - mid;
        if (n == 0) {
            taps[i] = 2.0f * fc;
        } else {
            taps[i] = sinf(2.0f * (float)M_PI * fc * (float)n) / ((float)M_PI * (float)n);
        }
        taps[i] *= window[i];
        sum += taps[i];
    }

    /* Normalize for unity gain at DC */
    for (int i = 0; i < ntaps; i++) {
        taps[i] /= sum;
    }
}

/* ── Demodulator State ── */

struct fm_demod_simple {
    uint32_t sample_rate;       /* Input sample rate */
    uint32_t audio_rate;        /* Output audio rate */
    uint32_t cic_rate;          /* Rate after CIC decimation */

    /* CIC decimator state (3rd order, complex) */
    float cic_integrator_i[3];
    float cic_integrator_q[3];
    float cic_comb_delay_i[3];
    float cic_comb_delay_q[3];
    int   cic_count;

    /* FM discriminator state */
    float prev_i;
    float prev_q;
    float fm_scale;         /* 1.0 / max_angle for normalization */

    /* De-emphasis IIR state */
    float deemph_alpha;
    float deemph_prev;

    /* FIR LPF state */
    float fir_taps[FIR_TAPS];
    float fir_delay[FIR_TAPS];
    int   fir_pos;

    /* Resampler state */
    float resample_ratio;       /* cic_rate / audio_rate */
    float resample_phase;

    /* Signal strength */
    float signal_rms;
    float signal_acc;
    int   signal_count;
};

fm_demod_simple_t *fm_demod_simple_create(uint32_t sample_rate, uint32_t audio_rate)
{
    fm_demod_simple_t *d = calloc(1, sizeof(fm_demod_simple_t));
    if (!d) return NULL;

    d->sample_rate = sample_rate;
    d->audio_rate = audio_rate;
    d->cic_rate = sample_rate / CIC_DECIMATION;

    /* FM discriminator scale: max angle per sample at 75kHz deviation.
     * atan2 output range is [-pi, pi]. For WBFM, max deviation = 75kHz,
     * max_angle = 2*pi*75000/256000 = 1.842 rad.
     * Scale output to [-1, 1] at max deviation. */
    float max_angle = 2.0f * (float)M_PI * 75000.0f / (float)d->cic_rate;
    d->fm_scale = 1.0f / max_angle;

    /* De-emphasis: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
     * alpha = 1 - exp(-1 / (tau * fs))
     * For 75us at 256kHz: alpha ~ 0.051 */
    float tau = (float)DEEMPH_TAU_US * 1e-6f;
    d->deemph_alpha = 1.0f - expf(-1.0f / (tau * (float)d->cic_rate));

    /* Design 15 kHz LPF at the CIC output rate */
    design_lpf(d->fir_taps, FIR_TAPS, 15000.0f, (float)d->cic_rate);

    /* Resample ratio: how many input samples per output sample */
    d->resample_ratio = (float)d->cic_rate / (float)audio_rate;
    d->resample_phase = 0.0f;

    return d;
}

void fm_demod_simple_free(fm_demod_simple_t *d)
{
    free(d);
}

/* ── Processing Pipeline ── */

int fm_demod_simple_process(fm_demod_simple_t *d, const uint8_t *iq_u8, int iq_bytes,
                            float *audio_out, int audio_max)
{
    if (!d || !iq_u8 || !audio_out || iq_bytes < 2) return 0;

    int iq_pairs = iq_bytes / 2;
    int audio_count = 0;

    /* Temporary buffer for post-CIC demodulated samples.
     * After CIC decimation by 4, we get at most iq_pairs/4 samples.
     * After demod, same count. We process in the same buffer. */
    int max_cic_out = iq_pairs / CIC_DECIMATION + 1;

    /* Stack-allocate for reasonable sizes, heap for large */
    float *demod_buf;
    int heap_alloc = 0;
    if (max_cic_out <= 4096) {
        demod_buf = alloca(max_cic_out * sizeof(float));
    } else {
        demod_buf = malloc(max_cic_out * sizeof(float));
        heap_alloc = 1;
        if (!demod_buf) return 0;
    }

    int demod_count = 0;

    for (int i = 0; i < iq_pairs; i++) {
        /* Step 1: U8 to float [-1, 1] */
        float fi = ((float)iq_u8[i * 2]     - 127.5f) / 127.5f;
        float fq = ((float)iq_u8[i * 2 + 1] - 127.5f) / 127.5f;

        /* Signal strength accumulator */
        d->signal_acc += fi * fi + fq * fq;
        d->signal_count++;

        /* Step 2: CIC decimator (3rd order) — integrator stage */
        d->cic_integrator_i[0] += fi;
        d->cic_integrator_q[0] += fq;
        d->cic_integrator_i[1] += d->cic_integrator_i[0];
        d->cic_integrator_q[1] += d->cic_integrator_q[0];
        d->cic_integrator_i[2] += d->cic_integrator_i[1];
        d->cic_integrator_q[2] += d->cic_integrator_q[1];

        d->cic_count++;
        if (d->cic_count < CIC_DECIMATION) continue;
        d->cic_count = 0;

        /* CIC comb stage */
        float ci = d->cic_integrator_i[2];
        float cq = d->cic_integrator_q[2];

        float di, dq;
        di = ci - d->cic_comb_delay_i[0];
        d->cic_comb_delay_i[0] = ci;
        ci = di;
        dq = cq - d->cic_comb_delay_q[0];
        d->cic_comb_delay_q[0] = cq;
        cq = dq;

        di = ci - d->cic_comb_delay_i[1];
        d->cic_comb_delay_i[1] = ci;
        ci = di;
        dq = cq - d->cic_comb_delay_q[1];
        d->cic_comb_delay_q[1] = cq;
        cq = dq;

        di = ci - d->cic_comb_delay_i[2];
        d->cic_comb_delay_i[2] = ci;
        ci = di;
        dq = cq - d->cic_comb_delay_q[2];
        d->cic_comb_delay_q[2] = cq;
        cq = dq;

        /* Normalize CIC output (gain = R^N = 4^3 = 64) */
        ci /= 64.0f;
        cq /= 64.0f;

        /* Step 3: FM discriminator using atan2f(cross, dot) */
        float dot   = ci * d->prev_i + cq * d->prev_q;
        float cross = cq * d->prev_i - ci * d->prev_q;
        d->prev_i = ci;
        d->prev_q = cq;

        float fm_out = atan2f(cross, dot);

        /* Scale so max deviation (75kHz) maps to [-1, 1] */
        fm_out *= d->fm_scale;

        /* Step 4: De-emphasis IIR */
        d->deemph_prev = d->deemph_alpha * fm_out + (1.0f - d->deemph_alpha) * d->deemph_prev;
        float demod_sample = d->deemph_prev;

        if (demod_count < max_cic_out) {
            demod_buf[demod_count++] = demod_sample;
        }
    }

    /* Step 5: FIR LPF + Step 6: Resample 256k -> 48k using linear interpolation */
    /* First apply FIR to demod_buf in-place */
    /* We use direct-form FIR with circular delay line */

    for (int i = 0; i < demod_count; i++) {
        /* Push sample into FIR delay line */
        d->fir_delay[d->fir_pos] = demod_buf[i];
        d->fir_pos = (d->fir_pos + 1) % FIR_TAPS;

        /* Compute FIR output */
        float acc = 0.0f;
        int idx = d->fir_pos;
        for (int t = 0; t < FIR_TAPS; t++) {
            idx--;
            if (idx < 0) idx = FIR_TAPS - 1;
            acc += d->fir_delay[idx] * d->fir_taps[t];
        }
        demod_buf[i] = acc;
    }

    /* Resample using linear interpolation */
    while (d->resample_phase < (float)demod_count && audio_count < audio_max) {
        int idx0 = (int)d->resample_phase;
        float frac = d->resample_phase - (float)idx0;

        float s0 = demod_buf[idx0];
        float s1 = (idx0 + 1 < demod_count) ? demod_buf[idx0 + 1] : s0;

        float sample = s0 + frac * (s1 - s0);

        /* Clamp to [-1, 1] */
        if (sample > 1.0f) sample = 1.0f;
        if (sample < -1.0f) sample = -1.0f;

        audio_out[audio_count++] = sample;
        d->resample_phase += d->resample_ratio;
    }

    /* Adjust phase for next call */
    d->resample_phase -= (float)demod_count;
    if (d->resample_phase < 0.0f) d->resample_phase = 0.0f;

    /* Update signal strength RMS */
    if (d->signal_count > 0) {
        d->signal_rms = sqrtf(d->signal_acc / (float)d->signal_count);
        d->signal_acc = 0.0f;
        d->signal_count = 0;
    }

    if (heap_alloc) free(demod_buf);
    return audio_count;
}

float fm_demod_simple_get_signal_strength(fm_demod_simple_t *d)
{
    if (!d) return 0.0f;
    return d->signal_rms;
}
