/*
 * DSP Engine for ESP32-P4 WebSDR
 *
 * Radix-2 Cooley-Tukey FFT and simple DDC (NCO mixer + CIC-like decimator).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dsp.h"

/* ──────────────────────── FFT Engine ──────────────────────── */

#define FFT_MAX_SIZE    4096
#define FFT_AVG_COUNT   4       /* Average this many FFT frames before output */
#define DB_MIN          (-100.0f)
#define DB_MAX          (0.0f)

static int      fft_n = 1024;
static float    fft_window[FFT_MAX_SIZE];
static float    fft_twiddle_re[FFT_MAX_SIZE / 2];
static float    fft_twiddle_im[FFT_MAX_SIZE / 2];
static float    fft_power[FFT_MAX_SIZE];
static int      fft_avg_count;
static float    fft_input_re[FFT_MAX_SIZE];
static float    fft_input_im[FFT_MAX_SIZE];
static int      fft_input_pos;

void dsp_fft_init(int fft_size)
{
    if (fft_size > FFT_MAX_SIZE) fft_size = FFT_MAX_SIZE;
    /* Clamp to power of 2 */
    int n = 1;
    while (n < fft_size) n <<= 1;
    fft_n = n;

    /* Hanning window */
    for (int i = 0; i < fft_n; i++) {
        fft_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (fft_n - 1)));
    }

    /* Twiddle factors for radix-2 FFT */
    for (int i = 0; i < fft_n / 2; i++) {
        float angle = -2.0f * (float)M_PI * i / fft_n;
        fft_twiddle_re[i] = cosf(angle);
        fft_twiddle_im[i] = sinf(angle);
    }

    dsp_fft_reset();
}

void dsp_fft_reset(void)
{
    memset(fft_power, 0, sizeof(fft_power));
    fft_avg_count = 0;
    fft_input_pos = 0;
}

/* In-place radix-2 DIT FFT */
static void fft_radix2(float *re, float *im, int n)
{
    /* Bit-reversal permutation */
    int j = 0;
    for (int i = 0; i < n - 1; i++) {
        if (i < j) {
            float tmp;
            tmp = re[i]; re[i] = re[j]; re[j] = tmp;
            tmp = im[i]; im[i] = im[j]; im[j] = tmp;
        }
        int m = n >> 1;
        while (m >= 1 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    /* Butterfly stages */
    for (int stage = 1; stage < n; stage <<= 1) {
        int step = stage << 1;
        int tw_step = n / step;
        for (int k = 0; k < n; k += step) {
            for (int s = 0; s < stage; s++) {
                int tw_idx = s * tw_step;
                float wr = fft_twiddle_re[tw_idx];
                float wi = fft_twiddle_im[tw_idx];
                int a = k + s;
                int b = a + stage;
                float tr = wr * re[b] - wi * im[b];
                float ti = wr * im[b] + wi * re[b];
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
    }
}

void dsp_fft_compute(const uint8_t *iq_data, uint32_t len,
                     uint8_t *fft_out, int *fft_out_len)
{
    *fft_out_len = 0;

    /* Accumulate IQ samples into input buffer */
    uint32_t i = 0;
    while (i + 1 < len) {
        float fi = (iq_data[i]     - 127.5f) / 127.5f;
        float fq = (iq_data[i + 1] - 127.5f) / 127.5f;
        fft_input_re[fft_input_pos] = fi * fft_window[fft_input_pos];
        fft_input_im[fft_input_pos] = fq * fft_window[fft_input_pos];
        fft_input_pos++;
        i += 2;

        if (fft_input_pos >= fft_n) {
            /* Run FFT */
            float work_re[FFT_MAX_SIZE], work_im[FFT_MAX_SIZE];
            memcpy(work_re, fft_input_re, fft_n * sizeof(float));
            memcpy(work_im, fft_input_im, fft_n * sizeof(float));
            fft_radix2(work_re, work_im, fft_n);

            /* Accumulate power spectrum */
            for (int k = 0; k < fft_n; k++) {
                float pwr = work_re[k] * work_re[k] + work_im[k] * work_im[k];
                fft_power[k] += pwr;
            }
            fft_avg_count++;
            fft_input_pos = 0;

            /* Output averaged spectrum */
            if (fft_avg_count >= FFT_AVG_COUNT) {
                float inv = 1.0f / fft_avg_count;
                for (int k = 0; k < fft_n; k++) {
                    float avg_pwr = fft_power[k] * inv;
                    float db;
                    if (avg_pwr < 1e-20f) {
                        db = DB_MIN;
                    } else {
                        db = 10.0f * log10f(avg_pwr);
                    }
                    /* Clamp and scale to 0-255 */
                    if (db < DB_MIN) db = DB_MIN;
                    if (db > DB_MAX) db = DB_MAX;
                    float scaled = (db - DB_MIN) / (DB_MAX - DB_MIN) * 255.0f;

                    /* FFT output is in natural order: DC at bin 0.
                     * Rearrange so DC is in the center (fftshift). */
                    int dst = (k + fft_n / 2) % fft_n;
                    fft_out[dst] = (uint8_t)(scaled + 0.5f);
                }
                *fft_out_len = fft_n;

                /* Reset accumulator */
                memset(fft_power, 0, fft_n * sizeof(float));
                fft_avg_count = 0;
            }
        }
    }
}

/* ──────────────────────── DDC Engine ──────────────────────── */

#define NCO_TABLE_SIZE  1024

struct dsp_ddc {
    uint32_t    sample_rate;
    int32_t     offset_hz;
    uint32_t    output_bw;
    int         decim_ratio;

    /* NCO (Numerically Controlled Oscillator) */
    float       nco_re[NCO_TABLE_SIZE];
    float       nco_im[NCO_TABLE_SIZE];
    uint32_t    nco_phase;      /* Phase accumulator (fractional index * 2^16) */
    uint32_t    nco_step;       /* Phase increment per sample */

    /* CIC-like accumulator */
    float       acc_re;
    float       acc_im;
    int         acc_count;
};

/* Round up to nearest power of 2 */
static int next_pow2(int v)
{
    int p = 1;
    while (p < v) p <<= 1;
    return p;
}

dsp_ddc_t *dsp_ddc_create(uint32_t sample_rate, uint32_t center_offset_hz,
                           uint32_t output_bw_hz)
{
    if (sample_rate == 0 || output_bw_hz == 0) return NULL;

    dsp_ddc_t *ddc = calloc(1, sizeof(dsp_ddc_t));
    if (!ddc) return NULL;

    ddc->sample_rate = sample_rate;
    ddc->offset_hz = (int32_t)center_offset_hz;
    ddc->output_bw = output_bw_hz;

    /* Compute decimation ratio (clamp to power of 2) */
    int ratio = (int)(sample_rate / output_bw_hz);
    if (ratio < 1) ratio = 1;
    ddc->decim_ratio = next_pow2(ratio);
    /* Don't decimate more than input rate allows */
    if (ddc->decim_ratio > (int)(sample_rate / 1000)) {
        ddc->decim_ratio = next_pow2(sample_rate / 1000);
    }
    if (ddc->decim_ratio < 1) ddc->decim_ratio = 1;

    /* Precompute NCO table (one full cycle) */
    for (int i = 0; i < NCO_TABLE_SIZE; i++) {
        float angle = -2.0f * (float)M_PI * i / NCO_TABLE_SIZE;
        ddc->nco_re[i] = cosf(angle);
        ddc->nco_im[i] = sinf(angle);
    }

    /* NCO phase step: offset_hz cycles per sample, mapped to table */
    /* step = (offset / sample_rate) * TABLE_SIZE * 65536 (fixed-point) */
    double step = ((double)ddc->offset_hz / (double)sample_rate)
                  * NCO_TABLE_SIZE * 65536.0;
    /* Handle negative offsets via unsigned wrap */
    ddc->nco_step = (uint32_t)(int32_t)step;
    ddc->nco_phase = 0;

    ddc->acc_re = 0;
    ddc->acc_im = 0;
    ddc->acc_count = 0;

    return ddc;
}

void dsp_ddc_free(dsp_ddc_t *ddc)
{
    free(ddc);
}

int dsp_ddc_process(dsp_ddc_t *ddc, const uint8_t *iq_in, uint32_t in_len,
                    uint8_t *iq_out, uint32_t *out_len)
{
    if (!ddc || !iq_in || !iq_out || !out_len) return -1;

    uint32_t capacity = *out_len;
    uint32_t written = 0;
    int decim = ddc->decim_ratio;
    float inv_decim = 1.0f / decim;

    for (uint32_t i = 0; i + 1 < in_len; i += 2) {
        /* Convert to float */
        float si = (iq_in[i]     - 127.5f) / 127.5f;
        float sq = (iq_in[i + 1] - 127.5f) / 127.5f;

        /* Mix with NCO (complex multiply) */
        uint32_t idx = (ddc->nco_phase >> 16) & (NCO_TABLE_SIZE - 1);
        float nre = ddc->nco_re[idx];
        float nim = ddc->nco_im[idx];
        ddc->nco_phase += ddc->nco_step;

        float mixed_re = si * nre - sq * nim;
        float mixed_im = si * nim + sq * nre;

        /* Accumulate (CIC-like decimation) */
        ddc->acc_re += mixed_re;
        ddc->acc_im += mixed_im;
        ddc->acc_count++;

        if (ddc->acc_count >= decim) {
            /* Output decimated sample */
            float out_re = ddc->acc_re * inv_decim;
            float out_im = ddc->acc_im * inv_decim;

            /* Convert back to uint8 */
            int ival = (int)(out_re * 127.5f + 127.5f);
            int qval = (int)(out_im * 127.5f + 127.5f);
            if (ival < 0) ival = 0;
            if (ival > 255) ival = 255;
            if (qval < 0) qval = 0;
            if (qval > 255) qval = 255;

            if (written + 1 < capacity) {
                iq_out[written++] = (uint8_t)ival;
                iq_out[written++] = (uint8_t)qval;
            }

            ddc->acc_re = 0;
            ddc->acc_im = 0;
            ddc->acc_count = 0;
        }
    }

    *out_len = written;
    return 0;
}
