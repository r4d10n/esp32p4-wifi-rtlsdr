/*
 * DSP Engine for ESP32-P4 WebSDR
 *
 * Uses Espressif esp-dsp library with PIE (Processor Instruction Extensions)
 * hardware acceleration for FFT and signal processing on ESP32-P4's RISC-V core.
 *
 * PIE provides 128-bit SIMD operations on 8x16-bit or 4x32-bit vectors,
 * giving ~2-8x speedup over ANSI C for FFT, windowing, and multiply operations.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dsp.h"

/* esp-dsp PIE-optimized functions */
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"
#include "dsps_mul.h"
#include "dsps_add.h"
#include "dsps_mulc.h"
#include "dsps_addc.h"

static const char *TAG = "dsp";

/* ──────────────────────── FFT Engine (esp-dsp accelerated) ──────────────────────── */

#define FFT_MAX_SIZE    16384
#define FFT_AVG_COUNT   4       /* Average this many FFT frames before output */

/* Configurable dB range */
static float s_db_min = -40.0f;
static float s_db_max =  40.0f;

/* Dynamically allocated, 16-byte aligned buffers */
static float *fft_window  = NULL;   /* [fft_n] */
static float *fft_work    = NULL;   /* [fft_n * 2] interleaved */
static float *fft_power   = NULL;   /* [fft_n] */
static float *fft_input   = NULL;   /* [fft_n * 2] interleaved */

/* Scratch buffers for esp-dsp batch power spectrum */
static float *fft_tmp_a   = NULL;   /* [fft_n] */
static float *fft_tmp_b   = NULL;   /* [fft_n] */

static int      fft_n = 0;
static int      fft_avg_count = 0;
static int      fft_input_pos = 0;

static void fft_free_buffers(void)
{
    if (fft_window) { free(fft_window); fft_window = NULL; }
    if (fft_work)   { free(fft_work);   fft_work   = NULL; }
    if (fft_power)  { free(fft_power);  fft_power  = NULL; }
    if (fft_input)  { free(fft_input);  fft_input  = NULL; }
    if (fft_tmp_a)  { free(fft_tmp_a);  fft_tmp_a  = NULL; }
    if (fft_tmp_b)  { free(fft_tmp_b);  fft_tmp_b  = NULL; }
}

void dsp_fft_init(int fft_size)
{
    /* Clamp to power of 2, min 256, max FFT_MAX_SIZE */
    int n = 1;
    while (n < fft_size && n < FFT_MAX_SIZE) n <<= 1;
    if (n > FFT_MAX_SIZE) n = FFT_MAX_SIZE;
    if (n < 256) n = 256;

    /* Free previous buffers if reinitializing */
    fft_free_buffers();

    fft_n = n;

    /* Allocate 16-byte aligned buffers for PIE SIMD */
    fft_window = heap_caps_aligned_alloc(16, n * sizeof(float), MALLOC_CAP_DEFAULT);
    fft_work   = heap_caps_aligned_alloc(16, n * 2 * sizeof(float), MALLOC_CAP_DEFAULT);
    fft_power  = heap_caps_aligned_alloc(16, n * sizeof(float), MALLOC_CAP_DEFAULT);
    fft_input  = heap_caps_aligned_alloc(16, n * 2 * sizeof(float), MALLOC_CAP_DEFAULT);
    fft_tmp_a  = heap_caps_aligned_alloc(16, n * sizeof(float), MALLOC_CAP_DEFAULT);
    fft_tmp_b  = heap_caps_aligned_alloc(16, n * sizeof(float), MALLOC_CAP_DEFAULT);

    if (!fft_window || !fft_work || !fft_power || !fft_input ||
        !fft_tmp_a || !fft_tmp_b) {
        ESP_LOGE(TAG, "Failed to allocate FFT buffers for size %d", n);
        fft_free_buffers();
        fft_n = 0;
        return;
    }

    /* Generate Hann window using esp-dsp (hardware-optimized) */
    dsps_wind_hann_f32(fft_window, fft_n);

    /* Initialize esp-dsp FFT tables (allocates twiddle factors internally) */
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp-dsp FFT init failed: %s", esp_err_to_name(ret));
        fft_free_buffers();
        fft_n = 0;
        return;
    }

    ESP_LOGI(TAG, "FFT initialized: %d-point, esp-dsp PIE-accelerated", fft_n);
    dsp_fft_reset();
}

void dsp_fft_reset(void)
{
    if (fft_power && fft_n > 0) {
        memset(fft_power, 0, fft_n * sizeof(float));
    }
    fft_avg_count = 0;
    fft_input_pos = 0;
}

int dsp_fft_get_size(void)
{
    return fft_n;
}

void dsp_fft_set_range(float db_min, float db_max)
{
    if (db_min >= db_max) return;
    s_db_min = db_min;
    s_db_max = db_max;
    ESP_LOGI(TAG, "FFT dB range set: %.1f to %.1f", s_db_min, s_db_max);
}

void dsp_fft_compute(const uint8_t *iq_data, uint32_t len,
                     uint8_t *fft_out, int *fft_out_len)
{
    *fft_out_len = 0;

    if (fft_n == 0 || !fft_input || !fft_work || !fft_power) return;

    /* Accumulate IQ samples into interleaved float buffer
     * Format: [re0, im0, re1, im1, ...] as esp-dsp expects */
    uint32_t i = 0;
    while (i + 1 < len) {
        float fi = (iq_data[i]     - 127.5f) / 127.5f;
        float fq = (iq_data[i + 1] - 127.5f) / 127.5f;

        /* Apply window during accumulation */
        float w = fft_window[fft_input_pos];
        fft_input[fft_input_pos * 2]     = fi * w;
        fft_input[fft_input_pos * 2 + 1] = fq * w;
        fft_input_pos++;
        i += 2;

        if (fft_input_pos >= fft_n) {
            /* Copy to work buffer (FFT is in-place) */
            memcpy(fft_work, fft_input, fft_n * 2 * sizeof(float));

            /* Run PIE-accelerated FFT */
            dsps_fft2r_fc32(fft_work, fft_n);

            /* Bit-reverse reorder */
            dsps_bit_rev2r_fc32(fft_work, fft_n);

            /* Batch power spectrum using esp-dsp PIE:
             * Deinterleave into separate re/im arrays, square, sum */
            for (int k = 0; k < fft_n; k++) {
                fft_tmp_a[k] = fft_work[k * 2];
                fft_tmp_b[k] = fft_work[k * 2 + 1];
            }

            /* re^2 (in-place) */
            dsps_mul_f32(fft_tmp_a, fft_tmp_a, fft_tmp_a, fft_n, 1, 1, 1);
            /* im^2 (in-place) */
            dsps_mul_f32(fft_tmp_b, fft_tmp_b, fft_tmp_b, fft_n, 1, 1, 1);
            /* re^2 + im^2 -> fft_tmp_a */
            dsps_add_f32(fft_tmp_a, fft_tmp_b, fft_tmp_a, fft_n, 1, 1, 1);

            /* Accumulate into power spectrum */
            dsps_add_f32(fft_power, fft_tmp_a, fft_power, fft_n, 1, 1, 1);

            fft_avg_count++;
            fft_input_pos = 0;

            /* Output averaged spectrum */
            if (fft_avg_count >= FFT_AVG_COUNT) {
                float inv = 1.0f / fft_avg_count;
                float db_range = s_db_max - s_db_min;
                if (db_range < 1.0f) db_range = 1.0f;

                /* Scale power by 1/avg_count using PIE */
                dsps_mulc_f32(fft_power, fft_power, fft_n, inv, 1, 1);

                for (int k = 0; k < fft_n; k++) {
                    float avg_pwr = fft_power[k];
                    float db;
                    if (avg_pwr < 1e-20f) {
                        db = s_db_min;
                    } else {
                        db = 10.0f * log10f(avg_pwr);
                    }
                    /* Clamp and scale to 0-255 */
                    if (db < s_db_min) db = s_db_min;
                    if (db > s_db_max) db = s_db_max;
                    float scaled = (db - s_db_min) / db_range * 255.0f;

                    /* FFT shift: move DC to center */
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

/* ──────────────────────── DDC (Digital Down Converter) ──────────────────────── */

#define DDC_BATCH_SIZE  1024    /* Batch processing size for esp-dsp ops */

struct dsp_ddc {
    uint32_t    sample_rate;
    int32_t     center_offset;
    uint32_t    output_bw;
    uint32_t    decim_ratio;

    /* NCO (Numerically Controlled Oscillator) -- precomputed, 16-byte aligned */
    float      *nco_re;         /* [DDC_BATCH_SIZE] */
    float      *nco_im;         /* [DDC_BATCH_SIZE] */
    uint32_t    nco_len;
    uint32_t    nco_pos;

    /* Scratch buffers for batch processing, 16-byte aligned */
    float      *buf_re;         /* input I samples */
    float      *buf_im;         /* input Q samples */
    float      *tmp1;           /* product scratch */
    float      *tmp2;           /* product scratch */
    float      *mix_re;         /* mixed output I */
    float      *mix_im;         /* mixed output Q */

    /* CIC-like decimator accumulator */
    float       accum_re;
    float       accum_im;
    uint32_t    accum_count;
};

static float *ddc_alloc(void)
{
    return heap_caps_aligned_alloc(16, DDC_BATCH_SIZE * sizeof(float), MALLOC_CAP_DEFAULT);
}

dsp_ddc_t *dsp_ddc_create(uint32_t sample_rate, uint32_t center_offset_hz,
                           uint32_t output_bw_hz)
{
    dsp_ddc_t *ddc = calloc(1, sizeof(dsp_ddc_t));
    if (!ddc) return NULL;

    ddc->sample_rate = sample_rate;
    ddc->center_offset = (int32_t)center_offset_hz;
    ddc->output_bw = output_bw_hz;

    /* Decimation ratio (nearest power of 2) */
    uint32_t ratio = sample_rate / output_bw_hz;
    if (ratio < 1) ratio = 1;
    uint32_t p2 = 1;
    while (p2 * 2 <= ratio) p2 *= 2;
    ddc->decim_ratio = p2;

    /* Allocate 16-byte aligned buffers for PIE */
    ddc->nco_re = ddc_alloc();
    ddc->nco_im = ddc_alloc();
    ddc->buf_re = ddc_alloc();
    ddc->buf_im = ddc_alloc();
    ddc->tmp1   = ddc_alloc();
    ddc->tmp2   = ddc_alloc();
    ddc->mix_re = ddc_alloc();
    ddc->mix_im = ddc_alloc();

    if (!ddc->nco_re || !ddc->nco_im || !ddc->buf_re || !ddc->buf_im ||
        !ddc->tmp1 || !ddc->tmp2 || !ddc->mix_re || !ddc->mix_im) {
        ESP_LOGE(TAG, "DDC buffer allocation failed");
        dsp_ddc_free(ddc);
        return NULL;
    }

    /* Precompute NCO table for batch processing */
    ddc->nco_len = DDC_BATCH_SIZE;
    double phase_inc = 2.0 * M_PI * (double)center_offset_hz / (double)sample_rate;
    for (uint32_t j = 0; j < ddc->nco_len; j++) {
        double phase = phase_inc * j;
        ddc->nco_re[j] = (float)cos(phase);
        ddc->nco_im[j] = (float)(-sin(phase)); /* Negative for downconversion */
    }
    ddc->nco_pos = 0;

    ESP_LOGI(TAG, "DDC created: offset=%ldHz bw=%luHz decim=%lu",
             (long)ddc->center_offset, (unsigned long)output_bw_hz,
             (unsigned long)ddc->decim_ratio);
    return ddc;
}

void dsp_ddc_free(dsp_ddc_t *ddc)
{
    if (!ddc) return;
    free(ddc->nco_re);
    free(ddc->nco_im);
    free(ddc->buf_re);
    free(ddc->buf_im);
    free(ddc->tmp1);
    free(ddc->tmp2);
    free(ddc->mix_re);
    free(ddc->mix_im);
    free(ddc);
}

int dsp_ddc_process(dsp_ddc_t *ddc, const uint8_t *iq_in, uint32_t in_len,
                    uint8_t *iq_out, uint32_t *out_len)
{
    uint32_t max_out = *out_len;
    uint32_t out_pos = 0;
    uint32_t i = 0;

    while (i + 1 < in_len && out_pos + 1 < max_out) {
        /* Determine batch size */
        uint32_t remaining_pairs = (in_len - i) / 2;
        uint32_t batch = remaining_pairs;
        if (batch > DDC_BATCH_SIZE) batch = DDC_BATCH_SIZE;
        uint32_t nco_remaining = ddc->nco_len - ddc->nco_pos;
        if (batch > nco_remaining) batch = nco_remaining;
        if (batch == 0) break;

        /* Deinterleave uint8 IQ into separate float arrays */
        for (uint32_t j = 0; j < batch; j++) {
            ddc->buf_re[j] = (float)iq_in[i + j * 2];
            ddc->buf_im[j] = (float)iq_in[i + j * 2 + 1];
        }

        /* Batch convert: subtract 127.5, scale by 1/127.5 using PIE */
        dsps_addc_f32(ddc->buf_re, ddc->buf_re, batch, -127.5f, 1, 1);
        dsps_mulc_f32(ddc->buf_re, ddc->buf_re, batch, 1.0f / 127.5f, 1, 1);
        dsps_addc_f32(ddc->buf_im, ddc->buf_im, batch, -127.5f, 1, 1);
        dsps_mulc_f32(ddc->buf_im, ddc->buf_im, batch, 1.0f / 127.5f, 1, 1);

        /* Complex multiply with NCO using PIE:
         * mix_re = buf_re * nco_re - buf_im * nco_im
         * mix_im = buf_re * nco_im + buf_im * nco_re */
        float *nco_r = ddc->nco_re + ddc->nco_pos;
        float *nco_i = ddc->nco_im + ddc->nco_pos;

        /* tmp1 = buf_re * nco_re */
        dsps_mul_f32(ddc->buf_re, nco_r, ddc->tmp1, batch, 1, 1, 1);
        /* tmp2 = buf_im * nco_im */
        dsps_mul_f32(ddc->buf_im, nco_i, ddc->tmp2, batch, 1, 1, 1);
        /* mix_re = tmp1 - tmp2 (negate tmp2, then add) */
        dsps_mulc_f32(ddc->tmp2, ddc->tmp2, batch, -1.0f, 1, 1);
        dsps_add_f32(ddc->tmp1, ddc->tmp2, ddc->mix_re, batch, 1, 1, 1);

        /* tmp1 = buf_re * nco_im */
        dsps_mul_f32(ddc->buf_re, nco_i, ddc->tmp1, batch, 1, 1, 1);
        /* tmp2 = buf_im * nco_re */
        dsps_mul_f32(ddc->buf_im, nco_r, ddc->tmp2, batch, 1, 1, 1);
        /* mix_im = tmp1 + tmp2 */
        dsps_add_f32(ddc->tmp1, ddc->tmp2, ddc->mix_im, batch, 1, 1, 1);

        /* CIC decimation (accumulate and dump) */
        for (uint32_t j = 0; j < batch && out_pos + 1 < max_out; j++) {
            ddc->accum_re += ddc->mix_re[j];
            ddc->accum_im += ddc->mix_im[j];
            ddc->accum_count++;

            if (ddc->accum_count >= ddc->decim_ratio) {
                float o_re = ddc->accum_re / ddc->decim_ratio;
                float o_im = ddc->accum_im / ddc->decim_ratio;

                /* Convert back to uint8 */
                int ival = (int)(o_re * 127.5f + 127.5f + 0.5f);
                int qval = (int)(o_im * 127.5f + 127.5f + 0.5f);
                if (ival < 0) ival = 0;
                if (ival > 255) ival = 255;
                if (qval < 0) qval = 0;
                if (qval > 255) qval = 255;
                iq_out[out_pos]     = (uint8_t)ival;
                iq_out[out_pos + 1] = (uint8_t)qval;
                out_pos += 2;

                ddc->accum_re = 0;
                ddc->accum_im = 0;
                ddc->accum_count = 0;
            }
        }

        ddc->nco_pos = (ddc->nco_pos + batch) % ddc->nco_len;
        i += batch * 2;
    }

    *out_len = out_pos;
    return 0;
}
