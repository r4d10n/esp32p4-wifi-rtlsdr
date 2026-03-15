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
#include "dsp.h"

/* esp-dsp PIE-optimized functions */
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"

static const char *TAG = "dsp";

/* ──────────────────────── FFT Engine (esp-dsp accelerated) ──────────────────────── */

#define FFT_SIZE        1024
#define FFT_AVG_COUNT   4       /* Average this many FFT frames before output */
#define DB_MIN          (-40.0f)
#define DB_MAX          (40.0f)
#define DB_RANGE        (DB_MAX - DB_MIN)

/* All buffers aligned to 16 bytes for PIE 128-bit access */
static float __attribute__((aligned(16))) fft_window[FFT_SIZE];
static float __attribute__((aligned(16))) fft_work[FFT_SIZE * 2];    /* interleaved [re,im,re,im,...] */
static float __attribute__((aligned(16))) fft_power[FFT_SIZE];
static int      fft_n = FFT_SIZE;
static int      fft_avg_count = 0;
static int      fft_input_pos = 0;
/* Accumulate IQ as interleaved float pairs for esp-dsp format */
static float __attribute__((aligned(16))) fft_input[FFT_SIZE * 2];

void dsp_fft_init(int fft_size)
{
    if (fft_size > FFT_SIZE) fft_size = FFT_SIZE;
    /* Clamp to power of 2 */
    int n = 1;
    while (n < fft_size) n <<= 1;
    if (n > FFT_SIZE) n = FFT_SIZE;
    fft_n = n;

    /* Generate Hann window using esp-dsp (hardware-optimized) */
    dsps_wind_hann_f32(fft_window, fft_n);

    /* Initialize esp-dsp FFT tables (allocates twiddle factors internally) */
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp-dsp FFT init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "FFT initialized: %d-point, esp-dsp PIE-accelerated", fft_n);
    dsp_fft_reset();
}

void dsp_fft_reset(void)
{
    memset(fft_power, 0, sizeof(fft_power));
    fft_avg_count = 0;
    fft_input_pos = 0;
}

void dsp_fft_compute(const uint8_t *iq_data, uint32_t len,
                     uint8_t *fft_out, int *fft_out_len)
{
    *fft_out_len = 0;

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

            /* Run PIE-accelerated FFT
             * dsps_fft2r_fc32 auto-selects arp4 (PIE) on ESP32-P4 */
            dsps_fft2r_fc32(fft_work, fft_n);

            /* Bit-reverse reorder */
            dsps_bit_rev2r_fc32(fft_work, fft_n);

            /* Accumulate power spectrum */
            for (int k = 0; k < fft_n; k++) {
                float re = fft_work[k * 2];
                float im = fft_work[k * 2 + 1];
                fft_power[k] += re * re + im * im;
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
                    float scaled = (db - DB_MIN) / DB_RANGE * 255.0f;

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

struct dsp_ddc {
    uint32_t    sample_rate;
    int32_t     center_offset;
    uint32_t    output_bw;
    uint32_t    decim_ratio;

    /* NCO (Numerically Controlled Oscillator) — precomputed for PIE alignment */
    float __attribute__((aligned(16))) nco_re[FFT_SIZE];
    float __attribute__((aligned(16))) nco_im[FFT_SIZE];
    uint32_t    nco_len;        /* Length of one NCO period */
    uint32_t    nco_pos;        /* Current position in NCO table */

    /* CIC-like decimator accumulator */
    float       accum_re;
    float       accum_im;
    uint32_t    accum_count;
};

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
    /* Round down to power of 2 */
    uint32_t p2 = 1;
    while (p2 * 2 <= ratio) p2 *= 2;
    ddc->decim_ratio = p2;

    /* Precompute NCO table for one period
     * We compute enough samples for efficient batch processing */
    ddc->nco_len = FFT_SIZE;  /* Use full FFT_SIZE for batch processing */
    double phase_inc = 2.0 * M_PI * (double)center_offset_hz / (double)sample_rate;
    for (uint32_t i = 0; i < ddc->nco_len; i++) {
        double phase = phase_inc * i;
        ddc->nco_re[i] = (float)cos(phase);
        ddc->nco_im[i] = (float)(-sin(phase)); /* Negative for downconversion */
    }
    ddc->nco_pos = 0;
    ddc->accum_re = 0;
    ddc->accum_im = 0;
    ddc->accum_count = 0;

    ESP_LOGI(TAG, "DDC created: offset=%ldHz bw=%luHz decim=%lu",
             (long)ddc->center_offset, (unsigned long)output_bw_hz,
             (unsigned long)ddc->decim_ratio);
    return ddc;
}

void dsp_ddc_free(dsp_ddc_t *ddc)
{
    free(ddc);
}

int dsp_ddc_process(dsp_ddc_t *ddc, const uint8_t *iq_in, uint32_t in_len,
                    uint8_t *iq_out, uint32_t *out_len)
{
    uint32_t max_out = *out_len;
    uint32_t out_pos = 0;

    for (uint32_t i = 0; i + 1 < in_len && out_pos + 1 < max_out; i += 2) {
        /* Convert uint8 IQ to float */
        float in_re = (iq_in[i]     - 127.5f) / 127.5f;
        float in_im = (iq_in[i + 1] - 127.5f) / 127.5f;

        /* Complex multiply with NCO (frequency shift) */
        float nco_r = ddc->nco_re[ddc->nco_pos];
        float nco_i = ddc->nco_im[ddc->nco_pos];
        float mix_re = in_re * nco_r - in_im * nco_i;
        float mix_im = in_re * nco_i + in_im * nco_r;

        ddc->nco_pos = (ddc->nco_pos + 1) % ddc->nco_len;

        /* Accumulate for CIC decimation */
        ddc->accum_re += mix_re;
        ddc->accum_im += mix_im;
        ddc->accum_count++;

        /* Output decimated sample */
        if (ddc->accum_count >= ddc->decim_ratio) {
            float out_re = ddc->accum_re / ddc->decim_ratio;
            float out_im = ddc->accum_im / ddc->decim_ratio;

            /* Convert back to uint8 */
            iq_out[out_pos]     = (uint8_t)((out_re * 127.5f) + 127.5f);
            iq_out[out_pos + 1] = (uint8_t)((out_im * 127.5f) + 127.5f);
            out_pos += 2;

            ddc->accum_re = 0;
            ddc->accum_im = 0;
            ddc->accum_count = 0;
        }
    }

    *out_len = out_pos;
    return 0;
}
