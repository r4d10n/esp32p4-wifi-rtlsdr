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
#include "esp_timer.h"
#include "dsp.h"
#include "pie_kernels.h"

/* ──────────────────────── Performance Counters ──────────────────────── */
/* Enable DSP_PERF_BENCH for cycle-accurate benchmarking, disable for production */
#define DSP_PERF_BENCH 0  /* Set to 1 for benchmarking */

#if DSP_PERF_BENCH
static uint32_t perf_fft_count = 0;
static uint64_t perf_fft_total_us = 0;
static uint64_t perf_power_total_us = 0;
static uint64_t perf_db_total_us = 0;
#define PERF_LOG_INTERVAL 200  /* Log every N FFT output frames */
#define PERF_START(var) int64_t var = esp_timer_get_time()
#define PERF_END(var, accum) accum += (esp_timer_get_time() - var)
#else
#define PERF_START(var) (void)0
#define PERF_END(var, accum) (void)0
#endif

/* esp-dsp functions — only FFT and window generation still needed */
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"

static const char *TAG = "dsp";

/* ──────────────────────── FFT Engine ──────────────────────── */

/* Max FFT size limited by internal RAM for aligned int16 buffers.
 * 8192: 8192×2×2 = 32KB per buffer, 3 buffers = 96KB — fits.
 * 16384: 16384×2×2 = 64KB per buffer, 3 buffers = 192KB — heap fragmentation risk.
 * Use 8192 as safe max. For 16384, would need PSRAM-backed buffers. */
#define FFT_MAX_SIZE    8192
#define FFT_AVG_COUNT   4       /* Average this many FFT frames before output */

/* Configurable dB range
 * Note: int16 FFT with <<8 scaling produces power values ~48 dB higher
 * than float FFT normalized to [-1,1]. Adjust range accordingly. */
static float s_db_min =  10.0f;
static float s_db_max =  90.0f;

/* Dynamically allocated, 16-byte aligned buffers */
static int32_t *fft_power   = NULL;   /* [fft_n] — accumulated power spectrum (int32) */

/* INT16 FFT path (PIE SIMD accelerated — 4.3-4.8x faster than float) */
static int16_t *fft_sc16_window = NULL;  /* [fft_n] — Hann window as Q15 */
static int16_t *fft_sc16_input  = NULL;  /* [fft_n * 2] — interleaved I,Q */
static int16_t *fft_sc16_work   = NULL;  /* [fft_n * 2] — FFT work buffer */

static int      fft_n = 0;
static int      fft_avg_count = 0;
static int      fft_input_pos = 0;

static void fft_free_buffers(void)
{
    if (fft_power)       { free(fft_power);       fft_power       = NULL; }
    if (fft_sc16_window) { free(fft_sc16_window); fft_sc16_window = NULL; }
    if (fft_sc16_input)  { free(fft_sc16_input);  fft_sc16_input  = NULL; }
    if (fft_sc16_work)   { free(fft_sc16_work);   fft_sc16_work   = NULL; }
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

    /* Allocate 16-byte aligned buffers */
    fft_power       = heap_caps_aligned_alloc(16, n * sizeof(int32_t), MALLOC_CAP_DEFAULT);
    fft_sc16_window = heap_caps_aligned_alloc(16, n * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    fft_sc16_input  = heap_caps_aligned_alloc(16, n * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    fft_sc16_work   = heap_caps_aligned_alloc(16, n * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    if (!fft_power || !fft_sc16_window || !fft_sc16_input || !fft_sc16_work) {
        ESP_LOGE(TAG, "Failed to allocate FFT buffers for size %d", n);
        fft_free_buffers();
        fft_n = 0;
        return;
    }

    /* Generate Hann window as Q15 int16 for PIE sc16 FFT path
     * Use esp-dsp to generate float, then convert to int16 */
    {
        float *tmp_win = malloc(n * sizeof(float));
        if (tmp_win) {
            dsps_wind_hann_f32(tmp_win, n);
            for (int j = 0; j < n; j++) {
                fft_sc16_window[j] = (int16_t)(tmp_win[j] * 32767.0f);
            }
            free(tmp_win);
        }
    }

    /* Initialize esp-dsp FFT tables for int16 path */
    esp_err_t ret = dsps_fft2r_init_sc16(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp-dsp sc16 FFT init failed: %s", esp_err_to_name(ret));
        fft_free_buffers();
        fft_n = 0;
        return;
    }

    ESP_LOGI(TAG, "FFT initialized: %d-point, int16 PIE SIMD (4.3-4.8x faster)", fft_n);
    dsp_fft_reset();
}

void dsp_fft_reset(void)
{
    if (fft_power && fft_n > 0) {
        memset(fft_power, 0, fft_n * sizeof(int32_t));
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

    if (fft_n == 0 || !fft_sc16_input || !fft_sc16_work || !fft_power) return;

    /* Accumulate IQ samples into int16 interleaved buffer using SIMD-friendly batch conversion */
    uint32_t i = 0;
    while (i + 1 < len) {
        /* How many IQ pairs can we accept before FFT buffer is full? */
        uint32_t pairs_available = (len - i) / 2;
        uint32_t pairs_needed = fft_n - fft_input_pos;
        uint32_t batch = pairs_available < pairs_needed ? pairs_available : pairs_needed;

        /* Round down to multiple of 8 for SIMD, process remainder one-by-one */
        uint32_t batch_simd = batch & ~7u;

        if (batch_simd > 0) {
            pie_u8iq_to_s16_windowed(
                iq_data + i,
                fft_sc16_window + fft_input_pos,
                fft_sc16_input + fft_input_pos * 2,
                batch_simd);
            fft_input_pos += batch_simd;
            i += batch_simd * 2;
        }

        /* Handle remaining 0-7 pairs with scalar fallback */
        uint32_t remainder = batch - batch_simd;
        for (uint32_t r = 0; r < remainder; r++) {
            int16_t si = ((int16_t)iq_data[i] - 128) << 8;
            int16_t sq = ((int16_t)iq_data[i + 1] - 128) << 8;
            int32_t wi = fft_sc16_window[fft_input_pos];
            fft_sc16_input[fft_input_pos * 2]     = (int16_t)((si * wi) >> 15);
            fft_sc16_input[fft_input_pos * 2 + 1] = (int16_t)((sq * wi) >> 15);
            fft_input_pos++;
            i += 2;
        }

        if (fft_input_pos >= fft_n) {
            /* Copy to work buffer (FFT is in-place) */
            memcpy(fft_sc16_work, fft_sc16_input, fft_n * 2 * sizeof(int16_t));

            PERF_START(t_fft);

            /* Run PIE SIMD int16 FFT — 4.3-4.8x faster than float! */
            dsps_fft2r_sc16(fft_sc16_work, fft_n);

            /* Bit-reverse reorder */
            dsps_bit_rev_sc16_ansi(fft_sc16_work, fft_n);

            PERF_END(t_fft, perf_fft_total_us);

            PERF_START(t_pwr);

            /* Power spectrum: accumulate re^2 + im^2 into int32 buffer */
            pie_power_spectrum_accumulate(fft_sc16_work, fft_power, fft_n);

            PERF_END(t_pwr, perf_power_total_us);

            fft_avg_count++;
            fft_input_pos = 0;

            /* Output averaged spectrum */
            if (fft_avg_count >= FFT_AVG_COUNT) {
                PERF_START(t_db);

                /* Convert int32 power -> dB -> uint8 with FFT shift */
                pie_power_to_db_u8(fft_power, fft_out, fft_n,
                                   fft_avg_count, s_db_min, s_db_max);
                *fft_out_len = fft_n;

                PERF_END(t_db, perf_db_total_us);

#if DSP_PERF_BENCH
                perf_fft_count++;
                if (perf_fft_count % PERF_LOG_INTERVAL == 0) {
                    uint32_t n = perf_fft_count;
                    ESP_LOGI(TAG, "DSP BENCH (%lu frames, %d-pt): "
                             "FFT=%lluus/frame Power=%lluus/frame dB+shift=%lluus/frame "
                             "Total=%lluus/frame",
                             (unsigned long)n, fft_n,
                             (unsigned long long)(perf_fft_total_us * FFT_AVG_COUNT / n),
                             (unsigned long long)(perf_power_total_us * FFT_AVG_COUNT / n),
                             (unsigned long long)(perf_db_total_us / n),
                             (unsigned long long)((perf_fft_total_us + perf_power_total_us) * FFT_AVG_COUNT / n + perf_db_total_us / n));
                }
#endif

                /* Reset accumulator */
                memset(fft_power, 0, fft_n * sizeof(int32_t));
                fft_avg_count = 0;
            }
        }
    }
}

/* ──────────────────────── DDC (Digital Down Converter) ──────────────────────── */

#define DDC_BATCH_SIZE  1024    /* Must be multiple of 8 */

struct dsp_ddc {
    uint32_t    sample_rate;
    int32_t     center_offset;
    uint32_t    output_bw;
    uint32_t    decim_ratio;

    /* NCO for int16 Q15 mixing */
    pie_nco_t  *nco;

    /* Int16 scratch buffers (16-byte aligned) */
    int16_t    *buf_s16;       /* uint8 -> int16 conversion [DDC_BATCH_SIZE * 2] */
    int16_t    *mix_s16;       /* NCO mixed output [DDC_BATCH_SIZE * 2] */

    /* CIC decimator persistent state */
    int32_t     cic_accum[4];  /* {accum_re, accum_im, count, 0} */
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
    uint32_t p2 = 1;
    while (p2 * 2 <= ratio) p2 *= 2;
    ddc->decim_ratio = p2;

    /* Create int16 Q15 NCO table */
    ddc->nco = pie_nco_create(sample_rate, (int32_t)center_offset_hz);
    if (!ddc->nco) {
        free(ddc);
        return NULL;
    }

    /* Allocate int16 scratch buffers (interleaved IQ, so *2) */
    ddc->buf_s16 = heap_caps_aligned_alloc(16, DDC_BATCH_SIZE * 2 * sizeof(int16_t),
                                            MALLOC_CAP_DEFAULT);
    ddc->mix_s16 = heap_caps_aligned_alloc(16, DDC_BATCH_SIZE * 2 * sizeof(int16_t),
                                            MALLOC_CAP_DEFAULT);

    if (!ddc->buf_s16 || !ddc->mix_s16) {
        ESP_LOGE(TAG, "DDC buffer allocation failed");
        dsp_ddc_free(ddc);
        return NULL;
    }

    memset(ddc->cic_accum, 0, sizeof(ddc->cic_accum));

    ESP_LOGI(TAG, "DDC created: offset=%ldHz bw=%luHz decim=%lu (int16 pipeline)",
             (long)ddc->center_offset, (unsigned long)output_bw_hz,
             (unsigned long)ddc->decim_ratio);
    return ddc;
}

uint32_t dsp_ddc_get_output_rate(dsp_ddc_t *ddc)
{
    if (!ddc || ddc->decim_ratio == 0) return 0;
    return ddc->sample_rate / ddc->decim_ratio;
}

void dsp_ddc_free(dsp_ddc_t *ddc)
{
    if (!ddc) return;
    pie_nco_free(ddc->nco);
    free(ddc->buf_s16);
    free(ddc->mix_s16);
    free(ddc);
}

int dsp_ddc_process(dsp_ddc_t *ddc, const uint8_t *iq_in, uint32_t in_len,
                    uint8_t *iq_out, uint32_t *out_len)
{
    uint32_t max_out = *out_len;
    uint32_t out_pos = 0;
    uint32_t i = 0;

    /* Temporary int16 output from CIC before converting to uint8 */
    int16_t *cic_out = ddc->buf_s16;  /* Reuse buf_s16 since we're done with input by then */

    while (i + 1 < in_len && out_pos + 1 < max_out) {
        /* Determine batch size */
        uint32_t remaining_pairs = (in_len - i) / 2;
        uint32_t batch = remaining_pairs;
        if (batch > DDC_BATCH_SIZE) batch = DDC_BATCH_SIZE;

        /* Round down to multiple of 8 for SIMD */
        batch = batch & ~7u;
        if (batch == 0) {
            /* Handle remaining < 8 pairs with scalar fallback */
            break;
        }

        /* Step 1: uint8 IQ -> int16 with bias removal */
        pie_u8_to_s16_bias(iq_in + i, ddc->buf_s16, batch * 2);

        /* Step 2: Complex multiply with NCO (frequency shift) */
        pie_nco_mix_s16(ddc->buf_s16, ddc->nco, ddc->mix_s16, batch);

        /* Step 3: CIC decimation */
        int cic_out_pairs = (int)((max_out - out_pos) / 2);
        pie_cic_decimate_s16(ddc->mix_s16, batch,
                              cic_out, &cic_out_pairs,
                              ddc->decim_ratio, ddc->cic_accum);

        /* Step 4: int16 -> uint8 output conversion */
        if (cic_out_pairs > 0) {
            pie_s16_to_u8(cic_out, iq_out + out_pos, cic_out_pairs * 2);
            out_pos += cic_out_pairs * 2;
        }

        i += batch * 2;
    }

    *out_len = out_pos;
    return 0;
}
