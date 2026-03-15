/*
 * FFT Benchmark: Compare all ESP32-P4 FFT implementations
 *
 * Tests: dsps_fft2r_fc32 (float radix-2)
 *        dsps_fft4r_fc32 (float radix-4)
 *        dsps_fft2r_sc16 (int16 radix-2 PIE SIMD)
 *
 * Run on device via: add this to main.c and call bench_fft_all()
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dsps_fft2r.h"
#include "dsps_fft4r.h"
#include "dsps_wind_hann.h"

static const char *TAG = "bench_fft";

#define BENCH_ITERATIONS 100

static void fill_test_signal(float *data, int n)
{
    /* Generate test signal: tone + noise */
    for (int i = 0; i < n; i++) {
        float phase = 2.0f * (float)M_PI * 100.0f * i / n;
        data[i * 2]     = sinf(phase) * 0.5f + ((float)(i % 7) / 7.0f - 0.5f) * 0.1f;
        data[i * 2 + 1] = cosf(phase) * 0.5f + ((float)(i % 11) / 11.0f - 0.5f) * 0.1f;
    }
}

static void fill_test_signal_sc16(int16_t *data, int n)
{
    for (int i = 0; i < n; i++) {
        float phase = 2.0f * (float)M_PI * 100.0f * i / n;
        data[i * 2]     = (int16_t)(sinf(phase) * 16000.0f);
        data[i * 2 + 1] = (int16_t)(cosf(phase) * 16000.0f);
    }
}

void bench_fft_all(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  ESP32-P4 FFT Benchmark — All Implementations        ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "  Iterations per test: %d", BENCH_ITERATIONS);

    int sizes[] = {256, 512, 1024, 2048, 4096};
    int num_sizes = sizeof(sizes) / sizeof(sizes[0]);

    /* Initialize FFT tables for all methods */
    dsps_fft2r_init_fc32(NULL, 4096);
    dsps_fft4r_init_fc32(NULL, 4096);
    dsps_fft2r_init_sc16(NULL, 4096);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  %-6s │ %12s │ %12s │ %12s │ %8s",
             "Size", "Radix2-FC32", "Radix4-FC32", "Radix2-SC16", "SC16 vs R2F");
    ESP_LOGI(TAG, "  ────── │ ──────────── │ ──────────── │ ──────────── │ ─────────");

    for (int si = 0; si < num_sizes; si++) {
        int n = sizes[si];

        /* Allocate aligned buffers */
        float *fc32_data = heap_caps_aligned_alloc(16, n * 2 * sizeof(float), MALLOC_CAP_DEFAULT);
        float *fc32_data4 = heap_caps_aligned_alloc(16, n * 2 * sizeof(float), MALLOC_CAP_DEFAULT);
        int16_t *sc16_data = heap_caps_aligned_alloc(16, n * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);

        if (!fc32_data || !fc32_data4 || !sc16_data) {
            ESP_LOGE(TAG, "  %d: alloc failed", n);
            free(fc32_data); free(fc32_data4); free(sc16_data);
            continue;
        }

        /* ── Benchmark 1: dsps_fft2r_fc32 (radix-2 float) ── */
        int64_t t_start, t_end;
        uint64_t sum_r2f = 0;
        for (int iter = 0; iter < BENCH_ITERATIONS; iter++) {
            fill_test_signal(fc32_data, n);
            t_start = esp_timer_get_time();
            dsps_fft2r_fc32(fc32_data, n);
            dsps_bit_rev2r_fc32(fc32_data, n);
            t_end = esp_timer_get_time();
            sum_r2f += (t_end - t_start);
        }
        uint64_t avg_r2f = sum_r2f / BENCH_ITERATIONS;

        /* ── Benchmark 2: dsps_fft4r_fc32 (radix-4 float) ── */
        uint64_t sum_r4f = 0;
        /* Radix-4 requires N to be power of 4 */
        int is_pow4 = (n == 256 || n == 1024 || n == 4096);
        if (is_pow4) {
            for (int iter = 0; iter < BENCH_ITERATIONS; iter++) {
                fill_test_signal(fc32_data4, n);
                t_start = esp_timer_get_time();
                dsps_fft4r_fc32(fc32_data4, n);
                dsps_bit_rev4r_fc32(fc32_data4, n);
                t_end = esp_timer_get_time();
                sum_r4f += (t_end - t_start);
            }
        }
        uint64_t avg_r4f = is_pow4 ? sum_r4f / BENCH_ITERATIONS : 0;

        /* ── Benchmark 3: dsps_fft2r_sc16 (radix-2 int16 PIE) ── */
        uint64_t sum_sc16 = 0;
        for (int iter = 0; iter < BENCH_ITERATIONS; iter++) {
            fill_test_signal_sc16(sc16_data, n);
            t_start = esp_timer_get_time();
            dsps_fft2r_sc16(sc16_data, n);
            dsps_bit_rev_sc16_ansi(sc16_data, n);
            t_end = esp_timer_get_time();
            sum_sc16 += (t_end - t_start);
        }
        uint64_t avg_sc16 = sum_sc16 / BENCH_ITERATIONS;

        /* Report */
        float speedup = (float)avg_r2f / (float)(avg_sc16 > 0 ? avg_sc16 : 1);

        if (is_pow4) {
            ESP_LOGI(TAG, "  %-6d │ %8llu µs  │ %8llu µs  │ %8llu µs  │ %5.1fx",
                     n,
                     (unsigned long long)avg_r2f,
                     (unsigned long long)avg_r4f,
                     (unsigned long long)avg_sc16,
                     speedup);
        } else {
            ESP_LOGI(TAG, "  %-6d │ %8llu µs  │     N/A      │ %8llu µs  │ %5.1fx",
                     n,
                     (unsigned long long)avg_r2f,
                     (unsigned long long)avg_sc16,
                     speedup);
        }

        free(fc32_data);
        free(fc32_data4);
        free(sc16_data);
    }

    ESP_LOGI(TAG, "  ────── │ ──────────── │ ──────────── │ ──────────── │ ─────────");
    ESP_LOGI(TAG, "  R2-FC32 = dsps_fft2r_fc32 (hw loop, scalar FPU)");
    ESP_LOGI(TAG, "  R4-FC32 = dsps_fft4r_fc32 (hw loop, scalar FPU, radix-4)");
    ESP_LOGI(TAG, "  R2-SC16 = dsps_fft2r_sc16 (PIE SIMD, 8×int16 per cycle)");
    ESP_LOGI(TAG, "");

    dsps_fft2r_deinit_fc32();
    dsps_fft4r_deinit_fc32();
    dsps_fft2r_deinit_sc16();
}
