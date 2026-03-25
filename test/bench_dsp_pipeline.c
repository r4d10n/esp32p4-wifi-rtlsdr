/*
 * DSP Pipeline Benchmark Suite
 *
 * Measures cycle counts for each DSP stage in the FM radio pipeline.
 * Compares scalar C implementations vs PIE SIMD where applicable.
 *
 * Enable with CONFIG_FM_BENCH_ENABLE=y, runs at startup before main loop.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "pie_kernels.h"
#include "pie_fm_discriminator.h"
#include "polyphase_resamp.h"

static const char *TAG = "bench";

#define BENCH_SAMPLES   4096    /* Samples per benchmark iteration */
#define BENCH_ITERS     100     /* Iterations for averaging */

/* Generate test IQ data: FM-modulated 1 kHz tone */
static void gen_test_iq(int16_t *iq, int n_pairs, float fm_deviation, float sample_rate)
{
    float phase = 0;
    float mod_phase = 0;
    float mod_inc = 2.0f * 3.14159f * 1000.0f / sample_rate; /* 1 kHz modulation */
    float dev_per_sample = 2.0f * 3.14159f * fm_deviation / sample_rate;

    for (int k = 0; k < n_pairs; k++) {
        float fm = sinf(mod_phase) * dev_per_sample;
        phase += fm;
        iq[k * 2]     = (int16_t)(cosf(phase) * 16384);  /* I */
        iq[k * 2 + 1] = (int16_t)(sinf(phase) * 16384);  /* Q */
        mod_phase += mod_inc;
    }
}

typedef struct {
    const char *name;
    int64_t     total_us;
    int         iterations;
    int         samples_per_iter;
} bench_result_t;

#define MAX_RESULTS 20
static bench_result_t results[MAX_RESULTS];
static int n_results = 0;

static void record_result(const char *name, int64_t total_us, int iters, int samples)
{
    if (n_results < MAX_RESULTS) {
        results[n_results].name = name;
        results[n_results].total_us = total_us;
        results[n_results].iterations = iters;
        results[n_results].samples_per_iter = samples;
        n_results++;
    }
}

/* ── Individual Benchmarks ── */

static void bench_u8_to_s16_bias(void)
{
    uint8_t *u8_buf = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2, MALLOC_CAP_DEFAULT);
    int16_t *s16_buf = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    /* Fill with random-ish data */
    for (int i = 0; i < BENCH_SAMPLES * 2; i++) u8_buf[i] = (uint8_t)(i & 0xFF);

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        pie_u8_to_s16_bias(u8_buf, s16_buf, BENCH_SAMPLES * 2);
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("u8_to_s16_bias", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    free(u8_buf); free(s16_buf);
}

static void bench_nco_mix(void)
{
    int16_t *iq_in  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_out = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    pie_nco_t *nco = pie_nco_create(256000, 10000); /* 10 kHz offset */

    gen_test_iq(iq_in, BENCH_SAMPLES, 75000, 256000);

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        nco->phase_acc = 0; /* Reset for consistency */
        pie_nco_mix_s16(iq_in, nco, iq_out, BENCH_SAMPLES);
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("nco_mix_s16 (PIE cmul)", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    pie_nco_free(nco); free(iq_in); free(iq_out);
}

static void bench_cic_decimate(void)
{
    int16_t *iq_in  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_out = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int32_t cic_state[26];

    gen_test_iq(iq_in, BENCH_SAMPLES, 75000, 1024000);

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        memset(cic_state, 0, sizeof(cic_state));
        int out_pairs = BENCH_SAMPLES;
        pie_cic_decimate_s16(iq_in, BENCH_SAMPLES, iq_out, &out_pairs, 4, cic_state);
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("cic_decimate_s16 (R=4)", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    free(iq_in); free(iq_out);
}

static void bench_fm_discriminator_poly(void)
{
    int16_t *iq_in  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    fm_disc_state_t disc;

    gen_test_iq(iq_in, BENCH_SAMPLES, 75000, 256000);
    fm_disc_init(&disc, FM_DISC_POLY_CORRECTED, 256000, 75000);

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        fm_disc_reset(&disc);
        fm_disc_process(&disc, iq_in, audio, BENCH_SAMPLES);
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("fm_disc POLY (<0.3% THD)", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    free(iq_in); free(audio);
}

static void bench_fm_discriminator_linear(void)
{
    int16_t *iq_in  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    fm_disc_state_t disc;

    gen_test_iq(iq_in, BENCH_SAMPLES, 75000, 256000);
    fm_disc_init(&disc, FM_DISC_FAST_LINEAR, 256000, 75000);

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        fm_disc_reset(&disc);
        fm_disc_process(&disc, iq_in, audio, BENCH_SAMPLES);
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("fm_disc LINEAR (~1% THD)", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    free(iq_in); free(audio);
}

static void bench_fir_filter(void)
{
    /* 63-tap FIR with SIMD */
    int16_t taps[64];
    for (int i = 0; i < 64; i++) taps[i] = (int16_t)(i * 100); /* Dummy taps */

    int16_t *input  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *output = heap_caps_aligned_alloc(16, BENCH_SAMPLES * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    for (int i = 0; i < BENCH_SAMPLES; i++) input[i] = (int16_t)(i * 7);

    pie_fir_state_t *fir = pie_fir_create(taps, 63);
    if (!fir) { ESP_LOGE(TAG, "FIR create failed"); free(input); free(output); return; }

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        pie_fir_reset(fir);
        pie_fir_process(fir, input, output, BENCH_SAMPLES);
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("fir_s16 63-tap (PIE xacc)", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    pie_fir_free(fir); free(input); free(output);
}

static void bench_volume(void)
{
    int16_t *audio = heap_caps_aligned_alloc(16, BENCH_SAMPLES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    for (int i = 0; i < BENCH_SAMPLES; i++) audio[i] = (int16_t)(i * 3);

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        pie_volume_s16(audio, BENCH_SAMPLES, 23000); /* ~70% volume in Q15 */
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("volume_s16 (PIE vmul)", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    free(audio);
}

static void bench_polyphase_resamp(void)
{
    int16_t *input  = heap_caps_aligned_alloc(16, BENCH_SAMPLES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *output = heap_caps_aligned_alloc(16, BENCH_SAMPLES * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    for (int i = 0; i < BENCH_SAMPLES; i++) input[i] = (int16_t)(sinf(2.0f * 3.14159f * 1000.0f * i / 256000.0f) * 16384);

    polyphase_resamp_t *r = polyphase_resamp_create(256000, 48000, 16);
    if (!r) { ESP_LOGE(TAG, "Resampler create failed"); free(input); free(output); return; }

    int64_t start = esp_timer_get_time();
    for (int iter = 0; iter < BENCH_ITERS; iter++) {
        polyphase_resamp_reset(r);
        polyphase_resamp_process(r, input, BENCH_SAMPLES, output, BENCH_SAMPLES);
    }
    int64_t elapsed = esp_timer_get_time() - start;

    record_result("polyphase 256k->48k", elapsed, BENCH_ITERS, BENCH_SAMPLES);
    polyphase_resamp_free(r); free(input); free(output);
}

/* ── Print Results ── */

static void print_results(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              DSP Pipeline Benchmark Results                          ║");
    ESP_LOGI(TAG, "║  %d samples x %d iterations, ESP32-P4 @ 400 MHz                    ║", BENCH_SAMPLES, BENCH_ITERS);
    ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ %-28s | %8s | %8s | %8s ║", "Stage", "us/iter", "cyc/samp", "MSPS");
    ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════╣");

    for (int i = 0; i < n_results; i++) {
        bench_result_t *r = &results[i];
        float us_per_iter = (float)r->total_us / r->iterations;
        float cyc_per_sample = us_per_iter * 400.0f / r->samples_per_iter; /* 400 MHz */
        float msps = (float)r->samples_per_iter / us_per_iter; /* M samples/sec */

        ESP_LOGI(TAG, "║ %-28s | %8.1f | %8.1f | %8.2f ║",
                 r->name, us_per_iter, cyc_per_sample, msps);
    }

    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
}

/* ── Public Entry Point ── */

void bench_dsp_pipeline(void)
{
    ESP_LOGI(TAG, "Starting DSP pipeline benchmarks (%d samples x %d iters)...",
             BENCH_SAMPLES, BENCH_ITERS);

    n_results = 0;

    bench_u8_to_s16_bias();
    bench_nco_mix();
    bench_cic_decimate();
    bench_fm_discriminator_poly();
    bench_fm_discriminator_linear();
    bench_fir_filter();
    bench_volume();
    bench_polyphase_resamp();

    print_results();

    ESP_LOGI(TAG, "Benchmarks complete.");
}
