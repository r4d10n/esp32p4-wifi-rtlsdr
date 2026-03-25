/*
 * End-to-End Pipeline Integration Test
 *
 * Generates complete RTL-SDR-format uint8 IQ data with FM-modulated content,
 * runs through the full DSP pipeline, and validates audio output.
 * Tests clean signal, noisy signal, impulse noise, frequency offset, and mode switching.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "pie_kernels.h"
#include "fm_demod.h"

static const char *TAG = "test_e2e";
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { ESP_LOGE(TAG, "FAIL: %s (line %d)", msg, __LINE__); tests_failed++; } \
    else { ESP_LOGI(TAG, "PASS: %s", msg); tests_passed++; } \
} while(0)

/*
 * Generate RTL-SDR format uint8 IQ data with FM-modulated tone.
 * This simulates what the RTL-SDR USB driver produces.
 * @param iq_out     uint8 interleaved IQ [I0,Q0,I1,Q1,...]
 * @param n_pairs    Number of IQ pairs
 * @param carrier_offset  Carrier frequency offset from center in Hz
 * @param mod_freq   Modulation tone frequency in Hz
 * @param deviation  FM deviation in Hz
 * @param sample_rate SDR sample rate
 * @param amplitude  Signal amplitude 0-127 (centered at 128)
 */
static void generate_rtlsdr_iq(uint8_t *iq_out, int n_pairs,
                                 float carrier_offset, float mod_freq,
                                 float deviation, float sample_rate,
                                 int amplitude)
{
    float carrier_phase = 0;
    float mod_phase = 0;
    float carrier_inc = 2.0f * M_PI * carrier_offset / sample_rate;
    float mod_inc = 2.0f * M_PI * mod_freq / sample_rate;
    float dev_per_sample = 2.0f * M_PI * deviation / sample_rate;

    for (int k = 0; k < n_pairs; k++) {
        float fm_inst = sinf(mod_phase) * dev_per_sample;
        carrier_phase += carrier_inc + fm_inst;

        float i = cosf(carrier_phase) * amplitude;
        float q = sinf(carrier_phase) * amplitude;

        iq_out[k * 2]     = (uint8_t)(i + 128);
        iq_out[k * 2 + 1] = (uint8_t)(q + 128);
        mod_phase += mod_inc;
    }
}

static void add_white_noise_u8(uint8_t *iq, int count, int noise_amplitude)
{
    uint32_t rng = 54321;
    for (int i = 0; i < count; i++) {
        rng = rng * 1103515245 + 12345;
        int noise = ((int)(rng >> 16) - 32768) * noise_amplitude / 32768;
        int val = (int)iq[i] + noise;
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        iq[i] = (uint8_t)val;
    }
}

/*
 * Run the complete pipeline on uint8 IQ data and return audio output.
 * Mimics what main.c's fm_pipeline_task does.
 */
static int run_full_pipeline(const uint8_t *u8_iq, int n_pairs,
                               int16_t *audio_out, int audio_max,
                               fm_demod_mode_t mode)
{
    /* Allocate aligned buffers */
    int16_t *iq_s16 = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_mixed = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_decim = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int32_t cic_state[26] = {0};

    if (!iq_s16 || !iq_mixed || !iq_decim) {
        ESP_LOGE(TAG, "Buffer allocation failed");
        heap_caps_free(iq_s16);
        heap_caps_free(iq_mixed);
        heap_caps_free(iq_decim);
        return 0;
    }

    /* Step 1: uint8 -> int16 bias removal */
    pie_u8_to_s16_bias(u8_iq, iq_s16, n_pairs * 2);

    /* Step 2: NCO mix (0 offset for on-center signal) */
    pie_nco_t *nco = pie_nco_create(1024000, 0);
    pie_nco_mix_s16(iq_s16, nco, iq_mixed, n_pairs);
    pie_nco_free(nco);

    /* Step 3: CIC decimation */
    int decim_ratio = (mode == FM_DEMOD_WBFM) ? 4 : 32;
    int decim_pairs = n_pairs;
    pie_cic_decimate_s16(iq_mixed, n_pairs, iq_decim, &decim_pairs, decim_ratio, cic_state);

    /* Step 4: FM demod -> audio */
    fm_demod_config_t demod_cfg;
    if (mode == FM_DEMOD_WBFM) {
        fm_demod_config_t tmp = FM_DEMOD_CONFIG_WBFM();
        demod_cfg = tmp;
    } else {
        fm_demod_config_t tmp = FM_DEMOD_CONFIG_NBFM();
        demod_cfg = tmp;
    }
    fm_demod_t *demod = fm_demod_create(&demod_cfg);
    if (!demod) {
        heap_caps_free(iq_s16);
        heap_caps_free(iq_mixed);
        heap_caps_free(iq_decim);
        return 0;
    }

    int audio_count = fm_demod_process(demod, iq_decim, decim_pairs, audio_out, audio_max);
    fm_demod_free(demod);
    heap_caps_free(iq_s16);
    heap_caps_free(iq_mixed);
    heap_caps_free(iq_decim);

    return audio_count;
}

/* Check for a tone by zero-crossing analysis. Returns 0-1 correlation. */
static float measure_tone(const int16_t *audio, int n, float freq, float fs)
{
    int crossings = 0;
    for (int i = 1; i < n; i++) {
        if ((audio[i] > 0 && audio[i-1] <= 0) || (audio[i] <= 0 && audio[i-1] > 0))
            crossings++;
    }
    float expected = 2.0f * freq * n / fs;
    if (expected < 2) return 0;
    float ratio = crossings / expected;
    return (ratio > 0.7f && ratio < 1.3f) ? ratio : 0;
}

/* ── Test: WBFM Clean Signal ── */

static void test_e2e_wbfm_clean(void)
{
    ESP_LOGI(TAG, "--- test_e2e_wbfm_clean ---");

    const int n_pairs = 32768;  /* 32ms at 1024kSPS */
    uint8_t *iq = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "WBFM clean alloc");
    if (!iq || !audio) goto wbfm_clean_cleanup;

    /* FM 1kHz tone at 75kHz deviation, no carrier offset, amplitude 100 */
    generate_rtlsdr_iq(iq, n_pairs, 0, 1000.0f, 75000.0f, 1024000.0f, 100);

    int audio_count = run_full_pipeline(iq, n_pairs, audio, n_pairs, FM_DEMOD_WBFM);
    ESP_LOGI(TAG, "  Produced %d audio samples from %d IQ pairs", audio_count, n_pairs);
    TEST_ASSERT(audio_count > 0, "WBFM clean produces audio");

    if (audio_count > 0) {
        float corr = measure_tone(audio, audio_count, 1000.0f, 48000.0f);
        ESP_LOGI(TAG, "  1kHz tone correlation: %.3f", corr);
        TEST_ASSERT(corr > 0.7f, "WBFM clean 1kHz tone detected (corr > 0.7)");
    }

wbfm_clean_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: WBFM Noisy Signal ── */

static void test_e2e_wbfm_noisy(void)
{
    ESP_LOGI(TAG, "--- test_e2e_wbfm_noisy ---");

    const int n_pairs = 32768;
    uint8_t *iq = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "WBFM noisy alloc");
    if (!iq || !audio) goto wbfm_noisy_cleanup;

    generate_rtlsdr_iq(iq, n_pairs, 0, 1000.0f, 75000.0f, 1024000.0f, 100);

    /* Add white noise at ~20dB SNR (noise amplitude ~10 with signal amplitude 100) */
    add_white_noise_u8(iq, n_pairs * 2, 10);

    int audio_count = run_full_pipeline(iq, n_pairs, audio, n_pairs, FM_DEMOD_WBFM);
    ESP_LOGI(TAG, "  Produced %d audio samples from %d noisy IQ pairs", audio_count, n_pairs);
    TEST_ASSERT(audio_count > 0, "WBFM noisy produces audio");

    if (audio_count > 0) {
        float corr = measure_tone(audio, audio_count, 1000.0f, 48000.0f);
        ESP_LOGI(TAG, "  1kHz tone correlation with noise: %.3f", corr);
        TEST_ASSERT(corr > 0.5f, "WBFM noisy 1kHz tone detected (corr > 0.5)");
    }

wbfm_noisy_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: Impulse Noise ── */

static void test_e2e_impulse_noise(void)
{
    ESP_LOGI(TAG, "--- test_e2e_impulse_noise ---");

    const int n_pairs = 32768;
    uint8_t *iq_clean = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    uint8_t *iq_spike = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    int16_t *audio_clean = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio_spike = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq_clean && iq_spike && audio_clean && audio_spike, "Impulse noise alloc");
    if (!iq_clean || !iq_spike || !audio_clean || !audio_spike) goto impulse_cleanup;

    generate_rtlsdr_iq(iq_clean, n_pairs, 0, 1000.0f, 75000.0f, 1024000.0f, 100);
    memcpy(iq_spike, iq_clean, n_pairs * 2);

    /* Add 10 impulse spikes at amplitude 255 */
    uint32_t rng = 99999;
    for (int s = 0; s < 10; s++) {
        rng = rng * 1103515245 + 12345;
        int pos = (rng >> 16) % n_pairs;
        iq_spike[pos * 2]     = 255;
        iq_spike[pos * 2 + 1] = 255;
    }

    int clean_count = run_full_pipeline(iq_clean, n_pairs, audio_clean, n_pairs, FM_DEMOD_WBFM);
    int spike_count = run_full_pipeline(iq_spike, n_pairs, audio_spike, n_pairs, FM_DEMOD_WBFM);

    /* Find max amplitude in clean audio */
    int max_clean = 0;
    for (int i = 0; i < clean_count; i++) {
        int v = audio_clean[i] < 0 ? -audio_clean[i] : audio_clean[i];
        if (v > max_clean) max_clean = v;
    }

    /* Find max amplitude in spike audio */
    int max_spike = 0;
    for (int i = 0; i < spike_count; i++) {
        int v = audio_spike[i] < 0 ? -audio_spike[i] : audio_spike[i];
        if (v > max_spike) max_spike = v;
    }

    ESP_LOGI(TAG, "  Clean max: %d, Spike max: %d", max_clean, max_spike);
    TEST_ASSERT(max_spike < max_clean * 2 + 1000,
                "Impulse noise: output max < 2x normal level");

impulse_cleanup:
    heap_caps_free(iq_clean);
    heap_caps_free(iq_spike);
    heap_caps_free(audio_clean);
    heap_caps_free(audio_spike);
}

/* ── Test: Frequency Offset ── */

static void test_e2e_freq_offset(void)
{
    ESP_LOGI(TAG, "--- test_e2e_freq_offset ---");

    const int n_pairs = 32768;
    uint8_t *iq = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "Freq offset alloc");
    if (!iq || !audio) goto offset_cleanup;

    /* Generate IQ with +50kHz carrier offset */
    generate_rtlsdr_iq(iq, n_pairs, 50000.0f, 1000.0f, 75000.0f, 1024000.0f, 100);

    /* Run pipeline with NCO compensation at +50kHz */
    int16_t *iq_s16 = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_mixed = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_decim = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int32_t cic_state[26] = {0};

    if (!iq_s16 || !iq_mixed || !iq_decim) {
        TEST_ASSERT(0, "Freq offset buffer alloc");
        goto offset_buf_cleanup;
    }

    pie_u8_to_s16_bias(iq, iq_s16, n_pairs * 2);

    /* NCO at +50kHz to compensate carrier offset */
    pie_nco_t *nco = pie_nco_create(1024000, 50000);
    pie_nco_mix_s16(iq_s16, nco, iq_mixed, n_pairs);
    pie_nco_free(nco);

    int decim_pairs = n_pairs;
    pie_cic_decimate_s16(iq_mixed, n_pairs, iq_decim, &decim_pairs, 4, cic_state);

    fm_demod_config_t demod_cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod = fm_demod_create(&demod_cfg);
    if (!demod) {
        TEST_ASSERT(0, "Freq offset demod create");
        goto offset_buf_cleanup;
    }

    int audio_count = fm_demod_process(demod, iq_decim, decim_pairs, audio, n_pairs);
    fm_demod_free(demod);

    ESP_LOGI(TAG, "  Produced %d audio samples with +50kHz offset", audio_count);
    TEST_ASSERT(audio_count > 0, "Freq offset produces audio");

    if (audio_count > 0) {
        float corr = measure_tone(audio, audio_count, 1000.0f, 48000.0f);
        ESP_LOGI(TAG, "  1kHz tone correlation with offset: %.3f", corr);
        TEST_ASSERT(corr > 0.7f, "Freq offset 1kHz tone detected (corr > 0.7)");
    }

offset_buf_cleanup:
    heap_caps_free(iq_s16);
    heap_caps_free(iq_mixed);
    heap_caps_free(iq_decim);
offset_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: NBFM ── */

static void test_e2e_nbfm(void)
{
    ESP_LOGI(TAG, "--- test_e2e_nbfm ---");

    const int n_pairs = 32768;
    uint8_t *iq = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "NBFM alloc");
    if (!iq || !audio) goto nbfm_cleanup;

    /* FM 1kHz tone at 5kHz deviation (NBFM) */
    generate_rtlsdr_iq(iq, n_pairs, 0, 1000.0f, 5000.0f, 1024000.0f, 100);

    int audio_count = run_full_pipeline(iq, n_pairs, audio, n_pairs, FM_DEMOD_NBFM);
    ESP_LOGI(TAG, "  NBFM produced %d audio samples from %d IQ pairs", audio_count, n_pairs);
    TEST_ASSERT(audio_count > 0, "NBFM produces audio");

    if (audio_count > 0) {
        float corr = measure_tone(audio, audio_count, 1000.0f, 48000.0f);
        ESP_LOGI(TAG, "  1kHz tone correlation: %.3f", corr);
        TEST_ASSERT(corr > 0.7f, "NBFM 1kHz tone detected (corr > 0.7)");
    }

nbfm_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: Mode Switch ── */

static void test_e2e_mode_switch(void)
{
    ESP_LOGI(TAG, "--- test_e2e_mode_switch ---");

    const int n_pairs = 16384;
    uint8_t *iq = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    int16_t *audio_wb = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio_nb = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio_wb && audio_nb, "Mode switch alloc");
    if (!iq || !audio_wb || !audio_nb) goto modeswitch_cleanup;

    /* Run WBFM pipeline for 16k samples */
    generate_rtlsdr_iq(iq, n_pairs, 0, 1000.0f, 75000.0f, 1024000.0f, 100);
    int wb_count = run_full_pipeline(iq, n_pairs, audio_wb, n_pairs, FM_DEMOD_WBFM);
    ESP_LOGI(TAG, "  WBFM produced %d audio samples", wb_count);
    TEST_ASSERT(wb_count > 0, "Mode switch WBFM produces audio");

    /* Run NBFM pipeline for 16k samples */
    generate_rtlsdr_iq(iq, n_pairs, 0, 1000.0f, 5000.0f, 1024000.0f, 100);
    int nb_count = run_full_pipeline(iq, n_pairs, audio_nb, n_pairs, FM_DEMOD_NBFM);
    ESP_LOGI(TAG, "  NBFM produced %d audio samples", nb_count);
    TEST_ASSERT(nb_count > 0, "Mode switch NBFM produces audio");

    /* Verify no crash and both produced output */
    TEST_ASSERT(wb_count > 0 && nb_count > 0, "Mode switch: both modes produce audio");

modeswitch_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio_wb);
    heap_caps_free(audio_nb);
}

/* ── Test: Silence (pure carrier, no modulation) ── */

static void test_e2e_silence(void)
{
    ESP_LOGI(TAG, "--- test_e2e_silence ---");

    const int n_pairs = 32768;
    uint8_t *iq = heap_caps_aligned_alloc(16, n_pairs * 2, MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "Silence alloc");
    if (!iq || !audio) goto silence_cleanup;

    /* Pure carrier: no modulation (mod_freq=0, deviation=0) */
    generate_rtlsdr_iq(iq, n_pairs, 0, 0, 0, 1024000.0f, 100);

    int audio_count = run_full_pipeline(iq, n_pairs, audio, n_pairs, FM_DEMOD_WBFM);
    ESP_LOGI(TAG, "  Silence produced %d audio samples", audio_count);

    if (audio_count > 0) {
        /* Compute RMS */
        int64_t sum_sq = 0;
        for (int i = 0; i < audio_count; i++) {
            sum_sq += (int64_t)audio[i] * audio[i];
        }
        float rms = sqrtf((float)(sum_sq / audio_count));
        ESP_LOGI(TAG, "  Silence RMS: %.1f", rms);
        TEST_ASSERT(rms < 500.0f, "Silence: audio RMS < 500");
    }

silence_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Public Entry Point ── */

void test_pipeline_e2e(void)
{
    ESP_LOGI(TAG, "========== E2E Pipeline Tests ==========");
    tests_passed = 0;
    tests_failed = 0;

    test_e2e_wbfm_clean();
    test_e2e_wbfm_noisy();
    test_e2e_impulse_noise();
    test_e2e_freq_offset();
    test_e2e_nbfm();
    test_e2e_mode_switch();
    test_e2e_silence();

    ESP_LOGI(TAG, "========== Results: %d passed, %d failed ==========",
             tests_passed, tests_failed);
}
