/*
 * DSP Kernel Unit Tests
 *
 * Validates each DSP kernel with known-good test vectors.
 * Runs on ESP32-P4 target, results logged via ESP_LOGI.
 * Call test_dsp_kernels() from app_main when CONFIG_FM_TEST_ENABLE=y.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "pie_kernels.h"
#include "pie_fm_discriminator.h"
#include "polyphase_resamp.h"

static const char *TAG = "test_dsp";
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "FAIL: %s (line %d)", msg, __LINE__); \
        tests_failed++; \
    } else { \
        ESP_LOGI(TAG, "PASS: %s", msg); \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_INT_WITHIN(delta, expected, actual, msg) do { \
    int _d = abs((int)(actual) - (int)(expected)); \
    if (_d > (delta)) { \
        ESP_LOGE(TAG, "FAIL: %s — expected %d ±%d, got %d (delta=%d)", \
                 msg, (int)(expected), (delta), (int)(actual), _d); \
        tests_failed++; \
    } else { \
        ESP_LOGI(TAG, "PASS: %s (expected %d, got %d)", msg, (int)(expected), (int)(actual)); \
        tests_passed++; \
    } \
} while(0)

/* ── test_u8_to_s16_bias ── */

static void test_u8_to_s16_bias(void)
{
    ESP_LOGI(TAG, "--- test_u8_to_s16_bias ---");
    uint8_t input[16] __attribute__((aligned(16))) = {0, 1, 127, 128, 129, 255, 64, 192, 0,0,0,0,0,0,0,0};
    int16_t output[16] __attribute__((aligned(16)));

    pie_u8_to_s16_bias(input, output, 16);

    /* (val - 128) << 8 */
    TEST_ASSERT_INT_WITHIN(1, -32768, output[0], "u8=0 → -32768");
    TEST_ASSERT_INT_WITHIN(1, -32512, output[1], "u8=1 → -32512");
    TEST_ASSERT_INT_WITHIN(1, -256, output[2], "u8=127 → -256");
    TEST_ASSERT_INT_WITHIN(1, 0, output[3], "u8=128 → 0");
    TEST_ASSERT_INT_WITHIN(1, 256, output[4], "u8=129 → 256");
    TEST_ASSERT_INT_WITHIN(1, 32512, output[5], "u8=255 → 32512");
}

/* ── test_nco_mix ── */

static void test_nco_mix(void)
{
    ESP_LOGI(TAG, "--- test_nco_mix ---");

    /* Create NCO at 10kHz offset, 256kSPS */
    pie_nco_t *nco = pie_nco_create(256000, 10000);
    TEST_ASSERT(nco != NULL, "NCO create");
    if (!nco) return;

    const int n_pairs = 256;  /* 1ms of data */
    int16_t *iq_in  = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_out = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq_in && iq_out, "NCO buffer alloc");
    if (!iq_in || !iq_out) goto nco_cleanup;

    /* DC input: constant I=16384, Q=0 */
    for (int i = 0; i < n_pairs; i++) {
        iq_in[i * 2]     = 16384;  /* I */
        iq_in[i * 2 + 1] = 0;      /* Q */
    }

    pie_nco_mix_s16(iq_in, nco, iq_out, n_pairs);

    /* Output should oscillate (not DC anymore) — check that I values change sign */
    int sign_changes = 0;
    for (int i = 1; i < n_pairs; i++) {
        if ((iq_out[i * 2] > 0) != (iq_out[(i - 1) * 2] > 0)) {
            sign_changes++;
        }
    }
    /* 10kHz at 256kSPS → ~25.6 samples/cycle → ~2 sign changes/cycle → ~20 changes in 256 samples */
    TEST_ASSERT(sign_changes > 5, "NCO output oscillates (not DC)");

    /* Energy conservation: RMS of output ≈ RMS of input */
    int64_t rms_in = 0, rms_out = 0;
    for (int i = 0; i < n_pairs; i++) {
        rms_in  += (int64_t)iq_in[i * 2] * iq_in[i * 2] + (int64_t)iq_in[i * 2 + 1] * iq_in[i * 2 + 1];
        rms_out += (int64_t)iq_out[i * 2] * iq_out[i * 2] + (int64_t)iq_out[i * 2 + 1] * iq_out[i * 2 + 1];
    }
    /* Allow 25% tolerance for Q15 rounding */
    double ratio = (double)rms_out / (double)rms_in;
    TEST_ASSERT(ratio > 0.5 && ratio < 1.5, "NCO energy conservation (0.5-1.5x)");

nco_cleanup:
    heap_caps_free(iq_in);
    heap_caps_free(iq_out);
    pie_nco_free(nco);
}

/* ── test_cic_decimate ── */

static void test_cic_decimate(void)
{
    ESP_LOGI(TAG, "--- test_cic_decimate ---");

    const int in_pairs = 1024;
    const int decim = 4;
    int16_t *iq_in  = heap_caps_aligned_alloc(16, in_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_out = heap_caps_aligned_alloc(16, in_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int32_t accum[26];
    memset(accum, 0, sizeof(accum));

    TEST_ASSERT(iq_in && iq_out, "CIC buffer alloc");
    if (!iq_in || !iq_out) goto cic_cleanup;

    /* DC input: constant I=4096, Q=2048 */
    for (int i = 0; i < in_pairs; i++) {
        iq_in[i * 2]     = 4096;
        iq_in[i * 2 + 1] = 2048;
    }

    int out_pairs = in_pairs;  /* max output */
    pie_cic_decimate_s16(iq_in, in_pairs, iq_out, &out_pairs, decim, accum);

    /* Output count should be in_pairs / decim = 256 */
    TEST_ASSERT_INT_WITHIN(2, 256, out_pairs, "CIC output count = 256");

    /* After CIC settles (skip first few), DC level should be stable.
     * CIC gain for order 3, ratio 4 = 4^3 = 64, but normalized back to int16.
     * Check that output is non-zero and has consistent sign. */
    if (out_pairs > 16) {
        int nonzero = 0;
        for (int i = out_pairs / 2; i < out_pairs; i++) {
            if (iq_out[i * 2] != 0) nonzero++;
        }
        TEST_ASSERT(nonzero > 0, "CIC DC output non-zero after settling");
    }

cic_cleanup:
    heap_caps_free(iq_in);
    heap_caps_free(iq_out);
}

/* ── test_fir_impulse ── */

static void test_fir_impulse(void)
{
    ESP_LOGI(TAG, "--- test_fir_impulse ---");

    /* Known taps */
    int16_t taps[8] = {1000, 2000, 3000, 2000, 1000, 0, 0, 0};
    pie_fir_state_t *fir = pie_fir_create(taps, 8);
    TEST_ASSERT(fir != NULL, "FIR create");
    if (!fir) return;

    /* Impulse input: sample 0 = 32767, rest = 0 */
    const int n = 32;
    int16_t input[32] __attribute__((aligned(16)));
    int16_t output[32] __attribute__((aligned(16)));
    memset(input, 0, sizeof(input));
    input[0] = 32767;

    pie_fir_process(fir, input, output, n);

    /* Impulse response = taps convolved with impulse = taps (scaled by 32767/32768 in Q15) */
    /* Expected: output[k] ≈ taps[k] * 32767 / 32768 ≈ taps[k] */
    TEST_ASSERT_INT_WITHIN(50, 1000, output[0], "FIR impulse h[0]=1000");
    TEST_ASSERT_INT_WITHIN(50, 2000, output[1], "FIR impulse h[1]=2000");
    TEST_ASSERT_INT_WITHIN(50, 3000, output[2], "FIR impulse h[2]=3000");
    TEST_ASSERT_INT_WITHIN(50, 2000, output[3], "FIR impulse h[3]=2000");
    TEST_ASSERT_INT_WITHIN(50, 1000, output[4], "FIR impulse h[4]=1000");
    TEST_ASSERT_INT_WITHIN(50, 0,    output[5], "FIR impulse h[5]=0");

    pie_fir_free(fir);
}

/* ── test_fir_dc_passthrough ── */

static void test_fir_dc_passthrough(void)
{
    ESP_LOGI(TAG, "--- test_fir_dc_passthrough ---");

    /* Design a simple lowpass FIR: 63-tap Nuttall window at 15kHz/256kHz.
     * Generate taps using windowed sinc. */
    const int N = 63;
    const float fc = 15000.0f / 256000.0f;  /* Normalized cutoff */
    int16_t taps_q15[63];

    float sum = 0.0f;
    for (int n = 0; n < N; n++) {
        int m = n - (N - 1) / 2;
        /* Sinc */
        float h;
        if (m == 0) {
            h = 2.0f * fc;
        } else {
            h = sinf(2.0f * (float)M_PI * fc * m) / ((float)M_PI * m);
        }
        /* Nuttall window */
        float x = 2.0f * (float)M_PI * n / (N - 1);
        float w = 0.3635819f - 0.4891775f * cosf(x)
                 + 0.1365995f * cosf(2.0f * x)
                 - 0.0106411f * cosf(3.0f * x);
        h *= w;
        sum += h;
    }

    /* Normalize for unity DC gain, convert to Q15 */
    for (int n = 0; n < N; n++) {
        int m = n - (N - 1) / 2;
        float h;
        if (m == 0) {
            h = 2.0f * fc;
        } else {
            h = sinf(2.0f * (float)M_PI * fc * m) / ((float)M_PI * m);
        }
        float x = 2.0f * (float)M_PI * n / (N - 1);
        float w = 0.3635819f - 0.4891775f * cosf(x)
                 + 0.1365995f * cosf(2.0f * x)
                 - 0.0106411f * cosf(3.0f * x);
        h *= w;
        h /= sum;
        int val = (int)(h * 32767.0f + 0.5f);
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        taps_q15[n] = (int16_t)val;
    }

    pie_fir_state_t *fir = pie_fir_create(taps_q15, N);
    TEST_ASSERT(fir != NULL, "FIR DC create");
    if (!fir) return;

    /* Feed DC signal: all samples = 16384 */
    const int n_samples = 256;
    int16_t *input  = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *output = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!input || !output) {
        TEST_ASSERT(0, "FIR DC buffer alloc");
        goto fir_dc_cleanup;
    }
    for (int i = 0; i < n_samples; i++) {
        input[i] = 16384;
    }

    pie_fir_process(fir, input, output, n_samples);

    /* After filter settles (skip first N taps), output should converge to ~16384 */
    int settled_val = output[n_samples - 1];
    TEST_ASSERT_INT_WITHIN(512, 16384, settled_val, "FIR DC passthrough ≈16384");

fir_dc_cleanup:
    heap_caps_free(input);
    heap_caps_free(output);
    pie_fir_free(fir);
}

/* ── test_volume ── */

static void test_volume(void)
{
    ESP_LOGI(TAG, "--- test_volume ---");

    /* Volume 50% = Q15 16384 */
    int16_t audio[8] __attribute__((aligned(16))) = {32767, -32768, 16384, 0, 0, 0, 0, 0};

    pie_volume_s16(audio, 8, 16384);

    TEST_ASSERT_INT_WITHIN(1, 16383, audio[0], "vol 50% of 32767 → 16383");
    TEST_ASSERT_INT_WITHIN(1, -16384, audio[1], "vol 50% of -32768 → -16384");
    TEST_ASSERT_INT_WITHIN(1, 8192, audio[2], "vol 50% of 16384 → 8192");
}

/* ── test_polyphase_resamp ── */

static void test_polyphase_resamp(void)
{
    ESP_LOGI(TAG, "--- test_polyphase_resamp ---");

    /* 256000 → 48000 */
    polyphase_resamp_t *r = polyphase_resamp_create(256000, 48000, 16);
    TEST_ASSERT(r != NULL, "Polyphase resamp create");
    if (!r) return;

    const int in_count = 2560;  /* 10ms at 256kHz */
    const int out_max = 1024;
    int16_t *input  = heap_caps_aligned_alloc(16, in_count * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *output = heap_caps_aligned_alloc(16, out_max * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    if (!input || !output) {
        TEST_ASSERT(0, "Resamp buffer alloc");
        goto resamp_cleanup;
    }

    /* Generate 1kHz tone at 256kSPS */
    for (int i = 0; i < in_count; i++) {
        input[i] = (int16_t)(16384.0f * sinf(2.0f * (float)M_PI * 1000.0f * i / 256000.0f));
    }

    int out_count = polyphase_resamp_process(r, input, in_count, output, out_max);

    /* Expected: ~480 samples (48000 * 0.01) */
    TEST_ASSERT_INT_WITHIN(24, 480, out_count, "Resamp output count ≈480");

    /* Verify 1kHz tone passes through: check output has oscillation */
    if (out_count > 48) {
        int sign_changes = 0;
        for (int i = 1; i < out_count; i++) {
            if ((output[i] > 0) != (output[i - 1] > 0)) {
                sign_changes++;
            }
        }
        /* 1kHz at 48kSPS, 480 samples = 10ms → 10 cycles → ~20 sign changes */
        TEST_ASSERT(sign_changes > 8, "Resamp 1kHz tone preserved");
    }

resamp_cleanup:
    heap_caps_free(input);
    heap_caps_free(output);
    polyphase_resamp_free(r);
}

/* ── test_fm_discriminator_dc ── */

static void test_fm_discriminator_dc(void)
{
    ESP_LOGI(TAG, "--- test_fm_discriminator_dc ---");

    const int n_pairs = 256;
    int16_t *iq_in  = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio  = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    if (!iq_in || !audio) {
        TEST_ASSERT(0, "FM disc DC buffer alloc");
        goto disc_dc_cleanup;
    }

    /* Constant phase IQ: I=16384, Q=0 (zero frequency offset) */
    for (int i = 0; i < n_pairs; i++) {
        iq_in[i * 2]     = 16384;
        iq_in[i * 2 + 1] = 0;
    }

    /* Test POLY method */
    fm_disc_state_t state;
    fm_disc_init(&state, FM_DISC_POLY_CORRECTED, 256000, 75000);
    fm_disc_process(&state, iq_in, audio, n_pairs);

    /* After first sample, output should be ~0 (no modulation) */
    int64_t sum_abs = 0;
    for (int i = 4; i < n_pairs; i++) {
        sum_abs += abs(audio[i]);
    }
    int avg_abs = (int)(sum_abs / (n_pairs - 4));
    TEST_ASSERT_INT_WITHIN(500, 0, avg_abs, "FM disc POLY DC → ~0");

    /* Test LINEAR method */
    fm_disc_init(&state, FM_DISC_FAST_LINEAR, 256000, 75000);
    fm_disc_process(&state, iq_in, audio, n_pairs);

    sum_abs = 0;
    for (int i = 4; i < n_pairs; i++) {
        sum_abs += abs(audio[i]);
    }
    avg_abs = (int)(sum_abs / (n_pairs - 4));
    TEST_ASSERT_INT_WITHIN(500, 0, avg_abs, "FM disc LINEAR DC → ~0");

disc_dc_cleanup:
    heap_caps_free(iq_in);
    heap_caps_free(audio);
}

/* ── test_fm_discriminator_tone ── */

static void test_fm_discriminator_tone(void)
{
    ESP_LOGI(TAG, "--- test_fm_discriminator_tone ---");

    /* Generate FM IQ for 1kHz tone at 75kHz deviation, 256kSPS.
     * FM: phase(t) = 2*pi*fc*t + (deviation/f_mod)*sin(2*pi*f_mod*t)
     * IQ: I = cos(phase), Q = sin(phase)
     * With fc=0 (baseband): phase(t) = beta * sin(2*pi*f_mod*t)
     * where beta = deviation / f_mod = 75000 / 1000 = 75
     */
    const int n_pairs = 32768;  /* 128ms at 256kSPS */
    const float fs = 256000.0f;
    const float f_mod = 1000.0f;
    const float deviation = 75000.0f;
    const float beta = deviation / f_mod;

    int16_t *iq_in  = heap_caps_aligned_alloc(16, n_pairs * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio  = heap_caps_aligned_alloc(16, n_pairs * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    if (!iq_in || !audio) {
        TEST_ASSERT(0, "FM disc tone buffer alloc");
        goto disc_tone_cleanup;
    }

    /* Generate FM IQ */
    for (int i = 0; i < n_pairs; i++) {
        float t = (float)i / fs;
        float phase = beta * sinf(2.0f * (float)M_PI * f_mod * t);
        iq_in[i * 2]     = (int16_t)(16384.0f * cosf(phase));
        iq_in[i * 2 + 1] = (int16_t)(16384.0f * sinf(phase));
    }

    /* Test POLY method */
    fm_disc_state_t state;
    fm_disc_init(&state, FM_DISC_POLY_CORRECTED, 256000, 75000);
    fm_disc_process(&state, iq_in, audio, n_pairs);

    /* Count zero-crossings in the demodulated audio.
     * Expected: 1kHz over 128ms = 128 cycles → ~256 zero crossings.
     * Allow wide tolerance since discriminator output may have offset/noise. */
    int crossings = 0;
    for (int i = 257; i < n_pairs; i++) {  /* Skip first ms for settling */
        if ((audio[i] > 0) != (audio[i - 1] > 0)) {
            crossings++;
        }
    }
    /* 128ms * 1kHz = 128 cycles → ~256 crossings. Accept 100-400 range. */
    TEST_ASSERT(crossings > 100 && crossings < 400,
                "FM disc POLY 1kHz tone zero-crossings (100-400)");
    ESP_LOGI(TAG, "  POLY crossings: %d (expected ~256)", crossings);

    /* Test LINEAR method */
    fm_disc_init(&state, FM_DISC_FAST_LINEAR, 256000, 75000);
    fm_disc_process(&state, iq_in, audio, n_pairs);

    crossings = 0;
    for (int i = 257; i < n_pairs; i++) {
        if ((audio[i] > 0) != (audio[i - 1] > 0)) {
            crossings++;
        }
    }
    TEST_ASSERT(crossings > 100 && crossings < 400,
                "FM disc LINEAR 1kHz tone zero-crossings (100-400)");
    ESP_LOGI(TAG, "  LINEAR crossings: %d (expected ~256)", crossings);

disc_tone_cleanup:
    heap_caps_free(iq_in);
    heap_caps_free(audio);
}

/* ── Public entry point ── */

void test_dsp_kernels(void)
{
    ESP_LOGI(TAG, "========== DSP Kernel Unit Tests ==========");
    tests_passed = 0;
    tests_failed = 0;

    test_u8_to_s16_bias();
    test_nco_mix();
    test_cic_decimate();
    test_fir_impulse();
    test_fir_dc_passthrough();
    test_volume();
    test_polyphase_resamp();
    test_fm_discriminator_dc();
    test_fm_discriminator_tone();

    ESP_LOGI(TAG, "========== Results: %d passed, %d failed ==========",
             tests_passed, tests_failed);
}
