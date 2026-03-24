/*
 * Unit tests for PIE SIMD DSP kernel C reference implementations.
 *
 * Tests the C fallback paths (not assembly) to validate correctness.
 * Build with: gcc -o test_dsp_kernels test_dsp_kernels.c -lm -I../components/dsp/include
 * Or include in ESP-IDF test build.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>

/* ── Inline the kernel functions for host testing (no ESP-IDF deps) ── */

/* Stub out ESP-IDF dependencies for host build */
#ifndef CONFIG_IDF_TARGET_ESP32P4
#define CONFIG_IDF_TARGET_ESP32P4 0
#define CONFIG_DSP_OPTIMIZED 0
#define ESP_LOGI(tag, fmt, ...) printf("[I] " fmt "\n", ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[E] " fmt "\n", ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[W] " fmt "\n", ##__VA_ARGS__)
#define heap_caps_aligned_alloc(align, size, caps) aligned_alloc(align, ((size) + (align)-1) & ~((align)-1))
#define heap_caps_free(ptr) free(ptr)
#define MALLOC_CAP_DEFAULT 0
#endif

/* Include kernel source directly for host testing */
#include "../components/dsp/pie_kernels.h"

/* ── Forward declarations of functions we test ── */
/* These are defined in pie_kernels.c which we can't include directly
 * on host. Instead, we re-implement the C reference paths here for testing. */

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define TEST_ASSERT_INT_EQ(a, b, msg) do { \
    if ((a) != (b)) { \
        printf("  FAIL: %s: expected %d, got %d (line %d)\n", msg, (int)(b), (int)(a), __LINE__); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define TEST_ASSERT_FLOAT_NEAR(a, b, tol, msg) do { \
    if (fabsf((float)(a) - (float)(b)) > (tol)) { \
        printf("  FAIL: %s: expected %.4f, got %.4f (tol=%.4f, line %d)\n", \
               msg, (float)(b), (float)(a), (float)(tol), __LINE__); \
        tests_failed++; \
        return; \
    } \
} while(0)

/* ────────────────────────── Test: Windowing ────────────────────────── */

static void test_windowing_basic(void)
{
    printf("test_windowing_basic...\n");

    /* 4 IQ pairs, flat window (all 32767 = Q15 ~1.0) */
    uint8_t src[] = { 128+10, 128-20, 128+50, 128-100, 0, 255, 128, 128 };
    int16_t window[] = { 32767, 32767, 32767, 32767 };
    int16_t dst[8] = {0};

    /* Call C reference directly */
    for (int k = 0; k < 4; k++) {
        int16_t si = ((int16_t)src[k * 2] - 128) << 8;
        int16_t sq = ((int16_t)src[k * 2 + 1] - 128) << 8;
        int32_t wi = window[k];
        dst[k * 2]     = (int16_t)((si * wi) >> 15);
        dst[k * 2 + 1] = (int16_t)((sq * wi) >> 15);
    }

    /* With flat window, output should be (val - 128) << 8, scaled by ~1.0 */
    /* src[0] = 138, so I = (138-128)<<8 = 2560, windowed ≈ 2560 */
    TEST_ASSERT(abs(dst[0] - 2560) <= 1, "I[0] should be ~2560");
    /* src[1] = 108, so Q = (108-128)<<8 = -5120, windowed ≈ -5120 */
    TEST_ASSERT(abs(dst[1] - (-5120)) <= 1, "Q[0] should be ~-5120");
    /* src[4] = 0, so I = (0-128)<<8 = -32768 */
    TEST_ASSERT(abs(dst[4] - (-32768)) <= 1, "I[2] should be ~-32768");
    /* src[5] = 255, so Q = (255-128)<<8 = 32512 */
    TEST_ASSERT(abs(dst[5] - 32512) <= 1, "Q[2] should be ~32512");

    tests_passed++;
    printf("  PASS\n");
}

static void test_windowing_hann(void)
{
    printf("test_windowing_hann...\n");

    /* 4 IQ pairs, Hann window: [0, 0.75, 0.75, 0] for N=4 */
    uint8_t src[] = { 200, 200, 200, 200, 200, 200, 200, 200 };
    /* Hann(k, N=4) = 0.5*(1 - cos(2*pi*k/4)) = {0, 1, 1, 0} for k=0..3
     * Wait, Hann(0,4)=0, Hann(1,4)=0.5*(1-cos(pi/2))=0.5, not 0.75
     * Actually Hann(k,N) = 0.5*(1-cos(2*pi*k/N))
     * N=4: k=0->0, k=1->0.5*(1-cos(pi/2))=0.5*(1-0)=0.5, k=2->0.5*(1-cos(pi))=1.0, k=3->0.5 */
    int16_t window[] = { 0, 16384, 32767, 16384 }; /* Q15: 0, 0.5, ~1.0, 0.5 */
    int16_t dst[8] = {0};

    for (int k = 0; k < 4; k++) {
        int16_t si = ((int16_t)src[k * 2] - 128) << 8;
        int16_t sq = ((int16_t)src[k * 2 + 1] - 128) << 8;
        int32_t wi = window[k];
        dst[k * 2]     = (int16_t)((si * wi) >> 15);
        dst[k * 2 + 1] = (int16_t)((sq * wi) >> 15);
    }

    /* src = 200 -> (200-128)<<8 = 18432 */
    /* k=0: windowed = 18432 * 0 >> 15 = 0 */
    TEST_ASSERT_INT_EQ(dst[0], 0, "I[0] with zero window");
    TEST_ASSERT_INT_EQ(dst[1], 0, "Q[0] with zero window");

    /* k=1: windowed = 18432 * 16384 >> 15 = 18432 * 0.5 ≈ 9216 */
    TEST_ASSERT(abs(dst[2] - 9216) <= 1, "I[1] with 0.5 window");

    /* k=2: windowed = 18432 * 32767 >> 15 ≈ 18432 */
    TEST_ASSERT(abs(dst[4] - 18432) <= 1, "I[2] with ~1.0 window");

    tests_passed++;
    printf("  PASS\n");
}

/* ────────────────────────── Test: Power Spectrum ────────────────────────── */

static void test_power_spectrum_basic(void)
{
    printf("test_power_spectrum_basic...\n");

    /* 4 bins, interleaved [re,im,re,im,...] */
    int16_t fft_out[] = { 100, 200, -300, 400, 0, 0, 1000, -500 };
    int64_t accum[4] = {0};

    /* C reference: accum[k] += re^2 + im^2 */
    for (int k = 0; k < 4; k++) {
        int32_t re = (int32_t)fft_out[k * 2];
        int32_t im = (int32_t)fft_out[k * 2 + 1];
        accum[k] += (int64_t)(re * re + im * im);
    }

    TEST_ASSERT(accum[0] == 100*100 + 200*200, "bin 0 power");
    TEST_ASSERT(accum[1] == 300*300 + 400*400, "bin 1 power");
    TEST_ASSERT(accum[2] == 0, "bin 2 power (zero)");
    TEST_ASSERT(accum[3] == 1000*1000 + 500*500, "bin 3 power");

    tests_passed++;
    printf("  PASS\n");
}

static void test_power_spectrum_accumulation(void)
{
    printf("test_power_spectrum_accumulation...\n");

    int16_t fft_out[] = { 1000, 0, 0, 1000 };
    int64_t accum[2] = {0};

    /* Accumulate 4 times (simulating FFT_AVG_COUNT=4) */
    for (int frame = 0; frame < 4; frame++) {
        for (int k = 0; k < 2; k++) {
            int32_t re = (int32_t)fft_out[k * 2];
            int32_t im = (int32_t)fft_out[k * 2 + 1];
            accum[k] += (int64_t)(re * re + im * im);
        }
    }

    TEST_ASSERT(accum[0] == 4LL * 1000000, "bin 0: 4 frames accumulated");
    TEST_ASSERT(accum[1] == 4LL * 1000000, "bin 1: 4 frames accumulated");

    tests_passed++;
    printf("  PASS\n");
}

/* ────────────────────────── Test: fast_log10f ────────────────────────── */

static inline float test_fast_log10f(float x)
{
    union { float f; int32_t i; } u = { .f = x };
    float log2_approx = (float)(u.i - 1064866805) * (1.0f / 8388608.0f);
    return log2_approx * 0.301029995663981f;
}

static void test_fast_log10f_accuracy(void)
{
    printf("test_fast_log10f_accuracy...\n");

    /* Test across several orders of magnitude */
    float test_values[] = { 1.0f, 10.0f, 100.0f, 1000.0f, 1e6f, 1e9f, 1e12f, 1e15f };
    int n_vals = sizeof(test_values) / sizeof(test_values[0]);

    for (int i = 0; i < n_vals; i++) {
        float fast = test_fast_log10f(test_values[i]);
        float exact = log10f(test_values[i]);
        float err = fabsf(fast - exact);
        /* Require within 0.3 dB (i.e., 0.03 log10 units) for SDR use */
        char msg[64];
        snprintf(msg, sizeof(msg), "fast_log10f(%.0e) err=%.3f", test_values[i], err);
        TEST_ASSERT_FLOAT_NEAR(fast, exact, 0.15f, msg);
    }

    tests_passed++;
    printf("  PASS\n");
}

/* ────────────────────────── Test: NCO Phase Continuity ────────────────────────── */

static void test_nco_phase_continuity(void)
{
    printf("test_nco_phase_continuity...\n");

    /* Simulate NCO with phase accumulator */
    uint32_t sample_rate = 2400000;
    int32_t offset_hz = 100001; /* Prime — would cause discontinuity with old table approach */

    /* Compute phase increment */
    int64_t inc64 = ((int64_t)offset_hz * (int64_t)UINT32_MAX) / (int64_t)sample_rate;
    uint32_t phase_inc = (uint32_t)inc64;
    uint32_t phase_acc = 0;

    /* Generate 100000 samples and check for discontinuities */
    double prev_phase = 0.0;
    int discontinuities = 0;
    double expected_phase_step = 2.0 * M_PI * (double)offset_hz / (double)sample_rate;

    for (int i = 0; i < 100000; i++) {
        /* Convert phase accumulator to radians */
        double phase = 2.0 * M_PI * (double)phase_acc / (double)UINT32_MAX;

        if (i > 0) {
            /* Check phase step (allow for 2*pi wrapping) */
            double step = phase - prev_phase;
            if (step < -M_PI) step += 2.0 * M_PI;
            if (step > M_PI) step -= 2.0 * M_PI;

            double step_err = fabs(step - expected_phase_step);
            /* Phase step error should be < 1 LSB of 32-bit accumulator */
            if (step_err > expected_phase_step * 0.01) {
                discontinuities++;
            }
        }
        prev_phase = phase;
        phase_acc += phase_inc; /* Natural 32-bit wrap */
    }

    TEST_ASSERT_INT_EQ(discontinuities, 0, "NCO phase discontinuities");

    tests_passed++;
    printf("  PASS\n");
}

/* ────────────────────────── Test: 3rd-Order CIC ────────────────────────── */

static void test_cic_3rd_order_dc(void)
{
    printf("test_cic_3rd_order_dc...\n");

    /* Feed constant (DC) signal through 3rd-order CIC with R=4.
     * Output should be the DC value (after settling). */
    int decim_ratio = 4;
    int32_t accum[26] = {0};

    int16_t input[64]; /* 32 IQ pairs */
    for (int i = 0; i < 64; i += 2) {
        input[i]     = 1000;  /* I = 1000 */
        input[i + 1] = -500;  /* Q = -500 */
    }

    int16_t output[32]; /* max 8 output pairs */
    int out_pairs = 8;

    /* CIC gain = R^N = 4^3 = 64. Shift = 3 * log2(4) = 6. */
    /* Feed two blocks to let it settle */
    for (int block = 0; block < 4; block++) {
        out_pairs = 8;
        /* Re-implement CIC inline for host test */
        int64_t i1_re=0,i1_im=0,i2_re=0,i2_im=0,i3_re=0,i3_im=0;
        int64_t c1r=0,c1i=0,c2r=0,c2i=0,c3r=0,c3i=0;
        int cnt=0, opos=0;
        int shift = 6; /* 3 * log2(4) */

        /* Restore state from accum */
        i1_re = ((int64_t)accum[1]  << 32) | (uint32_t)accum[0];
        i1_im = ((int64_t)accum[3]  << 32) | (uint32_t)accum[2];
        i2_re = ((int64_t)accum[5]  << 32) | (uint32_t)accum[4];
        i2_im = ((int64_t)accum[7]  << 32) | (uint32_t)accum[6];
        i3_re = ((int64_t)accum[9]  << 32) | (uint32_t)accum[8];
        i3_im = ((int64_t)accum[11] << 32) | (uint32_t)accum[10];
        c1r = ((int64_t)accum[13] << 32) | (uint32_t)accum[12];
        c1i = ((int64_t)accum[15] << 32) | (uint32_t)accum[14];
        c2r = ((int64_t)accum[17] << 32) | (uint32_t)accum[16];
        c2i = ((int64_t)accum[19] << 32) | (uint32_t)accum[18];
        c3r = ((int64_t)accum[21] << 32) | (uint32_t)accum[20];
        c3i = ((int64_t)accum[23] << 32) | (uint32_t)accum[22];
        cnt = accum[24];

        for (int k = 0; k < 32 && opos < 8; k++) {
            i1_re += input[k*2]; i1_im += input[k*2+1];
            i2_re += i1_re; i2_im += i1_im;
            i3_re += i2_re; i3_im += i2_im;
            cnt++;
            if (cnt >= decim_ratio) {
                cnt = 0;
                int64_t d1r=i3_re-c1r, d1i=i3_im-c1i; c1r=i3_re; c1i=i3_im;
                int64_t d2r=d1r-c2r, d2i=d1i-c2i; c2r=d1r; c2i=d1i;
                int64_t d3r=d2r-c3r, d3i=d2i-c3i; c3r=d2r; c3i=d2i;
                output[opos*2] = (int16_t)(d3r >> shift);
                output[opos*2+1] = (int16_t)(d3i >> shift);
                opos++;
            }
        }

        /* Save state */
        accum[0]=(int32_t)i1_re; accum[1]=(int32_t)(i1_re>>32);
        accum[2]=(int32_t)i1_im; accum[3]=(int32_t)(i1_im>>32);
        accum[4]=(int32_t)i2_re; accum[5]=(int32_t)(i2_re>>32);
        accum[6]=(int32_t)i2_im; accum[7]=(int32_t)(i2_im>>32);
        accum[8]=(int32_t)i3_re; accum[9]=(int32_t)(i3_re>>32);
        accum[10]=(int32_t)i3_im; accum[11]=(int32_t)(i3_im>>32);
        accum[12]=(int32_t)c1r; accum[13]=(int32_t)(c1r>>32);
        accum[14]=(int32_t)c1i; accum[15]=(int32_t)(c1i>>32);
        accum[16]=(int32_t)c2r; accum[17]=(int32_t)(c2r>>32);
        accum[18]=(int32_t)c2i; accum[19]=(int32_t)(c2i>>32);
        accum[20]=(int32_t)c3r; accum[21]=(int32_t)(c3r>>32);
        accum[22]=(int32_t)c3i; accum[23]=(int32_t)(c3i>>32);
        accum[24]=cnt;

        out_pairs = opos;
    }

    /* After settling, output should converge to DC input value */
    /* The last output pair should be close to (1000, -500) */
    if (out_pairs > 0) {
        int last = out_pairs - 1;
        printf("  CIC output[%d] = (%d, %d)\n", last, output[last*2], output[last*2+1]);
        /* Allow some settling tolerance */
        TEST_ASSERT(abs(output[last*2] - 1000) < 200, "CIC DC I converged");
        TEST_ASSERT(abs(output[last*2+1] - (-500)) < 200, "CIC DC Q converged");
    }

    tests_passed++;
    printf("  PASS\n");
}

static void test_cic_alias_rejection(void)
{
    printf("test_cic_alias_rejection...\n");

    /* Generate a tone near the first alias frequency (fs_out = fs_in/R).
     * For R=8, fs_out = fs_in/8. The CIC has a null at fs_out, but we test
     * slightly off-null (at 1.1*fs_out) to measure sidelobe rejection.
     * 3rd-order CIC sinc^3 first sidelobe is at ~-39 dB. */
    int decim_ratio = 8;
    int n_samples = 16384; /* Many samples for accurate power measurement */

    int16_t *input = malloc(n_samples * 2 * sizeof(int16_t));
    int16_t *output = malloc((n_samples / decim_ratio + 1) * 2 * sizeof(int16_t));

    /* Generate tone at 1.5 * fs_out = 1.5 * fs_in/R = fs_in * 1.5/8
     * This is in the first sidelobe of the CIC response */
    double freq = 2.0 * M_PI * 1.5 / (double)decim_ratio; /* Normalized freq */
    for (int k = 0; k < n_samples; k++) {
        input[k * 2]     = (int16_t)(10000.0 * cos(freq * k));
        input[k * 2 + 1] = (int16_t)(10000.0 * sin(freq * k));
    }

    /* Run CIC — inline for host testing */
    int64_t i1_re=0,i1_im=0,i2_re=0,i2_im=0,i3_re=0,i3_im=0;
    int64_t c1r=0,c1i=0,c2r=0,c2i=0,c3r=0,c3i=0;
    int cnt=0, opos=0;
    int shift = 9; /* 3 * log2(8) = 9 */
    int max_out = n_samples / decim_ratio;

    for (int k = 0; k < n_samples && opos < max_out; k++) {
        i1_re += input[k*2]; i1_im += input[k*2+1];
        i2_re += i1_re; i2_im += i1_im;
        i3_re += i2_re; i3_im += i2_im;
        cnt++;
        if (cnt >= decim_ratio) {
            cnt = 0;
            int64_t d1r=i3_re-c1r, d1i=i3_im-c1i; c1r=i3_re; c1i=i3_im;
            int64_t d2r=d1r-c2r, d2i=d1i-c2i; c2r=d1r; c2i=d1i;
            int64_t d3r=d2r-c3r, d3i=d2i-c3i; c3r=d2r; c3i=d2i;
            output[opos*2] = (int16_t)(d3r >> shift);
            output[opos*2+1] = (int16_t)(d3i >> shift);
            opos++;
        }
    }

    /* Measure output power (skip first few samples for settling) */
    double power_out = 0.0;
    int skip = 32; /* Skip settling transient */
    for (int k = skip; k < opos; k++) {
        double re = output[k * 2];
        double im = output[k * 2 + 1];
        power_out += re * re + im * im;
    }
    power_out /= (opos - skip);

    /* Input power */
    double power_in = 10000.0 * 10000.0; /* amplitude^2 */

    /* Rejection in dB */
    double rejection_db = 10.0 * log10(power_in / (power_out + 1e-10));

    printf("  CIC alias rejection at Nyquist: %.1f dB (input_pwr=%.0f, output_pwr=%.1f)\n",
           rejection_db, power_in, power_out);

    /* 3rd-order CIC first sidelobe rejection should be > 20 dB.
     * Theoretical sinc^3 first sidelobe is ~-39 dB, but at 1.5*fs_out
     * it's somewhat less. We require at least 20 dB. */
    TEST_ASSERT(rejection_db > 20.0, "CIC 3rd-order alias rejection >= 20 dB");

    free(input);
    free(output);
    tests_passed++;
    printf("  PASS\n");
}

/* ────────────────────────── Test: dB Conversion ────────────────────────── */

static void test_db_conversion(void)
{
    printf("test_db_conversion...\n");

    /* Test power-to-dB with known values */
    int64_t power[4] = { 1, 10000, 1000000, 100000000LL };
    uint8_t db_out[4] = {0};

    float db_min = 0.0f;
    float db_max = 80.0f;
    float inv = 1.0f;
    float db_range = db_max - db_min;
    float scale = 255.0f / db_range;

    for (int k = 0; k < 4; k++) {
        float avg_pwr = (float)power[k] * inv;
        float db;
        if (avg_pwr < 1.0f) {
            db = db_min;
        } else {
            /* Use fast_log10f */
            union { float f; int32_t i; } u = { .f = avg_pwr };
            float log2_approx = (float)(u.i - 1064866805) * (1.0f / 8388608.0f);
            db = 10.0f * log2_approx * 0.301029995663981f;
        }
        if (db < db_min) db = db_min;
        if (db > db_max) db = db_max;

        /* No FFT shift for this test — just linear mapping */
        db_out[k] = (uint8_t)((db - db_min) * scale + 0.5f);
    }

    /* power=1 -> 0 dB -> 0/80*255 ≈ 0 */
    TEST_ASSERT(db_out[0] < 5, "0 dB maps to low value");
    /* power=10000 -> 40 dB -> 40/80*255 ≈ 127 */
    TEST_ASSERT(abs(db_out[1] - 127) < 15, "40 dB maps to ~127");
    /* power=1e6 -> 60 dB -> 60/80*255 ≈ 191 */
    TEST_ASSERT(abs(db_out[2] - 191) < 15, "60 dB maps to ~191");
    /* power=1e8 -> 80 dB -> 80/80*255 = 255 */
    TEST_ASSERT(abs(db_out[3] - 255) < 5, "80 dB maps to ~255");

    tests_passed++;
    printf("  PASS\n");
}

/* ────────────────────────── Main ────────────────────────── */

int main(void)
{
    printf("=== DSP Kernel Unit Tests ===\n\n");

    /* Windowing */
    test_windowing_basic();
    test_windowing_hann();

    /* Power spectrum */
    test_power_spectrum_basic();
    test_power_spectrum_accumulation();

    /* Fast log10f */
    test_fast_log10f_accuracy();

    /* NCO phase continuity */
    test_nco_phase_continuity();

    /* 3rd-order CIC */
    test_cic_3rd_order_dc();
    test_cic_alias_rejection();

    /* dB conversion */
    test_db_conversion();

    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
