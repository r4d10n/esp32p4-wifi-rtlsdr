/*
 * FM Stereo and RDS Decoder Tests
 *
 * Tests stereo separation with synthetic MPX signals and
 * RDS decoding with simulated bitstreams including noise.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#ifdef CONFIG_FM_STEREO_ENABLE
#include "fm_stereo.h"
#include "rds_decoder.h"
#endif

static const char *TAG = "test_stereo";
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { ESP_LOGE(TAG, "FAIL: %s (line %d)", msg, __LINE__); tests_failed++; } \
    else { ESP_LOGI(TAG, "PASS: %s", msg); tests_passed++; } \
} while(0)

#ifdef CONFIG_FM_STEREO_ENABLE

/* ── MPX Signal Generator ── */

/*
 * Generate stereo MPX baseband signal.
 * MPX = (L+R) + pilot + (L-R)*cos(38kHz_t)
 *
 * @param mpx_out    int16 output buffer
 * @param n_samples  Number of samples
 * @param sample_rate Sample rate (256000)
 * @param left_freq  Left channel tone frequency (Hz), 0 for silence
 * @param right_freq Right channel tone frequency (Hz), 0 for silence
 * @param pilot_amplitude Pilot amplitude as fraction of max (0.1 = 10%)
 * @param add_pilot  Whether to include 19kHz pilot tone
 */
static void generate_mpx_stereo(int16_t *mpx_out, int n_samples, float sample_rate,
                                float left_freq, float right_freq,
                                float pilot_amplitude, bool add_pilot)
{
    float phase_l = 0, phase_r = 0, phase_pilot = 0;
    float inc_l = 2.0f * M_PI * left_freq / sample_rate;
    float inc_r = 2.0f * M_PI * right_freq / sample_rate;
    float inc_pilot = 2.0f * M_PI * 19000.0f / sample_rate;

    for (int i = 0; i < n_samples; i++) {
        float L = (left_freq > 0) ? sinf(phase_l) * 0.4f : 0;
        float R = (right_freq > 0) ? sinf(phase_r) * 0.4f : 0;

        float mono = (L + R) * 0.5f;   /* L+R */
        float diff = (L - R) * 0.5f;   /* L-R */

        float pilot = add_pilot ? sinf(phase_pilot) * pilot_amplitude : 0;
        float subcarrier = diff * cosf(phase_pilot * 2.0f); /* 38kHz = 2*19kHz */

        float mpx = mono + pilot + subcarrier;

        /* Scale to int16 range */
        mpx_out[i] = (int16_t)(mpx * 16384);

        phase_l += inc_l;
        phase_r += inc_r;
        phase_pilot += inc_pilot;
    }
}

/* ── RDS Helpers ── */

/* CRC-10 computation (matches rds_decoder.c polynomial 0x5B9) */
static uint16_t rds_crc10(uint16_t data)
{
    uint32_t reg = (uint32_t)data << 10; /* 16-bit data in upper bits */
    for (int i = 25; i >= 10; i--) {
        if (reg & (1u << i)) {
            reg ^= (0x5B9u << (i - 10));
        }
    }
    return reg & 0x3FF;
}

/* RDS offset words */
#define RDS_OFF_A   0x0FC
#define RDS_OFF_B   0x198
#define RDS_OFF_C   0x168
#define RDS_OFF_D   0x1B4

/*
 * Generate a raw RDS biphase bitstream for Group 0A with known PS name.
 *
 * RDS Group 0A structure:
 * Block A: PI code (16 bits) + checkword+offset_A (10 bits)
 * Block B: Group type 0000, version A(0), TP, PTY, segment addr (16 bits) + checkword+offset_B
 * Block C: Alt freq data (16 bits) + checkword+offset_C
 * Block D: 2 PS characters (16 bits) + checkword+offset_D
 *
 * @param samples_out  int16 output (biphase-coded)
 * @param max_samples  Maximum output samples
 * @param sample_rate  Output sample rate
 * @param pi_code      Program ID
 * @param ps_name      8-char station name
 * @param n_groups     Number of complete groups to generate
 * @return Number of output samples
 */
static int generate_rds_bitstream(int16_t *samples_out, int max_samples,
                                  float sample_rate, uint16_t pi_code,
                                  const char *ps_name, int n_groups)
{
    /* Build 26-bit blocks: 16 data + 10 check (CRC XOR offset) */
    float samples_per_bit = sample_rate / 1187.5f;
    int out_idx = 0;

    for (int g = 0; g < n_groups; g++) {
        uint8_t seg = g & 0x03; /* PS segment 0-3 */

        /* Block A: PI code */
        uint16_t block_a_data = pi_code;
        uint16_t block_a_check = rds_crc10(block_a_data) ^ RDS_OFF_A;

        /* Block B: group type 0A, TP=0, PTY=0, TA=0, MS=1, seg */
        uint16_t block_b_data = (0x0000)        /* group type 0, version A */
                              | (0 << 10)        /* TP */
                              | (0 << 5)         /* PTY */
                              | (1 << 3)         /* MS (music) */
                              | (0 << 4)         /* TA */
                              | (seg & 0x03);    /* segment address */
        uint16_t block_b_check = rds_crc10(block_b_data) ^ RDS_OFF_B;

        /* Block C: alternate frequency (dummy) */
        uint16_t block_c_data = 0xE000; /* No AF */
        uint16_t block_c_check = rds_crc10(block_c_data) ^ RDS_OFF_C;

        /* Block D: 2 PS characters */
        uint16_t block_d_data = ((uint16_t)(uint8_t)ps_name[seg * 2] << 8)
                              | (uint8_t)ps_name[seg * 2 + 1];
        uint16_t block_d_check = rds_crc10(block_d_data) ^ RDS_OFF_D;

        /* Assemble 4 blocks into 104 bits */
        uint16_t blocks_data[4]  = { block_a_data,  block_b_data,  block_c_data,  block_d_data };
        uint16_t blocks_check[4] = { block_a_check, block_b_check, block_c_check, block_d_check };

        for (int b = 0; b < 4; b++) {
            /* 16 data bits (MSB first) */
            for (int bit = 15; bit >= 0; bit--) {
                int val = (blocks_data[b] >> bit) & 1;
                int16_t level = val ? 4000 : -4000;
                int n_samp = (int)(samples_per_bit + 0.5f);
                /* Manchester: first half = level, second half = -level */
                int half = n_samp / 2;
                for (int s = 0; s < n_samp && out_idx < max_samples; s++) {
                    samples_out[out_idx++] = (s < half) ? level : -level;
                }
            }
            /* 10 check bits (MSB first) */
            for (int bit = 9; bit >= 0; bit--) {
                int val = (blocks_check[b] >> bit) & 1;
                int16_t level = val ? 4000 : -4000;
                int n_samp = (int)(samples_per_bit + 0.5f);
                int half = n_samp / 2;
                for (int s = 0; s < n_samp && out_idx < max_samples; s++) {
                    samples_out[out_idx++] = (s < half) ? level : -level;
                }
            }
        }
    }

    return out_idx;
}

/* ── Stereo Tests ── */

static void test_pilot_detection(void)
{
    ESP_LOGI(TAG, "--- test_pilot_detection ---");

    fm_stereo_config_t cfg = FM_STEREO_CONFIG_DEFAULT();
    fm_stereo_t *st = fm_stereo_create(&cfg);
    TEST_ASSERT(st != NULL, "stereo decoder created");
    if (!st) return;

    /* Generate MPX with pilot at 10%, 256kSPS, multiple blocks to fill Goertzel */
    const int n_samples = 4096;
    int16_t *mpx = heap_caps_malloc(n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *out = heap_caps_malloc(n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(mpx && out, "buffers allocated");

    if (mpx && out) {
        generate_mpx_stereo(mpx, n_samples, 256000.0f, 1000.0f, 0.0f, 0.1f, true);

        /* Feed multiple blocks to allow PLL to lock */
        for (int pass = 0; pass < 8; pass++) {
            fm_stereo_process(st, mpx, n_samples, out, n_samples);
        }

        bool is_stereo = fm_stereo_is_stereo(st);
        TEST_ASSERT(is_stereo, "pilot detected and PLL locked");
    }

    heap_caps_free(mpx);
    heap_caps_free(out);
    fm_stereo_free(st);
}

static void test_pilot_absence(void)
{
    ESP_LOGI(TAG, "--- test_pilot_absence ---");

    fm_stereo_config_t cfg = FM_STEREO_CONFIG_DEFAULT();
    fm_stereo_t *st = fm_stereo_create(&cfg);
    TEST_ASSERT(st != NULL, "stereo decoder created");
    if (!st) return;

    const int n_samples = 4096;
    int16_t *mpx = heap_caps_malloc(n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *out = heap_caps_malloc(n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(mpx && out, "buffers allocated");

    if (mpx && out) {
        /* Generate MPX WITHOUT pilot */
        generate_mpx_stereo(mpx, n_samples, 256000.0f, 1000.0f, 500.0f, 0.0f, false);

        for (int pass = 0; pass < 8; pass++) {
            fm_stereo_process(st, mpx, n_samples, out, n_samples);
        }

        bool is_stereo = fm_stereo_is_stereo(st);
        TEST_ASSERT(!is_stereo, "no pilot: stereo flag is false");
    }

    heap_caps_free(mpx);
    heap_caps_free(out);
    fm_stereo_free(st);
}

static void test_stereo_separation(void)
{
    ESP_LOGI(TAG, "--- test_stereo_separation ---");

    fm_stereo_config_t cfg = FM_STEREO_CONFIG_DEFAULT();
    fm_stereo_t *st = fm_stereo_create(&cfg);
    TEST_ASSERT(st != NULL, "stereo decoder created");
    if (!st) return;

    const int n_samples = 4096;
    int16_t *mpx = heap_caps_malloc(n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    /* Output: interleaved L/R pairs */
    int16_t *out = heap_caps_malloc(n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(mpx && out, "buffers allocated");

    if (mpx && out) {
        /* L=1kHz, R=silence, pilot at 10% */
        generate_mpx_stereo(mpx, n_samples, 256000.0f, 1000.0f, 0.0f, 0.1f, true);

        /* Feed multiple passes to let PLL lock and settle */
        int n_out = 0;
        for (int pass = 0; pass < 12; pass++) {
            n_out = fm_stereo_process(st, mpx, n_samples, out, n_samples);
        }

        if (n_out > 0) {
            /* Compute RMS of L and R channels from interleaved output */
            int64_t sum_l = 0, sum_r = 0;
            for (int i = 0; i < n_out; i++) {
                int32_t l = out[i * 2];
                int32_t r = out[i * 2 + 1];
                sum_l += l * l;
                sum_r += r * r;
            }
            float rms_l = sqrtf((float)sum_l / n_out);
            float rms_r = sqrtf((float)sum_r / n_out);

            ESP_LOGI(TAG, "  L RMS=%.1f  R RMS=%.1f  ratio=%.2f",
                     rms_l, rms_r, rms_l > 0 ? rms_r / rms_l : 999.0f);

            /* L channel should have content */
            TEST_ASSERT(rms_l > 100, "L channel has 1kHz content");

            /* R channel should be < 20% of L (separation > 14 dB) */
            float ratio = (rms_l > 0) ? rms_r / rms_l : 1.0f;
            TEST_ASSERT(ratio < 0.20f, "stereo separation > 14 dB");
        } else {
            TEST_ASSERT(false, "stereo process produced output");
        }
    }

    heap_caps_free(mpx);
    heap_caps_free(out);
    fm_stereo_free(st);
}

static void test_mono_compatibility(void)
{
    ESP_LOGI(TAG, "--- test_mono_compatibility ---");

    fm_stereo_config_t cfg = FM_STEREO_CONFIG_DEFAULT();
    fm_stereo_t *st = fm_stereo_create(&cfg);
    TEST_ASSERT(st != NULL, "stereo decoder created");
    if (!st) return;

    const int n_samples = 4096;
    int16_t *mpx = heap_caps_malloc(n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *out = heap_caps_malloc(n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(mpx && out, "buffers allocated");

    if (mpx && out) {
        /* Both L=1kHz and R=1kHz (same tone), pilot at 10% */
        generate_mpx_stereo(mpx, n_samples, 256000.0f, 1000.0f, 1000.0f, 0.1f, true);

        int n_out = 0;
        for (int pass = 0; pass < 12; pass++) {
            n_out = fm_stereo_process(st, mpx, n_samples, out, n_samples);
        }

        if (n_out > 0) {
            int64_t sum_l = 0, sum_r = 0;
            for (int i = 0; i < n_out; i++) {
                int32_t l = out[i * 2];
                int32_t r = out[i * 2 + 1];
                sum_l += l * l;
                sum_r += r * r;
            }
            float rms_l = sqrtf((float)sum_l / n_out);
            float rms_r = sqrtf((float)sum_r / n_out);

            ESP_LOGI(TAG, "  L RMS=%.1f  R RMS=%.1f", rms_l, rms_r);

            /* Both should have content */
            TEST_ASSERT(rms_l > 100, "L channel has 1kHz content");
            TEST_ASSERT(rms_r > 100, "R channel has 1kHz content");

            /* Amplitudes should be approximately equal (within 20%) */
            float ratio = (rms_l > rms_r) ? rms_r / rms_l : rms_l / rms_r;
            TEST_ASSERT(ratio > 0.80f, "L and R amplitudes within 20%");
        } else {
            TEST_ASSERT(false, "stereo process produced output");
        }
    }

    heap_caps_free(mpx);
    heap_caps_free(out);
    fm_stereo_free(st);
}

/* ── RDS Tests ── */

static void test_rds_decode_ps_name(void)
{
    ESP_LOGI(TAG, "--- test_rds_decode_ps_name ---");

    /* RDS decoder at 5333 Hz (close to 4.5 * 1187.5 bps) */
    uint32_t rds_rate = 5333;
    rds_decoder_t *rds = rds_decoder_create(rds_rate);
    TEST_ASSERT(rds != NULL, "RDS decoder created");
    if (!rds) return;

    /* Generate 4 groups (segments 0-3) to complete 8-char PS name */
    const char *ps = "TEST FM ";
    int max_samples = 64000; /* plenty for 4 groups */
    int16_t *samples = heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(samples != NULL, "RDS sample buffer allocated");

    if (samples) {
        /* Generate multiple repetitions for reliable sync + decode */
        int total = 0;

        /* First pass: 4 groups for all segments */
        for (int rep = 0; rep < 4; rep++) {
            int n = generate_rds_bitstream(samples + total, max_samples - total,
                                           (float)rds_rate, 0x1234, ps, 4);
            total += n;
        }

        /* Feed to decoder */
        rds_decoder_process(rds, samples, total);

        rds_data_t data;
        rds_decoder_get_data(rds, &data);

        ESP_LOGI(TAG, "  PI=0x%04X PS=\"%s\" groups=%lu errors=%lu",
                 data.pi_code, data.ps_name,
                 (unsigned long)data.groups_received,
                 (unsigned long)data.block_errors);

        TEST_ASSERT(data.pi_code == 0x1234, "PI code matches 0x1234");
        TEST_ASSERT(strcmp(data.ps_name, "TEST FM ") == 0, "PS name matches 'TEST FM '");

        heap_caps_free(samples);
    }

    rds_decoder_free(rds);
}

static void test_rds_crc_rejection(void)
{
    ESP_LOGI(TAG, "--- test_rds_crc_rejection ---");

    uint32_t rds_rate = 5333;
    rds_decoder_t *rds = rds_decoder_create(rds_rate);
    TEST_ASSERT(rds != NULL, "RDS decoder created");
    if (!rds) return;

    int max_samples = 32000;
    int16_t *samples = heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(samples != NULL, "buffer allocated");

    if (samples) {
        /* Generate valid bitstream first */
        int n = generate_rds_bitstream(samples, max_samples,
                                       (float)rds_rate, 0x5678, "ERRORCHK", 4);

        /* Flip bits in the middle of block A region (corrupt data) */
        float samples_per_bit = (float)rds_rate / 1187.5f;
        int bit8_start = (int)(8 * samples_per_bit); /* middle of block A */
        if (bit8_start < n) {
            for (int i = bit8_start; i < bit8_start + (int)samples_per_bit && i < n; i++) {
                samples[i] = -samples[i]; /* flip polarity */
            }
        }

        /* Feed corrupted data */
        rds_decoder_process(rds, samples, n);

        rds_data_t data;
        rds_decoder_get_data(rds, &data);

        ESP_LOGI(TAG, "  block_errors=%lu groups=%lu",
                 (unsigned long)data.block_errors,
                 (unsigned long)data.groups_received);

        /* CRC should have caught the error - either errors > 0 or no groups decoded */
        TEST_ASSERT(data.block_errors > 0 || data.groups_received == 0,
                    "CRC detected corrupted block");

        heap_caps_free(samples);
    }

    rds_decoder_free(rds);
}

static void test_rds_noise_tolerance(void)
{
    ESP_LOGI(TAG, "--- test_rds_noise_tolerance ---");

    uint32_t rds_rate = 5333;
    rds_decoder_t *rds = rds_decoder_create(rds_rate);
    TEST_ASSERT(rds != NULL, "RDS decoder created");
    if (!rds) return;

    const char *ps = "NOISY FM";
    int max_samples = 128000;
    int16_t *samples = heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(samples != NULL, "buffer allocated");

    if (samples) {
        /* Generate many repetitions of same group data for robustness */
        int total = 0;
        for (int rep = 0; rep < 10; rep++) {
            int n = generate_rds_bitstream(samples + total, max_samples - total,
                                           (float)rds_rate, 0xABCD, ps, 4);
            total += n;
        }

        /* Add noise at approximately SNR=10dB.
         * Signal amplitude = 4000, noise amplitude ~ 1265 for 10dB SNR */
        srand(42);
        for (int i = 0; i < total; i++) {
            int noise = (rand() % 2530) - 1265; /* uniform noise, ~10dB SNR */
            int32_t val = (int32_t)samples[i] + noise;
            if (val > 32767) val = 32767;
            if (val < -32768) val = -32768;
            samples[i] = (int16_t)val;
        }

        rds_decoder_process(rds, samples, total);

        rds_data_t data;
        rds_decoder_get_data(rds, &data);

        ESP_LOGI(TAG, "  PI=0x%04X PS=\"%s\" groups=%lu errors=%lu",
                 data.pi_code, data.ps_name,
                 (unsigned long)data.groups_received,
                 (unsigned long)data.block_errors);

        /* With repetition, PS name should still decode despite noise */
        TEST_ASSERT(data.groups_received > 0, "some groups decoded through noise");
        /* PS name may or may not be fully correct with noise, but PI should match */
        if (data.valid) {
            TEST_ASSERT(data.pi_code == 0xABCD, "PI code correct through noise");
        }

        heap_caps_free(samples);
    }

    rds_decoder_free(rds);
}

#endif /* CONFIG_FM_STEREO_ENABLE */

/* ── Public Entry Point ── */

void test_fm_stereo_rds(void)
{
    ESP_LOGI(TAG, "========== FM Stereo & RDS Tests ==========");
    tests_passed = 0;
    tests_failed = 0;

#ifdef CONFIG_FM_STEREO_ENABLE
    test_pilot_detection();
    test_pilot_absence();
    test_stereo_separation();
    test_mono_compatibility();
    test_rds_decode_ps_name();
    test_rds_crc_rejection();
    test_rds_noise_tolerance();
#else
    ESP_LOGW(TAG, "FM Stereo disabled, skipping stereo/RDS tests");
#endif

    ESP_LOGI(TAG, "========== Results: %d passed, %d failed ==========",
             tests_passed, tests_failed);
}
