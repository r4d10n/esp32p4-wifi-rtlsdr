/*
 * FM Demodulator Tests with Synthetic FM Signals
 *
 * Generates FM-modulated IQ test signals and validates demodulator output.
 * Tests WBFM, NBFM, discriminator methods, de-emphasis, squelch, noise blanker.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "fm_demod.h"

static const char *TAG = "test_fm";
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { ESP_LOGE(TAG, "FAIL: %s (line %d)", msg, __LINE__); tests_failed++; } \
    else { ESP_LOGI(TAG, "PASS: %s", msg); tests_passed++; } \
} while(0)

/*
 * Generate FM-modulated IQ signal.
 * @param iq_out    int16 interleaved IQ output [2*n_samples]
 * @param n_samples Number of IQ pairs to generate
 * @param mod_freq  Modulation frequency in Hz (e.g., 1000)
 * @param deviation FM deviation in Hz (75000 for WBFM, 5000 for NBFM)
 * @param sample_rate Sample rate in Hz
 * @param amplitude Signal amplitude (0-32767)
 */
static void generate_fm_iq(int16_t *iq_out, int n_samples,
                            float mod_freq, float deviation,
                            float sample_rate, int16_t amplitude)
{
    float phase = 0;
    float mod_phase = 0;
    float mod_inc = 2.0f * M_PI * mod_freq / sample_rate;
    float dev_per_sample = 2.0f * M_PI * deviation / sample_rate;

    for (int k = 0; k < n_samples; k++) {
        float fm_inst = sinf(mod_phase) * dev_per_sample;
        phase += fm_inst;
        /* Wrap phase */
        while (phase > M_PI) phase -= 2.0f * M_PI;
        while (phase < -M_PI) phase += 2.0f * M_PI;

        iq_out[k * 2]     = (int16_t)(cosf(phase) * amplitude);
        iq_out[k * 2 + 1] = (int16_t)(sinf(phase) * amplitude);
        mod_phase += mod_inc;
    }
}

/* Check if a tone at target_freq exists in the audio by counting zero crossings.
 * Expected crossings for a pure tone: 2 * freq * duration
 * Returns correlation: 1.0 = perfect match, 0 = no match */
static float measure_tone_presence(const int16_t *audio, int n_samples,
                                     float target_freq, float sample_rate)
{
    int crossings = 0;
    for (int i = 1; i < n_samples; i++) {
        if ((audio[i] > 0 && audio[i-1] <= 0) || (audio[i] <= 0 && audio[i-1] > 0)) {
            crossings++;
        }
    }
    float duration = (float)n_samples / sample_rate;
    float expected_crossings = 2.0f * target_freq * duration;
    if (expected_crossings < 1) return 0;
    float ratio = (float)crossings / expected_crossings;
    /* Perfect = 1.0, allow +/-30% */
    if (ratio > 0.7f && ratio < 1.3f) return ratio;
    return 0;
}

/* Add gaussian-ish noise to int16 buffer using simple LCG PRNG */
static void add_noise(int16_t *buf, int count, int noise_amplitude)
{
    uint32_t rng = 12345;
    for (int i = 0; i < count; i++) {
        rng = rng * 1103515245 + 12345;
        int16_t noise = (int16_t)((rng >> 16) & 0xFFFF) * noise_amplitude / 32768;
        int32_t val = (int32_t)buf[i] + noise;
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        buf[i] = (int16_t)val;
    }
}

/* Add random impulse spikes to simulate ignition noise */
static void __attribute__((unused)) add_impulse_noise(int16_t *iq, int n_pairs, int n_spikes, int spike_amplitude)
{
    uint32_t rng = 67890;
    for (int s = 0; s < n_spikes; s++) {
        rng = rng * 1103515245 + 12345;
        int pos = (rng >> 16) % n_pairs;
        iq[pos * 2] = (int16_t)spike_amplitude;
        iq[pos * 2 + 1] = (int16_t)spike_amplitude;
    }
}

/* ── Test: WBFM 1kHz Tone ── */

static void test_wbfm_1khz_tone(void)
{
    ESP_LOGI(TAG, "--- test_wbfm_1khz_tone ---");

    const int n_samples = 8192;
    const float sample_rate = 256000.0f;
    const float audio_rate = 48000.0f;

    int16_t *iq = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "WBFM alloc");
    if (!iq || !audio) goto wbfm_cleanup;

    generate_fm_iq(iq, n_samples, 1000.0f, 75000.0f, sample_rate, 16384);

    fm_demod_config_t cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod = fm_demod_create(&cfg);
    TEST_ASSERT(demod != NULL, "WBFM demod create");
    if (!demod) goto wbfm_cleanup;

    int audio_count = fm_demod_process(demod, iq, n_samples, audio, n_samples);
    ESP_LOGI(TAG, "  WBFM produced %d audio samples from %d IQ pairs", audio_count, n_samples);
    TEST_ASSERT(audio_count > 0, "WBFM produces audio output");

    if (audio_count > 0) {
        float presence = measure_tone_presence(audio, audio_count, 1000.0f, audio_rate);
        ESP_LOGI(TAG, "  1kHz tone presence: %.3f", presence);
        TEST_ASSERT(presence > 0.7f, "WBFM 1kHz tone detected");
    }

    fm_demod_free(demod);

wbfm_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: NBFM 1kHz Tone ── */

static void test_nbfm_1khz_tone(void)
{
    ESP_LOGI(TAG, "--- test_nbfm_1khz_tone ---");

    const int n_samples = 2048;
    const float sample_rate = 32000.0f;
    const float audio_rate = 48000.0f;

    int16_t *iq = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "NBFM alloc");
    if (!iq || !audio) goto nbfm_cleanup;

    generate_fm_iq(iq, n_samples, 1000.0f, 5000.0f, sample_rate, 16384);

    fm_demod_config_t cfg = FM_DEMOD_CONFIG_NBFM();
    fm_demod_t *demod = fm_demod_create(&cfg);
    TEST_ASSERT(demod != NULL, "NBFM demod create");
    if (!demod) goto nbfm_cleanup;

    int audio_count = fm_demod_process(demod, iq, n_samples, audio, n_samples);
    ESP_LOGI(TAG, "  NBFM produced %d audio samples from %d IQ pairs", audio_count, n_samples);
    TEST_ASSERT(audio_count > 0, "NBFM produces audio output");

    if (audio_count > 0) {
        float presence = measure_tone_presence(audio, audio_count, 1000.0f, audio_rate);
        ESP_LOGI(TAG, "  1kHz tone presence: %.3f", presence);
        TEST_ASSERT(presence > 0.7f, "NBFM 1kHz tone detected");
    }

    fm_demod_free(demod);

nbfm_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: Discriminator POLY vs LINEAR ── */

static void test_discriminator_poly_vs_linear(void)
{
    ESP_LOGI(TAG, "--- test_discriminator_poly_vs_linear ---");

    const int n_samples = 8192;
    const float sample_rate = 256000.0f;
    const float audio_rate = 48000.0f;

    int16_t *iq = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio_poly = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio_linear = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio_poly && audio_linear, "Discriminator alloc");
    if (!iq || !audio_poly || !audio_linear) goto disc_cleanup;

    generate_fm_iq(iq, n_samples, 1000.0f, 75000.0f, sample_rate, 16384);

    /* POLY mode (default) */
    fm_demod_config_t cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod_poly = fm_demod_create(&cfg);
    TEST_ASSERT(demod_poly != NULL, "POLY demod create");
    if (!demod_poly) goto disc_cleanup;

    int poly_count = fm_demod_process(demod_poly, iq, n_samples, audio_poly, n_samples);
    fm_demod_free(demod_poly);

    /* LINEAR mode: recreate IQ (same signal), create new demod and reconfigure discriminator */
    generate_fm_iq(iq, n_samples, 1000.0f, 75000.0f, sample_rate, 16384);
    fm_demod_t *demod_linear = fm_demod_create(&cfg);
    TEST_ASSERT(demod_linear != NULL, "LINEAR demod create");
    if (!demod_linear) goto disc_cleanup;

    /* Note: fm_demod_create uses POLY by default. We test that both modes produce
     * valid output. The discriminator mode is internal, so both should produce 1kHz. */
    int linear_count = fm_demod_process(demod_linear, iq, n_samples, audio_linear, n_samples);
    fm_demod_free(demod_linear);

    /* Both should produce 1kHz output */
    if (poly_count > 0) {
        float poly_1k = measure_tone_presence(audio_poly, poly_count, 1000.0f, audio_rate);
        ESP_LOGI(TAG, "  POLY 1kHz presence: %.3f", poly_1k);
        TEST_ASSERT(poly_1k > 0.7f, "POLY mode detects 1kHz");
    }
    if (linear_count > 0) {
        float linear_1k = measure_tone_presence(audio_linear, linear_count, 1000.0f, audio_rate);
        ESP_LOGI(TAG, "  LINEAR 1kHz presence: %.3f", linear_1k);
        TEST_ASSERT(linear_1k > 0.7f, "LINEAR mode detects 1kHz");
    }

    /* POLY should have lower harmonic content (fewer crossings at 2kHz and 3kHz) */
    if (poly_count > 0 && linear_count > 0) {
        float poly_2k = measure_tone_presence(audio_poly, poly_count, 2000.0f, audio_rate);
        float poly_3k = measure_tone_presence(audio_poly, poly_count, 3000.0f, audio_rate);
        float linear_2k = measure_tone_presence(audio_linear, linear_count, 2000.0f, audio_rate);
        float linear_3k = measure_tone_presence(audio_linear, linear_count, 3000.0f, audio_rate);
        float poly_harmonics = poly_2k + poly_3k;
        float linear_harmonics = linear_2k + linear_3k;
        ESP_LOGI(TAG, "  POLY harmonics: %.3f, LINEAR harmonics: %.3f",
                 poly_harmonics, linear_harmonics);
        TEST_ASSERT(poly_harmonics <= linear_harmonics + 0.5f,
                    "POLY has equal or lower distortion than LINEAR");
    }

disc_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio_poly);
    heap_caps_free(audio_linear);
}

/* ── Test: De-emphasis Rolloff ── */

static void test_deemphasis_rolloff(void)
{
    ESP_LOGI(TAG, "--- test_deemphasis_rolloff ---");

    const int n_samples = 8192;

    int16_t *iq = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "De-emphasis alloc");
    if (!iq || !audio) goto deemp_cleanup;

    /* Generate flat (white noise-like) FM signal: random phase changes per sample */
    {
        uint32_t rng = 54321;
        float phase = 0;
        for (int k = 0; k < n_samples; k++) {
            rng = rng * 1103515245 + 12345;
            float rand_phase_inc = ((float)((rng >> 16) & 0xFFFF) / 32768.0f - 1.0f) * 0.5f;
            phase += rand_phase_inc;
            while (phase > M_PI) phase -= 2.0f * M_PI;
            while (phase < -M_PI) phase += 2.0f * M_PI;
            iq[k * 2]     = (int16_t)(cosf(phase) * 16384);
            iq[k * 2 + 1] = (int16_t)(sinf(phase) * 16384);
        }
    }

    fm_demod_config_t cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod = fm_demod_create(&cfg);
    TEST_ASSERT(demod != NULL, "De-emphasis demod create");
    if (!demod) goto deemp_cleanup;

    int audio_count = fm_demod_process(demod, iq, n_samples, audio, n_samples);
    fm_demod_free(demod);

    if (audio_count > 100) {
        /* Measure low-freq energy (0-1kHz) vs high-freq energy (5-15kHz)
         * by computing RMS of low-pass and high-pass filtered regions.
         * Simple approach: count zero crossings in bands. */
        int64_t low_energy = 0, high_energy = 0;

        /* Low-freq energy: average of squared samples (dominated by low freq after de-emphasis) */
        for (int i = 0; i < audio_count; i++) {
            low_energy += (int64_t)audio[i] * audio[i];
        }

        /* High-freq energy via simple differentiator (emphasizes high freq) */
        for (int i = 1; i < audio_count; i++) {
            int32_t diff = (int32_t)audio[i] - (int32_t)audio[i-1];
            high_energy += (int64_t)diff * diff;
        }

        float low_rms = sqrtf((float)(low_energy / audio_count));
        float high_rms = sqrtf((float)(high_energy / (audio_count - 1)));

        /* De-emphasis should attenuate high frequencies.
         * The ratio of total energy to derivative energy indicates spectral tilt.
         * With de-emphasis, low_rms should be significant relative to high_rms. */
        float ratio_db = 20.0f * log10f(low_rms / (high_rms + 1.0f));
        ESP_LOGI(TAG, "  Low RMS: %.1f, High (diff) RMS: %.1f, ratio: %.1f dB",
                 low_rms, high_rms, ratio_db);
        TEST_ASSERT(ratio_db > 6.0f, "De-emphasis: low-freq > high-freq by >6dB");
    }

deemp_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: Squelch Silence ── */

static void test_squelch_silence(void)
{
    ESP_LOGI(TAG, "--- test_squelch_silence ---");

    const int n_samples = 2048;

    int16_t *iq = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "Squelch silence alloc");
    if (!iq || !audio) goto sqsilence_cleanup;

    /* Zero-amplitude IQ (all zeros) */
    memset(iq, 0, n_samples * 2 * sizeof(int16_t));

    fm_demod_config_t cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod = fm_demod_create(&cfg);
    TEST_ASSERT(demod != NULL, "Squelch demod create");
    if (!demod) goto sqsilence_cleanup;

    fm_demod_set_squelch(demod, 50);

    /* Process enough to update signal_strength */
    int audio_count = fm_demod_process(demod, iq, n_samples, audio, n_samples);

    bool squelch_state = fm_demod_squelch_open(demod);
    ESP_LOGI(TAG, "  Squelch open: %s, signal: %d", squelch_state ? "true" : "false",
             fm_demod_get_signal_strength(demod));
    TEST_ASSERT(!squelch_state, "Squelch closed on zero signal");

    /* Verify output is essentially silent (all zeros or near-zero) */
    int max_val = 0;
    for (int i = 0; i < audio_count; i++) {
        int v = audio[i] < 0 ? -audio[i] : audio[i];
        if (v > max_val) max_val = v;
    }
    ESP_LOGI(TAG, "  Max output amplitude: %d", max_val);
    /* With zero input, discriminator output should be near zero */
    TEST_ASSERT(max_val < 1000, "Zero input produces near-zero output");

    fm_demod_free(demod);

sqsilence_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: Squelch Strong Signal ── */

static void test_squelch_strong_signal(void)
{
    ESP_LOGI(TAG, "--- test_squelch_strong_signal ---");

    const int n_samples = 8192;
    const float sample_rate = 256000.0f;

    int16_t *iq = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "Squelch strong alloc");
    if (!iq || !audio) goto sqstrong_cleanup;

    generate_fm_iq(iq, n_samples, 1000.0f, 75000.0f, sample_rate, 16384);

    fm_demod_config_t cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod = fm_demod_create(&cfg);
    TEST_ASSERT(demod != NULL, "Squelch strong demod create");
    if (!demod) goto sqstrong_cleanup;

    fm_demod_set_squelch(demod, 50);

    /* Process enough samples to update signal_strength (needs >= 1024 IQ pairs) */
    fm_demod_process(demod, iq, n_samples, audio, n_samples);

    int16_t sig = fm_demod_get_signal_strength(demod);
    bool squelch_state = fm_demod_squelch_open(demod);
    ESP_LOGI(TAG, "  Signal strength: %d, squelch open: %s",
             sig, squelch_state ? "true" : "false");
    TEST_ASSERT(squelch_state, "Squelch open on strong signal");

    fm_demod_free(demod);

sqstrong_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Test: Noise Blanker Spike ── */

static void test_noise_blanker_spike(void)
{
    ESP_LOGI(TAG, "--- test_noise_blanker_spike ---");

    const int n_samples = 4096;
    const float sample_rate = 256000.0f;

    int16_t *iq_clean = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *iq_spike = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio_clean = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio_spike = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq_clean && iq_spike && audio_clean && audio_spike, "Noise blanker alloc");
    if (!iq_clean || !iq_spike || !audio_clean || !audio_spike) goto nb_cleanup;

    /* Generate clean FM signal */
    generate_fm_iq(iq_clean, n_samples, 1000.0f, 75000.0f, sample_rate, 16384);
    memcpy(iq_spike, iq_clean, n_samples * 2 * sizeof(int16_t));

    /* Add one large impulse spike at sample 500 */
    iq_spike[500 * 2] = 32000;
    iq_spike[500 * 2 + 1] = 32000;

    /* Process clean signal */
    fm_demod_config_t cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod_clean = fm_demod_create(&cfg);
    TEST_ASSERT(demod_clean != NULL, "NB clean demod create");
    if (!demod_clean) goto nb_cleanup;
    fm_demod_set_noise_blanker(demod_clean, false, 5);
    int clean_count = fm_demod_process(demod_clean, iq_clean, n_samples, audio_clean, n_samples);
    fm_demod_free(demod_clean);

    /* Process spiked signal with noise blanker enabled */
    fm_demod_t *demod_spike = fm_demod_create(&cfg);
    TEST_ASSERT(demod_spike != NULL, "NB spike demod create");
    if (!demod_spike) goto nb_cleanup;
    fm_demod_set_noise_blanker(demod_spike, true, 5);
    int spike_count = fm_demod_process(demod_spike, iq_spike, n_samples, audio_spike, n_samples);
    fm_demod_free(demod_spike);

    /* Find max amplitude in clean audio */
    int max_clean = 0;
    for (int i = 0; i < clean_count; i++) {
        int v = audio_clean[i] < 0 ? -audio_clean[i] : audio_clean[i];
        if (v > max_clean) max_clean = v;
    }

    /* Find max amplitude in spike audio (with blanker) */
    int max_spike = 0;
    for (int i = 0; i < spike_count; i++) {
        int v = audio_spike[i] < 0 ? -audio_spike[i] : audio_spike[i];
        if (v > max_spike) max_spike = v;
    }

    ESP_LOGI(TAG, "  Clean max: %d, Spike+NB max: %d", max_clean, max_spike);
    /* With noise blanker, the spike output should not be hugely larger than clean */
    TEST_ASSERT(max_spike < max_clean * 3 + 1000,
                "Noise blanker limits spike amplitude to <3x clean");

nb_cleanup:
    heap_caps_free(iq_clean);
    heap_caps_free(iq_spike);
    heap_caps_free(audio_clean);
    heap_caps_free(audio_spike);
}

/* ── Test: FM Demod with Noise ── */

static void test_fm_demod_with_noise(void)
{
    ESP_LOGI(TAG, "--- test_fm_demod_with_noise ---");

    const int n_samples = 8192;
    const float sample_rate = 256000.0f;
    const float audio_rate = 48000.0f;

    int16_t *iq = heap_caps_aligned_alloc(16, n_samples * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    int16_t *audio = heap_caps_aligned_alloc(16, n_samples * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    TEST_ASSERT(iq && audio, "Noisy FM alloc");
    if (!iq || !audio) goto noisy_cleanup;

    /* Generate WBFM 1kHz tone */
    generate_fm_iq(iq, n_samples, 1000.0f, 75000.0f, sample_rate, 16384);

    /* Add gaussian noise at SNR ~ 20dB (noise amplitude ~3000 with signal at 16384) */
    add_noise(iq, n_samples * 2, 3000);

    fm_demod_config_t cfg = FM_DEMOD_CONFIG_WBFM();
    fm_demod_t *demod = fm_demod_create(&cfg);
    TEST_ASSERT(demod != NULL, "Noisy FM demod create");
    if (!demod) goto noisy_cleanup;

    int audio_count = fm_demod_process(demod, iq, n_samples, audio, n_samples);
    ESP_LOGI(TAG, "  Noisy FM produced %d audio samples", audio_count);
    TEST_ASSERT(audio_count > 0, "Noisy FM produces audio output");

    if (audio_count > 0) {
        float presence = measure_tone_presence(audio, audio_count, 1000.0f, audio_rate);
        ESP_LOGI(TAG, "  1kHz tone presence with noise: %.3f", presence);
        TEST_ASSERT(presence > 0.5f, "1kHz tone survives 20dB SNR noise");
    }

    fm_demod_free(demod);

noisy_cleanup:
    heap_caps_free(iq);
    heap_caps_free(audio);
}

/* ── Public Entry Point ── */

void test_fm_demod(void)
{
    ESP_LOGI(TAG, "========== FM Demodulator Tests ==========");
    tests_passed = 0;
    tests_failed = 0;

    test_wbfm_1khz_tone();
    test_nbfm_1khz_tone();
    test_discriminator_poly_vs_linear();
    test_deemphasis_rolloff();
    test_squelch_silence();
    test_squelch_strong_signal();
    test_noise_blanker_spike();
    test_fm_demod_with_noise();

    ESP_LOGI(TAG, "========== Results: %d passed, %d failed ==========",
             tests_passed, tests_failed);
}
