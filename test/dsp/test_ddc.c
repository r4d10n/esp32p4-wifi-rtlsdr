/*
 * Host-compilable DDC pipeline test.
 *
 * Compile:
 *   gcc -o test_ddc test_ddc.c ../../components/decoders/dsp_ddc.c -lm \
 *       -I../../components/decoders/include/
 *
 * The esp_err.h dependency is stubbed out below so we can build on host
 * without the ESP-IDF SDK.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "dsp_ddc.h"

#define SAMPLE_RATE 250000
#define IQ_CENTER   128

/* ── Tone generator (copied from test_signal_gen.c) ───── */
static int gen_tone(uint8_t *iq, int sample_rate, double offset_hz,
                    int duration_ms, double amplitude) {
    int num_samples = sample_rate * duration_ms / 1000;
    for (int i = 0; i < num_samples; i++) {
        double t = (double)i / sample_rate;
        double phase = 2.0 * M_PI * offset_hz * t;
        iq[i * 2]     = (uint8_t)(IQ_CENTER + amplitude * cos(phase));
        iq[i * 2 + 1] = (uint8_t)(IQ_CENTER + amplitude * sin(phase));
    }
    return num_samples;
}

/* ── Test harness ─────────────────────────────────────── */
static int tests_passed = 0;
static int tests_failed = 0;

static void check(const char *name, int condition) {
    if (condition) {
        printf("  PASS: %s\n", name);
        tests_passed++;
    } else {
        printf("  FAIL: %s\n", name);
        tests_failed++;
    }
}

/* Helper: compute RMS of audio buffer */
static double audio_rms(const int16_t *audio, int n) {
    if (n <= 0) return 0.0;
    int64_t energy = 0;
    for (int i = 0; i < n; i++)
        energy += (int64_t)audio[i] * audio[i];
    return sqrt((double)energy / n);
}

/* Test 1: Tone at +5000 Hz should produce non-zero audio output */
static void test_tone_in_band(void) {
    printf("\n=== Test: DDC with in-band tone (+5kHz) ===\n");

    int duration_ms = 200;
    int num_iq = SAMPLE_RATE * duration_ms / 1000;  /* 50000 samples */
    uint8_t *iq = malloc(num_iq * 2);
    gen_tone(iq, SAMPLE_RATE, 5000.0, duration_ms, 50.0);

    /* DDC: tune to +5000 Hz, bandwidth 3000 Hz, output 22050 Hz */
    ddc_state_t *ddc = ddc_create(SAMPLE_RATE, 5000.0, 3000, 22050);
    check("ddc_create succeeds", ddc != NULL);
    check("ddc_get_output_rate returns 22050",
          ddc_get_output_rate(ddc) == 22050);

    int max_out = num_iq;  /* generous output buffer */
    int16_t *audio = calloc(max_out, sizeof(int16_t));

    int n_out = ddc_process(ddc, iq, num_iq, audio, max_out);
    printf("  DDC produced %d audio samples from %d IQ samples\n", n_out, num_iq);
    check("DDC produces output samples", n_out > 0);

    /* Expected: ~num_iq / decimation = 50000 / (250000/22050) ~ 4410 */
    int expected_approx = num_iq / (SAMPLE_RATE / 22050);
    check("Output count in expected range",
          n_out >= expected_approx * 0.8 && n_out <= expected_approx * 1.2);

    /* Check that audio has energy (not all zeros) */
    int nonzero = 0;
    for (int i = 0; i < n_out; i++)
        if (audio[i] != 0) nonzero++;
    double rms = audio_rms(audio, n_out);
    printf("  In-band RMS: %.1f, nonzero samples: %d/%d\n", rms, nonzero, n_out);

    /* A CW tone through FM demod should produce near-DC (constant phase rate),
     * so audio may be small but not zero during transient.  Accept any nonzero. */
    check("Audio output has energy", nonzero > 0);

    ddc_destroy(ddc);
    free(iq);
    free(audio);
}

/* Test 2: Silence input (no signal) should produce near-zero audio */
static void test_silence(void) {
    printf("\n=== Test: DDC with silence (no signal) ===\n");

    int duration_ms = 200;
    int num_iq = SAMPLE_RATE * duration_ms / 1000;
    uint8_t *iq = malloc(num_iq * 2);
    /* Fill with DC center value -- no signal */
    memset(iq, IQ_CENTER, num_iq * 2);

    ddc_state_t *ddc = ddc_create(SAMPLE_RATE, 5000.0, 3000, 22050);
    int max_out = num_iq;
    int16_t *audio = calloc(max_out, sizeof(int16_t));

    int n_out = ddc_process(ddc, iq, num_iq, audio, max_out);
    double rms = audio_rms(audio, n_out);
    printf("  Silence RMS: %.1f (should be ~0)\n", rms);
    check("Silence produces near-zero audio (RMS < 10)", rms < 10.0);

    ddc_destroy(ddc);
    free(iq);
    free(audio);
}

/* Test 3: FM demod produces audio from a tone offset from DDC center,
 * and silence (near-zero) from a tone exactly at center.
 * This validates the NCO + FIR + FM discriminator chain. */
static void test_fm_demod_behavior(void) {
    printf("\n=== Test: DDC FM demod frequency response ===\n");

    int duration_ms = 200;
    int num_iq = SAMPLE_RATE * duration_ms / 1000;
    int max_out = num_iq;

    /* Tone exactly at DDC center -> FM demod should produce near-DC (low RMS) */
    uint8_t *iq1 = malloc(num_iq * 2);
    gen_tone(iq1, SAMPLE_RATE, 5000.0, duration_ms, 50.0);
    ddc_state_t *ddc1 = ddc_create(SAMPLE_RATE, 5000.0, 3000, 22050);
    int16_t *audio1 = calloc(max_out, sizeof(int16_t));
    int n1 = ddc_process(ddc1, iq1, num_iq, audio1, max_out);
    double rms_center = audio_rms(audio1, n1);

    /* Tone 1kHz off center (+6000 Hz) -> FM demod should produce 1kHz audio */
    uint8_t *iq2 = malloc(num_iq * 2);
    gen_tone(iq2, SAMPLE_RATE, 6000.0, duration_ms, 50.0);
    ddc_state_t *ddc2 = ddc_create(SAMPLE_RATE, 5000.0, 3000, 22050);
    int16_t *audio2 = calloc(max_out, sizeof(int16_t));
    int n2 = ddc_process(ddc2, iq2, num_iq, audio2, max_out);
    double rms_offset = audio_rms(audio2, n2);

    printf("  On-center RMS:  %.1f (expect low -- constant phase)\n", rms_center);
    printf("  Off-center RMS: %.1f (expect higher -- 1kHz FM audio)\n", rms_offset);

    /* On-center tone has constant phase -> FM demod ~ 0 */
    check("On-center tone produces low FM output (RMS < 200)", rms_center < 200);
    /* Off-center tone produces changing phase -> FM demod has audio */
    check("Off-center tone produces FM audio (RMS > 10)", rms_offset > 10);
    /* Off-center should be louder than on-center */
    check("Off-center RMS > on-center RMS", rms_offset > rms_center);

    ddc_destroy(ddc1);
    ddc_destroy(ddc2);
    free(iq1);
    free(iq2);
    free(audio1);
    free(audio2);
}

/* Test 3: Verify DDC handles NULL/edge cases */
static void test_edge_cases(void) {
    printf("\n=== Test: DDC edge cases ===\n");

    check("ddc_get_output_rate(NULL) returns 0",
          ddc_get_output_rate(NULL) == 0);

    /* Destroy NULL should not crash */
    ddc_destroy(NULL);
    check("ddc_destroy(NULL) does not crash", 1);

    /* Zero-length input */
    ddc_state_t *ddc = ddc_create(SAMPLE_RATE, 0.0, 5000, 22050);
    int16_t audio[1];
    int n = ddc_process(ddc, NULL, 0, audio, 1);
    check("Zero-length input produces zero output", n == 0);
    ddc_destroy(ddc);
}

int main(int argc, char **argv) {
    printf("DDC Pipeline Tests\n");
    printf("Input rate: %d Hz\n", SAMPLE_RATE);

    test_tone_in_band();
    test_silence();
    test_fm_demod_behavior();
    test_edge_cases();

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
