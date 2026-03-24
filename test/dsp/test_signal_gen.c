#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define SAMPLE_RATE 250000  /* Default RTL-SDR sample rate */
#define IQ_CENTER   128     /* RTL-SDR uint8 center value */

/* ── AWGN (Additive White Gaussian Noise) ─────────────── */

/* Box-Muller transform for Gaussian random numbers */
static double gaussian_rand(void) {
    static int has_spare = 0;
    static double spare;
    if (has_spare) { has_spare = 0; return spare; }
    double u, v, s;
    do {
        u = (double)rand() / RAND_MAX * 2.0 - 1.0;
        v = (double)rand() / RAND_MAX * 2.0 - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);
    s = sqrt(-2.0 * log(s) / s);
    spare = v * s;
    has_spare = 1;
    return u * s;
}

/* Add AWGN to IQ buffer at specified SNR (dB) */
static void add_noise(uint8_t *iq, int num_samples, double snr_db) {
    /* Calculate signal power (assume signal amplitude ~50 from center) */
    double sig_amp = 50.0;
    double noise_amp = sig_amp / pow(10.0, snr_db / 20.0);
    for (int i = 0; i < num_samples * 2; i++) {
        double val = (double)iq[i] + gaussian_rand() * noise_amp;
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        iq[i] = (uint8_t)val;
    }
}

/* ── Signal Generators ────────────────────────────────── */

/* Generate pure CW tone at offset_hz from center, duration_ms */
static int gen_tone(uint8_t *iq, int sample_rate, double offset_hz,
                    int duration_ms, double amplitude) {
    int num_samples = sample_rate * duration_ms / 1000;
    for (int i = 0; i < num_samples; i++) {
        double t = (double)i / sample_rate;
        double phase = 2.0 * M_PI * offset_hz * t;
        iq[i * 2]     = (uint8_t)(IQ_CENTER + amplitude * cos(phase));  /* I */
        iq[i * 2 + 1] = (uint8_t)(IQ_CENTER + amplitude * sin(phase));  /* Q */
    }
    return num_samples;
}

/* Generate AFSK signal (Bell 202: mark=1200Hz, space=2200Hz at 1200 baud)
 * bits: array of 0/1 values, num_bits: count
 * The output is FM-modulated IQ at the given carrier offset */
static int gen_afsk(uint8_t *iq, int sample_rate, double carrier_offset_hz,
                    const uint8_t *bits, int num_bits,
                    double mark_hz, double space_hz, int baud_rate, double amplitude) {
    int samples_per_bit = sample_rate / baud_rate;
    int sample_idx = 0;

    double phase = 0.0;

    for (int bit = 0; bit < num_bits; bit++) {
        double tone_hz = bits[bit] ? mark_hz : space_hz;
        double freq = carrier_offset_hz + tone_hz;  /* Total frequency offset */
        for (int s = 0; s < samples_per_bit; s++) {
            phase += 2.0 * M_PI * freq / sample_rate;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
            iq[sample_idx * 2]     = (uint8_t)(IQ_CENTER + amplitude * cos(phase));
            iq[sample_idx * 2 + 1] = (uint8_t)(IQ_CENTER + amplitude * sin(phase));
            sample_idx++;
        }
    }
    return sample_idx;
}

/* Generate FSK signal (e.g., POCSAG: +/-4500Hz shift) */
static int gen_fsk(uint8_t *iq, int sample_rate, double carrier_offset_hz,
                   const uint8_t *bits, int num_bits,
                   double shift_hz, int baud_rate, double amplitude) {
    int samples_per_bit = sample_rate / baud_rate;
    double phase = 0.0;
    int sample_idx = 0;

    for (int bit = 0; bit < num_bits; bit++) {
        double freq = carrier_offset_hz + (bits[bit] ? shift_hz / 2 : -shift_hz / 2);
        for (int s = 0; s < samples_per_bit; s++) {
            phase += 2.0 * M_PI * freq / sample_rate;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
            iq[sample_idx * 2]     = (uint8_t)(IQ_CENTER + amplitude * cos(phase));
            iq[sample_idx * 2 + 1] = (uint8_t)(IQ_CENTER + amplitude * sin(phase));
            sample_idx++;
        }
    }
    return sample_idx;
}

/* Generate OOK (on-off keying) signal */
static int gen_ook(uint8_t *iq, int sample_rate, double carrier_offset_hz,
                   const uint8_t *bits, int num_bits,
                   int symbol_rate, double amplitude) {
    int samples_per_symbol = sample_rate / symbol_rate;
    double phase = 0.0;
    int sample_idx = 0;

    for (int bit = 0; bit < num_bits; bit++) {
        for (int s = 0; s < samples_per_symbol; s++) {
            phase += 2.0 * M_PI * carrier_offset_hz / sample_rate;
            double amp = bits[bit] ? amplitude : 0.0;
            iq[sample_idx * 2]     = (uint8_t)(IQ_CENTER + amp * cos(phase));
            iq[sample_idx * 2 + 1] = (uint8_t)(IQ_CENTER + amp * sin(phase));
            sample_idx++;
        }
    }
    return sample_idx;
}

/* Generate CW Morse keying (dit/dah pattern with tone) */
static int gen_morse(uint8_t *iq, int sample_rate, double tone_offset_hz,
                     const char *text, int wpm, double amplitude) {
    /* PARIS standard: dit = 1200/wpm ms */
    int dit_ms = 1200 / wpm;
    int dit_samples = sample_rate * dit_ms / 1000;
    int sample_idx = 0;
    double phase = 0.0;

    /* Simple morse table for A-Z and 0-9 */
    static const char *morse[] = {
        ".-","-...","-.-.","-..",".","..-.","--.","....","..",".---",
        "-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-",
        "..-","...-",".--","-..-","-.--","--..",
        "-----",".----","..---","...--","....-",".....",
        "-....","--...","---..","----."
    };

    for (const char *c = text; *c; c++) {
        const char *pattern = NULL;
        if (*c >= 'A' && *c <= 'Z') pattern = morse[*c - 'A'];
        else if (*c >= 'a' && *c <= 'z') pattern = morse[*c - 'a'];
        else if (*c >= '0' && *c <= '9') pattern = morse[*c - '0' + 26];
        else if (*c == ' ') {
            /* Word space: 7 dits (already 3 from inter-char, add 4 more) */
            for (int i = 0; i < dit_samples * 4; i++) {
                iq[sample_idx * 2] = IQ_CENTER;
                iq[sample_idx * 2 + 1] = IQ_CENTER;
                sample_idx++;
            }
            continue;
        }
        if (!pattern) continue;

        for (const char *p = pattern; *p; p++) {
            int len = (*p == '.') ? dit_samples : dit_samples * 3;
            /* Tone on */
            for (int i = 0; i < len; i++) {
                phase += 2.0 * M_PI * tone_offset_hz / sample_rate;
                iq[sample_idx * 2]     = (uint8_t)(IQ_CENTER + amplitude * cos(phase));
                iq[sample_idx * 2 + 1] = (uint8_t)(IQ_CENTER + amplitude * sin(phase));
                sample_idx++;
            }
            /* Inter-element gap: 1 dit */
            for (int i = 0; i < dit_samples; i++) {
                iq[sample_idx * 2] = IQ_CENTER;
                iq[sample_idx * 2 + 1] = IQ_CENTER;
                sample_idx++;
            }
        }
        /* Inter-character gap: 3 dits (already 1 from element, add 2 more) */
        for (int i = 0; i < dit_samples * 2; i++) {
            iq[sample_idx * 2] = IQ_CENTER;
            iq[sample_idx * 2 + 1] = IQ_CENTER;
            sample_idx++;
        }
    }
    return sample_idx;
}

/* Generate DTMF dual-tone (row + col frequencies) */
static int gen_dtmf_digit(uint8_t *iq, int sample_rate, char digit,
                           int duration_ms, double amplitude) {
    static const int row_freq[] = {697, 770, 852, 941};
    static const int col_freq[] = {1209, 1336, 1477, 1633};
    static const char keys[4][4] = {
        {'1','2','3','A'}, {'4','5','6','B'},
        {'7','8','9','C'}, {'*','0','#','D'}
    };

    int row = -1, col = -1;
    for (int r = 0; r < 4 && row < 0; r++)
        for (int c = 0; c < 4; c++)
            if (keys[r][c] == digit) { row = r; col = c; break; }

    if (row < 0) return 0;

    int num_samples = sample_rate * duration_ms / 1000;
    for (int i = 0; i < num_samples; i++) {
        double t = (double)i / sample_rate;
        /* DTMF is audio -- represent as FM-modulated IQ around baseband */
        double audio = amplitude * 0.5 * (sin(2*M_PI*row_freq[row]*t) + sin(2*M_PI*col_freq[col]*t));
        /* Simple FM: phase = integral of audio */
        double phase = 2.0 * M_PI * t * 1000.0; /* carrier at 1kHz offset */
        iq[i*2]     = (uint8_t)(IQ_CENTER + 40.0 * cos(phase + audio * 5.0));
        iq[i*2 + 1] = (uint8_t)(IQ_CENTER + 40.0 * sin(phase + audio * 5.0));
    }
    return num_samples;
}

/* ── Simple DFT bin magnitude check ───────────────────── */

/* Compute magnitude of a single DFT bin at target_hz */
static double dft_bin_magnitude(const uint8_t *iq, int num_samples,
                                int sample_rate, double target_hz) {
    double sum_i = 0.0, sum_q = 0.0;
    for (int n = 0; n < num_samples; n++) {
        double i_val = (double)iq[n * 2] - IQ_CENTER;
        double q_val = (double)iq[n * 2 + 1] - IQ_CENTER;
        double complex_i = i_val;
        double complex_q = q_val;
        /* Analytic signal: I + jQ, correlate with e^(-j*2*pi*f*t) */
        double t = (double)n / sample_rate;
        double angle = 2.0 * M_PI * target_hz * t;
        double cos_a = cos(angle);
        double sin_a = sin(angle);
        /* (I+jQ) * (cos-jsin) = (I*cos+Q*sin) + j(Q*cos-I*sin) */
        sum_i += complex_i * cos_a + complex_q * sin_a;
        sum_q += complex_q * cos_a - complex_i * sin_a;
    }
    return sqrt(sum_i * sum_i + sum_q * sum_q) / num_samples;
}

/* ── Test Harness ─────────────────────────────────────── */

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

static void write_iq_file(const char *path, const uint8_t *iq, int num_samples) {
    FILE *f = fopen(path, "wb");
    if (f) {
        fwrite(iq, 1, num_samples * 2, f);
        fclose(f);
        printf("  Wrote %d IQ samples to %s\n", num_samples, path);
    } else {
        printf("  ERROR: Could not write %s\n", path);
    }
}

static void test_tone(void) {
    printf("\n=== Test: CW Tone Generator ===\n");
    uint8_t *iq = malloc(SAMPLE_RATE * 2);  /* 1 second max */
    int n = gen_tone(iq, SAMPLE_RATE, 5000.0, 100, 50.0);

    check("gen_tone returns expected sample count",
          n == SAMPLE_RATE * 100 / 1000);

    /* Check amplitude range */
    int min_val = 255, max_val = 0;
    for (int i = 0; i < n * 2; i++) {
        if (iq[i] < min_val) min_val = iq[i];
        if (iq[i] > max_val) max_val = iq[i];
    }
    check("tone amplitude in expected range",
          min_val >= (IQ_CENTER - 55) && max_val <= (IQ_CENTER + 55));

    /* DFT check: energy at 5000 Hz should be strong */
    double mag_on  = dft_bin_magnitude(iq, n, SAMPLE_RATE, 5000.0);
    double mag_off = dft_bin_magnitude(iq, n, SAMPLE_RATE, 50000.0);
    check("DFT: strong energy at 5kHz",  mag_on > 10.0);
    check("DFT: weak energy at 50kHz",   mag_off < mag_on * 0.1);

    write_iq_file("/tmp/test_tone_5khz.iq", iq, n);
    free(iq);
}

static void test_afsk(void) {
    printf("\n=== Test: AFSK Generator ===\n");
    uint8_t bits[] = {1,0,1,1,0,0,1,0,1,0,1,1,0,1,0,0};
    int num_bits = sizeof(bits);
    int max_samples = SAMPLE_RATE;  /* 1 sec buffer */
    uint8_t *iq = malloc(max_samples * 2);

    int n = gen_afsk(iq, SAMPLE_RATE, 0.0, bits, num_bits,
                     1200.0, 2200.0, 1200, 50.0);

    int expected = num_bits * (SAMPLE_RATE / 1200);
    check("gen_afsk returns expected sample count", n == expected);

    /* Check that samples are not all center (signal present) */
    int non_center = 0;
    for (int i = 0; i < n * 2; i++)
        if (iq[i] != IQ_CENTER) non_center++;
    check("AFSK signal has non-trivial content", non_center > n);

    write_iq_file("/tmp/test_afsk.iq", iq, n);
    free(iq);
}

static void test_fsk(void) {
    printf("\n=== Test: FSK Generator ===\n");
    uint8_t bits[] = {1,0,1,1,0,0,1,0};
    int num_bits = sizeof(bits);
    uint8_t *iq = malloc(SAMPLE_RATE * 2);

    int n = gen_fsk(iq, SAMPLE_RATE, 10000.0, bits, num_bits,
                    9000.0, 1200, 50.0);

    int expected = num_bits * (SAMPLE_RATE / 1200);
    check("gen_fsk returns expected sample count", n == expected);

    /* Verify signal energy exists */
    int non_center = 0;
    for (int i = 0; i < n * 2; i++)
        if (iq[i] != IQ_CENTER) non_center++;
    check("FSK signal has content", non_center > n);

    write_iq_file("/tmp/test_fsk.iq", iq, n);
    free(iq);
}

static void test_ook(void) {
    printf("\n=== Test: OOK Generator ===\n");
    uint8_t bits[] = {1,0,1,1,0,1,0,0,1,1};
    int num_bits = sizeof(bits);
    uint8_t *iq = malloc(SAMPLE_RATE * 2);

    int n = gen_ook(iq, SAMPLE_RATE, 5000.0, bits, num_bits, 1000, 50.0);

    int expected = num_bits * (SAMPLE_RATE / 1000);
    check("gen_ook returns expected sample count", n == expected);

    /* During '0' bits, amplitude should be near center */
    int sps = SAMPLE_RATE / 1000;
    int center_count = 0;
    /* Check second bit (index 1, which is 0) */
    for (int i = sps; i < sps * 2; i++) {
        if (iq[i * 2] == IQ_CENTER && iq[i * 2 + 1] == IQ_CENTER)
            center_count++;
    }
    check("OOK '0' bit produces silence", center_count == sps);

    write_iq_file("/tmp/test_ook.iq", iq, n);
    free(iq);
}

static void test_morse(void) {
    printf("\n=== Test: Morse Generator ===\n");
    uint8_t *iq = malloc(SAMPLE_RATE * 4);  /* up to 4 seconds */

    int n = gen_morse(iq, SAMPLE_RATE, 800.0, "SOS", 20, 50.0);

    check("gen_morse returns nonzero samples", n > 0);

    /* Check there is both signal and silence */
    int center_count = 0, signal_count = 0;
    for (int i = 0; i < n; i++) {
        if (iq[i * 2] == IQ_CENTER && iq[i * 2 + 1] == IQ_CENTER)
            center_count++;
        else
            signal_count++;
    }
    check("Morse has both tone and silence", center_count > 0 && signal_count > 0);

    write_iq_file("/tmp/test_morse_sos.iq", iq, n);
    free(iq);
}

static void test_dtmf(void) {
    printf("\n=== Test: DTMF Generator ===\n");
    uint8_t *iq = malloc(SAMPLE_RATE * 2);

    int n = gen_dtmf_digit(iq, SAMPLE_RATE, '5', 100, 50.0);
    check("gen_dtmf returns expected sample count",
          n == SAMPLE_RATE * 100 / 1000);

    /* Verify signal is not flat */
    int min_val = 255, max_val = 0;
    for (int i = 0; i < n * 2; i++) {
        if (iq[i] < min_val) min_val = iq[i];
        if (iq[i] > max_val) max_val = iq[i];
    }
    check("DTMF has amplitude variation", (max_val - min_val) > 20);

    /* Invalid digit should return 0 */
    int n2 = gen_dtmf_digit(iq, SAMPLE_RATE, 'X', 100, 50.0);
    check("DTMF invalid digit returns 0", n2 == 0);

    write_iq_file("/tmp/test_dtmf_5.iq", iq, n);
    free(iq);
}

static void test_noise(void) {
    printf("\n=== Test: AWGN Noise Injection ===\n");
    uint8_t *iq = malloc(SAMPLE_RATE * 2);

    /* Generate clean tone */
    int n = gen_tone(iq, SAMPLE_RATE, 5000.0, 100, 50.0);

    /* Make a copy and add noise at different SNRs */
    double snr_levels[] = {30.0, 20.0, 10.0, 0.0};
    for (int s = 0; s < 4; s++) {
        uint8_t *noisy = malloc(n * 2);
        memcpy(noisy, iq, n * 2);
        /* Re-generate clean tone for each test */
        gen_tone(noisy, SAMPLE_RATE, 5000.0, 100, 50.0);
        add_noise(noisy, n, snr_levels[s]);

        /* Check that signal is still detectable (DFT at 5kHz) */
        double mag = dft_bin_magnitude(noisy, n, SAMPLE_RATE, 5000.0);
        char label[64];
        snprintf(label, sizeof(label), "Tone detectable at SNR=%.0fdB (mag=%.1f)",
                 snr_levels[s], mag);
        check(label, mag > 1.0);

        char path[64];
        snprintf(path, sizeof(path), "/tmp/test_tone_snr%.0f.iq", snr_levels[s]);
        write_iq_file(path, noisy, n);
        free(noisy);
    }

    free(iq);
}

int main(int argc, char **argv) {
    srand((unsigned)time(NULL));

    printf("RTL-SDR Synthetic IQ Signal Generator Tests\n");
    printf("Sample rate: %d Hz\n", SAMPLE_RATE);

    test_tone();
    test_afsk();
    test_fsk();
    test_ook();
    test_morse();
    test_dtmf();
    test_noise();

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
