/*
 * test_noise_matrix.c - Decoder noise resilience test matrix
 *
 * Tests each decoder's core DSP algorithm at various SNR levels to
 * characterize noise tolerance thresholds.
 *
 * Build:  gcc -o test_noise_matrix test_noise_matrix.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* ── Box-Muller Gaussian RNG (from test_signal_gen.c) ── */

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

/* ── Goertzel single-bin power ───────────────────────── */

static double goertzel_power(const int16_t *samples, int N, int sr, int freq_hz) {
    double k = (double)N * freq_hz / sr;
    double w = 2.0 * M_PI * k / N;
    double coeff = 2.0 * cos(w);
    double s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < N; i++) {
        s0 = (double)samples[i] / 32768.0 + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    return s1*s1 + s2*s2 - coeff*s1*s2;
}

/* ── Popcount (Hamming weight) ───────────────────────── */

static int popcount32(uint32_t x) {
    int c = 0;
    while (x) { c += x & 1; x >>= 1; }
    return c;
}

/* ── DTMF at SNR ─────────────────────────────────────── */

static int test_dtmf_at_snr(double snr_db) {
    int sr = 8000;
    int N = 800;  /* 100ms block for robust detection */
    int16_t *audio = calloc(N, sizeof(int16_t));

    /* Generate DTMF '5' (770 + 1336 Hz) */
    double sig_amp = 8000.0;
    double noise_amp = sig_amp / pow(10.0, snr_db / 20.0);
    for (int i = 0; i < N; i++) {
        double t = (double)i / sr;
        double signal = sig_amp * (sin(2*M_PI*770*t) + sin(2*M_PI*1336*t));
        double noise = gaussian_rand() * noise_amp;
        double val = signal + noise;
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        audio[i] = (int16_t)val;
    }

    /* Goertzel detection on all 8 DTMF frequencies */
    static const int dtmf_freqs[] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    double powers[8];
    for (int f = 0; f < 8; f++)
        powers[f] = goertzel_power(audio, N, sr, dtmf_freqs[f]);

    /* Find strongest row and column */
    int best_row = 0, best_col = 4;
    for (int r = 1; r < 4; r++) if (powers[r] > powers[best_row]) best_row = r;
    for (int c = 5; c < 8; c++) if (powers[c] > powers[best_col]) best_col = c;

    static const char keys[4][4] = {
        {'1','2','3','A'}, {'4','5','6','B'},
        {'7','8','9','C'}, {'*','0','#','D'}
    };
    char detected = keys[best_row][best_col - 4];

    free(audio);
    return (detected == '5') ? 1 : 0;
}

/* ── CW dit detection at SNR ─────────────────────────── */

static int test_cw_at_snr(double snr_db) {
    int sr = 8000;
    int dit_ms = 1200 / 15;  /* 80ms at 15 WPM */
    int dit_samples = sr * dit_ms / 1000;
    int total = dit_samples * 3;
    int16_t *audio = calloc(total, sizeof(int16_t));

    /* Generate dit with noise */
    double sig_amp = 16000.0;
    double noise_amp = sig_amp / pow(10.0, snr_db / 20.0);
    for (int i = 0; i < dit_samples; i++) {
        double t = (double)i / sr;
        double signal = sig_amp * sin(2 * M_PI * 700.0 * t);
        double noise = gaussian_rand() * noise_amp;
        double val = signal + noise;
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        audio[i] = (int16_t)val;
    }
    /* Silence portion also gets noise */
    for (int i = dit_samples; i < total; i++) {
        double val = gaussian_rand() * noise_amp;
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        audio[i] = (int16_t)val;
    }

    /* Goertzel on tone and silence blocks */
    double tone_power = goertzel_power(audio, dit_samples, sr, 700);

    int silence_len = dit_samples;
    if (dit_samples + silence_len > total) silence_len = total - dit_samples;
    double silence_power = goertzel_power(audio + dit_samples, silence_len, sr, 700);

    free(audio);

    double ratio = tone_power / (silence_power + 1e-10);
    return (ratio > 10.0) ? 1 : 0;  /* Relaxed threshold for noisy conditions */
}

/* ── ADS-B CRC at SNR (bit-flip noise model) ────────── */

static int test_adsb_at_snr(double snr_db) {
    /* Build a valid DF17 message */
    uint8_t msg[] = {0x8D, 0xA1, 0xB2, 0xC3, 0x20, 0x04, 0x64,
                     0xB6, 0xE8, 0x32, 0x00, 0x00, 0x00, 0x00};

    /* Compute CRC-24 and append */
    uint32_t crc = 0;
    for (int i = 0; i < 11; i++) {
        crc ^= ((uint32_t)msg[i]) << 16;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) crc = (crc << 1) ^ 0xFFF409;
            else crc = crc << 1;
        }
    }
    msg[11] = (crc >> 16) & 0xFF;
    msg[12] = (crc >> 8) & 0xFF;
    msg[13] = crc & 0xFF;

    /*
     * Noise model for digital: flip bits with probability proportional to BER.
     * At high SNR, BER ~ 0; at low SNR, simulate random bit flips.
     * BER approximation: erfc(sqrt(SNR_linear)) / 2
     */
    double snr_lin = pow(10.0, snr_db / 10.0);
    double ber = erfc(sqrt(snr_lin)) / 2.0;

    /* Apply bit flips */
    uint8_t noisy[14];
    memcpy(noisy, msg, 14);
    for (int i = 0; i < 14; i++) {
        for (int b = 0; b < 8; b++) {
            double r = (double)rand() / RAND_MAX;
            if (r < ber) noisy[i] ^= (1 << b);
        }
    }

    /* Verify CRC */
    uint32_t check = 0;
    for (int i = 0; i < 14; i++) {
        check ^= ((uint32_t)noisy[i]) << 16;
        for (int j = 0; j < 8; j++) {
            if (check & 0x800000) check = (check << 1) ^ 0xFFF409;
            else check = check << 1;
        }
    }
    check &= 0xFFFFFF;

    return (check == 0) ? 1 : 0;
}

/* ── AX.25 CRC at SNR (bit-flip noise model) ────────── */

static int test_ax25_at_snr(double snr_db) {
    /* Compute CRC for "TEST" */
    uint8_t data[] = {'T', 'E', 'S', 'T'};
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 4; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x8408;
            else crc >>= 1;
        }
    }
    crc ^= 0xFFFF;

    /* Pack data + CRC into a frame */
    uint8_t frame[6];
    memcpy(frame, data, 4);
    frame[4] = crc & 0xFF;
    frame[5] = (crc >> 8) & 0xFF;

    /* Apply bit-flip noise */
    double snr_lin = pow(10.0, snr_db / 10.0);
    double ber = erfc(sqrt(snr_lin)) / 2.0;

    uint8_t noisy[6];
    memcpy(noisy, frame, 6);
    for (int i = 0; i < 6; i++) {
        for (int b = 0; b < 8; b++) {
            double r = (double)rand() / RAND_MAX;
            if (r < ber) noisy[i] ^= (1 << b);
        }
    }

    /* Recompute CRC on received data and compare */
    uint16_t rx_crc = 0xFFFF;
    for (int i = 0; i < 4; i++) {
        rx_crc ^= noisy[i];
        for (int j = 0; j < 8; j++) {
            if (rx_crc & 1) rx_crc = (rx_crc >> 1) ^ 0x8408;
            else rx_crc >>= 1;
        }
    }
    rx_crc ^= 0xFFFF;

    uint16_t rx_expected = (uint16_t)noisy[4] | ((uint16_t)noisy[5] << 8);
    return (rx_crc == rx_expected) ? 1 : 0;
}

/* ── POCSAG sync detection at SNR (bit-flip noise) ──── */

static int test_pocsag_at_snr(double snr_db) {
    uint32_t sync = 0x7CD215D8;

    /* Apply bit-flip noise */
    double snr_lin = pow(10.0, snr_db / 10.0);
    double ber = erfc(sqrt(snr_lin)) / 2.0;

    uint32_t received = sync;
    for (int b = 0; b < 32; b++) {
        double r = (double)rand() / RAND_MAX;
        if (r < ber) received ^= (1u << b);
    }

    /* Detect sync with up to 2-bit error tolerance */
    int bit_errors = popcount32(sync ^ received);
    return (bit_errors <= 2) ? 1 : 0;
}

/* ── Noise Matrix Runner ─────────────────────────────── */

#define NUM_SNRS    4
#define NUM_TRIALS  20  /* Run multiple trials for statistical confidence */

typedef int (*snr_test_fn)(double snr_db);

typedef struct {
    const char *name;
    snr_test_fn test;
} decoder_test_t;

static void run_noise_matrix(void) {
    double snr_levels[NUM_SNRS] = {20.0, 10.0, 6.0, 3.0};

    decoder_test_t decoders[] = {
        {"DTMF '5'",     test_dtmf_at_snr},
        {"CW dit",       test_cw_at_snr},
        {"ADS-B CRC",    test_adsb_at_snr},
        {"AX.25 CRC",    test_ax25_at_snr},
        {"POCSAG sync",  test_pocsag_at_snr},
    };
    int num_decoders = sizeof(decoders) / sizeof(decoders[0]);

    /* Results: pass rate per decoder per SNR */
    int results[5][NUM_SNRS];

    for (int d = 0; d < num_decoders; d++) {
        for (int s = 0; s < NUM_SNRS; s++) {
            int passes = 0;
            for (int trial = 0; trial < NUM_TRIALS; trial++) {
                passes += decoders[d].test(snr_levels[s]);
            }
            /* Pass if majority of trials succeed */
            results[d][s] = (passes > NUM_TRIALS / 2) ? 1 : 0;
        }
    }

    /* Print matrix */
    printf("=== Noise Resilience Matrix (%d trials per cell) ===\n\n", NUM_TRIALS);
    printf("%-14s", "");
    for (int s = 0; s < NUM_SNRS; s++)
        printf("%6.0f dB  ", snr_levels[s]);
    printf("\n");

    int total_pass = 0, total_cells = 0;
    for (int d = 0; d < num_decoders; d++) {
        printf("%-14s", decoders[d].name);
        for (int s = 0; s < NUM_SNRS; s++) {
            printf("%-10s", results[d][s] ? "PASS" : "FAIL");
            total_pass += results[d][s];
            total_cells++;
        }
        printf("\n");
    }

    printf("\nCells passed: %d / %d\n", total_pass, total_cells);
}

/* ── Detailed per-SNR report ─────────────────────────── */

static void run_detailed_report(void) {
    double snr_levels[NUM_SNRS] = {20.0, 10.0, 6.0, 3.0};

    decoder_test_t decoders[] = {
        {"DTMF '5'",     test_dtmf_at_snr},
        {"CW dit",       test_cw_at_snr},
        {"ADS-B CRC",    test_adsb_at_snr},
        {"AX.25 CRC",    test_ax25_at_snr},
        {"POCSAG sync",  test_pocsag_at_snr},
    };
    int num_decoders = sizeof(decoders) / sizeof(decoders[0]);

    printf("\n=== Detailed Pass Rates ===\n\n");
    printf("%-14s", "");
    for (int s = 0; s < NUM_SNRS; s++)
        printf("%6.0f dB  ", snr_levels[s]);
    printf("\n");

    for (int d = 0; d < num_decoders; d++) {
        printf("%-14s", decoders[d].name);
        for (int s = 0; s < NUM_SNRS; s++) {
            int passes = 0;
            for (int trial = 0; trial < NUM_TRIALS; trial++)
                passes += decoders[d].test(snr_levels[s]);
            printf("%2d/%-2d     ", passes, NUM_TRIALS);
        }
        printf("\n");
    }
}

/* ── Main ────────────────────────────────────────────── */

int main(void) {
    srand((unsigned)time(NULL));

    printf("Decoder Noise Resilience Matrix\n");
    printf("========================================\n\n");

    run_noise_matrix();
    run_detailed_report();

    printf("\n========================================\n");
    return 0;
}
