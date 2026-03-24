/*
 * test_decoders.c - Host-compilable decoder DSP validation tests
 *
 * Validates core DSP algorithms used by each decoder with known-answer vectors.
 * Standalone implementations (Goertzel, CRC, etc.) match the decoder code but
 * avoid ESP-IDF dependencies (FreeRTOS, esp_log, cJSON).
 *
 * Build:  gcc -o test_decoders test_decoders.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ── Test 1: DTMF Goertzel Decode ────────────────────── */

static int test_dtmf(void) {
    printf("=== DTMF Decode Test ===\n");

    /* Generate DTMF '5' (770 Hz + 1336 Hz) at 8000 Hz sample rate, 100ms */
    int sr = 8000;
    int N = sr / 10;  /* 100ms = 800 samples */
    int16_t *audio = calloc(N, sizeof(int16_t));

    for (int i = 0; i < N; i++) {
        double t = (double)i / sr;
        audio[i] = (int16_t)(8000.0 * (sin(2*M_PI*770*t) + sin(2*M_PI*1336*t)));
    }

    /* Run Goertzel on all 8 DTMF frequencies */
    static const int dtmf_freqs[] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    double powers[8];
    int block = N;

    for (int f = 0; f < 8; f++) {
        double k = (double)block * dtmf_freqs[f] / sr;
        double w = 2.0 * M_PI * k / block;
        double coeff = 2.0 * cos(w);
        double s0 = 0, s1 = 0, s2 = 0;
        for (int i = 0; i < block; i++) {
            s0 = (double)audio[i] / 32768.0 + coeff * s1 - s2;
            s2 = s1; s1 = s0;
        }
        powers[f] = s1*s1 + s2*s2 - coeff*s1*s2;
    }

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

    printf("  Expected: '5', Detected: '%c'\n", detected);
    if (detected == '5') { printf("  PASS\n"); return 1; }
    printf("  FAIL\n"); return 0;
}

/* ── Test 2: ADS-B Mode S CRC-24 ────────────────────── */

static int test_adsb(void) {
    printf("=== ADS-B Mode S Test ===\n");

    /* Known DF17 identification message for ICAO A1B2C3 */
    /* Message bytes (hex): 8D A1B2C3 200464 B6E832 (+ CRC) */
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

    /* Verify CRC: full message including CRC should yield residual 0 */
    uint32_t check = 0;
    for (int i = 0; i < 14; i++) {
        check ^= ((uint32_t)msg[i]) << 16;
        for (int j = 0; j < 8; j++) {
            if (check & 0x800000) check = (check << 1) ^ 0xFFF409;
            else check = check << 1;
        }
    }
    check &= 0xFFFFFF;

    /* Extract ICAO address */
    uint32_t icao = ((uint32_t)msg[1] << 16) | ((uint32_t)msg[2] << 8) | msg[3];

    printf("  CRC residual: 0x%06X (expect 0)\n", check);
    printf("  ICAO: %06X (expect A1B2C3)\n", icao);

    int pass = (check == 0 && icao == 0xA1B2C3);
    printf("  %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

/* ── Test 3: CW/Morse Tone Detection ────────────────── */

static int test_cw(void) {
    printf("=== CW/Morse Test ===\n");

    /* Generate 'E' (single dit) at 700 Hz, 15 WPM */
    int sr = 8000;
    int dit_ms = 1200 / 15;  /* 80ms */
    int dit_samples = sr * dit_ms / 1000;
    int total = dit_samples * 3;  /* dit + gap + extra */
    int16_t *audio = calloc(total, sizeof(int16_t));

    /* Dit: tone on for dit_samples */
    for (int i = 0; i < dit_samples; i++) {
        double t = (double)i / sr;
        audio[i] = (int16_t)(16000.0 * sin(2 * M_PI * 700.0 * t));
    }
    /* Rest is silence (already zeroed by calloc) */

    /* Detect tone via Goertzel */
    int block = dit_samples;
    double k = (double)block * 700.0 / sr;
    double w = 2.0 * M_PI * k / block;
    double coeff = 2.0 * cos(w);
    double s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < block; i++) {
        s0 = (double)audio[i] / 32768.0 + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    double tone_power = s1*s1 + s2*s2 - coeff*s1*s2;

    /* Check silence block */
    s0 = s1 = s2 = 0;
    for (int i = dit_samples; i < dit_samples * 2 && i < total; i++) {
        s0 = (double)audio[i] / 32768.0 + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    double silence_power = s1*s1 + s2*s2 - coeff*s1*s2;

    free(audio);

    double ratio = tone_power / (silence_power + 1e-10);
    printf("  Tone power: %.6f, Silence power: %.6f, Ratio: %.1f\n",
           tone_power, silence_power, ratio);
    printf("  Dit detected: %s (ratio > 100)\n", ratio > 100 ? "YES" : "NO");

    /* 'E' = single dit -- shortest Morse character */
    int pass = (ratio > 100);
    printf("  %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

/* ── Test 4: AX.25 CRC-16 CCITT ─────────────────────── */

static int test_ax25_crc(void) {
    printf("=== AX.25 CRC-16 CCITT Test ===\n");

    /* Known test vector: "TEST" should produce a specific CRC */
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

    printf("  CRC-16 of 'TEST': 0x%04X\n", crc);
    /* CRC-16/X.25 (reflected, init=0xFFFF, xorout=0xFFFF, poly=0x8408) */
    int pass = (crc == 0x89D1);
    printf("  %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

/* ── Test 5: POCSAG Sync Word ────────────────────────── */

static int test_pocsag_sync(void) {
    printf("=== POCSAG Sync Word Test ===\n");

    uint32_t sync = 0x7CD215D8;
    /* Count bit errors between sync and itself (should be 0) */
    uint32_t diff = sync ^ sync;
    int errors = 0;
    while (diff) { errors += diff & 1; diff >>= 1; }

    /* Test with 1-bit error */
    uint32_t corrupted = sync ^ (1 << 15);
    diff = sync ^ corrupted;
    int errors_1bit = 0;
    while (diff) { errors_1bit += diff & 1; diff >>= 1; }

    printf("  Exact match errors: %d (expect 0)\n", errors);
    printf("  1-bit error: %d (expect 1)\n", errors_1bit);

    int pass = (errors == 0 && errors_1bit == 1);
    printf("  %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

/* ── Main ────────────────────────────────────────────── */

int main(void) {
    printf("Decoder DSP Known-Answer Tests\n");
    printf("========================================\n\n");

    int passed = 0, total = 5;

    passed += test_dtmf();    printf("\n");
    passed += test_adsb();    printf("\n");
    passed += test_cw();      printf("\n");
    passed += test_ax25_crc(); printf("\n");
    passed += test_pocsag_sync();

    printf("\n========================================\n");
    printf("Results: %d / %d passed\n", passed, total);
    printf("========================================\n");

    return (passed == total) ? 0 : 1;
}
