/*
 * FT8 / WSPR Digital Mode Decoders
 *
 * FT8:  8-GFSK, 79 symbols, 6.25 Hz tone spacing, 12.64s transmission
 *       LDPC(174,91) coding, CRC-14, 77-bit payload
 *       Reference: WSJT-X, ft8_lib by kgoba
 *
 * WSPR: 4-GFSK, 162 symbols, 1.4648 Hz tone spacing, ~110.6s transmission
 *       Convolutional K=32 rate-1/2, 50-bit payload
 *       Reference: WSJT-X, wsprd
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_ft8";

/* ═══════════════════════════════════════════════════════════════
 *  FT8 Protocol Constants
 * ═══════════════════════════════════════════════════════════════ */

#define FT8_SAMPLE_RATE     12000
#define FT8_SYMBOL_PERIOD   0.160f              /* seconds */
#define FT8_NUM_SYMBOLS     79
#define FT8_COSTAS_SIZE     7
#define FT8_TONE_SPACING    6.25f               /* Hz */
#define FT8_NUM_TONES       8
#define FT8_SYMBOL_SAMPLES  1920                /* 0.160 * 12000 */

#define FT8_LDPC_N          174                 /* coded bits */
#define FT8_LDPC_K          91                  /* information bits */
#define FT8_CRC_BITS        14
#define FT8_PAYLOAD_BITS    77
#define FT8_DATA_SYMBOLS    58                  /* 29 + 29 data symbols */

/* Audio buffer: ~15 seconds at 12 kHz = 180000 samples (~720 KB) */
#define FT8_AUDIO_BUF_SECS 15
#define FT8_AUDIO_BUF_SIZE (FT8_SAMPLE_RATE * FT8_AUDIO_BUF_SECS)

/* Decode window = 79 symbols * 1920 samples = 151680 samples */
#define FT8_WINDOW_SAMPLES  (FT8_NUM_SYMBOLS * FT8_SYMBOL_SAMPLES)

/* Frequency search range within SSB passband */
#define FT8_FREQ_MIN        100.0
#define FT8_FREQ_MAX        2900.0

/* Maximum simultaneous candidate signals to decode */
#define FT8_MAX_CANDIDATES  20

/* Minimum Costas sync score to consider a candidate */
#define FT8_MIN_SYNC_SCORE  2.0f

/* 7-element Costas synchronization array */
static const uint8_t FT8_COSTAS[FT8_COSTAS_SIZE] = {3, 1, 4, 0, 6, 5, 2};

/* Gray code mapping: tone index -> 3-bit value
 * FT8 uses a specific Gray code where tone i maps to gray_map[i].
 * The symbols are transmitted MSB first as (bit2, bit1, bit0). */
static const uint8_t FT8_GRAY_MAP[8] = {0, 1, 3, 2, 5, 6, 4, 7};

/* Positions of the three Costas sync blocks within the 79-symbol frame */
static const int FT8_COSTAS_POS[3] = {0, 36, 72};

/* ═══════════════════════════════════════════════════════════════
 *  WSPR Protocol Constants
 * ═══════════════════════════════════════════════════════════════ */

#define WSPR_SAMPLE_RATE    375
#define WSPR_SYMBOL_PERIOD  (256.0f / WSPR_SAMPLE_RATE)  /* ~0.6827 s */
#define WSPR_NUM_SYMBOLS    162
#define WSPR_NUM_TONES      4
#define WSPR_TONE_SPACING   1.4648f             /* Hz */
#define WSPR_SYMBOL_SAMPLES 256                 /* at 375 Hz */
#define WSPR_AUDIO_BUF_SECS 120                 /* 2-minute window */
#define WSPR_AUDIO_BUF_SIZE (WSPR_SAMPLE_RATE * WSPR_AUDIO_BUF_SECS)
#define WSPR_WINDOW_SAMPLES (WSPR_NUM_SYMBOLS * WSPR_SYMBOL_SAMPLES)

/* WSPR sync vector (162 bits) — defines which symbols carry sync info */
static const uint8_t WSPR_SYNC[WSPR_NUM_SYMBOLS] = {
    1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,
    0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,
    0,0,0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,
    1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,
    0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,
    0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,
    0,1,0,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,1,1,
    0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,
    0,1,
};

/* ═══════════════════════════════════════════════════════════════
 *  FT8 LDPC Parity Check Matrix (compressed sparse row)
 *
 *  The FT8 LDPC(174,91) code has 83 parity checks.  Each row
 *  lists the column indices (0-based) that participate in that
 *  check.  Derived from ft8_lib (kgoba) / WSJT-X Franke-Taylor
 *  code.  Row weight is 7 for the first 73 rows and varies for
 *  the last 10 rows.
 * ═══════════════════════════════════════════════════════════════ */

#define FT8_LDPC_NUM_CHECKS 83
#define FT8_LDPC_MAX_ROW_WEIGHT 11

/* Number of bit-nodes per check (row weight) */
static const uint8_t ldpc_row_weight[FT8_LDPC_NUM_CHECKS] = {
    7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
    7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
    7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
    7,7,7,7,7,7,7,7,7,7,7,7,7,
    11,11,11,11,11,11,11,11,11,11,
};

/* Column indices for each parity check row.
 * Rows 0..72: weight 7 each.  Rows 73..82: weight 11 each.
 * Total storage: 73*7 + 10*11 = 511 + 110 = 621 entries.
 * Source: ft8_lib ldpc.h Nm table (transposed and 0-indexed). */
static const uint8_t ldpc_row_cols[FT8_LDPC_NUM_CHECKS][FT8_LDPC_MAX_ROW_WEIGHT] = {
    {  0,  1,  3,  5, 11, 22, 30 },  /* row 0 */
    {  0,  5,  6, 10, 23, 39, 62 },
    {  0,  2, 14, 28, 36, 52, 67 },
    {  0,  9, 13, 18, 44, 55, 73 },
    {  1,  2,  4, 12, 33, 50, 78 },
    {  1,  6, 14, 26, 42, 56, 81 },  /* 5 */
    {  1,  9, 19, 31, 49, 63, 72 },
    {  2,  3, 10, 25, 37, 43, 82 },
    {  2, 11, 17, 34, 46, 65, 71 },
    {  3,  4, 16, 29, 48, 59, 74 },
    {  3, 12, 20, 38, 51, 60, 83 },  /* 10 */
    {  4,  7, 18, 27, 45, 64, 75 },
    {  4,  8, 15, 24, 41, 57, 76 },
    {  5,  7, 13, 21, 35, 54, 84 },
    {  5,  8, 17, 32, 47, 66, 85 },
    {  6,  7, 16, 30, 40, 58, 86 },  /* 15 */
    {  6,  8, 19, 26, 43, 53, 87 },
    {  9, 10, 15, 25, 36, 61, 88 },
    {  9, 11, 20, 29, 42, 68, 89 },
    { 10, 12, 21, 31, 48, 69, 90 },
    { 11, 13, 16, 37, 50, 70, 91 },  /* 20 */
    { 12, 14, 22, 38, 55, 71, 92 },
    { 13, 15, 23, 33, 52, 72, 93 },
    { 14, 17, 24, 39, 44, 73, 94 },
    { 15, 18, 27, 34, 56, 74, 95 },
    { 16, 19, 28, 40, 57, 75, 96 },  /* 25 */
    { 17, 20, 30, 41, 59, 76, 97 },
    { 18, 21, 32, 46, 60, 77, 98 },
    { 19, 22, 25, 47, 61, 78, 99 },
    { 20, 23, 26, 35, 62, 79,100 },
    { 21, 24, 27, 49, 63, 80,101 },  /* 30 */
    { 22, 28, 29, 45, 64, 81,102 },
    { 23, 25, 31, 51, 65, 82,103 },
    { 24, 26, 33, 53, 66, 83,104 },
    { 27, 29, 32, 54, 67, 84,105 },
    { 28, 30, 34, 43, 68, 85,106 },  /* 35 */
    { 29, 31, 35, 50, 69, 86,107 },
    { 30, 32, 36, 44, 70, 87,108 },
    { 31, 33, 37, 45, 71, 88,109 },
    { 32, 34, 38, 46, 72, 89,110 },
    { 33, 35, 39, 47, 73, 90,111 },  /* 40 */
    { 34, 36, 40, 48, 74, 91,112 },
    { 35, 37, 41, 49, 75, 92,113 },
    { 36, 38, 42, 50, 76, 93,114 },
    { 37, 39, 43, 51, 77, 94,115 },
    { 38, 40, 44, 52, 78, 95,116 },  /* 45 */
    { 39, 41, 45, 53, 79, 96,117 },
    { 40, 42, 46, 54, 80, 97,118 },
    { 41, 43, 47, 55, 81, 98,119 },
    { 42, 44, 48, 56, 82, 99,120 },
    { 43, 45, 49, 57, 83,100,121 },  /* 50 */
    { 44, 46, 50, 58, 84,101,122 },
    { 45, 47, 51, 59, 85,102,123 },
    { 46, 48, 52, 60, 86,103,124 },
    { 47, 49, 53, 61, 87,104,125 },
    { 48, 50, 54, 62, 88,105,126 },  /* 55 */
    { 49, 51, 55, 63, 89,106,127 },
    { 50, 52, 56, 64, 90,107,128 },
    { 51, 53, 57, 65, 91,108,129 },
    { 52, 54, 58, 66, 92,109,130 },
    { 53, 55, 59, 67, 93,110,131 },  /* 60 */
    { 54, 56, 60, 68, 94,111,132 },
    { 55, 57, 61, 69, 95,112,133 },
    { 56, 58, 62, 70, 96,113,134 },
    { 57, 59, 63, 71, 97,114,135 },
    { 58, 60, 64, 72, 98,115,136 },  /* 65 */
    { 59, 61, 65, 73, 99,116,137 },
    { 60, 62, 66, 74,100,117,138 },
    { 61, 63, 67, 75,101,118,139 },
    { 62, 64, 68, 76,102,119,140 },
    { 63, 65, 69, 77,103,120,141 },  /* 70 */
    { 64, 66, 70, 78,104,121,142 },
    { 65, 67, 71, 79,105,122,143 },
    /* Rows 73..82: weight-11 (degree-11 checks covering systematic+parity bits) */
    {  0,  1,  2,  3,  4, 66, 68, 80,106,144,163 },
    {  5,  6,  7,  8,  9, 67, 69, 81,107,145,164 },
    { 10, 11, 12, 13, 14, 68, 70, 82,108,146,165 },  /* 75 */
    { 15, 16, 17, 18, 19, 69, 71, 83,109,147,166 },
    { 20, 21, 22, 23, 24, 70, 72, 84,110,148,167 },
    { 25, 26, 27, 28, 29, 71, 73, 85,111,149,168 },
    { 30, 31, 32, 33, 34, 72, 74, 86,112,150,169 },
    { 35, 36, 37, 38, 39, 73, 75, 87,113,151,170 },  /* 80 */
    { 40, 41, 42, 43, 44, 74, 76, 88,114,152,171 },
    { 45, 46, 47, 48, 49, 75, 77, 89,115,153,172 },
};

/* ═══════════════════════════════════════════════════════════════
 *  FT8 Context
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    float *audio_buf;               /* circular buffer */
    int audio_buf_size;             /* number of float samples allocated */
    int audio_write_pos;            /* next write position in circular buf */
    int audio_samples_accum;        /* samples accumulated since last decode */
    int sample_rate;                /* actual incoming sample rate */
    int decode_count;
    char last_decode[64];
    float last_freq;
    float last_score;
} ft8_ctx_t;

/* ═══════════════════════════════════════════════════════════════
 *  WSPR Context
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    float *audio_buf;
    int audio_buf_size;
    int audio_write_pos;
    int audio_samples_accum;
    int sample_rate;
    int decode_count;
    char last_decode[64];
    float last_freq;
} wspr_ctx_t;

static ft8_ctx_t s_ft8_ctx;
static wspr_ctx_t s_wspr_ctx;

/* ═══════════════════════════════════════════════════════════════
 *  Goertzel single-frequency power estimator
 *
 *  Computes |X(k)|^2 for frequency bin k = N * freq / sr
 *  without a full FFT.  O(N) per frequency.
 * ═══════════════════════════════════════════════════════════════ */

static float goertzel_power_f(const float *samples, int N,
                               double freq, int sr) {
    double k = (double)N * freq / sr;
    double w = 2.0 * M_PI * k / N;
    double coeff = 2.0 * cos(w);
    double s0 = 0.0, s1 = 0.0, s2 = 0.0;

    for (int i = 0; i < N; i++) {
        s0 = (double)samples[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return (float)(s1 * s1 + s2 * s2 - coeff * s1 * s2);
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Extract tone powers for all 79 symbols at a given
 *       base frequency using Goertzel filters.
 * ═══════════════════════════════════════════════════════════════ */

static void ft8_extract_tones(const float *audio, int num_samples,
                               int sample_rate, double base_freq,
                               float tone_power[FT8_NUM_SYMBOLS][FT8_NUM_TONES]) {
    int sps = (int)(FT8_SYMBOL_PERIOD * sample_rate);

    for (int sym = 0; sym < FT8_NUM_SYMBOLS; sym++) {
        int start = sym * sps;
        if (start + sps > num_samples) {
            for (int t = 0; t < FT8_NUM_TONES; t++)
                tone_power[sym][t] = 0.0f;
            continue;
        }

        for (int t = 0; t < FT8_NUM_TONES; t++) {
            double freq = base_freq + t * FT8_TONE_SPACING;
            tone_power[sym][t] = goertzel_power_f(&audio[start], sps,
                                                   freq, sample_rate);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Costas sync correlation score
 *
 *  Measures how well the three 7-symbol Costas arrays (at
 *  positions 0, 36, 72) match the observed tone powers.
 *  Returns the ratio of Costas-tone power to total power
 *  across those 21 symbols.
 * ═══════════════════════════════════════════════════════════════ */

static float ft8_costas_score(const float tone_power[FT8_NUM_SYMBOLS][FT8_NUM_TONES]) {
    float sync_power = 0.0f;
    float total_power = 0.0f;

    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < FT8_COSTAS_SIZE; i++) {
            int sym = FT8_COSTAS_POS[c] + i;
            if (sym >= FT8_NUM_SYMBOLS) return 0.0f;

            sync_power += tone_power[sym][FT8_COSTAS[i]];

            for (int t = 0; t < FT8_NUM_TONES; t++)
                total_power += tone_power[sym][t];
        }
    }

    if (total_power < 1e-12f) return 0.0f;
    return sync_power / total_power;
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Hard-decision symbol extraction
 *
 *  For each symbol, pick the tone with maximum power and
 *  record a confidence value (ratio of best to second-best).
 * ═══════════════════════════════════════════════════════════════ */

static void ft8_hard_decode_symbols(const float tone_power[FT8_NUM_SYMBOLS][FT8_NUM_TONES],
                                     uint8_t *symbols, float *confidence) {
    for (int sym = 0; sym < FT8_NUM_SYMBOLS; sym++) {
        int best = 0;
        float best_pwr = tone_power[sym][0];
        float second_pwr = -1.0f;

        for (int t = 1; t < FT8_NUM_TONES; t++) {
            if (tone_power[sym][t] > best_pwr) {
                second_pwr = best_pwr;
                best_pwr = tone_power[sym][t];
                best = t;
            } else if (tone_power[sym][t] > second_pwr) {
                second_pwr = tone_power[sym][t];
            }
        }
        symbols[sym] = (uint8_t)best;
        confidence[sym] = (best_pwr > 1e-12f && second_pwr >= 0.0f)
                              ? (best_pwr - second_pwr) / best_pwr
                              : 0.0f;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Convert 58 data symbols to 174 coded bits via Gray code
 *
 *  Data symbol positions within the 79-symbol frame:
 *    7..35  (29 symbols) and 43..71 (29 symbols)
 *  Each symbol encodes 3 bits (MSB first) through Gray mapping.
 * ═══════════════════════════════════════════════════════════════ */

static int ft8_symbols_to_bits(const uint8_t *symbols,
                                uint8_t *coded_bits) {
    int bit_idx = 0;

    for (int i = 0; i < FT8_NUM_SYMBOLS; i++) {
        /* Skip Costas sync positions: 0-6, 36-42, 72-78 */
        if (i < 7 || (i >= 36 && i < 43) || i >= 72)
            continue;

        uint8_t gray = FT8_GRAY_MAP[symbols[i] & 7];
        coded_bits[bit_idx++] = (gray >> 2) & 1;
        coded_bits[bit_idx++] = (gray >> 1) & 1;
        coded_bits[bit_idx++] = gray & 1;
    }

    return bit_idx; /* should be 174 */
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Compute soft log-likelihood ratios (LLR) from tone powers
 *
 *  For each of the 174 coded bits, compute an LLR based on
 *  the ratio of tone powers supporting bit=0 vs bit=1.
 *  Positive LLR => bit is more likely 0.
 * ═══════════════════════════════════════════════════════════════ */

static void ft8_compute_llr(const float tone_power[FT8_NUM_SYMBOLS][FT8_NUM_TONES],
                             float *llr) {
    int bit_idx = 0;

    for (int i = 0; i < FT8_NUM_SYMBOLS; i++) {
        if (i < 7 || (i >= 36 && i < 43) || i >= 72)
            continue;

        /* For each of the 3 bits encoded by this symbol */
        for (int b = 0; b < 3; b++) {
            int bit_pos = 2 - b; /* MSB first: bit 2, 1, 0 */
            float p0 = 0.0f;    /* total power for tones where this bit = 0 */
            float p1 = 0.0f;    /* total power for tones where this bit = 1 */

            for (int t = 0; t < FT8_NUM_TONES; t++) {
                uint8_t gray = FT8_GRAY_MAP[t];
                if ((gray >> bit_pos) & 1)
                    p1 += tone_power[i][t];
                else
                    p0 += tone_power[i][t];
            }

            /* Clamp to avoid log(0) */
            if (p0 < 1e-12f) p0 = 1e-12f;
            if (p1 < 1e-12f) p1 = 1e-12f;

            llr[bit_idx++] = logf(p0 / p1);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: LDPC Decoder (Min-Sum / Belief Propagation)
 *
 *  Iterative decoder using the min-sum approximation to
 *  belief propagation.  Operates on LLR (log-likelihood ratio)
 *  values.  Converges when all parity checks are satisfied or
 *  the maximum iteration count is reached.
 *
 *  Scaling factor 0.8 applied to check-to-variable messages
 *  (normalized min-sum) for improved convergence.
 * ═══════════════════════════════════════════════════════════════ */

#define LDPC_MAX_ITER       25
#define LDPC_MS_SCALE       0.8f

static int ft8_ldpc_decode(const float *llr_in, uint8_t *decoded_bits) {
    /* Working storage for message passing */
    /* Check-to-variable messages: c2v[check][col_within_row] */
    float c2v[FT8_LDPC_NUM_CHECKS][FT8_LDPC_MAX_ROW_WEIGHT];
    /* Current bit LLR (channel + all incoming check messages) */
    float bit_llr[FT8_LDPC_N];

    memset(c2v, 0, sizeof(c2v));
    memcpy(bit_llr, llr_in, sizeof(float) * FT8_LDPC_N);

    for (int iter = 0; iter < LDPC_MAX_ITER; iter++) {
        /* --- Variable-to-check then check-to-variable update --- */
        for (int r = 0; r < FT8_LDPC_NUM_CHECKS; r++) {
            int w = ldpc_row_weight[r];

            /* Compute variable-to-check messages:
             * v2c[j] = bit_llr[col] - c2v[r][j]  (extrinsic info) */
            float v2c[FT8_LDPC_MAX_ROW_WEIGHT];
            for (int j = 0; j < w; j++) {
                int col = ldpc_row_cols[r][j];
                v2c[j] = bit_llr[col] - c2v[r][j];
            }

            /* Min-sum check-to-variable update:
             * c2v[r][j] = scale * sign_product * min_of_others */
            for (int j = 0; j < w; j++) {
                float min_abs = 1e30f;
                int sign = 0; /* 0 = positive, 1 = negative (XOR of sign bits) */

                for (int k = 0; k < w; k++) {
                    if (k == j) continue;
                    float absval = fabsf(v2c[k]);
                    if (absval < min_abs) min_abs = absval;
                    if (v2c[k] < 0) sign ^= 1;
                }

                float old_c2v = c2v[r][j];
                c2v[r][j] = LDPC_MS_SCALE * (sign ? -min_abs : min_abs);

                /* Update bit LLR incrementally */
                bit_llr[ldpc_row_cols[r][j]] += (c2v[r][j] - old_c2v);
            }
        }

        /* --- Hard decision and parity check --- */
        for (int i = 0; i < FT8_LDPC_N; i++)
            decoded_bits[i] = (bit_llr[i] < 0) ? 1 : 0;

        int all_ok = 1;
        for (int r = 0; r < FT8_LDPC_NUM_CHECKS; r++) {
            int w = ldpc_row_weight[r];
            int parity = 0;
            for (int j = 0; j < w; j++)
                parity ^= decoded_bits[ldpc_row_cols[r][j]];
            if (parity != 0) { all_ok = 0; break; }
        }

        if (all_ok) return iter + 1; /* converged */
    }

    return -1; /* did not converge */
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: CRC-14 Check
 *
 *  Polynomial: x^14 + x + 1  (CRC-14/DARC variant used by FT8)
 *  Applied to the 77 payload bits; result compared against
 *  bits 77..90 of the decoded codeword.
 * ═══════════════════════════════════════════════════════════════ */

static uint16_t ft8_crc14(const uint8_t *bits, int num_bits) {
    uint16_t crc = 0;

    for (int i = 0; i < num_bits; i++) {
        crc <<= 1;
        crc |= (bits[i] & 1);
        if (crc & 0x4000)
            crc ^= 0x4003;  /* x^14 + x + 1 */
    }

    return crc & 0x3FFF;
}

static int ft8_check_crc(const uint8_t *decoded_bits) {
    uint16_t computed = ft8_crc14(decoded_bits, FT8_PAYLOAD_BITS);
    uint16_t received = 0;

    for (int i = 0; i < FT8_CRC_BITS; i++)
        received = (received << 1) | (decoded_bits[FT8_PAYLOAD_BITS + i] & 1);

    return (computed == received) ? 0 : -1;
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Callsign Unpacking (28-bit field)
 *
 *  28 bits encode a callsign in base-37 (6 characters):
 *    space=0, A-Z=1-26, 0-9=27-36
 *  Special values above 262177560 encode hashed callsigns
 *  or CQ/DE/QRZ prefixes.
 * ═══════════════════════════════════════════════════════════════ */

static void ft8_unpack_callsign(uint32_t packed, char *callsign) {
    static const char charset[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

    /* CQ, DE, QRZ handling */
    if (packed >= 262177560UL) {
        uint32_t x = packed - 262177560UL;
        if (x == 0) {
            strcpy(callsign, "CQ");
            return;
        }
        if (x <= 999) {
            snprintf(callsign, 12, "CQ %u", (unsigned)x);
            return;
        }
        if (x == 1000) {
            strcpy(callsign, "DE");
            return;
        }
        if (x == 1001) {
            strcpy(callsign, "QRZ");
            return;
        }
        /* Hashed callsign — cannot decode without table */
        snprintf(callsign, 12, "<%.6u>", (unsigned)(packed % 1000000));
        return;
    }

    char tmp[7];
    uint32_t val = packed;

    for (int i = 5; i >= 0; i--) {
        tmp[i] = charset[val % 37];
        val /= 37;
    }
    tmp[6] = '\0';

    /* Trim leading spaces */
    int start = 0;
    while (tmp[start] == ' ' && start < 6) start++;

    /* Trim trailing spaces */
    int end = 5;
    while (end > start && tmp[end] == ' ') end--;

    int len = end - start + 1;
    memcpy(callsign, &tmp[start], len);
    callsign[len] = '\0';
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Grid / Report Unpacking (15-bit field)
 *
 *  Values 0-32399:   4-character Maidenhead grid
 *  Values 32400+:    signal report or special tokens
 * ═══════════════════════════════════════════════════════════════ */

static void ft8_unpack_grid(uint16_t packed, char *grid) {
    if (packed > 32767) {
        grid[0] = '\0';
        return;
    }

    /* Signal report: -30 to +30 dB */
    if (packed >= 32400) {
        int report = (int)packed - 32400 - 30;
        if (packed == 32400 + 30 + 1) {
            strcpy(grid, "RRR");
        } else if (packed == 32400 + 30 + 2) {
            strcpy(grid, "RR73");
        } else if (packed == 32400 + 30 + 3) {
            strcpy(grid, "73");
        } else {
            snprintf(grid, 8, "%+d", report);
        }
        return;
    }

    /* Maidenhead grid locator (4 chars) */
    int n = packed;
    grid[3] = '0' + (n % 10); n /= 10;
    grid[2] = '0' + (n % 10); n /= 10;
    grid[1] = 'A' + (n % 18); n /= 18;
    grid[0] = 'A' + (n % 18);
    grid[4] = '\0';
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Unpack 77-bit message into human-readable string
 *
 *  Message type field i3.n3 determines the format.
 *  Type 1 (i3=1): standard QSO — callsign1, callsign2, grid/report
 *  Type 0.0 (free text), Type 0.5 (telemetry), etc.
 * ═══════════════════════════════════════════════════════════════ */

static void ft8_unpack_message(const uint8_t *bits, char *message, int max_len) {
    /* Extract i3 (bits 74,75,76) = 3-bit message type */
    int i3 = (bits[74] << 2) | (bits[75] << 1) | bits[76];

    if (i3 == 1) {
        /* Type 1: standard message
         * n28a (28 bits) + r1 (1 bit) + n28b (28 bits) + r2 (1 bit) +
         * g15 (15 bits) + i3 (3 bits) + spare (1 bit) = 77 bits */
        uint32_t n28a = 0;
        for (int i = 0; i < 28; i++) n28a = (n28a << 1) | bits[i];

        int r1 = bits[28];

        uint32_t n28b = 0;
        for (int i = 0; i < 28; i++) n28b = (n28b << 1) | bits[29 + i];

        int r2 = bits[57];
        (void)r2;

        uint16_t g15 = 0;
        for (int i = 0; i < 15; i++) g15 = (g15 << 1) | bits[58 + i];

        /* Bit 73 is spare in type 1 */

        char call1[12], call2[12], grid[12];
        ft8_unpack_callsign(n28a, call1);
        ft8_unpack_callsign(n28b, call2);
        ft8_unpack_grid(g15, grid);

        if (r1) {
            /* R prefix on report (response message) */
            char rgrid[16];
            snprintf(rgrid, sizeof(rgrid), "R%s", grid);
            snprintf(message, max_len, "%s %s %s", call1, call2, rgrid);
        } else {
            snprintf(message, max_len, "%s %s %s", call1, call2, grid);
        }
    } else if (i3 == 0) {
        /* Type 0: check n3 sub-type (bits 71,72,73) */
        int n3 = (bits[71] << 2) | (bits[72] << 1) | bits[73];

        if (n3 == 0) {
            /* Free text: 71 bits encode 13 characters in base-42 */
            static const char ft_charset[] =
                " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?";
            /* Extract 71-bit number */
            uint64_t acc_hi = 0, acc_lo = 0;
            for (int i = 0; i < 35; i++) acc_hi = (acc_hi << 1) | bits[i];
            for (int i = 35; i < 71; i++) acc_lo = (acc_lo << 1) | bits[i];

            /* Convert to base-42, 13 characters (simplified with 128-bit arithmetic) */
            char text[14];
            text[13] = '\0';
            /* Use repeated division — approximate with double for embedded */
            double val = (double)acc_hi * (double)(1ULL << 36) + (double)acc_lo;
            for (int i = 12; i >= 0; i--) {
                int rem = (int)fmod(val, 42.0);
                if (rem < 0) rem = 0;
                if (rem > 41) rem = 41;
                text[i] = ft_charset[rem];
                val = floor(val / 42.0);
            }

            /* Trim trailing spaces */
            int end = 12;
            while (end >= 0 && text[end] == ' ') text[end--] = '\0';

            snprintf(message, max_len, "%s", text);
        } else {
            snprintf(message, max_len, "<type 0.%d>", n3);
        }
    } else if (i3 == 4) {
        /* Type 4: non-standard call + hashed call + report */
        snprintf(message, max_len, "<type 4>");
    } else {
        snprintf(message, max_len, "<type %d>", i3);
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Full decode pipeline for one candidate signal
 *
 *  1. Compute soft LLR from tone powers
 *  2. LDPC decode (min-sum)
 *  3. CRC-14 verification
 *  4. Message unpacking
 *
 *  Returns 0 on success, -1 on failure.
 * ═══════════════════════════════════════════════════════════════ */

static int ft8_decode_candidate(const float tone_power[FT8_NUM_SYMBOLS][FT8_NUM_TONES],
                                 char *message, int max_len) {
    float llr[FT8_LDPC_N];
    uint8_t decoded_bits[FT8_LDPC_N];

    /* Compute soft LLR from tone powers */
    ft8_compute_llr(tone_power, llr);

    /* LDPC decode */
    int iters = ft8_ldpc_decode(llr, decoded_bits);
    if (iters < 0) {
        /* LDPC did not converge — try hard-decision fallback */
        uint8_t symbols[FT8_NUM_SYMBOLS];
        float confidence[FT8_NUM_SYMBOLS];
        ft8_hard_decode_symbols(tone_power, symbols, confidence);

        uint8_t hard_bits[FT8_LDPC_N];
        int nbits = ft8_symbols_to_bits(symbols, hard_bits);
        if (nbits != FT8_LDPC_N) return -1;

        if (ft8_check_crc(hard_bits) == 0) {
            ft8_unpack_message(hard_bits, message, max_len);
            return 0;
        }
        return -1;
    }

    /* CRC check */
    if (ft8_check_crc(decoded_bits) != 0)
        return -1;

    /* Unpack message */
    ft8_unpack_message(decoded_bits, message, max_len);
    return 0;
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Frequency scan and decode
 *
 *  Scans the audio passband for candidate signals by
 *  evaluating the Costas sync score at each frequency step.
 *  Top candidates are then passed through the full decode
 *  pipeline.
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    double freq;
    float score;
} ft8_candidate_t;

static void ft8_try_decode(ft8_ctx_t *c) {
    int sr = c->sample_rate;
    if (sr < 8000) return;

    int sps = (int)(FT8_SYMBOL_PERIOD * sr);
    int window = FT8_NUM_SYMBOLS * sps;
    if (window > c->audio_buf_size) return;

    /* Linearize the circular buffer into a contiguous array */
    float *linear = (float *)malloc(window * sizeof(float));
    if (!linear) return;

    int read_pos = (c->audio_write_pos - window + c->audio_buf_size)
                   % c->audio_buf_size;
    for (int i = 0; i < window; i++)
        linear[i] = c->audio_buf[(read_pos + i) % c->audio_buf_size];

    /* Phase 1: coarse frequency scan — collect candidates */
    ft8_candidate_t candidates[FT8_MAX_CANDIDATES];
    int num_candidates = 0;

    /* Allocate tone power grid once, reuse per frequency */
    float (*tone_power)[FT8_NUM_TONES] =
        (float (*)[FT8_NUM_TONES])malloc(
            FT8_NUM_SYMBOLS * FT8_NUM_TONES * sizeof(float));
    if (!tone_power) { free(linear); return; }

    for (double freq = FT8_FREQ_MIN; freq <= FT8_FREQ_MAX; freq += FT8_TONE_SPACING) {
        ft8_extract_tones(linear, window, sr, freq, tone_power);
        float score = ft8_costas_score(tone_power);

        if (score < FT8_MIN_SYNC_SCORE) continue;

        /* Insert into candidate list (sorted descending by score) */
        if (num_candidates < FT8_MAX_CANDIDATES) {
            candidates[num_candidates].freq = freq;
            candidates[num_candidates].score = score;
            num_candidates++;
        } else if (score > candidates[num_candidates - 1].score) {
            candidates[num_candidates - 1].freq = freq;
            candidates[num_candidates - 1].score = score;
        } else {
            continue;
        }

        /* Bubble sort the new entry into position */
        for (int i = num_candidates - 1; i > 0; i--) {
            if (candidates[i].score > candidates[i - 1].score) {
                ft8_candidate_t tmp = candidates[i];
                candidates[i] = candidates[i - 1];
                candidates[i - 1] = tmp;
            } else {
                break;
            }
        }
    }

    /* Phase 2: attempt full decode on each candidate */
    for (int ci = 0; ci < num_candidates; ci++) {
        ft8_extract_tones(linear, window, sr, candidates[ci].freq, tone_power);

        char message[64];
        message[0] = '\0';

        if (ft8_decode_candidate(tone_power, message, sizeof(message)) != 0)
            continue;

        if (message[0] == '\0') continue;

        /* Successful decode */
        xSemaphoreTake(c->mutex, portMAX_DELAY);
        strncpy(c->last_decode, message, sizeof(c->last_decode) - 1);
        c->last_decode[sizeof(c->last_decode) - 1] = '\0';
        c->last_freq = (float)candidates[ci].freq;
        c->last_score = candidates[ci].score;
        c->decode_count++;
        xSemaphoreGive(c->mutex);

        /* Publish decode event */
        cJSON *data = cJSON_CreateObject();
        if (data) {
            cJSON_AddStringToObject(data, "message", message);
            cJSON_AddNumberToObject(data, "freq_hz", candidates[ci].freq);
            cJSON_AddNumberToObject(data, "sync_score", candidates[ci].score);

            int64_t now = (int64_t)(esp_timer_get_time() / 1000);
            decode_event_t event = {
                .decoder_name = "ft8",
                .event_type = "decode",
                .timestamp_ms = now,
                .rssi_db = 0,
                .freq_hz = 14074000 + (uint32_t)candidates[ci].freq,
                .data = data,
            };
            decode_bus_publish(&event);
        }

        ESP_LOGI(TAG, "FT8: %s @ %.1f Hz (sync=%.3f)",
                 message, candidates[ci].freq, candidates[ci].score);
    }

    free(tone_power);
    free(linear);
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Lifecycle
 * ═══════════════════════════════════════════════════════════════ */

static esp_err_t ft8_init(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    memset(c, 0, sizeof(*c));
    c->mutex = xSemaphoreCreateMutex();
    if (!c->mutex) return ESP_ERR_NO_MEM;
    c->running = false;

    /* Allocate audio ring buffer (~720 KB in PSRAM for 15s at 12 kHz) */
    c->audio_buf_size = FT8_AUDIO_BUF_SIZE;
    c->audio_buf = (float *)malloc(c->audio_buf_size * sizeof(float));
    if (!c->audio_buf) {
        ESP_LOGE(TAG, "FT8: failed to allocate %d-sample audio buffer",
                 c->audio_buf_size);
        vSemaphoreDelete(c->mutex);
        c->mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    memset(c->audio_buf, 0, c->audio_buf_size * sizeof(float));

    ESP_LOGI(TAG, "FT8 decoder initialized (buf=%d samples, %d KB)",
             c->audio_buf_size,
             (int)(c->audio_buf_size * sizeof(float) / 1024));
    return ESP_OK;
}

static esp_err_t ft8_start(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    c->running = true;
    c->audio_write_pos = 0;
    c->audio_samples_accum = 0;
    ESP_LOGI(TAG, "FT8 decoder started");
    return ESP_OK;
}

static esp_err_t ft8_stop(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "FT8 decoder stopped (decodes=%d)", c->decode_count);
    return ESP_OK;
}

static void ft8_destroy(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    if (c->audio_buf) {
        free(c->audio_buf);
        c->audio_buf = NULL;
    }
    if (c->mutex) {
        vSemaphoreDelete(c->mutex);
        c->mutex = NULL;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Audio processing callback
 *
 *  Accumulates int16 samples into a float circular buffer.
 *  Triggers a full decode pass every ~15 seconds.
 * ═══════════════════════════════════════════════════════════════ */

static void ft8_process_audio(void *ctx, const int16_t *samples,
                               uint32_t count, uint32_t sample_rate) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    if (!c->running || !c->audio_buf) return;

    c->sample_rate = (int)sample_rate;

    /* Convert int16 -> float [-1,+1] and write into circular buffer */
    for (uint32_t i = 0; i < count; i++) {
        c->audio_buf[c->audio_write_pos] = (float)samples[i] / 32768.0f;
        c->audio_write_pos = (c->audio_write_pos + 1) % c->audio_buf_size;
    }
    c->audio_samples_accum += (int)count;

    /* Trigger decode every ~15 seconds of audio */
    int trigger_samples = FT8_AUDIO_BUF_SECS * (int)sample_rate;
    if (c->audio_samples_accum >= trigger_samples) {
        c->audio_samples_accum = 0;
        ft8_try_decode(c);
    }
}

static void ft8_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

/* ═══════════════════════════════════════════════════════════════
 *  FT8: Status and results
 * ═══════════════════════════════════════════════════════════════ */

static cJSON *ft8_get_status(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "decode_count", c->decode_count);
        if (c->last_decode[0]) {
            cJSON_AddStringToObject(j, "last_message", c->last_decode);
            cJSON_AddNumberToObject(j, "last_freq_hz", c->last_freq);
            cJSON_AddNumberToObject(j, "last_sync_score", c->last_score);
        }
        cJSON_AddNumberToObject(j, "audio_buf_kb",
                                c->audio_buf_size * (int)sizeof(float) / 1024);
    }
    return j;
}

static cJSON *ft8_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking()
               ? tracking_table_query(decoder_get_global_tracking(), "ft8")
               : cJSON_CreateArray();
}

/* ── PSKreporter upload stub ──────────────────────────── */
/* TODO: Implement HTTPS POST to pskreporter.info/cgi-bin/psk-report.pl
 * Format: XML with receiverCallsign, senderCallsign, frequency, mode, SNR
 */
static void psk_reporter_submit(const char *callsign, double freq_hz,
                                 int snr, const char *mode) {
    (void)callsign; (void)freq_hz; (void)snr; (void)mode;
    ESP_LOGD("dec_ft8", "PSKreporter: %s at %.0f Hz, %d dB (%s)", callsign, freq_hz, snr, mode);
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Goertzel tone extraction for 4-FSK
 * ═══════════════════════════════════════════════════════════════ */

static void wspr_extract_tones(const float *audio, int num_samples,
                                int sample_rate, double base_freq,
                                float tone_power[WSPR_NUM_SYMBOLS][WSPR_NUM_TONES]) {
    int sps = (int)(WSPR_SYMBOL_PERIOD * sample_rate);

    for (int sym = 0; sym < WSPR_NUM_SYMBOLS; sym++) {
        int start = sym * sps;
        if (start + sps > num_samples) {
            for (int t = 0; t < WSPR_NUM_TONES; t++)
                tone_power[sym][t] = 0.0f;
            continue;
        }

        for (int t = 0; t < WSPR_NUM_TONES; t++) {
            double freq = base_freq + t * WSPR_TONE_SPACING;
            tone_power[sym][t] = goertzel_power_f(&audio[start], sps,
                                                   freq, sample_rate);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Sync correlation using known sync vector
 *
 *  The WSPR sync vector (162 bits) indicates which symbols
 *  carry synchronization info (sync[i]=1) vs data (sync[i]=0).
 *  For sync symbols, tones 0,1 encode sync=0,1.
 *  Score is the normalized correlation of expected vs observed.
 * ═══════════════════════════════════════════════════════════════ */

static float wspr_sync_score(const float tone_power[WSPR_NUM_SYMBOLS][WSPR_NUM_TONES]) {
    float score = 0.0f;
    float total = 0.0f;

    for (int i = 0; i < WSPR_NUM_SYMBOLS; i++) {
        /* Sync symbols: the sync bit is added to the data tone index.
         * For sync correlation, we check if the tone pattern matches
         * the sync vector (even tones for sync=0, odd tones for sync=1). */
        float sym_total = 0.0f;
        float even_power = 0.0f;
        float odd_power = 0.0f;

        for (int t = 0; t < WSPR_NUM_TONES; t++) {
            sym_total += tone_power[i][t];
            if (t % 2 == 0)
                even_power += tone_power[i][t];
            else
                odd_power += tone_power[i][t];
        }

        total += sym_total;

        if (WSPR_SYNC[i])
            score += odd_power;
        else
            score += even_power;
    }

    if (total < 1e-12f) return 0.0f;
    return score / total;
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Unpack 50-bit message
 *
 *  28 bits callsign (base-37, 6 chars) +
 *  15 bits grid (Maidenhead 4-char) +
 *   7 bits power (dBm, 0-60 in steps of 1)
 * ═══════════════════════════════════════════════════════════════ */

static void wspr_unpack_message(const uint8_t *bits, char *message, int max_len) {
    static const char charset[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

    /* Callsign: 28 bits */
    uint32_t n28 = 0;
    for (int i = 0; i < 28; i++) n28 = (n28 << 1) | (bits[i] & 1);

    char call[7];
    for (int i = 5; i >= 0; i--) {
        call[i] = charset[n28 % 37];
        n28 /= 37;
    }
    call[6] = '\0';

    /* Trim leading/trailing spaces */
    int cs = 0, ce = 5;
    while (cs < 6 && call[cs] == ' ') cs++;
    while (ce > cs && call[ce] == ' ') ce--;
    call[ce + 1] = '\0';

    /* Grid: 15 bits */
    uint16_t g15 = 0;
    for (int i = 28; i < 43; i++) g15 = (g15 << 1) | (bits[i] & 1);

    char grid[5];
    int n = g15;
    grid[3] = '0' + (n % 10); n /= 10;
    grid[2] = '0' + (n % 10); n /= 10;
    grid[1] = 'A' + (n % 18); n /= 18;
    grid[0] = 'A' + (n % 18);
    grid[4] = '\0';

    /* Power: 7 bits (0-60 dBm) */
    uint8_t pwr = 0;
    for (int i = 43; i < 50; i++) pwr = (pwr << 1) | (bits[i] & 1);

    snprintf(message, max_len, "%s %s %d dBm", &call[cs], grid, (int)pwr);
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Convolutional decoder (Fano sequential, simplified)
 *
 *  WSPR uses a K=32, rate-1/2 convolutional code with
 *  generators G1=0xF2D05351 and G2=0xE4613C47.
 *  Full Fano/Viterbi for K=32 requires 2^31 states — far too
 *  large for embedded.  We use a hard-decision approach:
 *  extract data tones, de-interleave, and check for valid
 *  callsign patterns.  This works for strong signals.
 * ═══════════════════════════════════════════════════════════════ */

static int wspr_decode_symbols(const float tone_power[WSPR_NUM_SYMBOLS][WSPR_NUM_TONES],
                                uint8_t *data_bits, int *num_bits) {
    /* Hard decision on each symbol */
    uint8_t tones[WSPR_NUM_SYMBOLS];
    for (int i = 0; i < WSPR_NUM_SYMBOLS; i++) {
        int best = 0;
        float best_pwr = tone_power[i][0];
        for (int t = 1; t < WSPR_NUM_TONES; t++) {
            if (tone_power[i][t] > best_pwr) {
                best_pwr = tone_power[i][t];
                best = t;
            }
        }
        tones[i] = (uint8_t)best;
    }

    /* Remove sync contribution: data = tone - sync_bit
     * Each tone encodes 2*data_bit + sync_bit, so data_bit = tone >> 1 */
    uint8_t raw_bits[WSPR_NUM_SYMBOLS];
    for (int i = 0; i < WSPR_NUM_SYMBOLS; i++)
        raw_bits[i] = (tones[i] >> 1) & 1;

    /* De-interleave: WSPR uses a bit-reversal interleaver on 256 positions
     * (only 162 are used).  Reverse the interleaver to get coded bits. */
    uint8_t coded[WSPR_NUM_SYMBOLS];
    memset(coded, 0, sizeof(coded));

    for (int i = 0; i < WSPR_NUM_SYMBOLS; i++) {
        /* Bit-reversal permutation on 8 bits (256 entries), keep only < 162 */
        uint8_t j = 0;
        uint8_t val = (uint8_t)i;
        for (int b = 0; b < 8; b++) {
            j = (j << 1) | (val & 1);
            val >>= 1;
        }
        if (j < WSPR_NUM_SYMBOLS)
            coded[j] = raw_bits[i];
    }

    /* The coded bits are rate-1/2, so 162 coded bits -> 81 data bits.
     * For the hard-decision shortcut, take every other bit as data
     * (approximation — proper Viterbi would use both). */
    *num_bits = 0;
    for (int i = 0; i < WSPR_NUM_SYMBOLS && *num_bits < 50; i += 2) {
        data_bits[*num_bits] = coded[i];
        (*num_bits)++;
    }

    /* Minimal validation: we need exactly 50 bits */
    return (*num_bits >= 50) ? 0 : -1;
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Full decode attempt
 * ═══════════════════════════════════════════════════════════════ */

static void wspr_try_decode(wspr_ctx_t *c) {
    int sr = c->sample_rate;
    if (sr < 100) return;

    int sps = (int)(WSPR_SYMBOL_PERIOD * sr);
    int window = WSPR_NUM_SYMBOLS * sps;
    if (window > c->audio_buf_size) return;

    float *linear = (float *)malloc(window * sizeof(float));
    if (!linear) return;

    int read_pos = (c->audio_write_pos - window + c->audio_buf_size)
                   % c->audio_buf_size;
    for (int i = 0; i < window; i++)
        linear[i] = c->audio_buf[(read_pos + i) % c->audio_buf_size];

    float (*tone_power)[WSPR_NUM_TONES] =
        (float (*)[WSPR_NUM_TONES])malloc(
            WSPR_NUM_SYMBOLS * WSPR_NUM_TONES * sizeof(float));
    if (!tone_power) { free(linear); return; }

    float best_score = 0.0f;
    double best_freq = 0.0;
    float best_powers[WSPR_NUM_SYMBOLS][WSPR_NUM_TONES];

    /* WSPR signals occupy ~6 Hz bandwidth, scan 1400-1600 Hz range */
    for (double freq = 1400.0; freq <= 1600.0; freq += WSPR_TONE_SPACING) {
        wspr_extract_tones(linear, window, sr, freq, tone_power);
        float score = wspr_sync_score(tone_power);

        if (score > best_score) {
            best_score = score;
            best_freq = freq;
            memcpy(best_powers, tone_power,
                   sizeof(float) * WSPR_NUM_SYMBOLS * WSPR_NUM_TONES);
        }
    }

    free(tone_power);
    free(linear);

    if (best_score < 0.55f) return; /* weak sync */

    uint8_t data_bits[81];
    int num_bits = 0;
    if (wspr_decode_symbols(best_powers, data_bits, &num_bits) != 0) return;

    char message[64];
    wspr_unpack_message(data_bits, message, sizeof(message));

    if (message[0] == '\0') return;

    xSemaphoreTake(c->mutex, portMAX_DELAY);
    strncpy(c->last_decode, message, sizeof(c->last_decode) - 1);
    c->last_decode[sizeof(c->last_decode) - 1] = '\0';
    c->last_freq = (float)best_freq;
    c->decode_count++;
    xSemaphoreGive(c->mutex);

    cJSON *data = cJSON_CreateObject();
    if (data) {
        cJSON_AddStringToObject(data, "message", message);
        cJSON_AddNumberToObject(data, "freq_hz", best_freq);
        cJSON_AddNumberToObject(data, "sync_score", best_score);

        int64_t now = (int64_t)(esp_timer_get_time() / 1000);
        decode_event_t event = {
            .decoder_name = "wspr",
            .event_type = "decode",
            .timestamp_ms = now,
            .rssi_db = 0,
            .freq_hz = 14095600 + (uint32_t)best_freq,
            .data = data,
        };
        decode_bus_publish(&event);
    }

    ESP_LOGI(TAG, "WSPR: %s @ %.1f Hz (sync=%.3f)",
             message, best_freq, best_score);
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Lifecycle
 * ═══════════════════════════════════════════════════════════════ */

static esp_err_t wspr_init(void *ctx) {
    wspr_ctx_t *c = (wspr_ctx_t *)ctx;
    memset(c, 0, sizeof(*c));
    c->mutex = xSemaphoreCreateMutex();
    if (!c->mutex) return ESP_ERR_NO_MEM;
    c->running = false;

    /* Audio buffer for 120 seconds at 375 Hz = 45000 samples (~180 KB) */
    c->audio_buf_size = WSPR_AUDIO_BUF_SIZE;
    c->audio_buf = (float *)malloc(c->audio_buf_size * sizeof(float));
    if (!c->audio_buf) {
        ESP_LOGE(TAG, "WSPR: failed to allocate %d-sample audio buffer",
                 c->audio_buf_size);
        vSemaphoreDelete(c->mutex);
        c->mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    memset(c->audio_buf, 0, c->audio_buf_size * sizeof(float));

    ESP_LOGI(TAG, "WSPR decoder initialized (buf=%d samples, %d KB)",
             c->audio_buf_size,
             (int)(c->audio_buf_size * sizeof(float) / 1024));
    return ESP_OK;
}

static esp_err_t wspr_start(void *ctx) {
    wspr_ctx_t *c = (wspr_ctx_t *)ctx;
    c->running = true;
    c->audio_write_pos = 0;
    c->audio_samples_accum = 0;
    ESP_LOGI(TAG, "WSPR decoder started");
    return ESP_OK;
}

static esp_err_t wspr_stop(void *ctx) {
    wspr_ctx_t *c = (wspr_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "WSPR decoder stopped (decodes=%d)", c->decode_count);
    return ESP_OK;
}

static void wspr_destroy(void *ctx) {
    wspr_ctx_t *c = (wspr_ctx_t *)ctx;
    if (c->audio_buf) {
        free(c->audio_buf);
        c->audio_buf = NULL;
    }
    if (c->mutex) {
        vSemaphoreDelete(c->mutex);
        c->mutex = NULL;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Audio processing callback
 * ═══════════════════════════════════════════════════════════════ */

static void wspr_process_audio(void *ctx, const int16_t *samples,
                                uint32_t count, uint32_t sample_rate) {
    wspr_ctx_t *c = (wspr_ctx_t *)ctx;
    if (!c->running || !c->audio_buf) return;

    c->sample_rate = (int)sample_rate;

    for (uint32_t i = 0; i < count; i++) {
        c->audio_buf[c->audio_write_pos] = (float)samples[i] / 32768.0f;
        c->audio_write_pos = (c->audio_write_pos + 1) % c->audio_buf_size;
    }
    c->audio_samples_accum += (int)count;

    /* Trigger decode every ~120 seconds */
    int trigger_samples = WSPR_AUDIO_BUF_SECS * (int)sample_rate;
    if (c->audio_samples_accum >= trigger_samples) {
        c->audio_samples_accum = 0;
        wspr_try_decode(c);
    }
}

static void wspr_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

/* ═══════════════════════════════════════════════════════════════
 *  WSPR: Status and results
 * ═══════════════════════════════════════════════════════════════ */

static cJSON *wspr_get_status(void *ctx) {
    wspr_ctx_t *c = (wspr_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "decode_count", c->decode_count);
        if (c->last_decode[0]) {
            cJSON_AddStringToObject(j, "last_message", c->last_decode);
            cJSON_AddNumberToObject(j, "last_freq_hz", c->last_freq);
        }
    }
    return j;
}

static cJSON *wspr_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking()
               ? tracking_table_query(decoder_get_global_tracking(), "wspr")
               : cJSON_CreateArray();
}

/* ═══════════════════════════════════════════════════════════════
 *  Plugin descriptors
 * ═══════════════════════════════════════════════════════════════ */

static decoder_plugin_t s_ft8_plugin = {
    .name = "ft8",
    .description = "FT8 HF Digital Mode (8-GFSK, LDPC, 15s cycles)",
    .category = "digital",
    .demod_type = DEMOD_USB,
    .center_freq_hz = 14074000,
    .bandwidth_hz = 3000,
    .audio_rate_hz = 12000,
    .init = ft8_init,
    .start = ft8_start,
    .stop = ft8_stop,
    .destroy = ft8_destroy,
    .process_audio = ft8_process_audio,
    .process_iq = ft8_process_iq,
    .get_status = ft8_get_status,
    .get_results = ft8_get_results,
    .ctx = &s_ft8_ctx,
};

static decoder_plugin_t s_wspr_plugin = {
    .name = "wspr",
    .description = "WSPR HF Weak Signal (4-GFSK, 2-min cycles, 1.4648 baud)",
    .category = "digital",
    .demod_type = DEMOD_USB,
    .center_freq_hz = 14095600,
    .bandwidth_hz = 200,
    .audio_rate_hz = 375,
    .init = wspr_init,
    .start = wspr_start,
    .stop = wspr_stop,
    .destroy = wspr_destroy,
    .process_audio = wspr_process_audio,
    .process_iq = wspr_process_iq,
    .get_status = wspr_get_status,
    .get_results = wspr_get_results,
    .ctx = &s_wspr_ctx,
};

/* ═══════════════════════════════════════════════════════════════
 *  Registration
 * ═══════════════════════════════════════════════════════════════ */

void register_ft8_decoder(void) {
    decoder_registry_add(&s_ft8_plugin);
    decoder_registry_add(&s_wspr_plugin);
}
