/*
 * GSM Channel Decoder — SCH, BCCH, and System Information parsing
 *
 * Implements:
 *   - Viterbi K=5 decoder (rate 1/2 convolutional code)
 *   - SCH burst decode (BSIC + frame number)
 *   - Differential GMSK demodulation
 *   - BCCH block decode (de-interleave + Viterbi + Fire CRC)
 *   - SI3/SI1 parsing (cell identity, ARFCN list)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "gsm_decode.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "gsm_dec";

/* ════════════════════════════════════════════════════════════════
 *  Viterbi K=5 Constants and Tables
 *  Rate 1/2, K=5, generators G0=0x19 (11001b), G1=0x1B (11011b)
 *  16 states (2^(K-1) = 2^4)
 * ════════════════════════════════════════════════════════════════ */

#define VITERBI_STATES  16
#define VITERBI_INF     0x7FFF

/* Parity (popcount mod 2) of a 5-bit value */
static inline int parity5(int v)
{
    v ^= (v >> 2);
    v ^= (v >> 1);
    return v & 1;
}

/*
 * Pre-computed state transition and output tables.
 *
 * For state s (0..15), input bit b (0 or 1):
 *   reg = (s << 1) | b   (5-bit shift register value)
 *   next_state = (s >> 1) | (b << 3)
 *   output_g0 = parity(reg & 0x19)
 *   output_g1 = parity(reg & 0x1B)
 */

/* next_state[state][input_bit] */
static const uint8_t vit_next[VITERBI_STATES][2] = {
    { 0,  8}, { 0,  8}, { 1,  9}, { 1,  9},
    { 2, 10}, { 2, 10}, { 3, 11}, { 3, 11},
    { 4, 12}, { 4, 12}, { 5, 13}, { 5, 13},
    { 6, 14}, { 6, 14}, { 7, 15}, { 7, 15},
};

/* output[state][input_bit] = (g0 << 1) | g1, packed as 2-bit value */
static uint8_t vit_output[VITERBI_STATES][2];

/* Previous states that lead into state s: prev_state[s][0] with bit 0, prev_state[s][1] with bit 1 */
static uint8_t vit_prev[VITERBI_STATES][2];
static uint8_t vit_prev_bit[VITERBI_STATES][2];

static bool vit_tables_ready = false;

static void viterbi_init_tables(void)
{
    if (vit_tables_ready) return;

    /* Build output table */
    for (int s = 0; s < VITERBI_STATES; s++) {
        for (int b = 0; b < 2; b++) {
            int reg = (s << 1) | b;
            int g0 = parity5(reg & 0x19);
            int g1 = parity5(reg & 0x1B);
            vit_output[s][b] = (uint8_t)((g0 << 1) | g1);
        }
    }

    /* Build reverse lookup: for each state ns, find which (prev_state, bit) leads to it */
    memset(vit_prev, 0, sizeof(vit_prev));
    memset(vit_prev_bit, 0, sizeof(vit_prev_bit));

    /* Each state ns has exactly 2 predecessors */
    uint8_t prev_count[VITERBI_STATES] = {0};
    for (int s = 0; s < VITERBI_STATES; s++) {
        for (int b = 0; b < 2; b++) {
            int ns = vit_next[s][b];
            int idx = prev_count[ns]++;
            vit_prev[ns][idx] = (uint8_t)s;
            vit_prev_bit[ns][idx] = (uint8_t)b;
        }
    }

    vit_tables_ready = true;
}

/* ════════════════════════════════════════════════════════════════
 *  Viterbi K=5 Decoder
 * ════════════════════════════════════════════════════════════════ */

int gsm_viterbi_k5_decode(const int8_t *soft_bits, int n_enc_bits,
                          uint8_t *out_bits, int out_len)
{
    viterbi_init_tables();

    if (!soft_bits || !out_bits || n_enc_bits < 2 || (n_enc_bits & 1)) {
        ESP_LOGE(TAG, "viterbi: invalid params (n_enc=%d)", n_enc_bits);
        return -1;
    }

    int n_stages = n_enc_bits / 2;  /* number of trellis stages */
    int n_decoded = n_stages;       /* includes tail bits */

    if (out_len < (n_decoded + 7) / 8) {
        ESP_LOGE(TAG, "viterbi: output buffer too small (%d < %d)",
                 out_len, (n_decoded + 7) / 8);
        return -1;
    }

    /* Path metrics: current and previous */
    int16_t pm_cur[VITERBI_STATES];
    int16_t pm_prev[VITERBI_STATES];

    /* Survivor memory — store the input bit that won at each (stage, state).
     * For SCH (39 stages) or BCCH (228 stages), stack allocation is fine. */
    /* Max stages: BCCH = 456/2 = 228 */
    if (n_stages > 256) {
        ESP_LOGE(TAG, "viterbi: too many stages (%d)", n_stages);
        return -1;
    }
    uint8_t survivor[256][VITERBI_STATES];

    /* Initialize: start from state 0 (known tail) */
    for (int s = 0; s < VITERBI_STATES; s++) {
        pm_prev[s] = VITERBI_INF;
    }
    pm_prev[0] = 0;

    /* ACS (Add-Compare-Select) loop */
    for (int t = 0; t < n_stages; t++) {
        int8_t s0 = soft_bits[2 * t];      /* soft bit for G0 output */
        int8_t s1 = soft_bits[2 * t + 1];  /* soft bit for G1 output */

        for (int s = 0; s < VITERBI_STATES; s++) {
            pm_cur[s] = VITERBI_INF;
        }

        for (int s = 0; s < VITERBI_STATES; s++) {
            if (pm_prev[s] == VITERBI_INF) continue;

            for (int b = 0; b < 2; b++) {
                int ns = vit_next[s][b];
                uint8_t out = vit_output[s][b];
                int exp_g0 = (out >> 1) & 1;
                int exp_g1 = out & 1;

                /* Branch metric: Manhattan distance in soft domain.
                 * Expected soft value: bit 0 -> -127, bit 1 -> +127 */
                int expected_g0 = exp_g0 ? 127 : -127;
                int expected_g1 = exp_g1 ? 127 : -127;
                int bm = abs(s0 - expected_g0) + abs(s1 - expected_g1);

                int16_t new_metric = pm_prev[s] + (int16_t)bm;
                if (new_metric < pm_cur[ns]) {
                    pm_cur[ns] = new_metric;
                    survivor[t][ns] = (uint8_t)b;
                }
            }
        }

        memcpy(pm_prev, pm_cur, sizeof(pm_prev));
    }

    /* Find best final state (prefer state 0 for tail-biting, but take minimum) */
    int best_state = 0;
    int16_t best_metric = pm_prev[0];
    for (int s = 1; s < VITERBI_STATES; s++) {
        if (pm_prev[s] < best_metric) {
            best_metric = pm_prev[s];
            best_state = s;
        }
    }

    /* Traceback */
    uint8_t decoded[256];
    int state = best_state;
    for (int t = n_stages - 1; t >= 0; t--) {
        decoded[t] = survivor[t][state];
        /* Reverse the state transition to find previous state */
        int bit = decoded[t];
        /* state = prev_state such that vit_next[prev_state][bit] == state
         * Since next = (prev >> 1) | (bit << 3), we have:
         * prev = (state << 1) | (some bit that was shifted out)
         * Actually we need to invert: if current state at t+1 is 'state',
         * and the bit entering at stage t is decoded[t], then the state at t is
         * found from the survivor path. We use the reverse table. */
        /* The state at time t: we entered 'state' via bit 'decoded[t]' from some prev.
         * vit_next[prev][decoded[t]] == state, so prev has bit pattern such that
         * (prev >> 1) | (decoded[t] << 3) == state
         * => prev = ((state & 0x07) << 1) | prev_lsb, but prev_lsb is the bit
         * that was shifted out. Since we track survivor[t][state] = bit that won,
         * we need the predecessor state. */
        /* Simpler: from current state and the bit that was input,
         * prev_state = ((state << 1) & 0x0F) | (some unknown bit)
         * But we don't know that bit. Use reverse table instead. */
        for (int p = 0; p < 2; p++) {
            if (vit_prev_bit[state][p] == bit) {
                state = vit_prev[state][p];
                break;
            }
        }
    }

    /* Pack decoded bits into output bytes, MSB first */
    memset(out_bits, 0, (n_decoded + 7) / 8);
    for (int i = 0; i < n_decoded; i++) {
        if (decoded[i]) {
            out_bits[i / 8] |= (0x80 >> (i % 8));
        }
    }

    return n_decoded;
}

/* ════════════════════════════════════════════════════════════════
 *  SCH Burst Decode
 * ════════════════════════════════════════════════════════════════ */

/* SCH training sequence (64 bits) */
/* SCH training sequence — used for burst timing alignment in future.
 * Kept as reference for the known 64-bit SCH sync word. */
static const uint8_t sch_train[64]
#if defined(__GNUC__)
    __attribute__((used))
#endif
= {
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,
    1,0,1,0,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,1,0,1,0,0,1,0,1,0,0,1,0,0
};

/* SCH parity polynomial: 10-bit CRC, generator = x^10 + x^8 + x^6 + x^5 + x^4 + x^2 + x + 1
 * Binary: 10101110111 = 0x575 (per 3GPP TS 05.03 section 4.7) */
#define SCH_PARITY_POLY  0x0575
#define SCH_PARITY_BITS  10

/* Extract a single bit from packed byte array (MSB first) */
static inline int get_bit(const uint8_t *packed, int bit_idx)
{
    return (packed[bit_idx / 8] >> (7 - (bit_idx % 8))) & 1;
}

/* Check SCH 10-bit parity on 25 data bits + 10 parity bits = 35 bits */
static bool sch_parity_check(const uint8_t *packed, int n_data, int n_parity)
{
    uint16_t reg = 0;

    for (int i = 0; i < n_data + n_parity; i++) {
        int bit = get_bit(packed, i);
        int feedback = ((reg >> (SCH_PARITY_BITS - 1)) ^ bit) & 1;
        reg <<= 1;
        if (feedback) {
            reg ^= SCH_PARITY_POLY;
        }
        reg &= (1 << SCH_PARITY_BITS) - 1;
    }

    return (reg == 0);
}

bool gsm_sch_decode(const int8_t *burst_bits, gsm_sch_info_t *info)
{
    if (!burst_bits || !info) return false;

    memset(info, 0, sizeof(*info));

    /* SCH burst structure (148 bits):
     * [3 tail] [39 encoded_a] [64 training] [39 encoded_b] [3 tail] [guard]
     * Encoded bits start at offset 3 (first half) and 3+39+64=106 (second half) */

    /* Validate training sequence correlation (bits 42..105 = 64-bit training) */
    int train_corr = 0;
    for (int i = 0; i < 64; i++) {
        /* soft_bit > 0 means bit=1, sch_train[i] is hard 0/1 */
        int expected = sch_train[i] ? 1 : -1;
        int received = (burst_bits[42 + i] > 0) ? 1 : -1;
        train_corr += expected * received;
    }
    if (train_corr < 32) {
        ESP_LOGD(TAG, "SCH: training correlation too low (%d/64)", train_corr);
        return false;
    }

    int8_t enc_soft[GSM_SCH_ENC_BITS]; /* 78 encoded soft bits */

    /* First half: bits 3..41 (39 bits) */
    memcpy(&enc_soft[0], &burst_bits[3], 39 * sizeof(int8_t));

    /* Second half: bits 106..144 (39 bits) */
    memcpy(&enc_soft[39], &burst_bits[106], 39 * sizeof(int8_t));

    /* Viterbi decode: 78 encoded bits -> 39 decoded bits
     * 39 decoded = 25 data + 10 parity + 4 tail */
    uint8_t decoded_packed[5]; /* 39 bits = 5 bytes */
    int n_dec = gsm_viterbi_k5_decode(enc_soft, GSM_SCH_ENC_BITS,
                                       decoded_packed, sizeof(decoded_packed));
    if (n_dec < 0) {
        ESP_LOGD(TAG, "SCH: Viterbi decode failed");
        return false;
    }

    /* Check parity: 25 data + 10 parity bits */
    if (!sch_parity_check(decoded_packed, GSM_SCH_DATA_BITS, SCH_PARITY_BITS)) {
        ESP_LOGD(TAG, "SCH: parity check failed");
        return false;
    }

    /* Reject all-zero decoded data (noise artifact) */
    if (decoded_packed[0] == 0 && decoded_packed[1] == 0 &&
        decoded_packed[2] == 0 && decoded_packed[3] == 0) {
        ESP_LOGD(TAG, "SCH: all-zero data rejected");
        return false;
    }

    /* Extract fields from 25 data bits (MSB first in packed format):
     * bits[0..10]  = T1 (11 bits)
     * bits[11..15] = T2 (5 bits)
     * bits[16..18] = T3' (3 bits)
     * bits[19..24] = BSIC (6 bits): NCC = bits[19..21], BCC = bits[22..24] */

    uint16_t t1 = 0;
    for (int i = 0; i < 11; i++) {
        t1 = (t1 << 1) | get_bit(decoded_packed, i);
    }

    uint8_t t2 = 0;
    for (int i = 11; i < 16; i++) {
        t2 = (t2 << 1) | get_bit(decoded_packed, i);
    }

    uint8_t t3p = 0;
    for (int i = 16; i < 19; i++) {
        t3p = (t3p << 1) | get_bit(decoded_packed, i);
    }

    uint8_t ncc = 0;
    for (int i = 19; i < 22; i++) {
        ncc = (ncc << 1) | get_bit(decoded_packed, i);
    }

    uint8_t bcc = 0;
    for (int i = 22; i < 25; i++) {
        bcc = (bcc << 1) | get_bit(decoded_packed, i);
    }

    info->t1  = t1;
    info->t2  = t2;
    info->t3p = t3p;
    info->ncc = ncc;
    info->bcc = bcc;
    info->bsic = (ncc << 3) | bcc;

    /* Compute full frame number:
     * FN = 51 * ((T3' - 1) * 10 + T1) + T2   if T3' > 0
     * FN = 51 * T1 + T2                        if T3' == 0 */
    if (t3p > 0) {
        info->fn = 51 * ((uint32_t)(t3p - 1) * 10 + t1) + t2;
    } else {
        info->fn = 51 * (uint32_t)t1 + t2;
    }

    info->valid = true;

    ESP_LOGI(TAG, "SCH: BSIC=%u (NCC=%u BCC=%u) FN=%lu T1=%u T2=%u T3'=%u",
             info->bsic, info->ncc, info->bcc, (unsigned long)info->fn,
             info->t1, info->t2, info->t3p);

    return true;
}

/* ════════════════════════════════════════════════════════════════
 *  GMSK Differential Demodulation
 * ════════════════════════════════════════════════════════════════ */

int gsm_gmsk_demod(const float *iq_re, const float *iq_im, int n_samples,
                   int8_t *soft_out, int max_out)
{
    if (!iq_re || !iq_im || !soft_out || n_samples < 2) return 0;

    int n_bits = 0;
    float scale = 127.0f / (float)(M_PI / 2.0);

    for (int i = 1; i < n_samples && n_bits < max_out; i++) {
        /* Compute z[i] * conj(z[i-1]) */
        float re_cur = iq_re[i];
        float im_cur = iq_im[i];
        float re_prev = iq_re[i - 1];
        float im_prev = iq_im[i - 1];

        /* Product: (re_cur + j*im_cur) * (re_prev - j*im_prev) */
        float prod_re = re_cur * re_prev + im_cur * im_prev;
        float prod_im = im_cur * re_prev - re_cur * im_prev;

        /* Phase difference */
        float phase_diff = atan2f(prod_im, prod_re);

        /* Scale to soft bit range: positive phase -> bit 1, negative -> bit 0 */
        float soft_val = phase_diff * scale;

        /* Clamp to [-127, +127] */
        if (soft_val > 127.0f) soft_val = 127.0f;
        if (soft_val < -127.0f) soft_val = -127.0f;

        soft_out[n_bits++] = (int8_t)soft_val;
    }

    return n_bits;
}

/* ════════════════════════════════════════════════════════════════
 *  BCCH Block Decode
 * ════════════════════════════════════════════════════════════════ */

/*
 * Fire code CRC-40 for BCCH.
 * Polynomial: g(x) = (x^23 + 1)(x^17 + x^3 + 1)
 *
 * Expanded: x^40 + x^26 + x^23 + x^17 + x^3 + 1
 *
 * We use a 40-bit shift register. Since C doesn't have a native 40-bit type,
 * use uint64_t.
 */
#define FIRE_POLY_HI  0x00000100ULL   /* bits 40 and 26 in a 41-bit poly */
#define FIRE_POLY     0x0004820009ULL /* x^40 + x^26 + x^23 + x^17 + x^3 + 1
                                       * = (1 << 40) | (1 << 26) | (1 << 23) | (1 << 17) | (1 << 3) | 1
                                       * But we only store the lower 40 bits (excluding x^40):
                                       * (1 << 26) | (1 << 23) | (1 << 17) | (1 << 3) | 1
                                       * = 0x04820009 */
#define FIRE_CRC_MASK 0xFFFFFFFFFFULL /* 40-bit mask */

static bool fire_crc_check(const uint8_t *packed_bits, int n_total_bits)
{
    /* Check CRC over n_total_bits (184 data + 40 CRC = 224 bits).
     * If the CRC is correct, the remainder should be zero. */
    uint64_t reg = 0;

    for (int i = 0; i < n_total_bits; i++) {
        int bit = get_bit(packed_bits, i);
        int feedback = (int)((reg >> 39) & 1) ^ bit;
        reg = (reg << 1) & FIRE_CRC_MASK;
        if (feedback) {
            reg ^= 0x0004820009ULL;
        }
    }

    return (reg == 0);
}

bool gsm_bcch_decode(const int8_t burst_bits[4][116], uint8_t l2_frame[GSM_BCCH_BLOCK_LEN])
{
    if (!burst_bits || !l2_frame) return false;

    /* Step 1: De-interleave 456 encoded soft bits from 4 bursts.
     * Interleaving pattern: encoded bit j -> burst (j % 4), position (j / 4)
     * De-interleave: encoded[j] = burst[j % 4][j / 4] */
    int8_t enc_soft[GSM_BCCH_ENC_BITS]; /* 456 soft bits */

    for (int j = 0; j < GSM_BCCH_ENC_BITS; j++) {
        int burst_idx = j % 4;
        int bit_pos = j / 4;
        if (bit_pos >= 116) {
            ESP_LOGE(TAG, "BCCH: deinterleave overflow at j=%d", j);
            return false;
        }
        enc_soft[j] = burst_bits[burst_idx][bit_pos];
    }

    /* Step 2: Viterbi decode: 456 encoded -> 228 decoded bits
     * 228 = 184 data + 40 Fire CRC + 4 tail */
    uint8_t decoded_packed[29]; /* 228 bits = 28.5 bytes -> 29 bytes */
    int n_dec = gsm_viterbi_k5_decode(enc_soft, GSM_BCCH_ENC_BITS,
                                       decoded_packed, sizeof(decoded_packed));
    if (n_dec < 0) {
        ESP_LOGD(TAG, "BCCH: Viterbi decode failed");
        return false;
    }

    /* Step 3: Fire code CRC check over 184 + 40 = 224 bits */
    if (!fire_crc_check(decoded_packed, GSM_BCCH_DATA_BITS + GSM_FIRE_CRC_LEN)) {
        ESP_LOGD(TAG, "BCCH: Fire CRC check failed");
        return false;
    }

    /* Reject all-zero frames (noise decodes to all-zero, which has valid CRC=0) */
    bool all_zero = true;
    for (int i = 0; i < GSM_BCCH_BLOCK_LEN && all_zero; i++) {
        if (decoded_packed[i] != 0) all_zero = false;
    }
    if (all_zero) {
        ESP_LOGD(TAG, "BCCH: all-zero frame rejected");
        return false;
    }

    /* Step 4: Extract 184 data bits = 23 bytes */
    memcpy(l2_frame, decoded_packed, GSM_BCCH_BLOCK_LEN);

    ESP_LOGI(TAG, "BCCH: block decoded successfully");
    return true;
}

/* ════════════════════════════════════════════════════════════════
 *  System Information Parsers
 * ════════════════════════════════════════════════════════════════ */

/* 3GPP TS 04.08 message types for RR management */
#define GSM_SI1_MSG_TYPE  0x19
#define GSM_SI3_MSG_TYPE  0x1B

bool gsm_parse_si3(const uint8_t l2_frame[GSM_BCCH_BLOCK_LEN], gsm_cell_id_t *cell_id)
{
    if (!l2_frame || !cell_id) return false;

    memset(cell_id, 0, sizeof(*cell_id));

    /* L2 pseudo-header on BCCH:
     * Byte 0: Skip Indicator (4 bits hi) | Protocol Discriminator (4 bits lo)
     * Byte 1: Message Type
     * Check for SI3 message type */
    uint8_t msg_type = l2_frame[1];
    if (msg_type != GSM_SI3_MSG_TYPE) {
        return false;
    }

    /* SI3 structure after header (3GPP TS 04.08, 9.1.35):
     * Byte 2-3: Cell Identity (16 bits, big-endian)
     * Byte 4-6: LAI (Location Area Identification)
     *   Byte 4: MCC digit 2 (hi) | MCC digit 1 (lo)
     *   Byte 5: MNC digit 3 (hi) | MCC digit 3 (lo)
     *   Byte 6: MNC digit 2 (hi) | MNC digit 1 (lo)
     * Byte 7-8: LAC (16 bits, big-endian)
     */

    if (GSM_BCCH_BLOCK_LEN < 9) return false; /* sanity */

    /* Cell Identity */
    cell_id->cell_id = ((uint16_t)l2_frame[2] << 8) | l2_frame[3];

    /* MCC: digit1 (lo nibble byte4), digit2 (hi nibble byte4), digit3 (lo nibble byte5) */
    uint8_t mcc_d1 = l2_frame[4] & 0x0F;
    uint8_t mcc_d2 = (l2_frame[4] >> 4) & 0x0F;
    uint8_t mcc_d3 = l2_frame[5] & 0x0F;
    cell_id->mcc = mcc_d1 * 100 + mcc_d2 * 10 + mcc_d3;

    /* MNC: digit1 (lo nibble byte6), digit2 (hi nibble byte6), digit3 (hi nibble byte5) */
    uint8_t mnc_d1 = l2_frame[6] & 0x0F;
    uint8_t mnc_d2 = (l2_frame[6] >> 4) & 0x0F;
    uint8_t mnc_d3 = (l2_frame[5] >> 4) & 0x0F;

    cell_id->mnc = mnc_d1 * 10 + mnc_d2;
    if (mnc_d3 != 0x0F) {
        cell_id->mnc = mnc_d3 * 100 + cell_id->mnc;  /* 3-digit MNC */
    }

    /* LAC */
    cell_id->lac = ((uint16_t)l2_frame[7] << 8) | l2_frame[8];

    cell_id->valid = true;

    ESP_LOGI(TAG, "SI3: MCC=%u MNC=%u LAC=%u CellID=%u",
             cell_id->mcc, cell_id->mnc, cell_id->lac, cell_id->cell_id);

    return true;
}

bool gsm_parse_si1(const uint8_t l2_frame[GSM_BCCH_BLOCK_LEN], uint16_t *arfcn_list, int *n_arfcns)
{
    if (!l2_frame || !arfcn_list || !n_arfcns) return false;

    *n_arfcns = 0;

    /* Check message type */
    uint8_t msg_type = l2_frame[1];
    if (msg_type != GSM_SI1_MSG_TYPE) {
        return false;
    }

    /* SI1 structure (3GPP TS 04.08, 9.1.31):
     * Byte 0-1: L2 header
     * Byte 2-17: Cell Channel Description (16 bytes)
     *   This is a frequency list IE encoding the Cell Allocation (CA).
     *   Format: bit-map 0 format (format identifier in bits 7-6 of byte 2)
     *
     * Bit-map 0 format: each bit represents an ARFCN.
     * Byte 2, bits 0-5: ARFCNs 1-6 (after format bits)
     * Bytes 3-17: ARFCNs 7-124
     * ARFCN 0 is indicated by bit 3 of byte 2 (the spare bit in format 0).
     */

    /* The Cell Channel Description is 16 bytes starting at byte 2 */
    const uint8_t *ccd = &l2_frame[2];

    /* Format identification: bits 7-6 of first byte
     * 00 = bit map 0 (the most common for SI1) */
    uint8_t format = (ccd[0] >> 6) & 0x03;

    if (format != 0) {
        ESP_LOGW(TAG, "SI1: unsupported Cell Channel Description format %u", format);
        /* Only bit-map 0 is implemented for now */
        return false;
    }

    /* Bit-map 0 format:
     * 128 ARFCNs (0..124 for GSM900, we check bits for 0..127)
     * Byte 0: bit5..bit0 correspond to ARFCN 1..6 (after skipping format bits 7-6)
     *         Actually the mapping is more nuanced per 3GPP TS 04.08 10.5.2.1b
     *
     * Simplified bit-map 0:
     * The 16 bytes encode 128 bits. Bit i (where i is counted from MSB of byte 0)
     * corresponds to ARFCN (128 - i - 1) for the first format, but the exact mapping
     * depends on the band. For GSM900:
     *
     * We decode the bitmap and list all set ARFCNs. */

    /* Bitmap: 16 bytes = 128 bits. Bit (byte_n * 8 + (7 - bit_n)) maps to ARFCN.
     * Per 3GPP: ARFCN n is represented by bit (n) in the bitmap, where bit 0
     * of the last byte is ARFCN 0, and bit 7 of the first byte holds the format. */

    /* Skip the 2 format bits (bits 7-6 of byte 0). Parse remaining 126 bits for ARFCNs 1-124.
     * ARFCN 0 is special (controlled by ccd[0] bit 0 in some interpretations). */

    int count = 0;

    /* Parse ARFCNs 1-124 from the bit map */
    for (int arfcn = 1; arfcn <= 124; arfcn++) {
        /* ARFCN n maps to: byte index = (128 - n) / 8, bit index = (128 - n) % 8
         * within the 16-byte CCD */
        int bit_pos = 128 - arfcn;
        int byte_idx = bit_pos / 8;
        int bit_idx = bit_pos % 8;

        if (byte_idx < 0 || byte_idx >= 16) continue;

        if (ccd[byte_idx] & (1 << bit_idx)) {
            arfcn_list[count++] = (uint16_t)arfcn;
        }
    }

    /* Check ARFCN 0 (bit position 128 -> byte 16 which is out of range,
     * so ARFCN 0 is encoded in byte 0 bit 0 per the standard) */
    if (ccd[0] & 0x01) {
        /* Shift existing entries and insert ARFCN 0 at front */
        if (count > 0) {
            for (int i = count; i > 0; i--) {
                arfcn_list[i] = arfcn_list[i - 1];
            }
        }
        arfcn_list[0] = 0;
        count++;
    }

    *n_arfcns = count;

    ESP_LOGI(TAG, "SI1: %d ARFCNs in Cell Allocation", count);

    return true;
}
