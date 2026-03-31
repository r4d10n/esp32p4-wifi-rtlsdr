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
    /* Accept both positive and negative correlation (phase inversion) */
    int abs_corr = (train_corr >= 0) ? train_corr : -train_corr;
    ESP_LOGI(TAG, "SCH: train_corr=%d/64 (abs=%d)", train_corr, abs_corr);
    if (abs_corr < 32) {
        return false;
    }
    /* If negative correlation, invert all burst bits */
    if (train_corr < 0) {
        /* Make a local inverted copy for decoding */
        int8_t inv_burst[GSM_BURST_LEN];
        for (int i = 0; i < GSM_BURST_LEN; i++) inv_burst[i] = -burst_bits[i];
        /* Recurse with inverted bits (train_corr will be positive) */
        return gsm_sch_decode(inv_burst, info);
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
 *  GMSK-MLSE Demodulation (airprobe / gr-gsm algorithm)
 *
 *  Pipeline:
 *    1. Map known training sequence to GMSK IQ waveform
 *    2. Correlate received IQ with training template → burst timing
 *    3. Estimate channel impulse response (CIR) at training position
 *    4. Matched filter (MAFI): convolve with conjugate CIR → symbol rate
 *    5. 16-state Viterbi MLSE equalization → 148 hard/soft bits
 * ════════════════════════════════════════════════════════════════ */

#define CHAN_IMP_RESP_LENGTH  GSM_CHAN_IMP_LEN   /* 5 symbols */
#define MLSE_STATES           16                 /* 2^(CHAN_IMP_RESP_LENGTH-1) */

/* GSM normal burst training sequences (3GPP TS 05.02, Table 5.2.3) */
static const uint8_t gsm_train_seq[8][26] = {
    {0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1},
    {0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1},
    {0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0},
    {0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0},
    {0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1},
    {0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0},
    {1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1},
    {1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0},
};

/* Training sequence used for correlation: middle 16 of 26 bits (indices 5..20)
 * to avoid edge effects from the channel impulse response */
#define TRAIN_CORR_START    5
#define TRAIN_CORR_LEN      16

/* SCH training: 64 bits, use middle 48 for correlation (indices 8..55) */
#define SCH_TRAIN_CORR_START  8
#define SCH_TRAIN_CORR_LEN    48

/* ── A. GMSK Mapper ── */

/* Generate GMSK-mapped complex IQ waveform for a known bit sequence.
 * Uses Laurent principal pulse approximation (90-degree rotation per symbol).
 *
 * bits: input bit sequence (0/1)
 * n_bits: number of bits
 * osr: oversampling ratio (4 typical)
 * out_re, out_im: output complex waveform (n_bits * osr samples)
 */
static void gsm_gmsk_mapper(const uint8_t *bits, int n_bits, int osr,
                            float *out_re, float *out_im)
{
    float phase = 0;
    int prev_nrz = 1;  /* assume previous bit was 1 */

    for (int i = 0; i < n_bits; i++) {
        int nrz = 2 * bits[i] - 1;
        int diff = nrz * prev_nrz;
        /* GMSK: each symbol rotates +/-pi/2 */
        float delta_phase = diff * (float)(M_PI / 2.0);
        phase += delta_phase;

        for (int k = 0; k < osr; k++) {
            int idx = i * osr + k;
            out_re[idx] = cosf(phase);
            out_im[idx] = sinf(phase);
        }
        prev_nrz = nrz;
    }
}

/* ── B. Training Sequence Correlator ── */

/* Correlate received IQ signal with GMSK-mapped training sequence.
 * Returns the sample offset of the best correlation peak.
 *
 * iq_re, iq_im: received IQ at OSR samples/symbol
 * n_samples: total received samples
 * train_re, train_im: GMSK-mapped training template
 * template_len: length of training template in samples
 * search_start, search_end: range to search (sample indices)
 * peak_power: output - correlation power at peak
 *
 * Returns: sample offset of peak, or -1 if no significant peak found
 */
static int gsm_correlate_training(const float *iq_re, const float *iq_im,
                                  int n_samples,
                                  const float *train_re, const float *train_im,
                                  int template_len,
                                  int search_start, int search_end,
                                  float *peak_power)
{
    float best_pwr = 0;
    int best_off = -1;

    if (search_end > n_samples - template_len)
        search_end = n_samples - template_len;
    if (search_start < 0)
        search_start = 0;

    for (int off = search_start; off <= search_end; off++) {
        float acc_re = 0, acc_im = 0;
        for (int k = 0; k < template_len; k++) {
            /* conj(template) * received */
            float tr = train_re[k];
            float ti = train_im[k];
            float rr = iq_re[off + k];
            float ri = iq_im[off + k];
            acc_re += tr * rr + ti * ri;
            acc_im += tr * ri - ti * rr;
        }
        float pwr = acc_re * acc_re + acc_im * acc_im;
        if (pwr > best_pwr) {
            best_pwr = pwr;
            best_off = off;
        }
    }

    if (peak_power) *peak_power = best_pwr;
    return best_off;
}

/* ── C. Channel Estimator ── */

/* Estimate channel impulse response from training sequence position.
 *
 * rx_re, rx_im: received IQ at training position (TRAIN_LEN * OSR samples)
 * train_re, train_im: known GMSK training waveform
 * train_samps: number of training template samples
 * osr: oversampling ratio
 * chan_re, chan_im: output CIR (CHAN_IMP_RESP_LENGTH * osr taps)
 * rhh_re, rhh_im: output CIR autocorrelation (CHAN_IMP_RESP_LENGTH values,
 *                  sampled at symbol rate, i.e. every osr-th lag)
 */
static void gsm_channel_estimate(const float *rx_re, const float *rx_im,
                                 const float *train_re, const float *train_im,
                                 int train_samps, int osr,
                                 float *chan_re, float *chan_im,
                                 float *rhh_re, float *rhh_im)
{
    int cir_len = CHAN_IMP_RESP_LENGTH * osr;

    /* Compute CIR: chan[l] = sum_k conj(train[k]) * rx[k+l] */
    float energy = 0;
    for (int k = 0; k < train_samps; k++) {
        energy += train_re[k] * train_re[k] + train_im[k] * train_im[k];
    }
    if (energy < 1e-10f) energy = 1.0f;

    for (int l = 0; l < cir_len; l++) {
        float acc_re = 0, acc_im = 0;
        int limit = train_samps;
        if (limit > train_samps - l) limit = train_samps - l;
        for (int k = 0; k < limit; k++) {
            float tr = train_re[k];
            float ti = train_im[k];
            float rr = rx_re[k + l];
            float ri = rx_im[k + l];
            acc_re += tr * rr + ti * ri;   /* conj(train) * rx */
            acc_im += tr * ri - ti * rr;
        }
        chan_re[l] = acc_re / energy;
        chan_im[l] = acc_im / energy;
    }

    /* Compute autocorrelation: rhh[i] = sum_k conj(chan[k]) * chan[k + i*osr]
     * for i = 0..CHAN_IMP_RESP_LENGTH-1.
     * Normalize so rhh[0] = 1.0 (unit channel power). */
    float rhh0 = 0;
    for (int i = 0; i < CHAN_IMP_RESP_LENGTH; i++) {
        float acc_re = 0, acc_im = 0;
        for (int k = 0; k + i * osr < cir_len; k++) {
            float cr = chan_re[k];
            float ci = chan_im[k];
            float dr = chan_re[k + i * osr];
            float di = chan_im[k + i * osr];
            acc_re += cr * dr + ci * di;
            acc_im += cr * di - ci * dr;
        }
        rhh_re[i] = acc_re;
        rhh_im[i] = acc_im;
        if (i == 0) rhh0 = acc_re;  /* rhh[0] is real (autocorrelation at lag 0) */
    }

    /* Scale rhh to improve MLSE branch metric discrimination.
     * Instead of normalizing rhh[0]=1 (which changes MAFI/MLSE balance),
     * scale both rhh and MAFI output by 1/sqrt(rhh0) so everything is
     * in a consistent amplitude range. This is done implicitly by
     * scaling rhh only — the MLSE compares mafi*rhh products. */
    (void)rhh0; /* Normalization deferred — MLSE works on relative metrics */
}

/* ── D. Matched Filter (MAFI) ── */

/* Convolve received signal with time-reversed conjugate CIR.
 * Input: oversampled received IQ (n_symbols * OSR samples)
 * Output: 1 complex sample per symbol (n_symbols samples)
 * filter: channel impulse response (CHAN_IMP_RESP_LENGTH * OSR taps)
 */
static void gsm_mafi(const float *rx_re, const float *rx_im, int n_symbols,
                     int osr,
                     const float *filt_re, const float *filt_im, int filt_len,
                     float *out_re, float *out_im)
{
    for (int n = 0; n < n_symbols; n++) {
        float acc_re = 0, acc_im = 0;
        int base = n * osr;
        for (int k = 0; k < filt_len && (base + k) < n_symbols * osr; k++) {
            /* conj(filter) * received */
            acc_re += filt_re[k] * rx_re[base + k] + filt_im[k] * rx_im[base + k];
            acc_im += filt_re[k] * rx_im[base + k] - filt_im[k] * rx_re[base + k];
        }
        out_re[n] = acc_re;
        out_im[n] = acc_im;
    }
}

/* ── E. 16-state Viterbi MLSE Equalizer ── */

/* 16-state Viterbi MLSE equalizer for GMSK.
 *
 * Input: matched-filtered symbol-rate complex samples (n_sym symbols)
 * rhh: channel autocorrelation (CHAN_IMP_RESP_LENGTH complex values)
 * Output: n_sym detected bits as hard decisions in out_bits[]
 *
 * States represent the last (CHAN_IMP_RESP_LENGTH-1)=4 transmitted bits.
 * 16 states = 2^4.
 */
static void gsm_viterbi_mlse(const float *mf_re, const float *mf_im,
                             int n_sym,
                             const float *cir_ds_re, const float *cir_ds_im,
                             uint8_t *out_bits)
{
    /* Pre-compute expected received signal for each (state, input_bit) pair.
     * Expected = Σ h[k] * nrz[k] where h is the downsampled CIR.
     * State encodes bits [n-1, n-2, ..., n-K+1] (most recent in LSB).
     * The input bit b is at position n (index 0 in the convolution).
     */
    float exp_re[MLSE_STATES][2];
    float exp_im[MLSE_STATES][2];
    int next_state_tbl[MLSE_STATES][2];

    for (int s = 0; s < MLSE_STATES; s++) {
        for (int b = 0; b < 2; b++) {
            float er = 0, ei = 0;
            int nrz_bits[CHAN_IMP_RESP_LENGTH];
            nrz_bits[0] = 2 * b - 1;
            for (int k = 1; k < CHAN_IMP_RESP_LENGTH; k++) {
                int bit = (s >> (k - 1)) & 1;
                nrz_bits[k] = 2 * bit - 1;
            }
            /* Use CIR (not rhh) for expected signal */
            for (int k = 0; k < CHAN_IMP_RESP_LENGTH; k++) {
                er += cir_ds_re[k] * nrz_bits[k];
                ei += cir_ds_im[k] * nrz_bits[k];
            }
            exp_re[s][b] = er;
            exp_im[s][b] = ei;

            /* Next state: shift in bit b, drop oldest bit
             * next = (s >> 1) | (b << (CHAN_IMP_RESP_LENGTH-2)) */
            next_state_tbl[s][b] = (s >> 1) | (b << (CHAN_IMP_RESP_LENGTH - 2));
        }
    }

    /* Path metrics and survivor memory.
     * Use malloc for survivor array since 148 * 16 = 2368 bytes on stack is fine,
     * but be safe for larger bursts. */
    float pm_prev[MLSE_STATES];
    float pm_cur[MLSE_STATES];
    uint8_t survivor[GSM_BURST_LEN][MLSE_STATES];

    for (int s = 0; s < MLSE_STATES; s++) {
        pm_prev[s] = -1e30f;  /* negative infinity */
    }
    pm_prev[0] = 0;  /* start from known state 0 */

    /* ACS (Add-Compare-Select) */
    for (int t = 0; t < n_sym; t++) {
        float rx_r = mf_re[t];
        float rx_i = mf_im[t];

        for (int s = 0; s < MLSE_STATES; s++) {
            pm_cur[s] = -1e30f;
        }

        for (int s = 0; s < MLSE_STATES; s++) {
            if (pm_prev[s] < -1e29f) continue;

            for (int b = 0; b < 2; b++) {
                int ns = next_state_tbl[s][b];
                /* Branch metric: Re(received * conj(expected))
                 * This is the real correlation — higher is better */
                float bm = rx_r * exp_re[s][b] + rx_i * exp_im[s][b];
                float new_pm = pm_prev[s] + bm;
                if (new_pm > pm_cur[ns]) {
                    pm_cur[ns] = new_pm;
                    survivor[t][ns] = (uint8_t)b;
                }
            }
        }

        memcpy(pm_prev, pm_cur, sizeof(pm_prev));
    }

    /* Find best final state */
    int best_state = 0;
    float best_pm = pm_prev[0];
    for (int s = 1; s < MLSE_STATES; s++) {
        if (pm_prev[s] > best_pm) {
            best_pm = pm_prev[s];
            best_state = s;
        }
    }

    /* Traceback */
    uint8_t decoded[GSM_BURST_LEN];
    int state = best_state;
    for (int t = n_sym - 1; t >= 0; t--) {
        int bit = survivor[t][state];
        decoded[t] = (uint8_t)bit;
        /* Reverse state transition: find predecessor
         * next_state = (prev >> 1) | (bit << (K-2))
         * => prev = (next << 1) & mask, but we need to recover the dropped bit.
         * prev = (state & ((1 << (K-2)) - 1)) << 1 | ??? -- use the table */
        /* From next_state_tbl: if next_state_tbl[prev][bit] == state,
         * then prev = (state << 1 | lsb) & mask for some lsb.
         * Since next = (prev >> 1) | (bit << (K-2)),
         * prev = ((state & ~(1 << (K-2))) << 1) | unknown_lsb
         * But more directly: prev had bit 'bit' shifted in to make state,
         * so prev = ((state ^ (bit << (K-2))) << 1) | X for unknown X.
         * Actually: next = (s >> 1) | (b << (K-2))
         * so state = (prev >> 1) | (bit << (K-2))
         * => prev >> 1 = state - (bit << (K-2)), but that's not right for bit ops.
         * => prev = ((state - (bit << (K-2))) << 1) | prev_lsb  -- wrong
         * Simpler: state = (prev >> 1) | (bit << (K-2))
         * => prev & ~1 = (state & ((1 << (K-2)) - 1)) << 1   [upper bits]
         * => prev_msb = bit  [since it was shifted out from prev bit (K-2)]
         * Actually just search: */
        for (int p = 0; p < MLSE_STATES; p++) {
            if (next_state_tbl[p][bit] == state) {
                state = p;
                break;
            }
        }
    }

    memcpy(out_bits, decoded, n_sym);
}

/* ── F. Public API: Full GMSK-MLSE demodulation pipeline ── */

/* Static buffers for pre-computed GMSK training templates.
 * SCH: 64 bits * OSR = 256 samples
 * Normal: 26 bits * OSR = 104 samples */
#define SCH_TRAIN_LEN       64
#define NORM_TRAIN_LEN      26
#define MAX_TRAIN_SAMPS     (SCH_TRAIN_LEN * GSM_OSR)  /* 256 */

static float sch_tmpl_re[MAX_TRAIN_SAMPS];
static float sch_tmpl_im[MAX_TRAIN_SAMPS];
static float norm_tmpl_re[8][NORM_TRAIN_LEN * GSM_OSR];
static float norm_tmpl_im[8][NORM_TRAIN_LEN * GSM_OSR];
static bool mlse_templates_ready = false;

static void mlse_init_templates(void)
{
    if (mlse_templates_ready) return;

    /* Map SCH training sequence */
    gsm_gmsk_mapper(sch_train, SCH_TRAIN_LEN, GSM_OSR,
                    sch_tmpl_re, sch_tmpl_im);

    /* Map all 8 normal burst training sequences */
    for (int i = 0; i < 8; i++) {
        gsm_gmsk_mapper(gsm_train_seq[i], NORM_TRAIN_LEN, GSM_OSR,
                        norm_tmpl_re[i], norm_tmpl_im[i]);
    }

    mlse_templates_ready = true;
    ESP_LOGI(TAG, "MLSE training templates initialized (OSR=%d)", GSM_OSR);
}

int gsm_gmsk_demod(const float *iq_re, const float *iq_im, int n_samples,
                   int8_t *soft_out, int max_out)
{
    if (!iq_re || !iq_im || !soft_out || n_samples < GSM_BURST_LEN * GSM_OSR)
        return 0;

    mlse_init_templates();

    const int osr = GSM_OSR;
    const int burst_samps = GSM_BURST_LEN * osr;  /* 148 * 4 = 592 */
    int total_bits = 0;

    /* Use middle portion of SCH training for correlation template */
    int sch_corr_start_samp = SCH_TRAIN_CORR_START * osr;
    int sch_corr_len_samp = SCH_TRAIN_CORR_LEN * osr;

    /* In the SCH burst, training starts at symbol 42 (sample 42*osr = 168).
     * The correlation uses the middle of training, so the template offset
     * from burst start is (42 + SCH_TRAIN_CORR_START) * osr. */
    int sch_train_offset_samp = (42 + SCH_TRAIN_CORR_START) * osr;

    /* Search for SCH bursts across the entire capture */
    float peak_pwr = 0;
    int peak_off = gsm_correlate_training(
        iq_re, iq_im, n_samples,
        &sch_tmpl_re[sch_corr_start_samp], &sch_tmpl_im[sch_corr_start_samp],
        sch_corr_len_samp,
        0, n_samples - sch_corr_len_samp,
        &peak_pwr);

    if (peak_off < 0) {
        ESP_LOGD(TAG, "MLSE: no SCH correlation peak found");
        return 0;
    }

    /* Compute average signal power for threshold */
    float avg_pwr = 0;
    int pwr_count = (n_samples < 4000) ? n_samples : 4000;
    for (int i = 0; i < pwr_count; i++) {
        avg_pwr += iq_re[i] * iq_re[i] + iq_im[i] * iq_im[i];
    }
    avg_pwr /= pwr_count;

    /* Threshold: peak correlation power should be significantly above noise.
     * The normalized peak_pwr is proportional to template_len^2 * signal_power.
     * Use a threshold relative to avg_pwr * template_len. */
    float threshold = avg_pwr * sch_corr_len_samp * 0.5f;
    if (peak_pwr < threshold) {
        ESP_LOGD(TAG, "MLSE: SCH peak power %.1f below threshold %.1f",
                 peak_pwr, threshold);
        return 0;
    }

    /* Burst start = correlation peak position - training offset within burst */
    int burst_start = peak_off - sch_train_offset_samp;
    if (burst_start < 0) burst_start = 0;
    if (burst_start + burst_samps > n_samples) {
        ESP_LOGD(TAG, "MLSE: burst extends past buffer");
        return 0;
    }

    ESP_LOGI(TAG, "MLSE: SCH peak at sample %d (pwr=%.0f, thresh=%.0f), burst_start=%d",
             peak_off, peak_pwr, threshold, burst_start);

    /* Point to the burst IQ data */
    const float *burst_re = &iq_re[burst_start];
    const float *burst_im = &iq_im[burst_start];

    /* Training position within burst: symbol 42, length 64 */
    int train_pos_samp = 42 * osr;
    int train_full_samps = SCH_TRAIN_LEN * osr;

    /* Channel estimation using the full SCH training sequence */
    int cir_len = CHAN_IMP_RESP_LENGTH * osr;  /* 20 taps */
    float chan_re[CHAN_IMP_RESP_LENGTH * GSM_OSR];
    float chan_im[CHAN_IMP_RESP_LENGTH * GSM_OSR];
    float rhh_re[CHAN_IMP_RESP_LENGTH];
    float rhh_im[CHAN_IMP_RESP_LENGTH];

    gsm_channel_estimate(&burst_re[train_pos_samp], &burst_im[train_pos_samp],
                         sch_tmpl_re, sch_tmpl_im,
                         train_full_samps, osr,
                         chan_re, chan_im, rhh_re, rhh_im);

    /* Debug: log CIR and rhh to verify channel estimation */
    ESP_LOGI(TAG, "MLSE: CIR[0]=(%.4f,%.4f) CIR[4]=(%.4f,%.4f) CIR[8]=(%.4f,%.4f)",
             chan_re[0], chan_im[0], chan_re[4], chan_im[4],
             chan_re[8], chan_im[8]);
    ESP_LOGI(TAG, "MLSE: rhh[0]=(%.4f,%.4f) rhh[1]=(%.4f,%.4f) rhh[2]=(%.4f,%.4f)",
             rhh_re[0], rhh_im[0], rhh_re[1], rhh_im[1], rhh_re[2], rhh_im[2]);

    /* Matched filter: collapse oversampled burst to symbol rate */
    float mf_re[GSM_BURST_LEN];
    float mf_im[GSM_BURST_LEN];
    gsm_mafi(burst_re, burst_im, GSM_BURST_LEN, osr,
             chan_re, chan_im, cir_len, mf_re, mf_im);

    /* Normalize MAFI output and rhh together:
     * MAFI output amplitude depends on channel power (rhh[0] before normalization).
     * Scale both so that the MLSE branch metrics have meaningful magnitude.
     * MAFI_normalized = MAFI / sqrt(rhh0), rhh_normalized = rhh / rhh0
     * Then branch_metric = MAFI_norm * rhh_norm ≈ O(1) per symbol. */
    float mf_power = 0;
    for (int i = 0; i < GSM_BURST_LEN; i++) {
        mf_power += mf_re[i] * mf_re[i] + mf_im[i] * mf_im[i];
    }
    mf_power /= (float)GSM_BURST_LEN;
    float mf_scale = (mf_power > 1e-15f) ? (1.0f / sqrtf(mf_power)) : 1.0f;

    for (int i = 0; i < GSM_BURST_LEN; i++) {
        mf_re[i] *= mf_scale;
        mf_im[i] *= mf_scale;
    }
    /* Also normalize rhh by rhh[0] */
    if (rhh_re[0] > 1e-15f) {
        float inv_rhh0 = 1.0f / rhh_re[0];
        for (int i = 0; i < CHAN_IMP_RESP_LENGTH; i++) {
            rhh_re[i] *= inv_rhh0;
            rhh_im[i] *= inv_rhh0;
        }
    }

    ESP_LOGI(TAG, "MLSE: mf_scale=%.2f mafi[0]=(%.3f,%.3f) rhh[0]=(%.3f,%.3f)",
             mf_scale, mf_re[0], mf_im[0], rhh_re[0], rhh_im[0]);

    /* Downsample CIR: take every OSR-th sample to get symbol-rate CIR */
    float cir_ds_re[CHAN_IMP_RESP_LENGTH];
    float cir_ds_im[CHAN_IMP_RESP_LENGTH];
    for (int k = 0; k < CHAN_IMP_RESP_LENGTH; k++) {
        cir_ds_re[k] = chan_re[k * osr];
        cir_ds_im[k] = chan_im[k * osr];
    }
    /* Normalize downsampled CIR */
    float cir_power = 0;
    for (int k = 0; k < CHAN_IMP_RESP_LENGTH; k++)
        cir_power += cir_ds_re[k] * cir_ds_re[k] + cir_ds_im[k] * cir_ds_im[k];
    if (cir_power > 1e-15f) {
        float cir_scale = 1.0f / sqrtf(cir_power);
        for (int k = 0; k < CHAN_IMP_RESP_LENGTH; k++) {
            cir_ds_re[k] *= cir_scale;
            cir_ds_im[k] *= cir_scale;
        }
    }

    ESP_LOGI(TAG, "MLSE: cir_ds[0]=(%.3f,%.3f) [1]=(%.3f,%.3f) [2]=(%.3f,%.3f)",
             cir_ds_re[0], cir_ds_im[0], cir_ds_re[1], cir_ds_im[1],
             cir_ds_re[2], cir_ds_im[2]);

    /* Two approaches: MLSE and simple differential phase slicer.
     * Try both and see which produces valid data. */

    /* Approach A: Direct phase decision from MAFI output.
     * In GMSK with differential encoding, the phase of the MAFI output
     * rotates by ±π/2 per symbol. Use differential phase to decide bits. */
    /* Differential phase decode from MAFI output.
     * GMSK encoding: nrz = 2*bit-1, diff_nrz = nrz[n]*nrz[n-1], phase += diff_nrz*π/2
     * Demod recovers diff_nrz (the differential encoded NRZ).
     * To recover original bits: nrz[n] = product of all diff_nrz[0..n]
     *   (cumulative product), then bit = (nrz+1)/2.
     *
     * Actually, in GSM the data bits go through: data → diff_encode → GMSK.
     * Diff demod gives back diff_nrz. To get data: XOR consecutive diff bits.
     *
     * Try BOTH interpretations and see which gives valid SCH. */
    uint8_t hard_bits[GSM_BURST_LEN];
    uint8_t diff_bits[GSM_BURST_LEN];  /* raw differential phase decisions */
    int ones_diff = 0;

    /* Step 1: Get raw differential phase decisions */
    for (int i = 0; i < GSM_BURST_LEN; i++) {
        if (i == 0) {
            diff_bits[i] = (mf_re[i] > 0) ? 1 : 0;
        } else {
            float di = mf_im[i] * mf_re[i-1] - mf_re[i] * mf_im[i-1];
            diff_bits[i] = (di > 0) ? 1 : 0;
        }
    }

    /* In GSM, the transmitted burst bits ARE the phase rotations.
     * No cumulative differential decode needed — diff_bits are the bits.
     * Copy diff_bits as the primary output. */
    for (int i = 0; i < GSM_BURST_LEN; i++) {
        hard_bits[i] = diff_bits[i];
        if (hard_bits[i]) ones_diff++;
    }

    ESP_LOGI(TAG, "MLSE: diff-phase bits: %d ones, %d zeros out of %d",
             ones_diff, GSM_BURST_LEN - ones_diff, GSM_BURST_LEN);

    /* If diff-phase gives ~50% ones (looks like data), use it.
     * If mostly zeros (<20 ones), fall back to MLSE. */
    ESP_LOGI(TAG, "MLSE: cumul-diff bits: %d ones, %d zeros out of %d",
             ones_diff, GSM_BURST_LEN - ones_diff, GSM_BURST_LEN);

    /* Also try raw diff_bits (without cumulative decode) — GSM may use
     * a different encoding convention */
    int ones_raw = 0;
    for (int i = 0; i < GSM_BURST_LEN; i++) if (diff_bits[i]) ones_raw++;
    ESP_LOGI(TAG, "MLSE: raw-diff bits: %d ones, %d zeros out of %d",
             ones_raw, GSM_BURST_LEN - ones_raw, GSM_BURST_LEN);

    if (ones_diff < 20 || ones_diff > 128) {
        /* Approach B: Viterbi MLSE */
        gsm_viterbi_mlse(mf_re, mf_im, GSM_BURST_LEN, cir_ds_re, cir_ds_im, hard_bits);
        int ones_mlse = 0;
        for (int i = 0; i < GSM_BURST_LEN; i++) if (hard_bits[i]) ones_mlse++;
        ESP_LOGI(TAG, "MLSE: viterbi bits: %d ones, %d zeros out of %d",
                 ones_mlse, GSM_BURST_LEN - ones_mlse, GSM_BURST_LEN);
    }

    /* Convert hard bits to soft bits (+127 / -127) */
    int n_out = GSM_BURST_LEN;
    if (n_out > max_out) n_out = max_out;
    for (int i = 0; i < n_out; i++) {
        soft_out[total_bits++] = hard_bits[i] ? 127 : -127;
    }

    ESP_LOGI(TAG, "MLSE: decoded %d SCH bits", total_bits);

    /* Now search for normal bursts.
     * GSM frame: bursts repeat every 156.25 symbols.
     * Scan forward from the SCH burst position for additional bursts. */
    int norm_burst_period_samp = (int)(156.25f * osr + 0.5f);  /* 625 samples */

    /* Try to decode bursts forward and backward from SCH position */
    for (int dir = -1; dir <= 1; dir += 2) {
        for (int step = 1; step <= 20; step++) {
            if (total_bits + GSM_BURST_LEN > max_out) break;

            int nb_start = burst_start + dir * step * norm_burst_period_samp;
            if (nb_start < 0 || nb_start + burst_samps > n_samples) continue;

            const float *nb_re = &iq_re[nb_start];
            const float *nb_im = &iq_im[nb_start];

            /* Try all 8 training sequences to find the best match */
            float best_norm_pwr = 0;
            int best_tsc = -1;
            int best_norm_off = -1;

            /* Normal burst: training at symbol 61, length 26 bits.
             * Correlate middle 16 bits (indices 5..20). */
            int nb_train_offset = (61 + TRAIN_CORR_START) * osr;
            int nb_corr_len = TRAIN_CORR_LEN * osr;

            for (int tsc = 0; tsc < 8; tsc++) {
                float tsc_pwr = 0;
                int tsc_off = gsm_correlate_training(
                    nb_re, nb_im, burst_samps,
                    &norm_tmpl_re[tsc][TRAIN_CORR_START * osr],
                    &norm_tmpl_im[tsc][TRAIN_CORR_START * osr],
                    nb_corr_len,
                    nb_train_offset - 4 * osr,  /* search window */
                    nb_train_offset + 4 * osr,
                    &tsc_pwr);
                if (tsc_pwr > best_norm_pwr) {
                    best_norm_pwr = tsc_pwr;
                    best_tsc = tsc;
                    best_norm_off = tsc_off;
                }
            }

            float norm_thresh = avg_pwr * nb_corr_len * 0.3f;
            if (best_tsc < 0 || best_norm_pwr < norm_thresh) continue;

            /* Refine burst start based on correlation peak */
            int refined_start_samp = nb_start + best_norm_off - (61 + TRAIN_CORR_START) * osr;
            if (refined_start_samp < 0 || refined_start_samp + burst_samps > n_samples)
                continue;

            const float *rnb_re = &iq_re[refined_start_samp];
            const float *rnb_im = &iq_im[refined_start_samp];

            /* Channel estimate from normal burst training (26 bits at symbol 61) */
            int nb_train_pos = 61 * osr;
            int nb_train_full = NORM_TRAIN_LEN * osr;

            float nb_chan_re[CHAN_IMP_RESP_LENGTH * GSM_OSR];
            float nb_chan_im[CHAN_IMP_RESP_LENGTH * GSM_OSR];
            float nb_rhh_re[CHAN_IMP_RESP_LENGTH];
            float nb_rhh_im[CHAN_IMP_RESP_LENGTH];

            gsm_channel_estimate(&rnb_re[nb_train_pos], &rnb_im[nb_train_pos],
                                 norm_tmpl_re[best_tsc], norm_tmpl_im[best_tsc],
                                 nb_train_full, osr,
                                 nb_chan_re, nb_chan_im,
                                 nb_rhh_re, nb_rhh_im);

            /* MAFI + MLSE */
            float nb_mf_re[GSM_BURST_LEN];
            float nb_mf_im[GSM_BURST_LEN];
            gsm_mafi(rnb_re, rnb_im, GSM_BURST_LEN, osr,
                     nb_chan_re, nb_chan_im, cir_len, nb_mf_re, nb_mf_im);

            uint8_t nb_hard[GSM_BURST_LEN];
            gsm_viterbi_mlse(nb_mf_re, nb_mf_im, GSM_BURST_LEN,
                             nb_rhh_re, nb_rhh_im, nb_hard);

            int n_out2 = GSM_BURST_LEN;
            if (total_bits + n_out2 > max_out) n_out2 = max_out - total_bits;
            for (int i = 0; i < n_out2; i++) {
                soft_out[total_bits++] = nb_hard[i] ? 127 : -127;
            }
        }
    }

    ESP_LOGI(TAG, "MLSE: total %d bits decoded", total_bits);
    return total_bits;
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
