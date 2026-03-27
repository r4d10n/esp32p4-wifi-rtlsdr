/*
 * RDS Decoder — Plain C, Float
 *
 * Based on analysis of redsea, gr-rds, and SDR++ implementations:
 * 1. Biphase matched filter: correlate with [+1,+1,+1,+1,-1,-1,-1,-1]
 *    for +9 dB SNR gain over raw sampling
 * 2. Zero-crossing clock recovery PLL: tracks Manchester transitions,
 *    compensates decimation rate mismatch and oscillator drift
 * 3. Differential decode: XOR successive symbols per IEC 62106
 * 4. CRC-10 with GF(2) long division, offset word syndrome check
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "rds_decoder.h"

static const char *TAG = "rds";

/* ── RDS Constants ── */

#define RDS_SYMBOL_RATE     2375.0f     /* Biphase symbols/sec */
#define RDS_CRC_POLY        0x5B9
#define RDS_BLOCK_LEN       26
#define RDS_GROUP_LEN       4

#define RDS_OFFSET_A        0x0FC
#define RDS_OFFSET_B        0x198
#define RDS_OFFSET_C        0x168
#define RDS_OFFSET_Cp       0x350
#define RDS_OFFSET_D        0x1B4

/* Matched filter length (samples per biphase symbol half) */
#define MF_HALF             4           /* 4 samples per half-symbol */
#define MF_LEN              (MF_HALF * 2)  /* 8 samples total */

/* Clock recovery PLL parameters */
#define CR_BN               12.0f       /* Loop bandwidth Hz */
#define CR_ZETA             0.707f      /* Damping factor */

/* ── CRC-10 ── */

static uint16_t rds_syndrome(uint32_t block_26bit)
{
    uint32_t reg = block_26bit;
    for (int i = 15; i >= 0; i--) {
        if (reg & ((uint32_t)1 << (i + 10))) {
            reg ^= ((uint32_t)RDS_CRC_POLY << i);
        }
    }
    return (uint16_t)(reg & 0x3FF);
}

static bool rds_check_block(uint32_t block, uint16_t offset)
{
    return rds_syndrome(block) == offset;
}

/* ── Decoder State ── */

struct rds_decoder {
    float sample_rate;

    /* Matched filter delay line */
    float mf_delay[MF_LEN];
    int   mf_pos;

    /* Clock recovery PLL */
    float cr_phase;             /* 0..1 = one symbol period */
    float cr_freq;              /* Nominal: sample_rate / symbol_rate */
    float cr_integrator;
    float cr_kp, cr_ki;        /* PI loop gains */
    float cr_prev_sample;       /* For zero-crossing detection */

    /* Symbol pairing (Manchester decode) */
    int   sym_count;            /* 0=first half, 1=second half */
    float first_half_val;       /* Value of first half-symbol */

    /* Differential decode */
    int   prev_diff_bit;

    /* Bit stream */
    uint32_t shift_reg;
    int      bit_count;

    /* Block sync */
    int   block_idx;
    bool  synced;
    int   good_blocks;
    int   bad_blocks;
    uint16_t group_data[4];

    /* Decoded data */
    rds_data_t data;
};

/* ── Create / Free ── */

rds_decoder_t *rds_decoder_create(float sample_rate)
{
    rds_decoder_t *d = calloc(1, sizeof(rds_decoder_t));
    if (!d) return NULL;

    d->sample_rate = sample_rate;

    /* Clock recovery: nominal samples per symbol */
    d->cr_freq = sample_rate / RDS_SYMBOL_RATE;

    /* PI loop filter gains (from loop bandwidth and damping) */
    float wn = 2.0f * (float)M_PI * CR_BN / sample_rate;
    float denom = 1.0f + 2.0f * CR_ZETA * wn + wn * wn;
    d->cr_kp = (4.0f * CR_ZETA * wn) / denom;
    d->cr_ki = (4.0f * wn * wn) / denom;

    ESP_LOGI(TAG, "RDS decoder: rate=%.0f samp/sym=%.2f kp=%.4f ki=%.6f",
             sample_rate, d->cr_freq, d->cr_kp, d->cr_ki);
    return d;
}

void rds_decoder_free(rds_decoder_t *d) { free(d); }

void rds_decoder_reset(rds_decoder_t *d)
{
    if (!d) return;
    float sr = d->sample_rate;
    float freq = d->cr_freq;
    float kp = d->cr_kp, ki = d->cr_ki;
    memset(d, 0, sizeof(*d));
    d->sample_rate = sr;
    d->cr_freq = freq;
    d->cr_kp = kp;
    d->cr_ki = ki;
}

/* ── Group Decode ── */

static void rds_decode_group(rds_decoder_t *d)
{
    uint16_t a = d->group_data[0];
    uint16_t b = d->group_data[1];
    uint16_t c = d->group_data[2];
    uint16_t dd = d->group_data[3];

    d->data.pi_code = a;
    d->data.valid = true;

    uint8_t group_type = (b >> 12) & 0xF;
    bool version_b = (b >> 11) & 1;
    d->data.tp = (b >> 10) & 1;
    d->data.pty = (b >> 5) & 0x1F;
    d->data.groups_received++;

    /* Group 0A/0B: Programme Service name */
    if (group_type == 0) {
        uint8_t seg = b & 0x03;
        d->data.music = (b >> 3) & 1;
        d->data.ta = (b >> 4) & 1;
        d->data.ps_name[seg * 2]     = (dd >> 8) & 0xFF;
        d->data.ps_name[seg * 2 + 1] = dd & 0xFF;
        d->data.ps_name[8] = '\0';
        if (seg == 3) {
            ESP_LOGI(TAG, "PS: \"%s\" PI:0x%04X PTY:%d",
                     d->data.ps_name, d->data.pi_code, d->data.pty);
        }
    }

    /* Group 2A: RadioText */
    if (group_type == 2 && !version_b) {
        uint8_t seg = b & 0x0F;
        d->data.radio_text[seg * 4]     = (c >> 8) & 0xFF;
        d->data.radio_text[seg * 4 + 1] = c & 0xFF;
        d->data.radio_text[seg * 4 + 2] = (dd >> 8) & 0xFF;
        d->data.radio_text[seg * 4 + 3] = dd & 0xFF;
        d->data.radio_text[64] = '\0';
    }

    /* Group 2B: RadioText (short) */
    if (group_type == 2 && version_b) {
        uint8_t seg = b & 0x0F;
        d->data.radio_text[seg * 2]     = (dd >> 8) & 0xFF;
        d->data.radio_text[seg * 2 + 1] = dd & 0xFF;
        d->data.radio_text[32] = '\0';
    }

    /* Group 4A: Clock Time */
    if (group_type == 4 && !version_b) {
        d->data.hours = ((c & 1) << 4) | (dd >> 12);
        d->data.minutes = (dd >> 6) & 0x3F;
    }
}

/* ── Process Bit ── */

static void rds_process_bit(rds_decoder_t *d, int bit)
{
    d->shift_reg = ((d->shift_reg << 1) | (bit & 1)) & 0x3FFFFFF;
    d->bit_count++;

    if (!d->synced) {
        /* Try block A sync */
        if (rds_check_block(d->shift_reg, RDS_OFFSET_A)) {
            d->synced = true;
            d->block_idx = 1;  /* Next expected: B */
            d->group_data[0] = d->shift_reg >> 10;
            d->bit_count = 0;
            d->good_blocks = 1;
            d->bad_blocks = 0;
            ESP_LOGI(TAG, "RDS sync acquired, PI=0x%04X", d->group_data[0]);
        }
        return;
    }

    if (d->bit_count < RDS_BLOCK_LEN) return;
    d->bit_count = 0;

    static const uint16_t offsets[] = {
        RDS_OFFSET_A, RDS_OFFSET_B, RDS_OFFSET_C, RDS_OFFSET_D
    };
    bool ok = rds_check_block(d->shift_reg, offsets[d->block_idx]);

    if (!ok && d->block_idx == 2) {
        ok = rds_check_block(d->shift_reg, RDS_OFFSET_Cp);
    }

    if (ok) {
        d->group_data[d->block_idx] = d->shift_reg >> 10;
        d->good_blocks++;
        d->bad_blocks = 0;
    } else {
        d->bad_blocks++;
        d->data.block_errors++;
    }

    d->block_idx = (d->block_idx + 1) % RDS_GROUP_LEN;

    if (d->block_idx == 0 && ok) {
        rds_decode_group(d);
    }

    if (d->bad_blocks > 20) {
        d->synced = false;
        ESP_LOGD(TAG, "RDS sync lost");
    }
}

/* ── Main Processing ── */

void rds_decoder_process(rds_decoder_t *d, const float *samples, int count)
{
    if (!d || !samples) return;

    /* Debug counters */
    static int dbg_n = 0, dbg_bits = 0, dbg_cross = 0;
    static float dbg_peak = 0;

    for (int i = 0; i < count; i++) {
        float s = samples[i];
        dbg_n++;
        float as = fabsf(s);
        if (as > dbg_peak) dbg_peak = as;

        /* Step 1: Biphase matched filter.
         * Correlate with [+1,+1,+1,+1,-1,-1,-1,-1] pattern.
         * Implemented as sum(first_4) - sum(last_4).
         * This gives +9 dB SNR gain over raw sampling. */
        d->mf_delay[d->mf_pos] = s;
        d->mf_pos = (d->mf_pos + 1) % MF_LEN;

        float sum_first = 0, sum_last = 0;
        for (int k = 0; k < MF_HALF; k++) {
            sum_first += d->mf_delay[(d->mf_pos + k) % MF_LEN];
            sum_last  += d->mf_delay[(d->mf_pos + MF_HALF + k) % MF_LEN];
        }
        float mf_out = sum_first - sum_last;

        /* Step 2: Zero-crossing clock recovery PLL.
         * Detect zero crossings in matched filter output.
         * Adjust NCO phase to align symbol boundaries. */
        bool crossing = (mf_out > 0 && d->cr_prev_sample <= 0) ||
                        (mf_out <= 0 && d->cr_prev_sample > 0);
        d->cr_prev_sample = mf_out;
        if (crossing) dbg_cross++;

        /* NCO phase advance */
        d->cr_phase += 1.0f;

        /* Clock recovery: nudge on zero crossings.
         * Zero crossings should occur at symbol boundaries (phase = 0 or cr_freq).
         * The timing error is how far the phase is from a boundary. */
        if (crossing) {
            float half = d->cr_freq * 0.5f;
            float err;
            if (d->cr_phase < half) {
                err = d->cr_phase;        /* Early: phase should be near 0 */
            } else {
                err = d->cr_phase - d->cr_freq;  /* Late: phase should be near cr_freq */
            }
            /* Normalize error to [-0.5, 0.5] */
            err /= d->cr_freq;

            /* PI loop filter */
            d->cr_integrator += d->cr_ki * err;
            float correction = d->cr_kp * err + d->cr_integrator;

            /* Apply correction (adjust phase) */
            d->cr_phase += correction * d->cr_freq;
        }

        /* Step 3: Symbol strobe — fire when phase wraps */
        if (d->cr_phase >= d->cr_freq) {
            d->cr_phase -= d->cr_freq;

            /* Symbol decision: sign of matched filter output */
            int sym_positive = (mf_out > 0) ? 1 : 0;

            /* Step 4: Manchester decode — pair consecutive symbols */
            if (d->sym_count == 0) {
                d->first_half_val = mf_out;
                d->sym_count = 1;
            } else {
                /* Decode: first half determines diff bit */
                int diff_bit = (d->first_half_val > 0) ? 1 : 0;

                /* Differential decode: data = same→1, changed→0 */
                int data_bit = (diff_bit == d->prev_diff_bit) ? 1 : 0;
                d->prev_diff_bit = diff_bit;

                rds_process_bit(d, data_bit);
                dbg_bits++;

                d->sym_count = 0;
            }
        }
    }

    /* Debug log every ~5 seconds */
    if (dbg_n >= (int)(d->sample_rate * 5)) {
        d->data.synced = d->synced;
        ESP_LOGI(TAG, "RDS: %d samp %d bits %d cross peak=%.1f sync=%d good=%d bad=%d err=%lu grp=%lu",
                 dbg_n, dbg_bits, dbg_cross, dbg_peak,
                 d->synced, d->good_blocks, d->bad_blocks,
                 (unsigned long)d->data.block_errors,
                 (unsigned long)d->data.groups_received);
        dbg_n = 0; dbg_bits = 0; dbg_cross = 0; dbg_peak = 0;
    }
}

void rds_decoder_get_data(rds_decoder_t *d, rds_data_t *out)
{
    if (d && out) {
        d->data.synced = d->synced;
        *out = d->data;
    }
}
