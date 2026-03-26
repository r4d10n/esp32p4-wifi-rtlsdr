/*
 * RDS (Radio Data System) Decoder
 *
 * Manchester clock recovery, block sync, CRC-10 error detection,
 * and group decoding for Type 0A/0B (PS), 2A/2B (RT), 4A (CT).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "sdkconfig.h"
#ifdef CONFIG_FM_STEREO_ENABLE

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "rds_decoder.h"

static const char *TAG = "rds";

/* RDS constants.
 * RDS uses biphase (Manchester) encoding at 2375 symbols/sec = 2 × 1187.5 bps.
 * Each data bit produces two half-bit symbols: '1' → [+,-], '0' → [-,+].
 * The clock recovery must run at the SYMBOL rate (2375), not the data rate. */
#define RDS_SYMBOL_RATE 2375.0f /* biphase symbols/sec */
#define RDS_BLOCK_LEN   26      /* 16 data + 10 check bits */
#define RDS_GROUP_LEN   4       /* 4 blocks per group */

/* Offset words for block sync detection */
#define RDS_OFFSET_A    0x0FC
#define RDS_OFFSET_B    0x198
#define RDS_OFFSET_C    0x168
#define RDS_OFFSET_Cp   0x350
#define RDS_OFFSET_D    0x1B4

/* CRC polynomial: x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1 = 0x5B9 */
#define RDS_CRC_POLY    0x5B9

struct rds_decoder {
    /* Biphase (Manchester) symbol clock recovery */
    int16_t     prev_sample;        /* Previous sample for zero-crossing */
    int32_t     clock_phase;        /* Clock recovery phase (wraps at 65536 = 1 symbol) */
    int32_t     clock_inc;          /* Phase increment per input sample */
    int32_t     sym_acc;            /* Integrate-and-dump accumulator for current symbol */

    /* Manchester decode: pair symbols into data bits */
    int         sym_count;          /* Symbol counter within bit (0=first half, 1=second half) */
    int32_t     first_half;         /* First half-symbol value */

    /* Bit stream */
    uint32_t    shift_reg;          /* 26-bit shift register */
    int         bit_count;

    /* Block sync */
    int         block_idx;          /* Current block in group (0=A, 1=B, 2=C, 3=D) */
    bool        synced;             /* Block sync achieved */
    int         good_blocks;        /* Consecutive good blocks (for sync) */
    int         bad_blocks;         /* Consecutive bad blocks (for sync loss) */

    /* Group assembly */
    uint16_t    group_data[4];      /* 4 x 16-bit data blocks */

    /* Decoded data */
    rds_data_t  data;

    /* Differential decode */
    int         prev_diff_bit;

    /* Config */
    uint32_t    sample_rate;
};

/* CRC-10 syndrome check using polynomial long division in GF(2).
 * Generator: g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1 = 0x5B9
 * For a valid block, syndrome equals the offset word for that block position. */
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

/* Check block with offset word, return true if valid */
static bool rds_check_block(uint32_t block_26bit, uint16_t offset)
{
    return rds_syndrome(block_26bit) == offset;
}

/* Process a complete group (4 blocks decoded) */
static void rds_decode_group(rds_decoder_t *rds)
{
    uint16_t a = rds->group_data[0];
    uint16_t b = rds->group_data[1];
    uint16_t c = rds->group_data[2];
    uint16_t d = rds->group_data[3];

    /* Block A: PI code */
    rds->data.pi_code = a;
    rds->data.valid = true;

    /* Block B: Group type, TP, PTY */
    uint8_t group_type = (b >> 12) & 0xF;
    bool version_b = (b >> 11) & 1;
    rds->data.tp = (b >> 10) & 1;
    rds->data.pty = (b >> 5) & 0x1F;

    rds->data.groups_received++;

    /* Group Type 0A/0B: Basic tuning (PS name) */
    if (group_type == 0) {
        uint8_t seg = b & 0x03;  /* PS segment index (0-3) */
        rds->data.music = (b >> 3) & 1;
        rds->data.ta = (b >> 4) & 1;

        /* Block D contains 2 PS characters */
        rds->data.ps_name[seg * 2]     = (d >> 8) & 0xFF;
        rds->data.ps_name[seg * 2 + 1] = d & 0xFF;
        rds->data.ps_name[8] = '\0';

        if (seg == 3) {
            ESP_LOGI(TAG, "PS: \"%s\" PI:%04X PTY:%d",
                     rds->data.ps_name, rds->data.pi_code, rds->data.pty);
        }
    }

    /* Group Type 2A: Radio Text (64 chars) */
    if (group_type == 2 && !version_b) {
        uint8_t seg = b & 0x0F;
        bool ab_flag = (b >> 4) & 1;
        (void)ab_flag; /* TODO: clear text on flag change */

        rds->data.radio_text[seg * 4]     = (c >> 8) & 0xFF;
        rds->data.radio_text[seg * 4 + 1] = c & 0xFF;
        rds->data.radio_text[seg * 4 + 2] = (d >> 8) & 0xFF;
        rds->data.radio_text[seg * 4 + 3] = d & 0xFF;
        rds->data.radio_text[64] = '\0';
    }

    /* Group Type 2B: Radio Text (32 chars) */
    if (group_type == 2 && version_b) {
        uint8_t seg = b & 0x0F;
        rds->data.radio_text[seg * 2]     = (d >> 8) & 0xFF;
        rds->data.radio_text[seg * 2 + 1] = d & 0xFF;
        rds->data.radio_text[32] = '\0';
    }

    /* Group Type 4A: Clock Time */
    if (group_type == 4 && !version_b) {
        uint32_t mjd = ((uint32_t)(b & 0x03) << 15) | (c >> 1);
        rds->data.hours = ((c & 1) << 4) | (d >> 12);
        rds->data.minutes = (d >> 6) & 0x3F;
        (void)mjd; /* MJD not decoded further */
    }
}

/* Process one decoded bit */
static void rds_process_bit(rds_decoder_t *rds, int bit)
{
    rds->shift_reg = ((rds->shift_reg << 1) | (bit & 1)) & 0x3FFFFFF; /* 26 bits */
    rds->bit_count++;

    if (!rds->synced) {
        /* Try to sync: check if current 26 bits match any offset */
        if (rds_check_block(rds->shift_reg, RDS_OFFSET_A)) {
            rds->synced = true;
            rds->block_idx = 1;  /* Next expected block is B */
            rds->group_data[0] = rds->shift_reg >> 10;
            rds->bit_count = 0;
            rds->good_blocks = 1;
            rds->bad_blocks = 0;
            ESP_LOGD(TAG, "RDS sync acquired");
        }
        return;
    }

    /* Synced: wait for 26 bits per block */
    if (rds->bit_count < RDS_BLOCK_LEN) return;
    rds->bit_count = 0;

    /* Check block against expected offset */
    static const uint16_t offsets[] = {
        RDS_OFFSET_A, RDS_OFFSET_B, RDS_OFFSET_C, RDS_OFFSET_D
    };
    bool ok = rds_check_block(rds->shift_reg, offsets[rds->block_idx]);

    if (ok) {
        rds->group_data[rds->block_idx] = rds->shift_reg >> 10;
        rds->good_blocks++;
        rds->bad_blocks = 0;
    } else {
        /* Try C' offset for block C */
        if (rds->block_idx == 2 && rds_check_block(rds->shift_reg, RDS_OFFSET_Cp)) {
            rds->group_data[2] = rds->shift_reg >> 10;
            ok = true;
        } else {
            rds->bad_blocks++;
            rds->data.block_errors++;
            ESP_LOGD(TAG, "Block %c fail: 0x%07lX syn=0x%03X exp=0x%03X",
                     "ABCD"[rds->block_idx],
                     (unsigned long)rds->shift_reg,
                     rds_syndrome(rds->shift_reg),
                     offsets[rds->block_idx]);
        }
    }

    rds->block_idx = (rds->block_idx + 1) % RDS_GROUP_LEN;

    /* Complete group? */
    if (rds->block_idx == 0 && ok) {
        rds_decode_group(rds);
    }

    /* Lose sync after too many bad blocks */
    if (rds->bad_blocks > 10) {
        rds->synced = false;
        ESP_LOGD(TAG, "RDS sync lost");
    }
}

/* Public API */

rds_decoder_t *rds_decoder_create(uint32_t sample_rate)
{
    rds_decoder_t *rds = calloc(1, sizeof(*rds));
    if (!rds) return NULL;

    rds->sample_rate = sample_rate;
    /* When input is already at symbol rate (~2375 sps), each sample IS one
     * symbol.  No clock recovery needed — just sign-detect and Manchester pair. */
    rds->clock_inc = (int32_t)((RDS_SYMBOL_RATE * 65536.0f) / sample_rate);

    ESP_LOGI(TAG, "RDS decoder created: input %"PRIu32" SPS (%.1f samp/sym)",
             sample_rate, (float)sample_rate / RDS_SYMBOL_RATE);
    return rds;
}

void rds_decoder_free(rds_decoder_t *rds) { free(rds); }

void rds_decoder_reset(rds_decoder_t *rds)
{
    if (!rds) return;
    uint32_t sr = rds->sample_rate;
    int32_t ci = rds->clock_inc;
    memset(rds, 0, sizeof(*rds));
    rds->sample_rate = sr;
    rds->clock_inc = ci;
}

void rds_decoder_process(rds_decoder_t *rds, const int16_t *samples, int count)
{
    if (!rds || !samples) return;

    /* Debug counters */
    static int dbg_samples = 0, dbg_bits = 0, dbg_crossings = 0;
    static int16_t dbg_peak = 0;

    for (int i = 0; i < count; i++) {
        int16_t s = samples[i];
        dbg_samples++;
        int16_t abs_s = s > 0 ? s : -s;
        if (abs_s > dbg_peak) dbg_peak = abs_s;

        /* Zero-crossing detection (for debug stats only) */
        bool crossing = (s > 0 && rds->prev_sample <= 0) ||
                        (s <= 0 && rds->prev_sample > 0);
        rds->prev_sample = s;
        if (crossing) dbg_crossings++;

        /* Input is at biphase symbol rate (~2375 sps from accumulate-and-dump).
         * Each input sample IS one biphase symbol — no clock recovery needed.
         * Just sign-detect and Manchester-pair consecutive symbols. */
        int sym_positive = (s > 0) ? 1 : 0;

        /* Manchester/biphase decode: pair consecutive symbols.
         * TX: diff_bit 1 → symbols [+1, -1] (positive first)
         * TX: diff_bit 0 → symbols [-1, +1] (negative first)
         * We use the first half-symbol to determine the diff bit. */
        if (rds->sym_count == 0) {
            /* First half-symbol of pair */
            rds->first_half = sym_positive;
            rds->sym_count = 1;
        } else {
            /* Second half-symbol: decode the pair.
             * diff_bit = 1 if first half was positive */
            int diff_bit = rds->first_half;

            /* Differential decode: data '1' = no phase change, '0' = toggled */
            int data_bit = (diff_bit == rds->prev_diff_bit) ? 1 : 0;
            rds->prev_diff_bit = diff_bit;

            rds_process_bit(rds, data_bit);
            dbg_bits++;

            rds->sym_count = 0;
        }
    }

    /* Debug log every ~5 seconds */
    if (dbg_samples >= 26665) {
        ESP_LOGI(TAG, "RDS: %d samp %d bits %d cross peak=%d sync=%d "
                 "good=%d bad=%d err=%"PRIu32" bits=0x%08lX grp=%"PRIu32,
                 dbg_samples, dbg_bits, dbg_crossings, (int)dbg_peak,
                 rds->synced, rds->good_blocks, rds->bad_blocks,
                 rds->data.block_errors, (unsigned long)rds->shift_reg,
                 rds->data.groups_received);
        dbg_samples = 0;
        dbg_bits = 0;
        dbg_crossings = 0;
        dbg_peak = 0;
    }
}

void rds_decoder_get_data(rds_decoder_t *rds, rds_data_t *out)
{
    if (rds && out) {
        *out = rds->data; /* Atomic-ish copy (single-core reader assumed) */
    }
}

#endif /* CONFIG_FM_STEREO_ENABLE */
