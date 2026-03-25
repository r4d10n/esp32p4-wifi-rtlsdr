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

/* RDS constants */
#define RDS_BITRATE     1187.5f
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
    /* Biphase / Manchester decoder */
    int16_t     prev_sample;        /* Previous sample for zero-crossing */
    int32_t     clock_phase;        /* Clock recovery phase */
    int32_t     clock_inc;          /* Phase increment per sample */
    int16_t     sample_acc;         /* Sample accumulator for bit decision */
    int         sample_count;

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

    /* Config */
    uint32_t    sample_rate;
};

/* CRC-10 syndrome check */
static uint16_t rds_syndrome(uint32_t block_26bit)
{
    uint16_t reg = 0;
    for (int i = 25; i >= 0; i--) {
        uint16_t fb = ((reg >> 9) ^ ((block_26bit >> i) & 1)) & 1;
        reg = (reg << 1) & 0x3FF;
        if (fb) reg ^= RDS_CRC_POLY;
    }
    return reg;
}

/* Check block with offset word, return true if valid */
static bool rds_check_block(uint32_t block_26bit, uint16_t offset)
{
    uint16_t syndrome = rds_syndrome(block_26bit) ^ offset;
    return syndrome == 0;
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
            rds->block_idx = 0;
            rds->group_data[0] = rds->shift_reg >> 10;
            rds->bit_count = 0;
            rds->good_blocks = 1;
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
    rds->clock_inc = (int32_t)((RDS_BITRATE * 65536.0f) / sample_rate);

    ESP_LOGI(TAG, "RDS decoder created: input %"PRIu32" SPS, bitrate %.1f bps",
             sample_rate, RDS_BITRATE);
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

    for (int i = 0; i < count; i++) {
        int16_t s = samples[i];

        /* Manchester clock recovery: detect zero crossings */
        bool crossing = (s > 0 && rds->prev_sample <= 0) ||
                        (s <= 0 && rds->prev_sample > 0);
        rds->prev_sample = s;

        /* Advance clock */
        rds->clock_phase += rds->clock_inc;

        /* Adjust clock on zero crossing (early/late) */
        if (crossing) {
            /* Nudge clock phase toward mid-bit position */
            if (rds->clock_phase > 32768) {
                rds->clock_phase -= rds->clock_inc / 4; /* late: speed up */
            } else {
                rds->clock_phase += rds->clock_inc / 4; /* early: slow down */
            }
        }

        /* Accumulate samples for bit decision */
        rds->sample_acc += s;
        rds->sample_count++;

        /* Sample at mid-bit (clock wraps at 65536 = one bit period) */
        if (rds->clock_phase >= 65536) {
            rds->clock_phase -= 65536;

            /* Bit decision based on accumulated samples */
            int bit = (rds->sample_acc > 0) ? 1 : 0;
            rds_process_bit(rds, bit);

            rds->sample_acc = 0;
            rds->sample_count = 0;
        }
    }
}

void rds_decoder_get_data(rds_decoder_t *rds, rds_data_t *out)
{
    if (rds && out) {
        *out = rds->data; /* Atomic-ish copy (single-core reader assumed) */
    }
}

#endif /* CONFIG_FM_STEREO_ENABLE */
