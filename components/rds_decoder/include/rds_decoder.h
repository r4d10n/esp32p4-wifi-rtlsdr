/*
 * RDS Decoder — Plain C, Float
 *
 * Decodes RDS from 57kHz baseband after accumulate-and-dump decimation.
 * Includes matched filter, symbol clock recovery PLL, differential decode,
 * CRC-10 check, and group processing (0A PS, 2A RT, 4A CT).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t pi_code;
    uint8_t  pty;
    bool     tp;
    bool     ta;
    bool     music;
    char     ps_name[9];        /* 8 chars + NUL */
    char     radio_text[65];    /* 64 chars + NUL */
    uint8_t  hours;
    uint8_t  minutes;
    bool     valid;
    bool     synced;
    uint32_t groups_received;
    uint32_t block_errors;
} rds_data_t;

typedef struct rds_decoder rds_decoder_t;

rds_decoder_t *rds_decoder_create(float sample_rate);
void rds_decoder_free(rds_decoder_t *d);
void rds_decoder_reset(rds_decoder_t *d);

/* Process baseband RDS samples (from 57kHz mixing + accumulate-and-dump) */
void rds_decoder_process(rds_decoder_t *d, const float *samples, int count);

/* Get decoded data */
void rds_decoder_get_data(rds_decoder_t *d, rds_data_t *out);

#ifdef __cplusplus
}
#endif
