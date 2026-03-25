/*
 * RDS (Radio Data System) Decoder
 *
 * Decodes RDS data from the 57 kHz subcarrier in the FM MPX signal.
 * Extracts: PI code, PS name, Radio Text, PTY, Clock Time.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include "sdkconfig.h"

#ifdef CONFIG_FM_STEREO_ENABLE

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Decoded RDS data */
typedef struct {
    uint16_t    pi_code;            /* Program Identification */
    char        ps_name[9];         /* Program Service name (8 chars + null) */
    char        radio_text[65];     /* Radio Text (64 chars + null) */
    uint8_t     pty;                /* Program Type (0-31) */
    bool        tp;                 /* Traffic Program flag */
    bool        ta;                 /* Traffic Announcement flag */
    bool        music;              /* Music/Speech flag */
    uint8_t     hours;              /* Clock time hours (0-23) */
    uint8_t     minutes;            /* Clock time minutes (0-59) */
    bool        valid;              /* At least PI code received */
    uint32_t    groups_received;    /* Total groups successfully decoded */
    uint32_t    block_errors;       /* Block CRC errors */
} rds_data_t;

typedef struct rds_decoder rds_decoder_t;

/**
 * Create RDS decoder.
 * @param sample_rate  Rate of RDS baseband input (after 57kHz mixing + decimation)
 */
rds_decoder_t *rds_decoder_create(uint32_t sample_rate);
void rds_decoder_free(rds_decoder_t *rds);
void rds_decoder_reset(rds_decoder_t *rds);

/**
 * Feed RDS baseband samples (after 57kHz mixing, LPF, decimation to ~5kHz).
 * @param rds       Decoder handle
 * @param samples   int16 RDS baseband samples
 * @param count     Number of samples
 */
void rds_decoder_process(rds_decoder_t *rds, const int16_t *samples, int count);

/**
 * Get current decoded RDS data.
 * Thread-safe -- returns a snapshot copy.
 */
void rds_decoder_get_data(rds_decoder_t *rds, rds_data_t *out);

#ifdef __cplusplus
}
#endif
#endif /* CONFIG_FM_STEREO_ENABLE */
