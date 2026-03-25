/*
 * Web Radio Control Interface
 *
 * HTTP server with REST API and WebSocket for FM radio control.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Radio parameter change callback */
typedef struct {
    uint32_t    frequency;
    int         gain;           /* tenths of dB, 0=auto */
    uint8_t     volume;         /* 0-100 */
    int         mode;           /* 0=WBFM, 1=NBFM */
    uint32_t    filter_bw;      /* Hz */
    bool        muted;
    uint8_t     squelch;        /* 0=off, 1-100 */
    bool        nb_enabled;
    uint8_t     nb_threshold;
    bool        stereo;         /* true if stereo pilot detected and PLL locked */
    uint8_t     stereo_blend;   /* blend ratio 0-100 (0=mono, 100=full stereo) */
    int8_t      scan_request;   /* 0=none, 1=scan up, -1=scan down, 2=stop */
} web_radio_params_t;

typedef void (*web_radio_change_cb_t)(const web_radio_params_t *params, void *ctx);

typedef struct {
    uint16_t            http_port;      /* Default 8080 */
    web_radio_change_cb_t change_cb;    /* Parameter change callback */
    void               *cb_ctx;         /* Callback context */
} web_radio_config_t;

#define WEB_RADIO_CONFIG_DEFAULT() { \
    .http_port = 8080, \
    .change_cb = NULL, \
    .cb_ctx = NULL, \
}

esp_err_t web_radio_start(const web_radio_config_t *config);
esp_err_t web_radio_stop(void);

/* Update displayed status (called from main loop) */
void web_radio_update_status(const web_radio_params_t *params, int16_t signal_strength);

/* Update RDS data for /api/rds endpoint */
void web_radio_update_rds(const char *ps_name, const char *radio_text,
                          uint16_t pi_code, uint8_t pty, bool stereo);

#ifdef __cplusplus
}
#endif
