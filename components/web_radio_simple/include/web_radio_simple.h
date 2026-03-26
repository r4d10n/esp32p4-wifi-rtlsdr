/*
 * Simple Web Radio Control Interface
 *
 * HTTPS server with REST API and WebSocket for FM radio control.
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
    bool        muted;
} web_radio_simple_params_t;

typedef void (*web_radio_simple_change_cb_t)(const web_radio_simple_params_t *params, void *ctx);

typedef struct {
    uint16_t                      http_port;    /* Default 8080 */
    web_radio_simple_change_cb_t  change_cb;    /* Parameter change callback */
    void                         *cb_ctx;       /* Callback context */
} web_radio_simple_config_t;

#define WEB_RADIO_SIMPLE_CONFIG_DEFAULT() { \
    .http_port = 8080, \
    .change_cb = NULL, \
    .cb_ctx = NULL, \
}

esp_err_t web_radio_simple_start(const web_radio_simple_config_t *config);
esp_err_t web_radio_simple_stop(void);

/* Update displayed status (called from main loop) */
void web_radio_simple_update_status(const web_radio_simple_params_t *params, int16_t signal_strength);

/* Push demodulated PCM audio to WebSocket clients (called from DSP pipeline) */
void web_radio_simple_push_audio(const int16_t *samples, int count);

#ifdef __cplusplus
}
#endif
