/*
 * WebSDR Server for ESP32-P4
 *
 * Combined HTTP + WebSocket server that streams FFT waterfall data
 * and narrowband DDC IQ to browser clients.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "rtlsdr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WEBSDR_DEFAULT_PORT     8080
#define WEBSDR_DEFAULT_FFT_SIZE 1024
#define WEBSDR_DEFAULT_FFT_RATE 20

/* WebSocket binary message types */
#define WEBSDR_MSG_FFT  0x01
#define WEBSDR_MSG_IQ   0x02

typedef struct websdr_server websdr_server_t;

typedef struct {
    uint16_t        http_port;      /* HTTP/WS port (default 8080) */
    rtlsdr_dev_t   *dev;           /* RTL-SDR device handle */
    uint32_t        fft_size;       /* FFT bins (default 1024, power of 2) */
    uint32_t        fft_rate;       /* FFT frames per second (default 20) */
} websdr_config_t;

#define WEBSDR_CONFIG_DEFAULT() { \
    .http_port = WEBSDR_DEFAULT_PORT, \
    .dev = NULL, \
    .fft_size = WEBSDR_DEFAULT_FFT_SIZE, \
    .fft_rate = WEBSDR_DEFAULT_FFT_RATE, \
}

/**
 * Start the WebSDR server. Creates HTTP server, WebSocket endpoint,
 * and FFT processing task.
 */
esp_err_t websdr_server_start(websdr_server_t **server, const websdr_config_t *config);

/**
 * Stop the WebSDR server and free resources.
 */
esp_err_t websdr_server_stop(websdr_server_t *server);

/**
 * Push IQ samples for FFT and DDC processing.
 * Called from the RTL-SDR async read callback.
 */
void websdr_push_samples(websdr_server_t *server, const uint8_t *iq_data, uint32_t len);

#ifdef __cplusplus
}
#endif
