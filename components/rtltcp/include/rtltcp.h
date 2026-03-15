/*
 * RTL-TCP Server for ESP32-P4
 *
 * Implements the rtl_tcp protocol (port 1234) for streaming IQ samples
 * from an RTL-SDR device to remote SDR clients over TCP.
 *
 * Protocol: https://k3xec.com/rtl-tcp/
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "rtlsdr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RTLTCP_DEFAULT_PORT     1234
#define RTLTCP_DONGLE_INFO_LEN  12

/* RTL-TCP command opcodes (client → server) */
#define RTLTCP_CMD_SET_FREQ         0x01
#define RTLTCP_CMD_SET_SAMPLE_RATE  0x02
#define RTLTCP_CMD_SET_GAIN_MODE    0x03
#define RTLTCP_CMD_SET_GAIN         0x04
#define RTLTCP_CMD_SET_FREQ_CORR    0x05
#define RTLTCP_CMD_SET_IF_GAIN      0x06
#define RTLTCP_CMD_SET_TEST_MODE    0x07
#define RTLTCP_CMD_SET_AGC_MODE     0x08
#define RTLTCP_CMD_SET_DIRECT_SAMP  0x09
#define RTLTCP_CMD_SET_OFFSET_TUNE  0x0A
#define RTLTCP_CMD_SET_RTL_XTAL     0x0B
#define RTLTCP_CMD_SET_TUNER_XTAL   0x0C
#define RTLTCP_CMD_SET_GAIN_INDEX   0x0D
#define RTLTCP_CMD_SET_BIAS_TEE     0x0E

/* DongleInfo header (server → client, 12 bytes, big-endian) */
typedef struct __attribute__((packed)) {
    uint32_t magic;         /* "RTL0" = 0x52544C30 */
    uint32_t tuner_type;    /* rtlsdr_tuner_t value */
    uint32_t gain_count;    /* number of gain steps */
} rtltcp_dongle_info_t;

/* Command packet (client → server, 5 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  cmd;
    uint32_t param;         /* big-endian */
} rtltcp_cmd_t;

/* Server configuration */
typedef struct {
    uint16_t        port;       /* TCP listen port (default 1234) */
    rtlsdr_dev_t   *dev;       /* RTL-SDR device handle */
    uint32_t        ring_size;  /* Ring buffer size in bytes (default 2MB) */
} rtltcp_config_t;

#define RTLTCP_CONFIG_DEFAULT() { \
    .port = RTLTCP_DEFAULT_PORT, \
    .dev = NULL, \
    .ring_size = (2 * 1024 * 1024), \
}

/* Server handle */
typedef struct rtltcp_server rtltcp_server_t;

/*
 * Start the RTL-TCP server. Creates tasks for:
 * - TCP listener (accepts one client at a time)
 * - IQ data sender (ring buffer → TCP socket)
 * - Command receiver (TCP socket → rtlsdr control)
 */
esp_err_t rtltcp_server_start(rtltcp_server_t **server, const rtltcp_config_t *config);

/*
 * Stop the RTL-TCP server and free resources.
 */
esp_err_t rtltcp_server_stop(rtltcp_server_t *server);

/*
 * Push IQ data into the ring buffer (called from rtlsdr async callback).
 * Returns number of bytes written (may be less than len if buffer full).
 */
uint32_t rtltcp_push_samples(rtltcp_server_t *server, const uint8_t *data, uint32_t len);

/*
 * Get current client connection status.
 */
bool rtltcp_is_client_connected(rtltcp_server_t *server);

#ifdef __cplusplus
}
#endif
