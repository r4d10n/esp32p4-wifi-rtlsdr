/*
 * RTL-UDP IQ Streaming for ESP32-P4
 *
 * Streams IQ samples over UDP with a thin header for sequence tracking.
 * Lower overhead than TCP — no ACKs, no congestion control.
 * Use the udp2tcp bridge on the host for rtl_tcp client compatibility.
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

/*
 * UDP Packet Header (8 bytes, prepended to each IQ datagram)
 *
 * Offset  Size  Field
 * 0       4     sequence number (uint32, little-endian, wraps at 2^32)
 * 4       4     timestamp_us (uint32, microseconds since stream start, wraps)
 *
 * Followed by PAYLOAD_SIZE bytes of raw interleaved uint8 IQ data.
 * Total packet size = 8 + PAYLOAD_SIZE (default 1024 = 1032 bytes)
 */

#define RTLUDP_HEADER_SIZE      8
#define RTLUDP_DEFAULT_PORT     1235
#define RTLUDP_DEFAULT_PAYLOAD  1024

/* UDP header structure */
typedef struct __attribute__((packed)) {
    uint32_t seq;           /* Packet sequence number */
    uint32_t timestamp_us;  /* Microseconds since stream start */
} rtludp_header_t;

/* Server configuration */
typedef struct {
    uint16_t        port;           /* UDP port (default 1235) */
    rtlsdr_dev_t   *dev;           /* RTL-SDR device handle */
    uint32_t        payload_size;   /* IQ bytes per packet (default 1024) */
} rtludp_config_t;

#define RTLUDP_CONFIG_DEFAULT() { \
    .port = RTLUDP_DEFAULT_PORT, \
    .dev = NULL, \
    .payload_size = RTLUDP_DEFAULT_PAYLOAD, \
}

/* Server handle */
typedef struct rtludp_server rtludp_server_t;

/*
 * Start the UDP IQ streaming server.
 * Listens for a "subscribe" packet from a client, then streams IQ data.
 * Also accepts 5-byte rtl_tcp commands on the same port.
 */
esp_err_t rtludp_server_start(rtludp_server_t **server, const rtludp_config_t *config);

/*
 * Stop the UDP server.
 */
esp_err_t rtludp_server_stop(rtludp_server_t *server);

/*
 * Push IQ samples for UDP transmission (called from USB bulk callback).
 * Packetizes data with sequence numbers and sends immediately.
 * Returns number of bytes accepted.
 */
uint32_t rtludp_push_samples(rtludp_server_t *server, const uint8_t *data, uint32_t len);

/*
 * Check if a UDP client is subscribed.
 */
bool rtludp_is_client_active(rtludp_server_t *server);

#ifdef __cplusplus
}
#endif
