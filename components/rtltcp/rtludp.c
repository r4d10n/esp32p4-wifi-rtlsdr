/*
 * RTL-UDP IQ Streaming Server for ESP32-P4
 *
 * Streams IQ samples over UDP with thin header (seq + timestamp).
 * Accepts rtl_tcp 5-byte commands on the same UDP port.
 * Client subscribes by sending any packet to the port.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "rtludp.h"
#include "rtltcp.h"  /* for command opcodes */

static const char *TAG = "rtludp";

struct rtludp_server {
    rtlsdr_dev_t       *dev;
    uint16_t            port;
    uint32_t            payload_size;
    int                 sock_fd;
    volatile bool       running;
    volatile bool       client_active;
    struct sockaddr_in  client_addr;
    socklen_t           client_addr_len;
    uint32_t            seq;
    int64_t             stream_start_us;
    TaskHandle_t        recv_task;
    /* Pre-allocated send buffer: header + payload */
    uint8_t            *send_buf;
    /* Accumulation buffer for partial bulk callbacks */
    uint8_t            *accum_buf;
    uint32_t            accum_len;
    /* Stats */
    uint32_t            packets_sent;
    uint32_t            bytes_sent;
};

/* ──────────────────────── Command Handler ──────────────────────── */

/* Reuse rtl_tcp command opcodes from rtltcp.h */
extern void rtltcp_handle_cmd_ext(rtlsdr_dev_t *dev, uint8_t cmd, uint32_t param);

static void handle_udp_command(rtludp_server_t *srv, const uint8_t *data, int len)
{
    if (len == 5) {
        /* Standard rtl_tcp 5-byte command */
        uint8_t cmd = data[0];
        uint32_t param = ((uint32_t)data[1] << 24) |
                         ((uint32_t)data[2] << 16) |
                         ((uint32_t)data[3] << 8)  |
                         (uint32_t)data[4];
        ESP_LOGI(TAG, "UDP CMD: 0x%02x param=%lu", cmd, (unsigned long)param);

        /* Apply command to device */
        switch (cmd) {
            case RTLTCP_CMD_SET_FREQ:
                rtlsdr_set_center_freq(srv->dev, param);
                break;
            case RTLTCP_CMD_SET_SAMPLE_RATE:
                rtlsdr_set_sample_rate(srv->dev, param);
                break;
            case RTLTCP_CMD_SET_GAIN_MODE:
                rtlsdr_set_tuner_gain_mode(srv->dev, param);
                break;
            case RTLTCP_CMD_SET_GAIN:
                rtlsdr_set_tuner_gain(srv->dev, param);
                break;
            case RTLTCP_CMD_SET_FREQ_CORR:
                rtlsdr_set_freq_correction(srv->dev, (int)param);
                break;
            case RTLTCP_CMD_SET_AGC_MODE:
                rtlsdr_set_agc_mode(srv->dev, param != 0);
                break;
            case RTLTCP_CMD_SET_BIAS_TEE:
                rtlsdr_set_bias_tee(srv->dev, param != 0);
                break;
            case RTLTCP_CMD_SET_GAIN_INDEX: {
                int count;
                const int *gains = rtlsdr_get_tuner_gains(srv->dev, &count);
                if (gains && (int)param < count) {
                    rtlsdr_set_tuner_gain(srv->dev, gains[param]);
                }
                break;
            }
            default:
                ESP_LOGW(TAG, "Unknown UDP command: 0x%02x", cmd);
                break;
        }
    }
}

/* ──────────────────────── UDP Receive Task ──────────────────────── */

static void udp_recv_task(void *arg)
{
    rtludp_server_t *srv = (rtludp_server_t *)arg;
    uint8_t recv_buf[64];

    while (srv->running) {
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);

        int n = recvfrom(srv->sock_fd, recv_buf, sizeof(recv_buf), 0,
                         (struct sockaddr *)&from_addr, &from_len);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (srv->running) ESP_LOGW(TAG, "recvfrom error: %d", errno);
            continue;
        }

        if (n > 0) {
            /* Any incoming packet subscribes the client */
            if (!srv->client_active ||
                from_addr.sin_addr.s_addr != srv->client_addr.sin_addr.s_addr ||
                from_addr.sin_port != srv->client_addr.sin_port) {
                srv->client_addr = from_addr;
                srv->client_addr_len = from_len;
                srv->client_active = true;
                srv->seq = 0;
                srv->stream_start_us = esp_timer_get_time();
                srv->packets_sent = 0;
                srv->bytes_sent = 0;
                ESP_LOGI(TAG, "UDP client subscribed: %s:%d",
                         inet_ntoa(from_addr.sin_addr), ntohs(from_addr.sin_port));
            }

            /* Process as command if 5 bytes */
            if (n == 5) {
                handle_udp_command(srv, recv_buf, n);
            }
        }
    }

    ESP_LOGI(TAG, "UDP recv task stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t rtludp_server_start(rtludp_server_t **out_server, const rtludp_config_t *config)
{
    ESP_RETURN_ON_FALSE(config->dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    rtludp_server_t *srv = calloc(1, sizeof(rtludp_server_t));
    ESP_RETURN_ON_FALSE(srv, ESP_ERR_NO_MEM, TAG, "alloc failed");

    srv->dev = config->dev;
    srv->port = config->port;
    srv->payload_size = config->payload_size;
    srv->sock_fd = -1;
    srv->running = true;

    /* Allocate send buffer: header + max payload */
    srv->send_buf = malloc(RTLUDP_HEADER_SIZE + srv->payload_size);
    ESP_RETURN_ON_FALSE(srv->send_buf, ESP_ERR_NO_MEM, TAG, "send_buf alloc failed");

    /* Allocate accumulation buffer */
    srv->accum_buf = malloc(srv->payload_size * 2);
    ESP_RETURN_ON_FALSE(srv->accum_buf, ESP_ERR_NO_MEM, TAG, "accum_buf alloc failed");
    srv->accum_len = 0;

    /* Create UDP socket */
    srv->sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (srv->sock_fd < 0) {
        ESP_LOGE(TAG, "Socket create failed: %d", errno);
        free(srv->send_buf);
        free(srv->accum_buf);
        free(srv);
        return ESP_FAIL;
    }

    /* Set recv timeout for non-blocking check */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 }; /* 100ms */
    setsockopt(srv->sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(srv->port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(srv->sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Bind failed: %d", errno);
        close(srv->sock_fd);
        free(srv->send_buf);
        free(srv->accum_buf);
        free(srv);
        return ESP_FAIL;
    }

    /* Start receive task */
    xTaskCreatePinnedToCore(udp_recv_task, "rtludp_recv", 4096,
                            srv, 5, &srv->recv_task, 1);

    ESP_LOGI(TAG, "UDP IQ server listening on port %d (payload=%lu bytes)",
             srv->port, (unsigned long)srv->payload_size);

    *out_server = srv;
    return ESP_OK;
}

esp_err_t rtludp_server_stop(rtludp_server_t *srv)
{
    if (!srv) return ESP_ERR_INVALID_ARG;

    srv->running = false;
    srv->client_active = false;

    if (srv->sock_fd >= 0) close(srv->sock_fd);
    vTaskDelay(pdMS_TO_TICKS(200));

    if (srv->send_buf) free(srv->send_buf);
    if (srv->accum_buf) free(srv->accum_buf);
    free(srv);
    return ESP_OK;
}

uint32_t rtludp_push_samples(rtludp_server_t *srv, const uint8_t *data, uint32_t len)
{
    if (!srv || !srv->client_active) return 0;

    /* Accumulate data */
    uint32_t consumed = 0;
    while (consumed < len) {
        uint32_t space = srv->payload_size - srv->accum_len;
        uint32_t copy = (len - consumed < space) ? (len - consumed) : space;

        memcpy(srv->accum_buf + srv->accum_len, data + consumed, copy);
        srv->accum_len += copy;
        consumed += copy;

        /* Send when we have a full payload */
        if (srv->accum_len >= srv->payload_size) {
            /* Build packet: header + payload */
            rtludp_header_t *hdr = (rtludp_header_t *)srv->send_buf;
            hdr->seq = srv->seq++;
            hdr->timestamp_us = (uint32_t)(esp_timer_get_time() - srv->stream_start_us);

            memcpy(srv->send_buf + RTLUDP_HEADER_SIZE, srv->accum_buf, srv->payload_size);

            int sent = sendto(srv->sock_fd, srv->send_buf,
                              RTLUDP_HEADER_SIZE + srv->payload_size, 0,
                              (struct sockaddr *)&srv->client_addr, srv->client_addr_len);

            if (sent < 0) {
                if (errno == ENOMEM || errno == EAGAIN) {
                    /* Network congestion — drop this packet */
                } else {
                    ESP_LOGW(TAG, "sendto error: %d", errno);
                    srv->client_active = false;
                }
            } else {
                srv->packets_sent++;
                srv->bytes_sent += sent;
            }

            srv->accum_len = 0;
        }
    }

    return consumed;
}

bool rtludp_is_client_active(rtludp_server_t *server)
{
    return server ? server->client_active : false;
}
