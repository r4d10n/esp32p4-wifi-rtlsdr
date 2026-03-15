/*
 * RTL-TCP Server for ESP32-P4
 *
 * Streams IQ samples from RTL-SDR over TCP using the rtl_tcp protocol.
 * Accepts tuning/gain/sample-rate commands from remote SDR clients.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_check.h"
#include "rtltcp.h"

static const char *TAG = "rtltcp";

/* Ring buffer and server state */
struct rtltcp_server {
    rtlsdr_dev_t       *dev;
    uint16_t            port;
    RingbufHandle_t     ring_buf;
    uint32_t            ring_size;
    int                 listen_fd;
    int                 client_fd;
    volatile bool       running;
    volatile bool       client_connected;
    TaskHandle_t        listener_task;
    TaskHandle_t        sender_task;
    TaskHandle_t        cmd_task;
};

/* ──────────────────────── DongleInfo Header ──────────────────────── */

static void rtltcp_send_dongle_info(rtltcp_server_t *srv)
{
    rtltcp_dongle_info_t info;
    int gain_count = 0;
    rtlsdr_get_tuner_gains(srv->dev, &gain_count);

    info.magic      = htonl(0x52544C30);  /* "RTL0" */
    info.tuner_type = htonl((uint32_t)rtlsdr_get_tuner_type(srv->dev));
    info.gain_count = htonl((uint32_t)gain_count);

    send(srv->client_fd, &info, sizeof(info), 0);
    ESP_LOGI(TAG, "Sent DongleInfo (tuner=%d gains=%d)",
             rtlsdr_get_tuner_type(srv->dev), gain_count);
}

/* ──────────────────────── Command Handler ──────────────────────── */

static void rtltcp_handle_cmd(rtltcp_server_t *srv, uint8_t cmd, uint32_t param)
{
    ESP_LOGI(TAG, "CMD: 0x%02x param=%lu", cmd, (unsigned long)param);

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
        case RTLTCP_CMD_SET_IF_GAIN:
            rtlsdr_set_if_gain(srv->dev, (param >> 16) & 0xFFFF, param & 0xFFFF);
            break;
        case RTLTCP_CMD_SET_TEST_MODE:
            rtlsdr_set_test_mode(srv->dev, param != 0);
            break;
        case RTLTCP_CMD_SET_AGC_MODE:
            rtlsdr_set_agc_mode(srv->dev, param != 0);
            break;
        case RTLTCP_CMD_SET_DIRECT_SAMP:
            rtlsdr_set_direct_sampling(srv->dev, param);
            break;
        case RTLTCP_CMD_SET_OFFSET_TUNE:
            rtlsdr_set_offset_tuning(srv->dev, param != 0);
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
            ESP_LOGW(TAG, "Unknown command: 0x%02x", cmd);
            break;
    }
}

/* ──────────────────────── Command Receiver Task ──────────────────────── */

static void cmd_receiver_task(void *arg)
{
    rtltcp_server_t *srv = (rtltcp_server_t *)arg;
    rtltcp_cmd_t cmd_pkt;

    while (srv->running && srv->client_connected) {
        /* Read 5-byte command packet */
        int total = 0;
        uint8_t *buf = (uint8_t *)&cmd_pkt;

        while (total < sizeof(cmd_pkt) && srv->client_connected) {
            int n = recv(srv->client_fd, buf + total, sizeof(cmd_pkt) - total, 0);
            if (n <= 0) {
                if (n == 0) {
                    ESP_LOGI(TAG, "Client disconnected (cmd)");
                } else {
                    ESP_LOGW(TAG, "Cmd recv error: %d", errno);
                }
                srv->client_connected = false;
                break;
            }
            total += n;
        }

        if (total == sizeof(cmd_pkt)) {
            uint32_t param = ntohl(cmd_pkt.param);
            rtltcp_handle_cmd(srv, cmd_pkt.cmd, param);
        }
    }

    ESP_LOGI(TAG, "Command receiver stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── IQ Sender Task ──────────────────────── */

static void iq_sender_task(void *arg)
{
    rtltcp_server_t *srv = (rtltcp_server_t *)arg;

    while (srv->running && srv->client_connected) {
        size_t item_size = 0;
        uint8_t *data = xRingbufferReceiveUpTo(srv->ring_buf, &item_size,
                                                pdMS_TO_TICKS(5), 32768);
        if (data && item_size > 0) {
            int sent = 0;
            while (sent < (int)item_size && srv->client_connected) {
                int n = send(srv->client_fd, data + sent, item_size - sent, MSG_NOSIGNAL);
                if (n <= 0) {
                    if (n == 0 || errno == EPIPE || errno == ECONNRESET) {
                        ESP_LOGI(TAG, "Client disconnected (send)");
                    } else {
                        ESP_LOGW(TAG, "Send error: %d", errno);
                    }
                    srv->client_connected = false;
                    break;
                }
                sent += n;
            }
            vRingbufferReturnItem(srv->ring_buf, data);
        }
    }

    ESP_LOGI(TAG, "IQ sender stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── TCP Listener Task ──────────────────────── */

static void listener_task(void *arg)
{
    rtltcp_server_t *srv = (rtltcp_server_t *)arg;

    /* Create listening socket */
    srv->listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv->listen_fd < 0) {
        ESP_LOGE(TAG, "Socket create failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(srv->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(srv->port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(srv->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Bind failed: %d", errno);
        close(srv->listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(srv->listen_fd, 1) < 0) {
        ESP_LOGE(TAG, "Listen failed: %d", errno);
        close(srv->listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "RTL-TCP server listening on port %d", srv->port);

    while (srv->running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        srv->client_fd = accept(srv->listen_fd, (struct sockaddr *)&client_addr, &client_len);
        if (srv->client_fd < 0) {
            if (srv->running) {
                ESP_LOGW(TAG, "Accept failed: %d", errno);
            }
            continue;
        }

        /* Disable TCP_NODELAY — let Nagle batch for throughput */
        opt = 0;
        setsockopt(srv->client_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

        /* Maximize send buffer */
        int sndbuf = 131072;
        setsockopt(srv->client_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        ESP_LOGI(TAG, "Client connected from %s:%d",
                 inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        srv->client_connected = true;

        /* Send dongle info header */
        rtltcp_send_dongle_info(srv);

        /* Start command receiver and IQ sender tasks */
        xTaskCreatePinnedToCore(cmd_receiver_task, "rtltcp_cmd", 4096,
                                srv, 5, &srv->cmd_task, 1);
        xTaskCreatePinnedToCore(iq_sender_task, "rtltcp_iq", 16384,
                                srv, 8, &srv->sender_task, 1);

        /* Wait for client disconnect */
        while (srv->client_connected && srv->running) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* Cleanup client connection */
        srv->client_connected = false;
        close(srv->client_fd);
        srv->client_fd = -1;

        /* Wait for tasks to finish */
        vTaskDelay(pdMS_TO_TICKS(200));

        ESP_LOGI(TAG, "Client session ended, waiting for new connection...");
    }

    close(srv->listen_fd);
    ESP_LOGI(TAG, "Listener stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t rtltcp_server_start(rtltcp_server_t **out_server, const rtltcp_config_t *config)
{
    ESP_RETURN_ON_FALSE(config->dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    rtltcp_server_t *srv = calloc(1, sizeof(rtltcp_server_t));
    ESP_RETURN_ON_FALSE(srv, ESP_ERR_NO_MEM, TAG, "alloc failed");

    srv->dev = config->dev;
    srv->port = config->port;
    srv->ring_size = config->ring_size;
    srv->listen_fd = -1;
    srv->client_fd = -1;
    srv->running = true;

    /* Create ring buffer in PSRAM if available */
    srv->ring_buf = xRingbufferCreate(srv->ring_size, RINGBUF_TYPE_BYTEBUF);
    ESP_RETURN_ON_FALSE(srv->ring_buf, ESP_ERR_NO_MEM, TAG, "ring buffer alloc failed");

    /* Start listener task on Core 1 */
    BaseType_t ret = xTaskCreatePinnedToCore(listener_task, "rtltcp_listen", 4096,
                                              srv, 5, &srv->listener_task, 1);
    if (ret != pdPASS) {
        vRingbufferDelete(srv->ring_buf);
        free(srv);
        return ESP_ERR_NO_MEM;
    }

    *out_server = srv;
    return ESP_OK;
}

esp_err_t rtltcp_server_stop(rtltcp_server_t *srv)
{
    if (!srv) return ESP_ERR_INVALID_ARG;

    srv->running = false;
    srv->client_connected = false;

    /* Close sockets to unblock accept/recv */
    if (srv->client_fd >= 0) close(srv->client_fd);
    if (srv->listen_fd >= 0) close(srv->listen_fd);

    /* Wait for tasks to finish */
    vTaskDelay(pdMS_TO_TICKS(500));

    if (srv->ring_buf) vRingbufferDelete(srv->ring_buf);
    free(srv);
    return ESP_OK;
}

uint32_t rtltcp_push_samples(rtltcp_server_t *srv, const uint8_t *data, uint32_t len)
{
    if (!srv || !srv->client_connected) return 0;

    /* Try to send to ring buffer, drop if full (non-blocking) */
    if (xRingbufferSend(srv->ring_buf, data, len, 0) == pdTRUE) {
        return len;
    }

    /* Buffer full — drop samples */
    return 0;
}

bool rtltcp_is_client_connected(rtltcp_server_t *srv)
{
    return srv ? srv->client_connected : false;
}
