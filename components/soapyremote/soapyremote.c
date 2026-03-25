/*
 * SoapySDR Remote Server for ESP32-P4
 *
 * Implements a simplified SoapyRemote TCP RPC server with UDP IQ streaming.
 * Allows SoapySDR clients to discover and control the RTL-SDR via Wi-Fi.
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
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_check.h"
#include "mdns.h"
#include "rtlsdr.h"
#include "soapyremote.h"

static const char *TAG = "soapyremote";

/* ──────────────────────── RPC Command IDs ──────────────────────── */

#define CMD_DISCOVER            0x00
#define CMD_MAKE_DEVICE         0x01
#define CMD_UNMAKE_DEVICE       0x02
#define CMD_GET_HARDWARE_KEY    0x10
#define CMD_GET_HARDWARE_INFO   0x11
#define CMD_SET_FREQUENCY       0x20
#define CMD_GET_FREQUENCY       0x21
#define CMD_SET_SAMPLE_RATE     0x30
#define CMD_GET_SAMPLE_RATE     0x31
#define CMD_SET_GAIN            0x40
#define CMD_GET_GAIN            0x41
#define CMD_LIST_GAINS          0x42
#define CMD_SET_GAIN_MODE       0x43
#define CMD_SETUP_STREAM        0x50
#define CMD_ACTIVATE_STREAM     0x51
#define CMD_DEACTIVATE_STREAM   0x52
#define CMD_CLOSE_STREAM        0x53

/* ──────────────────────── Wire Format ──────────────────────── */

/* RPC message header: cmd (4 LE) + payload_len (4 LE) */
typedef struct __attribute__((packed)) {
    uint32_t cmd;
    uint32_t len;
} rpc_hdr_t;

/* UDP IQ packet: seq (4 LE) + IQ data */
#define UDP_SEQ_SIZE    4
#define IQ_RING_SIZE    (256 * 1024)

/* ──────────────────────── Server State ──────────────────────── */

struct soapy_remote_server {
    rtlsdr_dev_t       *dev;
    uint16_t            port;
    char                format[8];
    uint16_t            mtu;
    bool                announce_mdns;

    RingbufHandle_t     ring_buf;

    int                 listen_fd;
    int                 client_fd;
    struct sockaddr_in  client_addr;    /* TCP client address (used for UDP) */

    volatile bool       running;
    volatile bool       client_connected;
    volatile bool       stream_active;

    int                 udp_fd;
    uint16_t            udp_port;       /* port client wants UDP on */
    uint32_t            udp_seq;

    TaskHandle_t        listener_task;
    TaskHandle_t        udp_sender_task;
};

/* ──────────────────────── Helpers: send/recv full buffers ──────────────────────── */

static int send_all(int fd, const void *buf, size_t len)
{
    size_t sent = 0;
    const uint8_t *p = (const uint8_t *)buf;
    while (sent < len) {
        int n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) return -1;
        sent += n;
    }
    return (int)sent;
}

static int recv_all(int fd, void *buf, size_t len)
{
    size_t got = 0;
    uint8_t *p = (uint8_t *)buf;
    while (got < len) {
        int n = recv(fd, p + got, len - got, 0);
        if (n <= 0) return -1;
        got += n;
    }
    return (int)got;
}

/* ──────────────────────── RPC reply helpers ──────────────────────── */

/* Send a reply header + payload */
static void rpc_reply(int fd, uint32_t cmd, const void *payload, uint32_t payload_len)
{
    rpc_hdr_t hdr;
    hdr.cmd = cmd;          /* echo cmd back */
    hdr.len = payload_len;
    send_all(fd, &hdr, sizeof(hdr));
    if (payload && payload_len > 0) {
        send_all(fd, payload, payload_len);
    }
}

/* Reply with a NUL-terminated string (length includes NUL) */
static void rpc_reply_str(int fd, uint32_t cmd, const char *str)
{
    rpc_reply(fd, cmd, str, (uint32_t)(strlen(str) + 1));
}

/* Reply with a double value (8 bytes, native byte order matches SoapyRemote) */
static void rpc_reply_double(int fd, uint32_t cmd, double val)
{
    rpc_reply(fd, cmd, &val, sizeof(double));
}

/* Reply with a single uint32 status (0 = OK) */
static void rpc_reply_ok(int fd, uint32_t cmd)
{
    uint32_t status = 0;
    rpc_reply(fd, cmd, &status, sizeof(status));
}

/* ──────────────────────── RPC Command Handlers ──────────────────────── */

static void handle_discover(struct soapy_remote_server *srv, int fd)
{
    static const char *info =
        "{\"driver\":\"rtlsdr\",\"label\":\"ESP32-P4 RTL-SDR\",\"serial\":\"esp32p4\"}";
    rpc_reply_str(fd, CMD_DISCOVER, info);
}

static void handle_make_device(struct soapy_remote_server *srv, int fd)
{
    rpc_reply_ok(fd, CMD_MAKE_DEVICE);
}

static void handle_unmake_device(struct soapy_remote_server *srv, int fd)
{
    rpc_reply_ok(fd, CMD_UNMAKE_DEVICE);
}

static void handle_get_hardware_key(struct soapy_remote_server *srv, int fd)
{
    rpc_reply_str(fd, CMD_GET_HARDWARE_KEY, "RTL-SDR");
}

static void handle_get_hardware_info(struct soapy_remote_server *srv, int fd)
{
    /* Key-value pairs encoded as "key=value\nkey=value\n" */
    static const char *info = "driver=rtlsdr\norigin=ESP32-P4\n";
    rpc_reply_str(fd, CMD_GET_HARDWARE_INFO, info);
}

static void handle_set_frequency(struct soapy_remote_server *srv, int fd,
                                  const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(double)) { rpc_reply_ok(fd, CMD_SET_FREQUENCY); return; }
    double freq;
    memcpy(&freq, payload, sizeof(double));
    ESP_LOGI(TAG, "SET_FREQUENCY: %.0f Hz", freq);
    if (srv->dev) rtlsdr_set_center_freq(srv->dev, (uint32_t)freq);
    rpc_reply_ok(fd, CMD_SET_FREQUENCY);
}

static void handle_get_frequency(struct soapy_remote_server *srv, int fd)
{
    double freq = srv->dev ? (double)rtlsdr_get_center_freq(srv->dev) : 0.0;
    rpc_reply_double(fd, CMD_GET_FREQUENCY, freq);
}

static void handle_set_sample_rate(struct soapy_remote_server *srv, int fd,
                                    const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(double)) { rpc_reply_ok(fd, CMD_SET_SAMPLE_RATE); return; }
    double rate;
    memcpy(&rate, payload, sizeof(double));
    ESP_LOGI(TAG, "SET_SAMPLE_RATE: %.0f", rate);
    if (srv->dev) rtlsdr_set_sample_rate(srv->dev, (uint32_t)rate);
    rpc_reply_ok(fd, CMD_SET_SAMPLE_RATE);
}

static void handle_get_sample_rate(struct soapy_remote_server *srv, int fd)
{
    double rate = srv->dev ? (double)rtlsdr_get_sample_rate(srv->dev) : 0.0;
    rpc_reply_double(fd, CMD_GET_SAMPLE_RATE, rate);
}

static void handle_set_gain(struct soapy_remote_server *srv, int fd,
                             const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(double)) { rpc_reply_ok(fd, CMD_SET_GAIN); return; }
    double gain_db;
    memcpy(&gain_db, payload, sizeof(double));
    ESP_LOGI(TAG, "SET_GAIN: %.1f dB", gain_db);
    /* rtlsdr gain is in tenths of dB */
    if (srv->dev) rtlsdr_set_tuner_gain(srv->dev, (int)(gain_db * 10.0));
    rpc_reply_ok(fd, CMD_SET_GAIN);
}

static void handle_get_gain(struct soapy_remote_server *srv, int fd)
{
    double gain = srv->dev ? (double)rtlsdr_get_tuner_gain(srv->dev) / 10.0 : 0.0;
    rpc_reply_double(fd, CMD_GET_GAIN, gain);
}

static void handle_list_gains(struct soapy_remote_server *srv, int fd)
{
    /* Return newline-separated list of gain element names */
    rpc_reply_str(fd, CMD_LIST_GAINS, "TUNER\n");
}

static void handle_set_gain_mode(struct soapy_remote_server *srv, int fd,
                                  const uint8_t *payload, uint32_t len)
{
    if (len < 4) { rpc_reply_ok(fd, CMD_SET_GAIN_MODE); return; }
    uint32_t mode;
    memcpy(&mode, payload, sizeof(uint32_t));
    /* mode: 0=auto, 1=manual — rtlsdr manual gain mode: 1=manual, 0=auto */
    ESP_LOGI(TAG, "SET_GAIN_MODE: %s", mode ? "manual" : "auto");
    if (srv->dev) rtlsdr_set_tuner_gain_mode(srv->dev, (int)mode);
    rpc_reply_ok(fd, CMD_SET_GAIN_MODE);
}

/*
 * SETUP_STREAM: Client tells us which UDP port to send IQ data to.
 * If client doesn't specify, use a default (client_tcp_port + 1).
 * Reply with the port we'll send to, so client can bind and listen.
 */
static void handle_setup_stream(struct soapy_remote_server *srv, int fd,
                                 const uint8_t *payload, uint32_t len)
{
    if (len >= 2) {
        uint16_t udp_port;
        memcpy(&udp_port, payload, sizeof(uint16_t));
        if (udp_port > 0) srv->udp_port = udp_port;
    }
    /* If client didn't specify a port, use default data port */
    if (srv->udp_port == 0) {
        /* Use client's source port + 1 as default data port */
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        getpeername(fd, (struct sockaddr *)&peer, &plen);
        srv->udp_port = ntohs(peer.sin_port) + 1;
    }
    ESP_LOGI(TAG, "SETUP_STREAM: UDP data port=%u", srv->udp_port);
    /* Return the UDP port so client knows where to listen */
    uint32_t resp_port = srv->udp_port;
    rpc_reply(fd, CMD_SETUP_STREAM, &resp_port, sizeof(resp_port));
}

/* ──────────────────────── UDP Sender Task ──────────────────────── */

static void udp_sender_task(void *arg)
{
    struct soapy_remote_server *srv = (struct soapy_remote_server *)arg;

    /* Open UDP socket */
    srv->udp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (srv->udp_fd < 0) {
        ESP_LOGE(TAG, "UDP socket failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(srv->udp_port),
        .sin_addr   = srv->client_addr.sin_addr,
    };

    /* Max payload per UDP packet */
    uint32_t max_payload = srv->mtu - UDP_SEQ_SIZE - 28; /* 28 = IP+UDP headers */
    if (max_payload < 64) max_payload = 64;

    static uint8_t pkt_buf[1500];
    srv->udp_seq = 0;

    ESP_LOGI(TAG, "UDP sender started -> %s:%u mtu=%u",
             inet_ntoa(dest.sin_addr), srv->udp_port, srv->mtu);

    while (srv->stream_active && srv->running) {
        size_t item_size = 0;
        uint8_t *data = xRingbufferReceiveUpTo(srv->ring_buf, &item_size,
                                                pdMS_TO_TICKS(5), max_payload);
        if (data && item_size > 0) {
            /* Build packet: seq + IQ data */
            uint32_t seq = srv->udp_seq++;
            memcpy(pkt_buf, &seq, UDP_SEQ_SIZE);
            memcpy(pkt_buf + UDP_SEQ_SIZE, data, item_size);

            sendto(srv->udp_fd, pkt_buf, UDP_SEQ_SIZE + item_size, 0,
                   (struct sockaddr *)&dest, sizeof(dest));

            vRingbufferReturnItem(srv->ring_buf, data);
        }
    }

    close(srv->udp_fd);
    srv->udp_fd = -1;
    ESP_LOGI(TAG, "UDP sender stopped");
    vTaskDelete(NULL);
}

static void handle_activate_stream(struct soapy_remote_server *srv, int fd)
{
    if (!srv->stream_active) {
        srv->stream_active = true;
        xTaskCreatePinnedToCore(udp_sender_task, "soapy_udp", 4096,
                                srv, 6, &srv->udp_sender_task, 1);
        ESP_LOGI(TAG, "Stream activated");
    }
    rpc_reply_ok(fd, CMD_ACTIVATE_STREAM);
}

static void handle_deactivate_stream(struct soapy_remote_server *srv, int fd)
{
    srv->stream_active = false;
    /* Give UDP task time to exit */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Stream deactivated");
    rpc_reply_ok(fd, CMD_DEACTIVATE_STREAM);
}

static void handle_close_stream(struct soapy_remote_server *srv, int fd)
{
    srv->stream_active = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    rpc_reply_ok(fd, CMD_CLOSE_STREAM);
}

/* ──────────────────────── RPC Dispatch ──────────────────────── */

static void dispatch_rpc(struct soapy_remote_server *srv, int fd,
                          uint32_t cmd, const uint8_t *payload, uint32_t len)
{
    ESP_LOGD(TAG, "RPC cmd=0x%02lx len=%lu", (unsigned long)cmd, (unsigned long)len);

    switch (cmd) {
        case CMD_DISCOVER:          handle_discover(srv, fd); break;
        case CMD_MAKE_DEVICE:       handle_make_device(srv, fd); break;
        case CMD_UNMAKE_DEVICE:     handle_unmake_device(srv, fd); break;
        case CMD_GET_HARDWARE_KEY:  handle_get_hardware_key(srv, fd); break;
        case CMD_GET_HARDWARE_INFO: handle_get_hardware_info(srv, fd); break;
        case CMD_SET_FREQUENCY:     handle_set_frequency(srv, fd, payload, len); break;
        case CMD_GET_FREQUENCY:     handle_get_frequency(srv, fd); break;
        case CMD_SET_SAMPLE_RATE:   handle_set_sample_rate(srv, fd, payload, len); break;
        case CMD_GET_SAMPLE_RATE:   handle_get_sample_rate(srv, fd); break;
        case CMD_SET_GAIN:          handle_set_gain(srv, fd, payload, len); break;
        case CMD_GET_GAIN:          handle_get_gain(srv, fd); break;
        case CMD_LIST_GAINS:        handle_list_gains(srv, fd); break;
        case CMD_SET_GAIN_MODE:     handle_set_gain_mode(srv, fd, payload, len); break;
        case CMD_SETUP_STREAM:      handle_setup_stream(srv, fd, payload, len); break;
        case CMD_ACTIVATE_STREAM:   handle_activate_stream(srv, fd); break;
        case CMD_DEACTIVATE_STREAM: handle_deactivate_stream(srv, fd); break;
        case CMD_CLOSE_STREAM:      handle_close_stream(srv, fd); break;
        default:
            ESP_LOGW(TAG, "Unknown RPC cmd: 0x%02lx", (unsigned long)cmd);
            rpc_reply_ok(fd, cmd);
            break;
    }
}

/* ──────────────────────── Client Session ──────────────────────── */

static void handle_client(struct soapy_remote_server *srv, int fd)
{
    /* Max RPC payload we'll buffer on stack */
    static uint8_t payload_buf[256];

    while (srv->running && srv->client_connected) {
        rpc_hdr_t hdr;
        if (recv_all(fd, &hdr, sizeof(hdr)) < 0) {
            ESP_LOGI(TAG, "Client disconnected or recv error");
            break;
        }

        uint32_t cmd = hdr.cmd;
        uint32_t len = hdr.len;

        const uint8_t *payload = NULL;
        if (len > 0) {
            if (len <= sizeof(payload_buf)) {
                if (recv_all(fd, payload_buf, len) < 0) {
                    ESP_LOGW(TAG, "Payload recv error");
                    break;
                }
                payload = payload_buf;
            } else {
                /* Drain oversized payload */
                ESP_LOGW(TAG, "Oversized payload %lu for cmd 0x%02lx — draining",
                         (unsigned long)len, (unsigned long)cmd);
                uint8_t drain[64];
                uint32_t remaining = len;
                while (remaining > 0) {
                    uint32_t chunk = remaining < sizeof(drain) ? remaining : sizeof(drain);
                    if (recv_all(fd, drain, chunk) < 0) goto disconnect;
                    remaining -= chunk;
                }
            }
        }

        dispatch_rpc(srv, fd, cmd, payload, len);
    }

disconnect:
    srv->client_connected = false;
    srv->stream_active = false;
}

/* ──────────────────────── TCP Listener Task ──────────────────────── */

static void listener_task(void *arg)
{
    struct soapy_remote_server *srv = (struct soapy_remote_server *)arg;

    srv->listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv->listen_fd < 0) {
        ESP_LOGE(TAG, "Socket create failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(srv->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(srv->port),
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

    ESP_LOGI(TAG, "SoapyRemote listening on TCP port %d", srv->port);

    while (srv->running) {
        socklen_t client_len = sizeof(srv->client_addr);
        srv->client_fd = accept(srv->listen_fd,
                                (struct sockaddr *)&srv->client_addr, &client_len);
        if (srv->client_fd < 0) {
            if (srv->running) ESP_LOGW(TAG, "Accept failed: %d", errno);
            continue;
        }

        ESP_LOGI(TAG, "Client connected from %s:%d",
                 inet_ntoa(srv->client_addr.sin_addr),
                 ntohs(srv->client_addr.sin_port));

        srv->client_connected = true;
        handle_client(srv, srv->client_fd);

        srv->client_connected = false;
        srv->stream_active = false;
        close(srv->client_fd);
        srv->client_fd = -1;

        ESP_LOGI(TAG, "Client session ended");
    }

    close(srv->listen_fd);
    srv->listen_fd = -1;
    ESP_LOGI(TAG, "Listener stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t soapyremote_start(soapyremote_handle_t *handle,
                             const soapyremote_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle && config, ESP_ERR_INVALID_ARG, TAG, "NULL arg");

    struct soapy_remote_server *srv = calloc(1, sizeof(struct soapy_remote_server));
    ESP_RETURN_ON_FALSE(srv, ESP_ERR_NO_MEM, TAG, "alloc failed");

    srv->dev           = (rtlsdr_dev_t *)config->rtlsdr_dev;
    srv->port          = config->port;
    srv->mtu           = config->mtu ? config->mtu : 1500;
    srv->announce_mdns = config->announce_mdns;
    srv->listen_fd     = -1;
    srv->client_fd     = -1;
    srv->udp_fd        = -1;
    srv->udp_port      = 55133;   /* default UDP port if not set by client */
    srv->running       = true;

    strncpy(srv->format, config->format[0] ? config->format : "CS8",
            sizeof(srv->format) - 1);

    /* Allocate IQ ring buffer (PSRAM via heap caps if available) */
    srv->ring_buf = xRingbufferCreate(IQ_RING_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!srv->ring_buf) {
        free(srv);
        ESP_LOGE(TAG, "Ring buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* Register mDNS service */
    if (srv->announce_mdns) {
        esp_err_t err = mdns_service_add(NULL, "_soapysdr", "_tcp",
                                         config->port, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "mDNS registration failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "mDNS: _soapysdr._tcp port %d registered", config->port);
        }
    }

    /* Start listener task on Core 1, priority 5 */
    BaseType_t ret = xTaskCreatePinnedToCore(listener_task, "soapy_listen", 8192,
                                              srv, 5, &srv->listener_task, 1);
    if (ret != pdPASS) {
        vRingbufferDelete(srv->ring_buf);
        free(srv);
        return ESP_ERR_NO_MEM;
    }

    *handle = srv;
    ESP_LOGI(TAG, "SoapyRemote server started (port=%d fmt=%s mtu=%u)",
             srv->port, srv->format, srv->mtu);
    return ESP_OK;
}

esp_err_t soapyremote_stop(soapyremote_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    struct soapy_remote_server *srv = handle;

    srv->running       = false;
    srv->client_connected = false;
    srv->stream_active = false;

    /* Close sockets to unblock blocking calls */
    if (srv->client_fd >= 0) { close(srv->client_fd); srv->client_fd = -1; }
    if (srv->listen_fd >= 0) { close(srv->listen_fd); srv->listen_fd = -1; }
    if (srv->udp_fd    >= 0) { close(srv->udp_fd);    srv->udp_fd    = -1; }

    /* Allow tasks to exit */
    vTaskDelay(pdMS_TO_TICKS(500));

    if (srv->announce_mdns) {
        mdns_service_remove("_soapysdr", "_tcp");
    }

    if (srv->ring_buf) vRingbufferDelete(srv->ring_buf);
    free(srv);
    return ESP_OK;
}

void soapyremote_push_samples(soapyremote_handle_t handle,
                               const uint8_t *data, uint32_t len)
{
    if (!handle || !data || len == 0) return;
    struct soapy_remote_server *srv = handle;

    if (!srv->stream_active) return;

    /* Non-blocking push: drop if ring buffer is full */
    if (xRingbufferSend(srv->ring_buf, data, len, 0) != pdTRUE) {
        /* Drop samples — buffer overrun */
    }
}
