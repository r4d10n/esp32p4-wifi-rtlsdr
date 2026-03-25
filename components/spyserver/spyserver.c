#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "esp_err.h"
#include "spyserver.h"
#include "spyserver_protocol.h"
#include "rtlsdr.h"

static const char *TAG = "spyserver";

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */
/* protocol_id in message header: ASCII "SPYS" (4 bytes little-endian = 0x53595053) */
#define SPYSERVER_PROTOCOL_ID         0x53595053U   /* 'S','P','Y','S' */

#define RTL_MIN_FREQ_HZ               24000000U
#define RTL_MAX_FREQ_HZ               1700000000U
#define RTL_MAX_SAMPLE_RATE           3200000U
#define RTL_MAX_BANDWIDTH             RTL_MAX_SAMPLE_RATE
#define RTL_DECIMATION_STAGES         8
#define RTL_GAIN_STAGES               1
#define RTL_MAX_GAIN_INDEX            28
#define RTL_RESOLUTION                8

#define IQ_RINGBUF_SIZE               (64 * 1024)
#define FFT_SIZE                      1024
#define LISTENER_TASK_PRIORITY        5
#define LISTENER_TASK_CORE            1
#define CLIENT_TASK_PRIORITY          4
#define CLIENT_TASK_STACK              4096
#define LISTENER_TASK_STACK            4096

/* -------------------------------------------------------------------------
 * Per-client state
 * ---------------------------------------------------------------------- */
typedef struct {
    int                  sock;
    bool                 active;
    bool                 streaming_iq;
    bool                 streaming_fft;
    bool                 can_control;
    uint32_t             gain;
    uint32_t             iq_frequency;
    uint32_t             fft_frequency;
    uint32_t             iq_format;
    uint32_t             fft_format;
    uint32_t             fft_display_pixels;
    uint32_t             fft_db_offset;
    uint32_t             fft_db_range;
    uint32_t             decimation;
    uint32_t             seq_iq;
    uint32_t             seq_fft;
    RingbufHandle_t      iq_ringbuf;
    TaskHandle_t         sender_task;
    SemaphoreHandle_t    send_mutex;
    struct spyserver_server *server;
} spyserver_client_t;

/* -------------------------------------------------------------------------
 * Server state
 * ---------------------------------------------------------------------- */
struct spyserver_server {
    spyserver_config_t   config;
    int                  listen_sock;
    bool                 running;
    TaskHandle_t         listener_task;
    SemaphoreHandle_t    clients_mutex;
    spyserver_client_t  *clients;       /* array of max_clients */
    uint32_t             device_serial;
};

/* -------------------------------------------------------------------------
 * Helpers: send a framed message
 * ---------------------------------------------------------------------- */
static int send_message(spyserver_client_t *client,
                        spyserver_msg_type_t msg_type,
                        spyserver_stream_type_t stream_type,
                        const void *body, uint32_t body_size)
{
    spyserver_msg_header_t hdr = {
        .protocol_id    = SPYSERVER_PROTOCOL_ID,
        .msg_type       = (uint32_t)msg_type,
        .stream_type    = (uint32_t)stream_type,
        .sequence_number = 0,   /* filled below per stream */
        .body_size      = body_size,
    };

    /* assign sequence number per stream type */
    switch (stream_type) {
        case STREAM_TYPE_IQ:  hdr.sequence_number = client->seq_iq++;  break;
        case STREAM_TYPE_FFT: hdr.sequence_number = client->seq_fft++; break;
        default:              hdr.sequence_number = 0;                  break;
    }

    xSemaphoreTake(client->send_mutex, portMAX_DELAY);

    int ret = send(client->sock, &hdr, sizeof(hdr), 0);
    if (ret < 0) {
        xSemaphoreGive(client->send_mutex);
        return ret;
    }
    if (body && body_size > 0) {
        ret = send(client->sock, body, body_size, 0);
    }

    xSemaphoreGive(client->send_mutex);
    return ret;
}

/* -------------------------------------------------------------------------
 * Send DeviceInfo
 * ---------------------------------------------------------------------- */
static void send_device_info(spyserver_client_t *client)
{
    /* Report actual device sample rate (not max theoretical).
     * DecimationStageCount=0 tells SDR++ we do NO server-side decimation.
     * MinimumIQDecimation=0 means SDR++ uses the rate as-is.
     * This prevents SDR++ from expecting decimated rates we can't provide. */
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)client->server->config.rtlsdr_dev;
    uint32_t actual_rate = dev ? rtlsdr_get_sample_rate(dev) : RTL_MAX_SAMPLE_RATE;

    spyserver_device_info_t info = {
        .device_type          = DEVICE_RTLSDR,
        .device_serial        = client->server->device_serial,
        .max_sample_rate      = actual_rate,
        .max_bandwidth        = actual_rate,
        .decimation_stage_count = 0,    /* no server-side decimation */
        .gain_stage_count     = RTL_GAIN_STAGES,
        .max_gain_index       = RTL_MAX_GAIN_INDEX,
        .min_frequency        = RTL_MIN_FREQ_HZ,
        .max_frequency        = RTL_MAX_FREQ_HZ,
        .resolution           = RTL_RESOLUTION,
        .min_iq_decimation    = 0,
        .forced_iq_format     = STREAM_FORMAT_UINT8,
    };
    send_message(client, MSG_TYPE_DEVICE_INFO, STREAM_TYPE_STATUS,
                 &info, sizeof(info));
}

/* -------------------------------------------------------------------------
 * Send ClientSync
 * ---------------------------------------------------------------------- */
static void send_client_sync(spyserver_client_t *client)
{
    spyserver_client_sync_t sync = {
        .can_control              = client->can_control ? 1u : 0u,
        .gain                     = client->gain,
        .device_center_frequency  = client->iq_frequency,
        .iq_center_frequency      = client->iq_frequency,
        .fft_center_frequency     = client->fft_frequency,
        .min_iq_center_frequency  = RTL_MIN_FREQ_HZ,
        .max_iq_center_frequency  = RTL_MAX_FREQ_HZ,
        .min_fft_center_frequency = RTL_MIN_FREQ_HZ,
        .max_fft_center_frequency = RTL_MAX_FREQ_HZ,
    };
    send_message(client, MSG_TYPE_CLIENT_SYNC, STREAM_TYPE_STATUS,
                 &sync, sizeof(sync));
}

/* -------------------------------------------------------------------------
 * Handle CMD_SET_SETTING
 * ---------------------------------------------------------------------- */
static void handle_set_setting(spyserver_client_t *client,
                                uint32_t setting_type, uint32_t value)
{
    if (!client->can_control) {
        ESP_LOGW(TAG, "client %d attempted control but not allowed", client->sock);
        return;
    }

    switch ((spyserver_setting_type_t)setting_type) {
        case SETTING_STREAMING_MODE:
            /* mode bitmask: bit0=IQ, bit2=FFT */
            client->streaming_iq  = (value & (uint32_t)STREAM_TYPE_IQ)  != 0;
            client->streaming_fft = (value & (uint32_t)STREAM_TYPE_FFT) != 0;
            ESP_LOGI(TAG, "client %d stream mode iq=%d fft=%d",
                     client->sock, client->streaming_iq, client->streaming_fft);
            break;
        case SETTING_STREAMING_ENABLED:
            /* value != 0 keeps current mode, 0 disables all */
            if (value == 0) {
                client->streaming_iq  = false;
                client->streaming_fft = false;
            }
            ESP_LOGI(TAG, "client %d streaming_enabled=%"PRIu32, client->sock, value);
            break;
        case SETTING_GAIN:
            client->gain = value;
            {
                rtlsdr_dev_t *dev = (rtlsdr_dev_t *)client->server->config.rtlsdr_dev;
                if (dev) {
                    /* value is a gain INDEX (0..MaxGainIndex), not tenths of dB.
                     * Look up actual gain from the tuner's gain table. */
                    int count = 0;
                    const int *gains = rtlsdr_get_tuner_gains(dev, &count);
                    int gain_tenth_db = 0;
                    if (gains && (int)value < count) {
                        gain_tenth_db = gains[value];
                    }
                    rtlsdr_set_tuner_gain_mode(dev, 1);
                    rtlsdr_set_tuner_gain(dev, gain_tenth_db);
                    ESP_LOGI(TAG, "client %d gain idx=%"PRIu32" -> %d.%d dB",
                             client->sock, value, gain_tenth_db/10, gain_tenth_db%10);
                }
            }
            break;
        case SETTING_IQ_FORMAT:
            client->iq_format = value;
            break;
        case SETTING_IQ_FREQUENCY:
            client->iq_frequency = value;
            client->fft_frequency = value;
            ESP_LOGI(TAG, "client %d freq=%"PRIu32, client->sock, value);
            {
                rtlsdr_dev_t *dev = (rtlsdr_dev_t *)client->server->config.rtlsdr_dev;
                if (dev) rtlsdr_set_center_freq(dev, value);
            }
            break;
        case SETTING_IQ_DECIMATION:
            /* decimation acknowledged but not applied at hardware layer */
            break;
        case SETTING_IQ_DIGITAL_GAIN:
            break;
        case SETTING_FFT_FORMAT:
            client->fft_format = value;
            break;
        case SETTING_FFT_FREQUENCY:
            client->fft_frequency = value;
            break;
        case SETTING_FFT_DECIMATION:
            break;
        case SETTING_FFT_DB_OFFSET:
            client->fft_db_offset = value;
            break;
        case SETTING_FFT_DB_RANGE:
            client->fft_db_range = value;
            break;
        case SETTING_FFT_DISPLAY_PIXELS:
            client->fft_display_pixels = value;
            break;
        default:
            ESP_LOGW(TAG, "unknown setting %"PRIu32, setting_type);
            break;
    }

    send_client_sync(client);
}

/* -------------------------------------------------------------------------
 * recv_all: blocking receive of exactly len bytes
 * ---------------------------------------------------------------------- */
static int recv_all(int sock, void *buf, size_t len)
{
    size_t received = 0;
    while (received < len) {
        int r = recv(sock, (uint8_t *)buf + received, len - received, 0);
        if (r <= 0) return r;
        received += (size_t)r;
    }
    return (int)received;
}

/* -------------------------------------------------------------------------
 * Command dispatch loop (runs in client sender task)
 * ---------------------------------------------------------------------- */
static void process_commands(spyserver_client_t *client)
{
    spyserver_cmd_header_t cmd_hdr;
    uint8_t body[SPYSERVER_MAX_COMMAND_BODY];

    while (client->active) {
        /* read 8-byte command header */
        int r = recv_all(client->sock, &cmd_hdr, sizeof(cmd_hdr));
        if (r <= 0) {
            ESP_LOGI(TAG, "client %d disconnected (recv=%d)", client->sock, r);
            break;
        }

        if (cmd_hdr.body_size > SPYSERVER_MAX_COMMAND_BODY) {
            ESP_LOGW(TAG, "client %d cmd body too large (%"PRIu32")", client->sock, cmd_hdr.body_size);
            break;
        }

        if (cmd_hdr.body_size > 0) {
            r = recv_all(client->sock, body, cmd_hdr.body_size);
            if (r <= 0) break;
        }

        switch ((spyserver_cmd_type_t)cmd_hdr.cmd_type) {
            case CMD_HELLO: {
                /* body contains client's protocol version (4 bytes) and optional name */
                uint32_t client_proto = 0;
                if (cmd_hdr.body_size >= 4) {
                    memcpy(&client_proto, body, 4);
                }
                ESP_LOGI(TAG, "client %d HELLO proto=0x%08"PRIx32, client->sock, client_proto);
                /* Must send DeviceInfo first, then ClientSync — SDR++ blocks on DeviceInfo */
                send_device_info(client);
                send_client_sync(client);
                break;
            }
            case CMD_GET_SETTING:
                /* not commonly used; send sync as acknowledgement */
                send_client_sync(client);
                break;
            case CMD_SET_SETTING: {
                if (cmd_hdr.body_size < 8) {
                    ESP_LOGW(TAG, "CMD_SET_SETTING body too short");
                    break;
                }
                uint32_t setting_type, value;
                memcpy(&setting_type, body,     4);
                memcpy(&value,        body + 4, 4);
                handle_set_setting(client, setting_type, value);
                break;
            }
            case CMD_PING: {
                uint32_t cookie = 0;
                if (cmd_hdr.body_size >= 4) memcpy(&cookie, body, 4);
                send_message(client, MSG_TYPE_PONG, STREAM_TYPE_STATUS, &cookie, 4);
                break;
            }
            default:
                ESP_LOGW(TAG, "unknown cmd %"PRIu32, cmd_hdr.cmd_type);
                break;
        }
    }
}

/* -------------------------------------------------------------------------
 * FFT helper: compute power spectrum from uint8 IQ, scale to uint8
 * Simple DFT — replace with kiss_fft or the DSP component if available.
 * ---------------------------------------------------------------------- */
static void compute_fft_uint8(const uint8_t *iq, uint32_t iq_samples,
                               uint8_t *out, uint32_t fft_bins,
                               int32_t db_offset, int32_t db_range)
{
    /* iq is interleaved I/Q uint8 (bias 127).  We compute a decimated power
     * spectrum using a simple rectangular-window DFT over the first
     * min(iq_samples, FFT_SIZE) samples, then interpolate/decimate to
     * fft_bins output pixels. */
    uint32_t n = (iq_samples < FFT_SIZE) ? iq_samples : FFT_SIZE;

    float power[FFT_SIZE] = {0};

    for (uint32_t k = 0; k < n; k++) {
        float re = 0.0f, im = 0.0f;
        for (uint32_t j = 0; j < n; j++) {
            float i_s = (float)iq[2 * j]     - 127.5f;
            float q_s = (float)iq[2 * j + 1] - 127.5f;
            float ang = -2.0f * (float)M_PI * (float)k * (float)j / (float)n;
            re += i_s * cosf(ang) - q_s * sinf(ang);
            im += i_s * sinf(ang) + q_s * cosf(ang);
        }
        power[k] = re * re + im * im;
    }

    /* map power[] -> fft_bins output pixels */
    for (uint32_t b = 0; b < fft_bins; b++) {
        uint32_t src_idx = b * n / fft_bins;
        float db = 10.0f * log10f(power[src_idx] + 1e-10f);
        /* map db_offset .. (db_offset+db_range) -> 0..255 */
        float norm = (db - (float)db_offset) / (float)(db_range ? db_range : 1);
        int32_t val = (int32_t)(norm * 255.0f);
        if (val < 0)   val = 0;
        if (val > 255) val = 255;
        out[b] = (uint8_t)val;
    }
}

/* -------------------------------------------------------------------------
 * Per-client sender task
 * Handles outbound streaming (IQ from ring buffer, FFT at configured fps)
 * and inbound command dispatch (on its own blocking recv loop).
 * ---------------------------------------------------------------------- */
static void client_sender_task(void *arg)
{
    spyserver_client_t *client = (spyserver_client_t *)arg;

    /* send device info immediately on connect */
    send_device_info(client);

    /* command loop — blocks on recv, drives all state machine */
    process_commands(client);

    /* cleanup */
    client->active      = false;
    client->streaming_iq  = false;
    client->streaming_fft = false;
    close(client->sock);
    client->sock = -1;

    if (client->iq_ringbuf) {
        vRingbufferDelete(client->iq_ringbuf);
        client->iq_ringbuf = NULL;
    }

    ESP_LOGI(TAG, "client slot freed");
    client->sender_task = NULL;
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Listener task: accept connections, allocate client slots
 * ---------------------------------------------------------------------- */
static void listener_task(void *arg)
{
    struct spyserver_server *srv = (struct spyserver_server *)arg;

    while (srv->running) {
        struct sockaddr_in remote;
        socklen_t addr_len = sizeof(remote);
        int sock = accept(srv->listen_sock, (struct sockaddr *)&remote, &addr_len);
        if (sock < 0) {
            if (srv->running) {
                ESP_LOGE(TAG, "accept error: %d", errno);
            }
            continue;
        }

        ESP_LOGI(TAG, "new connection from %s:%d",
                 inet_ntoa(remote.sin_addr), ntohs(remote.sin_port));

        /* set socket keepalive / send timeout */
        int keepalive = 1;
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        /* find a free client slot */
        xSemaphoreTake(srv->clients_mutex, portMAX_DELAY);
        spyserver_client_t *slot = NULL;
        for (int i = 0; i < srv->config.max_clients; i++) {
            if (!srv->clients[i].active) {
                slot = &srv->clients[i];
                break;
            }
        }

        if (!slot) {
            xSemaphoreGive(srv->clients_mutex);
            ESP_LOGW(TAG, "max clients reached, rejecting connection");
            close(sock);
            continue;
        }

        /* initialise slot */
        memset(slot, 0, sizeof(*slot));
        slot->sock               = sock;
        slot->active             = true;
        slot->can_control        = srv->config.allow_control;
        slot->gain               = 0;
        slot->iq_frequency       = (RTL_MIN_FREQ_HZ + RTL_MAX_FREQ_HZ) / 2;
        slot->fft_frequency      = slot->iq_frequency;
        slot->iq_format          = STREAM_FORMAT_UINT8;
        slot->fft_format         = STREAM_FORMAT_UINT8;
        slot->fft_display_pixels = 1024;
        slot->fft_db_offset      = -100;
        slot->fft_db_range       = 70;
        slot->seq_iq             = 0;
        slot->seq_fft            = 0;
        slot->server             = srv;

        slot->send_mutex = xSemaphoreCreateMutex();
        slot->iq_ringbuf = xRingbufferCreate(IQ_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);

        if (!slot->send_mutex || !slot->iq_ringbuf) {
            ESP_LOGE(TAG, "failed to allocate client resources");
            if (slot->send_mutex) vSemaphoreDelete(slot->send_mutex);
            if (slot->iq_ringbuf) vRingbufferDelete(slot->iq_ringbuf);
            slot->active = false;
            xSemaphoreGive(srv->clients_mutex);
            close(sock);
            continue;
        }

        xSemaphoreGive(srv->clients_mutex);

        /* spawn per-client task */
        char task_name[24];
        snprintf(task_name, sizeof(task_name), "spy_cli_%d", sock);
        BaseType_t ret = xTaskCreate(client_sender_task, task_name,
                                     CLIENT_TASK_STACK, slot,
                                     CLIENT_TASK_PRIORITY, &slot->sender_task);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "failed to create client task");
            vSemaphoreDelete(slot->send_mutex);
            vRingbufferDelete(slot->iq_ringbuf);
            slot->active = false;
            close(sock);
        }
    }

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Public API: spyserver_start
 * ---------------------------------------------------------------------- */
esp_err_t spyserver_start(spyserver_handle_t *handle, const spyserver_config_t *config)
{
    if (!handle || !config) return ESP_ERR_INVALID_ARG;

    struct spyserver_server *srv = calloc(1, sizeof(*srv));
    if (!srv) return ESP_ERR_NO_MEM;

    memcpy(&srv->config, config, sizeof(*config));
    srv->device_serial = 0x00000001U;

    srv->clients = calloc(config->max_clients, sizeof(spyserver_client_t));
    if (!srv->clients) {
        free(srv);
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < config->max_clients; i++) {
        srv->clients[i].sock = -1;
    }

    srv->clients_mutex = xSemaphoreCreateMutex();
    if (!srv->clients_mutex) {
        free(srv->clients);
        free(srv);
        return ESP_ERR_NO_MEM;
    }

    /* create TCP listen socket */
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vSemaphoreDelete(srv->clients_mutex);
        free(srv->clients);
        free(srv);
        return ESP_FAIL;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(config->port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(sock);
        vSemaphoreDelete(srv->clients_mutex);
        free(srv->clients);
        free(srv);
        return ESP_FAIL;
    }

    if (listen(sock, config->max_clients) < 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(sock);
        vSemaphoreDelete(srv->clients_mutex);
        free(srv->clients);
        free(srv);
        return ESP_FAIL;
    }

    srv->listen_sock = sock;
    srv->running     = true;

    BaseType_t ret = xTaskCreatePinnedToCore(
        listener_task, "spy_listener",
        LISTENER_TASK_STACK, srv,
        LISTENER_TASK_PRIORITY, &srv->listener_task,
        LISTENER_TASK_CORE);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create listener task");
        close(sock);
        vSemaphoreDelete(srv->clients_mutex);
        free(srv->clients);
        free(srv);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SpyServer listening on port %d", config->port);
    *handle = srv;
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Public API: spyserver_stop
 * ---------------------------------------------------------------------- */
esp_err_t spyserver_stop(spyserver_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    struct spyserver_server *srv = handle;

    srv->running = false;

    /* unblock accept() */
    shutdown(srv->listen_sock, SHUT_RDWR);
    close(srv->listen_sock);
    srv->listen_sock = -1;

    /* disconnect all clients */
    xSemaphoreTake(srv->clients_mutex, portMAX_DELAY);
    for (int i = 0; i < srv->config.max_clients; i++) {
        if (srv->clients[i].active && srv->clients[i].sock >= 0) {
            shutdown(srv->clients[i].sock, SHUT_RDWR);
            close(srv->clients[i].sock);
            srv->clients[i].active = false;
        }
    }
    xSemaphoreGive(srv->clients_mutex);

    /* give tasks a moment to exit */
    vTaskDelay(pdMS_TO_TICKS(200));

    vSemaphoreDelete(srv->clients_mutex);
    free(srv->clients);
    free(srv);

    ESP_LOGI(TAG, "SpyServer stopped");
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Public API: spyserver_push_samples
 * Called from the RTL-SDR async callback with raw uint8 IQ data.
 * Fans out to all active streaming clients via their ring buffers,
 * and computes / sends FFT frames when enough data has accumulated.
 * ---------------------------------------------------------------------- */
void spyserver_push_samples(spyserver_handle_t handle,
                            const uint8_t *data, uint32_t len)
{
    if (!handle || !data || len == 0) return;
    struct spyserver_server *srv = handle;

    xSemaphoreTake(srv->clients_mutex, portMAX_DELAY);

    for (int i = 0; i < srv->config.max_clients; i++) {
        spyserver_client_t *client = &srv->clients[i];
        if (!client->active) continue;

        /* --- IQ streaming --- */
        if (client->streaming_iq && client->iq_ringbuf) {
            /* try to push; if ring is full, drop oldest by peeking/receiving */
            BaseType_t ok = xRingbufferSend(client->iq_ringbuf, data, len,
                                            0 /* don't block */);
            if (ok != pdTRUE) {
                /* ring full: drain half to make room */
                size_t item_size = 0;
                void *item = xRingbufferReceiveUpTo(client->iq_ringbuf,
                                                    &item_size,
                                                    0, IQ_RINGBUF_SIZE / 2);
                if (item) vRingbufferReturnItem(client->iq_ringbuf, item);
                xRingbufferSend(client->iq_ringbuf, data, len, 0);
            }

            /* drain ring buffer and send IQ frames to client */
            size_t chunk_size = 0;
            uint8_t *chunk = (uint8_t *)xRingbufferReceive(
                                client->iq_ringbuf, &chunk_size, 0);
            if (chunk) {
                send_message(client, MSG_TYPE_UINT8_IQ, STREAM_TYPE_IQ,
                             chunk, (uint32_t)chunk_size);
                vRingbufferReturnItem(client->iq_ringbuf, chunk);
            }
        }

        /* --- FFT streaming --- */
        if (client->streaming_fft && len >= (FFT_SIZE * 2)) {
            uint32_t pixels = client->fft_display_pixels ?
                              client->fft_display_pixels : FFT_SIZE;
            if (pixels > FFT_SIZE) pixels = FFT_SIZE;

            uint8_t fft_buf[FFT_SIZE];
            compute_fft_uint8(data, len / 2,
                              fft_buf, pixels,
                              client->fft_db_offset,
                              client->fft_db_range);
            send_message(client, MSG_TYPE_UINT8_FFT, STREAM_TYPE_FFT,
                         fft_buf, pixels);
        }
    }

    xSemaphoreGive(srv->clients_mutex);
}
