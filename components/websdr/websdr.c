/*
 * WebSDR Server for ESP32-P4
 *
 * HTTP server serves static web UI, WebSocket endpoint streams FFT
 * spectrum data and narrowband DDC IQ to browser clients.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "websdr.h"
#include "dsp.h"

static const char *TAG = "websdr";

/* Embedded web files (via EMBED_FILES in CMakeLists.txt) */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t sdr_js_start[]     asm("_binary_sdr_js_start");
extern const uint8_t sdr_js_end[]       asm("_binary_sdr_js_end");
extern const uint8_t sdr_css_start[]    asm("_binary_sdr_css_start");
extern const uint8_t sdr_css_end[]      asm("_binary_sdr_css_end");

/* ──────────────────────── Constants ──────────────────────── */

#define MAX_WS_CLIENTS      3
#define IQ_BUF_SIZE         (256 * 1024)    /* IQ ring — 125ms at 1MSPS, PSRAM backed */
#define TASK_PERIOD_MS      5               /* Internal loop rate (200Hz) for DDC throughput */
#define DDC_OUT_BUF_SIZE    8192

/* ──────────────────────── Server State ──────────────────────── */

typedef struct {
    int             fd;
    bool            active;
    bool            iq_subscribed;
    int32_t         iq_offset_hz;
    uint32_t        iq_bw_hz;
    dsp_ddc_t      *ddc;
} ws_client_t;

struct websdr_server {
    httpd_handle_t      httpd;
    rtlsdr_dev_t       *dev;
    uint32_t            fft_size;
    uint32_t            fft_rate;

    /* Configurable dB range */
    float               db_min;
    float               db_max;

    /* WebSocket clients */
    ws_client_t         clients[MAX_WS_CLIENTS];
    SemaphoreHandle_t   client_mutex;

    /* IQ sample buffer (producer: push_samples, consumer: FFT task) */
    uint8_t            *iq_buf;
    _Atomic uint32_t    iq_write_pos;
    _Atomic uint32_t    iq_read_pos;

    /* FFT output buffer */
    uint8_t            *fft_out;

    /* Deferred FFT size change (applied in task loop to avoid race) */
    volatile int        pending_fft_size;

    /* Deferred freq change — coalesces rapid tuning into one USB call */
    volatile uint32_t   pending_freq;

    /* Processing task */
    TaskHandle_t        fft_task;
    volatile bool       running;
};

/* ──────────────────────── IQ Ring Buffer ──────────────────────── */

static uint32_t iq_ring_available(websdr_server_t *srv)
{
    uint32_t w = atomic_load_explicit(&srv->iq_write_pos, memory_order_acquire);
    uint32_t r = atomic_load_explicit(&srv->iq_read_pos, memory_order_acquire);
    if (w >= r) return w - r;
    return IQ_BUF_SIZE - r + w;
}

static uint32_t iq_ring_read(websdr_server_t *srv, uint8_t *dst, uint32_t len)
{
    uint32_t avail = iq_ring_available(srv);
    if (len > avail) len = avail;
    if (len == 0) return 0;

    uint32_t r = atomic_load_explicit(&srv->iq_read_pos, memory_order_acquire);
    uint32_t first = IQ_BUF_SIZE - r;
    if (first > len) first = len;

    memcpy(dst, srv->iq_buf + r, first);
    if (len > first) {
        memcpy(dst + first, srv->iq_buf, len - first);
    }
    atomic_store_explicit(&srv->iq_read_pos, (r + len) % IQ_BUF_SIZE, memory_order_release);
    return len;
}

/* ──────────────────────── WebSocket Helpers ──────────────────────── */

static ws_client_t *find_client_by_fd(websdr_server_t *srv, int fd)
{
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (srv->clients[i].active && srv->clients[i].fd == fd) {
            return &srv->clients[i];
        }
    }
    return NULL;
}

static ws_client_t *alloc_client(websdr_server_t *srv, int fd)
{
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (!srv->clients[i].active) {
            memset(&srv->clients[i], 0, sizeof(ws_client_t));
            srv->clients[i].fd = fd;
            srv->clients[i].active = true;
            return &srv->clients[i];
        }
    }
    return NULL;
}

static void free_client(ws_client_t *c)
{
    if (c->ddc) {
        dsp_ddc_free(c->ddc);
        c->ddc = NULL;
    }
    c->active = false;
    c->iq_subscribed = false;
}

static esp_err_t send_ws_text(httpd_handle_t hd, int fd, const char *text)
{
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)text,
        .len = strlen(text),
    };
    return httpd_ws_send_frame_async(hd, fd, &frame);
}

static esp_err_t send_ws_binary(httpd_handle_t hd, int fd, const uint8_t *data, size_t len)
{
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = (uint8_t *)data,
        .len = len,
    };
    return httpd_ws_send_frame_async(hd, fd, &frame);
}

/* Mark a client as dead by fd.  Must be called WITHOUT client_mutex held. */
static void evict_client_by_fd(websdr_server_t *srv, int fd)
{
    xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
    ws_client_t *c = find_client_by_fd(srv, fd);
    if (c) {
        ESP_LOGW(TAG, "Evicting dead client fd=%d", fd);
        free_client(c);
    }
    xSemaphoreGive(srv->client_mutex);
}

static void send_device_info(websdr_server_t *srv, int fd)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"info\",\"freq\":%lu,\"rate\":%lu,\"gain\":%d,"
             "\"fft_size\":%lu,\"db_min\":%.1f,\"db_max\":%.1f}",
             (unsigned long)rtlsdr_get_center_freq(srv->dev),
             (unsigned long)rtlsdr_get_sample_rate(srv->dev),
             rtlsdr_get_tuner_gain(srv->dev),
             (unsigned long)srv->fft_size,
             (double)srv->db_min, (double)srv->db_max);
    send_ws_text(srv->httpd, fd, buf);
}

static void send_config_to_all(websdr_server_t *srv)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"config\",\"fft_size\":%lu,\"sample_rate\":%lu,"
             "\"db_min\":%.1f,\"db_max\":%.1f}",
             (unsigned long)srv->fft_size,
             (unsigned long)rtlsdr_get_sample_rate(srv->dev),
             (double)srv->db_min, (double)srv->db_max);

    xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (srv->clients[i].active) {
            send_ws_text(srv->httpd, srv->clients[i].fd, buf);
        }
    }
    xSemaphoreGive(srv->client_mutex);
}

/* ──────────────────────── WebSocket Command Handler ──────────────────────── */

static void handle_ws_command(websdr_server_t *srv, ws_client_t *client,
                              const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGW(TAG, "Invalid JSON from client fd=%d", client->fd);
        return;
    }

    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (!cmd || !cJSON_IsString(cmd)) {
        cJSON_Delete(root);
        return;
    }

    const char *cmd_str = cmd->valuestring;

    if (strcmp(cmd_str, "freq") == 0) {
        cJSON *val = cJSON_GetObjectItem(root, "value");
        if (val && cJSON_IsNumber(val)) {
            uint32_t freq = (uint32_t)val->valuedouble;
            ESP_LOGI(TAG, "Set freq: %lu Hz", (unsigned long)freq);
            /* Defer to FFT task — coalesces rapid tuning so only the
             * latest value hits USB, preventing STALL cascades. */
            srv->pending_freq = freq;
        }
    } else if (strcmp(cmd_str, "gain") == 0) {
        cJSON *val = cJSON_GetObjectItem(root, "value");
        if (val && cJSON_IsNumber(val)) {
            int gain = (int)val->valuedouble;
            ESP_LOGI(TAG, "Set gain: %d", gain);
            rtlsdr_set_tuner_gain_mode(srv->dev, 1);
            rtlsdr_set_tuner_gain(srv->dev, gain);
        }
    } else if (strcmp(cmd_str, "rate") == 0 || strcmp(cmd_str, "sample_rate") == 0) {
        cJSON *val = cJSON_GetObjectItem(root, "value");
        if (val && cJSON_IsNumber(val)) {
            uint32_t rate = (uint32_t)val->valuedouble;
            ESP_LOGI(TAG, "Set sample rate: %lu", (unsigned long)rate);
            rtlsdr_set_sample_rate(srv->dev, rate);
            dsp_fft_reset();
            send_config_to_all(srv);
        }
    } else if (strcmp(cmd_str, "fft_size") == 0) {
        cJSON *val = cJSON_GetObjectItem(root, "value");
        if (val && cJSON_IsNumber(val)) {
            int new_size = (int)val->valuedouble;
            /* Validate: must be power of 2 in range 256..16384 */
            if (new_size >= 256 && new_size <= 16384 && (new_size & (new_size - 1)) == 0) {
                ESP_LOGI(TAG, "Set FFT size: %d (deferred to task loop)", new_size);
                srv->pending_fft_size = new_size;
            }
        }
    } else if (strcmp(cmd_str, "db_range") == 0) {
        cJSON *jmin = cJSON_GetObjectItem(root, "min");
        cJSON *jmax = cJSON_GetObjectItem(root, "max");
        if (jmin && cJSON_IsNumber(jmin) && jmax && cJSON_IsNumber(jmax)) {
            float db_min = (float)jmin->valuedouble;
            float db_max = (float)jmax->valuedouble;
            if (db_min < db_max) {
                srv->db_min = db_min;
                srv->db_max = db_max;
                dsp_fft_set_range(db_min, db_max);
                ESP_LOGI(TAG, "Set dB range: %.1f to %.1f", (double)db_min, (double)db_max);
                send_config_to_all(srv);
            }
        }
    } else if (strcmp(cmd_str, "subscribe_iq") == 0) {
        cJSON *offset = cJSON_GetObjectItem(root, "offset");
        cJSON *bw = cJSON_GetObjectItem(root, "bw");
        int32_t off_hz = (offset && cJSON_IsNumber(offset)) ? (int32_t)offset->valuedouble : 0;
        uint32_t bw_hz = (bw && cJSON_IsNumber(bw)) ? (uint32_t)bw->valuedouble : 25000;

        ESP_LOGI(TAG, "Subscribe IQ: offset=%ld bw=%lu",
                 (long)off_hz, (unsigned long)bw_hz);

        xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
        /* Free existing DDC if any */
        if (client->ddc) {
            dsp_ddc_free(client->ddc);
        }
        client->ddc = dsp_ddc_create(rtlsdr_get_sample_rate(srv->dev),
                                      (uint32_t)off_hz, bw_hz);
        client->iq_subscribed = (client->ddc != NULL);
        client->iq_offset_hz = off_hz;
        client->iq_bw_hz = bw_hz;
        xSemaphoreGive(srv->client_mutex);

        if (client->iq_subscribed) {
            uint32_t out_rate = dsp_ddc_get_output_rate(client->ddc);
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "{\"type\":\"iq_start\",\"offset\":%ld,\"bw\":%lu,\"rate\":%lu}",
                     (long)off_hz, (unsigned long)bw_hz, (unsigned long)out_rate);
            send_ws_text(srv->httpd, client->fd, buf);
        }
    } else if (strcmp(cmd_str, "unsubscribe_iq") == 0) {
        xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
        if (client->ddc) {
            dsp_ddc_free(client->ddc);
            client->ddc = NULL;
        }
        client->iq_subscribed = false;
        xSemaphoreGive(srv->client_mutex);
        send_ws_text(srv->httpd, client->fd, "{\"type\":\"iq_stop\"}");
    }

    cJSON_Delete(root);
}

/* ──────────────────────── HTTP Handlers ──────────────────────── */

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t js_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char *)sdr_js_start,
                    sdr_js_end - sdr_js_start);
    return ESP_OK;
}

static esp_err_t css_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)sdr_css_start,
                    sdr_css_end - sdr_css_start);
    return ESP_OK;
}

/* ──────────────────────── WebSocket Handler ──────────────────────── */

static esp_err_t ws_handler(httpd_req_t *req)
{
    websdr_server_t *srv = (websdr_server_t *)req->user_ctx;

    if (req->method == HTTP_GET) {
        /* New WebSocket connection */
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket connected: fd=%d", fd);

        xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
        ws_client_t *c = alloc_client(srv, fd);
        xSemaphoreGive(srv->client_mutex);

        if (!c) {
            ESP_LOGW(TAG, "Max WebSocket clients reached, rejecting fd=%d", fd);
            return ESP_FAIL;
        }

        /* Send device info */
        send_device_info(srv, fd);
        return ESP_OK;
    }

    /* Receive WebSocket frame */
    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = HTTPD_WS_TYPE_TEXT;

    /* Get frame length first */
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    if (frame.len == 0) return ESP_OK;

    if (frame.len > 1024) {
        ESP_LOGW(TAG, "WS frame too large: %d bytes", frame.len);
        return ESP_ERR_INVALID_SIZE;
    }

    /* Allocate buffer and receive payload */
    uint8_t *buf = malloc(frame.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;

    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        free(buf);
        return ret;
    }

    int fd = httpd_req_to_sockfd(req);

    if (frame.type == HTTPD_WS_TYPE_TEXT) {
        buf[frame.len] = '\0';
        xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
        ws_client_t *c = find_client_by_fd(srv, fd);
        xSemaphoreGive(srv->client_mutex);
        if (c) {
            handle_ws_command(srv, c, (const char *)buf);
        }
    } else if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "WebSocket closed: fd=%d", fd);
        xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
        ws_client_t *c = find_client_by_fd(srv, fd);
        if (c) free_client(c);
        xSemaphoreGive(srv->client_mutex);
    }

    free(buf);
    return ESP_OK;
}

/* ──────────────────────── Closed Socket Callback ──────────────────────── */

static void on_close_socket(httpd_handle_t hd, int fd)
{
    /* We use a global pointer since the close callback has no user_ctx.
     * This is safe because we only have one websdr server instance. */
    extern websdr_server_t *g_websdr_srv;
    websdr_server_t *srv = g_websdr_srv;

    if (srv) {
        xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
        ws_client_t *c = find_client_by_fd(srv, fd);
        if (c) {
            ESP_LOGI(TAG, "Socket closed: fd=%d", fd);
            free_client(c);
        }
        xSemaphoreGive(srv->client_mutex);
    }

    close(fd);
}

/* Global server pointer for close callback */
websdr_server_t *g_websdr_srv = NULL;

/* ──────────────────────── FFT Processing Task ──────────────────────── */

static void fft_process_task(void *arg)
{
    websdr_server_t *srv = (websdr_server_t *)arg;

    /* Internal loop runs at TASK_PERIOD_MS (5ms = 200Hz) for DDC throughput.
     * FFT frames are only broadcast at the configured display rate (fft_rate). */
    uint32_t fft_interval_ms = 1000 / srv->fft_rate;
    if (fft_interval_ms < TASK_PERIOD_MS) fft_interval_ms = TASK_PERIOD_MS;

    /* Buffer large enough for one full task period of IQ data.
     * At 3.2MSPS max: 5ms × 3.2M × 2 = 32KB. Use 3/4 of ring. */
    uint32_t chunk_size = (IQ_BUF_SIZE * 3) / 4;
    uint8_t *iq_tmp = malloc(chunk_size);
    if (!iq_tmp) {
        ESP_LOGE(TAG, "Failed to allocate IQ buffer (%lu)", (unsigned long)chunk_size);
        vTaskDelete(NULL);
        return;
    }

    uint8_t *ddc_out = malloc(DDC_OUT_BUF_SIZE);
    uint8_t *fft_frame = malloc(1 + 16384);
    uint8_t *iq_frame = malloc(1 + DDC_OUT_BUF_SIZE);

    /* Per-client DDC output buffers for send-outside-mutex pattern.
     * Heap-allocated to avoid blowing the task stack (3 × 8KB = 24KB). */
    uint8_t *ddc_buf_pool = malloc(MAX_WS_CLIENTS * DDC_OUT_BUF_SIZE);

    if (!ddc_out || !fft_frame || !iq_frame || !ddc_buf_pool) {
        free(iq_tmp); free(ddc_out); free(fft_frame); free(iq_frame);
        free(ddc_buf_pool);
        ESP_LOGE(TAG, "Failed to allocate task buffers");
        vTaskDelete(NULL);
        return;
    }
    fft_frame[0] = WEBSDR_MSG_FFT;
    iq_frame[0] = WEBSDR_MSG_IQ;

    ESP_LOGI(TAG, "DSP task started (FFT %lu bins @ %lu Hz, DDC loop %d ms)",
             (unsigned long)srv->fft_size, (unsigned long)srv->fft_rate,
             TASK_PERIOD_MS);

    TickType_t last_fft_send = xTaskGetTickCount();

    while (srv->running) {
        TickType_t start = xTaskGetTickCount();

        /* Apply deferred FFT size change safely within the task context */
        int pend = srv->pending_fft_size;
        if (pend > 0) {
            srv->pending_fft_size = 0;
            ESP_LOGI(TAG, "Applying deferred FFT size: %d", pend);
            dsp_fft_init(pend);
            srv->fft_size = dsp_fft_get_size();

            /* Reallocate FFT output buffer */
            uint8_t *new_fft_out = realloc(srv->fft_out, srv->fft_size);
            if (new_fft_out) {
                srv->fft_out = new_fft_out;
            }

            send_config_to_all(srv);
        }

        /* Apply deferred frequency change (coalesces rapid tuning) */
        uint32_t pend_freq = srv->pending_freq;
        if (pend_freq > 0) {
            srv->pending_freq = 0;
            esp_err_t fret = rtlsdr_set_center_freq(srv->dev, pend_freq);
            if (fret == ESP_OK) {
                /* Notify all clients of new freq */
                char fbuf[128];
                snprintf(fbuf, sizeof(fbuf), "{\"type\":\"freq\",\"value\":%lu}",
                         (unsigned long)pend_freq);
                xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
                for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                    if (srv->clients[i].active) {
                        send_ws_text(srv->httpd, srv->clients[i].fd, fbuf);
                    }
                }
                xSemaphoreGive(srv->client_mutex);
            } else {
                ESP_LOGW(TAG, "Freq change to %lu Hz failed", (unsigned long)pend_freq);
            }
        }

        /* Read all available IQ data from ring */
        uint32_t avail = iq_ring_available(srv);
        if (avail > chunk_size) avail = chunk_size;
        avail &= ~1u;  /* align to IQ pairs */

        if (avail > 0) {
            uint32_t got = iq_ring_read(srv, iq_tmp, avail);

            /* ── FFT: always feed data (internal accumulation) ── */
            int fft_len = 0;
            dsp_fft_compute(iq_tmp, got, srv->fft_out, &fft_len);

            /* Broadcast FFT frame at display rate only */
            if (fft_len > 0) {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_fft_send) >= pdMS_TO_TICKS(fft_interval_ms)) {
                    last_fft_send = now;
                    memcpy(fft_frame + 1, srv->fft_out, fft_len);
                    size_t frame_len = 1 + fft_len;

                    /* Collect active FDs under mutex, send outside */
                    int fft_fds[MAX_WS_CLIENTS];
                    int fft_fd_count = 0;

                    xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
                    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                        if (srv->clients[i].active) {
                            fft_fds[fft_fd_count++] = srv->clients[i].fd;
                        }
                    }
                    xSemaphoreGive(srv->client_mutex);

                    for (int j = 0; j < fft_fd_count; j++) {
                        if (send_ws_binary(srv->httpd, fft_fds[j],
                                           fft_frame, frame_len) != ESP_OK) {
                            evict_client_by_fd(srv, fft_fds[j]);
                        }
                    }
                }
            }

            /* ── DDC: runs every cycle for full audio throughput ── */
            /* Each client has its own DDC (different offset/bw), so
             * process under mutex but buffer outputs, then send outside
             * to minimise mutex hold time and prevent ring overruns. */
            typedef struct { int fd; uint32_t len; } ddc_pending_t;
            ddc_pending_t ddc_sends[MAX_WS_CLIENTS];
            int ddc_send_count = 0;

            xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                ws_client_t *c = &srv->clients[i];
                if (c->active && c->iq_subscribed && c->ddc) {
                    uint32_t out_len = DDC_OUT_BUF_SIZE;
                    if (dsp_ddc_process(c->ddc, iq_tmp, got, ddc_out, &out_len) == 0
                        && out_len > 0) {
                        ddc_sends[ddc_send_count].fd = c->fd;
                        ddc_sends[ddc_send_count].len = out_len;
                        memcpy(ddc_buf_pool + ddc_send_count * DDC_OUT_BUF_SIZE,
                               ddc_out, out_len);
                        ddc_send_count++;
                    }
                }
            }
            xSemaphoreGive(srv->client_mutex);

            /* Send DDC frames outside mutex */
            for (int j = 0; j < ddc_send_count; j++) {
                iq_frame[0] = WEBSDR_MSG_IQ;
                memcpy(iq_frame + 1,
                       ddc_buf_pool + j * DDC_OUT_BUF_SIZE,
                       ddc_sends[j].len);
                if (send_ws_binary(srv->httpd, ddc_sends[j].fd,
                                   iq_frame, 1 + ddc_sends[j].len) != ESP_OK) {
                    evict_client_by_fd(srv, ddc_sends[j].fd);
                }
            }
        }

        /* Sleep for remainder of 5ms period */
        TickType_t elapsed = xTaskGetTickCount() - start;
        TickType_t target = pdMS_TO_TICKS(TASK_PERIOD_MS);
        if (elapsed < target) {
            vTaskDelay(target - elapsed);
        }
    }

    free(iq_tmp);
    free(ddc_out);
    free(fft_frame);
    free(iq_frame);
    free(ddc_buf_pool);
    ESP_LOGI(TAG, "DSP task stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t websdr_server_start(websdr_server_t **out_server, const websdr_config_t *config)
{
    ESP_RETURN_ON_FALSE(config->dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    esp_err_t ret __attribute__((unused)) = ESP_OK;
    websdr_server_t *srv = calloc(1, sizeof(websdr_server_t));
    ESP_RETURN_ON_FALSE(srv, ESP_ERR_NO_MEM, TAG, "alloc failed");

    srv->dev = config->dev;
    srv->fft_size = config->fft_size ? config->fft_size : WEBSDR_DEFAULT_FFT_SIZE;
    srv->fft_rate = config->fft_rate ? config->fft_rate : WEBSDR_DEFAULT_FFT_RATE;
    srv->db_min = 10.0f;
    srv->db_max = 90.0f;
    srv->running = true;

    srv->client_mutex = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(srv->client_mutex, ESP_ERR_NO_MEM, err, TAG, "mutex alloc failed");

    /* Allocate IQ ring buffer */
    srv->iq_buf = malloc(IQ_BUF_SIZE);
    ESP_GOTO_ON_FALSE(srv->iq_buf, ESP_ERR_NO_MEM, err, TAG, "IQ buf alloc failed");
    srv->iq_write_pos = 0;
    srv->iq_read_pos = 0;

    /* Allocate FFT output buffer (max possible size for runtime changes) */
    srv->fft_out = malloc(16384);
    ESP_GOTO_ON_FALSE(srv->fft_out, ESP_ERR_NO_MEM, err, TAG, "FFT out alloc failed");

    /* Initialize DSP FFT engine */
    dsp_fft_init(srv->fft_size);

    /* Set global pointer for close callback */
    g_websdr_srv = srv;

    /* Start HTTP server */
    httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();
    httpd_config.server_port = config->http_port ? config->http_port : WEBSDR_DEFAULT_PORT;
    httpd_config.max_uri_handlers = 8;
    httpd_config.max_open_sockets = 3;  /* Keep within lwIP socket limit */
    httpd_config.close_fn = on_close_socket;
    httpd_config.stack_size = 8192;

    ESP_GOTO_ON_ERROR(httpd_start(&srv->httpd, &httpd_config), err, TAG,
                      "HTTP server start failed");

    /* Register HTTP endpoints */
    const httpd_uri_t uri_index = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler,
    };
    const httpd_uri_t uri_js = {
        .uri = "/sdr.js", .method = HTTP_GET, .handler = js_handler,
    };
    const httpd_uri_t uri_css = {
        .uri = "/sdr.css", .method = HTTP_GET, .handler = css_handler,
    };
    const httpd_uri_t uri_ws = {
        .uri = "/ws", .method = HTTP_GET, .handler = ws_handler,
        .user_ctx = srv, .is_websocket = true,
    };

    httpd_register_uri_handler(srv->httpd, &uri_index);
    httpd_register_uri_handler(srv->httpd, &uri_js);
    httpd_register_uri_handler(srv->httpd, &uri_css);
    httpd_register_uri_handler(srv->httpd, &uri_ws);

    /* Start FFT processing task on Core 1 */
    BaseType_t xret = xTaskCreatePinnedToCore(fft_process_task, "websdr_fft", 16384,
                                               srv, 6, &srv->fft_task, 1);
    ESP_GOTO_ON_FALSE(xret == pdPASS, ESP_ERR_NO_MEM, err_httpd, TAG,
                      "FFT task create failed");

    *out_server = srv;
    ESP_LOGI(TAG, "WebSDR server started on port %d (FFT %lu bins @ %lu Hz)",
             httpd_config.server_port, (unsigned long)srv->fft_size,
             (unsigned long)srv->fft_rate);
    return ESP_OK;

err_httpd:
    httpd_stop(srv->httpd);
err:
    if (srv->fft_out) free(srv->fft_out);
    if (srv->iq_buf) free(srv->iq_buf);
    if (srv->client_mutex) vSemaphoreDelete(srv->client_mutex);
    free(srv);
    return ESP_FAIL;
}

esp_err_t websdr_server_stop(websdr_server_t *srv)
{
    if (!srv) return ESP_ERR_INVALID_ARG;

    srv->running = false;
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Free all client DDCs */
    xSemaphoreTake(srv->client_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (srv->clients[i].active) {
            free_client(&srv->clients[i]);
        }
    }
    xSemaphoreGive(srv->client_mutex);

    httpd_stop(srv->httpd);

    free(srv->fft_out);
    free(srv->iq_buf);
    vSemaphoreDelete(srv->client_mutex);
    free(srv);

    g_websdr_srv = NULL;
    return ESP_OK;
}

void websdr_push_samples(websdr_server_t *srv, const uint8_t *iq_data, uint32_t len)
{
    if (!srv || !srv->running) return;

    /* Write to ring buffer using memcpy (overwrite old data if full) */
    uint32_t w = atomic_load_explicit(&srv->iq_write_pos, memory_order_acquire);
    uint32_t first = IQ_BUF_SIZE - w;
    if (first > len) first = len;

    memcpy(srv->iq_buf + w, iq_data, first);
    if (len > first) {
        memcpy(srv->iq_buf, iq_data + first, len - first);
    }
    atomic_store_explicit(&srv->iq_write_pos, (w + len) % IQ_BUF_SIZE, memory_order_release);
}
