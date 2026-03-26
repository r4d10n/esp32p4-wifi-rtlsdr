/*
 * Simple Web Radio Control Interface
 *
 * HTTPS server with REST API and WebSocket for mono FM radio control.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_https_server.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include <unistd.h>
#include "cJSON.h"
#include "web_radio_simple.h"

/* Embedded TLS certificates */
extern const uint8_t servercert_start[] asm("_binary_servercert_pem_start");
extern const uint8_t servercert_end[]   asm("_binary_servercert_pem_end");
extern const uint8_t prvtkey_start[]    asm("_binary_prvtkey_pem_start");
extern const uint8_t prvtkey_end[]      asm("_binary_prvtkey_pem_end");

static const char *TAG = "web_radio";

/* Embedded web files */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* ── Constants ── */

#define MAX_WS_CLIENTS      4
#define WS_TIMER_PERIOD_MS  500     /* 2 Hz signal strength push (reduced to avoid TLS contention with audio) */
#define POST_BODY_MAX       256

/* Audio buffer: 9600 samples = 200ms @ 48kHz mono.
 * Larger buffer reduces TLS frame rate (5/s instead of 10/s),
 * preventing mbedTLS overload that causes connection drops. */
#define AUDIO_BUF_SAMPLES   9600
static int16_t              s_audio_buf[AUDIO_BUF_SAMPLES];
static int                  s_audio_buf_pos = 0;

/* ── Server State ── */

static httpd_handle_t       s_httpd      = NULL;
static bool                 s_ws_audio[MAX_WS_CLIENTS];
static web_radio_simple_params_t s_params = {
    .frequency    = 100000000,
    .gain         = 0,
    .volume       = 70,
    .muted        = false,
};
static int16_t              s_signal_strength = 0;
static web_radio_simple_change_cb_t s_change_cb = NULL;
static void                *s_cb_ctx     = NULL;

/* WebSocket client fd table */
static int  s_ws_fds[MAX_WS_CLIENTS];
static int  s_ws_count = 0;
static portMUX_TYPE s_ws_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_timer_handle_t   s_ws_timer   = NULL;

/* ── WebSocket client tracking ── */

static void ws_add_fd(int fd)
{
    taskENTER_CRITICAL(&s_ws_mux);
    if (s_ws_count < MAX_WS_CLIENTS) {
        s_ws_audio[s_ws_count] = false;
        s_ws_fds[s_ws_count++] = fd;
    }
    taskEXIT_CRITICAL(&s_ws_mux);
}

static void ws_remove_fd(int fd)
{
    taskENTER_CRITICAL(&s_ws_mux);
    for (int i = 0; i < s_ws_count; i++) {
        if (s_ws_fds[i] == fd) {
            int last = --s_ws_count;
            s_ws_fds[i] = s_ws_fds[last];
            s_ws_audio[i] = s_ws_audio[last];
            break;
        }
    }
    taskEXIT_CRITICAL(&s_ws_mux);
}

/* ── Socket close callback ── */

static void on_close_socket(httpd_handle_t hd, int fd)
{
    ws_remove_fd(fd);
    close(fd);
}

/* ── WebSocket signal-strength timer ── */

static void ws_timer_cb(void *arg)
{
    if (!s_httpd) return;

    /* Skip signal strength push if any client has audio active —
     * avoid TLS frame contention that causes connection drops. */
    taskENTER_CRITICAL(&s_ws_mux);
    int count = s_ws_count;
    int fds[MAX_WS_CLIENTS];
    bool any_audio = false;
    for (int i = 0; i < count; i++) {
        fds[i] = s_ws_fds[i];
        if (s_ws_audio[i]) any_audio = true;
    }
    taskEXIT_CRITICAL(&s_ws_mux);

    if (any_audio) return;  /* Audio frames have priority */

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "{\"signal_strength\":%d}",
                       (int)s_signal_strength);

    for (int i = 0; i < count; i++) {
        httpd_ws_frame_t frame = {
            .type    = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)buf,
            .len     = (size_t)len,
            .final   = true,
        };
        esp_err_t err = httpd_ws_send_frame_async(s_httpd, fds[i], &frame);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "ws send failed fd=%d, removing", fds[i]);
            ws_remove_fd(fds[i]);
        }
    }
}

/* ── Helper: read full POST body ── */

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t bufsz)
{
    int remaining = req->content_len;
    if (remaining <= 0 || remaining >= (int)bufsz) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad body");
        return ESP_FAIL;
    }
    int received = httpd_req_recv(req, buf, remaining);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv error");
        return ESP_FAIL;
    }
    buf[received] = '\0';
    return ESP_OK;
}

/* ── Helper: send JSON response ── */

static void send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json);
}

/* ── CORS preflight ── */

static esp_err_t handler_cors_preflight(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_hdr(req, "Access-Control-Max-Age", "86400");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ── REST: GET /api/status ── */

static esp_err_t handler_status(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "freq",            (double)s_params.frequency);
    cJSON_AddNumberToObject(root, "gain",            s_params.gain);
    cJSON_AddNumberToObject(root, "volume",          s_params.volume);
    cJSON_AddBoolToObject  (root, "muted",           s_params.muted);
    cJSON_AddNumberToObject(root, "signal_strength", (int)s_signal_strength);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    send_json(req, json);
    free(json);
    return ESP_OK;
}

/* ── REST: POST /api/freq ── */

static esp_err_t handler_freq(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsNumber(val)) {
        s_params.frequency = (uint32_t)val->valuedouble;
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    }
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── REST: POST /api/gain ── */

static esp_err_t handler_gain(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsNumber(val)) {
        s_params.gain = (int)val->valueint;
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    }
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── REST: POST /api/volume ── */

static esp_err_t handler_volume(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsNumber(val)) {
        int v = val->valueint;
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        s_params.volume = (uint8_t)v;
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    }
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── WebSocket: GET /ws ── */

static esp_err_t handler_ws(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "ws client connected fd=%d", fd);
        ws_add_fd(fd);
        return ESP_OK;
    }

    /* Receive incoming frames */
    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_TEXT };
    uint8_t buf[128] = {0};
    frame.payload = buf;
    esp_err_t err = httpd_ws_recv_frame(req, &frame, sizeof(buf) - 1);
    if (err != ESP_OK) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGD(TAG, "ws recv error fd=%d, removing", fd);
        ws_remove_fd(fd);
        return ESP_OK;
    }

    /* Parse audio streaming commands */
    if (frame.type == HTTPD_WS_TYPE_TEXT && frame.len > 0) {
        buf[frame.len] = '\0';
        cJSON *root = cJSON_Parse((char *)buf);
        if (root) {
            cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
            if (cJSON_IsString(cmd)) {
                int fd = httpd_req_to_sockfd(req);
                if (strcmp(cmd->valuestring, "audio_on") == 0) {
                    taskENTER_CRITICAL(&s_ws_mux);
                    for (int i = 0; i < s_ws_count; i++) {
                        if (s_ws_fds[i] == fd) { s_ws_audio[i] = true; break; }
                    }
                    taskEXIT_CRITICAL(&s_ws_mux);
                    ESP_LOGI(TAG, "Audio streaming enabled for fd=%d", fd);
                } else if (strcmp(cmd->valuestring, "audio_off") == 0) {
                    taskENTER_CRITICAL(&s_ws_mux);
                    for (int i = 0; i < s_ws_count; i++) {
                        if (s_ws_fds[i] == fd) { s_ws_audio[i] = false; break; }
                    }
                    taskEXIT_CRITICAL(&s_ws_mux);
                    ESP_LOGI(TAG, "Audio streaming disabled for fd=%d", fd);
                }
            }
            cJSON_Delete(root);
        }
    }
    return ESP_OK;
}

/* ── Static: GET / ── */

static esp_err_t handler_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}

/* ── URI table ── */

static const httpd_uri_t s_uris[] = {
    { .uri = "/",             .method = HTTP_GET,     .handler = handler_index    },
    { .uri = "/api/status",   .method = HTTP_GET,     .handler = handler_status   },
    { .uri = "/api/freq",     .method = HTTP_POST,    .handler = handler_freq     },
    { .uri = "/api/gain",     .method = HTTP_POST,    .handler = handler_gain     },
    { .uri = "/api/volume",   .method = HTTP_POST,    .handler = handler_volume   },
    { .uri = "/ws",           .method = HTTP_GET,     .handler = handler_ws,
      .is_websocket = true                                                        },
    { .uri = "/api/*",        .method = HTTP_OPTIONS, .handler = handler_cors_preflight },
};

/* ── Public API ── */

esp_err_t web_radio_simple_start(const web_radio_simple_config_t *config)
{
    ESP_RETURN_ON_FALSE(config,    ESP_ERR_INVALID_ARG, TAG, "null config");
    ESP_RETURN_ON_FALSE(!s_httpd, ESP_ERR_INVALID_STATE, TAG, "already started");

    s_change_cb = config->change_cb;
    s_cb_ctx    = config->cb_ctx;
    memset(s_ws_fds, -1, sizeof(s_ws_fds));
    s_ws_count = 0;

    httpd_ssl_config_t hcfg = HTTPD_SSL_CONFIG_DEFAULT();
    hcfg.httpd.max_open_sockets  = 6;
    hcfg.httpd.max_uri_handlers  = 12;
    hcfg.httpd.lru_purge_enable  = true;
    hcfg.httpd.stack_size        = 12288;
    hcfg.httpd.close_fn          = on_close_socket;
    hcfg.port_secure             = config->http_port;
    hcfg.servercert              = servercert_start;
    hcfg.servercert_len          = servercert_end - servercert_start;
    hcfg.prvtkey_pem             = prvtkey_start;
    hcfg.prvtkey_len             = prvtkey_end - prvtkey_start;

    ESP_RETURN_ON_ERROR(httpd_ssl_start(&s_httpd, &hcfg), TAG, "httpd_ssl_start");

    for (int i = 0; i < (int)(sizeof(s_uris) / sizeof(s_uris[0])); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &s_uris[i]),
                            TAG, "register uri %s", s_uris[i].uri);
    }

    /* 10 Hz timer for WS signal strength push */
    esp_timer_create_args_t ta = {
        .callback        = ws_timer_cb,
        .arg             = NULL,
        .name            = "web_radio_ws",
        .dispatch_method = ESP_TIMER_TASK,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&ta, &s_ws_timer), TAG, "timer create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_ws_timer,
                            WS_TIMER_PERIOD_MS * 1000ULL), TAG, "timer start");

    ESP_LOGI(TAG, "started on port %u", config->http_port);
    return ESP_OK;
}

esp_err_t web_radio_simple_stop(void)
{
    if (s_ws_timer) {
        esp_timer_stop(s_ws_timer);
        esp_timer_delete(s_ws_timer);
        s_ws_timer = NULL;
    }
    if (s_httpd) {
        httpd_ssl_stop(s_httpd);
        s_httpd = NULL;
    }
    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}

void web_radio_simple_update_status(const web_radio_simple_params_t *params, int16_t signal_strength)
{
    if (params) {
        s_params = *params;
    }
    s_signal_strength = signal_strength;
}

static void ws_flush_audio(void)
{
    if (s_audio_buf_pos == 0) return;

    taskENTER_CRITICAL(&s_ws_mux);
    int n = s_ws_count;
    int fds[MAX_WS_CLIENTS];
    bool audio_flag[MAX_WS_CLIENTS];
    for (int i = 0; i < n; i++) {
        fds[i] = s_ws_fds[i];
        audio_flag[i] = s_ws_audio[i];
    }
    taskEXIT_CRITICAL(&s_ws_mux);

    bool any_audio = false;
    for (int i = 0; i < n; i++) {
        if (audio_flag[i]) { any_audio = true; break; }
    }
    if (!any_audio) {
        s_audio_buf_pos = 0;
        return;
    }

    for (int i = 0; i < n; i++) {
        if (!audio_flag[i]) continue;

        httpd_ws_frame_t frame = {
            .type    = HTTPD_WS_TYPE_BINARY,
            .payload = (uint8_t *)s_audio_buf,
            .len     = (size_t)(s_audio_buf_pos * sizeof(int16_t)),
            .final   = true,
        };
        esp_err_t err = httpd_ws_send_frame_async(s_httpd, fds[i], &frame);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "audio ws send failed fd=%d, removing", fds[i]);
            ws_remove_fd(fds[i]);
        }
    }
    s_audio_buf_pos = 0;
}

void web_radio_simple_push_audio(const int16_t *samples, int count)
{
    if (!s_httpd || count <= 0) return;

    while (count > 0) {
        int space = AUDIO_BUF_SAMPLES - s_audio_buf_pos;
        int copy = (count < space) ? count : space;
        memcpy(&s_audio_buf[s_audio_buf_pos], samples, copy * sizeof(int16_t));
        s_audio_buf_pos += copy;
        samples += copy;
        count -= copy;

        if (s_audio_buf_pos >= AUDIO_BUF_SAMPLES) {
            ws_flush_audio();
        }
    }
}
