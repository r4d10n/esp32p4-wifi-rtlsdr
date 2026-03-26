/*
 * Web Radio Control Interface
 *
 * HTTP server with REST API and WebSocket for FM radio control.
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
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include <unistd.h>
#include "cJSON.h"
#include "web_radio.h"

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
#define WS_TIMER_PERIOD_MS  100     /* 10 Hz signal strength push */
#define POST_BODY_MAX       256

/* Audio send buffer — accumulate PCM and flush in ~100ms chunks.
 * Stereo sends interleaved L/R (2 samples per pair), so 100ms stereo
 * = 9600 samples = 19200 bytes.  Size for stereo; mono gets 200ms
 * chunks which is fine (still only ~5 TLS frames/sec). */
#define AUDIO_BUF_SAMPLES   9600    /* 100 ms stereo or 200 ms mono @ 48 kHz */
static int16_t              s_audio_buf[AUDIO_BUF_SAMPLES];
static int                  s_audio_buf_pos = 0;

/* ── Server State ── */

static httpd_handle_t       s_httpd      = NULL;
static bool                 s_ws_audio[MAX_WS_CLIENTS];  /* per-client audio streaming flag */
static web_radio_params_t   s_params     = {
    .frequency    = 100000000,
    .gain         = 0,
    .volume       = 70,
    .mode         = 0,
    .filter_bw    = 15000,
    .muted        = false,
    .nb_enabled   = true,
    .nb_threshold = 5,
};
static int16_t              s_signal_strength = 0;
static web_radio_change_cb_t s_change_cb = NULL;
static void                *s_cb_ctx     = NULL;

/* RDS data cache */
static struct {
    char        ps_name[9];
    char        radio_text[65];
    uint16_t    pi_code;
    uint8_t     pty;
    bool        stereo;
} s_rds = {0};

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

/* ── Socket close callback (required for TLS socket cleanup) ── */

static void on_close_socket(httpd_handle_t hd, int fd)
{
    ws_remove_fd(fd);
    close(fd);
}

/* ── WebSocket signal-strength timer ── */

static void ws_timer_cb(void *arg)
{
    if (!s_httpd) return;

    char buf[96];
    int len = snprintf(buf, sizeof(buf), "{\"signal_strength\":%d,\"stereo\":%s}",
                       (int)s_signal_strength, s_params.stereo ? "true" : "false");

    taskENTER_CRITICAL(&s_ws_mux);
    int count = s_ws_count;
    int fds[MAX_WS_CLIENTS];
    for (int i = 0; i < count; i++) fds[i] = s_ws_fds[i];
    taskEXIT_CRITICAL(&s_ws_mux);

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

/* ── CORS preflight handler for all POST endpoints ── */

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
    cJSON_AddNumberToObject(root, "mode",            s_params.mode);
    cJSON_AddNumberToObject(root, "filter_bw",       (double)s_params.filter_bw);
    cJSON_AddBoolToObject  (root, "muted",           s_params.muted);
    cJSON_AddNumberToObject(root, "signal_strength", (int)s_signal_strength);
    cJSON_AddBoolToObject  (root, "stereo",          s_params.stereo);
    cJSON_AddNumberToObject(root, "sample_rate",     2400000);
    cJSON *nb = cJSON_CreateObject();
    cJSON_AddBoolToObject  (nb, "enabled",   s_params.nb_enabled);
    cJSON_AddNumberToObject(nb, "threshold", s_params.nb_threshold);
    cJSON_AddItemToObject  (root, "noise_blanker", nb);

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

/* ── REST: POST /api/mode ── */

static esp_err_t handler_mode(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsNumber(val)) {
        s_params.mode = val->valueint;
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    }
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── REST: POST /api/filter_bw ── */

static esp_err_t handler_filter_bw(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsNumber(val)) {
        s_params.filter_bw = (uint32_t)val->valuedouble;
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    }
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── REST: POST /api/mute ── */

static esp_err_t handler_mute(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsBool(val)) {
        s_params.muted = cJSON_IsTrue(val);
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    }
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── REST: POST /api/squelch ── */

static esp_err_t handler_squelch(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsNumber(val)) {
        s_params.squelch = (uint8_t)cJSON_GetNumberValue(val);
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    }
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── REST: POST /api/noise_blanker ── */

static esp_err_t handler_noise_blanker(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }
    cJSON *en  = cJSON_GetObjectItem(root, "enabled");
    cJSON *thr = cJSON_GetObjectItem(root, "threshold");
    if (cJSON_IsBool(en)) {
        s_params.nb_enabled = cJSON_IsTrue(en);
    }
    if (cJSON_IsNumber(thr)) {
        int t = thr->valueint;
        if (t < 1) t = 1;
        if (t > 10) t = 10;
        s_params.nb_threshold = (uint8_t)t;
    }
    if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── WebSocket: GET /ws ── */

static esp_err_t handler_ws(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* New WebSocket handshake */
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "ws client connected fd=%d", fd);
        ws_add_fd(fd);
        return ESP_OK;
    }

    /* Receive incoming frames — parse JSON commands */
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

/* ── REST: GET /api/rds ── */

static esp_err_t handler_rds(httpd_req_t *req)
{
    char pi_str[8];
    snprintf(pi_str, sizeof(pi_str), "0x%04X", s_rds.pi_code);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "pi_code",    pi_str);
    cJSON_AddStringToObject(root, "ps_name",    s_rds.ps_name);
    cJSON_AddStringToObject(root, "radio_text", s_rds.radio_text);
    cJSON_AddNumberToObject(root, "pty",        s_rds.pty);
    cJSON_AddBoolToObject  (root, "stereo",     s_rds.stereo);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    send_json(req, json);
    free(json);
    return ESP_OK;
}

/* ── REST: POST /api/ota — firmware binary upload ── */

static esp_err_t handler_ota(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA update started, content_length=%d", req->content_len);

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;
    esp_err_t ret = esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    char *buf = malloc(4096);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int received = 0;
    int total = req->content_len;

    while (received < total) {
        int read_len = httpd_req_recv(req, buf, 4096);
        if (read_len <= 0) {
            if (read_len == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "OTA recv failed");
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv failed");
            return ESP_FAIL;
        }

        ret = esp_ota_write(ota_handle, buf, read_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(ret));
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }

        received += read_len;
        ESP_LOGI(TAG, "OTA progress: %d/%d (%d%%)", received, total, received * 100 / total);
    }

    free(buf);

    ret = esp_ota_end(ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation failed");
        return ESP_FAIL;
    }

    ret = esp_ota_set_boot_partition(update);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed: %s", esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot failed");
        return ESP_FAIL;
    }

    send_json(req, "{\"ok\":true,\"msg\":\"OTA complete, rebooting...\"}");

    /* Reboot after short delay */
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

/* ── REST: POST /api/scan — {direction:"up"|"down"} or {action:"stop"} ── */

static esp_err_t handler_scan(httpd_req_t *req)
{
    char buf[POST_BODY_MAX];
    ESP_RETURN_ON_ERROR(read_body(req, buf, sizeof(buf)), TAG, "read_body");

    cJSON *root = cJSON_Parse(buf);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON"); return ESP_FAIL; }

    cJSON *action = cJSON_GetObjectItem(root, "action");
    cJSON *dir    = cJSON_GetObjectItem(root, "direction");

    if (action && cJSON_IsString(action) && strcmp(action->valuestring, "stop") == 0) {
        s_params.scan_request = 2; /* stop */
        if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
        s_params.scan_request = 0;
    } else if (dir && cJSON_IsString(dir)) {
        if (strcmp(dir->valuestring, "up") == 0) {
            s_params.scan_request = 1;
        } else if (strcmp(dir->valuestring, "down") == 0) {
            s_params.scan_request = -1;
        }
        if (s_params.scan_request != 0) {
            if (s_change_cb) s_change_cb(&s_params, s_cb_ctx);
            s_params.scan_request = 0;
        }
    }

    cJSON_Delete(root);
    send_json(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── URI table ── */

static const httpd_uri_t s_uris[] = {
    { .uri = "/",             .method = HTTP_GET,  .handler = handler_index    },
    { .uri = "/api/status",   .method = HTTP_GET,  .handler = handler_status   },
    { .uri = "/api/freq",     .method = HTTP_POST, .handler = handler_freq     },
    { .uri = "/api/gain",     .method = HTTP_POST, .handler = handler_gain     },
    { .uri = "/api/volume",   .method = HTTP_POST, .handler = handler_volume   },
    { .uri = "/api/mode",     .method = HTTP_POST, .handler = handler_mode     },
    { .uri = "/api/filter_bw",.method = HTTP_POST, .handler = handler_filter_bw},
    { .uri = "/api/mute",          .method = HTTP_POST, .handler = handler_mute          },
    { .uri = "/api/squelch",      .method = HTTP_POST, .handler = handler_squelch       },
    { .uri = "/api/noise_blanker",.method = HTTP_POST, .handler = handler_noise_blanker },
    { .uri = "/api/scan",         .method = HTTP_POST, .handler = handler_scan          },
    { .uri = "/api/rds",          .method = HTTP_GET,  .handler = handler_rds          },
    { .uri = "/ws",               .method = HTTP_GET,  .handler = handler_ws,
      .is_websocket = true                                                      },
    { .uri = "/api/ota",          .method = HTTP_POST, .handler = handler_ota  },
    /* CORS preflight for all POST endpoints */
    { .uri = "/api/*",        .method = HTTP_OPTIONS, .handler = handler_cors_preflight },
};

/* ── Public API ── */

esp_err_t web_radio_start(const web_radio_config_t *config)
{
    ESP_RETURN_ON_FALSE(config,    ESP_ERR_INVALID_ARG, TAG, "null config");
    ESP_RETURN_ON_FALSE(!s_httpd, ESP_ERR_INVALID_STATE, TAG, "already started");

    s_change_cb = config->change_cb;
    s_cb_ctx    = config->cb_ctx;
    memset(s_ws_fds, -1, sizeof(s_ws_fds));
    s_ws_count = 0;

    httpd_ssl_config_t hcfg = HTTPD_SSL_CONFIG_DEFAULT();
    hcfg.httpd.max_open_sockets  = 6;     /* Page + API + WS + headroom */
    hcfg.httpd.max_uri_handlers  = 20;
    hcfg.httpd.lru_purge_enable  = true;
    hcfg.httpd.stack_size        = 12288;  /* TLS needs more stack */
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

esp_err_t web_radio_stop(void)
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

void web_radio_update_status(const web_radio_params_t *params, int16_t signal_strength)
{
    if (params) {
        s_params = *params;
    }
    s_signal_strength = signal_strength;
}

void web_radio_update_rds(const char *ps_name, const char *radio_text,
                          uint16_t pi_code, uint8_t pty, bool stereo)
{
    if (ps_name) {
        strncpy(s_rds.ps_name, ps_name, sizeof(s_rds.ps_name) - 1);
        s_rds.ps_name[sizeof(s_rds.ps_name) - 1] = '\0';
    }
    if (radio_text) {
        strncpy(s_rds.radio_text, radio_text, sizeof(s_rds.radio_text) - 1);
        s_rds.radio_text[sizeof(s_rds.radio_text) - 1] = '\0';
    }
    s_rds.pi_code = pi_code;
    s_rds.pty = pty;
    s_rds.stereo = stereo;
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

void web_radio_push_audio(const int16_t *samples, int count)
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
