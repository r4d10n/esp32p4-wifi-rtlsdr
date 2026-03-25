/*
 * WiFi Manager — Captive Portal & HTTP Server
 *
 * Runs on port 80. Handles captive portal detection for Android/iOS/Windows/macOS.
 * Serves the single-page config UI and registers REST API endpoints.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"

#include "wifimgr_portal.h"
#include "wifimgr_api.h"

static const char *TAG = "wifimgr_portal";
static httpd_handle_t s_server;

/* ── Embedded static files ──────────────────────────────────── */
extern const uint8_t portal_html_start[] asm("_binary_portal_html_start");
extern const uint8_t portal_html_end[]   asm("_binary_portal_html_end");
extern const uint8_t portal_js_start[]   asm("_binary_portal_js_start");
extern const uint8_t portal_js_end[]     asm("_binary_portal_js_end");
extern const uint8_t portal_css_start[]  asm("_binary_portal_css_start");
extern const uint8_t portal_css_end[]    asm("_binary_portal_css_end");

/* ── Captive portal detection handlers ──────────────────────── */

/* Android: GET /generate_204 -> 302 redirect to portal */
static esp_err_t generate_204_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* iOS/macOS: GET /hotspot-detect.html -> 302 redirect to portal */
static esp_err_t hotspot_detect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Windows: GET /connecttest.txt -> 302 redirect to portal */
static esp_err_t connecttest_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Windows: GET /redirect -> 302 redirect to portal */
static esp_err_t redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ── Static file handlers ───────────────────────────────────── */

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, (const char *)portal_html_start,
                    portal_html_end - portal_html_start);
    return ESP_OK;
}

static esp_err_t js_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    httpd_resp_send(req, (const char *)portal_js_start,
                    portal_js_end - portal_js_start);
    return ESP_OK;
}

static esp_err_t css_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    httpd_resp_send(req, (const char *)portal_css_start,
                    portal_css_end - portal_css_start);
    return ESP_OK;
}

/* ── URI definitions ────────────────────────────────────────── */

static const httpd_uri_t s_uri_table[] = {
    /* Captive portal detection */
    { .uri = "/generate_204",        .method = HTTP_GET, .handler = generate_204_handler },
    { .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = hotspot_detect_handler },
    { .uri = "/connecttest.txt",     .method = HTTP_GET, .handler = connecttest_handler },
    { .uri = "/redirect",            .method = HTTP_GET, .handler = redirect_handler },
    /* Static files */
    { .uri = "/",           .method = HTTP_GET, .handler = index_handler },
    { .uri = "/portal.js",  .method = HTTP_GET, .handler = js_handler },
    { .uri = "/portal.css", .method = HTTP_GET, .handler = css_handler },
};

#define URI_TABLE_SIZE (sizeof(s_uri_table) / sizeof(s_uri_table[0]))

/* ── Start / Stop ───────────────────────────────────────────── */

esp_err_t wifimgr_portal_start(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "Portal already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WIFIMGR_PORTAL_PORT;
    config.max_uri_handlers = 48;       /* 7 portal + 35 API + 4 decode + CORS */
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start portal server: %s", esp_err_to_name(err));
        return err;
    }

    /* Register captive portal + static URIs */
    for (size_t i = 0; i < URI_TABLE_SIZE; i++) {
        httpd_register_uri_handler(s_server, &s_uri_table[i]);
    }

    /* Register REST API endpoints */
    wifimgr_api_register(s_server);

    ESP_LOGI(TAG, "Config portal started on port %d", CONFIG_WIFIMGR_PORTAL_PORT);
    return ESP_OK;
}

esp_err_t wifimgr_portal_stop(void)
{
    if (!s_server) return ESP_OK;

    httpd_stop(s_server);
    s_server = NULL;
    ESP_LOGI(TAG, "Config portal stopped");
    return ESP_OK;
}
