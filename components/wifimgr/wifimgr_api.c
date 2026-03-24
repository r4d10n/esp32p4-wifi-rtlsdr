/*
 * WiFi Manager — REST API Handlers
 *
 * JSON-based REST API for all configuration endpoints.
 * Registered on the portal HTTP server (port 80).
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "mbedtls/base64.h"

#include "wifimgr.h"
#include "wifimgr_config.h"
#include "wifimgr_api.h"
#include "wifimgr_notify.h"
#include "wifimgr_chatbot.h"

/* OTA handlers (defined in wifimgr_ota.c) */
extern esp_err_t wifimgr_ota_handler(httpd_req_t *req);
extern esp_err_t wifimgr_ota_info_handler(httpd_req_t *req);

static const char *TAG = "wifimgr_api";

/* ── Basic Auth ─────────────────────────────────────────────── */

/* Admin password cache (loaded from NVS on first check) */
static char s_admin_pass[65] = {0};
static bool s_admin_pass_loaded = false;

static void load_admin_pass(void) {
    if (!s_admin_pass_loaded) {
        wifimgr_config_get_api_key("admin_pass", s_admin_pass, sizeof(s_admin_pass));
        s_admin_pass_loaded = true;
    }
}

/* Returns true if request is authorized (no password set, or valid Basic Auth) */
static bool check_auth(httpd_req_t *req) {
    load_admin_pass();
    if (s_admin_pass[0] == '\0') return true; /* No password set — open access */

    char auth_header[256] = {0};
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth_header, sizeof(auth_header)) != ESP_OK) {
        return false;
    }

    /* Expect "Basic <base64(admin:password)>" */
    if (strncmp(auth_header, "Basic ", 6) != 0) return false;

    /* Decode base64 */
    unsigned char decoded[128] = {0};
    size_t decoded_len = 0;
    if (mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &decoded_len,
                               (unsigned char *)auth_header + 6, strlen(auth_header + 6)) != 0) {
        return false;
    }
    decoded[decoded_len] = '\0';

    /* Expected format: "admin:<password>" */
    char expected[128];
    snprintf(expected, sizeof(expected), "admin:%s", s_admin_pass);

    return strcmp((char *)decoded, expected) == 0;
}

esp_err_t require_auth(httpd_req_t *req) {
    if (check_auth(req)) return ESP_OK;
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"ESP32-P4 SDR\"");
    cJSON *json = cJSON_CreateObject();
    if (json) {
        cJSON_AddStringToObject(json, "status", "error");
        cJSON_AddStringToObject(json, "message", "Authentication required");
        char *str = cJSON_PrintUnformatted(json);
        if (str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, str, strlen(str));
            cJSON_free(str);
        }
        cJSON_Delete(json);
    }
    return ESP_FAIL;
}

/* ── Helper: read request body as cJSON ─────────────────────── */
static cJSON *read_req_json(httpd_req_t *req)
{
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 8192) return NULL;

    char *buf = calloc(1, total_len + 1);
    if (!buf) return NULL;

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) { free(buf); return NULL; }
        received += ret;
    }

    cJSON *json = cJSON_Parse(buf);
    free(buf);
    return json;
}

/* ── Helper: send JSON response ─────────────────────────────── */
static esp_err_t send_json(httpd_req_t *req, cJSON *json)
{
    char *str = cJSON_PrintUnformatted(json);
    if (!str) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    return ESP_OK;
}

static esp_err_t send_ok(httpd_req_t *req, const char *msg)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) { httpd_resp_send_500(req); return ESP_FAIL; }
    cJSON_AddStringToObject(json, "status", "ok");
    if (msg) cJSON_AddStringToObject(json, "message", msg);
    esp_err_t err = send_json(req, json);
    cJSON_Delete(json);
    return err;
}

static esp_err_t send_error(httpd_req_t *req, int status, const char *msg)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) { httpd_resp_send_500(req); return ESP_FAIL; }
    cJSON_AddStringToObject(json, "status", "error");
    cJSON_AddStringToObject(json, "message", msg);
    const char *status_str = "500 Internal Server Error";
    switch (status) {
    case 400: status_str = "400 Bad Request"; break;
    case 401: status_str = "401 Unauthorized"; break;
    case 404: status_str = "404 Not Found"; break;
    case 408: status_str = "408 Request Timeout"; break;
    case 500: status_str = "500 Internal Server Error"; break;
    }
    httpd_resp_set_status(req, status_str);
    esp_err_t err = send_json(req, json);
    cJSON_Delete(json);
    return err;
}

/* ── CORS preflight handler ─────────────────────────────────── */
static esp_err_t options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════
 *  WiFi endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* GET /api/wifi/scan */
static esp_err_t api_wifi_scan(httpd_req_t *req)
{
    esp_err_t err = wifimgr_scan();
    if (err != ESP_OK) {
        return send_error(req, 500, "Scan failed");
    }

    wifimgr_scan_result_t *results = NULL;
    uint16_t count = 0;
    wifimgr_get_scan_results(&results, &count);

    cJSON *resp = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(resp, "networks");

    for (uint16_t i = 0; i < count; i++) {
        cJSON *net = cJSON_CreateObject();
        cJSON_AddStringToObject(net, "ssid", results[i].ssid);
        cJSON_AddNumberToObject(net, "rssi", results[i].rssi);
        cJSON_AddNumberToObject(net, "channel", results[i].channel);
        cJSON_AddBoolToObject(net, "saved", results[i].saved);

        const char *auth = "OPEN";
        switch (results[i].auth_mode) {
        case 1: auth = "WEP"; break;
        case 2: auth = "WPA"; break;
        case 3: auth = "WPA2"; break;
        case 4: auth = "WPA/WPA2"; break;
        case 5: auth = "WPA2-Enterprise"; break;
        case 6: auth = "WPA3"; break;
        default: auth = "OPEN"; break;
        }
        cJSON_AddStringToObject(net, "auth", auth);
        cJSON_AddItemToArray(arr, net);
    }

    free(results);
    err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* GET /api/wifi/networks */
static esp_err_t api_wifi_networks_get(httpd_req_t *req)
{
    wifimgr_network_t networks[CONFIG_WIFIMGR_MAX_NETWORKS];
    uint8_t count = 0;
    wifimgr_get_saved_networks(networks, CONFIG_WIFIMGR_MAX_NETWORKS, &count);

    cJSON *resp = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(resp, "networks");
    for (uint8_t i = 0; i < count; i++) {
        cJSON *net = cJSON_CreateObject();
        cJSON_AddStringToObject(net, "ssid", networks[i].ssid);
        cJSON_AddNumberToObject(net, "auth", networks[i].auth_mode);
        cJSON_AddItemToArray(arr, net);
    }

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* POST /api/wifi/networks — add/update a network */
static esp_err_t api_wifi_networks_post(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *ssid_j = cJSON_GetObjectItem(body, "ssid");
    cJSON *pass_j = cJSON_GetObjectItem(body, "password");

    if (!cJSON_IsString(ssid_j) || !cJSON_IsString(pass_j)) {
        cJSON_Delete(body);
        return send_error(req, 400, "Missing ssid or password");
    }

    wifimgr_network_t net = {0};
    strncpy(net.ssid, ssid_j->valuestring, WIFIMGR_SSID_MAX_LEN - 1);
    strncpy(net.password, pass_j->valuestring, WIFIMGR_PASS_MAX_LEN - 1);
    net.auth_mode = WIFI_AUTH_WPA2_PSK;

    cJSON *auth_j = cJSON_GetObjectItem(body, "auth");
    if (cJSON_IsNumber(auth_j)) {
        net.auth_mode = (wifi_auth_mode_t)auth_j->valueint;
    }

    cJSON_Delete(body);

    esp_err_t err = wifimgr_save_network(&net);
    if (err != ESP_OK) {
        return send_error(req, 500, "Failed to save network");
    }
    return send_ok(req, "Network saved");
}

/* DELETE /api/wifi/networks?ssid=... */
static esp_err_t api_wifi_networks_delete(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    char query[128] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return send_error(req, 400, "Missing ssid parameter");
    }

    char ssid[WIFIMGR_SSID_MAX_LEN] = {0};
    if (httpd_query_key_value(query, "ssid", ssid, sizeof(ssid)) != ESP_OK) {
        return send_error(req, 400, "Missing ssid parameter");
    }

    esp_err_t err = wifimgr_delete_network(ssid);
    if (err == ESP_ERR_NOT_FOUND) {
        return send_error(req, 404, "Network not found");
    }
    if (err != ESP_OK) {
        return send_error(req, 500, "Failed to delete");
    }
    return send_ok(req, "Network deleted");
}

/* POST /api/wifi/connect */
static esp_err_t api_wifi_connect(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *ssid_j = cJSON_GetObjectItem(body, "ssid");
    cJSON *pass_j = cJSON_GetObjectItem(body, "password");

    if (!cJSON_IsString(ssid_j)) {
        cJSON_Delete(body);
        return send_error(req, 400, "Missing ssid");
    }

    const char *password = cJSON_IsString(pass_j) ? pass_j->valuestring : NULL;

    esp_err_t err = wifimgr_test_connect(ssid_j->valuestring, password, 15000);
    cJSON_Delete(body);

    if (err == ESP_OK) {
        return send_ok(req, "Connected successfully");
    }
    return send_error(req, 408, "Connection failed or timed out");
}

/* GET /api/wifi/status */
static esp_err_t api_wifi_status(httpd_req_t *req)
{
    wifimgr_status_t status;
    wifimgr_get_status(&status);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "connected", status.state == WIFIMGR_STATE_CONNECTED ||
                                              status.state == WIFIMGR_STATE_APSTA_MODE);
    cJSON_AddStringToObject(resp, "ssid", status.ssid);
    cJSON_AddStringToObject(resp, "ip", status.ip);
    cJSON_AddNumberToObject(resp, "rssi", status.rssi);

    const char *mode_str = "NULL";
    switch (status.mode) {
    case WIFI_MODE_STA:  mode_str = "STA"; break;
    case WIFI_MODE_AP:   mode_str = "AP"; break;
    case WIFI_MODE_APSTA: mode_str = "APSTA"; break;
    default: break;
    }
    cJSON_AddStringToObject(resp, "mode", mode_str);

    const char *state_str = "init";
    switch (status.state) {
    case WIFIMGR_STATE_SCANNING: state_str = "scanning"; break;
    case WIFIMGR_STATE_CONNECTING: state_str = "connecting"; break;
    case WIFIMGR_STATE_CONNECTED: state_str = "connected"; break;
    case WIFIMGR_STATE_AP_MODE: state_str = "ap_mode"; break;
    case WIFIMGR_STATE_APSTA_MODE: state_str = "apsta_mode"; break;
    default: break;
    }
    cJSON_AddStringToObject(resp, "state", state_str);

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* ═══════════════════════════════════════════════════════════════
 *  Ethernet endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* GET /api/eth/config */
static esp_err_t api_eth_config_get(httpd_req_t *req)
{
    ethernet_config_t cfg;
    wifimgr_config_load_ethernet(&cfg);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "enable", cfg.enable);
    cJSON_AddBoolToObject(resp, "dhcp", cfg.dhcp);
    cJSON_AddStringToObject(resp, "static_ip", cfg.static_ip);
    cJSON_AddStringToObject(resp, "static_mask", cfg.static_mask);
    cJSON_AddStringToObject(resp, "static_gw", cfg.static_gw);
    cJSON_AddStringToObject(resp, "static_dns", cfg.static_dns);
    cJSON_AddStringToObject(resp, "phy_type", cfg.phy_type);
    cJSON_AddNumberToObject(resp, "phy_addr", cfg.phy_addr);
    cJSON_AddNumberToObject(resp, "mdc_gpio", cfg.mdc_gpio);
    cJSON_AddNumberToObject(resp, "mdio_gpio", cfg.mdio_gpio);
    cJSON_AddBoolToObject(resp, "prefer_over_wifi", cfg.prefer_over_wifi);

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* PUT /api/eth/config */
static esp_err_t api_eth_config_put(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    ethernet_config_t cfg;
    wifimgr_config_load_ethernet(&cfg);

    cJSON *item;
    if ((item = cJSON_GetObjectItem(body, "enable"))) cfg.enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "dhcp"))) cfg.dhcp = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "static_ip")) && cJSON_IsString(item))
        strncpy(cfg.static_ip, item->valuestring, sizeof(cfg.static_ip) - 1);
    if ((item = cJSON_GetObjectItem(body, "static_mask")) && cJSON_IsString(item))
        strncpy(cfg.static_mask, item->valuestring, sizeof(cfg.static_mask) - 1);
    if ((item = cJSON_GetObjectItem(body, "static_gw")) && cJSON_IsString(item))
        strncpy(cfg.static_gw, item->valuestring, sizeof(cfg.static_gw) - 1);
    if ((item = cJSON_GetObjectItem(body, "static_dns")) && cJSON_IsString(item))
        strncpy(cfg.static_dns, item->valuestring, sizeof(cfg.static_dns) - 1);
    if ((item = cJSON_GetObjectItem(body, "phy_type")) && cJSON_IsString(item))
        strncpy(cfg.phy_type, item->valuestring, sizeof(cfg.phy_type) - 1);
    if ((item = cJSON_GetObjectItem(body, "phy_addr")))
        cfg.phy_addr = (int8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(body, "mdc_gpio")))
        cfg.mdc_gpio = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(body, "mdio_gpio")))
        cfg.mdio_gpio = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(body, "prefer_over_wifi")))
        cfg.prefer_over_wifi = cJSON_IsTrue(item);

    cJSON_Delete(body);

    esp_err_t err = wifimgr_config_save_ethernet(&cfg);
    if (err != ESP_OK) return send_error(req, 500, "Save failed");
    return send_ok(req, "Ethernet config saved (reboot to apply)");
}

/* ═══════════════════════════════════════════════════════════════
 *  SDR config endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* GET /api/sdr/config */
static esp_err_t api_sdr_config_get(httpd_req_t *req)
{
    sdr_config_t cfg;
    wifimgr_config_load_sdr(&cfg);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddNumberToObject(resp, "center_freq", cfg.center_freq);
    cJSON_AddNumberToObject(resp, "sample_rate", cfg.sample_rate);
    cJSON_AddStringToObject(resp, "gain_mode", cfg.gain_mode);
    cJSON_AddNumberToObject(resp, "tuner_gain_tenth_db", cfg.tuner_gain_tenth_db);
    cJSON_AddBoolToObject(resp, "rtl_agc", cfg.rtl_agc);
    cJSON_AddBoolToObject(resp, "tuner_agc", cfg.tuner_agc);
    cJSON_AddNumberToObject(resp, "ppm_correction", cfg.ppm_correction);
    cJSON_AddStringToObject(resp, "direct_sampling", cfg.direct_sampling);
    cJSON_AddBoolToObject(resp, "offset_tuning", cfg.offset_tuning);
    cJSON_AddNumberToObject(resp, "offset_freq_hz", cfg.offset_freq_hz);
    cJSON_AddBoolToObject(resp, "bias_tee", cfg.bias_tee);
    cJSON_AddBoolToObject(resp, "dc_offset_correction", cfg.dc_offset_correction);
    cJSON_AddBoolToObject(resp, "iq_imbalance_correction", cfg.iq_imbalance_correction);
    cJSON_AddBoolToObject(resp, "invert_iq", cfg.invert_iq);
    cJSON_AddNumberToObject(resp, "max_total_users", cfg.max_total_users);
    cJSON_AddStringToObject(resp, "hostname", cfg.hostname);

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* PUT /api/sdr/config */
static esp_err_t api_sdr_config_put(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    sdr_config_t cfg;
    wifimgr_config_load_sdr(&cfg);

    cJSON *item;
    if ((item = cJSON_GetObjectItem(body, "center_freq")))
        cfg.center_freq = (uint32_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(body, "sample_rate")))
        cfg.sample_rate = (uint32_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(body, "gain_mode")) && cJSON_IsString(item))
        strncpy(cfg.gain_mode, item->valuestring, sizeof(cfg.gain_mode) - 1);
    if ((item = cJSON_GetObjectItem(body, "tuner_gain_tenth_db")))
        cfg.tuner_gain_tenth_db = (uint16_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(body, "rtl_agc"))) cfg.rtl_agc = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "tuner_agc"))) cfg.tuner_agc = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "ppm_correction")))
        cfg.ppm_correction = (int16_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(body, "direct_sampling")) && cJSON_IsString(item))
        strncpy(cfg.direct_sampling, item->valuestring, sizeof(cfg.direct_sampling) - 1);
    if ((item = cJSON_GetObjectItem(body, "offset_tuning")))
        cfg.offset_tuning = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "offset_freq_hz")))
        cfg.offset_freq_hz = (int32_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(body, "bias_tee"))) cfg.bias_tee = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "dc_offset_correction")))
        cfg.dc_offset_correction = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "iq_imbalance_correction")))
        cfg.iq_imbalance_correction = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "invert_iq"))) cfg.invert_iq = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "max_total_users")))
        cfg.max_total_users = (uint8_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(body, "hostname")) && cJSON_IsString(item))
        strncpy(cfg.hostname, item->valuestring, sizeof(cfg.hostname) - 1);

    cJSON_Delete(body);

    esp_err_t err = wifimgr_config_save_sdr(&cfg);
    if (err != ESP_OK) return send_error(req, 500, "Save failed");
    return send_ok(req, "SDR config saved");
}

/* ═══════════════════════════════════════════════════════════════
 *  Services endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* GET /api/services */
static esp_err_t api_services_list(httpd_req_t *req)
{
    cJSON *services = wifimgr_config_load_services();
    esp_err_t err = send_json(req, services);
    cJSON_Delete(services);
    return err;
}

/* GET /api/services/:name (wildcard match) */
static esp_err_t api_service_get(httpd_req_t *req)
{
    /* Extract service name from URI: /api/services/rtl_tcp -> "rtl_tcp" */
    const char *uri = req->uri;
    const char *name = uri + strlen("/api/services/");
    if (!name || name[0] == '\0') {
        return api_services_list(req);
    }

    /* Strip query string if present */
    char svc_name[32] = {0};
    const char *q = strchr(name, '?');
    size_t len = q ? (size_t)(q - name) : strlen(name);
    if (len >= sizeof(svc_name)) len = sizeof(svc_name) - 1;
    memcpy(svc_name, name, len);

    cJSON *services = wifimgr_config_load_services();
    cJSON *svc = cJSON_GetObjectItem(services, svc_name);
    if (!svc) {
        cJSON_Delete(services);
        return send_error(req, 404, "Service not found");
    }

    /* Return a copy (detached) */
    cJSON *copy = cJSON_Duplicate(svc, true);
    cJSON_Delete(services);

    esp_err_t err = send_json(req, copy);
    cJSON_Delete(copy);
    return err;
}

/* PUT /api/services/:name */
static esp_err_t api_service_put(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    const char *uri = req->uri;
    const char *name = uri + strlen("/api/services/");

    char svc_name[32] = {0};
    const char *q = strchr(name, '?');
    size_t len = q ? (size_t)(q - name) : strlen(name);
    if (len >= sizeof(svc_name)) len = sizeof(svc_name) - 1;
    memcpy(svc_name, name, len);

    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *services = wifimgr_config_load_services();
    cJSON *existing = cJSON_GetObjectItem(services, svc_name);

    if (existing) {
        /* Merge: update existing fields with incoming values */
        cJSON *field = NULL;
        cJSON_ArrayForEach(field, body) {
            cJSON_DeleteItemFromObject(existing, field->string);
            cJSON_AddItemToObject(existing, field->string,
                                  cJSON_Duplicate(field, true));
        }
    } else {
        /* New service — just add it */
        cJSON_AddItemToObject(services, svc_name, cJSON_Duplicate(body, true));
    }

    cJSON_Delete(body);
    esp_err_t err = wifimgr_config_save_services(services);
    cJSON_Delete(services);

    if (err != ESP_OK) return send_error(req, 500, "Save failed");
    return send_ok(req, "Service config updated");
}

/* ═══════════════════════════════════════════════════════════════
 *  Notification endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* GET /api/notify/config */
static esp_err_t api_notify_config_get(httpd_req_t *req)
{
    notify_config_t cfg;
    wifimgr_config_load_notify(&cfg);

    cJSON *resp = cJSON_CreateObject();

    cJSON *tg = cJSON_AddObjectToObject(resp, "telegram");
    cJSON_AddBoolToObject(tg, "enable", cfg.telegram.enable);
    cJSON_AddNumberToObject(tg, "rate_limit_s", cfg.telegram.rate_limit_s);
    /* Mask token: show first 8 chars + "..." */
    if (cfg.telegram.bot_token[0]) {
        char masked[16] = {0};
        strncpy(masked, cfg.telegram.bot_token, 8);
        strcat(masked, "...");
        cJSON_AddStringToObject(tg, "bot_token", masked);
    } else {
        cJSON_AddStringToObject(tg, "bot_token", "");
    }
    cJSON_AddStringToObject(tg, "chat_id", cfg.telegram.chat_id);

    cJSON *dc = cJSON_AddObjectToObject(resp, "discord");
    cJSON_AddBoolToObject(dc, "enable", cfg.discord.enable);
    cJSON_AddNumberToObject(dc, "rate_limit_s", cfg.discord.rate_limit_s);
    if (cfg.discord.webhook_url[0]) {
        cJSON_AddStringToObject(dc, "webhook_url", "https://discord.com/api/webhooks/...");
    } else {
        cJSON_AddStringToObject(dc, "webhook_url", "");
    }

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* PUT /api/notify/config */
static esp_err_t api_notify_config_put(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    notify_config_t cfg;
    wifimgr_config_load_notify(&cfg);

    cJSON *tg = cJSON_GetObjectItem(body, "telegram");
    if (tg) {
        cJSON *item;
        if ((item = cJSON_GetObjectItem(tg, "enable"))) cfg.telegram.enable = cJSON_IsTrue(item);
        if ((item = cJSON_GetObjectItem(tg, "rate_limit_s")))
            cfg.telegram.rate_limit_s = (uint16_t)item->valueint;
        if ((item = cJSON_GetObjectItem(tg, "bot_token")) && cJSON_IsString(item) &&
            strcmp(item->valuestring, "") != 0 && !strstr(item->valuestring, "..."))
            strncpy(cfg.telegram.bot_token, item->valuestring, sizeof(cfg.telegram.bot_token) - 1);
        if ((item = cJSON_GetObjectItem(tg, "chat_id")) && cJSON_IsString(item))
            strncpy(cfg.telegram.chat_id, item->valuestring, sizeof(cfg.telegram.chat_id) - 1);
    }

    cJSON *dc = cJSON_GetObjectItem(body, "discord");
    if (dc) {
        cJSON *item;
        if ((item = cJSON_GetObjectItem(dc, "enable"))) cfg.discord.enable = cJSON_IsTrue(item);
        if ((item = cJSON_GetObjectItem(dc, "rate_limit_s")))
            cfg.discord.rate_limit_s = (uint16_t)item->valueint;
        if ((item = cJSON_GetObjectItem(dc, "webhook_url")) && cJSON_IsString(item) &&
            strcmp(item->valuestring, "") != 0 && !strstr(item->valuestring, "..."))
            strncpy(cfg.discord.webhook_url, item->valuestring, sizeof(cfg.discord.webhook_url) - 1);
    }

    cJSON_Delete(body);

    esp_err_t err = wifimgr_config_save_notify(&cfg);
    if (err != ESP_OK) return send_error(req, 500, "Save failed");

    wifimgr_notify_reload_config();
    return send_ok(req, "Notification config saved");
}

/* POST /api/notify/test */
static esp_err_t api_notify_test(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *channel = cJSON_GetObjectItem(body, "channel");
    if (!cJSON_IsString(channel)) {
        cJSON_Delete(body);
        return send_error(req, 400, "Missing channel");
    }

    esp_err_t err = wifimgr_notify_test(channel->valuestring);
    cJSON_Delete(body);

    if (err == ESP_OK) return send_ok(req, "Test notification sent");
    return send_error(req, 500, "Send failed (check credentials)");
}

/* ═══════════════════════════════════════════════════════════════
 *  System endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* GET /api/system/info */
static esp_err_t api_system_info(httpd_req_t *req)
{
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddNumberToObject(resp, "uptime_s", (double)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));
    cJSON_AddNumberToObject(resp, "heap_free", esp_get_free_heap_size());
    cJSON_AddNumberToObject(resp, "psram_free", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    cJSON_AddNumberToObject(resp, "heap_min", esp_get_minimum_free_heap_size());

    /* Hostname from SDR config */
    sdr_config_t sdr;
    wifimgr_config_load_sdr(&sdr);
    cJSON_AddStringToObject(resp, "hostname", sdr.hostname);
    cJSON_AddStringToObject(resp, "version", "1.0.0");

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    cJSON_AddNumberToObject(resp, "cores", chip.cores);

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* POST /api/system/reboot */
static esp_err_t api_system_reboot(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *confirm = cJSON_GetObjectItem(body, "confirm");
    bool confirmed = cJSON_IsTrue(confirm);
    cJSON_Delete(body);

    if (!confirmed) {
        return send_error(req, 400, "Confirmation required");
    }

    send_ok(req, "Rebooting...");

    /* Delay to let response be sent */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK; /* unreachable */
}

/* GET /api/system/backup */
static esp_err_t api_system_backup(httpd_req_t *req)
{
    cJSON *backup = wifimgr_config_export_all();
    if (!backup) return send_error(req, 500, "Export failed");

    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"esp32p4-sdr-config.json\"");
    esp_err_t err = send_json(req, backup);
    cJSON_Delete(backup);
    return err;
}

/* POST /api/system/restore */
static esp_err_t api_system_restore(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    esp_err_t err = wifimgr_config_import_all(body);
    cJSON_Delete(body);

    if (err != ESP_OK) return send_error(req, 500, "Import failed");
    return send_ok(req, "Config restored (reboot to apply)");
}

/* POST /api/system/factory-reset */
static esp_err_t api_system_factory_reset(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *confirm = cJSON_GetObjectItem(body, "confirm");
    bool confirmed = cJSON_IsTrue(confirm);
    cJSON_Delete(body);

    if (!confirmed) {
        return send_error(req, 400, "Confirmation required");
    }

    wifimgr_config_factory_reset();
    send_ok(req, "Factory reset complete. Rebooting...");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════
 *  Chat endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* POST /api/chat/message — {message: "..."} -> {reply: "..."} */
static esp_err_t api_chat_message(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *msg_j = cJSON_GetObjectItem(body, "message");
    if (!cJSON_IsString(msg_j) || msg_j->valuestring[0] == '\0') {
        cJSON_Delete(body);
        return send_error(req, 400, "Missing message");
    }

    char *reply = NULL;
    esp_err_t err = wifimgr_chatbot_message(msg_j->valuestring, &reply);
    cJSON_Delete(body);

    if (err != ESP_OK || !reply) {
        return send_error(req, 500, "Chatbot error");
    }

    cJSON *resp = cJSON_CreateObject();
    if (!resp) { free(reply); httpd_resp_send_500(req); return ESP_FAIL; }
    cJSON_AddStringToObject(resp, "reply", reply);
    free(reply);

    err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* GET /api/chat/history — return conversation history array */
static esp_err_t api_chat_history_get(httpd_req_t *req)
{
    cJSON *history = wifimgr_chatbot_get_history();
    if (!history) { httpd_resp_send_500(req); return ESP_FAIL; }

    cJSON *resp = cJSON_CreateObject();
    if (!resp) { cJSON_Delete(history); httpd_resp_send_500(req); return ESP_FAIL; }
    cJSON_AddItemToObject(resp, "history", history);

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* DELETE /api/chat/history — clear conversation history */
static esp_err_t api_chat_history_delete(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    wifimgr_chatbot_clear_history();
    return send_ok(req, "Chat history cleared");
}

/* GET /api/chat/config — return chatbot config (API key masked) */
static esp_err_t api_chat_config_get(httpd_req_t *req)
{
    chatbot_config_t cfg;
    wifimgr_config_load_chatbot(&cfg);

    cJSON *resp = cJSON_CreateObject();
    if (!resp) { httpd_resp_send_500(req); return ESP_FAIL; }

    cJSON_AddBoolToObject(resp, "enable", cfg.enable);
    cJSON_AddStringToObject(resp, "provider", cfg.provider);
    cJSON_AddStringToObject(resp, "model", cfg.model);
    cJSON_AddBoolToObject(resp, "web_enable", cfg.web_enable);
    cJSON_AddBoolToObject(resp, "telegram_enable", cfg.telegram_enable);
    cJSON_AddBoolToObject(resp, "discord_enable", cfg.discord_enable);
    cJSON_AddNumberToObject(resp, "max_history", cfg.max_history);
    cJSON_AddStringToObject(resp, "allowed_tools", cfg.allowed_tools);

    /* Mask API key */
    if (cfg.api_key[0]) {
        char masked[16] = {0};
        strncpy(masked, cfg.api_key, 8);
        strcat(masked, "...");
        cJSON_AddStringToObject(resp, "api_key", masked);
    } else {
        cJSON_AddStringToObject(resp, "api_key", "");
    }

    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* PUT /api/chat/config — update chatbot config */
static esp_err_t api_chat_config_put(httpd_req_t *req)
{
    if (require_auth(req) != ESP_OK) return ESP_OK;
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    chatbot_config_t cfg;
    wifimgr_config_load_chatbot(&cfg);

    cJSON *item;
    if ((item = cJSON_GetObjectItem(body, "enable")))
        cfg.enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "provider")) && cJSON_IsString(item))
        strncpy(cfg.provider, item->valuestring, sizeof(cfg.provider) - 1);
    if ((item = cJSON_GetObjectItem(body, "model")) && cJSON_IsString(item))
        strncpy(cfg.model, item->valuestring, sizeof(cfg.model) - 1);
    if ((item = cJSON_GetObjectItem(body, "web_enable")))
        cfg.web_enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "telegram_enable")))
        cfg.telegram_enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "discord_enable")))
        cfg.discord_enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(body, "max_history")) && cJSON_IsNumber(item))
        cfg.max_history = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(body, "allowed_tools")) && cJSON_IsString(item))
        strncpy(cfg.allowed_tools, item->valuestring, sizeof(cfg.allowed_tools) - 1);
    /* Only update API key if non-empty and not masked */
    if ((item = cJSON_GetObjectItem(body, "api_key")) && cJSON_IsString(item) &&
        item->valuestring[0] != '\0' && !strstr(item->valuestring, "..."))
        strncpy(cfg.api_key, item->valuestring, sizeof(cfg.api_key) - 1);

    cJSON_Delete(body);

    esp_err_t err = wifimgr_config_save_chatbot(&cfg);
    if (err != ESP_OK) return send_error(req, 500, "Save failed");

    return send_ok(req, "Chatbot config saved");
}

/* ═══════════════════════════════════════════════════════════════
 *  Auth endpoints
 * ═══════════════════════════════════════════════════════════════ */

/* POST /api/auth/setup */
static esp_err_t api_auth_setup(httpd_req_t *req) {
    cJSON *body = read_req_json(req);
    if (!body) return send_error(req, 400, "Invalid JSON");

    cJSON *new_pass = cJSON_GetObjectItem(body, "password");
    if (!cJSON_IsString(new_pass) || strlen(new_pass->valuestring) < 8) {
        cJSON_Delete(body);
        return send_error(req, 400, "Password must be at least 8 characters");
    }

    /* If admin password already set, require current password */
    load_admin_pass();
    if (s_admin_pass[0] != '\0') {
        cJSON *current = cJSON_GetObjectItem(body, "current_password");
        if (!cJSON_IsString(current) || strcmp(current->valuestring, s_admin_pass) != 0) {
            cJSON_Delete(body);
            return send_error(req, 401, "Current password incorrect");
        }
    }

    /* Save new password */
    wifimgr_config_set_api_key("admin_pass", new_pass->valuestring);
    strncpy(s_admin_pass, new_pass->valuestring, sizeof(s_admin_pass) - 1);

    cJSON_Delete(body);
    return send_ok(req, "Admin password set");
}

/* GET /api/auth/status */
static esp_err_t api_auth_status(httpd_req_t *req) {
    load_admin_pass();
    cJSON *resp = cJSON_CreateObject();
    if (!resp) { httpd_resp_send_500(req); return ESP_FAIL; }
    cJSON_AddBoolToObject(resp, "configured", s_admin_pass[0] != '\0');
    esp_err_t err = send_json(req, resp);
    cJSON_Delete(resp);
    return err;
}

/* ═══════════════════════════════════════════════════════════════
 *  Registration
 * ═══════════════════════════════════════════════════════════════ */

esp_err_t wifimgr_api_register(httpd_handle_t server)
{
    if (!server) return ESP_ERR_INVALID_ARG;

    /* CORS preflight for all API paths */
    const httpd_uri_t cors = {
        .uri = "/api/*", .method = HTTP_OPTIONS, .handler = options_handler
    };
    httpd_register_uri_handler(server, &cors);

    /* WiFi */
    const httpd_uri_t uris[] = {
        { "/api/wifi/scan",     HTTP_GET,    api_wifi_scan },
        { "/api/wifi/networks", HTTP_GET,    api_wifi_networks_get },
        { "/api/wifi/networks", HTTP_POST,   api_wifi_networks_post },
        { "/api/wifi/networks", HTTP_DELETE, api_wifi_networks_delete },
        { "/api/wifi/connect",  HTTP_POST,   api_wifi_connect },
        { "/api/wifi/status",   HTTP_GET,    api_wifi_status },
        /* Ethernet */
        { "/api/eth/config",    HTTP_GET,    api_eth_config_get },
        { "/api/eth/config",    HTTP_PUT,    api_eth_config_put },
        /* SDR */
        { "/api/sdr/config",    HTTP_GET,    api_sdr_config_get },
        { "/api/sdr/config",    HTTP_PUT,    api_sdr_config_put },
        /* Services */
        { "/api/services",      HTTP_GET,    api_services_list },
        { "/api/services/*",    HTTP_GET,    api_service_get },
        { "/api/services/*",    HTTP_PUT,    api_service_put },
        /* Notifications */
        { "/api/notify/config", HTTP_GET,    api_notify_config_get },
        { "/api/notify/config", HTTP_PUT,    api_notify_config_put },
        { "/api/notify/test",   HTTP_POST,   api_notify_test },
        /* System */
        { "/api/system/info",          HTTP_GET,  api_system_info },
        { "/api/system/reboot",        HTTP_POST, api_system_reboot },
        { "/api/system/backup",        HTTP_GET,  api_system_backup },
        { "/api/system/restore",       HTTP_POST, api_system_restore },
        { "/api/system/factory-reset", HTTP_POST, api_system_factory_reset },
        /* Auth */
        { "/api/auth/setup",  HTTP_POST, api_auth_setup },
        { "/api/auth/status", HTTP_GET,  api_auth_status },
        /* Chat */
        { "/api/chat/message",  HTTP_POST,   api_chat_message },
        { "/api/chat/history",  HTTP_GET,    api_chat_history_get },
        { "/api/chat/history",  HTTP_DELETE, api_chat_history_delete },
        { "/api/chat/config",   HTTP_GET,    api_chat_config_get },
        { "/api/chat/config",   HTTP_PUT,    api_chat_config_put },
        /* OTA */
        { "/api/system/ota",     HTTP_POST, wifimgr_ota_handler },
        { "/api/system/ota/info", HTTP_GET, wifimgr_ota_info_handler },
    };

    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }

    ESP_LOGI(TAG, "REST API registered (%d endpoints)",
             (int)(sizeof(uris) / sizeof(uris[0])) + 1);
    return ESP_OK;
}
