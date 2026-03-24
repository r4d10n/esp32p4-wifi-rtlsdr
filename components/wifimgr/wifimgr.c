/*
 * WiFi Manager — Core state machine
 *
 * Handles: scanning, RSSI-sorted auto-connect, AP fallback,
 * APSTA concurrent mode, UART '+++' detection, BOOT button long-press.
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "wifimgr.h"
#include "wifimgr_config.h"
#include "wifimgr_api.h"
#include "wifimgr_portal.h"
#include "wifimgr_notify.h"

static const char *TAG = "wifimgr";

/* ── Event bits ─────────────────────────────────────────────── */
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1
#define WIFI_SCAN_DONE_BIT    BIT2
#define CONFIG_MODE_BIT       BIT3

/* ── Internal state ─────────────────────────────────────────── */
static wifimgr_config_t s_config;
static wifimgr_state_t  s_state = WIFIMGR_STATE_INIT;
static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;
static TaskHandle_t s_task_handle;

/* Scan results (protected by mutex) */
static SemaphoreHandle_t s_scan_mutex;
static wifi_ap_record_t *s_ap_records;
static uint16_t s_ap_count;

/* Connection tracking */
static char s_connected_ssid[WIFIMGR_SSID_MAX_LEN];
static char s_connected_ip[16];
static int8_t s_connected_rssi;
static uint8_t s_scan_cycle_count;
static volatile bool s_config_mode_requested;

/* ── Forward declarations ───────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data);
static void ip_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data);
static void wifimgr_task(void *arg);
static void uart_pattern_task(void *arg);
static esp_err_t start_softap(void);
static esp_err_t stop_softap(void);
static int compare_rssi(const void *a, const void *b);

/* ── WiFi event handler ─────────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected");
        s_connected_ssid[0] = '\0';
        s_connected_ip[0] = '\0';
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (id == WIFI_EVENT_SCAN_DONE) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_SCAN_DONE_BIT);
    } else if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *evt = data;
        ESP_LOGI(TAG, "AP: station " MACSTR " joined", MAC2STR(evt->mac));
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *evt = data;
        ESP_LOGI(TAG, "AP: station " MACSTR " left", MAC2STR(evt->mac));
    }
}

static void ip_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = data;
        snprintf(s_connected_ip, sizeof(s_connected_ip), IPSTR,
                 IP2STR(&evt->ip_info.ip));
        ESP_LOGI(TAG, "STA got IP: %s", s_connected_ip);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
}

/* ── Initialization ─────────────────────────────────────────── */
esp_err_t wifimgr_init(const wifimgr_config_t *config)
{
    if (config) {
        s_config = *config;
    } else {
        wifimgr_config_t def = WIFIMGR_CONFIG_DEFAULT();
        s_config = def;
    }

    s_wifi_event_group = xEventGroupCreate();
    s_scan_mutex = xSemaphoreCreateMutex();

    /* Initialize NVS + LittleFS config store */
    esp_err_t err = wifimgr_config_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Config init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Initialize TCP/IP and event loop */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create netif instances */
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    /* WiFi init with default config */
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                ip_event_handler, NULL));

    /* UART pattern detection for '+++' */
    uart_enable_pattern_det_baud_intr(UART_NUM_0, '+', 3, 9, 0, 0);

    xTaskCreate(uart_pattern_task, "uart_pat", 2048, NULL, 5, NULL);

    /* Button long-press detection (using simple GPIO polling — no iot_button dep) */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << s_config.button_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    ESP_LOGI(TAG, "WiFi manager initialized (AP SSID: %s)", s_config.ap_ssid);
    return ESP_OK;
}

/* ── Start state machine ────────────────────────────────────── */
esp_err_t wifimgr_start(void)
{
    xTaskCreatePinnedToCore(wifimgr_task, "wifimgr", 6144, NULL, 7, &s_task_handle, 1);
    return ESP_OK;
}

/* ── Status queries ─────────────────────────────────────────── */
wifimgr_state_t wifimgr_get_state(void)
{
    return s_state;
}

esp_err_t wifimgr_get_status(wifimgr_status_t *status)
{
    if (!status) return ESP_ERR_INVALID_ARG;
    status->state = s_state;
    strncpy(status->ssid, s_connected_ssid, sizeof(status->ssid) - 1);
    status->ssid[sizeof(status->ssid) - 1] = '\0';
    strncpy(status->ip, s_connected_ip, sizeof(status->ip) - 1);
    status->ip[sizeof(status->ip) - 1] = '\0';
    status->rssi = s_connected_rssi;

    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) == ESP_OK)
        status->mode = mode;
    else
        status->mode = WIFI_MODE_NULL;

    return ESP_OK;
}

esp_netif_t *wifimgr_get_sta_netif(void) { return s_sta_netif; }
esp_netif_t *wifimgr_get_ap_netif(void) { return s_ap_netif; }

/* ── Scanning ───────────────────────────────────────────────── */
esp_err_t wifimgr_scan(void)
{
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    if (mode == WIFI_MODE_NULL) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active = { .min = 100, .max = 300 } },
    };

    xEventGroupClearBits(s_wifi_event_group, WIFI_SCAN_DONE_BIT);
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, false);
    if (err != ESP_OK) return err;

    /* Wait for scan done event (max 10s) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_SCAN_DONE_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));

    if (!(bits & WIFI_SCAN_DONE_BIT)) {
        esp_wifi_scan_stop();
        return ESP_ERR_TIMEOUT;
    }

    /* Retrieve results */
    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);

    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    free(s_ap_records);
    s_ap_records = NULL;
    s_ap_count = 0;

    if (count > 0) {
        s_ap_records = calloc(count, sizeof(wifi_ap_record_t));
        if (s_ap_records) {
            esp_wifi_scan_get_ap_records(&count, s_ap_records);
            s_ap_count = count;
            /* Sort by RSSI descending */
            qsort(s_ap_records, count, sizeof(wifi_ap_record_t), compare_rssi);
        }
    }
    xSemaphoreGive(s_scan_mutex);

    ESP_LOGI(TAG, "Scan complete: %d APs found", count);
    return ESP_OK;
}

esp_err_t wifimgr_get_scan_results(wifimgr_scan_result_t **results, uint16_t *count)
{
    if (!results || !count) return ESP_ERR_INVALID_ARG;

    /* Get saved networks for matching */
    wifimgr_network_t saved[CONFIG_WIFIMGR_MAX_NETWORKS];
    uint8_t saved_count = 0;
    wifimgr_get_saved_networks(saved, CONFIG_WIFIMGR_MAX_NETWORKS, &saved_count);

    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);

    *count = s_ap_count;
    if (s_ap_count == 0) {
        *results = NULL;
        xSemaphoreGive(s_scan_mutex);
        return ESP_OK;
    }

    *results = calloc(s_ap_count, sizeof(wifimgr_scan_result_t));
    if (!*results) {
        xSemaphoreGive(s_scan_mutex);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < s_ap_count; i++) {
        wifimgr_scan_result_t *r = &(*results)[i];
        strncpy(r->ssid, (char *)s_ap_records[i].ssid, WIFIMGR_SSID_MAX_LEN - 1);
        r->rssi = s_ap_records[i].rssi;
        r->auth_mode = s_ap_records[i].authmode;
        r->channel = s_ap_records[i].primary;
        r->saved = false;
        for (uint8_t j = 0; j < saved_count; j++) {
            if (strcmp(r->ssid, saved[j].ssid) == 0) {
                r->saved = true;
                break;
            }
        }
    }

    xSemaphoreGive(s_scan_mutex);
    return ESP_OK;
}

/* ── Network CRUD (NVS) ────────────────────────────────────── */
esp_err_t wifimgr_save_network(const wifimgr_network_t *network)
{
    if (!network || network->ssid[0] == '\0') return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi_creds", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    uint8_t count = 0;
    nvs_get_u8(nvs, "wifi_count", &count);

    /* Check if SSID already exists — update in place */
    char key[16];
    for (uint8_t i = 0; i < count; i++) {
        char existing_ssid[WIFIMGR_SSID_MAX_LEN] = {0};
        size_t len = sizeof(existing_ssid);
        snprintf(key, sizeof(key), "wifi_%d_ssid", i);
        nvs_get_str(nvs, key, existing_ssid, &len);
        if (strcmp(existing_ssid, network->ssid) == 0) {
            /* Update password */
            snprintf(key, sizeof(key), "wifi_%d_pass", i);
            nvs_set_str(nvs, key, network->password);
            snprintf(key, sizeof(key), "wifi_%d_auth", i);
            nvs_set_u8(nvs, key, (uint8_t)network->auth_mode);
            nvs_commit(nvs);
            nvs_close(nvs);
            ESP_LOGI(TAG, "Updated network: %s", network->ssid);
            return ESP_OK;
        }
    }

    /* Add new entry */
    if (count >= CONFIG_WIFIMGR_MAX_NETWORKS) {
        nvs_close(nvs);
        return ESP_ERR_NO_MEM;
    }

    snprintf(key, sizeof(key), "wifi_%d_ssid", count);
    nvs_set_str(nvs, key, network->ssid);
    snprintf(key, sizeof(key), "wifi_%d_pass", count);
    nvs_set_str(nvs, key, network->password);
    snprintf(key, sizeof(key), "wifi_%d_auth", count);
    nvs_set_u8(nvs, key, (uint8_t)network->auth_mode);

    count++;
    nvs_set_u8(nvs, "wifi_count", count);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Saved network #%d: %s", count, network->ssid);
    return ESP_OK;
}

esp_err_t wifimgr_delete_network(const char *ssid)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi_creds", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    uint8_t count = 0;
    nvs_get_u8(nvs, "wifi_count", &count);

    int found = -1;
    char key[16];

    /* Find the network */
    for (uint8_t i = 0; i < count; i++) {
        char existing_ssid[WIFIMGR_SSID_MAX_LEN] = {0};
        size_t len = sizeof(existing_ssid);
        snprintf(key, sizeof(key), "wifi_%d_ssid", i);
        nvs_get_str(nvs, key, existing_ssid, &len);
        if (strcmp(existing_ssid, ssid) == 0) {
            found = i;
            break;
        }
    }

    if (found < 0) {
        nvs_close(nvs);
        return ESP_ERR_NOT_FOUND;
    }

    /* Shift remaining entries down */
    for (uint8_t i = found; i < count - 1; i++) {
        char buf[WIFIMGR_PASS_MAX_LEN];
        size_t len;

        /* Copy i+1 -> i for ssid, pass, auth */
        snprintf(key, sizeof(key), "wifi_%d_ssid", i + 1);
        len = sizeof(buf);
        nvs_get_str(nvs, key, buf, &len);
        snprintf(key, sizeof(key), "wifi_%d_ssid", i);
        nvs_set_str(nvs, key, buf);

        snprintf(key, sizeof(key), "wifi_%d_pass", i + 1);
        len = sizeof(buf);
        nvs_get_str(nvs, key, buf, &len);
        snprintf(key, sizeof(key), "wifi_%d_pass", i);
        nvs_set_str(nvs, key, buf);

        uint8_t auth;
        snprintf(key, sizeof(key), "wifi_%d_auth", i + 1);
        nvs_get_u8(nvs, key, &auth);
        snprintf(key, sizeof(key), "wifi_%d_auth", i);
        nvs_set_u8(nvs, key, auth);
    }

    /* Erase last slot and decrement count */
    snprintf(key, sizeof(key), "wifi_%d_ssid", count - 1);
    nvs_erase_key(nvs, key);
    snprintf(key, sizeof(key), "wifi_%d_pass", count - 1);
    nvs_erase_key(nvs, key);
    snprintf(key, sizeof(key), "wifi_%d_auth", count - 1);
    nvs_erase_key(nvs, key);

    count--;
    nvs_set_u8(nvs, "wifi_count", count);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Deleted network: %s (remaining: %d)", ssid, count);
    return ESP_OK;
}

esp_err_t wifimgr_get_saved_networks(wifimgr_network_t *networks,
                                      uint8_t max_count, uint8_t *count)
{
    if (!networks || !count) return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi_creds", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *count = 0;
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    uint8_t stored = 0;
    nvs_get_u8(nvs, "wifi_count", &stored);

    uint8_t n = (stored < max_count) ? stored : max_count;
    char key[16];

    for (uint8_t i = 0; i < n; i++) {
        memset(&networks[i], 0, sizeof(wifimgr_network_t));
        size_t len = WIFIMGR_SSID_MAX_LEN;
        snprintf(key, sizeof(key), "wifi_%d_ssid", i);
        nvs_get_str(nvs, key, networks[i].ssid, &len);

        /* Don't return passwords in the getter */
        networks[i].password[0] = '\0';

        uint8_t auth = 0;
        snprintf(key, sizeof(key), "wifi_%d_auth", i);
        nvs_get_u8(nvs, key, &auth);
        networks[i].auth_mode = (wifi_auth_mode_t)auth;
    }

    *count = n;
    nvs_close(nvs);
    return ESP_OK;
}

/* ── Test connect ───────────────────────────────────────────── */
esp_err_t wifimgr_test_connect(const char *ssid, const char *password,
                                uint32_t timeout_ms)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    wifi_config_t wifi_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    if (password) {
        strncpy((char *)wifi_cfg.sta.password, password,
                sizeof(wifi_cfg.sta.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(timeout_ms ? timeout_ms : 15000));

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }

    esp_wifi_disconnect();
    return ESP_FAIL;
}

/* ── Force config mode ──────────────────────────────────────── */
esp_err_t wifimgr_enter_config_mode(void)
{
    s_config_mode_requested = true;
    xEventGroupSetBits(s_wifi_event_group, CONFIG_MODE_BIT);
    ESP_LOGI(TAG, "Config mode requested");
    return ESP_OK;
}

/* ── Wait for connection ────────────────────────────────────── */
esp_err_t wifimgr_wait_connected(uint32_t timeout_ms)
{
    TickType_t wait = timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, wait);

    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ── SoftAP management ──────────────────────────────────────── */
static esp_err_t start_softap(void)
{
    wifi_mode_t current;
    esp_wifi_get_mode(&current);

    wifi_mode_t target = (current == WIFI_MODE_STA || current == WIFI_MODE_APSTA)
                          ? WIFI_MODE_APSTA : WIFI_MODE_AP;

    ESP_ERROR_CHECK(esp_wifi_set_mode(target));

    wifi_config_t ap_cfg = {
        .ap = {
            .max_connection = s_config.ap_max_conn,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .channel = s_config.ap_channel,
        },
    };
    strncpy((char *)ap_cfg.ap.ssid, s_config.ap_ssid, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = strlen(s_config.ap_ssid);
    strncpy((char *)ap_cfg.ap.password, s_config.ap_password,
            sizeof(ap_cfg.ap.password) - 1);

    if (strlen(s_config.ap_password) < 8) {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

    if (current == WIFI_MODE_NULL) {
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    if (target == WIFI_MODE_APSTA) {
        s_state = WIFIMGR_STATE_APSTA_MODE;
        ESP_LOGI(TAG, "APSTA mode: AP '%s' + STA connected", s_config.ap_ssid);
    } else {
        s_state = WIFIMGR_STATE_AP_MODE;
        ESP_LOGI(TAG, "AP mode: '%s' on 192.168.4.1:%d",
                 s_config.ap_ssid, CONFIG_WIFIMGR_PORTAL_PORT);
    }

    return ESP_OK;
}

static esp_err_t stop_softap(void)
{
    wifi_mode_t current;
    esp_wifi_get_mode(&current);

    if (current == WIFI_MODE_APSTA) {
        esp_wifi_set_mode(WIFI_MODE_STA);
    } else if (current == WIFI_MODE_AP) {
        esp_wifi_set_mode(WIFI_MODE_STA);
    }

    return ESP_OK;
}

/* ── RSSI sort (descending) ─────────────────────────────────── */
static int compare_rssi(const void *a, const void *b)
{
    const wifi_ap_record_t *ra = a;
    const wifi_ap_record_t *rb = b;
    return rb->rssi - ra->rssi;   /* descending */
}

/* ── Try connecting to saved networks by RSSI ───────────────── */
static esp_err_t try_connect_saved(void)
{
    /* Get saved networks (with passwords from NVS) */
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi_creds", NVS_READONLY, &nvs);
    if (err != ESP_OK) return ESP_ERR_NOT_FOUND;

    uint8_t saved_count = 0;
    nvs_get_u8(nvs, "wifi_count", &saved_count);
    if (saved_count == 0) {
        nvs_close(nvs);
        return ESP_ERR_NOT_FOUND;
    }

    /* Load saved networks with passwords */
    wifimgr_network_t *saved = calloc(saved_count, sizeof(wifimgr_network_t));
    if (!saved) { nvs_close(nvs); return ESP_ERR_NO_MEM; }

    char key[16];
    for (uint8_t i = 0; i < saved_count; i++) {
        size_t len = WIFIMGR_SSID_MAX_LEN;
        snprintf(key, sizeof(key), "wifi_%d_ssid", i);
        nvs_get_str(nvs, key, saved[i].ssid, &len);
        len = WIFIMGR_PASS_MAX_LEN;
        snprintf(key, sizeof(key), "wifi_%d_pass", i);
        nvs_get_str(nvs, key, saved[i].password, &len);
        uint8_t auth = 0;
        snprintf(key, sizeof(key), "wifi_%d_auth", i);
        nvs_get_u8(nvs, key, &auth);
        saved[i].auth_mode = (wifi_auth_mode_t)auth;
    }
    nvs_close(nvs);

    /* Match scan results against saved, ordered by RSSI (already sorted) */
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);

    bool connected = false;
    for (uint16_t i = 0; i < s_ap_count && !connected; i++) {
        if (s_ap_records[i].rssi < s_config.scan_rssi_threshold) continue;

        for (uint8_t j = 0; j < saved_count; j++) {
            if (strcmp((char *)s_ap_records[i].ssid, saved[j].ssid) != 0) continue;

            ESP_LOGI(TAG, "Trying '%s' (RSSI %d, ch %d)",
                     saved[j].ssid, s_ap_records[i].rssi, s_ap_records[i].primary);

            xSemaphoreGive(s_scan_mutex);

            s_state = WIFIMGR_STATE_CONNECTING;
            err = wifimgr_test_connect(saved[j].ssid, saved[j].password, 10000);

            if (err == ESP_OK) {
                strncpy(s_connected_ssid, saved[j].ssid, WIFIMGR_SSID_MAX_LEN - 1);
                s_connected_ssid[WIFIMGR_SSID_MAX_LEN - 1] = '\0';

                /* Update RSSI */
                wifi_ap_record_t ap_info;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                    s_connected_rssi = ap_info.rssi;
                }
                connected = true;
            }

            xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
            break;
        }
    }

    xSemaphoreGive(s_scan_mutex);
    free(saved);

    return connected ? ESP_OK : ESP_FAIL;
}

/* ── Button polling (simple debounced long-press) ───────────── */
static void button_check(void)
{
    static uint32_t press_start = 0;
    static bool was_pressed = false;

    bool pressed = (gpio_get_level(s_config.button_gpio) == 0);

    if (pressed && !was_pressed) {
        press_start = xTaskGetTickCount();
    } else if (pressed && was_pressed) {
        uint32_t held_ms = (xTaskGetTickCount() - press_start) * portTICK_PERIOD_MS;
        if (held_ms >= s_config.button_long_press_ms) {
            ESP_LOGI(TAG, "BOOT button long-press detected -> config mode");
            wifimgr_enter_config_mode();
            press_start = xTaskGetTickCount(); /* prevent re-trigger */
        }
    }
    was_pressed = pressed;
}

/* ── UART '+++' pattern detection task ──────────────────────── */
static void uart_pattern_task(void *arg)
{
    uart_event_t event;
    QueueHandle_t uart_queue;

    uart_driver_install(UART_NUM_0, 256, 0, 8, &uart_queue, 0);

    for (;;) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_PATTERN_DET) {
                int pos = uart_pattern_pop_pos(UART_NUM_0);
                if (pos >= 0) {
                    /* Flush the pattern bytes */
                    uint8_t buf[64];
                    int to_read = pos + 3;
                    if (to_read > (int)sizeof(buf)) to_read = sizeof(buf);
                    uart_read_bytes(UART_NUM_0, buf, to_read, 0);

                    ESP_LOGI(TAG, "UART '+++' detected -> config mode");
                    wifimgr_enter_config_mode();
                }
            }
        }
    }
}

/* ── Main state machine task ────────────────────────────────── */
static void wifimgr_task(void *arg)
{
    /* Set STA mode and start WiFi */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_scan_cycle_count = 0;

    /* Check if any networks are saved */
    uint8_t saved_count = 0;
    {
        wifimgr_network_t tmp;
        wifimgr_get_saved_networks(&tmp, 1, &saved_count);
    }

    if (saved_count == 0) {
        ESP_LOGW(TAG, "No saved networks — entering AP mode");
        start_softap();
        wifimgr_portal_start();
        goto monitor_loop;
    }

    /* Scan-connect loop */
    while (s_state != WIFIMGR_STATE_CONNECTED &&
           s_state != WIFIMGR_STATE_AP_MODE) {

        /* Check config mode request */
        if (s_config_mode_requested) {
            s_config_mode_requested = false;
            start_softap();
            wifimgr_portal_start();
            break;
        }

        s_state = WIFIMGR_STATE_SCANNING;
        ESP_LOGI(TAG, "Scan cycle %d/%d", s_scan_cycle_count + 1,
                 s_config.scan_cycles_before_ap);

        esp_err_t err = wifimgr_scan();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Scan failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        err = try_connect_saved();
        if (err == ESP_OK) {
            s_state = WIFIMGR_STATE_CONNECTED;
            ESP_LOGI(TAG, "Connected to '%s' (%s)", s_connected_ssid, s_connected_ip);

            /* Start portal on STA interface too (accessible from LAN) */
            wifimgr_portal_start();

            wifimgr_notify_text("system", "boot",
                "ESP32-P4 SDR Online",
                "Connected to WiFi. Services starting.");
            break;
        }

        s_scan_cycle_count++;
        if (s_scan_cycle_count >= s_config.scan_cycles_before_ap) {
            ESP_LOGW(TAG, "All scan cycles exhausted — entering AP mode");
            start_softap();
            wifimgr_portal_start();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

monitor_loop:
    /* Monitor loop: handle reconnect, config mode triggers, RSSI updates */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* Check button */
        button_check();

        /* Handle config mode request while connected */
        if (s_config_mode_requested && s_state == WIFIMGR_STATE_CONNECTED) {
            s_config_mode_requested = false;
            start_softap();  /* enters APSTA */
            ESP_LOGI(TAG, "Config mode active (APSTA)");
        }

        /* Handle disconnection */
        if (s_state == WIFIMGR_STATE_CONNECTED) {
            EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
            if (bits & WIFI_FAIL_BIT) {
                ESP_LOGW(TAG, "Connection lost — rescanning");
                xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
                s_state = WIFIMGR_STATE_SCANNING;
                s_scan_cycle_count = 0;

                /* Quick rescan-reconnect loop */
                for (int i = 0; i < 3; i++) {
                    wifimgr_scan();
                    if (try_connect_saved() == ESP_OK) {
                        s_state = WIFIMGR_STATE_CONNECTED;
                        ESP_LOGI(TAG, "Reconnected to '%s'", s_connected_ssid);
                        wifimgr_notify_text("system", "wifi_reconnect",
                            "WiFi Reconnected", s_connected_ssid);
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }

                if (s_state != WIFIMGR_STATE_CONNECTED) {
                    ESP_LOGW(TAG, "Reconnect failed — entering AP mode");
                    start_softap();
                }
            } else {
                /* Periodic RSSI update */
                wifi_ap_record_t ap_info;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                    s_connected_rssi = ap_info.rssi;
                }
            }
        }
    }
}
