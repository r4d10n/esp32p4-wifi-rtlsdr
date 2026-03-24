#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi_types.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── State machine states ───────────────────────────────────── */
typedef enum {
    WIFIMGR_STATE_INIT = 0,
    WIFIMGR_STATE_SCANNING,
    WIFIMGR_STATE_CONNECTING,
    WIFIMGR_STATE_CONNECTED,
    WIFIMGR_STATE_AP_MODE,
    WIFIMGR_STATE_APSTA_MODE,
} wifimgr_state_t;

/* ── Saved network entry ────────────────────────────────────── */
#define WIFIMGR_SSID_MAX_LEN    33
#define WIFIMGR_PASS_MAX_LEN    65

typedef struct {
    char ssid[WIFIMGR_SSID_MAX_LEN];
    char password[WIFIMGR_PASS_MAX_LEN];
    wifi_auth_mode_t auth_mode;
} wifimgr_network_t;

/* ── Scan result entry ──────────────────────────────────────── */
typedef struct {
    char ssid[WIFIMGR_SSID_MAX_LEN];
    int8_t rssi;
    wifi_auth_mode_t auth_mode;
    uint8_t channel;
    bool saved;         /* matches a saved network */
} wifimgr_scan_result_t;

/* ── Status info ────────────────────────────────────────────── */
typedef struct {
    wifimgr_state_t state;
    char ssid[WIFIMGR_SSID_MAX_LEN];
    char ip[16];
    int8_t rssi;
    wifi_mode_t mode;
} wifimgr_status_t;

/* ── Configuration ──────────────────────────────────────────── */
typedef struct {
    char ap_ssid[WIFIMGR_SSID_MAX_LEN];
    char ap_password[WIFIMGR_PASS_MAX_LEN];
    uint8_t ap_channel;
    uint8_t ap_max_conn;
    int8_t scan_rssi_threshold;
    uint8_t scan_cycles_before_ap;
    int button_gpio;
    uint32_t button_long_press_ms;
} wifimgr_config_t;

#define WIFIMGR_CONFIG_DEFAULT() { \
    .ap_ssid = CONFIG_WIFIMGR_AP_SSID, \
    .ap_password = CONFIG_WIFIMGR_AP_PASSWORD, \
    .ap_channel = CONFIG_WIFIMGR_AP_CHANNEL, \
    .ap_max_conn = CONFIG_WIFIMGR_AP_MAX_CONN, \
    .scan_rssi_threshold = CONFIG_WIFIMGR_SCAN_RSSI_THRESHOLD, \
    .scan_cycles_before_ap = CONFIG_WIFIMGR_SCAN_CYCLES_BEFORE_AP, \
    .button_gpio = CONFIG_WIFIMGR_BUTTON_GPIO, \
    .button_long_press_ms = CONFIG_WIFIMGR_BUTTON_LONG_PRESS_MS, \
}

/* ── Public API ─────────────────────────────────────────────── */

/**
 * Initialize WiFi manager. Sets up WiFi, NVS, LittleFS, button, UART.
 * Call once from app_main() before any network services.
 */
esp_err_t wifimgr_init(const wifimgr_config_t *config);

/**
 * Start the WiFi manager state machine task.
 * Scans for saved networks or falls back to AP mode.
 */
esp_err_t wifimgr_start(void);

/**
 * Get current connection status.
 */
esp_err_t wifimgr_get_status(wifimgr_status_t *status);

/**
 * Get current state.
 */
wifimgr_state_t wifimgr_get_state(void);

/**
 * Trigger a WiFi scan. Results available via wifimgr_get_scan_results().
 */
esp_err_t wifimgr_scan(void);

/**
 * Get last scan results. Caller must free *results with free().
 * @param results  Output pointer to array
 * @param count    Output number of entries
 */
esp_err_t wifimgr_get_scan_results(wifimgr_scan_result_t **results, uint16_t *count);

/**
 * Save a WiFi network to NVS.
 */
esp_err_t wifimgr_save_network(const wifimgr_network_t *network);

/**
 * Delete a saved WiFi network by SSID.
 */
esp_err_t wifimgr_delete_network(const char *ssid);

/**
 * Get all saved networks. Passwords are not returned.
 * @param networks  Output array (caller provides, max_count entries)
 * @param max_count Size of networks array
 * @param count     Output: actual number of saved networks
 */
esp_err_t wifimgr_get_saved_networks(wifimgr_network_t *networks,
                                      uint8_t max_count, uint8_t *count);

/**
 * Test connection to a specific network. Blocking, returns after attempt.
 */
esp_err_t wifimgr_test_connect(const char *ssid, const char *password,
                                uint32_t timeout_ms);

/**
 * Force entry into AP/config mode (e.g., from UART or button callback).
 */
esp_err_t wifimgr_enter_config_mode(void);

/**
 * Block until connected or AP mode entered. Useful in app_main().
 * @param timeout_ms  Max wait time (0 = forever)
 * @return ESP_OK if connected, ESP_ERR_TIMEOUT if AP mode
 */
esp_err_t wifimgr_wait_connected(uint32_t timeout_ms);

/**
 * Get the STA netif handle (for IP info, etc.)
 */
esp_netif_t *wifimgr_get_sta_netif(void);

/**
 * Get the AP netif handle.
 */
esp_netif_t *wifimgr_get_ap_netif(void);

#ifdef __cplusplus
}
#endif
