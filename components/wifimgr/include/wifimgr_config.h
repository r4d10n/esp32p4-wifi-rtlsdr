#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── SDR configuration ──────────────────────────────────────── */
typedef struct {
    uint32_t center_freq;
    uint32_t sample_rate;
    char gain_mode[8];              /* "auto" or "manual" */
    uint16_t tuner_gain_tenth_db;
    bool rtl_agc;
    bool tuner_agc;
    int16_t ppm_correction;
    char direct_sampling[8];        /* "off", "I-ADC", "Q-ADC" */
    bool offset_tuning;
    int32_t offset_freq_hz;
    bool bias_tee;
    bool dc_offset_correction;
    bool iq_imbalance_correction;
    bool invert_iq;
    uint8_t max_total_users;
    char hostname[33];
} sdr_config_t;

#define SDR_CONFIG_DEFAULT() { \
    .center_freq = 100000000, \
    .sample_rate = 250000, \
    .gain_mode = "auto", \
    .tuner_gain_tenth_db = 0, \
    .rtl_agc = false, \
    .tuner_agc = false, \
    .ppm_correction = 0, \
    .direct_sampling = "off", \
    .offset_tuning = false, \
    .offset_freq_hz = 0, \
    .bias_tee = false, \
    .dc_offset_correction = true, \
    .iq_imbalance_correction = false, \
    .invert_iq = false, \
    .max_total_users = 5, \
    .hostname = "esp32p4-rtlsdr", \
}

/* ── Ethernet configuration ─────────────────────────────────── */
typedef struct {
    bool enable;
    bool dhcp;
    char static_ip[16];
    char static_mask[16];
    char static_gw[16];
    char static_dns[16];
    char phy_type[8];               /* "IP101", "LAN8720", "RTL8201" */
    int8_t phy_addr;
    uint8_t mdc_gpio;
    uint8_t mdio_gpio;
    bool prefer_over_wifi;
} ethernet_config_t;

#define ETHERNET_CONFIG_DEFAULT() { \
    .enable = false, \
    .dhcp = true, \
    .static_ip = "192.168.1.100", \
    .static_mask = "255.255.255.0", \
    .static_gw = "192.168.1.1", \
    .static_dns = "8.8.8.8", \
    .phy_type = "IP101", \
    .phy_addr = -1, \
    .mdc_gpio = 31, \
    .mdio_gpio = 27, \
    .prefer_over_wifi = true, \
}

/* ── Service notification config (embedded in each service) ── */
typedef struct {
    bool telegram;
    bool discord;
    char events[256];               /* comma-separated event names */
    uint16_t throttle_s;
} service_notify_config_t;

/* ── Per-service config (generic, stored as JSON) ───────────── */
/* Individual services are accessed as cJSON objects from services.json.
   We only define the common wrapper here. */
typedef struct {
    bool enable;
    uint16_t port;
    /* Extended fields are service-specific, accessed via cJSON */
} service_common_t;

/* ── Notification channel config ────────────────────────────── */
typedef struct {
    bool enable;
    char bot_token[72];
    char chat_id[24];
    uint16_t rate_limit_s;
} telegram_config_t;

typedef struct {
    bool enable;
    char webhook_url[260];
    uint16_t rate_limit_s;
} discord_config_t;

typedef struct {
    telegram_config_t telegram;
    discord_config_t discord;
} notify_config_t;

#define NOTIFY_CONFIG_DEFAULT() { \
    .telegram = { .enable = false, .bot_token = "", .chat_id = "", .rate_limit_s = 60 }, \
    .discord = { .enable = false, .webhook_url = "", .rate_limit_s = 60 }, \
}

/* ── Chatbot config (Phase 2) ───────────────────────────────── */
typedef struct {
    bool enable;
    char provider[16];              /* "gemini", "openai", "claude" */
    char api_key[132];
    char model[64];
    bool web_enable;
    bool telegram_enable;
    bool discord_enable;
    uint8_t max_history;
    char allowed_tools[32];         /* "query" or "query,control" */
} chatbot_config_t;

#define CHATBOT_CONFIG_DEFAULT() { \
    .enable = false, \
    .provider = "gemini", \
    .api_key = "", \
    .model = "gemini-2.0-flash-lite", \
    .web_enable = true, \
    .telegram_enable = false, \
    .discord_enable = false, \
    .max_history = 10, \
    .allowed_tools = "query", \
}

/* ── Config store API ───────────────────────────────────────── */

/** Initialize config store (NVS + LittleFS mount). Call once. */
esp_err_t wifimgr_config_init(void);

/** Load SDR config from LittleFS. Returns defaults if file missing. */
esp_err_t wifimgr_config_load_sdr(sdr_config_t *config);

/** Save SDR config to LittleFS. */
esp_err_t wifimgr_config_save_sdr(const sdr_config_t *config);

/** Load Ethernet config. */
esp_err_t wifimgr_config_load_ethernet(ethernet_config_t *config);

/** Save Ethernet config. */
esp_err_t wifimgr_config_save_ethernet(const ethernet_config_t *config);

/** Load notification config. */
esp_err_t wifimgr_config_load_notify(notify_config_t *config);

/** Save notification config. */
esp_err_t wifimgr_config_save_notify(const notify_config_t *config);

/** Load chatbot config. */
esp_err_t wifimgr_config_load_chatbot(chatbot_config_t *config);

/** Save chatbot config. */
esp_err_t wifimgr_config_save_chatbot(const chatbot_config_t *config);

/**
 * Load full services.json as cJSON tree. Caller must cJSON_Delete().
 * Returns default services config if file missing.
 */
cJSON *wifimgr_config_load_services(void);

/** Save services cJSON tree to LittleFS. */
esp_err_t wifimgr_config_save_services(const cJSON *services);

/**
 * Get a specific service config as cJSON object.
 * Returns borrowed pointer (do not free), or NULL if not found.
 */
cJSON *wifimgr_config_get_service(cJSON *services, const char *name);

/** Export all config as single JSON (for backup). Caller must cJSON_Delete(). */
cJSON *wifimgr_config_export_all(void);

/** Import config from JSON backup. Validates before applying. */
esp_err_t wifimgr_config_import_all(const cJSON *backup);

/** Factory reset: erase NVS wifi_creds + api_keys, delete LittleFS files. */
esp_err_t wifimgr_config_factory_reset(void);

/* ── NVS helpers for sensitive data ─────────────────────────── */

/** Store API key in encrypted NVS. */
esp_err_t wifimgr_config_set_api_key(const char *key_name, const char *value);

/** Read API key from encrypted NVS. buf_len includes null terminator. */
esp_err_t wifimgr_config_get_api_key(const char *key_name, char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif
