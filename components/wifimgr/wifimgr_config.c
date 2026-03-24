/*
 * WiFi Manager — Config Store
 *
 * NVS (encrypted namespace) for WiFi credentials and API keys.
 * LittleFS for structured JSON config files.
 */

#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_littlefs.h"
#include "cJSON.h"

#include "wifimgr_config.h"

static const char *TAG = "wifimgr_cfg";

#define STORAGE_BASE_PATH   "/storage"
#define SDR_CONFIG_FILE     STORAGE_BASE_PATH "/sdr.json"
#define SERVICES_FILE       STORAGE_BASE_PATH "/services.json"
#define ETHERNET_FILE       STORAGE_BASE_PATH "/ethernet.json"
#define NOTIFY_FILE         STORAGE_BASE_PATH "/notify.json"
#define CHATBOT_FILE        STORAGE_BASE_PATH "/chatbot.json"

/* ── Helpers ────────────────────────────────────────────────── */

static char *read_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return NULL;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (sz <= 0 || sz > 32768) {
        fclose(f);
        return NULL;
    }

    char *buf = calloc(1, sz + 1);
    if (!buf) { fclose(f); return NULL; }
    fread(buf, 1, sz, f);
    fclose(f);
    return buf;
}

static esp_err_t write_file(const char *path, const char *data)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s for writing", path);
        return ESP_FAIL;
    }
    fputs(data, f);
    fclose(f);
    return ESP_OK;
}

static esp_err_t write_json_file(const char *path, const cJSON *json)
{
    char *str = cJSON_PrintUnformatted(json);
    if (!str) return ESP_ERR_NO_MEM;
    esp_err_t err = write_file(path, str);
    cJSON_free(str);
    return err;
}

static cJSON *read_json_file(const char *path)
{
    char *str = read_file(path);
    if (!str) return NULL;
    cJSON *json = cJSON_Parse(str);
    free(str);
    return json;
}

/* ── Initialization ─────────────────────────────────────────── */
esp_err_t wifimgr_config_init(void)
{
    /* Initialize NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Mount LittleFS */
    esp_vfs_littlefs_conf_t lfs_conf = {
        .base_path = STORAGE_BASE_PATH,
        .partition_label = CONFIG_WIFIMGR_LITTLEFS_PARTITION,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    err = esp_vfs_littlefs_register(&lfs_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(err));
        return err;
    }

    size_t total = 0, used = 0;
    esp_littlefs_info(CONFIG_WIFIMGR_LITTLEFS_PARTITION, &total, &used);
    ESP_LOGI(TAG, "LittleFS: %zu/%zu bytes used", used, total);

    return ESP_OK;
}

/* ── SDR Config ─────────────────────────────────────────────── */

esp_err_t wifimgr_config_load_sdr(sdr_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    /* Start with defaults */
    sdr_config_t def = SDR_CONFIG_DEFAULT();
    *config = def;

    cJSON *json = read_json_file(SDR_CONFIG_FILE);
    if (!json) return ESP_OK; /* use defaults */

    cJSON *item;
    if ((item = cJSON_GetObjectItem(json, "center_freq")))
        config->center_freq = (uint32_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "sample_rate")))
        config->sample_rate = (uint32_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "gain_mode")) && cJSON_IsString(item))
        strncpy(config->gain_mode, item->valuestring, sizeof(config->gain_mode) - 1);
    if ((item = cJSON_GetObjectItem(json, "tuner_gain_tenth_db")))
        config->tuner_gain_tenth_db = (uint16_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "rtl_agc")))
        config->rtl_agc = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "tuner_agc")))
        config->tuner_agc = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "ppm_correction")))
        config->ppm_correction = (int16_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "direct_sampling")) && cJSON_IsString(item))
        strncpy(config->direct_sampling, item->valuestring, sizeof(config->direct_sampling) - 1);
    if ((item = cJSON_GetObjectItem(json, "offset_tuning")))
        config->offset_tuning = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "offset_freq_hz")))
        config->offset_freq_hz = (int32_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "bias_tee")))
        config->bias_tee = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "dc_offset_correction")))
        config->dc_offset_correction = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "iq_imbalance_correction")))
        config->iq_imbalance_correction = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "invert_iq")))
        config->invert_iq = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "max_total_users")))
        config->max_total_users = (uint8_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "hostname")) && cJSON_IsString(item))
        strncpy(config->hostname, item->valuestring, sizeof(config->hostname) - 1);

    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t wifimgr_config_save_sdr(const sdr_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "center_freq", config->center_freq);
    cJSON_AddNumberToObject(json, "sample_rate", config->sample_rate);
    cJSON_AddStringToObject(json, "gain_mode", config->gain_mode);
    cJSON_AddNumberToObject(json, "tuner_gain_tenth_db", config->tuner_gain_tenth_db);
    cJSON_AddBoolToObject(json, "rtl_agc", config->rtl_agc);
    cJSON_AddBoolToObject(json, "tuner_agc", config->tuner_agc);
    cJSON_AddNumberToObject(json, "ppm_correction", config->ppm_correction);
    cJSON_AddStringToObject(json, "direct_sampling", config->direct_sampling);
    cJSON_AddBoolToObject(json, "offset_tuning", config->offset_tuning);
    cJSON_AddNumberToObject(json, "offset_freq_hz", config->offset_freq_hz);
    cJSON_AddBoolToObject(json, "bias_tee", config->bias_tee);
    cJSON_AddBoolToObject(json, "dc_offset_correction", config->dc_offset_correction);
    cJSON_AddBoolToObject(json, "iq_imbalance_correction", config->iq_imbalance_correction);
    cJSON_AddBoolToObject(json, "invert_iq", config->invert_iq);
    cJSON_AddNumberToObject(json, "max_total_users", config->max_total_users);
    cJSON_AddStringToObject(json, "hostname", config->hostname);

    esp_err_t err = write_json_file(SDR_CONFIG_FILE, json);
    cJSON_Delete(json);
    return err;
}

/* ── Ethernet Config ────────────────────────────────────────── */

esp_err_t wifimgr_config_load_ethernet(ethernet_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    ethernet_config_t def = ETHERNET_CONFIG_DEFAULT();
    *config = def;

    cJSON *json = read_json_file(ETHERNET_FILE);
    if (!json) return ESP_OK;

    cJSON *item;
    if ((item = cJSON_GetObjectItem(json, "enable")))
        config->enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "dhcp")))
        config->dhcp = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "static_ip")) && cJSON_IsString(item))
        strncpy(config->static_ip, item->valuestring, sizeof(config->static_ip) - 1);
    if ((item = cJSON_GetObjectItem(json, "static_mask")) && cJSON_IsString(item))
        strncpy(config->static_mask, item->valuestring, sizeof(config->static_mask) - 1);
    if ((item = cJSON_GetObjectItem(json, "static_gw")) && cJSON_IsString(item))
        strncpy(config->static_gw, item->valuestring, sizeof(config->static_gw) - 1);
    if ((item = cJSON_GetObjectItem(json, "static_dns")) && cJSON_IsString(item))
        strncpy(config->static_dns, item->valuestring, sizeof(config->static_dns) - 1);
    if ((item = cJSON_GetObjectItem(json, "phy_type")) && cJSON_IsString(item))
        strncpy(config->phy_type, item->valuestring, sizeof(config->phy_type) - 1);
    if ((item = cJSON_GetObjectItem(json, "phy_addr")))
        config->phy_addr = (int8_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "mdc_gpio")))
        config->mdc_gpio = (uint8_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "mdio_gpio")))
        config->mdio_gpio = (uint8_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "prefer_over_wifi")))
        config->prefer_over_wifi = cJSON_IsTrue(item);

    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t wifimgr_config_save_ethernet(const ethernet_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enable", config->enable);
    cJSON_AddBoolToObject(json, "dhcp", config->dhcp);
    cJSON_AddStringToObject(json, "static_ip", config->static_ip);
    cJSON_AddStringToObject(json, "static_mask", config->static_mask);
    cJSON_AddStringToObject(json, "static_gw", config->static_gw);
    cJSON_AddStringToObject(json, "static_dns", config->static_dns);
    cJSON_AddStringToObject(json, "phy_type", config->phy_type);
    cJSON_AddNumberToObject(json, "phy_addr", config->phy_addr);
    cJSON_AddNumberToObject(json, "mdc_gpio", config->mdc_gpio);
    cJSON_AddNumberToObject(json, "mdio_gpio", config->mdio_gpio);
    cJSON_AddBoolToObject(json, "prefer_over_wifi", config->prefer_over_wifi);

    esp_err_t err = write_json_file(ETHERNET_FILE, json);
    cJSON_Delete(json);
    return err;
}

/* ── Notification Config ────────────────────────────────────── */

esp_err_t wifimgr_config_load_notify(notify_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    notify_config_t def = NOTIFY_CONFIG_DEFAULT();
    *config = def;

    cJSON *json = read_json_file(NOTIFY_FILE);
    if (!json) return ESP_OK;

    cJSON *tg = cJSON_GetObjectItem(json, "telegram");
    if (tg) {
        cJSON *item;
        if ((item = cJSON_GetObjectItem(tg, "enable")))
            config->telegram.enable = cJSON_IsTrue(item);
        if ((item = cJSON_GetObjectItem(tg, "rate_limit_s")))
            config->telegram.rate_limit_s = (uint16_t)cJSON_GetNumberValue(item);
        /* bot_token and chat_id loaded from NVS */
    }

    cJSON *dc = cJSON_GetObjectItem(json, "discord");
    if (dc) {
        cJSON *item;
        if ((item = cJSON_GetObjectItem(dc, "enable")))
            config->discord.enable = cJSON_IsTrue(item);
        if ((item = cJSON_GetObjectItem(dc, "rate_limit_s")))
            config->discord.rate_limit_s = (uint16_t)cJSON_GetNumberValue(item);
        /* webhook_url loaded from NVS */
    }

    cJSON_Delete(json);

    /* Load sensitive tokens from NVS */
    wifimgr_config_get_api_key("tg_bot_token", config->telegram.bot_token,
                                sizeof(config->telegram.bot_token));
    wifimgr_config_get_api_key("tg_chat_id", config->telegram.chat_id,
                                sizeof(config->telegram.chat_id));
    wifimgr_config_get_api_key("dc_webhook_url", config->discord.webhook_url,
                                sizeof(config->discord.webhook_url));

    return ESP_OK;
}

esp_err_t wifimgr_config_save_notify(const notify_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    /* Save non-sensitive fields to JSON */
    cJSON *json = cJSON_CreateObject();

    cJSON *tg = cJSON_AddObjectToObject(json, "telegram");
    cJSON_AddBoolToObject(tg, "enable", config->telegram.enable);
    cJSON_AddNumberToObject(tg, "rate_limit_s", config->telegram.rate_limit_s);

    cJSON *dc = cJSON_AddObjectToObject(json, "discord");
    cJSON_AddBoolToObject(dc, "enable", config->discord.enable);
    cJSON_AddNumberToObject(dc, "rate_limit_s", config->discord.rate_limit_s);

    esp_err_t err = write_json_file(NOTIFY_FILE, json);
    cJSON_Delete(json);

    /* Save sensitive tokens to NVS */
    if (config->telegram.bot_token[0])
        wifimgr_config_set_api_key("tg_bot_token", config->telegram.bot_token);
    if (config->telegram.chat_id[0])
        wifimgr_config_set_api_key("tg_chat_id", config->telegram.chat_id);
    if (config->discord.webhook_url[0])
        wifimgr_config_set_api_key("dc_webhook_url", config->discord.webhook_url);

    return err;
}

/* ── Chatbot Config ─────────────────────────────────────────── */

esp_err_t wifimgr_config_load_chatbot(chatbot_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    chatbot_config_t def = CHATBOT_CONFIG_DEFAULT();
    *config = def;

    cJSON *json = read_json_file(CHATBOT_FILE);
    if (!json) return ESP_OK;

    cJSON *item;
    if ((item = cJSON_GetObjectItem(json, "enable")))
        config->enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "provider")) && cJSON_IsString(item))
        strncpy(config->provider, item->valuestring, sizeof(config->provider) - 1);
    if ((item = cJSON_GetObjectItem(json, "model")) && cJSON_IsString(item))
        strncpy(config->model, item->valuestring, sizeof(config->model) - 1);
    if ((item = cJSON_GetObjectItem(json, "web_enable")))
        config->web_enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "telegram_enable")))
        config->telegram_enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "discord_enable")))
        config->discord_enable = cJSON_IsTrue(item);
    if ((item = cJSON_GetObjectItem(json, "max_history")))
        config->max_history = (uint8_t)cJSON_GetNumberValue(item);
    if ((item = cJSON_GetObjectItem(json, "allowed_tools")) && cJSON_IsString(item))
        strncpy(config->allowed_tools, item->valuestring, sizeof(config->allowed_tools) - 1);

    cJSON_Delete(json);

    /* API key from NVS */
    wifimgr_config_get_api_key("llm_api_key", config->api_key, sizeof(config->api_key));

    return ESP_OK;
}

esp_err_t wifimgr_config_save_chatbot(const chatbot_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enable", config->enable);
    cJSON_AddStringToObject(json, "provider", config->provider);
    cJSON_AddStringToObject(json, "model", config->model);
    cJSON_AddBoolToObject(json, "web_enable", config->web_enable);
    cJSON_AddBoolToObject(json, "telegram_enable", config->telegram_enable);
    cJSON_AddBoolToObject(json, "discord_enable", config->discord_enable);
    cJSON_AddNumberToObject(json, "max_history", config->max_history);
    cJSON_AddStringToObject(json, "allowed_tools", config->allowed_tools);

    esp_err_t err = write_json_file(CHATBOT_FILE, json);
    cJSON_Delete(json);

    if (config->api_key[0])
        wifimgr_config_set_api_key("llm_api_key", config->api_key);

    return err;
}

/* ── Services Config ────────────────────────────────────────── */

static cJSON *create_default_services(void)
{
    cJSON *root = cJSON_CreateObject();

    /* RTL-TCP */
    cJSON *svc = cJSON_AddObjectToObject(root, "rtl_tcp");
    cJSON_AddBoolToObject(svc, "enable", true);
    cJSON_AddNumberToObject(svc, "port", 1234);
    cJSON_AddStringToObject(svc, "bind_addr", "0.0.0.0");
    cJSON_AddNumberToObject(svc, "max_clients", 1);

    /* RTL-UDP */
    svc = cJSON_AddObjectToObject(root, "rtl_udp");
    cJSON_AddBoolToObject(svc, "enable", true);
    cJSON_AddNumberToObject(svc, "port", 1235);
    cJSON_AddNumberToObject(svc, "payload_size", 1024);
    cJSON_AddBoolToObject(svc, "multicast_enable", false);
    cJSON_AddStringToObject(svc, "multicast_group", "239.1.2.3");
    cJSON_AddNumberToObject(svc, "multicast_port", 1236);

    /* SpyServer (stub) */
    svc = cJSON_AddObjectToObject(root, "spyserver");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON_AddNumberToObject(svc, "port", 5555);
    cJSON_AddNumberToObject(svc, "max_clients", 5);
    cJSON_AddBoolToObject(svc, "allow_control", true);
    cJSON_AddNumberToObject(svc, "fft_fps", 15);
    cJSON_AddNumberToObject(svc, "fft_bin_bits", 16);
    cJSON_AddBoolToObject(svc, "force_8bit", true);
    cJSON_AddNumberToObject(svc, "buffer_size_ms", 50);
    cJSON_AddNumberToObject(svc, "session_timeout_min", 0);
    cJSON_AddStringToObject(svc, "owner_name", "");
    cJSON_AddStringToObject(svc, "antenna_type", "");
    cJSON_AddStringToObject(svc, "antenna_location", "");

    /* SoapySDR Remote (stub) */
    svc = cJSON_AddObjectToObject(root, "soapysdr");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON_AddNumberToObject(svc, "port", 55132);
    cJSON_AddStringToObject(svc, "protocol", "udp");
    cJSON_AddStringToObject(svc, "format", "CS8");
    cJSON_AddNumberToObject(svc, "mtu", 1500);
    cJSON_AddBoolToObject(svc, "announce_mdns", true);

    /* WebSDR */
    svc = cJSON_AddObjectToObject(root, "websdr");
    cJSON_AddBoolToObject(svc, "enable", true);
    cJSON_AddNumberToObject(svc, "port", 8080);
    cJSON_AddNumberToObject(svc, "max_clients", 3);
    cJSON_AddNumberToObject(svc, "fft_rate_hz", 20);
    cJSON_AddBoolToObject(svc, "tls_enable", true);

    /* ADS-B (stub) */
    svc = cJSON_AddObjectToObject(root, "adsb");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON_AddNumberToObject(svc, "freq_hz", 1090000000);
    cJSON_AddStringToObject(svc, "output_format", "json");
    cJSON_AddBoolToObject(svc, "feed_flightaware", false);
    cJSON_AddBoolToObject(svc, "feed_fr24", false);
    cJSON_AddStringToObject(svc, "feed_host", "");
    cJSON_AddNumberToObject(svc, "feed_port", 30005);
    cJSON_AddNumberToObject(svc, "max_range_nm", 250);

    /* AIS (stub) */
    svc = cJSON_AddObjectToObject(root, "ais");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON_AddNumberToObject(svc, "freq1_hz", 161975000);
    cJSON_AddNumberToObject(svc, "freq2_hz", 162025000);
    cJSON_AddStringToObject(svc, "output_format", "nmea");
    cJSON_AddStringToObject(svc, "forward_host", "");
    cJSON_AddNumberToObject(svc, "forward_port", 10110);

    /* APRS (stub) */
    svc = cJSON_AddObjectToObject(root, "aprs");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON_AddNumberToObject(svc, "freq_hz", 144390000);
    cJSON_AddStringToObject(svc, "callsign", "");
    cJSON_AddNumberToObject(svc, "ssid", 10);
    cJSON_AddBoolToObject(svc, "igate_enable", false);
    cJSON_AddStringToObject(svc, "aprs_is_server", "rotate.aprs2.net");
    cJSON_AddNumberToObject(svc, "aprs_is_port", 14580);
    cJSON_AddStringToObject(svc, "passcode", "");

    /* GSM Scanner (stub) */
    svc = cJSON_AddObjectToObject(root, "gsm_scanner");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON *bands = cJSON_AddArrayToObject(svc, "scan_bands");
    cJSON_AddItemToArray(bands, cJSON_CreateString("GSM900"));
    cJSON_AddItemToArray(bands, cJSON_CreateString("DCS1800"));
    cJSON_AddNumberToObject(svc, "dwell_time_ms", 200);
    cJSON_AddBoolToObject(svc, "nco_lookup", true);
    cJSON_AddNumberToObject(svc, "report_interval_s", 300);

    /* FT8/WSPR (stub) */
    svc = cJSON_AddObjectToObject(root, "ft8_wspr");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON_AddStringToObject(svc, "mode", "FT8");
    cJSON_AddStringToObject(svc, "band", "20m");
    cJSON_AddNumberToObject(svc, "freq_hz", 14074000);
    cJSON_AddStringToObject(svc, "callsign", "");
    cJSON_AddStringToObject(svc, "grid_locator", "");
    cJSON_AddBoolToObject(svc, "upload_pskreporter", false);
    cJSON_AddBoolToObject(svc, "upload_wsprnet", false);

    /* FM Player */
    svc = cJSON_AddObjectToObject(root, "fm_player");
    cJSON_AddBoolToObject(svc, "enable", true);
    cJSON_AddNumberToObject(svc, "default_freq_hz", 100000000);
    cJSON_AddNumberToObject(svc, "de_emphasis_us", 75);
    cJSON_AddBoolToObject(svc, "stereo", false);
    cJSON_AddNumberToObject(svc, "wbfm_bandwidth_hz", 150000);
    cJSON_AddNumberToObject(svc, "nbfm_bandwidth_hz", 12500);

    /* rtl_433 (stub) */
    svc = cJSON_AddObjectToObject(root, "rtl_433");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON *freqs = cJSON_AddArrayToObject(svc, "frequency");
    cJSON_AddItemToArray(freqs, cJSON_CreateNumber(433920000));
    cJSON_AddItemToArray(freqs, cJSON_CreateNumber(315000000));
    cJSON_AddNumberToObject(svc, "hop_interval_s", 600);
    cJSON_AddNumberToObject(svc, "sample_rate", 250000);
    cJSON *protos = cJSON_AddArrayToObject(svc, "protocols");
    (void)protos; /* empty = all */
    cJSON_AddStringToObject(svc, "output_format", "json");
    cJSON_AddBoolToObject(svc, "mqtt_enable", false);
    cJSON_AddStringToObject(svc, "mqtt_host", "");
    cJSON_AddNumberToObject(svc, "mqtt_port", 1883);
    cJSON_AddStringToObject(svc, "mqtt_topic", "rtl_433/{model}/{id}");
    cJSON_AddNumberToObject(svc, "signal_level_threshold_db", -10);

    /* rtl_power Spectrum Monitor (stub) */
    svc = cJSON_AddObjectToObject(root, "rtl_power");
    cJSON_AddBoolToObject(svc, "enable", false);
    cJSON_AddNumberToObject(svc, "freq_start_hz", 88000000);
    cJSON_AddNumberToObject(svc, "freq_stop_hz", 108000000);
    cJSON_AddNumberToObject(svc, "bin_size_hz", 10000);
    cJSON_AddNumberToObject(svc, "interval_s", 60);
    cJSON_AddStringToObject(svc, "window_func", "hamming");
    cJSON_AddNumberToObject(svc, "crop_percent", 0);
    cJSON_AddStringToObject(svc, "output_format", "csv");
    cJSON_AddBoolToObject(svc, "cloud_upload_enable", false);
    cJSON_AddStringToObject(svc, "cloud_provider", "google_drive");
    cJSON_AddStringToObject(svc, "cloud_path", "/rtl_power/");
    cJSON_AddStringToObject(svc, "cloud_auth_token", "");
    cJSON_AddNumberToObject(svc, "upload_interval_min", 60);
    cJSON_AddBoolToObject(svc, "generate_spectrogram_png", true);

    return root;
}

cJSON *wifimgr_config_load_services(void)
{
    cJSON *json = read_json_file(SERVICES_FILE);
    if (json) return json;
    return create_default_services();
}

esp_err_t wifimgr_config_save_services(const cJSON *services)
{
    if (!services) return ESP_ERR_INVALID_ARG;
    return write_json_file(SERVICES_FILE, services);
}

cJSON *wifimgr_config_get_service(cJSON *services, const char *name)
{
    if (!services || !name) return NULL;
    return cJSON_GetObjectItem(services, name);
}

/* ── Backup / Restore / Factory Reset ───────────────────────── */

cJSON *wifimgr_config_export_all(void)
{
    cJSON *backup = cJSON_CreateObject();

    /* SDR */
    sdr_config_t sdr;
    wifimgr_config_load_sdr(&sdr);
    cJSON *sdr_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(sdr_json, "center_freq", sdr.center_freq);
    cJSON_AddNumberToObject(sdr_json, "sample_rate", sdr.sample_rate);
    cJSON_AddStringToObject(sdr_json, "gain_mode", sdr.gain_mode);
    cJSON_AddNumberToObject(sdr_json, "tuner_gain_tenth_db", sdr.tuner_gain_tenth_db);
    cJSON_AddNumberToObject(sdr_json, "ppm_correction", sdr.ppm_correction);
    cJSON_AddStringToObject(sdr_json, "hostname", sdr.hostname);
    cJSON_AddItemToObject(backup, "sdr", sdr_json);

    /* Services */
    cJSON *services = wifimgr_config_load_services();
    cJSON_AddItemToObject(backup, "services", services);

    /* Ethernet */
    ethernet_config_t eth;
    wifimgr_config_load_ethernet(&eth);
    cJSON *eth_json = read_json_file(ETHERNET_FILE);
    if (eth_json) cJSON_AddItemToObject(backup, "ethernet", eth_json);

    /* Notifications (non-sensitive) */
    cJSON *notify_json = read_json_file(NOTIFY_FILE);
    if (notify_json) cJSON_AddItemToObject(backup, "notify", notify_json);

    /* Chatbot (non-sensitive) */
    cJSON *chat_json = read_json_file(CHATBOT_FILE);
    if (chat_json) cJSON_AddItemToObject(backup, "chatbot", chat_json);

    return backup;
}

esp_err_t wifimgr_config_import_all(const cJSON *backup)
{
    if (!backup) return ESP_ERR_INVALID_ARG;

    cJSON *sdr = cJSON_GetObjectItem(backup, "sdr");
    if (sdr) write_json_file(SDR_CONFIG_FILE, sdr);

    cJSON *services = cJSON_GetObjectItem(backup, "services");
    if (services) write_json_file(SERVICES_FILE, services);

    cJSON *eth = cJSON_GetObjectItem(backup, "ethernet");
    if (eth) write_json_file(ETHERNET_FILE, eth);

    cJSON *notify = cJSON_GetObjectItem(backup, "notify");
    if (notify) write_json_file(NOTIFY_FILE, notify);

    cJSON *chat = cJSON_GetObjectItem(backup, "chatbot");
    if (chat) write_json_file(CHATBOT_FILE, chat);

    ESP_LOGI(TAG, "Config imported from backup");
    return ESP_OK;
}

esp_err_t wifimgr_config_factory_reset(void)
{
    ESP_LOGW(TAG, "FACTORY RESET — erasing all config");

    /* Erase NVS namespaces */
    nvs_handle_t nvs;
    if (nvs_open("wifi_creds", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (nvs_open("api_keys", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    /* Delete LittleFS files */
    remove(SDR_CONFIG_FILE);
    remove(SERVICES_FILE);
    remove(ETHERNET_FILE);
    remove(NOTIFY_FILE);
    remove(CHATBOT_FILE);

    ESP_LOGW(TAG, "Factory reset complete. Reboot to apply.");
    return ESP_OK;
}

/* ── NVS API key helpers ────────────────────────────────────── */

esp_err_t wifimgr_config_set_api_key(const char *key_name, const char *value)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("api_keys", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_str(nvs, key_name, value);
    nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

esp_err_t wifimgr_config_get_api_key(const char *key_name, char *buf, size_t buf_len)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("api_keys", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        buf[0] = '\0';
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
    }

    size_t len = buf_len;
    err = nvs_get_str(nvs, key_name, buf, &len);
    if (err != ESP_OK) buf[0] = '\0';
    nvs_close(nvs);
    return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
}
