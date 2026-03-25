#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_settings.h"

static const char *TAG = "nvs_set";
#define NVS_NAMESPACE "fm_radio"

/* Debounce: track last save time */
static int64_t s_last_save_time = 0;
#define SAVE_DEBOUNCE_US 1000000  /* 1 second */

esp_err_t nvs_settings_init(void)
{
    /* NVS already initialized in app_main, just verify */
    ESP_LOGI(TAG, "NVS settings initialized (namespace: %s)", NVS_NAMESPACE);
    return ESP_OK;
}

esp_err_t nvs_settings_load(nvs_radio_settings_t *s)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved settings found, using defaults");
        return ESP_ERR_NVS_NOT_FOUND;
    }

    nvs_get_u32(h, "freq", &s->frequency);
    nvs_get_i32(h, "gain", (int32_t*)&s->gain);
    nvs_get_u8(h, "volume", &s->volume);
    int32_t mode_tmp = 0;
    nvs_get_i32(h, "mode", &mode_tmp);
    s->mode = (int)mode_tmp;
    nvs_get_u32(h, "filter_bw", &s->filter_bw);
    nvs_get_u8(h, "squelch", &s->squelch);
    uint8_t nb_en = 0;
    nvs_get_u8(h, "nb_en", &nb_en);
    s->nb_enabled = nb_en;
    nvs_get_u8(h, "nb_thresh", &s->nb_threshold);

    nvs_close(h);
    ESP_LOGI(TAG, "Settings loaded: freq=%lu mode=%d vol=%d",
             (unsigned long)s->frequency, s->mode, s->volume);
    return ESP_OK;
}

esp_err_t nvs_settings_save(const nvs_radio_settings_t *s)
{
    /* Debounce: don't write more than once per second */
    int64_t now = esp_timer_get_time();
    if (now - s_last_save_time < SAVE_DEBOUNCE_US) {
        return ESP_OK; /* Silently skip */
    }
    s_last_save_time = now;

    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    nvs_set_u32(h, "freq", s->frequency);
    nvs_set_i32(h, "gain", s->gain);
    nvs_set_u8(h, "volume", s->volume);
    nvs_set_i32(h, "mode", s->mode);
    nvs_set_u32(h, "filter_bw", s->filter_bw);
    nvs_set_u8(h, "squelch", s->squelch);
    nvs_set_u8(h, "nb_en", s->nb_enabled ? 1 : 0);
    nvs_set_u8(h, "nb_thresh", s->nb_threshold);

    ret = nvs_commit(h);
    nvs_close(h);

    ESP_LOGD(TAG, "Settings saved");
    return ret;
}

esp_err_t nvs_settings_reset(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Settings reset to factory defaults");
    return ret;
}
