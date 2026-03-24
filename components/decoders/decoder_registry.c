#include <string.h>
#include "esp_log.h"
#include "decoder_framework.h"

static const char *TAG = "decoder_reg";
static decoder_plugin_t *s_head = NULL;
static int s_count = 0;

esp_err_t decoder_registry_add(decoder_plugin_t *plugin) {
    if (!plugin || !plugin->name) return ESP_ERR_INVALID_ARG;

    /* Check for duplicate */
    decoder_plugin_t *p = s_head;
    while (p) {
        if (strcmp(p->name, plugin->name) == 0) {
            ESP_LOGW(TAG, "Plugin '%s' already registered", plugin->name);
            return ESP_ERR_INVALID_STATE;
        }
        p = p->next;
    }

    plugin->next = s_head;
    s_head = plugin;
    s_count++;
    plugin->enabled = false;
    plugin->running = false;

    ESP_LOGI(TAG, "Registered: %s (%s) [%s]",
             plugin->name, plugin->description, plugin->category);
    return ESP_OK;
}

decoder_plugin_t *decoder_registry_find(const char *name) {
    decoder_plugin_t *p = s_head;
    while (p) {
        if (strcmp(p->name, name) == 0) return p;
        p = p->next;
    }
    return NULL;
}

decoder_plugin_t *decoder_registry_first(void) { return s_head; }

decoder_plugin_t *decoder_registry_next(decoder_plugin_t *current) {
    return current ? current->next : NULL;
}

int decoder_registry_count(void) { return s_count; }

/* Forward declarations for builtin decoders */
extern void register_adsb_decoder(void);
extern void register_ais_decoder(void);
extern void register_gsm_decoder(void);
extern void register_ft8_decoder(void);
extern void register_ax25_decoders(void);
extern void register_pocsag_decoders(void);
extern void register_dtmf_decoder(void);
extern void register_ctcss_decoder(void);
extern void register_cw_decoder(void);

esp_err_t decoder_registry_init(void) {
    ESP_LOGI(TAG, "Initializing decoder registry...");

    /* Initialize subsystems */
    decode_bus_init();
    decoder_channel_manager_init();

    /* Register all built-in decoders */
    register_adsb_decoder();
    register_ais_decoder();
    register_gsm_decoder();
    register_ft8_decoder();
    register_ax25_decoders();
    register_pocsag_decoders();
    register_dtmf_decoder();
    register_ctcss_decoder();
    register_cw_decoder();

    ESP_LOGI(TAG, "Registry initialized: %d decoders registered", s_count);
    return ESP_OK;
}
