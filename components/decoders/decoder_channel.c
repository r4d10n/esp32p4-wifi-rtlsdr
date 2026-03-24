#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "decode_chan";

static decoder_channel_t s_channels[DECODER_MAX_CHANNELS];
static int s_channel_count = 0;
static SemaphoreHandle_t s_mutex;

esp_err_t decoder_channel_manager_init(void) {
    memset(s_channels, 0, sizeof(s_channels));
    s_channel_count = 0;
    s_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Channel manager initialized (%d max channels)", DECODER_MAX_CHANNELS);
    return ESP_OK;
}

int decoder_channel_manager_add(decoder_plugin_t *plugin, uint32_t center_freq_hz) {
    if (!plugin) return -1;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Check if an existing channel matches this plugin's requirements */
    for (int i = 0; i < s_channel_count; i++) {
        if (s_channels[i].active &&
            s_channels[i].demod_type == plugin->demod_type &&
            s_channels[i].center_freq_hz == center_freq_hz &&
            s_channels[i].bandwidth_hz == plugin->bandwidth_hz &&
            s_channels[i].plugin_count < 8) {
            /* Attach to existing channel */
            s_channels[i].plugins[s_channels[i].plugin_count++] = plugin;
            ESP_LOGI(TAG, "Plugin '%s' attached to existing channel %d (%d plugins)",
                     plugin->name, i, s_channels[i].plugin_count);
            xSemaphoreGive(s_mutex);
            return i;
        }
    }

    /* Create new channel */
    if (s_channel_count >= DECODER_MAX_CHANNELS) {
        ESP_LOGE(TAG, "No free channels for '%s'", plugin->name);
        xSemaphoreGive(s_mutex);
        return -1;
    }

    int idx = s_channel_count++;
    decoder_channel_t *ch = &s_channels[idx];
    ch->center_freq_hz = center_freq_hz;
    ch->offset_hz = (int32_t)(plugin->center_freq_hz) - (int32_t)center_freq_hz;
    ch->bandwidth_hz = plugin->bandwidth_hz;
    ch->demod_type = plugin->demod_type;
    ch->audio_rate_hz = plugin->audio_rate_hz;
    ch->plugins[0] = plugin;
    ch->plugin_count = 1;
    ch->active = true;

    ESP_LOGI(TAG, "Channel %d created for '%s' (offset=%d, bw=%lu, demod=%d)",
             idx, plugin->name, (int)ch->offset_hz,
             (unsigned long)ch->bandwidth_hz, ch->demod_type);

    xSemaphoreGive(s_mutex);
    return idx;
}

esp_err_t decoder_channel_manager_remove(decoder_plugin_t *plugin) {
    if (!plugin) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (int i = 0; i < s_channel_count; i++) {
        for (int j = 0; j < s_channels[i].plugin_count; j++) {
            if (s_channels[i].plugins[j] == plugin) {
                /* Remove plugin from channel */
                for (int k = j; k < s_channels[i].plugin_count - 1; k++) {
                    s_channels[i].plugins[k] = s_channels[i].plugins[k + 1];
                }
                s_channels[i].plugin_count--;

                if (s_channels[i].plugin_count == 0) {
                    s_channels[i].active = false;
                    ESP_LOGI(TAG, "Channel %d deactivated (no plugins)", i);
                }

                xSemaphoreGive(s_mutex);
                return ESP_OK;
            }
        }
    }

    xSemaphoreGive(s_mutex);
    return ESP_ERR_NOT_FOUND;
}

void decoder_channel_manager_push_iq(const uint8_t *iq_data, uint32_t len,
                                      uint32_t sample_rate) {
    /* Fast path: no lock needed for reads if channels are append-only during streaming */
    for (int i = 0; i < s_channel_count; i++) {
        decoder_channel_t *ch = &s_channels[i];
        if (!ch->active) continue;

        for (int j = 0; j < ch->plugin_count; j++) {
            decoder_plugin_t *p = ch->plugins[j];
            if (!p || !p->enabled || !p->running) continue;

            if (p->demod_type == DEMOD_RAW_IQ && p->process_iq) {
                /* Direct IQ pass-through (ADS-B, etc.) */
                p->process_iq(p->ctx, iq_data, len);
            } else if (p->process_audio) {
                /* TODO: Actual DDC + demodulation pipeline
                 * For now, pass raw IQ as placeholder.
                 * When DDC is implemented:
                 * 1. Apply NCO mixing at ch->offset_hz
                 * 2. Low-pass filter at ch->bandwidth_hz
                 * 3. Decimate to ch->audio_rate_hz
                 * 4. Apply demodulator (FM/AM/SSB/FSK)
                 * 5. Output int16_t audio samples
                 *
                 * Reference: components/dsp/ and esp-dsp library
                 */
                p->process_iq(p->ctx, iq_data, len);
            }
        }
    }
}
