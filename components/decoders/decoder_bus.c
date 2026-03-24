#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "decode_bus";

/* Subscribers */
static struct {
    decode_listener_fn fn;
    void *ctx;
} s_listeners[DECODE_BUS_MAX_LISTENERS];
static int s_listener_count = 0;

/* Event log (circular buffer) */
static struct {
    char decoder_name[24];
    char event_type[24];
    char key[64];
    int64_t timestamp_ms;
    int8_t rssi_db;
    uint32_t freq_hz;
} s_event_log[DECODE_EVENT_LOG_SIZE];
static int s_log_head = 0;
static int s_log_count = 0;
static SemaphoreHandle_t s_log_mutex;

esp_err_t decode_bus_init(void) {
    s_log_mutex = xSemaphoreCreateMutex();
    s_listener_count = 0;
    s_log_head = 0;
    s_log_count = 0;
    ESP_LOGI(TAG, "Decode bus initialized");
    return ESP_OK;
}

esp_err_t decode_bus_subscribe(decode_listener_fn fn, void *user_ctx) {
    if (!fn || s_listener_count >= DECODE_BUS_MAX_LISTENERS)
        return ESP_ERR_NO_MEM;
    s_listeners[s_listener_count].fn = fn;
    s_listeners[s_listener_count].ctx = user_ctx;
    s_listener_count++;
    return ESP_OK;
}

esp_err_t decode_bus_publish(decode_event_t *event) {
    if (!event) return ESP_ERR_INVALID_ARG;

    /* Add to event log */
    if (s_log_mutex) {
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
        int idx = s_log_head;
        strncpy(s_event_log[idx].decoder_name, event->decoder_name, 23);
        s_event_log[idx].decoder_name[23] = '\0';
        strncpy(s_event_log[idx].event_type, event->event_type, 23);
        s_event_log[idx].event_type[23] = '\0';
        /* Extract key from data JSON if present */
        if (event->data) {
            cJSON *key = cJSON_GetObjectItem(event->data, "key");
            if (key && cJSON_IsString(key)) {
                strncpy(s_event_log[idx].key, key->valuestring, 63);
                s_event_log[idx].key[63] = '\0';
            } else {
                s_event_log[idx].key[0] = '\0';
            }
        } else {
            s_event_log[idx].key[0] = '\0';
        }
        s_event_log[idx].timestamp_ms = event->timestamp_ms;
        s_event_log[idx].rssi_db = event->rssi_db;
        s_event_log[idx].freq_hz = event->freq_hz;
        s_log_head = (s_log_head + 1) % DECODE_EVENT_LOG_SIZE;
        if (s_log_count < DECODE_EVENT_LOG_SIZE) s_log_count++;
        xSemaphoreGive(s_log_mutex);
    }

    /* Dispatch to all listeners */
    for (int i = 0; i < s_listener_count; i++) {
        s_listeners[i].fn(event, s_listeners[i].ctx);
    }

    /* Free the event data JSON */
    if (event->data) {
        cJSON_Delete(event->data);
        event->data = NULL;
    }

    return ESP_OK;
}

cJSON *decode_event_log_get(void) {
    cJSON *arr = cJSON_CreateArray();
    if (!arr || !s_log_mutex) return arr;

    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    for (int i = 0; i < s_log_count; i++) {
        /* Read from oldest to newest */
        int idx = (s_log_head - s_log_count + i + DECODE_EVENT_LOG_SIZE) % DECODE_EVENT_LOG_SIZE;
        cJSON *entry = cJSON_CreateObject();
        cJSON_AddStringToObject(entry, "decoder", s_event_log[idx].decoder_name);
        cJSON_AddStringToObject(entry, "event", s_event_log[idx].event_type);
        cJSON_AddStringToObject(entry, "key", s_event_log[idx].key);
        cJSON_AddNumberToObject(entry, "timestamp_ms", (double)s_event_log[idx].timestamp_ms);
        cJSON_AddNumberToObject(entry, "rssi_db", s_event_log[idx].rssi_db);
        cJSON_AddNumberToObject(entry, "freq_hz", s_event_log[idx].freq_hz);
        cJSON_AddItemToArray(arr, entry);
    }
    xSemaphoreGive(s_log_mutex);

    return arr;
}
