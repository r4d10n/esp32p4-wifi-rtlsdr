#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "decoder_framework.h"

static const char *TAG = "decode_track";

/* Global shared tracking table */
static tracking_table_t *s_global_table = NULL;

tracking_table_t *tracking_table_create(uint16_t capacity, uint32_t expiry_ms) {
    tracking_table_t *t = calloc(1, sizeof(tracking_table_t));
    if (!t) return NULL;
    t->entries = calloc(capacity, sizeof(tracking_entry_t));
    if (!t->entries) { free(t); return NULL; }
    t->capacity = capacity;
    t->count = 0;
    t->expiry_ms = expiry_ms;
    t->mutex = xSemaphoreCreateMutex();
    return t;
}

void tracking_table_destroy(tracking_table_t *table) {
    if (!table) return;
    for (uint16_t i = 0; i < table->count; i++) {
        if (table->entries[i].data) {
            cJSON_Delete(table->entries[i].data);
        }
    }
    free(table->entries);
    if (table->mutex) vSemaphoreDelete(table->mutex);
    free(table);
}

esp_err_t tracking_table_upsert(tracking_table_t *table, const char *decoder_name,
                                 const char *key, const cJSON *data, int8_t rssi) {
    if (!table || !key) return ESP_ERR_INVALID_ARG;

    int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

    xSemaphoreTake(table->mutex, portMAX_DELAY);

    /* Search for existing entry */
    for (uint16_t i = 0; i < table->count; i++) {
        if (strcmp(table->entries[i].key, key) == 0 &&
            strcmp(table->entries[i].decoder_name, decoder_name) == 0) {
            /* Update existing */
            if (table->entries[i].data) cJSON_Delete(table->entries[i].data);
            table->entries[i].data = data ? cJSON_Duplicate(data, true) : NULL;
            table->entries[i].last_seen_ms = now;
            table->entries[i].message_count++;
            table->entries[i].last_rssi = rssi;
            xSemaphoreGive(table->mutex);
            return ESP_OK;
        }
    }

    /* Insert new entry */
    if (table->count >= table->capacity) {
        /* Evict oldest entry */
        int oldest_idx = 0;
        int64_t oldest_time = table->entries[0].last_seen_ms;
        for (uint16_t i = 1; i < table->count; i++) {
            if (table->entries[i].last_seen_ms < oldest_time) {
                oldest_time = table->entries[i].last_seen_ms;
                oldest_idx = i;
            }
        }
        if (table->entries[oldest_idx].data)
            cJSON_Delete(table->entries[oldest_idx].data);
        /* Move last entry to evicted slot */
        if (oldest_idx != table->count - 1) {
            table->entries[oldest_idx] = table->entries[table->count - 1];
        }
        table->count--;
    }

    tracking_entry_t *e = &table->entries[table->count];
    memset(e, 0, sizeof(*e));
    strncpy(e->decoder_name, decoder_name, sizeof(e->decoder_name) - 1);
    strncpy(e->key, key, sizeof(e->key) - 1);
    e->data = data ? cJSON_Duplicate(data, true) : NULL;
    e->first_seen_ms = now;
    e->last_seen_ms = now;
    e->message_count = 1;
    e->last_rssi = rssi;
    table->count++;

    xSemaphoreGive(table->mutex);
    return ESP_OK;
}

cJSON *tracking_table_to_json(tracking_table_t *table) {
    cJSON *arr = cJSON_CreateArray();
    if (!table || !arr) return arr;

    xSemaphoreTake(table->mutex, portMAX_DELAY);
    for (uint16_t i = 0; i < table->count; i++) {
        tracking_entry_t *e = &table->entries[i];
        cJSON *obj = cJSON_CreateObject();
        cJSON_AddStringToObject(obj, "decoder", e->decoder_name);
        cJSON_AddStringToObject(obj, "key", e->key);
        cJSON_AddNumberToObject(obj, "first_seen_ms", (double)e->first_seen_ms);
        cJSON_AddNumberToObject(obj, "last_seen_ms", (double)e->last_seen_ms);
        cJSON_AddNumberToObject(obj, "message_count", e->message_count);
        cJSON_AddNumberToObject(obj, "rssi", e->last_rssi);
        if (e->data) {
            cJSON_AddItemToObject(obj, "data", cJSON_Duplicate(e->data, true));
        }
        cJSON_AddItemToArray(arr, obj);
    }
    xSemaphoreGive(table->mutex);
    return arr;
}

cJSON *tracking_table_query(tracking_table_t *table, const char *decoder_name) {
    cJSON *arr = cJSON_CreateArray();
    if (!table || !decoder_name || !arr) return arr;

    xSemaphoreTake(table->mutex, portMAX_DELAY);
    for (uint16_t i = 0; i < table->count; i++) {
        if (strcmp(table->entries[i].decoder_name, decoder_name) == 0) {
            tracking_entry_t *e = &table->entries[i];
            cJSON *obj = cJSON_CreateObject();
            cJSON_AddStringToObject(obj, "key", e->key);
            cJSON_AddNumberToObject(obj, "last_seen_ms", (double)e->last_seen_ms);
            cJSON_AddNumberToObject(obj, "message_count", e->message_count);
            cJSON_AddNumberToObject(obj, "rssi", e->last_rssi);
            if (e->data) {
                cJSON_AddItemToObject(obj, "data", cJSON_Duplicate(e->data, true));
            }
            cJSON_AddItemToArray(arr, obj);
        }
    }
    xSemaphoreGive(table->mutex);
    return arr;
}

void tracking_table_gc(tracking_table_t *table) {
    if (!table || table->expiry_ms == 0) return;

    int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    int64_t cutoff = now - (int64_t)table->expiry_ms;

    xSemaphoreTake(table->mutex, portMAX_DELAY);
    uint16_t i = 0;
    while (i < table->count) {
        if (table->entries[i].last_seen_ms < cutoff) {
            if (table->entries[i].data)
                cJSON_Delete(table->entries[i].data);
            /* Swap with last */
            if (i != table->count - 1) {
                table->entries[i] = table->entries[table->count - 1];
            }
            table->count--;
            /* Don't increment i — re-check the swapped entry */
        } else {
            i++;
        }
    }
    xSemaphoreGive(table->mutex);
}

int tracking_table_count(tracking_table_t *table) {
    if (!table) return 0;
    xSemaphoreTake(table->mutex, portMAX_DELAY);
    int n = table->count;
    xSemaphoreGive(table->mutex);
    return n;
}

/* Global tracking table */
tracking_table_t *decoder_get_global_tracking(void) {
    if (!s_global_table) {
        s_global_table = tracking_table_create(256, 300000); /* 256 entries, 5 min expiry */
        ESP_LOGI(TAG, "Global tracking table created (256 entries, 5 min expiry)");
    }
    return s_global_table;
}
