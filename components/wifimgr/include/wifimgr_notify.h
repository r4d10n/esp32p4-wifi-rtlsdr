#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Event types per service ────────────────────────────────── */
#define NOTIFY_EVENT_MAX_PAYLOAD    512

typedef struct {
    char service[24];           /* e.g., "adsb", "ais", "rtl_power" */
    char event[32];             /* e.g., "new_aircraft", "periodic_scan_complete" */
    char title[64];             /* Short title for notification */
    char message[NOTIFY_EVENT_MAX_PAYLOAD];  /* Formatted text (MarkdownV2 for TG) */
    uint8_t *image_data;        /* Optional PNG data (heap-allocated, dispatcher frees) */
    uint32_t image_len;         /* PNG data length (0 if no image) */
} notify_event_t;

/**
 * Initialize notification dispatcher.
 * Creates FreeRTOS queue and dispatcher task.
 */
esp_err_t wifimgr_notify_init(void);

/**
 * Queue a notification event for dispatch.
 * Non-blocking. Returns ESP_ERR_TIMEOUT if queue full.
 * If image_data is set, ownership transfers to dispatcher (it will free).
 */
esp_err_t wifimgr_notify_send(const notify_event_t *event);

/**
 * Send a simple text notification (convenience wrapper).
 */
esp_err_t wifimgr_notify_text(const char *service, const char *event_type,
                               const char *title, const char *message);

/**
 * Send a test notification to verify channel config.
 * @param channel  "telegram" or "discord"
 */
esp_err_t wifimgr_notify_test(const char *channel);

/**
 * Reload notification config from LittleFS (after config change).
 */
esp_err_t wifimgr_notify_reload_config(void);

#ifdef __cplusplus
}
#endif
