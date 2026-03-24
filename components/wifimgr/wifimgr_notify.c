/*
 * WiFi Manager — Notification Dispatcher
 *
 * FreeRTOS queue-based dispatcher for Telegram Bot API and Discord Webhook.
 * Per-service rate limiting with exponential backoff retry.
 */

#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

#include "wifimgr_notify.h"
#include "wifimgr_config.h"

static const char *TAG = "wifimgr_notify";

static QueueHandle_t s_queue;
static notify_config_t s_config;

/* Rate limiter: track last send time per service+channel */
#define MAX_RATE_ENTRIES 32

typedef struct {
    char key[48];       /* "service:channel" */
    int64_t last_send;  /* epoch ms */
} rate_entry_t;

static rate_entry_t s_rate_map[MAX_RATE_ENTRIES];
static int s_rate_count;

/* ── Rate limiter ───────────────────────────────────────────── */

static bool rate_limit_check(const char *service, const char *channel, uint16_t limit_s)
{
    char key[48];
    snprintf(key, sizeof(key), "%s:%s", service, channel);

    int64_t now_ms = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < s_rate_count; i++) {
        if (strcmp(s_rate_map[i].key, key) == 0) {
            if ((now_ms - s_rate_map[i].last_send) < (int64_t)limit_s * 1000) {
                return false; /* throttled */
            }
            s_rate_map[i].last_send = now_ms;
            return true;
        }
    }

    /* New entry */
    if (s_rate_count < MAX_RATE_ENTRIES) {
        strncpy(s_rate_map[s_rate_count].key, key, sizeof(s_rate_map[0].key) - 1);
        s_rate_map[s_rate_count].last_send = now_ms;
        s_rate_count++;
    }
    return true;
}

/* ── HTTP POST helper ───────────────────────────────────────── */

static esp_err_t http_post_json(const char *url, const char *json_body,
                                 const char *auth_header)
{
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    if (auth_header) {
        esp_http_client_set_header(client, "Authorization", auth_header);
    }

    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    if (err == ESP_OK && (status < 200 || status >= 300)) {
        ESP_LOGW(TAG, "HTTP POST %s -> status %d", url, status);
        err = ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    return err;
}

/* ── Telegram sender ────────────────────────────────────────── */

static esp_err_t send_telegram_text(const char *title, const char *message)
{
    if (!s_config.telegram.enable ||
        s_config.telegram.bot_token[0] == '\0' ||
        s_config.telegram.chat_id[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }

    char url[160];
    snprintf(url, sizeof(url),
             "https://api.telegram.org/bot%s/sendMessage",
             s_config.telegram.bot_token);

    /* Build JSON body */
    cJSON *body = cJSON_CreateObject();
    cJSON_AddStringToObject(body, "chat_id", s_config.telegram.chat_id);

    char text[600];
    snprintf(text, sizeof(text), "*%s*\n%s", title, message);
    cJSON_AddStringToObject(body, "text", text);
    cJSON_AddStringToObject(body, "parse_mode", "Markdown");

    char *json_str = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);

    if (!json_str) return ESP_ERR_NO_MEM;

    esp_err_t err = http_post_json(url, json_str, NULL);
    cJSON_free(json_str);
    return err;
}

static esp_err_t send_telegram_photo(const char *caption,
                                      const uint8_t *png_data, uint32_t png_len)
{
    if (!s_config.telegram.enable ||
        s_config.telegram.bot_token[0] == '\0' ||
        png_data == NULL || png_len == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    char url[160];
    snprintf(url, sizeof(url),
             "https://api.telegram.org/bot%s/sendPhoto",
             s_config.telegram.bot_token);

    /* Build multipart/form-data body */
    const char *boundary = "----ESP32P4Boundary";

    /* Calculate total size */
    size_t header_len = 512;
    size_t total = header_len + png_len + 256;
    char *body = malloc(total);
    if (!body) return ESP_ERR_NO_MEM;

    int offset = 0;
    offset += snprintf(body + offset, total - offset,
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n"
        "%s\r\n"
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"caption\"\r\n\r\n"
        "%s\r\n"
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"photo\"; filename=\"image.png\"\r\n"
        "Content-Type: image/png\r\n\r\n",
        boundary, s_config.telegram.chat_id,
        boundary, caption ? caption : "",
        boundary);

    memcpy(body + offset, png_data, png_len);
    offset += png_len;

    offset += snprintf(body + offset, total - offset,
        "\r\n--%s--\r\n", boundary);

    /* Send with multipart content type */
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    char ct_header[80];
    snprintf(ct_header, sizeof(ct_header), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", ct_header);
    esp_http_client_set_post_field(client, body, offset);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    if (err == ESP_OK && (status < 200 || status >= 300)) {
        ESP_LOGW(TAG, "Telegram sendPhoto -> status %d", status);
        err = ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    free(body);
    return err;
}

/* ── Discord sender ─────────────────────────────────────────── */

static esp_err_t send_discord_text(const char *title, const char *message)
{
    if (!s_config.discord.enable ||
        s_config.discord.webhook_url[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }

    cJSON *body = cJSON_CreateObject();

    /* Use embed for structured messages */
    cJSON *embeds = cJSON_AddArrayToObject(body, "embeds");
    cJSON *embed = cJSON_CreateObject();
    cJSON_AddStringToObject(embed, "title", title);
    cJSON_AddStringToObject(embed, "description", message);
    cJSON_AddNumberToObject(embed, "color", 3447003); /* blue */
    cJSON_AddItemToArray(embeds, embed);

    char *json_str = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);

    if (!json_str) return ESP_ERR_NO_MEM;

    esp_err_t err = http_post_json(s_config.discord.webhook_url, json_str, NULL);
    cJSON_free(json_str);
    return err;
}

/* ── Dispatcher task ────────────────────────────────────────── */

static void notify_task(void *arg)
{
    notify_event_t event;

    for (;;) {
        if (xQueueReceive(s_queue, &event, portMAX_DELAY) != pdTRUE) continue;

        ESP_LOGI(TAG, "Dispatching: %s/%s", event.service, event.event);

        /* Telegram */
        if (s_config.telegram.enable &&
            rate_limit_check(event.service, "telegram", s_config.telegram.rate_limit_s)) {

            esp_err_t err = ESP_FAIL;
            for (int retry = 0; retry < CONFIG_WIFIMGR_NOTIFY_MAX_RETRIES; retry++) {
                if (event.image_data && event.image_len > 0) {
                    err = send_telegram_photo(event.title,
                                               event.image_data, event.image_len);
                } else {
                    err = send_telegram_text(event.title, event.message);
                }
                if (err == ESP_OK) break;
                /* Exponential backoff: 1s, 2s, 4s */
                vTaskDelay(pdMS_TO_TICKS(1000 << retry));
            }
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Telegram delivery failed for %s/%s",
                         event.service, event.event);
            }
        }

        /* Discord */
        if (s_config.discord.enable &&
            rate_limit_check(event.service, "discord", s_config.discord.rate_limit_s)) {

            esp_err_t err = ESP_FAIL;
            for (int retry = 0; retry < CONFIG_WIFIMGR_NOTIFY_MAX_RETRIES; retry++) {
                err = send_discord_text(event.title, event.message);
                if (err == ESP_OK) break;
                vTaskDelay(pdMS_TO_TICKS(1000 << retry));
            }
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Discord delivery failed for %s/%s",
                         event.service, event.event);
            }
        }

        /* Free image data if allocated */
        free(event.image_data);
    }
}

/* ── Public API ─────────────────────────────────────────────── */

esp_err_t wifimgr_notify_init(void)
{
    s_queue = xQueueCreate(CONFIG_WIFIMGR_NOTIFY_QUEUE_SIZE, sizeof(notify_event_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    /* Load config */
    wifimgr_config_load_notify(&s_config);

    xTaskCreatePinnedToCore(notify_task, "notify", 8192, NULL, 3, NULL, 1);

    ESP_LOGI(TAG, "Notification dispatcher started (TG=%s, DC=%s)",
             s_config.telegram.enable ? "on" : "off",
             s_config.discord.enable ? "on" : "off");
    return ESP_OK;
}

esp_err_t wifimgr_notify_send(const notify_event_t *event)
{
    if (!event || !s_queue) return ESP_ERR_INVALID_STATE;

    if (xQueueSend(s_queue, event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Notification queue full, dropping %s/%s",
                 event->service, event->event);
        /* Free image data since we're dropping */
        free(event->image_data);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t wifimgr_notify_text(const char *service, const char *event_type,
                               const char *title, const char *message)
{
    notify_event_t event = {0};
    strncpy(event.service, service, sizeof(event.service) - 1);
    strncpy(event.event, event_type, sizeof(event.event) - 1);
    strncpy(event.title, title, sizeof(event.title) - 1);
    strncpy(event.message, message, sizeof(event.message) - 1);
    event.image_data = NULL;
    event.image_len = 0;

    return wifimgr_notify_send(&event);
}

esp_err_t wifimgr_notify_test(const char *channel)
{
    if (!channel) return ESP_ERR_INVALID_ARG;

    if (strcmp(channel, "telegram") == 0) {
        return send_telegram_text("ESP32-P4 SDR Test",
                                   "Notification channel working.");
    } else if (strcmp(channel, "discord") == 0) {
        return send_discord_text("ESP32-P4 SDR Test",
                                  "Notification channel working.");
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t wifimgr_notify_reload_config(void)
{
    return wifimgr_config_load_notify(&s_config);
}
