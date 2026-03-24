/*
 * WiFi Manager — LLM Chatbot Gateway
 *
 * Supports Gemini, OpenAI, and Claude APIs.
 * Maintains conversation history with mutex protection.
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "wifimgr_chatbot.h"
#include "wifimgr_config.h"

static const char *TAG = "chatbot";
static chatbot_config_t s_chatbot_cfg;
static cJSON *s_history;  /* JSON array of {role, content} objects */
static SemaphoreHandle_t s_mutex;

#define MAX_RESPONSE_BUF  8192

static const char *SYSTEM_PROMPT =
    "You are an AI assistant for an ESP32-P4 RTL-SDR receiver. "
    "You can query device status, frequency, gain, and active services. "
    "Available tools: get_sdr_status, get_system_info. "
    "Respond concisely. If asked to change settings, explain how via the config portal.";

/* ── HTTP response buffer ───────────────────────────────────── */

typedef struct {
    char *buf;
    int len;
    int capacity;
} http_response_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_response_t *resp = (http_response_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (resp->len + evt->data_len < resp->capacity) {
            memcpy(resp->buf + resp->len, evt->data, evt->data_len);
            resp->len += evt->data_len;
            resp->buf[resp->len] = '\0';
        }
    }
    return ESP_OK;
}

/* ── Shared HTTP POST helper ────────────────────────────────── */

static char *http_post_json(const char *url, const char *body,
                             const char *hdr1_name, const char *hdr1_val,
                             const char *hdr2_name, const char *hdr2_val,
                             const char *hdr3_name, const char *hdr3_val)
{
    char *response_buf = calloc(1, MAX_RESPONSE_BUF);
    if (!response_buf) return NULL;

    http_response_t resp = {
        .buf = response_buf,
        .len = 0,
        .capacity = MAX_RESPONSE_BUF - 1,
    };

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 30000,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        free(response_buf);
        return NULL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    if (hdr1_name && hdr1_val) esp_http_client_set_header(client, hdr1_name, hdr1_val);
    if (hdr2_name && hdr2_val) esp_http_client_set_header(client, hdr2_name, hdr2_val);
    if (hdr3_name && hdr3_val) esp_http_client_set_header(client, hdr3_name, hdr3_val);

    esp_http_client_set_post_field(client, body, strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status < 200 || status >= 300) {
        ESP_LOGE(TAG, "HTTP POST failed: err=%d status=%d", err, status);
        free(response_buf);
        return NULL;
    }

    return response_buf;
}

/* ── Gemini ─────────────────────────────────────────────────── */

static char *call_gemini(const char *user_msg)
{
    /* Build contents array from history */
    cJSON *root = cJSON_CreateObject();
    cJSON *contents = cJSON_AddArrayToObject(root, "contents");

    /* Add system instruction as first user turn (Gemini approach) */
    cJSON *sys_turn = cJSON_CreateObject();
    cJSON_AddStringToObject(sys_turn, "role", "user");
    cJSON *sys_parts = cJSON_AddArrayToObject(sys_turn, "parts");
    cJSON *sys_part = cJSON_CreateObject();
    cJSON_AddStringToObject(sys_part, "text", SYSTEM_PROMPT);
    cJSON_AddItemToArray(sys_parts, sys_part);
    cJSON_AddItemToArray(contents, sys_turn);

    /* Add a model acknowledgement turn */
    cJSON *ack_turn = cJSON_CreateObject();
    cJSON_AddStringToObject(ack_turn, "role", "model");
    cJSON *ack_parts = cJSON_AddArrayToObject(ack_turn, "parts");
    cJSON *ack_part = cJSON_CreateObject();
    cJSON_AddStringToObject(ack_part, "text", "Understood. I am ready to assist.");
    cJSON_AddItemToArray(ack_parts, ack_part);
    cJSON_AddItemToArray(contents, ack_turn);

    /* Add history entries (skip the current user_msg which was already appended) */
    int hist_size = cJSON_GetArraySize(s_history);
    /* Include all but the last entry (current user message) */
    for (int i = 0; i < hist_size - 1; i++) {
        cJSON *entry = cJSON_GetArrayItem(s_history, i);
        cJSON *role_j = cJSON_GetObjectItem(entry, "role");
        cJSON *content_j = cJSON_GetObjectItem(entry, "content");
        if (!cJSON_IsString(role_j) || !cJSON_IsString(content_j)) continue;

        const char *role = role_j->valuestring;
        /* Gemini uses "model" not "assistant" */
        const char *gemini_role = strcmp(role, "assistant") == 0 ? "model" : "user";

        cJSON *turn = cJSON_CreateObject();
        cJSON_AddStringToObject(turn, "role", gemini_role);
        cJSON *parts = cJSON_AddArrayToObject(turn, "parts");
        cJSON *part = cJSON_CreateObject();
        cJSON_AddStringToObject(part, "text", content_j->valuestring);
        cJSON_AddItemToArray(parts, part);
        cJSON_AddItemToArray(contents, turn);
    }

    /* Add current user message */
    cJSON *user_turn = cJSON_CreateObject();
    cJSON_AddStringToObject(user_turn, "role", "user");
    cJSON *user_parts = cJSON_AddArrayToObject(user_turn, "parts");
    cJSON *user_part = cJSON_CreateObject();
    cJSON_AddStringToObject(user_part, "text", user_msg);
    cJSON_AddItemToArray(user_parts, user_part);
    cJSON_AddItemToArray(contents, user_turn);

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body) return NULL;

    /* Build URL: .../models/{model}:generateContent?key={api_key} */
    char url[384];
    snprintf(url, sizeof(url),
             "https://generativelanguage.googleapis.com/v1beta/models/%s:generateContent?key=%s",
             s_chatbot_cfg.model, s_chatbot_cfg.api_key);

    char *raw = http_post_json(url, body, NULL, NULL, NULL, NULL, NULL, NULL);
    cJSON_free(body);
    if (!raw) return NULL;

    /* Parse: candidates[0].content.parts[0].text */
    char *reply = NULL;
    cJSON *json = cJSON_Parse(raw);
    free(raw);
    if (json) {
        cJSON *candidates = cJSON_GetObjectItem(json, "candidates");
        cJSON *c0 = cJSON_GetArrayItem(candidates, 0);
        cJSON *content = cJSON_GetObjectItem(c0, "content");
        cJSON *parts = cJSON_GetObjectItem(content, "parts");
        cJSON *p0 = cJSON_GetArrayItem(parts, 0);
        cJSON *text = cJSON_GetObjectItem(p0, "text");
        if (cJSON_IsString(text)) {
            reply = strdup(text->valuestring);
        }
        cJSON_Delete(json);
    }

    return reply;
}

/* ── OpenAI ─────────────────────────────────────────────────── */

static char *call_openai(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", s_chatbot_cfg.model);

    cJSON *messages = cJSON_AddArrayToObject(root, "messages");

    /* System message */
    cJSON *sys_msg = cJSON_CreateObject();
    cJSON_AddStringToObject(sys_msg, "role", "system");
    cJSON_AddStringToObject(sys_msg, "content", SYSTEM_PROMPT);
    cJSON_AddItemToArray(messages, sys_msg);

    /* History */
    int hist_size = cJSON_GetArraySize(s_history);
    for (int i = 0; i < hist_size; i++) {
        cJSON *entry = cJSON_GetArrayItem(s_history, i);
        cJSON *role_j = cJSON_GetObjectItem(entry, "role");
        cJSON *content_j = cJSON_GetObjectItem(entry, "content");
        if (!cJSON_IsString(role_j) || !cJSON_IsString(content_j)) continue;

        cJSON *msg = cJSON_CreateObject();
        cJSON_AddStringToObject(msg, "role", role_j->valuestring);
        cJSON_AddStringToObject(msg, "content", content_j->valuestring);
        cJSON_AddItemToArray(messages, msg);
    }

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body) return NULL;

    char auth_header[160];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", s_chatbot_cfg.api_key);

    char *raw = http_post_json("https://api.openai.com/v1/chat/completions", body,
                               "Authorization", auth_header,
                               NULL, NULL, NULL, NULL);
    cJSON_free(body);
    if (!raw) return NULL;

    /* Parse: choices[0].message.content */
    char *reply = NULL;
    cJSON *json = cJSON_Parse(raw);
    free(raw);
    if (json) {
        cJSON *choices = cJSON_GetObjectItem(json, "choices");
        cJSON *c0 = cJSON_GetArrayItem(choices, 0);
        cJSON *message = cJSON_GetObjectItem(c0, "message");
        cJSON *content = cJSON_GetObjectItem(message, "content");
        if (cJSON_IsString(content)) {
            reply = strdup(content->valuestring);
        }
        cJSON_Delete(json);
    }

    return reply;
}

/* ── Claude ─────────────────────────────────────────────────── */

static char *call_claude(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", s_chatbot_cfg.model);
    cJSON_AddNumberToObject(root, "max_tokens", 1024);
    cJSON_AddStringToObject(root, "system", SYSTEM_PROMPT);

    cJSON *messages = cJSON_AddArrayToObject(root, "messages");

    /* History */
    int hist_size = cJSON_GetArraySize(s_history);
    for (int i = 0; i < hist_size; i++) {
        cJSON *entry = cJSON_GetArrayItem(s_history, i);
        cJSON *role_j = cJSON_GetObjectItem(entry, "role");
        cJSON *content_j = cJSON_GetObjectItem(entry, "content");
        if (!cJSON_IsString(role_j) || !cJSON_IsString(content_j)) continue;

        cJSON *msg = cJSON_CreateObject();
        cJSON_AddStringToObject(msg, "role", role_j->valuestring);
        cJSON_AddStringToObject(msg, "content", content_j->valuestring);
        cJSON_AddItemToArray(messages, msg);
    }

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body) return NULL;

    char *raw = http_post_json("https://api.anthropic.com/v1/messages", body,
                               "x-api-key", s_chatbot_cfg.api_key,
                               "anthropic-version", "2023-06-01",
                               NULL, NULL);
    cJSON_free(body);
    if (!raw) return NULL;

    /* Parse: content[0].text */
    char *reply = NULL;
    cJSON *json = cJSON_Parse(raw);
    free(raw);
    if (json) {
        cJSON *content = cJSON_GetObjectItem(json, "content");
        cJSON *c0 = cJSON_GetArrayItem(content, 0);
        cJSON *text = cJSON_GetObjectItem(c0, "text");
        if (cJSON_IsString(text)) {
            reply = strdup(text->valuestring);
        }
        cJSON_Delete(json);
    }

    return reply;
}

/* ── Public API ─────────────────────────────────────────────── */

esp_err_t wifimgr_chatbot_init(void)
{
    wifimgr_config_load_chatbot(&s_chatbot_cfg);
    s_history = cJSON_CreateArray();
    s_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Chatbot init (provider=%s, enabled=%d)",
             s_chatbot_cfg.provider, s_chatbot_cfg.enable);
    return ESP_OK;
}

esp_err_t wifimgr_chatbot_message(const char *user_msg, char **response)
{
    if (!s_chatbot_cfg.enable || s_chatbot_cfg.api_key[0] == '\0') {
        *response = strdup("Chatbot not configured. Set API key in Notifications tab.");
        return ESP_OK;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Add user message to history */
    cJSON *user_entry = cJSON_CreateObject();
    cJSON_AddStringToObject(user_entry, "role", "user");
    cJSON_AddStringToObject(user_entry, "content", user_msg);
    cJSON_AddItemToArray(s_history, user_entry);

    /* Trim history to max (pairs of user+assistant = max_history * 2 entries) */
    while (cJSON_GetArraySize(s_history) > s_chatbot_cfg.max_history * 2) {
        cJSON_DeleteItemFromArray(s_history, 0);
    }

    /* Build request based on provider */
    char *reply = NULL;
    if (strcmp(s_chatbot_cfg.provider, "gemini") == 0) {
        reply = call_gemini(user_msg);
    } else if (strcmp(s_chatbot_cfg.provider, "openai") == 0) {
        reply = call_openai();
    } else if (strcmp(s_chatbot_cfg.provider, "claude") == 0) {
        reply = call_claude();
    } else {
        ESP_LOGW(TAG, "Unknown provider: %s", s_chatbot_cfg.provider);
    }

    if (reply) {
        /* Add assistant response to history */
        cJSON *asst_entry = cJSON_CreateObject();
        cJSON_AddStringToObject(asst_entry, "role", "assistant");
        cJSON_AddStringToObject(asst_entry, "content", reply);
        cJSON_AddItemToArray(s_history, asst_entry);

        *response = reply;
    } else {
        *response = strdup("Error communicating with LLM provider.");
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

cJSON *wifimgr_chatbot_get_history(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON *copy = cJSON_Duplicate(s_history, true);
    xSemaphoreGive(s_mutex);
    return copy;
}

esp_err_t wifimgr_chatbot_clear_history(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON_Delete(s_history);
    s_history = cJSON_CreateArray();
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}
