#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize chatbot (loads config, no-op if disabled) */
esp_err_t wifimgr_chatbot_init(void);

/* Process a chat message. Returns allocated response string (caller must free). */
esp_err_t wifimgr_chatbot_message(const char *user_msg, char **response);

/* Get conversation history as JSON array. Caller must cJSON_Delete(). */
struct cJSON *wifimgr_chatbot_get_history(void);

/* Clear conversation history */
esp_err_t wifimgr_chatbot_clear_history(void);

#ifdef __cplusplus
}
#endif
