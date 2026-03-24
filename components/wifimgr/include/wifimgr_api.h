#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Register all REST API handlers on the given HTTP server.
 *
 * Endpoints: wifi, eth, sdr, services, notify, chat, system
 */
esp_err_t wifimgr_api_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
