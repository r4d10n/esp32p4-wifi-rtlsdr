#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Register all REST API handlers on the given HTTP server.
 *
 * Endpoints registered:
 *   GET/POST/PUT/DELETE /api/wifi/*
 *   GET/PUT             /api/eth/*
 *   GET/PUT             /api/sdr/config
 *   GET/PUT             /api/services, /api/services/{name}
 *   GET/PUT/POST        /api/notify/*
 *   GET/PUT/POST/DELETE /api/chat/*
 *   GET/POST            /api/system/*
 */
esp_err_t wifimgr_api_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
