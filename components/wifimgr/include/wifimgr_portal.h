#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the config portal HTTP server (port 80).
 * Serves the SPA UI and handles captive portal detection.
 * Also registers REST API handlers on this server.
 */
esp_err_t wifimgr_portal_start(void);

/**
 * Stop the config portal HTTP server.
 */
esp_err_t wifimgr_portal_stop(void);

#ifdef __cplusplus
}
#endif
