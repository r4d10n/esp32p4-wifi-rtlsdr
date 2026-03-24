#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct spyserver_server *spyserver_handle_t;

typedef struct {
    uint16_t port;
    uint8_t  max_clients;
    bool     allow_control;
    uint8_t  fft_fps;
    void    *rtlsdr_dev;    /* rtlsdr_dev_t pointer */
} spyserver_config_t;

#define SPYSERVER_CONFIG_DEFAULT() { \
    .port = 5555, \
    .max_clients = 5, \
    .allow_control = true, \
    .fft_fps = 15, \
    .rtlsdr_dev = NULL, \
}

esp_err_t spyserver_start(spyserver_handle_t *handle, const spyserver_config_t *config);
esp_err_t spyserver_stop(spyserver_handle_t handle);
void      spyserver_push_samples(spyserver_handle_t handle, const uint8_t *data, uint32_t len);
