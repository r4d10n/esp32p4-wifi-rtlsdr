#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct soapy_remote_server *soapyremote_handle_t;

typedef struct {
    uint16_t port;          /* TCP RPC port (default 55132) */
    char format[8];         /* "CS8" or "CS16" */
    uint16_t mtu;           /* UDP MTU (default 1500) */
    bool announce_mdns;     /* Advertise via mDNS */
    void *rtlsdr_dev;      /* rtlsdr_dev_t pointer */
} soapyremote_config_t;

#define SOAPYREMOTE_CONFIG_DEFAULT() { \
    .port = 55132, \
    .format = "CS8", \
    .mtu = 1500, \
    .announce_mdns = true, \
    .rtlsdr_dev = NULL, \
}

esp_err_t soapyremote_start(soapyremote_handle_t *handle, const soapyremote_config_t *config);
esp_err_t soapyremote_stop(soapyremote_handle_t handle);
void soapyremote_push_samples(soapyremote_handle_t handle, const uint8_t *data, uint32_t len);
