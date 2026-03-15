/*
 * ESP32-P4 RTL-SDR WiFi Bridge — Main Application
 *
 * USB Host → RTL-SDR → Ring Buffer → RTL-TCP Server → WiFi/Ethernet → Client
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "usb/usb_host.h"
#include "mdns.h"
#include "rtlsdr.h"
#include "rtltcp.h"

static const char *TAG = "main";

/* WiFi credentials — set via menuconfig or sdkconfig.defaults */
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASSWORD

/* RTL-SDR default configuration */
#define DEFAULT_FREQ        100000000   /* 100 MHz */
#define DEFAULT_SAMPLE_RATE 1024000     /* 1.024 MSPS */

static rtlsdr_dev_t    *sdr_dev = NULL;
static rtltcp_server_t *tcp_srv = NULL;

/* ──────────────────────── WiFi Event Handler ──────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static esp_err_t wifi_init_sta(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");

    esp_netif_create_default_wifi_sta();

    /* Initialize ESP-Hosted transport (SDIO to ESP32-C6) before WiFi */
    ESP_LOGI(TAG, "Initializing ESP-Hosted...");
    ESP_RETURN_ON_ERROR(esp_hosted_init(), TAG, "esp_hosted_init failed");
    ESP_LOGI(TAG, "Connecting to C6 slave over SDIO...");
    ESP_RETURN_ON_ERROR(esp_hosted_connect_to_slave(), TAG, "esp_hosted_connect failed");
    ESP_LOGI(TAG, "ESP-Hosted connected to C6");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");

    esp_event_handler_instance_t any_id, got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, &any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, &got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "set config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    ESP_LOGI(TAG, "WiFi STA connecting to '%s'...", WIFI_SSID);
    return ESP_OK;
}

/* ──────────────────────── mDNS Service ──────────────────────── */

static void mdns_init_service(void)
{
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
        return;
    }

    mdns_hostname_set("esp32p4-rtlsdr");
    mdns_instance_name_set("ESP32-P4 RTL-SDR");

    mdns_service_add(NULL, "_rtl_tcp", "_tcp", RTLTCP_DEFAULT_PORT, NULL, 0);
    ESP_LOGI(TAG, "mDNS: esp32p4-rtlsdr._rtl_tcp._tcp:%d", RTLTCP_DEFAULT_PORT);
}

/* ──────────────────────── USB Host Task ──────────────────────── */

static void usb_host_task(void *arg)
{
    while (1) {
        usb_host_lib_handle_events(portMAX_DELAY, NULL);
    }
}

/* ──────────────────────── IQ Callback (USB → Ring Buffer) ──────────────────────── */

static void iq_data_cb(uint8_t *buf, uint32_t len, void *ctx)
{
    rtltcp_server_t *srv = (rtltcp_server_t *)ctx;
    rtltcp_push_samples(srv, buf, len);
}

/* ──────────────────────── SDR Streaming Task ──────────────────────── */

static void sdr_stream_task(void *arg)
{
    ESP_LOGI(TAG, "Starting IQ streaming (rate=%lu, freq=%lu)",
             (unsigned long)rtlsdr_get_sample_rate(sdr_dev),
             (unsigned long)rtlsdr_get_center_freq(sdr_dev));

    /* This blocks until rtlsdr_stop_async() is called */
    esp_err_t ret = rtlsdr_read_async(sdr_dev, iq_data_cb, tcp_srv, 8, 16 * 512);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Async read ended with error: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "SDR streaming stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── Main ──────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 RTL-SDR WiFi Bridge starting...");

    /* Initialize NVS (required by WiFi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize WiFi */
    ESP_ERROR_CHECK(wifi_init_sta());

    /* Wait for IP address */
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Start mDNS */
    mdns_init_service();

    /* Install USB Host Library */
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host library installed");

    /* Start USB host event task on Core 0 */
    xTaskCreatePinnedToCore(usb_host_task, "usb_host", 4096, NULL, 10, NULL, 0);

    /* Initialize RTL-SDR (blocks until device connects) */
    ret = rtlsdr_init(&sdr_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTL-SDR init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Configure SDR */
    rtlsdr_config_t sdr_config = RTLSDR_CONFIG_DEFAULT();
    sdr_config.center_freq = DEFAULT_FREQ;
    sdr_config.sample_rate = DEFAULT_SAMPLE_RATE;
    ESP_ERROR_CHECK(rtlsdr_configure(sdr_dev, &sdr_config));

    /* Start RTL-TCP server */
    rtltcp_config_t tcp_config = RTLTCP_CONFIG_DEFAULT();
    tcp_config.dev = sdr_dev;
    ESP_ERROR_CHECK(rtltcp_server_start(&tcp_srv, &tcp_config));

    /* Start SDR streaming task on Core 0 (with USB) */
    xTaskCreatePinnedToCore(sdr_stream_task, "sdr_stream", 8192, NULL, 8, NULL, 0);

    ESP_LOGI(TAG, "=== RTL-TCP server ready on port %d ===", RTLTCP_DEFAULT_PORT);
    ESP_LOGI(TAG, "Connect with: rtl_tcp -a <this_ip> -p %d", RTLTCP_DEFAULT_PORT);
    ESP_LOGI(TAG, "  or SDR++ / GQRX → rtl_tcp source");

    /* Main task just monitors status */
    while (1) {
        bool connected = rtltcp_is_client_connected(tcp_srv);
        ESP_LOGI(TAG, "Status: client=%s freq=%luHz rate=%luHz",
                 connected ? "YES" : "no",
                 (unsigned long)rtlsdr_get_center_freq(sdr_dev),
                 (unsigned long)rtlsdr_get_sample_rate(sdr_dev));
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
