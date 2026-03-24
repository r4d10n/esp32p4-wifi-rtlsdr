/*
 * ESP32-P4 RTL-SDR WiFi Bridge — Main Application
 *
 * USB Host → RTL-SDR → Ring Buffer → RTL-TCP Server → WiFi/Ethernet → Client
 *
 * WiFi/Ethernet managed by wifimgr component (runtime config via web portal).
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
#include "usb/usb_host.h"
#include "mdns.h"
#include "rtlsdr.h"
#include "rtltcp.h"
#include "rtludp.h"
#include "websdr.h"
#include "wifimgr.h"
#include "wifimgr_config.h"
#include "wifimgr_notify.h"
#include "wifimgr_chatbot.h"
#include "spyserver.h"
#include "soapyremote.h"
#include "decoder_framework.h"

static const char *TAG = "main";

/* RTL-SDR default fallback (overridden by sdr.json if present) */
#define DEFAULT_FREQ        100000000   /* 100 MHz */
#define DEFAULT_SAMPLE_RATE 250000      /* 250 kSPS */

static rtlsdr_dev_t        *sdr_dev = NULL;
static rtltcp_server_t     *tcp_srv = NULL;
static rtludp_server_t     *udp_srv = NULL;
static websdr_server_t     *websdr_srv = NULL;
static spyserver_handle_t   spyserver_srv = NULL;
static soapyremote_handle_t soapy_srv = NULL;

/* ──────────────────────── Multicast Setup ──────────────────────── */

#ifdef CONFIG_RTLSDR_MULTICAST_ENABLE
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static int multicast_fd = -1;
static struct sockaddr_in multicast_addr;

static esp_err_t multicast_init(void)
{
    ESP_LOGI(TAG, "Initializing Multicast UDP to %s:%d",
             CONFIG_RTLSDR_MULTICAST_GROUP, CONFIG_RTLSDR_MULTICAST_PORT);

    multicast_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    ESP_RETURN_ON_FALSE(multicast_fd >= 0, ESP_FAIL, TAG, "Multicast socket failed");

    int ttl = 4;
    setsockopt(multicast_fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    int loop = 1;
    setsockopt(multicast_fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

    memset(&multicast_addr, 0, sizeof(multicast_addr));
    multicast_addr.sin_family = AF_INET;
    multicast_addr.sin_port = htons(CONFIG_RTLSDR_MULTICAST_PORT);
    inet_aton(CONFIG_RTLSDR_MULTICAST_GROUP, &multicast_addr.sin_addr);

    ESP_LOGI(TAG, "Multicast ready: %s:%d", CONFIG_RTLSDR_MULTICAST_GROUP,
             CONFIG_RTLSDR_MULTICAST_PORT);
    return ESP_OK;
}

static void multicast_push_samples(const uint8_t *data, uint32_t len)
{
    if (multicast_fd < 0) return;
    uint32_t offset = 0;
    while (offset < len) {
        uint32_t chunk = (len - offset > 1024) ? 1024 : (len - offset);
        sendto(multicast_fd, data + offset, chunk, MSG_DONTWAIT,
               (struct sockaddr *)&multicast_addr, sizeof(multicast_addr));
        offset += chunk;
    }
}
#endif /* CONFIG_RTLSDR_MULTICAST_ENABLE */

/* ──────────────────────── mDNS Service ──────────────────────── */

static void mdns_init_service(const sdr_config_t *sdr_cfg)
{
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
        return;
    }

    mdns_hostname_set(sdr_cfg->hostname);
    mdns_instance_name_set("ESP32-P4 RTL-SDR");

    /* Advertise all active services */
    mdns_txt_item_t txt_rtltcp[] = {
        {"board", "esp32-p4"},
        {"tuner", "R828D"},
    };
    mdns_service_add(NULL, "_rtl_tcp", "_tcp", RTLTCP_DEFAULT_PORT, txt_rtltcp, 2);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);   /* Config portal */
    mdns_service_add(NULL, "_http", "_tcp", WEBSDR_DEFAULT_PORT, NULL, 0); /* WebSDR */

    ESP_LOGI(TAG, "mDNS: %s._rtl_tcp._tcp:%d", sdr_cfg->hostname, RTLTCP_DEFAULT_PORT);
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
    (void)ctx;
    if (tcp_srv) rtltcp_push_samples(tcp_srv, buf, len);
    if (udp_srv) rtludp_push_samples(udp_srv, buf, len);
    if (websdr_srv) websdr_push_samples(websdr_srv, buf, len);
    if (spyserver_srv) spyserver_push_samples(spyserver_srv, buf, len);
    if (soapy_srv) soapyremote_push_samples(soapy_srv, buf, len);
    decoder_channel_manager_push_iq(buf, len, rtlsdr_get_sample_rate(sdr_dev));
#ifdef CONFIG_RTLSDR_MULTICAST_ENABLE
    multicast_push_samples(buf, len);
#endif
}

/* ──────────────────────── SDR Streaming Task ──────────────────────── */

static void sdr_stream_task(void *arg)
{
    ESP_LOGI(TAG, "Starting IQ streaming (rate=%lu, freq=%lu)",
             (unsigned long)rtlsdr_get_sample_rate(sdr_dev),
             (unsigned long)rtlsdr_get_center_freq(sdr_dev));

    esp_err_t ret = rtlsdr_read_async(sdr_dev, iq_data_cb, tcp_srv, 0, 0);
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

    /* ── WiFi Manager init (handles NVS, LittleFS, WiFi, Ethernet) ── */

    /* ESP-Hosted transport must be initialized before WiFi */
    ESP_LOGI(TAG, "Initializing ESP-Hosted...");
    ESP_ERROR_CHECK(esp_hosted_init());
    ESP_LOGI(TAG, "Connecting to C6 slave over SDIO...");
    ESP_ERROR_CHECK(esp_hosted_connect_to_slave());
    ESP_LOGI(TAG, "ESP-Hosted connected to C6");

    /* Initialize WiFi manager (sets up NVS, LittleFS, WiFi, button, UART) */
    wifimgr_config_t wm_cfg = WIFIMGR_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(wifimgr_init(&wm_cfg));

    /* Start WiFi manager state machine (scans, connects, or enters AP mode) */
    ESP_ERROR_CHECK(wifimgr_start());

    /* Initialize notification dispatcher */
    wifimgr_notify_init();

    /* Initialize Ethernet (if enabled in config) */
    extern esp_err_t wifimgr_ethernet_init(void);
    wifimgr_ethernet_init();

    /* Wait for network (WiFi or AP mode with portal) — max 60s */
    ESP_LOGI(TAG, "Waiting for network connection...");
    esp_err_t ret = wifimgr_wait_connected(60000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No WiFi connection — AP mode active, portal at 192.168.4.1");
        /* Continue anyway — services will work on AP network or Ethernet */
    }

    /* ── Load SDR config from LittleFS ── */

    sdr_config_t sdr_cfg;
    wifimgr_config_load_sdr(&sdr_cfg);

#ifdef CONFIG_RTLSDR_MULTICAST_ENABLE
    multicast_init();
#endif

    /* Start mDNS with configured hostname */
    mdns_init_service(&sdr_cfg);

    /* ── USB Host + RTL-SDR ── */

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host library installed");

    xTaskCreatePinnedToCore(usb_host_task, "usb_host", 4096, NULL, 10, NULL, 0);

    /* Initialize RTL-SDR (blocks until device connects) */
    ret = rtlsdr_init(&sdr_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTL-SDR init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Configure SDR from saved config */
    rtlsdr_config_t rtl_cfg = RTLSDR_CONFIG_DEFAULT();
    rtl_cfg.center_freq = sdr_cfg.center_freq;
    rtl_cfg.sample_rate = sdr_cfg.sample_rate;
    ESP_ERROR_CHECK(rtlsdr_configure(sdr_dev, &rtl_cfg));

    /* Apply additional SDR parameters */
    if (sdr_cfg.ppm_correction != 0) {
        rtlsdr_set_freq_correction(sdr_dev, sdr_cfg.ppm_correction);
    }
    if (strcmp(sdr_cfg.gain_mode, "manual") == 0) {
        rtlsdr_set_tuner_gain_mode(sdr_dev, 1);
        rtlsdr_set_tuner_gain(sdr_dev, sdr_cfg.tuner_gain_tenth_db);
    } else {
        rtlsdr_set_tuner_gain_mode(sdr_dev, 0);
    }
    if (sdr_cfg.rtl_agc) {
        rtlsdr_set_agc_mode(sdr_dev, 1);
    }
    if (sdr_cfg.bias_tee) {
        rtlsdr_set_bias_tee(sdr_dev, 1);
    }
    if (strcmp(sdr_cfg.direct_sampling, "I-ADC") == 0) {
        rtlsdr_set_direct_sampling(sdr_dev, 1);
    } else if (strcmp(sdr_cfg.direct_sampling, "Q-ADC") == 0) {
        rtlsdr_set_direct_sampling(sdr_dev, 2);
    }
    if (sdr_cfg.offset_tuning) {
        rtlsdr_set_offset_tuning(sdr_dev, 1);
    }

    /* ── Start streaming services ── */

    rtltcp_config_t tcp_config = RTLTCP_CONFIG_DEFAULT();
    tcp_config.dev = sdr_dev;
    ESP_ERROR_CHECK(rtltcp_server_start(&tcp_srv, &tcp_config));

    rtludp_config_t udp_config = RTLUDP_CONFIG_DEFAULT();
    udp_config.dev = sdr_dev;
    ESP_ERROR_CHECK(rtludp_server_start(&udp_srv, &udp_config));

    websdr_config_t websdr_config = WEBSDR_CONFIG_DEFAULT();
    websdr_config.dev = sdr_dev;
    ESP_ERROR_CHECK(websdr_server_start(&websdr_srv, &websdr_config));

    xTaskCreatePinnedToCore(sdr_stream_task, "sdr_stream", 8192, NULL, 8, NULL, 0);

    /* ── Start config-driven services (SpyServer, SoapySDR, Chatbot) ── */
    {
        cJSON *services = wifimgr_config_load_services();

        /* SpyServer */
        cJSON *spy_cfg = cJSON_GetObjectItem(services, "spyserver");
        if (spy_cfg && cJSON_IsTrue(cJSON_GetObjectItem(spy_cfg, "enable"))) {
            spyserver_config_t spy_config = SPYSERVER_CONFIG_DEFAULT();
            spy_config.rtlsdr_dev = sdr_dev;
            cJSON *port = cJSON_GetObjectItem(spy_cfg, "port");
            if (port) spy_config.port = (uint16_t)cJSON_GetNumberValue(port);
            cJSON *max_cl = cJSON_GetObjectItem(spy_cfg, "max_clients");
            if (max_cl) spy_config.max_clients = (uint8_t)cJSON_GetNumberValue(max_cl);
            cJSON *fft = cJSON_GetObjectItem(spy_cfg, "fft_fps");
            if (fft) spy_config.fft_fps = (uint8_t)cJSON_GetNumberValue(fft);
            if (spyserver_start(&spyserver_srv, &spy_config) == ESP_OK) {
                ESP_LOGI(TAG, "=== SpyServer ready on port %d ===", spy_config.port);
            }
        }

        /* SoapySDR Remote */
        cJSON *soapy_cfg = cJSON_GetObjectItem(services, "soapysdr");
        if (soapy_cfg && cJSON_IsTrue(cJSON_GetObjectItem(soapy_cfg, "enable"))) {
            soapyremote_config_t soapy_config = SOAPYREMOTE_CONFIG_DEFAULT();
            soapy_config.rtlsdr_dev = sdr_dev;
            cJSON *port = cJSON_GetObjectItem(soapy_cfg, "port");
            if (port) soapy_config.port = (uint16_t)cJSON_GetNumberValue(port);
            if (soapyremote_start(&soapy_srv, &soapy_config) == ESP_OK) {
                ESP_LOGI(TAG, "=== SoapySDR Remote ready on port %d ===", soapy_config.port);
            }
        }

        cJSON_Delete(services);
    }

    /* Initialize chatbot (loads config, starts if enabled) */
    wifimgr_chatbot_init();

    /* Initialize pluggable decoder framework */
    decoder_registry_init();

    ESP_LOGI(TAG, "=== RTL-TCP server ready on port %d ===", RTLTCP_DEFAULT_PORT);
    ESP_LOGI(TAG, "=== RTL-UDP server ready on port %d ===", RTLUDP_DEFAULT_PORT);
    ESP_LOGI(TAG, "=== WebSDR server ready on port %d ===", WEBSDR_DEFAULT_PORT);
    ESP_LOGI(TAG, "=== Config portal at http://%s/ ===", sdr_cfg.hostname);

    wifimgr_notify_text("system", "boot", "ESP32-P4 SDR Ready",
                         "All services started successfully.");

    /* Main task monitors status */
    while (1) {
        bool tcp_conn = rtltcp_is_client_connected(tcp_srv);
        bool udp_conn = rtludp_is_client_active(udp_srv);
        ESP_LOGI(TAG, "Status: tcp=%s udp=%s freq=%luHz rate=%luHz",
                 tcp_conn ? "YES" : "no", udp_conn ? "YES" : "no",
                 (unsigned long)rtlsdr_get_center_freq(sdr_dev),
                 (unsigned long)rtlsdr_get_sample_rate(sdr_dev));
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
