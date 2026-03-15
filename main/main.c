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
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "driver/gpio.h"
#include "rtlsdr.h"
#include "rtltcp.h"
#include "rtludp.h"
#include "websdr.h"

static const char *TAG = "main";

/* WiFi credentials — set via menuconfig or sdkconfig.defaults */
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASSWORD

/* RTL-SDR default configuration */
#define DEFAULT_FREQ        100000000   /* 100 MHz */
#define DEFAULT_SAMPLE_RATE 1024000     /* 1.024 MSPS */

static rtlsdr_dev_t    *sdr_dev = NULL;
static rtltcp_server_t *tcp_srv = NULL;
static rtludp_server_t *udp_srv = NULL;
static websdr_server_t *websdr_srv = NULL;

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

/* ──────────────────────── Ethernet Init ──────────────────────── */

#ifdef CONFIG_RTLSDR_ETHERNET_ENABLE
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    uint8_t mac[6];
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac);
        ESP_LOGI(TAG, "Ethernet Link Up (MAC: %02x:%02x:%02x:%02x:%02x:%02x)",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void eth_got_ip_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Ethernet Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

static esp_err_t ethernet_init(void)
{
    ESP_LOGI(TAG, "Initializing Ethernet (IP101 PHY, RMII)...");

    /* EMAC config */
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = CONFIG_RTLSDR_ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = CONFIG_RTLSDR_ETH_MDIO_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    ESP_RETURN_ON_FALSE(mac, ESP_FAIL, TAG, "MAC create failed");

    /* PHY config (IP101) */
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_RTLSDR_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_RTLSDR_ETH_PHY_RST_GPIO;

    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
    ESP_RETURN_ON_FALSE(phy, ESP_FAIL, TAG, "PHY create failed");

    /* Ethernet driver */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_config, &eth_handle),
                        TAG, "ETH driver install failed");

    /* Attach to TCP/IP stack */
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);

    esp_eth_netif_glue_handle_t eth_glue = esp_eth_new_netif_glue(eth_handle);
    ESP_RETURN_ON_ERROR(esp_netif_attach(eth_netif, eth_glue),
                        TAG, "Netif attach failed");

    /* Register event handlers */
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_got_ip_handler, NULL);

    /* Start Ethernet */
    ESP_RETURN_ON_ERROR(esp_eth_start(eth_handle), TAG, "ETH start failed");

    ESP_LOGI(TAG, "Ethernet initialized (MDC=%d MDIO=%d PHY_ADDR=%d)",
             CONFIG_RTLSDR_ETH_MDC_GPIO, CONFIG_RTLSDR_ETH_MDIO_GPIO,
             CONFIG_RTLSDR_ETH_PHY_ADDR);
    return ESP_OK;
}
#endif /* CONFIG_RTLSDR_ETHERNET_ENABLE */

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

    /* Set TTL for multicast */
    int ttl = 4;
    setsockopt(multicast_fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    /* Enable loopback so local listeners can receive too */
    int loop = 1;
    setsockopt(multicast_fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

    /* Destination */
    memset(&multicast_addr, 0, sizeof(multicast_addr));
    multicast_addr.sin_family = AF_INET;
    multicast_addr.sin_port = htons(CONFIG_RTLSDR_MULTICAST_PORT);
    inet_aton(CONFIG_RTLSDR_MULTICAST_GROUP, &multicast_addr.sin_addr);

    ESP_LOGI(TAG, "Multicast ready: %s:%d (TTL=%d)",
             CONFIG_RTLSDR_MULTICAST_GROUP, CONFIG_RTLSDR_MULTICAST_PORT, ttl);
    return ESP_OK;
}

/* Called from IQ callback to send multicast */
static void multicast_push_samples(const uint8_t *data, uint32_t len)
{
    if (multicast_fd < 0) return;

    /* Send raw IQ in 1024-byte chunks */
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
    (void)ctx;
    /* Push to TCP server if running */
    if (tcp_srv) {
        rtltcp_push_samples(tcp_srv, buf, len);
    }
    /* Push to UDP server if running */
    if (udp_srv) {
        rtludp_push_samples(udp_srv, buf, len);
    }
    /* Push to WebSDR server if running */
    if (websdr_srv) {
        websdr_push_samples(websdr_srv, buf, len);
    }
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

    /* This blocks until rtlsdr_stop_async() is called */
    esp_err_t ret = rtlsdr_read_async(sdr_dev, iq_data_cb, tcp_srv, 0, 0); /* use defaults: 12 x 64KB */
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

    /* Run FFT benchmark at startup (disable for production) */
#if 1
    extern void bench_fft_all(void);
    bench_fft_all();
#endif

    /* Initialize NVS (required by WiFi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize WiFi */
    ESP_ERROR_CHECK(wifi_init_sta());

#ifdef CONFIG_RTLSDR_ETHERNET_ENABLE
    /* Initialize Ethernet (runs alongside WiFi) */
    ret = ethernet_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Ethernet init failed: %s (WiFi still active)", esp_err_to_name(ret));
    }
#endif

    /* Wait for IP address (WiFi or Ethernet) */
    ESP_LOGI(TAG, "Waiting for network connection...");
    vTaskDelay(pdMS_TO_TICKS(5000));

#ifdef CONFIG_RTLSDR_MULTICAST_ENABLE
    /* Initialize Multicast (best on Ethernet) */
    multicast_init();
#endif

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

    /* Start RTL-UDP server */
    rtludp_config_t udp_config = RTLUDP_CONFIG_DEFAULT();
    udp_config.dev = sdr_dev;
    ESP_ERROR_CHECK(rtludp_server_start(&udp_srv, &udp_config));

    /* Start WebSDR server */
    websdr_config_t websdr_config = WEBSDR_CONFIG_DEFAULT();
    websdr_config.dev = sdr_dev;
    ESP_ERROR_CHECK(websdr_server_start(&websdr_srv, &websdr_config));

    /* Start SDR streaming task on Core 0 (with USB) */
    xTaskCreatePinnedToCore(sdr_stream_task, "sdr_stream", 8192, NULL, 8, NULL, 0);

    ESP_LOGI(TAG, "=== RTL-TCP server ready on port %d ===", RTLTCP_DEFAULT_PORT);
    ESP_LOGI(TAG, "=== RTL-UDP server ready on port %d ===", RTLUDP_DEFAULT_PORT);
    ESP_LOGI(TAG, "=== WebSDR server ready on port %d ===", WEBSDR_DEFAULT_PORT);
    ESP_LOGI(TAG, "Connect TCP: SDR++ / GQRX → rtl_tcp at port %d", RTLTCP_DEFAULT_PORT);
    ESP_LOGI(TAG, "Connect UDP: send any packet to port %d to subscribe", RTLUDP_DEFAULT_PORT);

    /* Main task just monitors status */
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
