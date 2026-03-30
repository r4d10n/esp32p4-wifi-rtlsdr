/*
 * ESP32-P4 GSM Kalibrate — RTL-SDR PPM Calibration
 *
 * USB Host → RTL-SDR → IQ Samples → GSM FCCH Detection → PPM Offset
 *
 * Scans GSM bands for active BCCH channels, detects FCCH bursts,
 * and computes the RTL-SDR frequency correction in PPM.
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
#include "gsm_kalibrate.h"

static const char *TAG = "main";

/* WiFi credentials — set via menuconfig or sdkconfig.defaults */
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASSWORD

/* Default scan band */
#define DEFAULT_BAND    KAL_BAND_GSM900

static rtlsdr_dev_t *sdr_dev = NULL;

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

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = CONFIG_RTLSDR_ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = CONFIG_RTLSDR_ETH_MDIO_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    ESP_RETURN_ON_FALSE(mac, ESP_FAIL, TAG, "MAC create failed");

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_RTLSDR_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_RTLSDR_ETH_PHY_RST_GPIO;

    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
    ESP_RETURN_ON_FALSE(phy, ESP_FAIL, TAG, "PHY create failed");

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_config, &eth_handle),
                        TAG, "ETH driver install failed");

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);

    esp_eth_netif_glue_handle_t eth_glue = esp_eth_new_netif_glue(eth_handle);
    ESP_RETURN_ON_ERROR(esp_netif_attach(eth_netif, eth_glue),
                        TAG, "Netif attach failed");

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_got_ip_handler, NULL);

    ESP_RETURN_ON_ERROR(esp_eth_start(eth_handle), TAG, "ETH start failed");

    ESP_LOGI(TAG, "Ethernet initialized (MDC=%d MDIO=%d PHY_ADDR=%d)",
             CONFIG_RTLSDR_ETH_MDC_GPIO, CONFIG_RTLSDR_ETH_MDIO_GPIO,
             CONFIG_RTLSDR_ETH_PHY_ADDR);
    return ESP_OK;
}
#endif /* CONFIG_RTLSDR_ETHERNET_ENABLE */

/* ──────────────────────── mDNS Service ──────────────────────── */

static void mdns_init_service(void)
{
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
        return;
    }

    mdns_hostname_set("esp32p4-kalibrate");
    mdns_instance_name_set("ESP32-P4 GSM Kalibrate");

    mdns_service_add(NULL, "_http", "_tcp", 8085, NULL, 0);
    ESP_LOGI(TAG, "mDNS: esp32p4-kalibrate._http._tcp:8085");
}

/* ──────────────────────── USB Host Task ──────────────────────── */

static void usb_host_task(void *arg)
{
    while (1) {
        usb_host_lib_handle_events(portMAX_DELAY, NULL);
    }
}

/* ──────────────────────── IQ Callback → Kalibrate ──────────────────────── */

static void iq_data_cb(uint8_t *buf, uint32_t len, void *ctx)
{
    (void)ctx;
    kal_push_samples(buf, len);
}

/* ──────────────────────── SDR Streaming Task ──────────────────────── */

static void sdr_stream_task(void *arg)
{
    ESP_LOGI(TAG, "Starting IQ streaming (rate=%lu, freq=%lu)",
             (unsigned long)rtlsdr_get_sample_rate(sdr_dev),
             (unsigned long)rtlsdr_get_center_freq(sdr_dev));

    esp_err_t ret = rtlsdr_read_async(sdr_dev, iq_data_cb, NULL, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Async read ended with error: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "SDR streaming stopped");
    vTaskDelete(NULL);
}

/* ──────────────────────── Main ──────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ESP32-P4 GSM Kalibrate                ║");
    ESP_LOGI(TAG, "║   RTL-SDR PPM Calibration via FCCH      ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");

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
    ret = ethernet_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Ethernet init failed: %s (WiFi still active)", esp_err_to_name(ret));
    }
#endif

    /* Wait for network */
    ESP_LOGI(TAG, "Waiting for network connection...");
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

    /* Configure SDR — initial frequency doesn't matter, scan will retune */
    rtlsdr_config_t sdr_config = RTLSDR_CONFIG_DEFAULT();
    sdr_config.sample_rate = 1024000;
    sdr_config.center_freq = 935000000;  /* Start in GSM900 range */
    ESP_ERROR_CHECK(rtlsdr_configure(sdr_dev, &sdr_config));

    /* Initialize GSM Kalibrate engine */
    kal_config_t kal_config = KAL_CONFIG_DEFAULT();
    kal_config.dev = sdr_dev;
    kal_config.band = DEFAULT_BAND;
    kal_config.threshold_db = 6.0f;
    kal_config.gain = 0;        /* AGC for scanning */
    kal_config.dwell_ms = 200;
    kal_config.fcch_bursts = 8;
    ESP_ERROR_CHECK(kal_init(&kal_config));

    /* Start SDR streaming task on Core 0 (with USB) */
    xTaskCreatePinnedToCore(sdr_stream_task, "sdr_stream", 8192, NULL, 8, NULL, 0);

    /* Allow streaming to stabilize */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Auto-start scan */
    ESP_LOGI(TAG, "Starting automatic scan of %s...",
             kal_get_band_info(DEFAULT_BAND)->name);
    ESP_ERROR_CHECK(kal_scan_start(DEFAULT_BAND));

    ESP_LOGI(TAG, "=== Kalibrate HTTP API on port 8085 ===");
    ESP_LOGI(TAG, "  GET  /api/kalibrate         — results");
    ESP_LOGI(TAG, "  GET  /api/kalibrate/status   — progress");
    ESP_LOGI(TAG, "  POST /api/kalibrate/scan     — trigger scan");

    /* Monitor scan progress */
    while (1) {
        const kal_result_t *res = kal_get_result();

        switch (res->state) {
        case KAL_STATE_SCANNING:
            ESP_LOGI(TAG, "Scanning... step %d/%d, %d channels found",
                     res->current_step, res->total_steps, res->channel_count);
            break;
        case KAL_STATE_COMPLETE:
            if (res->fcch_count > 0) {
                ESP_LOGI(TAG, "Calibration: PPM = %+.2f (stddev %.2f, %d channels)",
                         res->avg_ppm, res->stddev_ppm, res->fcch_count);
            } else {
                ESP_LOGI(TAG, "Scan complete — %d channels found, no FCCH detected",
                         res->channel_count);
            }
            break;
        case KAL_STATE_ERROR:
            ESP_LOGE(TAG, "Scan error: %s", res->status_msg);
            break;
        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
