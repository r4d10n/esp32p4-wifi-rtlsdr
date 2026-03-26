/*
 * ESP32-P4 Standalone Mono FM Radio
 *
 * USB Host -> RTL-SDR -> Float WBFM Demod -> I2S Audio Out
 * Web interface for tuning and control.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "usb/usb_host.h"
#include "mdns.h"
#include "rtlsdr.h"
#include "fm_demod_simple.h"
#include "audio_out_simple.h"
#include "web_radio_simple.h"

static const char *TAG = "main";

/* WiFi credentials */
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASSWORD

/* DSP parameters */
#define SDR_SAMPLE_RATE     1024000
#define SDR_CENTER_FREQ     100000000   /* 100.0 MHz */
#define SDR_GAIN            496         /* 49.6 dB */
#define AUDIO_RATE          48000

/* IQ ring buffer: USB callback (Core 0) -> FM pipeline (Core 1) */
static RingbufHandle_t iq_ringbuf = NULL;
#define IQ_RINGBUF_SIZE     (128 * 1024)

/* Audio output buffer */
#define AUDIO_BUF_SIZE      2048
static float    audio_float_buf[AUDIO_BUF_SIZE];
static int16_t  audio_s16_buf[AUDIO_BUF_SIZE];

/* Shared state */
static rtlsdr_dev_t         *sdr_dev = NULL;
static fm_demod_simple_t    *demod   = NULL;

/* Radio parameters (protected by simplicity: single writer) */
static volatile uint32_t    radio_freq   = SDR_CENTER_FREQ;
static volatile int         radio_gain   = SDR_GAIN;
static volatile uint8_t     radio_volume = 90;
static volatile bool        radio_muted  = false;

/* ── WiFi Event Handler ── */

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

/* ── mDNS ── */

static void mdns_init_service(void)
{
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
        return;
    }
    mdns_hostname_set("esp32p4-fm-radio");
    mdns_instance_name_set("ESP32-P4 FM Radio");
    mdns_service_add(NULL, "_https", "_tcp", 8080, NULL, 0);
    ESP_LOGI(TAG, "mDNS: esp32p4-fm-radio._https._tcp:8080");
}

/* ── USB Host Task ── */

static void usb_host_task(void *arg)
{
    while (1) {
        usb_host_lib_handle_events(portMAX_DELAY, NULL);
    }
}

/* ── IQ Callback (USB -> Ring Buffer) ── */

static void iq_data_cb(uint8_t *buf, uint32_t len, void *ctx)
{
    (void)ctx;
    /* Best-effort push to ring buffer; drop if full */
    xRingbufferSend(iq_ringbuf, buf, len, 0);
}

/* ── Web Radio Change Callback ── */

static void on_radio_change(const web_radio_simple_params_t *params, void *ctx)
{
    (void)ctx;

    if (params->frequency != radio_freq) {
        radio_freq = params->frequency;
        if (sdr_dev) {
            rtlsdr_set_center_freq(sdr_dev, params->frequency);
            ESP_LOGI(TAG, "Tuned to %lu Hz", (unsigned long)params->frequency);
        }
    }

    if (params->gain != radio_gain) {
        radio_gain = params->gain;
        if (sdr_dev) {
            if (params->gain == 0) {
                rtlsdr_set_tuner_gain_mode(sdr_dev, 0);  /* Auto */
            } else {
                rtlsdr_set_tuner_gain_mode(sdr_dev, 1);  /* Manual */
                rtlsdr_set_tuner_gain(sdr_dev, params->gain);
            }
            ESP_LOGI(TAG, "Gain set to %d", params->gain);
        }
    }

    if (params->volume != radio_volume) {
        radio_volume = params->volume;
        audio_out_simple_set_volume(params->volume);
    }

    if (params->muted != radio_muted) {
        radio_muted = params->muted;
        audio_out_simple_set_mute(params->muted);
    }
}

/* ── FM Pipeline Task (Core 1) ── */

static void fm_pipeline_task(void *arg)
{
    ESP_LOGI(TAG, "FM pipeline started on core %d", xPortGetCoreID());

    while (1) {
        size_t item_size = 0;
        uint8_t *iq = (uint8_t *)xRingbufferReceive(iq_ringbuf, &item_size, pdMS_TO_TICKS(100));
        if (!iq || item_size == 0) continue;

        /* Process IQ in chunks to avoid watchdog timeout.
         * Float atan2f is expensive — process max 2048 bytes (1024 IQ pairs) per chunk,
         * yielding between chunks so the IDLE task can reset the watchdog. */
        int offset = 0;
        while (offset < (int)item_size) {
            int chunk = (int)item_size - offset;
            if (chunk > 2048) chunk = 2048;  /* 1024 IQ pairs max per chunk */

            int n = fm_demod_simple_process(demod, iq + offset, chunk,
                                            audio_float_buf, AUDIO_BUF_SIZE);
            offset += chunk;

            if (n <= 0) continue;

            /* Convert float [-1,1] to int16, using full range for loud audio */
            for (int i = 0; i < n; i++) {
                float s = audio_float_buf[i] * 32000.0f;
                if (s > 32767.0f) s = 32767.0f;
                if (s < -32768.0f) s = -32768.0f;
                audio_s16_buf[i] = (int16_t)s;
            }

            /* Write to I2S audio output */
            audio_out_simple_write(audio_s16_buf, n, 200);

            /* Push to web radio for browser streaming */
            web_radio_simple_push_audio(audio_s16_buf, n);

            /* Yield to let IDLE task feed the watchdog */
            taskYIELD();
        }

        /* Return ring buffer item after all chunks processed */
        vRingbufferReturnItem(iq_ringbuf, iq);

        /* Update signal strength for web UI */
        float sig = fm_demod_simple_get_signal_strength(demod);
        int16_t sig_int = (int16_t)(sig * 32767.0f);
        if (sig_int > 32767) sig_int = 32767;

        web_radio_simple_params_t status = {
            .frequency = radio_freq,
            .gain      = radio_gain,
            .volume    = radio_volume,
            .muted     = radio_muted,
        };
        web_radio_simple_update_status(&status, sig_int);
    }
}

/* ── SDR Streaming Task (Core 0) ── */

static void sdr_stream_task(void *arg)
{
    ESP_LOGI(TAG, "Starting IQ streaming (rate=%lu, freq=%lu)",
             (unsigned long)SDR_SAMPLE_RATE,
             (unsigned long)rtlsdr_get_center_freq(sdr_dev));

    /* This blocks until rtlsdr_stop_async() is called */
    esp_err_t ret = rtlsdr_read_async(sdr_dev, iq_data_cb, NULL, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Async read ended with error: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "SDR streaming stopped");
    vTaskDelete(NULL);
}

/* ── Main ── */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 Mono FM Radio starting...");

    /* Initialize NVS (required by WiFi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize WiFi */
    ESP_ERROR_CHECK(wifi_init_sta());

    /* Wait for network */
    ESP_LOGI(TAG, "Waiting for network connection...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Start mDNS */
    mdns_init_service();

    /* Initialize IQ ring buffer in PSRAM */
    iq_ringbuf = xRingbufferCreateNoSplit(IQ_RINGBUF_SIZE, IQ_RINGBUF_SIZE);
    if (!iq_ringbuf) {
        /* Fall back to internal RAM */
        ESP_LOGW(TAG, "PSRAM ringbuf failed, using internal RAM");
        iq_ringbuf = xRingbufferCreate(IQ_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    }
    ESP_ERROR_CHECK(iq_ringbuf ? ESP_OK : ESP_ERR_NO_MEM);

    /* Initialize FM demodulator */
    demod = fm_demod_simple_create(SDR_SAMPLE_RATE, AUDIO_RATE);
    ESP_ERROR_CHECK(demod ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "FM demodulator created: %d -> %d Hz", SDR_SAMPLE_RATE, AUDIO_RATE);

    /* Initialize audio output */
    audio_out_simple_config_t audio_cfg = AUDIO_OUT_SIMPLE_CONFIG_DEFAULT();
    audio_cfg.sample_rate = AUDIO_RATE;
    audio_cfg.volume = radio_volume;
    audio_cfg.speaker_enable = true;
    ESP_ERROR_CHECK(audio_out_simple_init(&audio_cfg));

    /* Initialize web radio */
    web_radio_simple_config_t web_cfg = WEB_RADIO_SIMPLE_CONFIG_DEFAULT();
    web_cfg.http_port = 8080;
    web_cfg.change_cb = on_radio_change;
    web_cfg.cb_ctx = NULL;
    ESP_ERROR_CHECK(web_radio_simple_start(&web_cfg));

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
    sdr_config.center_freq = SDR_CENTER_FREQ;
    sdr_config.sample_rate = SDR_SAMPLE_RATE;
    sdr_config.gain = SDR_GAIN;
    sdr_config.agc_mode = false;
    ESP_ERROR_CHECK(rtlsdr_configure(sdr_dev, &sdr_config));

    /* Set manual gain */
    rtlsdr_set_tuner_gain_mode(sdr_dev, 1);
    rtlsdr_set_tuner_gain(sdr_dev, SDR_GAIN);

    /* Start FM pipeline task on Core 1 */
    xTaskCreatePinnedToCore(fm_pipeline_task, "fm_pipe", 16384, NULL, 8, NULL, 1);

    /* Start SDR streaming task on Core 0 */
    xTaskCreatePinnedToCore(sdr_stream_task, "sdr_stream", 8192, NULL, 8, NULL, 0);

    ESP_LOGI(TAG, "=== FM Radio ready ===");
    ESP_LOGI(TAG, "=== Web UI: https://esp32p4-fm-radio.local:8080 ===");
    ESP_LOGI(TAG, "=== Frequency: %.3f MHz, Gain: %.1f dB ===",
             (float)SDR_CENTER_FREQ / 1e6f, (float)SDR_GAIN / 10.0f);

    /* Main task monitors status */
    while (1) {
        float sig = fm_demod_simple_get_signal_strength(demod);
        ESP_LOGI(TAG, "Status: freq=%.3f MHz signal=%.1f%% vol=%d",
                 (float)radio_freq / 1e6f,
                 sig * 100.0f,
                 radio_volume);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
