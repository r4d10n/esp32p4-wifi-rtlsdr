/*
 * ESP32-P4 Standalone FM Radio
 *
 * USB Host → RTL-SDR → DDC → FM Demod → I2S Audio Out
 * Web interface for tuning and control.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
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
#include "rtlsdr.h"
#include "pie_kernels.h"
#include "fm_demod.h"
#include "audio_out.h"
#include "web_radio.h"
#ifdef CONFIG_FM_STEREO_ENABLE
#include "fm_stereo.h"
#include "rds_decoder.h"
#endif
#include "ui_hw.h"
#include "nvs_settings.h"
#ifdef CONFIG_FM_USB_AUDIO_ENABLE
#include "usb_audio.h"
#endif
#include "civ_emu.h"
#include "dsp.h"

static const char *TAG = "main";

/* WiFi credentials */
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASSWORD

/* DSP buffer size (max IQ pairs per block) */
#define DSP_BUF_ENTRIES 8192

/* ── Radio State ── */

typedef enum {
    FM_MODE_WBFM = 0,
    FM_MODE_NBFM = 1,
} fm_mode_t;

typedef struct {
    uint32_t    frequency;      /* Center frequency in Hz */
    uint32_t    sample_rate;    /* SDR sample rate */
    int         gain;           /* Tuner gain in tenths of dB (0=auto) */
    uint8_t     volume;         /* 0-100 */
    fm_mode_t   mode;           /* WBFM or NBFM */
    uint32_t    filter_bw;      /* Audio filter bandwidth in Hz */
    bool        muted;
    uint8_t     squelch;        /* 0=off, 1-100 */
    bool        nb_enabled;
    uint8_t     nb_threshold;
} radio_state_t;

static radio_state_t radio = {
    .frequency    = 100000000,   /* 100.0 MHz */
    .sample_rate  = 1024000,     /* 1.024 MSPS */
    .gain         = 0,           /* Auto gain */
    .volume       = 70,
    .mode         = FM_MODE_WBFM,
    .filter_bw    = 15000,       /* 15 kHz for WBFM */
    .muted        = false,
    .squelch      = 0,
    .nb_enabled   = true,
    .nb_threshold = 5,
};

static rtlsdr_dev_t *sdr_dev = NULL;

/* IQ ring buffer: USB callback (Core 0) → FM pipeline (Core 1) */
static RingbufHandle_t iq_ringbuf = NULL;
#define IQ_RINGBUF_SIZE     (128 * 1024)

/* ── Scan State ── */

static volatile bool        scan_active       = false;
static volatile int         scan_direction    = 0;  /* +1 or -1 */
static TaskHandle_t         scan_task_handle  = NULL;

#define SCAN_STEP_HZ    100000      /* 100 kHz steps */
#define SCAN_DWELL_MS   200         /* Listen at each freq for 200ms */
#define SCAN_THRESHOLD  5000        /* Signal strength threshold */
#define SCAN_FM_LOW     88000000
#define SCAN_FM_HIGH    108000000

/* ── Radio Pipeline ── */

typedef struct {
    pie_nco_t      *nco;           /* NCO for frequency offset mixing */
    int32_t         cic_state[26]; /* CIC decimator persistent state */
    fm_demod_t     *demod;         /* FM demodulator */

    /* Aligned scratch buffers */
    int16_t        *iq_s16;        /* uint8→int16 converted IQ [16-byte aligned] */
    int16_t        *iq_mixed;      /* NCO-mixed IQ [16-byte aligned] */
    int16_t        *iq_decim;      /* Decimated IQ [16-byte aligned] */
    int16_t        *audio_buf;     /* Demodulated audio output */

    int             decim_ratio;   /* DDC decimation ratio */
    uint32_t        ddc_out_rate;  /* Post-decimation sample rate */

#ifdef CONFIG_FM_STEREO_ENABLE
    fm_stereo_t    *stereo;        /* Stereo MPX decoder */
    int16_t        *mpx_buf;       /* Raw discriminator output [16-byte aligned] */
    int16_t        *stereo_buf;    /* Interleaved stereo output [16-byte aligned] */
    bool            stereo_active; /* I2S is currently in stereo mode */
#endif
} radio_pipeline_t;

static radio_pipeline_t pipeline;

/* ── CI-V Emulator ── */

static civ_emu_t *civ_emu = NULL;

static void civ_on_change(const civ_params_t *params, void *ctx)
{
    (void)ctx;
    if (params->frequency != radio.frequency) {
        radio.frequency = params->frequency;
        rtlsdr_set_center_freq(sdr_dev, params->frequency);
        fm_demod_reset(pipeline.demod);
    }
    if (params->mode != (int)radio.mode) {
        radio.mode = params->mode == 0 ? FM_MODE_WBFM : FM_MODE_NBFM;
        fm_demod_set_mode(pipeline.demod, radio.mode == FM_MODE_WBFM ? FM_DEMOD_WBFM : FM_DEMOD_NBFM);
    }
    if (params->volume != radio.volume) {
        radio.volume = params->volume;
        audio_out_set_volume(params->volume);
    }
    if (params->squelch != radio.squelch) {
        radio.squelch = params->squelch;
        fm_demod_set_squelch(pipeline.demod, params->squelch);
    }
}

/* ── WiFi ── */

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
    ESP_RETURN_ON_ERROR(esp_hosted_connect_to_slave(), TAG, "esp_hosted_connect failed");

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
    return ESP_OK;
}

/* ── USB Host ── */

static void usb_host_task(void *arg)
{
    while (1) {
        usb_host_lib_handle_events(portMAX_DELAY, NULL);
    }
}

/* ── IQ Callback (USB → Ring Buffer) ── */

static void iq_data_cb(uint8_t *buf, uint32_t len, void *ctx)
{
    (void)ctx;
    /* Push to ring buffer — drop if full (non-blocking) */
    xRingbufferSend(iq_ringbuf, buf, len, 0);
}

/* ── SDR Streaming Task ── */

static void sdr_stream_task(void *arg)
{
    ESP_LOGI(TAG, "Starting IQ streaming (rate=%lu, freq=%lu)",
             (unsigned long)rtlsdr_get_sample_rate(sdr_dev),
             (unsigned long)rtlsdr_get_center_freq(sdr_dev));

    esp_err_t ret = rtlsdr_read_async(sdr_dev, iq_data_cb, NULL, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Async read error: %s", esp_err_to_name(ret));
    }
    vTaskDelete(NULL);
}

/* ── Pipeline Init ── */

static esp_err_t pipeline_init(radio_pipeline_t *pipe, fm_mode_t mode)
{
    memset(pipe, 0, sizeof(*pipe));

    /* DDC parameters based on mode */
    if (mode == FM_MODE_WBFM) {
        /* WBFM: 1024kSPS → 256kSPS (ratio 4), demod at 256kSPS → 48kHz audio */
        pipe->decim_ratio  = 4;
        pipe->ddc_out_rate = 256000;
    } else {
        /* NBFM: 1024kSPS → 32kSPS (ratio 32), demod at 32kSPS → 48kHz audio */
        pipe->decim_ratio  = 32;
        pipe->ddc_out_rate = 32000;
    }

    /* NCO — offset 0 initially; tune adjusts via pie_nco_create */
    pipe->nco = pie_nco_create(radio.sample_rate, 0);
    if (!pipe->nco) {
        ESP_LOGE(TAG, "Failed to create NCO");
        return ESP_ERR_NO_MEM;
    }

    /* FM demodulator */
    fm_demod_config_t demod_cfg;
    if (mode == FM_MODE_WBFM) {
        demod_cfg = (fm_demod_config_t)FM_DEMOD_CONFIG_WBFM();
    } else {
        demod_cfg = (fm_demod_config_t)FM_DEMOD_CONFIG_NBFM();
    }
    pipe->demod = fm_demod_create(&demod_cfg);
    if (!pipe->demod) {
        ESP_LOGE(TAG, "Failed to create FM demodulator");
        pie_nco_free(pipe->nco);
        return ESP_ERR_NO_MEM;
    }

    /* Allocate 16-byte aligned scratch buffers for PIE SIMD */
    pipe->iq_s16    = heap_caps_aligned_alloc(16, DSP_BUF_ENTRIES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    pipe->iq_mixed  = heap_caps_aligned_alloc(16, DSP_BUF_ENTRIES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    pipe->iq_decim  = heap_caps_aligned_alloc(16, DSP_BUF_ENTRIES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    pipe->audio_buf = heap_caps_aligned_alloc(16, DSP_BUF_ENTRIES * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    if (!pipe->iq_s16 || !pipe->iq_mixed || !pipe->iq_decim || !pipe->audio_buf) {
        ESP_LOGE(TAG, "Failed to allocate DSP buffers");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Pipeline init: mode=%s decim=%d ddc_out=%luHz",
             mode == FM_MODE_WBFM ? "WBFM" : "NBFM",
             pipe->decim_ratio, (unsigned long)pipe->ddc_out_rate);

#ifdef CONFIG_FM_STEREO_ENABLE
    if (mode == FM_MODE_WBFM) {
        fm_stereo_config_t stereo_cfg = FM_STEREO_CONFIG_DEFAULT();
        stereo_cfg.sample_rate = pipe->ddc_out_rate;
        pipe->stereo = fm_stereo_create(&stereo_cfg);
        if (!pipe->stereo) {
            ESP_LOGW(TAG, "Stereo decoder creation failed, mono only");
        } else {
            /* Buffers for stereo path */
            pipe->mpx_buf    = heap_caps_aligned_alloc(16, DSP_BUF_ENTRIES * sizeof(int16_t), MALLOC_CAP_DEFAULT);
            pipe->stereo_buf = heap_caps_aligned_alloc(16, DSP_BUF_ENTRIES * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
            if (!pipe->mpx_buf || !pipe->stereo_buf) {
                ESP_LOGW(TAG, "Stereo buffers allocation failed, mono only");
                fm_stereo_free(pipe->stereo);
                pipe->stereo = NULL;
                heap_caps_free(pipe->mpx_buf);
                heap_caps_free(pipe->stereo_buf);
                pipe->mpx_buf = NULL;
                pipe->stereo_buf = NULL;
            } else {
                ESP_LOGI(TAG, "Stereo decoder initialized");
            }
        }
        pipe->stereo_active = false;
    }
#endif

    return ESP_OK;
}

/* ── FM Pipeline Task ── */

static void fm_pipeline_task(void *arg)
{
    radio_pipeline_t *pipe = (radio_pipeline_t *)arg;

    ESP_LOGI(TAG, "FM pipeline started on Core 1");

    while (1) {
        size_t item_size = 0;
        uint8_t *iq_data = xRingbufferReceive(iq_ringbuf, &item_size, pdMS_TO_TICKS(100));
        if (!iq_data || item_size < 2) {
            continue;
        }

        int iq_pairs = item_size / 2;
        /* Clamp to buffer size (each IQ pair = 2 int16 samples) */
        if (iq_pairs > DSP_BUF_ENTRIES / 2) {
            iq_pairs = DSP_BUF_ENTRIES / 2;
        }

        /* Step 1: uint8 IQ → int16 with bias removal */
        pie_u8_to_s16_bias(iq_data, pipe->iq_s16, iq_pairs * 2);

        /* Step 2: NCO mix (frequency shift) — PIE SIMD accelerated */
        pie_nco_mix_s16(pipe->iq_s16, pipe->nco, pipe->iq_mixed, iq_pairs);

        /* Step 3: CIC decimation */
        int decim_pairs = DSP_BUF_ENTRIES / 2;  /* max output pairs */
        pie_cic_decimate_s16(pipe->iq_mixed, iq_pairs,
                              pipe->iq_decim, &decim_pairs,
                              pipe->decim_ratio, pipe->cic_state);

        /* Step 4: FM demodulation → audio */
        if (decim_pairs > 0) {
#ifdef CONFIG_FM_STEREO_ENABLE
            if (pipe->stereo) {
                /* Stereo path: discriminator only → stereo decoder handles the rest */
                int mpx_count = fm_demod_discriminate(pipe->demod, pipe->iq_decim, decim_pairs,
                                                       pipe->mpx_buf, DSP_BUF_ENTRIES / 2);
                if (mpx_count > 0) {
                    int stereo_pairs = fm_stereo_process(pipe->stereo, pipe->mpx_buf, mpx_count,
                                                          pipe->stereo_buf, DSP_BUF_ENTRIES / 2);

                    /* Switch I2S mode if stereo status changed */
                    bool is_stereo = fm_stereo_is_stereo(pipe->stereo);
                    if (is_stereo != pipe->stereo_active) {
                        audio_out_set_stereo(is_stereo);
                        pipe->stereo_active = is_stereo;
                    }

                    if (stereo_pairs > 0 && !radio.muted) {
                        /* Volume: apply to interleaved L/R (2 samples per pair) */
                        fm_demod_apply_volume(pipe->stereo_buf, stereo_pairs * 2, radio.volume);
                        audio_out_write(pipe->stereo_buf, stereo_pairs * 2, 50);
#ifdef CONFIG_FM_USB_AUDIO_ENABLE
                        /* USB audio is mono; send left channel sample count */
                        usb_audio_write(pipe->stereo_buf, stereo_pairs * 2);
#endif
                    }
                }
            } else
#endif
            {
                /* Mono path: full demod pipeline */
                int audio_count = fm_demod_process(pipe->demod, pipe->iq_decim, decim_pairs,
                                                    pipe->audio_buf, DSP_BUF_ENTRIES / 2);

                if (audio_count > 0 && !radio.muted) {
                    /* Step 5: Apply volume */
                    fm_demod_apply_volume(pipe->audio_buf, audio_count, radio.volume);

                    /* Step 6: Write to I2S */
                    audio_out_write(pipe->audio_buf, audio_count, 50);
#ifdef CONFIG_FM_USB_AUDIO_ENABLE
                    usb_audio_write(pipe->audio_buf, audio_count);
#endif
                }
            }
        }

        vRingbufferReturnItem(iq_ringbuf, iq_data);
    }
}

/* ── Runtime Parameter Helpers ── */

static void radio_set_frequency(uint32_t freq_hz); /* forward declaration */

/* ── Scan Task ── */

static void scan_task(void *arg)
{
    int dir = (int)(intptr_t)arg;
    uint32_t start_freq = radio.frequency;
    uint32_t freq = start_freq;

    ESP_LOGI(TAG, "Scan started: %s from %.3f MHz",
             dir > 0 ? "UP" : "DOWN", freq / 1e6);

    while (scan_active) {
        freq += dir * SCAN_STEP_HZ;

        /* Wrap around */
        if (freq > SCAN_FM_HIGH) freq = SCAN_FM_LOW;
        if (freq < SCAN_FM_LOW)  freq = SCAN_FM_HIGH;

        /* Check if we've scanned the full band */
        if (freq == start_freq) {
            ESP_LOGI(TAG, "Scan: full band scanned, no station found");
            break;
        }

        /* Tune to new frequency */
        radio_set_frequency(freq);

        /* Dwell and measure signal */
        vTaskDelay(pdMS_TO_TICKS(SCAN_DWELL_MS));

        int16_t sig = fm_demod_get_signal_strength(pipeline.demod);

        if (sig > SCAN_THRESHOLD) {
            /* Confirm with a second measurement */
            vTaskDelay(pdMS_TO_TICKS(100));
            sig = fm_demod_get_signal_strength(pipeline.demod);
            if (sig > SCAN_THRESHOLD) {
                ESP_LOGI(TAG, "Scan: found station at %.3f MHz (sig=%d)",
                         freq / 1e6, sig);
                break;
            }
        }
    }

    scan_active = false;
    scan_task_handle = NULL;
    vTaskDelete(NULL);
}

static void scan_start(int direction)
{
    if (scan_active) return; /* Already scanning */
    scan_active = true;
    scan_direction = direction;
    xTaskCreate(scan_task, "scan", 4096, (void *)(intptr_t)direction, 3, &scan_task_handle);
}

static void scan_stop(void)
{
    scan_active = false;
    /* Task will self-delete */
}

/* Called from web API to change frequency */
static void radio_set_frequency(uint32_t freq_hz)
{
    radio.frequency = freq_hz;
    rtlsdr_set_center_freq(sdr_dev, freq_hz);
    fm_demod_reset(pipeline.demod);
}

/* Called from web API to change mode */
static void radio_set_mode(fm_mode_t mode)
{
    radio.mode = mode;
    fm_demod_set_mode(pipeline.demod, mode == FM_MODE_WBFM ? FM_DEMOD_WBFM : FM_DEMOD_NBFM);

    /* Update DDC decimation parameters */
    if (mode == FM_MODE_WBFM) {
        pipeline.decim_ratio  = 4;
        pipeline.ddc_out_rate = 256000;
    } else {
        pipeline.decim_ratio  = 32;
        pipeline.ddc_out_rate = 32000;
    }

    /* Reset CIC state on mode change */
    memset(pipeline.cic_state, 0, sizeof(pipeline.cic_state));
    fm_demod_reset(pipeline.demod);
}

/* ── Web Radio Parameter Change Callback ── */

static void web_radio_on_change(const web_radio_params_t *params, void *ctx)
{
    (void)ctx;
    if (params->frequency != radio.frequency) {
        radio_set_frequency(params->frequency);
    }
    if (params->mode != (int)radio.mode) {
        radio_set_mode(params->mode == 0 ? FM_MODE_WBFM : FM_MODE_NBFM);
    }
    if (params->gain != radio.gain) {
        radio.gain = params->gain;
        if (params->gain == 0) {
            rtlsdr_set_tuner_gain_mode(sdr_dev, 0);
        } else {
            rtlsdr_set_tuner_gain_mode(sdr_dev, 1);
            rtlsdr_set_tuner_gain(sdr_dev, params->gain);
        }
    }
    if (params->volume != radio.volume) {
        radio.volume = params->volume;
        audio_out_set_volume(params->volume);
    }
    if (params->muted != radio.muted) {
        radio.muted = params->muted;
        audio_out_set_mute(params->muted);
    }
    if (params->squelch != radio.squelch) {
        radio.squelch = params->squelch;
        fm_demod_set_squelch(pipeline.demod, params->squelch);
    }
    if (params->nb_enabled != radio.nb_enabled || params->nb_threshold != radio.nb_threshold) {
        radio.nb_enabled   = params->nb_enabled;
        radio.nb_threshold = params->nb_threshold;
        fm_demod_set_noise_blanker(pipeline.demod, radio.nb_enabled, radio.nb_threshold);
    }
    radio.filter_bw = params->filter_bw;

    /* Persist settings to NVS (debounced) */
    nvs_radio_settings_t to_save = {
        .frequency    = radio.frequency,
        .gain         = radio.gain,
        .volume       = radio.volume,
        .mode         = (int)radio.mode,
        .filter_bw    = radio.filter_bw,
        .squelch      = radio.squelch,
        .nb_enabled   = radio.nb_enabled,
        .nb_threshold = radio.nb_threshold,
    };
    nvs_settings_save(&to_save);

    /* Handle scan requests */
    if (params->scan_request == 1) {
        scan_start(1);
    } else if (params->scan_request == -1) {
        scan_start(-1);
    } else if (params->scan_request == 2) {
        scan_stop();
    }
}

/* ── Main ── */

void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ESP32-P4 Standalone FM Radio       ║");
    ESP_LOGI(TAG, "║   %.1f MHz | %s                     ║",
             radio.frequency / 1e6,
             radio.mode == FM_MODE_WBFM ? "WBFM" : "NBFM");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Load saved radio settings from NVS */
    nvs_settings_init();
    nvs_radio_settings_t saved;
    /* Set defaults first */
    saved.frequency  = radio.frequency;
    saved.gain       = radio.gain;
    saved.volume     = radio.volume;
    saved.mode       = (int)radio.mode;
    saved.filter_bw  = radio.filter_bw;
    saved.squelch    = radio.squelch;
    saved.nb_enabled = radio.nb_enabled;
    saved.nb_threshold = radio.nb_threshold;

    if (nvs_settings_load(&saved) == ESP_OK) {
        radio.frequency   = saved.frequency;
        radio.gain        = saved.gain;
        radio.volume      = saved.volume;
        radio.mode        = (fm_mode_t)saved.mode;
        radio.filter_bw   = saved.filter_bw;
        radio.squelch     = saved.squelch;
        radio.nb_enabled  = saved.nb_enabled;
        radio.nb_threshold = saved.nb_threshold;
        ESP_LOGI(TAG, "Restored: %.3f MHz, %s, vol=%d",
                 radio.frequency / 1e6, radio.mode == FM_MODE_WBFM ? "WBFM" : "NBFM", radio.volume);
    }

    /* WiFi */
    ESP_ERROR_CHECK(wifi_init_sta());
    ESP_LOGI(TAG, "Waiting for network...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* IQ Ring Buffer */
    iq_ringbuf = xRingbufferCreate(IQ_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    assert(iq_ringbuf);

    /* USB Host */
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskCreatePinnedToCore(usb_host_task, "usb_host", 4096, NULL, 10, NULL, 0);

    /* RTL-SDR Init */
    ret = rtlsdr_init(&sdr_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTL-SDR init failed: %s", esp_err_to_name(ret));
        return;
    }

    rtlsdr_config_t sdr_config = RTLSDR_CONFIG_DEFAULT();
    sdr_config.center_freq = radio.frequency;
    sdr_config.sample_rate = radio.sample_rate;
    sdr_config.gain = radio.gain;
    ESP_ERROR_CHECK(rtlsdr_configure(sdr_dev, &sdr_config));

    /* Audio Output Init */
    audio_out_config_t audio_cfg = AUDIO_OUT_CONFIG_DEFAULT();
    audio_cfg.volume = radio.volume;
    ESP_ERROR_CHECK(audio_out_init(&audio_cfg));

#ifdef CONFIG_FM_USB_AUDIO_ENABLE
    /* USB Audio Device (UAC 2.0 + CDC) on Controller 1 */
    ESP_ERROR_CHECK(usb_audio_init());
#endif

    /* DSP Pipeline Init */
    ESP_ERROR_CHECK(pipeline_init(&pipeline, radio.mode));

#ifdef CONFIG_FM_BENCH_ENABLE
    extern void bench_dsp_pipeline(void);
    bench_dsp_pipeline();
#endif

#ifdef CONFIG_FM_TEST_ENABLE
    extern void test_dsp_kernels(void);
    extern void test_fm_demod(void);
    extern void test_fm_stereo_rds(void);
    extern void test_pipeline_e2e(void);
    extern void test_phase4(void);
    test_dsp_kernels();
    test_fm_demod();
    test_fm_stereo_rds();
    test_pipeline_e2e();
    test_phase4();
#endif

#ifdef CONFIG_FM_OLED_ENABLE
    ui_hw_config_t oled_cfg = {
        .encoder_cb = NULL,  /* TODO: wire encoder events */
        .cb_ctx = NULL,
    };
    esp_err_t oled_ret = ui_hw_init(&oled_cfg);
    if (oled_ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED init failed: %s (continuing without display)", esp_err_to_name(oled_ret));
    }
#endif

    /* CI-V Protocol Emulator (Icom IC-R8600) */
    civ_emu = civ_emu_create(civ_on_change, NULL);
    if (!civ_emu) {
        ESP_LOGW(TAG, "CI-V emulator creation failed (continuing without CAT control)");
    }

    /* Web Radio Control Interface */
    web_radio_config_t web_cfg = WEB_RADIO_CONFIG_DEFAULT();
    web_cfg.change_cb = web_radio_on_change;
    web_cfg.cb_ctx = NULL;
    ESP_ERROR_CHECK(web_radio_start(&web_cfg));

    /* Start FM pipeline on Core 1 */
    xTaskCreatePinnedToCore(fm_pipeline_task, "fm_pipe", 8192, &pipeline, 8, NULL, 1);

    /* Start SDR streaming on Core 0 (with USB) */
    xTaskCreatePinnedToCore(sdr_stream_task, "sdr_stream", 8192, NULL, 9, NULL, 0);

    ESP_LOGI(TAG, "FM Radio running — tune via web interface on port 8080");

    /* Status monitor + web UI update */
    while (1) {
        /* Update web UI with current state */
        bool is_stereo = false;
#ifdef CONFIG_FM_STEREO_ENABLE
        if (pipeline.stereo) {
            is_stereo = fm_stereo_is_stereo(pipeline.stereo);
        }
#endif
        web_radio_params_t wp = {
            .frequency    = radio.frequency,
            .gain         = radio.gain,
            .volume       = radio.volume,
            .mode         = (int)radio.mode,
            .filter_bw    = radio.filter_bw,
            .muted        = radio.muted,
            .squelch      = radio.squelch,
            .nb_enabled   = radio.nb_enabled,
            .nb_threshold = radio.nb_threshold,
            .stereo       = is_stereo,
        };
        int16_t sig = fm_demod_get_signal_strength(pipeline.demod);
        web_radio_update_status(&wp, sig);

        /* Update CI-V emulator state */
        if (civ_emu) {
            civ_emu_update_state(civ_emu, radio.frequency, (int)radio.mode,
                                  sig, radio.volume, radio.squelch);
        }

        /* Send spectrum data via CI-V scope command */
        if (civ_emu) {
            static int scope_counter = 0;
            /* Status loop runs every 5000ms; send scope at CONFIG_FM_SCOPE_INTERVAL_MS */
            if (++scope_counter >= (5000 / CONFIG_FM_SCOPE_INTERVAL_MS)) {
                scope_counter = 0;
                int fft_size = dsp_fft_get_size();
                if (fft_size > 0) {
                    uint8_t *fft_buf = malloc(fft_size);
                    if (fft_buf) {
                        int fft_len = dsp_fft_get_spectrum(fft_buf, fft_size);
                        if (fft_len > 0) {
                            int frame_max = fft_len + 17;
                            uint8_t *scope_frame = malloc(frame_max);
                            if (scope_frame) {
                                int frame_len = civ_emu_make_scope_frame(
                                    civ_emu, fft_buf, fft_len,
                                    radio.frequency, radio.sample_rate,
                                    scope_frame, frame_max);
                                if (frame_len > 0) {
                                    /* TODO: send scope_frame over USB CDC serial */
                                    ESP_LOGD(TAG, "Scope frame: %d bytes", frame_len);
                                }
                                free(scope_frame);
                            }
                        }
                        free(fft_buf);
                    }
                }
            }
        }

#ifdef CONFIG_FM_STEREO_ENABLE
        if (pipeline.stereo) {
            rds_data_t rds;
            fm_stereo_get_rds(pipeline.stereo, &rds);
            web_radio_update_rds(rds.ps_name, rds.radio_text,
                                 rds.pi_code, rds.pty, is_stereo);
        }
#endif

#ifdef CONFIG_FM_OLED_ENABLE
        ui_hw_state_t oled_state = {
            .frequency = radio.frequency,
            .mode = (int)radio.mode,
            .volume = radio.volume,
            .signal_strength = sig,
            .muted = radio.muted,
            .gain = radio.gain,
        };
        ui_hw_update(&oled_state);

        /* Poll encoder */
        ui_hw_event_t evt = ui_hw_poll_encoder();
        if (evt == UI_HW_EVT_CW) radio_set_frequency(radio.frequency + 100000);
        else if (evt == UI_HW_EVT_CCW) radio_set_frequency(radio.frequency - 100000);
        else if (evt == UI_HW_EVT_PRESS) radio_set_mode(radio.mode == FM_MODE_WBFM ? FM_MODE_NBFM : FM_MODE_WBFM);
        else if (evt == UI_HW_EVT_LONG_PRESS) { radio.muted = !radio.muted; audio_out_set_mute(radio.muted); }
#endif

        ESP_LOGI(TAG, "Status: freq=%.3fMHz mode=%s%s vol=%d gain=%d sig=%d",
                 radio.frequency / 1e6,
                 radio.mode == FM_MODE_WBFM ? "WBFM" : "NBFM",
                 is_stereo ? " [STEREO]" : "",
                 radio.volume, radio.gain, sig);
        ESP_LOGI(TAG, "Heap: free=%lu min=%lu PSRAM=%lu",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size(),
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
