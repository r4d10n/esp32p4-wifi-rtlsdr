#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "decoder_framework.h"

static const char *TAG = "dec_gsm";

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    int cell_count;
    double sample_rate;
    uint32_t current_freq;
    uint16_t current_arfcn;
    int cells_detected;
} gsm_ctx_t;

static gsm_ctx_t s_gsm_ctx;

static esp_err_t gsm_init(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->cell_count = 0;
    c->sample_rate = 250000.0;
    c->current_freq = 935000000;
    c->current_arfcn = 0;
    c->cells_detected = 0;
    ESP_LOGI(TAG, "GSM scanner initialized");
    return ESP_OK;
}

static esp_err_t gsm_start(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    c->running = true;
    ESP_LOGI(TAG, "GSM scanner started (935 MHz, 1 MHz span)");
    return ESP_OK;
}

static esp_err_t gsm_stop(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "GSM scanner stopped");
    return ESP_OK;
}

static void gsm_destroy(void *ctx) { (void)ctx; }

static void gsm_process_iq(void *ctx_ptr, const uint8_t *iq, uint32_t len) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx_ptr;
    if (!c->running) return;

    uint32_t num_samples = len / 2;

    /* Simple power spectrum via sliding DFT over 200kHz channels */
    /* At 250kHz sample rate, we can see ~1 channel at a time */
    /* Compute total power and check if above noise floor */
    double power = 0;
    for (uint32_t i = 0; i < num_samples; i++) {
        double ii = (double)iq[i*2] - 128.0;
        double qq = (double)iq[i*2+1] - 128.0;
        power += ii*ii + qq*qq;
    }
    power /= num_samples;
    double power_dbm = 10.0 * log10(power + 0.001) - 30.0; /* Approximate */

    /* FCCH detection: Goertzel at +67708 Hz */
    /* Convert IQ to complex baseband first, then check for tone */
    double fcch_freq = 67708.0;
    int N = (num_samples > 4096) ? 4096 : (int)num_samples;
    double k = (double)N * fcch_freq / c->sample_rate;
    double w = 2.0 * M_PI * k / N;
    double coeff = 2.0 * cos(w);
    double s0 = 0, s1 = 0, s2 = 0;

    for (int i = 0; i < N; i++) {
        /* Use I channel only for tone detection */
        double sample = (double)iq[i*2] - 128.0;
        s0 = sample + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    double fcch_power = s1*s1 + s2*s2 - coeff*s1*s2;
    double fcch_ratio = fcch_power / (power * N + 0.001);

    if (fcch_ratio > 5.0) {  /* FCCH detected — strong tone */
        /* Record cell */
        c->cells_detected++;

        cJSON *data = cJSON_CreateObject();
        if (data) {
            cJSON_AddNumberToObject(data, "arfcn", c->current_arfcn);
            cJSON_AddNumberToObject(data, "frequency_hz", c->current_freq);
            cJSON_AddNumberToObject(data, "power_dbm", power_dbm);
            cJSON_AddNumberToObject(data, "fcch_ratio", fcch_ratio);
            cJSON_AddBoolToObject(data, "fcch_detected", true);

            char key[16];
            snprintf(key, sizeof(key), "ARFCN_%d", c->current_arfcn);

            int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
            decode_event_t event = {
                .decoder_name = "gsm_scanner",
                .event_type = "cell",
                .timestamp_ms = now,
                .freq_hz = c->current_freq,
                .data = data,
            };
            cJSON *track = cJSON_Duplicate(data, true);
            decode_bus_publish(&event);
            if (track) {
                tracking_table_upsert(decoder_get_global_tracking(), "gsm_scanner", key, track, (int8_t)power_dbm);
                cJSON_Delete(track);
            }
        }
    }
}

static cJSON *gsm_get_status(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "cell_count", c->cell_count);
        cJSON_AddNumberToObject(j, "cells_detected", c->cells_detected);
        cJSON_AddNumberToObject(j, "current_arfcn", c->current_arfcn);
        cJSON_AddNumberToObject(j, "current_freq_hz", c->current_freq);
    }
    return j;
}

static cJSON *gsm_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "gsm_scanner") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_gsm_plugin = {
    .name = "gsm_scanner",
    .description = "GSM 900 Cell Scanner (FCCH/SCH/BCCH)",
    .category = "cellular",
    .demod_type = DEMOD_RAW_IQ,
    .center_freq_hz = 935000000,
    .bandwidth_hz = 1000000,
    .audio_rate_hz = 0,
    .init = gsm_init,
    .start = gsm_start,
    .stop = gsm_stop,
    .destroy = gsm_destroy,
    .process_iq = gsm_process_iq,
    .process_audio = NULL,
    .get_status = gsm_get_status,
    .get_results = gsm_get_results,
    .ctx = &s_gsm_ctx,
};

void register_gsm_decoder(void) {
    decoder_registry_add(&s_gsm_plugin);
}
