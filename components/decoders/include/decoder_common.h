#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Common decoder state */
typedef enum {
    DECODER_STOPPED = 0,
    DECODER_RUNNING,
    DECODER_ERROR,
} decoder_state_t;

typedef struct {
    const char *name;
    uint32_t freq_hz;
    uint32_t sample_rate;
    decoder_state_t state;
    TaskHandle_t task;
    void (*push_samples)(const uint8_t *data, uint32_t len);
    void *user_data;
} decoder_ctx_t;

/* ADS-B */
typedef struct {
    uint32_t icao;
    char callsign[9];
    double lat, lon;
    int32_t altitude_ft;
    uint16_t heading;
    uint16_t speed_kt;
    int64_t last_seen_ms;
} adsb_aircraft_t;

esp_err_t decoder_adsb_init(void);
esp_err_t decoder_adsb_start(void);
esp_err_t decoder_adsb_stop(void);
void decoder_adsb_push_samples(const uint8_t *data, uint32_t len);
int decoder_adsb_get_aircraft(adsb_aircraft_t *out, int max_count);

/* AIS */
typedef struct {
    uint32_t mmsi;
    char vessel_name[21];
    double lat, lon;
    float speed_knots;
    float course;
    int64_t last_seen_ms;
} ais_vessel_t;

esp_err_t decoder_ais_init(void);
esp_err_t decoder_ais_start(void);
esp_err_t decoder_ais_stop(void);
void decoder_ais_push_samples(const uint8_t *data, uint32_t len);
int decoder_ais_get_vessels(ais_vessel_t *out, int max_count);

/* APRS */
typedef struct {
    char callsign[12];
    double lat, lon;
    char comment[64];
    int64_t last_seen_ms;
} aprs_station_t;

esp_err_t decoder_aprs_init(void);
esp_err_t decoder_aprs_start(void);
esp_err_t decoder_aprs_stop(void);
void decoder_aprs_push_samples(const uint8_t *data, uint32_t len);
int decoder_aprs_get_stations(aprs_station_t *out, int max_count);

/* GSM Scanner */
typedef struct {
    uint16_t arfcn;
    uint16_t mcc, mnc;
    uint16_t lac;
    uint32_t cid;
    int8_t signal_dbm;
    int64_t last_seen_ms;
} gsm_cell_t;

esp_err_t decoder_gsm_init(void);
esp_err_t decoder_gsm_start(void);
esp_err_t decoder_gsm_stop(void);
void decoder_gsm_push_samples(const uint8_t *data, uint32_t len);
int decoder_gsm_get_cells(gsm_cell_t *out, int max_count);

/* FT8/WSPR */
typedef struct {
    char callsign[12];
    char grid[7];
    int8_t snr;
    uint32_t freq_hz;
    int64_t timestamp_ms;
} ft8_decode_t;

esp_err_t decoder_ft8_init(void);
esp_err_t decoder_ft8_start(void);
esp_err_t decoder_ft8_stop(void);
void decoder_ft8_push_samples(const uint8_t *data, uint32_t len);
int decoder_ft8_get_decodes(ft8_decode_t *out, int max_count);
