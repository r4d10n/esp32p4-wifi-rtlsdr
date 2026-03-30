/*
 * GSM Kalibrate — RTL-SDR PPM Calibration via FCCH Detection
 *
 * Scans GSM bands for active BCCH channels, detects FCCH bursts,
 * and computes the RTL-SDR dongle's frequency offset in PPM.
 *
 * Algorithm:
 *   1. Wideband FFT power scan (1 MHz steps, 5 channels per step)
 *   2. Narrowband FCCH validation (FM discriminator variance)
 *   3. Precise frequency estimation → PPM computation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"
#include "rtlsdr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────── GSM Band Definitions ──────────────────────── */

typedef enum {
    KAL_BAND_GSM850,
    KAL_BAND_GSM900,
    KAL_BAND_DCS1800,
    KAL_BAND_PCS1900,
    KAL_BAND_LTE_B1,     /* 2110-2170 MHz (FDD Band 1) */
    KAL_BAND_LTE_B3,     /* 1805-1880 MHz (FDD Band 3) */
    KAL_BAND_LTE_B7,     /* 2620-2690 MHz (FDD Band 7) */
    KAL_BAND_LTE_B20,    /* 791-821 MHz (FDD Band 20) */
    KAL_BAND_LTE_B28,    /* 758-803 MHz (FDD Band 28, APAC) */
    KAL_BAND_COUNT
} kal_band_t;

typedef struct {
    const char *name;
    uint16_t    arfcn_start;
    uint16_t    arfcn_end;
    uint32_t    dl_freq_start;  /* Downlink start frequency (Hz) */
    uint32_t    dl_freq_end;    /* Downlink end frequency (Hz) */
} kal_band_info_t;

/* ──────────────────────── Result Structures ──────────────────────── */

#define KAL_MAX_CHANNELS  64

typedef struct {
    uint16_t arfcn;
    uint32_t freq_hz;
    float    power_dbfs;        /* Relative power in dBFS */
    bool     fcch_detected;
    float    freq_error_hz;     /* FCCH frequency error (Hz) */
    float    ppm;               /* PPM offset for this channel */
    int      burst_count;       /* Number of FCCH bursts measured */
    uint8_t  n_id_2;           /* LTE: PSS root index (0-2), GSM: unused */
    uint16_t pci;              /* LTE: Physical Cell ID (0-503), GSM: unused */
    float    confidence;        /* LTE: correlation confidence 0-1, GSM: unused */
} kal_channel_t;

typedef enum {
    KAL_STATE_IDLE,
    KAL_STATE_SCANNING,
    KAL_STATE_COMPLETE,
    KAL_STATE_ERROR
} kal_state_t;

typedef struct {
    kal_band_t      band;
    kal_channel_t   channels[KAL_MAX_CHANNELS];
    int             channel_count;
    int             fcch_count;       /* Channels with confirmed FCCH */
    float           avg_ppm;
    float           stddev_ppm;
    kal_state_t     state;
    int             current_step;
    int             total_steps;
    char            status_msg[128];
} kal_result_t;

/* ──────────────────────── Configuration ──────────────────────── */

typedef struct {
    rtlsdr_dev_t   *dev;
    kal_band_t      band;
    float           threshold_db;   /* Detection threshold above noise floor */
    int             gain;           /* Tuner gain in tenths of dB, 0 = auto */
    int             dwell_ms;       /* Dwell time per 1 MHz step (ms) */
    int             fcch_bursts;    /* Number of FCCH bursts to average */
} kal_config_t;

#define KAL_CONFIG_DEFAULT() { \
    .dev = NULL,               \
    .band = KAL_BAND_GSM900,  \
    .threshold_db = 6.0f,      \
    .gain = 0,                 \
    .dwell_ms = 200,           \
    .fcch_bursts = 8,          \
}

/* ──────────────────────── ARFCN / Frequency API ──────────────────────── */

/**
 * Get static band information for a given band.
 */
const kal_band_info_t *kal_get_band_info(kal_band_t band);

/**
 * Convert ARFCN to downlink frequency (Hz).
 * Auto-detects band from ARFCN range (DCS1800 default for 512+).
 * Returns 0 for invalid ARFCNs.
 */
uint32_t kal_arfcn_to_freq(uint16_t arfcn);

/**
 * Convert ARFCN to downlink frequency with explicit band context.
 * Required for PCS1900 (overlapping ARFCN range with DCS1800).
 */
uint32_t kal_arfcn_to_freq_band(uint16_t arfcn, kal_band_t band);

/**
 * Convert downlink frequency to ARFCN within a given band.
 * Returns 0xFFFF if frequency is outside the band.
 */
uint16_t kal_freq_to_arfcn(uint32_t freq_hz, kal_band_t band);

/**
 * Return the number of channels in a band.
 */
int kal_band_channel_count(kal_band_t band);

/**
 * Return true if the band is an LTE band.
 */
bool kal_band_is_lte(kal_band_t band);

/**
 * Convert EARFCN to downlink frequency (Hz) for supported LTE bands.
 * Returns 0 for unsupported EARFCNs.
 */
uint32_t kal_earfcn_to_freq(uint32_t earfcn);

/* ──────────────────────── Scan Control API ──────────────────────── */

/**
 * Initialize the GSM kalibrate engine.
 * Sets up capture buffer, scan task, and HTTP handlers.
 */
esp_err_t kal_init(const kal_config_t *config);

/**
 * Start scanning the specified band (or default from config).
 * Non-blocking — scan runs on a background task.
 */
esp_err_t kal_scan_start(kal_band_t band);

/**
 * Stop an in-progress scan.
 */
esp_err_t kal_scan_stop(void);

/**
 * Get current scan results (read-only pointer to internal state).
 */
const kal_result_t *kal_get_result(void);

/**
 * Push IQ samples from the RTL-SDR async callback.
 * Thread-safe — called from USB task context.
 */
void kal_push_samples(const uint8_t *iq_data, uint32_t len);

/**
 * Register HTTP API handlers on an existing server.
 *   GET  /api/kalibrate         — scan results JSON
 *   GET  /api/kalibrate/status  — scan progress JSON
 *   POST /api/kalibrate/scan    — trigger scan (body: {"band": "GSM900"})
 */
esp_err_t kal_register_http_handlers(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
