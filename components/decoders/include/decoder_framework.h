/*
 * Pluggable Decoder Framework
 *
 * Generic plugin interface for signal decoders. Each decoder registers
 * a decoder_plugin_t struct describing its signal requirements and
 * providing process/output callbacks. The framework handles DDC routing,
 * tracking tables, event bus, and REST API automatically.
 *
 * To add a new decoder:
 * 1. Create decoder_xxx.c
 * 2. Fill in a decoder_plugin_t struct
 * 3. Call decoder_registry_add() in register_builtin_decoders()
 * 4. Done — REST API, tracking, notifications come for free.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════
 *  Demodulator types
 * ═══════════════════════════════════════════════════════════════ */

typedef enum {
    DEMOD_FM_WIDE,      /* WBFM: 150 kHz deviation */
    DEMOD_FM_NARROW,    /* NBFM: 5 kHz deviation */
    DEMOD_AM,           /* AM envelope detection */
    DEMOD_USB,          /* Upper sideband (CW, SSB, FT8) */
    DEMOD_LSB,          /* Lower sideband */
    DEMOD_FSK,          /* Binary FSK (configurable shift + baud) */
    DEMOD_GFSK,         /* Gaussian FSK (AIS, BLE) */
    DEMOD_OOK,          /* On-Off Keying (ISM sensors) */
    DEMOD_PSK,          /* Phase-shift keying */
    DEMOD_RAW_IQ,       /* Pass-through raw IQ (ADS-B, direct decode) */
    DEMOD_RAW_AUDIO,    /* Audio-rate real samples */
} demod_type_t;

/* ═══════════════════════════════════════════════════════════════
 *  Decode event (unified output from all decoders)
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    const char *decoder_name;    /* "adsb", "ax25_1200", "pocsag_1200" */
    const char *event_type;      /* "aircraft", "packet", "page", "tone" */
    int64_t     timestamp_ms;
    int8_t      rssi_db;
    uint32_t    freq_hz;         /* Actual frequency of decode */
    cJSON      *data;            /* Protocol-specific JSON (bus frees after dispatch) */
} decode_event_t;

/* ═══════════════════════════════════════════════════════════════
 *  Decoder plugin interface
 * ═══════════════════════════════════════════════════════════════ */

typedef struct decoder_plugin {
    /* ── Identity ─────────────────────────────────────────── */
    const char *name;            /* Unique ID: "ax25_1200", "pocsag_512" */
    const char *description;     /* Human-readable: "AX.25 1200 baud (APRS/Packet)" */
    const char *category;        /* "packet", "pager", "tone", "aviation", "marine", "iot", "digital" */

    /* ── Signal requirements ──────────────────────────────── */
    demod_type_t demod_type;     /* What demodulator to use */
    uint32_t center_freq_hz;     /* Default frequency (0 = user-configurable) */
    uint32_t bandwidth_hz;       /* DDC bandwidth needed */
    uint32_t audio_rate_hz;      /* Required audio sample rate (0 = IQ mode) */

    /* ── Demodulator parameters ───────────────────────────── */
    union {
        struct { uint16_t mark_hz; uint16_t space_hz; } afsk;
        struct { uint32_t shift_hz; uint32_t baud; } fsk;
        struct { uint32_t symbol_rate; } ook;
        struct { float tone_hz; float threshold; } tone;
    } demod_params;

    /* ── Lifecycle ────────────────────────────────────────── */
    esp_err_t (*init)(void *ctx);
    esp_err_t (*start)(void *ctx);
    esp_err_t (*stop)(void *ctx);
    void      (*destroy)(void *ctx);

    /* ── Processing (called from demod output) ────────────── */
    void (*process_audio)(void *ctx, const int16_t *samples,
                          uint32_t count, uint32_t sample_rate);
    void (*process_iq)(void *ctx, const uint8_t *iq_data, uint32_t len);

    /* ── Output (polled by REST API) ──────────────────────── */
    cJSON *(*get_status)(void *ctx);     /* Plugin status JSON */
    cJSON *(*get_results)(void *ctx);    /* Tracking data as JSON array */

    /* ── Runtime state ────────────────────────────────────── */
    void *ctx;                   /* Private state (allocated by init) */
    bool  enabled;               /* Runtime enable/disable */
    bool  running;               /* Currently processing */

    /* ── Registry linkage ─────────────────────────────────── */
    struct decoder_plugin *next;
} decoder_plugin_t;

/* ═══════════════════════════════════════════════════════════════
 *  Decoder registry
 * ═══════════════════════════════════════════════════════════════ */

/** Register a decoder plugin. Does not start it. */
esp_err_t decoder_registry_add(decoder_plugin_t *plugin);

/** Find plugin by name. Returns NULL if not found. */
decoder_plugin_t *decoder_registry_find(const char *name);

/** Iterate: first registered plugin. */
decoder_plugin_t *decoder_registry_first(void);

/** Iterate: next plugin after current. */
decoder_plugin_t *decoder_registry_next(decoder_plugin_t *current);

/** Count registered plugins. */
int decoder_registry_count(void);

/** Initialize all built-in decoders and register them. */
esp_err_t decoder_registry_init(void);

/* ═══════════════════════════════════════════════════════════════
 *  Decode bus (pub/sub event system)
 * ═══════════════════════════════════════════════════════════════ */

typedef void (*decode_listener_fn)(const decode_event_t *event, void *user_ctx);

#define DECODE_BUS_MAX_LISTENERS 8

/** Subscribe to all decode events. */
esp_err_t decode_bus_subscribe(decode_listener_fn fn, void *user_ctx);

/** Publish a decode event. data JSON ownership transfers to bus (freed after dispatch). */
esp_err_t decode_bus_publish(decode_event_t *event);

/** Initialize the decode bus. */
esp_err_t decode_bus_init(void);

/* ═══════════════════════════════════════════════════════════════
 *  Generic tracking table
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    char     decoder_name[24];
    char     key[64];            /* Unique ID: ICAO, MMSI, callsign, RIC */
    cJSON   *data;               /* Full decoded data (owned by table) */
    int64_t  first_seen_ms;
    int64_t  last_seen_ms;
    uint32_t message_count;
    int8_t   last_rssi;
} tracking_entry_t;

typedef struct {
    tracking_entry_t *entries;
    uint16_t capacity;
    uint16_t count;
    SemaphoreHandle_t mutex;
    uint32_t expiry_ms;          /* Auto-expire older than this (0 = never) */
} tracking_table_t;

/** Create a tracking table. */
tracking_table_t *tracking_table_create(uint16_t capacity, uint32_t expiry_ms);

/** Destroy a tracking table and free all entries. */
void tracking_table_destroy(tracking_table_t *table);

/** Update or insert an entry. data is duplicated internally. */
esp_err_t tracking_table_upsert(tracking_table_t *table,
                                 const char *decoder_name,
                                 const char *key,
                                 const cJSON *data,
                                 int8_t rssi);

/** Get all entries as JSON array. Caller must cJSON_Delete(). */
cJSON *tracking_table_to_json(tracking_table_t *table);

/** Get entries for a specific decoder. Caller must cJSON_Delete(). */
cJSON *tracking_table_query(tracking_table_t *table, const char *decoder_name);

/** Remove expired entries. */
void tracking_table_gc(tracking_table_t *table);

/** Get entry count. */
int tracking_table_count(tracking_table_t *table);

/* ═══════════════════════════════════════════════════════════════
 *  Channel manager (DDC allocation + demod routing)
 * ═══════════════════════════════════════════════════════════════ */

#define DECODER_MAX_CHANNELS 8

typedef struct {
    uint32_t        center_freq_hz;   /* Tuned center frequency */
    int32_t         offset_hz;        /* DDC offset from center */
    uint32_t        bandwidth_hz;     /* DDC bandwidth */
    demod_type_t    demod_type;       /* Demodulator type */
    uint32_t        audio_rate_hz;    /* Output sample rate */
    decoder_plugin_t *plugins[8];     /* Plugins attached to this channel */
    uint8_t         plugin_count;
    bool            active;
} decoder_channel_t;

/** Initialize the channel manager. */
esp_err_t decoder_channel_manager_init(void);

/** Add a channel for a plugin (auto-assigns DDC). Returns channel index. */
int decoder_channel_manager_add(decoder_plugin_t *plugin, uint32_t center_freq_hz);

/** Remove a plugin from its channel. */
esp_err_t decoder_channel_manager_remove(decoder_plugin_t *plugin);

/** Push raw IQ samples to all active channels. Called from RTL-SDR callback. */
void decoder_channel_manager_push_iq(const uint8_t *iq_data, uint32_t len,
                                      uint32_t sample_rate);

/* ═══════════════════════════════════════════════════════════════
 *  Global tracking table (shared by all decoders)
 * ═══════════════════════════════════════════════════════════════ */

/** Get the global tracking table instance. */
tracking_table_t *decoder_get_global_tracking(void);

/* ═══════════════════════════════════════════════════════════════
 *  Event log (circular buffer of recent events)
 * ═══════════════════════════════════════════════════════════════ */

#define DECODE_EVENT_LOG_SIZE 64

/** Get recent decode events as JSON array. Caller must cJSON_Delete(). */
cJSON *decode_event_log_get(void);

#ifdef __cplusplus
}
#endif
