#pragma once
#include <stdint.h>

#define SPYSERVER_PROTOCOL_VERSION    (((2) << 24) | ((0) << 16) | (1700))
#define SPYSERVER_MAX_MESSAGE_BODY    (1024 * 1024)
#define SPYSERVER_MAX_COMMAND_BODY    256
#define SPYSERVER_INITIAL_FFT_BW      655360

/* Device types */
typedef enum {
    DEVICE_INVALID = 0,
    DEVICE_AIRSPY_ONE = 1,
    DEVICE_AIRSPY_HF = 2,
    DEVICE_RTLSDR = 3,
} spyserver_device_type_t;

/* Message types (server -> client) */
typedef enum {
    MSG_TYPE_DEVICE_INFO = 0,
    MSG_TYPE_CLIENT_SYNC = 1,
    MSG_TYPE_PONG = 2,
    MSG_TYPE_READ_SETTING = 3,
    MSG_TYPE_UINT8_IQ = 100,
    MSG_TYPE_INT16_IQ = 101,
    MSG_TYPE_INT24_IQ = 102,
    MSG_TYPE_FLOAT_IQ = 103,
    MSG_TYPE_UINT8_AF = 200,
    MSG_TYPE_INT16_AF = 201,
    MSG_TYPE_INT24_AF = 202,
    MSG_TYPE_FLOAT_AF = 203,
    MSG_TYPE_DINT4_FFT = 300,
    MSG_TYPE_UINT8_FFT = 301,
    MSG_TYPE_INT16_FFT = 302,
    MSG_TYPE_INT24_FFT = 303,
    MSG_TYPE_FLOAT_FFT = 304,
} spyserver_msg_type_t;

/* Command types (client -> server) */
typedef enum {
    CMD_HELLO = 0,
    CMD_GET_SETTING = 1,
    CMD_SET_SETTING = 2,
    CMD_PING = 3,
} spyserver_cmd_type_t;

/* Setting types for CMD_SET_SETTING
 * Reference: https://github.com/miweber67/spyserver_client/blob/master/spyserver_protocol.h */
typedef enum {
    SETTING_STREAMING_MODE = 0,
    SETTING_STREAMING_ENABLED = 1,
    SETTING_GAIN = 2,             /* NOTE: 2, not 5! Matches reference protocol */
    SETTING_IQ_FORMAT = 100,
    SETTING_IQ_FREQUENCY = 101,
    SETTING_IQ_DECIMATION = 102,
    SETTING_IQ_DIGITAL_GAIN = 103,
    SETTING_FFT_FORMAT = 200,
    SETTING_FFT_FREQUENCY = 201,
    SETTING_FFT_DECIMATION = 202,
    SETTING_FFT_DB_OFFSET = 203,
    SETTING_FFT_DB_RANGE = 204,
    SETTING_FFT_DISPLAY_PIXELS = 205,
} spyserver_setting_type_t;

/* Stream types */
typedef enum {
    STREAM_TYPE_STATUS = 0,
    STREAM_TYPE_IQ = 1,
    STREAM_TYPE_AF = 2,
    STREAM_TYPE_FFT = 4,
} spyserver_stream_type_t;

/* Stream formats */
typedef enum {
    STREAM_FORMAT_INVALID = 0,
    STREAM_FORMAT_UINT8 = 1,
    STREAM_FORMAT_INT16 = 2,
    STREAM_FORMAT_INT24 = 3,
    STREAM_FORMAT_FLOAT = 4,
    STREAM_FORMAT_DINT4 = 5,
} spyserver_stream_format_t;

/* Wire format: message header (20 bytes, little-endian) */
typedef struct __attribute__((packed)) {
    uint32_t protocol_id;      /* Protocol magic + version */
    uint32_t msg_type;         /* spyserver_msg_type_t */
    uint32_t stream_type;      /* spyserver_stream_type_t */
    uint32_t sequence_number;
    uint32_t body_size;
} spyserver_msg_header_t;

/* Wire format: command header (8 bytes) */
typedef struct __attribute__((packed)) {
    uint32_t cmd_type;
    uint32_t body_size;
} spyserver_cmd_header_t;

/* Device info body (sent on connect) */
typedef struct __attribute__((packed)) {
    uint32_t device_type;
    uint32_t device_serial;
    uint32_t max_sample_rate;
    uint32_t max_bandwidth;
    uint32_t decimation_stage_count;
    uint32_t gain_stage_count;
    uint32_t max_gain_index;
    uint32_t min_frequency;
    uint32_t max_frequency;
    uint32_t resolution;
    uint32_t min_iq_decimation;
    uint32_t forced_iq_format;
} spyserver_device_info_t;

/* Client sync body */
typedef struct __attribute__((packed)) {
    uint32_t can_control;
    uint32_t gain;
    uint32_t device_center_frequency;
    uint32_t iq_center_frequency;
    uint32_t fft_center_frequency;
    uint32_t min_iq_center_frequency;
    uint32_t max_iq_center_frequency;
    uint32_t min_fft_center_frequency;
    uint32_t max_fft_center_frequency;
} spyserver_client_sync_t;
