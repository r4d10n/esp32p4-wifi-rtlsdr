/*
 * DSP Engine for ESP32-P4 WebSDR
 *
 * Provides FFT computation and Digital Down Conversion (DDC) for
 * server-side spectrum analysis and narrowband IQ extraction.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────── FFT ──────────────────────── */

/**
 * Initialize FFT engine with given size (must be power of 2).
 * Supported sizes: 256, 512, 1024, 2048, 4096, 8192, 16384.
 * Dynamically allocates aligned buffers.  Call dsp_fft_reset() implicitly.
 */
void dsp_fft_init(int fft_size);

/**
 * Reset FFT accumulator (call when switching parameters).
 */
void dsp_fft_reset(void);

/**
 * Feed IQ data and compute FFT when enough samples are accumulated.
 *
 * @param iq_data   Raw uint8 interleaved IQ data (I0,Q0,I1,Q1,...)
 * @param len       Number of bytes in iq_data (must be even)
 * @param fft_out   Output buffer for power spectrum (fft_size bytes, dB as 0-255)
 * @param fft_out_len  Set to fft_size when a new FFT frame is ready, 0 otherwise
 */
void dsp_fft_compute(const uint8_t *iq_data, uint32_t len,
                     uint8_t *fft_out, int *fft_out_len);

/**
 * Set the dB range used for 0-255 scaling in FFT output.
 */
void dsp_fft_set_range(float db_min, float db_max);

/**
 * Return the current FFT size.
 */
int dsp_fft_get_size(void);

/**
 * Get the last computed FFT spectrum (for scope display).
 * Thread-safe: copies to provided buffer.
 * @param fft_out   Output buffer for FFT dB data (uint8, 0-255)
 * @param max_len   Buffer capacity
 * @return Number of FFT bins copied (0 if no data available)
 */
int dsp_fft_get_spectrum(uint8_t *fft_out, int max_len);

/* ──────────────────────── DDC ──────────────────────── */

typedef struct dsp_ddc dsp_ddc_t;

/**
 * Create a DDC (Digital Down Converter) instance.
 *
 * @param sample_rate       Input sample rate in Hz
 * @param center_offset_hz  Frequency offset from center (signed via uint32 cast)
 * @param output_bw_hz      Desired output bandwidth (determines decimation ratio)
 * @return DDC handle, or NULL on failure
 */
dsp_ddc_t *dsp_ddc_create(uint32_t sample_rate, uint32_t center_offset_hz,
                           uint32_t output_bw_hz);

/**
 * Free a DDC instance.
 */
void dsp_ddc_free(dsp_ddc_t *ddc);

/**
 * Get the DDC output sample rate (= input_rate / decimation_ratio).
 */
uint32_t dsp_ddc_get_output_rate(dsp_ddc_t *ddc);

/**
 * Process IQ data through the DDC.
 *
 * @param ddc       DDC handle
 * @param iq_in     Input uint8 IQ data (full bandwidth)
 * @param in_len    Input length in bytes
 * @param iq_out    Output uint8 IQ data (decimated narrowband)
 * @param out_len   On entry: capacity of iq_out. On exit: bytes written.
 * @return 0 on success, -1 on error
 */
int dsp_ddc_process(dsp_ddc_t *ddc, const uint8_t *iq_in, uint32_t in_len,
                    uint8_t *iq_out, uint32_t *out_len);

#ifdef __cplusplus
}
#endif
