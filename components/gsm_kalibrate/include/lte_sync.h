/*
 * LTE Cell Synchronization -- FFT-based PSS/SSS Detection
 * 3GPP TS 36.211 compliant PSS + SSS for cell search and calibration.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LTE OFDM parameters at 1.92 MSPS (1.4 MHz BW, 128-pt FFT) */
#define LTE_SAMPLE_RATE_HZ  1920000
#define LTE_N_FFT           128
#define LTE_CP0             10      /* CP length: first symbol of slot */
#define LTE_CPN             9       /* CP length: symbols 1-6 */
#define LTE_N_SC_PSS        62      /* PSS/SSS subcarriers (ZC len 63, DC removed) */
#define LTE_N_SYMB_SLOT     7       /* OFDM symbols per slot (normal CP) */
#define LTE_N_SLOT_FRAME    20      /* Slots per 10ms frame */
#define LTE_FRAME_SAMPLES   19200   /* 1.92 MSPS x 10ms */
#define LTE_SLOT_SAMPLES    960     /* 1.92 MSPS x 0.5ms */
#define LTE_PSS_ROOTS       3

/* Symbol FFT start offsets within a slot (after CP) */
#define LTE_SYM6_OFFSET     832     /* PSS: last symbol of slot */
#define LTE_SYM5_OFFSET     695     /* SSS: second-to-last symbol */

/* PSS Zadoff-Chu roots */
#define LTE_ZC_ROOT_0       25      /* N_ID_2 = 0 */
#define LTE_ZC_ROOT_1       29      /* N_ID_2 = 1 */
#define LTE_ZC_ROOT_2       34      /* N_ID_2 = 2 */

/* Detected cell info */
typedef struct {
    uint16_t pci;           /* Physical Cell ID = 3*N_ID_1 + N_ID_2 */
    uint8_t  n_id_1;        /* 0-167, from SSS */
    uint8_t  n_id_2;        /* 0-2, from PSS root */
    float    pss_power;     /* PSS correlation magnitude */
    float    freq_error_hz; /* Frequency offset in Hz */
    float    ppm;           /* PPM at the carrier frequency */
    int      pss_offset;    /* Sample offset of detected PSS */
    bool     sss_valid;     /* True if SSS was successfully decoded */
    uint8_t  subframe;      /* 0 or 5 (which SSS pattern matched) */
} lte_cell_t;

/* Sync engine context (opaque allocation) */
typedef struct lte_sync lte_sync_t;

/**
 * Initialize the LTE sync engine.
 * Generates PSS/SSS templates, initializes ESP-DSP FFT.
 * Returns NULL on failure.
 */
lte_sync_t *lte_sync_init(void);

/**
 * Deinitialize and free the sync engine.
 */
void lte_sync_deinit(lte_sync_t *ctx);

/**
 * Search for LTE cells in an IQ buffer.
 *
 * @param ctx        Sync engine context
 * @param iq_data    Raw uint8 IQ data at 1.92 MSPS (interleaved I,Q)
 * @param iq_len     Length in bytes (must be >= 2 * LTE_FRAME_SAMPLES for 10ms)
 * @param carrier_hz Center frequency in Hz (for PPM calculation)
 * @param cells      Output array for detected cells
 * @param max_cells  Capacity of cells array
 * @return Number of cells detected (0 if none found)
 */
int lte_sync_detect(lte_sync_t *ctx,
                    const uint8_t *iq_data, uint32_t iq_len,
                    uint32_t carrier_hz,
                    lte_cell_t *cells, int max_cells);

#ifdef __cplusplus
}
#endif
