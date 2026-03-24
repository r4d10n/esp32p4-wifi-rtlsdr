#pragma once
#include <stdint.h>
#ifdef ESP_PLATFORM
#include "esp_err.h"
#endif

typedef struct ddc_state ddc_state_t;

/* Create DDC with given parameters */
ddc_state_t *ddc_create(int input_sample_rate, double offset_hz,
                         int bandwidth_hz, int output_sample_rate);

/* Process IQ samples through DDC. Output is int16_t audio (FM demod).
 * Returns number of output audio samples written. */
int ddc_process(ddc_state_t *ddc, const uint8_t *iq_in, int num_iq_samples,
                int16_t *audio_out, int max_audio_samples);

/* Get current output sample rate */
int ddc_get_output_rate(ddc_state_t *ddc);

/* Destroy DDC */
void ddc_destroy(ddc_state_t *ddc);
