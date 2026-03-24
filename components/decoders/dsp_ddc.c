#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dsp_ddc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FIR_TAPS 31

struct ddc_state {
    /* NCO */
    double nco_phase;
    double nco_phase_inc;  /* 2*PI*offset/sample_rate */

    /* FIR LPF */
    double fir_coeff[FIR_TAPS];
    double fir_buf_i[FIR_TAPS];
    double fir_buf_q[FIR_TAPS];
    int fir_idx;

    /* Decimation */
    int decimation;
    int decim_counter;

    /* FM demod state */
    double prev_i;
    double prev_q;

    /* Config */
    int input_rate;
    int output_rate;
};

ddc_state_t *ddc_create(int input_sample_rate, double offset_hz,
                         int bandwidth_hz, int output_sample_rate) {
    ddc_state_t *d = calloc(1, sizeof(ddc_state_t));
    if (!d) return NULL;

    d->input_rate = input_sample_rate;
    d->output_rate = output_sample_rate;
    d->nco_phase = 0.0;
    d->nco_phase_inc = 2.0 * M_PI * offset_hz / input_sample_rate;
    d->decimation = input_sample_rate / output_sample_rate;
    if (d->decimation < 1) d->decimation = 1;
    d->decim_counter = 0;
    d->fir_idx = 0;
    d->prev_i = 0;
    d->prev_q = 0;

    /* Design windowed-sinc FIR LPF */
    double cutoff = (double)bandwidth_hz / input_sample_rate;
    int M = FIR_TAPS - 1;
    double sum = 0;
    for (int i = 0; i < FIR_TAPS; i++) {
        int n = i - M / 2;
        if (n == 0) {
            d->fir_coeff[i] = 2.0 * cutoff;
        } else {
            d->fir_coeff[i] = sin(2.0 * M_PI * cutoff * n) / (M_PI * n);
        }
        /* Hamming window */
        d->fir_coeff[i] *= 0.54 - 0.46 * cos(2.0 * M_PI * i / M);
        sum += d->fir_coeff[i];
    }
    /* Normalize */
    for (int i = 0; i < FIR_TAPS; i++) {
        d->fir_coeff[i] /= sum;
    }

    return d;
}

int ddc_process(ddc_state_t *ddc, const uint8_t *iq_in, int num_iq_samples,
                int16_t *audio_out, int max_audio_samples) {
    int out_idx = 0;

    for (int i = 0; i < num_iq_samples && out_idx < max_audio_samples; i++) {
        /* Convert uint8 IQ to float centered at 0 */
        double in_i = (double)iq_in[i * 2] - 128.0;
        double in_q = (double)iq_in[i * 2 + 1] - 128.0;

        /* NCO mixing: multiply by e^(-j*phase) */
        double cos_p = cos(ddc->nco_phase);
        double sin_p = sin(ddc->nco_phase);
        double mixed_i = in_i * cos_p + in_q * sin_p;
        double mixed_q = -in_i * sin_p + in_q * cos_p;
        ddc->nco_phase += ddc->nco_phase_inc;
        if (ddc->nco_phase > 2.0 * M_PI) ddc->nco_phase -= 2.0 * M_PI;
        if (ddc->nco_phase < -2.0 * M_PI) ddc->nco_phase += 2.0 * M_PI;

        /* FIR filter (circular buffer) */
        ddc->fir_buf_i[ddc->fir_idx] = mixed_i;
        ddc->fir_buf_q[ddc->fir_idx] = mixed_q;

        /* Decimation check */
        ddc->decim_counter++;
        if (ddc->decim_counter < ddc->decimation) {
            ddc->fir_idx = (ddc->fir_idx + 1) % FIR_TAPS;
            continue;
        }
        ddc->decim_counter = 0;

        /* Apply FIR filter */
        double filt_i = 0, filt_q = 0;
        for (int j = 0; j < FIR_TAPS; j++) {
            int idx = (ddc->fir_idx - j + FIR_TAPS) % FIR_TAPS;
            filt_i += ddc->fir_coeff[j] * ddc->fir_buf_i[idx];
            filt_q += ddc->fir_coeff[j] * ddc->fir_buf_q[idx];
        }

        ddc->fir_idx = (ddc->fir_idx + 1) % FIR_TAPS;

        /* FM demodulator: cross-product discriminator */
        double demod = (filt_i * ddc->prev_q - filt_q * ddc->prev_i);
        /* Normalize by magnitude squared to get instantaneous frequency */
        double mag_sq = filt_i * filt_i + filt_q * filt_q;
        if (mag_sq > 0.001) demod /= mag_sq;

        ddc->prev_i = filt_i;
        ddc->prev_q = filt_q;

        /* Scale to int16 range */
        double scaled = demod * 8000.0;
        if (scaled > 32767) scaled = 32767;
        if (scaled < -32767) scaled = -32767;
        audio_out[out_idx++] = (int16_t)scaled;
    }

    return out_idx;
}

int ddc_get_output_rate(ddc_state_t *ddc) {
    return ddc ? ddc->output_rate : 0;
}

void ddc_destroy(ddc_state_t *ddc) {
    free(ddc);
}
