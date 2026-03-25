/*
 * PIE SIMD: FIR inner MAC using esp.vmulas.s16.xacc
 *
 * int32_t pie_fir_mac_arp4(const int16_t *delay_ptr, const int16_t *taps_ptr,
 *                           int n_groups);
 *
 * Computes dot product of delay_ptr[0..n_groups*8-1] and taps_ptr[0..n_groups*8-1]
 * using the scalar xacc accumulator (true sum across all lanes).
 *
 * Returns raw 40-bit accumulator shifted right by 0 (caller handles Q15 shift).
 * Uses the same pattern as esp-dsp dspi_dotprod_s16_arp4.
 *
 * a0 = delay_ptr (const int16_t *, 16-byte aligned)
 * a1 = taps_ptr  (const int16_t *, 16-byte aligned)
 * a2 = n_groups  (n_taps_padded / 8, must be >= 1)
 *
 * Returns: a0 = low 32 bits of xacc (raw sum of int16*int16 products)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED

    .text
    .align 4
    .global pie_fir_mac_arp4
    .type   pie_fir_mac_arp4, @function

pie_fir_mac_arp4:
    /* a0 = delay_ptr, a1 = taps_ptr, a2 = n_groups */
    beqz    a2, .L_fir_mac_zero

    /* Clear xacc scalar accumulator */
    esp.zero.xacc

    /* Pre-load first pair of vectors */
    esp.vld.128.ip  q0, a0, 16         /* q0 = 8 delay samples */
    esp.vld.128.ip  q1, a1, 16         /* q1 = 8 tap coefficients */

    addi    a2, a2, -1                  /* One group consumed by pre-load */
    beqz    a2, .L_fir_mac_last

    /* Hardware zero-overhead loop: fused load + MAC */
    esp.lp.setup    0, a2, .L_fir_mac_loop_end
        esp.vmulas.s16.xacc.ld.ip  q0, a0, 16, q0, q1  /* MAC q0*q1 -> xacc, load next delay */
.L_fir_mac_loop_end:
        esp.vld.128.ip  q1, a1, 16                      /* Load next taps */

.L_fir_mac_last:
    /* Final MAC for last (or only) pair */
    esp.vmulas.s16.xacc  q0, q1

    /* Extract scalar result: shift xacc right by 0, store low 32 bits */
    li      a5, 0
    esp.srs.s.xacc  a0, a5             /* a0 = (int32_t)(xacc >> 0) */
    ret

.L_fir_mac_zero:
    li      a0, 0
    ret

    .size   pie_fir_mac_arp4, . - pie_fir_mac_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
