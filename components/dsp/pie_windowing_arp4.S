/*
 * PIE SIMD: Convert uint8 IQ to windowed int16 with bias removal.
 *
 * void pie_u8iq_to_s16_windowed_arp4(const uint8_t *src,
 *                                     const int16_t *window,
 *                                     int16_t *dst, int iq_pairs);
 *
 * For each IQ pair k:
 *   si = ((int16)src[k*2]   - 128) << 8
 *   sq = ((int16)src[k*2+1] - 128) << 8
 *   dst[k*2]   = (si * window[k]) >> 15
 *   dst[k*2+1] = (sq * window[k]) >> 15
 *
 * Uses hardware loop for zero branch overhead. Processes one IQ pair
 * per iteration with scalar mul (the uint8->int16 widening + interleaved
 * window application is hard to vectorize with PIE due to the
 * non-uniform data widths and interleaved access pattern).
 *
 * The main benefit is the zero-overhead loop and tight register usage.
 *
 * a0 = src (const uint8_t *)
 * a1 = window (const int16_t *)
 * a2 = dst (int16_t *)
 * a3 = iq_pairs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED

    .text
    .align 4
    .global pie_u8iq_to_s16_windowed_arp4
    .type   pie_u8iq_to_s16_windowed_arp4, @function

pie_u8iq_to_s16_windowed_arp4:
    /* a0=src, a1=window, a2=dst, a3=iq_pairs */
    beqz    a3, .L_win_done

    li      t4, 128                    /* bias constant */

    esp.lp.setup  0, a3, .L_win_loop_end
        /* Load I and Q as uint8 (zero-extended to int32 by lbu) */
        lbu     t0, 0(a0)              /* t0 = I (uint8) */
        lbu     t1, 1(a0)              /* t1 = Q (uint8) */
        addi    a0, a0, 2              /* advance src by 2 bytes */

        /* Bias subtract and scale: ((val - 128) << 8) */
        sub     t0, t0, t4             /* t0 = I - 128 (signed) */
        sub     t1, t1, t4             /* t1 = Q - 128 (signed) */
        slli    t0, t0, 8              /* t0 = (I - 128) << 8 */
        slli    t1, t1, 8              /* t1 = (Q - 128) << 8 */

        /* Load window coefficient (Q15 int16) */
        lh      t2, 0(a1)              /* t2 = window[k] */
        addi    a1, a1, 2              /* advance window */

        /* Q15 multiply: (sample * window) >> 15 */
        mul     t3, t0, t2             /* t3 = I_scaled * window (int32) */
        srai    t3, t3, 15             /* t3 >>= 15 (arithmetic right shift) */
        sh      t3, 0(a2)              /* store windowed I as int16 */

        mul     t0, t1, t2             /* reuse t0: Q_scaled * window (int32) */
        srai    t0, t0, 15             /* t0 >>= 15 */
        sh      t0, 2(a2)              /* store windowed Q as int16 */
.L_win_loop_end:
        addi    a2, a2, 4              /* advance dst — MUST be inside loop body */

.L_win_done:
    ret

    .size   pie_u8iq_to_s16_windowed_arp4, . - pie_u8iq_to_s16_windowed_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
