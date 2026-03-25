/*
 * PIE SIMD: Complex multiply int16 IQ with NCO table.
 *
 * void pie_nco_mix_s16_arp4(const int16_t *iq_in, const int16_t *nco_table,
 *                            int16_t *iq_out, int iq_pairs);
 *
 * Uses esp.cmul.s16 for native complex multiply.
 * Input/output: interleaved [re,im,re,im,...] int16 Q15.
 * NCO table: interleaved [cos,-sin,cos,-sin,...] int16 Q15.
 *
 * Processes 4 complex pairs per iteration.
 *
 * a0 = iq_in, a1 = nco_table, a2 = iq_out, a3 = iq_pairs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED

    .text
    .align 4
    .global pie_nco_mix_s16_arp4
    .type   pie_nco_mix_s16_arp4, @function

pie_nco_mix_s16_arp4:
    /* a0=iq_in, a1=nco_table, a2=iq_out, a3=iq_pairs */
    srli    a3, a3, 2               /* loop count = iq_pairs / 4 */
    beqz    a3, .L_mix_done

    esp.lp.setup  0, a3, .L_mix_loop_end
        esp.vld.128.ip  q0, a0, 16         /* load 4 IQ pairs from input */
        esp.vld.128.ip  q1, a1, 16         /* load 4 NCO [cos,-sin] pairs */

        /* Complex multiply: mode 0 = standard complex mul
         * out_re = in_re*nco_re - in_im*nco_im
         * out_im = in_re*nco_im + in_im*nco_re
         * Result is Q15 (automatically scaled by the instruction) */
        esp.cmul.s16    q2, q0, q1, 0

.L_mix_loop_end:
        esp.vst.128.ip  q2, a2, 16         /* store 4 mixed IQ pairs */

.L_mix_done:
    ret

    .size   pie_nco_mix_s16_arp4, . - pie_nco_mix_s16_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
