/* ===========================================================================
 *  cdr_probe.c — instrument the CDR loop in isolation.
 *
 *  Debug methodology, and it is the same one to describe in an interview:
 *  when a loop will not converge, FIRST separate the loop from the plant.
 *  Freeze everything else, drive the loop alone, and dump its internal state
 *  over time. Diverging, oscillating and stuck are three different pictures
 *  and they point at three different bugs. Guessing at loop constants without
 *  looking is how a day disappears.
 *
 *  Usage:  cdr_probe [IL_dB] [ppm] [blocks]
 * =========================================================================*/
#include "hw_lane.h"
#include "hal.h"
#include "fw.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    const double il     = (argc > 1) ? atof(argv[1]) : DEFAULT_IL_DB_AT_NYQUIST;
    const double ppm    = (argc > 2) ? atof(argv[2]) : 120.0;
    const unsigned nblk = (argc > 3) ? (unsigned)atoi(argv[3]) : 20u;
    const double kp_o   = (argc > 4) ? atof(argv[4]) : 0.0;
    const double ki_o   = (argc > 5) ? atof(argv[5]) : 0.0;
    const unsigned vga  = (argc > 6) ? (unsigned)atoi(argv[6]) : 36u;

    hal_reset_all();

    hw_lane_t hw;
    if (hw_lane_init(&hw, il, AFE_ELECTRICAL, ppm) != 0) {
        return 1;
    }

    if (kp_o > 0.0) { hw.cdr.kp = kp_o; }
    if (ki_o > 0.0) { hw.cdr.ki = ki_o; }

    /* Fixed AFE, no AGC: the gain must not move while we study the CDR. */
    hal_write32(REG_AFE_VGA,  vga);
    hal_write32(REG_AFE_TIA,   8u);
    hal_write32(REG_AFE_CTLE, 12u);
    /* CDR only. FFE and DFE frozen at the centre spike. */
    fw_adapt_reset();
    hal_write32(REG_ADAPT_CTRL, ADAPT_CDR_EN);

    printf("  CDR loop probe: IL %.0f dB, %.0f ppm, kp=%.1e ki=%.1e\n",
           il, ppm, hw.cdr.kp, hw.cdr.ki);
    printf("  ppm drift per symbol = %.3e samples (%.2f samples per block)\n\n",
           ppm * 1e-6 * OSR, ppm * 1e-6 * OSR * (double)HW_BLOCK_SYMS);
    printf("  %-4s %-9s %-11s %-11s %-8s %s\n",
           "blk", "phase", "mean(e)", "integ", "lockcnt", "ppm_est");
    printf("  --------------------------------------------------------------\n");

    for (unsigned b = 0; b < nblk; ++b) {
        hw_lane_run(&hw, 1u);            /* training mode: known symbols */
        printf("  %-4u %-9.4f %-+11.5f %-+11.3e %-8u %+.1f%s\n",
               b, hw.cdr.phase, hw.cdr.ted_avg, hw.cdr.integ,
               hw.cdr.lock_count, cdr_ppm(&hw.cdr),
               hw.cdr.locked ? "   LOCKED" : "");
    }

    printf("\n  final: phase %.4f  ppm est %+.1f (actual %+.1f)  %s\n",
           hw.cdr.phase, cdr_ppm(&hw.cdr), ppm,
           hw.cdr.locked ? "LOCKED" : "NOT LOCKED");

    hw_lane_free(&hw);
    return 0;
}
