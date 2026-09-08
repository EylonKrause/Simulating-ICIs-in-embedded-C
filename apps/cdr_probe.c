/* ===========================================================================
 *  cdr_probe.c -- instrument the CDR loop in isolation.
 *
 *  Debug methodology, and it is the same one to describe in an interview:
 *  when a loop will not converge, FIRST separate the loop from the plant.
 *  Freeze everything else, drive the loop alone, and dump its internal state
 *  over time. Diverging, oscillating and stuck are three different pictures
 *  and they point at three different bugs. Guessing at loop constants without
 *  looking is how a day disappears.
 *
 *  Usage:  cdr_probe [IL_dB] [ppm] [blocks] [kp] [ki] [vga] [h1_target]
 *          A zero for kp, ki or h1_target keeps the built-in value.
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
    const double h1t    = (argc > 7) ? atof(argv[7]) : -1.0;

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
    /* The timing detector's h1 target is a register the datapath reads every
     * block, so setting the field is what actually takes effect -- writing the
     * struct directly would be overwritten on the next block. Default it to
     * the value firmware would choose for a mid-loss channel so the probe
     * shows a loop that locks; the CLI argument overrides it. */
    hal_write32(REG_CDR_CTRL, (h1t >= 0.0)
                ? (uint32_t)(h1t * CDR_H1_SCALE + 0.5) : 23u);
    /* CDR only. FFE and DFE frozen at the centre spike. */
    fw_adapt_reset();
    hal_write32(REG_ADAPT_CTRL, ADAPT_CDR_EN);

    printf("  CDR loop probe: IL %.0f dB, %.0f ppm, kp=%.1e ki=%.1e\n",
           il, ppm, hw.cdr.kp, hw.cdr.ki);
    printf("  ppm drift per symbol = %.3e samples (%.2f samples per block)\n\n",
           ppm * 1e-6 * OSR, ppm * 1e-6 * OSR * (double)HW_BLOCK_SYMS);
    printf("  %-4s %-9s %-11s %-11s %-10s %-8s %s\n",
           "blk", "phase", "mean(e)", "integ", "slew", "lockcnt", "ppm_est");
    printf("  --------------------------------------------------------------\n");

    for (unsigned b = 0; b < nblk; ++b) {
        hw_lane_run(&hw, HW_MODE_TRAIN);   /* known symbols */
        printf("  %-4u %-9.4f %-+11.5f %-+11.3e %-+10.2e %-8u %+.1f%s\n",
               b, hw.cdr.phase, hw.cdr.ted_avg, hw.cdr.integ, hw.cdr.slew_slow,
               hw.cdr.lock_count, cdr_ppm(&hw.cdr),
               hw.cdr.locked ? "   LOCKED" : "");
    }

    /* SAY WHICH CONDITION FAILED. A bare "NOT LOCKED" sends you looking at the
     * whole loop; naming the condition sends you to one line of it. Lock here
     * is the conjunction of four independent tests, and each one exists
     * because a previous version of this detector passed without it -- see
     * the notes in cdr.c. */
    printf("\n  lock conditions:\n");
    printf("    |mean(e)|  %-10.2e  < %-9.2e  %s\n",
           fabs(hw.cdr.ted_avg), 0.02,
           (fabs(hw.cdr.ted_avg) < 0.02) ? "ok" : "FAILS");
    printf("    |slew|     %-10.2e  < %-9.2e  %s   (samples/symbol)\n",
           fabs(hw.cdr.slew_slow), 6.0e-4,
           (fabs(hw.cdr.slew_slow) < 6.0e-4) ? "ok" : "FAILS");
    printf("    since_wrap %-10u  > %-9u  %s\n",
           hw.cdr.since_wrap, 8000u,
           (hw.cdr.since_wrap > 8000u) ? "ok" : "FAILS");
    printf("    |integ|    %-10.2e  < %-9.2e  %s   (anti-windup rail)\n",
           fabs(hw.cdr.integ), 0.95 * 1000.0 * 1e-6 * (double)OSR,
           (fabs(hw.cdr.integ) < 0.95 * 1000.0 * 1e-6 * (double)OSR) ? "ok" : "FAILS");

    printf("\n  Note: this probe runs the CDR with the equaliser FROZEN at its\n");
    printf("  initial spike and no AGC, which is deliberately harsher than the\n");
    printf("  link. A constant offset between the estimate and the true ppm is\n");
    printf("  expected here: it is residual Mueller-Muller DC that an adapting\n");
    printf("  equaliser removes. What matters is that the offset does not depend\n");
    printf("  on the true ppm -- the loop tracks with unity slope.\n");

    printf("\n  final: phase %.4f  ppm est %+.1f (actual %+.1f)  %s\n",
           hw.cdr.phase, cdr_ppm(&hw.cdr), ppm,
           hw.cdr.locked ? "LOCKED" : "NOT LOCKED");

    hw_lane_free(&hw);
    return 0;
}
