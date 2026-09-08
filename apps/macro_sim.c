#define _CRT_SECURE_NO_WARNINGS
/* ===========================================================================
 *  macro_sim.c -- bring up a whole 8-lane macro, coupled, on one processor.
 *
 *  link_sim.c runs one lane in isolation. This runs the thing the chip
 *  actually contains: eight lanes side by side, each aggressing on its
 *  neighbours, all serviced by a single control processor round-robin through
 *  a windowed register aperture.
 *
 *  What only shows up here:
 *    - crosstalk, which needs neighbours to exist;
 *    - per-lane state, which a shared-static bug turns into one lane's answer
 *      applied to eight different channels;
 *    - control-loop bandwidth divided by the service rate, because eight lanes
 *      share one CPU;
 *    - lanes that come up at different times, and a macro that is only as
 *      good as its worst lane.
 *
 *  Usage:
 *      macro_sim [lanes] [IL_dB]      [xtalk] [max_ticks] [lanes_per_block]
 *      macro_sim [lanes] <file.s4p>   [xtalk] [max_ticks] [lanes_per_block]
 *
 *  lanes_per_block starves the supervisor on purpose: 0 (the default) services
 *  every lane every block, 2 out of 8 quarters each lane's loop bandwidth.
 *
 *  Crosstalk requires S-parameters: a fitted loss curve is a single number per
 *  frequency and has no off-diagonal term to couple with.
 * =========================================================================*/
#include "hw_macro.h"
#include "hal.h"
#include "mgmt.h"
#include "pcs.h"
#include "touchstone.h"
#include "fixed.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TICKS      6000u
#define TRAFFIC_TICKS   200u

static int is_touchstone(const char *s)
{
    const char *dot = strrchr(s, '.');
    return dot != NULL && (dot[1] == 's' || dot[1] == 'S') &&
           (dot[2] >= '1' && dot[2] <= '9');
}

int main(int argc, char **argv)
{
    const unsigned lanes = (argc > 1) ? (unsigned)atoi(argv[1]) : MACRO_LANES;
    const char *chan = (argc > 2) ? argv[2] : NULL;
    const double xtalk = (argc > 3) ? atof(argv[3]) : 1.0;
    const unsigned max_ticks = (argc > 4) ? (unsigned)atoi(argv[4]) : MAX_TICKS;
    /* Lanes serviced per block. 0 means all of them, which is the default and
     * what a real supervisor at kHz rates can afford. Setting it lower is the
     * interesting case: it divides every control loop's bandwidth and the
     * firmware's timeouts have to scale with it. */
    const unsigned service = (argc > 5) ? (unsigned)atoi(argv[5]) : 0u;

    hw_macro_t *M = (hw_macro_t *)malloc(sizeof(hw_macro_t));
    if (M == NULL) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    printf("=========================================================\n");
    printf(" macro: %u lanes x %.0f Gb/s PAM4 = %.1f Tb/s\n",
           lanes, LANE_RATE_GBPS, (double)lanes * LANE_RATE_GBPS / 1000.0);
    printf(" %u macros per chip = %u lanes = %.1f Tb/s\n",
           MACROS_PER_CHIP, LANES_PER_CHIP, CHIP_TBPS);

    int rc;
    if (chan != NULL && is_touchstone(chan)) {
        printf(" channel: %s   crosstalk x%.2f\n", chan, xtalk);
        rc = hw_macro_init_sparam(M, lanes, chan, AFE_ELECTRICAL, -40.0, 160.0);
        if (rc != 0) {
            fprintf(stderr, "cannot load %s: %s\n", chan, ts_error());
            free(M);
            return 1;
        }
    } else {
        const double il = (chan != NULL) ? atof(chan) : DEFAULT_IL_DB_AT_NYQUIST;
        printf(" channel: fitted, %.0f dB +/- 2 dB across lanes\n", il);
        printf(" NO CROSSTALK: a fitted loss curve has no off-diagonal term.\n");
        printf(" Pass a .s4p to couple the lanes.\n");
        rc = hw_macro_init(M, lanes, il, 4.0, AFE_ELECTRICAL, -40.0, 160.0);
        if (rc != 0) {
            fprintf(stderr, "hw_macro_init failed\n");
            free(M);
            return 1;
        }
    }
    M->xtalk = xtalk;
    hw_macro_set_service(M, service);

    printf(" supervisor: one CPU, %u lane%s serviced per tick\n",
           M->service, (M->service == 1u) ? "" : "s");
    printf("   -> each lane is serviced every %u ticks, so every control loop\n",
           (M->n_lanes + M->service - 1u) / M->service);
    printf("      on a lane runs at 1/%u of the block rate\n",
           (M->n_lanes + M->service - 1u) / M->service);
    printf("=========================================================\n\n");

    printf("  per-lane setup\n");
    printf("  %-5s %-9s %-9s %-12s %s\n",
           "lane", "IL@Nyq", "ppm", "xtalk", "note");
    printf("  ------------------------------------------------------------\n");
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        const double xr = hw_macro_xtalk_ratio_db(M, i);
        char xs[32];
        if (xr < -900.0) {
            (void)snprintf(xs, sizeof(xs), "%s", "none");
        } else {
            (void)snprintf(xs, sizeof(xs), "%+.1f dB", xr);
        }
        printf("  %-5u %-9.2f %+-9.0f %-12s %s\n",
               i, M->lane[i].ch.il_at_nyquist_db, M->lane[i].ppm_offset, xs,
               (i == 0u || i + 1u == M->n_lanes) ? "edge lane, one neighbour"
                                                 : "interior, two neighbours");
    }

    /* ---- bring the macro up -------------------------------------------- */
    printf("\n  bring-up\n");
    unsigned up_at[MACRO_LANES];
    for (unsigned i = 0; i < MACRO_LANES; ++i) {
        up_at[i] = 0u;
    }

    unsigned t = 0u;
    for (; t < max_ticks; ++t) {
        const unsigned before = hw_macro_lanes_up(M);
        hw_macro_block(M);
        const unsigned after = hw_macro_lanes_up(M);
        if (after != before) {
            for (unsigned i = 0; i < M->n_lanes; ++i) {
                if (up_at[i] == 0u && fw_is_up(&M->fw[i])) {
                    up_at[i] = t;
                    printf("    t=%4u  lane %u UP\n", t, i);
                }
            }
        }
        if (after == M->n_lanes) {
            break;
        }
    }

    const unsigned n_up = hw_macro_lanes_up(M);
    printf("\n  %u of %u lanes up after %u ms\n", n_up, M->n_lanes, t);
    if (n_up < M->n_lanes) {
        printf("  lanes still down:");
        for (unsigned i = 0; i < M->n_lanes; ++i) {
            if (!fw_is_up(&M->fw[i])) {
                printf(" %u(%s)", i, fw_state_name(M->fw[i].state));
            }
        }
        printf("\n");
    }
    printf("\n  A MACRO IS ONLY AS GOOD AS ITS WORST LANE. The link is a bonded\n");
    printf("  group: one lane down takes the whole port down, so the number that\n");
    printf("  matters is the slowest and the weakest, never the average.\n");

    /* ---- per-lane converged state -------------------------------------- */
    printf("\n  converged per-lane state (proves the contexts are not shared)\n");
    printf("  %-5s %-6s %-5s %-5s %-6s %-22s %s\n",
           "lane", "state", "VGA", "TIA", "CTLE", "FFE taps w2..w5", "DFE b0,b1");
    printf("  ---------------------------------------------------------------------------\n");
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        hal_select_lane(i);
        printf("  %-5u %-6s %-5u %-5u %-6u %+4d %+4d %+4d %+4d      %+4d %+4d\n",
               i, fw_state_name(M->fw[i].state),
               hal_field_get(REG_AFE_VGA,  VGA_GAIN_MASK,  VGA_GAIN_SHIFT),
               hal_field_get(REG_AFE_TIA,  TIA_GAIN_MASK,  TIA_GAIN_SHIFT),
               hal_field_get(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT),
               hal_read_signed(REG_FFE_TAP(2), TAP_APPLY_BITS),
               hal_read_signed(REG_FFE_TAP(3), TAP_APPLY_BITS),
               hal_read_signed(REG_FFE_TAP(4), TAP_APPLY_BITS),
               hal_read_signed(REG_FFE_TAP(5), TAP_APPLY_BITS),
               hal_read_signed(REG_DFE_TAP(0), TAP_APPLY_BITS),
               hal_read_signed(REG_DFE_TAP(1), TAP_APPLY_BITS));
    }
    printf("\n  Identical rows here would mean the per-lane contexts are NOT\n");
    printf("  independent -- the single most likely bug when one processor is\n");
    printf("  made to serve many lanes. The lanes have different losses and\n");
    printf("  different reference offsets, so their answers must differ.\n");

    /* ---- carry FEC traffic --------------------------------------------- */
    printf("\n  traffic: RS(%u,%u) on every lane, %u blocks\n",
           RS_N, RS_K, TRAFFIC_TICKS);
    hw_macro_set_fec(M, 1u);
    for (unsigned k = 0; k < TRAFFIC_TICKS; ++k) {
        hw_macro_block(M);
    }

    printf("\n  %-5s %-12s %-13s %-9s %s\n",
           "lane", "pre-FEC BER", "post-FEC BER", "uncorr", "codewords");
    printf("  ---------------------------------------------------------------------\n");
    double worst = 0.0;
    unsigned worst_lane = 0u;
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        const pcs_rx_t *p = &M->lane[i].pcs_rx;
        const double pre = pcs_pre_fec_ber(p);
        if (pre > worst) {
            worst = pre;
            worst_lane = i;
        }
        printf("  %-5u %-12.3e ", i, pre);
        if (p->codewords > 0u && p->post_bit_errors == 0u) {
            printf("%-13s ", "clean");
        } else if (p->codewords > 0u) {
            printf("%-13.3e ", pcs_post_fec_ber(p));
        } else {
            printf("%-13s ", "-");
        }
        printf("%-9llu %llu\n",
               (unsigned long long)p->uncorrectable,
               (unsigned long long)p->codewords);
    }
    printf("\n  worst lane %u at %.3e pre-FEC -- that is the macro's number\n",
           worst_lane, worst);

    /* ---- bus hygiene across every lane --------------------------------- */
    const hal_stats_t *hs = hal_stats();
    printf("\n  HAL audit across all %u apertures\n", M->n_lanes);
    printf("    reads / writes      %llu / %llu\n",
           (unsigned long long)hs->reads, (unsigned long long)hs->writes);
    printf("    UNGUARDED RMW       %llu   %s\n", (unsigned long long)hs->unguarded_rmw,
           hs->unguarded_rmw ? "<-- BUG" : "(ok)");
    printf("    W1C RMW bugs        %llu   %s\n", (unsigned long long)hs->w1c_rmw_bugs,
           hs->w1c_rmw_bugs ? "<-- BUG" : "(ok)");
    printf("    unmapped accesses   %llu   %s\n", (unsigned long long)hs->unmapped,
           hs->unmapped ? "<-- BUG: an address silicon would fault on" : "(ok)");
    printf("    lane window depth   %u   %s\n", hal_current_lane(),
           "(last selected)");
    printf("    supervisor services ");
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        printf("%llu ", (unsigned long long)M->services[i]);
    }
    printf("\n    -- these must be within one of each other, or the round robin\n");
    printf("       is starving a lane.\n");

    printf("\n  management bus (one per macro, not one per lane)\n");
    printf("    bytes dropped       %u   %s\n", mgmt_bus_dropped(),
           mgmt_bus_dropped() ? "<-- BUG: pushed into a full FIFO" : "(ok)");

    /* ---- the exit code ---------------------------------------------------
     *
     * Same reasoning as link_sim: CI runs this, so the code has to assert the
     * multi-lane claim rather than a status bit. Two things are specific to a
     * macro.
     *
     * The round robin has to be FAIR. A supervisor that quietly stopped
     * servicing one lane would still report the other seven up, and the lane
     * it starved would fail slowly and for a reason that looks like the
     * channel. Services must be within one of each other -- one, not a
     * percentage, because the scheduler is exact and any drift is a bug.
     *
     * The macro's number is its WORST lane. Averaging eight lanes is how a
     * part passes on the bench and fails in the rack. */
    unsigned lo = 0xFFFFFFFFu, hi = 0u, worst_uncorr = 0u;
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        const unsigned long long sv = (unsigned long long)M->services[i];
        if (sv < lo) { lo = (unsigned)sv; }
        if (sv > hi) { hi = (unsigned)sv; }
        if (M->lane[i].pcs_rx.uncorrectable != 0u) { ++worst_uncorr; }
    }
    const int all_up = (n_up == lanes);
    const int fair   = ((hi - lo) <= 1u);
    const int payload = (worst_uncorr == 0u);
    const int clean  = (hs->unguarded_rmw == 0u) && (hs->w1c_rmw_bugs == 0u) &&
                       (hs->unmapped == 0u) && (mgmt_bus_dropped() == 0u);

    printf("\n  PASS CRITERIA\n");
    printf("    all %u lanes up      %s\n", lanes, all_up  ? "yes" : "NO");
    printf("    no uncorrectable    %s\n", payload ? "yes" : "NO");
    printf("    round robin fair    %s   (spread %u)\n", fair ? "yes" : "NO", hi - lo);
    printf("    bus hygiene clean   %s\n", clean   ? "yes" : "NO");
    printf("  ==> %s\n",
           (all_up && payload && fair && clean) ? "PASS" : "FAIL");

    hw_macro_free(M);
    free(M);
    return (all_up && payload && fair && clean) ? 0 : 1;
}
