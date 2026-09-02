#define _CRT_SECURE_NO_WARNINGS
/* ===========================================================================
 *  link_sim.c -- full link bring-up, end to end.
 *
 *  Runs the real firmware state machine against the hardware model. The
 *  firmware in fw_*.c is the same source that would run on the control
 *  processor inside the macro; only the HAL's backing store differs.
 *
 *  Usage:  link_sim [IL_dB] [ppm] [optical|electrical]
 * =========================================================================*/
#include "fw.h"
#include "hal.h"
#include "hw_lane.h"
#include "eye.h"
#include "fixed.h"
#include "mgmt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TICK_MS      1u
#define MAX_TICKS    3000u

int main(int argc, char **argv)
{
    const double il  = (argc > 1) ? atof(argv[1]) : DEFAULT_IL_DB_AT_NYQUIST;
    const double ppm = (argc > 2) ? atof(argv[2]) : 120.0;
    const afe_mode_t mode = (argc > 3 && strcmp(argv[3], "optical") == 0)
                          ? AFE_OPTICAL : AFE_ELECTRICAL;

    printf("=========================================================\n");
    printf(" %.0f Gb/s/lane PAM4 | %.0f GBd | Nyquist %.0f GHz | %u lanes = %.1f Tb/s\n",
           LANE_RATE_GBPS, BAUD_RATE_GBD, NYQUIST_GHZ, LANES_PER_CHIP, CHIP_TBPS);
    printf(" channel %.0f dB @ Nyquist | %.0f ppm ref offset | %s front end\n",
           il, ppm, (mode == AFE_OPTICAL) ? "optical (PD+TIA)" : "electrical");
    printf("=========================================================\n\n");

    hw_lane_t hw;
    if (hw_lane_init(&hw, il, mode, ppm) != 0) {
        fprintf(stderr, "hw_lane_init failed\n");
        return 1;
    }

    /* ~128 bytes per 1 ms block == ~1 Mb/s management bus, five orders of
     * magnitude below the 100 GBd data path. Everything about telemetry
     * pacing follows from that ratio. */
    hw_lane_attach_platform(&hw, 128u);

    fw_link_t fw;
    fw_init(&fw);

    printf("  %-5s  %-12s %-4s %-4s %-6s  %s\n",
           "t[ms]", "state", "VGA", "TIA", "CTLE", "note");
    printf("  ---------------------------------------------------------\n");

    link_state_t last = LS_COUNT;
    uint32_t t = 0u;
    for (; t < MAX_TICKS && !fw_is_up(&fw); ++t) {
        /* One firmware tick per millisecond; one hardware block per tick.
         * The ratio is the point: firmware runs at kHz, the datapath at
         * 100 GBd. Everything the firmware sees is an accumulated statistic,
         * never a per-symbol value. */
        hw_lane_run(&hw, (fw.state == LS_EQ_TRAIN || fw.state == LS_CDR_LOCK) ? 1u : 0u);
        fw_tick(&fw, t * TICK_MS);

        if (getenv("LINK_TRACE") && (t % 20u) == 0u) {
            printf("        .. t=%4u %-11s phase %6.2f mean(e) %+8.5f lock %4u ppm %+7.1f\n",
                   t, fw_state_name(fw.state), hw.cdr.phase, hw.cdr.ted_avg,
                   hw.cdr.lock_count, cdr_ppm(&hw.cdr));
            printf("           adapt activity %d\n", fw_adapt_activity());
        }
        if (fw.state != last) {
            printf("  %5u  %-12s %-4u %-4u %-6u  %s\n", t, fw_state_name(fw.state),
                   hal_field_get(REG_AFE_VGA,  VGA_GAIN_MASK,  VGA_GAIN_SHIFT),
                   hal_field_get(REG_AFE_TIA,  TIA_GAIN_MASK,  TIA_GAIN_SHIFT),
                   hal_field_get(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT),
                   (fw.state == LS_FAULT) ? "<-- fault, backing off" : "");
            last = fw.state;
        }
    }

    printf("\n");
    if (fw_is_up(&fw)) {
        printf("  LINK UP after %u ms\n", fw.tm.ms_to_up);
    } else {
        printf("  LINK DID NOT COME UP within %u ms\n", MAX_TICKS);
    }

    /* ---- converged taps ------------------------------------------------- */
    printf("\n  converged FFE taps (applied codes, %d-bit signed):\n", TAP_APPLY_BITS);
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        const int32_t code = hal_read_signed(REG_FFE_TAP(i), TAP_APPLY_BITS);
        printf("    w[%u] = %+4d  %.*s\n", i, code,
               (int)(code < 0 ? -code : code), "########################");
    }
    printf("  converged DFE taps:\n");
    for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
        printf("    b[%u] = %+4d\n", i, hal_read_signed(REG_DFE_TAP(i), TAP_APPLY_BITS));
    }

    /* ---- capture an eye, then let telemetry stream it out --------------- */
    eye_t eye;
    eye_init(&eye, -1.6, 1.6);
    for (unsigned k = 0; k < 8u; ++k) {
        hw_lane_capture_eye(&hw, &eye);
    }
    hw_lane_load_eye_ram(&hw, &eye);   /* hardware fills the capture RAM */

    /* 60 ticks of steady state. The eye is 768 bytes = 30 frames = 960 bytes
     * on the wire, and the bus moves 128 bytes per tick -- so a full eye takes
     * ~8 ticks to stream while the link keeps running underneath it. */
    for (unsigned k = 0; k < 60u; ++k) {
        hw_lane_run(&hw, 0u);
        fw_tick(&fw, (t + k) * TICK_MS);
    }
    printf("\n  telemetry\n");
    printf("    state transitions   %u\n", fw.tm.transitions);
    printf("    faults / retries    %u / %u\n", fw.tm.faults, fw.retries);
    printf("    AGC updates         %u\n", fw.tm.agc_updates);
    printf("    tap updates         %u\n", fw.tm.tap_updates);
    printf("    symbols observed    %llu\n", (unsigned long long)fw.tm.symbols);
    printf("    bit errors          %llu\n", (unsigned long long)fw.tm.bit_errors);
    if (fw.tm.symbols > 0u) {
        const double ber = (double)fw.tm.bit_errors /
                           ((double)fw.tm.symbols * (double)BITS_PER_SYMBOL);
        printf("    pre-FEC BER         %.3e\n", ber);
    }
    printf("    CDR ppm estimate    %+.1f  (actual %+.1f)\n", cdr_ppm(&hw.cdr), ppm);

    printf("\n  management bus (telemetry out, ~1 Mb/s)\n");
    printf("    frames emitted      %u\n", fw_telem_frames());
    printf("    ticks deferred      %u   (FIFO was full; backpressure respected)\n",
           fw_telem_deferred());
    printf("    bytes dropped       %u   %s\n", mgmt_bus_dropped(),
           mgmt_bus_dropped() ? "<-- BUG: pushed into a full FIFO" : "(ok)");
    printf("    bytes on the wire   %zu\n", mgmt_wire_available());

    /* ---- bus hygiene: these must be zero -------------------------------- */
    const hal_stats_t *hs = hal_stats();
    printf("\n  HAL access audit\n");
    printf("    reads / writes      %llu / %llu\n",
           (unsigned long long)hs->reads, (unsigned long long)hs->writes);
    printf("    read-modify-writes  %llu\n", (unsigned long long)hs->rmw);
    printf("    UNGUARDED RMW       %llu   %s\n", (unsigned long long)hs->unguarded_rmw,
           hs->unguarded_rmw ? "<-- BUG: RMW with no critical section" : "(ok)");
    printf("    W1C RMW bugs        %llu   %s\n", (unsigned long long)hs->w1c_rmw_bugs,
           hs->w1c_rmw_bugs ? "<-- BUG: read-modify-wrote a W1C register" : "(ok)");

    /* ---- eye diagram ---------------------------------------------------- */
    printf("\n  eye at the slicer input (%llu samples)\n", (unsigned long long)eye.hits);
    eye_print_ascii(&eye, 96u, 26u);
    if (eye_write_pgm(&eye, "eye.pgm") == 0) {
        printf("  wrote eye.pgm\n");
    }

    double open[4];
    const unsigned n = eye_openings(&eye, open, 4u);
    printf("  vertical openings found: %u", n);
    for (unsigned i = 0; i < n; ++i) {
        printf("   %.3f", open[i]);
    }
    printf("\n");

    hw_lane_free(&hw);
    return fw_is_up(&fw) ? 0 : 1;
}
