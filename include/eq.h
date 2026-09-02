/* ===========================================================================
 *  eq.h -- the equaliser DATAPATH. This is hardware, not firmware.
 *
 *  The split matters and is worth stating in an interview:
 *
 *    HARDWARE (this file) runs at line rate. It filters every symbol and it
 *    ACCUMULATES the correlation between the error and each tap's regressor.
 *    Nothing here decides anything.
 *
 *    FIRMWARE (fw_adapt.c) runs at kHz. It reads the accumulators, does the
 *    fixed-point tap update, and writes new tap codes back. Every decision --
 *    step size, leakage, freeze, gear shift, convergence detection -- lives
 *    there, where it can be changed after tapeout.
 *
 *  That is why adaptation is firmware at all: channels vary, standards move,
 *  and bugs are found after silicon. A policy in firmware can be patched; the
 *  same policy hardened into logic cannot. And one control processor serves
 *  many lanes instead of replicating control logic per lane.
 *
 *  FFE cancels precursor AND postcursor but is LINEAR, so it lifts the noise
 *  wherever it lifts the signal. DFE feeds back DECISIONS, which are already
 *  sliced and therefore noiseless, so it removes postcursor ISI with no noise
 *  penalty at all -- but it cannot touch precursor ISI, because those symbols
 *  have not been decided yet. That asymmetry is why both exist.
 * =========================================================================*/
#ifndef EQ_H
#define EQ_H

#include "link_config.h"
#include "hal.h"

#define FFE_TAPS    NUM_FFE_TAPS      /* 8 */
#define FFE_CURSOR  3u
#define DFE_TAPS    NUM_DFE_TAPS      /* 4 */

/* Applied tap code -> real weight. The hardware register holds a small signed
 * integer; this is the DAC that turns it into an analogue/digital weight. */
#define TAP_CODE_SCALE  (1.0 / 32.0)

typedef struct {
    real_t   x[FFE_TAPS];        /* baud-rate sample history                 */
    real_t   d[DFE_TAPS];        /* past DECISION levels                     */
    real_t   w[FFE_TAPS];        /* FFE weights, refreshed from registers    */
    real_t   b[DFE_TAPS];        /* DFE weights, refreshed from registers    */
    int32_t  grad[FFE_TAPS];     /* sign-sign gradient accumulators          */
    int32_t  dgrad[DFE_TAPS];
    int32_t  amp_acc;            /* |y| accumulator, feeds the AGC           */
    uint32_t sym_cnt;
    uint32_t err_cnt;
} eq_t;

void eq_init(eq_t *e);

/* Pull the tap weights out of the register file. Called by the hardware model
 * whenever firmware may have written new codes -- on silicon the datapath sees
 * the register bits directly and this is a no-op. */
void eq_refresh_taps(eq_t *e);

/* One symbol through the equaliser.
 *   y_in       : the CDR-sampled input for this symbol
 *   train_sym  : known symbol during training, or UINT32_MAX for
 *                decision-directed operation
 *   sym_out    : the sliced Gray symbol
 * Returns the equalised soft value at the slicer input. */
real_t eq_step(eq_t *e, real_t y_in, unsigned train_sym, unsigned *sym_out);

/* Publish the accumulators into the RO/W1C registers the firmware reads. */
void eq_publish(const eq_t *e);
void eq_clear_accumulators(eq_t *e);

#endif /* EQ_H */
