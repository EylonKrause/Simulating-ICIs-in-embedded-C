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

#define FFE_TAPS    NUM_FFE_TAPS      /* 16 */
/* Which tap the cursor sits on: 4 precursor taps ahead of it and 11 postcursor
 * taps behind. Taps BELOW the cursor index hold newer samples and so cancel
 * PREcursor ISI; taps above hold older samples and cancel POSTcursor ISI.
 *
 * A minimum-phase channel has little precursor and a long postcursor tail, so
 * the split is deliberately lopsided. The value is measured, not assumed --
 * swept over 3, 4, 5 and 6 at 8, 16, 20 and 24 dB. Four was the only choice
 * that improved on three everywhere (24 dB went from 9.5e-4 to 3.0e-4) without
 * losing an operating point; six was better still at 24 dB and stopped 16 dB
 * coming up at all. */
#define FFE_CURSOR  4u
#define DFE_TAPS    NUM_DFE_TAPS      /* 8  */

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
    /* Set by the lane model before each eq_step: whether this symbol counts
     * toward the error statistic. The first few symbols of a simulation block
     * are a block-boundary artefact -- their training reference reaches back
     * into the previous block and the receiver pipeline straddles the seam --
     * so they are equalised and adapted on, but not SCORED. The firmware's
     * error counter and the PCS must measure the same population or bring-up
     * ends up rejecting a link the BER tester says is perfect. */
    unsigned scoring;
    /* DECISION-DIRECTED: run exactly as the link will in traffic.
     *
     * 0 = data-aided. The LMS error and the DFE feedback both come from the
     *     KNOWN training symbol. Feedback is perfect and no error propagates,
     *     which is what lets the taps converge while the eye is still closed.
     * 1 = decision-directed. Both come from the receiver's own decision, which
     *     is all it has once traffic starts.
     *
     * These are not interchangeable and the gap is not small. A solution that
     * looks flawless with oracle feedback can collapse the moment the DFE
     * starts feeding back its own mistakes: measured at 12 dB, zero errors
     * data-aided and 7.7e-2 decision-directed, a factor of a thousand.
     *
     * `scoring` is INDEPENDENT of this, and that independence is the point.
     * Verification sets dd=1 so the loop behaves exactly as it will in
     * traffic, while still scoring against the known pattern -- the only
     * configuration in which a pre-traffic measurement predicts the
     * post-traffic one. */
    unsigned dd;
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
