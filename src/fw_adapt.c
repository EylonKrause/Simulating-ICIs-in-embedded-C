/* ===========================================================================
 *  fw_adapt.c -- equaliser tap adaptation. The heart of the firmware.
 *
 *  THE HARDWARE/FIRMWARE SPLIT, made concrete:
 *
 *    Hardware, at 100 GBd, per symbol, per tap:
 *        grad[i] += sign(e) * sign(x[i])
 *      One comparator per operand and an add. No multiplier -- at this rate
 *      and this tap count a multiplier per tap is unaffordable in area and
 *      power. That, and not convergence quality, is why sign-sign LMS is what
 *      silicon implements.
 *
 *    Firmware, here, once per block (kHz):
 *        acc[i] += grad[i] >> mu_shift        <- programmable step
 *        acc[i] -= acc[i] >> leak_shift       <- leakage
 *        tap[i]  = acc[i] >> TAP_APPLY_SHIFT  <- publish the high bits only
 *
 *  ACCUMULATE WIDE, APPLY NARROW. The tap register is 7 bits. If the update
 *  wrote it directly, the smallest possible step would be one applied LSB and
 *  the taps would rattle between adjacent codes forever, never settling. The
 *  20-bit accumulator makes the effective step 2^-14 of a code, so a one-bit
 *  gradient produces smooth convergence AND low steady-state dither. This one
 *  idea is why the whole scheme works.
 *
 *  THREE KNOBS, and a lab-tuning question wants all three named:
 *      mu_shift    -> convergence rate
 *      applied LSB -> steady-state tap dither
 *      leak_shift  -> drift floor, and how far the solution is biased
 * =========================================================================*/
#include "fw.h"
#include "hal.h"
#include "fixed.h"
#include <stdbool.h>

/* CONVERGENCE IS MEASURED FROM TAP MOVEMENT, NOT GRADIENT MAGNITUDE.
 *
 * Two earlier attempts used the summed |gradient| as the convergence signal
 * and both failed, in opposite directions:
 *
 *   threshold 600  -- below the noise floor, so convergence was NEVER
 *                     declared and the state machine timed out every time
 *   threshold 5600 -- above the floor, so it fired on the very first block
 *                     and the link came "up" with the taps untouched
 *
 * The reason is that a sign-sign gradient does not shrink as the taps settle:
 * it keeps oscillating about zero with roughly constant magnitude. Its SIZE
 * carries no convergence information at all -- only its running SUM does.
 *
 * So watch what actually matters: how far the applied taps moved this block.
 * That genuinely goes to zero when the solution settles. */
/* The threshold is summed over EVERY tap, so it has to scale with how many
 * there are. It was a bare 3 when the equaliser had 12 taps; at 24 taps the
 * same number demands each tap hold to an eighth of a code, which steady-state
 * dither never achieves.
 *
 * Half a code per tap is the right budget, and the reason it can be this
 * generous is that convergence is no longer the last word: LS_EQ_VERIFY
 * measures the actual error rate afterwards. A premature "converged" now costs
 * a verification window, not a bad link. Before that state existed this had to
 * be tight, and being tight is what stopped a near-ideal 4 dB channel -- where
 * the gradient is pure noise and the taps random-walk -- from ever declaring
 * convergence at all.
 */
#define CONV_TAP_DELTA   ((int32_t)((NUM_FFE_TAPS + NUM_DFE_TAPS) / 2u))
#define CONV_BLOCKS      25u    /* consecutive quiet blocks before UP    */

/* STEP SIZE IS A BIASED EXPONENT, NOT A RIGHT SHIFT.
 *
 * The first version of this file treated ADAPT_MU as a right shift only, so
 * the smallest step it could take was one gradient unit and the LARGEST was
 * the raw gradient. That turned out to be far too coarse a range in the wrong
 * direction. Measured on a 20 dB channel: a sign-sign gradient accumulates
 * about 400 counts per 4096-symbol block, one applied tap code is 2^14
 * accumulator units, so even at a shift of zero a tap moved 0.02 codes per
 * block. Converging a cursor plus four postcursors needs tens of codes, so
 * training would have had to run for tens of thousands of blocks. It ran for
 * 150, the taps never left their initial values, and the link declared itself
 * up on a completely unequalised channel.
 *
 * The gradient is a CORRELATION, not an error magnitude: its size is set by
 * the block length and the tap's correlation with the error, and there is no
 * reason for that to land near the accumulator's LSB. So the step has to be
 * able to AMPLIFY as well as attenuate. ADAPT_MU is read as a biased
 * exponent -- field value minus MU_BIAS -- which spans 2^-8 to 2^+7 in one
 * unchanged 4-bit register field.
 *
 * Biasing a control field like this is ordinary hardware practice, and it is
 * the reason the register map did not have to change to fix the bug. */
#define MU_BIAS         8

/* The FFE tap the cursor sits on. This MUST match FFE_CURSOR in eq.h.
 *
 * The firmware deliberately cannot include the datapath header -- that is the
 * whole point of the register interface -- so the two constants cannot be
 * derived from one another and nothing in the compiler will notice if they
 * drift apart. What happens if they do is quiet and nasty: adaptation pins one
 * tap while the datapath treats a different one as the cursor, so the
 * equaliser has two cursors and no reference, and every operating point
 * degrades without anything reporting an error.
 *
 * So the agreement is asserted where it can be: a unit test that includes both
 * headers and compares them. fw_adapt_cursor_tap() exists for that test and
 * for nothing else. */
#define FFE_CURSOR_TAP  4u

/* PER-LANE STATE.
 *
 * One control processor services every lane in the macro, so every piece of
 * loop state here has to exist once per lane, not once per build. The
 * supervisor calls fw_adapt_select_lane() before servicing a lane and the
 * whole file then reads and writes that lane's context -- the same trick the
 * HAL plays with its register window, and for the same reason: the per-lane
 * code stays identical no matter which lane it is running for.
 *
 * Getting this wrong is a classic. Leave the accumulators global and eight
 * lanes share one set of taps, so the macro converges to the average of eight
 * different channels and every lane is equally wrong -- while each one
 * individually reports itself converged. */
typedef struct {
    int32_t  ffe_acc[NUM_FFE_TAPS];
    int32_t  dfe_acc[NUM_DFE_TAPS];
    unsigned settled;
    int32_t  last_activity;
    int32_t  prev_tap[NUM_FFE_TAPS + NUM_DFE_TAPS];
} adapt_ctx_t;

static adapt_ctx_t g_ctx[HAL_MAX_LANES];
static unsigned    g_ln;

/* Step size and leakage live in this lane's ADAPT_CTRL register, so they
 * follow the register window. They are cached here only to save a read per
 * tap, which means the cache has to be refreshed when the lane changes. */
static int      g_mu_exp     = 0;
static unsigned g_leak_shift = 0u;     /* 0 disables leakage */

/* acc += g * 2^exp, without ever left-shifting a negative signed value --
 * that is undefined behaviour and these gradients are routinely negative. */
static int32_t mu_scale(int32_t g)
{
    if (g_mu_exp >= 0) {
        return g * (int32_t)(1u << (unsigned)g_mu_exp);
    }
    return g >> (unsigned)(-g_mu_exp);
}

unsigned fw_adapt_cursor_tap(void) { return FFE_CURSOR_TAP; }

void fw_adapt_select_lane(unsigned lane)
{
    g_ln = (lane < HAL_MAX_LANES) ? lane : 0u;
    /* Re-read this lane's gear. The previous lane may have been in a
     * different one -- acquisition on a lane that just came up, tracking on
     * the seven that are already running. */
    g_mu_exp     = (int)hal_field_get(REG_ADAPT_CTRL, ADAPT_MU_MASK,
                                      ADAPT_MU_SHIFT) - MU_BIAS;
    g_leak_shift = hal_field_get(REG_ADAPT_CTRL, ADAPT_LEAK_MASK,
                                 ADAPT_LEAK_SHIFT);
}

void fw_adapt_reset(void)
{
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        g_ctx[g_ln].ffe_acc[i] = 0;
        hal_write_signed(REG_FFE_TAP(i), 0, TAP_APPLY_BITS);
    }
    for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
        g_ctx[g_ln].dfe_acc[i] = 0;
        hal_write_signed(REG_DFE_TAP(i), 0, TAP_APPLY_BITS);
    }
    for (unsigned i = 0; i < NUM_FFE_TAPS + NUM_DFE_TAPS; ++i) {
        g_ctx[g_ln].prev_tap[i] = 0;
    }
    /* Centre spike: pass the signal through untouched until we learn better.
     * FFE_CURSOR is tap 3; one applied code of 32 == unity in TAP_CODE_SCALE. */
    g_ctx[g_ln].ffe_acc[FFE_CURSOR_TAP] = 32 << TAP_APPLY_SHIFT;
    hal_write_signed(REG_FFE_TAP(FFE_CURSOR_TAP),
                     tap_publish(g_ctx[g_ln].ffe_acc[FFE_CURSOR_TAP]), TAP_APPLY_BITS);

    g_ctx[g_ln].settled       = 0u;
    g_ctx[g_ln].last_activity = 0;
}

void fw_adapt_set_gear(unsigned mu_shift, unsigned leak_shift)
{
    /* GEAR SHIFTING: a large step during acquisition to converge quickly, a
     * small one during tracking so steady-state dither stays low. Publishing
     * it into ADAPT_CTRL means the setting is visible to a debugger and to
     * post-silicon tuning, rather than buried in firmware state. */
    g_mu_exp     = (int)mu_shift - MU_BIAS;
    g_leak_shift = leak_shift;
    hal_critical_enter();
    hal_field_set(REG_ADAPT_CTRL, ADAPT_MU_MASK,   ADAPT_MU_SHIFT,   mu_shift);
    hal_field_set(REG_ADAPT_CTRL, ADAPT_LEAK_MASK, ADAPT_LEAK_SHIFT, leak_shift);
    hal_critical_exit();
}

int fw_adapt_converged(void)
{
    return (g_ctx[g_ln].settled >= CONV_BLOCKS) ? 1 : 0;
}

int32_t fw_adapt_tap(unsigned i)
{
    return (i < NUM_FFE_TAPS) ? g_ctx[g_ln].ffe_acc[i] : 0;
}

int fw_adapt_step(void)
{
    const uint32_t ctrl = hal_read32(REG_ADAPT_CTRL);
    int32_t activity = 0;

    if ((ctrl & ADAPT_FFE_EN) != 0u) {
        for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
            /* read-and-clear the gradient accumulator: sample it and arm it
             * for the next block in one operation */
            const int32_t g = (int32_t)hal_read_clear(REG_GRAD_ACC(i));
            activity += (g < 0) ? -g : g;

            /* THE CURSOR TAP IS FROZEN. Adapting all of the FFE together with
             * the DFE leaves the overall gain unconstrained: the two can trade
             * amplitude back and forth -- the FFE growing a postcursor the DFE
             * then subtracts -- and settle into a solution that is optimal for
             * the cost function and useless for the eye. Measured here, the
             * FFE's first two postcursor taps went POSITIVE while the DFE's
             * went positive to match, and the BER sat at 20% while both loops
             * reported themselves happy.
             *
             * Pinning one tap removes that degree of freedom. The cursor is
             * the natural choice because the AGC already owns gain, so the
             * equaliser only has to own SHAPE. Real adaptive equalisers pin
             * the cursor for exactly this reason. */
            if (i == FFE_CURSOR_TAP) {
                hal_write_signed(REG_FFE_TAP(i), tap_publish(g_ctx[g_ln].ffe_acc[i]),
                                 TAP_APPLY_BITS);
                continue;
            }

            g_ctx[g_ln].ffe_acc[i] = sat_add32(g_ctx[g_ln].ffe_acc[i], mu_scale(g));
            g_ctx[g_ln].ffe_acc[i] = tap_leak(g_ctx[g_ln].ffe_acc[i], g_leak_shift);
            hal_write_signed(REG_FFE_TAP(i), tap_publish(g_ctx[g_ln].ffe_acc[i]), TAP_APPLY_BITS);
        }
    }

    if ((ctrl & ADAPT_DFE_EN) != 0u) {
        for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
            const int32_t g = (int32_t)hal_read_clear(REG_DFE_GRAD(i));
            activity += (g < 0) ? -g : g;

            /* The DFE SUBTRACTS its taps in the datapath, so the gradient sign
             * is inverted relative to the FFE. Getting this backwards turns
             * negative feedback into positive and the loop diverges -- it is
             * the single most common sign bug in an adaptation port, and the
             * first thing to check when silicon will not converge. */
            g_ctx[g_ln].dfe_acc[i] = sat_add32(g_ctx[g_ln].dfe_acc[i], -mu_scale(g));
            g_ctx[g_ln].dfe_acc[i] = tap_leak(g_ctx[g_ln].dfe_acc[i], g_leak_shift);
            hal_write_signed(REG_DFE_TAP(i), tap_publish(g_ctx[g_ln].dfe_acc[i]), TAP_APPLY_BITS);
        }
    }

    g_ctx[g_ln].last_activity = activity;

    /* How far did the APPLIED taps actually move this block? */
    int32_t moved = 0;
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        const int32_t t = hal_read_signed(REG_FFE_TAP(i), TAP_APPLY_BITS);
        moved += (t > g_ctx[g_ln].prev_tap[i]) ? (t - g_ctx[g_ln].prev_tap[i]) : (g_ctx[g_ln].prev_tap[i] - t);
        g_ctx[g_ln].prev_tap[i] = t;
    }
    for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
        const int32_t t = hal_read_signed(REG_DFE_TAP(i), TAP_APPLY_BITS);
        const unsigned j = NUM_FFE_TAPS + i;
        moved += (t > g_ctx[g_ln].prev_tap[j]) ? (t - g_ctx[g_ln].prev_tap[j]) : (g_ctx[g_ln].prev_tap[j] - t);
        g_ctx[g_ln].prev_tap[j] = t;
    }

    if (moved <= CONV_TAP_DELTA) {
        if (g_ctx[g_ln].settled < CONV_BLOCKS) {
            g_ctx[g_ln].settled++;
        }
    } else {
        g_ctx[g_ln].settled = 0u;
    }

    /* NOTE: this used to publish convergence into REG_ADAPT_STAT. Three things
     * were wrong with it and none of them was caught by the compiler:
     * REG_ADAPT_STAT is declared RO and firmware was read-modify-writing it;
     * STAT_EQ_CONV belongs to the STATUS bit namespace, not this register's,
     * so it only meant "EQ converged" by numerical accident; and nothing read
     * the result. fw_adapt_converged() is the interface. */

    return fw_adapt_converged();
}

int32_t fw_adapt_activity(void) { return g_ctx[g_ln].last_activity; }



