/* ===========================================================================
 *  fw_adapt.c — equaliser tap adaptation. The heart of the firmware.
 *
 *  JD: "Implement and debug firmware-based adaptation algorithms for
 *       equalizer tap updates..."
 *  JD: "Fixed-Point arithmetic optimization for resource-constrained
 *       embedded systems."
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

#define CONV_THRESHOLD   600       /* |grad| sum below this = settled       */
#define CONV_BLOCKS      4u

static int32_t  g_ffe_acc[NUM_FFE_TAPS];
static int32_t  g_dfe_acc[NUM_DFE_TAPS];
static unsigned g_mu_shift   = 6u;
static unsigned g_leak_shift = 0u;     /* 0 disables leakage */
static unsigned g_settled;
static int32_t  g_last_activity;

void fw_adapt_reset(void)
{
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        g_ffe_acc[i] = 0;
        hal_write_signed(REG_FFE_TAP(i), 0, TAP_APPLY_BITS);
    }
    for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
        g_dfe_acc[i] = 0;
        hal_write_signed(REG_DFE_TAP(i), 0, TAP_APPLY_BITS);
    }
    /* Centre spike: pass the signal through untouched until we learn better.
     * FFE_CURSOR is tap 3; one applied code of 32 == unity in TAP_CODE_SCALE. */
    g_ffe_acc[3] = 32 << TAP_APPLY_SHIFT;
    hal_write_signed(REG_FFE_TAP(3), tap_publish(g_ffe_acc[3]), TAP_APPLY_BITS);

    g_settled       = 0u;
    g_last_activity = 0;
}

void fw_adapt_set_gear(unsigned mu_shift, unsigned leak_shift)
{
    /* GEAR SHIFTING: a large step during acquisition to converge quickly, a
     * small one during tracking so steady-state dither stays low. Publishing
     * it into ADAPT_CTRL means the setting is visible to a debugger and to
     * post-silicon tuning, rather than buried in firmware state. */
    g_mu_shift   = mu_shift;
    g_leak_shift = leak_shift;
    hal_critical_enter();
    hal_field_set(REG_ADAPT_CTRL, ADAPT_MU_MASK,   ADAPT_MU_SHIFT,   mu_shift);
    hal_field_set(REG_ADAPT_CTRL, ADAPT_LEAK_MASK, ADAPT_LEAK_SHIFT, leak_shift);
    hal_critical_exit();
}

int fw_adapt_converged(void)
{
    return (g_settled >= CONV_BLOCKS) ? 1 : 0;
}

int32_t fw_adapt_tap(unsigned i)
{
    return (i < NUM_FFE_TAPS) ? g_ffe_acc[i] : 0;
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

            g_ffe_acc[i] = sat_add32(g_ffe_acc[i], g >> g_mu_shift);
            g_ffe_acc[i] = tap_leak(g_ffe_acc[i], g_leak_shift);
            hal_write_signed(REG_FFE_TAP(i), tap_publish(g_ffe_acc[i]), TAP_APPLY_BITS);
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
            g_dfe_acc[i] = sat_add32(g_dfe_acc[i], -(g >> g_mu_shift));
            g_dfe_acc[i] = tap_leak(g_dfe_acc[i], g_leak_shift);
            hal_write_signed(REG_DFE_TAP(i), tap_publish(g_dfe_acc[i]), TAP_APPLY_BITS);
        }
    }

    g_last_activity = activity;

    if (activity < CONV_THRESHOLD) {
        if (g_settled < CONV_BLOCKS) {
            g_settled++;
        }
    } else {
        g_settled = 0u;
    }

    hal_critical_enter();
    hal_field_set(REG_ADAPT_STAT, STAT_EQ_CONV, 0u, fw_adapt_converged() ? 1u : 0u);
    hal_critical_exit();

    return fw_adapt_converged();
}
