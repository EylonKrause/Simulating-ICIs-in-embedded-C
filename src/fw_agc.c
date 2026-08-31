/* ===========================================================================
 *  fw_agc.c â€” automatic gain control over VGA and TIA.
 *
 *  JD: "...gain control (VGA/TIA)."
 *
 *  Integer only. Reads the hardware's |y| accumulator, compares it to a target,
 *  and walks the VGA code. When the VGA runs out of range it hands off to the
 *  TIA -- and that handoff is the interesting part, because raising TIA gain
 *  raises R_f and therefore LOWERS the front-end bandwidth. Gain you buy from
 *  the TIA is paid for in bandwidth, which comes back as ISI for the equaliser
 *  to remove. So: exhaust the VGA first, touch the TIA only when you must.
 * =========================================================================*/
#include "fw.h"
#include "hal.h"
#include "fixed.h"

/* Target mean |y|, in the units the hardware accumulator uses (Q4.12 per
 * symbol). Chosen so the PAM4 outer levels land inside the slicer range with
 * headroom for noise. */
/* Nominal PAM4 mean level is (1 + 1/3)/2 = 0.667 of full scale; the
 * accumulator carries |y| in Q4.12, so the target is 0.667 * 4096.
 * Setting this too low starves every downstream loop: the TED gain scales
 * with signal amplitude, so an under-driven AGC silently detunes the CDR. */
#define AGC_TARGET_Q12      2731
#define AGC_DEADBAND_Q12    120        /* hysteresis: no update inside this  */
#define AGC_SETTLE_BLOCKS   3u         /* consecutive in-band blocks = done  */
#define AGC_MAX_STEP        4          /* code steps per update, slew limit  */

/* The AGC target is SEARCHABLE. A single fixed target is a bet that one
 * amplitude suits every downstream loop, and it is not: the CDR acquires
 * over a finite range of signal level, and where that range sits depends on
 * the channel. So bring-up starts at the nominal target and, if the CDR
 * fails to acquire, retries at a different one. Real link training does
 * exactly this -- it searches, it does not assume. */
static int32_t  g_target = AGC_TARGET_Q12;
static int32_t  g_vga_bias;
static unsigned g_settled;

void fw_agc_set_target(unsigned attempt)
{
    /* Walk the target across attempts: nominal, then progressively higher,
     * covering the amplitude range over which the CDR can acquire. */
    static const int32_t LADDER[] = { 2731, 3100, 2400, 3400, 2100 };
    g_target = LADDER[attempt % (sizeof(LADDER) / sizeof(LADDER[0]))];
    /* Also bias the VGA directly. The AGC converges to an AMPLITUDE, but the
     * CDR acquires over a range of VGA CODE, and at a given channel loss the
     * two need not coincide. Walking the code offset explores that gap. */
    g_vga_bias = (int32_t)(attempt % 6u) * 2;
}

int32_t fw_agc_target(void) { return g_target; }

void fw_agc_reset(void)
{
    g_settled  = 0u;
    hal_critical_enter();
    hal_field_set(REG_AFE_VGA, VGA_GAIN_MASK, VGA_GAIN_SHIFT,
                  (uint32_t)sat_to((int32_t)(VGA_GAIN_CODES / 2u) + g_vga_bias, 0, (int32_t)VGA_GAIN_CODES - 1));
    hal_field_set(REG_AFE_TIA, TIA_GAIN_MASK, TIA_GAIN_SHIFT, 8u);
    hal_critical_exit();
}

int fw_agc_converged(void)
{
    return (g_settled >= AGC_SETTLE_BLOCKS) ? 1 : 0;
}

int fw_agc_step(void)
{
    /* Read-and-clear: these are RO/W1C accumulators, so one call both samples
     * them and arms them for the next block. */
    const uint32_t amp = hal_read_clear(REG_AMP_ACC);
    const uint32_t n   = hal_read_clear(REG_SYM_CNT);
    if (n == 0u) {
        return fw_agc_converged();
    }

    const int32_t mean = (int32_t)(amp / n);
    const int32_t err  = g_target - mean;   /* g_vga_bias shifts where we start */

    if (err > -AGC_DEADBAND_Q12 && err < AGC_DEADBAND_Q12) {
        if (g_settled < AGC_SETTLE_BLOCKS) {
            g_settled++;
        }
        return fw_agc_converged();
    }
    g_settled = 0u;

    /* Proportional step with a slew limit. The limit matters: an unbounded
     * proportional jump would overshoot, the next block would overshoot back,
     * and the loop would ring instead of settle. */
    int32_t step = err / (AGC_DEADBAND_Q12 * 2);
    step = sat_to(step, -AGC_MAX_STEP, AGC_MAX_STEP);
    if (step == 0) {
        step = (err > 0) ? 1 : -1;
    }

    /* Every read-modify-write on a shared control register is guarded. An ISR
     * touching another field of REG_AFE_VGA between our read and our write
     * would otherwise have its update silently erased -- volatile guarantees
     * the accesses happen, not that they are atomic. */
    hal_critical_enter();
    int32_t vga = (int32_t)hal_field_get(REG_AFE_VGA, VGA_GAIN_MASK, VGA_GAIN_SHIFT);
    vga += step;

    if (vga > (int32_t)VGA_GAIN_CODES - 1) {
        /* VGA is out of headroom. Borrow one code of TIA gain -- and pay for
         * it in front-end bandwidth. */
        int32_t tia = (int32_t)hal_field_get(REG_AFE_TIA, TIA_GAIN_MASK, TIA_GAIN_SHIFT);
        if (tia < (int32_t)TIA_GAIN_CODES - 1) {
            tia++;
            hal_field_set(REG_AFE_TIA, TIA_GAIN_MASK, TIA_GAIN_SHIFT, (uint32_t)tia);
            vga = (int32_t)VGA_GAIN_CODES / 2;      /* re-centre the VGA */
        } else {
            vga = (int32_t)VGA_GAIN_CODES - 1;      /* nothing left to give */
        }
    } else if (vga < 0) {
        int32_t tia = (int32_t)hal_field_get(REG_AFE_TIA, TIA_GAIN_MASK, TIA_GAIN_SHIFT);
        if (tia > 0) {
            tia--;
            hal_field_set(REG_AFE_TIA, TIA_GAIN_MASK, TIA_GAIN_SHIFT, (uint32_t)tia);
            vga = (int32_t)VGA_GAIN_CODES / 2;
        } else {
            vga = 0;
        }
    }
    hal_field_set(REG_AFE_VGA, VGA_GAIN_MASK, VGA_GAIN_SHIFT, (uint32_t)vga);
    hal_critical_exit();

    return 0;
}






