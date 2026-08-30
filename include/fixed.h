/* ===========================================================================
 *  fixed.h — fixed-point arithmetic for the firmware side.
 *
 *  JD: "Fixed-Point arithmetic optimization for resource-constrained
 *       embedded systems"
 *
 *  Everything the firmware computes is fixed point. There is no float in any
 *  fw_*.c translation unit -- the target is a control processor embedded in
 *  the SerDes macro, with no FPU. Floats appear only in the hardware/physics
 *  model (hw_*.c, channel.c), which stands in for analogue silicon.
 *
 *  CONVENTION, stated once:
 *      Qm.n in an N-bit signed word:  N = 1 sign + m integer + n fraction
 *      value = raw * 2^-n,  range [-2^m, 2^m - 2^-n],  resolution 2^-n
 *
 *  The two formats that matter here:
 *      q15_t  Q0.15 in int16   range [-1, 1-2^-15)   signals, error terms
 *      q20_t  Q11.20 in int32  tap accumulators      (see TAP_ACC_FRAC)
 *
 *  THE KEY IDEA -- accumulate wide, apply narrow.
 *  A tap register in hardware is only TAP_APPLY_BITS wide. If the LMS update
 *  wrote it directly, the smallest possible step would be one applied LSB and
 *  the taps would rattle between adjacent codes forever. Instead the firmware
 *  keeps a TAP_ACC_FRAC-bit accumulator and publishes only its high bits. The
 *  effective step is then far finer than the hardware resolution, which is how
 *  a one-bit sign-sign update produces smooth, stable convergence.
 * =========================================================================*/
#ifndef FIXED_H
#define FIXED_H

#include <stdint.h>

typedef int16_t q15_t;      /* Q0.15  */
typedef int32_t q31_t;      /* Q0.31 or a wide accumulator, per use */

#define Q15_FRAC        15
#define Q15_ONE         (1 << Q15_FRAC)          /* 32768 == "1.0" */

/* Tap number formats. */
#define TAP_ACC_FRAC    20      /* firmware accumulator: Q11.20            */
#define TAP_APPLY_BITS  7       /* signed field width written to hardware  */
#define TAP_APPLY_SHIFT (TAP_ACC_FRAC - (TAP_APPLY_BITS - 1))  /* 14 */
#define TAP_APPLY_MAX   ((1 << (TAP_APPLY_BITS - 1)) - 1)      /* +63 */
#define TAP_APPLY_MIN   (-(1 << (TAP_APPLY_BITS - 1)))         /* -64 */

/* ---- saturation --------------------------------------------------------- */
static inline int32_t sat_to(int32_t v, int32_t lo, int32_t hi)
{
    if (v > hi) { return hi; }
    if (v < lo) { return lo; }
    return v;
}

static inline q15_t sat16(int32_t v)
{
    return (q15_t)sat_to(v, INT16_MIN, INT16_MAX);
}

/* Saturating add. Overflow detection by sign agreement: if both operands share
 * a sign and the result differs, it overflowed.
 *
 * WHY SATURATE AND NOT WRAP: inside a feedback path a wrap turns +0.99 into
 * -0.99 -- a full-scale sign inversion inside the loop. That is divergence,
 * not a glitch. Saturation degrades gracefully. */
static inline int32_t sat_add32(int32_t a, int32_t b)
{
    const uint32_t ua = (uint32_t)a, ub = (uint32_t)b;
    const uint32_t s  = ua + ub;
    if ((((ua ^ s) & (ub ^ s)) >> 31) != 0u) {
        return (a < 0) ? INT32_MIN : INT32_MAX;
    }
    return (int32_t)s;
}

/* ---- multiply ----------------------------------------------------------- */
/* Q0.15 * Q0.15 -> Q1.30 in int32, renormalised back to Q0.15.
 *
 * ROUNDING, NOT TRUNCATION. An arithmetic right shift rounds toward -inf, so
 * truncating every product injects about -0.5 LSB of DC. Open loop that may be
 * tolerable; inside an integrator -- a CDR loop filter, an LMS accumulator --
 * the bias accumulates without bound and walks the loop off target. */
static inline q15_t q15_mul(q15_t a, q15_t b)
{
    int32_t p = (int32_t)a * (int32_t)b;         /* Q1.30 */
    p += (1 << (Q15_FRAC - 1));                  /* + half an LSB */
    return sat16(p >> Q15_FRAC);
}

static inline q15_t q15_mul_trunc(q15_t a, q15_t b)
{
    return sat16(((int32_t)a * (int32_t)b) >> Q15_FRAC);
}

/* ---- sign --------------------------------------------------------------- */
/* Branch-free three-way sign. Used by the sign-sign LMS update, where it is
 * the ONLY nonlinearity in the whole tap-update path. */
static inline int32_t sgn32(int32_t v)
{
    return (v > 0) - (v < 0);
}

/* ---- tap accumulator publication ---------------------------------------- */
/* Convert a wide accumulator to the narrow signed code the hardware register
 * takes, with saturation. This is the "apply narrow" half of the idea above. */
static inline int32_t tap_publish(int32_t acc)
{
    return sat_to(acc >> TAP_APPLY_SHIFT, TAP_APPLY_MIN, TAP_APPLY_MAX);
}

/* Leakage: bleed a tap toward zero by 2^-shift of itself, per update.
 *
 * One shift and one subtract. It kills the slow parameter drift you get when
 * the update stops correlating with the error -- no signal, stuck input, an
 * offset in the error path. The cost is that it BIASES the converged solution
 * toward zero, so size it as small as the drift allows. */
static inline int32_t tap_leak(int32_t acc, unsigned shift)
{
    return (shift == 0u) ? acc : (acc - (acc >> shift));
}

#endif /* FIXED_H */
