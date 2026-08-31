#include "cdr.h"
#include <math.h>
#include <string.h>

/* Lock is |MEAN of e|, not mean of |e|. The TED output is a product of
 * data-dependent levels, so its MAGNITUDE stays O(1) even when perfectly
 * locked -- only its signed mean goes to zero. Averaging |e| can never
 * detect lock, it just measures modulation depth. */
#define LOCK_THRESHOLD   0.02     /* |mean(e)| below this counts as locked */
#define LOCK_DWELL       2000u    /* consecutive symbols required      */

/* Anti-windup limit, in samples of phase advance per symbol. 1000 ppm is far
 * beyond any real reference mismatch (specs are tens to low hundreds), so this
 * bounds the integrator without constraining legitimate tracking. */
#define CDR_PPM_LIMIT    1000.0
#define CDR_INTEG_MAX    (CDR_PPM_LIMIT * 1e-6 * (double)OSR)

void cdr_init(cdr_t *c, double kp, double ki)
{
    memset(c, 0, sizeof(*c));
    c->phase   = (double)OSR / 2.0;   /* start mid-UI, deliberately wrong */
    c->kp      = kp;
    c->ki      = ki;
    c->ted_avg  = 1.0;   /* start far from lock */
    c->amp_slow = 0.5;
}

real_t cdr_sample(const cdr_t *c, const real_t *osr_buf, size_t nsamples, size_t sym_index)
{
    double pos = (double)sym_index * (double)OSR + c->phase;
    if (pos < 0.0) {
        pos = 0.0;
    }
    const size_t i0 = (size_t)pos;
    const size_t i1 = i0 + 1u;
    if (i1 >= nsamples) {
        return (i0 < nsamples) ? osr_buf[i0] : 0.0;
    }
    /* Linear interpolation between adjacent samples: this is the phase
     * interpolator. Real silicon uses a weighted mix of clock phases; the
     * arithmetic effect is the same. */
    const double frac = pos - (double)i0;
    return (real_t)((1.0 - frac) * osr_buf[i0] + frac * osr_buf[i1]);
}

void cdr_update(cdr_t *c, real_t y, real_t decision_level)
{
    /* Mueller-Muller timing error detector:
     *
     *      e[n] = a[n] * y[n-1]  -  a[n-1] * y[n]
     *
     * SIGN MATTERS, AND IT IS EASY TO INVERT. Written the other way round the
     * loop becomes positive feedback: it drives the phase AWAY from the null,
     * the error never crosses zero, and the integrator walks to its limit
     * while the phase error stays stubbornly one-signed.
     *
     * That pattern is the fingerprint. A small CONSTANT mean(e) alongside a
     * monotonically running integrator means the SIGN is wrong -- it does not
     * mean the gains need tuning. I spent a long time tuning gains against
     * this before checking the sign. */
    double e = (double)decision_level * (double)c->y_prev
             - (double)c->a_prev * (double)y;

    /* NORMALISE BY AMPLITUDE. The raw TED output scales with signal level, so
     * the loop gain would otherwise depend on whatever the AGC happened to
     * choose -- and every AGC step would silently retune the CDR. Measured:
     * with the raw TED this loop acquired at VGA code 36 and diverged at 30,
     * a 3 dB difference. Normalising makes it amplitude-independent.
     *
     * This is the "decouple the error signals" half of stopping loops from
     * fighting: make each loop deaf to the quantity another loop controls. */
    c->amp_slow += 0.001 * (fabs((double)y) - c->amp_slow);
    e /= (c->amp_slow * c->amp_slow + 0.55);

    /* Type-2 PI loop filter, with ANTI-WINDUP on the integrator.
     *
     * The clamp is not optional. Any residual DC in the TED -- and Mueller-
     * Muller has plenty of it until the equaliser has made the pulse response
     * roughly symmetric about the cursor -- drives an unclamped integrator
     * without bound. Observed here: the frequency estimate ran to -156000 ppm
     * while the phase error stayed small, which is the classic windup
     * signature. Clamping to a physically possible frequency offset keeps the
     * loop recoverable instead of latched. */
    c->integ += c->ki * e;
    if (c->integ >  CDR_INTEG_MAX) { c->integ =  CDR_INTEG_MAX; }
    if (c->integ < -CDR_INTEG_MAX) { c->integ = -CDR_INTEG_MAX; }
    c->phase += c->kp * e + c->integ;

    /* Keep the phase inside one UI. Wrapping is not cosmetic -- it is how a
     * real interpolator rolls over, and the wrap is what lets the loop track
     * an unbounded frequency offset with a bounded phase register. */
    if (c->phase >= (double)OSR || c->phase < 0.0) {
        while (c->phase >= (double)OSR) { c->phase -= (double)OSR; }
        while (c->phase <  0.0)         { c->phase += (double)OSR; }
        c->wraps++;
        c->since_wrap = 0u;      /* a wrap means we are still slewing */
    } else if (c->since_wrap < 0xFFFFFFFFu) {
        c->since_wrap++;
    }

    /* Lock detection: a slow average of |e|, with dwell so a momentary dip
     * does not declare lock. */
    /* TWO independent conditions. |mean(e)| alone is not enough: a phase
     * sweeping uniformly through the UI drives the TED across its whole
     * S-curve, and the signed mean of that is also ~0. A sweeping loop and a
     * locked loop look identical to that test. Requiring that the phase has
     * not WRAPPED for a while is the second, independent condition -- a
     * tracking loop does not wrap. */
    c->ted_avg += 0.001 * (e - c->ted_avg);          /* SIGNED average */
    /* Second condition: the FREQUENCY ESTIMATE has stopped moving. A loop
     * still slewing has an integrator that is still climbing; an acquired
     * loop has one that has settled. This is robust to the phase wrapping,
     * which a correctly-tracking loop still does slowly under residual
     * frequency error -- the earlier wrap-based test rejected those. */
    c->integ_slow += 0.0005 * (c->integ - c->integ_slow);
    const double integ_drift = fabs(c->integ - c->integ_slow);

    if (fabs(c->ted_avg) < LOCK_THRESHOLD && integ_drift < 8.0e-4) {
        if (c->lock_count < LOCK_DWELL) {
            c->lock_count++;
        }
    } else if (c->lock_count > 0u) {
        c->lock_count--;
    }
    c->locked = (c->lock_count >= LOCK_DWELL) ? 1u : 0u;

    c->y_prev = y;
    c->a_prev = decision_level;
}

double cdr_ppm(const cdr_t *c)
{
    /* integ is the per-symbol phase correction the loop applies, in samples.
     * To CANCEL a positive drift it must be NEGATIVE, so the offset being
     * compensated is its negation. Getting this sign wrong makes a perfectly
     * healthy loop look like it is tracking backwards. */
    return -c->integ / (double)OSR * 1e6;
}




