#include "cdr.h"
#include <math.h>
#include <string.h>

/* Lock is |MEAN of e|, not mean of |e|. The TED output is a product of
 * data-dependent levels, so its MAGNITUDE stays O(1) even when perfectly
 * locked -- only its signed mean goes to zero. Averaging |e| can never
 * detect lock, it just measures modulation depth. */
#define LOCK_THRESHOLD   0.02     /* |mean(e)| below this counts as locked */
#define LOCK_DWELL       8000u    /* consecutive symbols required      */
#define SLEW_ALPHA       1.0e-4   /* EMA rate for the residual phase slew */
#define SLEW_LIMIT       8.0e-4   /* samples/symbol still counting as held */
#define NO_WRAP_SYMBOLS  8000u    /* a tracking loop does not wrap         */

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
    c->h1_target = 0.0;
    c->phase_prev = c->phase;
    /* Start above the lock threshold, but only just. Seeding this at 1.0
     * meant the average needed ln(1/8e-4)/alpha = 71000 symbols -- ninety
     * blocks -- to decay below the limit on its own, so a loop that acquired
     * in five blocks still read NOT LOCKED for another eighty-five. The seed
     * only has to prevent a lock declaration before there is any evidence;
     * eight times the threshold does that and clears in five blocks. */
    c->slew_slow = 8.0 * SLEW_LIMIT;
}

void cdr_set_h1_target(cdr_t *c, double t)
{
    c->h1_target = t;
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
     *      e[n] = a[n-1] * y[n]  -  a[n] * y[n-1]
     *
     * SIGN MATTERS, AND IT IS EASY TO INVERT -- this file had it backwards.
     * With `phase` increasing toward LATER (cdr_sample adds it to the sample
     * index), sampling late shrinks the postcursor and grows the precursor, so
     * d/dphase of (h(+1) - h(-1)) is negative and the form above is negative
     * feedback. Written the other way round the loop becomes POSITIVE
     * feedback: it drives the phase away from the null, the error never
     * crosses zero, and the integrator walks to its limit while the phase
     * error stays stubbornly one-signed.
     *
     * That pattern is the fingerprint. A small CONSTANT mean(e) alongside a
     * monotonically running integrator means the SIGN is wrong -- it does not
     * mean the gains need tuning. A lot of time went into tuning gains against
     * this before anyone checked the sign against the header. */
    /* Residual slew: how far the phase actually moved since the last update,
     * INCLUDING the reference drift applied outside this function. A loop that
     * is tracking nulls this by construction; a loop that is sweeping cannot.
     * This is the quantity the lock detector should have been watching all
     * along -- see the note on the detector below. */
    double dphi = c->phase - c->phase_prev;
    if (dphi >  0.5 * (double)OSR) { dphi -= (double)OSR; }
    if (dphi < -0.5 * (double)OSR) { dphi += (double)OSR; }
    /* AVERAGE LONG ENOUGH THAT THE SIGNAL BEATS THE NOISE. Each symbol moves
     * the phase by kp*e, whose spread here is about 0.045 samples. An EMA at
     * alpha leaves a standard deviation of 0.045*sqrt(alpha/2); at alpha=1e-3
     * that is 1.0e-3 samples/symbol, which is LARGER than the 1.28e-3 an
     * untracked 80 ppm offset produces. A threshold anywhere in that region
     * separates nothing. At alpha=2e-5 the noise falls to 1.4e-4 and the two
     * are cleanly apart, at the cost of a 50000-symbol memory -- a dozen
     * blocks, which is a perfectly reasonable time to take before asserting
     * that a loop has acquired. */
    c->slew_slow += SLEW_ALPHA * (dphi - c->slew_slow);   /* SIGNED */
    c->phase_prev = c->phase;

    double e = (double)c->a_prev * (double)y
             - (double)decision_level * (double)c->y_prev;

    /* TARGET h1: the detector needs a zero crossing, and on this channel plain
     * Mueller-Muller does not have one.
     *
     * MM nulls where the first postcursor equals the first precursor. That is
     * a statement about a SYMMETRIC pulse, and a lossy minimum-phase channel
     * is nothing of the kind: at 30 dB the postcursor is 0.60 of the cursor
     * and the precursor 0.20, and the difference stays positive at every phase
     * in the unit interval. There is no null to find. Measured directly: the
     * detector output sat at a phase-independent +0.30 while the phase swept
     * the whole UI and the integrator wound to its rail.
     *
     * So the detector is given a target: null where
     *
     *      h(+1) - h(-1) = 2 * h1_target * h(0)
     *
     * instead of where the difference is zero. A DFE receiver WANTS a
     * postcursor -- the DFE will subtract it noiselessly -- so aiming at a
     * nonzero h1 is not a fudge, it is what the receiver architecture asks
     * for. It also places the sampling instant nearer the pulse peak, where
     * the eye is tallest, rather than out on the symmetric point.
     *
     * The third correlation, E[a[n]*y[n]] = h(0), is the cursor, and it is
     * already available for the price of one more multiply. */
    e -= 2.0 * c->h1_target * (double)decision_level * (double)y;

    /* NORMALISE BY AMPLITUDE. The raw TED output scales with signal level, so
     * the loop gain would otherwise depend on whatever the AGC happened to
     * choose -- and every AGC step would silently retune the CDR. Measured:
     * with the raw TED this loop acquired at VGA code 36 and diverged at 30,
     * a 3 dB difference. Normalising makes it amplitude-independent.
     *
     * This is the "decouple the error signals" half of stopping loops from
     * fighting: make each loop deaf to the quantity another loop controls. */
    c->amp_slow += 0.001 * (fabs((double)y) - c->amp_slow);
    /* LINEAR, not quadratic. The error is a product of a fixed constellation
     * LEVEL and a sample, so it scales as the first power of the amplitude;
     * dividing by the square overcorrects. The original amp^2 + 0.55 divisor
     * peaks at an amplitude of 1.11 and falls away on both sides -- gain
     * varied 1.9x over the amplitudes the AGC actually visits, which is the
     * opposite of the amplitude independence it was there to provide. At the
     * AGC's own target of 0.667 it evaluated to 0.994, so it was a divide by
     * one doing nothing at all. A linear divisor holds the gain to 1.25x over
     * the same range, and the 0.1 floor bounds it when the signal is absent
     * without shifting the operating point. */
    e /= (c->amp_slow + 0.1);

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
    c->integ_slow += 0.0005 * (c->integ - c->integ_slow);

    /* THE LOCK DETECTOR MUST MEASURE THE THING IT CLAIMS.
     *
     * The previous version watched |mean(e)| together with "the frequency
     * estimate has stopped moving", and it reported LOCKED on essentially
     * every block while the phase swept the entire unit interval. Three
     * separate reasons, all of which have to be closed:
     *
     *   - A phase sweeping uniformly through the UI drives the detector across
     *     its whole S-curve, and the signed mean of that is also near zero. A
     *     sweeping loop and a locked loop look identical to that test.
     *   - "Stopped moving" was measured as the gap between the integrator and
     *     a slow average of itself. When the anti-windup clamp engages the
     *     integrator stops dead, the average catches up, and the gap goes to
     *     exactly zero -- so the test was GUARANTEED to pass at precisely the
     *     moment the loop was most broken.
     *   - The wrap-based condition the comment claimed as independent was not
     *     in the predicate at all; `wraps` and `since_wrap` are written and
     *     never read.
     *
     * So: measure the residual phase slew directly, and refuse to call it lock
     * while the integrator is sitting on its rail. A tracking loop has nulled
     * its slew by construction; nothing else has.
     *
     * A lock detector that lies is worse than none, because every stage
     * downstream is gated on it and will proceed on the strength of it. */
    if (fabs(c->ted_avg) < LOCK_THRESHOLD &&
        fabs(c->slew_slow) < SLEW_LIMIT &&
        c->since_wrap > NO_WRAP_SYMBOLS &&
        fabs(c->integ) < 0.95 * CDR_INTEG_MAX) {
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




