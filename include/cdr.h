/* ===========================================================================
 *  cdr.h -- clock and data recovery: Mueller-Muller TED + type-2 PI loop.
 *
 *  WHY MUELLER-MULLER AND NOT GARDNER. Gardner needs two samples per symbol
 *  (a data sample and an edge sample). At 100 GBd that means running the
 *  sampler and ADC at 200 GSa/s, which is not affordable in power. Mueller-
 *  Muller is a BAUD-RATE detector -- one sample per symbol:
 *
 *      e[n] = a[n-1] * y[n]  -  a[n] * y[n-1]
 *
 *  where a[] are decisions and y[] are samples. It balances the first pre- and
 *  post-cursor ISI, so it converges to the phase where the pulse response is
 *  symmetric about the cursor. The cost is that it needs a partly equalised
 *  eye to work, which is why bring-up sequences AGC, then CDR, then the
 *  equaliser rather than starting them all at once.
 *
 *  WHY TYPE-2. A type-1 loop (one integrator, the phase accumulator itself)
 *  drives a phase STEP to zero but leaves a constant error under a FREQUENCY
 *  offset. TX and RX have independent references with ppm mismatch, so the
 *  error would ramp without bound. Adding the integral term makes it type-2:
 *  zero steady-state error to a frequency offset, at the cost of a second-
 *  order response with jitter peaking to manage.
 *
 *      phase[n+1] = phase[n] + Kp*e[n] + integ[n]
 *      integ[n+1] = integ[n] + Ki*e[n]        <- this is what makes it type-2
 * =========================================================================*/
#ifndef CDR_H
#define CDR_H

#include "link_config.h"
#include <stdint.h>

typedef struct {
    double   phase;        /* sampling phase within the UI, in samples   */
    double   integ;        /* loop-filter integrator: frequency estimate */
    double   kp, ki;
    double   ted_avg;      /* slow average of |e|, for lock detection    */
    unsigned locked;
    unsigned lock_count;
    double   y_prev;
    double   a_prev;
    double   integ_slow;   /* slow average of integ, for stability test  */
    double   amp_slow;     /* slow average of |y|, normalises TED gain   */
    uint32_t wraps;        /* phase wraps seen                           */
    uint32_t since_wrap;
} cdr_t;

void cdr_init(cdr_t *c, double kp, double ki);

/* Sample the oversampled waveform for symbol n at the current phase, using
 * linear interpolation between adjacent samples -- the model of a phase
 * interpolator. */
real_t cdr_sample(const cdr_t *c, const real_t *osr_buf, size_t nsamples, size_t sym_index);

/* Update the loop from this symbol's sample and decision level. */
void cdr_update(cdr_t *c, real_t y, real_t decision_level);

/* Recovered frequency offset in ppm, as the loop currently estimates it. */
double cdr_ppm(const cdr_t *c);

#endif /* CDR_H */
