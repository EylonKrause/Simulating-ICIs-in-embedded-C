#include "channel.h"
#include "fft.h"
#include "touchstone.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* FFT length used for the synthesis. Long enough that the impulse response
 * decays well inside it, so the circular wrap of the IFFT is negligible. */
#define SYNTH_LOG2  14u
#define SYNTH_N     (1u << SYNTH_LOG2)      /* 16384 */

/* Magnitudes below this are floored before taking a log, to keep the cepstrum
 * finite where the channel is effectively dead (hundreds of dB down). */
#define MAG_FLOOR   1e-9

double channel_il_db(const channel_t *ch, double f_ghz)
{
    if (f_ghz <= 0.0) {
        return 0.0;
    }
    return ch->a_skin * sqrt(f_ghz) + ch->a_diel * f_ghz;
}

int channel_build(channel_t *ch, double il_db, size_t span_ui)
{
    memset(ch, 0, sizeof(*ch));
    ch->il_target_db = il_db;

    /* Split the requested loss between the two mechanisms so that the sum
     * lands exactly on il_db at Nyquist. */
    const double fn = NYQUIST_GHZ;
    ch->a_skin = SKIN_FRACTION        * il_db / sqrt(fn);
    ch->a_diel = (1.0 - SKIN_FRACTION) * il_db / fn;

    cplx *buf = (cplx *)calloc(SYNTH_N, sizeof(cplx));
    if (buf == NULL) {
        return -1;
    }

    /* ---- 1. log-magnitude spectrum, Hermitian-symmetric --------------- */
    const double fs_ghz = (BAUD_RATE_GBD * (double)OSR);   /* sample rate, GHz */
    for (size_t k = 0; k < SYNTH_N; ++k) {
        const size_t kk    = (k <= SYNTH_N / 2u) ? k : (SYNTH_N - k);
        const double f_ghz = (double)kk * fs_ghz / (double)SYNTH_N;
        double mag = pow(10.0, -channel_il_db(ch, f_ghz) / 20.0);
        if (mag < MAG_FLOOR) {
            mag = MAG_FLOOR;
        }
        buf[k].re = log(mag);
        buf[k].im = 0.0;
    }

    /* ---- 2. real cepstrum --------------------------------------------- */
    fft_run(buf, SYNTH_N, 1);            /* inverse -> cepstrum, real         */

    /* ---- 3. fold to make it causal (minimum phase) --------------------- */
    for (size_t n = 1; n < SYNTH_N / 2u; ++n) {
        buf[n].re *= 2.0;
        buf[n].im  = 0.0;
    }
    for (size_t n = SYNTH_N / 2u + 1u; n < SYNTH_N; ++n) {
        buf[n].re = 0.0;
        buf[n].im = 0.0;
    }
    buf[0].im = 0.0;
    buf[SYNTH_N / 2u].im = 0.0;

    /* ---- 4. back to the complex log spectrum, then exponentiate -------- */
    fft_run(buf, SYNTH_N, 0);
    for (size_t k = 0; k < SYNTH_N; ++k) {
        const double m = exp(buf[k].re);
        const double p = buf[k].im;
        buf[k].re = m * cos(p);
        buf[k].im = m * sin(p);
    }

    /* ---- 5. impulse response ------------------------------------------ */
    fft_run(buf, SYNTH_N, 1);

    const size_t n = span_ui * OSR;
    ch->h = (real_t *)calloc(n, sizeof(real_t));
    if (ch->h == NULL) {
        free(buf);
        return -1;
    }
    for (size_t i = 0; i < n && i < SYNTH_N; ++i) {
        ch->h[i] = buf[i].re;
    }
    ch->n = n;

    free(buf);
    ch->il_at_nyquist_db = channel_measured_il_db(ch, NYQUIST_GHZ);
    return 0;
}

/* ===========================================================================
 *  Building a channel from measured S-parameters
 * =========================================================================*/

/* Interpolate a measured response onto one simulation frequency bin.
 *
 * MAGNITUDE AND UNWRAPPED PHASE ARE INTERPOLATED SEPARATELY, never real and
 * imaginary parts. A channel with propagation delay has a phase that rotates
 * many times between adjacent measured points at high frequency; interpolating
 * Re/Im across half a rotation shrinks the magnitude toward zero and invents
 * loss that is not there. Interpolating |S| and the UNWRAPPED phase keeps both
 * physically meaningful. */
typedef struct {
    double *mag;      /* |S| at each measured frequency        */
    double *phase;    /* unwrapped phase, radians              */
    size_t  n;
    const double *f;  /* measured frequencies, Hz              */
} resp_t;

static int resp_build(resp_t *r, const touchstone_t *ts,
                      unsigned out_port, unsigned in_port)
{
    r->mag   = (double *)malloc(ts->n * sizeof(double));
    r->phase = (double *)malloc(ts->n * sizeof(double));
    if (r->mag == NULL || r->phase == NULL) {
        free(r->mag);
        free(r->phase);
        r->mag = NULL;
        r->phase = NULL;
        return -1;
    }
    r->n = ts->n;
    r->f = ts->f_hz;

    double prev = 0.0;
    double acc  = 0.0;
    for (size_t k = 0; k < ts->n; ++k) {
        const ts_cplx s = ts_get(ts, k, out_port, in_port);
        r->mag[k] = sqrt(s.re * s.re + s.im * s.im);
        const double raw = atan2(s.im, s.re);
        if (k > 0) {
            /* Unwrap: fold the step into (-pi, pi] and accumulate. */
            double d = raw - prev;
            while (d >  M_PI) { d -= 2.0 * M_PI; }
            while (d < -M_PI) { d += 2.0 * M_PI; }
            acc += d;
        } else {
            acc = raw;
        }
        prev = raw;
        r->phase[k] = acc;
    }
    return 0;
}

static void resp_free(resp_t *r)
{
    free(r->mag);
    free(r->phase);
    r->mag = NULL;
    r->phase = NULL;
}

/* Value of the response at an arbitrary frequency, with the extrapolation a
 * measured file always needs.
 *
 * A VNA sweep stops at 40, 50, maybe 110 GHz. The simulation runs at
 * OSR * baud = 1600 GS/s, so its spectrum extends to 800 GHz and most of it has
 * no data behind it. Three regions:
 *
 *   below the first point   Hold the magnitude and force the phase to zero at
 *                           DC. A real impulse response is real, which forces
 *                           the DC bin to be real; anything else puts an
 *                           imaginary part into the time domain.
 *   inside the sweep        Linear interpolation of magnitude and unwrapped
 *                           phase.
 *   above the last point    Continue the loss trend in dB and the group delay,
 *                           both taken from the top decade of real data. The
 *                           magnitude therefore keeps falling rather than
 *                           stopping dead at the band edge -- a hard cut is a
 *                           brick-wall filter, and it rings for hundreds of
 *                           samples in the time domain, which would look
 *                           exactly like reflections and be entirely fictional.
 */
static void resp_at(const resp_t *r, double f_hz, double *mag, double *ph)
{
    if (f_hz <= r->f[0]) {
        *mag = r->mag[0];
        *ph  = r->phase[0] * (r->f[0] > 0.0 ? (f_hz / r->f[0]) : 0.0);
        return;
    }
    if (f_hz >= r->f[r->n - 1u]) {
        const size_t last = r->n - 1u;
        /* Fit the trend over the top 20% of the sweep, not the last two
         * points: measured data is noisiest exactly where it is thinnest. */
        size_t ref = (size_t)((double)last * 0.8);
        if (ref >= last) {
            ref = last - 1u;
        }
        const double df = r->f[last] - r->f[ref];
        if (df <= 0.0) {
            *mag = r->mag[last];
            *ph  = r->phase[last];
            return;
        }
        const double db_last = 20.0 * log10(r->mag[last] > 1e-12 ? r->mag[last] : 1e-12);
        const double db_ref  = 20.0 * log10(r->mag[ref]  > 1e-12 ? r->mag[ref]  : 1e-12);
        const double slope_db = (db_last - db_ref) / df;          /* dB per Hz */
        const double slope_ph = (r->phase[last] - r->phase[ref]) / df;

        const double dx = f_hz - r->f[last];
        double db = db_last + slope_db * dx;
        if (db > db_last) {
            db = db_last;              /* never extrapolate to MORE signal */
        }
        if (db < -240.0) {
            db = -240.0;
        }
        *mag = pow(10.0, db / 20.0);
        *ph  = r->phase[last] + slope_ph * dx;
        return;
    }

    /* Binary search for the bracketing pair. */
    size_t lo = 0u, hi = r->n - 1u;
    while (hi - lo > 1u) {
        const size_t mid = (lo + hi) / 2u;
        if (r->f[mid] <= f_hz) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const double t = (f_hz - r->f[lo]) / (r->f[hi] - r->f[lo]);
    *mag = r->mag[lo] + t * (r->mag[hi] - r->mag[lo]);
    *ph  = r->phase[lo] + t * (r->phase[hi] - r->phase[lo]);
}

/* Turn one response into a real impulse response on the simulation grid. */
static real_t *resp_to_impulse(const resp_t *r, size_t n_out, double *bulk_ui)
{
    cplx *buf = (cplx *)calloc(SYNTH_N, sizeof(cplx));
    real_t *h = (real_t *)calloc(n_out, sizeof(real_t));
    if (buf == NULL || h == NULL) {
        free(buf);
        free(h);
        return NULL;
    }

    const double fs_hz = BAUD_RATE_GBD * 1e9 * (double)OSR;

    for (size_t k = 0; k <= SYNTH_N / 2u; ++k) {
        const double f = (double)k * fs_hz / (double)SYNTH_N;
        double mag, ph;
        resp_at(r, f, &mag, &ph);

        if (k == 0u || k == SYNTH_N / 2u) {
            /* DC and Nyquist bins of a real signal must be real. */
            buf[k].re = mag * cos(ph);
            buf[k].im = 0.0;
        } else {
            buf[k].re = mag * cos(ph);
            buf[k].im = mag * sin(ph);
            /* HERMITIAN SYMMETRY. Skip this and the inverse FFT returns a
             * complex "impulse response", and taking its real part quietly
             * halves the energy and mirrors the response about t=0 -- which
             * looks like an acausal precursor that is not in the data. */
            buf[SYNTH_N - k].re =  buf[k].re;
            buf[SYNTH_N - k].im = -buf[k].im;
        }
    }

    fft_run(buf, SYNTH_N, 1);          /* inverse -> real impulse response */

    /* Find the bulk propagation delay and remove it.
     *
     * A 10 cm trace is about 700 ps of flight time, which at 100 GBd is 70 UI.
     * Keeping it would push the whole response past the end of a 48 UI buffer
     * and leave nothing but zeros. A CONSTANT delay carries no information --
     * it shifts every symbol equally and the CDR absorbs it. What must NOT be
     * removed is the delay BETWEEN features: reflections arrive at two and
     * three times the one-way flight time, and a constant shift preserves
     * those intervals exactly. */
    size_t pk = 0u;
    double peak = 0.0;
    for (size_t i = 0; i < SYNTH_N; ++i) {
        const double v = fabs(buf[i].re);
        if (v > peak) {
            peak = v;
            pk = i;
        }
    }
    /* Back off to where the leading edge actually starts, so genuine
     * precursors survive the shift. */
    size_t start = pk;
    const double edge = peak * 0.02;
    while (start > 0u && fabs(buf[start - 1u].re) > edge) {
        --start;
    }
    const size_t guard = 2u * OSR;
    start = (start > guard) ? (start - guard) : 0u;

    for (size_t i = 0; i < n_out; ++i) {
        const size_t j = start + i;
        h[i] = (j < SYNTH_N) ? (real_t)buf[j].re : (real_t)0.0;
    }
    if (bulk_ui != NULL) {
        *bulk_ui = (double)start / (double)OSR;
    }

    free(buf);
    return h;
}

int channel_build_from_sparam(channel_t *ch, const char *path,
                              const channel_ports_t *ports, size_t span_ui)
{
    memset(ch, 0, sizeof(*ch));

    channel_ports_t pm = CHANNEL_PORTS_DEFAULT;
    if (ports != NULL) {
        pm = *ports;
    }

    touchstone_t ts;
    if (ts_load(&ts, path, 0u) != 0) {
        return -1;
    }
    if (pm.thru_out > ts.ports || pm.thru_in > ts.ports ||
        pm.thru_out == 0u || pm.thru_in == 0u) {
        ts_free(&ts);
        return -2;
    }

    const size_t n = span_ui * OSR;

    resp_t thru;
    if (resp_build(&thru, &ts, pm.thru_out, pm.thru_in) != 0) {
        ts_free(&ts);
        return -1;
    }
    ch->h = resp_to_impulse(&thru, n, &ch->bulk_delay_ui);
    resp_free(&thru);
    if (ch->h == NULL) {
        ts_free(&ts);
        return -1;
    }
    ch->n = n;

    if (pm.xt_in != 0u && pm.xt_out != 0u &&
        pm.xt_in <= ts.ports && pm.xt_out <= ts.ports) {
        resp_t xt;
        if (resp_build(&xt, &ts, pm.xt_out, pm.xt_in) == 0) {
            double dummy = 0.0;
            ch->hx = resp_to_impulse(&xt, n, &dummy);
            ch->nx = (ch->hx != NULL) ? n : 0u;
            resp_free(&xt);
        }
    }

    ch->from_sparam = 1;
    ch->f_max_ghz   = ts.f_hz[ts.n - 1u] / 1e9;
    ts_free(&ts);

    ch->il_at_nyquist_db = channel_measured_il_db(ch, NYQUIST_GHZ);
    ch->il_target_db     = ch->il_at_nyquist_db;
    return 0;
}

/* Insertion loss of whatever is actually in h[], by direct evaluation of the
 * DTFT at one frequency. Slow and obvious on purpose -- this is the check that
 * the synthesis did what was asked, so it must not share code with it. */
double channel_measured_il_db(const channel_t *ch, double f_ghz)
{
    const double fs_ghz = BAUD_RATE_GBD * (double)OSR;
    const double w = 2.0 * M_PI * f_ghz / fs_ghz;
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < ch->n; ++i) {
        re += (double)ch->h[i] * cos(w * (double)i);
        im -= (double)ch->h[i] * sin(w * (double)i);
    }
    const double mag = sqrt(re * re + im * im);
    return -20.0 * log10(mag > 1e-12 ? mag : 1e-12);
}

void channel_free(channel_t *ch)
{
    free(ch->fft_h);
    free(ch->fft_hx);
    free(ch->ola_tail_h);
    free(ch->ola_tail_hx);
    ch->fft_h  = NULL;
    ch->fft_hx = NULL;
    ch->ola_tail_h  = NULL;
    ch->ola_tail_hx = NULL;
    free(ch->h);
    free(ch->hx);
    ch->h  = NULL;
    ch->hx = NULL;
    ch->n  = 0;
    ch->nx = 0;
}

/* ---------------------------------------------------------------------------
 *  Convolution, by overlap-add.
 *
 *  The direct form below is the definition and is what the unit test checks
 *  against. It is also 768 multiply-accumulates per output sample, which at
 *  65536 samples per lane-block and eight lanes is 26 billion per simulated
 *  millisecond -- enough to make an 8-lane run take hours instead of a minute.
 *
 *  Overlap-add splits the input into blocks of B, transforms each into a
 *  2B-point FFT along with the (cached) transform of the impulse response,
 *  multiplies, transforms back, and adds the overlapping tails. Cost per
 *  output sample drops from O(taps) to O(log B). The padding to 2B is what
 *  makes it a LINEAR convolution rather than a circular one -- without it the
 *  tail of each block wraps around and lands on top of the block's own start,
 *  which is a silent corruption that looks like a plausible echo.
 * ------------------------------------------------------------------------*/
#define OLA_LOG2   11u
#define OLA_B      (1u << OLA_LOG2)          /* 2048 input samples per block */
#define OLA_N      (OLA_B * 2u)              /* 4096-point transform         */

static int ola_prepare(const channel_t *ch, const real_t *h, size_t hn,
                       cplx **cache, real_t **tail)
{
    if (*cache != NULL) {
        return 0;
    }
    if (hn == 0u || hn > OLA_B) {
        return -1;             /* impulse response longer than a block */
    }
    cplx   *H = (cplx *)calloc(OLA_N, sizeof(cplx));
    real_t *T = (real_t *)calloc(OLA_N, sizeof(real_t));
    if (H == NULL || T == NULL) {
        free(H);
        free(T);
        return -1;
    }
    for (size_t i = 0; i < hn; ++i) {
        H[i].re = (double)h[i];
    }
    fft_run(H, OLA_N, 0);
    *cache = H;
    *tail  = T;
    (void)ch;
    return 0;
}

/* `tail` is the channel's MEMORY and it persists across calls.
 *
 * A channel with 48 UI of impulse response does not forget what went into it
 * 48 UI ago just because the simulator chose to process the waveform in
 * blocks. Zeroing this between calls convolves the first 768 samples of every
 * block against silence, so 48 symbols in every 4096 carry ISI that is simply
 * absent -- and those symbols are then wrong at a rate the receiver can do
 * nothing about.
 *
 * That artefact is not harmless. Measured at 20 dB it put a floor of about
 * 2.4e-4 on the training BER, which is exactly the KP4 limit, so bring-up kept
 * rejecting a receiver that was in fact working and retrying forever. A model
 * that manufactures errors at the specification limit is worse than no model.
 *
 * Overlap-add makes the fix free: the tail it already carries between
 * sub-blocks IS the channel state, so it only has to stop being thrown away. */
static void ola_convolve(const cplx *H, real_t *tail, const real_t *x,
                         real_t *y, size_t n, int accumulate, double scale)
{
    /* One scratch transform, reused across every lane and every call. Static
     * storage rather than a lazily-allocated buffer, so there is nothing to
     * leak: a never-freed static allocation is exactly what a leak checker
     * reports, and CI runs one. The model is single threaded by construction
     * -- the hardware it stands in for is not, but the simulation of it is. */
    static cplx X[OLA_N];

    size_t out = 0u;
    for (size_t base = 0; base < n; base += OLA_B) {
        const size_t len = (n - base < OLA_B) ? (n - base) : OLA_B;

        for (size_t i = 0; i < OLA_N; ++i) {
            X[i].re = (i < len) ? (double)x[base + i] : 0.0;
            X[i].im = 0.0;
        }
        fft_run(X, OLA_N, 0);
        for (size_t i = 0; i < OLA_N; ++i) {
            const double re = X[i].re * H[i].re - X[i].im * H[i].im;
            const double im = X[i].re * H[i].im + X[i].im * H[i].re;
            X[i].re = re;
            X[i].im = im;
        }
        fft_run(X, OLA_N, 1);

        /* Add this block's result to the tail left by the previous one, emit
         * the first B samples, and carry the rest forward. */
        for (size_t i = 0; i < OLA_N; ++i) {
            tail[i] += X[i].re;
        }
        for (size_t i = 0; i < len && out < n; ++i, ++out) {
            const real_t v = (real_t)(scale * tail[i]);
            if (accumulate) {
                y[out] += v;
            } else {
                y[out] = v;
            }
        }
        /* Shift by what was actually EMITTED, not by the nominal block size.
         * They differ only on a final partial block -- and every call in this
         * project happens to use a length that is an exact multiple, so the
         * difference never shows up here. That is precisely what makes it
         * worth getting right: it is a landmine armed for the first caller
         * who passes an odd length. */
        for (size_t i = 0; i + len < OLA_N; ++i) {
            tail[i] = tail[i + len];
        }
        for (size_t i = OLA_N - len; i < OLA_N; ++i) {
            tail[i] = 0.0;
        }
    }
}

/* The definition, kept as the reference the fast path is tested against. */
void channel_apply_direct(const channel_t *ch, const real_t *x, real_t *y,
                          size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        real_t acc = 0.0;
        const size_t kmax = (ch->n - 1u < i) ? (ch->n - 1u) : i;
        for (size_t k = 0; k <= kmax; ++k) {
            acc += ch->h[k] * x[i - k];
        }
        y[i] = acc;
    }
}

void channel_apply(const channel_t *ch, const real_t *x, real_t *y, size_t n)
{
    channel_t *m = (channel_t *)ch;      /* the cache is the only mutation */
    if (ola_prepare(ch, ch->h, ch->n, &m->fft_h, &m->ola_tail_h) == 0) {
        ola_convolve(m->fft_h, m->ola_tail_h, x, y, n, 0, 1.0);
        return;
    }
    channel_apply_direct(ch, x, y, n);
}

void channel_apply_xtalk(const channel_t *ch, const real_t *x, real_t *y,
                         size_t n, double scale)
{
    if (ch->hx == NULL || ch->nx == 0u) {
        return;
    }
    channel_t *m = (channel_t *)ch;
    if (ola_prepare(ch, ch->hx, ch->nx, &m->fft_hx, &m->ola_tail_hx) == 0) {
        /* ACCUMULATE. An aggressor adds to what the victim already carries;
         * it does not replace it. Overwriting here would silently model a
         * switch rather than a coupling. */
        ola_convolve(m->fft_hx, m->ola_tail_hx, x, y, n, 1, scale);
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        real_t acc = 0.0;
        const size_t kmax = (ch->nx - 1u < i) ? (ch->nx - 1u) : i;
        for (size_t k = 0; k <= kmax; ++k) {
            acc += ch->hx[k] * x[i - k];
        }
        y[i] += (real_t)(scale * (double)acc);
    }
}

int channel_pulse_response(const channel_t *ch, real_t *p, size_t n)
{
    /* A single symbol held high for one UI = a rectangle of OSR samples. */
    real_t *x = (real_t *)calloc(n, sizeof(real_t));
    if (x == NULL) {
        return -1;
    }
    for (size_t i = 0; i < OSR && i < n; ++i) {
        x[i] = 1.0;
    }
    /* THE DIRECT FORM ON PURPOSE, not channel_apply().
     *
     * A pulse response is a single-shot measurement from a quiescent channel:
     * it must start from zero state and, crucially, must not LEAVE state
     * behind. channel_apply() is the streaming path and carries an
     * overlap-add tail from one call to the next, so measuring the pulse
     * response through it would both start from whatever the channel happened
     * to be carrying and then poison the next real block with the
     * measurement's own tail.
     *
     * hw_lane_init() calls this to seed the training-reference delay, so that
     * poisoning would have landed on the very first block of every link. */
    channel_apply_direct(ch, x, p, n);
    free(x);
    return 0;
}

size_t channel_baud_taps(const real_t *p, size_t np, unsigned phase,
                         real_t *taps, size_t ntaps)
{
    /* Find the peak of the pulse response; that UI is the cursor. */
    size_t pk = 0;
    for (size_t i = 1; i < np; ++i) {
        if (fabs(p[i]) > fabs(p[pk])) {
            pk = i;
        }
    }
    const size_t cursor_ui = pk / OSR;

    /* Slice one sample per UI at the requested sub-UI phase. */
    size_t cursor_tap = 0;
    for (size_t t = 0; t < ntaps; ++t) {
        const long ui  = (long)t - (long)ntaps / 2 + (long)cursor_ui;
        const long idx = ui * (long)OSR + (long)phase;
        taps[t] = (idx >= 0 && (size_t)idx < np) ? p[idx] : 0.0;
        if ((size_t)ui == cursor_ui) {
            cursor_tap = t;
        }
    }
    return cursor_tap;
}
