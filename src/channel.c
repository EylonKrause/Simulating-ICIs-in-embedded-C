#include "channel.h"
#include "fft.h"

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
    return 0;
}

void channel_free(channel_t *ch)
{
    free(ch->h);
    ch->h = NULL;
    ch->n = 0;
}

void channel_apply(const channel_t *ch, const real_t *x, real_t *y, size_t n)
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
    channel_apply(ch, x, p, n);
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
