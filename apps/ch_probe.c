/* ===========================================================================
 *  ch_probe.c -- verify the channel, from either source.
 *
 *  Builds a channel, then measures what it actually got by transforming the
 *  synthesised impulse response back to the frequency domain. If the synthesis
 *  is right, the measured loss lands on the requested loss. Prints the
 *  baud-rate pulse response so you can see the cursor, the precursors and the
 *  postcursor tail.
 *
 *  Usage:
 *      ch_probe [IL_dB_at_Nyquist]        fitted two-term loss model
 *      ch_probe <file.s4p> [ports]        measured / synthesised S-parameters
 *
 *  With a Touchstone file it also prints the LONG pulse response, out to a few
 *  hundred UI, because that is where the things a fitted model cannot express
 *  actually live: reflection echoes arrive tens of UI after the cursor, long
 *  past the end of any equaliser.
 * =========================================================================*/
#include "channel.h"
#include "fft.h"
#include "touchstone.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPAN_UI   48u
#define LONG_UI  320u
#define NTAPS     17u
#define MEAS_N  16384u

static int is_touchstone(const char *s)
{
    const char *dot = strrchr(s, '.');
    return dot != NULL && (dot[1] == 's' || dot[1] == 'S') &&
           (dot[2] >= '1' && dot[2] <= '9');
}

static void measure_il(const channel_t *ch)
{
    cplx *b = (cplx *)calloc(MEAS_N, sizeof(cplx));
    if (b == NULL) {
        return;
    }
    for (size_t i = 0; i < ch->n && i < MEAS_N; ++i) {
        b[i].re = ch->h[i];
    }
    fft_run(b, MEAS_N, 0);

    const double fs_ghz = BAUD_RATE_GBD * (double)OSR;
    const double probes[] = { 1.0, 5.0, 10.0, 25.0, 40.0, NYQUIST_GHZ, 62.0, 75.0 };

    if (ch->from_sparam) {
        printf("  freq[GHz]   synthesised[dB]\n");
    } else {
        printf("  freq[GHz]   model[dB]   synthesised[dB]\n");
    }
    for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); ++i) {
        const double f = probes[i];
        const size_t k = (size_t)(f / fs_ghz * (double)MEAS_N + 0.5);
        if (k >= MEAS_N / 2u) {
            continue;
        }
        const double mag = sqrt(b[k].re * b[k].re + b[k].im * b[k].im);
        const double db  = 20.0 * log10(mag > 0.0 ? mag : 1e-30);
        const char  *tag = (fabs(f - NYQUIST_GHZ) < 1e-9) ? "   <- Nyquist" : "";
        if (ch->from_sparam) {
            printf("  %8.1f   %12.2f%s%s\n", f, db, tag,
                   (db < -60.0) ? "   <- notch" : "");
        } else {
            printf("  %8.1f   %8.2f   %12.2f%s\n",
                   f, -channel_il_db(ch, f), db, tag);
        }
    }
    free(b);
}

/* Sample the pulse response one per UI at the given phase and print anything
 * above a floor -- the point being to SEE the echo, not to list 320 zeros. */
static void print_long_tail(const channel_t *ch, unsigned phase)
{
    const size_t np = (size_t)LONG_UI * OSR;
    real_t *p = (real_t *)calloc(np, sizeof(real_t));
    if (p == NULL) {
        return;
    }
    /* The stored impulse response is only SPAN_UI long, so convolve against a
     * longer buffer explicitly: what we want to see is what the channel does
     * beyond the window the equaliser gets to work in. */
    channel_pulse_response(ch, p, np);

    double peak = 0.0;
    size_t pk = 0u;
    for (size_t i = 0; i < np; ++i) {
        if (fabs(p[i]) > peak) {
            peak = fabs(p[i]);
            pk = i;
        }
    }
    const size_t cur_ui = pk / OSR;

    printf("\n  long tail: everything above 0.5%% of the cursor, out to %u UI\n",
           LONG_UI);
    printf("  (a fitted loss model decays smoothly and has nothing out here)\n");
    unsigned shown = 0u;
    for (size_t ui = cur_ui + 1u; ui < LONG_UI; ++ui) {
        const size_t idx = ui * OSR + phase;
        if (idx >= np) {
            break;
        }
        const double v = fabs((double)p[idx]);
        if (v > 0.005 * peak) {
            const int len = (int)(v / peak * 40.0 + 0.5);
            printf("    +%3u UI  %+9.5f  %.*s\n",
                   (unsigned)(ui - cur_ui), (double)p[idx],
                   len, "========================================");
            shown++;
        }
        if (shown >= 24u) {
            printf("    ... (truncated)\n");
            break;
        }
    }
    if (shown == 0u) {
        printf("    nothing above the floor -- no reflections in this channel\n");
    }
    free(p);
}

int main(int argc, char **argv)
{
    channel_t ch;
    int from_file = 0;

    printf("=====================================================\n");
    printf(" %.0f Gb/s/lane PAM4  |  %.0f GBd  |  Nyquist %.0f GHz\n",
           LANE_RATE_GBPS, BAUD_RATE_GBD, NYQUIST_GHZ);
    printf(" %u lanes/macro x %u macros = %.1f Tb/s per chip\n",
           LANES_PER_MACRO, MACROS_PER_CHIP, CHIP_TBPS);
    printf(" simulation: %u samples/UI\n", OSR);
    printf("=====================================================\n\n");

    if (argc > 1 && is_touchstone(argv[1])) {
        from_file = 1;
        channel_ports_t pm = CHANNEL_PORTS_DEFAULT;
        if (argc > 5) {
            pm.thru_out = (unsigned)atoi(argv[2]);
            pm.thru_in  = (unsigned)atoi(argv[3]);
            pm.xt_out   = (unsigned)atoi(argv[4]);
            pm.xt_in    = (unsigned)atoi(argv[5]);
        }
        if (channel_build_from_sparam(&ch, argv[1], &pm, SPAN_UI) != 0) {
            fprintf(stderr, "cannot build channel from %s: %s\n",
                    argv[1], ts_error());
            return 1;
        }
        printf("channel: %s\n", argv[1]);
        printf("  through  S(%u,%u)      crosstalk  S(%u,%u)%s\n",
               pm.thru_out, pm.thru_in, pm.xt_out, pm.xt_in,
               (ch.hx != NULL) ? "" : "   (absent)");
        printf("  measured to %.1f GHz; above that the loss trend and group\n",
               ch.f_max_ghz);
        printf("  delay are extrapolated from the top of the sweep\n");
        printf("  bulk propagation delay removed: %.1f UI\n", ch.bulk_delay_ui);
        printf("  insertion loss at Nyquist: %.2f dB\n\n", ch.il_at_nyquist_db);
    } else {
        const double il = (argc > 1) ? atof(argv[1]) : DEFAULT_IL_DB_AT_NYQUIST;
        if (channel_build(&ch, il, SPAN_UI) != 0) {
            fprintf(stderr, "channel_build failed\n");
            return 1;
        }
        printf("channel: fitted model, IL = %.1f dB at Nyquist\n", il);
        printf("  a_skin = %.4f dB/sqrt(GHz)   a_diel = %.4f dB/GHz\n\n",
               ch.a_skin, ch.a_diel);
    }

    measure_il(&ch);

    real_t *p = (real_t *)calloc(SPAN_UI * OSR, sizeof(real_t));
    real_t  taps[NTAPS];
    if (p == NULL) {
        channel_free(&ch);
        return 1;
    }
    channel_pulse_response(&ch, p, SPAN_UI * OSR);

    /* Sample at the phase that maximises the cursor -- what a locked CDR does. */
    unsigned best_phase = 0;
    double   best_cur   = -1.0;
    for (unsigned ph = 0; ph < OSR; ++ph) {
        size_t c = channel_baud_taps(p, SPAN_UI * OSR, ph, taps, NTAPS);
        if (fabs(taps[c]) > best_cur) {
            best_cur   = fabs(taps[c]);
            best_phase = ph;
        }
    }
    const size_t cur = channel_baud_taps(p, SPAN_UI * OSR, best_phase, taps, NTAPS);

    double isi = 0.0;
    for (size_t t = 0; t < NTAPS; ++t) {
        if (t != cur) {
            isi += fabs(taps[t]);
        }
    }

    printf("\nbaud-rate pulse response (best sampling phase = %u/%u)\n",
           best_phase, OSR);
    for (size_t t = 0; t < NTAPS; ++t) {
        const int k   = (int)t - (int)cur;
        const int len = (int)(fabs(taps[t]) / best_cur * 46.0 + 0.5);
        printf("  k=%+3d %+9.5f  %.*s%s\n", k, taps[t],
               len, "==============================================",
               (k == 0) ? "  <- cursor" : "");
    }
    printf("\n  cursor      %.5f\n", taps[cur]);
    printf("  sum|ISI|    %.5f\n", isi);
    printf("  worst-case eye %s  (%.5f)\n",
           (fabs(taps[cur]) > isi) ? "OPEN" : "CLOSED",
           2.0 * (fabs(taps[cur]) - isi));

    if (from_file) {
        print_long_tail(&ch, best_phase);

        if (ch.hx != NULL) {
            double xpk = 0.0;
            for (size_t i = 0; i < ch.nx; ++i) {
                if (fabs(ch.hx[i]) > xpk) {
                    xpk = fabs(ch.hx[i]);
                }
            }
            double hpk = 0.0;
            for (size_t i = 0; i < ch.n; ++i) {
                if (fabs(ch.h[i]) > hpk) {
                    hpk = fabs(ch.h[i]);
                }
            }
            printf("\n  far-end crosstalk\n");
            printf("    peak coupling   %.3e  (%.1f dB below the through peak)\n",
                   xpk, 20.0 * log10(xpk / (hpk > 0.0 ? hpk : 1.0)));
            printf("    This is ONE aggressor. A lane with neighbours on both\n");
            printf("    sides sees them add in power, and the 48-lane part has\n");
            printf("    seven of them inside the same macro.\n");
        }
    }

    free(p);
    channel_free(&ch);
    return 0;
}
