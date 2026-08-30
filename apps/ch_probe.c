/* ===========================================================================
 *  ch_probe.c — verify the channel synthesiser.
 *
 *  Builds a channel to a requested insertion loss, then measures what it
 *  actually got by transforming the synthesised impulse response back to the
 *  frequency domain. If the synthesis is right, the measured loss lands on the
 *  requested loss. Prints the baud-rate pulse response so you can see the
 *  cursor, the precursors, and the postcursor tail.
 *
 *  Usage:  ch_probe [IL_dB_at_Nyquist]
 * =========================================================================*/
#include "channel.h"
#include "fft.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define SPAN_UI  48u
#define NTAPS    17u
#define MEAS_N   16384u

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
    const double probes[] = { 1.0, 5.0, 10.0, 25.0, NYQUIST_GHZ, 75.0 };

    printf("  freq[GHz]   model[dB]   synthesised[dB]\n");
    for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); ++i) {
        const double f = probes[i];
        const size_t k = (size_t)(f / fs_ghz * (double)MEAS_N + 0.5);
        if (k >= MEAS_N / 2u) {
            continue;
        }
        const double mag = sqrt(b[k].re * b[k].re + b[k].im * b[k].im);
        printf("  %8.1f   %8.2f   %12.2f%s\n",
               f, -channel_il_db(ch, f), 20.0 * log10(mag > 0.0 ? mag : 1e-30),
               (fabs(f - NYQUIST_GHZ) < 1e-9) ? "   <- Nyquist" : "");
    }
    free(b);
}

int main(int argc, char **argv)
{
    const double il = (argc > 1) ? atof(argv[1]) : DEFAULT_IL_DB_AT_NYQUIST;

    printf("=====================================================\n");
    printf(" %.0f Gb/s/lane PAM4  |  %.0f GBd  |  Nyquist %.0f GHz\n",
           LANE_RATE_GBPS, BAUD_RATE_GBD, NYQUIST_GHZ);
    printf(" %u lanes/macro x %u macros = %.1f Tb/s per chip\n",
           LANES_PER_MACRO, MACROS_PER_CHIP, CHIP_TBPS);
    printf(" simulation: %u samples/UI\n", OSR);
    printf("=====================================================\n\n");

    channel_t ch;
    if (channel_build(&ch, il, SPAN_UI) != 0) {
        fprintf(stderr, "channel_build failed\n");
        return 1;
    }
    printf("channel: IL = %.1f dB at Nyquist\n", il);
    printf("  fitted  a_skin = %.4f dB/sqrt(GHz)   a_diel = %.4f dB/GHz\n\n",
           ch.a_skin, ch.a_diel);

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

    free(p);
    channel_free(&ch);
    return 0;
}
