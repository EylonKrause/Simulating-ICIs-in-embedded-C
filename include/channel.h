/* ===========================================================================
 *  channel.h — synthesise a lossy interconnect channel from its insertion loss.
 *
 *  The channel is specified the way a real one is measured: an insertion-loss
 *  curve in dB versus frequency. We fit the standard two-term form
 *
 *      IL(f) [dB] = a_skin * sqrt(f_GHz) + a_diel * f_GHz
 *
 *  (skin effect goes as sqrt(f), dielectric loss goes as f), then reconstruct
 *  a MINIMUM-PHASE impulse response from that magnitude. Minimum phase matters:
 *  a magnitude-only, linear-phase channel produces a symmetric impulse response
 *  with equal precursors and postcursors, which is physically wrong. Real
 *  channels are causal and their energy trails behind the cursor -- which is
 *  precisely why a DFE (postcursor-only) is worth building at all.
 * =========================================================================*/
#ifndef CHANNEL_H
#define CHANNEL_H

#include "link_config.h"

typedef struct {
    real_t *h;        /* impulse response, OSR samples per UI          */
    size_t  n;        /* number of samples in h                        */
    double  a_skin;   /* fitted coefficients, dB per sqrt(GHz)         */
    double  a_diel;   /* dB per GHz                                    */
    double  il_target_db;
} channel_t;

/* Build a channel whose insertion loss is `il_db` at the Nyquist frequency.
 * `span_ui` sets how many unit intervals of impulse response are retained.
 * Returns 0 on success, non-zero on allocation failure. */
int  channel_build(channel_t *ch, double il_db, size_t span_ui);
void channel_free(channel_t *ch);

/* Insertion loss of the fitted model at an arbitrary frequency, in dB. */
double channel_il_db(const channel_t *ch, double f_ghz);

/* Convolve an oversampled waveform with the channel. `y` must hold `n`
 * samples; the tail beyond `n` is discarded (streaming semantics). */
void channel_apply(const channel_t *ch, const real_t *x, real_t *y, size_t n);

/* Pulse response: the channel's response to a single symbol held for one UI.
 * `p` receives `span_ui * OSR` samples. This is what the equaliser sees. */
int  channel_pulse_response(const channel_t *ch, real_t *p, size_t n);

/* Baud-rate slice of the pulse response at sampling phase `phase` (0..OSR-1).
 * Writes `ntaps` taps into `taps` and returns the index of the main cursor. */
size_t channel_baud_taps(const real_t *p, size_t np, unsigned phase,
                         real_t *taps, size_t ntaps);

#endif /* CHANNEL_H */
