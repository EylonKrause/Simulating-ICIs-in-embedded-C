/* ===========================================================================
 *  channel.h -- the interconnect channel, from either of two sources.
 *
 *  1. A FITTED LOSS MODEL. The channel is specified the way a datasheet
 *     specifies one: an insertion-loss curve in dB versus frequency, fitted to
 *     the standard two-term form
 *
 *         IL(f) [dB] = a_skin * sqrt(f_GHz) + a_diel * f_GHz
 *
 *     (skin effect goes as sqrt(f), dielectric loss goes as f). A MINIMUM-PHASE
 *     impulse response is then reconstructed from that magnitude. Minimum phase
 *     matters: a magnitude-only, linear-phase channel gives a symmetric impulse
 *     response with equal precursors and postcursors, which is physically
 *     wrong. Real channels are causal and their energy trails behind the
 *     cursor -- which is precisely why a DFE is worth building at all.
 *
 *  2. MEASURED S-PARAMETERS, from a Touchstone file. This is what a real
 *     project uses, and it carries three things the fitted model cannot:
 *
 *       reflections   Every impedance discontinuity returns energy. It comes
 *                     back as a long-delayed echo, appearing in the pulse
 *                     response tens or hundreds of UI after the cursor --
 *                     outside any equaliser's span, and invisible to a smooth
 *                     monotonic loss fit.
 *
 *       via stubs     The unused barrel of a through via is a quarter-wave
 *                     resonator, putting a deep narrow notch in the insertion
 *                     loss. If it lands near Nyquist the link is dead: no
 *                     equaliser recovers a frequency the channel does not
 *                     pass. Back-drilling exists because of this.
 *
 *       crosstalk     The off-diagonal terms. On a 48-lane part this, not
 *                     loss, is usually the dominant impairment.
 *
 *     The fitted model has none of them. It is smooth, monotonic and
 *     optimistic, and every one of those three is the kind of thing that gets
 *     found on the bench after tapeout.
 * =========================================================================*/
#ifndef CHANNEL_H
#define CHANNEL_H

#include "link_config.h"
#include "fft.h"

typedef struct {
    real_t *h;        /* through impulse response, OSR samples per UI     */
    size_t  n;
    real_t *hx;       /* far-end crosstalk impulse response, or NULL      */
    size_t  nx;

    double  a_skin;   /* fitted coefficients, dB per sqrt(GHz)            */
    double  a_diel;   /* dB per GHz                                       */
    double  il_target_db;

    int     from_sparam;      /* non-zero if built from a Touchstone file */
    double  f_max_ghz;        /* highest measured frequency               */
    double  bulk_delay_ui;    /* propagation delay removed from h         */
    double  il_at_nyquist_db; /* measured, whatever the source            */

    /* Cached transforms of h and hx for the overlap-add convolution. Built
     * lazily on first use and owned by channel_free(). */
    cplx   *fft_h;
    cplx   *fft_hx;
    /* Overlap-add carry: this IS the channel memory between blocks. */
    real_t *ola_tail_h;
    real_t *ola_tail_hx;
} channel_t;

/* Build a channel whose insertion loss is `il_db` at the Nyquist frequency.
 * `span_ui` sets how many unit intervals of impulse response are retained.
 * Returns 0 on success, non-zero on allocation failure. */
int  channel_build(channel_t *ch, double il_db, size_t span_ui);

/* Which S-parameters to use out of an N-port file.
 *
 * There is no universal port numbering for a coupled-pair Touchstone file. The
 * two common conventions are "chained" (1,2 are the ends of the first line;
 * 3,4 the ends of the second) and "paired" (1,3 are the near ends; 2,4 the far
 * ends). Reading a file under the wrong one gives a through response that is
 * actually crosstalk -- 40 dB down and completely wrong, which at least fails
 * loudly. Making the map an explicit parameter is the only safe answer. */
typedef struct {
    unsigned thru_out, thru_in;    /* S(thru_out, thru_in)  = the victim path */
    unsigned xt_out,   xt_in;      /* S(xt_out, xt_in)      = FEXT, 0 to skip */
} channel_ports_t;

/* Default: through = S21, far-end crosstalk = S23 (aggressor driven into
 * port 3, coupling out of the victim's far end). Chained convention. */
#define CHANNEL_PORTS_DEFAULT  ((channel_ports_t){ 2u, 1u, 2u, 3u })

/* Build from a Touchstone file. `ports` may be NULL for the default map.
 * Returns 0 on success; on failure ts_error() carries the reason. */
int  channel_build_from_sparam(channel_t *ch, const char *path,
                               const channel_ports_t *ports, size_t span_ui);

void channel_free(channel_t *ch);

/* Insertion loss of the fitted model at an arbitrary frequency, in dB. Only
 * meaningful for a fitted channel; returns 0 for an S-parameter one. */
double channel_il_db(const channel_t *ch, double f_ghz);

/* Convolve an oversampled waveform with the channel. `y` must hold `n`
 * samples.
 *
 * `ch` IS NOT CONST, and the missing const is the interface, not an oversight.
 * This is the STREAMING path: the convolution tail that runs past the end of
 * one block is held inside the channel and added to the front of the next, so
 * consecutive calls are a continuous filter rather than a sequence of
 * independent ones. The channel carries memory, and the signature says so.
 *
 * It used to say `const` and cast that away internally, which is worse than
 * useless -- it advertises a guarantee the code then breaks, and it invites a
 * caller to share one channel between two streams, which silently splices
 * their tails together. Calling it on a genuinely const object was also
 * undefined behaviour.
 *
 * For a single-shot measurement from a quiescent channel, use the direct form
 * below, which really is stateless. */
void channel_apply(channel_t *ch, const real_t *x, real_t *y, size_t n);

/* The textbook form, O(taps) per sample. Kept because it IS the definition:
 * the fast path is verified against it in the unit tests rather than against
 * itself. */
void channel_apply_direct(const channel_t *ch, const real_t *x, real_t *y,
                          size_t n);

/* Same, through the CROSSTALK response, accumulating into `y` rather than
 * overwriting it -- an aggressor adds to whatever the victim already has.
 * A no-op if the channel carries no crosstalk data.
 *
 * Also stateful, and for the same reason: it keeps its own overlap-add tail,
 * separate from the through path's. */
void channel_apply_xtalk(channel_t *ch, const real_t *x, real_t *y,
                         size_t n, double scale);

/* Pulse response: the channel's response to a single symbol held for one UI.
 * `p` receives `span_ui * OSR` samples. This is what the equaliser sees. */
int  channel_pulse_response(const channel_t *ch, real_t *p, size_t n);

/* Baud-rate slice of the pulse response at sampling phase `phase` (0..OSR-1).
 * Writes `ntaps` taps into `taps` and returns the index of the main cursor. */
size_t channel_baud_taps(const real_t *p, size_t np, unsigned phase,
                         real_t *taps, size_t ntaps);

/* Insertion loss of the ACTUAL synthesised response at a frequency, in dB.
 * Works for both sources, so a fitted and a measured channel can be compared
 * on the same axis. */
double channel_measured_il_db(const channel_t *ch, double f_ghz);

#endif /* CHANNEL_H */
