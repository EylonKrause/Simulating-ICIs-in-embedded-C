/* ===========================================================================
 *  touchstone.h -- read a Touchstone (.sNp) S-parameter file.
 *
 *  This is how a channel actually arrives in practice. Nobody hands a SerDes
 *  team a loss coefficient; they hand over a VNA sweep or a field-solver
 *  export of the package, the connector, the backplane and the vias, and the
 *  receiver has to work against whatever is in it.
 *
 *  What a measured file has that a fitted sqrt(f) + f loss curve does not:
 *
 *    REFLECTIONS. Every impedance discontinuity -- a connector, a via, a
 *    package ball -- sends energy back. It returns as a long-delayed echo,
 *    which shows up in the pulse response as ISI hundreds of UI after the
 *    cursor, far outside any equaliser's reach.
 *
 *    VIA STUBS. The unused barrel of a through via is a quarter-wave
 *    resonator. It puts a deep, narrow notch in the insertion loss, and if
 *    that notch lands near Nyquist the link is simply dead -- no amount of
 *    equalisation recovers a frequency the channel does not pass. Back-
 *    drilling exists entirely because of this.
 *
 *    CROSSTALK. The off-diagonal terms. In a 48-lane part this is the
 *    dominant impairment, not loss, and it is the reason lanes are simulated
 *    together rather than one at a time.
 *
 *  A fitted two-term model has none of these: it is smooth, monotonic and
 *  optimistic. Every one of them is the sort of thing that is found on the
 *  bench after tapeout by people who wonder why the model said it would work.
 * =========================================================================*/
#ifndef TOUCHSTONE_H
#define TOUCHSTONE_H

#include <stddef.h>

#define TS_MAX_PORTS   8u
#define TS_MAX_POINTS  4096u

typedef struct {
    double re, im;
} ts_cplx;

typedef struct {
    unsigned ports;
    size_t   n;                    /* frequency points                     */
    double   z0;                   /* reference impedance, ohms            */
    double  *f_hz;                 /* n frequencies, ascending             */
    ts_cplx *s;                    /* n * ports * ports, row major per f   */
} touchstone_t;

/* Load a Touchstone v1 file. `ports` may be 0 to deduce it from the file
 * extension (.s2p, .s4p, ...). Returns 0 on success. */
int  ts_load(touchstone_t *ts, const char *path, unsigned ports);
void ts_free(touchstone_t *ts);

/* S[i][j] at frequency index k, ports numbered from 1 as in the file. */
ts_cplx ts_get(const touchstone_t *ts, size_t k, unsigned i, unsigned j);

/* Human-readable reason for the last ts_load failure. */
const char *ts_error(void);

#endif /* TOUCHSTONE_H */
