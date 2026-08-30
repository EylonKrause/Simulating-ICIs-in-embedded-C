#include "tx.h"

#include <math.h>
#include <string.h>

/* ---- PRBS31 -------------------------------------------------------------- */
/* x^31 + x^28 + 1. Period 2^31-1, so it never repeats inside any simulation
 * we run, and it contains long runs of identical symbols -- which is the point:
 * runs are what stress the DC wander and the CDR's transition density. */
void prbs_init(prbs_t *p, uint32_t seed)
{
    p->state = (seed == 0u) ? 0x5EEDu : (seed & 0x7FFFFFFFu);
    if (p->state == 0u) {
        p->state = 1u;
    }
}

uint32_t prbs_bit(prbs_t *p)
{
    const uint32_t fb = ((p->state >> 30) ^ (p->state >> 27)) & 1u;
    p->state = ((p->state << 1) | fb) & 0x7FFFFFFFu;
    return fb;
}

uint32_t prbs_bits(prbs_t *p, unsigned n)
{
    uint32_t v = 0u;
    for (unsigned i = 0; i < n; ++i) {
        v = (v << 1) | prbs_bit(p);
    }
    return v;
}

/* ---- PAM4 Gray ----------------------------------------------------------- */
/* index = Gray symbol code, value = level. Ordered so that adjacent LEVELS
 * differ in one bit: 00 -> -3, 01 -> -1, 11 -> +1, 10 -> +3. */
static const real_t PAM4_LEVEL[4] = { -1.0, -1.0 / 3.0, +1.0 / 3.0, +1.0 };
/* level order index -> the 2-bit Gray word carried there */
static const unsigned PAM4_GRAY[4] = { 0u, 1u, 3u, 2u };

real_t pam4_level(unsigned gray_sym)
{
    return PAM4_LEVEL[gray_sym & 3u];
}

void pam4_bits(unsigned gray_sym, unsigned *b1, unsigned *b0)
{
    const unsigned g = PAM4_GRAY[gray_sym & 3u];
    *b1 = (g >> 1) & 1u;
    *b0 = g & 1u;
}

unsigned pam4_slice(real_t y)
{
    /* Decision thresholds midway between levels: -2/3, 0, +2/3. */
    if (y < -2.0 / 3.0) { return 0u; }
    if (y <  0.0)       { return 1u; }
    if (y <  2.0 / 3.0) { return 2u; }
    return 3u;
}

unsigned pam4_bit_errors(unsigned a, unsigned b)
{
    const unsigned ga = PAM4_GRAY[a & 3u];
    const unsigned gb = PAM4_GRAY[b & 3u];
    const unsigned d  = ga ^ gb;
    return ((d >> 1) & 1u) + (d & 1u);
}

/* ---- TX FFE -------------------------------------------------------------- */
real_t tx_ffe_set(tx_ffe_t *f, real_t pre, real_t post)
{
    memset(f, 0, sizeof(*f));
    f->n = TX_FFE_TAPS;
    f->c[0] = pre;          /* precursor tap, normally negative  */
    f->c[1] = 1.0;          /* cursor, scaled by the L1 norm next */
    f->c[2] = post;         /* postcursor tap, normally negative  */

    real_t l1 = 0.0;
    for (unsigned i = 0; i < TX_FFE_TAPS; ++i) {
        l1 += fabs(f->c[i]);
    }
    if (l1 <= 0.0) {
        l1 = 1.0;
    }
    for (unsigned i = 0; i < TX_FFE_TAPS; ++i) {
        f->c[i] /= l1;      /* now sum|c| == 1 : the driver cannot clip */
    }
    return f->c[TX_FFE_CURSOR];
}

real_t tx_ffe_step(tx_ffe_t *f, real_t sym)
{
    for (unsigned i = TX_FFE_TAPS - 1u; i > 0u; --i) {
        f->hist[i] = f->hist[i - 1u];
    }
    f->hist[0] = sym;

    real_t acc = 0.0;
    for (unsigned i = 0; i < TX_FFE_TAPS; ++i) {
        acc += f->c[i] * f->hist[i];
    }
    return acc;
}

/* ---- transmitter --------------------------------------------------------- */
void tx_init(tx_t *tx, uint32_t seed, real_t pre, real_t post)
{
    prbs_init(&tx->prbs, seed);
    (void)tx_ffe_set(&tx->ffe, pre, post);
}

real_t tx_step(tx_t *tx, unsigned *sym_out)
{
    const unsigned sym = (unsigned)prbs_bits(&tx->prbs, 2u) & 3u;
    if (sym_out != NULL) {
        *sym_out = sym;
    }
    return tx_ffe_step(&tx->ffe, pam4_level(sym));
}

void tx_upsample(real_t level, real_t *dst)
{
    /* Zero-order hold over one UI. A real driver has finite rise time; that
     * is folded into the channel's low-pass response rather than modelled
     * separately here. */
    for (unsigned i = 0; i < OSR; ++i) {
        dst[i] = level;
    }
}
