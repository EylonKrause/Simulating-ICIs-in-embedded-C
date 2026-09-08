/* ===========================================================================
 *  fec.c -- Reed-Solomon RS(544,514) over GF(2^10): the KP4 code, plus an
 *  errors-and-erasures decoder so soft information from the slicer is worth
 *  something.
 *
 *  Everything here is integer. No floating point, no division, no library
 *  calls beyond memcpy -- the field arithmetic is two table lookups and an
 *  add. That is deliberate: this is the one block in the receiver that would
 *  be synthesised as a wide parallel datapath, and it has to be expressible
 *  in that form.
 * =========================================================================*/
#include "fec.h"

#include <string.h>

/* Primitive polynomial of GF(2^10): x^10 + x^3 + 1.
 * Clause 91 uses exactly this field for KP4. */
#define GF_POLY   0x409u

static uint16_t gf_exp[2u * GF_N];        /* doubled so a+b needs no modulo */
static uint16_t gf_log[GF_SIZE];
static uint16_t gf_gen[RS_PARITY + 1u];   /* generator polynomial           */
static int      g_ready;

static fec_stats_t g_st;

/* ---- field arithmetic ---------------------------------------------------- */
static uint16_t gf_mul(uint16_t a, uint16_t b)
{
    if (a == 0u || b == 0u) {
        return 0u;
    }
    return gf_exp[(unsigned)gf_log[a] + (unsigned)gf_log[b]];
}

static uint16_t gf_div(uint16_t a, uint16_t b)
{
    if (a == 0u) {
        return 0u;
    }
    /* b == 0 is a caller bug. The decoder never divides by a zero discrepancy
     * or a zero derivative; both are checked at the call site. */
    return gf_exp[(unsigned)gf_log[a] + GF_N - (unsigned)gf_log[b]];
}

static uint16_t gf_pow_alpha(unsigned e)
{
    return gf_exp[e % GF_N];
}

void fec_init(void)
{
    if (g_ready) {
        return;
    }

    unsigned x = 1u;
    for (unsigned i = 0; i < GF_N; ++i) {
        gf_exp[i] = (uint16_t)x;
        gf_log[x] = (uint16_t)i;
        x <<= 1;
        if ((x & GF_SIZE) != 0u) {
            x ^= GF_POLY;
        }
    }
    for (unsigned i = GF_N; i < 2u * GF_N; ++i) {
        gf_exp[i] = gf_exp[i - GF_N];
    }
    gf_log[0] = 0u;              /* undefined; guarded at every use */

    /* g(x) = prod over i = 1..30 of (x - alpha^i). Monic, so only the 30 low
     * coefficients are stored plus the implicit leading one. */
    memset(gf_gen, 0, sizeof(gf_gen));
    gf_gen[0] = 1u;
    unsigned deg = 0u;
    for (unsigned i = 1u; i <= RS_PARITY; ++i) {
        const uint16_t root = gf_pow_alpha(i);
        deg++;
        for (unsigned k = deg; k > 0u; --k) {
            gf_gen[k] = (uint16_t)(gf_gen[k - 1u] ^ gf_mul(gf_gen[k], root));
        }
        gf_gen[0] = gf_mul(gf_gen[0], root);
    }
    g_ready = 1;
}

/* ---- encode -------------------------------------------------------------- */
/* Systematic: the message is transmitted untouched and the 30 parity symbols
 * are the remainder of m(x)*x^30 divided by g(x). Systematic matters on a
 * link -- a receiver that cannot decode can still hand the payload up with an
 * error flag, and the encoder is a 30-stage LFSR rather than a matrix
 * multiply. */
void fec_encode(const uint16_t *msg, uint16_t *cw)
{
    fec_init();

    uint16_t r[RS_PARITY];
    memset(r, 0, sizeof(r));

    for (unsigned i = 0; i < RS_K; ++i) {
        const uint16_t fb = (uint16_t)(msg[i] ^ r[RS_PARITY - 1u]);
        for (unsigned k = RS_PARITY - 1u; k > 0u; --k) {
            r[k] = (uint16_t)(r[k - 1u] ^ gf_mul(fb, gf_gen[k]));
        }
        r[0] = gf_mul(fb, gf_gen[0]);
    }

    memcpy(cw, msg, RS_K * sizeof(uint16_t));
    for (unsigned k = 0; k < RS_PARITY; ++k) {
        cw[RS_K + k] = r[RS_PARITY - 1u - k];
    }
}

/* ---- syndromes ----------------------------------------------------------- */
/* cw[0] is the coefficient of x^(N-1), so Horner runs forwards over the array.
 * S_i = c(alpha^i) for i = 1..30. All zero means the received word IS a
 * codeword, which is not the same statement as "no errors" -- see the
 * miscorrection note further down. */
static int syndromes(const uint16_t *cw, uint16_t *S)
{
    int nonzero = 0;
    for (unsigned i = 0; i < RS_PARITY; ++i) {
        const uint16_t a = gf_pow_alpha(i + 1u);
        uint16_t s = 0u;
        for (unsigned j = 0; j < RS_N; ++j) {
            s = (uint16_t)(gf_mul(s, a) ^ cw[j]);
        }
        S[i] = s;
        if (s != 0u) {
            nonzero = 1;
        }
    }
    return nonzero;
}

/* ---- decode -------------------------------------------------------------- */
int fec_decode_erasures(uint16_t *cw, const uint8_t *erased,
                        unsigned *n_erasures)
{
    fec_init();

    uint16_t S[RS_PARITY];
    if (n_erasures != NULL) {
        *n_erasures = 0u;
    }
    if (!syndromes(cw, S)) {
        g_st.codewords++;
        return 0;                    /* already a codeword */
    }

    /* ---- erasure locator Gamma(x) = prod (1 - Y_j x) --------------------- */
    /* A symbol the slicer flagged as unreliable costs the code ONE parity
     * symbol instead of two, because its position is already known and only
     * its value has to be solved for. That is the whole of the soft-decision
     * gain here: erasures are half price. */
    unsigned ne = 0u;
    if (erased != NULL) {
        for (unsigned j = 0; j < RS_N; ++j) {
            if (erased[j] != 0u) {
                ne++;
            }
        }
        /* More erasures than parity symbols is not a decodable configuration.
         * Real hardware does the same thing: it checks the flag count and
         * falls back to hard decoding rather than failing outright. */
        if (ne > RS_PARITY) {
            erased = NULL;
            ne = 0u;
        }
    }

    uint16_t gamma[RS_PARITY + 1u];
    memset(gamma, 0, sizeof(gamma));
    gamma[0] = 1u;
    unsigned gdeg = 0u;
    if (erased != NULL && ne > 0u) {
        for (unsigned j = 0; j < RS_N; ++j) {
            if (erased[j] == 0u) {
                continue;
            }
            const uint16_t Y = gf_pow_alpha(RS_N - 1u - j);
            gdeg++;
            for (unsigned k = gdeg; k > 0u; --k) {
                gamma[k] = (uint16_t)(gamma[k] ^ gf_mul(gamma[k - 1u], Y));
            }
        }
    }

    /* ---- modified syndrome T(x) = S(x)*Gamma(x) mod x^30 ----------------- */
    uint16_t T[RS_PARITY];
    for (unsigned i = 0; i < RS_PARITY; ++i) {
        uint16_t v = 0u;
        for (unsigned k = 0; k <= gdeg && k <= i; ++k) {
            v = (uint16_t)(v ^ gf_mul(gamma[k], S[i - k]));
        }
        T[i] = v;
    }

    /* ---- Berlekamp-Massey on the last (30 - ne) modified syndromes ------- */
    /* BM finds the shortest LFSR that generates the syndrome sequence; that
     * LFSR's characteristic polynomial IS the error locator. With ne erasures
     * already accounted for, only 30-ne syndromes remain and the recursion
     * starts at index ne. */
    uint16_t lam[RS_PARITY + 1u];
    uint16_t B[RS_PARITY + 1u];
    uint16_t tmp[RS_PARITY + 1u];
    memset(lam, 0, sizeof(lam));
    memset(B,   0, sizeof(B));
    lam[0] = 1u;
    B[0]   = 1u;
    unsigned L = 0u;
    unsigned m = 1u;
    uint16_t b = 1u;

    for (unsigned n = ne; n < RS_PARITY; ++n) {
        const unsigned it = n - ne;          /* iteration index from zero */
        uint16_t d = T[n];
        for (unsigned i = 1u; i <= L; ++i) {
            d = (uint16_t)(d ^ gf_mul(lam[i], T[n - i]));
        }
        if (d == 0u) {
            m++;
        } else if (2u * L <= it) {
            memcpy(tmp, lam, sizeof(lam));
            const uint16_t coef = gf_div(d, b);
            for (unsigned i = 0; i + m <= RS_PARITY; ++i) {
                lam[i + m] = (uint16_t)(lam[i + m] ^ gf_mul(coef, B[i]));
            }
            L = it + 1u - L;
            memcpy(B, tmp, sizeof(lam));
            b = d;
            m = 1u;
        } else {
            const uint16_t coef = gf_div(d, b);
            for (unsigned i = 0; i + m <= RS_PARITY; ++i) {
                lam[i + m] = (uint16_t)(lam[i + m] ^ gf_mul(coef, B[i]));
            }
            m++;
        }
    }

    /* Budget: 2*errors + erasures must not exceed the 30 parity symbols. */
    if (2u * L + ne > RS_PARITY) {
        g_st.codewords++;
        g_st.uncorrectable++;
        return -1;
    }

    /* ---- errata locator Psi(x) = Gamma(x) * Lambda(x) -------------------- */
    uint16_t psi[2u * RS_PARITY + 2u];
    memset(psi, 0, sizeof(psi));
    for (unsigned i = 0; i <= gdeg; ++i) {
        if (gamma[i] == 0u) {
            continue;
        }
        for (unsigned j = 0; j <= L; ++j) {
            psi[i + j] = (uint16_t)(psi[i + j] ^ gf_mul(gamma[i], lam[j]));
        }
    }
    const unsigned pdeg = gdeg + L;
    if (pdeg > RS_PARITY) {
        g_st.codewords++;
        g_st.uncorrectable++;
        return -1;
    }

    /* ---- Omega(x) = S(x) * Psi(x) mod x^30 ------------------------------- */
    uint16_t omega[RS_PARITY];
    for (unsigned i = 0; i < RS_PARITY; ++i) {
        uint16_t v = 0u;
        for (unsigned k = 0; k <= pdeg && k <= i; ++k) {
            v = (uint16_t)(v ^ gf_mul(psi[k], S[i - k]));
        }
        omega[i] = v;
    }

    /* ---- Chien search ---------------------------------------------------- */
    /* Evaluate Psi at alpha^-p for every codeword position p. On silicon this
     * is 544 evaluations updated by one constant multiply per cycle -- no
     * lookups, no branches. Here it is the obvious loop. */
    unsigned pos[RS_PARITY];
    uint16_t xinv[RS_PARITY];
    unsigned nroots = 0u;

    for (unsigned j = 0; j < RS_N; ++j) {
        const unsigned e   = RS_N - 1u - j;                    /* X = alpha^e */
        const uint16_t xin = gf_pow_alpha(GF_N - (e % GF_N));  /* X^-1        */
        uint16_t v = 0u;
        uint16_t p = 1u;
        for (unsigned k = 0; k <= pdeg; ++k) {
            v = (uint16_t)(v ^ gf_mul(psi[k], p));
            p = gf_mul(p, xin);
        }
        if (v == 0u) {
            if (nroots >= RS_PARITY) {
                g_st.codewords++;
                g_st.uncorrectable++;
                return -1;
            }
            pos[nroots]  = j;
            xinv[nroots] = xin;
            nroots++;
        }
    }

    /* Fewer roots than the locator's degree means the locator has roots
     * outside the positions this shortened code occupies: the received word is
     * closer to some codeword we cannot reach. Declaring that uncorrectable is
     * the correct answer, and far better than silently "correcting" to the
     * wrong codeword. */
    if (nroots != pdeg) {
        g_st.codewords++;
        g_st.uncorrectable++;
        return -1;
    }

    /* ---- Forney ---------------------------------------------------------- */
    /* Psi'(x) over GF(2): even-power terms differentiate to zero, so the
     * derivative is the odd coefficients shifted down by one. One of the small
     * mercies of characteristic two. */
    uint16_t dpsi[2u * RS_PARITY + 2u];
    memset(dpsi, 0, sizeof(dpsi));
    for (unsigned k = 1u; k <= pdeg; k += 2u) {
        dpsi[k - 1u] = psi[k];
    }

    uint16_t saved[RS_PARITY];
    for (unsigned r = 0; r < nroots; ++r) {
        saved[r] = cw[pos[r]];
    }

    int ok = 1;
    unsigned ncorr = 0u;        /* positions whose error value was NON-ZERO */
    for (unsigned r = 0; r < nroots; ++r) {
        const uint16_t xin = xinv[r];

        uint16_t num = 0u;
        uint16_t p   = 1u;
        for (unsigned k = 0; k < RS_PARITY; ++k) {
            num = (uint16_t)(num ^ gf_mul(omega[k], p));
            p = gf_mul(p, xin);
        }
        uint16_t den = 0u;
        p = 1u;
        for (unsigned k = 0; k + 1u <= pdeg; ++k) {
            den = (uint16_t)(den ^ gf_mul(dpsi[k], p));
            p = gf_mul(p, xin);
        }
        if (den == 0u) {
            ok = 0;
            break;
        }
        /* The first root of g is alpha^1, so the X^(1-b) factor is unity. */
        const uint16_t e = gf_div(num, den);
        if (e != 0u) {
            ncorr++;
        }
        cw[pos[r]] = (uint16_t)(cw[pos[r]] ^ e);
    }

    if (ok) {
        uint16_t s2[RS_PARITY];
        if (syndromes(cw, s2)) {
            ok = 0;                 /* correction did not land on a codeword */
        }
    }

    if (!ok) {
        for (unsigned r = 0; r < nroots; ++r) {
            cw[pos[r]] = saved[r];
        }
        g_st.codewords++;
        g_st.uncorrectable++;
        return -1;
    }

    if (n_erasures != NULL) {
        *n_erasures = ne;
    }
    g_st.codewords++;
    /* COUNT CORRECTIONS, NOT ERRATA POSITIONS.
     *
     * `nroots` is gdeg + L: every erasure position plus every located error.
     * An erased symbol that was not actually corrupt gets an error value of
     * zero XORed into it -- no correction happens -- but it was still an
     * errata position, so counting nroots here reported work that was never
     * done. With erasure decoding driven by a soft flag that over-flags (the
     * measured case in fec_probe part 3), the overstatement is exactly the
     * number of false flags, which is precisely the quantity you are trying to
     * measure when you evaluate whether the flags are worth having.
     *
     * nroots is still the right thing for the pdeg consistency check above;
     * it is the wrong thing for a statistic named "corrected". */
    g_st.corrected_symbols += ncorr;
    g_st.erasures_used += ne;
    return (int)ncorr;
}

int fec_decode(uint16_t *cw)
{
    return fec_decode_erasures(cw, NULL, NULL);
}

/* ---- PAM4 packing -------------------------------------------------------- */
/* Five 2-bit values per 10-bit field element, most significant pair first.
 * The mapping from a 2-bit value to a PAM4 LEVEL is Gray and lives in tx.c;
 * this layer only moves bits, which keeps fec.c free of any DSP dependency
 * and unit-testable on its own. */
void fec_pack_pam4(const uint8_t *pam4, uint16_t *gf, size_t n_gf)
{
    for (size_t i = 0; i < n_gf; ++i) {
        uint16_t v = 0u;
        for (unsigned k = 0; k < 5u; ++k) {
            v = (uint16_t)((v << 2) | (pam4[i * 5u + k] & 3u));
        }
        gf[i] = v;
    }
}

void fec_unpack_pam4(const uint16_t *gf, uint8_t *pam4, size_t n_gf)
{
    for (size_t i = 0; i < n_gf; ++i) {
        for (unsigned k = 0; k < 5u; ++k) {
            pam4[i * 5u + k] = (uint8_t)((gf[i] >> (8u - 2u * k)) & 3u);
        }
    }
}

/* ---- statistics ---------------------------------------------------------- */
void fec_stats_reset(void)
{
    memset(&g_st, 0, sizeof(g_st));
}

const fec_stats_t *fec_stats(void)
{
    return &g_st;
}

void fec_stats_add_pre(uint64_t sym_errors, uint64_t bit_errors)
{
    g_st.pre_fec_sym_errors += sym_errors;
    g_st.pre_fec_bit_errors += bit_errors;
}

void fec_stats_add_post(uint64_t sym_errors, uint64_t bit_errors)
{
    g_st.post_fec_sym_errors += sym_errors;
    g_st.post_fec_bit_errors += bit_errors;
}
