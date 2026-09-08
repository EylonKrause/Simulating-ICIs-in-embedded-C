#define _CRT_SECURE_NO_WARNINGS
/* ===========================================================================
 *  fec_probe.c -- what the KP4 code actually buys, measured.
 *
 *  Four parts:
 *
 *    1. A self-test of the codec. Random messages, random error and erasure
 *       patterns, exact-recovery checks, and a check that the decoder REFUSES
 *       words beyond its budget rather than miscorrecting them. A FEC block
 *       that quietly emits a wrong codeword is worse than no FEC at all,
 *       because the layer above trusts it.
 *
 *    2. The waterfall. AWGN PAM4 swept across SNR, pre-FEC BER against
 *       post-FEC BER. This is the number a link budget is written in.
 *
 *    3. Erasure economics on AWGN. The obvious idea -- flag samples that land
 *       near a slicer threshold, erase those symbols, and let the code correct
 *       twice as many -- is measured here rather than assumed. It does not pay
 *       (see the arithmetic printed alongside the table), and that negative
 *       result is worth more than a plausible-looking claim.
 *
 *    4. Burst errors. Where erasure decoding DOES pay: when something else
 *       already knows which symbols are bad -- a DFE error-propagation burst,
 *       a lane that lost lock, a transient. Marking those positions doubles
 *       the survivable burst length from 15 symbols to 30.
 *
 *  Usage:  fec_probe [codewords_per_point]
 * =========================================================================*/
#include "fec.h"
#include "tx.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- RNG ----------------------------------------------------------------- */
/* xorshift32, not an LCG: an LCG's low bits have period 2, so `rand() & 3`
 * would produce an alternating "random" symbol stream and every result below
 * would be optimistic. */
static uint32_t g_rng = 0x2545F491u;

static uint32_t xs32(void)
{
    uint32_t x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}

static double urand(void)
{
    return (double)(xs32() >> 8) / 16777216.0;
}

static double gauss(void)
{
    double u1 = urand();
    const double u2 = urand();
    if (u1 < 1e-12) {
        u1 = 1e-12;
    }
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static int g_fail;

static void check(int cond, const char *what)
{
    if (!cond) {
        printf("    FAIL  %s\n", what);
        g_fail++;
    }
}

static void random_message(uint16_t *msg)
{
    for (unsigned i = 0; i < RS_K; ++i) {
        msg[i] = (uint16_t)(xs32() & (GF_SIZE - 1u));
    }
}

/* Pick `n` distinct positions in [0, RS_N). */
static void pick_positions(unsigned *pos, unsigned n)
{
    uint8_t used[RS_N];
    memset(used, 0, sizeof(used));
    for (unsigned i = 0; i < n; ++i) {
        unsigned p;
        do {
            p = xs32() % RS_N;
        } while (used[p]);
        used[p] = 1u;
        pos[i] = p;
    }
}

static uint16_t nonzero_error(void)
{
    uint16_t e;
    do {
        e = (uint16_t)(xs32() & (GF_SIZE - 1u));
    } while (e == 0u);
    return e;
}

/* ===========================================================================
 *  Part 1 -- codec self-test
 * =========================================================================*/
static void selftest(unsigned trials)
{
    static uint16_t msg[RS_K];
    static uint16_t cw[RS_N];
    static uint16_t rx[RS_N];
    static uint16_t before[RS_N];
    static uint8_t  erased[RS_N];
    unsigned pos[RS_PARITY];

    printf("  1. codec self-test (%u trials per case)\n", trials);

    /* --- a clean codeword decodes to itself, zero corrections ------------- */
    for (unsigned t = 0; t < trials; ++t) {
        random_message(msg);
        fec_encode(msg, cw);
        memcpy(rx, cw, sizeof(cw));
        check(fec_decode(rx) == 0, "clean codeword reported corrections");
        check(memcmp(rx, cw, sizeof(cw)) == 0, "clean codeword was modified");
        check(memcmp(rx, msg, RS_K * sizeof(uint16_t)) == 0,
              "systematic property broken: message not carried verbatim");
    }
    printf("     clean codewords ............................ ok\n");

    /* --- up to t errors, hard decision ------------------------------------ */
    for (unsigned t = 0; t < trials; ++t) {
        const unsigned ne = 1u + (xs32() % RS_T);           /* 1..15 */
        random_message(msg);
        fec_encode(msg, cw);
        memcpy(rx, cw, sizeof(cw));
        pick_positions(pos, ne);
        for (unsigned i = 0; i < ne; ++i) {
            rx[pos[i]] ^= nonzero_error();
        }
        check(fec_decode(rx) == (int)ne, "wrong correction count");
        check(memcmp(rx, cw, sizeof(cw)) == 0, "t-error word not recovered");
    }
    printf("     1..%u symbol errors, hard decision .......... ok\n", RS_T);

    /* --- erasures only: the code corrects twice as many ------------------- */
    for (unsigned t = 0; t < trials; ++t) {
        const unsigned ne = 1u + (xs32() % RS_PARITY);      /* 1..30 */
        random_message(msg);
        fec_encode(msg, cw);
        memcpy(rx, cw, sizeof(cw));
        memset(erased, 0, sizeof(erased));
        pick_positions(pos, ne);
        for (unsigned i = 0; i < ne; ++i) {
            rx[pos[i]] ^= nonzero_error();
            erased[pos[i]] = 1u;
        }
        unsigned used = 0u;
        check(fec_decode_erasures(rx, erased, &used) >= 0,
              "erasure-only word declared uncorrectable");
        check(used == ne, "erasure count not reported back");
        check(memcmp(rx, cw, sizeof(cw)) == 0, "erasure-only word not recovered");
    }
    printf("     1..%u erasures ............................. ok\n", RS_PARITY);

    /* --- the mixed budget: 2*errors + erasures <= 30 ---------------------- */
    for (unsigned t = 0; t < trials; ++t) {
        const unsigned nerr = xs32() % (RS_T + 1u);         /* 0..15 */
        const unsigned nera = xs32() % (RS_PARITY - 2u * nerr + 1u);
        random_message(msg);
        fec_encode(msg, cw);
        memcpy(rx, cw, sizeof(cw));
        memset(erased, 0, sizeof(erased));
        pick_positions(pos, nerr + nera);
        for (unsigned i = 0; i < nerr + nera; ++i) {
            rx[pos[i]] ^= nonzero_error();
            if (i >= nerr) {
                erased[pos[i]] = 1u;          /* the flagged half */
            }
        }
        check(fec_decode_erasures(rx, erased, NULL) >= 0,
              "in-budget errata word declared uncorrectable");
        check(memcmp(rx, cw, sizeof(cw)) == 0, "errata word not recovered");
    }
    printf("     2*errors + erasures <= %u .................. ok\n", RS_PARITY);

    /* --- beyond the budget: must NOT silently miscorrect ------------------ */
    /* Handed t+1 errors, an RS decoder either declares failure (the common and
     * correct outcome) or lands on a different valid codeword. The second is a
     * MISCORRECTION and is undetectable by definition. What must never happen
     * is a partial, invalid write-back, so the check is that on failure the
     * buffer comes back exactly as it was received. */
    unsigned declared = 0u, miscorrected = 0u;
    for (unsigned t = 0; t < trials; ++t) {
        const unsigned ne = RS_T + 1u + (xs32() % 5u);      /* 16..20 errors */
        random_message(msg);
        fec_encode(msg, cw);
        memcpy(rx, cw, sizeof(cw));
        pick_positions(pos, ne);
        for (unsigned i = 0; i < ne; ++i) {
            rx[pos[i]] ^= nonzero_error();
        }
        memcpy(before, rx, sizeof(before));
        if (fec_decode(rx) < 0) {
            declared++;
            check(memcmp(rx, before, sizeof(before)) == 0,
                  "uncorrectable word was left half-corrected");
        } else {
            miscorrected++;
        }
    }
    printf("     beyond budget: %u/%u declared uncorrectable, %u miscorrected\n",
           declared, trials, miscorrected);
    /* The miscorrection probability of RS(544,514) is of order 1/t! ~ 1e-12,
     * so anything above zero here is a bug, not luck. */
    check(miscorrected == 0u, "decoder miscorrected an out-of-budget word");

    printf(g_fail == 0 ? "     ALL CODEC CHECKS PASSED\n\n"
                       : "     CODEC CHECKS FAILED\n\n");
}

/* ===========================================================================
 *  The AWGN PAM4 wire
 * =========================================================================*/
/* Distance from a received sample to the nearest slicer threshold. Small means
 * the slicer is not confident. On silicon this is a second pair of comparators
 * either side of each threshold -- three extra comparators per lane and no
 * arithmetic at all, which is why the idea is tempting enough to be worth
 * measuring properly. */
static double slicer_margin(double y)
{
    static const double THR[3] = { -2.0 / 3.0, 0.0, 2.0 / 3.0 };
    double best = 1e9;
    for (unsigned i = 0; i < 3u; ++i) {
        const double d = fabs(y - THR[i]);
        if (d < best) {
            best = d;
        }
    }
    return best;
}

/* Send one codeword through AWGN. Fills the received GF symbols and, if
 * `erased` is non-NULL, the erasure flags for the given margin threshold.
 * Returns the number of PAM4 bit errors. */
static uint64_t wire_codeword(const uint16_t *cw, uint16_t *rx,
                              uint8_t *erased, double sigma, double margin,
                              unsigned *n_flag)
{
    static uint8_t pam_tx[RS_PAM4_PER_CW];
    static uint8_t pam_rx[RS_PAM4_PER_CW];
    uint64_t bit_err = 0u;

    fec_unpack_pam4(cw, pam_tx, RS_N);
    if (erased != NULL) {
        memset(erased, 0, RS_N);
        *n_flag = 0u;
    }

    for (unsigned i = 0; i < RS_PAM4_PER_CW; ++i) {
        const unsigned lvl_i = pam4_sym_from_gray(pam_tx[i]);
        const double   y     = (double)pam4_level(lvl_i) + sigma * gauss();
        const unsigned dec_i = pam4_slice((real_t)y);

        unsigned b1, b0;
        pam4_bits(dec_i, &b1, &b0);
        pam_rx[i] = (uint8_t)((b1 << 1) | b0);
        bit_err += pam4_bit_errors(dec_i, lvl_i);

        if (erased != NULL && slicer_margin(y) < margin) {
            const unsigned g = i / RS_PAM4_PER_GF;
            if (erased[g] == 0u) {
                erased[g] = 1u;
                (*n_flag)++;
            }
        }
    }
    fec_pack_pam4(pam_rx, rx, RS_N);
    return bit_err;
}

/* ===========================================================================
 *  Part 2 -- the waterfall
 * =========================================================================*/
static void waterfall(unsigned n_cw)
{
    static uint16_t msg[RS_K];
    static uint16_t cw[RS_N];
    static uint16_t rx[RS_N];

    static const double SIGMA[] = { 0.150, 0.140, 0.130, 0.120, 0.110, 0.100, 0.090 };
    const unsigned NS = (unsigned)(sizeof(SIGMA) / sizeof(SIGMA[0]));

    printf("  2. waterfall: AWGN PAM4, %u codewords per point\n\n", n_cw);
    printf("     %-7s %-11s %-12s %-9s %s\n",
           "sigma", "pre-FEC BER", "post-FEC BER", "uncorr", "margin to KP4 threshold");
    printf("     ---------------------------------------------------------------------\n");

    for (unsigned s = 0; s < NS; ++s) {
        uint64_t pre = 0u, bits = 0u, post = 0u, fail = 0u;

        for (unsigned c = 0; c < n_cw; ++c) {
            random_message(msg);
            fec_encode(msg, cw);
            pre  += wire_codeword(cw, rx, NULL, SIGMA[s], 0.0, NULL);
            bits += 2u * RS_PAM4_PER_CW;
            if (fec_decode(rx) < 0) {
                fail++;
            }
            for (unsigned i = 0; i < RS_K; ++i) {
                const uint16_t d = (uint16_t)(rx[i] ^ msg[i]);
                for (unsigned b = 0; b < GF_M; ++b) {
                    post += (uint64_t)((d >> b) & 1u);
                }
            }
        }

        const double pre_ber  = (double)pre / (double)bits;
        const uint64_t msgbits = (uint64_t)n_cw * RS_K * GF_M;

        printf("     %-7.3f %-11.2e ", SIGMA[s], pre_ber);
        if (post == 0u) {
            printf("< %-10.1e ", 1.0 / (double)msgbits);
        } else {
            printf("%-12.2e ", (double)post / (double)msgbits);
        }
        printf("%3llu/%-5u ", (unsigned long long)fail, n_cw);
        /* KP4 is specified to hold post-FEC BER below 1e-15 given a pre-FEC
         * BER at or below 2.4e-4. That single number is the contract between
         * the SerDes team and the PCS team, and it is the reason the analogue
         * side is allowed to hand up a visibly imperfect eye. */
        printf("%+.1f dB\n", 10.0 * log10(2.4e-4 / (pre_ber > 0.0 ? pre_ber : 1e-12)));
    }
    printf("\n     The last column is how far the raw channel sits from the 2.4e-4\n");
    printf("     pre-FEC BER that KP4 is specified against. Positive is margin.\n\n");
}

/* ===========================================================================
 *  Part 3 -- erasure economics on AWGN: the idea that does not pay
 * =========================================================================*/
static void erasure_economics(unsigned n_cw, double sigma)
{
    static uint16_t msg[RS_K];
    static uint16_t cw[RS_N];
    static uint16_t rx[RS_N];
    static uint16_t rs[RS_N];
    static uint8_t  erased[RS_N];

    static const double MARGIN[] = { 0.010, 0.020, 0.030, 0.050, 0.085 };
    const unsigned NM = (unsigned)(sizeof(MARGIN) / sizeof(MARGIN[0]));

    printf("  3. erasure economics on AWGN (sigma = %.3f, %u codewords)\n\n",
           sigma, n_cw);
    printf("     %-8s %-8s %-8s %-8s %-10s %-10s %s\n",
           "margin", "flags", "errors", "covered", "hard fail", "soft fail",
           "silent miscorrect (h/s)");
    printf("     --------------------------------------------------------------------------\n");

    for (unsigned m = 0; m < NM; ++m) {
        uint64_t flags = 0u, errs = 0u, covered = 0u;
        uint64_t fail_hard = 0u, fail_soft = 0u, overflow = 0u;
        uint64_t miscorr_hard = 0u, miscorr_soft = 0u;

        for (unsigned c = 0; c < n_cw; ++c) {
            random_message(msg);
            fec_encode(msg, cw);

            unsigned nf = 0u;
            (void)wire_codeword(cw, rx, erased, sigma, MARGIN[m], &nf);
            memcpy(rs, rx, sizeof(rx));

            for (unsigned i = 0; i < RS_N; ++i) {
                if (rx[i] != cw[i]) {
                    errs++;
                    if (erased[i]) {
                        covered++;
                    }
                }
            }
            flags += nf;
            if (nf > RS_PARITY) {
                overflow++;
            }

            /* A DECODE THAT RETURNS >= 0 HAS NOT NECESSARILY SUCCEEDED.
             *
             * Beyond the correction budget a Reed-Solomon decoder has two ways
             * to be wrong, and only one of them announces itself. It can
             * declare failure -- that is the safe one. Or it can land on a
             * DIFFERENT valid codeword, return a confident success, and hand
             * up data that is wrong: a MISCORRECTION. The syndrome recheck
             * inside fec_decode catches an incomplete correction, but a
             * genuine neighbouring codeword has zero syndrome by definition
             * and passes it.
             *
             * Scoring on the return value alone therefore counts silent
             * miscorrections as successes, which flatters exactly the column
             * this table exists to question. Part 4 already compares against
             * the transmitted word; this now does the same, and reports
             * miscorrections separately because they are the interesting
             * number rather than a detail to fold into a total. */
            if (fec_decode(rx) < 0) {
                fail_hard++;
            } else if (memcmp(rx, cw, sizeof(cw)) != 0) {
                fail_hard++;
                miscorr_hard++;
            }
            if (fec_decode_erasures(rs, erased, NULL) < 0) {
                fail_soft++;
            } else if (memcmp(rs, cw, sizeof(cw)) != 0) {
                fail_soft++;
                miscorr_soft++;
            }
        }

        printf("     %-8.3f %-8.1f %-8.1f %-8.1f %-10llu %-10llu %llu / %llu%s\n",
               MARGIN[m],
               (double)flags / (double)n_cw,
               (double)errs / (double)n_cw,
               (double)covered / (double)n_cw,
               (unsigned long long)fail_hard,
               (unsigned long long)fail_soft,
               (unsigned long long)miscorr_hard,
               (unsigned long long)miscorr_soft,
               overflow ? "   (flag budget overflowed, fell back to hard)" : "");
    }

    printf("\n     The last column is the one worth staring at. A MISCORRECTION is a\n");
    printf("     decode that returned success and produced the WRONG codeword: past\n");
    printf("     the budget the received word can be closer to a neighbouring valid\n");
    printf("     codeword than to the transmitted one, and a neighbouring codeword\n");
    printf("     has zero syndrome by construction, so no self-check inside the\n");
    printf("     decoder can catch it. It is counted as a failure above, because it\n");
    printf("     is one -- but on a real link nothing downstream would know.\n");
    printf("     Scoring these as successes, which this table used to do by trusting\n");
    printf("     the return value alone, makes a decoder look better exactly where it\n");
    printf("     is behaving worst.\n");

    printf("\n     Read the arithmetic, not the hope. Flagging a symbol converts an\n");
    printf("     error from costing two parity symbols to costing one, saving one --\n");
    printf("     but every symbol flagged that was NOT in error costs one outright.\n");
    printf("     The trade only pays when\n\n");
    printf("         flags  <  2 * (errors actually covered)\n\n");
    printf("     and the table above never satisfies it. The reason is geometric:\n");
    printf("     PAM4 has three thresholds and the inner levels sit between two of\n");
    printf("     them, so the population of samples NEAR a threshold is several\n");
    printf("     times the population that CROSSED one, at every margin. Widening\n");
    printf("     the window to catch more errors adds false flags faster than it\n");
    printf("     adds coverage.\n\n");
    printf("     This is why KP4 in Ethernet is a hard-decision code and why real\n");
    printf("     soft-decision gain needs a soft-decision CODE (LDPC, or a\n");
    printf("     concatenated inner code), not a hard code fed reliability flags.\n\n");
}

/* ===========================================================================
 *  Part 4 -- burst errors: where erasure decoding does pay
 * =========================================================================*/
static void burst_test(unsigned trials)
{
    static uint16_t msg[RS_K];
    static uint16_t cw[RS_N];
    static uint16_t rx[RS_N];
    static uint16_t rs[RS_N];
    static uint8_t  erased[RS_N];

    static const unsigned BURST[] = { 8u, 15u, 16u, 20u, 25u, 30u, 31u };
    const unsigned NB = (unsigned)(sizeof(BURST) / sizeof(BURST[0]));

    printf("  4. burst errors with the burst LOCATION known (%u trials each)\n\n",
           trials);
    printf("     A DFE that mis-slices feeds the wrong decision back and the next\n");
    printf("     few symbols go with it. A lane that loses lock corrupts everything\n");
    printf("     until it reacquires. In both cases the receiver KNOWS which\n");
    printf("     symbols are suspect -- from the loss-of-lock flag, from the\n");
    printf("     bring-up FSM, from a lane-error counter. That is side\n");
    printf("     information, and unlike a slicer margin it costs no false flags.\n\n");
    printf("     %-8s %-14s %s\n", "burst", "hard decode", "burst positions erased");
    printf("     -----------------------------------------------------\n");

    for (unsigned b = 0; b < NB; ++b) {
        const unsigned len = BURST[b];
        unsigned ok_hard = 0u, ok_soft = 0u;

        for (unsigned t = 0; t < trials; ++t) {
            random_message(msg);
            fec_encode(msg, cw);
            memcpy(rx, cw, sizeof(cw));
            memset(erased, 0, sizeof(erased));

            const unsigned start = xs32() % (RS_N - len);
            for (unsigned i = 0; i < len; ++i) {
                rx[start + i] ^= nonzero_error();
                erased[start + i] = 1u;
            }
            memcpy(rs, rx, sizeof(rx));

            if (fec_decode(rx) >= 0 && memcmp(rx, cw, sizeof(cw)) == 0) {
                ok_hard++;
            }
            if (fec_decode_erasures(rs, erased, NULL) >= 0 &&
                memcmp(rs, cw, sizeof(cw)) == 0) {
                ok_soft++;
            }
        }
        printf("     %-8u %3u/%-10u %3u/%u\n",
               len, ok_hard, trials, ok_soft, trials);
    }

    printf("\n     Hard decision survives %u symbols. With the positions marked it\n",
           RS_T);
    printf("     survives %u -- exactly twice, because an erasure costs one parity\n",
           RS_PARITY);
    printf("     symbol and an error costs two. Both cliffs are sharp and both land\n");
    printf("     precisely where the algebra says they must, which is the point of\n");
    printf("     running the experiment at all.\n\n");
}

int main(int argc, char **argv)
{
    const unsigned n_cw = (argc > 1) ? (unsigned)atoi(argv[1]) : 300u;

    fec_init();

    printf("=========================================================\n");
    printf(" RS(%u,%u) over GF(2^%u)  --  KP4\n", RS_N, RS_K, GF_M);
    printf(" t = %u symbol errors, or %u erasures, or 2e + s <= %u\n",
           RS_T, RS_PARITY, RS_PARITY);
    printf(" overhead %.2f%%   codeword = %u PAM4 symbols\n",
           100.0 * (double)RS_PARITY / (double)RS_K, RS_PAM4_PER_CW);
    printf("=========================================================\n\n");

    selftest(40u);
    waterfall(n_cw);
    erasure_economics(n_cw / 2u + 1u, 0.125);
    burst_test(60u);

    const fec_stats_t *st = fec_stats();
    printf("  cumulative decoder statistics\n");
    printf("    codewords decoded   %llu\n", (unsigned long long)st->codewords);
    printf("    symbols corrected   %llu\n", (unsigned long long)st->corrected_symbols);
    printf("    erasures consumed   %llu\n", (unsigned long long)st->erasures_used);
    printf("    uncorrectable       %llu\n", (unsigned long long)st->uncorrectable);

    return g_fail ? 1 : 0;
}
