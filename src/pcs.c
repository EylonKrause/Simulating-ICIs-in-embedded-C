#include "pcs.h"

#include <string.h>

/* ---- transmit ------------------------------------------------------------ */
static void pcs_tx_new_codeword(pcs_tx_t *t)
{
    for (unsigned i = 0; i < RS_K; ++i) {
        t->msg[i] = (uint16_t)prbs_bits(&t->prbs, GF_M);
    }
    fec_encode(t->msg, t->cw);

    /* Codeword symbols -> 2-bit values -> PAM4 level indices. The Gray step
     * is what makes a one-level slicing mistake cost one bit instead of two,
     * which is worth roughly 3 dB of pre-FEC BER for free. */
    uint8_t bits2[RS_PAM4_PER_CW];
    fec_unpack_pam4(t->cw, bits2, RS_N);
    for (unsigned i = 0; i < RS_PAM4_PER_CW; ++i) {
        t->sym[i] = (uint8_t)pam4_sym_from_gray(bits2[i]);
    }

    t->idx = 0u;
    t->codewords++;
}

void pcs_tx_init(pcs_tx_t *t, uint32_t seed)
{
    memset(t, 0, sizeof(*t));
    prbs_init(&t->prbs, seed);
    pcs_tx_new_codeword(t);
}

unsigned pcs_tx_next(pcs_tx_t *t)
{
    if (t->idx >= RS_PAM4_PER_CW) {
        pcs_tx_new_codeword(t);
    }
    return t->sym[t->idx++];
}

/* ---- receive ------------------------------------------------------------- */
void pcs_rx_init(pcs_rx_t *r, uint32_t seed)
{
    memset(r, 0, sizeof(*r));
    r->seed  = seed;
    r->state = PCS_ALIGNING;
    pcs_tx_init(&r->ref, seed);
}

/* Slide the reference against the buffered decisions and take the offset with
 * the fewest bit errors. Ties cannot happen in practice: a wrong offset scores
 * near 50% and the right one scores near zero. */
static void pcs_rx_align(pcs_rx_t *r)
{
    static uint8_t ref_sym[PCS_ALIGN_SYMS];

    pcs_tx_init(&r->ref, r->seed);
    for (unsigned i = 0; i < PCS_ALIGN_SYMS; ++i) {
        ref_sym[i] = (uint8_t)pcs_tx_next(&r->ref);
    }

    unsigned best_off = 0u;
    unsigned best_err = 0xFFFFFFFFu;
    for (unsigned s = 0; s <= PCS_ALIGN_MAX; ++s) {
        unsigned err = 0u;
        for (unsigned i = 0; i < PCS_ALIGN_SYMS; ++i) {
            err += pam4_bit_errors(r->align_buf[i + s], ref_sym[i]);
        }
        if (err < best_err) {
            best_err = err;
            best_off = s;
        }
    }
    r->align_offset = best_off;
    r->align_errors = best_err;

    /* The next decision to arrive carries transmitted symbol number
     * (buffered - offset). Run the reference forward to the first codeword
     * boundary at or after that point and drop the decisions in between, so
     * scoring starts on a framed codeword. Codeword lock in a real PCS comes
     * from alignment markers in the stream; here the framing is deterministic
     * once the symbol offset is known. */
    const unsigned base    = r->align_n - best_off;
    const unsigned n_cw    = (base + RS_PAM4_PER_CW - 1u) / RS_PAM4_PER_CW;
    const unsigned advance = n_cw * RS_PAM4_PER_CW;

    pcs_tx_init(&r->ref, r->seed);
    for (unsigned i = 0; i < advance; ++i) {
        (void)pcs_tx_next(&r->ref);
    }

    r->skip_left = advance - base;
    r->idx       = 0u;
    r->n_flagged = 0u;
    memset(r->erased, 0, sizeof(r->erased));
    r->state = (r->skip_left > 0u) ? PCS_SKIPPING : PCS_RUNNING;
}

static void pcs_rx_finish_codeword(pcs_rx_t *r)
{
    uint16_t rx_cw[RS_N];
    fec_pack_pam4(r->sym, rx_cw, RS_N);

    if (r->n_flagged > RS_PARITY) {
        r->flag_overflows++;
    }

    unsigned used = 0u;
    const int n = fec_decode_erasures(rx_cw, (r->n_flagged > 0u) ? r->erased : NULL,
                                      &used);
    if (n < 0) {
        r->uncorrectable++;
    } else {
        r->corrected_symbols += (uint64_t)n;
        r->erasures_used     += used;
    }

    /* Post-FEC errors are counted over the MESSAGE only. The parity symbols
     * are consumed by the decoder and never delivered upward, so residual
     * errors in them are not errors the link made. */
    for (unsigned i = 0; i < RS_K; ++i) {
        uint16_t d = (uint16_t)(rx_cw[i] ^ r->ref_cw[i]);
        while (d != 0u) {
            r->post_bit_errors += (uint64_t)(d & 1u);
            d = (uint16_t)(d >> 1);
        }
    }

    r->codewords++;
    r->idx = 0u;
    r->n_flagged = 0u;
    memset(r->erased, 0, sizeof(r->erased));
}

void pcs_rx_push(pcs_rx_t *r, unsigned dec_sym, int weak)
{
    if (r->state == PCS_ALIGNING) {
        r->align_buf[r->align_n++] = (uint8_t)(dec_sym & 3u);
        if (r->align_n >= PCS_ALIGN_SYMS + PCS_ALIGN_MAX) {
            pcs_rx_align(r);
        }
        return;
    }
    if (r->state == PCS_SKIPPING) {
        if (--r->skip_left == 0u) {
            r->state = PCS_RUNNING;
        }
        return;
    }

    if (r->idx == 0u) {
        /* Pull the reference symbol first so ref.cw holds the codeword that is
         * about to arrive, then snapshot it. The reference generator runs in
         * lockstep with the received stream; there is no symbol-slip model
         * here, which is the one assumption this measurement rests on. */
        const unsigned expect0 = pcs_tx_next(&r->ref);
        memcpy(r->ref_cw, r->ref.cw, sizeof(r->ref_cw));

        r->pre_bit_errors += pam4_bit_errors(dec_sym, expect0);
        r->confusion[expect0 & 3u][dec_sym & 3u]++;
        r->bits += BITS_PER_SYMBOL;

        unsigned b1, b0;
        pam4_bits(dec_sym, &b1, &b0);
        r->sym[0] = (uint8_t)((b1 << 1) | b0);
        if (weak) {
            r->erased[0] = 1u;
            r->n_flagged++;
        }
        r->idx = 1u;
        return;
    }

    const unsigned expect = pcs_tx_next(&r->ref);
    r->pre_bit_errors += pam4_bit_errors(dec_sym, expect);
    r->confusion[expect & 3u][dec_sym & 3u]++;
    r->bits += BITS_PER_SYMBOL;

    unsigned b1, b0;
    pam4_bits(dec_sym, &b1, &b0);
    r->sym[r->idx] = (uint8_t)((b1 << 1) | b0);

    if (weak) {
        const unsigned g = r->idx / RS_PAM4_PER_GF;
        if (r->erased[g] == 0u) {
            r->erased[g] = 1u;
            r->n_flagged++;
        }
    }

    r->idx++;
    if (r->idx >= RS_PAM4_PER_CW) {
        pcs_rx_finish_codeword(r);
    }
}

double pcs_pre_fec_ber(const pcs_rx_t *r)
{
    return (r->bits == 0u) ? 0.0
                           : (double)r->pre_bit_errors / (double)r->bits;
}

double pcs_post_fec_ber(const pcs_rx_t *r)
{
    const uint64_t msg_bits = r->codewords * (uint64_t)RS_K * GF_M;
    return (msg_bits == 0u) ? 0.0
                            : (double)r->post_bit_errors / (double)msg_bits;
}
