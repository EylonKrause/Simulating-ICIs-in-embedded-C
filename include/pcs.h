/* ===========================================================================
 *  pcs.h -- the layer between the payload and the PAM4 symbols.
 *
 *  On silicon this is the PCS: it takes the traffic the host handed down,
 *  encodes it with the FEC, and presents a symbol stream to the SerDes. On the
 *  way back it collects decisions, decodes, and reports both BERs.
 *
 *  It is a separate module from hw_lane.c on purpose. The PCS runs in the
 *  digital core at a divided rate over a wide bus; the SerDes runs at 100 GBd.
 *  They are different clock domains, different design teams and, in a lot of
 *  products, different dies. Keeping the codeword framing out of the lane
 *  model reflects that.
 *
 *  MEASURING BER. The transmitter's payload comes from a PRBS, so the receiver
 *  can regenerate the identical stream from the same seed and compare against
 *  it. That is precisely what a BER tester does, and it is the only honest way
 *  to get a post-FEC number: post-FEC BER is the decoded MESSAGE against the
 *  transmitted MESSAGE, not the syndrome count.
 * =========================================================================*/
#ifndef PCS_H
#define PCS_H

#include "fec.h"
#include "tx.h"

/* ---- transmit ------------------------------------------------------------ */
typedef struct {
    prbs_t   prbs;
    uint16_t msg[RS_K];
    uint16_t cw[RS_N];
    uint8_t  sym[RS_PAM4_PER_CW];   /* PAM4 LEVEL indices, ready for the DAC */
    unsigned idx;
    uint64_t codewords;
} pcs_tx_t;

void     pcs_tx_init(pcs_tx_t *t, uint32_t seed);
/* Next PAM4 level index (0..3). Encodes a fresh codeword when one runs out. */
unsigned pcs_tx_next(pcs_tx_t *t);

/* ---- receive ------------------------------------------------------------- */
/* PATTERN ALIGNMENT. The receiver's decision for a given symbol comes out of
 * the equaliser some number of symbols after that symbol went in: the FFE has
 * sixteen taps and adaptation is free to place the cursor on any of them, so
 * latency is a property of the CONVERGED filter and is not known in advance.
 *
 * A BER tester handles this the same way, and it is worth saying out loud
 * because it is the step people forget: before it can count a single error it
 * has to LOCK to the pattern. It slides the reference against the received
 * stream, picks the offset with the lowest error count, and only then starts
 * counting. Skip that and you measure the pipeline delay, not the link -- a
 * PAM4 stream compared against a reference one symbol out of step reads
 * roughly 50% BER, which looks exactly like a dead link.
 *
 * 512 symbols is ample: at any BER a working link can have, the correct offset
 * wins by orders of magnitude, so there is no ambiguity to resolve. */
#define PCS_ALIGN_SYMS   512u
#define PCS_ALIGN_MAX     24u        /* widest latency searched, in symbols  */

typedef enum {
    PCS_ALIGNING = 0,   /* buffering, looking for the pattern offset */
    PCS_SKIPPING,       /* offset found, running up to a codeword boundary */
    PCS_RUNNING         /* locked; every symbol from here on is scored */
} pcs_state_t;

typedef struct {
    pcs_tx_t ref;                   /* golden reference, same seed as the TX */
    uint16_t ref_cw[RS_N];          /* the codeword currently in flight      */
    uint8_t  sym[RS_PAM4_PER_CW];   /* received 2-bit Gray words             */
    uint8_t  erased[RS_N];
    unsigned idx;
    unsigned n_flagged;

    pcs_state_t state;
    uint32_t seed;
    uint8_t  align_buf[PCS_ALIGN_SYMS + PCS_ALIGN_MAX];
    unsigned align_n;
    unsigned align_offset;          /* measured equaliser latency, symbols   */
    unsigned align_errors;          /* errors at the winning offset          */
    unsigned skip_left;

    uint64_t bits;
    uint64_t pre_bit_errors;
    uint64_t post_bit_errors;
    uint64_t codewords;
    uint64_t corrected_symbols;
    uint64_t uncorrectable;
    uint64_t erasures_used;
    uint64_t flag_overflows;

    /* Confusion matrix, [transmitted level][decided level]. A BER number says
     * how bad; this says HOW it is bad. Errors spread evenly over the
     * off-diagonal are noise; a single hot cell is a systematic slicing
     * error, which is a completely different bug with a completely different
     * fix. It costs four counters. */
    uint64_t confusion[4][4];
} pcs_rx_t;

void pcs_rx_init(pcs_rx_t *r, uint32_t seed);

/* One decision in. `weak` is side information: non-zero means something else
 * in the receiver already believes this symbol is untrustworthy (a lane that
 * lost lock, a bring-up FSM not yet in TRACK, an error counter over
 * threshold). It is NOT a slicer margin -- see fec_probe part 3 for why
 * margin-based flags do not pay on a hard-decision code. */
void pcs_rx_push(pcs_rx_t *r, unsigned dec_sym, int weak);

double pcs_pre_fec_ber (const pcs_rx_t *r);
double pcs_post_fec_ber(const pcs_rx_t *r);

#endif /* PCS_H */
