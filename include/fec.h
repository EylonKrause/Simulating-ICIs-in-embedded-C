/* ===========================================================================
 *  fec.h -- Reed-Solomon RS(544,514) over GF(2^10), the KP4 code.
 *
 *  This is the FEC every 100G-and-above Ethernet and AI-interconnect lane
 *  runs. It is not optional decoration: PAM4 through 30 dB of channel loss
 *  does not close a 1e-15 link on its own. The raw channel is allowed to run
 *  at a pre-FEC BER around 1e-4 and the code buys the remaining eleven orders
 *  of magnitude.
 *
 *      symbols        544 total, 514 message, 30 parity
 *      field          GF(2^10), x^10 + x^3 + 1
 *      correction     t = 15 symbols, or 30 erasures, or any mix with
 *                     2*errors + erasures <= 30
 *      overhead       544/514 = 5.8%
 *
 *  WHY SYMBOLS AND NOT BITS. A DFE propagates errors: one wrong decision is
 *  fed back, corrupts the next few, and the errors arrive in BURSTS rather
 *  than independently. A symbol-oriented code does not care how many bits
 *  inside a 10-bit symbol are wrong -- one corrupted symbol costs one of the
 *  fifteen corrections whether one bit flipped or all ten. That is exactly the
 *  shape of a DFE-equalised link's error statistics, and it is why RS rather
 *  than a bit-oriented code sits at the end of a SerDes.
 *
 *  Each GF(2^10) symbol carries 10 bits = five PAM4 symbols, so one codeword
 *  is 2720 PAM4 symbols on the wire.
 * =========================================================================*/
#ifndef FEC_H
#define FEC_H

#include <stdint.h>
#include <stddef.h>

#define GF_M          10u
#define GF_SIZE       1024u
#define GF_N          1023u            /* 2^10 - 1, the multiplicative order */

#define RS_N          544u             /* codeword symbols                  */
#define RS_K          514u             /* message symbols                   */
#define RS_PARITY     (RS_N - RS_K)    /* 30                                */
#define RS_T          (RS_PARITY / 2u) /* 15 correctable symbols            */

#define RS_PAM4_PER_GF      5u
#define RS_PAM4_PER_CW      (RS_N * RS_PAM4_PER_GF)   /* 2720 PAM4 symbols */

/* Builds the GF tables and the generator polynomial. Idempotent; every entry
 * point calls it, so no caller has to remember to. */
void fec_init(void);

/* Systematic encode: msg[RS_K] -> cw[RS_N]. The message is copied verbatim
 * into cw[0..513] and the 30 parity symbols land in cw[514..543]. */
void fec_encode(const uint16_t *msg, uint16_t *cw);

/* Hard-decision decode in place. Returns the number of symbols corrected, or
 * -1 if the word carries more than t errors and is UNCORRECTABLE.
 *
 * The -1 case matters. A real link counts uncorrectable codewords separately
 * from corrected ones because the two mean different things: corrections are
 * the code doing its job, uncorrectables are lost frames. On a -1 the buffer
 * is left exactly as received, never half-corrected. */
int fec_decode(uint16_t *cw);

/* Errors-and-erasures decode. `erased` is an RS_N-byte flag array (non-zero =
 * the receiver marked that symbol unreliable) or NULL for the hard-decision
 * path. An erasure costs one parity symbol instead of two, so the decoder can
 * survive 2*errors + erasures <= 30.
 *
 * If more than 30 symbols are flagged the erasure information is discarded and
 * the word is decoded hard -- exactly what a real decoder does when the flag
 * count exceeds the budget. `n_erasures`, if non-NULL, receives the erasure
 * count that was actually used. */
int fec_decode_erasures(uint16_t *cw, const uint8_t *erased,
                        unsigned *n_erasures);

/* Move bits between 2-bit values (five per field element, most significant
 * pair first) and 10-bit GF symbols. The 2-bit value to PAM4 LEVEL mapping is
 * Gray and lives in tx.c; this layer is pure bit movement. */
void fec_pack_pam4(const uint8_t *pam4, uint16_t *gf, size_t n_gf);
void fec_unpack_pam4(const uint16_t *gf, uint8_t *pam4, size_t n_gf);

/* ---- running statistics -------------------------------------------------- */
typedef struct {
    uint64_t codewords;
    uint64_t corrected_symbols;
    uint64_t uncorrectable;        /* codewords beyond the correction budget */
    uint64_t erasures_used;
    uint64_t pre_fec_sym_errors;
    uint64_t pre_fec_bit_errors;
    uint64_t post_fec_sym_errors;
    uint64_t post_fec_bit_errors;
} fec_stats_t;

void               fec_stats_reset(void);
const fec_stats_t *fec_stats(void);
void               fec_stats_add_pre (uint64_t sym_errors, uint64_t bit_errors);
void               fec_stats_add_post(uint64_t sym_errors, uint64_t bit_errors);

#endif /* FEC_H */
