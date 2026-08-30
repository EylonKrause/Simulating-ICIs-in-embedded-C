/* ===========================================================================
 *  tx.h — transmitter: PRBS source, Gray-coded PAM4 mapper, TX FFE.
 *
 *  PAM4 GRAY CODING is not cosmetic. The four levels are ordered so adjacent
 *  levels differ in exactly one bit:
 *
 *      bits 00 -> -3     bits 01 -> -1     bits 11 -> +1     bits 10 -> +3
 *
 *  Almost every symbol error is a slip to an ADJACENT level, so Gray coding
 *  turns one symbol error into one bit error instead of two. That is a factor
 *  of two in pre-FEC BER for free, and it is why every PAM4 standard uses it.
 *
 *  TX FFE and the L1 CONSTRAINT: the taps satisfy sum|c_k| = 1, because the
 *  driver is PEAK limited, not energy limited. For a symbol alphabet bounded
 *  by 1, the worst-case output over all data patterns is exactly
 *
 *      max |x[n]| = sum_k |c_k|                     (the L1 norm)
 *
 *  attained when every symbol aligns with the sign of its tap. Constraining
 *  L1 = 1 therefore says exactly "the driver never clips". An L2 constraint
 *  would permit a peak of sqrt(L) and the output stage would saturate.
 *
 *  Physically the constraint is a resource count: a segmented current-mode
 *  driver has N unit slices and you allocate them between the taps, so
 *  sum|c_k| = 1 is not a normalisation someone chose, it is how many slices
 *  exist. Increasing pre-shoot always means taking slices from the cursor.
 * =========================================================================*/
#ifndef TX_H
#define TX_H

#include "link_config.h"
#include <stdint.h>

/* ---- PRBS31: x^31 + x^28 + 1, the standard SerDes stress pattern --------- */
typedef struct { uint32_t state; } prbs_t;

void     prbs_init(prbs_t *p, uint32_t seed);
uint32_t prbs_bit (prbs_t *p);
uint32_t prbs_bits(prbs_t *p, unsigned n);

/* ---- PAM4 --------------------------------------------------------------- */
/* Gray symbol (0..3) -> normalised level in {-1, -1/3, +1/3, +1}. */
real_t pam4_level(unsigned gray_sym);
/* Gray symbol -> the two bits it carries, MSB first. */
void   pam4_bits (unsigned gray_sym, unsigned *b1, unsigned *b0);
/* Nearest-level slicer: level -> Gray symbol. */
unsigned pam4_slice(real_t y);
/* Bit errors between two Gray symbols (0, 1 or 2). */
unsigned pam4_bit_errors(unsigned a, unsigned b);

/* ---- TX FFE ------------------------------------------------------------- */
#define TX_FFE_TAPS 3u
#define TX_FFE_CURSOR 1u

typedef struct {
    real_t   c[TX_FFE_TAPS];      /* [pre, cursor, post], sum|c| == 1        */
    real_t   hist[TX_FFE_TAPS];
    unsigned n;
} tx_ffe_t;

/* Set taps from a pre-shoot / de-emphasis pair, then L1-normalise so the
 * driver peak is exactly 1.0. Returns the realised cursor amplitude, which is
 * the launch amplitude you gave up to buy flatness. */
real_t tx_ffe_set(tx_ffe_t *f, real_t pre, real_t post);
real_t tx_ffe_step(tx_ffe_t *f, real_t sym);

/* ---- full transmitter --------------------------------------------------- */
typedef struct {
    prbs_t   prbs;
    tx_ffe_t ffe;
} tx_t;

void   tx_init(tx_t *tx, uint32_t seed, real_t pre, real_t post);
/* Produce one symbol: returns the pre-distorted level and reports the Gray
 * symbol that was sent, so the receiver can be scored against it. */
real_t tx_step(tx_t *tx, unsigned *sym_out);

/* Render one symbol as OSR samples of a held rectangle (the DAC output). */
void   tx_upsample(real_t level, real_t *dst);

#endif /* TX_H */
