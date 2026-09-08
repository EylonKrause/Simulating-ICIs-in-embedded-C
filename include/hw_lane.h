/* ===========================================================================
 *  hw_lane.h -- the "silicon". TX, channel, AFE, CDR and equaliser wired
 *  together, running per symbol, driving the register file.
 *
 *  Everything in here is the analogue of RTL plus analogue circuitry. It uses
 *  floating point freely, because it stands in for physics. It NEVER calls a
 *  fw_* function, and firmware never calls into it except through registers.
 *
 *  Control flows in one direction only:
 *      firmware --writes--> registers --read by--> hardware
 *      hardware --writes--> RO/W1C registers --read by--> firmware
 * =========================================================================*/
#ifndef HW_LANE_H
#define HW_LANE_H

#include "link_config.h"
#include "channel.h"
#include "tx.h"
#include "afe.h"
#include "eq.h"
#include "cdr.h"
#include "pll.h"
#include "mgmt.h"
#include "pcs.h"

#define HW_BLOCK_SYMS 4096u

/* How a block is run. The distinction between TRAIN and VERIFY is the whole
 * reason bring-up can trust its own answer: both know the transmitted pattern
 * and score against it, but only TRAIN hands that pattern to the DFE. */
typedef enum {
    HW_MODE_DATA = 0,   /* decision-directed, unscored: real traffic      */
    HW_MODE_TRAIN,      /* data-aided, DFE fed the known symbols, scored  */
    HW_MODE_VERIFY      /* scored against the pattern, DFE on its own     */
} hw_mode_t;

/* Widest TX-symbol-to-decision latency the receiver will look for. The real
 * figure is the channel's group delay plus wherever adaptation put the FFE
 * cursor, so it is a handful of symbols; 32 is generous. */
#define HW_LAT_MAX      32u
/* Symbols used to measure that latency by correlation, once per block. */
#define HW_LAT_CORR   2048u

typedef struct {
    channel_t ch;
    tx_t      tx;
    afe_t     afe;
    eq_t      eq;
    cdr_t     cdr;
    pll_t     pll;

    /* Eye-monitor capture RAM. Hardware fills it; firmware reads it one byte
     * at a time through the REG_EYE_ADDR / REG_EYE_DATA window, because
     * firmware has no pointer into a hardware buffer -- it has an address
     * register and a data register. */
    uint8_t   eye_ram[MGMT_EYE_BYTES];

    unsigned *syms;         /* transmitted symbols, this block         */
    unsigned *decs;         /* slicer decisions, this block            */
    real_t   *osr_buf;      /* oversampled TX-through-channel waveform */
    real_t   *rx_buf;       /* after the AFE                           */
    real_t   *xt_sum;       /* weighted sum of aggressor TX waveforms   */
    unsigned  xt_any;       /* whether any aggressor contributed        */
    size_t    buf_samples;

    /* TRAINING ALIGNMENT. The decision for a transmitted symbol comes out of
     * the receiver some symbols later: the channel has group delay and the FFE
     * is a filter whose cursor sits on whichever tap adaptation chose. Data-
     * aided LMS compares the slicer output against the KNOWN symbol, so if the
     * reference is not delayed by the same amount, the error signal is
     * uncorrelated with the sample history, the gradients average to zero, and
     * the taps sit wherever they were initialised while the link reports a
     * confident 50% BER.
     *
     * `tx_lat` is seeded from the channel's own pulse response at init and
     * then re-measured every training block by correlating decisions against
     * the transmitted symbols. A real receiver does the same thing, and for
     * the same reason: it has to lock to the training pattern before the
     * pattern is worth anything. */
    unsigned  tx_lat;
    unsigned  lat_locked;
    unsigned  lat_err;                    /* residual at the chosen offset */
    unsigned  prev_syms[HW_LAT_MAX];      /* tail of the previous block     */

    /* Raw CDR-sampled input, delayed to line up with the FFE cursor. The
     * timing loop runs on this, NOT on the equaliser output -- see the long
     * comment in hw_lane.c. */
    real_t    cdr_dl[FFE_CURSOR];

    uint32_t  rng;
    double    ppm_offset;   /* TX/RX reference mismatch                */
    double    noise_scale;  /* stress knob; 1.0 is the modelled AFE noise */
    uint64_t  symbols;
    unsigned  signal_present;
    unsigned  lane_id;      /* decorrelates data and noise across lanes */

    /* FEC-encoded traffic. Bring-up runs on PRBS31 because the equaliser
     * needs a known training pattern with a hard transition density; once the
     * link is up it carries real codewords, which is the only configuration
     * in which a post-FEC BER means anything. */
    unsigned  fec_on;
    unsigned  rx_suspect;   /* side information: this lane is known degraded */
    unsigned  pcs_lat;      /* measured equaliser latency, symbols           */
    unsigned  pcs_calib;    /* still measuring it                            */
    pcs_tx_t  pcs_tx;
    pcs_rx_t  pcs_rx;
} hw_lane_t;

int  hw_lane_init(hw_lane_t *L, double il_db, afe_mode_t mode, double ppm);

/* Same, but the channel comes from a Touchstone file -- which is the only way
 * to get an off-diagonal term, and therefore the only way to get crosstalk. */
int  hw_lane_init_sparam(hw_lane_t *L, const char *s4p, afe_mode_t mode,
                         double ppm);
void hw_lane_free(hw_lane_t *L);

/* Run one block of symbols. `training` selects data-aided (known symbols) or
 * decision-directed operation. Updates the accumulators and publishes them. */
void hw_lane_run(hw_lane_t *L, hw_mode_t mode);

/* The same block, in the four pieces a MACRO needs, so that every lane can
 * transmit before any lane receives and the crosstalk can be summed in
 * between. hw_lane_run() is exactly these four in order. */
void hw_lane_begin  (hw_lane_t *L);                  /* per-block prologue  */
void hw_lane_tx     (hw_lane_t *L);                  /* fill osr_buf        */
void hw_lane_channel(hw_lane_t *L);                  /* osr_buf -> rx_buf   */
void hw_lane_rx     (hw_lane_t *L, hw_mode_t mode);

/* CROSSTALK: SUM THE AGGRESSORS FIRST, THEN CONVOLVE ONCE.
 *
 * The obvious shape -- convolve each aggressor separately and add the results
 * -- is wrong here, and wrong in a way that is invisible inside a block. The
 * crosstalk convolution carries an overlap-add tail from one block to the
 * next, and that tail lives on the VICTIM's channel. Run three aggressors
 * through it in turn and they all share one tail, so the residue left by
 * neighbour -3 at the end of a block is emitted at the start of the next one
 * scaled by neighbour +1's coupling weight. Superposition inside a block looks
 * perfect; only the block-boundary term is wrong, which is exactly the sort of
 * error a BER number will never show you.
 *
 * Convolution is linear, so summing the aggressors' transmitted waveforms
 * first and filtering once is not merely a workaround -- it is the same
 * arithmetic with one state variable instead of N, and it costs one FFT pass
 * instead of N:
 *
 *      y += h_xt * ( sum_k w_k * x_k )
 *
 * Call begin, then add for each neighbour, then apply, between
 * hw_lane_channel() and hw_lane_rx(). */
void hw_lane_xtalk_begin(hw_lane_t *victim);
void hw_lane_xtalk_add  (hw_lane_t *victim, const hw_lane_t *aggressor,
                         double scale);
void hw_lane_xtalk_apply(hw_lane_t *victim);

/* Model a loss-of-signal event, so the bring-up FSM has something to fail on. */
void hw_lane_set_signal(hw_lane_t *L, unsigned present);

/* Switch the payload from the PRBS training pattern to RS(544,514)-encoded
 * traffic and start scoring pre- and post-FEC BER. Call this once the link is
 * up: running codewords through an unconverged equaliser measures the
 * bring-up, not the code. */
void hw_lane_set_fec(hw_lane_t *L, unsigned on);

/* Scale the front-end noise. The nominal model is 1.0; larger values walk the
 * link down its waterfall so the FEC has something to do. */
void hw_lane_set_noise(hw_lane_t *L, double scale);

/* Re-seed this lane's data and noise from its lane number. Lanes carrying
 * identical data make crosstalk correlated with the victim, which is not
 * crosstalk at all -- see the comment in hw_lane.c. */
void hw_lane_set_seed(hw_lane_t *L, unsigned lane);

/* Attach the register side-effect hooks and start the management bus. */
void hw_lane_attach_platform(hw_lane_t *L, unsigned mgmt_bytes_per_block);

/* Capture one block into an eye-diagram accumulator (see eye.h). */
struct eye_s;
void hw_lane_capture_eye(hw_lane_t *L, struct eye_s *eye);

/* Downsample an accumulated eye into the capture RAM the firmware reads. */
void hw_lane_load_eye_ram(hw_lane_t *L, const struct eye_s *eye);

#endif /* HW_LANE_H */
