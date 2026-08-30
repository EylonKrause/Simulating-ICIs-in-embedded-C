/* ===========================================================================
 *  hw_lane.h — the "silicon". TX, channel, AFE, CDR and equaliser wired
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

#define HW_BLOCK_SYMS 4096u

typedef struct {
    channel_t ch;
    tx_t      tx;
    afe_t     afe;
    eq_t      eq;
    cdr_t     cdr;

    real_t   *osr_buf;      /* oversampled TX-through-channel waveform */
    real_t   *rx_buf;       /* after the AFE                           */
    size_t    buf_samples;

    uint32_t  rng;
    double    ppm_offset;   /* TX/RX reference mismatch                */
    uint64_t  symbols;
    unsigned  signal_present;
} hw_lane_t;

int  hw_lane_init(hw_lane_t *L, double il_db, afe_mode_t mode, double ppm);
void hw_lane_free(hw_lane_t *L);

/* Run one block of symbols. `training` selects data-aided (known symbols) or
 * decision-directed operation. Updates the accumulators and publishes them. */
void hw_lane_run(hw_lane_t *L, unsigned training);

/* Model a loss-of-signal event, so the bring-up FSM has something to fail on. */
void hw_lane_set_signal(hw_lane_t *L, unsigned present);

/* Capture one block into an eye-diagram accumulator (see eye.h). */
struct eye_s;
void hw_lane_capture_eye(hw_lane_t *L, struct eye_s *eye);

#endif /* HW_LANE_H */
