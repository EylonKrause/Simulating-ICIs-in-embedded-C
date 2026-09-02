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
#include "pll.h"
#include "mgmt.h"
#include "pll.h"
#include "mgmt.h"

#define HW_BLOCK_SYMS 4096u

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

/* Attach the register side-effect hooks and start the management bus. */
void hw_lane_attach_platform(hw_lane_t *L, unsigned mgmt_bytes_per_block);

/* Capture one block into an eye-diagram accumulator (see eye.h). */
struct eye_s;
void hw_lane_capture_eye(hw_lane_t *L, struct eye_s *eye);

/* Downsample an accumulated eye into the capture RAM the firmware reads. */
void hw_lane_load_eye_ram(hw_lane_t *L, const struct eye_s *eye);

#endif /* HW_LANE_H */
