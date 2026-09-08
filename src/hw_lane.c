#include "hw_lane.h"
#include "hal.h"
#include "eye.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SPAN_UI 48u

/* xorshift32. NOT an LCG: an LCG's low-order bits have a period of 2, so
 * `lcg() & 1` alternates 1,0,1,0 and your "random" data never produces the
 * worst-case ISI patterns. Every eye then looks wide open and the model
 * silently validates a receiver that does not work. */
static uint32_t xs32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

static double gauss(uint32_t *s)
{
    double u1 = (double)(xs32(s) >> 8) / 16777216.0;
    const double u2 = (double)(xs32(s) >> 8) / 16777216.0;
    if (u1 < 1e-12) {
        u1 = 1e-12;
    }
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static int hw_lane_finish_init(hw_lane_t *L, afe_mode_t mode, double ppm);

int hw_lane_init(hw_lane_t *L, double il_db, afe_mode_t mode, double ppm)
{
    memset(L, 0, sizeof(*L));
    if (channel_build(&L->ch, il_db, SPAN_UI) != 0) {
        return -1;
    }
    return hw_lane_finish_init(L, mode, ppm);
}

int hw_lane_init_sparam(hw_lane_t *L, const char *s4p, afe_mode_t mode,
                        double ppm)
{
    memset(L, 0, sizeof(*L));
    if (channel_build_from_sparam(&L->ch, s4p, NULL, SPAN_UI) != 0) {
        return -1;
    }
    return hw_lane_finish_init(L, mode, ppm);
}

static int hw_lane_finish_init(hw_lane_t *L, afe_mode_t mode, double ppm)
{
    tx_init(&L->tx, 0xC0FFEEu, -0.10, -0.18);
    afe_init(&L->afe, mode);
    eq_init(&L->eq);
    /* Loop constants chosen an order of magnitude apart from the equaliser's
     * update rate -- see the bandwidth-separation note in fw_bringup.c. */
    /* Type-2 gains, MEASURED not guessed -- see apps/cdr_probe.c.
     *
     * ki sets the PULL-IN RANGE: the integrator has to reach ppm*1e-6*OSR
     * samples per symbol (1.9e-3 at 120 ppm) before the loop can hold phase.
     * kp sets how fast it gets there and how much jitter it lets through.
     *
     * The original 1.2e-2 / 1.0e-4 did not lock at any offset. The phase
     * slewed a full UI every two blocks, which swept the TED across its entire
     * S-curve so its mean went to zero -- and the lock detector, watching that
     * mean, called it LOCKED. Everything downstream then sampled at a phase
     * that was walking through the eye.
     *
     * Swept over kp in [0.02, 0.25] and ki in [1.5e-4, 4e-3] at +120, 0, -80
     * and -200 ppm, the loop locks over the whole range with these, and the
     * recovered offset tracks with unity slope. ki above about 6e-4 is
     * unstable: the integrator overshoots inside one block and the loop
     * oscillates instead of settling. */
    cdr_init(&L->cdr, 1.5e-1, 3.0e-4);

    L->buf_samples = (size_t)HW_BLOCK_SYMS * OSR;
    L->osr_buf = (real_t *)calloc(L->buf_samples, sizeof(real_t));
    L->rx_buf  = (real_t *)calloc(L->buf_samples, sizeof(real_t));
    /* Transmitted symbols and decisions are PER LANE, not file statics. Eight
     * lanes interleave their transmit and receive phases inside one block, so
     * a shared buffer would leave every lane training against lane 7's data. */
    L->xt_sum  = (real_t *)calloc(L->buf_samples, sizeof(real_t));
    L->syms = (unsigned *)calloc(HW_BLOCK_SYMS, sizeof(unsigned));
    L->decs = (unsigned *)calloc(HW_BLOCK_SYMS, sizeof(unsigned));
    if (L->osr_buf == NULL || L->rx_buf == NULL || L->xt_sum == NULL ||
        L->syms == NULL || L->decs == NULL) {
        hw_lane_free(L);
        return -1;
    }
    /* Seed the training-reference delay from the channel itself: find the
     * peak of the pulse response, which is the cursor UI, and add the FFE tap
     * the cursor sits on. This is the design-time estimate; the correlation in
     * hw_lane_run() refines it against the real thing. */
    {
        const size_t np = (size_t)SPAN_UI * OSR;
        real_t *p = (real_t *)calloc(np, sizeof(real_t));
        unsigned d = 0u;
        if (p != NULL && channel_pulse_response(&L->ch, p, np) == 0) {
            size_t pk = 0;
            for (size_t i = 1; i < np; ++i) {
                if (fabs(p[i]) > fabs(p[pk])) {
                    pk = i;
                }
            }
            d = (unsigned)(pk / OSR);
        }
        free(p);
        L->tx_lat = d + FFE_CURSOR;
        if (L->tx_lat >= HW_LAT_MAX) {
            L->tx_lat = HW_LAT_MAX - 1u;
        }
    }

    L->rng            = 0x1234ABCDu;
    L->ppm_offset     = ppm;
    L->noise_scale    = 1.0;
    L->signal_present = 1u;
    hw_lane_set_seed(L, 0u);
    return 0;
}

void hw_lane_set_seed(hw_lane_t *L, unsigned lane)
{
    /* EVERY LANE MUST CARRY DIFFERENT DATA.
     *
     * Give eight lanes the same PRBS seed and they transmit the same symbols
     * at the same time. The crosstalk arriving at a victim is then a scaled,
     * filtered copy of its OWN signal -- perfectly correlated with it. What
     * that models is not crosstalk; it is a slightly different channel
     * response, and it produces a modest, well-behaved, entirely fictional
     * penalty. Real aggressors are independent data, their contribution is
     * noise-like, and it is far more damaging.
     *
     * The same applies to the noise generator: one seed shared across lanes
     * makes eight "independent" noise sources identical.
     *
     * Odd multipliers keep the streams from colliding: PRBS31 has period
     * 2^31-1, so distinct non-zero seeds put the lanes at distinct, far-apart
     * points on the same sequence. */
    L->lane_id = lane;
    prbs_init(&L->tx.prbs, 0xC0FFEEu + lane * 0x9E3779B9u);
    L->rng = 0x1234ABCDu + lane * 0x85EBCA6Bu;
    if (L->rng == 0u) {
        L->rng = 1u;                 /* xorshift32 is dead at zero */
    }
}

void hw_lane_set_fec(hw_lane_t *L, unsigned on)
{
    L->fec_on    = on;
    L->pcs_lat   = 0u;
    L->pcs_calib = on;
    if (on) {
        /* Same seed both ends. The receiver regenerates the transmitted
         * payload to score against, exactly as a BER tester does -- there is
         * no back channel here and none is implied. */
        const uint32_t seed = 0xA5A51234u + L->lane_id * 0x27220A95u;
        pcs_tx_init(&L->pcs_tx, seed);
        pcs_rx_init(&L->pcs_rx, seed);
    }
}

void hw_lane_set_noise(hw_lane_t *L, double scale)
{
    L->noise_scale = (scale < 0.0) ? 0.0 : scale;
}

void hw_lane_free(hw_lane_t *L)
{
    channel_free(&L->ch);
    free(L->osr_buf);
    free(L->rx_buf);
    free(L->xt_sum);
    free(L->syms);
    free(L->decs);
    L->osr_buf = NULL;
    L->rx_buf  = NULL;
    L->xt_sum  = NULL;
    L->syms    = NULL;
    L->decs    = NULL;
}

void hw_lane_set_signal(hw_lane_t *L, unsigned present)
{
    L->signal_present = present;
}

/* Apply whatever the firmware most recently wrote to the AFE registers.
 * On silicon the analogue blocks are wired to the register bits directly;
 * this function is where that wiring lives in the model. */
static void hw_apply_afe_regs(hw_lane_t *L)
{
    /* The timing detector's h1 target is a control register like any other:
     * the datapath reads it, firmware owns it. */
    cdr_set_h1_target(&L->cdr,
                      (double)hal_field_get(REG_CDR_CTRL, CDR_H1_MASK, CDR_H1_SHIFT)
                      / CDR_H1_SCALE);
    tia_set_code (&L->afe.tia,  hal_field_get(REG_AFE_TIA,  TIA_GAIN_MASK,  TIA_GAIN_SHIFT));
    vga_set_code (&L->afe.vga,  hal_field_get(REG_AFE_VGA,  VGA_GAIN_MASK,  VGA_GAIN_SHIFT));
    ctle_set_code(&L->afe.ctle, hal_field_get(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT));
}

/* ===========================================================================
 *  A block, in four pieces.
 *
 *  One lane on its own does not need the split -- hw_lane_run() below just
 *  calls all four in order. A MACRO does: eight lanes couple into each other,
 *  so every lane's transmitter has to run before any lane's receiver, and the
 *  crosstalk has to be summed into the victim after its own channel and before
 *  its front end. That ordering is physical, not a convenience:
 *
 *      all TX  ->  each victim's own channel  ->  add aggressors  ->  each AFE
 *
 *  Doing it lane-at-a-time instead would have lane 0 receiving crosstalk from
 *  the block lane 1 has not transmitted yet.
 * =========================================================================*/

void hw_lane_begin(hw_lane_t *L)
{
    /* The reference PLL runs regardless of everything else: it is the clock
     * the rest of the lane depends on, and nothing downstream is meaningful
     * until it has locked. */
    pll_step(&L->pll);

    hw_apply_afe_regs(L);
    eq_refresh_taps(&L->eq);

    /* Clear the accumulators at the start of every block.
     *
     * Without this they integrate for the lifetime of the link, so the
     * firmware reads a running average that barely moves when the gain
     * changes -- the AGC then concludes its correction did nothing and keeps
     * cranking until it hits the rail. Hardware statistics registers are
     * per-observation-window, and the window has to actually close. */
    eq_clear_accumulators(&L->eq);
}

void hw_lane_tx(hw_lane_t *L)
{
    unsigned *syms = L->syms;

    /* ---- codeword window ------------------------------------------------
     * The equaliser loop below cannot process the first or last symbol of a
     * block: the CDR interpolator needs a neighbour on each side. On top of
     * that the equaliser is a FILTER, so its decision for a symbol emerges
     * some symbols later, and how many depends on which tap adaptation chose
     * as the cursor.
     *
     * Both effects are block-boundary artefacts of simulating a continuous
     * link in finite chunks, and neither must be allowed to leak into the BER.
     * So the payload window is closed on both sides: the transmitter stops
     * feeding codeword symbols `lat` symbols early, and the receiver starts
     * scoring `lat` symbols late. Decision j then carries payload symbol j,
     * every block, with no drift to accumulate.
     *
     * `lat` itself is measured, not assumed -- see the calibration block at
     * the end of this function. */
    const unsigned lat   = L->pcs_lat;
    const unsigned tx_hi = (HW_BLOCK_SYMS - 2u) - (L->pcs_calib ? 0u : lat);

    /* ---- transmit and propagate ---------------------------------------- */
    for (unsigned n = 0; n < HW_BLOCK_SYMS; ++n) {
        unsigned s;
        real_t   lvl;
        if (L->fec_on && n >= 1u && n <= tx_hi) {
            s   = pcs_tx_next(&L->pcs_tx);
            lvl = tx_step_sym(&L->tx, s);
        } else {
            lvl = tx_step(&L->tx, &s);
        }
        syms[n] = s;
        tx_upsample(L->signal_present ? lvl : (real_t)0.0,
                    &L->osr_buf[(size_t)n * OSR]);
    }
}

void hw_lane_channel(hw_lane_t *L)
{
    channel_apply(&L->ch, L->osr_buf, L->rx_buf, L->buf_samples);
}

void hw_lane_xtalk_begin(hw_lane_t *L)
{
    if (L->xt_sum != NULL) {
        memset(L->xt_sum, 0, L->buf_samples * sizeof(real_t));
    }
    L->xt_any = 0u;
}

void hw_lane_xtalk_add(hw_lane_t *victim, const hw_lane_t *aggressor,
                       double scale)
{
    /* The aggressor's TRANSMITTED waveform is what couples, not its received
     * one: far-end crosstalk is picked up along the trace from the neighbour's
     * driver. Using the aggressor's rx_buf would double-count its channel loss
     * and understate the coupling by exactly that much. */
    if (victim->xt_sum == NULL || aggressor->osr_buf == NULL) {
        return;
    }
    for (size_t i = 0; i < victim->buf_samples; ++i) {
        victim->xt_sum[i] += (real_t)(scale * (double)aggressor->osr_buf[i]);
    }
    victim->xt_any = 1u;
}

void hw_lane_xtalk_apply(hw_lane_t *L)
{
    /* One filter pass, one piece of state. See the note in hw_lane.h for why
     * this is not the same as filtering each aggressor separately. */
    if (L->xt_sum == NULL) {
        return;
    }
    channel_apply_xtalk(&L->ch, L->xt_sum, L->rx_buf, L->buf_samples, 1.0);
}

void hw_lane_rx(hw_lane_t *L, hw_mode_t mode)
{
    unsigned *syms = L->syms;
    unsigned *decs = L->decs;

    /* ---- analogue front end, sample by sample --------------------------- */
    for (size_t i = 0; i < L->buf_samples; ++i) {
        L->rx_buf[i] = afe_step(&L->afe, L->rx_buf[i],
                                (real_t)(gauss(&L->rng) * L->noise_scale));
    }

    /* ---- CDR sampling, equaliser, accumulation -------------------------- */
    const unsigned lat   = L->pcs_lat;
    const unsigned rx_lo = 1u + (L->pcs_calib ? 0u : lat);
    const unsigned adapt = hal_read32(REG_ADAPT_CTRL);
    const unsigned lat_t = L->tx_lat;

    for (unsigned n = 1; n + 1u < HW_BLOCK_SYMS; ++n) {
        const real_t y_in = cdr_sample(&L->cdr, L->rx_buf, L->buf_samples, n);

        /* The training symbol this decision is actually about, reaching back
         * into the previous block when the delay runs off the start. */
        const unsigned training = (mode != HW_MODE_DATA) ? 1u : 0u;
        /* VERIFY scores against the known pattern but denies the receiver
         * access to it: the LMS and the DFE both run on decisions, exactly as
         * they will in traffic, while the errors are still countable. That is
         * the only configuration in which a pre-traffic measurement predicts
         * the post-traffic one. */
        L->eq.dd = (mode == HW_MODE_TRAIN) ? 0u : 1u;
        unsigned ref = 0xFFFFFFFFu;
        if (training) {
            ref = (n >= lat_t) ? syms[n - lat_t]
                               : L->prev_syms[HW_LAT_MAX - (lat_t - n)];
        }

        /* Equalise and adapt on every symbol; SCORE only the ones whose
         * reference and pipeline are entirely inside this block. The first few
         * reach back across the block seam, and a block seam is an artefact of
         * simulating a continuous link in chunks. Twenty-one symbols out of
         * 4094 changes no statistic that matters, and it makes the firmware's
         * error counter measure the same population the BER tester does --
         * which it has to, or bring-up rejects links the tester calls clean. */
        L->eq.scoring = (n >= lat_t + FFE_TAPS) ? 1u : 0u;

        unsigned sym;
        const real_t y = eq_step(&L->eq, y_in, ref, &sym);
        (void)y;   /* the slicer output; the timing loop deliberately does
                    * NOT use it -- see the note above cdr_dl */
        decs[n] = sym;

        /* THE TIMING LOOP MUST NOT BE TAPPED OFF THE EQUALISER OUTPUT.
         *
         * This was the bug under everything else, and it is the subtlest one
         * in the project. Mueller-Muller's entire output is
         *
         *      E[e] = sigma_a^2 * ( h(+1UI) - h(-1UI) )
         *
         * the difference between the first postcursor and the first precursor.
         * That is EXACTLY the quantity the FFE and DFE exist to drive to zero.
         * Feed the CDR the equalised sample and the two loops are driven from
         * one node, competing for the same error -- and the tap loop, with
         * twenty-four degrees of freedom against the timing loop's one, wins.
         * The equaliser nulls the S-curve at whatever phase the sampler
         * currently sits at, so the detector's setpoint follows its own state
         * and the phase is free to wander. On top of that the DFE subtracts a
         * term perfectly correlated with a[n-1], which lands on the detector
         * as a phase-INDEPENDENT DC offset; if it is larger than what is left
         * of the S-curve there is no zero crossing anywhere in the unit
         * interval, for either sign, and the integrator simply winds to its
         * rail.
         *
         * Measured before the fix: the detector's peak output was under 0.005
         * where a working normalised MM detector gives 0.1 to 0.5 -- twenty to
         * a hundred times down. It was not inverted so much as absent, and the
         * lock detector, watching a signed average that was near zero for the
         * wrong reason, reported LOCKED throughout.
         *
         * So the CDR observes the RAW sampled waveform, which is what its
         * interpolator actually moves on, delayed by the FFE's cursor tap so
         * that the sample and the symbol it is compared against line up. The
         * FFE cursor is pinned (see fw_adapt.c), so that delay is a constant
         * and not something adaptation can move underneath the loop. */
        const real_t y_cdr = L->cdr_dl[FFE_CURSOR - 1u];
        for (unsigned k = FFE_CURSOR - 1u; k > 0u; --k) {
            L->cdr_dl[k] = L->cdr_dl[k - 1u];
        }
        L->cdr_dl[0] = y_in;

        if ((adapt & ADAPT_CDR_EN) != 0u) {
            /* DATA-AIDED WHILE TRAINING, DECISION-DIRECTED AFTERWARDS.
             *
             * The detector is built out of decisions, and while the eye is
             * closed those are wrong often enough that the timing error is
             * computed from noise. During training the receiver already knows
             * the symbols, so it uses them, exactly as the tap loop does. Once
             * the eye is open it switches to its own decisions, which is what
             * it has to live on in traffic. */
            cdr_update(&L->cdr, y_cdr,
                       (ref == 0xFFFFFFFFu) ? pam4_level(sym) : pam4_level(ref));
        }
        if (L->fec_on && n >= rx_lo) {
            /* Hand the decision to the PCS. `rx_suspect` is side information
             * from outside the datapath -- a lane the supervisor knows is in
             * trouble -- not a slicer confidence measure. */
            pcs_rx_push(&L->pcs_rx, sym, (int)L->rx_suspect);
        }
        /* Reference-clock mismatch: the sampling instant drifts by ppm every
         * symbol whether or not the CDR is tracking it. This is what forces a
         * type-2 loop -- a type-1 loop would accumulate phase error forever.
         *
         * The wrap here is NOT redundant with the one in cdr_update(). This
         * drift is applied even while the CDR is disabled (during AGC), and
         * cdr_update() is the only other place the phase is wrapped. Without
         * this, the phase ran away unbounded before the CDR was ever enabled:
         * negative offsets drove it below zero, cdr_sample() clamped to index
         * 0, and every symbol in the block read the same sample. Negative ppm
         * failed 100% of the time and positive ppm only survived because the
         * AGC happened to finish quickly. */
        L->cdr.phase += L->ppm_offset * 1e-6 * (double)OSR;
        while (L->cdr.phase >= (double)OSR) { L->cdr.phase -= (double)OSR; }
        while (L->cdr.phase <  0.0)         { L->cdr.phase += (double)OSR; }
    }
    L->symbols += HW_BLOCK_SYMS;

    /* ---- re-measure the training latency -------------------------------
     * Slide the transmitted symbols against the decisions and take the offset
     * with the fewest bit errors. The winner is unambiguous once the eye is
     * open at all: the correct offset scores near zero and every other offset
     * scores near one bit error per symbol, because a PAM4 stream compared
     * against an unrelated one does exactly that. Costs 32 x 2048 comparisons
     * against the block's 50 million channel multiply-accumulates. */
    if (mode != HW_MODE_DATA && !L->lat_locked) {
        unsigned best_off = L->tx_lat;
        unsigned best_err = 0xFFFFFFFFu;
        for (unsigned off = 0; off < HW_LAT_MAX; ++off) {
            unsigned err = 0u;
            for (unsigned i = 0; i < HW_LAT_CORR; ++i) {
                const unsigned n = off + 1u + i;
                err += pam4_bit_errors(decs[n], syms[n - off]);
            }
            if (err < best_err) {
                best_err = err;
                best_off = off;
            }
        }
        L->tx_lat  = best_off;
        L->lat_err = best_err;

        /* LOCK IT AND LEAVE IT ALONE.
         *
         * Re-measuring every block looks harmless and is not. The training
         * reference defines which symbol the cursor tap is supposed to
         * reproduce; the cursor tap position defines where the energy ends up.
         * Let both float and they chase each other: the taps shift the group
         * delay by a symbol, the next correlation moves the reference to
         * match, and the equaliser converges to a filter with two large taps
         * and the first postcursor at the WRONG SIGN -- measured here as a
         * stable 11% BER with every loop reporting itself converged.
         *
         * So the pattern is locked once, while the FFE is still the initial
         * single spike and the answer is therefore unambiguous. From then on
         * the reference is fixed and adaptation has exactly one solution to
         * find. This is what "lock to the training pattern, then train" means
         * -- the order is not a detail. */
        if (best_err < HW_LAT_CORR / 2u) {
            L->lat_locked = 1u;
        }
    }

    /* Carry the block's tail forward so the next block's reference can reach
     * back across the boundary. */
    for (unsigned i = 0; i < HW_LAT_MAX; ++i) {
        L->prev_syms[i] = syms[HW_BLOCK_SYMS - HW_LAT_MAX + i];
    }

    /* ---- drive the status and accumulator registers --------------------- */
    uint32_t st = hal_read32(REG_STATUS) & ~(STAT_SIGDET | STAT_CDR_LOCK);
    if (L->signal_present) {
        st |= STAT_SIGDET;
    } else {
        st |= STAT_LOS;                       /* W1C: sticky until acked */
    }
    if (L->cdr.locked) {
        st |= STAT_CDR_LOCK;
    }
    hw_reg_set(REG_STATUS, st);
    hw_reg_set(REG_CDR_PHASE, (uint32_t)(int32_t)(L->cdr.phase * 256.0));
    hw_reg_set(REG_CDR_FREQ,  (uint32_t)(int32_t)(cdr_ppm(&L->cdr) * 256.0));
    eq_publish(&L->eq);
    (void)syms;

    /* ---- equaliser latency calibration ---------------------------------
     * The first FEC block runs with the window wide open purely so the PCS
     * aligner can measure how far the decisions lag the transmitted symbols.
     * Once it has, everything measured so far is thrown away and both ends
     * restart with the window closed to that latency. Discarding the
     * calibration data matters: it was taken on a stream that was, by
     * construction, misframed at the block edges. */
    if (L->fec_on && L->pcs_calib && L->pcs_rx.state != PCS_ALIGNING) {
        L->pcs_lat   = L->pcs_rx.align_offset;
        L->pcs_calib = 0u;
        const uint32_t seed = 0xA5A51234u + L->lane_id * 0x27220A95u;
        pcs_tx_init(&L->pcs_tx, seed);
        pcs_rx_init(&L->pcs_rx, seed);
    }
}

void hw_lane_run(hw_lane_t *L, hw_mode_t mode)
{
    hw_lane_begin(L);
    /* One management bus per macro, so it ticks here in the single-lane path
     * and once per block in hw_macro.c -- never once per lane, which would
     * drain the FIFO eight times too fast and hide every backpressure bug. */
    mgmt_bus_tick();
    hw_lane_tx(L);
    hw_lane_channel(L);
    hw_lane_rx(L, mode);
}

void hw_lane_capture_eye(hw_lane_t *L, struct eye_s *eye)
{
    hw_apply_afe_regs(L);
    eq_refresh_taps(&L->eq);

    for (unsigned n = 0; n < HW_BLOCK_SYMS; ++n) {
        unsigned s;
        const real_t lvl = tx_step(&L->tx, &s);
        L->syms[n] = s;
        tx_upsample(lvl, &L->osr_buf[(size_t)n * OSR]);
    }
    channel_apply(&L->ch, L->osr_buf, L->rx_buf, L->buf_samples);
    for (size_t i = 0; i < L->buf_samples; ++i) {
        L->rx_buf[i] = afe_step(&L->afe, L->rx_buf[i], (real_t)gauss(&L->rng));
    }
    eye_accumulate(eye, L->rx_buf, L->buf_samples, (unsigned)(L->cdr.phase));
}

/* ===========================================================================
 *  Platform glue: registers that are not storage.
 *
 *  REG_MGMT_DATA is a write-only FIFO port -- the write is a PUSH and the
 *  value is never readable back. REG_EYE_ADDR latches an index and the
 *  hardware presents that byte in REG_EYE_DATA. Both are side effects, which
 *  is why they need a hook rather than a memory cell.
 * =========================================================================*/
static hw_lane_t *g_hooked_lane;

static void hw_write_hook(uint32_t off, uint32_t val)
{
    if (off == REG_MGMT_DATA) {
        mgmt_bus_push(val);
        return;
    }
    if (off == REG_EYE_ADDR && g_hooked_lane != NULL) {
        const uint32_t idx = val % MGMT_EYE_BYTES;
        hw_reg_set(REG_EYE_DATA, g_hooked_lane->eye_ram[idx]);
    }
}

void hw_lane_attach_platform(hw_lane_t *L, unsigned mgmt_bytes_per_block)
{
    g_hooked_lane = L;
    mgmt_bus_init(mgmt_bytes_per_block);
    hal_set_write_hook(hw_write_hook);
    pll_init(&L->pll, 8u);          /* ~8 blocks to settle */
}

void hw_lane_load_eye_ram(hw_lane_t *L, const struct eye_s *eye)
{
    /* Box-average the full 128x96 histogram down to 32x24 and log-compress to
     * one byte per bin. 12288 bins would take half a second on this bus; 768
     * is what a real eye monitor reports and it is enough to see the opening. */
    uint32_t peak = 1u;
    for (unsigned y = 0; y < EYE_H; ++y) {
        for (unsigned x = 0; x < EYE_W; ++x) {
            if (eye->hist[y][x] > peak) { peak = eye->hist[y][x]; }
        }
    }
    for (unsigned r = 0; r < MGMT_EYE_H; ++r) {
        for (unsigned c = 0; c < MGMT_EYE_W; ++c) {
            uint32_t acc = 0u;
            for (unsigned y = r * EYE_H / MGMT_EYE_H; y < (r + 1u) * EYE_H / MGMT_EYE_H; ++y) {
                for (unsigned x = c * EYE_W / MGMT_EYE_W; x < (c + 1u) * EYE_W / MGMT_EYE_W; ++x) {
                    acc += eye->hist[y][x];
                }
            }
            const double v = log1p((double)acc) / log1p((double)peak);
            int p = (int)(v * 255.0 + 0.5);
            if (p > 255) { p = 255; }
            L->eye_ram[r * MGMT_EYE_W + c] = (uint8_t)p;
        }
    }
}
