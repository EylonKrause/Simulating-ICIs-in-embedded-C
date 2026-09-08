/* ===========================================================================
 *  fw_bringup.c -- link bring-up state machine, timeouts, retry, telemetry.
 *
 *  FOUR PROPERTIES, and it is worth naming them while writing a state machine
 *  in an interview rather than after:
 *
 *    1. EVERY state has a timeout. A state with no timeout is a hang waiting
 *       for a customer to find it. The timeouts live in a table, not in the
 *       code, so they can be tuned without touching logic.
 *    2. Timeout arithmetic is WRAP-SAFE: (now - start) >= timeout. The
 *       obvious form, now >= start + timeout, breaks once every 49 days on a
 *       32-bit millisecond tick, which is exactly the kind of bug that ships.
 *    3. Failure BACKS OFF exponentially and retries, rather than hammering.
 *    4. Every transition is COUNTED. When a link takes 40 ms to come up
 *       instead of 4, the counters tell you which state ate the time --
 *       otherwise you are attaching a debugger to a rack.
 *
 *  The sequencing is the answer to "how do you stop the loops fighting":
 *  AGC settles first (fastest loop, amplitude only), then the CDR is enabled
 *  and allowed to lock, and only then does tap adaptation start. Loops that
 *  are not being trained are DISABLED, not merely slow -- freeze-and-hold.
 * =========================================================================*/
#include "fw.h"
#include "hal.h"
#include "fixed.h"
#include "link_config.h"

#include <string.h>

/* Convergence may not be declared before this much training has elapsed. */
/* Adaptation gears, as ADAPT_MU register codes (a biased exponent -- see
 * fw_adapt.c). Acquisition amplifies the sign-sign gradient so the taps can
 * cross tens of codes inside the training window; tracking backs off so
 * steady-state dither stays under an applied LSB. */
#define EQ_GEAR_ACQ      10u      /* 2^(10-8) = x4   */
#define EQ_GEAR_TRACK     8u      /* 2^(8-8)  = x1   */
#define EQ_MIN_TRAIN_MS  250u
/* CTLE code used only while the AGC measures the channel. Any fixed value
 * works; it just has to be the SAME every attempt, or the VGA code the AGC
 * lands on stops being comparable between attempts. */
#define CTLE_PROBE        8u
/* Training BER ceiling, in units of 1e-5, above which the equaliser is judged
 * to have converged on an answer the link should not come up on.
 *
 * KP4 is specified against a pre-FEC BER of 2.4e-4, but this measurement is
 * taken DATA-AIDED: the DFE is being fed the true symbols, so its feedback is
 * perfect and it never propagates an error. In traffic it feeds back its own
 * decisions and does. Measured at 16 dB: 2e-4 during training, 6.3e-4 in
 * traffic -- the training figure is optimistic by about 3x, and a gate set at
 * the specification limit therefore lets through links that miss it.
 *
 * 5e-5 leaves roughly a factor of five for that gap. It is not conservatism
 * for its own sake; it is the known bias of the instrument being corrected
 * for. */
#define EQ_MAX_TRAIN_BER_E5   5u
/* Symbols the training-BER judgement must see before it is allowed to decide.
 * 100k symbols is 200k bits, so a 2e-5 threshold allows four bit errors --
 * enough that ordinary noise does not condemn a good solution. */
#define EQ_JUDGE_SYMS   300000u

/* Consecutive ticks of CDR unlock before the link is declared down. */
#define LOSS_OF_LOCK_TICKS  25u

/* Per-state timeout, milliseconds. A table, not scattered constants. */
static const uint32_t STATE_TIMEOUT_MS[LS_COUNT] = {
    [LS_RESET]       =    2u,
    [LS_PLL_LOCK]    =   60u,   /* charge-pump settling, then give up      */
    [LS_WAIT_SIGNAL] =  200u,
    [LS_AGC]         =  120u,
    [LS_CDR_LOCK]    =  300u,
    [LS_EQ_TRAIN]    = 1200u,
    [LS_EQ_VERIFY]   =  300u,   /* long enough for the judging window */
    [LS_TRACK]       =  200u,
    [LS_UP]          =    0u,     /* 0 == no timeout; steady state */
    [LS_FAULT]       = 1000u,
};

static const char *const STATE_NAME[LS_COUNT] = {
    "RESET", "PLL_LOCK", "WAIT_SIGNAL", "AGC", "CDR_LOCK",
    "EQ_TRAIN", "EQ_VERIFY", "TRACK", "UP", "FAULT"
};

/* EVERY TIMEOUT IS IN WALL-CLOCK MILLISECONDS, BUT EVERY LOOP RUNS AT ITS
 * SERVICE RATE. One control processor serving N lanes S at a time gives each
 * lane one iteration every N/S milliseconds, so a 300 ms budget that bought
 * 300 loop iterations on a single lane buys 75 on an eight-lane macro serviced
 * two at a time. The loops have not got slower in seconds; they have got
 * slower in ITERATIONS, and it is iterations that converge a loop.
 *
 * So the supervisor tells the firmware how many lanes share it and the
 * timeouts scale with it. Leaving them unscaled is a bug that only appears
 * when the lane count goes up: the single-lane bench passes and the product
 * does not. */
static uint32_t g_tmo_scale = 1u;

void fw_set_timeout_scale(unsigned scale)
{
    g_tmo_scale = (scale == 0u) ? 1u : scale;
}

const char *fw_state_name(link_state_t s)
{
    return (s < LS_COUNT) ? STATE_NAME[s] : "?";
}

bool fw_is_up(const fw_link_t *L)
{
    return L->state == LS_UP;
}

static void enter(fw_link_t *L, link_state_t s)
{
    /* Remember what we were doing when we failed. Which state a bring-up dies
     * in is information, and the search below uses it. */
    if (s == LS_FAULT) {
        L->fail_from = L->state;
    }
    L->state      = s;
    L->entered_ms = L->now_ms;
    L->tm.state_entries[s]++;
    L->tm.transitions++;
}

/* Select a lane across the whole firmware in one call: the register window and
 * every per-lane loop context together. Two separate selections is one of them
 * being forgotten -- the supervisor should never be able to have the HAL
 * pointing at lane 3 while the AGC is servoing lane 5. */
void fw_select_lane(unsigned lane)
{
    hal_select_lane(lane);
    fw_adapt_select_lane(lane);
    fw_agc_select_lane(lane);
    fw_telem_select_lane(lane);
}

/* CHOOSE THE FRONT END FROM WHAT THE AGC JUST MEASURED.
 *
 * Two settings have to suit the channel before anything downstream works, and
 * both depend on the same hidden variable -- how lossy the channel is:
 *
 *   CTLE peaking. Under-peak a 30 dB channel and the eye never opens far
 *   enough for the CDR to find a phase, so bring-up stalls in CDR_LOCK with no
 *   clue as to why. Over-peak a 8 dB one and the boost amplifies noise and
 *   manufactures precursor ISI of its own.
 *
 *   The timing detector's h1 target. A lossy channel has a strongly asymmetric
 *   pulse and needs a large target, or the detector has no zero crossing
 *   anywhere in the unit interval. A short channel is nearly symmetric and a
 *   large target pushes the null off the other end.
 *
 * Blind-searching a two-dimensional space of them costs an attempt each time,
 * and an attempt is over a second. But the loss does not have to be guessed:
 * THE AGC HAS ALREADY MEASURED IT. The VGA code it converged to is how much
 * gain the channel needed, and gain needed is loss present. It is a free,
 * already-paid-for estimate, and using it turns a search into a calculation.
 *
 * Calibration, from measured operating points at a fixed probe CTLE:
 *      8 dB -> VGA 22,  20 dB -> VGA 39      =>  1.42 codes per dB
 * The CTLE is then set to roughly half the loss (0.8 dB of peaking per code)
 * and the h1 target to the pulse asymmetry the loss implies, which measurement
 * of the pulse response puts near 0.0098 per dB.
 *
 * Retries perturb the estimate rather than abandoning it -- the calibration is
 * a good first guess, not gospel, and a channel with a via stub or a bad
 * connector will not sit on the line. */
static void fw_set_front_end(fw_link_t *L)
{
    const int32_t vga = (int32_t)hal_field_get(REG_AFE_VGA, VGA_GAIN_MASK,
                                               VGA_GAIN_SHIFT);
    /* VGA code -> insertion loss at Nyquist, in dB. */
    int32_t il = ((vga - 22) * 70) / 100 + 8;

    /* STEER THE SEARCH WITH THE FAILURE MODE, DO NOT JUST WALK A LADDER.
     *
     * A blind ladder (0, +4, -4, +8, -8, ...) tries the wrong direction half
     * the time, and each wrong attempt costs the better part of a second. It
     * also has a nasty failure of its own: on a channel that wants LESS
     * peaking, the ladder's first step is MORE, and by the time it comes back
     * the other way the retry budget is gone. Measured, that is exactly what
     * happened between 14 and 17 dB -- those lanes all ended up pinned at the
     * maximum CTLE code, which is the signature of a search that walked the
     * wrong way and never came back.
     *
     * But the failure mode says which way to go, and it costs nothing to read:
     *
     *   died in CDR_LOCK   the eye never opened far enough for the detector to
     *                      find a phase  ->  the channel is LOSSIER than
     *                      estimated, ask for more peaking
     *   died in EQ_VERIFY  the loops converged and the error rate was still
     *                      too high. The CDR locked, so the eye was open; the
     *                      usual cause is over-peaking amplifying noise  ->
     *                      the channel is SHORTER than estimated, back off
     *
     * That turns a blind search into a bisection with a sign. */
    if (L->retries > 0u) {
        switch (L->fail_from) {
        case LS_CDR_LOCK:
        case LS_AGC:
            L->fe_nudge_db += 3;
            break;
        case LS_EQ_VERIFY:
        case LS_TRACK:
            L->fe_nudge_db -= 3;
            break;
        default:
            /* EQ_TRAIN timed out, or something else. No directional
             * information, so alternate to cover both sides. */
            L->fe_nudge_db = (L->fe_nudge_db > 0) ? -(L->fe_nudge_db + 2)
                                                  : -(L->fe_nudge_db - 2);
            break;
        }
        L->fe_nudge_db = (int8_t)sat_to(L->fe_nudge_db, -14, 14);
    }
    il += L->fe_nudge_db;
    il = sat_to(il, 2, 34);
    L->il_estimate_db = il;

    const int32_t ctle = sat_to((il * 100) / 160, 0, (int32_t)CTLE_PEAK_CODES - 1);
    const int32_t h1   = sat_to((il * 125) / 100, 1, (int32_t)CDR_H1_MASK);

    hal_critical_enter();
    hal_field_set(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT, (uint32_t)ctle);
    hal_field_set(REG_CDR_CTRL, CDR_H1_MASK,    CDR_H1_SHIFT,    (uint32_t)h1);
    hal_critical_exit();
}

void fw_init(fw_link_t *L)
{
    /* Firmware state only. It does NOT reset the register file.
     *
     * It used to, and that was the wrong layer twice over. The whole-file
     * reset entry point is declared in hal.h under "for the hardware model and
     * for tests -- firmware must never call these", so this was the control
     * plane reaching around its own interface. On silicon the register file is
     * reset by the reset controller before firmware runs at all, and no such
     * call is available to it.
     *
     * It was also a latent multi-lane bug: that reset clears EVERY lane's
     * aperture, so a supervisor initialising eight lanes in turn wiped lanes 0
     * through 6 while setting up lane 7. Benign only because this function
     * writes no configuration -- the first tick of LS_RESET does that -- which
     * is a thin reason for it to have been correct.
     *
     * The platform resets the register file once now, before any lane's
     * firmware is initialised. CI greps this directory to keep it that way. */
    memset(L, 0, sizeof(*L));
    L->backoff_ms = 2u;
    enter(L, LS_RESET);
}

void fw_tick(fw_link_t *L, uint32_t now_ms)
{
    L->now_ms = now_ms;

    /* --- loss of signal is checked in EVERY state, not just the waiting one.
     * A W1C bit: we acknowledge it by writing only that bit, never by reading
     * the word and writing it back -- a read-modify-write here would clear
     * every other pending event too. */
    const uint32_t st = hal_read32(REG_STATUS);
    if ((st & STAT_LOS) != 0u) {
        hal_w1c(REG_STATUS, STAT_LOS);
        L->tm.los_events++;
        if (L->state != LS_WAIT_SIGNAL && L->state != LS_RESET) {
            enter(L, LS_FAULT);
            return;
        }
    }

    const uint32_t tmo = STATE_TIMEOUT_MS[L->state] * g_tmo_scale;
    const bool expired = (tmo != 0u) &&
                         fw_deadline_passed(L->entered_ms, now_ms, tmo);

    switch (L->state) {

    case LS_RESET:
        /* Pulse reset, then bring the lane out of it. Two separate writes;
         * `volatile` is what stops the compiler folding them into one. */
        hal_write32(REG_CTRL, CTRL_RESET);
        hal_write32(REG_CTRL, CTRL_EN | CTRL_TX_EN | CTRL_RX_EN);
        hal_write32(REG_ADAPT_CTRL, 0u);              /* every loop frozen */
        /* Give the CTLE its opening bid before anything else runs. This is
         * how the chicken-and-egg gets broken: the CDR needs an open eye to
         * lock and the equaliser needs correct sampling to adapt, so the
         * ANALOGUE equaliser -- which needs neither -- goes first and opens
         * the eye enough for the CDR to find a phase. */
        /* A FIXED PROBE POINT WHILE THE AGC MEASURES THE CHANNEL.
         *
         * The CTLE has to be somewhere before the AGC runs, and what it is set
         * to changes the amplitude the AGC sees -- so if it varied per retry,
         * the VGA code the AGC converges to would mean something different
         * every attempt. Pinning it here makes that code a REPEATABLE measure
         * of the channel, which is what LS_AGC then uses to choose the real
         * operating point. Measure first, decide second. */
        hal_critical_enter();
        hal_field_set(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT, CTLE_PROBE);
        hal_field_set(REG_CDR_CTRL, CDR_H1_MASK, CDR_H1_SHIFT, 13u);
        hal_critical_exit();
        fw_agc_set_target(L->retries);   /* search a new amplitude each retry */
        fw_agc_reset();
        fw_adapt_reset();
        fw_telem_reset();
        /* Start the reference PLL. NOTHING else can begin until it locks --
         * a CDR cannot recover a clock when there is no clock to recover
         * against, and the AGC would be measuring an unclocked datapath. */
        hal_write32(REG_PLL_CTRL, PLL_EN);
        enter(L, LS_PLL_LOCK);
        break;

    case LS_PLL_LOCK:
        /* A charge-pump PLL settles in a time set by its loop bandwidth, and
         * it can fail outright -- wrong divider, absent reference, VCO out of
         * band. So this WAITS with a timeout; it never assumes. */
        if ((st & STAT_PLL_LOCK) != 0u) {
            enter(L, LS_WAIT_SIGNAL);
        } else if (expired) {
            L->tm.timeouts[LS_PLL_LOCK]++;
            enter(L, LS_FAULT);
        }
        break;

    case LS_WAIT_SIGNAL:
        if ((st & STAT_SIGDET) != 0u) {
            hal_critical_enter();
            hal_bit_write(REG_ADAPT_CTRL, ADAPT_AGC_EN, true);
            hal_critical_exit();
            enter(L, LS_AGC);
        } else if (expired) {
            L->tm.timeouts[LS_WAIT_SIGNAL]++;
            enter(L, LS_FAULT);
        }
        break;

    case LS_AGC:
        /* Only the AGC runs here. The equaliser would otherwise be solving
         * for a gain that is still moving underneath it. */
        L->tm.agc_updates++;
        if (fw_agc_step()) {
            /* NOTE: this used to raise STAT_AGC_CONV through the hardware-
             * side backend. That was firmware reaching around its own register
             * interface to drive a bit the map declares RO -- on silicon that
             * entry point does not exist and this file would not link against
             * a production HAL. It was also pointless: nothing read the bit.
             * Convergence is reported by fw_agc_converged(), which is what the
             * state machine actually uses.
             *
             * CI now greps for it, because a claim that is only written down
             * in a header is a claim that drifts. */
            fw_set_front_end(L);
            /* Enable the CDR AND the equaliser together.
             *
             * A Mueller-Muller TED balances the first pre- and post-cursor, so
             * on a MINIMUM-PHASE channel -- precursor near zero, postcursor
             * large -- its S-curve carries a DC offset until the equaliser has
             * made the pulse response roughly symmetric about the cursor.
             * Frozen taps therefore mean a permanently biased TED and an
             * integrator that winds up rather than settles.
             *
             * So these two must converge JOINTLY. What keeps them from
             * fighting is bandwidth, not ordering: the CDR closes every symbol
             * at 100 GBd while the taps update once per block at kHz, five
             * orders of magnitude apart. */
            /* Step size, sized from the numbers rather than guessed: the gradient
             * accumulates to ~500 per block, and TAP_APPLY_SHIFT is 14, so the
             * accumulator needs 16384 to move ONE applied tap code. At
             * mu_shift 8 that is 16384 blocks per code -- the link would come
             * "up" with the taps untouched, which is exactly what happened. */
            fw_adapt_set_gear(EQ_GEAR_ACQ, 0u);
            hal_critical_enter();
            hal_bit_write(REG_ADAPT_CTRL, ADAPT_CDR_EN, true);
            hal_bit_write(REG_ADAPT_CTRL, ADAPT_FFE_EN, true);
            hal_bit_write(REG_ADAPT_CTRL, ADAPT_DFE_EN, true);
            hal_critical_exit();
            enter(L, LS_CDR_LOCK);
        } else if (expired) {
            L->tm.timeouts[LS_AGC]++;
            enter(L, LS_FAULT);
        }
        break;

    case LS_CDR_LOCK:
        /* FREEZE-AND-HOLD: nothing else adapts in this state.
         *
         * The AGC used to keep stepping here, on the theory that it is much
         * faster than the CDR and therefore could not interfere. Measurement
         * said otherwise: the VGA code drifted 30 -> 39 during acquisition,
         * the signal amplitude moved with it, and because TED gain scales
         * with amplitude the CDR was being detuned while trying to lock.
         * Exactly one loop adapts at a time during bring-up. */
        (void)fw_adapt_step();        /* taps open the eye the CDR needs */
        if ((st & STAT_CDR_LOCK) != 0u) {
            L->train_errs = 0u;
            L->train_syms = 0u;
            (void)hal_read_clear(REG_ERR_CNT);   /* start the count clean */
            enter(L, LS_EQ_TRAIN);
        } else if (expired) {
            L->tm.timeouts[LS_CDR_LOCK]++;
            enter(L, LS_FAULT);
        }
        break;

    case LS_EQ_TRAIN:
        L->tm.tap_updates++;
        /* MINIMUM TRAINING TIME. A "settled" test alone is not enough: early
         * in training the taps are barely moving simply because they have not
         * started, and a quiet-taps detector cannot tell that apart from
         * genuine convergence. Real link-training specs mandate a minimum
         * duration for the same reason. Do not accept convergence before the
         * loop has had time to act. */
        if (fw_adapt_step() &&
            fw_deadline_passed(L->entered_ms, now_ms, EQ_MIN_TRAIN_MS)) {
            /* Shift down a gear BEFORE judging: finer step, leakage on to stop
             * the taps drifting when the gradient stops being informative. The
             * acquisition gear is deliberately coarse, so the error rate while
             * it is engaged is NOT the error rate the link will run at -- a
             * factor of a hundred separated the two here. */
            fw_adapt_set_gear(EQ_GEAR_TRACK, 14u);
            L->train_errs = 0u;
            L->train_syms = 0u;
            (void)hal_read_clear(REG_ERR_CNT);   /* start the count clean */
            enter(L, LS_EQ_VERIFY);
        } else if (expired) {
            L->tm.timeouts[LS_EQ_TRAIN]++;
            enter(L, LS_FAULT);
        }
        break;

    case LS_EQ_VERIFY:
        /* MEASURE THE THING THE LINK IS FOR.
         *
         * Every test before this one asks whether a loop has stopped moving.
         * None of them asks whether the answer it stopped at is any good, and
         * a loop settles perfectly happily on a bad one: the CDR can hold a
         * rock-steady phase at the wrong point in the eye, and the taps will
         * then converge to the best filter for that wrong phase and report
         * success. Measured before this state existed, at 12 dB: every loop
         * converged, every status bit green, pre-FEC BER 7.6e-2.
         *
         * The transmitter is still sending the known training pattern here, so
         * the hardware is already counting real errors against it. That counter
         * is the only end-to-end measurement available before traffic starts,
         * and it is worth a state of its own. */
        L->tm.tap_updates++;
        (void)fw_adapt_step();
        L->train_errs += hal_read_clear(REG_ERR_CNT);
        L->train_syms += hal_read32(REG_SYM_CNT);
        /* The AGC runs here too, because it runs in TRACK and in UP. A loop
         * that is frozen during verification and live afterwards means the
         * operating point that was measured is not the operating point that
         * ships. Read the counters BEFORE this call -- fw_agc_step drains
         * REG_SYM_CNT, and a read-clear register has exactly one consumer. */
        (void)fw_agc_step();

        if (L->train_syms >= EQ_JUDGE_SYMS) {
            const uint64_t bits = L->train_syms * (uint64_t)BITS_PER_SYMBOL;
            if (L->train_errs * 100000u > bits * (uint64_t)EQ_MAX_TRAIN_BER_E5) {
                /* Converged on an answer the FEC could not carry. Go back and
                 * try another point in the search rather than bring the link up
                 * on it. */
                L->tm.eq_rejected++;
                enter(L, LS_FAULT);
            } else {
                enter(L, LS_TRACK);
            }
        } else if (expired) {
            L->tm.timeouts[LS_EQ_VERIFY]++;
            enter(L, LS_FAULT);
        }
        break;

    case LS_TRACK:
        L->tm.tap_updates++;
        (void)fw_agc_step();
        (void)fw_adapt_step();
        if (fw_adapt_converged() && fw_agc_converged()) {
            L->tm.ms_to_up = now_ms;
            enter(L, LS_UP);
        } else if (expired) {
            L->tm.timeouts[LS_TRACK]++;
            enter(L, LS_FAULT);
        }
        break;

    case LS_UP:
        /* Steady state: harvest telemetry, THEN keep the loops tracking.
         *
         * Order matters. A read-and-clear register can have exactly ONE
         * consumer: fw_agc_step() also drains REG_SYM_CNT for its average, so
         * reading telemetry after it returned zero symbols and zero errors
         * forever. Either read it first, or give each consumer its own
         * counter. Shared read-clear state is a design smell in a register
         * map for precisely this reason. */
        L->tm.symbols    += hal_read32(REG_SYM_CNT);
        /* REG_ERR_CNT only counts while the hardware is given a training
         * symbol to compare against, and there is none in this state. So this
         * is drained to keep the read-clear register from saturating, and its
         * value is deliberately NOT accumulated into a BER: in LS_UP it is
         * always zero, and a counter that cannot be non-zero is a metric that
         * lies. The error rate that means something in traffic is measured by
         * the PCS against a pattern-aligned reference. */
        (void)hal_read_clear(REG_ERR_CNT);
        (void)fw_agc_step();
        (void)fw_adapt_step();
        /* Telemetry is a BACKGROUND task: one frame per tick at most, and
         * only when the FIFO has room. It must never delay a control loop. */
        fw_telem_step(L);
        /* LOSS OF LOCK NEEDS HYSTERESIS. The CDR lock flag is a live
         * indication and it dips momentarily under noise even on a perfectly
         * healthy link -- measured here, the link came up, emitted exactly one
         * telemetry frame, then tore itself down on a single dropped sample.
         * Requiring a SUSTAINED loss is the difference between a glitch and an
         * outage.
         *
         * PLL lock is not the same case. If the clock is gone then nothing
         * downstream means anything, so that one is immediate. */
        if ((st & STAT_PLL_LOCK) == 0u) {
            enter(L, LS_FAULT);
        } else if ((st & STAT_CDR_LOCK) == 0u) {
            L->unlock_ticks++;
            if (L->unlock_ticks >= LOSS_OF_LOCK_TICKS) {
                enter(L, LS_FAULT);
            }
        } else {
            L->unlock_ticks = 0u;
        }
        break;

    case LS_FAULT:
        L->tm.faults++;
        hal_write32(REG_ADAPT_CTRL, 0u);        /* freeze everything */
        if (fw_deadline_passed(L->entered_ms, now_ms, L->backoff_ms)) {
            /* Exponential backoff, capped. Retrying immediately and forever
             * turns a marginal link into a hot loop that also makes the fault
             * harder to diagnose. */
            L->backoff_ms = (L->backoff_ms < 512u) ? (L->backoff_ms * 2u) : 512u;
            L->retries++;
            enter(L, LS_RESET);
        }
        break;

    default:
        enter(L, LS_FAULT);
        break;
    }
}





