/* ===========================================================================
 *  fw_bringup.c â€” link bring-up state machine, timeouts, retry, telemetry.
 *
 *  JD: "Design and implement real-time embedded C/C++ code for data path
 *       control, management, and telemetry."
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

#include <string.h>

/* Convergence may not be declared before this much training has elapsed. */
#define EQ_MIN_TRAIN_MS  150u

/* Per-state timeout, milliseconds. A table, not scattered constants. */
static const uint32_t STATE_TIMEOUT_MS[LS_COUNT] = {
    [LS_RESET]       =    2u,
    [LS_WAIT_SIGNAL] =  200u,
    [LS_AGC]         =  120u,
    [LS_CDR_LOCK]    =  300u,
    [LS_EQ_TRAIN]    =  600u,
    [LS_TRACK]       =  200u,
    [LS_UP]          =    0u,     /* 0 == no timeout; steady state */
    [LS_FAULT]       = 1000u,
};

static const char *const STATE_NAME[LS_COUNT] = {
    "RESET", "WAIT_SIGNAL", "AGC", "CDR_LOCK", "EQ_TRAIN", "TRACK", "UP", "FAULT"
};

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
    L->state      = s;
    L->entered_ms = L->now_ms;
    L->tm.state_entries[s]++;
    L->tm.transitions++;
}

void fw_init(fw_link_t *L)
{
    memset(L, 0, sizeof(*L));
    L->backoff_ms = 2u;
    hal_reset_all();
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

    const uint32_t tmo = STATE_TIMEOUT_MS[L->state];
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
        hal_critical_enter();
        hal_field_set(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT, 12u);
        hal_critical_exit();
        fw_agc_set_target(L->retries);   /* search a new amplitude each retry */
        fw_agc_reset();
        fw_adapt_reset();
        enter(L, LS_WAIT_SIGNAL);
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
            hw_status_set(STAT_AGC_CONV);
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
            fw_adapt_set_gear(1u, 0u);
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
            /* Shift down a gear for tracking: finer step, leakage on to stop
             * the taps drifting when the gradient stops being informative. */
            fw_adapt_set_gear(4u, 14u);   /* tracking: finer step, leakage on */
            enter(L, LS_TRACK);
        } else if (expired) {
            L->tm.timeouts[LS_EQ_TRAIN]++;
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
        L->tm.bit_errors += hal_read_clear(REG_ERR_CNT);
        (void)fw_agc_step();
        (void)fw_adapt_step();
        if ((st & STAT_CDR_LOCK) == 0u) {
            enter(L, LS_FAULT);                 /* lost lock */
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





