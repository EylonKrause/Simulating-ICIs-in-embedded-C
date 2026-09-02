/* ===========================================================================
 *  fw.h — the firmware. Fixed point only; no float below this line.
 *
 *  JD: "Design and implement real-time embedded C/C++ code for data path
 *       control, management, and telemetry."
 *  JD: "Implement and debug firmware-based adaptation algorithms for
 *       equalizer tap updates and gain control (VGA/TIA)."
 *
 *  Three subsystems, three timescales -- and the separation is deliberate:
 *
 *    fw_agc     amplitude only. Fastest loop.
 *    fw_adapt   equaliser taps. Slowest loop, by an order of magnitude.
 *    fw_bringup sequences them, and owns every timeout and retry.
 *
 *  BANDWIDTH SEPARATION IS THE WHOLE DESIGN. AGC, the CDR and the equaliser
 *  all derive their error from the same signal. Two loops with similar
 *  bandwidth will fight: the AGC will chase gain error the equaliser just
 *  created, the equaliser will re-solve for a gain the AGC is about to change,
 *  and the pair will oscillate without either ever being "wrong". The fixes,
 *  all present in this code:
 *
 *    1. time constants an order of magnitude apart
 *    2. sequencing -- AGC converges, then CDR locks, then taps adapt
 *    3. freeze-and-hold: loops not being trained are disabled, not just slow
 *    4. gear shifting: a large step during acquisition, small during tracking
 *
 *  And the chicken-and-egg worth naming out loud: the CDR needs an open eye to
 *  lock, the equaliser needs correct sampling to converge. Bring-up breaks it
 *  by training on a known pattern where the eye is open before equalisation.
 * =========================================================================*/
#ifndef FW_H
#define FW_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LS_RESET = 0,
    LS_PLL_LOCK,          /* nothing else can start until the clock exists */
    LS_WAIT_SIGNAL,
    LS_AGC,
    LS_CDR_LOCK,
    LS_EQ_TRAIN,
    LS_TRACK,
    LS_UP,
    LS_FAULT,
    LS_COUNT
} link_state_t;

/* Telemetry. JD: "data path control, management, and telemetry." */
typedef struct {
    uint32_t state_entries[LS_COUNT];
    uint32_t timeouts[LS_COUNT];
    uint32_t transitions;
    uint32_t faults;
    uint32_t los_events;
    uint32_t agc_updates;
    uint32_t tap_updates;
    uint32_t ms_to_up;            /* bring-up time, the headline number */
    uint64_t symbols;
    uint64_t bit_errors;
} fw_telemetry_t;

typedef struct {
    link_state_t   state;
    uint32_t       entered_ms;
    uint32_t       now_ms;
    uint32_t       retries;
    uint32_t       backoff_ms;
    uint32_t       unlock_ticks;   /* consecutive ticks with CDR unlocked */
    fw_telemetry_t tm;
} fw_link_t;

void        fw_init(fw_link_t *L);
void        fw_tick(fw_link_t *L, uint32_t now_ms);
const char *fw_state_name(link_state_t s);
bool        fw_is_up(const fw_link_t *L);

/* ---- subsystems, exposed so unit tests can drive them in isolation ------- */
void fw_agc_reset(void);
void fw_agc_set_target(unsigned attempt);
int32_t fw_agc_target(void);
int  fw_agc_step(void);                 /* 1 when the amplitude is on target */
int  fw_agc_converged(void);

void fw_adapt_reset(void);
void fw_adapt_set_gear(unsigned mu_shift, unsigned leak_shift);
int  fw_adapt_step(void);               /* 1 when the taps have settledint32_t fw_adapt_activity(void);        /* last block's summed |gradient|     */

void     fw_telem_reset(void);
void     fw_telem_step(const fw_link_t *L);
uint32_t fw_telem_frames(void);
uint32_t fw_telem_deferred(void);
int  fw_adapt_converged(void);
int32_t fw_adapt_tap(unsigned i);       /* accumulator value, for tests      */
int32_t fw_adapt_activity(void);        /* last block's summed |gradient|      */

/* ---- the one timing rule that matters ----------------------------------- */
/* Unsigned subtraction is correct ACROSS THE WRAP of the millisecond counter;
 * `now >= start + timeout` is not, and fails once every 49 days on a 32-bit
 * millisecond tick. Always write it this way. */
static inline bool fw_deadline_passed(uint32_t start, uint32_t now, uint32_t timeout)
{
    return (uint32_t)(now - start) >= timeout;
}

#endif /* FW_H */
