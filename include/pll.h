/* ===========================================================================
 *  pll.h — the reference PLL that clocks the whole lane.
 *
 *  JD: "...manages critical power-on, reset, and calibration sequences..."
 *
 *  This is the first thing that has to come up and the one every other loop
 *  depends on. A CDR cannot recover a clock if there is no clock to recover
 *  against, so bring-up order is:
 *
 *      RESET -> PLL LOCK -> signal detect -> AGC -> CDR -> equaliser
 *
 *  Lock is not instantaneous and it is not guaranteed. A charge-pump PLL takes
 *  a settling time set by its loop bandwidth (microseconds to tens of
 *  microseconds), and it can fail to lock entirely -- wrong divider, reference
 *  absent, VCO out of band. So the firmware must WAIT with a timeout, not
 *  assume. `pll_force_fail()` exists so the failure path is testable.
 * =========================================================================*/
#ifndef PLL_H
#define PLL_H

#include <stdint.h>

typedef struct {
    unsigned enabled;
    unsigned locked;
    uint32_t settle_blocks;   /* blocks from enable to lock */
    uint32_t elapsed;
    unsigned force_fail;      /* injected fault, for the timeout path */
} pll_t;

void pll_init(pll_t *p, uint32_t settle_blocks);
void pll_reset(pll_t *p);
void pll_force_fail(pll_t *p, unsigned fail);

/* One block of time. Reads REG_PLL_CTRL, drives REG_PLL_STAT and the
 * STAT_PLL_LOCK bit of REG_STATUS. */
void pll_step(pll_t *p);

#endif /* PLL_H */
