#include "pll.h"
#include "hal.h"

#include <string.h>

void pll_init(pll_t *p, uint32_t settle_blocks)
{
    memset(p, 0, sizeof(*p));
    p->settle_blocks = settle_blocks;
}

void pll_reset(pll_t *p)
{
    p->locked  = 0u;
    p->elapsed = 0u;
}

void pll_force_fail(pll_t *p, unsigned fail)
{
    p->force_fail = fail;
    if (fail) {
        pll_reset(p);
    }
}

void pll_step(pll_t *p)
{
    const uint32_t ctrl = hal_read32(REG_PLL_CTRL);
    const unsigned en   = ((ctrl & PLL_EN) != 0u);

    if (!en) {
        /* Disabled: lock drops immediately. Everything downstream must treat
         * loss of PLL lock as fatal and restart, because its clock is gone. */
        p->enabled = 0u;
        pll_reset(p);
    } else {
        if (!p->enabled) {
            p->enabled = 1u;
            pll_reset(p);          /* rising edge of enable restarts settling */
        }
        if (!p->force_fail) {
            if (p->elapsed < p->settle_blocks) {
                p->elapsed++;
            } else {
                p->locked = 1u;
            }
        }
    }

    /* Bypass is an escape hatch used on real parts when an external clock is
     * supplied directly. It reports locked without waiting. */
    if ((ctrl & PLL_BYPASS) != 0u) {
        p->locked = 1u;
    }

    hw_reg_set(REG_PLL_STAT, (p->locked ? PLL_STAT_LOCKED : 0u) | (p->elapsed << 8));

    uint32_t st = hal_read32(REG_STATUS) & ~STAT_PLL_LOCK;
    if (p->locked) {
        st |= STAT_PLL_LOCK;
    }
    hw_reg_set(REG_STATUS, st);
}
