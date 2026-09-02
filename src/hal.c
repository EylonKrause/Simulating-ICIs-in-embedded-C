/* ===========================================================================
 *  hal.c — the mock MMIO backend.
 *
 *  On real silicon REG_FILE would be a pointer to a peripheral aperture:
 *      static volatile uint32_t * const REG_FILE = (volatile uint32_t *)0x40001000u;
 *  Here it is a plain array, still declared `volatile`, so the firmware source
 *  is byte-identical between the two. That substitution is the entire trick
 *  behind testing firmware without hardware.
 *
 *  `volatile` is not decoration. Without it the compiler is entitled to cache
 *  a register in a CPU register and never re-read it -- a polling loop like
 *      while (hal_read32(REG_STATUS) & STAT_CDR_LOCK) { }
 *  compiles to an infinite loop, or vanishes entirely. It is also entitled to
 *  delete a "redundant" write, so a reset pulse (write 1, write 0) becomes a
 *  single write and never reaches the pin. volatile forbids both.
 *
 *  What volatile does NOT give: atomicity, ordering against non-volatile
 *  accesses, or cache coherency. Hence hal_critical_enter/exit.
 * =========================================================================*/
#include "hal.h"

#include <string.h>

#define REG_SPACE_BYTES   0x100u
#define REG_COUNT         (REG_SPACE_BYTES / 4u)

static volatile uint32_t REG_FILE[REG_COUNT];
static uint32_t          g_w1c_map[REG_COUNT];   /* which bits are W1C       */
static unsigned          g_crit_depth;
static hal_stats_t       g_stats;

static void hal_maybe_preempt(void);   /* defined below, used by field_set */

static hal_wr_hook_fn g_wr_hook;
void hal_set_write_hook(hal_wr_hook_fn fn) { g_wr_hook = fn; }

static hal_isr_fn g_isr;
static void      *g_isr_ctx;
static unsigned   g_isr_pending;
static uint32_t   g_isr_runs;
static uint32_t   g_isr_deferred;
static unsigned   g_in_isr;

static size_t idx_of(uint32_t off)
{
    return (size_t)((off & (REG_SPACE_BYTES - 1u)) >> 2);
}

void hal_reset_all(void)
{
    for (size_t i = 0; i < REG_COUNT; ++i) {
        REG_FILE[i] = 0u;
        g_w1c_map[i] = 0u;
    }
    g_w1c_map[idx_of(REG_STATUS)]   = STATUS_W1C_MASK;
    g_w1c_map[idx_of(REG_ERR_CNT)]  = 0xFFFFFFFFu;
    g_w1c_map[idx_of(REG_SYM_CNT)]  = 0xFFFFFFFFu;
    g_w1c_map[idx_of(REG_AMP_ACC)]  = 0xFFFFFFFFu;
    g_w1c_map[idx_of(REG_EYE_ERR)]  = 0xFFFFFFFFu;
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        g_w1c_map[idx_of(REG_GRAD_ACC(i))] = 0xFFFFFFFFu;
    }
    for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
        g_w1c_map[idx_of(REG_DFE_GRAD(i))] = 0xFFFFFFFFu;
    }
    REG_FILE[idx_of(REG_LANE_ID)] = 0x00010200u;   /* id 0x0200, rev 1 */
    g_crit_depth = 0u;
    g_isr_pending = 0u;
    /* NOTE: the write hook and any attached ISR are PLATFORM WIRING, not
     * register state. A register reset must not tear down the bus fabric --
     * doing so silently disconnected the management FIFO when fw_init() ran
     * after the platform was attached. */
    memset(&g_stats, 0, sizeof(g_stats));
}

/* ---- primitive accessors ------------------------------------------------- */
uint32_t hal_read32(uint32_t off)
{
    g_stats.reads++;
    return REG_FILE[idx_of(off)];
}

void hal_write32(uint32_t off, uint32_t val)
{
    const size_t i = idx_of(off);
    g_stats.writes++;

    if (g_wr_hook != NULL) {
        g_wr_hook(off, val);      /* registers with side effects */
    }

    const uint32_t w1c = g_w1c_map[i];
    if (w1c != 0u) {
        /* W1C semantics: a 1 clears, a 0 leaves alone. Bits outside the W1C
         * mask keep normal RW behaviour. */
        const uint32_t cur = REG_FILE[i];
        REG_FILE[i] = (cur & ~(val & w1c)) | (val & ~w1c);
    } else {
        REG_FILE[i] = val;
    }
}

/* ---- fields -------------------------------------------------------------- */
uint32_t hal_field_get(uint32_t off, uint32_t mask, unsigned shift)
{
    return (hal_read32(off) & mask) >> shift;
}

void hal_field_set(uint32_t off, uint32_t mask, unsigned shift, uint32_t val)
{
    g_stats.rmw++;
    if (g_crit_depth == 0u) {
        g_stats.unguarded_rmw++;
    }
    if (g_w1c_map[idx_of(off)] != 0u) {
        /* Read-modify-write on a register containing W1C bits acknowledges
         * events nobody handled. Counted so a test can assert it never
         * happens; on real silicon this is a silent, intermittent bug. */
        g_stats.w1c_rmw_bugs++;
    }

    const uint32_t cur = hal_read32(off);

    /* <-- PREEMPTION POINT. Everything the hazard is about happens here. */
    hal_maybe_preempt();

    const uint32_t nxt = (cur & ~mask) | ((val << shift) & mask);
    hal_write32(off, nxt);
}

void hal_bit_write(uint32_t off, uint32_t bitmask, bool on)
{
    g_stats.rmw++;
    if (g_crit_depth == 0u) {
        g_stats.unguarded_rmw++;
    }
    if (g_w1c_map[idx_of(off)] != 0u) {
        g_stats.w1c_rmw_bugs++;
    }
    const uint32_t cur = hal_read32(off);
    hal_maybe_preempt();
    hal_write32(off, on ? (cur | bitmask) : (cur & ~bitmask));
}

void hal_w1c(uint32_t off, uint32_t bits)
{
    /* No read. Write only the bits being acknowledged. */
    hal_write32(off, bits);
}

uint32_t hal_read_clear(uint32_t off)
{
    const uint32_t v = hal_read32(off);
    hal_write32(off, v);          /* W1C: writing what we read clears it */
    return v;
}

/* ---- signed fields ------------------------------------------------------- */
void hal_write_signed(uint32_t off, int32_t val, unsigned bits)
{
    const uint32_t mask = (bits >= 32u) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    hal_write32(off, (uint32_t)val & mask);
}

int32_t hal_read_signed(uint32_t off, unsigned bits)
{
    const uint32_t mask = (bits >= 32u) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    uint32_t raw = hal_read32(off) & mask;
    /* Sign-extend from `bits` wide. The shift pair is the portable idiom;
     * relying on >> of a negative signed value is implementation-defined. */
    const uint32_t sign = 1u << (bits - 1u);
    if ((raw & sign) != 0u) {
        raw |= ~mask;
    }
    return (int32_t)raw;
}

/* ---- interrupts ---------------------------------------------------------- */

void hal_attach_isr(hal_isr_fn fn, void *ctx)
{
    g_isr = fn;
    g_isr_ctx = ctx;
    g_isr_pending = 0u;
    g_isr_runs = 0u;
    g_isr_deferred = 0u;
}

void hal_detach_isr(void) { g_isr = NULL; g_isr_ctx = NULL; }
uint32_t hal_isr_runs(void)     { return g_isr_runs; }
uint32_t hal_isr_deferred(void) { return g_isr_deferred; }

/* The preemption point. Called from inside hal_field_set(), between the read
 * and the write -- the exact instant a real interrupt would land. */
static void hal_maybe_preempt(void)
{
    if (g_isr == NULL || g_in_isr) {
        return;
    }
    if (g_crit_depth != 0u) {
        g_isr_pending++;          /* masked: it will run on critical_exit */
        g_isr_deferred++;
        return;
    }
    g_in_isr = 1u;
    g_isr_runs++;
    g_isr(g_isr_ctx);
    g_in_isr = 0u;
}

/* ---- critical section ---------------------------------------------------- */
void hal_critical_enter(void) { g_crit_depth++; }        /* __disable_irq() */

void hal_critical_exit(void)
{
    if (g_crit_depth) {
        g_crit_depth--;
    }
    /* Replay anything that arrived while interrupts were masked. Real silicon
     * does this in hardware: the interrupt stays pending in the NVIC and fires
     * the moment PRIMASK clears. */
    while (g_crit_depth == 0u && g_isr_pending > 0u && g_isr != NULL && !g_in_isr) {
        g_isr_pending--;
        g_in_isr = 1u;
        g_isr_runs++;
        g_isr(g_isr_ctx);
        g_in_isr = 0u;
    }
}
unsigned hal_critical_depth(void) { return g_crit_depth; }

/* ---- hardware side ------------------------------------------------------- */
void hw_reg_set(uint32_t off, uint32_t val) { REG_FILE[idx_of(off)] = val; }
uint32_t hw_reg_get(uint32_t off)           { return REG_FILE[idx_of(off)]; }

void hw_status_set(uint32_t bits)
{
    REG_FILE[idx_of(REG_STATUS)] |= bits;
}

const hal_stats_t *hal_stats(void) { return &g_stats; }
void hal_stats_reset(void) { memset(&g_stats, 0, sizeof(g_stats)); }
