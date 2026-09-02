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

int hw_lane_init(hw_lane_t *L, double il_db, afe_mode_t mode, double ppm)
{
    memset(L, 0, sizeof(*L));
    if (channel_build(&L->ch, il_db, SPAN_UI) != 0) {
        return -1;
    }
    tx_init(&L->tx, 0xC0FFEEu, -0.10, -0.18);
    afe_init(&L->afe, mode);
    eq_init(&L->eq);
    /* Loop constants chosen an order of magnitude apart from the equaliser's
     * update rate -- see the bandwidth-separation note in fw_bringup.c. */
    /* Type-2 gains. ki sets the PULL-IN RANGE: the integrator has to reach
     * ppm*1e-6*OSR samples per symbol (1.9e-3 at 120 ppm) before the loop can
     * hold phase. Too small and the phase slews right past lock, the TED
     * sweeps its whole S-curve, and its mean goes to zero for the wrong
     * reason. Damping zeta = kp/(2*sqrt(ki)) ~ 0.6 here. */
    cdr_init(&L->cdr, 1.2e-2, 1.0e-4);

    L->buf_samples = (size_t)HW_BLOCK_SYMS * OSR;
    L->osr_buf = (real_t *)calloc(L->buf_samples, sizeof(real_t));
    L->rx_buf  = (real_t *)calloc(L->buf_samples, sizeof(real_t));
    if (L->osr_buf == NULL || L->rx_buf == NULL) {
        hw_lane_free(L);
        return -1;
    }
    L->rng            = 0x1234ABCDu;
    L->ppm_offset     = ppm;
    L->signal_present = 1u;
    return 0;
}

void hw_lane_free(hw_lane_t *L)
{
    channel_free(&L->ch);
    free(L->osr_buf);
    free(L->rx_buf);
    L->osr_buf = NULL;
    L->rx_buf  = NULL;
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
    tia_set_code (&L->afe.tia,  hal_field_get(REG_AFE_TIA,  TIA_GAIN_MASK,  TIA_GAIN_SHIFT));
    vga_set_code (&L->afe.vga,  hal_field_get(REG_AFE_VGA,  VGA_GAIN_MASK,  VGA_GAIN_SHIFT));
    ctle_set_code(&L->afe.ctle, hal_field_get(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT));
}

void hw_lane_run(hw_lane_t *L, unsigned training)
{
    /* The reference PLL runs regardless of everything else: it is the clock
     * the rest of the lane depends on, and nothing downstream is meaningful
     * until it has locked. The management bus drains at its own fixed rate,
     * likewise independent of what the data path is doing. */
    pll_step(&L->pll);
    mgmt_bus_tick();

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

    static unsigned syms[HW_BLOCK_SYMS];

    /* ---- transmit and propagate ---------------------------------------- */
    for (unsigned n = 0; n < HW_BLOCK_SYMS; ++n) {
        unsigned s;
        const real_t lvl = tx_step(&L->tx, &s);
        syms[n] = s;
        tx_upsample(L->signal_present ? lvl : (real_t)0.0,
                    &L->osr_buf[(size_t)n * OSR]);
    }
    channel_apply(&L->ch, L->osr_buf, L->rx_buf, L->buf_samples);

    /* ---- analogue front end, sample by sample --------------------------- */
    for (size_t i = 0; i < L->buf_samples; ++i) {
        L->rx_buf[i] = afe_step(&L->afe, L->rx_buf[i], (real_t)gauss(&L->rng));
    }

    /* ---- CDR sampling, equaliser, accumulation -------------------------- */
    const unsigned adapt = hal_read32(REG_ADAPT_CTRL);
    for (unsigned n = 1; n + 1u < HW_BLOCK_SYMS; ++n) {
        const real_t y_in = cdr_sample(&L->cdr, L->rx_buf, L->buf_samples, n);

        unsigned sym;
        const real_t y = eq_step(&L->eq, y_in,
                                 training ? syms[n] : 0xFFFFFFFFu, &sym);

        if ((adapt & ADAPT_CDR_EN) != 0u) {
            cdr_update(&L->cdr, y, pam4_level(sym));
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
}

void hw_lane_capture_eye(hw_lane_t *L, struct eye_s *eye)
{
    hw_apply_afe_regs(L);
    eq_refresh_taps(&L->eq);

    static unsigned syms[HW_BLOCK_SYMS];
    for (unsigned n = 0; n < HW_BLOCK_SYMS; ++n) {
        unsigned s;
        const real_t lvl = tx_step(&L->tx, &s);
        syms[n] = s;
        tx_upsample(lvl, &L->osr_buf[(size_t)n * OSR]);
    }
    (void)syms;
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
