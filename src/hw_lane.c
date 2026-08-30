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
         * type-2 loop -- a type-1 loop would accumulate phase error forever. */
        L->cdr.phase += L->ppm_offset * 1e-6 * (double)OSR;
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
