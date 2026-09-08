/* ===========================================================================
 *  test_all.c -- unit tests. Runs with no hardware.
 *
 *  THIS IS THE POINT OF THE HAL. The firmware talks to the datapath only
 *  through hal_read32/hal_write32, so in CI we swap the backing store for a
 *  behavioural model and the adaptation loop closes against a simulated plant.
 *  No silicon, no FPGA, no bench -- and it runs in milliseconds on every push.
 *
 *  Note the difference between mocking REGISTERS and modelling the PLANT.
 *  Asserting that a write followed by a read returns the same value tests
 *  nothing about the algorithm. test_adapt_closes_loop() below drives real
 *  gradients into the accumulator registers and asserts the firmware's taps
 *  converge to the right answer -- that is a test that can actually fail when
 *  someone breaks the adaptation.
 * =========================================================================*/
#include "fixed.h"
#include "pll.h"
#include "mgmt.h"
#include "hal.h"
#include "fw.h"
#include "tx.h"
#include "channel.h"
#include "fft.h"
#include "fec.h"
#include "pcs.h"
#include "cdr.h"
#include "touchstone.h"
#include "eq.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail;
static int g_run;

#define CHECK(cond, msg) do {                                   \
    g_run++;                                                    \
    if (!(cond)) { printf("  FAIL  %s\n", (msg)); g_fail++; }   \
    else         { printf("  ok    %s\n", (msg)); }             \
} while (0)

/* ---------------------------------------------------------------- fixed.h */
static void test_fixed(void)
{
    printf("\nfixed-point\n");
    CHECK(q15_mul(16384, 16384) == 8192, "Q15 0.5*0.5 = 0.25");
    CHECK(q15_mul(-32768, -32768) == 32767, "Q15 -1*-1 saturates, does not wrap");
    CHECK(sat_add32(INT32_MAX, 1) == INT32_MAX, "saturating add clamps high");
    CHECK(sat_add32(INT32_MIN, -1) == INT32_MIN, "saturating add clamps low");

    /* Truncation drifts toward -inf; rounding does not. In an integrator that
     * bias accumulates without bound. */
long long bias_t = 0, bias_r = 0;
    long nops = 0;
    for (int i = -30000; i < 30000; i += 7) {
        const int32_t exact = (int32_t)i * 21845;                 /* Q1.30 */
        /* MULTIPLY, do not shift. Left-shifting a NEGATIVE signed value is
         * undefined behaviour in C -- these products are routinely negative.
         * MSVC compiled it silently; UBSan caught it on the first run. Same
         * family as 1 << 31 on a signed int. */
        bias_t += (long long)q15_mul_trunc((q15_t)i, 21845) * (1LL << Q15_FRAC) - exact;
        bias_r += (long long)q15_mul((q15_t)i, 21845) * (1LL << Q15_FRAC) - exact;
        nops++;
    }
    /* Measure the ERROR against the exact product, not the result -- summing
     * results just measures the input distribution's own asymmetry. */
    printf("        error sum over %ld ops (Q30 units, 1 LSB = 32768):\n"
           "          truncation %+lld   rounding %+lld\n", nops, bias_t, bias_r);
    CHECK(bias_t < -(nops * 8000L), "truncation drifts toward -inf, ~0.5 LSB per op");
    CHECK(llabs(bias_r) < llabs(bias_t) / 4, "rounding is far less biased");

    /* accumulate wide, apply narrow */
    CHECK(tap_publish(32 << TAP_APPLY_SHIFT) == 32, "tap_publish keeps the high bits");
    CHECK(tap_publish(1 << 30) == TAP_APPLY_MAX, "tap_publish saturates high");
    CHECK(tap_publish(-(1 << 30)) == TAP_APPLY_MIN, "tap_publish saturates low");
    CHECK(tap_leak(1 << 20, 4) < (1 << 20), "leakage bleeds a positive tap down");
    CHECK(tap_leak(-(1 << 20), 4) > -(1 << 20), "leakage bleeds a negative tap up");
    CHECK(tap_leak(12345, 0) == 12345, "leak shift 0 disables leakage");
}

/* -------------------------------------------------------------------- HAL */
static void test_hal(void)
{
    printf("\nHAL / register semantics\n");
    hal_reset_all();

    hal_write32(REG_AFE_VGA, 0xDEADBEEFu);
    CHECK(hal_read32(REG_AFE_VGA) == 0xDEADBEEFu, "plain RW register round-trips");

    /* field insert must not disturb neighbours */
    hal_write32(REG_ADAPT_CTRL, 0xFFFFFFFFu);
    hal_critical_enter();
    hal_field_set(REG_ADAPT_CTRL, ADAPT_MU_MASK, ADAPT_MU_SHIFT, 5u);
    hal_critical_exit();
    CHECK(hal_field_get(REG_ADAPT_CTRL, ADAPT_MU_MASK, ADAPT_MU_SHIFT) == 5u,
          "field_set writes the field");
    CHECK((hal_read32(REG_ADAPT_CTRL) & ADAPT_AGC_EN) != 0u,
          "field_set preserves neighbouring bits");

    /* oversized value must be masked, not allowed to spill */
    hal_write32(REG_ADAPT_CTRL, 0u);
    hal_critical_enter();
    hal_field_set(REG_ADAPT_CTRL, ADAPT_MU_MASK, ADAPT_MU_SHIFT, 0xFFFFu);
    hal_critical_exit();
    CHECK((hal_read32(REG_ADAPT_CTRL) & ~ADAPT_MU_MASK) == 0u,
          "oversized field value cannot corrupt neighbours");

    /* W1C: writing a 1 clears, writing a 0 leaves alone */
    hw_reg_set(REG_STATUS, STAT_LOS | STAT_ERR_OVF | STAT_SIGDET);
    hal_w1c(REG_STATUS, STAT_LOS);
    CHECK((hal_read32(REG_STATUS) & STAT_LOS) == 0u, "W1C: writing 1 clears the bit");
    CHECK((hal_read32(REG_STATUS) & STAT_ERR_OVF) != 0u,
          "W1C: an unwritten flag survives");

    /* signed tap fields sign-extend correctly */
    hal_write_signed(REG_FFE_TAP(0), -37, TAP_APPLY_BITS);
    CHECK(hal_read_signed(REG_FFE_TAP(0), TAP_APPLY_BITS) == -37,
          "signed tap field round-trips negative");
    hal_write_signed(REG_FFE_TAP(1), 63, TAP_APPLY_BITS);
    CHECK(hal_read_signed(REG_FFE_TAP(1), TAP_APPLY_BITS) == 63,
          "signed tap field round-trips positive");

    CHECK(hal_critical_depth() == 0u, "critical sections are balanced");
}

/* ---------------------------------------------------- firmware bus hygiene */
static void test_firmware_bus_hygiene(void)
{
    printf("\nfirmware bus hygiene (the RMW and W1C traps)\n");
    hal_reset_all();
    hal_stats_reset();

    fw_link_t L;
    fw_init(&L);
    for (uint32_t t = 0; t < 200u; ++t) {
        hw_reg_set(REG_SYM_CNT, 4096u);
        hw_reg_set(REG_AMP_ACC, 4096u * 2731u);
        hw_status_set(STAT_SIGDET);
        fw_tick(&L, t);
    }

    const hal_stats_t *s = hal_stats();
    CHECK(s->rmw > 0u, "the firmware does perform read-modify-writes");
    CHECK(s->unguarded_rmw == 0u,
          "EVERY read-modify-write is inside a critical section");
    CHECK(s->w1c_rmw_bugs == 0u,
          "no read-modify-write ever touches a W1C register");
    CHECK(hal_critical_depth() == 0u, "no critical section is left open");
}

/* --------------------------------------- the adaptation loop, closed in CI */
static void test_adapt_closes_loop(void)
{
    printf("\nadaptation loop against a modelled plant (no silicon)\n");
    hal_reset_all();
    fw_adapt_reset();
    fw_adapt_set_gear(4u, 0u);
    hal_write32(REG_ADAPT_CTRL, ADAPT_FFE_EN);

    /* A plant: the "true" tap the loop should find, and a gradient that is
     * proportional to how far away we currently are. That is what real
     * hardware accumulates. */
    const int32_t truth_code = 20;
    for (unsigned iter = 0; iter < 30000u; ++iter) {
        for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
            const int32_t want = (i == 5u) ? truth_code : 0;
            const int32_t have = hal_read_signed(REG_FFE_TAP(i), TAP_APPLY_BITS);
            hw_reg_set(REG_GRAD_ACC(i), (uint32_t)((want - have) * 64));
        }
        (void)fw_adapt_step();
    }

    const int32_t got = hal_read_signed(REG_FFE_TAP(5), TAP_APPLY_BITS);
    printf("        tap[5] converged to %d, target %d\n", got, truth_code);
    CHECK(abs(got - truth_code) <= 2, "LMS tap converges to the plant's value");

    /* Tap 3 is the CURSOR and is deliberately not adapted -- it holds the
     * unity spike so the FFE and the DFE cannot trade gain between them. Any
     * other undriven tap must stay at zero. */
    int32_t others = 0;
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        if (i != 5u && i != FFE_CURSOR) {
            others += abs(hal_read_signed(REG_FFE_TAP(i), TAP_APPLY_BITS));
        }
    }
    CHECK(others <= 4, "taps with no gradient stay near zero");
    CHECK(hal_read_signed(REG_FFE_TAP(FFE_CURSOR), TAP_APPLY_BITS) == 32,
          "the cursor tap is pinned and does not adapt");
    /* The datapath and the firmware hold this constant separately, either side
     * of the register interface, and no compiler can catch them drifting
     * apart. This is the assertion that does. */
    CHECK(fw_adapt_cursor_tap() == FFE_CURSOR,
          "firmware and datapath agree on which tap is the cursor");
    CHECK(fw_adapt_converged() == 1, "convergence is detected and reported");
}

/* --------------------------------------------------- leakage bleeds to zero */
static void test_leakage(void)
{
    printf("\nleakage\n");
    hal_reset_all();
    fw_adapt_reset();
    fw_adapt_set_gear(4u, 8u);              /* leakage ON */
    hal_write32(REG_ADAPT_CTRL, ADAPT_FFE_EN);

    /* Drive one non-cursor tap up, then remove the drive and let leakage pull
     * it back. Tap 3 cannot be used for this test any more: it is the pinned
     * cursor, so by design nothing -- gradient or leakage -- moves it. */
    for (unsigned k = 0; k < 400u; ++k) {
        for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
            hw_reg_set(REG_GRAD_ACC(i), (i == 6u && i != FFE_CURSOR) ? 16000u : 0u);
        }
        (void)fw_adapt_step();
    }
    const int32_t driven = hal_read_signed(REG_FFE_TAP(6), TAP_APPLY_BITS);
    CHECK(driven > 6, "a driven tap climbs away from zero");

    for (unsigned k = 0; k < 8000u; ++k) {
        for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
            hw_reg_set(REG_GRAD_ACC(i), 0u);   /* no drive at all */
        }
        (void)fw_adapt_step();
    }
    CHECK(abs(hal_read_signed(REG_FFE_TAP(6), TAP_APPLY_BITS)) <= 1,
          "with no gradient, leakage bleeds a tap back to zero");
    CHECK(hal_read_signed(REG_FFE_TAP(FFE_CURSOR), TAP_APPLY_BITS) == 32,
          "leakage does not touch the pinned cursor");
}

/* ------------------------------------------------------------ PAM4 and TX */
static void test_pam4(void)
{
    printf("\nPAM4 / Gray coding / TX FFE\n");

    /* Gray: adjacent LEVELS must differ in exactly one bit. That is what turns
     * a symbol slip into one bit error instead of two. */
    int ok = 1;
    for (unsigned s = 0; s + 1u < 4u; ++s) {
        if (pam4_bit_errors(s, s + 1u) != 1u) { ok = 0; }
    }
    CHECK(ok, "Gray: adjacent levels differ in exactly one bit");
    CHECK(pam4_bit_errors(0u, 2u) == 2u, "levels two apart differ in two bits");

    CHECK(pam4_slice(pam4_level(0u)) == 0u, "slicer recovers level 0");
    CHECK(pam4_slice(pam4_level(3u)) == 3u, "slicer recovers level 3");
    CHECK(pam4_slice((real_t)-0.9) == 0u, "slicer thresholds: -0.9 -> 0");
    CHECK(pam4_slice((real_t)+0.1) == 2u, "slicer thresholds: +0.1 -> 2");

    /* TX FFE L1 constraint: the peak output over ALL data patterns is the L1
     * norm, so L1 == 1 means the driver can never clip. */
    tx_ffe_t f;
    (void)tx_ffe_set(&f, -0.12, -0.20);
    double l1 = 0.0;
    for (unsigned i = 0; i < TX_FFE_TAPS; ++i) { l1 += fabs(f.c[i]); }
    CHECK(fabs(l1 - 1.0) < 1e-9, "TX FFE taps are L1-normalised (driver cannot clip)");
    CHECK(f.c[TX_FFE_CURSOR] < 1.0,
          "de-emphasis costs launch amplitude at the cursor");

    /* PRBS31 must not be degenerate, and its low bit must not be periodic --
     * an LCG's low bit alternates with period 2 and would make every eye look
     * open regardless of the channel. */
    prbs_t p;
    prbs_init(&p, 0x5A5A5A5Au);   /* a well-mixed seed, not a sparse one */
    unsigned ones = 0u;
    unsigned alternating = 1u;
    uint32_t prev = prbs_bit(&p);
    for (unsigned i = 0; i < 200000u; ++i) {
        const uint32_t b = prbs_bit(&p);
        ones += b;
        if (b == prev) { alternating = 0u; }
        prev = b;
    }
    printf("        PRBS31 ones: %u of 200000\n", ones);
    CHECK(ones > 98000u && ones < 102000u, "PRBS31 is roughly balanced");
    CHECK(!alternating, "PRBS31 is not a period-2 alternation");
}

/* ---------------------------------------------------------- FFT + channel */
static void test_channel(void)
{
    printf("\nFFT and channel synthesis\n");

    /* FFT round-trip */
    enum { N = 256 };
    cplx a[N], b[N];
    for (unsigned i = 0; i < N; ++i) {
        a[i].re = sin(0.1 * i) + 0.3 * cos(0.7 * i);
        a[i].im = 0.0;
        b[i] = a[i];
    }
    fft_run(b, N, 0);
    fft_run(b, N, 1);
    double err = 0.0;
    for (unsigned i = 0; i < N; ++i) { err += fabs(b[i].re - a[i].re); }
    CHECK(err / N < 1e-12, "FFT forward+inverse is the identity");

    /* The synthesised channel must actually have the loss it was asked for. */
    for (double want = 12.0; want <= 30.0; want += 9.0) {
        channel_t ch;
        CHECK(channel_build(&ch, want, 48u) == 0, "channel_build succeeds");

        cplx *m = (cplx *)calloc(16384u, sizeof(cplx));
        for (size_t i = 0; i < ch.n && i < 16384u; ++i) { m[i].re = ch.h[i]; }
        fft_run(m, 16384u, 0);
        const double fs = BAUD_RATE_GBD * (double)OSR;
        const size_t k  = (size_t)(NYQUIST_GHZ / fs * 16384.0 + 0.5);
        const double got = -20.0 * log10(sqrt(m[k].re * m[k].re + m[k].im * m[k].im));
        printf("        asked %.0f dB, synthesised %.2f dB\n", want, got);
        CHECK(fabs(got - want) < 1.0, "synthesised insertion loss matches the model");

        /* Minimum phase: the postcursor tail must dominate the precursors.
         * A magnitude-only channel would be symmetric, which is unphysical. */
        size_t pk = 0;
        for (size_t i = 1; i < ch.n; ++i) {
            if (fabs(ch.h[i]) > fabs(ch.h[pk])) { pk = i; }
        }
        double pre = 0.0, post = 0.0;
        for (size_t i = 0; i < pk; ++i)          { pre  += fabs(ch.h[i]); }
        for (size_t i = pk + 1u; i < ch.n; ++i)  { post += fabs(ch.h[i]); }
        CHECK(post > pre, "channel is minimum phase: postcursors dominate");

        free(m);
        channel_free(&ch);
    }
}

/* ===========================================================================
 *  Interrupt preemption: the RMW hazard, DEMONSTRATED rather than asserted.
 *
 *  The HAL used only to COUNT unguarded read-modify-writes. A count is a claim
 *  about a hazard. This fires a real interrupt at the one instant that matters
 *  -- between the read and the write -- and watches the update disappear.
 * =========================================================================*/
static void isr_touch_other_field(void *ctx)
{
    /* A plausible ISR: something completed, so it sets ENABLE in the very
     * register the foreground code is editing a different field of. Short, and
     * using hal_write32 directly, as a real ISR would. */
    (void)ctx;
    hal_write32(REG_CTRL, hal_read32(REG_CTRL) | CTRL_EN);
}

static void test_rmw_preemption(void)
{
    /* --- unguarded: the interrupt's update is DESTROYED ------------------ */
    hal_reset_all();
    hal_attach_isr(isr_touch_other_field, NULL);
    hal_write32(REG_CTRL, 0u);

    /* No critical section. The ISR lands between our read and our write, sets
     * CTRL_EN, and then our stale write-back erases it. */
    hal_field_set(REG_CTRL, 0xF0u, 4u, 0x5u);

    CHECK(hal_isr_runs() == 1u, "ISR ran inside the unguarded RMW");
    CHECK((hal_read32(REG_CTRL) & 0xF0u) == 0x50u, "our own field was written");
    CHECK((hal_read32(REG_CTRL) & CTRL_EN) == 0u,
          "LOST UPDATE: the ISR set CTRL_EN and the RMW wiped it out");
    CHECK(hal_stats()->unguarded_rmw == 1u, "the unguarded RMW was counted");

    /* --- guarded: deferred, replayed, nothing lost ------------------------ */
    hal_reset_all();
    hal_attach_isr(isr_touch_other_field, NULL);
    hal_write32(REG_CTRL, 0u);

    hal_critical_enter();
    hal_field_set(REG_CTRL, 0xF0u, 4u, 0x5u);
    hal_critical_exit();               /* the pending interrupt replays here */

    CHECK(hal_isr_deferred() == 1u, "the guard held the interrupt off");
    CHECK(hal_isr_runs() == 1u,     "and it replayed on critical_exit");
    CHECK((hal_read32(REG_CTRL) & 0xF0u) == 0x50u, "our field survived");
    CHECK((hal_read32(REG_CTRL) & CTRL_EN) != 0u,
          "and so did the interrupt's bit -- nothing was lost");
    CHECK(hal_stats()->unguarded_rmw == 0u, "no unguarded RMW this time");

    hal_detach_isr();
}

/* ---- PLL bring-up -------------------------------------------------------- */
static void test_pll(void)
{
    hal_reset_all();
    pll_t p;
    pll_init(&p, 5u);

    hal_write32(REG_PLL_CTRL, 0u);
    for (unsigned i = 0; i < 50u; ++i) { pll_step(&p); }
    CHECK((hal_read32(REG_STATUS) & STAT_PLL_LOCK) == 0u,
          "a disabled PLL never reports lock, however long you wait");

    hal_write32(REG_PLL_CTRL, PLL_EN);
    for (unsigned i = 0; i < 4u; ++i) { pll_step(&p); }
    CHECK((hal_read32(REG_STATUS) & STAT_PLL_LOCK) == 0u,
          "not locked before the settling time has elapsed");
    for (unsigned i = 0; i < 4u; ++i) { pll_step(&p); }
    CHECK((hal_read32(REG_STATUS) & STAT_PLL_LOCK) != 0u, "locked after settling");

    /* The failure path has to be reachable, or the FSM timeout is dead code. */
    pll_force_fail(&p, 1u);
    for (unsigned i = 0; i < 50u; ++i) { pll_step(&p); }
    CHECK((hal_read32(REG_STATUS) & STAT_PLL_LOCK) == 0u,
          "a failing PLL stays unlocked so the bring-up FSM can time out");
    pll_force_fail(&p, 0u);
}

/* ---- management framing -------------------------------------------------- */
static void test_mgmt_framing(void)
{
    uint8_t f[MGMT_FRAME_BYTES];
    const uint8_t pay[4] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };

    CHECK(mgmt_frame_build(f, MGMT_T_STATUS, 0x1234u, pay, 4u) == 0, "frame builds");
    CHECK(f[0] == MGMT_SOF, "SOF present");
    CHECK(f[2] == 0x34u && f[3] == 0x12u, "sequence is little endian");
    CHECK(mgmt_frame_check(f) == 0, "a good frame validates");

    /* Every single-bit corruption must be caught. A CRC that misses bit flips
     * is worse than no CRC: it makes corrupt telemetry look trustworthy. */
    unsigned caught = 0u, tried = 0u;
    for (unsigned byte = 0; byte < MGMT_FRAME_BYTES; ++byte) {
        for (unsigned bit = 0; bit < 8u; ++bit) {
            uint8_t g[MGMT_FRAME_BYTES];
            memcpy(g, f, sizeof(g));
            g[byte] ^= (uint8_t)(1u << bit);
            tried++;
            if (mgmt_frame_check(g) != 0) { caught++; }
        }
    }
    CHECK(caught == tried, "CRC-8 catches every single-bit error in the frame");

    CHECK(mgmt_frame_build(f, MGMT_T_STATUS, 0u, pay, MGMT_PAYLOAD_MAX + 1u) != 0,
          "an oversized payload is rejected, not silently truncated");
}

/* ---- management bus backpressure ---------------------------------------- */
static void test_mgmt_backpressure(void)
{
    hal_reset_all();
    mgmt_bus_init(4u);                      /* deliberately slow drain */
    hal_set_write_hook(NULL);               /* push directly, no platform */
    hal_write32(REG_MGMT_CTRL, MGMT_TX_EN);

    for (unsigned i = 0; i < MGMT_FIFO_BYTES; ++i) {
        mgmt_bus_push(0xAAu);
    }
    CHECK(mgmt_bus_free() == 0u, "the FIFO fills");
    CHECK(mgmt_bus_dropped() == 0u, "nothing dropped while there was room");

    mgmt_bus_push(0x55u);
    CHECK(mgmt_bus_dropped() == 1u,
          "pushing into a full FIFO LOSES the byte -- hence check free space first");

    mgmt_bus_tick();
    CHECK(mgmt_bus_free() == 4u, "the bus drains at its configured rate, not faster");
    CHECK(mgmt_wire_available() == 4u, "and the drained bytes appear on the wire");
}

static uint32_t t_rng = 0x9E3779B9u;

static uint32_t t_rand(void)
{
    uint32_t x = t_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    t_rng = x;
    return x;
}

/* ---- the fast convolution must equal the definition ---------------------- */
static void test_convolution(void)
{
    printf("\noverlap-add convolution against the direct form\n");
    channel_t ch;
    CHECK(channel_build(&ch, 20.0, 48u) == 0, "channel builds");

    const size_t n = 8192u;
    real_t *x = (real_t *)calloc(n, sizeof(real_t));
    real_t *a = (real_t *)calloc(n, sizeof(real_t));
    real_t *b = (real_t *)calloc(n, sizeof(real_t));
    if (x == NULL || a == NULL || b == NULL) {
        CHECK(0, "allocation");
        free(x); free(a); free(b);
        channel_free(&ch);
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        x[i] = (real_t)(((double)(t_rand() >> 8) / 8388608.0) - 1.0);
    }

    channel_apply_direct(&ch, x, a, n);
    channel_apply(&ch, x, b, n);

    double worst = 0.0, energy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = fabs((double)a[i] - (double)b[i]);
        if (d > worst) {
            worst = d;
        }
        energy += (double)a[i] * (double)a[i];
    }
    const double rms = sqrt(energy / (double)n);
    printf("        worst sample error %.3e against an rms signal of %.3e\n",
           worst, rms);
    /* An FFT convolution is not bit-identical to the direct form -- it is a
     * different order of operations on the same numbers. What it must be is
     * identical to within floating-point rounding, which for these lengths is
     * around 1e-12 relative. Anything larger means the padding is wrong and
     * the block tails are wrapping. */
    CHECK(worst < rms * 1e-9,
          "overlap-add matches the direct convolution to rounding");

    free(x); free(a); free(b);
    channel_free(&ch);
}


/* ---- Touchstone reader --------------------------------------------------- */
static void test_touchstone(void)
{
    printf("\nTouchstone reader\n");

    /* Write a tiny 2-port file covering the format's awkward corners: a
     * comment mid-line, MA format, MHz units, and a record split across two
     * lines. Every real .sNp does at least one of these. */
    const char *path = "test_tmp.s2p";
    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        CHECK(0, "can create a temporary Touchstone file");
        return;
    }
    fputs("! a comment line\n", fp);
    fputs("# MHZ S MA R 50\n", fp);
    /* A 2-port record is four complex pairs: S11 S21 S12 S22. */
    fputs("1000  0.1 0   0.9 -10  0.9 -10  0.1 0\n", fp);
    fputs("2000  0.2 0   0.8 -20  0.8 -20  0.2 0   ! trailing comment\n", fp);
    fputs("3000  0.3 0\n", fp);          /* record split across two lines */
    fputs("      0.7 -30  0.7 -30  0.3 0\n", fp);
    fclose(fp);

    touchstone_t ts;
    const int rc = ts_load(&ts, path, 0u);
    CHECK(rc == 0, "loads a .s2p and deduces two ports from the extension");
    if (rc != 0) {
        printf("        (%s)\n", ts_error());
        remove(path);
        return;
    }
    CHECK(ts.n == 3u, "three frequency points, one of them split over two lines");
    CHECK(fabs(ts.f_hz[0] - 1e9) < 1.0, "MHZ unit applied: 1000 MHz is 1 GHz");
    CHECK(fabs(ts.z0 - 50.0) < 1e-9, "reference impedance read from the option line");

    /* MA format: magnitude 0.9 at -10 degrees. */
    const ts_cplx s21 = ts_get(&ts, 0u, 2u, 1u);
    const double mag = sqrt(s21.re * s21.re + s21.im * s21.im);
    CHECK(fabs(mag - 0.9) < 1e-9, "MA magnitude decoded");
    CHECK(s21.im < 0.0, "and its negative phase angle");

    /* THE TWO-PORT TRANSPOSE. v1 stores a 2-port column major, so S21 is the
     * SECOND pair on the line and S12 the third. Reading it row major swaps
     * the through path with the reverse one -- nearly invisible on a
     * reciprocal channel, which is what makes it dangerous. */
    const ts_cplx s11 = ts_get(&ts, 0u, 1u, 1u);
    CHECK(fabs(sqrt(s11.re * s11.re + s11.im * s11.im) - 0.1) < 1e-9,
          "S11 is the first pair");
    CHECK(fabs(mag - 0.9) < 1e-9,
          "S21 is the SECOND pair: the 2-port column-major transpose is handled");

    ts_free(&ts);
    remove(path);

    CHECK(ts_load(&ts, "no_such_file.s4p", 0u) != 0,
          "a missing file fails rather than returning garbage");
}

/* ---- the channel carries memory across blocks ---------------------------- */
static void test_channel_streaming(void)
{
    printf("\nchannel state across block boundaries\n");
    channel_t a, b;
    CHECK(channel_build(&a, 20.0, 48u) == 0, "channel builds");
    CHECK(channel_build(&b, 20.0, 48u) == 0, "and a second identical one");

    const size_t half = 4096u;
    real_t *x  = (real_t *)calloc(2u * half, sizeof(real_t));
    real_t *y1 = (real_t *)calloc(2u * half, sizeof(real_t));
    real_t *y2 = (real_t *)calloc(2u * half, sizeof(real_t));
    if (x == NULL || y1 == NULL || y2 == NULL) {
        CHECK(0, "allocation");
        free(x); free(y1); free(y2);
        channel_free(&a); channel_free(&b);
        return;
    }
    for (size_t i = 0; i < 2u * half; ++i) {
        x[i] = (real_t)(((double)(t_rand() >> 8) / 8388608.0) - 1.0);
    }

    /* One long call, versus two half-length calls on an identical channel. */
    channel_apply(&a, x, y1, 2u * half);
    channel_apply(&b, x, y2, half);
    channel_apply(&b, x + half, y2 + half, half);

    double worst = 0.0;
    for (size_t i = 0; i < 2u * half; ++i) {
        const double d = fabs((double)y1[i] - (double)y2[i]);
        if (d > worst) {
            worst = d;
        }
    }
    printf("        worst difference across the seam: %.3e\n", worst);
    /* Without persistent state the second call would convolve its first 768
     * samples against silence, and this would be O(1e-2), not O(1e-15). That
     * artefact put a 2.4e-4 floor on the training BER -- exactly the KP4
     * limit -- and made bring-up reject a receiver that was working. */
    CHECK(worst < 1e-12,
          "splitting a waveform into two calls gives the identical result");

    /* UNEQUAL splits, so the overlap-add sees partial final blocks. The
     * carried tail has to shift by what was actually emitted, not by the
     * nominal block size. Every call in this project happens to use an exact
     * multiple of the block size, so nothing else here would ever catch it. */
    channel_t c;
    CHECK(channel_build(&c, 20.0, 48u) == 0, "a third identical channel");
    real_t *y3 = (real_t *)calloc(2u * half, sizeof(real_t));
    if (y3 != NULL) {
        const size_t cuts[] = { 1000u, 3000u, 777u, 2u * half - 4777u };
        size_t off = 0u;
        for (unsigned k = 0; k < 4u; ++k) {
            channel_apply(&c, x + off, y3 + off, cuts[k]);
            off += cuts[k];
        }
        double w2 = 0.0;
        for (size_t i = 0; i < 2u * half; ++i) {
            const double d = fabs((double)y1[i] - (double)y3[i]);
            if (d > w2) {
                w2 = d;
            }
        }
        printf("        worst difference across four ragged splits: %.3e\n", w2);
        CHECK(w2 < 1e-12, "and splitting it at ragged, non-block-aligned lengths");
        free(y3);
    }
    channel_free(&c);

    free(x); free(y1); free(y2);
    channel_free(&a);
    channel_free(&b);
}

/* ---- the CDR lock detector must not lie ---------------------------------- */
static void test_cdr_lock_detector(void)
{
    printf("\nCDR lock detector\n");
    cdr_t c;
    cdr_init(&c, 1.5e-1, 3.0e-4);

    /* A loop pinned at its anti-windup rail is maximally broken. The previous
     * detector PASSED here, because it asked whether the frequency estimate
     * had stopped moving -- and a clamped integrator has stopped moving by
     * definition. */
    c.integ     = 0.016;          /* CDR_INTEG_MAX */
    c.ted_avg   = 0.0;            /* the signed mean says nothing is wrong */
    c.slew_slow = 0.0;
    c.lock_count = 0u;
    for (unsigned i = 0; i < 20000u; ++i) {
        const double keep_integ = c.integ;
        cdr_update(&c, (real_t)0.0, (real_t)0.0);
        c.integ = keep_integ;     /* hold it on the rail */
        c.ted_avg = 0.0;
        c.slew_slow = 0.0;
    }
    CHECK(c.locked == 0u, "a railed integrator is never reported as locked");

    /* A phase that is sweeping drives the detector across its whole S-curve,
     * so the signed mean is also near zero. The residual-slew condition is
     * what separates that from a loop that is actually holding. */
    cdr_init(&c, 1.5e-1, 3.0e-4);
    c.ted_avg   = 0.0;
    c.integ     = 0.0;
    c.slew_slow = 2.0e-3;        /* still slewing, ~125 ppm uncorrected */
    c.since_wrap = 100000u;
    c.lock_count = 0u;
    for (unsigned i = 0; i < 20000u; ++i) {
        const double keep = c.slew_slow;
        cdr_update(&c, (real_t)0.0, (real_t)0.0);
        c.slew_slow = keep;
        c.ted_avg = 0.0;
        c.integ = 0.0;
    }
    CHECK(c.locked == 0u, "a sweeping phase is never reported as locked");
}

/* ---- the lane window across an ISR --------------------------------------- */
/* ---- the lane register window -------------------------------------------- */
static unsigned g_isr_lane_target;

static void isr_touch_other_lane(void *ctx)
{
    (void)ctx;
    /* A rude interrupt handler: it switches the window, does its business on
     * another lane, and returns without restoring anything. */
    hal_select_lane(g_isr_lane_target);
    hal_write32(REG_CTRL, 0xBEEFu);
}

static void test_lane_window(void)
{
    printf("\nper-lane register window\n");
    hal_reset_all();
    hal_attach_isr(NULL, NULL);

    /* Each aperture is genuinely separate storage. */
    for (unsigned i = 0; i < HAL_MAX_LANES; ++i) {
        hal_select_lane(i);
        hal_write32(REG_AFE_VGA, 0x100u + i);
    }
    unsigned ok = 1u;
    for (unsigned i = 0; i < HAL_MAX_LANES; ++i) {
        hal_select_lane(i);
        if (hal_read32(REG_AFE_VGA) != 0x100u + i) {
            ok = 0u;
        }
    }
    CHECK(ok, "each lane aperture is independent storage");

    hal_select_lane(3u);
    CHECK((hal_read32(REG_LANE_ID) & 0xFu) == 3u,
          "LANE_ID reports which aperture is selected");

    /* An offset that already carries LANE_BASE(n) must fold back into the
     * selected window rather than escaping into a neighbour. */
    hal_select_lane(5u);
    hal_write32(LANE_BASE(2u) + REG_CTRL, 0x1234u);
    CHECK(hal_read32(REG_CTRL) == 0x1234u,
          "an offset with a stale LANE_BASE folds into the selected window");
    hal_select_lane(2u);
    CHECK(hal_read32(REG_CTRL) != 0x1234u,
          "and does not reach the lane whose base it carried");

    /* The hazard: an ISR that repoints the window must not corrupt the
     * interrupted lane's accesses. */
    hal_reset_all();
    g_isr_lane_target = 7u;
    hal_attach_isr(isr_touch_other_lane, NULL);
    hal_select_lane(1u);
    hal_write32(REG_CTRL, 0u);
    hal_field_set(REG_CTRL, 0xF0u, 4u, 0x5u);      /* preempted mid-RMW */
    CHECK(hal_current_lane() == 1u,
          "the lane window survives an ISR that repointed it");
    CHECK(hal_read32(REG_CTRL) == 0x50u,
          "and the interrupted write landed on the right lane");
    hal_select_lane(7u);
    CHECK(hal_read32(REG_CTRL) == 0xBEEFu, "the ISR's own write landed on its lane");
    hal_attach_isr(NULL, NULL);
    hal_select_lane(0u);
}

/* ---- FEC ----------------------------------------------------------------- */
static void test_fec(void)
{
    static uint16_t msg[RS_K], cw[RS_N], rx[RS_N], before[RS_N];
    static uint8_t  erased[RS_N];

    printf("\nRS(544,514) KP4 FEC\n");
    fec_init();

    for (unsigned i = 0; i < RS_K; ++i) {
        msg[i] = (uint16_t)(t_rand() & (GF_SIZE - 1u));
    }
    fec_encode(msg, cw);
    CHECK(memcmp(cw, msg, RS_K * sizeof(uint16_t)) == 0,
          "encode is systematic: the message is carried verbatim");

    memcpy(rx, cw, sizeof(cw));
    CHECK(fec_decode(rx) == 0, "a clean codeword needs no corrections");

    /* Exactly t errors is the boundary the whole code is specified at. */
    memcpy(rx, cw, sizeof(cw));
    for (unsigned i = 0; i < RS_T; ++i) {
        rx[i * 31u] ^= (uint16_t)(1u + (t_rand() % (GF_SIZE - 1u)));
    }
    CHECK(fec_decode(rx) == (int)RS_T, "t = 15 errors are all corrected");
    CHECK(memcmp(rx, cw, sizeof(cw)) == 0, "and the codeword is exactly restored");

    /* One past the boundary must FAIL, and must fail cleanly: an uncorrectable
     * word comes back byte-for-byte as received, never half-corrected. */
    memcpy(rx, cw, sizeof(cw));
    for (unsigned i = 0; i < RS_T + 1u; ++i) {
        rx[i * 29u] ^= (uint16_t)(1u + (t_rand() % (GF_SIZE - 1u)));
    }
    memcpy(before, rx, sizeof(rx));
    CHECK(fec_decode(rx) < 0, "t+1 = 16 errors are declared uncorrectable");
    CHECK(memcmp(rx, before, sizeof(rx)) == 0,
          "a failed decode leaves the buffer untouched");

    /* Erasures are half price, so twice as many fit in the same parity. */
    memcpy(rx, cw, sizeof(cw));
    memset(erased, 0, sizeof(erased));
    for (unsigned i = 0; i < RS_PARITY; ++i) {
        rx[i * 17u] ^= (uint16_t)(1u + (t_rand() % (GF_SIZE - 1u)));
        erased[i * 17u] = 1u;
    }
    unsigned used = 0u;
    CHECK(fec_decode_erasures(rx, erased, &used) >= 0,
          "30 erasures are corrected: an erasure costs one parity symbol");
    CHECK(used == RS_PARITY, "the erasure count is reported back");
    CHECK(memcmp(rx, cw, sizeof(cw)) == 0, "erasure decode restores the codeword");

    /* And the same 30 positions with the flags withheld must NOT decode --
     * this is what proves the erasure path is doing real work rather than the
     * hard decoder quietly succeeding underneath it. */
    memcpy(rx, cw, sizeof(cw));
    for (unsigned i = 0; i < RS_PARITY; ++i) {
        rx[i * 17u] ^= (uint16_t)(1u + (t_rand() % (GF_SIZE - 1u)));
    }
    CHECK(fec_decode(rx) < 0, "the same 30 errors WITHOUT flags are uncorrectable");

    /* Bit packing round-trips. */
    static uint8_t p_a[RS_PAM4_PER_CW], p_b[RS_PAM4_PER_CW];
    fec_unpack_pam4(cw, p_a, RS_N);
    static uint16_t back[RS_N];
    fec_pack_pam4(p_a, back, RS_N);
    CHECK(memcmp(back, cw, sizeof(cw)) == 0,
          "GF symbol <-> five PAM4 symbols round-trips");
    fec_unpack_pam4(back, p_b, RS_N);
    CHECK(memcmp(p_a, p_b, sizeof(p_a)) == 0, "and is stable");

    unsigned gray_ok = 1u;
    for (unsigned s = 0; s < 4u; ++s) {
        unsigned b1, b0;
        pam4_bits(s, &b1, &b0);
        if (pam4_sym_from_gray((b1 << 1) | b0) != s) {
            gray_ok = 0u;
        }
    }
    CHECK(gray_ok, "Gray word <-> PAM4 level index is a bijection");
}

/* ---- PCS ----------------------------------------------------------------- */
/* Loopback with no channel at all. If this fails, nothing measured through the
 * lane means anything -- so it runs before any of the link-level tests. */
static void test_pcs_loopback(void)
{
    static pcs_tx_t tx;
    static pcs_rx_t rx;

    printf("\nPCS framing (ideal loopback)\n");
    pcs_tx_init(&tx, 0xA5A51234u);
    pcs_rx_init(&rx, 0xA5A51234u);

    /* Alignment consumes symbols before any scoring starts, then the framer
     * runs on to the next codeword boundary. Both are deterministic here, so
     * the test simply feeds one codeword of lead-in plus the three it wants
     * measured. */
    const unsigned lead = RS_PAM4_PER_CW;
    for (unsigned i = 0; i < lead + RS_PAM4_PER_CW * 3u; ++i) {
        pcs_rx_push(&rx, pcs_tx_next(&tx), 0);
    }

    CHECK(rx.state == PCS_RUNNING, "the PCS locks to the pattern");
    CHECK(rx.align_offset == 0u,
          "an ideal loopback measures zero equaliser latency");
    CHECK(rx.align_errors == 0u, "and zero errors at the winning offset");
    CHECK(rx.codewords == 3u, "three codewords framed and decoded");
    CHECK(rx.pre_bit_errors == 0u,
          "the receiver's reference tracks the transmitted stream exactly");
    CHECK(rx.post_bit_errors == 0u, "and no residual errors survive the decode");
    CHECK(rx.uncorrectable == 0u, "no codeword declared uncorrectable");

    /* A deliberately delayed loopback: the decision stream lags the transmitted
     * stream by seven symbols, which is what a converged FFE with its cursor
     * away from tap zero actually does. The aligner must find it. */
    pcs_tx_init(&tx, 0x0BADC0DEu);
    pcs_rx_init(&rx, 0x0BADC0DEu);
    {
        unsigned pipe[7] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u };
        for (unsigned i = 0; i < lead + RS_PAM4_PER_CW; ++i) {
            const unsigned s = pcs_tx_next(&tx);
            const unsigned out = pipe[6];
            for (unsigned k = 6u; k > 0u; --k) {
                pipe[k] = pipe[k - 1u];
            }
            pipe[0] = s;
            pcs_rx_push(&rx, out, 0);
        }
    }
    CHECK(rx.align_offset == 7u, "a 7-symbol pipeline delay is measured exactly");
    CHECK(rx.pre_bit_errors == 0u,
          "and once aligned the delayed stream reads zero errors, not 50%");

    /* Now corrupt exactly t symbols of one scored codeword and confirm the PCS
     * both counts them pre-FEC and removes them post-FEC. */
    pcs_tx_init(&tx, 0x1234A5A5u);
    pcs_rx_init(&rx, 0x1234A5A5u);
    for (unsigned i = 0; i < lead; ++i) {
        pcs_rx_push(&rx, pcs_tx_next(&tx), 0);
    }
    for (unsigned i = 0; i < RS_PAM4_PER_CW; ++i) {
        unsigned s = pcs_tx_next(&tx);
        /* One PAM4 symbol inside each of the first t field elements. */
        if (i < RS_T * RS_PAM4_PER_GF && (i % RS_PAM4_PER_GF) == 0u) {
            s = (s + 1u) & 3u;
        }
        pcs_rx_push(&rx, s, 0);
    }
    CHECK(rx.pre_bit_errors > 0u, "pre-FEC errors are counted");
    CHECK(rx.corrected_symbols == RS_T, "the decoder corrects exactly t symbols");
    CHECK(rx.post_bit_errors == 0u, "post-FEC the payload is clean");
}


/* ===========================================================================
 *  The bring-up state machine, driven against a stub datapath.
 *
 *  This is the module the whole project is pitched on and it was, for a long
 *  time, the only one with no behavioural test at all: fw_tick() ran in CI but
 *  the only assertions were bus-hygiene counters. Every claim about the FSM
 *  was measured by running link_sim and reading the trace, which is a demo,
 *  not a regression test. The seam exists precisely so this can be tested with
 *  no hardware -- not using it here was the sharpest thing missing.
 * =========================================================================*/

/* A datapath that always behaves. Everything the firmware polls is presented
 * as healthy, so the FSM has no excuse not to reach UP. */
static void stub_healthy_block(void)
{
    hw_status_set(STAT_SIGDET | STAT_PLL_LOCK | STAT_CDR_LOCK);
    hw_reg_set(REG_SYM_CNT, 4094u);
    hw_reg_set(REG_AMP_ACC, 4094u * 2731u);   /* exactly on the AGC target */
    hw_reg_set(REG_ERR_CNT, 0u);              /* and making no errors      */
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        hw_reg_set(REG_GRAD_ACC(i), 0u);      /* taps have nothing to do   */
    }
    for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
        hw_reg_set(REG_DFE_GRAD(i), 0u);
    }
}

static void test_bringup_reaches_up(void)
{
    printf("\nbring-up FSM against a stub datapath\n");
    hal_reset_all();
    fw_set_timeout_scale(1u);

    fw_link_t L;
    fw_init(&L);

    uint32_t t = 0u;
    unsigned saw_verify = 0u;
    for (; t < 2000u && !fw_is_up(&L); ++t) {
        stub_healthy_block();
        fw_tick(&L, t);
        if (L.state == LS_EQ_VERIFY) {
            saw_verify = 1u;
        }
    }

    CHECK(fw_is_up(&L), "a healthy datapath brings the link up");
    CHECK(saw_verify, "and it passes through EQ_VERIFY on the way");
    CHECK(L.tm.ms_to_up > 0u && L.tm.ms_to_up == t - 1u,
          "ms_to_up records when it happened");
    CHECK(L.tm.faults == 0u, "with no faults on a datapath that never misbehaves");
    /* Every state on the nominal path must have been entered exactly once. */
    CHECK(L.tm.state_entries[LS_PLL_LOCK] == 1u, "PLL_LOCK entered once");
    CHECK(L.tm.state_entries[LS_AGC] == 1u, "AGC entered once");
    CHECK(L.tm.state_entries[LS_CDR_LOCK] == 1u, "CDR_LOCK entered once");
    CHECK(L.tm.state_entries[LS_EQ_TRAIN] == 1u, "EQ_TRAIN entered once");
    CHECK(L.tm.state_entries[LS_EQ_VERIFY] == 1u, "EQ_VERIFY entered once");
    CHECK(L.tm.state_entries[LS_TRACK] == 1u, "TRACK entered once");
    printf("        reached UP in %u ticks, %u transitions\n",
           L.tm.ms_to_up, L.tm.transitions);
}

static void test_bringup_timeout_and_backoff(void)
{
    printf("\nbring-up timeouts, faults and backoff\n");
    hal_reset_all();
    fw_set_timeout_scale(1u);

    fw_link_t L;
    fw_init(&L);

    /* A PLL that never locks. Everything else is healthy, so the ONLY reason
     * this can fail is the state it is waiting in -- which is the point of
     * giving every state a timeout. */
    for (uint32_t t = 0; t < 400u; ++t) {
        hw_status_set(STAT_SIGDET);              /* deliberately no PLL_LOCK */
        hw_reg_set(REG_SYM_CNT, 4094u);
        hw_reg_set(REG_AMP_ACC, 4094u * 2731u);
        fw_tick(&L, t);
    }

    CHECK(L.tm.timeouts[LS_PLL_LOCK] > 0u,
          "a PLL that never locks times out rather than hanging");
    CHECK(L.tm.faults > 0u, "and the timeout is counted as a fault");
    CHECK(!fw_is_up(&L), "the link never comes up on a dead reference clock");
    CHECK(L.retries > 1u, "and it retries rather than giving up");
    /* Backoff must GROW, or a failing lane hammers the bus forever. */
    CHECK(L.backoff_ms > 2u, "the retry backoff grows rather than staying flat");
    printf("        %u timeouts, %u retries, backoff now %u ms\n",
           L.tm.timeouts[LS_PLL_LOCK], L.retries, L.backoff_ms);
}

static void test_bringup_loss_of_lock_hysteresis(void)
{
    printf("\nloss-of-lock hysteresis\n");
    hal_reset_all();
    fw_set_timeout_scale(1u);

    fw_link_t L;
    fw_init(&L);
    uint32_t t = 0u;
    for (; t < 2000u && !fw_is_up(&L); ++t) {
        stub_healthy_block();
        fw_tick(&L, t);
    }
    CHECK(fw_is_up(&L), "link is up before the lock is disturbed");

    /* Drop CDR lock for fewer ticks than the hysteresis allows. A live lock
     * indication dips under noise on a perfectly healthy link; tearing the
     * link down on one sample is the difference between a glitch and an
     * outage. */
    for (unsigned k = 0; k < 20u; ++k, ++t) {
        /* ASSIGN the status word, do not OR into it: hw_status_set() sets
         * bits and never clears them, so modelling a LOSS of lock means
         * writing the whole word. Getting this wrong makes the test pass for
         * the wrong reason -- the FSM would still see a lock that the test
         * believed it had removed. */
        hw_reg_set(REG_STATUS, STAT_SIGDET | STAT_PLL_LOCK);
        hw_reg_set(REG_SYM_CNT, 4094u);
        hw_reg_set(REG_AMP_ACC, 4094u * 2731u);
        hw_reg_set(REG_ERR_CNT, 0u);
        fw_tick(&L, t);
    }
    CHECK(fw_is_up(&L), "a brief dip in CDR lock does NOT take the link down");

    /* Sustained loss must. */
    for (unsigned k = 0; k < 40u; ++k, ++t) {
        hw_reg_set(REG_STATUS, STAT_SIGDET | STAT_PLL_LOCK);
        hw_reg_set(REG_SYM_CNT, 4094u);
        hw_reg_set(REG_AMP_ACC, 4094u * 2731u);
        hw_reg_set(REG_ERR_CNT, 0u);
        fw_tick(&L, t);
    }
    CHECK(!fw_is_up(&L), "but a sustained loss of lock does");
}

static void test_deadline_wrap(void)
{
    printf("\nmillisecond counter wrap\n");
    /* The comment above fw_deadline_passed() names the 49-day rollover as
     * "exactly the kind of bug that ships", and there was no test. Five lines.
     *
     * The naive form, now >= start + timeout, is FALSE for the first case
     * below: 0x00000005 >= 0xFFFFFFF0 + 10 wraps to 0x00000005 >= 0xFFFFFFFA,
     * which is false, so the deadline silently never fires. Unsigned
     * subtraction is correct across the wrap. */
    CHECK(fw_deadline_passed(0xFFFFFFF0u, 0x00000005u, 10u),
          "a deadline set before the wrap fires after it");
    CHECK(!fw_deadline_passed(0xFFFFFFF0u, 0x00000005u, 100u),
          "and one that has not elapsed across the wrap does not");
    CHECK(fw_deadline_passed(100u, 200u, 100u), "exactly at the deadline counts");
    CHECK(!fw_deadline_passed(100u, 199u, 100u), "one tick before it does not");
    CHECK(!fw_deadline_passed(0u, 0u, 1u), "a fresh deadline has not passed");
}

int main(void)
{
    printf("=====================================================\n");
    printf(" unit tests -- no hardware required\n");
    printf("=====================================================\n");

    test_fixed();
    test_hal();
    test_firmware_bus_hygiene();
    test_adapt_closes_loop();
    test_leakage();
    test_pam4();
    test_channel();
    test_rmw_preemption();
    test_pll();
    test_mgmt_framing();
    test_mgmt_backpressure();
    test_convolution();
    test_channel_streaming();
    test_touchstone();
    test_cdr_lock_detector();
    test_lane_window();
    test_deadline_wrap();
    test_bringup_reaches_up();
    test_bringup_timeout_and_backoff();
    test_bringup_loss_of_lock_hysteresis();
    test_fec();
    test_pcs_loopback();

    printf("\n-----------------------------------------------------\n");
    printf(" %d checks, %d failures\n", g_run, g_fail);
    printf("-----------------------------------------------------\n");
    return g_fail ? 1 : 0;
}



