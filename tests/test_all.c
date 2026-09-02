/* ===========================================================================
 *  test_all.c -- unit tests. Runs with no hardware.
 *
 *  JD: "Write unit tests and participate in continuous integration flows to
 *       ensure firmware stability across hardware revisions and emulation
 *       platforms."
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

    int32_t others = 0;
    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        if (i != 5u) { others += abs(hal_read_signed(REG_FFE_TAP(i), TAP_APPLY_BITS)); }
    }
    CHECK(others <= 4, "taps with no gradient stay near zero");
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

    for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
        hw_reg_set(REG_GRAD_ACC(i), 0u);    /* no drive at all */
    }
    for (unsigned k = 0; k < 3000u; ++k) {
        (void)fw_adapt_step();
        for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
            hw_reg_set(REG_GRAD_ACC(i), 0u);
        }
    }
    CHECK(abs(hal_read_signed(REG_FFE_TAP(3), TAP_APPLY_BITS)) <= 1,
          "with no gradient, leakage bleeds the centre tap to zero");
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

    printf("\n-----------------------------------------------------\n");
    printf(" %d checks, %d failures\n", g_run, g_fail);
    printf("-----------------------------------------------------\n");
    return g_fail ? 1 : 0;
}



