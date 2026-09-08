/* ===========================================================================
 *  hal.h -- Hardware Abstraction Layer for one SerDes lane.
 *
 *  THIS IS THE ONLY SEAM between firmware and hardware. Every fw_*.c file
 *  reaches the datapath through these calls and through nothing else. That is
 *  not tidiness -- it is what makes the firmware testable with no silicon:
 *  swap the backing store for a behavioural model and the adaptation loop
 *  closes in CI. (See tests/.)
 *
 *  REGISTER SEMANTICS, which are not uniform and must not be treated as such:
 *      RW    read/write, read-modify-write is safe (with a critical section)
 *      RO    read only; hardware owns it
 *      W1C   write-1-to-clear. READ-MODIFY-WRITE ON THESE IS A BUG: you read
 *            a set flag, write the word back, and the 1 you wrote CLEARS the
 *            event you never handled. Use hal_w1c(), which writes only the
 *            bits named and never reads first.
 * =========================================================================*/
#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

/* ---- address map, one lane ----------------------------------------------
 *
 * REVISION 2. The first version gave the equaliser 8 FFE and 4 DFE taps and
 * packed the whole lane into 256 bytes. Measured against the channel the part
 * is specified for, that is not enough filter: at 30 dB the pulse response
 * carries 1.78 UI of normalised ISI with a tail still at 5% of the cursor
 * twelve symbols out, and a 4-tap DFE leaves more residual than a PAM4 eye is
 * tall. The link closed at 12 dB and 20 dB and could not close at 26 or 30.
 *
 * So the tap count went to 16 FFE and 8 DFE and the aperture doubled to 512
 * bytes to hold them. Real 100 GBd receivers are in this range for the same
 * reason. Growing an aperture is a normal register-map revision; what matters
 * is that the LANE_ID version field moves with it, so a driver can tell which
 * silicon it is talking to before it starts writing tap addresses that used to
 * mean something else.
 */
#define LANE_STRIDE        0x200u
#define LANE_BASE(n)       ((uint32_t)(n) * LANE_STRIDE)
#define HAL_MAX_LANES      8u          /* one macro's worth of apertures */

#define REG_LANE_ID        0x000u   /* RO  [15:0] id, [31:16] version        */
#define REG_CTRL           0x004u   /* RW                                    */
#define REG_STATUS         0x008u   /* RO + W1C bits                         */
#define REG_AFE_TIA        0x00Cu   /* RW  [3:0]  TIA transimpedance code    */
#define REG_AFE_VGA        0x010u   /* RW  [5:0]  VGA gain code              */
#define REG_AFE_CTLE       0x014u   /* RW  [3:0]  CTLE peaking code          */
#define REG_ADAPT_CTRL     0x018u   /* RW  enables + loop parameters         */
#define REG_ADAPT_STAT     0x01Cu   /* RO  convergence flags                 */
#define REG_FFE_TAP(i)     (0x020u + 4u * (uint32_t)(i))   /* RW, 16 taps    */
#define REG_DFE_TAP(i)     (0x060u + 4u * (uint32_t)(i))   /* RW, 8 taps     */
#define REG_GRAD_ACC(i)    (0x080u + 4u * (uint32_t)(i))   /* RO/W1C, 16     */
#define REG_DFE_GRAD(i)    (0x0C0u + 4u * (uint32_t)(i))   /* RO/W1C, 8      */
#define REG_CDR_PHASE      0x0E0u   /* RW  phase interpolator code           */
#define REG_CDR_FREQ       0x0E4u   /* RO  accumulated frequency offset      */
#define REG_AMP_ACC        0x0E8u   /* RO/W1C  |y| accumulator, for AGC      */
#define REG_ERR_CNT        0x0ECu   /* RO/W1C  symbol errors                 */
#define REG_SYM_CNT        0x0F0u   /* RO/W1C  symbols observed              */
#define REG_EYE_CTRL       0x0F4u   /* RW  margining: phase + level          */
#define REG_EYE_ERR        0x0F8u   /* RO/W1C  errors at that margin point   */
#define REG_PLL_CTRL       0x0FCu   /* RW  [0] PLL_EN, [1] PLL_BYPASS        */
#define REG_PLL_STAT       0x100u   /* RO  [0] LOCKED, [15:8] lock counter   */
#define REG_MGMT_CTRL      0x104u   /* RW  [0] TX_EN                         */
#define REG_MGMT_STAT      0x108u   /* RO  [15:0] free bytes in the TX FIFO  */
#define REG_MGMT_DATA      0x10Cu   /* WO  push one byte into the TX FIFO    */
#define REG_EYE_ADDR       0x110u   /* RW  index into the eye capture RAM    */
#define REG_EYE_DATA       0x114u   /* RO  byte at REG_EYE_ADDR              */
#define REG_CDR_CTRL       0x118u   /* RW  [5:0] timing-detector h1 target   */

/* The highest mapped offset in a lane aperture. Every 4-byte offset from 0x000
 * to here is a real register, so "is this address mapped" is a comparison
 * rather than a table -- ADD A REGISTER AND THIS MOVES WITH IT.
 *
 * It exists because the model is otherwise MORE FORGIVING THAN SILICON, which
 * is the wrong direction for a model to err in. An access past the aperture
 * folds back into it (see idx_of), and a misaligned one is silently rounded
 * down; on a real part the first reaches a neighbouring lane or an unmapped
 * address and the second is a bus fault. Firmware that computed a bad offset
 * would therefore pass here and fault on the part. The accesses are counted
 * instead, so the model reports what the hardware would refuse. */
#define REG_LAST           0x118u

/* ---- CTRL ---------------------------------------------------------------- */
#define CTRL_EN            (1u << 0)
#define CTRL_TX_EN         (1u << 1)
#define CTRL_RX_EN         (1u << 2)
#define CTRL_RESET         (1u << 3)

/* ---- STATUS -------------------------------------------------------------- */
#define STAT_SIGDET        (1u << 0)   /* RO  */
#define STAT_CDR_LOCK      (1u << 1)   /* RO  */
#define STAT_EQ_CONV       (1u << 2)   /* RO  */
#define STAT_AGC_CONV      (1u << 3)   /* RO  */
#define STAT_ERR_OVF       (1u << 8)   /* W1C -- error counter overflowed    */
#define STAT_LOS           (1u << 9)   /* W1C -- loss of signal was seen     */
#define STAT_PLL_LOCK      (1u << 4)   /* RO  -- reference PLL is locked     */
#define STATUS_W1C_MASK    (STAT_ERR_OVF | STAT_LOS)

/* ---- ADAPT_CTRL ---------------------------------------------------------- */
#define ADAPT_AGC_EN       (1u << 0)
#define ADAPT_CDR_EN       (1u << 1)
#define ADAPT_FFE_EN       (1u << 2)
#define ADAPT_DFE_EN       (1u << 3)
#define ADAPT_MU_MASK      (0xFu << 8)     /* step-size shift, gear shifting */
#define ADAPT_MU_SHIFT     8u
#define ADAPT_LEAK_MASK    (0xFu << 12)
#define ADAPT_LEAK_SHIFT   12u

/* ---- CDR_CTRL -----------------------------------------------------------
 * The timing detector's h1 target, as a fraction of the cursor: target =
 * code / 128, so 6 bits span 0 to 0.49 in steps of 0.008. It is a REGISTER,
 * not a constant, because the right value depends on how asymmetric the
 * channel is and firmware is the only thing that gets to find that out. */
#define CDR_H1_MASK        0x0000003Fu
#define CDR_H1_SHIFT       0u
#define CDR_H1_SCALE       128.0

/* ---- PLL / management ---------------------------------------------------- */
#define PLL_EN             (1u << 0)
#define PLL_BYPASS         (1u << 1)
#define PLL_STAT_LOCKED    (1u << 0)
#define MGMT_TX_EN         (1u << 0)
#define MGMT_FIFO_BYTES    512u

/* ---- field widths -------------------------------------------------------- */
#define TIA_GAIN_MASK      0x0000000Fu
#define TIA_GAIN_SHIFT     0u
#define VGA_GAIN_MASK      0x0000003Fu
#define VGA_GAIN_SHIFT     0u
#define CTLE_PEAK_MASK     0x0000000Fu
#define CTLE_PEAK_SHIFT    0u

#define TIA_GAIN_CODES     (TIA_GAIN_MASK  + 1u)   /* 16 */
#define VGA_GAIN_CODES     (VGA_GAIN_MASK  + 1u)   /* 64 */
#define CTLE_PEAK_CODES    (CTLE_PEAK_MASK + 1u)   /* 16 */

#define NUM_FFE_TAPS       16u
#define NUM_DFE_TAPS       8u

/* ===========================================================================
 *  The firmware-facing API. Nothing else may touch hardware.
 * =========================================================================*/
uint32_t hal_read32 (uint32_t off);
void     hal_write32(uint32_t off, uint32_t val);

/* ---- the lane window ----------------------------------------------------
 * Eight lanes share one register map layout, one aperture each, LANE_STRIDE
 * apart. Rather than add LANE_BASE(n) to every access in every firmware
 * module -- which is a lot of places to forget it -- the base is held in a
 * window register and every hal access is relative to it. That is how paged
 * peripherals are actually addressed, and it means the per-lane control code
 * is byte-identical no matter which lane it is servicing.
 *
 * THE HAZARD THAT COMES WITH IT: the window is shared mutable state. Anything
 * that can preempt lane servicing -- an ISR, another task -- and touches a
 * register will do it through whatever window happens to be selected, and on
 * return the interrupted code carries on believing its own selection still
 * holds. The bug is silent and it corrupts a DIFFERENT lane than the one being
 * debugged, which is the worst possible failure to chase.
 *
 * So an interrupt handler must save and restore the window, exactly as it
 * saves registers. This HAL's dispatcher does that directly around the
 * handler call -- see hal_maybe_preempt() in hal.c -- and test_lane_window()
 * asserts it by installing an ISR that deliberately repoints the window and
 * checking the interrupted write still lands on the right lane.
 *
 * (An earlier version of this header advertised a hal_lane_push/pop pair here.
 * They existed, were never called, and were wrong: push(1); push(2); pop();
 * left the window on lane 2. Advertising an uncalled, broken implementation of
 * the very hazard the paragraph above describes was worse than having none, so
 * they are gone.) */
void     hal_select_lane(unsigned lane);
unsigned hal_current_lane(void);

/* Read-modify-write of one field.
 *
 * NOTE WHAT THIS DOES NOT DO: it does not take a critical section for you. A
 * RMW is three bus transactions and `volatile` gives no atomicity, so an ISR
 * that touches another field of the same register between the read and the
 * write has its update silently erased. The CALLER brackets it with
 * hal_critical_enter/exit, and this function COUNTS whether the caller did --
 * see hal_stats()->unguarded_rmw, which the unit tests assert is zero across
 * the whole firmware.
 *
 * Counting rather than locking is deliberate. A lock in here would make every
 * call safe and make the hazard untestable; the counter turns "the firmware
 * never does an unguarded RMW" from a claim in a comment into an assertion in
 * CI. test_rmw_preemption() exercises the lost update itself. */
void     hal_field_set(uint32_t off, uint32_t mask, unsigned shift, uint32_t val);
uint32_t hal_field_get(uint32_t off, uint32_t mask, unsigned shift);

/* Set or clear whole bits by MASK, with no shift argument to get wrong.
 *
 * hal_field_set(reg, ADAPT_CDR_EN, 0, 1) is a bug: the field helper computes
 * (val << shift) & mask, so a value of 1 with shift 0 against a mask of
 * (1<<1) evaluates to 0 and the bit is never set. Single-bit controls take
 * this function instead -- there is no shift to supply and none to mismatch. */
void     hal_bit_write(uint32_t off, uint32_t bitmask, bool on);

/* Write-1-to-clear. Writes ONLY the named bits; never reads first. */
void     hal_w1c(uint32_t off, uint32_t bits);

/* Read a RO/W1C accumulator and clear it in one step (read-then-clear). */
uint32_t hal_read_clear(uint32_t off);

/* Signed field helpers -- tap registers hold two's-complement codes narrower
 * than 32 bits, so they need explicit sign extension on read. */
void     hal_write_signed(uint32_t off, int32_t val, unsigned bits);
int32_t  hal_read_signed (uint32_t off, unsigned bits);

/* ===========================================================================
 *  INTERRUPTS
 *
 *  The RMW hazard is not asserted here, it is EXERCISED. Attach an ISR and the
 *  mock bus invokes it at the one instant that matters: between the read and
 *  the write inside hal_field_set(). That is precisely where a real interrupt
 *  lands, and precisely what `volatile` does NOT protect you from -- volatile
 *  guarantees the accesses happen, not that they happen atomically.
 *
 *  With no critical section the ISR runs immediately and its update to another
 *  field of the same register is erased by our stale write-back. Inside a
 *  critical section it is DEFERRED and replayed on exit, and nothing is lost.
 *  tests/test_all.c asserts both halves. */
typedef void (*hal_isr_fn)(void *ctx);

void     hal_attach_isr(hal_isr_fn fn, void *ctx);
void     hal_detach_isr(void);
uint32_t hal_isr_runs(void);        /* times the ISR actually executed   */
uint32_t hal_isr_deferred(void);    /* times it was held off by a guard  */

/* Critical section. On real silicon these are __disable_irq()/__enable_irq();
 * here they count nesting so tests can assert the firmware never does an
 * unguarded RMW. */
void     hal_critical_enter(void);
void     hal_critical_exit (void);
unsigned hal_critical_depth(void);

/* Some registers are not storage. A write-only FIFO port pushes a byte into a
 * queue and the value is never readable again; a "clear" register triggers an
 * action. Hardware models such registers by attaching a write hook. */
typedef void (*hal_wr_hook_fn)(uint32_t off, uint32_t val);
void     hal_set_write_hook(hal_wr_hook_fn fn);

/* ---- backend, for the hardware model and for tests ----------------------- */
/* hw_* entry points are the "silicon side" of the register file. Firmware
 * must never call these; the test harness and hw_lane.c do. */
void     hal_reset_all(void);
void     hw_reg_set  (uint32_t off, uint32_t val);   /* hardware drives RO   */
uint32_t hw_reg_get  (uint32_t off);
void     hw_status_set(uint32_t bits);               /* raise RO/W1C flags   */

/* Instrumentation: every access is counted, so a test can assert on bus
 * traffic -- e.g. that the adaptation supervisor does not hammer the bus at
 * line rate, or that a W1C register is never read-modify-written. */
typedef struct {
    uint64_t reads;
    uint64_t writes;
    uint64_t rmw;
    uint64_t unguarded_rmw;   /* RMW performed with no critical section held */
    uint64_t w1c_rmw_bugs;    /* a RMW that touched a W1C register           */
    uint64_t unmapped;        /* access past REG_LAST, or not 4-byte aligned */
} hal_stats_t;

const hal_stats_t *hal_stats(void);
void hal_stats_reset(void);

#endif /* HAL_H */
