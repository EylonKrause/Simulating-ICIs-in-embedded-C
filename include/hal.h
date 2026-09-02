/* ===========================================================================
 *  hal.h — Hardware Abstraction Layer for one SerDes lane.
 *
 *  JD: "Develop low-level drivers and Hardware Abstraction Layers (HAL) to
 *       interface with custom digital signal processing (DSP) hardware blocks
 *       and registers."
 *
 *  THIS IS THE ONLY SEAM between firmware and hardware. Every fw_*.c file
 *  reaches the datapath through these calls and through nothing else. That is
 *  not tidiness -- it is what makes the firmware testable with no silicon:
 *  swap the backing store for a behavioural model and the adaptation loop
 *  closes in CI. (See tests/ and JD item 4.)
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

/* ---- address map, one lane ---------------------------------------------- */
#define LANE_STRIDE        0x100u
#define LANE_BASE(n)       ((uint32_t)(n) * LANE_STRIDE)

#define REG_LANE_ID        0x000u   /* RO  [15:0] id, [31:16] version        */
#define REG_CTRL           0x004u   /* RW                                    */
#define REG_STATUS         0x008u   /* RO + W1C bits                         */
#define REG_AFE_TIA        0x00Cu   /* RW  [3:0]  TIA transimpedance code    */
#define REG_AFE_VGA        0x010u   /* RW  [5:0]  VGA gain code              */
#define REG_AFE_CTLE       0x014u   /* RW  [3:0]  CTLE peaking code          */
#define REG_ADAPT_CTRL     0x018u   /* RW  enables + loop parameters         */
#define REG_ADAPT_STAT     0x01Cu   /* RO  convergence flags                 */
#define REG_FFE_TAP(i)     (0x020u + 4u * (uint32_t)(i))   /* RW, 8 taps     */
#define REG_DFE_TAP(i)     (0x040u + 4u * (uint32_t)(i))   /* RW, 4 taps     */
#define REG_CDR_PHASE      0x050u   /* RW  phase interpolator code           */
#define REG_CDR_FREQ       0x054u   /* RO  accumulated frequency offset      */
#define REG_GRAD_ACC(i)    (0x060u + 4u * (uint32_t)(i))   /* RO/W1C, 8      */
#define REG_DFE_GRAD(i)    (0x080u + 4u * (uint32_t)(i))   /* RO/W1C, 4      */
#define REG_AMP_ACC        0x090u   /* RO/W1C  |y| accumulator, for AGC      */
#define REG_ERR_CNT        0x094u   /* RO/W1C  symbol errors                 */
#define REG_SYM_CNT        0x098u   /* RO/W1C  symbols observed              */
#define REG_EYE_CTRL       0x09Cu   /* RW  margining: phase + level          */
#define REG_EYE_ERR        0x0A0u   /* RO/W1C  errors at that margin point   */
#define REG_PLL_CTRL       0x0A4u   /* RW  [0] PLL_EN, [1] PLL_BYPASS        */
#define REG_PLL_STAT       0x0A8u   /* RO  [0] LOCKED, [15:8] lock counter   */
#define REG_MGMT_CTRL      0x0ACu   /* RW  [0] TX_EN                         */
#define REG_MGMT_STAT      0x0B0u   /* RO  [15:0] free bytes in the TX FIFO  */
#define REG_MGMT_DATA      0x0B4u   /* WO  push one byte into the TX FIFO    */
#define REG_EYE_ADDR       0x0B8u   /* RW  index into the eye capture RAM    */
#define REG_EYE_DATA       0x0BCu   /* RO  byte at REG_EYE_ADDR              */

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

#define NUM_FFE_TAPS       8u
#define NUM_DFE_TAPS       4u

/* ===========================================================================
 *  The firmware-facing API. Nothing else may touch hardware.
 * =========================================================================*/
uint32_t hal_read32 (uint32_t off);
void     hal_write32(uint32_t off, uint32_t val);

/* Read-modify-write of one field. Wrapped in a critical section because a
 * RMW is three bus transactions and `volatile` gives no atomicity: an ISR
 * that touches another field of the same register between our read and our
 * write would have its update silently erased. */
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
} hal_stats_t;

const hal_stats_t *hal_stats(void);
void hal_stats_reset(void);

#endif /* HAL_H */
