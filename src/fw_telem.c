/* ===========================================================================
 *  fw_telem.c -- telemetry producer. Firmware side of the management bus.
 *
 *  This is what "telemetry" means once you are on silicon and cannot write a
 *  file. It is a paced, framed, checksummed byte stream over an interface
 *  200,000x slower than the data path, and every design decision follows from
 *  that ratio:
 *
 *    - It is a BACKGROUND task. It never blocks the control loops. One frame
 *      per tick at most, and only if the FIFO has room. EXACTLY one: the eye
 *      transfer used to send its metadata frame and the first chunk on the
 *      same tick, 64 bytes after checking there was room for 32, which is the
 *      very thing the next bullet claims does not happen. The metadata frame
 *      now owns a tick of its own.
 *    - It CHECKS BACKPRESSURE first. REG_MGMT_STAT reports free bytes; if
 *      there is no room the producer simply returns and tries next tick. A
 *      firmware that blasts into a full FIFO silently loses the middle of its
 *      own eye histogram and reports a corrupt one.
 *    - Its state is PER LANE, like every other firmware module here. One
 *      control processor serves eight lanes, so a file-static sequence number
 *      or round-robin phase would mean eight lanes sharing one slot: no lane
 *      would ever send a complete eye, and the sequence numbers on the wire
 *      would look like a lane dropping frames when nothing was dropped.
 *    - It ROUND-ROBINS so a long eye transfer cannot starve status. Losing
 *      the link and not being able to say so is the worst failure mode here.
 *    - The eye is read through an INDEXED REGISTER WINDOW (REG_EYE_ADDR then
 *      REG_EYE_DATA), not a pointer. Firmware has no pointer into a hardware
 *      capture RAM; it has an address register and a data register.
 *
 *  Integer only, like every other fw_*.c.
 * =========================================================================*/
#include "fw.h"
#include "hal.h"
#include "mgmt.h"
#include "fixed.h"

#include <string.h>

/* PER-LANE STATE. fw_agc.c and fw_adapt.c already do this; telemetry did not,
 * and the omission is worse here than in a control loop. A shared round-robin
 * phase means lane 0 sends status, lane 1 sends counters, lane 2 sends taps --
 * each lane advancing the SAME phase by one -- so no lane ever emits a
 * complete record set, and a shared g_eye_off means eight lanes take turns
 * writing bytes into what the host reassembles as one eye. */
typedef struct {
    uint16_t seq;
    unsigned phase;         /* round-robin slot                             */
    unsigned eye_off;       /* byte offset into the eye being streamed      */
    unsigned meta_sent;     /* the eye's metadata frame has gone out        */
    uint32_t frames_sent;
    uint32_t deferred;      /* ticks where the FIFO had no room             */
} telem_ctx_t;

static telem_ctx_t g_tm[HAL_MAX_LANES];
static unsigned    g_ln;    /* which lane this module is currently serving  */

void fw_telem_select_lane(unsigned lane)
{
    g_ln = (lane < HAL_MAX_LANES) ? lane : 0u;
}

void fw_telem_reset(void)
{
    memset(&g_tm[g_ln], 0, sizeof(g_tm[g_ln]));
    hal_write32(REG_MGMT_CTRL, MGMT_TX_EN);
}

/* Summed across lanes, because the management bus is one bus per macro and
 * these two numbers describe the BUS, not a lane. */
uint32_t fw_telem_frames(void)
{
    uint32_t n = 0u;
    for (unsigned i = 0; i < HAL_MAX_LANES; ++i) { n += g_tm[i].frames_sent; }
    return n;
}

uint32_t fw_telem_deferred(void)
{
    uint32_t n = 0u;
    for (unsigned i = 0; i < HAL_MAX_LANES; ++i) { n += g_tm[i].deferred; }
    return n;
}

/* Push a built frame out byte by byte. The caller has already checked room. */
static void emit(const uint8_t *frame)
{
    for (unsigned i = 0; i < MGMT_FRAME_BYTES; ++i) {
        hal_write32(REG_MGMT_DATA, frame[i]);
    }
    g_tm[g_ln].frames_sent++;
    g_tm[g_ln].seq++;
}

static uint8_t eye_byte(unsigned index)
{
    /* Indexed window: write the address, read the data. Two bus transactions
     * per byte, which is exactly why you stream a DOWNSAMPLED eye and not the
     * full 12288-bin histogram. */
    hal_write32(REG_EYE_ADDR, index);
    return (uint8_t)(hal_read32(REG_EYE_DATA) & 0xFFu);
}

void fw_telem_step(const fw_link_t *L)
{
    uint8_t frame[MGMT_FRAME_BYTES];
    uint8_t pay[MGMT_PAYLOAD_MAX];
    telem_ctx_t *T = &g_tm[g_ln];

    /* BACKPRESSURE FIRST. Never write without checking. One frame is emitted
     * per call, so one frame's worth of room is the correct thing to demand. */
    if (hal_read32(REG_MGMT_STAT) < MGMT_FRAME_BYTES) {
        T->deferred++;
        return;
    }

    switch (T->phase) {

    case 0: {   /* link status */
        const uint32_t st = hal_read32(REG_STATUS);
        pay[0] = (uint8_t)L->state;
        pay[1] = (uint8_t)(st & 0xFFu);
        pay[2] = (uint8_t)hal_field_get(REG_AFE_VGA,  VGA_GAIN_MASK,  VGA_GAIN_SHIFT);
        pay[3] = (uint8_t)hal_field_get(REG_AFE_TIA,  TIA_GAIN_MASK,  TIA_GAIN_SHIFT);
        pay[4] = (uint8_t)hal_field_get(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT);
        pay[5] = (uint8_t)((hal_read32(REG_PLL_STAT) & PLL_STAT_LOCKED) ? 1u : 0u);
        (void)mgmt_frame_build(frame, MGMT_T_STATUS, T->seq, pay, 6u);
        emit(frame);
        break;
    }

    case 1: {   /* counters */
        /* THESE COME FROM THE LINK STRUCT, NOT FROM THE REGISTERS, and that is
         * the entire point of this frame.
         *
         * REG_SYM_CNT and REG_ERR_CNT are read-and-clear. A read-and-clear
         * register can have exactly ONE consumer, and in LS_UP it already has
         * two ahead of this call: fw_bringup accumulates REG_SYM_CNT into
         * L->tm.symbols and fw_agc_step() drains it again for its average.
         * Reading them here returned 0 and 0 on every frame for the life of
         * the project -- the same defect the bring-up code has a comment
         * warning about twelve lines further up, committed in the function
         * that comment was written to protect.
         *
         * The error count is gone rather than fixed. REG_ERR_CNT only counts
         * when the hardware is handed a training symbol, and there is none in
         * LS_UP, so no ordering makes it meaningful: it would be a field that
         * reads zero because it cannot read anything else. The error rate that
         * means something in traffic is measured by the PCS against a
         * pattern-aligned reference, and it is not on this bus.
         *
         * So the frame carries the 64-bit accumulated symbol count, which is a
         * real number that grows, and drops the field that could not. */
        const uint64_t sym = L->tm.symbols;
        memcpy(&pay[0], &sym, 8);
        memcpy(&pay[8], &L->tm.ms_to_up, 4);
        memcpy(&pay[12], &L->tm.faults, 4);
        (void)mgmt_frame_build(frame, MGMT_T_COUNTERS, T->seq, pay, 16u);
        emit(frame);
        break;
    }

    case 2: {   /* equaliser taps */
        for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
            pay[i] = (uint8_t)(int8_t)hal_read_signed(REG_FFE_TAP(i), TAP_APPLY_BITS);
        }
        for (unsigned i = 0; i < NUM_DFE_TAPS; ++i) {
            pay[NUM_FFE_TAPS + i] =
                (uint8_t)(int8_t)hal_read_signed(REG_DFE_TAP(i), TAP_APPLY_BITS);
        }
        (void)mgmt_frame_build(frame, MGMT_T_TAPS, T->seq,
                               pay, (uint8_t)(NUM_FFE_TAPS + NUM_DFE_TAPS));
        emit(frame);
        break;
    }

    case 3: {   /* eye: metadata, then chunks */
        /* THE METADATA FRAME GETS ITS OWN TICK.
         *
         * It used to be emitted here and then fall straight through into the
         * first chunk: two frames, 64 bytes, on a tick that had checked for
         * 32. On this bus the surplus is silently dropped by the FIFO, so the
         * host would lose the start of an eye and reassemble a corrupt one --
         * which is word for word the failure this file's header says the
         * backpressure check exists to prevent. It did not fail on the bench
         * only because every caller wires the bus at 128 bytes per block,
         * four times the peak demand. A guarantee that holds only because the
         * margin is generous is not a guarantee. */
        if (T->meta_sent == 0u) {
            pay[0] = (uint8_t)MGMT_EYE_W;
            pay[1] = (uint8_t)MGMT_EYE_H;
            pay[2] = 0u;                     /* format: 8-bit log density */
            (void)mgmt_frame_build(frame, MGMT_T_EYE_META, T->seq, pay, 3u);
            emit(frame);
            T->meta_sent = 1u;
            return;                           /* stay on the eye slot */
        }
        /* One chunk per tick. A 32x24 eye is 768 bytes -- 30 frames, so about
         * 30 ticks. The link keeps running throughout; this is background. */
        uint8_t n = 0u;
        while (n < MGMT_PAYLOAD_MAX - 2u && T->eye_off < MGMT_EYE_BYTES) {
            pay[2u + n] = eye_byte(T->eye_off);
            T->eye_off++;
            n++;
        }
        pay[0] = (uint8_t)((T->eye_off - n) & 0xFFu);          /* offset lo */
        pay[1] = (uint8_t)(((T->eye_off - n) >> 8) & 0xFFu);   /* offset hi */
        (void)mgmt_frame_build(frame, MGMT_T_EYE_CHUNK, T->seq, pay, (uint8_t)(n + 2u));
        emit(frame);

        if (T->eye_off >= MGMT_EYE_BYTES) {
            T->eye_off = 0u;                  /* start the next capture */
            T->meta_sent = 0u;
        } else {
            return;                           /* stay on the eye slot */
        }
        break;
    }

    default:
        T->phase = 0u;
        return;
    }

    T->phase = (T->phase + 1u) % 4u;
}
