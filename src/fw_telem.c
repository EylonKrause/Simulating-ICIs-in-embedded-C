/* ===========================================================================
 *  fw_telem.c — telemetry producer. Firmware side of the management bus.
 *
 *  JD: "...data path control, management, and telemetry."
 *
 *  This is what "telemetry" means once you are on silicon and cannot write a
 *  file. It is a paced, framed, checksummed byte stream over an interface
 *  200,000x slower than the data path, and every design decision follows from
 *  that ratio:
 *
 *    - It is a BACKGROUND task. It never blocks the control loops. One frame
 *      per tick at most, and only if the FIFO has room.
 *    - It CHECKS BACKPRESSURE first. REG_MGMT_STAT reports free bytes; if
 *      there is no room the producer simply returns and tries next tick. A
 *      firmware that blasts into a full FIFO silently loses the middle of its
 *      own eye histogram and reports a corrupt one.
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

static uint16_t g_seq;
static unsigned g_phase;        /* round-robin slot                        */
static unsigned g_eye_off;      /* byte offset into the eye being streamed */
static uint32_t g_frames_sent;
static uint32_t g_deferred;     /* ticks where the FIFO had no room        */

void fw_telem_reset(void)
{
    g_seq = 0u;
    g_phase = 0u;
    g_eye_off = 0u;
    g_frames_sent = 0u;
    g_deferred = 0u;
    hal_write32(REG_MGMT_CTRL, MGMT_TX_EN);
}

uint32_t fw_telem_frames(void)   { return g_frames_sent; }
uint32_t fw_telem_deferred(void) { return g_deferred; }

/* Push a built frame out byte by byte. The caller has already checked room. */
static void emit(const uint8_t *frame)
{
    for (unsigned i = 0; i < MGMT_FRAME_BYTES; ++i) {
        hal_write32(REG_MGMT_DATA, frame[i]);
    }
    g_frames_sent++;
    g_seq++;
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

    /* BACKPRESSURE FIRST. Never write without checking. */
    if (hal_read32(REG_MGMT_STAT) < MGMT_FRAME_BYTES) {
        g_deferred++;
        return;
    }

    switch (g_phase) {

    case 0: {   /* link status */
        const uint32_t st = hal_read32(REG_STATUS);
        pay[0] = (uint8_t)L->state;
        pay[1] = (uint8_t)(st & 0xFFu);
        pay[2] = (uint8_t)hal_field_get(REG_AFE_VGA,  VGA_GAIN_MASK,  VGA_GAIN_SHIFT);
        pay[3] = (uint8_t)hal_field_get(REG_AFE_TIA,  TIA_GAIN_MASK,  TIA_GAIN_SHIFT);
        pay[4] = (uint8_t)hal_field_get(REG_AFE_CTLE, CTLE_PEAK_MASK, CTLE_PEAK_SHIFT);
        pay[5] = (uint8_t)((hal_read32(REG_PLL_STAT) & PLL_STAT_LOCKED) ? 1u : 0u);
        (void)mgmt_frame_build(frame, MGMT_T_STATUS, g_seq, pay, 6u);
        emit(frame);
        break;
    }

    case 1: {   /* counters */
        const uint32_t sym = hal_read32(REG_SYM_CNT);
        const uint32_t err = hal_read32(REG_ERR_CNT);
        memcpy(&pay[0], &sym, 4);
        memcpy(&pay[4], &err, 4);
        memcpy(&pay[8], &L->tm.ms_to_up, 4);
        memcpy(&pay[12], &L->tm.faults, 4);
        (void)mgmt_frame_build(frame, MGMT_T_COUNTERS, g_seq, pay, 16u);
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
        (void)mgmt_frame_build(frame, MGMT_T_TAPS, g_seq,
                               pay, (uint8_t)(NUM_FFE_TAPS + NUM_DFE_TAPS));
        emit(frame);
        break;
    }

    case 3: {   /* eye: metadata, then chunks */
        if (g_eye_off == 0u) {
            pay[0] = (uint8_t)MGMT_EYE_W;
            pay[1] = (uint8_t)MGMT_EYE_H;
            pay[2] = 0u;                     /* format: 8-bit log density */
            (void)mgmt_frame_build(frame, MGMT_T_EYE_META, g_seq, pay, 3u);
            emit(frame);
        }
        /* One chunk per tick. A 32x24 eye is 768 bytes -- 30 frames, so about
         * 30 ticks. The link keeps running throughout; this is background. */
        uint8_t n = 0u;
        while (n < MGMT_PAYLOAD_MAX - 2u && g_eye_off < MGMT_EYE_BYTES) {
            pay[2u + n] = eye_byte(g_eye_off);
            g_eye_off++;
            n++;
        }
        pay[0] = (uint8_t)((g_eye_off - n) & 0xFFu);          /* offset lo */
        pay[1] = (uint8_t)(((g_eye_off - n) >> 8) & 0xFFu);   /* offset hi */
        (void)mgmt_frame_build(frame, MGMT_T_EYE_CHUNK, g_seq, pay, (uint8_t)(n + 2u));
        emit(frame);

        if (g_eye_off >= MGMT_EYE_BYTES) {
            g_eye_off = 0u;                   /* start the next capture */
        } else {
            return;                           /* stay on the eye slot */
        }
        break;
    }

    default:
        g_phase = 0u;
        return;
    }

    g_phase = (g_phase + 1u) % 4u;
}
