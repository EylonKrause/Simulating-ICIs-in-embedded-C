#include "mgmt.h"
#include "hal.h"

#include <string.h>

/* ---- CRC-8, ATM polynomial x^8 + x^2 + x + 1 ----------------------------- */
/* Bitwise, not table-driven, and deliberately so: a table costs 256 bytes of
 * ROM on a part where ROM is the scarce resource, to save cycles on a path
 * that runs a few hundred times a second. On the data path you would trade the
 * other way. Knowing which side of that trade you are on is the job. */
uint8_t mgmt_crc8(const uint8_t *p, size_t n)
{
    uint8_t crc = 0xFFu;
    for (size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (unsigned b = 0; b < 8u; ++b) {
            crc = (uint8_t)((crc & 0x80u) ? ((uint8_t)(crc << 1) ^ 0x07u)
                                          : (uint8_t)(crc << 1));
        }
    }
    return crc;
}

int mgmt_frame_build(uint8_t *out, uint8_t type, uint16_t seq,
                     const uint8_t *payload, uint8_t len)
{
    if (out == NULL || len > MGMT_PAYLOAD_MAX) {
        return -1;
    }
    memset(out, 0, MGMT_FRAME_BYTES);
    out[0] = MGMT_SOF;
    out[1] = type;
    out[2] = (uint8_t)(seq & 0xFFu);
    out[3] = (uint8_t)(seq >> 8);
    out[4] = len;
    if (payload != NULL && len > 0u) {
        memcpy(&out[5], payload, len);
    }
    out[MGMT_FRAME_BYTES - 1u] = mgmt_crc8(out, MGMT_FRAME_BYTES - 1u);
    return 0;
}

int mgmt_frame_check(const uint8_t *frame)
{
    if (frame[0] != MGMT_SOF) {
        return -1;
    }
    if (frame[4] > MGMT_PAYLOAD_MAX) {
        return -2;
    }
    if (mgmt_crc8(frame, MGMT_FRAME_BYTES - 1u) != frame[MGMT_FRAME_BYTES - 1u]) {
        return -3;
    }
    return 0;
}

/* ---- the FIFO and the wire ----------------------------------------------- */
#define WIRE_BYTES 65536u

static uint8_t  g_fifo[MGMT_FIFO_BYTES];
static uint32_t g_head, g_tail;          /* free-running; count = head - tail */
static uint32_t g_dropped;
static unsigned g_drain_per_block = 128u;

static uint8_t  g_wire[WIRE_BYTES];
static size_t   g_wire_head, g_wire_tail;

static uint32_t fifo_count(void) { return g_head - g_tail; }

uint32_t mgmt_bus_free(void)
{
    return MGMT_FIFO_BYTES - fifo_count();
}

uint32_t mgmt_bus_dropped(void) { return g_dropped; }

/* A write to REG_MGMT_DATA is a FIFO PUSH, not a store: the value is never
 * readable back. Writing into a full FIFO LOSES the byte -- which is exactly
 * why the firmware must consult REG_MGMT_STAT before every write. */
/* REG_MGMT_STAT IS A LIVE HARDWARE STATUS, SO KEEP IT LIVE.
 *
 * There is one FIFO per macro, but the register that reports its free space
 * lives inside the per-lane window, so the hardware model has to drive every
 * instance of it. Two separate bugs came out of getting that wrong.
 *
 * The first was scope: this was published through whichever lane happened to
 * be selected, so on an eight-lane macro exactly one aperture ever held a real
 * value. The other seven read their reset value of zero, concluded the FIFO
 * was permanently full, and deferred forever -- six lanes never emitted a
 * single telemetry byte, and the whole backpressure mechanism looked healthy
 * because nothing was pushing.
 *
 * The second only appeared once that was fixed: publishing once per block is a
 * SNAPSHOT, and eight producers then each reserve space against the same
 * stale number. Every lane reads "512 free", every lane pushes 32 bytes, and
 * the FIFO is overrun by seven of them -- a check that each producer passes
 * individually and they collectively fail. That is what a shared status
 * register consumed by multiple producers does when it is not live, and it is
 * the same class of error as any read-then-act race.
 *
 * On silicon this register is combinational off the FIFO pointers: a read
 * returns the state at the moment of the read. This function is the model of
 * that, called on every change, so a producer's check reflects what the
 * producers ahead of it have already taken. */
static void publish_free(void)
{
    const unsigned save = hal_current_lane();
    const uint32_t free_now = mgmt_bus_free();
    for (unsigned ln = 0; ln < HAL_MAX_LANES; ++ln) {
        hal_select_lane(ln);
        hw_reg_set(REG_MGMT_STAT, free_now);
    }
    hal_select_lane(save);
}

void mgmt_bus_push(uint32_t val)
{
    if ((hal_read32(REG_MGMT_CTRL) & MGMT_TX_EN) == 0u) {
        return;
    }
    if (fifo_count() >= MGMT_FIFO_BYTES) {
        g_dropped++;
        return;
    }
    g_fifo[g_head % MGMT_FIFO_BYTES] = (uint8_t)(val & 0xFFu);
    g_head++;
    publish_free();
}

void mgmt_bus_reset(void)
{
    g_head = 0u;
    g_tail = 0u;
    g_dropped = 0u;
    g_wire_head = 0u;
    g_wire_tail = 0u;
    hw_reg_set(REG_MGMT_STAT, MGMT_FIFO_BYTES);
}

void mgmt_bus_init(unsigned drain_bytes_per_block)
{
    g_drain_per_block = (drain_bytes_per_block == 0u) ? 1u : drain_bytes_per_block;
    mgmt_bus_reset();
}

void mgmt_bus_tick(void)
{
    /* The bus moves a fixed number of bytes per block -- that IS its bit rate.
     * Everything about telemetry pacing follows from this line. */
    for (unsigned i = 0; i < g_drain_per_block && fifo_count() > 0u; ++i) {
        const uint8_t b = g_fifo[g_tail % MGMT_FIFO_BYTES];
        g_tail++;
        if (g_wire_head - g_wire_tail < WIRE_BYTES) {
            g_wire[g_wire_head % WIRE_BYTES] = b;
            g_wire_head++;
        }
    }
    /* Draining frees space, so republish. See publish_free(). */
    publish_free();
}

size_t mgmt_wire_available(void) { return g_wire_head - g_wire_tail; }

size_t mgmt_wire_read(uint8_t *dst, size_t n)
{
    size_t got = 0u;
    while (got < n && g_wire_tail < g_wire_head) {
        dst[got++] = g_wire[g_wire_tail % WIRE_BYTES];
        g_wire_tail++;
    }
    return got;
}
