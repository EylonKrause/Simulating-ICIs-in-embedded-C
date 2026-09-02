/* ===========================================================================
 *  mgmt.h -- the low-speed management interface, and the telemetry framing
 *           that travels over it.
 *
 *  JD: "...data path control, management, and telemetry."
 *
 *  WHY THIS EXISTS. A previous version of this project wrote the eye diagram
 *  to eye.pgm. Real silicon cannot write a file. Everything a host learns
 *  about a lane arrives as bytes over a slow side-channel -- I2C, SPI, a
 *  vendor debug ring -- while the data path itself runs at 100 GBd. The whole
 *  design problem is the ratio between those two rates.
 *
 *      data path        100 GBd     = 2e11 bit/s
 *      management bus     1 Mb/s    = 1e6  bit/s        200,000x slower
 *
 *  So telemetry is not "print what you know". It is:
 *
 *    1. PACKETISE into fixed-size records, so a host can frame without
 *       needing to parse a length-prefixed stream byte by byte.
 *    2. CHECKSUM, because a management bus is slow, long, and noisy, and a
 *       corrupted eye histogram that looks plausible is worse than none.
 *    3. SEQUENCE, so the host can detect the drops that WILL happen.
 *    4. RESPECT BACKPRESSURE. The FIFO is 512 bytes. Push into a full FIFO
 *       and the byte is gone -- so firmware must check free space and pace
 *       itself, not blast.
 *    5. CHUNK. A 32x24 downsampled eye is 768 bytes = 30 records. At this bus
 *       rate that is several milliseconds of streaming, during which the link
 *       keeps running. Telemetry is a background task, never a blocking one.
 *
 *  Frame, 32 bytes fixed:
 *
 *      0       SOF (0xA5)
 *      1       type
 *      2..3    sequence, little endian
 *      4       payload length (0..26)
 *      5..30   payload
 *      31      CRC-8, ATM polynomial, over bytes 0..30
 * =========================================================================*/
#ifndef MGMT_H
#define MGMT_H

#include <stdint.h>
#include <stddef.h>

#define MGMT_FRAME_BYTES   32u
#define MGMT_PAYLOAD_MAX   26u
#define MGMT_SOF           0xA5u

/* Record types. */
#define MGMT_T_STATUS      0x01u   /* link state, lock flags, AFE codes      */
#define MGMT_T_COUNTERS    0x02u   /* symbols, errors, bring-up time         */
#define MGMT_T_TAPS        0x03u   /* FFE and DFE applied codes              */
#define MGMT_T_EYE_META    0x10u   /* eye dimensions, scaling                */
#define MGMT_T_EYE_CHUNK   0x11u   /* one slice of the eye histogram         */

/* Downsampled eye actually streamed. The full 128x96 histogram is 12288 bins;
 * at this bus rate that would take half a second. 32x24 is what a real eye
 * monitor reports, and it is enough to see the opening. */
#define MGMT_EYE_W         32u
#define MGMT_EYE_H         24u
#define MGMT_EYE_BYTES     (MGMT_EYE_W * MGMT_EYE_H)

uint8_t mgmt_crc8(const uint8_t *p, size_t n);

/* ---- the bus (hardware side) -------------------------------------------- */
/* Attaches the write hook so that writes to REG_MGMT_DATA push into the FIFO,
 * and sets how many bytes the bus drains per block (its bit rate). */
void     mgmt_bus_init(unsigned drain_bytes_per_block);
void     mgmt_bus_reset(void);
void     mgmt_bus_tick(void);          /* drain FIFO -> wire, once per block */
void     mgmt_bus_push(uint32_t val);  /* the REG_MGMT_DATA write side effect */
uint32_t mgmt_bus_free(void);          /* bytes of room, mirrored to a reg   */
uint32_t mgmt_bus_dropped(void);       /* pushes into a full FIFO            */

/* ---- the wire (host side) ------------------------------------------------ */
size_t   mgmt_wire_available(void);
size_t   mgmt_wire_read(uint8_t *dst, size_t n);

/* ---- framing helpers ----------------------------------------------------- */
/* Build a frame into `out` (32 bytes). Returns 0 on success. */
int  mgmt_frame_build(uint8_t *out, uint8_t type, uint16_t seq,
                      const uint8_t *payload, uint8_t len);
/* Validate a 32-byte frame: SOF, length and CRC. Returns 0 if good. */
int  mgmt_frame_check(const uint8_t *frame);

#endif /* MGMT_H */
