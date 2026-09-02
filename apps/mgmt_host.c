/* ===========================================================================
 *  mgmt_host.c — the HOST side of the management bus.
 *
 *  Brings a link up, then does what a board controller actually does: reads a
 *  byte stream off a slow side-channel, hunts for framing, validates CRCs,
 *  tracks sequence numbers, and reassembles the eye diagram from chunks.
 *
 *  The point is that nothing here shares memory with the firmware. Everything
 *  the host knows arrived as bytes over a 1 Mb/s interface while the data path
 *  ran at 100 GBd. If the framing, the CRC or the chunk offsets are wrong, the
 *  eye simply does not reassemble -- which is the honest test of a telemetry
 *  path, and a far better one than printing a struct.
 *
 *  Usage:  mgmt_host [IL_dB] [ppm]
 * =========================================================================*/
#include "fw.h"
#include "hal.h"
#include "hw_lane.h"
#include "eye.h"
#include "mgmt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TICKS 3000u

typedef struct {
    uint8_t  eye[MGMT_EYE_BYTES];
    unsigned eye_seen[MGMT_EYE_BYTES];
    unsigned eye_w, eye_h;
    uint32_t frames_ok, frames_bad, seq_gaps;
    uint32_t last_seq;
    unsigned have_seq;
    uint8_t  taps[NUM_FFE_TAPS + NUM_DFE_TAPS];
    unsigned have_taps;
    uint32_t sym, err, ms_to_up, faults;
    unsigned state, status, vga, tia, ctle, pll;
} host_t;

static void host_frame(host_t *H, const uint8_t *f)
{
    if (mgmt_frame_check(f) != 0) {
        H->frames_bad++;
        return;
    }
    H->frames_ok++;

    const uint16_t seq = (uint16_t)(f[2] | ((uint16_t)f[3] << 8));
    if (H->have_seq && (uint16_t)(H->last_seq + 1u) != seq) {
        H->seq_gaps++;          /* the host CAN see drops -- that is the point */
    }
    H->last_seq = seq;
    H->have_seq = 1u;

    const uint8_t  type = f[1];
    const uint8_t  len  = f[4];
    const uint8_t *p    = &f[5];

    switch (type) {
    case MGMT_T_STATUS:
        if (len >= 6u) {
            H->state = p[0]; H->status = p[1];
            H->vga = p[2]; H->tia = p[3]; H->ctle = p[4]; H->pll = p[5];
        }
        break;
    case MGMT_T_COUNTERS:
        if (len >= 16u) {
            memcpy(&H->sym, &p[0], 4);
            memcpy(&H->err, &p[4], 4);
            memcpy(&H->ms_to_up, &p[8], 4);
            memcpy(&H->faults, &p[12], 4);
        }
        break;
    case MGMT_T_TAPS:
        if (len <= sizeof(H->taps)) {
            memcpy(H->taps, p, len);
            H->have_taps = 1u;
        }
        break;
    case MGMT_T_EYE_META:
        if (len >= 2u) { H->eye_w = p[0]; H->eye_h = p[1]; }
        break;
    case MGMT_T_EYE_CHUNK: {
        if (len < 2u) { break; }
        const unsigned off = (unsigned)p[0] | ((unsigned)p[1] << 8);
        for (unsigned i = 0; i + 2u < len; ++i) {
            if (off + i < MGMT_EYE_BYTES) {
                H->eye[off + i] = p[2u + i];
                H->eye_seen[off + i] = 1u;
            }
        }
        break;
    }
    default:
        break;
    }
}

/* Byte-at-a-time framing. A real host has no frame boundaries handed to it --
 * it hunts for SOF, takes 32 bytes, checks the CRC, and resynchronises by one
 * byte if the check fails. */
static void host_feed(host_t *H, const uint8_t *bytes, size_t n)
{
    static uint8_t win[MGMT_FRAME_BYTES];
    static size_t  fill;

    for (size_t i = 0; i < n; ++i) {
        if (fill == 0u && bytes[i] != MGMT_SOF) {
            continue;                       /* hunting for a start of frame */
        }
        win[fill++] = bytes[i];
        if (fill == MGMT_FRAME_BYTES) {
            if (mgmt_frame_check(win) == 0) {
                host_frame(H, win);
                fill = 0u;
            } else {
                /* Bad CRC: slide the window by one and keep hunting. */
                H->frames_bad++;
                memmove(win, win + 1, MGMT_FRAME_BYTES - 1u);
                fill = MGMT_FRAME_BYTES - 1u;
            }
        }
    }
}

int main(int argc, char **argv)
{
    const double il  = (argc > 1) ? atof(argv[1]) : 20.0;
    const double ppm = (argc > 2) ? atof(argv[2]) : 120.0;

    hw_lane_t hw;
    if (hw_lane_init(&hw, il, AFE_ELECTRICAL, ppm) != 0) {
        return 1;
    }
    hw_lane_attach_platform(&hw, 128u);      /* ~1 Mb/s management bus */

    fw_link_t fw;
    fw_init(&fw);

    host_t H;
    memset(&H, 0, sizeof(H));
    uint8_t buf[4096];

    printf("=====================================================\n");
    printf(" Management-bus host: %0.f dB channel, %.0f ppm\n", il, ppm);
    printf(" data path 100 GBd  |  management bus ~1 Mb/s  |  ratio 200000:1\n");
    printf("=====================================================\n\n");

    uint32_t t = 0u;
    for (; t < MAX_TICKS && !fw_is_up(&fw); ++t) {
        hw_lane_run(&hw, (fw.state == LS_EQ_TRAIN || fw.state == LS_CDR_LOCK) ? 1u : 0u);
        fw_tick(&fw, t);
        host_feed(&H, buf, mgmt_wire_read(buf, sizeof(buf)));
    }
    printf("  link %s after %u ms\n\n",
           fw_is_up(&fw) ? "UP" : "DID NOT COME UP", fw.tm.ms_to_up);

    /* Capture an eye in hardware, then stream it out over the bus. */
    eye_t eye;
    eye_init(&eye, -1.6, 1.6);
    for (unsigned k = 0; k < 8u; ++k) {
        hw_lane_capture_eye(&hw, &eye);
    }
    hw_lane_load_eye_ram(&hw, &eye);

    for (unsigned k = 0; k < 200u; ++k) {
        hw_lane_run(&hw, 0u);
        fw_tick(&fw, t + k);
        host_feed(&H, buf, mgmt_wire_read(buf, sizeof(buf)));
    }

    unsigned covered = 0u;
    for (unsigned i = 0; i < MGMT_EYE_BYTES; ++i) {
        covered += H.eye_seen[i];
    }

    printf("  received over the wire\n");
    printf("    frames ok / bad     %u / %u\n", H.frames_ok, H.frames_bad);
    printf("    sequence gaps       %u\n", H.seq_gaps);
    printf("    bytes dropped by HW %u\n", mgmt_bus_dropped());
    printf("    state / status      %s / 0x%02X   PLL %s\n",
           fw_state_name((link_state_t)H.state), H.status, H.pll ? "locked" : "unlocked");
    printf("    AFE codes           VGA %u  TIA %u  CTLE %u\n", H.vga, H.tia, H.ctle);
    printf("    counters            %u symbols, %u errors, up in %u ms, %u faults\n",
           H.sym, H.err, H.ms_to_up, H.faults);
    if (H.have_taps) {
        printf("    FFE taps           ");
        for (unsigned i = 0; i < NUM_FFE_TAPS; ++i) {
            printf(" %+4d", (int)(int8_t)H.taps[i]);
        }
        printf("\n");
    }
    printf("    eye reassembled     %u/%u bytes (%u x %u)\n\n",
           covered, MGMT_EYE_BYTES, H.eye_w, H.eye_h);

    if (covered == MGMT_EYE_BYTES && H.eye_w > 0u && H.eye_h > 0u) {
        static const char ramp[] = " .:-=+*#%@";
        printf("  eye, reconstructed from %u-byte frames off the wire:\n",
               MGMT_FRAME_BYTES);
        for (unsigned r = 0; r < H.eye_h; ++r) {
            printf("    |");
            for (unsigned c = 0; c < H.eye_w; ++c) {
                const unsigned v = H.eye[r * H.eye_w + c];
                putchar(ramp[(v * 9u) / 255u]);
            }
            printf("|\n");
        }
        printf("    +");
        for (unsigned c = 0; c < H.eye_w; ++c) { putchar('-'); }
        printf("+\n    0%*s2 UI\n", (int)H.eye_w - 5, "");
    }

    hw_lane_free(&hw);
    return (covered == MGMT_EYE_BYTES) ? 0 : 1;
}
