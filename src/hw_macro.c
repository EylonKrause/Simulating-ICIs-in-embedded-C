#include "hw_macro.h"
#include "hal.h"
#include "mgmt.h"

#include <math.h>
#include <string.h>

/* Nearest-neighbour coupling weights. Crosstalk falls off fast with spacing:
 * the lane next door dominates, the one beyond it is down another 10 dB or so
 * in voltage, and beyond that it is not worth the multiply. Both neighbours
 * couple, so an interior lane sees roughly twice the aggression an edge lane
 * does -- which is why the worst lane in a macro is usually one in the middle,
 * not one on the end, and why picking "a lane" to characterise is a decision
 * and not a detail. */
static const double NEIGHBOUR_W[3] = { 1.0, 0.32, 0.10 };
#define NEIGHBOUR_SPAN 3

static int macro_common(hw_macro_t *M, unsigned n_lanes)
{
    if (n_lanes == 0u || n_lanes > MACRO_LANES) {
        return -1;
    }
    M->n_lanes = n_lanes;
    /* Service every lane every block by default: at a 1 ms block and kHz
     * control rates a processor has ample time for eight lanes, and it keeps
     * each lane's loop bandwidth equal to the single-lane case. */
    M->service = n_lanes;
    M->rr      = 0u;
    M->xtalk   = 1.0;
    M->ms      = 0u;

    hal_reset_all();
    hw_macro_set_service(M, MACRO_SERVICE_ALL);

    for (unsigned i = 0; i < n_lanes; ++i) {
        /* Each lane's firmware is initialised through ITS OWN register window.
         * fw_init writes reset values into the register file, so selecting the
         * wrong lane here silently initialises one lane eight times and leaves
         * seven untouched -- which then come up on whatever the register file
         * happened to contain. */
        fw_select_lane(i);
        fw_init(&M->fw[i]);
    }

    /* The management bus and the eye capture window belong to the macro, not
     * to a lane. Attach them once, to lane 0, and tick the bus once per block
     * in hw_macro_block(). */
    hal_select_lane(0u);
    hw_lane_attach_platform(&M->lane[0], 128u);
    return 0;
}

int hw_macro_init(hw_macro_t *M, unsigned n_lanes, double il_db,
                  double il_spread_db, afe_mode_t mode,
                  double ppm_base, double ppm_spread)
{
    memset(M, 0, sizeof(*M));
    if (n_lanes == 0u || n_lanes > MACRO_LANES) {
        return -1;
    }

    for (unsigned i = 0; i < n_lanes; ++i) {
        /* Spread the loss and the reference offset across the lanes. Identical
         * lanes are a trap: a bug that shares state between them produces
         * identical, plausible results and stays hidden. */
        const double frac = (n_lanes > 1u)
                          ? ((double)i / (double)(n_lanes - 1u) - 0.5) * 2.0
                          : 0.0;
        const double il  = il_db + il_spread_db * frac;
        const double ppm = ppm_base + ppm_spread * frac;
        if (hw_lane_init(&M->lane[i], il, mode, ppm) != 0) {
            hw_macro_free(M);
            return -1;
        }
        hw_lane_set_seed(&M->lane[i], i);
    }
    return macro_common(M, n_lanes);
}

int hw_macro_init_sparam(hw_macro_t *M, unsigned n_lanes, const char *s4p,
                         afe_mode_t mode, double ppm_base, double ppm_spread)
{
    memset(M, 0, sizeof(*M));
    if (n_lanes == 0u || n_lanes > MACRO_LANES) {
        return -1;
    }

    for (unsigned i = 0; i < n_lanes; ++i) {
        const double frac = (n_lanes > 1u)
                          ? ((double)i / (double)(n_lanes - 1u) - 0.5) * 2.0
                          : 0.0;
        const double ppm = ppm_base + ppm_spread * frac;
        if (hw_lane_init_sparam(&M->lane[i], s4p, mode, ppm) != 0) {
            hw_macro_free(M);
            return -2;
        }
        hw_lane_set_seed(&M->lane[i], i);
    }
    return macro_common(M, n_lanes);
}

void hw_macro_set_service(hw_macro_t *M, unsigned lanes_per_block)
{
    if (lanes_per_block == 0u || lanes_per_block > M->n_lanes) {
        lanes_per_block = M->n_lanes;
    }
    M->service = lanes_per_block;

    /* A lane serviced every N blocks gets one control iteration every N
     * milliseconds. The loops have not become slower in seconds; they have
     * become slower in ITERATIONS, and iterations are what converge a loop. So
     * every timeout in the firmware stretches by the same factor.
     *
     * Leaving this out is a bug that only appears when the lane count goes up:
     * the single-lane bench passes and the product does not. */
    fw_set_timeout_scale((M->n_lanes + M->service - 1u) / M->service);
}

void hw_macro_free(hw_macro_t *M)
{
    for (unsigned i = 0; i < MACRO_LANES; ++i) {
        hw_lane_free(&M->lane[i]);
    }
    M->n_lanes = 0u;
}

unsigned hw_macro_lanes_up(const hw_macro_t *M)
{
    unsigned n = 0u;
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        if (fw_is_up(&M->fw[i])) {
            n++;
        }
    }
    return n;
}

void hw_macro_set_fec(hw_macro_t *M, unsigned on)
{
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        hw_lane_set_fec(&M->lane[i], on);
    }
}

double hw_macro_xtalk_ratio_db(const hw_macro_t *M, unsigned lane)
{
    if (lane >= M->n_lanes || M->lane[lane].ch.hx == NULL) {
        return -1000.0;
    }
    /* Energy, not peak: crosstalk is a filtered version of a random data
     * stream, so what reaches the slicer is its total power, spread over the
     * whole impulse response. Comparing peaks would flatter it. */
    double sig = 0.0, xt = 0.0;
    const channel_t *ch = &M->lane[lane].ch;
    for (size_t i = 0; i < ch->n; ++i) {
        sig += (double)ch->h[i] * (double)ch->h[i];
    }
    double wsum = 0.0;
    for (int d = 1; d <= NEIGHBOUR_SPAN; ++d) {
        const int a = (int)lane - d;
        const int b = (int)lane + d;
        if (a >= 0) {
            wsum += NEIGHBOUR_W[d - 1] * NEIGHBOUR_W[d - 1];
        }
        if (b < (int)M->n_lanes) {
            wsum += NEIGHBOUR_W[d - 1] * NEIGHBOUR_W[d - 1];
        }
    }
    for (size_t i = 0; i < ch->nx; ++i) {
        xt += (double)ch->hx[i] * (double)ch->hx[i];
    }
    xt *= wsum * M->xtalk * M->xtalk;
    if (sig <= 0.0 || xt <= 0.0) {
        return -1000.0;
    }
    return 10.0 * log10(xt / sig);
}

void hw_macro_block(hw_macro_t *M)
{
    /* One bus for the macro. Ticking it per lane would drain the FIFO eight
     * times too fast and make every backpressure test pass for free. */
    mgmt_bus_tick();

    /* ---- 1. every transmitter, and every lane's own channel ------------- */
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        hal_select_lane(i);
        hw_lane_begin(&M->lane[i]);
        hw_lane_tx(&M->lane[i]);
        hw_lane_channel(&M->lane[i]);
    }

    /* ---- 2. sum the aggressors into each victim ------------------------- */
    /* This is why the transmit phase had to finish first: lane 0 is being hit
     * by the block lane 1 is transmitting NOW, not the one it sent last time.
     * Getting that wrong shifts all the crosstalk by a block, which is
     * invisible in a BER number and completely wrong. */
    if (M->xtalk > 0.0) {
        for (unsigned v = 0; v < M->n_lanes; ++v) {
            hw_lane_xtalk_begin(&M->lane[v]);
            for (int d = 1; d <= NEIGHBOUR_SPAN; ++d) {
                const double w = NEIGHBOUR_W[d - 1] * M->xtalk;
                const int a = (int)v - d;
                const int b = (int)v + d;
                if (a >= 0) {
                    hw_lane_xtalk_add(&M->lane[v], &M->lane[a], w);
                }
                if (b < (int)M->n_lanes) {
                    hw_lane_xtalk_add(&M->lane[v], &M->lane[b], w);
                }
            }
            hw_lane_xtalk_apply(&M->lane[v]);
        }
    }

    /* ---- 3. every receiver --------------------------------------------- */
    for (unsigned i = 0; i < M->n_lanes; ++i) {
        hal_select_lane(i);
        const link_state_t st = M->fw[i].state;
        const hw_mode_t mode =
            (st == LS_EQ_TRAIN || st == LS_CDR_LOCK) ? HW_MODE_TRAIN
          : (st == LS_EQ_VERIFY)                     ? HW_MODE_VERIFY
                                                     : HW_MODE_DATA;
        hw_lane_rx(&M->lane[i], mode);
    }

    /* ---- 4. the supervisor --------------------------------------------- */
    /* ROUND ROBIN, WITH A BUDGET. One processor cannot service eight lanes
     * every block: each lane's tick reads a dozen registers and does a dozen
     * fixed-point updates, and the block is 4096 symbols at 100 GBd. So a
     * fixed number of lanes are serviced per tick and the rest wait.
     *
     * The consequence is not cosmetic: every control loop on a lane runs at
     * the SERVICE rate, not the block rate. At 2 lanes per tick out of 8, each
     * lane's AGC and adaptation see one update every four blocks, so all of
     * their bandwidths drop by four and every timeout has to be scaled to
     * match. Loop bandwidth being a function of how many lanes share the CPU
     * is one of the genuinely awkward facts about multi-lane firmware. */
    const unsigned budget = (M->service < M->n_lanes) ? M->service : M->n_lanes;
    for (unsigned k = 0; k < budget; ++k) {
        const unsigned i = M->rr;
        M->rr = (M->rr + 1u) % M->n_lanes;

        /* Window and per-lane contexts move together. */
        fw_select_lane(i);
        fw_tick(&M->fw[i], M->ms);
        M->services[i]++;
    }

    M->ms++;
}
