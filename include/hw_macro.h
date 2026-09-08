/* ===========================================================================
 *  hw_macro.h -- eight lanes, coupled, with one control processor.
 *
 *  The chip figure in link_config.h is 8 lanes per macro x 6 macros = 48 lanes
 *  = 9.6 Tb/s. Simulating one lane and multiplying by 48 does not test any of
 *  the things that make 48 lanes hard. This does:
 *
 *  CROSSTALK. Lanes sit next to each other. Far-end crosstalk from a
 *  neighbour's transmitter lands on this lane's receiver, and it rises with
 *  frequency, so it is worst exactly at Nyquist where PAM4 keeps its
 *  information. A lane that closes on its own can fail with its neighbours
 *  running -- and it will only fail when they are running, which is why
 *  single-lane bring-up on the bench can pass and the product still not work.
 *  The coupling comes from the off-diagonal S-parameters, so it needs a
 *  Touchstone channel; a fitted loss curve has no off-diagonal to give.
 *
 *  ONE PROCESSOR, MANY LANES. There is not a control CPU per lane. One
 *  processor services all of them round-robin, which means:
 *    - every piece of loop state has to exist per lane (see the context
 *      structs in fw_adapt.c and fw_agc.c),
 *    - the register window has to be switched per lane and restored across
 *      interrupts (see hal_select_lane, and the save/restore around the
 *      handler call in hal.c),
 *    - and each lane's control loops run at the round-robin rate, not the
 *      block rate. Servicing 8 lanes 2 at a time divides every loop bandwidth
 *      by four. That is a real design constraint and it is why the service
 *      budget is a parameter here rather than a hidden constant.
 *
 *  STAGGERED START. Lanes do not all come up at once on real hardware and
 *  they must not here either: bringing eight AGCs up simultaneously into a
 *  coupled channel is the worst case for the loops fighting each other.
 * =========================================================================*/
#ifndef HW_MACRO_H
#define HW_MACRO_H

#include "hw_lane.h"
#include "fw.h"

#define MACRO_LANES        LANES_PER_MACRO      /* 8 */
/* Lanes serviced per block by default. Servicing all of them is realistic at a
 * 1 ms block and kHz control rates. Set M->service lower (see
 * hw_macro_set_service) to model a busier supervisor -- that is not a cosmetic
 * knob, it divides every control loop's bandwidth. */
#define MACRO_SERVICE_ALL  0u

typedef struct {
    hw_lane_t lane[MACRO_LANES];
    fw_link_t fw[MACRO_LANES];
    unsigned  n_lanes;
    unsigned  service;          /* lanes serviced per tick, round-robin */
    unsigned  rr;               /* next lane in the rotation            */
    double    xtalk;            /* coupling multiplier, 1.0 = as measured */
    uint32_t  ms;
    uint64_t  services[MACRO_LANES];
} hw_macro_t;

/* Fitted-loss channels. `il_spread_db` gives each lane a slightly different
 * loss, because real lanes are not identical and identical lanes hide bugs
 * that shared state would otherwise expose. */
int  hw_macro_init(hw_macro_t *M, unsigned n_lanes, double il_db,
                   double il_spread_db, afe_mode_t mode,
                   double ppm_base, double ppm_spread);

/* Touchstone channels. Every lane gets the same measured through response and
 * the same measured crosstalk response; only the reference offsets differ. */
int  hw_macro_init_sparam(hw_macro_t *M, unsigned n_lanes, const char *s4p,
                          afe_mode_t mode, double ppm_base, double ppm_spread);

void hw_macro_free(hw_macro_t *M);

/* How many lanes the supervisor services per block, round robin. Zero (or a
 * value >= the lane count) means all of them. Anything less divides each
 * lane's control-loop bandwidth by lanes/service, so the firmware's timeouts
 * are rescaled to match -- see fw_set_timeout_scale. */
void hw_macro_set_service(hw_macro_t *M, unsigned lanes_per_block);

/* One block across the whole macro: all transmitters, then the channels, then
 * the crosstalk summation, then all receivers, then the supervisor. */
void hw_macro_block(hw_macro_t *M);

/* How many lanes are up. */
unsigned hw_macro_lanes_up(const hw_macro_t *M);

/* Turn the payload over to FEC-encoded traffic on every lane that is up. */
void hw_macro_set_fec(hw_macro_t *M, unsigned on);

/* Aggregate crosstalk seen by a lane, as a power ratio to its own signal,
 * measured from the impulse responses rather than asserted. */
double hw_macro_xtalk_ratio_db(const hw_macro_t *M, unsigned lane);

#endif /* HW_MACRO_H */
