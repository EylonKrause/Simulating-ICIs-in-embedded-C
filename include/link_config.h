/* ===========================================================================
 *  link_config.h -- the one place where the link is specified.
 *
 *  Target class: a 200 Gb/s/lane PAM4 SerDes lane, of the kind used in current
 *  AI-accelerator chip-to-chip interconnect. Eight lanes form an octal macro;
 *  six macros give 9.6 Tb/s of aggregate per-chip bandwidth.
 *
 *      200 Gb/s/lane PAM4  =  100 GBd  =  50 GHz Nyquist
 *      8 lanes/macro x 6 macros x 200 Gb/s = 9.6 Tb/s
 *
 *  This is a public-domain specification built from published part classes
 *  (e.g. Broadcom and MediaTek 200G/lane PHYs). It is NOT a model of any
 *  specific vendor's silicon and contains no proprietary information.
 * =========================================================================*/
#ifndef LINK_CONFIG_H
#define LINK_CONFIG_H

#include <stddef.h>
#include <stdint.h>

/* ---- link topology ------------------------------------------------------ */
#define LANES_PER_MACRO   8u
#define MACROS_PER_CHIP   6u
#define LANES_PER_CHIP    (LANES_PER_MACRO * MACROS_PER_CHIP)   /* 48 */

/* ---- signalling --------------------------------------------------------- */
#define BITS_PER_SYMBOL   2u                       /* PAM4                    */
#define PAM_LEVELS        4u
#define LANE_RATE_GBPS    200.0                    /* Gb/s per lane           */
#define BAUD_RATE_GBD     (LANE_RATE_GBPS / BITS_PER_SYMBOL)   /* 100 GBd     */
#define NYQUIST_GHZ       (BAUD_RATE_GBD / 2.0)                /* 50 GHz      */

#define CHIP_TBPS         (LANE_RATE_GBPS * LANES_PER_CHIP / 1000.0)  /* 9.6  */

/* ---- simulation resolution ---------------------------------------------- */
/* Samples per unit interval. The eye diagram and the CDR need sub-UI
 * resolution; the adaptation loops run at baud rate on the sampled stream. */
#ifndef OSR
#define OSR               16u
#endif

#define UI_SECONDS        (1.0 / (BAUD_RATE_GBD * 1e9))
#define SAMPLE_RATE_HZ    (BAUD_RATE_GBD * 1e9 * (double)OSR)

/* ---- channel ------------------------------------------------------------ */
/* Insertion loss is modelled as the standard two-term fit
 *      IL(f) [dB] = a_skin * sqrt(f_GHz) + a_diel * f_GHz
 * skin effect goes as sqrt(f), dielectric loss goes as f. The synthesiser in
 * channel.c scales both terms to hit a requested loss at Nyquist. */
#define DEFAULT_IL_DB_AT_NYQUIST   30.0
#define SKIN_FRACTION              0.55   /* share of the loss from skin effect */

/* ---- numeric ------------------------------------------------------------ */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef double real_t;

#endif /* LINK_CONFIG_H */
