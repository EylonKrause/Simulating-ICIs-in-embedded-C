/* ===========================================================================
 *  afe.h -- analogue front end: photodiode + TIA, VGA, CTLE.
 *
 *  JD: "...gain control (VGA/TIA)."
 *
 *  TIA is the tell that this is an OPTICAL front end. A transimpedance
 *  amplifier converts photodiode CURRENT to voltage; a copper link has no TIA.
 *  Both paths are modelled here and selected by afe_mode_t.
 *
 *  The one thing to say out loud about direct detection: it is SQUARE LAW.
 *  Photocurrent is proportional to optical POWER, not field, so
 *
 *          1 dB optical  =  2 dB electrical
 *
 *  Mixing the two is how a link budget comes out wrong by a factor of two.
 *
 *  TIA GAIN-BANDWIDTH TRADEOFF, the central tension in TIA design: the
 *  bandwidth of a shunt-feedback TIA falls roughly as 1/R_f, so every dB of
 *  transimpedance gain costs bandwidth. Turning the gain up to reach a weak
 *  signal narrows the front end and adds ISI of its own. The model reproduces
 *  this: raising TIA_GAIN raises R_f AND lowers the pole.
 * =========================================================================*/
#ifndef AFE_H
#define AFE_H

#include "link_config.h"
#include "hal.h"

typedef enum { AFE_ELECTRICAL = 0, AFE_OPTICAL = 1 } afe_mode_t;

/* ---- first-order section, used by the CTLE and the TIA pole -------------- */
typedef struct { real_t b0, b1, a1, x1, y1; } biquad1_t;
void   bq1_lowpass(biquad1_t *f, double fc_ghz);
void   bq1_zero_pole(biquad1_t *f, double fz_ghz, double fp_ghz);
real_t bq1_step(biquad1_t *f, real_t x);

/* ---- TIA ---------------------------------------------------------------- */
#define TIA_RF_MIN_OHM   200.0
#define TIA_RF_MAX_OHM   8000.0
#define TIA_GBW_GHZ_OHM  (60.0 * 1000.0)   /* R_f * f_3dB, constant           */
#define PD_RESPONSIVITY  0.8               /* A/W                             */
#define TIA_IN_NOISE_A   12e-6             /* input-referred rms noise current */

typedef struct {
    unsigned  code;
    double    rf_ohm;
    double    bw_ghz;
    biquad1_t pole;
} tia_t;

void   tia_set_code(tia_t *t, unsigned code);
/* Input is optical power in watts; output is volts. */
real_t tia_step(tia_t *t, real_t p_optical_w, real_t noise_unit);

/* ---- VGA ---------------------------------------------------------------- */
#define VGA_GAIN_MIN_DB  (-6.0)
#define VGA_GAIN_MAX_DB  (24.0)
#define VGA_NOISE_V      2.0e-3

typedef struct { unsigned code; double gain_lin; } vga_t;

void   vga_set_code(vga_t *v, unsigned code);
real_t vga_step(const vga_t *v, real_t x, real_t noise_unit);
double vga_gain_db(const vga_t *v);

/* ---- CTLE --------------------------------------------------------------- */
/* A zero below Nyquist and a pole above it: DC gain 1, high-frequency boost
 * equal to fp/fz. Peaking is code-selectable, 0 to ~12 dB.
 *
 * The CTLE sits FIRST, before the ADC, for a reason worth stating: it removes
 * ISI while the signal is still analogue, so the converter needs fewer bits.
 * An FFE placed after a saturated or coarsely-quantised ADC cannot recover
 * what the ADC threw away. */
#define CTLE_DB_PER_CODE 0.8

typedef struct { unsigned code; double peak_db; biquad1_t sec; } ctle_t;

void   ctle_set_code(ctle_t *c, unsigned code);
real_t ctle_step(ctle_t *c, real_t x);

/* ---- whole AFE ---------------------------------------------------------- */
typedef struct {
    afe_mode_t mode;
    tia_t      tia;
    vga_t      vga;
    ctle_t     ctle;
} afe_t;

void   afe_init(afe_t *a, afe_mode_t mode);
/* One oversampled input sample in, one out. noise_unit is a unit-variance
 * Gaussian draw supplied by the caller so the RNG stays in one place. */
real_t afe_step(afe_t *a, real_t x, real_t noise_unit);

#endif /* AFE_H */
