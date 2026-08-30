#include "afe.h"
#include <math.h>
#include <string.h>

/* Sample rate in GHz, so every corner frequency below is in GHz too. */
#define FS_GHZ  (BAUD_RATE_GBD * (double)OSR)

/* ---- first-order sections via the bilinear transform --------------------- */
/* H(s) = 1 / (1 + s/wp)  ->  bilinear  ->  y[n] = b0 x[n] + b1 x[n-1] - a1 y[n-1] */
void bq1_lowpass(biquad1_t *f, double fc_ghz)
{
    memset(f, 0, sizeof(*f));
    if (fc_ghz >= FS_GHZ / 2.0) {          /* above Nyquist: pass through */
        f->b0 = 1.0;
        return;
    }
    const double k = 2.0 * FS_GHZ;                 /* bilinear constant */
    const double w = 2.0 * M_PI * fc_ghz;
    const double b = k / w;
    f->b0 =  1.0 / (1.0 + b);
    f->b1 =  1.0 / (1.0 + b);
    f->a1 = (1.0 - b) / (1.0 + b);
}

/* H(s) = (1 + s/wz) / (1 + s/wp).  DC gain 1, high-frequency gain wp/wz. */
void bq1_zero_pole(biquad1_t *f, double fz_ghz, double fp_ghz)
{
    memset(f, 0, sizeof(*f));
    if (fz_ghz <= 0.0 || fp_ghz <= 0.0) {
        f->b0 = 1.0;
        return;
    }
    const double k = 2.0 * FS_GHZ;
    const double a = k / (2.0 * M_PI * fz_ghz);
    const double b = k / (2.0 * M_PI * fp_ghz);
    f->b0 = (1.0 + a) / (1.0 + b);
    f->b1 = (1.0 - a) / (1.0 + b);
    f->a1 = (1.0 - b) / (1.0 + b);
}

real_t bq1_step(biquad1_t *f, real_t x)
{
    const real_t y = f->b0 * x + f->b1 * f->x1 - f->a1 * f->y1;
    f->x1 = x;
    f->y1 = y;
    return y;
}

/* ---- TIA ---------------------------------------------------------------- */
void tia_set_code(tia_t *t, unsigned code)
{
    if (code >= TIA_GAIN_CODES) {
        code = TIA_GAIN_CODES - 1u;
    }
    t->code = code;

    /* Geometric spacing across the code range -- codes are dB-linear, which is
     * what a real gain DAC gives you. */
    const double frac = (double)code / (double)(TIA_GAIN_CODES - 1u);
    t->rf_ohm = TIA_RF_MIN_OHM * pow(TIA_RF_MAX_OHM / TIA_RF_MIN_OHM, frac);

    /* GAIN-BANDWIDTH TRADEOFF: f_3dB = GBW / R_f. More transimpedance gain
     * buys sensitivity and costs bandwidth, and the lost bandwidth comes back
     * as ISI the equaliser now has to remove. */
    t->bw_ghz = TIA_GBW_GHZ_OHM / t->rf_ohm;
    bq1_lowpass(&t->pole, t->bw_ghz);
}

real_t tia_step(tia_t *t, real_t p_optical_w, real_t noise_unit)
{
    /* Square-law detection: current follows optical POWER. */
    const real_t i_pd = PD_RESPONSIVITY * p_optical_w;
    const real_t i_n  = TIA_IN_NOISE_A * noise_unit;      /* input-referred */
    const real_t v    = (i_pd + i_n) * (real_t)t->rf_ohm;
    return bq1_step(&t->pole, v);
}

/* ---- VGA ---------------------------------------------------------------- */
void vga_set_code(vga_t *v, unsigned code)
{
    if (code >= VGA_GAIN_CODES) {
        code = VGA_GAIN_CODES - 1u;
    }
    v->code = code;
    const double db = VGA_GAIN_MIN_DB +
        (VGA_GAIN_MAX_DB - VGA_GAIN_MIN_DB) * (double)code / (double)(VGA_GAIN_CODES - 1u);
    v->gain_lin = pow(10.0, db / 20.0);
}

double vga_gain_db(const vga_t *v)
{
    return 20.0 * log10(v->gain_lin);
}

real_t vga_step(const vga_t *v, real_t x, real_t noise_unit)
{
    return (real_t)v->gain_lin * (x + VGA_NOISE_V * noise_unit);
}

/* ---- CTLE --------------------------------------------------------------- */
void ctle_set_code(ctle_t *c, unsigned code)
{
    if (code >= CTLE_PEAK_CODES) {
        code = CTLE_PEAK_CODES - 1u;
    }
    c->code    = code;
    c->peak_db = (double)code * CTLE_DB_PER_CODE;

    const double fp = 1.2 * NYQUIST_GHZ;                 /* pole above Nyquist */
    const double fz = fp / pow(10.0, c->peak_db / 20.0); /* zero sets the boost */
    bq1_zero_pole(&c->sec, fz, fp);
}

real_t ctle_step(ctle_t *c, real_t x)
{
    return bq1_step(&c->sec, x);
}

/* ---- whole AFE ---------------------------------------------------------- */
void afe_init(afe_t *a, afe_mode_t mode)
{
    memset(a, 0, sizeof(*a));
    a->mode = mode;
    tia_set_code(&a->tia, 8u);
    vga_set_code(&a->vga, 32u);
    ctle_set_code(&a->ctle, 6u);
}

real_t afe_step(afe_t *a, real_t x, real_t noise_unit)
{
    real_t v;
    if (a->mode == AFE_OPTICAL) {
        /* Map the normalised electrical waveform to an optical power with a
         * finite extinction ratio: power cannot go negative. */
        const real_t p_avg = 1.0e-3;                 /* 0 dBm average */
        const real_t p     = p_avg * (1.0 + 0.8 * x);
        v = tia_step(&a->tia, p > 0.0 ? p : 0.0, noise_unit);
        v -= (real_t)(PD_RESPONSIVITY * p_avg * a->tia.rf_ohm);   /* AC couple */
    } else {
        v = x + (real_t)(VGA_NOISE_V * 0.5) * noise_unit;
    }
    v = vga_step(&a->vga, v, noise_unit * 0.3);
    v = ctle_step(&a->ctle, v);
    return v;
}
