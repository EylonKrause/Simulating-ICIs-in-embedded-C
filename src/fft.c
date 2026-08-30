#include "fft.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Bit-reversal permutation. This is the same permutation as the bit-reverse
 * primitive in the embedded-C bit-manipulation set -- here it is doing its
 * original job. */
static void bit_reverse(cplx *x, size_t n)
{
    size_t j = 0;
    for (size_t i = 1; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            cplx t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }
}

void fft_run(cplx *x, size_t n, int inverse)
{
    if (n < 2u) {
        return;
    }
    bit_reverse(x, n);

    for (size_t len = 2u; len <= n; len <<= 1) {
        const double ang = 2.0 * M_PI / (double)len * (inverse ? 1.0 : -1.0);
        const cplx   wl  = { cos(ang), sin(ang) };
        const size_t half = len >> 1;

        for (size_t i = 0; i < n; i += len) {
            cplx w = { 1.0, 0.0 };
            for (size_t k = 0; k < half; ++k) {
                const cplx u = x[i + k];
                const cplx b = x[i + k + half];
                const cplx v = { b.re * w.re - b.im * w.im,
                                 b.re * w.im + b.im * w.re };

                x[i + k].re        = u.re + v.re;
                x[i + k].im        = u.im + v.im;
                x[i + k + half].re = u.re - v.re;
                x[i + k + half].im = u.im - v.im;

                const cplx nw = { w.re * wl.re - w.im * wl.im,
                                  w.re * wl.im + w.im * wl.re };
                w = nw;
            }
        }
    }

    if (inverse) {
        const double inv_n = 1.0 / (double)n;
        for (size_t i = 0; i < n; ++i) {
            x[i].re *= inv_n;
            x[i].im *= inv_n;
        }
    }
}
