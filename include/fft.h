/* ===========================================================================
 *  fft.h -- iterative radix-2 FFT, in place.
 *
 *  Used by the channel synthesiser: an insertion-loss curve is specified in
 *  the frequency domain and transformed to a causal impulse response. Also
 *  the standard tool for the minimum-phase reconstruction in channel.c.
 * =========================================================================*/
#ifndef FFT_H
#define FFT_H

#include <stddef.h>

typedef struct { double re, im; } cplx;

/* n MUST be a power of two. inverse != 0 also divides by n. */
void fft_run(cplx *x, size_t n, int inverse);

#endif /* FFT_H */
