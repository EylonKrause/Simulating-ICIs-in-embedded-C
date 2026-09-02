/* ===========================================================================
 *  eye.h -- eye-diagram accumulation and rendering.
 *
 *  This is the software equivalent of pointing a sampling oscilloscope at the
 *  receiver: fold the waveform modulo two unit intervals and build a 2-D
 *  histogram of (phase, amplitude). Persistence comes from the hit counts.
 *
 *  A NOTE ON EYE HEIGHT that is worth having ready. The min/max eye opening is
 *  a WORST-CASE metric, and with unbounded Gaussian noise a single tail
 *  excursion closes it -- so at high noise you will see a "closed" eye while
 *  BER keeps improving. That is not a contradiction. Real links specify eye
 *  contours at a target BER, never absolute min/max. Know which one someone is
 *  quoting before you believe a margin number.
 * =========================================================================*/
#ifndef EYE_H_INCLUDED
#define EYE_H_INCLUDED

#include "link_config.h"
#include <stdint.h>
#include <stddef.h>

#define EYE_W  128u      /* phase bins across two UI */
#define EYE_H  96u       /* amplitude bins           */

typedef struct eye_s {
    uint32_t hist[EYE_H][EYE_W];
    double   vmin, vmax;
    uint64_t hits;
} eye_t;

void eye_init(eye_t *e, double vmin, double vmax);
void eye_accumulate(eye_t *e, const real_t *osr_buf, size_t n, unsigned phase_offset);

/* Grayscale PGM, viewable in anything. Returns 0 on success. */
int  eye_write_pgm(const eye_t *e, const char *path);

/* Terminal rendering, for when you have no image viewer -- which on a bring-up
 * bench is most of the time. */
void eye_print_ascii(const eye_t *e, unsigned cols, unsigned rows);

/* Vertical opening of each of the three PAM4 eyes at the centre phase,
 * expressed in the same units as the samples. Returns the number found. */
unsigned eye_openings(const eye_t *e, double *out, unsigned max_out);

#endif /* EYE_H_INCLUDED */
