#include "eye.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void eye_init(eye_t *e, double vmin, double vmax)
{
    memset(e, 0, sizeof(*e));
    e->vmin = vmin;
    e->vmax = vmax;
}

void eye_accumulate(eye_t *e, const real_t *osr_buf, size_t n, unsigned phase_offset)
{
    const size_t period = (size_t)OSR * 2u;      /* fold over two UI */
    const double span   = e->vmax - e->vmin;
    if (span <= 0.0) {
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        const size_t ph = (i + phase_offset) % period;
        const size_t bx = ph * EYE_W / period;

        double t = ((double)osr_buf[i] - e->vmin) / span;
        if (t < 0.0) { t = 0.0; }
        if (t > 1.0) { t = 1.0; }
        size_t by = (size_t)((1.0 - t) * (double)(EYE_H - 1u) + 0.5);
        if (by >= EYE_H) { by = EYE_H - 1u; }

        e->hist[by][bx]++;
        e->hits++;
    }
}

int eye_write_pgm(const eye_t *e, const char *path)
{
    uint32_t peak = 1u;
    for (unsigned y = 0; y < EYE_H; ++y) {
        for (unsigned x = 0; x < EYE_W; ++x) {
            if (e->hist[y][x] > peak) {
                peak = e->hist[y][x];
            }
        }
    }

    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        return -1;
    }
    fprintf(f, "P5\n%u %u\n255\n", EYE_W, EYE_H);
    for (unsigned y = 0; y < EYE_H; ++y) {
        for (unsigned x = 0; x < EYE_W; ++x) {
            /* Log compression: without it a handful of dense level rails
             * saturate the image and the interesting transition density
             * disappears. Same reason a scope has persistence intensity. */
            const double v = log1p((double)e->hist[y][x]) / log1p((double)peak);
            const int    p = (int)(v * 255.0 + 0.5);
            fputc(p > 255 ? 255 : p, f);
        }
    }
    fclose(f);
    return 0;
}

void eye_print_ascii(const eye_t *e, unsigned cols, unsigned rows)
{
    static const char ramp[] = " .:-=+*#%@";
    const unsigned nramp = (unsigned)(sizeof(ramp) - 2u);

    if (cols == 0u || cols > EYE_W) { cols = EYE_W; }
    if (rows == 0u || rows > EYE_H) { rows = EYE_H; }

    uint32_t peak = 1u;
    for (unsigned y = 0; y < EYE_H; ++y) {
        for (unsigned x = 0; x < EYE_W; ++x) {
            if (e->hist[y][x] > peak) { peak = e->hist[y][x]; }
        }
    }

    for (unsigned r = 0; r < rows; ++r) {
        const double amp = e->vmax - (e->vmax - e->vmin) * ((double)r + 0.5) / (double)rows;
        printf("  %+6.3f |", amp);
        for (unsigned c = 0; c < cols; ++c) {
            /* box-average the histogram down to the requested size */
            uint32_t acc = 0u;
            const unsigned y0 = r * EYE_H / rows, y1 = (r + 1u) * EYE_H / rows;
            const unsigned x0 = c * EYE_W / cols, x1 = (c + 1u) * EYE_W / cols;
            for (unsigned y = y0; y < y1; ++y) {
                for (unsigned x = x0; x < x1; ++x) {
                    acc += e->hist[y][x];
                }
            }
            const double v = log1p((double)acc) / log1p((double)peak);
            unsigned idx = (unsigned)(v * (double)nramp + 0.5);
            if (idx > nramp) { idx = nramp; }
            putchar(ramp[idx]);
        }
        putchar('\n');
    }
    printf("         +");
    for (unsigned c = 0; c < cols; ++c) { putchar('-'); }
    printf("\n          0");
    for (unsigned c = 1; c + 8u < cols; ++c) { putchar(' '); }
    printf("2 UI\n");
}

unsigned eye_openings(const eye_t *e, double *out, unsigned max_out)
{
    /* Look one quarter-UI wide about the centre of the fold, and find the
     * amplitude bands that were never hit -- those are the open eyes. */
    const unsigned cx = EYE_W / 4u;                  /* centre of the first UI */
    const unsigned half = EYE_W / 32u ? EYE_W / 32u : 1u;

    unsigned found = 0u;
    unsigned run   = 0u;
    for (unsigned y = 0; y < EYE_H && found < max_out; ++y) {
        uint32_t hits = 0u;
        for (unsigned x = (cx > half ? cx - half : 0u); x <= cx + half && x < EYE_W; ++x) {
            hits += e->hist[y][x];
        }
        if (hits == 0u) {
            run++;
        } else {
            if (run > 1u) {
                out[found++] = (double)run / (double)EYE_H * (e->vmax - e->vmin);
            }
            run = 0u;
        }
    }
    if (run > 1u && found < max_out) {
        out[found++] = (double)run / (double)EYE_H * (e->vmax - e->vmin);
    }
    return found;
}
