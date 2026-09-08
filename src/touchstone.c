#define _CRT_SECURE_NO_WARNINGS
#include "touchstone.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* MSVC does not define M_PI without _USE_MATH_DEFINES, and this file has no
 * reason to pull in the project headers just for a constant. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static char g_err[192];

const char *ts_error(void) { return g_err; }

static void ts_fail(const char *msg)
{
    (void)snprintf(g_err, sizeof(g_err), "%s", msg);
}

/* ---- option line --------------------------------------------------------- */
typedef enum { FMT_RI = 0, FMT_MA, FMT_DB } ts_fmt_t;

static double unit_scale(const char *tok)
{
    if (strcmp(tok, "HZ")  == 0) { return 1.0; }
    if (strcmp(tok, "KHZ") == 0) { return 1e3; }
    if (strcmp(tok, "MHZ") == 0) { return 1e6; }
    if (strcmp(tok, "GHZ") == 0) { return 1e9; }
    return 0.0;
}

static void upcase(char *s)
{
    for (; *s != '\0'; ++s) {
        *s = (char)toupper((unsigned char)*s);
    }
}

/* ---- loader -------------------------------------------------------------- */
int ts_load(touchstone_t *ts, const char *path, unsigned ports)
{
    memset(ts, 0, sizeof(*ts));
    g_err[0] = '\0';

    if (ports == 0u) {
        /* Deduce from the extension: the port count is in the file NAME, not
         * in the file, which is one of Touchstone v1's less endearing
         * properties. (v2 fixed it with a [Number of Ports] keyword.) */
        const char *dot = strrchr(path, '.');
        if (dot != NULL && (dot[1] == 's' || dot[1] == 'S')) {
            ports = (unsigned)atoi(dot + 2);
        }
    }
    if (ports == 0u || ports > TS_MAX_PORTS) {
        ts_fail("cannot determine port count (expected .s2p, .s4p, ...)");
        return -1;
    }

    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        ts_fail("cannot open file");
        return -1;
    }

    double   fscale = 1e9;          /* Touchstone v1 default is GHz */
    ts_fmt_t fmt    = FMT_MA;       /* and MA */
    double   z0     = 50.0;

    const size_t per_point = 1u + 2u * (size_t)ports * (size_t)ports;
    double *nums = (double *)malloc(per_point * sizeof(double));
    ts->f_hz = (double *)malloc(TS_MAX_POINTS * sizeof(double));
    ts->s    = (ts_cplx *)malloc(TS_MAX_POINTS * (size_t)ports * ports * sizeof(ts_cplx));
    if (nums == NULL || ts->f_hz == NULL || ts->s == NULL) {
        ts_fail("out of memory");
        fclose(fp);
        ts_free(ts);
        free(nums);
        return -1;
    }

    char line[1024];
    size_t have = 0u;               /* numbers gathered for the current point */
    size_t npt  = 0u;

    while (fgets(line, (int)sizeof(line), fp) != NULL) {
        /* Strip comments. '!' starts one anywhere on the line. */
        char *bang = strchr(line, '!');
        if (bang != NULL) {
            *bang = '\0';
        }

        char *p = line;
        while (*p != '\0' && isspace((unsigned char)*p)) {
            ++p;
        }
        if (*p == '\0') {
            continue;
        }

        if (*p == '#') {
            /* # <unit> <parameter> <format> R <z0> -- any subset, any order */
            char opt[256];
            (void)snprintf(opt, sizeof(opt), "%s", p + 1);
            upcase(opt);
            char *tok = strtok(opt, " \t\r\n");
            while (tok != NULL) {
                const double u = unit_scale(tok);
                if (u > 0.0) {
                    fscale = u;
                } else if (strcmp(tok, "RI") == 0) {
                    fmt = FMT_RI;
                } else if (strcmp(tok, "MA") == 0) {
                    fmt = FMT_MA;
                } else if (strcmp(tok, "DB") == 0) {
                    fmt = FMT_DB;
                } else if (strcmp(tok, "R") == 0) {
                    tok = strtok(NULL, " \t\r\n");
                    if (tok != NULL) {
                        z0 = atof(tok);
                    }
                    continue;
                }
                tok = strtok(NULL, " \t\r\n");
            }
            continue;
        }
        if (*p == '[') {
            continue;               /* v2 keyword lines; ignored */
        }

        /* Data. One frequency point may be spread over several lines, so
         * numbers are accumulated until a full record is present rather than
         * assuming one point per line. Real .s4p files from every tool wrap
         * differently and half of them disagree. */
        for (;;) {
            while (*p != '\0' && (isspace((unsigned char)*p) || *p == ',')) {
                ++p;
            }
            if (*p == '\0') {
                break;
            }
            char *end = NULL;
            const double v = strtod(p, &end);
            if (end == p) {
                break;              /* not a number; skip the rest of the line */
            }
            p = end;

            if (have < per_point) {
                nums[have++] = v;
            }
            if (have == per_point) {
                if (npt >= TS_MAX_POINTS) {
                    ts_fail("too many frequency points");
                    fclose(fp);
                    free(nums);
                    ts_free(ts);
                    return -1;
                }
                ts->f_hz[npt] = nums[0] * fscale;
                for (unsigned q = 0; q < ports * ports; ++q) {
                    const double a = nums[1u + 2u * q];
                    const double b = nums[2u + 2u * q];
                    ts_cplx c;
                    if (fmt == FMT_RI) {
                        c.re = a;
                        c.im = b;
                    } else {
                        const double mag = (fmt == FMT_DB) ? pow(10.0, a / 20.0) : a;
                        const double ph  = b * M_PI / 180.0;
                        c.re = mag * cos(ph);
                        c.im = mag * sin(ph);
                    }
                    ts->s[npt * ports * ports + q] = c;
                }
                npt++;
                have = 0u;
            }
        }
    }
    fclose(fp);
    free(nums);

    if (npt < 2u) {
        ts_fail("fewer than two frequency points");
        ts_free(ts);
        return -1;
    }

    ts->ports = ports;
    ts->n     = npt;
    ts->z0    = z0;
    return 0;
}

void ts_free(touchstone_t *ts)
{
    free(ts->f_hz);
    free(ts->s);
    ts->f_hz = NULL;
    ts->s    = NULL;
    ts->n    = 0u;
}

ts_cplx ts_get(const touchstone_t *ts, size_t k, unsigned i, unsigned j)
{
    /* THE TWO-PORT TRANSPOSE. Touchstone v1 stores a 2-port as
     *     S11 S21 S12 S22
     * -- column major -- but every other port count as
     *     S11 S12 S13 ... S21 S22 ...
     * -- row major. It is a genuine wart in the format, it is not documented
     * anywhere near the data, and reading a .s2p as row-major silently swaps
     * the through path with the reverse one. On a reciprocal passive channel
     * they are nearly equal, so the bug does not show up until it does. */
    unsigned r = i - 1u;
    unsigned c = j - 1u;
    if (ts->ports == 2u) {
        const unsigned t = r;
        r = c;
        c = t;
    }
    return ts->s[k * ts->ports * ts->ports + r * ts->ports + c];
}
