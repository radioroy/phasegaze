// pg_gates.c
// Offline A/B of the hop_run rejection gates on pg_rfdiag captures.
// For each gate setting: CFAR hits, rejections, points and us/hop over the
// same blocks, plus an optional point dump for offline scoring.
//
//   pg_gates CAP.bin gain passes closure_rad balance_db [POINTS.bin]
// closure_rad / balance_db of 0 disable that gate.
// Point dump record: float lo, int32 rec, float rf, float u, float v, float I

#define _GNU_SOURCE
#include <math.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fftw3.h>

#include "hop.h"

#define WISDOM "/var/lib/quadrf/demos/phasegaze-fftw.wisdom"

typedef struct {
    double lo;
    int32_t gain, band, zone, fresh;
    uint8_t b[BLOCK_BYTES];
} rec_t;

static inline uint64_t ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int keep_all(double rf, const void *arg) { (void)rf; (void)arg; return 1; }

static int cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

int main(int argc, char **argv)
{
    if (argc < 6) {
        fprintf(stderr, "usage: %s CAP.bin gain passes closure_rad balance_db [PTS.bin]\n",
                argv[0]);
        return 1;
    }
    int gain = atoi(argv[2]), passes = atoi(argv[3]);
    float cl = (float)atof(argv[4]), bdb = (float)atof(argv[5]);
    /* Captures hold each LO's blocks back to back, not a sweep, so the
     * spur mask would never see an LO mix: leave it off. */
    hop_gates_t g = { .closure_max = cl,
                      .balance_max = bdb > 0 ? powf(10.0f, bdb / 10.0f) : 0.0f };

    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(2, &cs);
    sched_setaffinity(0, sizeof(cs), &cs);

    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    long nall = ftell(f) / (long)sizeof(rec_t);
    fseek(f, 0, SEEK_SET);
    rec_t *r = malloc((size_t)nall * sizeof(rec_t));
    if (fread(r, sizeof(rec_t), (size_t)nall, f) != (size_t)nall) return 1;
    fclose(f);

    fftwf_import_wisdom_from_filename(WISDOM);
    hop_ctx_t hc;
    if (hop_init(&hc) != 0) return 1;
    pg_point_t *pts = malloc(TOPK_BASE * sizeof(pg_point_t));
    FILE *pf = argc > 6 ? fopen(argv[6], "wb") : NULL;

    long hops = 0;
    uint64_t hits = 0, rb = 0, rc = 0, np = 0;
    uint64_t *tp = malloc(sizeof(uint64_t) * (size_t)passes);
    for (int p = 0; p < passes; ++p) {
        uint64_t t = 0;
        for (long i = 0; i < nall; ++i) {
            if (r[i].gain != gain) continue;
            hop_out_t o;
            uint64_t a = ns();
            hop_run(&hc, (const int8_t *)r[i].b, r[i].lo, 12.0f, TOPK_BASE,
                    keep_all, NULL, &g, pts, &o);
            t += ns() - a;
            if (p) continue;
            hops++;
            hits += o.hits; rb += o.rej_balance; rc += o.rej_closure; np += o.npts;
            for (uint32_t k = 0; pf && k < o.npts; ++k) {
                float rec[6];
                int32_t idx = (int32_t)i;
                rec[0] = (float)r[i].lo;
                memcpy(&rec[1], &idx, 4);
                rec[2] = pts[k].freq_mhz; rec[3] = pts[k].u; rec[4] = pts[k].v;
                rec[5] = pts[k].intensity;
                fwrite(rec, sizeof(rec), 1, pf);
            }
        }
        tp[p] = t;
    }
    if (pf) fclose(pf);
    qsort(tp, (size_t)passes, sizeof(uint64_t), cmp_u64);
    double h = (double)hops;
    printf("gain %d closure %.2f balance %.1f dB: hops %ld hits/hop %.2f rej_bal/hop %.2f "
           "rej_cl/hop %.2f pts/hop %.2f us/hop median %.2f min %.2f\n",
           gain, cl, bdb, hops, hits / h, rb / h, rc / h, np / h,
           tp[passes / 2] / h / 1e3, tp[0] / h / 1e3);
    return 0;
}
