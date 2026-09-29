// pg_bench.c
// Offline per-hop DSP bench on blocks from pg_capture. Runs the frozen
// 7c0b425 pipeline (ref_hop.c) and, when built with -DHAVE_NEW, the live
// src/hop.c on the same bytes, and diffs every output bit for bit.
//
//   pg_bench CAP.bin [passes] [topk]

#define _GNU_SOURCE
#include <math.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fftw3.h>

#include "pg_stream.h"
#include "ref_hop.h"
#ifdef HAVE_NEW
#include "hop.h"
#include "dsp.h"
#endif

#ifndef BLOCK_BYTES
#define BLOCK_BYTES (8192 * 8)
#define FFT_SIZE    8192
#endif
#define WISDOM      "/var/lib/quadrf/demos/phasegaze-fftw.wisdom"

typedef struct {
    double lo;
    int32_t gain, resv;
    uint8_t b[BLOCK_BYTES];
} rec_t;

static inline uint64_t ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

#ifdef HAVE_NEW
static int keep_all(double rf, const void *arg) { (void)rf; (void)arg; return 1; }
#endif

/* xorshift so the phase sweep is reproducible */
static uint32_t rng = 0x9E3779B9u;
static inline float frand(void)
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return (float)(rng >> 8) * (1.0f / 16777216.0f);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s CAP.bin [passes] [topk]\n", argv[0]);
        return 1;
    }
    int passes = argc > 2 ? atoi(argv[2]) : 5;
    int topk = argc > 3 ? atoi(argv[3]) : 512;

    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(2, &cs);
    sched_setaffinity(0, sizeof(cs), &cs);

    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    long nrec = ftell(f) / (long)sizeof(rec_t);
    fseek(f, 0, SEEK_SET);
    rec_t *r = malloc((size_t)nrec * sizeof(rec_t));
    if (fread(r, sizeof(rec_t), (size_t)nrec, f) != (size_t)nrec) return 1;
    fclose(f);

    fftwf_import_wisdom_from_filename(WISDOM);
    ref_ctx_t *ref = ref_init();
    pg_point_t *pr = malloc(512 * sizeof(pg_point_t));
    const float vmax = 12.0f;

#ifdef HAVE_NEW
    hop_ctx_t hc;
    if (hop_init(&hc) != 0) { fprintf(stderr, "hop_init failed\n"); return 1; }
    pg_point_t *pn = malloc(512 * sizeof(pg_point_t));
    long mism_pts = 0, mism_vraw = 0, mism_meta = 0, cmp_hops = 0, cmp_pts = 0;
    double max_du = 0, max_di = 0;
#endif

    /* ---- correctness pass: every block once ---- */
#ifdef HAVE_NEW
    for (long i = 0; i < nrec; ++i) {
        ref_out_t ro;
        ref_times_t rt = {0};
        ref_hop(ref, r[i].b, r[i].lo, vmax, topk, pr, &ro, &rt);
        hop_out_t no;
        hop_run(&hc, (const int8_t *)r[i].b, r[i].lo, vmax, topk,
                keep_all, NULL, pn, &no);
        cmp_hops++;
        if (no.adc_peak != ro.adc_peak || no.adc_sumsq != ro.adc_sumsq ||
            no.ran != ro.ran || no.npts != ro.npts ||
            (ro.ran && memcmp(&no.vmax_hop, &ro.vmax_hop, sizeof(float)))) {
            if (mism_meta < 5)
                fprintf(stderr, "meta mismatch rec %ld: peak %d/%d npts %u/%u vmax %g/%g\n",
                        i, ro.adc_peak, no.adc_peak, ro.npts, no.npts,
                        ro.vmax_hop, no.vmax_hop);
            mism_meta++;
        }
        uint32_t n = ro.npts < no.npts ? ro.npts : no.npts;
        cmp_pts += n;
        for (uint32_t p = 0; p < n; ++p) {
            if (memcmp(&pr[p], &pn[p], sizeof(pg_point_t))) {
                mism_pts++;
                double du = fabs(pr[p].u - pn[p].u) + fabs(pr[p].v - pn[p].v);
                double di = fabs(pr[p].intensity - pn[p].intensity);
                if (du > max_du) max_du = du;
                if (di > max_di) max_di = di;
            }
        }
        if (ro.ran) {
            const float *a = ref_vraw(ref), *b = hc.vraw;
            for (int k = ro.k_min; k <= ro.k_max; ++k)
                if (memcmp(&a[k], &b[k], sizeof(float))) mism_vraw++;
        }
    }
    printf("compare: %ld hops, %ld points, point mismatches %ld (max |du|+|dv| %.3g, max |dI| %.3g), "
           "keep-band vraw mismatches %ld, meta mismatches %ld\n",
           cmp_hops, cmp_pts, mism_pts, max_du, max_di, mism_vraw, mism_meta);
#endif

    /* ---- timing ---- */
    int gains[3] = {20, 45, 63};
    printf("%-5s %6s %6s | %7s %7s %7s %7s %7s %7s %8s", "gain", "hits", "pts",
           "adc", "extract", "fft", "power", "cfar", "doa", "ref_tot");
#ifdef HAVE_NEW
    printf(" %8s %7s", "new_tot", "speedup");
#endif
    printf("   (us/hop)\n");
    for (int g = 0; g < 3; ++g) {
        ref_times_t rt = {0};
        long hops = 0, hits = 0, pts = 0;
        uint64_t rtot = 0;
        for (int p = 0; p < passes; ++p)
            for (long i = 0; i < nrec; ++i) {
                if (r[i].gain != gains[g]) continue;
                ref_out_t ro;
                uint64_t a = ns();
                ref_hop(ref, r[i].b, r[i].lo, vmax, topk, pr, &ro, &rt);
                rtot += ns() - a;
                hops++;
                hits += ro.hits;
                pts += ro.npts;
            }
        if (!hops) continue;
        double h = (double)hops * 1e3;
        printf("%-5d %6.1f %6.1f | %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %8.1f",
               gains[g], (double)hits / hops, (double)pts / hops,
               rt.adc / h, rt.extract / h, rt.fft / h, rt.power / h,
               rt.cfar / h, rt.doa / h, rtot / h);
#ifdef HAVE_NEW
        uint64_t ntot = 0;
        for (int p = 0; p < passes; ++p)
            for (long i = 0; i < nrec; ++i) {
                if (r[i].gain != gains[g]) continue;
                hop_out_t no;
                uint64_t a = ns();
                hop_run(&hc, (const int8_t *)r[i].b, r[i].lo, vmax, topk,
                        keep_all, NULL, pn, &no);
                ntot += ns() - a;
            }
        printf(" %8.1f %6.2fx", ntot / h, (double)rtot / (double)ntot);
#endif
        printf("\n");
    }

#ifdef HAVE_NEW
    /* ---- new-path stage breakdown on the gain 45 blocks ---- */
    {
        static const float sc[4] = { 1.0f / 127, 1.0f / 127, 1.0f / 127, 1.0f / 127 };
        float *fin[4] = { (float *)hc.fin[0], (float *)hc.fin[1],
                          (float *)hc.fin[2], (float *)hc.fin[3] };
        float *chp[4] = { (float *)hc.fout[0], (float *)hc.fout[1],
                          (float *)hc.fout[2], (float *)hc.fout[3] };
        uint64_t t_split = 0, t_fft = 0, t_pow = 0, t_cfar = 0;
        long hops = 0;
        for (int p = 0; p < passes; ++p)
            for (long i = 0; i < nrec; ++i) {
                if (r[i].gain != 45) continue;
                int pk;
                int32_t ss;
                uint64_t a0 = ns();
                dsp_cs8_split4((const int8_t *)r[i].b, fin, FFT_SIZE, sc, &pk, &ss);
                uint64_t a1 = ns();
                for (int c = 0; c < 4; ++c) fftwf_execute(hc.plan[c]);
                uint64_t a2 = ns();
                dsp_power4_log_shifted_range(chp, hc.vraw, FFT_SIZE, 4, hc.k_min, hc.k_max);
                uint64_t a3 = ns();
                dsp_cfar_topk(hc.vraw, hc.k_min, hc.k_max, 64, 8, 1.6f, hc.topk, topk);
                uint64_t a4 = ns();
                t_split += a1 - a0; t_fft += a2 - a1; t_pow += a3 - a2; t_cfar += a4 - a3;
                hops++;
            }
        double h = (double)hops * 1e3;
        printf("new stages (gain 45): split4 %.1f  fft %.1f  power %.1f  cfar %.1f us/hop\n",
               t_split / h, t_fft / h, t_pow / h, t_cfar / h);

        /* Same CFAR source compiled in two TUs, same input. */
        uint64_t tr = 0, tn = 0;
        for (int p = 0; p < 2000; ++p) {
            uint64_t a0 = ns();
            ref_cfar(hc.vraw, hc.k_min, hc.k_max, topk);
            uint64_t a1 = ns();
            dsp_cfar_topk(hc.vraw, hc.k_min, hc.k_max, 64, 8, 1.6f, hc.topk, topk);
            uint64_t a2 = ns();
            tr += a1 - a0; tn += a2 - a1;
        }
        printf("cfar same input: ref copy %.1f us, live copy %.1f us\n", tr / 2e6, tn / 2e6);
    }
#endif

    /* ---- direction solver on random phase triples ---- */
    const long NS = 4000000;
    float *ph = malloc(NS * 3 * sizeof(float));
    for (long i = 0; i < NS * 3; ++i) ph[i] = (frand() * 2.0f - 1.0f) * 3.14159265f;
    float sink = 0;
    long accept = 0;
    uint64_t a = ns();
    for (long i = 0; i < NS; ++i) {
        float gx, gy;
        float c = ref_solve(ph[3 * i], ph[3 * i + 1], ph[3 * i + 2], &gx, &gy);
        sink += gx;
        accept += c <= 0.35f;
    }
    double ref_ns = (double)(ns() - a) / NS;
    printf("solver ref: %.1f ns/call, accept %.2f%% (uniform random phases)\n",
           ref_ns, 100.0 * accept / NS);
#ifdef HAVE_NEW
    long diff_acc = 0, diff_g = 0, n_acc = 0;
    double max_dg = 0;
    for (long i = 0; i < NS; ++i) {
        float gx0, gy0, gx1, gy1;
        float c0 = ref_solve(ph[3 * i], ph[3 * i + 1], ph[3 * i + 2], &gx0, &gy0);
        float c1 = dsp_solve_gradient(ph[3 * i], ph[3 * i + 1], ph[3 * i + 2],
                                      0.35f, &gx1, &gy1);
        int a0 = c0 <= 0.35f, a1 = c1 <= 0.35f;
        if (a0 != a1) diff_acc++;
        if (a0 && a1) {
            n_acc++;
            if (gx0 != gx1 || gy0 != gy1 || c0 != c1) {
                diff_g++;
                double d = fabs(gx0 - gx1) + fabs(gy0 - gy1);
                if (d > max_dg) max_dg = d;
            }
        }
    }
    a = ns();
    for (long i = 0; i < NS; ++i) {
        float gx, gy;
        float c = dsp_solve_gradient(ph[3 * i], ph[3 * i + 1], ph[3 * i + 2],
                                     0.35f, &gx, &gy);
        sink += gx;
        accept += c <= 0.35f;
    }
    double new_ns = (double)(ns() - a) / NS;
    printf("solver new: %.1f ns/call (%.1fx). accept/reject flips %ld, accepted with any "
           "bit difference in gx/gy/cost %ld of %ld (max |dg| %.3g)\n",
           new_ns, ref_ns / new_ns, diff_acc, diff_g, n_acc, max_dg);

    /* Near-accepted phases: plane waves plus small noise, the case that
     * actually reaches the sphere. */
    long nd = 0, nflip = 0, nacc2 = 0;
    for (long i = 0; i < NS; ++i) {
        float gx = (frand() * 2 - 1) * 6.0f, gy = (frand() * 2 - 1) * 6.0f;
        float y1 = -0.8660254f * gx + 0.5f * gy, y2 = gy, y3 = 0.8660254f * gx + 0.5f * gy;
        float nse = 0.3f;
        float p[3] = { y1 + nse * (frand() - 0.5f), y2 + nse * (frand() - 0.5f),
                       y3 + nse * (frand() - 0.5f) };
        for (int k = 0; k < 3; ++k) p[k] = atan2f(sinf(p[k]), cosf(p[k]));
        float gx0, gy0, gx1, gy1;
        float c0 = ref_solve(p[0], p[1], p[2], &gx0, &gy0);
        float c1 = dsp_solve_gradient(p[0], p[1], p[2], 0.35f, &gx1, &gy1);
        if ((c0 <= 0.35f) != (c1 <= 0.35f)) nflip++;
        if (c0 <= 0.35f && c1 <= 0.35f) {
            nacc2++;
            if (gx0 != gx1 || gy0 != gy1 || c0 != c1) nd++;
        }
    }
    printf("solver plane-wave set: %ld accepted, flips %ld, bit differences %ld\n",
           nacc2, nflip, nd);
#endif
    if (sink == 12345.0f) printf(" ");
    return 0;
}
