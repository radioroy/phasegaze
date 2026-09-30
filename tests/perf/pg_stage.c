// pg_stage.c
// Per-stage cost of hop_run and A/B of the detection chain on pg_rxcap hop
// captures, replayed in capture order so the spur mask and the receiver
// background see the real LO sequence. Uses the kept (second) half of
// every tagged span, the same bytes the workers copy.
//
//   pg_stage CAP.bin passes [PTS_PREFIX]
// Point dump per config: float lo, int32 seq, float rf, float u, float v, float I

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
#include "dsp.h"

#define WISDOM "/var/lib/quadrf/demos/phasegaze-fftw.wisdom"
#define SPAN   (2 * BLOCK_BYTES)

typedef struct {
    double   lo, prev_lo;
    int32_t  gain, mode, hop, nhops, valid, dup, lna_db, vga_db;
    uint32_t seq, sweep;
    uint64_t t_ns;
} rxrec_t;

static inline uint64_t ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int keep_all(double rf, const void *arg) { (void)rf; (void)arg; return 1; }

typedef struct {
    const char *name;
    float bal_db, cfar_db;
    int spur, bg;
} cfg_t;

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s CAP.bin passes [PTS_PREFIX]\n", argv[0]);
        return 1;
    }
    int passes = atoi(argv[2]);
    const char *pfx = argc > 3 ? argv[3] : NULL;

    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(2, &cs);
    sched_setaffinity(0, sizeof(cs), &cs);

    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    long nrec = ftell(f) / (long)(sizeof(rxrec_t) + SPAN);
    fseek(f, 0, SEEK_SET);
    rxrec_t *hd = malloc(sizeof(rxrec_t) * (size_t)nrec);
    uint8_t *blk = malloc((size_t)nrec * BLOCK_BYTES);
    long n = 0;
    static uint8_t span[SPAN];
    for (long i = 0; i < nrec; ++i) {
        rxrec_t h;
        if (fread(&h, sizeof(h), 1, f) != 1 || fread(span, 1, SPAN, f) != SPAN) break;
        if (h.valid != 1 || h.dup) continue;
        hd[n] = h;
        memcpy(blk + (size_t)n * BLOCK_BYTES, span + BLOCK_BYTES, BLOCK_BYTES);
        n++;
    }
    fclose(f);
    int gain = n ? hd[0].gain : 0;
    int nhops = n ? hd[0].nhops : 0;
    fprintf(stderr, "%ld kept blocks, gain %d, plan %d LOs\n", n, gain, nhops);

    fftwf_import_wisdom_from_filename(WISDOM);
    hop_ctx_t hc;
    if (hop_init(&hc) != 0) return 1;
    pg_point_t *pts = malloc(TOPK_BASE * sizeof(pg_point_t));

    /* Stage timings on the same blocks, hop_run's order. */
    {
        const int half = FFT_SIZE / 2;
        static const float scale[CHANNELS] = { 1 / 127.f, 1 / 127.f, 1 / 127.f, 1 / 127.f };
        float *fin[CHANNELS], *chp[CHANNELS];
        for (int c = 0; c < CHANNELS; ++c) {
            fin[c] = (float *)hc.fin[c];
            chp[c] = (float *)hc.fout[c];
        }
        double t_split = 0, t_fft = 0, t_pow = 0, t_bg = 0, t_cfar[5] = { 0 };
        uint64_t hits[5] = { 0 };
        const float cf_db[5] = { 5, 6, 7, 8, 10 };
        float *bg = calloc(FFT_SIZE, sizeof(float)), *vn = calloc(FFT_SIZE, sizeof(float));
        long cnt = 0;
        for (int p = 0; p < passes; ++p)
            for (long i = 0; i < n; ++i) {
                int peak; int32_t sumsq;
                uint64_t a = ns();
                dsp_cs8_split4((const int8_t *)(blk + (size_t)i * BLOCK_BYTES), fin,
                               FFT_SIZE, scale, &peak, &sumsq);
                uint64_t b = ns();
                for (int c = 0; c < CHANNELS; ++c) fftwf_execute(hc.plan[c]);
                uint64_t c2 = ns();
                dsp_power4_log_shifted_range(chp, hc.vraw, FFT_SIZE, 4, hc.k_min, hc.k_max);
                uint64_t d = ns();
                /* Same loop as hop.c bg_apply once warm. */
                for (int k = hc.k_min; k <= hc.k_max; ++k) {
                    float x = hc.vraw[k], e = x - bg[k];
                    float on = x > 0.0f ? 1.0f : 0.0f;
                    vn[k] = on * e;
                    bg[k] += on * (1.0f / 256.0f) * fminf(fmaxf(e, -1.0f), 1.0f);
                }
                uint64_t e2 = ns();
                t_split += (double)(b - a); t_fft += (double)(c2 - b);
                t_pow += (double)(d - c2); t_bg += (double)(e2 - d);
                for (int j = 0; j < 5; ++j) {
                    uint64_t x = ns();
                    int hs = dsp_cfar_topk(hc.vraw, hc.k_min, hc.k_max, 64, 8,
                                           cf_db[j] * 0.23025851f, hc.topk, TOPK_BASE);
                    t_cfar[j] += (double)(ns() - x);
                    hits[j] += (uint64_t)hs;
                }
                cnt++;
                (void)half;
            }
        printf("stage us/hop: split4 %.2f  fft4 %.2f  power+log %.2f  bg(apply+learn) %.2f\n",
               t_split / cnt / 1e3, t_fft / cnt / 1e3, t_pow / cnt / 1e3, t_bg / cnt / 1e3);
        for (int j = 0; j < 5; ++j)
            printf("  cfar %4.1f dB: %.2f us/hop, %.1f hits/hop (top-K %d)\n", cf_db[j],
                   t_cfar[j] / cnt / 1e3, (double)hits[j] / cnt, TOPK_BASE);
        free(bg); free(vn);
    }

    const cfg_t cfgs[] = {
        { "none",        0, 7, 0, 0 },
        { "bal",        10, 7, 0, 0 },
        { "bal+mask",   10, 7, 1, 0 },
        { "bal+bg",     10, 7, 0, 1 },
        { "bal+bg+mask",10, 7, 1, 1 },
        { "bg+mask",     0, 7, 1, 1 },
        { "bal+mask c5",10, 5, 1, 0 },
        { "bal+bg+mask c5", 10, 5, 1, 1 },
        { "bal+bg+mask c6", 10, 6, 1, 1 },
        { "bal+bg+mask c9", 10, 9, 1, 1 },
    };
    for (size_t ci = 0; ci < sizeof(cfgs) / sizeof(cfgs[0]); ++ci) {
        const cfg_t *c = &cfgs[ci];
        hop_free(&hc);
        memset(&hc, 0, sizeof(hc));
        if (hop_init(&hc) != 0) return 1;
        hop_gates_t g = {
            .closure_max = 0.0f,
            .balance_max = c->bal_db > 0 ? powf(10.0f, c->bal_db / 10.0f) : 0.0f,
            .plan_los = nhops, .spur_mask = c->spur, .bg_norm = c->bg, .gain = gain,
            .cfar_thresh = c->cfar_db * 0.23025851f, .spur_margin = 3.0f,
        };
        FILE *pf = NULL;
        if (pfx) {
            char path[512];
            snprintf(path, sizeof(path), "%s_%zu.bin", pfx, ci);
            pf = fopen(path, "wb");
        }
        uint64_t hits = 0, rs = 0, rb = 0, np = 0, t = 0;
        long cnt = 0;
        for (int p = 0; p < passes; ++p)
            for (long i = 0; i < n; ++i) {
                hop_out_t o;
                uint64_t a = ns();
                hop_run(&hc, (const int8_t *)(blk + (size_t)i * BLOCK_BYTES), hd[i].lo,
                        12.0f, TOPK_BASE, keep_all, NULL, &g, pts, &o);
                t += ns() - a;
                cnt++;
                /* Score the last pass only: both learners have converged. */
                if (p != passes - 1) continue;
                hits += o.hits; rs += o.rej_spur; rb += o.rej_balance; np += o.npts;
                for (uint32_t k = 0; pf && k < o.npts; ++k) {
                    float rec[6];
                    int32_t s = (int32_t)hd[i].seq;
                    rec[0] = (float)hd[i].lo;
                    memcpy(&rec[1], &s, 4);
                    rec[2] = pts[k].freq_mhz; rec[3] = pts[k].u; rec[4] = pts[k].v;
                    rec[5] = pts[k].intensity;
                    fwrite(rec, sizeof(rec), 1, pf);
                }
            }
        if (pf) fclose(pf);
        double h = (double)n;
        printf("%-16s cfar %4.1f dB: %7.2f us/hop  hits/hop %6.2f  rej_spur %6.2f  "
               "rej_bal %6.2f  pts/hop %6.2f\n", c->name, c->cfar_db,
               (double)t / cnt / 1e3, hits / h, rs / h, rb / h, np / h);
    }
    return 0;
}
