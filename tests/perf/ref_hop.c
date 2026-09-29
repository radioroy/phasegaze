// ref_hop.c
// Per-hop DSP exactly as the worker loop in src/main.c ran it at 7c0b425,
// with stage timers. The reference side of pg_bench.

#include "ref_names.h"
#include "ref_dsp.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <fftw3.h>

#include "pg_stream.h"
#include "ref_hop.h"

#define FFT_SIZE         8192
#define CHANNELS         4
#define BLOCK_BYTES      (FFT_SIZE * CHANNELS * 2)
#define LO_STEP_MHZ      20.0
#define FS_MHZ           37.3726
#define DC_GUARD_BINS    4
#define ANTENNA_SPACING_M   0.0455f
#define D_LAMBDA_PER_MHZ    (ANTENNA_SPACING_M / 299.792458f)
#define SCALE_FACTOR_AT_MHZ(f) (2.0f * 3.14159265358979f * D_LAMBDA_PER_MHZ * (f))
#define CFAR_WIN         64
#define CFAR_GUARD       8
#define CFAR_THRESH      1.6f
#define TOPK_BASE        512
#define DOA_COST_MAX     0.35f
#define ADC_PEAK_MIN     3

struct ref_ctx {
    float         *vraw;
    ref_peak_t    *topk;
    fftwf_complex *fin;
    fftwf_complex *fout[CHANNELS];
    fftwf_plan     plan[CHANNELS];
};

static inline uint64_t ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

ref_ctx_t *ref_init(void)
{
    ref_ctx_t *w = calloc(1, sizeof(*w));
    w->vraw = calloc(FFT_SIZE, sizeof(float));
    w->topk = malloc(TOPK_BASE * sizeof(ref_peak_t));
    w->fin = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    for (int c = 0; c < CHANNELS; ++c) {
        w->fout[c] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        w->plan[c] = fftwf_plan_dft_1d(FFT_SIZE, w->fin, w->fout[c],
                                       FFTW_FORWARD, FFTW_MEASURE);
    }
    return w;
}

const float *ref_vraw(const ref_ctx_t *w) { return w->vraw; }

int ref_hop(ref_ctx_t *w, const uint8_t *blk, double lo, float vmax,
            int active_topk, pg_point_t *pts, ref_out_t *o, ref_times_t *t)
{
    const int half = FFT_SIZE / 2;
    int k_min = half - (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    int k_max = half + (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    if (k_min < 0) k_min = 0;
    if (k_max > FFT_SIZE - 1) k_max = FFT_SIZE - 1;
    o->k_min = k_min;
    o->k_max = k_max;

    uint64_t t0 = ns();
    const int8_t *s8 = (const int8_t *)blk;
    int blk_peak = 0;
    int32_t sumsq_i = 0;
    for (int i = 0; i < BLOCK_BYTES; ++i) {
        int sv = (int)s8[i];
        int a = sv < 0 ? -sv : sv;
        blk_peak = a > blk_peak ? a : blk_peak;
        sumsq_i += sv * sv;
    }
    o->adc_peak = blk_peak;
    o->adc_sumsq = (double)sumsq_i;
    uint64_t t1 = ns();
    t->adc += t1 - t0;
    if (blk_peak < ADC_PEAK_MIN) {
        o->ran = 0;
        o->npts = 0;
        o->hits = 0;
        o->vmax_hop = 0.0f;
        return 0;
    }
    o->ran = 1;

    for (int c = 0; c < CHANNELS; ++c) {
        uint64_t a = ns();
        ref_cs8_extract_rotate(s8, c, (float *)w->fin, FFT_SIZE, 1.0f, 0.0f);
        uint64_t b = ns();
        fftwf_execute(w->plan[c]);
        uint64_t e = ns();
        t->extract += b - a;
        t->fft += e - b;
    }
    uint64_t t2 = ns();
    float *chp[CHANNELS] = {
        (float *)w->fout[0], (float *)w->fout[1],
        (float *)w->fout[2], (float *)w->fout[3]
    };
    ref_power4_log_shifted(chp, w->vraw, FFT_SIZE, DC_GUARD_BINS);
    uint64_t t3 = ns();
    t->power += t3 - t2;

    int hsz = ref_cfar_topk(w->vraw, k_min, k_max, CFAR_WIN, CFAR_GUARD,
                            CFAR_THRESH, w->topk, active_topk);
    uint64_t t4 = ns();
    t->cfar += t4 - t3;
    o->hits = hsz;

    uint32_t npts = 0;
    float vmax_hop = 1e-9f;
    for (int tt = 0; tt < hsz; ++tt) {
        int k = w->topk[tt].k;
        int i = (k + half) % FFT_SIZE;
        double rf = lo + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;

        float re0 = w->fout[0][i][0], im0 = w->fout[0][i][1];
        float re1 = w->fout[1][i][0], im1 = w->fout[1][i][1];
        float re2 = w->fout[2][i][0], im2 = w->fout[2][i][1];
        float re3 = w->fout[3][i][0], im3 = w->fout[3][i][1];

        float phi10 = atan2f(im1 * re0 - re1 * im0, re1 * re0 + im1 * im0);
        float phi23 = atan2f(im2 * re3 - re2 * im3, re2 * re3 + im2 * im3);
        float phi20 = atan2f(im2 * re0 - re2 * im0, re2 * re0 + im2 * im0);
        float phi30 = atan2f(im3 * re0 - re3 * im0, re3 * re0 + im3 * im0);
        float phi21 = atan2f(im2 * re1 - re2 * im1, re2 * re1 + im2 * im1);

        phi10 = atan2f(sinf(phi10) + sinf(phi23), cosf(phi10) + cosf(phi23));
        phi30 = atan2f(sinf(phi30) + sinf(phi21), cosf(phi30) + cosf(phi21));

        float gx, gy;
        float cost = ref_solve_gradient(phi10, phi20, phi30, &gx, &gy);
        if (cost > DOA_COST_MAX) continue;

        float scale = SCALE_FACTOR_AT_MHZ((float)rf);
        float u = gx / scale, v = gy / scale;
        if (u * u + v * v > 1.0f) continue;

        float vk = w->vraw[k];
        float inten = vk / vmax;
        if (inten > 1.0f) inten = 1.0f;
        if (inten < 0.0f) inten = 0.0f;
        if (vk > vmax_hop) vmax_hop = vk;

        pts[npts].u = u;
        pts[npts].v = v;
        pts[npts].freq_mhz = (float)rf;
        pts[npts].intensity = inten;
        npts++;
    }
    t->doa += ns() - t4;
    o->npts = npts;
    o->vmax_hop = vmax_hop;
    return (int)npts;
}

int ref_cfar(const float *v, int k0, int k1, int cap)
{
    static ref_peak_t heap[TOPK_BASE];
    return ref_cfar_topk(v, k0, k1, CFAR_WIN, CFAR_GUARD, CFAR_THRESH, heap, cap);
}

/* Isolated solver for the synthetic phase sweep. */
float ref_solve(float p10, float p20, float p30, float *gx, float *gy)
{
    return ref_solve_gradient(p10, p20, p30, gx, gy);
}
