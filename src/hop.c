// hop.c

#include "hop.h"

#include <math.h>
#include <stdlib.h>

#define DC_GUARD_BINS    4

#define ANTENNA_SPACING_M   0.0455f
#define D_LAMBDA_PER_MHZ    (ANTENNA_SPACING_M / 299.792458f)
#define SCALE_FACTOR_AT_MHZ(f) (2.0f * 3.14159265358979f * D_LAMBDA_PER_MHZ * (f))

#define CFAR_WIN         64
#define CFAR_GUARD       8
#define CFAR_THRESH      1.6f   /* same as csi_sweep; 1.2 filled top-K with noise */
/* Drop DOA solves whose 3-baseline residual is not a plane wave. */
#define DOA_COST_MAX     0.35f
/* Skip a dwell when the CS8 block never leaves the 1-LSB grid (starved ADC). */
#define ADC_PEAK_MIN     3

int hop_init(hop_ctx_t *h)
{
    const int half = FFT_SIZE / 2;
    /* Keep detections inside the digital-filter passband (±LO_STEP/2 around DC). */
    h->k_min = half - (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    h->k_max = half + (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    if (h->k_min < 0) h->k_min = 0;
    if (h->k_max > FFT_SIZE - 1) h->k_max = FFT_SIZE - 1;

    h->vraw = calloc(FFT_SIZE, sizeof(float));
    h->topk = malloc(TOPK_BASE * sizeof(dsp_peak_t));
    if (!h->vraw || !h->topk)
        return -1;
    /* FFTW planning is not thread-safe; plan here, execute in the workers.
     * MEASURE picks a 4x-FFT set that runs in 150 us instead of 228 us
     * with ESTIMATE. Only the first plan measures; the rest hit wisdom,
     * so all four channels run the same algorithm as before the split. */
    for (int c = 0; c < CHANNELS; ++c) {
        h->fin[c] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        h->fout[c] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        if (!h->fin[c] || !h->fout[c])
            return -1;
        h->plan[c] = fftwf_plan_dft_1d(FFT_SIZE, h->fin[c], h->fout[c],
                                       FFTW_FORWARD, FFTW_MEASURE);
        if (!h->plan[c])
            return -1;
    }
    return 0;
}

void hop_free(hop_ctx_t *h)
{
    for (int c = 0; c < CHANNELS; ++c) {
        if (h->plan[c]) fftwf_destroy_plan(h->plan[c]);
        if (h->fout[c]) fftwf_free(h->fout[c]);
        if (h->fin[c]) fftwf_free(h->fin[c]);
    }
    free(h->topk);
    free(h->vraw);
}

void hop_run(hop_ctx_t *h, const int8_t *blk, double lo, float vmax, int topk,
             hop_keep_fn keep, const void *keep_arg,
             pg_point_t *pts, hop_out_t *o)
{
    const int half = FFT_SIZE / 2;
    static const float scale[CHANNELS] = {
        1.0f / 127.0f, 1.0f / 127.0f, 1.0f / 127.0f, 1.0f / 127.0f
    };

    float *fin[CHANNELS] = {
        (float *)h->fin[0], (float *)h->fin[1], (float *)h->fin[2], (float *)h->fin[3]
    };
    int peak;
    int32_t sumsq;
    dsp_cs8_split4(blk, fin, FFT_SIZE, scale, &peak, &sumsq);
    o->adc_peak = peak;
    o->adc_sumsq = (double)sumsq;
    o->npts = 0;
    o->vmax_hop = 1e-9f;
    o->ran = peak >= ADC_PEAK_MIN;
    if (!o->ran)
        return;

    for (int c = 0; c < CHANNELS; ++c)
        fftwf_execute(h->plan[c]);

    float *chp[CHANNELS] = {
        (float *)h->fout[0], (float *)h->fout[1],
        (float *)h->fout[2], (float *)h->fout[3]
    };
    /* Nothing outside the keep-band is read: CFAR, the point intensities
     * and the spectrum fold all stay in [k_min, k_max]. */
    dsp_power4_log_shifted_range(chp, h->vraw, FFT_SIZE, DC_GUARD_BINS,
                                 h->k_min, h->k_max);

    int hsz = dsp_cfar_topk(h->vraw, h->k_min, h->k_max, CFAR_WIN, CFAR_GUARD,
                            CFAR_THRESH, h->topk, topk);

    uint32_t npts = 0;
    float vmax_hop = 1e-9f;
    for (int t = 0; t < hsz; ++t) {
        int k = h->topk[t].k;
        int i = (k + half) % FFT_SIZE;
        double rf = lo + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;

        if (keep && !keep(rf, keep_arg)) continue;

        float re0 = h->fout[0][i][0], im0 = h->fout[0][i][1];
        float re1 = h->fout[1][i][0], im1 = h->fout[1][i][1];
        float re2 = h->fout[2][i][0], im2 = h->fout[2][i][1];
        float re3 = h->fout[3][i][0], im3 = h->fout[3][i][1];

        float phi10 = atan2f(im1 * re0 - re1 * im0, re1 * re0 + im1 * im0);
        float phi23 = atan2f(im2 * re3 - re2 * im3, re2 * re3 + im2 * im3);
        float phi20 = atan2f(im2 * re0 - re2 * im0, re2 * re0 + im2 * im0);
        float phi30 = atan2f(im3 * re0 - re3 * im0, re3 * re0 + im3 * im0);
        float phi21 = atan2f(im2 * re1 - re2 * im1, re2 * re1 + im2 * im1);

        phi10 = atan2f(sinf(phi10) + sinf(phi23), cosf(phi10) + cosf(phi23));
        phi30 = atan2f(sinf(phi30) + sinf(phi21), cosf(phi30) + cosf(phi21));

        float gx, gy;
        float cost = dsp_solve_gradient(phi10, phi20, phi30, DOA_COST_MAX, &gx, &gy);
        if (cost > DOA_COST_MAX) continue;

        float scale_f = SCALE_FACTOR_AT_MHZ((float)rf);
        float u = gx / scale_f, v = gy / scale_f;
        if (u * u + v * v > 1.0f) continue;

        float vk = h->vraw[k];
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
    o->npts = npts;
    o->vmax_hop = vmax_hop;
}
