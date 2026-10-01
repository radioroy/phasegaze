// hop.c

#include "hop.h"

#include <math.h>
#include <stdlib.h>

#define DC_GUARD_BINS    4
/* Board spur comb at every multiple of 40 MHz RF, measured at 5200, 5520,
 * 5760, 5800 and 5840 with the LO anywhere in the slice. It is coherent
 * across the four RX, so it solves to one fixed direction and, as the
 * strongest bin of the sweep, sets vmax and dims every real emitter. On the
 * 5745 grid it lands 5 MHz off LO for ch 153/161/169/177; the 5180 and 5500
 * grids put it on DC or outside the keep band. +-3 bins (+-14 kHz) covers
 * the rectangular-window main lobe and first sidelobes of an off-bin tone.
 * IQ imbalance mirrors it about the LO (5800 -> 5810 at LO 5805) at enough
 * level to still own vmax, so the image bins are cut as well. */
#define SPUR_STEP_MHZ    40.0
#define SPUR_GUARD_BINS  3

#define ANTENNA_SPACING_M   0.0455f
#define D_LAMBDA_PER_MHZ    (ANTENNA_SPACING_M / 299.792458f)
#define SCALE_FACTOR_AT_MHZ(f) (2.0f * 3.14159265358979f * D_LAMBDA_PER_MHZ * (f))

#define CFAR_WIN         64
#define CFAR_GUARD       8
#define CFAR_THRESH      1.6f   /* same as csi_sweep; 1.2 filled top-K with noise */
/* Solver cost on the pair-averaged phases. Those always close, so this
 * never rejects a real solve; it only catches the solver's early exit
 * (gx = gy = 0). The plane-wave test is the closure gate in hop_gates_t. */
#define DOA_COST_MAX     0.35f
#define TWO_PI_F         6.283185307179586f
/* Skip a dwell when the CS8 block never leaves the 1-LSB grid (starved ADC). */
#define ADC_PEAK_MIN     3
/* Fixed-offset spur mask. A narrowband emitter sits in one LO slice, so its
 * baseband bin can fire at most once per visit to that LO: 1/n of the hops
 * of an n-LO plan (3.6% on the 28-LO Wi-Fi plan). Board spurs fire in
 * 20-99% of hops at the same offset: quadrf.local ch0 I has a ~209 kHz comb
 * (214, 625, 839, 1040, 1255, 1469 kHz) and an 8207 kHz tone on all RX;
 * quadrf-2 has 602/1204 kHz. Mask a bin at margin x the visits of this
 * worker's busiest LO (the workers do not split LOs evenly), never below
 * margin x H/30 (10% of hops at the default 3); release at half. Below 6
 * LOs 3/n > 50% and a busy emitter cannot be told from a spur, so the mask
 * stays off. The window covers >= 8 visits per LO; 256 hops is ~0.2 s per
 * worker on the Wi-Fi plan. */
#define SPUR_PLAN_MIN    6
#define SPUR_HOPS_PER_LO 30.0f
#define SPUR_PER_VISIT   3.0f
#define SPUR_WIN_MIN     256
#define SPUR_WIN_PER_LO  8
/* Receiver background. Dwell captures at 24 random LOs x 9 gains show the
 * keep-band PSD is set by gain and baseband bin, not by LO: every bin moves
 * 0.8 dB IQR across 4.93-6.08 GHz, the estimator noise. The ch0/ch3
 * ~209 kHz comb, the +-8.2 MHz lines, the ch1/ch2 sigma-delta idle-tone
 * humps at +-4.1 and +-8.3 MHz (gain <= 45) and the filter skirts are all
 * fixed in baseband, and hop order does not move them. An RF emitter sits
 * on a given baseband bin in 1/n of the hops, so a slow per-bin mean of
 * vraw learns the receiver and not the sky. CFAR runs on vraw - bg; point
 * intensities keep vraw. The clip (1 ln = 4.3 dB, ~2 sigma of an 8-dof
 * log-chi-square bin) caps what one strong emitter hop can pull the mean.
 * 256 hops is ~0.2 s per worker at 2280 hops/s over two workers. */
#define BG_TAU_HOPS      256
#define BG_WARM_HOPS     32
#define BG_CLIP          1.0f

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
    h->spur_cnt = calloc(FFT_SIZE, sizeof(uint16_t));
    h->spur_st = calloc(FFT_SIZE, 1);
    h->spur_hops = 0;
    h->spur_plan = 0;
    h->spur_nlo = 0;
    h->spur_margin = SPUR_PER_VISIT;
    for (int g = 0; g < HOP_BG_GAINS; ++g) {
        h->bg_bank[g] = NULL;
        h->bg_bank_n[g] = 0;
        h->bg_bank_mean[g] = 0.0f;
    }
    h->bg = calloc(FFT_SIZE, sizeof(float));
    h->vn = calloc(FFT_SIZE, sizeof(float));
    h->bg_n = 0;
    h->bg_gain = -1;
    h->bg_mean = 0.0f;
    if (!h->vraw || !h->topk || !h->spur_cnt || !h->spur_st || !h->bg || !h->vn)
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
    free(h->vn);
    /* h->bg is either the bootstrap buffer or one of the bank entries. */
    int bg_banked = 0;
    for (int g = 0; g < HOP_BG_GAINS; ++g) {
        if (h->bg_bank[g] == h->bg) bg_banked = 1;
        free(h->bg_bank[g]);
    }
    if (!bg_banked) free(h->bg);
    free(h->spur_st);
    free(h->spur_cnt);
    free(h->topk);
    free(h->vraw);
}

static void spur_visit(hop_ctx_t *h, double lo)
{
    for (int j = 0; j < h->spur_nlo; ++j)
        if (fabs(h->spur_lo[j] - lo) < 1e-3) { h->spur_visits[j]++; return; }
    if (h->spur_nlo < HOP_SPUR_LO_MAX) {
        h->spur_lo[h->spur_nlo] = lo;
        h->spur_visits[h->spur_nlo++] = 1;
    }
}

static void spur_update(hop_ctx_t *h)
{
    int vmax = 0;
    for (int j = 0; j < h->spur_nlo; ++j)
        if (h->spur_visits[j] > vmax) vmax = h->spur_visits[j];
    const int armed = h->spur_nlo >= SPUR_PLAN_MIN;
    const float on = h->spur_margin *
                     fmaxf((float)vmax, (float)h->spur_hops / SPUR_HOPS_PER_LO);
    const float off = 0.5f * on;

    for (int k = h->k_min; k <= h->k_max; ++k) {
        float c = (float)h->spur_cnt[k];
        uint8_t hot = h->spur_st[k] & 1u;
        if (c >= on) hot = 1;
        else if (c < off) hot = 0;
        h->spur_st[k] = armed ? hot : 0;
        h->spur_cnt[k] = 0;
    }
    /* Tone skirts leak into the neighbours (625 kHz shows at 620..634). */
    for (int k = h->k_min; k <= h->k_max; ++k) {
        uint8_t m = h->spur_st[k] & 1u;
        if (k > h->k_min) m |= h->spur_st[k - 1] & 1u;
        if (k < h->k_max) m |= h->spur_st[k + 1] & 1u;
        h->spur_st[k] |= (uint8_t)(m << 1);
    }
    h->spur_hops = 0;
    h->spur_nlo = 0;
}

/* Apply the background learned so far, then fold this hop in. Notched
 * bins (vraw exactly 0) are skipped: the 40 MHz comb notch moves with the
 * LO and would drag the mean down on the bins it visits. */
static const float *bg_apply(hop_ctx_t *h, int learn)
{
    const float *restrict v = h->vraw;
    float *restrict bg = h->bg;
    float *restrict vn = h->vn;
    const int k0 = h->k_min, k1 = h->k_max;
    const int warm = h->bg_n >= BG_WARM_HOPS;
    if (learn && h->bg_n == 0) {
        float sum = 0.0f;
        for (int k = k0; k <= k1; ++k) {
            bg[k] = v[k];
            sum += v[k];
        }
        h->bg_mean = sum / (float)(k1 - k0 + 1);
    } else if (learn) {
        const float a = h->bg_n < BG_TAU_HOPS ? 1.0f / (float)(h->bg_n + 1)
                                              : 1.0f / (float)BG_TAU_HOPS;
        float sum = 0.0f;
        /* One pass: vn uses bg from before this hop. */
        for (int k = k0; k <= k1; ++k) {
            float x = v[k], d = x - bg[k];
            float on = x > 0.0f ? 1.0f : 0.0f;
            vn[k] = on * d;
            bg[k] += on * a * fminf(fmaxf(d, -BG_CLIP), BG_CLIP);
            sum += bg[k];
        }
        h->bg_mean = sum / (float)(k1 - k0 + 1);
    } else if (warm) {
        for (int k = k0; k <= k1; ++k)
            vn[k] = v[k] > 0.0f ? v[k] - bg[k] : 0.0f;
    }
    if (learn)
        h->bg_n++;
    return warm ? vn : v;
}

/* Park the current background under its gain and bring up the one for
 * `gain`. A gain that was never learned starts from bg_n = 0. */
static void bg_select(hop_ctx_t *h, int gain)
{
    if (gain < 0) gain = 0;
    if (gain >= HOP_BG_GAINS) gain = HOP_BG_GAINS - 1;
    if (gain == h->bg_gain) return;
    if (!h->bg_bank[gain]) {
        h->bg_bank[gain] = calloc(FFT_SIZE, sizeof(float));
        if (!h->bg_bank[gain])
            return;
    }
    if (h->bg_gain >= 0 && h->bg_gain < HOP_BG_GAINS) {
        h->bg_bank_n[h->bg_gain] = h->bg_n;
        h->bg_bank_mean[h->bg_gain] = h->bg_mean;
    } else {
        free(h->bg);
    }
    h->bg = h->bg_bank[gain];
    h->bg_n = h->bg_bank_n[gain];
    h->bg_mean = h->bg_bank_mean[gain];
    h->bg_gain = gain;
}

static void spur_reset(hop_ctx_t *h, int plan)
{
    for (int k = h->k_min; k <= h->k_max; ++k) {
        h->spur_cnt[k] = 0;
        h->spur_st[k] = 0;
    }
    h->spur_hops = 0;
    h->spur_nlo = 0;
    h->spur_plan = plan;
}

void hop_run(hop_ctx_t *h, const int8_t *blk, double lo, float vmax, int topk,
             hop_keep_fn keep, const void *keep_arg, const hop_gates_t *gates,
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
    o->hits = o->rej_balance = o->rej_closure = o->rej_spur = 0;
    o->vfold = h->vraw;
    o->fold_off = 0.0f;
    o->fold_norm = 0;
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

    {
        const double bin_mhz = FS_MHZ / (double)FFT_SIZE;
        double f0 = lo + bin_mhz * (double)(h->k_min - half);
        double f1 = lo + bin_mhz * (double)(h->k_max - half);
        for (double s = ceil(f0 / SPUR_STEP_MHZ) * SPUR_STEP_MHZ; s <= f1;
             s += SPUR_STEP_MHZ) {
            int ks = (int)lround((s - lo) / bin_mhz);
            for (int side = -1; side <= 1; side += 2)
                for (int d = -SPUR_GUARD_BINS; d <= SPUR_GUARD_BINS; ++d) {
                    int k = half + side * ks + d;
                    if (k >= h->k_min && k <= h->k_max)
                        h->vraw[k] = 0.0f;
                }
        }
    }

    const int plan_los = gates ? gates->plan_los : 0;
    const float *cv = h->vraw;
    if (gates && gates->bg_norm) {
        bg_select(h, gates->gain);
        /* A short plan parks emitters on fixed bins; keep what was learned
         * on the last long sweep at this gain and stop updating it. */
        cv = bg_apply(h, plan_los >= SPUR_PLAN_MIN);
        if (cv == h->vn) {
            /* Floor sits at the band-average background, so a gain step
             * still moves the whole trace together. */
            o->vfold = h->vn;
            o->fold_off = h->bg_mean;
            o->fold_norm = 1;
        }
    }

    const float cfar_thresh = gates && gates->cfar_thresh > 0.0f
                            ? gates->cfar_thresh : CFAR_THRESH;
    int hsz = dsp_cfar_topk(cv, h->k_min, h->k_max, CFAR_WIN, CFAR_GUARD,
                            cfar_thresh, h->topk, topk);

    const float balance_max = gates ? gates->balance_max : 0.0f;
    const float closure_max = gates ? gates->closure_max : 0.0f;
    const int spur_plan = gates && gates->spur_mask && plan_los >= SPUR_PLAN_MIN
                        ? plan_los : 0;
    if (spur_plan != h->spur_plan)
        spur_reset(h, spur_plan);
    h->spur_margin = gates && gates->spur_margin > 0.0f
                   ? gates->spur_margin : SPUR_PER_VISIT;
    uint32_t npts = 0, hits = 0, rej_balance = 0, rej_closure = 0, rej_spur = 0;
    float vmax_hop = 1e-9f;
    for (int t = 0; t < hsz; ++t) {
        int k = h->topk[t].k;
        int i = (k + half) % FFT_SIZE;
        double rf = lo + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;

        if (keep && !keep(rf, keep_arg)) continue;
        hits++;

        /* Masked bins keep counting so a spur that goes away is released. */
        if (spur_plan) {
            if (h->spur_cnt[k] < UINT16_MAX) h->spur_cnt[k]++;
            if (h->spur_st[k] & 2u) { rej_spur++; continue; }
        }

        float re0 = h->fout[0][i][0], im0 = h->fout[0][i][1];
        float re1 = h->fout[1][i][0], im1 = h->fout[1][i][1];
        float re2 = h->fout[2][i][0], im2 = h->fout[2][i][1];
        float re3 = h->fout[3][i][0], im3 = h->fout[3][i][1];

        if (balance_max > 0.0f) {
            float p0 = re0 * re0 + im0 * im0, p1 = re1 * re1 + im1 * im1;
            float p2 = re2 * re2 + im2 * im2, p3 = re3 * re3 + im3 * im3;
            float a = fmaxf(p0, p1), b = fminf(p0, p1);
            float c = fmaxf(p2, p3), d = fminf(p2, p3);
            float top = fmaxf(a, c);
            float second = fmaxf(fminf(a, c), fmaxf(b, d));
            if (top > balance_max * second) { rej_balance++; continue; }
        }

        float phi10 = atan2f(im1 * re0 - re1 * im0, re1 * re0 + im1 * im0);
        float phi20 = atan2f(im2 * re0 - re2 * im0, re2 * re0 + im2 * im0);
        float phi30 = atan2f(im3 * re0 - re3 * im0, re3 * re0 + im3 * im0);

        /* Must run before the pair averaging below: the averaged phi10,
         * phi20, phi30 always close, so the solver cost cannot see it. */
        if (closure_max > 0.0f) {
            float cl = phi10 - phi20 + phi30;
            cl -= TWO_PI_F * rintf(cl * (1.0f / TWO_PI_F));
            if (fabsf(cl) > closure_max) { rej_closure++; continue; }
        }

        float phi23 = atan2f(im2 * re3 - re2 * im3, re2 * re3 + im2 * im3);
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
    o->hits = hits;
    o->rej_balance = rej_balance;
    o->rej_closure = rej_closure;
    o->rej_spur = rej_spur;

    if (spur_plan) {
        int win = SPUR_WIN_PER_LO * spur_plan;
        if (win < SPUR_WIN_MIN) win = SPUR_WIN_MIN;
        spur_visit(h, lo);
        if (++h->spur_hops >= win)
            spur_update(h);
    }
}
