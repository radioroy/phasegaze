// main.c — PhaseGaze backend
//
// Headless DSP pipeline for the QuadRF 4-lane CSI SDR:
//   /dev/csi_stream0 -> NEON CS8 unpack -> FFTW -> log power
//   -> band-limited CA-CFAR -> phase-gradient DOA -> packed WS stream.
//
// Browser renders the hemisphere (web/ served on the same port).
//
// Build: cmake (quadrf-phasegaze)
// Run:   quadrf-phasegaze [--port 8001] [--web DIR]

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>

#include <fftw3.h>

#include "csi_dev.h"
#include "dsp.h"
#include "server.h"
#include "pg_stream.h"
#include "external/mongoose.h"

// ---------------------------------------------------------------------------
// 4-lane CSI: 38 Msps. FFT 8192 keeps bin width ~4.6 kHz, matching the
// old 4096 / 18 Msps two-lane setup. LO step is the digital-filter BW.
// ---------------------------------------------------------------------------

#define DEVICE_PATH      "/dev/csi_stream0"
#define FFT_SIZE         8192
#define CHANNELS         4
#define BYTES_PER_IQ     2
#define BYTES_PER_FRAME  (CHANNELS * BYTES_PER_IQ)
#define BLOCK_BYTES      (FFT_SIZE * BYTES_PER_FRAME)

#define HW_LO_MIN_MHZ    4900.0
#define HW_LO_MAX_MHZ    6100.0
#define RF_GAIN_MAX      63
#define LO_STEP_MHZ      20.0
#define FS_MHZ           38.0
#define DC_GUARD_BINS    4

#define ANTENNA_SPACING_M   0.0455f
#define D_LAMBDA_PER_MHZ    (ANTENNA_SPACING_M / 299.792458f)
#define SCALE_FACTOR_AT_MHZ(f) (2.0f * 3.14159265358979f * D_LAMBDA_PER_MHZ * (f))

#define CFAR_WIN         64
#define CFAR_GUARD       8
#define CFAR_THRESH      1.6f   /* same as csi_sweep; 1.2 filled top-K with noise */
#define TOPK_BASE        512
/* Drop DOA solves whose 3-baseline residual is not a plane wave. */
#define DOA_COST_MAX     0.35f
/* Skip a dwell when the CS8 block never leaves the 1-LSB grid (starved ADC). */
#define ADC_PEAK_MIN     3

#define MAX_LO_STEPS     128
#define MAX_FRAME_POINTS 16384
#define MAX_BANDS        32


typedef struct {
    pthread_mutex_t mtx;
    uint32_t epoch;
    double   lo_start, lo_end;
    float    bands[MAX_BANDS][2];
    int      nbands;
    float    output_fraction;
    int      gain;
    int      spectrum;
} settings_t;

static settings_t g_set = {
    .mtx = PTHREAD_MUTEX_INITIALIZER,
    .epoch = 1,
    .lo_start = HW_LO_MIN_MHZ,
    .lo_end = HW_LO_MAX_MHZ,
    .nbands = 0,
    .output_fraction = 1.00f,
    .gain = 45,
    .spectrum = 0,
};

static csi_dev_t g_dev;
static volatile sig_atomic_t g_quit = 0;
static volatile float g_fps = 0.0f;
static volatile uint32_t g_last_points = 0;
static volatile int g_adc_peak = 0;
static volatile float g_adc_rms = 0.0f;

static void on_sig(int sig) { (void)sig; g_quit = 1; }

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int on_control(const char *msg, size_t len, void *user)
{
    (void)user;
    struct mg_str json = mg_str_n(msg, len);
    double d;
    int changed = 0;

    char *type = mg_json_get_str(json, "$.type");
    if (!type) return 0;

    if (strcmp(type, "get_state") == 0) {
        changed = 1;
    } else if (strcmp(type, "set") == 0) {
        int apply_gain = -1;
        pthread_mutex_lock(&g_set.mtx);

        if (mg_json_get_num(json, "$.gain", &d)) {
            int g = (int)d;
            if (g < 0) g = 0;
            if (g > RF_GAIN_MAX) g = RF_GAIN_MAX;
            if (g != g_set.gain) {
                g_set.gain = g;
                apply_gain = g;
                changed = 1;
            }
        }
        if (mg_json_get_num(json, "$.output_fraction", &d)) {
            float f = (float)d;
            if (f < 0.01f) f = 0.01f;
            if (f > 1.0f) f = 1.0f;
            g_set.output_fraction = f;
            changed = 1;
        }
        if (mg_json_get_num(json, "$.spectrum", &d))
            g_set.spectrum = d != 0;
        int sweep_changed = 0;
        if (mg_json_get_num(json, "$.lo_start", &d)) {
            g_set.lo_start = fmax(HW_LO_MIN_MHZ, fmin(HW_LO_MAX_MHZ, d));
            sweep_changed = 1;
        }
        if (mg_json_get_num(json, "$.lo_end", &d)) {
            g_set.lo_end = fmax(HW_LO_MIN_MHZ, fmin(HW_LO_MAX_MHZ, d));
            sweep_changed = 1;
        }
        int off, blen;
        if ((off = mg_json_get(json, "$.bands", &blen)) >= 0 &&
            json.buf[off] == '[') {
            int n = 0;
            char path[48];
            for (; n < MAX_BANDS; ++n) {
                double lo, hi;
                snprintf(path, sizeof(path), "$.bands[%d][0]", n);
                if (!mg_json_get_num(json, path, &lo)) break;
                snprintf(path, sizeof(path), "$.bands[%d][1]", n);
                if (!mg_json_get_num(json, path, &hi)) break;
                g_set.bands[n][0] = (float)fmin(lo, hi);
                g_set.bands[n][1] = (float)fmax(lo, hi);
            }
            g_set.nbands = n;
            sweep_changed = 1;
        }
        if (sweep_changed) {
            if (g_set.lo_end < g_set.lo_start) {
                double t = g_set.lo_start;
                g_set.lo_start = g_set.lo_end;
                g_set.lo_end = t;
            }
            g_set.epoch++;
            changed = 1;
        }
        pthread_mutex_unlock(&g_set.mtx);
        if (apply_gain >= 0) {
            csi_dev_set_gain(&g_dev, apply_gain);
            csi_dev_probe_analog_gain(&g_dev);
        }
    }
    free(type);
    return changed;
}

static void state_json(char *buf, size_t cap, void *user)
{
    (void)user;
    pthread_mutex_lock(&g_set.mtx);
    int n = snprintf(buf, cap,
        "{\"type\":\"state\",\"lo_start\":%.1f,\"lo_end\":%.1f,"
        "\"hw_min\":%.1f,\"hw_max\":%.1f,\"lo_step\":%.1f,"
        "\"gain\":%d,\"output_fraction\":%.3f,"
        "\"fps\":%.2f,\"points\":%u,\"adc_peak\":%d,\"adc_rms\":%.2f,"
        "\"lna_db\":%d,\"vga_db\":%d,\"bands\":[",
        g_set.lo_start, g_set.lo_end,
        HW_LO_MIN_MHZ, HW_LO_MAX_MHZ, LO_STEP_MHZ,
        g_set.gain, g_set.output_fraction,
        (double)g_fps, g_last_points, g_adc_peak, (double)g_adc_rms,
        g_dev.analog_lna_db, g_dev.analog_vga_db);
    for (int i = 0; i < g_set.nbands && n < (int)cap - 32; ++i)
        n += snprintf(buf + n, cap - n, "%s[%.1f,%.1f]",
                      i ? "," : "", g_set.bands[i][0], g_set.bands[i][1]);
    snprintf(buf + n, cap - n, "]}");
    pthread_mutex_unlock(&g_set.mtx);
}

typedef struct {
    double lo[MAX_LO_STEPS];
    int    n;
    double lo_start, lo_end;
} sweep_plan_t;

static int freq_in_bands(const settings_t *s, double f)
{
    if (s->nbands == 0)
        return f >= s->lo_start - 1e-6 && f <= s->lo_end + 1e-6;
    for (int i = 0; i < s->nbands; ++i)
        if (f >= s->bands[i][0] && f <= s->bands[i][1]) return 1;
    return 0;
}

/* One LO at the center of each 20 MHz slice. RANGE and WIFI share this
 * so 5490–5730 and UNII-2C 80s (106/122/138) hop the same grid. */
static void plan_add_span(sweep_plan_t *p, double a, double b)
{
    if (b < a) { double t = a; a = b; b = t; }
    for (double lo = a + 0.5 * LO_STEP_MHZ;
         lo + 0.5 * LO_STEP_MHZ <= b + 1e-6 && p->n < MAX_LO_STEPS;
         lo += LO_STEP_MHZ)
        p->lo[p->n++] = lo;
}

static void build_plan(sweep_plan_t *p, const settings_t *s)
{
    p->n = 0;

    if (s->nbands > 0) {
        for (int i = 0; i < s->nbands && p->n < MAX_LO_STEPS; ++i)
            plan_add_span(p, s->bands[i][0], s->bands[i][1]);
        if (p->n == 0)
            p->lo[p->n++] = 0.5 * (s->bands[0][0] + s->bands[0][1]);
    } else {
        plan_add_span(p, s->lo_start, s->lo_end);
        if (p->n == 0)
            p->lo[p->n++] = 0.5 * (s->lo_start + s->lo_end);
    }

    /* Point-frame header / shell scale use the keep-band, not the WIFI
     * catalog 5170–5895 the client sends with a channel subset. */
    if (p->n > 0) {
        p->lo_start = p->lo[0] - 0.5 * LO_STEP_MHZ;
        p->lo_end = p->lo[p->n - 1] + 0.5 * LO_STEP_MHZ;
    } else {
        p->lo_start = s->lo_start;
        p->lo_end = s->lo_end;
    }
}

static void *worker(void *arg)
{
    (void)arg;

    uint8_t *blk = malloc(BLOCK_BYTES);
    float   *vraw = malloc(FFT_SIZE * sizeof(float));
    dsp_peak_t *topk = malloc(TOPK_BASE * sizeof(dsp_peak_t));
    uint8_t *frame = malloc(sizeof(pg_hdr_t) + MAX_FRAME_POINTS * sizeof(pg_point_t));
    if (!blk || !vraw || !topk || !frame) {
        fprintf(stderr, "worker: alloc failed\n");
        g_quit = 1;
        return NULL;
    }

    fftwf_complex *fin = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    fftwf_complex *fout[CHANNELS];
    fftwf_plan plan[CHANNELS];
    for (int c = 0; c < CHANNELS; ++c) {
        fout[c] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        plan[c] = fftwf_plan_dft_1d(FFT_SIZE, fin, fout[c], FFTW_FORWARD, FFTW_ESTIMATE);
    }

    static double slot_sum[PG_SPECTRUM_BINS];
    static uint32_t slot_cnt[PG_SPECTRUM_BINS];
    float spec_out[PG_SPECTRUM_BINS];

    sweep_plan_t plan_lo = {0};
    uint32_t plan_epoch = 0;

    const int half = FFT_SIZE / 2;
    /* Keep detections inside the digital-filter passband (±LO_STEP/2 around DC). */
    int k_min = half - (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    int k_max = half + (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    if (k_min < 0) k_min = 0;
    if (k_max > FFT_SIZE - 1) k_max = FFT_SIZE - 1;

    float vmax = 1e-9f;
    uint32_t seq = 0;
    double pipelined_lo = HW_LO_MIN_MHZ;
    int lo_programmed = 0;

    while (!g_quit) {
        pthread_mutex_lock(&g_set.mtx);
        if (g_set.epoch != plan_epoch) {
            build_plan(&plan_lo, &g_set);
            plan_epoch = g_set.epoch;
            lo_programmed = 0;
            fprintf(stderr, "phasegaze: plan %d hop%s lo0=%.1f\n",
                    plan_lo.n, plan_lo.n == 1 ? "" : "s",
                    plan_lo.n ? plan_lo.lo[0] : 0.0);
        }
        float out_frac = g_set.output_fraction;
        settings_t set_snap = g_set;
        pthread_mutex_unlock(&g_set.mtx);

        if (!lo_programmed) {
            pipelined_lo = plan_lo.lo[0];
            csi_dev_set_lo(&g_dev, pipelined_lo);
            csi_dev_flush(&g_dev, g_dev.span_bytes);
            lo_programmed = 1;
        }

        int active_topk = (int)((float)TOPK_BASE * out_frac);
        if (active_topk < 1) active_topk = 1;

        uint64_t t0 = now_ns();
        pg_point_t *pts = (pg_point_t *)(frame + sizeof(pg_hdr_t));
        uint32_t npts = 0;
        float vmax_next = 1e-9f;
        int adc_peak = 0;
        double adc_sumsq = 0.0;
        uint32_t adc_n = 0;

        for (int idx = 0; idx < plan_lo.n && !g_quit; ++idx) {
            double lo_curr = pipelined_lo;
            double lo_next = (idx + 1 < plan_lo.n) ? plan_lo.lo[idx + 1] : plan_lo.lo[0];

            /* Read settled steady-state block (Block 1 of 128 KiB DMA span).
             * Consumes entire span, keeping ring empty and aligned for the next hop. */
            csi_dev_read_settled_block(&g_dev, blk, BLOCK_BYTES);
            if (lo_next != lo_curr)
                csi_dev_set_lo(&g_dev, lo_next);
            pipelined_lo = lo_next;

            {
                int8_t *s8 = (int8_t *)blk;
                int blk_peak = 0;
                for (int i = 0; i < BLOCK_BYTES; ++i) {
                    int sv = (int)s8[i];
                    int a = sv < 0 ? -sv : sv;
                    if (a > blk_peak) blk_peak = a;
                    adc_sumsq += (double)(sv * sv);
                }
                adc_n += (uint32_t)BLOCK_BYTES;
                if (blk_peak > adc_peak) adc_peak = blk_peak;
                if (blk_peak < ADC_PEAK_MIN)
                    continue;
            }

            for (int c = 0; c < CHANNELS; ++c) {
                dsp_cs8_extract_rotate((const int8_t *)blk, c,
                                       (float *)fin, FFT_SIZE, 1.0f, 0.0f);
                fftwf_execute(plan[c]);
            }

            float *chp[CHANNELS] = {
                (float *)fout[0], (float *)fout[1], (float *)fout[2], (float *)fout[3]
            };
            dsp_power4_log_shifted(chp, vraw, FFT_SIZE, DC_GUARD_BINS);

            if (set_snap.spectrum) {
                /* Fold only the digital keep-band (±LO_STEP/2) so analog
                 * skirts do not pile up at the edges of a narrow sweep. */
                for (int k = k_min; k <= k_max; ++k) {
                    double rf = lo_curr + FS_MHZ * ((double)k - (double)half)
                                / (double)FFT_SIZE;
                    int s = (int)((rf - HW_LO_MIN_MHZ) / PG_SPECTRUM_BIN_MHZ);
                    if ((unsigned)s < PG_SPECTRUM_BINS) {
                        slot_sum[s] += vraw[k];
                        slot_cnt[s]++;
                    }
                }
            }

            int hsz = dsp_cfar_topk(vraw, k_min, k_max, CFAR_WIN, CFAR_GUARD,
                                    CFAR_THRESH, topk, active_topk);

            for (int t = 0; t < hsz && npts < MAX_FRAME_POINTS; ++t) {
                int k = topk[t].k;
                int i = (k + half) % FFT_SIZE;
                double rf = lo_curr + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;

                if (!freq_in_bands(&set_snap, rf)) continue;

                float re0 = fout[0][i][0], im0 = fout[0][i][1];
                float re1 = fout[1][i][0], im1 = fout[1][i][1];
                float re2 = fout[2][i][0], im2 = fout[2][i][1];
                float re3 = fout[3][i][0], im3 = fout[3][i][1];

                float phi10 = atan2f(im1 * re0 - re1 * im0, re1 * re0 + im1 * im0);
                float phi23 = atan2f(im2 * re3 - re2 * im3, re2 * re3 + im2 * im3);
                float phi20 = atan2f(im2 * re0 - re2 * im0, re2 * re0 + im2 * im0);
                float phi30 = atan2f(im3 * re0 - re3 * im0, re3 * re0 + im3 * im0);
                float phi21 = atan2f(im2 * re1 - re2 * im1, re2 * re1 + im2 * im1);

                phi10 = atan2f(sinf(phi10) + sinf(phi23), cosf(phi10) + cosf(phi23));
                phi30 = atan2f(sinf(phi30) + sinf(phi21), cosf(phi30) + cosf(phi21));

                float gx, gy;
                float cost = dsp_solve_gradient(phi10, phi20, phi30, &gx, &gy);
                if (cost > DOA_COST_MAX) continue;

                float scale = SCALE_FACTOR_AT_MHZ((float)rf);
                float u = gx / scale, v = gy / scale;
                if (u * u + v * v > 1.0f) continue;

                float inten = vraw[k] / vmax;
                if (inten > 1.0f) inten = 1.0f;
                if (inten < 0.0f) inten = 0.0f;
                if (vraw[k] > vmax_next) vmax_next = vraw[k];

                pts[npts].u = u;
                pts[npts].v = v;
                pts[npts].freq_mhz = (float)rf;
                pts[npts].intensity = inten;
                npts++;
            }
        }
        if (vmax_next > 1e-9f) vmax = vmax_next;
        g_adc_peak = adc_peak;
        g_adc_rms = (adc_n > 0) ? sqrtf((float)(adc_sumsq / (double)adc_n)) : 0.0f;

        uint64_t t1 = now_ns();
        double dt_ms = (double)(t1 - t0) / 1e6;
        g_fps = (dt_ms > 1e-9) ? (float)(1000.0 / dt_ms) : 0.0f;
        g_last_points = npts;

        pg_hdr_t *hdr = (pg_hdr_t *)frame;
        hdr->magic = PG_MAGIC;
        hdr->version = PG_VERSION;
        hdr->type = PG_FRAME_POINTS;
        hdr->count = npts;
        hdr->lo_start = (float)plan_lo.lo_start;
        hdr->lo_end = (float)plan_lo.lo_end;
        hdr->fps = g_fps;
        hdr->seq = seq++;
        hdr->reserved = 0;
        server_publish(0, frame, sizeof(pg_hdr_t) + npts * sizeof(pg_point_t));

        if (set_snap.spectrum) {
            /* Raw log-power averages. Display AGC / video average live in the client. */
            for (int s = 0; s < PG_SPECTRUM_BINS; ++s) {
                spec_out[s] = slot_cnt[s] ? (float)(slot_sum[s] / slot_cnt[s]) : 0.0f;
                slot_sum[s] = 0.0;
                slot_cnt[s] = 0;
            }

            static uint8_t sframe[sizeof(pg_hdr_t) + sizeof(spec_out)];
            pg_hdr_t *sh = (pg_hdr_t *)sframe;
            sh->magic = PG_MAGIC;
            sh->version = PG_VERSION;
            sh->type = PG_FRAME_SPECTRUM;
            sh->count = PG_SPECTRUM_BINS;
            sh->lo_start = (float)HW_LO_MIN_MHZ;
            sh->lo_end = (float)HW_LO_MAX_MHZ;
            sh->fps = g_fps;
            sh->seq = seq++;
            sh->reserved = 0;
            memcpy(sframe + sizeof(pg_hdr_t), spec_out, sizeof(spec_out));
            server_publish(1, sframe, sizeof(sframe));
        }
    }

    for (int c = 0; c < CHANNELS; ++c) {
        fftwf_destroy_plan(plan[c]);
        fftwf_free(fout[c]);
    }
    fftwf_free(fin);
    free(frame);
    free(topk);
    free(vraw);
    free(blk);
    return NULL;
}

int main(int argc, char **argv)
{
    int port = 8001;
    const char *web_root = "/usr/share/quadrf/phasegaze";

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--web") && i + 1 < argc) web_root = argv[++i];
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            fprintf(stderr, "usage: %s [--port N] [--web DIR]\n", argv[0]);
            return 0;
        } else {
            fprintf(stderr, "usage: %s [--port N] [--web DIR]\n", argv[0]);
            return 1;
        }
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (csi_dev_open(&g_dev, DEVICE_PATH) != 0)
        return 1;
    csi_dev_set_gain(&g_dev, g_set.gain);
    csi_dev_probe_analog_gain(&g_dev);

    char url[64];
    snprintf(url, sizeof(url), "http://0.0.0.0:%d", port);
    server_cfg_t cfg = {
        .listen_url = url,
        .web_root = web_root,
        .on_control = on_control,
        .state_json = state_json,
        .user = NULL,
    };
    if (server_start(&cfg) != 0) {
        fprintf(stderr, "server_start failed\n");
        csi_dev_close(&g_dev);
        return 1;
    }

    pthread_t th;
    if (pthread_create(&th, NULL, worker, NULL) != 0) {
        fprintf(stderr, "pthread_create failed\n");
        return 1;
    }

    while (!g_quit) {
        usleep(500000);
        server_broadcast_state();
    }

    pthread_join(th, NULL);
    server_stop();
    csi_dev_close(&g_dev);
    return 0;
}
