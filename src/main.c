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

#define _GNU_SOURCE
#include <sched.h>
#include <stddef.h>
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
#include "tuner.h"
#include "dsp.h"
#include "server.h"
#include "pg_stream.h"
#include "external/mongoose.h"

// ---------------------------------------------------------------------------
// 4-lane CSI, nominally "38 Msps". Measured 37.3726 Msps: the 16384-sample
// span period is 438.39 us, and a 10 MHz LO step moves an OFDM signal by
// 2192.15 bins of 8192 (38 Msps would be 2155.8). FFT 8192 gives ~4.6 kHz
// bins. LO step is the digital-filter BW.
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
#define FS_MHZ           37.3726
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

static void push_plan_locked(void);

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
            push_plan_locked();
            changed = 1;
        }
        pthread_mutex_unlock(&g_set.mtx);
        /* JTAG from this thread would land between retunes at random. */
        if (apply_gain >= 0)
            tuner_set_gain(apply_gain);
    }
    free(type);
    return changed;
}

/* Span accounting on the DSP side, updated under g_claim_mtx. */
static struct {
    uint64_t hops, invalid, dup, untagged, dropped, frames;
} g_wstats;

static void state_json(char *buf, size_t cap, void *user)
{
    (void)user;
    tuner_stats_t ts;
    tuner_get_stats(&ts);
    pthread_mutex_lock(&g_set.mtx);
    int n = snprintf(buf, cap,
        "{\"type\":\"state\",\"lo_start\":%.1f,\"lo_end\":%.1f,"
        "\"hw_min\":%.1f,\"hw_max\":%.1f,\"lo_step\":%.1f,"
        "\"gain\":%d,\"output_fraction\":%.3f,"
        "\"fps\":%.2f,\"points\":%u,\"adc_peak\":%d,\"adc_rms\":%.2f,"
        "\"lna_db\":%d,\"vga_db\":%d,"
        "\"tuner\":{\"rt\":%d,\"spans\":%llu,\"retunes\":%llu,\"deferred\":%llu,"
        "\"late_land\":%llu,\"resyncs\":%llu,\"span_us\":%.4f,"
        "\"min_late_us\":%.1f,\"write_us\":%.1f},"
        "\"dsp\":{\"hops\":%llu,\"invalid\":%llu,\"dup\":%llu,\"untagged\":%llu,"
        "\"dropped\":%llu,\"frames\":%llu},\"bands\":[",
        g_set.lo_start, g_set.lo_end,
        HW_LO_MIN_MHZ, HW_LO_MAX_MHZ, LO_STEP_MHZ,
        g_set.gain, g_set.output_fraction,
        (double)g_fps, g_last_points, g_adc_peak, (double)g_adc_rms,
        g_dev.analog_lna_db, g_dev.analog_vga_db,
        ts.rt, (unsigned long long)ts.spans, (unsigned long long)ts.retunes,
        (unsigned long long)ts.deferred, (unsigned long long)ts.late_land,
        (unsigned long long)ts.resyncs, ts.span_us, ts.min_late_us, ts.write_us,
        (unsigned long long)g_wstats.hops, (unsigned long long)g_wstats.invalid,
        (unsigned long long)g_wstats.dup, (unsigned long long)g_wstats.untagged,
        (unsigned long long)g_wstats.dropped, (unsigned long long)g_wstats.frames);
    for (int i = 0; i < g_set.nbands && n < (int)cap - 32; ++i)
        n += snprintf(buf + n, cap - n, "%s[%.1f,%.1f]",
                      i ? "," : "", g_set.bands[i][0], g_set.bands[i][1]);
    snprintf(buf + n, cap - n, "]}");
    pthread_mutex_unlock(&g_set.mtx);
}

typedef tuner_plan_t sweep_plan_t;

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
        p->lo_start = (float)(p->lo[0] - 0.5 * LO_STEP_MHZ);
        p->lo_end = (float)(p->lo[p->n - 1] + 0.5 * LO_STEP_MHZ);
    } else {
        p->lo_start = (float)s->lo_start;
        p->lo_end = (float)s->lo_end;
    }
}

static void push_plan_locked(void)
{
    sweep_plan_t p;
    build_plan(&p, &g_set);
    fprintf(stderr, "phasegaze: plan %d hop%s lo0=%.1f\n",
            p.n, p.n == 1 ? "" : "s", p.n ? p.lo[0] : 0.0);
    tuner_set_plan(&p);
}

// ---------------------------------------------------------------------------
// DSP workers. Spans are claimed in ring order; the tuner's tag says which LO
// the kept half saw, so processing time only decides whether a hop is
// dropped, never how it is labeled. Frames are assembled per sweep.
// ---------------------------------------------------------------------------

#define N_WORKERS          2
#define TUNER_CPU          1
/* Publication arrives in bursts of several spans when the csi-copy kworker
 * is delayed. The workers have about 1 span/span of spare capacity, so a
 * 7 ms backlog drains within a few ms; beyond that they are really behind.
 * Must stay well under TAG_SLOTS (64) or the tags are overwritten first. */
#define MAX_BACKLOG_SPANS  16
#define N_ACC              4
#define FFTW_WISDOM_FILE   "phasegaze-fftw.wisdom"

typedef struct {
    int      used, closed;
    uint32_t sweep;
    int      claimed, done;
    uint32_t npts;
    float    vmax_next;
    int      adc_peak;
    double   adc_sumsq;
    uint64_t adc_n;
    float    lo_start, lo_end;
    int      spectrum;
    double   slot_sum[PG_SPECTRUM_BINS];
    uint32_t slot_cnt[PG_SPECTRUM_BINS];
    uint8_t *frame;
} frame_acc_t;

typedef struct {
    uint8_t       *blk;
    float         *vraw;
    dsp_peak_t    *topk;
    pg_point_t    *pts;
    fftwf_complex *fin;
    fftwf_complex *fout[CHANNELS];
    fftwf_plan     plan[CHANNELS];
    pthread_t      th;
} wctx_t;

static frame_acc_t g_acc[N_ACC];
static pthread_mutex_t g_claim_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_frame_mtx = PTHREAD_MUTEX_INITIALIZER;
static volatile float g_vmax = 1e-9f;
static uint32_t g_seq;
static uint64_t g_last_pub_ns;
static float    g_spec_out[PG_SPECTRUM_BINS];
static uint8_t  g_sframe[sizeof(pg_hdr_t) + sizeof(g_spec_out)];

/* g_frame_mtx held. */
static void frame_publish(frame_acc_t *a)
{
    pg_hdr_t *hdr = (pg_hdr_t *)a->frame;
    uint64_t t = now_ns();
    if (g_last_pub_ns) {
        float inst = 1e9f / (float)(t - g_last_pub_ns);
        g_fps = (g_fps > 0.0f) ? g_fps + 0.1f * (inst - g_fps) : inst;
    }
    g_last_pub_ns = t;

    if (a->vmax_next > 1e-9f) g_vmax = a->vmax_next;
    g_adc_peak = a->adc_peak;
    g_adc_rms = a->adc_n ? sqrtf((float)(a->adc_sumsq / (double)a->adc_n)) : 0.0f;
    g_last_points = a->npts;

    hdr->magic = PG_MAGIC;
    hdr->version = PG_VERSION;
    hdr->type = PG_FRAME_POINTS;
    hdr->count = a->npts;
    hdr->lo_start = a->lo_start;
    hdr->lo_end = a->lo_end;
    hdr->fps = g_fps;
    hdr->seq = g_seq++;
    hdr->reserved = 0;
    server_publish(0, a->frame, sizeof(pg_hdr_t) + a->npts * sizeof(pg_point_t));

    if (a->spectrum) {
        /* Raw log-power averages. Display AGC / video average live in the client. */
        for (int s = 0; s < PG_SPECTRUM_BINS; ++s)
            g_spec_out[s] = a->slot_cnt[s] ? (float)(a->slot_sum[s] / a->slot_cnt[s]) : 0.0f;
        pg_hdr_t *sh = (pg_hdr_t *)g_sframe;
        sh->magic = PG_MAGIC;
        sh->version = PG_VERSION;
        sh->type = PG_FRAME_SPECTRUM;
        sh->count = PG_SPECTRUM_BINS;
        sh->lo_start = (float)HW_LO_MIN_MHZ;
        sh->lo_end = (float)HW_LO_MAX_MHZ;
        sh->fps = g_fps;
        sh->seq = g_seq++;
        sh->reserved = 0;
        memcpy(g_sframe + sizeof(pg_hdr_t), g_spec_out, sizeof(g_spec_out));
        server_publish(1, g_sframe, sizeof(g_sframe));
    }
    g_wstats.frames++;
    a->used = 0;
}

/* Called in claim order (g_claim_mtx held). A new sweep closes the older
 * ones; each is published once its last in-flight hop finishes. */
static frame_acc_t *frame_register(const span_tag_t *t, int spectrum)
{
    pthread_mutex_lock(&g_frame_mtx);
    frame_acc_t *a = NULL;
    for (int i = 0; i < N_ACC; ++i)
        if (g_acc[i].used && g_acc[i].sweep == t->sweep)
            a = &g_acc[i];
    if (!a) {
        for (int i = 0; i < N_ACC; ++i) {
            frame_acc_t *o = &g_acc[i];
            if (o->used && !o->closed) {
                o->closed = 1;
                if (o->done == o->claimed)
                    frame_publish(o);
            }
        }
        for (int i = 0; i < N_ACC && !a; ++i)
            if (!g_acc[i].used)
                a = &g_acc[i];
        if (a) {
            uint8_t *fb = a->frame;
            memset(a, 0, offsetof(frame_acc_t, slot_sum));
            if (spectrum) {
                memset(a->slot_sum, 0, sizeof(a->slot_sum));
                memset(a->slot_cnt, 0, sizeof(a->slot_cnt));
            }
            a->frame = fb;
            a->used = 1;
            a->sweep = t->sweep;
            a->vmax_next = 1e-9f;
            a->lo_start = t->lo_start;
            a->lo_end = t->lo_end;
            a->spectrum = spectrum;
        }
    }
    if (a)
        a->claimed++;
    pthread_mutex_unlock(&g_frame_mtx);
    return a;
}

static inline uint32_t ring_used32(uint32_t head, uint32_t tail, uint64_t size)
{
    return (head >= tail) ? (head - tail) : ((uint32_t)size - (tail - head));
}

/* Copy the settled second half of the next usable span. */
static int claim_span(uint8_t *dst, span_tag_t *tag, frame_acc_t **acc, int spectrum)
{
    const uint32_t span = g_dev.span_bytes;
    const uint64_t rsz = g_dev.ring_size;
    const uint32_t off = span - BLOCK_BYTES;

    while (!g_quit) {
        pthread_mutex_lock(&g_claim_mtx);
        uint32_t head, tail;
        if (csi_dev_ring_pos(&g_dev, &head, &tail) == 0) {
            uint32_t used = ring_used32(head, tail, rsz);
            /* The head advances in whole DMA spans. A remainder means the
             * tail is off that grid (another reader consumed partial
             * spans); drop the fragment so reads and tags line up. */
            uint32_t frag = used % span;
            if (frag && csi_dev_consume(&g_dev, frag) == 0) {
                tail = (uint32_t)(((uint64_t)tail + frag) % rsz);
                used -= frag;
            }
            if (used > MAX_BACKLOG_SPANS * span &&
                csi_dev_consume(&g_dev, used - span) == 0) {
                g_wstats.dropped += (used - span) / span;
                tail = (uint32_t)(((uint64_t)tail + used - span) % rsz);
                used = span;
            }
            while (used >= span) {
                uint32_t end = (uint32_t)(((uint64_t)tail + span) % rsz);
                frame_acc_t *a = NULL;
                int have = tuner_lookup(end, tag);
                if (!have)
                    g_wstats.untagged++;
                else if (!tag->valid)
                    g_wstats.invalid++;
                else if (tag->dup)
                    g_wstats.dup++;
                else if ((a = frame_register(tag, spectrum)) != NULL) {
                    uint32_t start = (uint32_t)(((uint64_t)tail + off) % rsz);
                    uint32_t n1 = BLOCK_BYTES;
                    if ((uint64_t)start + n1 > rsz)
                        n1 = (uint32_t)(rsz - start);
                    memcpy(dst, (const uint8_t *)g_dev.ring + start, n1);
                    if (n1 < BLOCK_BYTES)
                        memcpy(dst + n1, g_dev.ring, BLOCK_BYTES - n1);
                }
                csi_dev_consume(&g_dev, span);
                if (a) {
                    g_wstats.hops++;
                    pthread_mutex_unlock(&g_claim_mtx);
                    *acc = a;
                    return 1;
                }
                tail = end;
                used -= span;
            }
        }
        pthread_mutex_unlock(&g_claim_mtx);
        csi_dev_wait(&g_dev, 20);
    }
    return 0;
}

static void frame_contribute(frame_acc_t *a, const pg_point_t *pts, uint32_t n,
                             float vmax_hop, int adc_peak, double adc_sumsq,
                             uint32_t adc_n, const float *vraw, double lo,
                             int k_min, int k_max)
{
    const int half = FFT_SIZE / 2;
    pthread_mutex_lock(&g_frame_mtx);
    pg_point_t *dst = (pg_point_t *)(a->frame + sizeof(pg_hdr_t));
    if (n > MAX_FRAME_POINTS - a->npts)
        n = MAX_FRAME_POINTS - a->npts;
    memcpy(dst + a->npts, pts, n * sizeof(pg_point_t));
    a->npts += n;
    if (vmax_hop > a->vmax_next) a->vmax_next = vmax_hop;
    if (adc_peak > a->adc_peak) a->adc_peak = adc_peak;
    a->adc_sumsq += adc_sumsq;
    a->adc_n += adc_n;
    if (a->spectrum && vraw) {
        /* Fold only the digital keep-band (±LO_STEP/2) so analog
         * skirts do not pile up at the edges of a narrow sweep. */
        for (int k = k_min; k <= k_max; ++k) {
            double rf = lo + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;
            int s = (int)((rf - HW_LO_MIN_MHZ) / PG_SPECTRUM_BIN_MHZ);
            if ((unsigned)s < PG_SPECTRUM_BINS) {
                a->slot_sum[s] += vraw[k];
                a->slot_cnt[s]++;
            }
        }
    }
    a->done++;
    if (a->closed && a->done == a->claimed)
        frame_publish(a);
    pthread_mutex_unlock(&g_frame_mtx);
}

static void *worker(void *arg)
{
    wctx_t *w = (wctx_t *)arg;

    /* Keep off the tuner's core so its wakeups are not delayed by DSP. */
    cpu_set_t cs;
    CPU_ZERO(&cs);
    for (int c = 0; c < CPU_SETSIZE && c < (int)sysconf(_SC_NPROCESSORS_ONLN); ++c)
        if (c != TUNER_CPU) CPU_SET(c, &cs);
    pthread_setaffinity_np(pthread_self(), sizeof(cs), &cs);

    const int half = FFT_SIZE / 2;
    /* Keep detections inside the digital-filter passband (±LO_STEP/2 around DC). */
    int k_min = half - (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    int k_max = half + (int)((LO_STEP_MHZ / 2.0) * ((double)FFT_SIZE / FS_MHZ));
    if (k_min < 0) k_min = 0;
    if (k_max > FFT_SIZE - 1) k_max = FFT_SIZE - 1;

    settings_t set_snap;
    while (!g_quit) {
        pthread_mutex_lock(&g_set.mtx);
        set_snap = g_set;
        pthread_mutex_unlock(&g_set.mtx);

        span_tag_t tag;
        frame_acc_t *acc;
        if (!claim_span(w->blk, &tag, &acc, set_snap.spectrum))
            break;

        int active_topk = (int)((float)TOPK_BASE * set_snap.output_fraction);
        if (active_topk < 1) active_topk = 1;
        const double lo = tag.lo;
        const float vmax = g_vmax;

        const int8_t *s8 = (const int8_t *)w->blk;
        int blk_peak = 0;
        /* 65536 * 128^2 < 2^31, so int32 is exact and the loop vectorizes
         * (41 -> 11 us on the A76 versus a double accumulator). */
        int32_t sumsq_i = 0;
        for (int i = 0; i < BLOCK_BYTES; ++i) {
            int sv = (int)s8[i];
            int a = sv < 0 ? -sv : sv;
            blk_peak = a > blk_peak ? a : blk_peak;
            sumsq_i += sv * sv;
        }
        double sumsq = (double)sumsq_i;
        if (blk_peak < ADC_PEAK_MIN) {
            frame_contribute(acc, NULL, 0, 0.0f, blk_peak, sumsq, BLOCK_BYTES,
                             NULL, lo, k_min, k_max);
            continue;
        }

        for (int c = 0; c < CHANNELS; ++c) {
            dsp_cs8_extract_rotate(s8, c, (float *)w->fin, FFT_SIZE, 1.0f, 0.0f);
            fftwf_execute(w->plan[c]);
        }
        float *chp[CHANNELS] = {
            (float *)w->fout[0], (float *)w->fout[1],
            (float *)w->fout[2], (float *)w->fout[3]
        };
        dsp_power4_log_shifted(chp, w->vraw, FFT_SIZE, DC_GUARD_BINS);

        int hsz = dsp_cfar_topk(w->vraw, k_min, k_max, CFAR_WIN, CFAR_GUARD,
                                CFAR_THRESH, w->topk, active_topk);

        uint32_t npts = 0;
        float vmax_hop = 1e-9f;
        for (int t = 0; t < hsz; ++t) {
            int k = w->topk[t].k;
            int i = (k + half) % FFT_SIZE;
            double rf = lo + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;

            if (!freq_in_bands(&set_snap, rf)) continue;

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
            float cost = dsp_solve_gradient(phi10, phi20, phi30, &gx, &gy);
            if (cost > DOA_COST_MAX) continue;

            float scale = SCALE_FACTOR_AT_MHZ((float)rf);
            float u = gx / scale, v = gy / scale;
            if (u * u + v * v > 1.0f) continue;

            float vk = w->vraw[k];
            float inten = vk / vmax;
            if (inten > 1.0f) inten = 1.0f;
            if (inten < 0.0f) inten = 0.0f;
            if (vk > vmax_hop) vmax_hop = vk;

            w->pts[npts].u = u;
            w->pts[npts].v = v;
            w->pts[npts].freq_mhz = (float)rf;
            w->pts[npts].intensity = inten;
            npts++;
        }

        frame_contribute(acc, w->pts, npts, vmax_hop, blk_peak, sumsq, BLOCK_BYTES,
                         w->vraw, lo, k_min, k_max);
    }
    return NULL;
}

static int wctx_init(wctx_t *w)
{
    w->blk = malloc(BLOCK_BYTES);
    w->vraw = malloc(FFT_SIZE * sizeof(float));
    w->topk = malloc(TOPK_BASE * sizeof(dsp_peak_t));
    w->pts = malloc(TOPK_BASE * sizeof(pg_point_t));
    w->fin = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    if (!w->blk || !w->vraw || !w->topk || !w->pts || !w->fin)
        return -1;
    /* FFTW planning is not thread-safe; plan here, execute in the workers.
     * MEASURE picks a 4x-FFT set that runs in 150 us instead of 228 us
     * with ESTIMATE. Only the first plan measures; the rest hit wisdom. */
    for (int c = 0; c < CHANNELS; ++c) {
        w->fout[c] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        if (!w->fout[c])
            return -1;
        w->plan[c] = fftwf_plan_dft_1d(FFT_SIZE, w->fin, w->fout[c],
                                       FFTW_FORWARD, FFTW_MEASURE);
        if (!w->plan[c])
            return -1;
    }
    return 0;
}

static void wctx_free(wctx_t *w)
{
    for (int c = 0; c < CHANNELS; ++c) {
        if (w->plan[c]) fftwf_destroy_plan(w->plan[c]);
        if (w->fout[c]) fftwf_free(w->fout[c]);
    }
    if (w->fin) fftwf_free(w->fin);
    free(w->pts);
    free(w->topk);
    free(w->vraw);
    free(w->blk);
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

    for (int i = 0; i < N_ACC; ++i) {
        g_acc[i].frame = malloc(sizeof(pg_hdr_t) + MAX_FRAME_POINTS * sizeof(pg_point_t));
        if (!g_acc[i].frame) {
            fprintf(stderr, "frame alloc failed\n");
            return 1;
        }
    }
    static wctx_t wk[N_WORKERS];
    /* Relative to WorkingDirectory; a cold MEASURE costs about a second. */
    int have_wisdom = fftwf_import_wisdom_from_filename(FFTW_WISDOM_FILE);
    for (int i = 0; i < N_WORKERS; ++i) {
        if (wctx_init(&wk[i]) != 0) {
            fprintf(stderr, "worker alloc failed\n");
            return 1;
        }
    }
    if (!have_wisdom)
        fftwf_export_wisdom_to_filename(FFTW_WISDOM_FILE);

    sweep_plan_t plan;
    pthread_mutex_lock(&g_set.mtx);
    build_plan(&plan, &g_set);
    pthread_mutex_unlock(&g_set.mtx);
    if (tuner_start(&g_dev, TUNER_CPU, &plan) != 0) {
        fprintf(stderr, "tuner start failed\n");
        return 1;
    }

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
        g_quit = 1;
        tuner_stop();
        csi_dev_close(&g_dev);
        return 1;
    }

    for (int i = 0; i < N_WORKERS; ++i) {
        if (pthread_create(&wk[i].th, NULL, worker, &wk[i]) != 0) {
            fprintf(stderr, "pthread_create failed\n");
            return 1;
        }
    }

    while (!g_quit) {
        usleep(500000);
        server_broadcast_state();
    }

    for (int i = 0; i < N_WORKERS; ++i)
        pthread_join(wk[i].th, NULL);
    tuner_stop();
    server_stop();
    for (int i = 0; i < N_WORKERS; ++i)
        wctx_free(&wk[i]);
    csi_dev_close(&g_dev);
    return 0;
}
