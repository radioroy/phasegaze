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
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <stdatomic.h>

#include <fftw3.h>

#include "csi_dev.h"
#include "tuner.h"
#include "hop.h"
#include "server.h"
#include "pg_stream.h"
#include "video.h"
#include "wifi.h"
#include "external/mongoose.h"

#define DEVICE_PATH      "/dev/csi_stream0"

/* Measured on quadrf.local, MAX2851 die rev 1. The lock pin is high from
 * 4440 to 6755 MHz and drops at 4430 and 6760 (VAS zone 0 / 7, VCO
 * sub-bands 0 and 63). Hop centers sit 10 MHz in from these edges:
 * 4490 is still in the acquisition zone, 6730 is on the top sub-band
 * with the pin high and the 40 MHz comb on the programmed offset.
 * Gain 0..63 does not pull the zone out. A per-board measure can
 * tighten this when the lock pin falls inside the span. The hop center
 * stays 10 MHz in from the pin-high edge, on this same 20 MHz grid. */
#define HW_LO_MIN_MHZ    4480.0
#define HW_LO_MAX_MHZ    6740.0
/* Beside the VCO seeds. The service runs as dietpi and /var/lib/quadrf
 * is root-owned, so creating a file there returns EACCES. */
#define LOCK_PATH        "/var/lib/quadrf/demos/phasegaze-lock.txt"
#define TUNER_CPU        1
#define RF_GAIN_MAX      63

#define MAX_LO_STEPS     128
#define MAX_FRAME_POINTS 16384
/* Every other 20 MHz Wi-Fi channel is its own band: 34 with 6 GHz in. */
#define MAX_BANDS        48

/* Off: real emitters indoors close anywhere in +-pi (multipath, uncalibrated
 * per-RX phase), and 1.02 rad (the old DOA_COST_MAX 0.35) dropped 50-73% of
 * repeatable emitters on a healthy board. */
#define CLOSURE_MAX_DEFAULT  0.0f
#define BALANCE_DB_DEFAULT   10.0f
#define BALANCE_LIN_DEFAULT  10.0f
/* hop.c CFAR_THRESH is 1.6 (6.95 dB); the UI moves in 0.5 dB steps. */
#define CFAR_DB_DEFAULT      7.0f
#define DB_TO_LN             0.23025851f
#define SPUR_MARGIN_DEFAULT  3.0f


typedef struct {
    pthread_mutex_t mtx;
    uint32_t epoch;
    double   lo_start, lo_end;
    float    bands[MAX_BANDS][2];
    int      nbands;
    float    output_fraction;
    int      gain;
    int      spectrum;
    float    closure_max;   /* rad, 0 = off */
    float    balance_db;    /* dB, 0 = off */
    int      spur_mask;     /* learned fixed-offset spur mask on/off */
    int      bg_norm;       /* receiver background normalization on/off */
    float    cfar_db;       /* CFAR threshold over the local log mean */
    float    spur_margin;   /* spur mask trip, x busiest-LO visits */
    hop_gates_t gates;      /* derived from the two above */
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
    .closure_max = CLOSURE_MAX_DEFAULT,
    .balance_db = BALANCE_DB_DEFAULT,
    .spur_mask = 1,
    .bg_norm = 1,
    .cfar_db = CFAR_DB_DEFAULT,
    .spur_margin = SPUR_MARGIN_DEFAULT,
    .gates = { .closure_max = CLOSURE_MAX_DEFAULT,
               .balance_max = BALANCE_LIN_DEFAULT,
               .cfar_thresh = CFAR_DB_DEFAULT * DB_TO_LN,
               .spur_margin = SPUR_MARGIN_DEFAULT },
};

static struct {
    uint64_t hits, rej_balance, rej_closure, rej_spur;
} g_gstats;

static csi_dev_t g_dev;
static volatile sig_atomic_t g_quit = 0;
/* Video parks the sweep and closes the CSI node. g_video_mode is what the
 * page sees; g_video_on means this process has handed the radio over. */
static int g_video_mode;
static int g_video_on;
static double g_video_mhz;
static double g_video_tuned;
static int g_video_nudges;
static char g_video_err[80];
static int g_wifi_mode;
static int g_wifi_on;
static double g_wifi_mhz;
static int g_wifi_ch;
static double g_wifi_tuned;
static char g_wifi_err[80];
static atomic_int g_csi_open;
static int g_tuner_up;
static atomic_int g_park;
static int g_parked;
static pthread_mutex_t g_park_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_park_cv = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t g_cmd_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cmd_cv = PTHREAD_COND_INITIALIZER;
static int g_cmd;
static double g_cmd_mhz;
static int g_cmd_snap;
/* Sweep ceiling. A config edit or a lock measure sets it, and either
 * one stays inside 4480–6740. Spectrum bins stay on the full span. */
static double g_hw_lo = HW_LO_MIN_MHZ;
static double g_hw_hi = HW_LO_MAX_MHZ;
static int g_lock_lo, g_lock_hi;
static volatile int g_lock_busy;
static int g_lock_err;
static volatile float g_fps = 0.0f;
static volatile uint32_t g_last_points = 0;
static volatile int g_adc_peak = 0;
static volatile float g_adc_rms = 0.0f;

static void on_sig(int sig) { (void)sig; g_quit = 1; }

static void push_plan_locked(void);
static void lock_cal_start(void);
static void lock_save(void);
static int sweep_limit_apply(double lo, double hi);
static void clamp_sweep_to_hw(void);

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
        if (mg_json_get_num(json, "$.closure_max", &d)) {
            float c = (float)fmax(0.0, fmin(M_PI, d));
            g_set.closure_max = c;
            g_set.gates.closure_max = c;
            changed = 1;
        }
        if (mg_json_get_num(json, "$.balance_db", &d)) {
            float b = (float)fmax(0.0, fmin(40.0, d));
            g_set.balance_db = b;
            g_set.gates.balance_max = b > 0.0f ? powf(10.0f, b / 10.0f) : 0.0f;
            changed = 1;
        }
        if (mg_json_get_num(json, "$.spur_mask", &d)) {
            g_set.spur_mask = d != 0;
            changed = 1;
        }
        if (mg_json_get_num(json, "$.bg_norm", &d)) {
            g_set.bg_norm = d != 0;
            changed = 1;
        }
        if (mg_json_get_num(json, "$.cfar_db", &d)) {
            float c = (float)fmax(2.0, fmin(20.0, d));
            g_set.cfar_db = c;
            g_set.gates.cfar_thresh = c * DB_TO_LN;
            changed = 1;
        }
        if (mg_json_get_num(json, "$.spur_margin", &d)) {
            float m = (float)fmax(1.0, fmin(10.0, d));
            g_set.spur_margin = m;
            g_set.gates.spur_margin = m;
            changed = 1;
        }
        int sweep_changed = 0;
        if (mg_json_get_num(json, "$.lo_start", &d)) {
            g_set.lo_start = fmax(g_hw_lo, fmin(g_hw_hi, d));
            sweep_changed = 1;
        }
        if (mg_json_get_num(json, "$.lo_end", &d)) {
            g_set.lo_end = fmax(g_hw_lo, fmin(g_hw_hi, d));
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
            /* The measure thread owns the tuner until it restarts it. */
            if (!g_lock_busy)
                push_plan_locked();
            changed = 1;
        }
        {
            double req_lo = g_hw_lo, req_hi = g_hw_hi;
            int lim = 0;
            if (mg_json_get_num(json, "$.hw_min", &d)) { req_lo = d; lim = 1; }
            if (mg_json_get_num(json, "$.hw_max", &d)) { req_hi = d; lim = 1; }
            /* Ignored while the measure thread owns the synthesizer.
             * A rejected pair still broadcasts, so the boxes snap back. */
            if (lim && !g_lock_busy && sweep_limit_apply(req_lo, req_hi)) {
                clamp_sweep_to_hw();
                lock_save();
                push_plan_locked();
                fprintf(stderr, "phasegaze: sweep %.0f..%.0f\n",
                        g_hw_lo, g_hw_hi);
            }
            if (lim)
                changed = 1;
        }
        pthread_mutex_unlock(&g_set.mtx);
        /* JTAG from this thread would land between retunes at random. */
        if (apply_gain >= 0)
            tuner_set_gain(apply_gain);
    } else if (strcmp(type, "video") == 0) {
        if (mg_json_get_num(json, "$.freq_mhz", &d) && d >= 1000.0 && d <= 8000.0) {
            int snap = 1;
            double snap_d;
            if (mg_json_get_num(json, "$.snap", &snap_d))
                snap = snap_d != 0;
            pthread_mutex_lock(&g_set.mtx);
            if (!g_lock_busy) {
                g_video_mode = 1;
                g_video_mhz = d;
                g_video_err[0] = 0;
                changed = 1;
            }
            pthread_mutex_unlock(&g_set.mtx);
            if (changed) {
                pthread_mutex_lock(&g_cmd_mtx);
                g_cmd = 1;
                g_cmd_mhz = d;
                g_cmd_snap = snap;
                pthread_cond_signal(&g_cmd_cv);
                pthread_mutex_unlock(&g_cmd_mtx);
            }
        }
    } else if (strcmp(type, "video_stop") == 0) {
        pthread_mutex_lock(&g_cmd_mtx);
        g_cmd = 2;
        pthread_cond_signal(&g_cmd_cv);
        pthread_mutex_unlock(&g_cmd_mtx);
        changed = 1;
    } else if (strcmp(type, "wifi") == 0) {
        double d = 0;
        if ((mg_json_get_num(json, "$.mhz", &d) || mg_json_get_num(json, "$.freq_mhz", &d)) && d >= 4480 && d <= 6740) {
            int ch = 0;
            const char *band = NULL;
            double snap_mhz = wifi_snap_mhz(d, &ch, &band);
            pthread_mutex_lock(&g_set.mtx);
            if (!g_lock_busy) {
                g_wifi_mode = 1;
                g_wifi_mhz = snap_mhz;
                g_wifi_ch = ch;
                g_wifi_err[0] = 0;
                changed = 1;
            }
            pthread_mutex_unlock(&g_set.mtx);
            if (changed) {
                pthread_mutex_lock(&g_cmd_mtx);
                g_cmd = 3;
                g_cmd_mhz = snap_mhz;
                pthread_cond_signal(&g_cmd_cv);
                pthread_mutex_unlock(&g_cmd_mtx);
            }
        }
    } else if (strcmp(type, "wifi_stop") == 0) {
        pthread_mutex_lock(&g_cmd_mtx);
        g_cmd = 4;
        pthread_cond_signal(&g_cmd_cv);
        pthread_mutex_unlock(&g_cmd_mtx);
        changed = 1;
    } else if (strcmp(type, "lock_cal") == 0) {
        int start = 0;
        pthread_mutex_lock(&g_set.mtx);
        /* The measure walks the VCO on the open CSI fd. Video has closed it. */
        if (!g_lock_busy && !g_video_mode && atomic_load(&g_csi_open)) {
            g_lock_busy = 1;
            g_lock_err = 0;
            start = 1;
            changed = 1;
        }
        pthread_mutex_unlock(&g_set.mtx);
        if (start)
            lock_cal_start();
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
    char verr[80];
    size_t vj = 0;
    for (size_t i = 0; g_video_err[i] && vj + 1 < sizeof verr; i++) {
        unsigned char c = (unsigned char)g_video_err[i];
        if (c < 32 || c == '"' || c == '\\') continue;
        verr[vj++] = (char)c;
    }
    verr[vj] = 0;
    char werr[80];
    size_t wj = 0;
    for (size_t i = 0; g_wifi_err[i] && wj + 1 < sizeof werr; i++) {
        unsigned char c = (unsigned char)g_wifi_err[i];
        if (c < 32 || c == '"' || c == '\\') continue;
        werr[wj++] = (char)c;
    }
    werr[wj] = 0;
    const char *current_mode = g_video_mode ? "video" : (g_wifi_mode ? "wifi" : "sweep");
    int n = snprintf(buf, cap,
        "{\"type\":\"state\",\"lo_start\":%.1f,\"lo_end\":%.1f,"
        "\"hw_min\":%.1f,\"hw_max\":%.1f,\"lo_step\":%.1f,"
        "\"lock_lo\":%d,\"lock_hi\":%d,\"lock_busy\":%d,\"lock_err\":%d,"
        "\"mode\":\"%s\",\"video_mhz\":%.1f,\"video_err\":\"%s\",\"wifi_mhz\":%.1f,\"wifi_ch\":%d,\"wifi_err\":\"%s\","
        "\"gain\":%d,\"output_fraction\":%.3f,"
        "\"closure_max\":%.3f,\"balance_db\":%.1f,\"spur_mask\":%d,"
        "\"bg_norm\":%d,\"cfar_db\":%.2f,\"spur_margin\":%.2f,"
        "\"gates\":{\"hits\":%llu,\"rej_balance\":%llu,\"rej_closure\":%llu,"
        "\"rej_spur\":%llu},"
        "\"fps\":%.2f,\"points\":%u,\"adc_peak\":%d,\"adc_rms\":%.2f,"
        "\"lna_db\":%d,\"vga_db\":%d,"
        "\"tuner\":{\"rt\":%d,\"spans\":%llu,\"retunes\":%llu,\"deferred\":%llu,"
        "\"late_land\":%llu,\"resyncs\":%llu,\"span_us\":%.4f,"
        "\"min_late_us\":%.1f,\"write_us\":%.1f},"
        "\"dsp\":{\"hops\":%llu,\"invalid\":%llu,\"dup\":%llu,\"untagged\":%llu,"
        "\"dropped\":%llu,\"frames\":%llu},\"bands\":[",
        g_set.lo_start, g_set.lo_end,
        g_hw_lo, g_hw_hi, LO_STEP_MHZ,
        g_lock_lo, g_lock_hi, g_lock_busy, g_lock_err,
        current_mode, g_video_mhz, verr, g_wifi_mhz, g_wifi_ch, werr,
        g_set.gain, g_set.output_fraction,
        (double)g_set.closure_max, (double)g_set.balance_db, g_set.spur_mask,
        g_set.bg_norm, (double)g_set.cfar_db, (double)g_set.spur_margin,
        (unsigned long long)__atomic_load_n(&g_gstats.hits, __ATOMIC_RELAXED),
        (unsigned long long)__atomic_load_n(&g_gstats.rej_balance, __ATOMIC_RELAXED),
        (unsigned long long)__atomic_load_n(&g_gstats.rej_closure, __ATOMIC_RELAXED),
        (unsigned long long)__atomic_load_n(&g_gstats.rej_spur, __ATOMIC_RELAXED),
        (double)g_fps, g_last_points, g_adc_peak, (double)g_adc_rms,
        (g_video_mode || g_wifi_mode) ? -1 : g_dev.analog_lna_db,
        (g_video_mode || g_wifi_mode) ? -1 : g_dev.analog_vga_db,
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

static int keep_rf(double rf_mhz, const void *arg)
{
    return freq_in_bands((const settings_t *)arg, rf_mhz);
}

/* One LO at the center of each 20 MHz slice of [a, b]. WIFI passes its
 * band edges (5170, 5490, 5735, 5945) so those centers stay on the
 * channel grid. The range slider snaps to the hardware slices first. */
static void plan_add_span(sweep_plan_t *p, double a, double b)
{
    if (b < a) { double t = a; a = b; b = t; }
    /* Center sits 10 MHz inside the board edge, so a hop on the last
     * slice is still where the lock pin was high. */
    double lo_min = g_hw_lo + 0.5 * LO_STEP_MHZ;
    double lo_max = g_hw_hi - 0.5 * LO_STEP_MHZ;
    for (double lo = a + 0.5 * LO_STEP_MHZ;
         lo + 0.5 * LO_STEP_MHZ <= b + 1e-6 && p->n < MAX_LO_STEPS;
         lo += LO_STEP_MHZ) {
        if (lo < lo_min - 1e-6 || lo > lo_max + 1e-6)
            continue;
        p->lo[p->n++] = lo;
    }
}

/* Range thumbs are continuous MHz. Starting the hop list at lo_start+10
 * slides every center by 1 MHz per slider step: the ±10 MHz keep-bands
 * walk across the 1 MHz spectrum slots, and each new center is a VCO
 * learn on the tuner thread. Expand to the slices anchored at
 * HW_LO_MIN (4480, 4500, ...) so the interior hops stay put. */
static void snap_hw_slices(double *a, double *b)
{
    const double step = LO_STEP_MHZ;
    const double org = HW_LO_MIN_MHZ;
    double a0 = org + step * floor((*a - org) / step + 1e-6);
    double b1 = org + step * ceil((*b - org) / step - 1e-6);
    if (b1 <= a0)
        b1 = a0 + step;
    if (a0 < g_hw_lo)
        a0 = org + step * ceil((g_hw_lo - org) / step - 1e-9);
    if (b1 > g_hw_hi)
        b1 = org + step * floor((g_hw_hi - org) / step + 1e-9);
    if (a0 < HW_LO_MIN_MHZ)
        a0 = HW_LO_MIN_MHZ;
    if (b1 > HW_LO_MAX_MHZ)
        b1 = HW_LO_MAX_MHZ;
    if (b1 <= a0)
        b1 = g_hw_hi;
    *a = a0;
    *b = b1;
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
        double a = s->lo_start, b = s->lo_end;
        snap_hw_slices(&a, &b);
        plan_add_span(p, a, b);
        if (p->n == 0)
            p->lo[p->n++] = 0.5 * (a + b);
    }

    /* Point-frame header / shell scale use the keep-band, not the WIFI
     * catalog 5170–6725 the client sends with a channel subset. */
    if (p->n > 0) {
        p->lo_start = (float)(p->lo[0] - 0.5 * LO_STEP_MHZ);
        p->lo_end = (float)(p->lo[p->n - 1] + 0.5 * LO_STEP_MHZ);
    } else {
        p->lo_start = (float)s->lo_start;
        p->lo_end = (float)s->lo_end;
    }
}

static sweep_plan_t g_plan;
static int g_plan_set;

static int plan_same(const sweep_plan_t *a, const sweep_plan_t *b)
{
    if (a->n != b->n)
        return 0;
    for (int i = 0; i < a->n; i++)
        if (fabs(a->lo[i] - b->lo[i]) > 0.05)
            return 0;
    return 1;
}

static void push_plan_locked(void)
{
    sweep_plan_t p;
    build_plan(&p, &g_set);
    /* Thumb drags send a new range every 80 ms. Same hop list: keep the
     * tuner on its envelope. A retune resync blanks the spectrum. */
    if (g_plan_set && plan_same(&g_plan, &p))
        return;
    g_plan = p;
    g_plan_set = 1;
    g_set.epoch++;
    fprintf(stderr, "phasegaze: plan %d hop%s lo0=%.1f\n",
            p.n, p.n == 1 ? "" : "s", p.n ? p.lo[0] : 0.0);
    tuner_set_plan(&p);
}

/* Inward onto the 20 MHz grid, inside 4480–6740. A reversed pair is
 * swapped. Returns 0, leaving the ceiling alone, when it collapses
 * below one slice. */
static int sweep_limit_apply(double lo, double hi)
{
    const double step = LO_STEP_MHZ;
    const double org = HW_LO_MIN_MHZ;
    if (hi < lo) {
        double t = lo;
        lo = hi;
        hi = t;
    }
    if (lo < HW_LO_MIN_MHZ) lo = HW_LO_MIN_MHZ;
    if (hi > HW_LO_MAX_MHZ) hi = HW_LO_MAX_MHZ;
    lo = org + step * ceil((lo - org) / step - 1e-9);
    hi = org + step * floor((hi - org) / step + 1e-9);
    if (lo < HW_LO_MIN_MHZ) lo = HW_LO_MIN_MHZ;
    if (hi > HW_LO_MAX_MHZ) hi = HW_LO_MAX_MHZ;
    if (hi < lo + step)
        return 0;
    g_hw_lo = lo;
    g_hw_hi = hi;
    return 1;
}

static void clamp_sweep_to_hw(void)
{
    if (g_set.lo_start < g_hw_lo) g_set.lo_start = g_hw_lo;
    if (g_set.lo_end > g_hw_hi) g_set.lo_end = g_hw_hi;
    if (g_set.lo_end < g_set.lo_start) {
        g_set.lo_start = g_hw_lo;
        g_set.lo_end = g_hw_hi;
    }
}

/* Pin-high edge, then the 20 MHz slice whose center is still 10 MHz
 * inside that edge. Never wider than the 4480–6740 span. */
static void hw_from_lock(int lock_lo, int lock_hi)
{
    if (!(lock_hi > lock_lo && lock_lo > 0)) {
        g_hw_lo = HW_LO_MIN_MHZ;
        g_hw_hi = HW_LO_MAX_MHZ;
        return;
    }
    double need_lo = lock_lo < HW_LO_MIN_MHZ ? HW_LO_MIN_MHZ : (double)lock_lo;
    double need_hi = lock_hi > HW_LO_MAX_MHZ ? HW_LO_MAX_MHZ : (double)lock_hi;
    if (!sweep_limit_apply(need_lo, need_hi)) {
        g_hw_lo = HW_LO_MIN_MHZ;
        g_hw_hi = HW_LO_MAX_MHZ;
    }
}

static void lock_load(void)
{
    FILE *f = fopen(LOCK_PATH, "r");
    if (!f)
        return;
    /* Line two is the sweep ceiling, when a config edit has saved one.
     * An older file is just the pin pair, and the ceiling is derived. */
    int pin_lo = 0, pin_hi = 0, swo = 0, shi = 0;
    int n = fscanf(f, "%d %d %d %d", &pin_lo, &pin_hi, &swo, &shi);
    fclose(f);
    if (n >= 2 && pin_hi > pin_lo && pin_lo > 4000 && pin_hi < 8000) {
        g_lock_lo = pin_lo;
        g_lock_hi = pin_hi;
    }
    if (n == 4 && shi > swo) {
        if (!sweep_limit_apply((double)swo, (double)shi) && g_lock_hi > g_lock_lo)
            hw_from_lock(g_lock_lo, g_lock_hi);
    } else if (g_lock_hi > g_lock_lo) {
        hw_from_lock(g_lock_lo, g_lock_hi);
    }
    if (g_lock_hi > g_lock_lo || n == 4)
        fprintf(stderr, "phasegaze: lock pin %d..%d MHz, sweep %.0f..%.0f\n",
                g_lock_lo, g_lock_hi, g_hw_lo, g_hw_hi);
}

static void lock_save(void)
{
    FILE *f = fopen(LOCK_PATH, "w");
    if (!f) {
        fprintf(stderr, "phasegaze: lock file: %s\n", strerror(errno));
        return;
    }
    fprintf(f, "%d %d\n%.0f %.0f\n", g_lock_lo, g_lock_hi, g_hw_lo, g_hw_hi);
    fclose(f);
}

static void *lock_cal_main(void *arg)
{
    (void)arg;
    tuner_stop();
    int lo = 0, hi = 0;
    int rc = csi_dev_measure_lock(&g_dev, &lo, &hi);
    pthread_mutex_lock(&g_set.mtx);
    if (rc == 0) {
        g_lock_lo = lo;
        g_lock_hi = hi;
        g_lock_err = 0;
        hw_from_lock(lo, hi);
        clamp_sweep_to_hw();
        lock_save();
        fprintf(stderr, "phasegaze: lock pin %d..%d MHz, sweep %.0f..%.0f\n",
                lo, hi, g_hw_lo, g_hw_hi);
        push_plan_locked();
    } else {
        g_lock_err = 1;
        fprintf(stderr, "phasegaze: lock measure failed\n");
    }
    sweep_plan_t plan = g_plan;
    pthread_mutex_unlock(&g_set.mtx);
    if (!g_quit)
        tuner_start(&g_dev, TUNER_CPU, &plan);
    pthread_mutex_lock(&g_set.mtx);
    g_lock_busy = 0;
    pthread_mutex_unlock(&g_set.mtx);
    server_broadcast_state();
    return NULL;
}

static void lock_cal_start(void)
{
    pthread_t th;
    if (pthread_create(&th, NULL, lock_cal_main, NULL) != 0) {
        pthread_mutex_lock(&g_set.mtx);
        g_lock_busy = 0;
        g_lock_err = 1;
        pthread_mutex_unlock(&g_set.mtx);
        return;
    }
    pthread_detach(th);
}

// ---------------------------------------------------------------------------
// DSP workers. Spans are claimed in ring order; the tuner's tag says which LO
// the kept half saw, so processing time only decides whether a hop is
// dropped, never how it is labeled. Frames are assembled per sweep.
// ---------------------------------------------------------------------------

#define N_WORKERS          2
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
    int      claimed, done, nhops;
    uint32_t npts;
    float    vmax_next;
    int      adc_peak;
    double   adc_sumsq;
    uint64_t adc_n;
    float    lo_start, lo_end;
    int      spectrum;
    int      spec_raw;      /* hops folded without a warm background */
    double   slot_sum[PG_SPECTRUM_BINS];
    uint32_t slot_cnt[PG_SPECTRUM_BINS];
    uint8_t *frame;
} frame_acc_t;

typedef struct {
    uint8_t       *blk;
    pg_point_t    *pts;
    hop_ctx_t      hop;
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

    /* A sweep cut short by a resync or a stalled retune can miss the hop
     * holding the strongest emitter. Normalizing the next sweep by its vmax
     * lifts every noise point toward 1.0 for one frame (a full-disk flash). */
    if (a->vmax_next > 1e-9f && 4 * a->done >= 3 * a->nhops)
        g_vmax = a->vmax_next;
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
        /* Log-power averages, receiver background removed per FFT bin when
         * every hop had it warm (reserved bit 0). Display AGC / video
         * average live in the client. */
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
        /* Bits 1..: first LO of this frame's plan in 0.1 MHz, so the
         * client lays the 20 MHz hop grid on the plan the data came
         * from, not on a slider that is already somewhere else. */
        sh->reserved = (a->spec_raw ? 0u : 1u)
                     | ((uint32_t)lroundf((a->lo_start + 0.5f * (float)LO_STEP_MHZ) * 10.0f) << 1);
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
            a->nhops = t->nhops;
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

static void workers_park_wait(void)
{
    pthread_mutex_lock(&g_park_mtx);
    g_parked++;
    pthread_cond_broadcast(&g_park_cv);
    while (atomic_load(&g_park) && !g_quit)
        pthread_cond_wait(&g_park_cv, &g_park_mtx);
    g_parked--;
    pthread_mutex_unlock(&g_park_mtx);
}

/* Block until both DSP threads are out of the CSI ring. On timeout the
 * park is cancelled and the sweep keeps the device. */
static int workers_quiesce(void)
{
    atomic_store(&g_park, 1);
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    pthread_mutex_lock(&g_park_mtx);
    while (g_parked < N_WORKERS && !g_quit) {
        if (pthread_cond_timedwait(&g_park_cv, &g_park_mtx, &ts) == ETIMEDOUT)
            break;
    }
    int ok = g_parked >= N_WORKERS;
    pthread_mutex_unlock(&g_park_mtx);
    if (!ok) {
        atomic_store(&g_park, 0);
        pthread_mutex_lock(&g_park_mtx);
        pthread_cond_broadcast(&g_park_cv);
        pthread_mutex_unlock(&g_park_mtx);
    }
    return ok;
}

static void workers_release(void)
{
    atomic_store(&g_park, 0);
    pthread_mutex_lock(&g_park_mtx);
    pthread_cond_broadcast(&g_park_cv);
    pthread_mutex_unlock(&g_park_mtx);
}

/* Copy the settled second half of the next usable span. */
static int claim_span(uint8_t *dst, span_tag_t *tag, frame_acc_t **acc, int spectrum)
{
    const uint32_t span = g_dev.span_bytes;
    const uint64_t rsz = g_dev.ring_size;
    const uint32_t off = span - BLOCK_BYTES;

    while (!g_quit && !atomic_load(&g_park)) {
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
                             uint32_t adc_n, const float *vraw,
                             const float *vfold, float fold_off, int fold_norm,
                             double lo, int k_min, int k_max)
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
         * skirts do not pile up at the edges of a narrow sweep. DC and
         * 40 MHz comb bins are notched to 0 in vraw; counting them would
         * dent the slot they land in. */
        if (!fold_norm) a->spec_raw++;
        for (int k = k_min; k <= k_max; ++k) {
            if (vraw[k] <= 0.0f) continue;
            double rf = lo + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;
            int s = (int)((rf - HW_LO_MIN_MHZ) / PG_SPECTRUM_BIN_MHZ);
            if ((unsigned)s < PG_SPECTRUM_BINS) {
                a->slot_sum[s] += vfold[k] + fold_off;
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

    settings_t set_snap;
    while (!g_quit) {
        if (atomic_load(&g_park)) {
            workers_park_wait();
            continue;
        }
        pthread_mutex_lock(&g_set.mtx);
        set_snap = g_set;
        pthread_mutex_unlock(&g_set.mtx);

        span_tag_t tag;
        frame_acc_t *acc;
        if (!claim_span(w->blk, &tag, &acc, set_snap.spectrum)) {
            if (g_quit) break;
            if (atomic_load(&g_park)) {
                workers_park_wait();
                continue;
            }
            break;
        }

        int active_topk = (int)((float)TOPK_BASE * set_snap.output_fraction);
        if (active_topk < 1) active_topk = 1;
        const double lo = tag.lo;

        hop_gates_t gates = set_snap.gates;
        gates.plan_los = tag.nhops;
        gates.spur_mask = set_snap.spur_mask;
        gates.bg_norm = set_snap.bg_norm;
        gates.gain = set_snap.gain;
        hop_out_t o;
        hop_run(&w->hop, (const int8_t *)w->blk, lo, g_vmax, active_topk,
                keep_rf, &set_snap, &gates, w->pts, &o);
        if (o.ran) {
            __atomic_fetch_add(&g_gstats.hits, o.hits, __ATOMIC_RELAXED);
            __atomic_fetch_add(&g_gstats.rej_balance, o.rej_balance, __ATOMIC_RELAXED);
            __atomic_fetch_add(&g_gstats.rej_closure, o.rej_closure, __ATOMIC_RELAXED);
            __atomic_fetch_add(&g_gstats.rej_spur, o.rej_spur, __ATOMIC_RELAXED);
        }
        if (!o.ran)
            frame_contribute(acc, NULL, 0, 0.0f, o.adc_peak, o.adc_sumsq,
                             BLOCK_BYTES, NULL, NULL, 0.0f, 0, lo,
                             w->hop.k_min, w->hop.k_max);
        else
            frame_contribute(acc, w->pts, o.npts, o.vmax_hop, o.adc_peak,
                             o.adc_sumsq, BLOCK_BYTES, w->hop.vraw,
                             o.vfold, o.fold_off, o.fold_norm, lo,
                             w->hop.k_min, w->hop.k_max);
    }
    return NULL;
}

static int wctx_init(wctx_t *w)
{
    w->blk = malloc(BLOCK_BYTES);
    w->pts = malloc(TOPK_BASE * sizeof(pg_point_t));
    if (!w->blk || !w->pts)
        return -1;
    return hop_init(&w->hop);
}

static void wctx_free(wctx_t *w)
{
    hop_free(&w->hop);
    free(w->pts);
    free(w->blk);
}

static void video_set_status(int mode, double mhz, const char *err)
{
    pthread_mutex_lock(&g_set.mtx);
    g_video_mode = mode;
    if (mhz > 0.0)
        g_video_mhz = mhz;
    snprintf(g_video_err, sizeof g_video_err, "%s", err ? err : "");
    pthread_mutex_unlock(&g_set.mtx);
}

/* csi_dev_open puts interleave, the 20 MHz front-end, and all four
 * antennas back. The sweep's manual gain word is restored here; video
 * ran on the decoder's AGC and did not change g_set.gain. */
static void radio_to_sweep(void)
{
    int gain;
    sweep_plan_t plan;
    pthread_mutex_lock(&g_set.mtx);
    gain = g_set.gain;
    plan = g_plan;
    pthread_mutex_unlock(&g_set.mtx);

    if (!atomic_load(&g_csi_open)) {
        int ok = 0;
        for (int i = 0; i < 10 && !g_quit; i++) {
            if (csi_dev_open(&g_dev, DEVICE_PATH) == 0) {
                ok = 1;
                break;
            }
            usleep(150000);
        }
        if (!ok) {
            fprintf(stderr, "phasegaze: csi reopen failed\n");
            return;
        }
        atomic_store(&g_csi_open, 1);
        csi_dev_set_gain(&g_dev, gain);
        csi_dev_probe_analog_gain(&g_dev);
    }
    if (!g_tuner_up) {
        if (tuner_start(&g_dev, TUNER_CPU, &plan) != 0)
            fprintf(stderr, "phasegaze: tuner restart failed\n");
        else
            g_tuner_up = 1;
    }
    /* A range edit can land between the snapshot above and tuner_start. */
    if (g_tuner_up) {
        pthread_mutex_lock(&g_set.mtx);
        if (!plan_same(&plan, &g_plan))
            tuner_set_plan(&g_plan);
        pthread_mutex_unlock(&g_set.mtx);
    }
    workers_release();
}

static void video_restore_sweep(const char *err)
{
    g_video_on = 0;
    g_video_tuned = 0;
    video_pipeline_stop();
    if (!g_quit)
        radio_to_sweep();
    video_set_status(0, 0, err ? err : "");
}

/* Park the synthesizer on mhz with the sweep's VCO table, then the
 * video front-end. The demod must not call setFrequency after this. */
static int video_lock_lo(double mhz)
{
    if (!atomic_load(&g_csi_open)) {
        int ok = 0;
        for (int i = 0; i < 10 && !g_quit; i++) {
            if (csi_dev_open(&g_dev, DEVICE_PATH) == 0) {
                ok = 1;
                break;
            }
            usleep(150000);
        }
        if (!ok)
            return -1;
        atomic_store(&g_csi_open, 1);
    }
    int rc = csi_dev_set_lo(&g_dev, mhz);
    if (rc == 0)
        rc = csi_dev_video_front_end(&g_dev);
    csi_dev_close(&g_dev);
    atomic_store(&g_csi_open, 0);
    return rc;
}

static void video_enter(double mhz)
{
    if (g_quit) return;
    pthread_mutex_lock(&g_set.mtx);
    int busy = g_lock_busy;
    pthread_mutex_unlock(&g_set.mtx);
    if (busy) {
        video_set_status(0, mhz, "lock measure");
        return;
    }
    if (g_video_on && !video_pipeline_dead() && fabs(g_video_tuned - mhz) < 0.05)
        return;

    if (!g_video_on) {
        if (!workers_quiesce()) {
            video_set_status(0, mhz, "sweep busy");
            return;
        }
        if (g_quit) {
            workers_release();
            return;
        }
        g_video_on = 1;
        tuner_stop();
        g_tuner_up = 0;
        pthread_mutex_lock(&g_frame_mtx);
        for (int i = 0; i < N_ACC; i++)
            g_acc[i].used = 0;
        pthread_mutex_unlock(&g_frame_mtx);
    } else {
        video_pipeline_stop();
    }

    if (g_quit) return;
    if (video_lock_lo(mhz) != 0) {
        video_restore_sweep("front end");
        return;
    }
    if (video_pipeline_start(mhz) != 0) {
        char err[80];
        video_pipeline_error(err, sizeof err);
        video_restore_sweep(err[0] ? err : "no picture");
        return;
    }
    g_video_tuned = mhz;
    video_set_status(1, mhz, "");
    fprintf(stderr, "phasegaze: video %.1f MHz\n", mhz);
}

static void video_leave(void)
{
    if (!g_video_on && !g_video_mode)
        return;
    fprintf(stderr, "phasegaze: sweep\n");
    video_restore_sweep("");
}

static void video_fail(void)
{
    char err[80];
    video_pipeline_error(err, sizeof err);
    fprintf(stderr, "phasegaze: video ended\n");
    video_restore_sweep(err[0] ? err : "no picture");
}

static void wifi_set_status(int mode, double mhz, int ch, const char *err)
{
    pthread_mutex_lock(&g_set.mtx);
    g_wifi_mode = mode;
    g_wifi_mhz = mhz;
    g_wifi_ch = ch;
    snprintf(g_wifi_err, sizeof g_wifi_err, "%s", err ? err : "");
    pthread_mutex_unlock(&g_set.mtx);
}

static void wifi_restore_sweep(const char *err)
{
    g_wifi_on = 0;
    g_wifi_tuned = 0;
    wifi_pipeline_stop();
    if (!g_quit && !g_video_on)
        radio_to_sweep();
    wifi_set_status(0, 0, 0, err ? err : "");
}

static int wifi_lock_lo(double mhz)
{
    if (!atomic_load(&g_csi_open)) {
        int ok = 0;
        for (int i = 0; i < 10 && !g_quit; i++) {
            if (csi_dev_open(&g_dev, DEVICE_PATH) == 0) {
                ok = 1;
                break;
            }
            usleep(150000);
        }
        if (!ok)
            return -1;
        atomic_store(&g_csi_open, 1);
    }
    int rc = csi_dev_set_lo(&g_dev, mhz);
    if (rc == 0)
        rc = csi_dev_wifi_front_end(&g_dev);
    csi_dev_close(&g_dev);
    atomic_store(&g_csi_open, 0);
    return rc;
}

static void wifi_enter(double mhz)
{
    if (g_quit) return;
    pthread_mutex_lock(&g_set.mtx);
    int busy = g_lock_busy;
    pthread_mutex_unlock(&g_set.mtx);
    if (busy) {
        wifi_set_status(0, mhz, 0, "lock measure");
        return;
    }
    if (g_video_on) {
        video_restore_sweep("");
    }
    if (g_wifi_on && !wifi_pipeline_dead() && fabs(g_wifi_tuned - mhz) < 0.05)
        return;

    if (!g_wifi_on) {
        if (!workers_quiesce()) {
            wifi_set_status(0, mhz, 0, "sweep busy");
            return;
        }
        if (g_quit) {
            workers_release();
            return;
        }
        g_wifi_on = 1;
        tuner_stop();
        g_tuner_up = 0;
        pthread_mutex_lock(&g_frame_mtx);
        for (int i = 0; i < N_ACC; i++)
            g_acc[i].used = 0;
        pthread_mutex_unlock(&g_frame_mtx);
    } else {
        wifi_pipeline_stop();
    }

    if (g_quit) return;
    int ch = 0;
    const char *band = NULL;
    double snap_mhz = wifi_snap_mhz(mhz, &ch, &band);
    if (wifi_lock_lo(snap_mhz) != 0) {
        wifi_restore_sweep("front end");
        return;
    }
    if (wifi_pipeline_start(snap_mhz) != 0) {
        char err[80];
        wifi_pipeline_error(err, sizeof err);
        wifi_restore_sweep(err[0] ? err : "no packets");
        return;
    }
    g_wifi_tuned = snap_mhz;
    wifi_set_status(1, snap_mhz, ch, "");
    fprintf(stderr, "phasegaze: wifi %.1f MHz (ch %d)\n", snap_mhz, ch);
}

static void wifi_leave(void)
{
    if (!g_wifi_on && !g_wifi_mode)
        return;
    fprintf(stderr, "phasegaze: sweep (exit wifi)\n");
    wifi_restore_sweep("");
}

static void wifi_fail(void)
{
    char err[80];
    wifi_pipeline_error(err, sizeof err);
    fprintf(stderr, "phasegaze: wifi ended\n");
    wifi_restore_sweep(err[0] ? err : "no packets");
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
    atomic_store(&g_csi_open, 1);
    lock_load();
    clamp_sweep_to_hw();
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
    g_plan = plan;
    g_plan_set = 1;
    pthread_mutex_unlock(&g_set.mtx);
    if (tuner_start(&g_dev, TUNER_CPU, &plan) != 0) {
        fprintf(stderr, "tuner start failed\n");
        return 1;
    }
    g_tuner_up = 1;

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
        int cmd = 0;
        double mhz = 0;
        pthread_mutex_lock(&g_cmd_mtx);
        if (!g_cmd) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 500000000L;
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000L;
            }
            pthread_cond_timedwait(&g_cmd_cv, &g_cmd_mtx, &ts);
        }
        cmd = g_cmd;
        mhz = g_cmd_mhz;
        int snap = g_cmd_snap;
        g_cmd = 0;
        pthread_mutex_unlock(&g_cmd_mtx);
        if (g_quit) break;
        if (cmd == 1) {
            /* A hold-bar step is the LO the user asked for. Snap and
             * the discriminator walk would pull it back onto a channel. */
            if (snap) {
                g_video_nudges = 0;
                video_enter(video_snap_mhz(mhz));
            } else {
                g_video_nudges = 2;
                video_enter(mhz);
            }
        } else if (cmd == 2)
            video_leave();
        else if (cmd == 3)
            wifi_enter(mhz);
        else if (cmd == 4)
            wifi_leave();
        if (g_video_on && g_video_nudges < 2) {
            double nudge = 0;
            if (video_steer_take(&nudge)) {
                g_video_nudges++;
                fprintf(stderr, "phasegaze: video center %.0f MHz\n", nudge);
                video_enter(nudge);
            }
        }
        if (g_video_on && video_pipeline_dead())
            video_fail();
        if (g_wifi_on && wifi_pipeline_dead())
            wifi_fail();
        if (!atomic_load(&g_csi_open) && !g_video_on && !g_wifi_on)
            radio_to_sweep();
        server_broadcast_state();
    }

    video_pipeline_stop();
    g_video_on = 0;
    wifi_pipeline_stop();
    g_wifi_on = 0;
    /* Park stays set if the ring is already unmapped. g_quit lets the
     * wait return; clearing park here would send a worker into it. */
    pthread_mutex_lock(&g_park_mtx);
    pthread_cond_broadcast(&g_park_cv);
    pthread_mutex_unlock(&g_park_mtx);
    for (int i = 0; i < N_WORKERS; ++i)
        pthread_join(wk[i].th, NULL);
    /* The measure thread stops and restarts the tuner. Wait it out so
     * this join is the only one. */
    for (int i = 0; i < 2000 && g_lock_busy; i++)
        usleep(10000);
    if (g_tuner_up)
        tuner_stop();
    server_stop();
    for (int i = 0; i < N_WORKERS; ++i)
        wctx_free(&wk[i]);
    if (atomic_load(&g_csi_open))
        csi_dev_close(&g_dev);
    return 0;
}
