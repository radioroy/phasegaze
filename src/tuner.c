// tuner.c — span-synchronous LO sequencer, see tuner.h.

#define _GNU_SOURCE
#include "tuner.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include "fpga_csi.h"

/* Delay from the end of the set_lo ioctl until the change is in the ring,
 * against the span-publication envelope (latprobe with the tail on the
 * span grid, gain 48):
 *   FPGA 0x6A gain step: 50% at +139..153 us
 *   4910 <-> 5470 (VCO band word first, Main2 + gain restrobe last, 118 us
 *   batch): starts +30..60 us, switching transient to +120 us, settled
 *   by +150 us
 * A same-band 20 MHz hop is the same batch minus Main2/gain (80 us); its
 * first word lands ~60 us before the ioctl returns + ~145 us. */
#define ADJ_BEGIN_NS      80000.0
#define ADJ_END_NS       165000.0
#define BAND_BEGIN_NS     35000.0
#define BAND_END_NS      165000.0
#define BIG_JUMP_MHZ        100.0   /* sweep wrap etc.: use band timing */

/* 16384 samples / 37.3726 Msps. Tracked against the lower envelope of
 * publication times, since FPGA and Pi clocks are independent. */
#define SPAN_NS_NOM      438394.0

/* Transition window inside span r+2, measured from its first sample. The
 * worker keeps the second half (from 219 us), so everything must be
 * settled a bit before that; the lower bound keeps span r+1 clean. */
#define LAND_TARGET_NS    20000.0
#define LAND_LIMIT_NS    210000.0

#define WAKE_EARLY_NS     12000.0
#define DETECT_EARLY_NS   20000.0
#define DETECT_LATE_NS    60000.0
#define DETECT_EVERY      4
#define ENV_WINDOW        16        /* detections per envelope update */
#define WARMUP_SPANS      16
#define LAG_RESYNC        8         /* spans the head may trail the envelope */
#define TAG_SLOTS         64        /* 8 MiB ring / 128 KiB span */
#define RT_PRIO           50

typedef struct {
    atomic_uint seq;
    span_tag_t  t;
} tag_slot_t;

static tag_slot_t g_tags[TAG_SLOTS];
static uint32_t   g_span_bytes;

static struct {
    csi_dev_t      *d;
    pthread_t       th;
    int             cpu;
    atomic_int      quit;

    pthread_mutex_t mtx;
    tuner_plan_t    next_plan;
    int             plan_pending;
    int             gain_pending;       /* -1: none */
    tuner_stats_t   st;

    /* Tuner thread only below. */
    tuner_plan_t    plan;
    int             idx;
    double          lo;
    uint32_t        sweep;
    uint64_t        ring;
    uint32_t        span;
    uint32_t        raw_head0, last_raw;
    uint64_t        abs_head;           /* bytes published since resync */
    int64_t         r_last;             /* last slot handled */
    double          p_ref;              /* span r_ref published at p_ref */
    int64_t         r_ref;
    double          span_ns;
    double          win_min;
    int             win_n;
    double          w_adj, w_band;      /* set_lo duration EMA */
    span_tag_t      last;               /* tag for span r_last + 2 */
    int             hold;
    int             warm;
    uint64_t        lost;               /* driver loss counters, summed */
} T = {
    .mtx = PTHREAD_MUTEX_INITIALIZER,
    .gain_pending = -1,
};

static inline double now_d(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static inline void cpu_relax(void)
{
#if defined(__aarch64__) || defined(__arm__)
    __asm__ __volatile__("yield" ::: "memory");
#else
    __asm__ __volatile__("" ::: "memory");
#endif
}

static void sleep_until(double t)
{
    if (t - now_d() < 3000.0)
        return;
    uint64_t ti = (uint64_t)t;
    struct timespec ts = { .tv_sec = (time_t)(ti / 1000000000ull),
                           .tv_nsec = (long)(ti % 1000000000ull) };
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR)
        ;
}

static void spin_until(double t)
{
    while (now_d() < t)
        cpu_relax();
}

static int read_head(void)
{
    uint32_t h;
    if (csi_dev_ring_pos(T.d, &h, NULL) != 0)
        return -1;
    T.abs_head += ((uint64_t)h + T.ring - T.last_raw) % T.ring;
    T.last_raw = h;
    return 0;
}

static inline int64_t newest(void) { return (int64_t)(T.abs_head / T.span); }

static inline uint32_t raw_end_of(int64_t r)
{
    return (uint32_t)(((uint64_t)T.raw_head0 + (uint64_t)r * T.span) % T.ring);
}

static inline double env(int64_t r)
{
    return T.p_ref + (double)(r - T.r_ref) * T.span_ns;
}

static void tag_put(const span_tag_t *t)
{
    tag_slot_t *s = &g_tags[(t->raw_end / T.span) % TAG_SLOTS];
    unsigned q = atomic_load_explicit(&s->seq, memory_order_relaxed);
    atomic_store_explicit(&s->seq, q + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    s->t = *t;
    atomic_store_explicit(&s->seq, q + 2, memory_order_release);
}

int tuner_lookup(uint32_t raw_end, span_tag_t *out)
{
    if (!g_span_bytes)
        return 0;
    tag_slot_t *s = &g_tags[(raw_end / g_span_bytes) % TAG_SLOTS];
    for (int i = 0; i < 16; i++) {
        unsigned q1 = atomic_load_explicit(&s->seq, memory_order_acquire);
        if (q1 & 1u)
            continue;
        span_tag_t t = s->t;
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load_explicit(&s->seq, memory_order_relaxed) != q1)
            continue;
        if (q1 == 0 || t.raw_end != raw_end)
            return 0;
        *out = t;
        return 1;
    }
    return 0;
}

static void put_span(int64_t r, int valid, int dup)
{
    span_tag_t t = {
        .raw_end = raw_end_of(r),
        .sweep = T.sweep,
        .hop = (uint16_t)T.idx,
        .nhops = (uint16_t)T.plan.n,
        .valid = (uint8_t)valid,
        .dup = (uint8_t)dup,
        .lo = T.lo,
        .lo_start = T.plan.lo_start,
        .lo_end = T.plan.lo_end,
    };
    tag_put(&t);
    T.last = t;
}

/* No retune this slot: span r keeps the current LO end to end. */
static void put_carry(int64_t r)
{
    int dup = T.last.valid && T.last.hop == T.idx && T.last.sweep == T.sweep;
    put_span(r, 1, dup);
}

/* Publication lateness vs the envelope. The envelope follows the window
 * minimum so it tracks the earliest (least delayed) publications. */
static void env_update(int64_t r, double late)
{
    T.st.detections++;
    double shift = 0.0;
    if (late < 0.0) {
        shift = late;
        T.win_min = INFINITY;
        T.win_n = 0;
    } else {
        if (late < T.win_min)
            T.win_min = late;
        if (++T.win_n < ENV_WINDOW)
            return;
        shift = T.win_min;
        T.st.min_late_us = T.win_min / 1e3;
        T.win_min = INFINITY;
        T.win_n = 0;
    }
    /* Phase only. A 100 ppm period error drifts ~3 us per window, well
     * inside the landing margin, and fitting the period to window minima
     * walks it upward because each new minimum is biased late. */
    T.p_ref = env(r) + shift;
    T.r_ref = r;
}

static uint64_t loss_count(void)
{
    struct csi_stats cs;
    memset(&cs, 0, sizeof(cs));
    if (ioctl(T.d->fd, CSI_IOC_GET_STATS, &cs) != 0)
        return T.lost;
    uint64_t n = cs.overflows + cs.overflows_ring;
    for (int i = 0; i < CSI_VC_MAX; i++)
        n += (uint64_t)cs.discards_overflow[i] + cs.discards_len_limit[i] +
             cs.discards_unmatched[i] + cs.discards_inactive[i];
    return n;
}

static int resync(void)
{
    uint32_t h0, h;
    if (csi_dev_ring_pos(T.d, &h0, NULL) != 0)
        return -1;
    double t0 = now_d(), t;
    for (;;) {
        if (atomic_load(&T.quit))
            return -1;
        if (csi_dev_ring_pos(T.d, &h, NULL) != 0)
            return -1;
        t = now_d();
        if (h != h0)
            break;
        /* ~4.5 spans. Longer means the stream is stalled, and a FIFO thread
         * spinning here would starve everything else pinned to this core. */
        if (t - t0 > 2e6)
            return -1;
        cpu_relax();
    }
    T.st.resyncs++;
    T.raw_head0 = h;
    T.last_raw = h;
    T.abs_head = 0;
    T.r_last = 0;
    T.p_ref = t;
    T.r_ref = 0;
    T.win_min = INFINITY;
    T.win_n = 0;
    T.hold = 0;
    T.warm = WARMUP_SPANS;
    T.lost = loss_count();
    /* A retune may already be in the pipe for these. */
    put_span(1, 0, 0);
    put_span(2, 0, 0);
    return 0;
}

static void apply_plan(void)
{
    csi_dev_prime_los(T.d, T.plan.lo, T.plan.n);
    T.idx = 0;
    T.lo = T.plan.lo[0];
    csi_dev_set_lo(T.d, T.lo);
    T.sweep++;
}

static void *tuner_main(void *arg)
{
    (void)arg;
    if (T.cpu >= 0) {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        CPU_SET(T.cpu, &cs);
        pthread_setaffinity_np(pthread_self(), sizeof(cs), &cs);
    }
    struct sched_param sp = { .sched_priority = RT_PRIO };
    T.st.rt = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0;
    prctl(PR_SET_TIMERSLACK, 1UL, 0, 0, 0);
    if (!T.st.rt)
        fprintf(stderr, "phasegaze: tuner without SCHED_FIFO (LimitRTPRIO?), "
                        "expect more deferred hops\n");

    int need_sync = 1;
    while (!atomic_load(&T.quit)) {
        pthread_mutex_lock(&T.mtx);
        int new_plan = T.plan_pending;
        if (new_plan) {
            T.plan = T.next_plan;
            T.plan_pending = 0;
        }
        int gain = T.gain_pending;
        T.gain_pending = -1;
        pthread_mutex_unlock(&T.mtx);

        if (new_plan) {
            apply_plan();
            need_sync = 1;
        }
        if (need_sync) {
            if (resync() != 0) {
                usleep(20000);
                continue;
            }
            need_sync = 0;
        }

        if (read_head() != 0) {
            need_sync = 1;
            continue;
        }
        /* Head already past what the envelope expects: that span was
         * published no later than now. */
        {
            int64_t rn = newest();
            double late = now_d() - env(rn);
            if (rn > T.r_last && late < 0.0)
                env_update(rn, late);
        }
        /* Overslept whole slots: nothing was retuned in them. */
        for (int64_t rn = newest(); T.r_last + 1 < rn; ) {
            T.r_last++;
            put_carry(T.r_last + 2);
            T.st.spans++;
            T.st.deferred++;
        }

        int64_t r = T.r_last + 1;
        double E = env(r);
        int want = !T.hold && !T.warm && T.plan.n > 0;
        int next = want ? (T.idx + 1) % T.plan.n : T.idx;
        double lo_next = T.plan.lo[next];
        int write = want && fabs(lo_next - T.lo) > 1e-6;
        int band = write && (csi_dev_lo_switches_band(T.d, lo_next) ||
                             fabs(lo_next - T.lo) > BIG_JUMP_MHZ);
        double w = band ? T.w_band : T.w_adj;
        double lat_b = band ? BAND_BEGIN_NS : ADJ_BEGIN_NS;
        double lat_e = band ? BAND_END_NS : ADJ_END_NS;
        double span_start = E + T.span_ns;      /* first sample of span r+2 */
        double t_write = span_start + LAND_TARGET_NS - lat_b - w;
        double t_dead = span_start + LAND_LIMIT_NS - lat_e - w;

        if (T.warm || (r % DETECT_EVERY) == 0) {
            sleep_until(E - DETECT_EARLY_NS);
            for (;;) {
                if (read_head() != 0)
                    break;
                double t = now_d();
                if (newest() >= r) {
                    env_update(r, t - E);
                    break;
                }
                if (t >= E + DETECT_LATE_NS || t >= t_write)
                    break;
                cpu_relax();
            }
            E = env(r);
            span_start = E + T.span_ns;
            t_write = span_start + LAND_TARGET_NS - lat_b - w;
            t_dead = span_start + LAND_LIMIT_NS - lat_e - w;
        }

        sleep_until(t_write - WAKE_EARLY_NS);
        spin_until(t_write);
        if (read_head() != 0) {
            need_sync = 1;
            continue;
        }
        /* Publication stalls (kworker latency) are harmless: retunes run
         * off the envelope and tags are keyed by span index. Only a long
         * one is suspicious enough to start over. */
        if (newest() < r - LAG_RESYNC) {
            need_sync = 1;
            continue;
        }

        T.st.spans++;
        double t0 = now_d();
        if (!want || t0 > t_dead) {
            if (want)
                T.st.deferred++;
            put_carry(r + 2);
            T.hold = 0;
            if (T.warm)
                T.warm--;
        } else {
            int valid = 1;
            if (write) {
                csi_dev_set_lo(T.d, lo_next);
                double t1 = now_d();
                double wd = t1 - t0;
                if (wd < 1e6) {
                    double *ema = band ? &T.w_band : &T.w_adj;
                    *ema += 0.05 * (wd - *ema);
                }
                if (t1 + lat_b < span_start) {
                    span_tag_t x = T.last;
                    x.valid = 0;
                    tag_put(&x);
                }
                valid = t1 + lat_e <= span_start + LAND_LIMIT_NS;
                T.st.retunes++;
                T.st.write_us = wd / 1e3;
            }
            if (next == 0)
                T.sweep++;
            T.idx = next;
            T.lo = lo_next;
            put_span(r + 2, valid, 0);
            if (!valid) {
                /* Hold one slot so span r+3 carries this hop cleanly. */
                T.st.late_land++;
                T.hold = 1;
            }
        }
        T.r_last = r;

        /* A frame lost anywhere before the ring shifts every later span
         * index by one against time. */
        if ((r % DETECT_EVERY) == 0) {
            uint64_t lost = loss_count();
            if (lost != T.lost) {
                T.lost = lost;
                need_sync = 1;
            }
        }

        if (gain >= 0) {
            csi_dev_set_gain(T.d, gain);
            csi_dev_probe_analog_gain(T.d);
        }
    }
    return NULL;
}

int tuner_start(csi_dev_t *d, int cpu, const tuner_plan_t *plan)
{
    T.d = d;
    T.cpu = cpu;
    T.span = d->span_bytes;
    T.ring = d->ring_size;
    T.span_ns = SPAN_NS_NOM;
    T.st.span_us = SPAN_NS_NOM / 1e3;
    T.w_adj = 80000.0;
    T.w_band = 118000.0;
    g_span_bytes = d->span_bytes;
    T.next_plan = *plan;
    T.plan_pending = 1;
    atomic_store(&T.quit, 0);
    return pthread_create(&T.th, NULL, tuner_main, NULL);
}

void tuner_stop(void)
{
    atomic_store(&T.quit, 1);
    pthread_join(T.th, NULL);
}

void tuner_set_plan(const tuner_plan_t *plan)
{
    pthread_mutex_lock(&T.mtx);
    T.next_plan = *plan;
    T.plan_pending = 1;
    pthread_mutex_unlock(&T.mtx);
}

void tuner_set_gain(int gain)
{
    pthread_mutex_lock(&T.mtx);
    T.gain_pending = gain;
    pthread_mutex_unlock(&T.mtx);
}

void tuner_get_stats(tuner_stats_t *s)
{
    pthread_mutex_lock(&T.mtx);
    *s = T.st;
    pthread_mutex_unlock(&T.mtx);
}
