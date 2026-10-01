// pg_agcprobe.c
// Time a commanded FPGA 0x6A gain step in the CSI ring, then watch FPGA
// AGC hunt after enable and after an LO hop. Analog MAX2851 settle is
// hundreds of ns; this measures the JTAG+SPI path plus any analog edge.
//
// Stop quadrf-phasegaze first.
//   pg_agcprobe [lo_mhz]

#include "../../src/csi_dev.c"

#include <math.h>

#define FS_HZ          37372600.0
#define FRAMES         16384
#define FRAME_B        8
#define SPAN_B         (FRAMES * FRAME_B)
#define NS_PER_FRAME   (1e9 / FS_HZ)
#define CHUNK          32       /* 0.856 us */
#define NSPAN_STEP     4
#define NSPAN_AGC      48
#define REPS           6

static double rms_chunk(const int8_t *p, int frames)
{
    long long s = 0;
    int n = frames * FRAME_B;
    for (int i = 0; i < n; i++) {
        int v = p[i];
        s += (long long)v * v;
    }
    return sqrt((double)s / (double)n);
}

static int wait_span(csi_dev_t *d, uint8_t *dst, uint64_t *t_pub)
{
    uint32_t span = d->span_bytes ? d->span_bytes : SPAN_B;
    uint64_t rsz = d->ring_size;
    int spins = 20000;
    for (;;) {
        struct csi_ring_info ri;
        if (ioctl(d->fd, CSI_IOC_GET_RING_INFO, &ri) < 0) {
            usleep(1);
            continue;
        }
        if (ri.ring_size) rsz = ri.ring_size;
        uint32_t used = ring_used(ri.head, ri.tail, rsz);
        used -= used % span;
        if (used > span) {
            if (consume(d->fd, used - span) != 0) continue;
            continue;
        }
        if (used >= span) {
            uint32_t n1 = span;
            if ((uint64_t)ri.tail + n1 > rsz)
                n1 = (uint32_t)(rsz - ri.tail);
            memcpy(dst, (const uint8_t *)d->ring + ri.tail, n1);
            if (span - n1)
                memcpy(dst + n1, (const uint8_t *)d->ring, span - n1);
            if (t_pub) *t_pub = now_ns();
            if (consume(d->fd, span) != 0) continue;
            return 0;
        }
        if (spins-- > 0) cpu_relax();
        else usleep(1);
    }
}

static void analog(csi_dev_t *d)
{
    csi_dev_probe_analog_gain(d);
    uint16_t v6a = 0;
    jtag_read(d, 0x6A, &v6a);
    fprintf(stderr, "    0x6A=0x%04x LNA %d dB VGA %d dB\n",
            v6a, d->analog_lna_db, d->analog_vga_db);
}

/* t50/t10-90 relative to ioctl return. Sample i of a span published at
 * t_pub was taken at t_pub - (FRAMES-1-i)*NS_PER_FRAME. */
static void measure_step(csi_dev_t *d, int g0, int g1, int reps)
{
    static uint8_t span[NSPAN_STEP][SPAN_B];
    uint64_t pub[NSPAN_STEP];
    double t50s[16], w1090[16];
    int nok = 0;
    double pre_m = 0, post_m = 0;

    fprintf(stderr, "\n== commanded step %d -> %d dB, %d reps ==\n", g0, g1, reps);
    csi_dev_set_gain(d, g0);
    usleep(3000);
    analog(d);

    for (int r = 0; r < reps; r++) {
        csi_dev_set_gain(d, g0);
        usleep(800);
        csi_dev_flush(d, 0);
        wait_span(d, span[0], NULL);   /* align: next span is in flight */

        uint64_t t0 = now_ns();
        csi_dev_set_gain(d, g1);
        uint64_t t1 = now_ns();

        for (int s = 0; s < NSPAN_STEP; s++)
            wait_span(d, span[s], &pub[s]);

        const int nch = (NSPAN_STEP * FRAMES) / CHUNK;
        double rms[nch];
        for (int c = 0; c < nch; c++) {
            int si = (c * CHUNK) / FRAMES;
            int fi = (c * CHUNK) % FRAMES;
            rms[c] = rms_chunk((const int8_t *)span[si] + fi * FRAME_B, CHUNK);
        }
        double pre = 0, post = 0;
        int npre = 8, npost = 16;
        for (int i = 0; i < npre; i++) pre += rms[i];
        for (int i = nch - npost; i < nch; i++) post += rms[i];
        pre /= npre;
        post /= npost;
        double mid = 0.5 * (pre + post);
        double lo = pre + 0.1 * (post - pre);
        double hi = pre + 0.9 * (post - pre);
        int dir = post > pre ? 1 : -1;
        if (fabs(post - pre) < 0.15) {
            fprintf(stderr, "  rep %d: no amplitude step (pre %.2f post %.2f) ioctl %.1f us\n",
                    r, pre, post, (t1 - t0) / 1e3);
            continue;
        }
        int i50 = -1, i10 = -1, i90 = -1;
        for (int c = 0; c < nch; c++) {
            int crossed = dir > 0 ? rms[c] >= mid : rms[c] <= mid;
            if (i50 < 0 && crossed) i50 = c;
            if (i10 < 0 && (dir > 0 ? rms[c] >= lo : rms[c] <= lo)) i10 = c;
            if (i90 < 0 && (dir > 0 ? rms[c] >= hi : rms[c] <= hi)) i90 = c;
        }
        if (i50 < 0) {
            fprintf(stderr, "  rep %d: step not found\n", r);
            continue;
        }
        int si = (i50 * CHUNK) / FRAMES;
        int fi = (i50 * CHUNK) % FRAMES + CHUNK / 2;
        double t_samp = (double)pub[si] - (FRAMES - 1 - fi) * NS_PER_FRAME;
        double t50 = (t_samp - (double)t1) / 1e3;
        double tw = 0;
        if (i10 >= 0 && i90 >= 0)
            tw = (i90 - i10) * CHUNK * NS_PER_FRAME / 1e3;
        fprintf(stderr,
                "  rep %d: ioctl %.1f us  t50 %+6.1f us  10-90 %.2f us  "
                "rms %.2f -> %.2f\n",
                r, (t1 - t0) / 1e3, t50, tw, pre, post);
        t50s[nok] = t50;
        w1090[nok] = tw;
        pre_m += pre;
        post_m += post;
        nok++;
    }
    if (!nok) return;
    double m50 = 0, m10 = 0, mn = 1e9, mx = -1e9;
    for (int i = 0; i < nok; i++) {
        m50 += t50s[i];
        m10 += w1090[i];
        if (t50s[i] < mn) mn = t50s[i];
        if (t50s[i] > mx) mx = t50s[i];
    }
    analog(d);
    fprintf(stderr, "  mean t50 %+5.1f us  range [%+.1f, %+.1f]  "
            "mean 10-90 %.2f us  rms %.2f -> %.2f  (x%.2f)\n",
            m50 / nok, mn, mx, m10 / nok, pre_m / nok, post_m / nok,
            (post_m / nok) / fmax(pre_m / nok, 1e-9));
}

static void dump_step_wave(csi_dev_t *d, int g0, int g1)
{
    static uint8_t span[NSPAN_STEP][SPAN_B];
    uint64_t pub[NSPAN_STEP];
    csi_dev_set_gain(d, g0);
    usleep(1500);
    csi_dev_flush(d, 0);
    wait_span(d, span[0], NULL);
    csi_dev_set_gain(d, g1);
    uint64_t t1 = now_ns();
    for (int s = 0; s < NSPAN_STEP; s++)
        wait_span(d, span[s], &pub[s]);
    fprintf(stderr, "\n== waveform %d -> %d (ioctl end, then 8 us bins) ==\n", g0, g1);
    const int bin_fr = 300; /* 8.03 us */
    for (int s = 0; s < NSPAN_STEP; s++) {
        for (int f = 0; f < FRAMES; f += bin_fr) {
            int n = FRAMES - f < bin_fr ? FRAMES - f : bin_fr;
            double r = rms_chunk((const int8_t *)span[s] + f * FRAME_B, n);
            double t = ((double)pub[s] - (FRAMES - 1 - f) * NS_PER_FRAME
                        - (double)t1) / 1e3;
            fprintf(stderr, "  %+7.1f us  rms %5.2f\n", t, r);
        }
    }
}

static void dump_agc_timeline(csi_dev_t *d, const char *tag, int nspan)
{
    static uint8_t buf[SPAN_B];
    double *srms = calloc((size_t)nspan, sizeof(double));
    uint16_t *g6a = calloc((size_t)nspan, sizeof(uint16_t));
    if (!srms || !g6a) return;

    csi_dev_flush(d, 0);
    uint64_t t0 = now_ns();
    for (int s = 0; s < nspan; s++) {
        wait_span(d, buf, NULL);
        srms[s] = rms_chunk((const int8_t *)buf, FRAMES);
        jtag_read(d, 0x6A, &g6a[s]);
    }
    uint64_t t1 = now_ns();

    int gchanges = 0;
    int gmin = (int)(g6a[0] & 0x7F), gmax = gmin;
    fprintf(stderr, "  %s  (%.1f ms, %.1f us/span)\n",
            tag, (t1 - t0) / 1e6, (t1 - t0) / 1e3 / nspan);
    for (int s = 0; s < nspan; s++) {
        int g = (int)(g6a[s] & 0x7F);
        int agc = !!(g6a[s] & 0x80);
        if (s && g != (int)(g6a[s - 1] & 0x7F)) gchanges++;
        if (g < gmin) gmin = g;
        if (g > gmax) gmax = g;
        if (s < 16 || s + 4 >= nspan ||
            (s && g != (int)(g6a[s - 1] & 0x7F)))
            fprintf(stderr, "    span %2d  rms %5.2f  0x6A=0x%04x  gain %2d  agc %d\n",
                    s, srms[s], g6a[s], g, agc);
    }
    fprintf(stderr, "  gain %d..%d, %d changes in %d spans (%.0f us/change if uniform)\n",
            gmin, gmax, gchanges, nspan,
            gchanges ? (t1 - t0) / 1e3 / gchanges : 0);
    free(srms);
    free(g6a);
}

static void enable_agc(csi_dev_t *d, double dbfs)
{
    double thr_f = 180.0 * pow(10.0, dbfs / 20.0);
    if (thr_f < 1.0) thr_f = 1.0;
    if (thr_f > 180.0) thr_f = 180.0;
    uint16_t thr = (uint16_t)lround(thr_f);
    jtag_write(d, 0x6A, 0x0080);
    jtag_write(d, 0x6B, thr);
    fprintf(stderr, "\n== FPGA AGC on, target %.1f dBFS thr %u ==\n", dbfs, thr);
}

int main(int argc, char **argv)
{
    double lo = argc > 1 ? atof(argv[1]) : 5000.0;
    csi_dev_t d;
    if (csi_dev_open(&d, "/dev/csi_stream0") != 0) return 1;
    csi_dev_set_lo(&d, lo);
    usleep(2000);
    fprintf(stderr, "LO %.1f MHz  span %u B  fs %.4f Msps  hop period %.1f us\n",
            lo, d.span_bytes, FS_HZ / 1e6, FRAMES * NS_PER_FRAME / 1e3);

    int agc_only = argc > 2 && strcmp(argv[2], "agc") == 0;
    if (!agc_only) {
        measure_step(&d, 45, 47, REPS);
        measure_step(&d, 47, 45, REPS);
        measure_step(&d, 40, 48, REPS);
        measure_step(&d, 20, 52, REPS);
        measure_step(&d, 52, 20, REPS);
        dump_step_wave(&d, 45, 47);
        dump_step_wave(&d, 20, 52);
    }

    /* FPGA closed-loop: start low, enable, watch hunt. Then hop. */
    csi_dev_set_gain(&d, 20);
    usleep(2000);
    analog(&d);
    enable_agc(&d, -15.0);
    dump_agc_timeline(&d, "agc after enable @ low gain", NSPAN_AGC);
    analog(&d);

    /* Same LNA-band hop so set_lo does not restrobe 0x6A and clear AGC. */
    double lo2 = lo + 20.0;
    csi_dev_set_lo(&d, lo2);
    dump_agc_timeline(&d, "agc after same-band +20 MHz hop", NSPAN_AGC);
    analog(&d);

    csi_dev_set_lo(&d, 5785.0);
    dump_agc_timeline(&d, "agc after band-crossing hop (Main2 restrobe)", NSPAN_AGC);
    analog(&d);

    /* Leave the radio in the same manual-gain state PhaseGaze uses. */
    csi_dev_set_gain(&d, 45);
    analog(&d);
    csi_dev_close(&d);
    return 0;
}
