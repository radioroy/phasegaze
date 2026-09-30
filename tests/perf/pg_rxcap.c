// pg_rxcap.c
// Raw full-span capture for receiver-response and hop-hysteresis tests.
//
// Record: pg_rxrec_t header, then uint8 span[span_bytes] (16384 frames x
// 4 ch x CS8 IQ; the workers only keep the second half).
//
//   pg_rxcap OUT.bin dwell SPANS GAINS LO [LO ...]
//       GAINS is comma separated. Per gain, per LO: set_lo, flush, settle 6
//       spans, record SPANS spans with the LO held (backlog dropped between
//       reads, so spans are not contiguous).
//   pg_rxcap OUT.bin hop SPANS GAIN LO [LO ...]
//       Runs the phasegaze tuner on that plan and records SPANS contiguous
//       spans with their tags (prev_lo = LO of the span before).
//
// Stop quadrf-phasegaze first. The VCO seed file is not written.

#define _GNU_SOURCE
#include "../../src/csi_dev.c"
#include "tuner.h"

#define SETTLE_SPANS 6

typedef struct {
    double   lo, prev_lo;
    int32_t  gain, mode;        /* mode 0 dwell, 1 hop */
    int32_t  hop, nhops;
    int32_t  valid, dup;
    int32_t  lna_db, vga_db;
    uint32_t seq, sweep;
    uint64_t t_ns;
} pg_rxrec_t;

static int parse_list(const char *s, double *out, int cap)
{
    int n = 0;
    while (*s && n < cap) {
        char *e;
        out[n++] = strtod(s, &e);
        if (e == s) break;
        s = *e ? e + 1 : e;
    }
    return n;
}

static int run_dwell(csi_dev_t *d, FILE *f, int spans, const double *gains,
                     int ng, const double *lo, int nlo)
{
    uint8_t *buf = malloc(d->span_bytes);
    if (!buf) return 1;
    csi_dev_prime_los(d, lo, nlo);
    g_seed_dirty = 0;
    uint32_t seq = 0;
    for (int g = 0; g < ng; ++g) {
        csi_dev_set_gain(d, (int)gains[g]);
        usleep(3000);
        csi_dev_probe_analog_gain(d);
        fprintf(stderr, "gain %d: lna %d vga %d\n", (int)gains[g],
                d->analog_lna_db, d->analog_vga_db);
        for (int i = 0; i < nlo; ++i) {
            csi_dev_set_lo(d, lo[i]);
            csi_dev_flush(d, 0);
            for (int s = 0; s < SETTLE_SPANS; ++s)
                csi_dev_read_settled_block(d, buf, d->span_bytes, 0.0);
            for (int s = 0; s < spans; ++s) {
                csi_dev_read_settled_block(d, buf, d->span_bytes, 0.0);
                pg_rxrec_t h = {
                    .lo = lo[i], .prev_lo = lo[i], .gain = (int)gains[g],
                    .mode = 0, .hop = i, .nhops = nlo, .valid = 1,
                    .lna_db = d->analog_lna_db, .vga_db = d->analog_vga_db,
                    .seq = seq++, .t_ns = now_ns(),
                };
                fwrite(&h, sizeof(h), 1, f);
                fwrite(buf, 1, d->span_bytes, f);
            }
        }
    }
    free(buf);
    return 0;
}

static int run_hop(csi_dev_t *d, FILE *f, int spans, int gain,
                   const double *lo, int nlo)
{
    const uint32_t span = d->span_bytes;
    const uint64_t rsz = d->ring_size;
    /* Kept in RAM and written after the tuner stops: 300 MB/s of spans
     * would not survive the SD card. */
    uint8_t *mem = malloc((size_t)spans * span);
    pg_rxrec_t *hdr = calloc((size_t)spans, sizeof(pg_rxrec_t));
    if (!mem || !hdr) { fprintf(stderr, "no memory\n"); return 1; }

    csi_dev_set_gain(d, gain);
    usleep(3000);
    csi_dev_probe_analog_gain(d);

    tuner_plan_t p = { .n = nlo, .lo_start = 4900.0f, .lo_end = 6100.0f };
    for (int i = 0; i < nlo; ++i) p.lo[i] = lo[i];
    csi_dev_flush(d, 0);
    if (tuner_start(d, 1, &p) != 0) return 1;
    usleep(300000);
    csi_dev_flush(d, 0);

    int n = 0;
    uint64_t lost = 0, untagged = 0;
    double prev = 0.0;
    while (n < spans) {
        uint32_t head, tail;
        if (csi_dev_ring_pos(d, &head, &tail) != 0) break;
        uint32_t used = ring_used(head, tail, rsz);
        uint32_t frag = used % span;
        if (frag) {
            consume(d->fd, frag);
            tail = (uint32_t)(((uint64_t)tail + frag) % rsz);
            used -= frag;
        }
        if (used > 32 * span) {
            /* Fell behind the ring; the sequence has a gap, restart it. */
            consume(d->fd, used);
            lost++;
            n = 0;
            prev = 0.0;
            continue;
        }
        if (used < span) {
            csi_dev_wait(d, 20);
            continue;
        }
        uint32_t end = (uint32_t)(((uint64_t)tail + span) % rsz);
        span_tag_t t;
        pg_rxrec_t *h = &hdr[n];
        memset(h, 0, sizeof(*h));
        if (tuner_lookup(end, &t)) {
            h->lo = t.lo;
            h->hop = t.hop;
            h->nhops = t.nhops;
            h->valid = t.valid;
            h->dup = t.dup;
            h->sweep = t.sweep;
        } else {
            untagged++;
            h->valid = -1;
        }
        h->prev_lo = prev;
        prev = h->lo;
        h->gain = gain;
        h->mode = 1;
        h->lna_db = d->analog_lna_db;
        h->vga_db = d->analog_vga_db;
        h->seq = (uint32_t)n;
        h->t_ns = now_ns();
        uint8_t *dst = mem + (size_t)n * span;
        uint32_t n1 = span;
        if ((uint64_t)tail + n1 > rsz) n1 = (uint32_t)(rsz - tail);
        memcpy(dst, (const uint8_t *)d->ring + tail, n1);
        if (n1 < span) memcpy(dst + n1, d->ring, span - n1);
        consume(d->fd, span);
        n++;
    }
    tuner_stats_t ts;
    tuner_get_stats(&ts);
    tuner_stop();
    fprintf(stderr, "hop: %d spans, restarts %llu, untagged %llu, rt %d, "
            "retunes %llu, deferred %llu, late_land %llu, resyncs %llu\n",
            n, (unsigned long long)lost, (unsigned long long)untagged, ts.rt,
            (unsigned long long)ts.retunes, (unsigned long long)ts.deferred,
            (unsigned long long)ts.late_land, (unsigned long long)ts.resyncs);
    for (int i = 0; i < n; ++i) {
        fwrite(&hdr[i], sizeof(pg_rxrec_t), 1, f);
        fwrite(mem + (size_t)i * span, 1, span, f);
    }
    free(mem);
    free(hdr);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 6) {
        fprintf(stderr,
                "usage: %s OUT.bin dwell SPANS GAINS LO...\n"
                "       %s OUT.bin hop SPANS GAIN LO...\n", argv[0], argv[0]);
        return 1;
    }
    int spans = atoi(argv[3]);
    double gains[32], lo[TUNER_MAX_HOPS];
    int ng = parse_list(argv[4], gains, 32);
    int nlo = 0;
    for (int i = 5; i < argc && nlo < TUNER_MAX_HOPS; ++i) lo[nlo++] = atof(argv[i]);

    csi_dev_t d;
    if (csi_dev_open(&d, "/dev/csi_stream0") != 0) return 1;
    FILE *f = fopen(argv[1], "wb");
    if (!f) { perror(argv[1]); return 1; }
    int rc;
    if (!strcmp(argv[2], "dwell"))
        rc = run_dwell(&d, f, spans, gains, ng, lo, nlo);
    else
        rc = run_hop(&d, f, spans, (int)gains[0], lo, nlo);
    fclose(f);
    g_seed_dirty = 0;
    csi_dev_close(&d);
    return rc;
}
