// csi_dev.c

#define _GNU_SOURCE
#include "csi_dev.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/types.h>   /* __u32/__u64 for the driver UAPI header */
#include "fpga_csi.h"

/* Tighter wait at 38 Msps / 4-lane CSI (same as quadrf-rf-vision). */
#define WAIT_SPIN_ITERS 100
#define WAIT_SLEEP_US   1

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline void cpu_relax(void)
{
#if defined(__aarch64__) || defined(__arm__)
    __asm__ __volatile__("yield" ::: "memory");
#else
    __asm__ __volatile__("" ::: "memory");
#endif
}

static inline uint32_t ring_used(uint32_t head, uint32_t tail, uint64_t size)
{
    return (head >= tail) ? (head - tail) : ((uint32_t)size - (tail - head));
}

/* Advance the CSI tail. The driver returns EINVAL if n > used (it may have
 * discarded unread bytes between GET_RING_INFO and this ioctl). Never abort
 * the process — mongoose is serving the page on another thread. */
static int consume(int fd, uint32_t n)
{
    if (!n) return 0;
    if (ioctl(fd, CSI_IOC_CONSUME_BYTES, &n) == 0) return 0;
    return -1;
}

/* Retry JTAG register access while another process holds the lease.
 * All JTAG ioctls on this fd go through these helpers so the sweep thread
 * (LO hops) cannot interleave with a gain write the way csi_sweep prevents. */
static int jtag_write(csi_dev_t *d, uint8_t addr, uint16_t value)
{
    struct csi_jtag_reg r = { .addr = addr, .value = value };
    pthread_mutex_lock(&d->jtag_mtx);
    uint64_t t0 = now_ns();
    int rc = 0;
    while (ioctl(d->fd, CSI_IOC_JTAG_REG_WRITE, &r) != 0) {
        if (errno != EBUSY) { rc = -1; break; }
        if (now_ns() - t0 > 1000000000ull) { rc = -1; break; }
        usleep(1000);
    }
    pthread_mutex_unlock(&d->jtag_mtx);
    return rc;
}

static int jtag_read(csi_dev_t *d, uint8_t addr, uint16_t *out)
{
    struct csi_jtag_reg r = { .addr = addr };
    pthread_mutex_lock(&d->jtag_mtx);
    uint64_t t0 = now_ns();
    int rc = 0;
    while (ioctl(d->fd, CSI_IOC_JTAG_REG_READ, &r) != 0) {
        if (errno != EBUSY) { rc = -1; break; }
        if (now_ns() - t0 > 1000000000ull) { rc = -1; break; }
        usleep(1000);
    }
    pthread_mutex_unlock(&d->jtag_mtx);
    if (rc == 0) *out = r.value;
    return rc;
}

static int jtag_batch(csi_dev_t *d, struct csi_jtag_reg *regs, uint32_t n)
{
    struct csi_jtag_batch batch = {
        .regs_ptr = (uint64_t)(uintptr_t)regs,
        .count = n,
        .delay_us = 0,
    };
    pthread_mutex_lock(&d->jtag_mtx);
    uint64_t t0 = now_ns();
    int rc = -1;
    for (;;) {
        if (ioctl(d->fd, CSI_IOC_JTAG_BATCH_WRITE, &batch) == 0) {
            rc = 0;
            break;
        }
        if (errno == EINTR) continue;
        if (errno != EBUSY) break;
        if (now_ns() - t0 > 1000000000ull) break;
        usleep(100);
    }
    pthread_mutex_unlock(&d->jtag_mtx);
    if (rc == 0) return 0;
    for (uint32_t i = 0; i < n; i++)
        if (jtag_write(d, regs[i].addr, regs[i].value) != 0)
            return -1;
    return 0;
}

static int max2851_read_main(csi_dev_t *d, uint16_t main_addr, uint16_t *out)
{
    const uint16_t r14 = 0x160;
    pthread_mutex_lock(&d->jtag_mtx);
    int rc = -1;
    struct csi_jtag_reg r;
    uint64_t t0;
    int wr(uint16_t addr, uint16_t val) {
        r.addr = (uint8_t)addr; r.value = val;
        t0 = now_ns();
        while (ioctl(d->fd, CSI_IOC_JTAG_REG_WRITE, &r) != 0) {
            if (errno != EBUSY) return -1;
            if (now_ns() - t0 > 1000000000ull) return -1;
            usleep(1000);
        }
        return 0;
    }
    int rd(uint16_t addr, uint16_t *o) {
        r.addr = (uint8_t)addr; r.value = 0;
        t0 = now_ns();
        while (ioctl(d->fd, CSI_IOC_JTAG_REG_READ, &r) != 0) {
            if (errno != EBUSY) return -1;
            if (now_ns() - t0 > 1000000000ull) return -1;
            usleep(1000);
        }
        *o = r.value;
        return 0;
    }
    if (wr(0x43, (uint16_t)((14u << 10) | (r14 | (1u << 1)))) != 0) goto done;
    if (wr(0x43, (uint16_t)(0x8000u | (main_addr << 10))) != 0) {
        wr(0x43, (uint16_t)((14u << 10) | r14));
        goto done;
    }
    uint16_t val = 0;
    if (rd(0x43, &val) != 0) {
        wr(0x43, (uint16_t)((14u << 10) | r14));
        goto done;
    }
    wr(0x43, (uint16_t)((14u << 10) | r14));
    *out = val & 0x3FFu;
    rc = 0;
done:
    pthread_mutex_unlock(&d->jtag_mtx);
    return rc;
}

static void decode_main1(uint16_t r1, int *lna_db, int *vga_db)
{
    uint8_t lna_val = (uint8_t)((r1 >> 5) & 0x7);
    uint8_t vga_val = (uint8_t)(r1 & 0x1F);
    int lna = 0;
    switch (lna_val) {
        case 0: lna = 0; break;
        case 1: lna = 8; break;
        case 4: lna = 16; break;
        case 5: lna = 24; break;
        case 6: lna = 32; break;
        case 7: lna = 40; break;
        default: lna = 0; break;
    }
    if (vga_val > 15) vga_val = 15;
    *lna_db = lna;
    *vga_db = (int)vga_val * 2;
}

int csi_dev_open(csi_dev_t *d, const char *path)
{
    memset(d, 0, sizeof(*d));
    d->fd = -1;
    d->last_gain = -1;
    d->last_lo_mhz = 0.0;
    d->analog_lna_db = -1;
    d->analog_vga_db = -1;
    pthread_mutex_init(&d->jtag_mtx, NULL);
    d->fd = open(path, O_RDWR | O_NONBLOCK);
    if (d->fd < 0) {
        perror("open csi");
        pthread_mutex_destroy(&d->jtag_mtx);
        return -1;
    }

    ioctl(d->fd, CSI_IOC_JTAG_SETUP);

    /* Hold the JTAG lease for the whole run. quadrf-gui polls
     * `quadrf-jtag --status rx/tx` at 1 Hz while its page is open; each run
     * holds the lease ~9 ms (~20 spans), so LO hops stall and the sweep
     * breaks up. With the lease held here those polls time out after
     * 100 ms instead. The driver drops it when the fd closes. */
    {
        uint64_t t0 = now_ns();
        while (ioctl(d->fd, CSI_IOC_JTAG_ACQUIRE_LEASE) != 0) {
            if (errno == EBUSY && now_ns() - t0 < 2000000000ull) {
                usleep(1000);
                continue;
            }
            if (errno != ENOTTY)
                fprintf(stderr, "csi_dev: JTAG lease not acquired: %s\n",
                        strerror(errno));
            break;
        }
    }

    /* Analog MAX2851: mesh / --rx off leave E_RX as a single antenna and
     * MODE=standby. FPGA 0x25/0x27/0x6A do not turn those paths back on. */
    jtag_write(d, 0x43, (uint16_t)((6u << 10) | 0x3FFu)); /* Main6: all 4 RX */
    jtag_write(d, 0x43, (uint16_t)((0u << 10) | 0x008u)); /* Main0: MODE=RX, 20 MHz */

    uint16_t v2e = 0;
    if (jtag_read(d, 0x2E, &v2e) == 0)
        jtag_write(d, 0x2E, (uint16_t)(v2e & ~0x0003u)); /* autosteer + tone off */

    /* FPGA: AGC off (gain written by caller), 4ch interleave, k=12, RHCP. */
    jtag_write(d, 0x25, 0x0001);
    jtag_write(d, 0x27, 12);
    jtag_write(d, 0x24, 0x0001);

    struct csi_ring_info ri;
    if (ioctl(d->fd, CSI_IOC_GET_RING_INFO, &ri) < 0) {
        perror("CSI_IOC_GET_RING_INFO");
        close(d->fd);
        pthread_mutex_destroy(&d->jtag_mtx);
        return -1;
    }
    if (!ri.ring_size) {
        fprintf(stderr, "csi_dev: ring_size=0\n");
        close(d->fd);
        pthread_mutex_destroy(&d->jtag_mtx);
        return -1;
    }
    d->ring_size = ri.ring_size;
    d->span_bytes = ri.span_bytes ? ri.span_bytes : 131072;

    long page = sysconf(_SC_PAGESIZE);
    d->map_len = (size_t)((ri.ring_size + page - 1) & ~((uint64_t)page - 1));
    d->ring = mmap(NULL, d->map_len, PROT_READ, MAP_SHARED, d->fd, 0);
    if (d->ring == MAP_FAILED) {
        perror("mmap csi ring");
        close(d->fd);
        pthread_mutex_destroy(&d->jtag_mtx);
        return -1;
    }
    return 0;
}

void csi_dev_close(csi_dev_t *d)
{
    if (d->ring && d->ring != MAP_FAILED) munmap(d->ring, d->map_len);
    if (d->fd >= 0) {
        ioctl(d->fd, CSI_IOC_JTAG_RELEASE_LEASE);
        close(d->fd);
    }
    pthread_mutex_destroy(&d->jtag_mtx);
}

void csi_dev_read_block(csi_dev_t *d, uint8_t *dst, uint32_t block_bytes,
                        uint32_t bytes_per_frame, int max_queued_blocks)
{
    int spins = WAIT_SPIN_ITERS;
    for (;;) {
        struct csi_ring_info ri;
        if (ioctl(d->fd, CSI_IOC_GET_RING_INFO, &ri) < 0) {
            perror("CSI_IOC_GET_RING_INFO");
            usleep(WAIT_SLEEP_US);
            continue;
        }
        uint64_t rsz = ri.ring_size ? (uint64_t)ri.ring_size : d->ring_size;
        uint32_t used = ring_used(ri.head, ri.tail, rsz);
        used -= used % bytes_per_frame;

        uint32_t max_keep = (uint32_t)max_queued_blocks * block_bytes;
        if (used > max_keep) {
            uint32_t drop = used - max_keep;
            drop -= drop % bytes_per_frame;
            if (drop && consume(d->fd, drop) != 0) {
                spins = WAIT_SPIN_ITERS;
                continue;
            }
            spins = WAIT_SPIN_ITERS;
            continue;
        }

        if (used >= block_bytes) {
            uint32_t tail = ri.tail;
            uint32_t n1 = block_bytes;
            if ((uint64_t)tail + n1 > rsz)
                n1 = (uint32_t)rsz - tail;
            memcpy(dst, (const uint8_t *)d->ring + tail, n1);
            if (block_bytes - n1)
                memcpy(dst + n1, (const uint8_t *)d->ring, block_bytes - n1);
            if (consume(d->fd, block_bytes) != 0)
                continue;
            return;
        }

        if (spins-- > 0) cpu_relax();
        else usleep(WAIT_SLEEP_US);
    }
}

int csi_dev_ring_pos(csi_dev_t *d, uint32_t *head, uint32_t *tail)
{
    struct csi_ring_info ri;
    if (ioctl(d->fd, CSI_IOC_GET_RING_INFO, &ri) != 0)
        return -1;
    if (head) *head = ri.head;
    if (tail) *tail = ri.tail;
    return 0;
}

int csi_dev_consume(csi_dev_t *d, uint32_t n)
{
    return consume(d->fd, n);
}

int csi_dev_wait(csi_dev_t *d, int timeout_ms)
{
    struct pollfd p = { .fd = d->fd, .events = POLLIN };
    return poll(&p, 1, timeout_ms);
}

void csi_dev_flush(csi_dev_t *d, uint32_t align_bytes)
{
    struct csi_ring_info ri;
    if (ioctl(d->fd, CSI_IOC_GET_RING_INFO, &ri) != 0)
        return;
    (void)align_bytes;
    uint64_t rsz = ri.ring_size ? (uint64_t)ri.ring_size : d->ring_size;
    /* Take everything. The driver keeps rpos across opens, so a previous
     * reader that consumed 64 KiB blocks can leave the tail mid-span;
     * trimming to whole spans would keep that phase and every "span"
     * read afterwards would straddle two DMA spans. The head only moves
     * in whole spans, so emptying the ring puts the tail on its grid. */
    uint32_t used = ring_used(ri.head, ri.tail, rsz);
    if (used)
        (void)consume(d->fd, used);
}

int csi_dev_read_settled_block(csi_dev_t *d, uint8_t *dst, uint32_t block_bytes,
                                double tune_mhz)
{
    uint32_t span = d->span_bytes ? d->span_bytes : (block_bytes * 2);
    uint64_t rsz = d->ring_size;
    uint32_t offset_in_span = (span >= block_bytes) ? (span - block_bytes) : 0;
    int spins = WAIT_SPIN_ITERS;

    for (;;) {
        struct csi_ring_info ri;
        if (ioctl(d->fd, CSI_IOC_GET_RING_INFO, &ri) < 0) {
            perror("CSI_IOC_GET_RING_INFO");
            usleep(WAIT_SLEEP_US);
            continue;
        }
        if (ri.ring_size) rsz = ri.ring_size;
        uint32_t used = ring_used(ri.head, ri.tail, rsz);

        /* Truncate to whole spans to preserve span alignment */
        used -= used % span;

        /* If backlog exceeds 1 span, drop older spans to bound latency */
        if (used > span) {
            uint32_t drop = used - span;
            if (consume(d->fd, drop) != 0) {
                spins = WAIT_SPIN_ITERS;
                continue;
            }
            used = span;
            ri.tail = (uint32_t)(((uint64_t)ri.tail + drop) % rsz);
        }

        if (used >= span) {
            /* The next span has just started. Program its LO before the copy
             * so the PLL gets that interval plus the first half of the span
             * (~215 us at 38 Msps) before the following read labels it. */
            if (tune_mhz > 0.0)
                csi_dev_set_lo(d, tune_mhz);

            uint32_t blk_start = (uint32_t)(((uint64_t)ri.tail + offset_in_span) % rsz);
            uint32_t n1 = block_bytes;
            if ((uint64_t)blk_start + n1 > rsz)
                n1 = (uint32_t)(rsz - blk_start);
            memcpy(dst, (const uint8_t *)d->ring + blk_start, n1);
            if (block_bytes - n1)
                memcpy(dst + n1, (const uint8_t *)d->ring, block_bytes - n1);

            if (consume(d->fd, span) != 0) {
                spins = WAIT_SPIN_ITERS;
                continue;
            }
            return 0;
        }

        if (spins-- > 0) cpu_relax();
        else usleep(WAIT_SLEEP_US);
    }
}

static uint16_t lna_band_reg2(double mhz)
{
    if (mhz < 5200.0) return 0x180;
    if (mhz < 5500.0) return 0x1A0;
    if (mhz < 5800.0) return 0x1C0;
    return 0x1E0;
}

/* Automatic VAS on Main17 does not finish inside the ~215 us second half of
 * a DMA span, so that block was still the previous 20 MHz channel. Manual
 * VAS_SPI[5:0] (Main19 bit6 = 0) is only the fractional-N relock. Seeds are
 * the same file quadrf-rf-vision writes. */
#define MAX2851_SPI         0x43u
#define MAIN19_VAS_RELOCK   (1u << 7)
#define MAIN19_VAS_MODE     (1u << 6)
#define VAS_DEFAULT_SEED    31u
#define VCO_CACHE_SIZE      512u
#define VCO_SEED_MAX        256
#define VCO_SEED_PATH       "/var/lib/quadrf/demos/max2851_vco_seeds.txt"

typedef struct {
    uint64_t key;
    uint8_t  band;
    uint8_t  valid;
} vco_entry_t;

static vco_entry_t g_vco[VCO_CACHE_SIZE];
static struct { double mhz; uint8_t band; } g_seed[VCO_SEED_MAX];
static unsigned g_nseed;
static int g_seed_loaded;
static int g_seed_dirty;

static uint32_t vco_hash(uint64_t key)
{
    uint32_t x = (uint32_t)key ^ (uint32_t)(key >> 32);
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    return x & (VCO_CACHE_SIZE - 1u);
}

static int vco_lookup(uint64_t key, uint8_t *band)
{
    uint32_t i = vco_hash(key);
    for (uint32_t n = 0; n < VCO_CACHE_SIZE; n++) {
        vco_entry_t *e = &g_vco[(i + n) & (VCO_CACHE_SIZE - 1u)];
        if (!e->valid)
            return 0;
        if (e->key == key) {
            *band = e->band;
            return 1;
        }
    }
    return 0;
}

static void vco_store(uint64_t key, uint8_t band)
{
    uint32_t i = vco_hash(key);
    for (uint32_t n = 0; n < VCO_CACHE_SIZE; n++) {
        vco_entry_t *e = &g_vco[(i + n) & (VCO_CACHE_SIZE - 1u)];
        if (!e->valid || e->key == key) {
            e->key = key;
            e->band = (uint8_t)(band & 0x3Fu);
            e->valid = 1;
            return;
        }
    }
    g_vco[i].key = key;
    g_vco[i].band = (uint8_t)(band & 0x3Fu);
    g_vco[i].valid = 1;
}

static void synth_words(double mhz, uint16_t *w15, uint16_t *w16, uint16_t *w17,
                        uint64_t *key)
{
    double ratio = mhz / 80.0;
    long long idiv = (long long)floor(ratio);
    long long fdiv = llround((ratio - (double)idiv) * (double)(1u << 20));
    if (fdiv == (1LL << 20)) {
        idiv++;
        fdiv = 0;
    }
    uint32_t f20 = (uint32_t)fdiv & 0xFFFFFu;
    uint32_t n7 = (uint32_t)idiv & 0x7Fu;
    *w15 = (uint16_t)((15u << 10) | (1u << 9) | n7); /* VAS_TRIG_EN */
    *w16 = (uint16_t)((16u << 10) | ((f20 >> 10) & 0x3FFu));
    *w17 = (uint16_t)((17u << 10) | (f20 & 0x3FFu));
    if (key)
        *key = ((uint64_t)n7 << 20) | f20;
}

static void seed_load(void)
{
    if (g_seed_loaded)
        return;
    g_seed_loaded = 1;
    FILE *f = fopen(VCO_SEED_PATH, "r");
    if (!f)
        return;
    double mhz;
    unsigned band;
    while (g_nseed < VCO_SEED_MAX && fscanf(f, "%lf %u", &mhz, &band) == 2) {
        if (band > 63u)
            continue;
        g_seed[g_nseed].mhz = mhz;
        g_seed[g_nseed].band = (uint8_t)band;
        g_nseed++;
        uint16_t w15, w16, w17;
        uint64_t key;
        synth_words(mhz, &w15, &w16, &w17, &key);
        vco_store(key, (uint8_t)band);
    }
    fclose(f);
}

static int seed_lookup(double mhz, uint8_t *band)
{
    for (unsigned i = 0; i < g_nseed; i++) {
        if (fabs(g_seed[i].mhz - mhz) < 0.01) {
            *band = g_seed[i].band;
            return 1;
        }
    }
    return 0;
}

static void seed_remember(double mhz, uint8_t band)
{
    for (unsigned i = 0; i < g_nseed; i++) {
        if (fabs(g_seed[i].mhz - mhz) < 0.01) {
            if (g_seed[i].band != band) {
                g_seed[i].band = band;
                g_seed_dirty = 1;
            }
            return;
        }
    }
    if (g_nseed >= VCO_SEED_MAX)
        return;
    g_seed[g_nseed].mhz = mhz;
    g_seed[g_nseed].band = band;
    g_nseed++;
    g_seed_dirty = 1;
}

static void seed_save(void)
{
    if (!g_seed_dirty)
        return;
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s.tmp", VCO_SEED_PATH);
    FILE *f = fopen(tmp, "w");
    if (!f)
        return;
    for (unsigned i = 0; i < g_nseed; i++)
        fprintf(f, "%.3f %u\n", g_seed[i].mhz, (unsigned)g_seed[i].band);
    if (fclose(f) != 0) {
        unlink(tmp);
        return;
    }
    if (rename(tmp, VCO_SEED_PATH) != 0) {
        unlink(tmp);
        return;
    }
    g_seed_dirty = 0;
}

/* Main27 D5 opens VCO readback. D[8:6] is the VTUNE zone, D[5:0] the band.
 * Zones 0 and 7 are outside the lock range. */
static int read_vco(csi_dev_t *d, uint8_t *band, uint8_t *adc)
{
    if (jtag_write(d, MAX2851_SPI, (uint16_t)((27u << 10) | 0x1A0u)) != 0)
        return -1;
    if (jtag_write(d, MAX2851_SPI, (uint16_t)((14u << 10) | 0x162u)) != 0)
        return -1;
    if (jtag_write(d, MAX2851_SPI, (uint16_t)(0x8000u | (19u << 10))) != 0)
        return -1;
    uint16_t v = 0;
    int rc = jtag_read(d, MAX2851_SPI, &v);
    jtag_write(d, MAX2851_SPI, (uint16_t)((14u << 10) | 0x160u));
    jtag_write(d, MAX2851_SPI, (uint16_t)((27u << 10) | 0x180u));
    if (rc != 0)
        return -1;
    *band = (uint8_t)(v & 0x3Fu);
    *adc = (uint8_t)((v >> 6) & 0x7u);
    return 0;
}

static int program_auto(csi_dev_t *d, double mhz, int from_current, uint8_t seed)
{
    uint16_t w15, w16, w17;
    synth_words(mhz, &w15, &w16, &w17, NULL);
    uint16_t main19 = from_current
        ? (uint16_t)(MAIN19_VAS_RELOCK | MAIN19_VAS_MODE | VAS_DEFAULT_SEED)
        : (uint16_t)(MAIN19_VAS_MODE | (seed & 0x3Fu));
    uint16_t w2 = (uint16_t)((2u << 10) | (lna_band_reg2(mhz) & 0x3FFu));
    struct csi_jtag_reg regs[5] = {
        { .addr = MAX2851_SPI, .value = (uint16_t)((19u << 10) | main19) },
        { .addr = MAX2851_SPI, .value = w15 },
        { .addr = MAX2851_SPI, .value = w16 },
        { .addr = MAX2851_SPI, .value = w17 },
        { .addr = MAX2851_SPI, .value = w2 },
    };
    return jtag_batch(d, regs, 5);
}

/* Automatic VAS, then hold the band it found. Not used on the hop loop. */
static int vco_learn(csi_dev_t *d, double mhz, int from_current, uint8_t seed)
{
    uint16_t w15, w16, w17;
    uint64_t key;
    synth_words(mhz, &w15, &w16, &w17, &key);
    uint8_t have;
    if (vco_lookup(key, &have))
        return 0;

    if (program_auto(d, mhz, from_current, seed) != 0)
        return -1;

    uint8_t band = 0, adc = 0xFF;
    for (int i = 0; i < 6; i++) {
        usleep(i == 0 ? 1500 : 1000);
        if (read_vco(d, &band, &adc) != 0)
            return -1;
        if (adc >= 1 && adc <= 6)
            break;
    }
    if (adc < 1 || adc > 6)
        return -1;

    if (jtag_write(d, MAX2851_SPI, (uint16_t)((19u << 10) | (band & 0x3Fu))) != 0)
        return -1;
    vco_store(key, band);
    seed_remember(mhz, band);
    d->last_lo_mhz = mhz;
    d->last_lna_word = (uint16_t)((2u << 10) | (lna_band_reg2(mhz) & 0x3FFu));
    d->have_lna_word = 1;
    return 1;
}

static int vco_learn_one(csi_dev_t *d, double mhz, int prefer_current)
{
    uint8_t seed = 0;
    int have_seed = seed_lookup(mhz, &seed);
    if (prefer_current && vco_learn(d, mhz, 1, 0) >= 0)
        return 0;
    if (have_seed && vco_learn(d, mhz, 0, seed) >= 0)
        return 0;
    if ((!have_seed || seed != VAS_DEFAULT_SEED) &&
        vco_learn(d, mhz, 0, VAS_DEFAULT_SEED) >= 0)
        return 0;
    return -1;
}

static int program_manual(csi_dev_t *d, uint8_t band,
                          uint16_t w15, uint16_t w16, uint16_t w17, uint16_t w2,
                          int send_lna)
{
    struct csi_jtag_reg regs[5];
    uint32_t n = 0;
    regs[n].addr = MAX2851_SPI;
    regs[n++].value = (uint16_t)((19u << 10) | (band & 0x3Fu));
    regs[n].addr = MAX2851_SPI; regs[n++].value = w15;
    regs[n].addr = MAX2851_SPI; regs[n++].value = w16;
    regs[n].addr = MAX2851_SPI; regs[n++].value = w17;
    if (send_lna) {
        regs[n].addr = MAX2851_SPI;
        regs[n++].value = w2;
    }
    return jtag_batch(d, regs, n);
}

int csi_dev_lo_switches_band(const csi_dev_t *d, double freq_mhz)
{
    uint16_t w2 = (uint16_t)((2u << 10) | (lna_band_reg2(freq_mhz) & 0x3FFu));
    return !d->have_lna_word || d->last_lna_word != w2;
}

int csi_dev_set_lo(csi_dev_t *d, double freq_mhz)
{
    if (d->last_lo_mhz > 1.0 && fabs(freq_mhz - d->last_lo_mhz) < 1e-6)
        return 0;

    seed_load();
    uint16_t w15, w16, w17;
    uint64_t key;
    synth_words(freq_mhz, &w15, &w16, &w17, &key);
    uint16_t w2 = (uint16_t)((2u << 10) | (lna_band_reg2(freq_mhz) & 0x3FFu));
    int send_lna = !d->have_lna_word || d->last_lna_word != w2;

    uint8_t band = 0;
    if (!vco_lookup(key, &band)) {
        if (vco_learn_one(d, freq_mhz, 1) != 0) {
            fprintf(stderr, "phasegaze: VCO learn failed at %.1f MHz\n", freq_mhz);
            if (program_auto(d, freq_mhz, 1, 0) != 0)
                return -1;
            d->last_lo_mhz = freq_mhz;
        }
        if (d->last_gain >= 0)
            csi_dev_set_gain(d, d->last_gain);
        return 0;
    }

    if (program_manual(d, band, w15, w16, w17, w2, send_lna) != 0)
        return -1;
    d->last_lo_mhz = freq_mhz;
    if (send_lna) {
        d->last_lna_word = w2;
        d->have_lna_word = 1;
        /* Main2 is analog; FPGA Main1 (LNA/VGA split) can go stale. */
        if (d->last_gain >= 0)
            csi_dev_set_gain(d, d->last_gain);
    }
    return 0;
}

static int cmp_double(const void *a, const void *b)
{
    double d = *(const double *)a - *(const double *)b;
    return (d > 0.0) - (d < 0.0);
}

static int prime_at(csi_dev_t *d, double mhz)
{
    uint16_t w15, w16, w17;
    uint64_t key;
    uint8_t band;
    synth_words(mhz, &w15, &w16, &w17, &key);
    if (vco_lookup(key, &band))
        return csi_dev_set_lo(d, mhz);
    return vco_learn_one(d, mhz, 1);
}

int csi_dev_prime_los(csi_dev_t *d, const double *mhz, int n)
{
    if (n <= 0)
        return 0;
    if (n > 128)
        n = 128;
    seed_load();

    double *f = malloc((size_t)n * sizeof(double));
    if (!f)
        return -1;
    memcpy(f, mhz, (size_t)n * sizeof(double));
    qsort(f, (size_t)n, sizeof(double), cmp_double);
    int m = 0;
    for (int i = 0; i < n; i++) {
        if (m > 0 && fabs(f[i] - f[m - 1]) < 0.05)
            continue;
        f[m++] = f[i];
    }

    int need = 0;
    for (int i = 0; i < m; i++) {
        uint16_t w15, w16, w17;
        uint64_t key;
        uint8_t band;
        synth_words(f[i], &w15, &w16, &w17, &key);
        if (!vco_lookup(key, &band))
            need++;
    }
    if (need == 0) {
        free(f);
        return 0;
    }

    uint64_t t0 = now_ns();
    int start = m / 2;
    int failed = 0;
    /* Cached steps are a manual retune so the next miss starts VAS next to
     * the band it just left. */
    for (int i = start; i >= 0; i--) {
        if (prime_at(d, f[i]) != 0)
            failed++;
    }
    if (csi_dev_set_lo(d, f[start]) != 0)
        failed++;
    for (int i = start + 1; i < m; i++) {
        if (prime_at(d, f[i]) != 0)
            failed++;
    }
    if (d->last_gain >= 0)
        csi_dev_set_gain(d, d->last_gain);
    seed_save();
    fprintf(stderr, "phasegaze: VCO primed %d new / %d hops, %d failed, %.0f ms\n",
            need, m, failed, (double)(now_ns() - t0) / 1e6);
    free(f);
    return failed;
}

int csi_dev_set_gain(csi_dev_t *d, int gain)
{
    if (gain < 0) gain = 0;
    if (gain > 63) gain = 63;
    /* Whole register: FPGA splits 0..63 into LNA+VGA+digital. Bit 7 is AGC,
     * so writing the dB value clears AGC the same way quadrf-jtag --rx gain= */
    if (jtag_write(d, 0x6A, (uint16_t)gain) != 0) return -1;
    d->last_gain = gain;
    return 0;
}

int csi_dev_get_gain(csi_dev_t *d)
{
    uint16_t v;
    if (jtag_read(d, 0x6A, &v) != 0) return -1;
    if (v & 0x0080) return d->last_gain >= 0 ? d->last_gain : 0;
    if ((v & 0x7F) > 63) return 63;
    return (int)(v & 0x3F);
}

int csi_dev_probe_analog_gain(csi_dev_t *d)
{
    uint16_t r1 = 0;
    if (max2851_read_main(d, 1, &r1) != 0) {
        d->analog_lna_db = -1;
        d->analog_vga_db = -1;
        return -1;
    }
    decode_main1(r1, &d->analog_lna_db, &d->analog_vga_db);
    return 0;
}
