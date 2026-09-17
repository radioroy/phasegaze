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
    if (d->fd >= 0) close(d->fd);
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

void csi_dev_flush(csi_dev_t *d, uint32_t align_bytes)
{
    struct csi_ring_info ri;
    if (ioctl(d->fd, CSI_IOC_GET_RING_INFO, &ri) != 0)
        return;
    uint64_t rsz = ri.ring_size ? (uint64_t)ri.ring_size : d->ring_size;
    uint32_t used = ring_used(ri.head, ri.tail, rsz);
    if (align_bytes > 0)
        used -= used % align_bytes;
    if (used)
        (void)consume(d->fd, used);
}

int csi_dev_read_settled_block(csi_dev_t *d, uint8_t *dst, uint32_t block_bytes)
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

int csi_dev_set_lo(csi_dev_t *d, double freq_mhz)
{
    if (d->last_lo_mhz > 1.0 && fabs(freq_mhz - d->last_lo_mhz) < 1e-6)
        return 0;
    double ratio = freq_mhz / 80.0;
    int idiv = (int)floor(ratio);
    int fdiv = (int)llround((ratio - idiv) * (double)(1u << 20));
    uint16_t w2 = (uint16_t)((2u << 10) | (lna_band_reg2(freq_mhz) & 0x3FF));
    int send_lna = !d->have_lna_word || d->last_lna_word != w2;

    struct csi_jtag_reg regs[4] = {
        { .addr = 0x43, .value = (uint16_t)((15u << 10) | (1u << 9) | (idiv & 0x7f)) },
        { .addr = 0x43, .value = (uint16_t)((16u << 10) | ((fdiv >> 10) & 0x3ff)) },
        { .addr = 0x43, .value = (uint16_t)((17u << 10) | (fdiv & 0x3ff)) },
        { .addr = 0x43, .value = w2 },
    };
    uint32_t n = send_lna ? 4u : 3u;
    if (jtag_batch(d, regs, n) != 0)
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
