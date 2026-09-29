// pg_ringcopy.c
// Time the 64 KiB settled-half copy out of the mmapped CSI ring against the
// same copy from ordinary heap memory. Stop quadrf-phasegaze first.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "csi_dev.h"

#define BLOCK_BYTES (8192 * 8)
#define N 2000

static inline uint64_t ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

int main(void)
{
    csi_dev_t d;
    if (csi_dev_open(&d, "/dev/csi_stream0") != 0) return 1;
    uint8_t *dst = aligned_alloc(64, BLOCK_BYTES);
    uint8_t *heap = aligned_alloc(64, BLOCK_BYTES * 2);
    memset(heap, 1, BLOCK_BYTES * 2);

    uint64_t ring_ns = 0, heap_ns = 0, ring_min = ~0ull;
    unsigned sum = 0;
    for (int i = 0; i < N; ++i) {
        uint32_t head, tail;
        csi_dev_wait(&d, 20);
        csi_dev_ring_pos(&d, &head, &tail);
        uint32_t start = (tail + d.span_bytes - BLOCK_BYTES) % (uint32_t)d.ring_size;
        if ((uint64_t)start + BLOCK_BYTES > d.ring_size) start = 0;
        uint64_t a = ns();
        memcpy(dst, (const uint8_t *)d.ring + start, BLOCK_BYTES);
        __asm__ volatile("" : : "r"(dst) : "memory");
        uint64_t b = ns();
        ring_ns += b - a;
        if (b - a < ring_min) ring_min = b - a;
        sum += dst[i & 1023];
        csi_dev_consume(&d, d.span_bytes);
        a = ns();
        memcpy(dst, heap + (i & 1) * BLOCK_BYTES, BLOCK_BYTES);
        __asm__ volatile("" : : "r"(dst) : "memory");
        heap_ns += ns() - a;
        sum += dst[i & 1023];
    }
    printf("64 KiB copy: ring mean %.1f us (min %.1f), heap %.1f us\n",
           ring_ns / 1e3 / N, ring_min / 1e3, heap_ns / 1e3 / N);
    csi_dev_close(&d);
    return sum == 0x7f;
}
