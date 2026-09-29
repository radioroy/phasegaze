// pg_capture.c
// Dump settled CSI blocks for offline DSP comparison. Each record is
//   double lo_mhz, int32 gain, int32 reserved, uint8 block[65536]
// where block is the second half of one 128 KiB DMA span (8192 frames of
// 4 ch x CS8 IQ), the same bytes the phasegaze workers copy per hop.
//
// Stop quadrf-phasegaze first; this opens /dev/csi_stream0 directly.
//   pg_capture OUT.bin [blocks_per_lo] [gain ...]

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "csi_dev.h"

#define BLOCK_BYTES  (8192 * 8)
#define SETTLE_SPANS 4

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s OUT.bin [blocks_per_lo] [gain ...]\n", argv[0]);
        return 1;
    }
    int per_lo = argc > 2 ? atoi(argv[2]) : 4;
    int gains[16], ng = 0;
    for (int i = 3; i < argc && ng < 16; ++i) gains[ng++] = atoi(argv[i]);
    if (!ng) gains[ng++] = 45;

    csi_dev_t d;
    if (csi_dev_open(&d, "/dev/csi_stream0") != 0) return 1;
    FILE *f = fopen(argv[1], "wb");
    if (!f) { perror(argv[1]); return 1; }

    double lo[64];
    int nlo = 0;
    for (double x = 4910.0; x <= 6090.0 + 1e-6; x += 20.0) lo[nlo++] = x;
    csi_dev_prime_los(&d, lo, nlo);

    static uint8_t blk[BLOCK_BYTES];
    long n = 0;
    for (int g = 0; g < ng; ++g) {
        csi_dev_set_gain(&d, gains[g]);
        for (int i = 0; i < nlo; ++i) {
            csi_dev_set_lo(&d, lo[i]);
            csi_dev_flush(&d, 0);
            for (int s = 0; s < SETTLE_SPANS; ++s)
                csi_dev_read_settled_block(&d, blk, BLOCK_BYTES, 0.0);
            for (int b = 0; b < per_lo; ++b) {
                csi_dev_read_settled_block(&d, blk, BLOCK_BYTES, 0.0);
                int32_t hdr[2] = { gains[g], 0 };
                fwrite(&lo[i], sizeof(double), 1, f);
                fwrite(hdr, sizeof(hdr), 1, f);
                fwrite(blk, 1, BLOCK_BYTES, f);
                n++;
            }
        }
        fprintf(stderr, "gain %d done\n", gains[g]);
    }
    fclose(f);
    csi_dev_close(&d);
    fprintf(stderr, "%ld blocks\n", n);
    return 0;
}
