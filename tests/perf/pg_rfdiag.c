// pg_rfdiag.c
// Per-LO RF health capture. Tunes through csi_dev_set_lo (the phasegaze hop
// path, including the seed file), records raw settled blocks, then reads back
// the MAX2851 VCO band and VTUNE zone at that LO. Zones 0 and 7 are outside
// the lock range.
//
// Record: double lo_mhz, int32 gain, int32 band, int32 zone, int32 fresh,
//         uint8 block[65536] (8192 frames x 4 ch x CS8 IQ)
//
// Stop quadrf-phasegaze first.
//   pg_rfdiag OUT.bin fresh(0|1) blocks_per_lo lo0 lo1 step gain [gain ...]
// fresh=1 ignores the seed file and runs VAS at every LO (file untouched).

#include "../../src/csi_dev.c"

#define BLOCK_BYTES  (8192 * 8)
#define SETTLE_SPANS 4

int main(int argc, char **argv)
{
    if (argc < 8) {
        fprintf(stderr, "usage: %s OUT.bin fresh blocks lo0 lo1 step gain...\n",
                argv[0]);
        return 1;
    }
    int fresh = atoi(argv[2]);
    int per_lo = atoi(argv[3]);
    double lo0 = atof(argv[4]), lo1 = atof(argv[5]), step = atof(argv[6]);
    int gains[16], ng = 0;
    for (int i = 7; i < argc && ng < 16; ++i) gains[ng++] = atoi(argv[i]);

    csi_dev_t d;
    if (csi_dev_open(&d, "/dev/csi_stream0") != 0) return 1;
    /* PG_MAIN6=0x3FE etc. overrides MAX2851 Main6 E_RX[5:1] (D4:D0) so one
     * analog RX can be switched off while its ADC keeps running. */
    const char *m6 = getenv("PG_MAIN6");
    if (m6)
        jtag_write(&d, MAX2851_SPI,
                   (uint16_t)((6u << 10) | (strtoul(m6, NULL, 0) & 0x3FFu)));
    if (fresh) {
        g_seed_loaded = 1;
        g_seed_dirty = 0;
    }
    FILE *f = fopen(argv[1], "wb");
    if (!f) { perror(argv[1]); return 1; }

    double lo[256];
    int nlo = 0;
    for (double x = lo0; x <= lo1 + 1e-6 && nlo < 256; x += step) lo[nlo++] = x;
    if (fresh)
        for (int i = 0; i < nlo; ++i) vco_learn_one(&d, lo[i], 1);

    static uint8_t blk[BLOCK_BYTES];
    for (int g = 0; g < ng; ++g) {
        csi_dev_set_gain(&d, gains[g]);
        usleep(2000);
        csi_dev_probe_analog_gain(&d);
        uint16_t v6a = 0;
        jtag_read(&d, 0x6A, &v6a);
        fprintf(stderr, "gain %d: 0x6A=0x%04x lna %d dB vga %d dB\n", gains[g],
                v6a, d.analog_lna_db, d.analog_vga_db);
        for (int i = 0; i < nlo; ++i) {
            csi_dev_set_lo(&d, lo[i]);
            csi_dev_flush(&d, 0);
            for (int s = 0; s < SETTLE_SPANS; ++s)
                csi_dev_read_settled_block(&d, blk, BLOCK_BYTES, 0.0);
            uint8_t band = 0xFF, zone = 0xFF;
            read_vco(&d, &band, &zone);
            fprintf(stderr, "%.1f g%d band %u zone %u\n", lo[i], gains[g],
                    band, zone);
            csi_dev_flush(&d, 0);
            for (int s = 0; s < 2; ++s)
                csi_dev_read_settled_block(&d, blk, BLOCK_BYTES, 0.0);
            for (int b = 0; b < per_lo; ++b) {
                csi_dev_read_settled_block(&d, blk, BLOCK_BYTES, 0.0);
                int32_t hdr[4] = { gains[g], band, zone, fresh };
                fwrite(&lo[i], sizeof(double), 1, f);
                fwrite(hdr, sizeof(hdr), 1, f);
                fwrite(blk, 1, BLOCK_BYTES, f);
            }
        }
    }
    fclose(f);
    csi_dev_close(&d);
    g_seed_dirty = 0;
    return 0;
}
