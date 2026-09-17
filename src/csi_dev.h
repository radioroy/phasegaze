// csi_dev.h
// Thin abstraction over the QuadRF CSI stream device: ring buffer reads,
// MAX2851 LO programming and RX gain via the JTAG register bridge.
// Kept deliberately small so lighter tools can reuse it as-is.

#ifndef CSI_DEV_H
#define CSI_DEV_H

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>

typedef struct {
    int      fd;
    void    *ring;
    uint64_t ring_size;
    uint32_t span_bytes;
    size_t   map_len;
    uint16_t last_lna_word; /* Main2 word last sent; skip repeats */
    int      have_lna_word;
    double   last_lo_mhz;   /* skip JTAG if set_lo is the same hop */
    int      last_gain;     /* FPGA 0x6A total dB, 0..63; restrobed after Main2 */
    int      analog_lna_db; /* last Main1 decode, or -1 */
    int      analog_vga_db;
    pthread_mutex_t jtag_mtx;
} csi_dev_t;

/* Open device, JTAG setup, analog RX bring-up (all 4 antennas, MODE=RX,
 * AGC off, 4ch interleave, 20 MHz digital BW, RHCP), mmap the ring. */
int  csi_dev_open(csi_dev_t *d, const char *path);
void csi_dev_close(csi_dev_t *d);

/* Blocking read of exactly one block (spin then sleep). Drops backlog beyond
 * max_queued_blocks to bound latency. */
void csi_dev_read_block(csi_dev_t *d, uint8_t *dst, uint32_t block_bytes,
                        uint32_t bytes_per_frame, int max_queued_blocks);

/* Read settled steady-state block (Block 1) from the current 128 KiB DMA span
 * and consume the entire span, keeping the ring empty and aligned for the
 * next pipelined LO retune. */
int  csi_dev_read_settled_block(csi_dev_t *d, uint8_t *dst, uint32_t block_bytes);

/* Discard complete spans currently in the ring. */
void csi_dev_flush(csi_dev_t *d, uint32_t align_bytes);

/* Program the synthesizer to freq_mhz (batched, ~atomic). */
int  csi_dev_set_lo(csi_dev_t *d, double freq_mhz);

/* Manual RX gain in dB. FPGA 0x6A is the total-gain word the fabric splits
 * into LNA+VGA+digital. Same 0..63 range as quadrf-jtag / the web UI. */
int  csi_dev_set_gain(csi_dev_t *d, int gain);
int  csi_dev_get_gain(csi_dev_t *d);
/* SPI-read MAX2851 Main1 (LNA+VGA). Invasive; call after user gain changes. */
int  csi_dev_probe_analog_gain(csi_dev_t *d);

#endif /* CSI_DEV_H */
