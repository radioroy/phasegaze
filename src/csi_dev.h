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

/* Read the settled tail of one 128 KiB DMA span and consume the span.
 * If tune_mhz > 0, program that LO after the span is observed and before
 * the copy, so PLL lock overlaps the copy instead of the next dwell. */
int  csi_dev_read_settled_block(csi_dev_t *d, uint8_t *dst, uint32_t block_bytes,
                                double tune_mhz);

/* Learn VCO sub-bands this plan does not already have. Cache and seed-file
 * hits do not run a search. Call when the hop list changes, not per hop. */
int  csi_dev_prime_los(csi_dev_t *d, const double *mhz, int n);

/* Discard everything queued; the tail lands on the producer's span grid.
 * align_bytes is ignored (kept for callers). */
void csi_dev_flush(csi_dev_t *d, uint32_t align_bytes);

/* Raw ring offsets (head = producer). Either pointer may be NULL. */
int  csi_dev_ring_pos(csi_dev_t *d, uint32_t *head, uint32_t *tail);
int  csi_dev_consume(csi_dev_t *d, uint32_t n);
/* Block until the ring is non-empty (driver wakes on every span publish). */
int  csi_dev_wait(csi_dev_t *d, int timeout_ms);

/* 1 if set_lo(freq_mhz) will also rewrite Main2 (LNA band) and restrobe
 * gain. That write is longer and its transition reaches the ADC earlier. */
int  csi_dev_lo_switches_band(const csi_dev_t *d, double freq_mhz);

/* Program the synthesizer to freq_mhz (batched, ~atomic). */
int  csi_dev_set_lo(csi_dev_t *d, double freq_mhz);

/* Manual RX gain in dB. FPGA 0x6A is the total-gain word the fabric splits
 * into LNA+VGA+digital. Same 0..63 range as quadrf-jtag / the web UI. */
int  csi_dev_set_gain(csi_dev_t *d, int gain);
int  csi_dev_get_gain(csi_dev_t *d);
/* SPI-read MAX2851 Main1 (LNA+VGA). Invasive; call after user gain changes. */
int  csi_dev_probe_analog_gain(csi_dev_t *d);

/* Walk the LO down and up. *lo_mhz and *hi_mhz are the last frequencies
 * where the lock pin is high in both directions. The tuner must be stopped;
 * this holds the synthesizer for several seconds. Parks at 5500 MHz after. */
int  csi_dev_measure_lock(csi_dev_t *d, int *lo_mhz, int *hi_mhz);

#endif /* CSI_DEV_H */
