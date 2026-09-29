// hop.h
// One hop of the DSP pipeline: a settled CSI block at one LO -> keep-band
// log power and direction-cosine points. Run by the worker threads and by
// tests/perf/pg_bench against the same captured blocks.

#ifndef HOP_H
#define HOP_H

#include <stdint.h>
#include <fftw3.h>

#include "dsp.h"
#include "pg_stream.h"

// ---------------------------------------------------------------------------
// 4-lane CSI, nominally "38 Msps". Measured 37.3726 Msps: the 16384-sample
// span period is 438.39 us, and a 10 MHz LO step moves an OFDM signal by
// 2192.15 bins of 8192 (38 Msps would be 2155.8). FFT 8192 gives ~4.6 kHz
// bins. LO step is the digital-filter BW.
// ---------------------------------------------------------------------------

#define FFT_SIZE         8192
#define CHANNELS         4
#define BYTES_PER_IQ     2
#define BYTES_PER_FRAME  (CHANNELS * BYTES_PER_IQ)
#define BLOCK_BYTES      (FFT_SIZE * BYTES_PER_FRAME)
#define LO_STEP_MHZ      20.0
#define FS_MHZ           37.3726
#define TOPK_BASE        512

typedef struct {
    /* Log power, fftshifted. Only [k_min, k_max] is written per hop. */
    float         *vraw;
    dsp_peak_t    *topk;
    fftwf_complex *fin[CHANNELS];
    fftwf_complex *fout[CHANNELS];
    fftwf_plan     plan[CHANNELS];
    int            k_min, k_max;   /* digital keep-band, +-LO_STEP/2 */
} hop_ctx_t;

typedef struct {
    int      ran;         /* 0: ADC starved, no FFT, no points */
    int      adc_peak;
    double   adc_sumsq;
    uint32_t npts;
    float    vmax_hop;    /* largest log power among emitted points */
} hop_out_t;

typedef int (*hop_keep_fn)(double rf_mhz, const void *arg);

/* Allocates buffers and FFTW plans. Not thread-safe (FFTW planning). */
int  hop_init(hop_ctx_t *h);
void hop_free(hop_ctx_t *h);

/* blk is BLOCK_BYTES of CS8. Points whose RF fails keep() are dropped
 * before the DOA solve. intensity = log power / vmax, clamped to 0..1.
 * pts needs room for topk points. */
void hop_run(hop_ctx_t *h, const int8_t *blk, double lo, float vmax, int topk,
             hop_keep_fn keep, const void *keep_arg,
             pg_point_t *pts, hop_out_t *out);

#endif /* HOP_H */
