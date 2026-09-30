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
#define HOP_SPUR_LO_MAX  128      /* >= MAX_LO_STEPS */

typedef struct {
    /* Log power, fftshifted. Only [k_min, k_max] is written per hop. */
    float         *vraw;
    dsp_peak_t    *topk;
    fftwf_complex *fin[CHANNELS];
    fftwf_complex *fout[CHANNELS];
    fftwf_plan     plan[CHANNELS];
    int            k_min, k_max;   /* digital keep-band, +-LO_STEP/2 */
    /* Fixed-offset spur mask, learned per worker. spur_cnt[k] counts CFAR
     * hits at baseband bin k over the current window of spur_hops hops;
     * spur_st bit0 = bin over threshold, bit1 = masked (bit0 of k-1..k+1).
     * spur_lo / spur_visits count this worker's hops per LO in the window. */
    uint16_t      *spur_cnt;
    uint8_t       *spur_st;
    int            spur_hops;
    int            spur_plan;      /* plan size the window was counted on */
    float          spur_margin;
    int            spur_nlo;
    double         spur_lo[HOP_SPUR_LO_MAX];
    uint16_t       spur_visits[HOP_SPUR_LO_MAX];
    /* Receiver background: slow per-bin mean of vraw over the sweep,
     * learned per worker. vn = vraw - bg is what CFAR sees. */
    float         *bg;
    float         *vn;
    int            bg_n;           /* hops averaged since the last reset */
    int            bg_gain;
} hop_ctx_t;

/* Per-hit rejection gates, runtime so the UI can move them. 0 disables. */
typedef struct {
    /* Max |phi0 - phi1 + phi2 - phi3| (rad) on the raw pairs. The array is
     * a parallelogram (p0 + p2 = p1 + p3), so a far-field plane wave closes
     * to 0 whatever its direction. */
    float closure_max;
    /* Max strongest/second-strongest channel power at the bin (linear).
     * A real emitter lands on all four patches; a spur born in one RX or
     * ADC does not. */
    float balance_max;
    /* LOs in the sweep plan. An RF emitter lands at a different baseband
     * bin on each LO, a spur from the board lands on the same one every
     * hop; both learners below need >= 6 LOs to tell them apart. */
    int   plan_los;
    int   spur_mask;
    /* Receiver background normalization; gain is the 0..63 setting the
     * background was learned at (a change restarts it). */
    int   bg_norm;
    int   gain;
    /* CFAR threshold over the local mean of ln(1 + sum |X|^2); 1 dB is
     * ln(10)/10 = 0.2303. 0 = CFAR_THRESH. */
    float cfar_thresh;
    /* Spur mask trip point in visits of this worker's busiest LO; a real
     * emitter's bin fires at most once per visit. 0 = SPUR_PER_VISIT. */
    float spur_margin;
} hop_gates_t;

typedef struct {
    int      ran;         /* 0: ADC starved, no FFT, no points */
    int      adc_peak;
    double   adc_sumsq;
    uint32_t npts;
    float    vmax_hop;    /* largest log power among emitted points */
    uint32_t hits;        /* CFAR hits inside the keep bands */
    uint32_t rej_balance;
    uint32_t rej_closure;
    uint32_t rej_spur;
} hop_out_t;

typedef int (*hop_keep_fn)(double rf_mhz, const void *arg);

/* Allocates buffers and FFTW plans. Not thread-safe (FFTW planning). */
int  hop_init(hop_ctx_t *h);
void hop_free(hop_ctx_t *h);

/* blk is BLOCK_BYTES of CS8. Points whose RF fails keep() are dropped
 * before the DOA solve. intensity = log power / vmax, clamped to 0..1.
 * pts needs room for topk points. gates may be NULL (all off). */
void hop_run(hop_ctx_t *h, const int8_t *blk, double lo, float vmax, int topk,
             hop_keep_fn keep, const void *keep_arg, const hop_gates_t *gates,
             pg_point_t *pts, hop_out_t *out);

#endif /* HOP_H */
