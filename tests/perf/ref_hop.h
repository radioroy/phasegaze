// ref_hop.h

#ifndef REF_HOP_H
#define REF_HOP_H

#include <stdint.h>
#include "pg_stream.h"

typedef struct ref_ctx ref_ctx_t;

typedef struct {
    int      ran;          /* 0: starved block, FFT skipped */
    int      adc_peak;
    double   adc_sumsq;
    int      hits;         /* CFAR top-K survivors fed to the DOA loop */
    uint32_t npts;
    float    vmax_hop;
    int      k_min, k_max;
} ref_out_t;

typedef struct {
    uint64_t adc, extract, fft, power, cfar, doa;
} ref_times_t;

ref_ctx_t   *ref_init(void);
const float *ref_vraw(const ref_ctx_t *w);
int   ref_hop(ref_ctx_t *w, const uint8_t *blk, double lo, float vmax,
              int active_topk, pg_point_t *pts, ref_out_t *o, ref_times_t *t);
int   ref_cfar(const float *v, int k0, int k1, int cap);
float ref_solve(float p10, float p20, float p30, float *gx, float *gy);

#endif /* REF_HOP_H */
