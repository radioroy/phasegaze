// dsp.h
// NEON-optimized DSP primitives for the sweep pipeline:
//   CS8 deinterleave + per-antenna delay-cal rotation -> complex float
//   4-channel log power spectrum (fftshifted)
//   CA-CFAR top-K detection
//   3-baseline phase-gradient DOA solve with 2*pi ambiguity search

#ifndef DSP_H
#define DSP_H

#include <stdint.h>

/* Extract channel `ch` from an interleaved CS8 block (4ch x IQ per frame),
 * scale to [-1,1] and rotate by the constant delay-cal phasor (rc, rs).
 * dst is interleaved complex float (fftwf layout). n = FFT size. */
void dsp_cs8_extract_rotate(const int8_t *src, int ch,
                            float *dst, int n, float rc, float rs);

/* Split an interleaved CS8 block into four complex float buffers in one
 * read, dst[c] = src channel c * scale[c]. Also returns the largest |byte|
 * and the sum of squares over all 8 rails (same values as a plain byte
 * loop). n = frames = FFT size. */
void dsp_cs8_split4(const int8_t *src, float *const dst[4], int n,
                    const float scale[4], int *peak, int32_t *sumsq);

/* Sum |X|^2 over 4 channels bin-by-bin, fftshift, and take log(1+p).
 * out[k] for k in [0, n) maps bin k to fft index (k + n/2) % n.
 * Center bins [n/2 - dc_guard, n/2 + dc_guard] are forced to 0 (DC block). */
void dsp_power4_log_shifted(float *const ch_out[4], float *out, int n, int dc_guard);

/* Same, but only out[k0..k1] (widened to 4-bin groups). Bins outside are
 * left untouched. n must be a multiple of 8. */
void dsp_power4_log_shifted_range(float *const ch_out[4], float *out, int n,
                                  int dc_guard, int k0, int k1);

/* Min-heap of the top-K values. */
typedef struct { float v; int k; } dsp_peak_t;

/* CA-CFAR over v[k0..k1] inclusive (passband). Window needs k1-k0 >= 2*win.
 * Returns number of items in heap (capacity cap). */
int dsp_cfar_topk(const float *v, int k0, int k1, int win, int guard, float thresh,
                  dsp_peak_t *heap, int cap);

/* Solve the phase gradient (gx, gy) from the three folded baseline phases,
 * searching the 2*pi ambiguity lattice. Returns the residual cost; large
 * values mean the three baselines are not a plane wave (noise / wrapping).
 * When the cost is above max_cost the search may stop early, returning a
 * value that is still > max_cost and leaving gx, gy unspecified. At or
 * below max_cost the result equals the exhaustive search. */
float dsp_solve_gradient(float phi10, float phi20, float phi30, float max_cost,
                         float *gx, float *gy);

#endif /* DSP_H */
