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

/* Sum |X|^2 over 4 channels bin-by-bin, fftshift, and take log(1+p).
 * out[k] for k in [0, n) maps bin k to fft index (k + n/2) % n.
 * Center bins [n/2 - dc_guard, n/2 + dc_guard] are forced to 0 (DC block). */
void dsp_power4_log_shifted(float *const ch_out[4], float *out, int n, int dc_guard);

/* Min-heap of the top-K values. */
typedef struct { float v; int k; } dsp_peak_t;

/* CA-CFAR over v[k0..k1] inclusive (passband). Window needs k1-k0 >= 2*win.
 * Returns number of items in heap (capacity cap). */
int dsp_cfar_topk(const float *v, int k0, int k1, int win, int guard, float thresh,
                  dsp_peak_t *heap, int cap);

/* Solve the phase gradient (gx, gy) from the three folded baseline phases,
 * searching the 2*pi ambiguity lattice. Returns the residual cost; large
 * values mean the three baselines are not a plane wave (noise / wrapping). */
float dsp_solve_gradient(float phi10, float phi20, float phi30,
                         float *gx, float *gy);

#endif /* DSP_H */
