// dsp.c

#include "dsp.h"

#include <math.h>
#include <string.h>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

// ---------------------------------------------------------------------------
// CS8 extract + rotate
// ---------------------------------------------------------------------------
//
// Block layout per sample frame (8 bytes): I0 Q0 I1 Q1 I2 Q2 I3 Q3.
// Treating each IQ pair as one little-endian s16 lets vld4q_s16 deinterleave
// the four channels in a single pass; reinterpreting the selected channel
// back to s8 recovers the interleaved I,Q byte stream for that channel.

void dsp_cs8_extract_rotate(const int8_t *src, int ch,
                            float *dst, int n, float rc, float rs)
{
    const float inv127 = 1.0f / 127.0f;
    int i = 0;

#if defined(__aarch64__)
    const int16_t *s16 = (const int16_t *)src;
    const float32x4_t vc = vdupq_n_f32(rc * inv127);
    /* Rotation on interleaved [I Q I Q]: out = v*c + rev64(v) * {s,-s,s,-s} */
    const float sq = rs * inv127;
    const float sgn_arr[4] = { sq, -sq, sq, -sq };
    const float32x4_t vs = vld1q_f32(sgn_arr);

    for (; i + 8 <= n; i += 8) {
        int16x8x4_t q = vld4q_s16(s16 + (size_t)i * 4);
        int8x16_t iq8 = vreinterpretq_s8_s16(q.val[ch]);

        int16x8_t lo = vmovl_s8(vget_low_s8(iq8));
        int16x8_t hi = vmovl_s8(vget_high_s8(iq8));

        float32x4_t f0 = vcvtq_f32_s32(vmovl_s16(vget_low_s16(lo)));
        float32x4_t f1 = vcvtq_f32_s32(vmovl_s16(vget_high_s16(lo)));
        float32x4_t f2 = vcvtq_f32_s32(vmovl_s16(vget_low_s16(hi)));
        float32x4_t f3 = vcvtq_f32_s32(vmovl_s16(vget_high_s16(hi)));

        float32x4_t o0 = vfmaq_f32(vmulq_f32(f0, vc), vrev64q_f32(f0), vs);
        float32x4_t o1 = vfmaq_f32(vmulq_f32(f1, vc), vrev64q_f32(f1), vs);
        float32x4_t o2 = vfmaq_f32(vmulq_f32(f2, vc), vrev64q_f32(f2), vs);
        float32x4_t o3 = vfmaq_f32(vmulq_f32(f3, vc), vrev64q_f32(f3), vs);

        float *d = dst + (size_t)i * 2;
        vst1q_f32(d,      o0);
        vst1q_f32(d + 4,  o1);
        vst1q_f32(d + 8,  o2);
        vst1q_f32(d + 12, o3);
    }
#endif

    for (; i < n; ++i) {
        size_t base = (size_t)i * 8 + (size_t)ch * 2;
        float re = (float)src[base + 0] * inv127;
        float im = (float)src[base + 1] * inv127;
        dst[i * 2 + 0] = re * rc + im * rs;
        dst[i * 2 + 1] = im * rc - re * rs;
    }
}

// ---------------------------------------------------------------------------
// 4-channel log power spectrum
// ---------------------------------------------------------------------------

#if defined(__aarch64__)
/* Fast log(x) for x >= 1: exponent extraction, then the atanh series on the
 * mantissa: log(m) = 2y(1 + y^2/3 + y^4/5 + y^6/7), y = (m-1)/(m+1).
 * y <= 1/3 so truncation error is ~1e-5, well below CFAR sensitivity. */
static inline float32x4_t vlogq_fast(float32x4_t x)
{
    const float32x4_t ln2 = vdupq_n_f32(0.69314718f);
    const float32x4_t one = vdupq_n_f32(1.0f);
    int32x4_t xi = vreinterpretq_s32_f32(x);
    int32x4_t e = vsubq_s32(vshrq_n_s32(xi, 23), vdupq_n_s32(127));
    /* mantissa in [1, 2) */
    int32x4_t mi = vorrq_s32(vandq_s32(xi, vdupq_n_s32(0x007FFFFF)),
                             vdupq_n_s32(0x3F800000));
    float32x4_t m = vreinterpretq_f32_s32(mi);
    float32x4_t y = vdivq_f32(vsubq_f32(m, one), vaddq_f32(m, one));
    float32x4_t y2 = vmulq_f32(y, y);
    float32x4_t p = vdupq_n_f32(2.0f / 7.0f);
    p = vfmaq_f32(vdupq_n_f32(2.0f / 5.0f), p, y2);
    p = vfmaq_f32(vdupq_n_f32(2.0f / 3.0f), p, y2);
    p = vfmaq_f32(vdupq_n_f32(2.0f), p, y2);
    float32x4_t logm = vmulq_f32(p, y);
    return vfmaq_f32(logm, vcvtq_f32_s32(e), ln2);
}

static void power4_log_range(float *const ch[4], int fft_off, float *out, int cnt)
{
    int i = 0;
    const float32x4_t one = vdupq_n_f32(1.0f);
    for (; i + 4 <= cnt; i += 4) {
        float32x4_t acc = vdupq_n_f32(0.0f);
        for (int c = 0; c < 4; ++c) {
            const float *p = ch[c] + (size_t)(fft_off + i) * 2;
            float32x4x2_t z = vld2q_f32(p);   /* z.val[0]=re, z.val[1]=im */
            acc = vfmaq_f32(acc, z.val[0], z.val[0]);
            acc = vfmaq_f32(acc, z.val[1], z.val[1]);
        }
        vst1q_f32(out + i, vlogq_fast(vaddq_f32(acc, one)));
    }
    for (; i < cnt; ++i) {
        float s = 0.0f;
        for (int c = 0; c < 4; ++c) {
            float re = ch[c][(size_t)(fft_off + i) * 2];
            float im = ch[c][(size_t)(fft_off + i) * 2 + 1];
            s += re * re + im * im;
        }
        out[i] = logf(1.0f + s);
    }
}
#else
static void power4_log_range(float *const ch[4], int fft_off, float *out, int cnt)
{
    for (int i = 0; i < cnt; ++i) {
        float s = 0.0f;
        for (int c = 0; c < 4; ++c) {
            float re = ch[c][(size_t)(fft_off + i) * 2];
            float im = ch[c][(size_t)(fft_off + i) * 2 + 1];
            s += re * re + im * im;
        }
        out[i] = logf(1.0f + s);
    }
}
#endif

void dsp_power4_log_shifted(float *const ch_out[4], float *out, int n, int dc_guard)
{
    const int half = n / 2;
    /* out[k] = log power of fft index (k + half) % n:
     *   out[0 .. half)  <- fft[half .. n)
     *   out[half .. n)  <- fft[0 .. half)                                  */
    power4_log_range(ch_out, half, out, half);
    power4_log_range(ch_out, 0, out + half, half);

    for (int k = half - dc_guard; k <= half + dc_guard; ++k)
        if (k >= 0 && k < n) out[k] = 0.0f;
}

// ---------------------------------------------------------------------------
// CA-CFAR + top-K heap
// ---------------------------------------------------------------------------

static inline void heap_swap(dsp_peak_t *a, dsp_peak_t *b)
{
    dsp_peak_t t = *a; *a = *b; *b = t;
}

static inline void sift_up(dsp_peak_t *h, int i)
{
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (h[p].v <= h[i].v) break;
        heap_swap(&h[p], &h[i]);
        i = p;
    }
}

static inline void sift_down(dsp_peak_t *h, int n, int i)
{
    for (;;) {
        int l = i * 2 + 1, r = l + 1, m = i;
        if (l < n && h[l].v < h[m].v) m = l;
        if (r < n && h[r].v < h[m].v) m = r;
        if (m == i) break;
        heap_swap(&h[m], &h[i]);
        i = m;
    }
}

static inline int heap_push(dsp_peak_t *h, int size, int cap, float v, int k)
{
    if (size < cap) {
        h[size].v = v; h[size].k = k;
        sift_up(h, size);
        return size + 1;
    }
    if (v <= h[0].v) return size;
    h[0].v = v; h[0].k = k;
    sift_down(h, cap, 0);
    return size;
}

int dsp_cfar_topk(const float *v, int k0, int k1, int win, int guard, float thresh,
                  dsp_peak_t *heap, int cap)
{
    const int noise_cells = (win - guard) * 2;
    int hsz = 0;
    int k_lo = k0 + win;
    int k_hi = k1 - win;
    if (k_lo > k_hi) return 0;

    float window_sum = 0.0f;
    for (int k = k0; k < k0 + win * 2 + 1; ++k) window_sum += v[k];

    for (int k = k_lo; k <= k_hi; ++k) {
        if (k > k_lo)
            window_sum += v[k + win] - v[k - win - 1];

        float guard_sum = 0.0f;
        for (int g = -guard; g <= guard; ++g) guard_sum += v[k + g];

        float noise_floor = (window_sum - guard_sum) / (float)noise_cells;
        if (v[k] > noise_floor + thresh)
            hsz = heap_push(heap, hsz, cap, v[k], k);
    }
    return hsz;
}

// ---------------------------------------------------------------------------
// Phase-gradient DOA solve
// ---------------------------------------------------------------------------

float dsp_solve_gradient(float phi10, float phi20, float phi30,
                         float *gx_out, float *gy_out)
{
    const float inv_sqrt3 = 0.5773502691896258f;
    const float reg = 1e-3f;
    const float TWO_PI = 6.283185307179586f;

    float best_cost = 1e30f, best_gx = 0.0f, best_gy = 0.0f;

    for (int n10 = -2; n10 <= 2; ++n10)
    for (int n20 = -2; n20 <= 2; ++n20)
    for (int n30 = -2; n30 <= 2; ++n30) {
        float y1 = phi10 + TWO_PI * (float)n10;
        float y2 = phi20 + TWO_PI * (float)n20;
        float y3 = phi30 + TWO_PI * (float)n30;

        float gx = (y3 - y1) * inv_sqrt3;
        float gy = (y1 + 2.0f * y2 + y3) * (1.0f / 3.0f);

        float p1 = -0.8660254037844386f * gx + 0.5f * gy;
        float p2 = gy;
        float p3 =  0.8660254037844386f * gx + 0.5f * gy;

        float r1 = y1 - p1, r2 = y2 - p2, r3 = y3 - p3;
        float cost = r1 * r1 + r2 * r2 + r3 * r3 + reg * (gx * gx + gy * gy);

        if (cost < best_cost) {
            best_cost = cost;
            best_gx = gx;
            best_gy = gy;
        }
    }
    *gx_out = best_gx;
    *gy_out = best_gy;
    return best_cost;
}
