// wifi.c — in-process 802.11a/g OFDM receiver for PhaseGaze.
//
// Wi-Fi mode parks the sweep, locks the LO on a 20 MHz channel and opens
// /dev/csi_stream0 here. The fabric sums the four antennas (0x25=0) at the
// 149.500743 MS/s CSI line rate; a cubic Farrow drops that to 20 MS/s:
//
//   producer:   CSI ring -> Farrow 149.5 -> 20 MS/s -> sample ring (2^20)
//   detector:   L-STF delay-16 autocorrelation plateau, coarse CFO, L-SIG
//               length, then hands the whole frame to a worker
//   2 workers:  L-LTF matched filter + fine CFO, 64-pt FFT, pilot CPE and
//               residual SFO tracking, LS (STA retry) equalizer, soft demap,
//               deinterleave, depuncture, K=7 Viterbi, descramble, FCS
//
// Sync, equalizer and L-SIG handling are ported from gr-ieee802-11
// (Bastian Bloessl, https://github.com/bastibl/gr-ieee802-11, GPL-3.0-or-later).
// Changes from it: the Viterbi is soft decision with |H|^2 weighting, a
// frame that fails FCS with LS is retried with STA, and HT/VHT (QBPSK after
// L-SIG) and HE (RL-SIG repeat) frames are dropped after L-SIG instead of
// decoded.

#define _GNU_SOURCE
#include "wifi.h"

#include <complex.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <fftw3.h>

#include "csi_dev.h"
#include "dsp.h"
#include "pg_stream.h"
#include "server.h"

typedef float complex cf;

typedef struct {
    int ch;
    int mhz;
    const char *band;
} wifi_ch_info_t;

static const wifi_ch_info_t k_wifi_channels[] = {
    // 5 GHz U-NII-1
    { 36, 5180, "5 GHz" }, { 40, 5200, "5 GHz" }, { 44, 5220, "5 GHz" }, { 48, 5240, "5 GHz" },
    // 5 GHz U-NII-2A
    { 52, 5260, "5 GHz" }, { 56, 5280, "5 GHz" }, { 60, 5300, "5 GHz" }, { 64, 5320, "5 GHz" },
    // 5 GHz U-NII-2C (DFS)
    { 100, 5500, "5 GHz" }, { 104, 5520, "5 GHz" }, { 108, 5540, "5 GHz" }, { 112, 5560, "5 GHz" },
    { 116, 5580, "5 GHz" }, { 120, 5600, "5 GHz" }, { 124, 5620, "5 GHz" }, { 128, 5640, "5 GHz" },
    { 132, 5660, "5 GHz" }, { 136, 5680, "5 GHz" }, { 140, 5700, "5 GHz" }, { 144, 5720, "5 GHz" },
    // 5 GHz U-NII-3
    { 149, 5745, "5 GHz" }, { 153, 5765, "5 GHz" }, { 157, 5785, "5 GHz" }, { 161, 5805, "5 GHz" },
    { 165, 5825, "5 GHz" }, { 169, 5845, "5 GHz" }, { 173, 5865, "5 GHz" }, { 177, 5885, "5 GHz" },
    // 6 GHz U-NII-5 to 8 (Standard 20 MHz channels within 4480..6740 MHz)
    { 1, 5955, "6 GHz" }, { 5, 5975, "6 GHz" }, { 9, 5995, "6 GHz" }, { 13, 6015, "6 GHz" },
    { 17, 6035, "6 GHz" }, { 21, 6055, "6 GHz" }, { 25, 6075, "6 GHz" }, { 29, 6095, "6 GHz" },
    { 33, 6115, "6 GHz" }, { 37, 6135, "6 GHz" }, { 41, 6155, "6 GHz" }, { 45, 6175, "6 GHz" },
    { 49, 6195, "6 GHz" }, { 53, 6215, "6 GHz" }, { 57, 6235, "6 GHz" }, { 61, 6255, "6 GHz" },
    { 65, 6275, "6 GHz" }, { 69, 6295, "6 GHz" }, { 73, 6315, "6 GHz" }, { 77, 6335, "6 GHz" },
    { 81, 6355, "6 GHz" }, { 85, 6375, "6 GHz" }, { 89, 6395, "6 GHz" }, { 93, 6415, "6 GHz" },
    { 97, 6435, "6 GHz" }, { 101, 6455, "6 GHz" }, { 105, 6475, "6 GHz" }, { 109, 6495, "6 GHz" },
    { 113, 6515, "6 GHz" }, { 117, 6535, "6 GHz" }, { 121, 6555, "6 GHz" }, { 125, 6575, "6 GHz" },
    { 129, 6595, "6 GHz" }, { 133, 6615, "6 GHz" }, { 137, 6635, "6 GHz" }, { 141, 6655, "6 GHz" },
    { 145, 6675, "6 GHz" }, { 149, 6695, "6 GHz" }, { 153, 6715, "6 GHz" }, { 157, 6735, "6 GHz" },
};

#define NUM_WIFI_CHANNELS (sizeof(k_wifi_channels) / sizeof(k_wifi_channels[0]))

double wifi_snap_mhz(double mhz, int *out_ch, const char **out_band)
{
    int best_idx = -1;
    double min_diff = 1e9;

    for (size_t i = 0; i < NUM_WIFI_CHANNELS; i++) {
        double d = fabs((double)k_wifi_channels[i].mhz - mhz);
        if (d < min_diff) {
            min_diff = d;
            best_idx = (int)i;
        }
    }

    if (best_idx >= 0 && min_diff <= 10.0) {
        if (out_ch)   *out_ch   = k_wifi_channels[best_idx].ch;
        if (out_band) *out_band = k_wifi_channels[best_idx].band;
        return (double)k_wifi_channels[best_idx].mhz;
    }

    if (out_ch)   *out_ch   = 0;
    if (out_band) *out_band = "Custom";
    return mhz;
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define DEVICE_PATH  "/dev/csi_stream0"
#define CSI_RATE     149500743.01952707   /* (65536/76714) * 175 MHz, summed lane */
#define FS           20e6
#define SRING_BITS   20
#define SRING        (1u << SRING_BITS)    /* 52 ms at 20 MS/s */
#define SMASK        (SRING - 1)
#define S0           1024                  /* first index; ring before it is zero */
#define N_DEC        2
#define JOBQ         64
#define NPEND        8

/* sync_short: |sum48 x[n]x*[n-16]| / sum64 |x[n]|^2 > 0.56 for 3 samples.
 * A new trigger 480 samples (24 us) into a frame aborts it, which is what
 * catches the legacy ACK/BlockAck one SIFS after an HT/VHT/HE frame. */
#define STS_THRESH   0.56
#define MIN_PLATEAU  2
#define MIN_GAP      480
#define MAX_COPY     43200
/* sync_long searches 240 samples from the trigger for the two L-LTF peaks. */
#define SYNC_LEN     240
#define FRONT_N      480

#define MAX_PSDU     4095
#define MAX_SYM      1366                  /* ceil((22 + 8*4095) / 24) */
#define MAX_DBITS    (152 * 216 + 64)      /* worst n_sym * n_dbps over all rates */
#define MAX_CBITS    (MAX_SYM * 48 + 288)

#define C(a, b) CMPLXF(a, b)

/* gr-ieee802-11 sync_long LONG: conj(time-reversed L-LTF), 64 taps. */
static const cf k_ltf_mf[64] = {
    C(-0.0455, -1.0679), C(0.3528, -0.9865),  C(0.8594, 0.7348),   C(0.1874, 0.2475),
    C(0.5309, -0.7784),  C(-1.0218, -0.4897), C(-0.3401, -0.9423), C(0.8657, -0.2298),
    C(0.4734, 0.0362),   C(0.0088, -1.0207),  C(-1.2142, -0.4205), C(0.2172, -0.5195),
    C(0.5207, -0.1326),  C(-0.1995, 1.4259),  C(1.0583, -0.0363),  C(0.5547, -0.5547),
    C(0.3277, 0.8728),   C(-0.5077, 0.3488),  C(-1.1650, 0.5789),  C(0.7297, 0.8197),
    C(0.6173, 0.1253),   C(-0.5353, 0.7214),  C(-0.5011, -0.1935), C(-0.3110, -1.3392),
    C(-1.0818, -0.1470), C(-1.1300, -0.1820), C(0.6663, -0.6571),  C(-0.0249, 0.4773),
    C(-0.8155, 1.0218),  C(0.8140, 0.9396),   C(0.1090, 0.8662),   C(-1.3868, -0.0000),
    C(0.1090, -0.8662),  C(0.8140, -0.9396),  C(-0.8155, -1.0218), C(-0.0249, -0.4773),
    C(0.6663, 0.6571),   C(-1.1300, 0.1820),  C(-1.0818, 0.1470),  C(-0.3110, 1.3392),
    C(-0.5011, 0.1935),  C(-0.5353, -0.7214), C(0.6173, -0.1253),  C(0.7297, -0.8197),
    C(-1.1650, -0.5789), C(-0.5077, -0.3488), C(0.3277, -0.8728),  C(0.5547, 0.5547),
    C(1.0583, 0.0363),   C(-0.1995, -1.4259), C(0.5207, 0.1326),   C(0.2172, 0.5195),
    C(-1.2142, 0.4205),  C(0.0088, 1.0207),   C(0.4734, -0.0362),  C(0.8657, 0.2298),
    C(-0.3401, 0.9423),  C(-1.0218, 0.4897),  C(0.5309, 0.7784),   C(0.1874, -0.2475),
    C(0.8594, -0.7348),  C(0.3528, 0.9865),   C(-0.0455, 1.0679),  C(1.3868, -0.0000),
};

/* L-LTF in frequency, FFT-shifted bins 0..63 (DC at 32). */
static const signed char k_ltf_f[64] = {
    0,  0,  0,  0,  0,  0,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,
    1,  1,  1,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,
    0,  1, -1, -1,  1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,
    1, -1, -1,  1, -1,  1, -1,  1,  1,  1,  1,  0,  0,  0,  0,  0,
};

/* Pilot polarity p_n, 127-periodic, starting at the SIGNAL symbol. */
static const signed char k_pol[127] = {
    1,  1,  1,  1, -1, -1, -1,  1, -1, -1, -1, -1,  1,  1, -1,  1, -1, -1,  1,  1, -1,  1,
    1, -1,  1,  1,  1,  1,  1,  1, -1,  1,  1,  1, -1,  1,  1, -1, -1,  1,  1,  1, -1,  1,
   -1, -1, -1,  1, -1,  1, -1, -1,  1, -1, -1,  1,  1,  1,  1,  1, -1, -1,  1,  1, -1, -1,
    1, -1,  1, -1,  1,  1, -1, -1, -1,  1,  1, -1, -1, -1, -1,  1, -1, -1,  1, -1,  1,  1,
    1,  1, -1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1, -1,  1,  1, -1,  1, -1,  1,  1,
    1, -1, -1,  1, -1, -1, -1,  1,  1,  1, -1, -1, -1, -1, -1, -1, -1,
};

typedef struct {
    int r;          /* L-SIG RATE, R1 in bit 0 */
    int bpsc, cbps, dbps;
    int punct;      /* 0: 1/2, 1: 2/3, 2: 3/4 */
    int mbps;
    const char *name;
} rate_t;

static const rate_t k_rates[8] = {
    { 11, 1,  48,  24, 0,  6, "BPSK 1/2"   },
    { 15, 1,  48,  36, 2,  9, "BPSK 3/4"   },
    { 10, 2,  96,  48, 0, 12, "QPSK 1/2"   },
    { 14, 2,  96,  72, 2, 18, "QPSK 3/4"   },
    {  9, 4, 192,  96, 0, 24, "16-QAM 1/2" },
    { 13, 4, 192, 144, 2, 36, "16-QAM 3/4" },
    {  8, 6, 288, 192, 1, 48, "64-QAM 2/3" },
    { 12, 6, 288, 216, 2, 54, "64-QAM 3/4" },
};

static const unsigned char k_punct[3][6] = {
    { 1, 1 }, { 1, 1, 1, 0 }, { 1, 1, 1, 0, 0, 1 },
};
static const int k_punct_len[3] = { 2, 4, 6 };

static int      k_data[48];        /* data bins, FFT-shifted */
static uint8_t  k_out[128];        /* (g0 << 1) | g1 for 7-bit encoder state */
static int      k_perm[7][288];    /* deinterleaver by n_bpsc */
static uint32_t k_crc[256];
static pthread_once_t g_tables_once = PTHREAD_ONCE_INIT;

static void tables_init(void)
{
    int c = 0;
    for (int i = 6; i <= 58; i++)
        if (i != 11 && i != 25 && i != 32 && i != 39 && i != 53)
            k_data[c++] = i;
    for (int r = 0; r < 128; r++) {
        int g0 = __builtin_parity(r & 0x6D);   /* 133 octal */
        int g1 = __builtin_parity(r & 0x4F);   /* 171 octal */
        k_out[r] = (uint8_t)((g0 << 1) | g1);
    }
    static const int bpscs[4] = { 1, 2, 4, 6 };
    for (int b = 0; b < 4; b++) {
        int bpsc = bpscs[b], cbps = 48 * bpsc;
        int s = bpsc / 2 > 1 ? bpsc / 2 : 1;
        for (int k = 0; k < cbps; k++) {
            int first = s * (k / s) + ((k + (16 * k) / cbps) % s);
            int second = 16 * first - (cbps - 1) * ((16 * first) / cbps);
            k_perm[bpsc][k] = second;
        }
    }
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t v = i;
        for (int k = 0; k < 8; k++)
            v = (v & 1) ? (v >> 1) ^ 0xEDB88320u : v >> 1;
        k_crc[i] = v;
    }
}

static uint32_t crc32_ieee(const uint8_t *p, int n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (int i = 0; i < n; i++)
        c = k_crc[(c ^ p[i]) & 0xFF] ^ (c >> 8);
    return ~c;
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

static pthread_mutex_t g_err_mtx = PTHREAD_MUTEX_INITIALIZER;
static char            g_err[80];
static atomic_int      g_stop;
static atomic_int      g_dead;
static atomic_int      g_gain = 45;
static atomic_int      g_running;
static double          g_mhz;
static int             g_ch;
static atomic_uint     g_seq;

static csi_dev_t       g_wdev;
static cf             *g_sr;               /* 20 MS/s sample ring */
static _Atomic uint64_t g_w;               /* producer write index */
static _Atomic uint64_t g_keep;            /* oldest index the detector still needs */

static struct {
    _Atomic uint64_t triggers, sig_ok, aborted, sync_fail, skip_ht, crc_fail;
    _Atomic uint64_t pkts, bytes, q_drop, in_drop, in_bytes;
} g_st;

static pthread_t g_prod_th, g_det_th, g_aim_th, g_dec_th[N_DEC];
static int       g_prod_alive, g_det_alive, g_aim_alive, g_dec_alive[N_DEC];

static void set_err(const char *s)
{
    pthread_mutex_lock(&g_err_mtx);
    snprintf(g_err, sizeof g_err, "%s", s ? s : "");
    pthread_mutex_unlock(&g_err_mtx);
    if (s && s[0])
        fprintf(stderr, "phasegaze: wifi: %s\n", s);
}

void wifi_pipeline_error(char *dst, size_t n)
{
    if (!dst || n == 0) return;
    pthread_mutex_lock(&g_err_mtx);
    snprintf(dst, n, "%s", g_err);
    pthread_mutex_unlock(&g_err_mtx);
}

int wifi_pipeline_dead(void)
{
    return atomic_load(&g_dead);
}

void wifi_set_gain(int gain)
{
    if (gain < 0) gain = 0;
    if (gain > 63) gain = 63;
    atomic_store(&g_gain, gain);
}

uint64_t wifi_packets_decoded(void) { return atomic_load(&g_st.pkts); }
uint64_t wifi_bytes_decoded(void)   { return atomic_load(&g_st.bytes); }

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

// ---------------------------------------------------------------------------
// Producer: CSI ring (CS8 at 149.5 MS/s) -> 4-tap cubic Lagrange -> 20 MS/s.
// The FPGA k=12 decimator already limits the summed lane to +-10 MHz, so the
// interpolator alone is enough (same path as the Soapy driver's Farrow).
// ---------------------------------------------------------------------------

#define BACKLOG_BYTES (8u << 20)
#define CHUNK_BYTES   (256u << 10)

static inline uint32_t ring_used(uint32_t head, uint32_t tail, uint64_t size)
{
    return head >= tail ? head - tail : (uint32_t)size - (tail - head);
}

/* pos is the next output instant in input samples relative to in[0]; it can
 * be down to -2 at chunk entry, and carry holds the previous chunk's last
 * 4 samples as indices -4..-1. */
static uint64_t farrow(const int8_t *in, int n, double *pos_io, int8_t carry[8],
                       uint64_t w, double ratio)
{
    double pos = *pos_io;
    for (;;) {
        int ip = (int)floor(pos);
        if (ip + 2 >= n) break;
        float mu = (float)(pos - ip);
        float s[4][2];
        if (ip >= 1) {
            const int8_t *p = in + 2 * (ip - 1);
            for (int t = 0; t < 4; t++) {
                s[t][0] = p[2 * t];
                s[t][1] = p[2 * t + 1];
            }
        } else {
            for (int t = 0; t < 4; t++) {
                int k = ip - 1 + t;
                const int8_t *p = k < 0 ? carry + 2 * (k + 4) : in + 2 * k;
                s[t][0] = p[0];
                s[t][1] = p[1];
            }
        }
        float y[2];
        for (int c = 0; c < 2; c++) {
            float s0 = s[0][c], s1 = s[1][c], s2 = s[2][c], s3 = s[3][c];
            float c1 = s2 - 0.5f * s1 - (1.0f / 3.0f) * s0 - (1.0f / 6.0f) * s3;
            float c2 = 0.5f * (s0 + s2) - s1;
            float c3 = (1.0f / 6.0f) * (s3 - s0) + 0.5f * (s1 - s2);
            y[c] = s1 + mu * (c1 + mu * (c2 + mu * c3));
        }
        g_sr[w & SMASK] = CMPLXF(y[0], y[1]);
        w++;
        pos += ratio;
    }
    *pos_io = pos - n;
    memcpy(carry, in + 2 * (n - 4), 8);
    return w;
}

static void *producer_main(void *arg)
{
    (void)arg;
    csi_dev_t *d = &g_wdev;
    const double ratio = CSI_RATE / FS;
    const uint32_t rsz = (uint32_t)d->ring_size;
    double pos = 0.0;
    int8_t carry[8] = { 0 };
    uint64_t w = atomic_load(&g_w);
    int applied = -1, errs = 0;

    csi_dev_flush(d, 0);
    while (!atomic_load(&g_stop)) {
        int g = atomic_load(&g_gain);
        if (g != applied) {
            csi_dev_set_gain(d, g);
            applied = g;
        }
        uint32_t head, tail;
        if (csi_dev_ring_pos(d, &head, &tail) != 0) {
            if (++errs > 200) {
                set_err("csi ring");
                atomic_store(&g_dead, 1);
                break;
            }
            usleep(5000);
            continue;
        }
        errs = 0;
        uint32_t used = ring_used(head, tail, rsz);
        if (used < 4096) {
            csi_dev_wait(d, 20);
            continue;
        }
        if (used > BACKLOG_BYTES) {
            uint32_t drop = (used - (1u << 20)) & ~1u;
            csi_dev_consume(d, drop);
            atomic_fetch_add(&g_st.in_drop, drop);
            continue;
        }
        uint32_t n = used;
        if ((uint64_t)tail + n > rsz) n = rsz - tail;
        if (n > CHUNK_BYTES) n = CHUNK_BYTES;
        n &= ~1u;
        if (n < 16) {
            csi_dev_consume(d, n);
            continue;
        }
        uint64_t nout = (uint64_t)(n / 2 / ratio) + 4;
        uint64_t keep = atomic_load_explicit(&g_keep, memory_order_acquire);
        if (w + nout - keep > SRING - 4096) {
            /* Detector or a pending frame is a full ring behind. */
            csi_dev_consume(d, n);
            atomic_fetch_add(&g_st.in_drop, n);
            continue;
        }
        w = farrow((const int8_t *)d->ring + tail, (int)(n / 2), &pos, carry, w, ratio);
        atomic_store_explicit(&g_w, w, memory_order_release);
        csi_dev_consume(d, n);
        atomic_fetch_add_explicit(&g_st.in_bytes, n, memory_order_relaxed);
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// Frame front end: sync_long, LTF channel estimate, L-SIG (gr-ieee802-11
// sync_long + frame_equalizer). Raw samples start at the sync_short trigger.
// ---------------------------------------------------------------------------

typedef struct {
    cf     H[64];
    cf     prev[4];
    double er, eps0, fc;
    float  snr;
    int    n;               /* 0,1 L-LTF; 2 L-SIG; 3.. data */
} eq_t;

typedef struct {
    int    fs;              /* L-LTF T1 start, samples from trigger */
    float  fo_s, fo_l;      /* rad/sample: coarse removed, fine removed */
    double dphi;            /* total rotation applied from the trigger on */
    const rate_t *rate;
    int    len, n_sym;
    float  sig_re[48];
    eq_t   eq;
} front_t;

typedef struct {
    fftwf_plan plan;
    cf    *fin, *fout;
    cf     buf[SYNC_LEN + 64];
    float  sig_soft[48];
    float *coded;           /* deinterleaved soft bits, n_sym * n_cbps */
    float *mother;          /* depunctured, 2 * n_data_bits */
    uint8_t *bits;
    uint64_t *dec;
    uint8_t *bytes;
} dctx_t;

static int dctx_init(dctx_t *c)
{
    memset(c, 0, sizeof *c);
    c->fin = fftwf_malloc(64 * sizeof(cf));
    c->fout = fftwf_malloc(64 * sizeof(cf));
    c->coded = malloc(MAX_CBITS * sizeof(float));
    c->mother = malloc(2 * MAX_DBITS * sizeof(float));
    c->bits = malloc(MAX_DBITS);
    c->dec = malloc(MAX_DBITS * sizeof(uint64_t));
    c->bytes = malloc(MAX_PSDU + 8);
    if (!c->fin || !c->fout || !c->coded || !c->mother || !c->bits || !c->dec || !c->bytes)
        return -1;
    c->plan = fftwf_plan_dft_1d(64, c->fin, c->fout, FFTW_FORWARD, FFTW_ESTIMATE);
    return c->plan ? 0 : -1;
}

static void dctx_free(dctx_t *c)
{
    if (c->plan) fftwf_destroy_plan(c->plan);
    if (c->fin) fftwf_free(c->fin);
    if (c->fout) fftwf_free(c->fout);
    free(c->coded);
    free(c->mother);
    free(c->bits);
    free(c->dec);
    free(c->bytes);
    memset(c, 0, sizeof *c);
}

/* 64 samples from raw[at], CFO-corrected, FFT, shifted so DC is bin 32. */
static void sym_fft(dctx_t *c, const cf *raw, int at, double dphi, cf *cs)
{
    double ph = dphi * at;
    cf rot = CMPLXF((float)cos(ph), (float)sin(ph));
    cf step = CMPLXF((float)cos(dphi), (float)sin(dphi));
    for (int k = 0; k < 64; k++) {
        c->fin[k] = raw[at + k] * rot;
        rot *= step;
    }
    fftwf_execute(c->plan);
    for (int i = 0; i < 64; i++)
        cs[i] = c->fout[(i + 32) & 63];
}

/* frame_equalizer: SFO phase ramp from eps0 + tracked residual, pilot CPE
 * (beta), residual SFO from pilot rotation between symbols (alpha 0.1). The
 * ramp uses 80 samples per symbol and bin offsets -32..31. */
static void eq_track(eq_t *e, cf *cs)
{
    const int n = e->n;
    double sfo = 2.0 * M_PI * n * 80.0 * (e->eps0 + e->er) / 64.0;
    cf rot = CMPLXF((float)cos(-32.0 * sfo), (float)sin(-32.0 * sfo));
    cf step = CMPLXF((float)cos(sfo), (float)sin(sfo));
    for (int i = 0; i < 64; i++) {
        cs[i] *= rot;
        rot *= step;
    }

    float p = n >= 2 ? (float)k_pol[(n - 2) % 127] : 1.0f;
    float beta;
    if (n < 2)
        beta = cargf(cs[11] - cs[25] + cs[39] + cs[53]);
    else
        beta = cargf(p * (cs[11] + cs[25] + cs[39]) - p * cs[53]);

    cf q = conjf(e->prev[0]) * cs[11] * p + conjf(e->prev[1]) * cs[25] * p +
           conjf(e->prev[2]) * cs[39] * p + conjf(e->prev[3]) * cs[53] * -p;
    double er = cargf(q) * FS / (2.0 * M_PI * e->fc * 80.0);

    if (n < 2) {
        e->prev[0] = cs[11];
        e->prev[1] = -cs[25];
        e->prev[2] = cs[39];
        e->prev[3] = cs[53];
    } else {
        e->prev[0] = cs[11] * p;
        e->prev[1] = cs[25] * p;
        e->prev[2] = cs[39] * p;
        e->prev[3] = cs[53] * -p;
    }

    cf derot = CMPLXF(cosf(-beta), sinf(-beta));
    for (int i = 0; i < 64; i++)
        cs[i] *= derot;
    if (n >= 2)
        e->er = 0.9 * e->er + 0.1 * er;

    if (n == 0) {
        memcpy(e->H, cs, sizeof e->H);
    } else if (n == 1) {
        double sig = 0, noise = 0;
        for (int i = 6; i <= 58; i++) {
            if (i == 32) continue;
            cf a = e->H[i], b = cs[i];
            noise += crealf((a - b) * conjf(a - b));
            sig += crealf((a + b) * conjf(a + b));
            e->H[i] = (a + b) / (2.0f * (float)k_ltf_f[i]);
        }
        e->snr = noise > 0 ? (float)(10.0 * log10(sig / noise / 2.0)) : 99.0f;
    }
    e->n++;
}

/* Soft K=7 (133, 171) Viterbi. sym holds 2*nbits soft values (>0 means 1,
 * 0 means erased). The encoder starts in state 0; end_zero forces the tail
 * state too (L-SIG). The data field ends in scrambled pad bits, so frames
 * take the best final state. */
static void viterbi(const float *sym, int nbits, uint8_t *out, uint64_t *dec, int end_zero)
{
    float m[64], nm[64];
    for (int s = 0; s < 64; s++) m[s] = -1e30f;
    m[0] = 0.0f;
    for (int t = 0; t < nbits; t++) {
        float a = sym[2 * t], b = sym[2 * t + 1];
        float bm[4] = { -a - b, -a + b, a - b, a + b };
        uint64_t d = 0;
        for (int n = 0; n < 64; n++) {
            int p0 = n >> 1, p1 = p0 | 32, b0 = n & 1;
            float c0 = m[p0] + bm[k_out[(p0 << 1) | b0]];
            float c1 = m[p1] + bm[k_out[(p1 << 1) | b0]];
            int take1 = c1 > c0;
            nm[n] = take1 ? c1 : c0;
            d |= (uint64_t)take1 << n;
        }
        dec[t] = d;
        if ((t & 255) == 255) {
            float mx = nm[0];
            for (int s = 1; s < 64; s++) if (nm[s] > mx) mx = nm[s];
            for (int s = 0; s < 64; s++) m[s] = nm[s] - mx;
        } else {
            memcpy(m, nm, sizeof m);
        }
    }
    int s = 0;
    if (!end_zero) {
        float mx = m[0];
        for (int k = 1; k < 64; k++)
            if (m[k] > mx) { mx = m[k]; s = k; }
    }
    for (int t = nbits - 1; t >= 0; t--) {
        out[t] = (uint8_t)(s & 1);
        int msb = (int)((dec[t] >> s) & 1);
        s = (s >> 1) | (msb << 5);
    }
}

static int front(dctx_t *c, const cf *raw, int nraw, float fo_s, double fc, front_t *f)
{
    if (nraw < FRONT_N) return -1;
    for (int k = 0; k < SYNC_LEN + 64; k++)
        c->buf[k] = raw[k] * CMPLXF(cosf(-fo_s * k), sinf(-fo_s * k));

    /* Four largest |corr| over the search window, largest first. */
    float tm[4] = { -1, -1, -1, -1 };
    int tk[4] = { 0, 0, 0, 0 };
    cf tv[4] = { 0, 0, 0, 0 };
    for (int k = 0; k < SYNC_LEN; k++) {
        cf acc = 0;
        for (int m = 0; m < 64; m++)
            acc += c->buf[k + m] * k_ltf_mf[63 - m];
        float mag = crealf(acc * conjf(acc));
        if (mag <= tm[3]) continue;
        int j = 3;
        while (j > 0 && mag > tm[j - 1]) {
            tm[j] = tm[j - 1];
            tk[j] = tk[j - 1];
            tv[j] = tv[j - 1];
            j--;
        }
        tm[j] = mag;
        tk[j] = k;
        tv[j] = acc;
    }

    int fs = -1;
    float fo = 0;
    for (int i = 0; i < 3; i++) {
        for (int k = i + 1; k < 4; k++) {
            int lo = tk[i] < tk[k] ? i : k, hi = lo == i ? k : i;
            int diff = tk[hi] - tk[lo];
            if (diff == 64 || diff == 63 || diff == 65) {
                fs = tk[lo];
                fo = cargf(tv[lo] * conjf(tv[hi])) / (float)diff;
                if (diff == 64) goto found;
            }
        }
    }
    if (fs < 0) return -1;
found:
    if (fs + 128 + 80 > nraw) return -1;

    memset(f, 0, sizeof *f);
    f->fs = fs;
    f->fo_s = fo_s;
    f->fo_l = fo;
    f->dphi = (double)fo - (double)fo_s;
    f->eq.fc = fc;
    f->eq.eps0 = ((double)fo_s - (double)fo) * FS / (2.0 * M_PI * fc);

    cf cs[64];
    sym_fft(c, raw, fs, f->dphi, cs);
    eq_track(&f->eq, cs);
    sym_fft(c, raw, fs + 64, f->dphi, cs);
    eq_track(&f->eq, cs);
    sym_fft(c, raw, fs + 128 + 16, f->dphi, cs);
    eq_track(&f->eq, cs);

    float soft[48];
    for (int k = 0; k < 48; k++) {
        int i = k_data[k];
        cf h = f->eq.H[i];
        float w = crealf(h * conjf(h));
        float v = crealf(cs[i] * conjf(h));
        soft[k] = v;
        f->sig_re[k] = w > 1e-20f ? v / w : 0.0f;
    }
    for (int k = 0; k < 48; k++)
        c->sig_soft[k_perm[1][k]] = soft[k];
    uint8_t b[24];
    viterbi(c->sig_soft, 24, b, c->dec, 1);

    int par = 0, r = 0, len = 0;
    for (int i = 0; i < 17; i++) {
        par ^= b[i];
        if (i < 4 && b[i]) r |= 1 << i;
        if (i > 4 && b[i]) len |= 1 << (i - 5);
    }
    if (par != b[17]) return -1;
    const rate_t *rt = NULL;
    for (int i = 0; i < 8; i++)
        if (k_rates[i].r == r) rt = &k_rates[i];
    if (!rt || len < 14 || len > MAX_PSDU) return -1;
    f->rate = rt;
    f->len = len;
    f->n_sym = (16 + 8 * len + 6 + rt->dbps - 1) / rt->dbps;
    return 0;
}

static inline int frame_end(const front_t *f)
{
    return f->fs + 128 + 80 * (f->n_sym + 1);
}

/* Nearest point on one axis of a square QAM with spacing 2L, |x| <= maxv*L. */
static inline float qam_axis(float x, float L, float maxv)
{
    float v = 2.0f * floorf(x / (2.0f * L)) + 1.0f;
    if (v > maxv) v = maxv;
    if (v < -maxv) v = -maxv;
    return v * L;
}

typedef struct {
    front_t f;
    float   evm_db;
} dres_t;

/* >= 0: PSDU length with good FCS. -1 sync/L-SIG, -2 HT/VHT/HE, -3 FCS. */
static int decode_frame(dctx_t *c, const cf *raw, int nraw, float fo_s, int sta, dres_t *res)
{
    front_t *f = &res->f;
    if (front(c, raw, nraw, fo_s, g_mhz * 1e6, f) != 0) return -1;
    if (frame_end(f) > nraw) return -1;

    const rate_t *rt = f->rate;
    const int bpsc = rt->bpsc, cbps = rt->cbps;
    const float L16 = 0.31622777f, L64 = 0.15430335f;
    double err = 0;
    cf cs[64], Hn[64];
    float tmp[288];

    for (int m = 0; m < f->n_sym; m++) {
        sym_fft(c, raw, f->fs + 128 + 80 * (m + 1) + 16, f->dphi, cs);
        eq_track(&f->eq, cs);
        cf *H = f->eq.H;
        double re2 = 0, im2 = 0, xc = 0, xa = 0, xb = 0;
        for (int k = 0; k < 48; k++) {
            int i = k_data[k];
            cf h = H[i];
            float w = crealf(h * conjf(h));
            cf s = w > 1e-20f ? cs[i] * conjf(h) / w : 0;
            float re = crealf(s), im = cimagf(s);
            cf pt;
            float *o = tmp + k * bpsc;
            switch (bpsc) {
            case 1:
                pt = re > 0 ? 1.0f : -1.0f;
                o[0] = re * w;
                break;
            case 2:
                pt = CMPLXF(re > 0 ? 0.70710678f : -0.70710678f,
                            im > 0 ? 0.70710678f : -0.70710678f);
                o[0] = re * w;
                o[1] = im * w;
                break;
            case 4:
                pt = CMPLXF(qam_axis(re, L16, 3), qam_axis(im, L16, 3));
                o[0] = re * w;
                o[1] = (2 * L16 - fabsf(re)) * w;
                o[2] = im * w;
                o[3] = (2 * L16 - fabsf(im)) * w;
                break;
            default:
                pt = CMPLXF(qam_axis(re, L64, 7), qam_axis(im, L64, 7));
                o[0] = re * w;
                o[1] = (4 * L64 - fabsf(re)) * w;
                o[2] = (2 * L64 - fabsf(fabsf(re) - 4 * L64)) * w;
                o[3] = im * w;
                o[4] = (4 * L64 - fabsf(im)) * w;
                o[5] = (2 * L64 - fabsf(fabsf(im) - 4 * L64)) * w;
                break;
            }
            cf e = s - pt;
            err += crealf(e * conjf(e));
            if (sta) Hn[i] = cs[i] / pt;
            re2 += re * re;
            im2 += im * im;
            if (m == 0) {
                xc += re * f->sig_re[k];
                xa += re * re;
                xb += f->sig_re[k] * f->sig_re[k];
            }
        }
        /* 6 Mb/s L-SIG followed by QBPSK is HT-SIG (sym 0) or VHT-SIG-A2
         * (sym 1); an L-SIG repeat is HE RL-SIG. None of it is a legacy PSDU. */
        if (rt->mbps == 6 && m < 2 && im2 > 2.0 * re2) return -2;
        if (rt->mbps == 6 && m == 0 && xa > 0 && xb > 0 && fabs(xc) > 0.8 * sqrt(xa * xb))
            return -2;

        if (sta) {
            /* gr-ieee802-11 STA: decision-directed H, +-2 bin average, alpha 0.5. */
            float p = (float)k_pol[(f->eq.n - 3) % 127];
            Hn[11] = cs[11] * p;
            Hn[25] = cs[25] * p;
            Hn[39] = cs[39] * p;
            Hn[53] = cs[53] * -p;
            cf upd[64];
            for (int i = 6; i <= 58; i++) {
                int cnt = 0;
                cf s = 0;
                for (int k = i - 2; k <= i + 2; k++) {
                    if (k == 32 || k < 6 || k > 58) continue;
                    cnt++;
                    s += Hn[k];
                }
                upd[i] = s / (float)cnt;
            }
            for (int i = 6; i <= 58; i++)
                H[i] = 0.5f * H[i] + 0.5f * upd[i];
        }

        const int *perm = k_perm[bpsc];
        float *dst = c->coded + m * cbps;
        for (int k = 0; k < cbps; k++)
            dst[perm[k]] = tmp[k];
    }
    res->evm_db = (float)(10.0 * log10(err / (48.0 * f->n_sym) + 1e-12));

    const int ndb = f->n_sym * rt->dbps;
    const unsigned char *pat = k_punct[rt->punct];
    const int plen = k_punct_len[rt->punct];
    int in = 0;
    for (int i = 0; i < 2 * ndb; i++)
        c->mother[i] = pat[i % plen] ? c->coded[in++] : 0.0f;
    viterbi(c->mother, ndb, c->bits, c->dec, 0);

    const int psdu = f->len;
    uint8_t *ob = c->bytes;
    memset(ob, 0, (size_t)psdu + 2);
    int state = 0;
    for (int i = 0; i < 7; i++)
        if (c->bits[i]) state |= 1 << (6 - i);
    ob[0] = (uint8_t)state;
    for (int i = 7; i < psdu * 8 + 16; i++) {
        int fb = ((state >> 6) & 1) ^ ((state >> 3) & 1);
        int bit = fb ^ (c->bits[i] & 1);
        ob[i / 8] |= (uint8_t)(bit << (i % 8));
        state = ((state << 1) & 0x7E) | fb;
    }
    if (crc32_ieee(ob + 2, psdu) != 0x2144DF1Cu) return -3;
    return psdu;
}

// ---------------------------------------------------------------------------
// MAC parse -> JSON (same fields the UI read from quadrf-wifi-rx)
// ---------------------------------------------------------------------------

typedef struct {
    char  *p;
    size_t n, cap;
} jb_t;

__attribute__((format(printf, 2, 3)))
static void jb(jb_t *b, const char *fmt, ...)
{
    if (b->n >= b->cap) return;
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(b->p + b->n, b->cap - b->n, fmt, ap);
    va_end(ap);
    if (k > 0) b->n += (size_t)k;
    if (b->n > b->cap - 1) b->n = b->cap - 1;
}

static void mac_str(char *d, const uint8_t *a)
{
    snprintf(d, 18, "%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], a[2], a[3], a[4], a[5]);
}

static const char *const k_mgmt_names[16] = {
    "Assoc Request", "Assoc Response", "Reassoc Request", "Reassoc Response",
    "Probe Request", "Probe Response", "Timing Advert", "Subtype 7",
    "Beacon", "ATIM", "Disassociation", "Authentication",
    "Deauthentication", "Action", "Action No ACK", "Subtype 15",
};
static const char *const k_ctrl_names[16] = {
    "Subtype 0", "Subtype 1", "Trigger", "TACK", "Beamforming Report", "VHT NDP Announce",
    "Ctrl Ext", "Control Wrapper", "Block ACK Req", "Block ACK", "PS-Poll", "RTS",
    "CTS", "ACK", "CF-End", "CF-End + CF-Ack",
};
static const char *const k_data_names[16] = {
    "Data", "Data + CF-Ack", "Data + CF-Poll", "Data + CF-Ack + CF-Poll",
    "Null Data", "CF-Ack", "CF-Poll", "CF-Ack + CF-Poll",
    "QoS Data", "QoS Data + CF-Ack", "QoS Data + CF-Poll", "QoS Data + CF-Ack + CF-Poll",
    "QoS Null", "Subtype 13", "QoS CF-Poll", "QoS CF-Ack + CF-Poll",
};

static int frame_json(const uint8_t *m, int len, const dres_t *r, char *out, size_t cap)
{
    jb_t b = { out, 0, cap };
    const front_t *f = &r->f;
    const int body = len - 4;
    const int type = (m[0] >> 2) & 3, sub = (m[0] >> 4) & 15;
    const int to_ds = m[1] & 1, from_ds = (m[1] >> 1) & 1;
    const char *tname = type == 0 ? "Mgmt" : type == 1 ? "Ctrl" : type == 2 ? "Data" : "Ext";
    char sname[40];
    if (type == 0) snprintf(sname, sizeof sname, "%s", k_mgmt_names[sub]);
    else if (type == 1) snprintf(sname, sizeof sname, "%s", k_ctrl_names[sub]);
    else if (type == 2) snprintf(sname, sizeof sname, "%s", k_data_names[sub]);
    else snprintf(sname, sizeof sname, "Subtype %d", sub);

    char da[18] = "", sa[18] = "", bssid[18] = "";
    int seq = -1, frag = 0;
    if (type == 0 && body >= 24) {
        mac_str(da, m + 4);
        mac_str(sa, m + 10);
        mac_str(bssid, m + 16);
    } else if (type == 1) {
        if ((sub == 8 || sub == 9 || sub == 10 || sub == 11) && body >= 16) {
            mac_str(da, m + 4);
            mac_str(sa, m + 10);
        } else if (body >= 10) {
            mac_str(da, m + 4);
        }
    } else if (type == 2 && body >= 24) {
        const uint8_t *a1 = m + 4, *a2 = m + 10, *a3 = m + 16;
        if (!to_ds && !from_ds)     { mac_str(da, a1); mac_str(sa, a2); mac_str(bssid, a3); }
        else if (to_ds && !from_ds) { mac_str(bssid, a1); mac_str(sa, a2); mac_str(da, a3); }
        else if (!to_ds && from_ds) { mac_str(da, a1); mac_str(bssid, a2); mac_str(sa, a3); }
        else                        { mac_str(da, a3); mac_str(sa, a2); }
    }
    if ((type == 0 || type == 2) && body >= 24) {
        int sc = m[22] | (m[23] << 8);
        frag = sc & 15;
        seq = sc >> 4;
    }

    char ssid[96] = "";
    int have_ssid = 0, chan = 0, bint = 0;
    if (type == 0) {
        int off = 0;
        if ((sub == 8 || sub == 5) && body >= 36) {
            bint = m[32] | (m[33] << 8);
            off = 36;
        } else if (sub == 4 && body >= 24) {
            off = 24;
        }
        while (off && off + 2 <= body) {
            int id = m[off], tl = m[off + 1];
            if (off + 2 + tl > body) break;
            const uint8_t *v = m + off + 2;
            if (id == 0 && tl <= 32 && !have_ssid) {
                size_t j = 0;
                int ok = 1;
                for (int k = 0; k < tl; k++)
                    if (v[k] < 32 || v[k] > 126) ok = 0;
                for (int k = 0; ok && k < tl && j + 3 < sizeof ssid; k++) {
                    if (v[k] == '"' || v[k] == '\\') ssid[j++] = '\\';
                    ssid[j++] = (char)v[k];
                }
                ssid[j] = 0;
                have_ssid = ok;
            } else if (id == 3 && tl >= 1) {
                chan = v[0];
            }
            off += 2 + tl;
        }
    }

    int has_llc = 0, et = 0, is_ip = 0, proto = 0, iplen = 0, ps = 0, pd = 0, qos = -1;
    char ips[16] = "", ipd[16] = "";
    if (type == 2) {
        int hl = 24;
        if (sub >= 8 && sub <= 11 && body >= 26) {
            qos = m[24] & 15;
            hl = 26;
        }
        if (to_ds && from_ds) hl += 6;
        if (hl + 8 <= body && m[hl] == 0xAA && m[hl + 1] == 0xAA && m[hl + 2] == 0x03) {
            has_llc = 1;
            et = (m[hl + 6] << 8) | m[hl + 7];
            int io = hl + 8;
            if (et == 0x0800 && io + 20 <= body && (m[io] >> 4) == 4) {
                is_ip = 1;
                int ihl = (m[io] & 15) * 4;
                iplen = (m[io + 2] << 8) | m[io + 3];
                proto = m[io + 9];
                snprintf(ips, sizeof ips, "%u.%u.%u.%u", m[io + 12], m[io + 13], m[io + 14], m[io + 15]);
                snprintf(ipd, sizeof ipd, "%u.%u.%u.%u", m[io + 16], m[io + 17], m[io + 18], m[io + 19]);
                int to = io + ihl;
                if ((proto == 6 || proto == 17) && to + 4 <= body) {
                    ps = (m[to] << 8) | m[to + 1];
                    pd = (m[to + 2] << 8) | m[to + 3];
                }
            }
        }
    }
    const char *etn = et == 0x0800 ? "IPv4" : et == 0x86DD ? "IPv6" : et == 0x0806 ? "ARP"
                    : et == 0x888E ? "EAPOL" : "";
    const char *pn = proto == 1 ? "ICMP" : proto == 6 ? "TCP" : proto == 17 ? "UDP" : "IP";

    jb(&b, "{\"type\":\"wifi_pkt\",\"fcs_ok\":true,\"pkt_type\":\"%s\",\"subtype\":\"%s\","
           "\"type_val\":%d,\"subtype_val\":%d,\"fc\":\"0x%04x\"",
       tname, sname, type, sub, m[0] | (m[1] << 8));
    if (da[0]) jb(&b, ",\"da\":\"%s\"", da);
    if (sa[0]) jb(&b, ",\"sa\":\"%s\"", sa);
    if (bssid[0]) jb(&b, ",\"bssid\":\"%s\"", bssid);
    if (have_ssid && ssid[0]) jb(&b, ",\"ssid\":\"%s\"", ssid);
    if (seq >= 0) jb(&b, ",\"seq\":%d,\"frag\":%d", seq, frag);
    jb(&b, ",\"to_ds\":%s,\"from_ds\":%s,\"retry\":%s,\"protected\":%s",
       to_ds ? "true" : "false", from_ds ? "true" : "false",
       (m[1] & 0x08) ? "true" : "false", (m[1] & 0x40) ? "true" : "false");
    if (chan) jb(&b, ",\"channel\":%d", chan);
    if (bint) jb(&b, ",\"beacon_int_tu\":%d", bint);
    if (qos >= 0) jb(&b, ",\"qos_tid\":%d", qos);
    if (has_llc) {
        jb(&b, ",\"has_llc\":true,\"ethertype\":\"0x%04x\"", et);
        if (etn[0]) jb(&b, ",\"ethertype_name\":\"%s\"", etn);
    }
    if (is_ip) {
        jb(&b, ",\"is_ipv4\":true,\"ip_src\":\"%s\",\"ip_dst\":\"%s\",\"ip_proto\":\"%s\",\"ip_len\":%d",
           ips, ipd, pn, iplen);
        if (ps || pd) jb(&b, ",\"port_src\":%d,\"port_dst\":%d", ps, pd);
    }
    jb(&b, ",\"rate\":\"%s\",\"rate_val\":%d,\"len\":%d,\"snr_db\":%.1f,\"evm_db\":%.1f,\"cfo_hz\":%.0f",
       f->rate->name, f->rate->mbps, len, (double)f->eq.snr, (double)r->evm_db,
       (double)(f->fo_s - f->fo_l) * FS / (2.0 * M_PI));

    jb(&b, ",\"summary\":\"%s", sname);
    if (have_ssid && ssid[0]) {
        jb(&b, " SSID:\\\"%s\\\"", ssid);
        if (chan) jb(&b, " [Ch %d]", chan);
        if (bssid[0]) jb(&b, " BSSID:%s", bssid);
    } else if (is_ip) {
        jb(&b, " %s %s", pn, ips);
        if (ps) jb(&b, ":%d", ps);
        jb(&b, " -> %s", ipd);
        if (pd) jb(&b, ":%d", pd);
    } else if (has_llc && etn[0]) {
        jb(&b, " (%s) %s -> %s", etn, sa, da);
    } else if (sa[0] && da[0]) {
        jb(&b, " %s -> %s", sa, da);
    } else if (da[0]) {
        jb(&b, " -> %s", da);
    } else {
        jb(&b, " (%d B)", len);
    }
    jb(&b, "\"}");
    return (int)b.n;
}

// ---------------------------------------------------------------------------
// Direction from the autosteer phase shifters (0x2A/0x2B)
// ---------------------------------------------------------------------------

#define AIM_PHASE_STEP (6.283185307179586f / 256.0f)
#define AIM_SCALE(f) (2.0f * 3.14159265358979f * (0.0455f / 299.792458f) * (f))

static atomic_uint g_aim_u;
static atomic_uint g_aim_v;
static atomic_int  g_aim_ok;

static void aim_store(float u, float v, int ok)
{
    uint32_t bu, bv;
    memcpy(&bu, &u, sizeof bu);
    memcpy(&bv, &v, sizeof bv);
    atomic_store(&g_aim_u, bu);
    atomic_store(&g_aim_v, bv);
    atomic_store(&g_aim_ok, ok);
}

static void *aim_main(void *arg)
{
    (void)arg;
    float su = 0, sv = 0;
    int primed = 0, logged = 0, miss = 0;
    uint64_t t_prev = now_ns();

    while (!atomic_load(&g_stop)) {
        uint16_t ra = 0, rb = 0;
        int bad = csi_dev_read_aoa(&g_wdev, &ra, &rb) != 0;
        uint64_t now = now_ns();
        float dt = (float)(now - t_prev) * 1e-9f;
        t_prev = now;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.2f) dt = 0.2f;

        if (!bad) {
            float p0 = (float)(ra >> 8) * AIM_PHASE_STEP;
            float p1 = (float)(ra & 255) * AIM_PHASE_STEP;
            float p2 = (float)(rb >> 8) * AIM_PHASE_STEP;
            float p3 = (float)(rb & 255) * AIM_PHASE_STEP;
            float gx, gy;
            float cost = dsp_solve_gradient(remainderf(-(p1 - p0), 6.283185307179586f),
                                            remainderf(-(p2 - p0), 6.283185307179586f),
                                            remainderf(-(p3 - p0), 6.283185307179586f),
                                            8.0f, &gx, &gy);
            float scale = AIM_SCALE((float)g_mhz);
            float u = 0, v = 0;
            int ok = cost <= 8.0f && scale > 1.0f;
            if (ok) {
                u = gx / scale;
                v = gy / scale;
                float r2 = u * u + v * v;
                if (r2 > 1.0f) {
                    if (r2 > 1.32f) ok = 0;
                    else {
                        float inv = 0.999f / sqrtf(r2);
                        u *= inv;
                        v *= inv;
                    }
                }
            }
            if (ok) {
                if (!primed) {
                    su = u;
                    sv = v;
                    primed = 1;
                } else {
                    float a = 1.0f - expf(-dt / 0.04f);
                    su += a * (u - su);
                    sv += a * (v - sv);
                }
                aim_store(su, sv, 1);
                miss = 0;
                if (!logged) {
                    fprintf(stderr, "phasegaze: wifi aim u %+.2f v %+.2f\n", su, sv);
                    logged = 1;
                }
            } else if (++miss > 15) {
                primed = 0;
                aim_store(0, 0, 0);
            }
        }
        usleep(16000);
    }
    return NULL;
}

static void publish_json(const char *json, size_t len)
{
    uint8_t buf[sizeof(pg_hdr_t) + 2048];
    if (len > 2048) return;
    pg_hdr_t *h = (pg_hdr_t *)buf;
    h->magic    = PG_MAGIC;
    h->version  = PG_VERSION;
    h->type     = PG_FRAME_WIFI;
    h->count    = (uint32_t)len;
    h->lo_start = (float)g_mhz;
    if (atomic_load(&g_aim_ok)) {
        uint32_t bu = atomic_load(&g_aim_u);
        uint32_t bv = atomic_load(&g_aim_v);
        memcpy(&h->lo_end, &bu, sizeof bu);
        memcpy(&h->reserved, &bv, sizeof bv);
    } else {
        h->lo_end   = 0.0f;
        h->reserved = 0;
    }
    h->fps = 0.0f;
    h->seq = atomic_fetch_add(&g_seq, 1);
    memcpy(buf + sizeof(pg_hdr_t), json, len);
    server_publish(SERVER_SLOT_WIFI, buf, sizeof(pg_hdr_t) + len);
}

// ---------------------------------------------------------------------------
// Job queue: detector -> decode workers
// ---------------------------------------------------------------------------

typedef struct {
    int   n;
    float fo_s;
    cf    raw[];
} job_t;

static pthread_mutex_t g_q_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_q_cv = PTHREAD_COND_INITIALIZER;
static job_t          *g_q[JOBQ];
static int             g_q_head, g_q_len;

static int job_push(job_t *j)
{
    pthread_mutex_lock(&g_q_mtx);
    if (g_q_len >= JOBQ) {
        pthread_mutex_unlock(&g_q_mtx);
        return -1;
    }
    g_q[(g_q_head + g_q_len) % JOBQ] = j;
    g_q_len++;
    pthread_cond_signal(&g_q_cv);
    pthread_mutex_unlock(&g_q_mtx);
    return 0;
}

static job_t *job_pop(void)
{
    pthread_mutex_lock(&g_q_mtx);
    while (!g_q_len && !atomic_load(&g_stop))
        pthread_cond_wait(&g_q_cv, &g_q_mtx);
    job_t *j = NULL;
    if (g_q_len) {
        j = g_q[g_q_head];
        g_q_head = (g_q_head + 1) % JOBQ;
        g_q_len--;
    }
    pthread_mutex_unlock(&g_q_mtx);
    return j;
}

static void *decode_main(void *arg)
{
    dctx_t *c = arg;
    char json[2048];
    for (;;) {
        job_t *j = job_pop();
        if (!j) break;
        if (atomic_load(&g_stop)) {
            free(j);
            continue;
        }
        dres_t r;
        int rc = decode_frame(c, j->raw, j->n, j->fo_s, 0, &r);
        if (rc == -3)
            rc = decode_frame(c, j->raw, j->n, j->fo_s, 1, &r);
        if (rc >= 14) {            atomic_fetch_add(&g_st.pkts, 1);
            atomic_fetch_add(&g_st.bytes, (uint64_t)rc);
            int n = frame_json(c->bytes + 2, rc, &r, json, sizeof json);
            if (n > 0) publish_json(json, (size_t)n);
        } else if (rc == -2) {
            atomic_fetch_add(&g_st.skip_ht, 1);
        } else if (rc == -3) {
            atomic_fetch_add(&g_st.crc_fail, 1);        } else {
            atomic_fetch_add(&g_st.sync_fail, 1);
        }
        free(j);
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// Detector: streaming sync_short over the sample ring
// ---------------------------------------------------------------------------

typedef struct {
    uint64_t start;   /* first sample handed to sync_long (trigger - 16) */
    uint64_t need;    /* one past the last sample of the frame, 0 = L-SIG pending */
    float    fo;
    int      live;
} pend_t;

static void ring_copy(cf *dst, uint64_t from, int n)
{
    uint32_t a = (uint32_t)(from & SMASK);
    uint32_t n1 = SRING - a;
    if ((uint32_t)n <= n1) {
        memcpy(dst, g_sr + a, (size_t)n * sizeof(cf));
    } else {
        memcpy(dst, g_sr + a, (size_t)n1 * sizeof(cf));
        memcpy(dst + n1, g_sr, (size_t)(n - n1) * sizeof(cf));
    }
}

/* Read L-SIG once its samples are in, then queue the frame once it has
 * fully arrived. Returns 1 while still pending. */
static int pend_step(dctx_t *c, pend_t *p, uint64_t w)
{
    if (!p->need) {
        if (w < p->start + FRONT_N) return 1;
        cf raw[FRONT_N];
        ring_copy(raw, p->start, FRONT_N);
        front_t f;
        if (front(c, raw, FRONT_N, p->fo, g_mhz * 1e6, &f) != 0) {
            atomic_fetch_add(&g_st.sync_fail, 1);
            p->live = 0;
            return 0;
        }
        atomic_fetch_add(&g_st.sig_ok, 1);
        p->need = p->start + (uint64_t)frame_end(&f) + 8;
    }
    if (w < p->need) return 1;
    int n = (int)(p->need - p->start);
    job_t *j = malloc(sizeof(job_t) + (size_t)n * sizeof(cf));
    if (j) {
        j->n = n;
        j->fo_s = p->fo;
        ring_copy(j->raw, p->start, n);
        if (job_push(j) != 0) {
            free(j);
            atomic_fetch_add(&g_st.q_drop, 1);
        }
    }
    p->live = 0;
    return 0;
}

static void *detect_main(void *arg)
{
    dctx_t *c = arg;
    pend_t pend[NPEND];
    memset(pend, 0, sizeof pend);
    const double thr2 = STS_THRESH * STS_THRESH;
    double cr = 0, ci = 0, pw = 0;
    int plateau = 0, copying = 0, copied = 0;
    uint64_t r = S0;
    uint64_t t_log = now_ns();
    uint64_t last_pk = 0, last_tr = 0, last_in = 0;

    while (!atomic_load(&g_stop)) {
        uint64_t w = atomic_load_explicit(&g_w, memory_order_acquire);
        if (r >= w) {
            usleep(300);
        } else {
            uint64_t end = w - r > 16384 ? r + 16384 : w;
            for (; r < end; r++) {
                cf x0 = g_sr[r & SMASK], x16 = g_sr[(r - 16) & SMASK];
                cf x48 = g_sr[(r - 48) & SMASK], x64 = g_sr[(r - 64) & SMASK];
                cf pn = x0 * conjf(x16), po = x48 * conjf(x64);
                cr += (double)crealf(pn) - (double)crealf(po);
                ci += (double)cimagf(pn) - (double)cimagf(po);
                pw += (double)crealf(x0 * conjf(x0)) - (double)crealf(x64 * conjf(x64));

                if (pw > 1.0 && cr * cr + ci * ci > thr2 * pw * pw) {
                    if (plateau < MIN_PLATEAU) {
                        plateau++;
                    } else if (!copying || copied > MIN_GAP) {
                        uint64_t s = r - 16;
                        for (int i = 0; i < NPEND; i++) {
                            if (!pend[i].live) continue;
                            if (pend_step(c, &pend[i], r) && pend[i].live) {
                                pend[i].live = 0;
                                atomic_fetch_add(&g_st.aborted, 1);
                            }
                        }
                        int slot = -1;
                        for (int i = 0; i < NPEND && slot < 0; i++)
                            if (!pend[i].live) slot = i;
                        if (slot >= 0) {
                            pend[slot].start = s;
                            pend[slot].need = 0;
                            pend[slot].fo = (float)(atan2(ci, cr) / 16.0);
                            pend[slot].live = 1;
                        }
                        atomic_fetch_add(&g_st.triggers, 1);
                        copying = 1;
                        copied = 0;
                        plateau = 0;
                        continue;
                    }
                } else {
                    plateau = 0;
                }
                if (copying && ++copied >= MAX_COPY)
                    copying = 0;
            }
            /* Recenter the running sums now and then; adds and subtracts of
             * identical floats cancel in double, but a ring wrap after an
             * overrun drop does not. */
            if ((r & 0xFFFFF) < 16384) {
                cr = ci = pw = 0;
                for (uint64_t k = r - 48; k < r; k++) {
                    cf a = g_sr[k & SMASK], b = g_sr[(k - 16) & SMASK];
                    cf p = a * conjf(b);
                    cr += crealf(p);
                    ci += cimagf(p);
                }
                for (uint64_t k = r - 64; k < r; k++) {
                    cf a = g_sr[k & SMASK];
                    pw += crealf(a * conjf(a));
                }
            }
        }

        uint64_t keep = r > 256 ? r - 256 : 0;
        for (int i = 0; i < NPEND; i++) {
            if (!pend[i].live) continue;
            pend_step(c, &pend[i], r);
            if (pend[i].live && pend[i].start < keep)
                keep = pend[i].start;
        }
        atomic_store_explicit(&g_keep, keep, memory_order_release);

        uint64_t t = now_ns();
        if (t - t_log >= 5000000000ull) {
            double dt = (double)(t - t_log) * 1e-9;
            uint64_t pk = atomic_load(&g_st.pkts), tr = atomic_load(&g_st.triggers);
            uint64_t in = atomic_load(&g_st.in_bytes);
            fprintf(stderr, "phasegaze: wifi %.1f pkt/s, %.0f trig/s, in %.1f MS/s, "
                    "sig %llu ht %llu crc %llu sync %llu abort %llu qdrop %llu indrop %llu\n",
                    (double)(pk - last_pk) / dt, (double)(tr - last_tr) / dt,
                    (double)(in - last_in) / 2.0 / dt / 1e6,
                    (unsigned long long)atomic_load(&g_st.sig_ok),
                    (unsigned long long)atomic_load(&g_st.skip_ht),
                    (unsigned long long)atomic_load(&g_st.crc_fail),
                    (unsigned long long)atomic_load(&g_st.sync_fail),
                    (unsigned long long)atomic_load(&g_st.aborted),
                    (unsigned long long)atomic_load(&g_st.q_drop),
                    (unsigned long long)atomic_load(&g_st.in_drop));
            last_pk = pk;
            last_tr = tr;
            last_in = in;
            t_log = t;
        }
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// Start / stop
// ---------------------------------------------------------------------------

static dctx_t g_det_ctx, g_dec_ctx[N_DEC];

int wifi_pipeline_start(double freq_mhz)
{
    wifi_pipeline_stop();
    pthread_once(&g_tables_once, tables_init);
    set_err("");
    atomic_store(&g_dead, 0);
    atomic_store(&g_stop, 0);
    aim_store(0, 0, 0);
    memset(&g_st, 0, sizeof g_st);

    const char *band = NULL;
    g_mhz = wifi_snap_mhz(freq_mhz, &g_ch, &band);

    if (!g_sr) {
        g_sr = calloc(SRING, sizeof(cf));
        if (!g_sr) {
            set_err("no wifi buffer");
            return -1;
        }
    } else {
        memset(g_sr, 0, SRING * sizeof(cf));
    }
    atomic_store(&g_w, S0);
    atomic_store(&g_keep, S0);

    int ok = 0;
    for (int i = 0; i < 10; i++) {
        if (csi_dev_open(&g_wdev, DEVICE_PATH) == 0) {
            ok = 1;
            break;
        }
        usleep(150000);
    }
    if (!ok) {
        set_err("csi open");
        return -1;
    }
    atomic_store(&g_running, 1);
    /* csi_dev_open restores the sweep front end (interleave on, autosteer
     * off); put the Wi-Fi one back with the requested manual gain. */
    csi_dev_set_gain(&g_wdev, atomic_load(&g_gain));
    if (csi_dev_wifi_front_end(&g_wdev) != 0) {
        wifi_pipeline_stop();
        set_err("front end");
        return -1;
    }

    if (dctx_init(&g_det_ctx) != 0) goto fail;
    for (int i = 0; i < N_DEC; i++)
        if (dctx_init(&g_dec_ctx[i]) != 0) goto fail;

    if (pthread_create(&g_prod_th, NULL, producer_main, NULL) != 0) goto fail;
    g_prod_alive = 1;
    if (pthread_create(&g_det_th, NULL, detect_main, &g_det_ctx) != 0) goto fail;
    g_det_alive = 1;
    for (int i = 0; i < N_DEC; i++) {
        if (pthread_create(&g_dec_th[i], NULL, decode_main, &g_dec_ctx[i]) != 0) goto fail;
        g_dec_alive[i] = 1;
    }
    if (pthread_create(&g_aim_th, NULL, aim_main, NULL) == 0)
        g_aim_alive = 1;

    fprintf(stderr, "phasegaze: wifi receiver on %.1f MHz (ch %d %s), gain %d dB\n",
            g_mhz, g_ch, band ? band : "", atomic_load(&g_gain));
    return 0;

fail:
    wifi_pipeline_stop();
    set_err("wifi start");
    return -1;
}

void wifi_pipeline_stop(void)
{
    atomic_store(&g_stop, 1);
    pthread_mutex_lock(&g_q_mtx);
    pthread_cond_broadcast(&g_q_cv);
    pthread_mutex_unlock(&g_q_mtx);

    if (g_aim_alive) { pthread_join(g_aim_th, NULL); g_aim_alive = 0; }
    if (g_prod_alive) { pthread_join(g_prod_th, NULL); g_prod_alive = 0; }
    if (g_det_alive) { pthread_join(g_det_th, NULL); g_det_alive = 0; }
    for (int i = 0; i < N_DEC; i++)
        if (g_dec_alive[i]) { pthread_join(g_dec_th[i], NULL); g_dec_alive[i] = 0; }

    pthread_mutex_lock(&g_q_mtx);
    while (g_q_len) {
        free(g_q[g_q_head]);
        g_q_head = (g_q_head + 1) % JOBQ;
        g_q_len--;
    }
    g_q_head = 0;
    pthread_mutex_unlock(&g_q_mtx);

    if (atomic_exchange(&g_running, 0)) {
        dctx_free(&g_det_ctx);
        for (int i = 0; i < N_DEC; i++)
            dctx_free(&g_dec_ctx[i]);
        csi_dev_close(&g_wdev);
    }
    atomic_store(&g_dead, 1);
}
