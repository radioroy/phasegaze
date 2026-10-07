// wifi.c — 802.11 OFDM Wi-Fi decoder pipeline for PhaseGaze.
//
// In Wi-Fi decode mode, the RF sweep is halted, the LO is locked to the
// selected 20 MHz Wi-Fi channel carrier frequency, and /dev/csi_stream0 is
// closed so that quadrf-wifi-rx can open the hardware via SoapySDR (driver=mipi).
//
// The quadrf-wifi-rx child process streams JSON packet summaries and
// IQ constellation points across a pipe to our reader thread, which wraps
// them into PG_FRAME_WIFI binary frames and broadcasts them over the
// PhaseGaze WebSocket (SERVER_SLOT_WIFI).

#define _GNU_SOURCE
#include "wifi.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/types.h>
#include "fpga_csi.h"
#include "dsp.h"
#include "pg_stream.h"
#include "server.h"

extern char **environ;

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

static pthread_mutex_t g_err_mtx = PTHREAD_MUTEX_INITIALIZER;
static char            g_err[80];
static atomic_int      g_stop;
static atomic_int      g_dead;
static pid_t           g_wifi_pid;
static int             g_fd = -1;
static pthread_t       g_reader;
static int             g_reader_alive;
static double          g_mhz;
static int             g_ch;
static uint32_t        g_seq;

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

static int spawn_fd(char *const argv[], int in_fd, int out_fd, pid_t *pid)
{
    posix_spawn_file_actions_t fa;
    int rc = posix_spawn_file_actions_init(&fa);
    if (rc != 0) return rc;
    if (in_fd >= 0 && in_fd != STDIN_FILENO)
        rc = posix_spawn_file_actions_adddup2(&fa, in_fd, STDIN_FILENO);
    if (rc == 0 && out_fd >= 0 && out_fd != STDOUT_FILENO)
        rc = posix_spawn_file_actions_adddup2(&fa, out_fd, STDOUT_FILENO);
    if (rc == 0)
        rc = posix_spawn_file_actions_addclosefrom_np(&fa, STDERR_FILENO + 1);
    if (rc == 0)
        rc = posix_spawn(pid, argv[0], &fa, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    return rc;
}

static void reap(void)
{
    if (g_wifi_pid <= 0) return;
    kill(g_wifi_pid, SIGTERM);
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < 20; i++) {
            int st;
            if (waitpid(g_wifi_pid, &st, WNOHANG) == g_wifi_pid) {
                g_wifi_pid = 0;
                return;
            }
            usleep(25000);
        }
        if (g_wifi_pid > 0) kill(g_wifi_pid, SIGKILL);
    }
    g_wifi_pid = 0;
}

#define AIM_PHASE_STEP (6.283185307179586f / 256.0f)
#define AIM_SCALE(f) (2.0f * 3.14159265358979f * (0.0455f / 299.792458f) * (f))

static atomic_uint g_aim_u;
static atomic_uint g_aim_v;
static atomic_int  g_aim_ok;
static pthread_t   g_phase_th;
static int         g_phase_alive;

static void aim_store(float u, float v, int ok)
{
    uint32_t bu, bv;
    memcpy(&bu, &u, sizeof bu);
    memcpy(&bv, &v, sizeof bv);
    atomic_store(&g_aim_u, bu);
    atomic_store(&g_aim_v, bv);
    atomic_store(&g_aim_ok, ok);
}

static int phase_fd_open(void)
{
    int fd = open("/dev/csi_stream0", O_RDWR | O_NONBLOCK);
    if (fd < 0) return -1;
    if (ioctl(fd, CSI_IOC_JTAG_SETUP) != 0) {
        close(fd);
        return -1;
    }
    struct timespec ts0;
    clock_gettime(CLOCK_MONOTONIC, &ts0);
    uint64_t t0 = (uint64_t)ts0.tv_sec * 1000000000ull + (uint64_t)ts0.tv_nsec;
    while (ioctl(fd, CSI_IOC_JTAG_ACQUIRE_LEASE) != 0) {
        struct timespec tsc;
        clock_gettime(CLOCK_MONOTONIC, &tsc);
        uint64_t tc = (uint64_t)tsc.tv_sec * 1000000000ull + (uint64_t)tsc.tv_nsec;
        if (errno != EBUSY || tc - t0 > 500000000ull) {
            close(fd);
            return -1;
        }
        usleep(2000);
    }
    return fd;
}

static void *phase_main(void *arg)
{
    (void)arg;
    for (int i = 0; i < 50 && !atomic_load(&g_stop); i++)
        usleep(20000);
    if (atomic_load(&g_stop)) return NULL;

    int fd = phase_fd_open();
    if (fd < 0) {
        fprintf(stderr, "phasegaze: wifi aim off\n");
        return NULL;
    }

    float su = 0, sv = 0;
    int primed = 0, logged = 0, miss = 0;
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    uint64_t t_prev = (uint64_t)tp.tv_sec * 1000000000ull + (uint64_t)tp.tv_nsec;

    while (!atomic_load(&g_stop)) {
        struct csi_jtag_reg r = { .addr = 0x2A };
        uint16_t ra = 0, rb = 0;
        int bad = ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) != 0;
        if (!bad) {
            ra = r.value;
            r.addr = 0x2B;
            r.value = 0;
            bad = ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) != 0;
            rb = r.value;
        }

        struct timespec tc;
        clock_gettime(CLOCK_MONOTONIC, &tc);
        uint64_t now = (uint64_t)tc.tv_sec * 1000000000ull + (uint64_t)tc.tv_nsec;
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

    ioctl(fd, CSI_IOC_JTAG_RELEASE_LEASE);
    close(fd);
    return NULL;
}

static void publish_json_frame(const char *json_str, size_t len)
{
    size_t need = sizeof(pg_hdr_t) + len;
    uint8_t *buf = malloc(need);
    if (!buf) return;

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
    h->fps      = 0.0f;
    h->seq      = g_seq++;

    memcpy(buf + sizeof(pg_hdr_t), json_str, len);
    server_publish(SERVER_SLOT_WIFI, buf, need);
    free(buf);
}

#define LINE_BUF_MAX (128 * 1024)

static void *reader_main(void *arg)
{
    int fd = (int)(intptr_t)arg;
    char *buf = malloc(LINE_BUF_MAX);
    if (!buf) {
        set_err("no wifi buffer");
        atomic_store(&g_dead, 1);
        return NULL;
    }

    size_t pos = 0;

    while (!atomic_load(&g_stop)) {
        struct pollfd p = { .fd = fd, .events = POLLIN };
        int pr = poll(&p, 1, 200);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (pr == 0) continue;

        ssize_t n = read(fd, buf + pos, LINE_BUF_MAX - 1 - pos);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break; // EOF

        pos += (size_t)n;
        buf[pos] = '\0';

        // Scan for complete newline-terminated lines
        char *line_start = buf;
        char *nl;
        while ((nl = strchr(line_start, '\n')) != NULL) {
            *nl = '\0';
            size_t line_len = nl - line_start;
            // Trim trailing \r if any
            if (line_len > 0 && line_start[line_len - 1] == '\r') {
                line_start[line_len - 1] = '\0';
                line_len--;
            }

            if (line_len > 2 && line_start[0] == '{') {
                publish_json_frame(line_start, line_len);
            }

            line_start = nl + 1;
        }

        // Shift remaining partial line to front
        size_t leftover = (buf + pos) - line_start;
        if (leftover > 0 && line_start != buf) {
            memmove(buf, line_start, leftover);
        }
        pos = leftover;
        if (pos >= LINE_BUF_MAX - 1) {
            // Buffer full with no newline: discard
            pos = 0;
        }
    }

    free(buf);
    atomic_store(&g_dead, 1);
    return NULL;
}

int wifi_pipeline_start(double freq_mhz)
{
    wifi_pipeline_stop();
    set_err("");
    atomic_store(&g_dead, 0);
    atomic_store(&g_stop, 0);
    aim_store(0, 0, 0);

    const char *band = NULL;
    g_mhz = wifi_snap_mhz(freq_mhz, &g_ch, &band);

    int pipe_fds[2];
    if (pipe2(pipe_fds, O_CLOEXEC) != 0) {
        set_err("pipe error");
        return -1;
    }

    char freq_str[32];
    snprintf(freq_str, sizeof(freq_str), "%.1f", g_mhz);

    char *argv[] = {
        "/usr/bin/quadrf-wifi-rx",
        "--freq", freq_str,
        "--stdout-json",
        "--skip-lo-retune",
        NULL
    };

    if (spawn_fd(argv, -1, pipe_fds[1], &g_wifi_pid) != 0) {
        close(pipe_fds[0]);
        close(pipe_fds[1]);
        set_err("spawn quadrf-wifi-rx failed");
        return -1;
    }

    // Close write end in parent
    close(pipe_fds[1]);
    g_fd = pipe_fds[0];

    if (pthread_create(&g_reader, NULL, reader_main, (void *)(intptr_t)g_fd) != 0) {
        reap();
        close(g_fd);
        g_fd = -1;
        set_err("reader thread failed");
        return -1;
    }
    g_reader_alive = 1;

    if (pthread_create(&g_phase_th, NULL, phase_main, NULL) == 0) {
        g_phase_alive = 1;
    }

    fprintf(stderr, "phasegaze: wifi pipeline started on %.1f MHz (ch %d %s, pid %d)\n",
            g_mhz, g_ch, band ? band : "", (int)g_wifi_pid);
    return 0;
}

void wifi_pipeline_stop(void)
{
    atomic_store(&g_stop, 1);
    reap();

    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }

    if (g_phase_alive) {
        pthread_join(g_phase_th, NULL);
        g_phase_alive = 0;
    }

    if (g_reader_alive) {
        pthread_join(g_reader, NULL);
        g_reader_alive = 0;
    }
    atomic_store(&g_dead, 1);
}
