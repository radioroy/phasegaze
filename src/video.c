// video.c — NTSC picture for the PhaseGaze tab.
//
// The sweep holds /dev/csi_stream0 with 4-channel interleave and a 20 MHz
// front-end. The decoder wants the other setup, which quadrf-ntsc applies
// before it starts: all four antennas summed (interleave 0), 12 MHz analog
// bandwidth, AGC at -14 dBFS, LO parked. Soapy then resamples the
// 149.5 Msps CSI line to 8x the NTSC color subcarrier (28.636 Msps).
//
// quadrf-ntsc-demod writes 640x480 YUYV at 59.94 fps (~37 MB/s). That stays
// on the board. ffmpeg turns it into 30 fps JPEG; the websocket ships
// those. This unit's fallback path is its own AP, so the frame is 480x360
// rather than the full raster.
//
// The LO is already locked by csi_dev_set_lo. Do not pass --freq: Soapy's
// setFrequency runs the MAX2851 automatic VCO search, and that search
// does not finish, so the PLL sits unlocked and the demod sees noise.
//
// The demod blocks inside SoapySDR::readStream. Closing the YUV pipe does
// not deliver SIGPIPE, and the process keeps the CSI node. Stop signals
// the demod pid.

#define _GNU_SOURCE
#include "video.h"

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

#include "pg_stream.h"
#include "server.h"

extern char **environ;

#define JPG_MAX (512 * 1024)

static pthread_mutex_t g_err_mtx = PTHREAD_MUTEX_INITIALIZER;
static char            g_err[80];
static atomic_int      g_stop;
static atomic_int      g_dead;
static pid_t           g_demod;
static pid_t           g_ff;
static int             g_fd = -1;
static pthread_t       g_reader;
static int             g_reader_alive;
static double          g_mhz;
static uint32_t        g_seq;
static uint32_t        g_frames;
static atomic_int      g_want_mhz;
static pthread_t       g_steer;
static int             g_steer_alive;

/* Same carriers as web/js/ntsc.js. A hemisphere hit is an FFT bin, not
 * the channel. 12 MHz is inside the FM skirt and short of the next
 * 20 MHz-spaced carrier, so a click on the blob lands on a channel and
 * a typed frequency outside the FPV list is left alone. */
static const int k_ntsc_mhz[] = {
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725,
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866,
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945,
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,
};

double video_snap_mhz(double mhz)
{
    int best = 0;
    double bd = 13.0;
    for (size_t i = 0; i < sizeof k_ntsc_mhz / sizeof k_ntsc_mhz[0]; i++) {
        double d = fabs((double)k_ntsc_mhz[i] - mhz);
        if (d < bd) {
            bd = d;
            best = k_ntsc_mhz[i];
        }
    }
    return bd <= 12.0 ? (double)best : mhz;
}

static void set_err(const char *s)
{
    pthread_mutex_lock(&g_err_mtx);
    snprintf(g_err, sizeof g_err, "%s", s ? s : "");
    pthread_mutex_unlock(&g_err_mtx);
    if (s && s[0])
        fprintf(stderr, "phasegaze: video: %s\n", s);
}

void video_pipeline_error(char *dst, size_t n)
{
    if (!dst || n == 0) return;
    pthread_mutex_lock(&g_err_mtx);
    snprintf(dst, n, "%s", g_err);
    pthread_mutex_unlock(&g_err_mtx);
}

int video_pipeline_dead(void)
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
    /* PhaseGaze's listen socket is not CLOEXEC. Leave it open in the
     * child and a restart cannot bind the port. */
    if (rc == 0)
        rc = posix_spawn_file_actions_addclosefrom_np(&fa, STDERR_FILENO + 1);
    if (rc == 0)
        rc = posix_spawn(pid, argv[0], &fa, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    return rc;
}

static void reap(void)
{
    if (g_demod <= 0 && g_ff <= 0) return;
    if (g_demod > 0) kill(g_demod, SIGTERM);
    if (g_ff > 0) kill(g_ff, SIGTERM);
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < 25; i++) {
            int st;
            if (g_demod > 0 && waitpid(g_demod, &st, WNOHANG) == g_demod)
                g_demod = 0;
            if (g_ff > 0 && waitpid(g_ff, &st, WNOHANG) == g_ff)
                g_ff = 0;
            if (g_demod <= 0 && g_ff <= 0) return;
            usleep(20000);
        }
        if (g_demod > 0) kill(g_demod, SIGKILL);
        if (g_ff > 0) kill(g_ff, SIGKILL);
    }
    g_demod = 0;
    g_ff = 0;
}

static uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void publish_jpeg(const uint8_t *jpg, size_t n, float fps)
{
    uint8_t frame[sizeof(pg_hdr_t) + 256];
    /* Small frames are the common case. A noisy analog picture can land
     * past this; fall back to a heap buffer rather than dropping it. */
    uint8_t *buf = frame;
    uint8_t *heap = NULL;
    size_t need = sizeof(pg_hdr_t) + n;
    if (need > sizeof frame) {
        heap = malloc(need);
        if (!heap) return;
        buf = heap;
    }
    pg_hdr_t *h = (pg_hdr_t *)buf;
    h->magic = PG_MAGIC;
    h->version = PG_VERSION;
    h->type = PG_FRAME_VIDEO;
    h->count = (uint32_t)n;
    h->lo_start = (float)g_mhz;
    h->lo_end = 0;
    h->fps = fps;
    h->seq = g_seq++;
    h->reserved = 0;
    memcpy(buf + sizeof(pg_hdr_t), jpg, n);
    server_publish(SERVER_SLOT_VIDEO, buf, need);
    free(heap);
}

static void *reader_main(void *arg)
{
    int fd = (int)(intptr_t)arg;
    uint8_t *acc = malloc(JPG_MAX);
    if (!acc) {
        set_err("no picture");
        atomic_store(&g_dead, 1);
        return NULL;
    }
    size_t len = 0;
    float fps = 0;
    uint32_t fps_n = 0;
    uint64_t fps_t0 = 0;
    uint64_t log_t = mono_ns();

    while (!atomic_load(&g_stop)) {
        if (len >= JPG_MAX) {
            len = 0;
        }
        struct pollfd p = { .fd = fd, .events = POLLIN };
        int pr = poll(&p, 1, 200);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (pr == 0) continue;
        ssize_t n = read(fd, acc + len, JPG_MAX - len);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;
        len += (size_t)n;

        size_t off = 0;
        while (off + 4 < len) {
            if (!(acc[off] == 0xFF && acc[off + 1] == 0xD8)) {
                off++;
                continue;
            }
            size_t i = off + 2;
            int found = 0;
            for (; i + 1 < len; i++) {
                if (acc[i] == 0xFF && acc[i + 1] == 0xD9) {
                    found = 1;
                    break;
                }
            }
            if (!found) break;
            size_t njpg = (i + 2) - off;
            if (njpg >= 64 && njpg < JPG_MAX) {
                uint64_t t = mono_ns();
                /* Several JPEGs can sit in one read. Gap timing then
                 * reports hundreds of fps for a 30 fps stream. */
                if (!fps_t0)
                    fps_t0 = t;
                fps_n++;
                if (t - fps_t0 >= 1000000000ull) {
                    fps = (float)fps_n * 1e9f / (float)(t - fps_t0);
                    fps_n = 0;
                    fps_t0 = t;
                }
                publish_jpeg(acc + off, njpg, fps);
                if (g_frames == 0 || t - log_t > 5000000000ull) {
                    fprintf(stderr, "phasegaze: video %.1f MHz  %.1f fps  %zu B\n",
                            g_mhz, (double)fps, njpg);
                    log_t = t;
                }
                g_frames++;
            }
            off = i + 2;
        }
        if (off && off <= len) {
            memmove(acc, acc + off, len - off);
            len -= off;
        }
    }

    if (!atomic_load(&g_stop)) {
        set_err(g_frames ? "picture lost" : "no picture");
        atomic_store(&g_dead, 1);
    }
    free(acc);
    return NULL;
}

/* freq_offset is the discriminator mean, MHz, positive when the carrier
 * sits above the LO. Two samples that agree, and only while sync is
 * locked, so snow's ~0.3 MHz bias and a one-frame APL kick do not walk
 * the LO. One step, at most 4 MHz from where this pipeline started. */
static int read_status(int *sync, double *snr, double *off, double *wall)
{
    FILE *f = fopen("/dev/shm/quadrf-ntsc-status", "r");
    if (!f)
        f = fopen("/tmp/quadrf-ntsc-status", "r");
    if (!f)
        return -1;
    char line[128];
    *sync = 0;
    *snr = 0;
    *off = 0;
    *wall = 0;
    while (fgets(line, sizeof line, f)) {
        int iv = 0;
        double dv = 0;
        if (sscanf(line, "sync_locked=%d", &iv) == 1)
            *sync = iv;
        else if (sscanf(line, "snr=%lf", &dv) == 1)
            *snr = dv;
        else if (sscanf(line, "freq_offset=%lf", &dv) == 1)
            *off = dv;
        else if (sscanf(line, "wall=%lf", &dv) == 1)
            *wall = dv;
    }
    fclose(f);
    return 0;
}

static void *steer_main(void *arg)
{
    (void)arg;
    double origin = g_mhz;
    double prev = 0, prev_wall = 0;
    int have = 0;
    for (int i = 0; i < 16 && !atomic_load(&g_stop); i++) {
        usleep(200000);
        int sync = 0;
        double snr = 0, off = 0, wall = 0;
        if (read_status(&sync, &snr, &off, &wall) != 0)
            continue;
        /* First diag interval is still the acquire transient. */
        if (wall < 0.8)
            continue;
        if (!sync || snr < 3.0 || fabs(off) > 6.0) {
            have = 0;
            continue;
        }
        if (fabs(off) < 0.8)
            return NULL;
        if (have && wall > prev_wall + 0.2 &&
            copysign(1.0, off) == copysign(1.0, prev) && fabs(off - prev) < 0.4) {
            int neu = (int)lround(origin + off);
            int cur = (int)lround(origin);
            if (neu >= 4900 && neu <= 6000 && neu != cur && abs(neu - cur) <= 4)
                atomic_store(&g_want_mhz, neu);
            return NULL;
        }
        prev = off;
        prev_wall = wall;
        have = 1;
    }
    return NULL;
}

int video_steer_take(double *mhz)
{
    int w = atomic_exchange(&g_want_mhz, 0);
    if (w < 1000 || !mhz)
        return 0;
    *mhz = (double)w;
    return 1;
}

int video_pipeline_start(double freq_mhz)
{
    atomic_store(&g_stop, 0);
    atomic_store(&g_dead, 0);
    atomic_store(&g_want_mhz, 0);
    g_frames = 0;
    g_mhz = freq_mhz;
    set_err("");
    /* The previous demod's status file still says "locked, offset 0"
     * until this one rewrites it. A steer that trusts that parks on
     * the wrong LO. */
    unlink("/dev/shm/quadrf-ntsc-status");
    unlink("/tmp/quadrf-ntsc-status");

    int yuv[2] = {-1, -1};
    int jpg[2] = {-1, -1};
    int devnull = -1;
    if (pipe2(yuv, O_CLOEXEC) != 0 || pipe2(jpg, O_CLOEXEC) != 0) {
        set_err("no picture");
        goto fail;
    }
    devnull = open("/dev/null", O_RDWR | O_CLOEXEC);
    if (devnull < 0) {
        set_err("no picture");
        goto fail;
    }

    char *demod_argv[] = {
        "/usr/bin/quadrf-ntsc-demod",
        "--bypass_iir", "true",
        "--disc", "atan2",
        "--no_deemph",
        "--read_samps", "65536",
        "--args", "numBuffers=2,bufferLength=65536",
        "--sat", "1.0",
        "--stdout",
        NULL
    };
    /* 480x360 qscale 8 at 30 fps. The demod's raster is 59.94 fps; shipping
     * every field is ~2x the bytes for a window. Snow is ~50 KB, a real
     * picture much less, so 30 fps stays a few megabits on the AP. */
    char *ff_argv[] = {
        "/usr/bin/ffmpeg",
        "-hide_banner", "-loglevel", "error",
        "-fflags", "nobuffer",
        "-probesize", "32",
        "-analyzeduration", "0",
        "-f", "rawvideo",
        "-pix_fmt", "yuyv422",
        "-s", "640x480",
        "-framerate", "60000/1001",
        "-i", "pipe:0",
        "-an",
        "-threads", "1",
        "-vf", "fps=30,scale=480:360:flags=fast_bilinear,format=yuvj420p",
        "-q:v", "8",
        "-flush_packets", "1",
        "-f", "mjpeg",
        "pipe:1",
        NULL
    };

    if (spawn_fd(demod_argv, devnull, yuv[1], &g_demod) != 0 || g_demod <= 0) {
        g_demod = 0;
        set_err("no decoder");
        goto fail;
    }
    if (spawn_fd(ff_argv, yuv[0], jpg[1], &g_ff) != 0 || g_ff <= 0) {
        g_ff = 0;
        set_err("no picture");
        goto fail;
    }
    close(devnull);
    close(yuv[0]);
    close(yuv[1]);
    close(jpg[1]);
    devnull = yuv[0] = yuv[1] = jpg[1] = -1;
    g_fd = jpg[0];

    if (pthread_create(&g_reader, NULL, reader_main, (void *)(intptr_t)g_fd) != 0) {
        set_err("no picture");
        goto fail;
    }
    g_reader_alive = 1;
    if (pthread_create(&g_steer, NULL, steer_main, NULL) != 0)
        g_steer_alive = 0;
    else
        g_steer_alive = 1;
    return 0;

fail:
    if (g_reader_alive) {
        atomic_store(&g_stop, 1);
        pthread_join(g_reader, NULL);
        g_reader_alive = 0;
    }
    reap();
    if (devnull >= 0) close(devnull);
    if (yuv[0] >= 0) close(yuv[0]);
    if (yuv[1] >= 0) close(yuv[1]);
    if (jpg[0] >= 0) close(jpg[0]);
    if (jpg[1] >= 0) close(jpg[1]);
    g_fd = -1;
    atomic_store(&g_stop, 0);
    return -1;
}

void video_pipeline_stop(void)
{
    atomic_store(&g_stop, 1);
    if (g_steer_alive) {
        pthread_join(g_steer, NULL);
        g_steer_alive = 0;
    }
    atomic_store(&g_want_mhz, 0);
    if (g_reader_alive) {
        pthread_join(g_reader, NULL);
        g_reader_alive = 0;
    }
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    reap();
    atomic_store(&g_dead, 0);
    atomic_store(&g_stop, 0);
}
