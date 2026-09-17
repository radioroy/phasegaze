// pg_stream.h
// Wire format for the PhaseGaze WebSocket stream.
//
// Everything little-endian, tightly packed. A frame is one 32-byte header
// followed by `count` payload records.

#ifndef PG_STREAM_H
#define PG_STREAM_H

#include <stdint.h>

#define PG_MAGIC   0x315A4750u   /* "PGZ1" */
#define PG_VERSION 1

enum {
    PG_FRAME_POINTS   = 0,   /* payload: pg_point_t[count] */
    PG_FRAME_SPECTRUM = 1,   /* payload: float[count] avg energy 0..1 */
};

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t count;
    float    lo_start;
    float    lo_end;
    float    fps;
    uint32_t seq;
    uint32_t reserved;
} pg_hdr_t;

/* Direction-cosine point. (u, v) on the unit disk; client derives
 * w = sqrt(1 - u^2 - v^2). 16 bytes per point. */
typedef struct __attribute__((packed)) {
    float u;
    float v;
    float freq_mhz;
    float intensity;
} pg_point_t;

#define PG_SPECTRUM_BIN_MHZ  1.0f
#define PG_SPECTRUM_BINS     1200  /* (6100 - 4900) / 1 */

#endif /* PG_STREAM_H */
