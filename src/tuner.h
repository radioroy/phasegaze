// tuner.h
// Span-synchronous LO sequencer. One RT thread retunes once per DMA span at a
// fixed offset from the span's publication, so the synthesizer transition
// reaches the ADC early in the span two ahead. Every span is tagged with the
// LO that was stable over its second half; DSP workers look tags up by ring
// offset, so worker latency never affects which LO a block is labeled with.

#ifndef TUNER_H
#define TUNER_H

#include <stdint.h>
#include "csi_dev.h"

#define TUNER_MAX_HOPS 128

typedef struct {
    double lo[TUNER_MAX_HOPS];
    int    n;
    float  lo_start, lo_end;    /* keep-band edges for the frame header */
} tuner_plan_t;

typedef struct {
    uint32_t raw_end;           /* ring offset just past the span's last byte */
    uint32_t sweep;             /* bumps at hop 0, never reset */
    uint16_t hop, nhops;
    uint8_t  valid;             /* second half saw one settled LO */
    uint8_t  dup;               /* repeat of the previous span's hop */
    double   lo;
    float    lo_start, lo_end;
} span_tag_t;

typedef struct {
    uint64_t spans;             /* slots handled */
    uint64_t retunes;
    uint64_t deferred;          /* woke past the deadline, held the LO */
    uint64_t late_land;         /* retune landed in a kept half */
    uint64_t resyncs;
    uint64_t detections;
    double   write_us;          /* last set_lo duration */
    double   min_late_us;       /* publication lateness, last window min */
    double   span_us;           /* tracked span period */
    int      rt;                /* got SCHED_FIFO */
} tuner_stats_t;

int  tuner_start(csi_dev_t *d, int cpu, const tuner_plan_t *plan);
void tuner_stop(void);
void tuner_set_plan(const tuner_plan_t *plan);
void tuner_set_gain(int gain);
int  tuner_lookup(uint32_t raw_end, span_tag_t *out);
void tuner_get_stats(tuner_stats_t *s);

#endif /* TUNER_H */
