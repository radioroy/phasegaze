#ifndef VIDEO_H
#define VIDEO_H

#include <stddef.h>

/* Picture path. The caller has already closed /dev/csi_stream0.
 * quadrf-ntsc-demod opens that node through Soapy; a second mapping
 * of the CSI ring splits samples out from under both. */

/* Nearest catalog carrier within 12 MHz. Farther away, mhz unchanged. */
double video_snap_mhz(double mhz);

int  video_pipeline_start(double freq_mhz);
void video_pipeline_stop(void);
int  video_pipeline_dead(void);
void video_pipeline_error(char *dst, size_t n);

/* One-shot LO correction from the demod's discriminator, in MHz.
 * 0 if it has not asked. Cleared on read. */
int  video_steer_take(double *mhz);

#endif
