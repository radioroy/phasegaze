#ifndef WIFI_H
#define WIFI_H

#include <stddef.h>

/* Nearest 802.11 20 MHz carrier within 10 MHz.
 * Returns snapped frequency in MHz (or original mhz if not within 10 MHz of any standard channel).
 * If out_ch is non-NULL, stores channel number (or 0).
 * If out_band is non-NULL, stores band string ("5 GHz", "6 GHz", etc.) */
double wifi_snap_mhz(double mhz, int *out_ch, const char **out_band);

/* Start the quadrf-wifi-rx child process and JSON reader thread. */
int  wifi_pipeline_start(double freq_mhz);
void wifi_pipeline_stop(void);
int  wifi_pipeline_dead(void);
void wifi_pipeline_error(char *dst, size_t n);

#endif /* WIFI_H */
