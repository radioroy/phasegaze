// ref_names.h
// Rename the frozen dsp_* symbols so they link next to the live dsp.c.
#define dsp_cs8_extract_rotate ref_cs8_extract_rotate
#define dsp_power4_log_shifted ref_power4_log_shifted
#define dsp_cfar_topk          ref_cfar_topk
#define dsp_solve_gradient     ref_solve_gradient
#define dsp_peak_t             ref_peak_t
