// ref_dsp_tu.c
// The frozen dsp.c as its own translation unit, as the old service built
// it. Keeping it out of ref_hop.c stops GCC from inlining it with constant
// arguments, which the production build never did.
#include "ref_names.h"
#include "ref_dsp.c"
