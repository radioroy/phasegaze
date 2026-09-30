#!/bin/sh
# Build the perf tools on the Pi with the production compile flags.
# Output goes to $OUT (default ~/pgperf).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
OUT=${OUT:-$HOME/pgperf}
mkdir -p "$OUT"
F="-O3 -ffast-math -mcpu=cortex-a76 -mtune=cortex-a76 -Wall -Wextra"
I="-I$ROOT/src -I$ROOT/include -I$HERE"

gcc -O2 $I "$HERE/pg_capture.c" "$ROOT/src/csi_dev.c" -o "$OUT/pg_capture" -lm -lpthread
gcc -O2 $I "$HERE/pg_rxcap.c" "$ROOT/src/tuner.c" -o "$OUT/pg_rxcap" -lm -lpthread
gcc -O2 $I "$HERE/pg_ringcopy.c" "$ROOT/src/csi_dev.c" -o "$OUT/pg_ringcopy" -lm -lpthread
gcc $F $I "$HERE/pg_bench.c" "$HERE/ref_hop.c" "$HERE/ref_dsp_tu.c" \
    -o "$OUT/pg_bench_ref" -lfftw3f -lm
gcc $F $I -DHAVE_NEW "$HERE/pg_bench.c" "$HERE/ref_hop.c" "$HERE/ref_dsp_tu.c" \
    "$ROOT/src/hop.c" "$ROOT/src/dsp.c" -o "$OUT/pg_bench" -lfftw3f -lm
gcc $F $I "$HERE/pg_stage.c" "$ROOT/src/hop.c" "$ROOT/src/dsp.c" \
    -o "$OUT/pg_stage" -lfftw3f -lm
gcc $F $I "$HERE/pg_gates.c" "$ROOT/src/hop.c" "$ROOT/src/dsp.c" \
    -o "$OUT/pg_gates" -lfftw3f -lm
echo "built in $OUT"
