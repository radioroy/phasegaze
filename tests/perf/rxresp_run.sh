#!/bin/sh
# Receiver-response and hop-hysteresis capture matrix (pg_rxcap).
# Run on the unit with quadrf-phasegaze stopped:
#   sudo sh rxresp_run.sh OUTDIR
# The VCO seed file is restored afterwards so off-grid LOs do not stick.
set -e
OUT=${1:-$HOME/rxcap}
CAP=${CAP:-$HOME/pgperf/pg_rxcap}
SEEDS=/var/lib/quadrf/demos/max2851_vco_seeds.txt
mkdir -p "$OUT"
[ -f "$SEEDS" ] && cp "$SEEDS" "$OUT/seeds.bak"
trap '[ -f "$OUT/seeds.bak" ] && cp "$OUT/seeds.bak" "$SEEDS"' EXIT

# 24 stratified-random LOs over 4910..6090 (python random.seed(20260930)).
RLO="4932.8 4990.2 5019.3 5072.6 5131.3 5202.1 5236.7 5294.6 5305.8 5387.3
5450.8 5473.8 5542.7 5577.9 5599.7 5649.5 5739.3 5754.6 5821.2 5847.9
5911.8 5958.3 6033.1 6079.3"
GAINS=${GAINS:-0,10,20,30,40,45,50,55,60,63}
$CAP "$OUT/dwell.bin" dwell ${DWELL_SPANS:-12} $GAINS $RLO

# Hop plans. 5765 is the probe: the 40 MHz comb tone at 5760 sits at
# -5 MHz there, a fixed-RF coherent reference for inter-channel phase.
N=${HOP_SPANS:-1800}
for G in ${HOP_GAINS:-45 20}; do
    $CAP "$OUT/hop_static_g$G.bin" hop $N $G 5765
    $CAP "$OUT/hop_up20_g$G.bin" hop $N $G 5745 5765 5785 5805 5825 5845 5865 5885
    $CAP "$OUT/hop_dn20_g$G.bin" hop $N $G 5885 5865 5845 5825 5805 5785 5765 5745
    $CAP "$OUT/hop_up40_g$G.bin" hop $N $G 5725 5765 5805 5845
    $CAP "$OUT/hop_jump_g$G.bin" hop $N $G 4910 5765 6090 5765
    $CAP "$OUT/hop_rand_g$G.bin" hop $N $G 5805 5745 5885 5765 5825 5865 5785 5845
    $CAP "$OUT/hop_wifi_g$G.bin" hop $N $G \
        5180 5200 5220 5240 5260 5280 5300 5320 \
        5500 5520 5540 5560 5580 5600 5620 5640 5660 5680 5700 5720 \
        5745 5765 5785 5805 5825 5845 5865 5885
done
ls -la "$OUT"
