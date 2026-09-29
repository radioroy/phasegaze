#!/bin/sh
# Run on the Pi. Starts BIN like quadrf-phasegaze.service (dietpi, RTPRIO 60,
# same working dir for the FFTW wisdom) and probes each plan over loopback.
#
#   run_matrix.sh BIN OUT.jsonl [SECS]
set -e
BIN=${1:?binary}
OUT=${2:?output jsonl}
SECS=${3:-20}
HERE=$(cd "$(dirname "$0")" && pwd)
WEB=$(cd "$HERE/../../web" && pwd)
UNIT=pg-bench

sudo systemctl stop quadrf-phasegaze 2>/dev/null || true
sudo systemctl stop $UNIT 2>/dev/null || true
sudo systemctl reset-failed $UNIT 2>/dev/null || true
sudo systemd-run --unit $UNIT -p User=dietpi -p Group=dietpi \
    -p LimitRTPRIO=60 -p WorkingDirectory=/var/lib/quadrf/demos \
    -p KillSignal=SIGINT "$BIN" --port 8001 --web "$WEB" >/dev/null
sleep 4
PID=$(systemctl show -p MainPID --value $UNIT)
[ "$PID" -gt 0 ] || { echo "no pid"; journalctl -u $UNIT -n 30; exit 1; }

: > "$OUT"
for spec in 0 1; do
    for plan in full wifiall wifi3 single; do
        t0=$(cat /sys/class/thermal/thermal_zone0/temp)
        python3 "$HERE/pg_probe.py" --plan $plan --spectrum $spec \
            --secs "$SECS" --pid "$PID" --label "$(basename "$BIN")" |
        python3 -c "import json,sys; j=json.loads(sys.stdin.read()); j['temp_c']=$t0/1000; print(json.dumps(j))" >> "$OUT"
        tail -n1 "$OUT"
    done
done
sudo systemctl stop $UNIT
