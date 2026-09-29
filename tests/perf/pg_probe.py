#!/usr/bin/env python3
"""WebSocket probe for quadrf-phasegaze. Stdlib only so it runs on the Pi.

Sets a sweep plan, waits for it to settle, then counts what arrives on /ws
for a fixed window: point / spectrum packets, points, sweeps folded into
each packet (header 'reserved'), bytes, inter-packet gaps, and the deltas
of the device's own state counters. Optionally samples /proc CPU time of a
local pid (use when running on the Pi).

    pg_probe.py --host 127.0.0.1 --port 8001 --plan full --secs 20
"""

import argparse
import base64
import json
import os
import socket
import struct
import sys
import time

PG_MAGIC = 0x315A4750
HDR = struct.Struct('<IHHIfffII')

PLANS = {
    'full':   {'lo_start': 4900, 'lo_end': 6100, 'bands': []},
    # ch 36, 100, 149: three separated 20 MHz hops
    'wifi3':  {'lo_start': 5170, 'lo_end': 5895,
               'bands': [[5170, 5190], [5490, 5510], [5735, 5755]]},
    'wifiall': {'lo_start': 5170, 'lo_end': 5895,
                'bands': [[5170, 5330], [5490, 5730], [5735, 5895]]},
    'single': {'lo_start': 5170, 'lo_end': 5895, 'bands': [[5735, 5755]]},
}


class Ws:
    def __init__(self, host, port, path='/ws', timeout=5.0):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f'GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\n'
               'Upgrade: websocket\r\nConnection: Upgrade\r\n'
               f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n')
        self.s.sendall(req.encode())
        self.buf = b''
        while b'\r\n\r\n' not in self.buf:
            chunk = self.s.recv(4096)
            if not chunk:
                raise ConnectionError('handshake closed')
            self.buf += chunk
        head, self.buf = self.buf.split(b'\r\n\r\n', 1)
        if b' 101 ' not in head.split(b'\r\n')[0]:
            raise ConnectionError(head.decode(errors='replace'))

    def _need(self, n):
        while len(self.buf) < n:
            chunk = self.s.recv(1 << 20)
            if not chunk:
                raise ConnectionError('closed')
            self.buf += chunk

    def recv(self):
        """Return (opcode, payload bytes, wire bytes)."""
        self._need(2)
        b0, b1 = self.buf[0], self.buf[1]
        op = b0 & 0x0F
        n = b1 & 0x7F
        off = 2
        if n == 126:
            self._need(4)
            n = struct.unpack('>H', self.buf[2:4])[0]
            off = 4
        elif n == 127:
            self._need(10)
            n = struct.unpack('>Q', self.buf[2:10])[0]
            off = 10
        if b1 & 0x80:
            off += 4    # server frames are never masked, but be safe
        self._need(off + n)
        payload = self.buf[off:off + n]
        self.buf = self.buf[off + n:]
        return op, payload, off + n

    def send_text(self, text):
        data = text.encode()
        mask = os.urandom(4)
        n = len(data)
        if n < 126:
            hdr = bytes([0x81, 0x80 | n])
        elif n < 65536:
            hdr = bytes([0x81, 0x80 | 126]) + struct.pack('>H', n)
        else:
            hdr = bytes([0x81, 0x80 | 127]) + struct.pack('>Q', n)
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
        self.s.sendall(hdr + mask + masked)

    def set(self, **kw):
        self.send_text(json.dumps({'type': 'set', **kw}))


def proc_cpu(pid):
    """(process ticks, {tid: (comm, ticks)})"""
    def ticks(path):
        with open(path) as f:
            s = f.read()
        comm = s[s.index('(') + 1:s.rindex(')')]
        rest = s[s.rindex(')') + 2:].split()
        return comm, int(rest[11]) + int(rest[12])
    _, tot = ticks(f'/proc/{pid}/stat')
    thr = {}
    for tid in os.listdir(f'/proc/{pid}/task'):
        try:
            thr[tid] = ticks(f'/proc/{pid}/task/{tid}/stat')
        except OSError:
            pass
    return tot, thr


TIS = '/sys/devices/system/cpu/cpu0/cpufreq/stats/time_in_state'


def time_in_state():
    """{kHz: 10 ms ticks} for the shared A76 clock domain, or None."""
    try:
        with open(TIS) as f:
            return {int(a): int(b) for a, b in (ln.split() for ln in f)}
    except OSError:
        return None


def mean_khz(t0, t1):
    if not t0 or not t1:
        return None
    d = {k: t1[k] - t0.get(k, 0) for k in t1}
    tot = sum(d.values())
    return sum(k * v for k, v in d.items()) / tot if tot else None


def pct(vals, p):
    if not vals:
        return 0.0
    v = sorted(vals)
    return v[min(len(v) - 1, int(p / 100.0 * len(v)))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8001)
    ap.add_argument('--plan', default='full', choices=sorted(PLANS))
    ap.add_argument('--spectrum', type=int, default=0)
    ap.add_argument('--density', type=float, default=1.0)
    ap.add_argument('--gain', type=int, default=45)
    ap.add_argument('--settle', type=float, default=3.0)
    ap.add_argument('--secs', type=float, default=20.0)
    ap.add_argument('--pid', type=int, default=0)
    ap.add_argument('--label', default='')
    a = ap.parse_args()

    ws = Ws(a.host, a.port)
    ws.set(gain=a.gain, output_fraction=a.density, spectrum=a.spectrum,
           **PLANS[a.plan])

    state0 = state1 = None
    t_end_settle = time.monotonic() + a.settle
    while time.monotonic() < t_end_settle:
        op, pl, _ = ws.recv()
        if op == 1:
            j = json.loads(pl)
            if j.get('type') == 'state':
                state0 = j

    cpu0 = proc_cpu(a.pid) if a.pid else None
    tis0 = time_in_state() if a.pid else None
    t0 = time.monotonic()
    t_end = t0 + a.secs
    n_pts_pkt = n_spec_pkt = 0
    points = sweeps = 0
    wire = 0
    fps_hdr = []
    gaps = []
    pkt_pts = []
    pkt_sweeps = []
    last_t = None
    seq_prev = None
    while True:
        now = time.monotonic()
        if now >= t_end:
            break
        op, pl, nb = ws.recv()
        now = time.monotonic()
        wire += nb
        if op == 1:
            j = json.loads(pl)
            if j.get('type') == 'state':
                if state0 is None:
                    state0 = j
                state1 = j
            continue
        if op != 2 or len(pl) < HDR.size:
            continue
        magic, ver, typ, count, lo0, lo1, fps, seq, resv = HDR.unpack_from(pl)
        if magic != PG_MAGIC:
            continue
        if typ == 0:
            n_pts_pkt += 1
            points += count
            sweeps += resv
            pkt_pts.append(count)
            pkt_sweeps.append(resv)
            fps_hdr.append(fps)
            if last_t is not None:
                gaps.append((now - last_t) * 1e3)
            last_t = now
        elif typ == 1:
            n_spec_pkt += 1
        seq_prev = seq
    dt = time.monotonic() - t0
    cpu1 = proc_cpu(a.pid) if a.pid else None
    tis1 = time_in_state() if a.pid else None

    def d(key, sub):
        if not state0 or not state1:
            return None
        return state1[sub][key] - state0[sub][key]

    out = {
        'label': a.label, 'plan': a.plan, 'spectrum': a.spectrum,
        'density': a.density, 'gain': a.gain, 'secs': round(dt, 2),
        'point_pkts_s': round(n_pts_pkt / dt, 1),
        'spec_pkts_s': round(n_spec_pkt / dt, 1),
        'sweeps_s_rx': round(sweeps / dt, 1),
        'hdr_fps_mean': round(sum(fps_hdr) / len(fps_hdr), 1) if fps_hdr else 0,
        'points_s': round(points / dt),
        'pts_per_pkt_p50': pct(pkt_pts, 50),
        'pts_per_pkt_max': max(pkt_pts) if pkt_pts else 0,
        'sweeps_per_pkt_p50': pct(pkt_sweeps, 50),
        'sweeps_per_pkt_max': max(pkt_sweeps) if pkt_sweeps else 0,
        'gap_ms_p50': round(pct(gaps, 50), 2),
        'gap_ms_p99': round(pct(gaps, 99), 2),
        'gap_ms_max': round(max(gaps), 2) if gaps else 0,
        'kbytes_s': round(wire / dt / 1024, 1),
    }
    if state0 and state1:
        span_s = dt
        out.update({
            'dsp_frames_s': round(d('frames', 'dsp') / span_s, 1),
            'dsp_hops_s': round(d('hops', 'dsp') / span_s, 1),
            'dsp_dropped_s': round(d('dropped', 'dsp') / span_s, 1),
            'dsp_invalid_s': round(d('invalid', 'dsp') / span_s, 1),
            'dsp_dup_s': round(d('dup', 'dsp') / span_s, 1),
            'dsp_untagged_s': round(d('untagged', 'dsp') / span_s, 1),
            'tuner_spans_s': round(d('spans', 'tuner') / span_s, 1),
            'tuner_deferred': d('deferred', 'tuner'),
            'tuner_late_land': d('late_land', 'tuner'),
            'tuner_resyncs': d('resyncs', 'tuner'),
            'adc_peak': state1.get('adc_peak'),
            'adc_rms': state1.get('adc_rms'),
            'state_points_last': state1.get('points'),
        })
    if cpu0 and cpu1:
        hz = os.sysconf('SC_CLK_TCK')
        out['cpu_pct_total'] = round(100.0 * (cpu1[0] - cpu0[0]) / hz / dt, 1)
        # Thread creation order: main, tuner, web server, then the DSP workers.
        per = []
        for tid in sorted(cpu1[1], key=int):
            comm, t1 = cpu1[1][tid]
            t0_ = cpu0[1].get(tid, (comm, 0))[1]
            per.append(round(100.0 * (t1 - t0_) / hz / dt, 1))
        out['cpu_pct_threads'] = per
        khz = mean_khz(tis0, tis1)
        if khz:
            out['cpu_mhz_mean'] = round(khz / 1000)
            wk = sum(per[-2:]) if len(per) >= 5 else 0
            hops = out.get('dsp_hops_s') or 0
            if hops:
                out['worker_kcyc_per_hop'] = round(wk / 100.0 * khz / hops, 1)
    print(json.dumps(out))
    sys.stdout.flush()


if __name__ == '__main__':
    main()
