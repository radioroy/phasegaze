"""Hop hysteresis: compare the same LO reached from different predecessors /
plans. Probe LO 5765: 40 MHz comb tone at 5760 lands at -5 MHz (bin -1096),
a fixed-RF coherent reference for inter-channel phase."""
import sys, glob, os
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, '/tmp/rx')
from rxload import load, iq

N = 8192; FS = 37.3726
f = (np.arange(N) - N // 2) * FS / N
keep = np.abs(f) <= 10.0
db = lambda x: 10 * np.log10(np.maximum(x, 1e-12))
root, tag, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
PROBE = 5765.0
KT = N // 2 - 1096            # 5760 MHz comb tone at LO 5765
plans = ['static', 'up20', 'dn20', 'up40', 'jump', 'rand', 'wifi']


def stats(path, lo_sel=PROBE):
    h, b = load(path)
    ok = (h['valid'] == 1) & (h['dup'] == 0) & (np.abs(h['lo'] - lo_sel) < 1e-6)
    # previous span must be tagged too, so prev_lo is real
    prev_ok = np.r_[False, h['valid'][:-1] >= 0]
    ok &= prev_ok
    idx = np.where(ok)[0]
    x = iq(b[idx])                                   # (n,16384,4)
    env = (np.abs(x) ** 2).reshape(len(idx), 256, 64, 4).mean(2)   # 64-sample chunks
    dcc = x.reshape(len(idx), 64, 256, 4).mean(2)                  # 256-sample chunks
    x2 = x[:, N:, :]
    X = np.fft.fftshift(np.fft.fft(x2, axis=1), axes=1)            # rect, as production
    P = (np.abs(X) ** 2).mean(0).T / N
    tone = X[:, KT - 1:KT + 2, :]
    kpk = np.argmax(np.abs(tone).sum(2).mean(0))
    t = tone[:, kpk, :]                                            # (n,4)
    return dict(n=len(idx), prev=h['prev_lo'][idx], env=env.mean(0), envs=env,
                dc=dcc.mean(0), P=P, tone=t, dc2=x2.mean(1))


res = {}
for g in [45, 20]:
    for p in plans:
        fn = f'{root}/hop_{p}_g{g}.bin'
        if not os.path.exists(fn): continue
        s = stats(fn)
        if s['n'] < 10:
            print('skip', fn, s['n']); continue
        res[(g, p)] = s
        t = s['tone']
        ph = np.angle(t[:, 1:] * np.conj(t[:, :1]))                # phi_c0
        R = np.abs(np.exp(1j * ph).mean(0))
        mph = np.degrees(np.angle(np.exp(1j * ph).mean(0)))
        fl = np.median(db(s['P'].sum(0)[keep]))
        print(f'g{g} {p:6s} n={s["n"]:4d} floor {fl:6.2f} dB  tone {db(np.abs(t)**2).mean():5.1f} dB  '
              f'phi10/20/30 {mph[0]:+7.1f} {mph[1]:+7.1f} {mph[2]:+7.1f} deg  R {R[0]:.3f} {R[1]:.3f} {R[2]:.3f}  '
              f'DC0 {np.abs(s["dc2"][:,0]).mean():.3f}', flush=True)
np.save(f'{outdir}/{tag}_hop_summary.npy', {k: {kk: v for kk, v in s.items() if kk in ('n', 'env', 'P', 'tone', 'prev')}
                                            for k, s in res.items()}, allow_pickle=True)

# A. retune transient: power envelope over the full span, per plan (g45), ch-sum.
for g in [45, 20]:
    fig, ax = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    t_us = (np.arange(256) * 64 + 32) / FS
    for p in plans:
        if (g, p) not in res: continue
        e = res[(g, p)]['env'].sum(1)
        ax[0].plot(t_us, db(e), lw=0.8, label=f'{p} (n={res[(g, p)]["n"]})')
        dcm = np.abs(res[(g, p)]['dc']).mean(1)
        ax[1].plot((np.arange(64) * 256 + 128) / FS, dcm, lw=0.8, label=p)
    for a in ax:
        a.axvline(N / FS, color='k', ls='--', lw=1)
        a.grid(alpha=0.3)
    ax[0].set_ylabel('4-ch power, dB LSB$^2$ (64-sample chunks)')
    ax[1].set_ylabel('|DC| mean over ch (LSB, 256-sample chunks)')
    ax[1].set_xlabel('time in span (us); dashed = start of kept half')
    ax[0].legend(ncol=4, fontsize=8)
    fig.suptitle(f'{tag} g{g}: LO {PROBE} span, mean over visits, by hop plan')
    fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_10_transient_g{g}.png', dpi=110); plt.close(fig)

# B. settled PSD difference vs static.
for g in [45, 20]:
    if (g, 'static') not in res: continue
    ref = res[(g, 'static')]['P']
    fig, ax = plt.subplots(figsize=(14, 5))
    for p in plans[1:]:
        if (g, p) not in res: continue
        d = db(res[(g, p)]['P'].sum(0)) - db(ref.sum(0))
        k = np.ones(33) / 33
        ax.plot(f[keep], np.convolve(d, k, 'same')[keep], lw=0.8, label=p)
    ax.axhline(0, color='k', lw=0.5); ax.set_ylim(-3, 3); ax.grid(alpha=0.3); ax.legend(ncol=6)
    ax.set_xlabel('baseband offset (MHz)'); ax.set_ylabel('dB vs static dwell (33-bin smoothed)')
    ax.set_title(f'{tag} g{g}: kept-half PSD at LO {PROBE} by plan minus static (4-ch sum)')
    fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_11_psd_vs_static_g{g}.png', dpi=110); plt.close(fig)

# C. inter-channel phase at the 5760 comb tone, by plan and by predecessor.
fig, ax = plt.subplots(1, 3, figsize=(16, 5), sharey=True)
for g, mk in [(45, 'o'), (20, 's')]:
    for pi, p in enumerate(plans):
        if (g, p) not in res: continue
        t = res[(g, p)]['tone']
        prev = res[(g, p)]['prev']
        for ci in range(3):
            ph = np.degrees(np.angle(t[:, ci + 1] * np.conj(t[:, 0])))
            for pv in np.unique(prev):
                m = prev == pv
                if m.sum() < 5: continue
                z = np.exp(1j * np.radians(ph[m])).mean()
                ax[ci].errorbar(pi + (0.15 if g == 20 else -0.15), np.degrees(np.angle(z)),
                                yerr=np.degrees(np.sqrt(-2 * np.log(max(abs(z), 1e-6)))),
                                fmt=mk, ms=4, capsize=2, alpha=0.8,
                                color='C0' if g == 45 else 'C3')
for ci in range(3):
    ax[ci].set_xticks(range(len(plans))); ax[ci].set_xticklabels(plans, rotation=30)
    ax[ci].set_title(f'phase CH{ci+1} - CH0 at 5760 MHz tone'); ax[ci].grid(alpha=0.3)
ax[0].set_ylabel('deg (one marker per predecessor LO; bar = circular std)')
fig.suptitle(f'{tag}: inter-channel phase at LO {PROBE} by plan and predecessor (blue g45, red g20)')
fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_12_phase_hysteresis.png', dpi=110); plt.close(fig)
print('done')
