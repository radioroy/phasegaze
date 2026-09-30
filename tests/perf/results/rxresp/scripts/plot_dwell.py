import sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm

npz, tag, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
d = np.load(npz)
gains = list(d['gains']); los = np.array(d['los'])
sel = [i for i, g in enumerate(gains) if g != 0]
gains = [gains[i] for i in sel]
P = d['P_hann'][sel]; PR = d['P_rect'][sel]; CSD = d['CSD'][sel]
RMS = d['RMS'][sel]; DC = d['DC'][sel]
LNA = d['LNA'][sel]; VGA = d['VGA'][sel]
N = P.shape[-1]; FS = 37.3726
f = (np.arange(N) - N // 2) * FS / N
keep = np.abs(f) <= 10.0
db = lambda x: 10 * np.log10(np.maximum(x, 1e-12))
cols = cm.viridis(np.linspace(0, 1, len(gains)))
chn = ['CH0', 'CH1', 'CH2', 'CH3']

# 1. Baseband receiver response: median over the 24 random LOs.
med = np.median(P, axis=1)                      # (G,4,N)
fig, ax = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
for c in range(4):
    for gi, g in enumerate(gains):
        ax[c].plot(f, db(med[gi, c]), lw=0.6, color=cols[gi],
                   label=f'g{g} (LNA {LNA[gi]} VGA {VGA[gi]})')
    ax[c].set_ylabel(f'{chn[c]} dB LSB$^2$/bin')
    ax[c].axvspan(-10, 10, color='k', alpha=0.04)
    ax[c].grid(alpha=0.3)
ax[0].legend(ncol=5, fontsize=7, loc='upper right')
ax[-1].set_xlabel('baseband offset from LO (MHz); shaded = keep band')
fig.suptitle(f'{tag}: receiver baseband PSD, median over 24 random LOs (Hann 8192)')
fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_01_bb_psd_vs_gain.png', dpi=110); plt.close(fig)

# 2. Gain x baseband heatmap, per channel, normalized to each gain's keep-band median.
fig, ax = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
for c in range(4):
    z = db(med[:, c]) - np.median(db(med[:, c][:, keep]), axis=1)[:, None]
    im = ax[c].imshow(z, aspect='auto', origin='lower', cmap='magma', vmin=-6, vmax=15,
                      extent=[f[0], f[-1], -0.5, len(gains) - 0.5], interpolation='nearest')
    ax[c].set_yticks(range(len(gains))); ax[c].set_yticklabels([f'g{g}' for g in gains], fontsize=7)
    ax[c].set_ylabel(chn[c])
    plt.colorbar(im, ax=ax[c], label='dB over keep-band median')
ax[-1].set_xlabel('baseband offset (MHz)')
fig.suptitle(f'{tag}: receiver response matrix (gain x baseband bin), relative to floor')
fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_02_gain_x_bb.png', dpi=110); plt.close(fig)

# 3. LO x baseband at selected gains: vertical lines are baseband-fixed
#    (receiver), diagonals are RF-fixed (environment, 40 MHz comb).
for g in [20, 45, 60]:
    if g not in gains: continue
    gi = gains.index(g)
    s = P[gi].sum(1)                                  # 4-ch sum (LO,N)
    z = db(s) - np.median(db(s[:, keep]), axis=1)[:, None]
    fig, ax = plt.subplots(figsize=(13, 7))
    ax.imshow(z, aspect='auto', origin='lower', cmap='magma', vmin=-4, vmax=15,
              extent=[f[0], f[-1], -0.5, len(los) - 0.5], interpolation='nearest')
    ax.set_yticks(range(len(los))); ax.set_yticklabels([f'{x:.1f}' for x in los], fontsize=7)
    ax.set_xlabel('baseband offset (MHz)'); ax.set_ylabel('LO (MHz)')
    ax.set_title('4-ch sum PSD vs LO (vertical lines = receiver-fixed)')
    fig.suptitle(f'{tag}: gain {g}, LO x baseband, dB over floor')
    fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_03_lo_x_bb_g{g}.png', dpi=110); plt.close(fig)

# 4. ADC rail RMS vs gain.
fig, ax = plt.subplots(figsize=(9, 5))
rails = ['0I', '0Q', '1I', '1Q', '2I', '2Q', '3I', '3Q']
for r in range(8):
    ax.plot(gains, RMS[:, :, r].mean(1), marker='o', ls='-' if r % 2 == 0 else '--', label=rails[r])
ax.set_yscale('log'); ax.set_xlabel('gain setting (dB)'); ax.set_ylabel('RMS (LSB, CS8)')
ax.grid(alpha=0.3, which='both'); ax.legend(ncol=4)
ax.set_title(f'{tag}: per-rail RMS vs gain, mean over LOs')
fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_04_rms_vs_gain.png', dpi=110); plt.close(fig)

# 5. Spur census: narrow lines in the median-over-LO PSD, keep band, per channel.
from scipy.ndimage import median_filter
rows = []
for gi, g in enumerate(gains):
    for c in range(4):
        y = db(med[gi, c])
        base = median_filter(y, size=65, mode='nearest')
        ex = y - base
        for k in np.where(keep & (ex > 6.0))[0]:
            if ex[k] >= ex[max(k - 3, 0):k + 4].max():
                # LO spread of this bin: std over LOs of its excess
                e_lo = db(P[gi, :, c, k]) - base[k]
                rows.append((g, c, f[k], ex[k], np.percentile(e_lo, 10), np.percentile(e_lo, 90)))
rows = np.array(rows, dtype=[('g', 'i4'), ('c', 'i4'), ('f', 'f8'), ('ex', 'f8'), ('p10', 'f8'), ('p90', 'f8')])
np.save(f'{outdir}/{tag}_spurs.npy', rows)
with open(f'{outdir}/{tag}_spurs.txt', 'w') as fo:
    fo.write('gain ch  f_MHz    excess_dB  LO_p10  LO_p90\n')
    for r in rows:
        if abs(r['f']) > 0.03:
            fo.write(f"{r['g']:4d} {r['c']:2d} {r['f']:+8.4f} {r['ex']:8.1f} {r['p10']:7.1f} {r['p90']:7.1f}\n")

# 6. Spur level vs gain for the strongest recurring lines (absolute and over floor).
fk = {}
for r in rows:
    if abs(r['f']) < 0.03: continue
    key = (r['c'], round(r['f'] * 1000 / 4.562))
    fk.setdefault(key, []).append(r['ex'])
top = sorted(fk, key=lambda k: -np.max(fk[k]) * min(len(fk[k]), 5))[:10]
fig, ax = plt.subplots(1, 2, figsize=(15, 5.5))
for key in top:
    c, kb = key
    k = N // 2 + int(kb)
    ks = slice(k - 2, k + 3)
    lvl = [db(med[gi, c, ks].max()) for gi in range(len(gains))]
    flo = [np.median(db(med[gi, c, keep])) for gi in range(len(gains))]
    lab = f'CH{c} {f[k]*1000:+.0f} kHz'
    ax[0].plot(gains, lvl, marker='o', label=lab)
    ax[1].plot(gains, np.array(lvl) - np.array(flo), marker='o', label=lab)
flo0 = [np.median(db(med[gi, :, keep])) for gi in range(len(gains))]
ax[0].plot(gains, flo0, 'k--', lw=2, label='keep-band floor (all ch)')
ax[0].set_ylabel('line level, dB LSB$^2$/bin'); ax[1].set_ylabel('line over its channel floor, dB')
for a in ax: a.set_xlabel('gain'); a.grid(alpha=0.3)
ax[0].legend(fontsize=7); ax[1].legend(fontsize=7)
fig.suptitle(f'{tag}: strongest baseband-fixed lines vs gain')
fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_05_spurs_vs_gain.png', dpi=110); plt.close(fig)

# 7. Coherence between channels at the median-over-LO level: |mean CSD| / sqrt(P_a P_b)
pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
fig, ax = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
for axi, g in enumerate([20, 45, 60]):
    if g not in gains: continue
    gi = gains.index(g)
    for pi, (a, c) in enumerate(pairs):
        coh = np.abs(np.median(CSD[gi, :, pi].real, 0) + 1j * np.median(CSD[gi, :, pi].imag, 0)) / \
              np.sqrt(med[gi, a] * med[gi, c])
        ax[axi].plot(f, coh, lw=0.5, label=f'{a}-{c}')
    ax[axi].set_ylabel(f'g{g} |coherence|'); ax[axi].set_ylim(0, 1); ax[axi].grid(alpha=0.3)
ax[0].legend(ncol=6, fontsize=7); ax[-1].set_xlabel('baseband offset (MHz)')
fig.suptitle(f'{tag}: inter-channel coherence of receiver-fixed content (median over LOs of the CSD)')
fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_06_coherence.png', dpi=110); plt.close(fig)

# 8. LO-independence: spread over LOs of each baseband bin (dB), at g45, 4-ch sum.
fig, ax = plt.subplots(figsize=(14, 4.5))
for g in [20, 45, 60]:
    if g not in gains: continue
    gi = gains.index(g)
    s = db(P[gi].sum(1))
    s = s - np.median(s[:, keep], 1)[:, None]
    ax.plot(f, np.percentile(s, 75, 0) - np.percentile(s, 25, 0), lw=0.5, label=f'g{g} IQR over LOs')
ax.set_xlim(-10.5, 10.5); ax.set_ylim(0, 6); ax.grid(alpha=0.3); ax.legend()
ax.set_xlabel('baseband offset (MHz)'); ax.set_ylabel('dB')
ax.set_title(f'{tag}: how much each baseband bin changes with LO (interquartile range over 24 LOs); '
             f'pure estimator noise is ~{10*np.log10(1+2/np.sqrt(d["nblk"]*4)):.1f} dB')
fig.tight_layout(); fig.savefig(f'{outdir}/{tag}_07_lo_spread.png', dpi=110); plt.close(fig)
print('done', tag)
