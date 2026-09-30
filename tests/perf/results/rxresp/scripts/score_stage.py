import sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

BIN = 37.3726 / 8192
NAMES = ['none', 'bal', 'bal+mask', 'bal+bg', 'bal+bg+mask', 'bg+mask',
         'bal+mask c5', 'bal+bg+mask c5', 'bal+bg+mask c6', 'bal+bg+mask c9']
PT = np.dtype([('lo', '<f4'), ('seq', '<i4'), ('rf', '<f4'), ('u', '<f4'), ('v', '<f4'), ('I', '<f4')])
d, tag, outdir, nhops_total = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
g = int(tag.split('g')[-1])
census = np.load('/tmp/rx/png/quadrf_spurs.npy')
cen = census[(census['g'] == g) & (np.abs(census['f']) > 0.03)]
cen_bins = set()
for r in cen:
    kb = int(round(r['f'] / BIN))
    cen_bins.update(range(kb - 2, kb + 3))

P = [np.fromfile(f'{d}/{tag}_{i}.bin', dtype=PT) for i in range(len(NAMES))]
NLO = 28
def kb_of(p): return np.round((p['rf'] - p['lo']) / BIN).astype(int)

# receiver-fixed bins from the ungated run: fire on >= 6 LOs and > 3 visits' worth
p0 = P[0]; k0 = kb_of(p0)
fixed = set()
for kb in np.unique(k0):
    m = np.abs(k0 - kb) <= 1
    if len(np.unique(p0['lo'][m])) >= 6 and m.sum() > 3 * nhops_total / NLO:
        fixed.add(int(kb))
rx_bins = fixed | cen_bins

# repeatable emitters: 100 kHz RF cells hit in >= 15% of sweeps with a stable direction
sweep0 = p0['seq'] // NLO
nsw = len(np.unique(sweep0))
cells = {}
rfc = np.round(p0['rf'] / 0.1).astype(int)
notrx = ~np.isin(k0, list(rx_bins))
for c in np.unique(rfc[notrx]):
    m = (rfc == c) & notrx
    ns = len(np.unique(sweep0[m]))
    if ns < 0.15 * nsw: continue
    uv = np.c_[p0['u'][m], p0['v'][m]]
    med = np.median(uv, 0)
    near = np.hypot(*(uv - med).T) < 0.25
    if near.mean() >= 0.5:
        cells[int(c)] = med

def classify(p):
    kb = kb_of(p)
    rx = np.isin(kb, list(rx_bins))
    c = np.round(p['rf'] / 0.1).astype(int)
    real = np.zeros(len(p), bool)
    for i in np.where(~rx)[0]:
        med = cells.get(int(c[i]))
        if med is not None and np.hypot(p['u'][i] - med[0], p['v'][i] - med[1]) < 0.25:
            real[i] = True
    return rx, real

H = nhops_total
print(f'{tag}: {len(rx_bins)} receiver-fixed bins ({len(fixed)} from hop rates, {len(cen_bins)} from dwell census), '
      f'{len(cells)} repeatable emitter cells over {nsw} sweeps')
print(f'{"config":16s} {"pts/hop":>8s} {"rx-fixed":>9s} {"emitter":>8s} {"other":>8s}   emitter kept')
rows = []
for i, (n, p) in enumerate(zip(NAMES, P)):
    rx, real = classify(p)
    rows.append((n, len(p) / H, rx.sum() / H, real.sum() / H, (~rx & ~real).sum() / H))
e0 = rows[1][3]
for r in rows:
    print(f'{r[0]:16s} {r[1]:8.2f} {r[2]:9.2f} {r[3]:8.2f} {r[4]:8.2f}   {100*r[3]/max(e0,1e-9):5.0f}% of bal')

# sky maps
show = [0, 1, 2, 3, 4, 7]
fig, ax = plt.subplots(2, 3, figsize=(16, 11))
for a, i in zip(ax.flat, show):
    p = P[i]
    rx, real = classify(p)
    a.add_patch(plt.Circle((0, 0), 1, fill=False, color='k', lw=0.5))
    a.scatter(p['u'][~rx & ~real], p['v'][~rx & ~real], s=1, c='0.6', alpha=0.4, label='other')
    a.scatter(p['u'][rx], p['v'][rx], s=1, c='r', alpha=0.4, label='receiver-fixed bins')
    a.scatter(p['u'][real], p['v'][real], s=2, c='b', alpha=0.6, label='repeatable emitters')
    a.set_aspect('equal'); a.set_xlim(-1.05, 1.05); a.set_ylim(-1.05, 1.05)
    a.set_title(f'{NAMES[i]}: {len(p)/H:.2f} pts/hop\nrx {rx.sum()/H:.2f}  emitters {real.sum()/H:.2f}  other {(~rx&~real).sum()/H:.2f}',
                fontsize=10)
ax[0, 0].legend(markerscale=6, fontsize=8, loc='lower left')
fig.suptitle(f'quadrf {tag}: points on the (u, v) disk per detection chain, same {H} recorded hops (Wi-Fi 28-LO plan)')
fig.tight_layout(); fig.savefig(f'{outdir}/quadrf_20_sky_{tag}.png', dpi=100); plt.close(fig)
