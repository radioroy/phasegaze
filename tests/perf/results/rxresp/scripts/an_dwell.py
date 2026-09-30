"""Receiver response from dwell captures: per (gain, LO, ch) Welch PSD of
8192-sample blocks, both halves of every held-LO span."""
import sys, json
import numpy as np
sys.path.insert(0, '/tmp/rx')
from rxload import load, iq, FS

N = 8192
src = sys.argv[1]
out = sys.argv[2]
h, b = load(src)
gains = sorted(set(h['gain'].tolist()))
los = sorted(set(h['lo'].tolist()))
win = np.hanning(N).astype(np.float32)
w2 = (win ** 2).sum()
res = {}
G, L = len(gains), len(los)
P_hann = np.zeros((G, L, 4, N), np.float32)   # fftshifted, LSB^2 per bin
P_rect = np.zeros((G, L, 4, N), np.float32)
CSD = np.zeros((G, L, 6, N), np.complex64)    # Hann cross spectra
DC = np.zeros((G, L, 4), np.complex64)
RMS = np.zeros((G, L, 8), np.float32)         # per rail
PEAK = np.zeros((G, L), np.int32)
LNA = np.zeros(G, int); VGA = np.zeros(G, int)
pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
for gi, g in enumerate(gains):
    for li, lo in enumerate(los):
        m = (h['gain'] == g) & (np.abs(h['lo'] - lo) < 1e-6)
        x = iq(b[m])                                # (s,16384,4)
        LNA[gi], VGA[gi] = h['lna'][m][0], h['vga'][m][0]
        raw = b[m].view(np.int8).reshape(-1, 16384, 8)
        RMS[gi, li] = np.sqrt((raw.astype(np.float32) ** 2).mean((0, 1)))
        PEAK[gi, li] = np.abs(raw.astype(np.int16)).max()
        DC[gi, li] = x.mean((0, 1))
        x = x.reshape(-1, N, 4)                     # blocks
        Xh = np.fft.fftshift(np.fft.fft(x * win[None, :, None], axis=1), axes=1)
        Xr = np.fft.fftshift(np.fft.fft(x, axis=1), axes=1)
        P_hann[gi, li] = (np.abs(Xh) ** 2).mean(0).T / w2
        P_rect[gi, li] = (np.abs(Xr) ** 2).mean(0).T / N
        for pi, (a, c) in enumerate(pairs):
            CSD[gi, li, pi] = (Xh[:, :, c] * np.conj(Xh[:, :, a])).mean(0) / w2
    print('gain', g, 'lna', LNA[gi], 'vga', VGA[gi],
          'rms', np.round(RMS[gi].mean(0), 2), 'peak', PEAK[gi].max(), flush=True)
np.savez_compressed(out, gains=gains, los=los, P_hann=P_hann, P_rect=P_rect,
                    CSD=CSD, DC=DC, RMS=RMS, PEAK=PEAK, LNA=LNA, VGA=VGA,
                    nblk=int(x.shape[0]))
