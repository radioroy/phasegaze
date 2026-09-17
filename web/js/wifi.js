// wifi.js — 5 GHz Wi-Fi channelization tables for the FREQ panel.
// Center frequency = 5000 + 5 * channel (MHz).

const CH20 = [36, 40, 44, 48, 52, 56, 60, 64,
              100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
              149, 153, 157, 161, 165, 169, 173, 177];
const CH40 = [38, 46, 54, 62, 102, 110, 118, 126, 134, 142, 151, 159, 167, 175];
const CH80 = [42, 58, 106, 122, 138, 155, 171];

function tiles(chans, bw) {
    return chans.map(ch => {
        const fc = 5000 + 5 * ch;
        return { ch, bw, f0: fc - bw / 2, f1: fc + bw / 2, label: String(ch) };
    });
}

export const WIFI_TIERS = [
    { bw: 80, tiles: tiles(CH80, 80) },
    { bw: 40, tiles: tiles(CH40, 40) },
    { bw: 20, tiles: tiles(CH20, 20) },
];

/* Occupied 5 GHz span (ch 36 low edge … ch 177 high edge). */
export const WIFI_F0 = 5170;
export const WIFI_F1 = 5895;
/* UNII-1/2A and UNII-2C/3. Gap 5330–5490 is not scanned. */
export const WIFI_SCAN_BANDS = [[5170, 5330], [5490, 5895]];
/* LO walk: 20 MHz hops at channel centers. Split 144/149 so UNII-3
 * stays on the 5745 MHz grid (5 MHz off the 5180/5500 grid). */
export const WIFI_HOP_BANDS = [[5170, 5330], [5490, 5730], [5735, 5895]];
/* Compressed hole between the two clusters, as a fraction of the axis. */
export const WIFI_GAP = 0.07;
