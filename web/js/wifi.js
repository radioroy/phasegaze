// wifi.js — 5 GHz and 6 GHz channelization for the FREQ panel.
// 5 GHz center = 5000 + 5 * channel (MHz).
// 6 GHz center = 5950 + 5 * channel (MHz), 802.11ax.
// 6 GHz stops at ch 153 (6705–6725). ch 157 runs 6725–6745, past the
// 6740 MHz hardware edge, so the 40/80 MHz channels that contain it
// (155, 151) are out too.

function tiles(chans, bw, origin, view) {
    const ghz = view === '6' ? '6 GHz' : '5 GHz';
    return chans.map(ch => {
        const fc = origin + 5 * ch;
        return {
            ch, bw, view,
            f0: fc - bw / 2, f1: fc + bw / 2,
            label: String(ch),
            title: `${ghz} ${ch}`,
        };
    });
}

function seq(first, step, last) {
    const out = [];
    for (let ch = first; ch <= last; ch += step) out.push(ch);
    return out;
}

const CH20 = [36, 40, 44, 48, 52, 56, 60, 64,
              100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
              149, 153, 157, 161, 165, 169, 173, 177];
const CH40 = [38, 46, 54, 62, 102, 110, 118, 126, 134, 142, 151, 159, 167, 175];
const CH80 = [42, 58, 106, 122, 138, 155, 171];
const CH20_6 = seq(1, 4, 153);
const CH40_6 = seq(3, 8, 147);
const CH80_6 = seq(7, 16, 135);

export const WIFI_VIEWS = [
    {
        id: '5',
        label: '5 GHZ',
        f0: 5170,
        f1: 5895,
        /* UNII-1/2A and UNII-2C/3/4. The 5330–5490 hole is not scanned. */
        scan: [[5170, 5330], [5490, 5895]],
    },
    {
        id: '6',
        label: '6 GHZ',
        f0: 5945,
        f1: 6725,
        scan: [[5945, 6725]],
    },
];

export const WIFI_TIERS = [
    { bw: 80, tiles: [...tiles(CH80, 80, 5000, '5'), ...tiles(CH80_6, 80, 5950, '6')] },
    { bw: 40, tiles: [...tiles(CH40, 40, 5000, '5'), ...tiles(CH40_6, 40, 5950, '6')] },
    { bw: 20, tiles: [...tiles(CH20, 20, 5000, '5'), ...tiles(CH20_6, 20, 5950, '6')] },
];

/* Full catalog the sphere hue stretches across, both views. */
export const WIFI_F0 = 5170;
export const WIFI_F1 = 6725;
/* LO walk: 20 MHz hops at channel centers. 5 GHz splits 144/149 so
 * UNII-3 stays on the 5745 MHz grid (5 MHz off 5180/5500). 6 GHz is
 * its own grid: centers 5955, 5975, ... */
export const WIFI_HOP_BANDS = [[5170, 5330], [5490, 5730], [5735, 5895], [5945, 6725]];
/* Compressed hole between the two 5 GHz clusters, as a fraction of the axis. */
export const WIFI_GAP = 0.07;
