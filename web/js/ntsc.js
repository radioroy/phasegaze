// ntsc.js — 5.8 GHz analog FPV channels for the FREQ panel.
// PAL VTXs use the same carriers. The trapezoid is the FM footprint:
// 10 MHz flat top (where the video energy sits), 30 MHz base (the width
// the channel charts call the analog channel). The 6 MHz NTSC figure is
// the baseband into the modulator, not this RF width.
//
// A selected channel is expanded onto the 20 MHz keep-band grid anchored
// at 4480 before it is sent. plan_add_span only emits an LO when the
// whole ±10 MHz keep fits in the span and starts the walk at a+10, so a
// raw ±15 MHz window would park one off-grid LO and drop the other half.

export const NTSC_TOP_MHZ = 10;
export const NTSC_BASE_MHZ = 30;

const HW_ORG = 4480;
const HW_STEP = 20;

const BANDS = [
    ['A', [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725]],
    ['B', [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866]],
    ['E', [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945]],
    ['F', [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880]],
    ['R', [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917]],
];

export const NTSC_ROWS = BANDS.map(([band]) => band);

export const NTSC_CHANNELS = BANDS.flatMap(([band, chans]) =>
    chans.map((fc, i) => ({ id: band + (i + 1), band, n: i + 1, fc })));

const NTSC_BY_ID = new Map(NTSC_CHANNELS.map(c => [c.id, c]));

export function ntscById(id) {
    return NTSC_BY_ID.get(id) || null;
}

/* [lo, hi] of the 20 MHz slices that intersect [f0, f1]. Edges stay on
 * the 4480 grid so the LOs are the same centers range mode already uses. */
export function ntscSlices(f0, f1) {
    if (f1 < f0) { const t = f0; f0 = f1; f1 = t; }
    const a = HW_ORG + HW_STEP * Math.floor((f0 - HW_ORG) / HW_STEP + 1e-9);
    let b = HW_ORG + HW_STEP * Math.ceil((f1 - HW_ORG) / HW_STEP - 1e-9);
    if (b <= a) b = a + HW_STEP;
    return [a, b];
}

export function ntscChannelSpan(ch) {
    const h = NTSC_BASE_MHZ / 2;
    return ntscSlices(ch.fc - h, ch.fc + h);
}

function mergeSpans(spans) {
    if (!spans.length) return [];
    const s = spans.map(b => [Math.min(b[0], b[1]), Math.max(b[0], b[1])])
        .sort((a, b) => a[0] - b[0]);
    const out = [s[0].slice()];
    for (let i = 1; i < s.length; i++) {
        const last = out[out.length - 1];
        if (s[i][0] <= last[1] + 0.01) last[1] = Math.max(last[1], s[i][1]);
        else out.push(s[i].slice());
    }
    return out;
}

/* Empty selection: every channel's footprint, merged. One span,
 * 5620–5960, seventeen hops. */
export const NTSC_SCAN = mergeSpans(NTSC_CHANNELS.map(ntscChannelSpan));
export const NTSC_F0 = NTSC_SCAN.length ? NTSC_SCAN[0][0] : 5620;
export const NTSC_F1 = NTSC_SCAN.length ? NTSC_SCAN[NTSC_SCAN.length - 1][1] : 5960;

/* User-space polygon, viewBox 0 0 BASE 16. Inset keeps the 1 px stroke
 * inside the element. */
export function ntscTrapPoints() {
    const base = NTSC_BASE_MHZ;
    const top = NTSC_TOP_MHZ;
    const x0 = (base - top) / 2;
    const x1 = x0 + top;
    return `${x0},0.4 ${x1},0.4 ${base - 0.4},15.6 0.4,15.6`;
}
