// colors.js — color scheme LUTs for the point shader.
//
// A scheme is a 256-entry RGB lookup plus a mapping mode telling the shader
// what drives the lookup coordinate:
//   mode 0: normalized frequency within the sweep
//   mode 1: point intensity
//   mode 2: proximity to a target frequency (discontinuous outside the window)

function hsv(h, s, v) {
    h = ((h % 1) + 1) % 1;
    const i = Math.floor(h * 6), f = h * 6 - i;
    const p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
    const c = [[v, t, p], [q, v, p], [p, v, t], [p, q, v], [t, p, v], [v, p, q]][i % 6];
    return c;
}

function lerpStops(stops, t) {
    // stops: [[t, r, g, b], ...] with t ascending, rgb 0..1
    if (t <= stops[0][0]) return stops[0].slice(1);
    for (let i = 1; i < stops.length; i++) {
        if (t <= stops[i][0]) {
            const a = stops[i - 1], b = stops[i];
            const f = (t - a[0]) / (b[0] - a[0]);
            return [a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f, a[3] + (b[3] - a[3]) * f];
        }
    }
    return stops[stops.length - 1].slice(1);
}

const IRON_STOPS = [
    [0.00, 0.00, 0.00, 0.00],
    [0.15, 0.10, 0.00, 0.30],
    [0.35, 0.55, 0.00, 0.55],
    [0.55, 0.90, 0.25, 0.10],
    [0.75, 1.00, 0.60, 0.00],
    [0.90, 1.00, 0.90, 0.30],
    [1.00, 1.00, 1.00, 0.95],
];

const VIRIDIS_STOPS = [
    [0.00, 0.267, 0.005, 0.329],
    [0.25, 0.229, 0.322, 0.546],
    [0.50, 0.128, 0.567, 0.551],
    [0.75, 0.369, 0.789, 0.383],
    [1.00, 0.993, 0.906, 0.144],
];

function buildLut(fn) {
    const data = new Uint8Array(256 * 4);
    for (let i = 0; i < 256; i++) {
        const [r, g, b] = fn(i / 255);
        data[i * 4 + 0] = Math.round(r * 255);
        data[i * 4 + 1] = Math.round(g * 255);
        data[i * 4 + 2] = Math.round(b * 255);
        data[i * 4 + 3] = 255;
    }
    return data;
}

export const SCHEMES = {
    spectrum: { label: 'SPECTRUM', mode: 0, lut: buildLut(t => hsv(t * 0.833, 1, 1)) },
    iron:     { label: 'IRON',     mode: 1, lut: buildLut(t => lerpStops(IRON_STOPS, t)) },
    whitehot: { label: 'WHITE HOT', mode: 1, lut: buildLut(t => [t, t, t]) },
    greenhot: { label: 'GREEN HOT', mode: 1, lut: buildLut(t => [t * 0.25, t, t * 0.35]) },
    viridis:  { label: 'VIRIDIS',  mode: 1, lut: buildLut(t => lerpStops(VIRIDIS_STOPS, t)) },
    target:   { label: 'TARGET',   mode: 2, lut: buildLut(t => lerpStops(IRON_STOPS, t)) },
};

export function lutRgb(lut, t) {
    const i = Math.max(0, Math.min(255, Math.round(t * 255)));
    return [lut[i * 4], lut[i * 4 + 1], lut[i * 4 + 2]];
}

/* CSS gradient preview string for the panel tiles. */
export function schemeCss(name) {
    const s = SCHEMES[name];
    const stops = [];
    for (let i = 0; i <= 8; i++) {
        const t = i / 8;
        const j = Math.round(t * 255) * 4;
        stops.push(`rgb(${s.lut[j]},${s.lut[j + 1]},${s.lut[j + 2]}) ${t * 100}%`);
    }
    return `linear-gradient(90deg, ${stops.join(',')})`;
}
