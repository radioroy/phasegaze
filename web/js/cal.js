// cal.js — AR alignment warp, ported from the Spatial RF Vision tool so the
// camera-mode projection lands identically.
//
// The warp is a bilinear map over four control points in screen uv (y down,
// 0..1). Those points sit at the 0.25/0.75 quarter marks and the input is
// extrapolated 4x about the centre, so the visible frame is the middle of a
// much larger quad; that keeps the gradient well conditioned at the edges.
// The same EXTRAP and corner convention live in the point vertex shader
// (uExtrap, uC0..uC3) — change both together.

export const EXTRAP = 4.0;

// Damping on each drag observation. One drag is one correction vector, not a
// solve; successive drags converge instead of overshooting.
const STEP = 0.45;

export const DEFAULT_CORNERS = [
    { u: 0.25, v: 0.25 },
    { u: 0.75, v: 0.25 },
    { u: 0.75, v: 0.75 },
    { u: 0.25, v: 0.75 },
];

export function defaultCorners() {
    return DEFAULT_CORNERS.map(c => ({ u: c.u, v: c.v }));
}

export function sanitizeCorners(c) {
    if (!Array.isArray(c) || c.length !== 4) return defaultCorners();
    const out = c.map(p => ({ u: Number(p && p.u), v: Number(p && p.v) }));
    return out.every(p => Number.isFinite(p.u) && Number.isFinite(p.v))
        ? out : defaultCorners();
}

function weights(u, v) {
    const um = (u - 0.5) * EXTRAP + 0.5;
    const vm = (v - 0.5) * EXTRAP + 0.5;
    return [
        (1 - um) * (1 - vm),
        um * (1 - vm),
        um * vm,
        (1 - um) * vm,
    ];
}

/* RF direction uv -> screen uv. */
export function mapPoint(corners, u, v) {
    const w = weights(u, v);
    let x = 0, y = 0;
    for (let i = 0; i < 4; i++) {
        x += w[i] * corners[i].u;
        y += w[i] * corners[i].v;
    }
    return { x, y };
}

/* Screen uv -> RF direction uv. Newton on the current warp; the centre is a
 * safe start for the modest, continuous warps calibration produces. */
export function unmapPoint(corners, tx, ty) {
    let u = 0.5, v = 0.5;
    for (let iter = 0; iter < 12; iter++) {
        const um = (u - 0.5) * EXTRAP + 0.5;
        const vm = (v - 0.5) * EXTRAP + 0.5;
        const w = [
            (1 - um) * (1 - vm),
            um * (1 - vm),
            um * vm,
            (1 - um) * vm,
        ];
        let x = 0, y = 0;
        for (let i = 0; i < 4; i++) {
            x += w[i] * corners[i].u;
            y += w[i] * corners[i].v;
        }
        const ex = tx - x, ey = ty - y;
        if (Math.hypot(ex, ey) < 1e-7) break;

        const dxdu = EXTRAP * ((1 - vm) * (corners[1].u - corners[0].u) +
                               vm * (corners[2].u - corners[3].u));
        const dydu = EXTRAP * ((1 - vm) * (corners[1].v - corners[0].v) +
                               vm * (corners[2].v - corners[3].v));
        const dxdv = EXTRAP * ((1 - um) * (corners[3].u - corners[0].u) +
                               um * (corners[2].u - corners[1].u));
        const dydv = EXTRAP * ((1 - um) * (corners[3].v - corners[0].v) +
                               um * (corners[2].v - corners[1].v));

        const det = dxdu * dydv - dxdv * dydu;
        if (Math.abs(det) < 1e-10) return null;

        u += (ex * dydv - dxdv * ey) / det;
        v += (dxdu * ey - ex * dydu) / det;
        if (!Number.isFinite(u) || !Number.isFinite(v)) return null;
    }
    return { u, v };
}

/* One drag: "the RF at (u, v) should move by (dx, dy)", both in normalized
 * screen units. Damped normalized gradient step onto all four corners, always
 * recomputed from the corners captured at drag start so the result depends on
 * the start-to-current vector alone and not on pointer event rate. */
export function applyDrag(corners, base, u, v, dx, dy) {
    const w = weights(u, v);
    let denom = 0;
    for (let i = 0; i < 4; i++) denom += w[i] * w[i];
    if (denom < 1e-9) return;
    for (let i = 0; i < 4; i++) {
        const k = STEP * w[i] / denom;
        corners[i].u = base[i].u + k * dx;
        corners[i].v = base[i].v + k * dy;
    }
}
