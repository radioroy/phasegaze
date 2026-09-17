// app.js — hemisphere viewer + live CSI stream. No camera, no IMU.

import { Net } from './js/net.js?v=pg55';
import { VrfRenderer } from './js/render.js?v=pg55';
import { Ui } from './js/ui.js?v=pg55';

const renderer = new VrfRenderer(document.getElementById('gl'));
const net = new Net();
const ui = new Ui({ net, renderer });
window.__renderer = renderer;
window.__ui = ui;

let netFps = 0, ptsLast = 0;

/* Points are raw CFAR hits. Hop IF EQ lives in Ui._fftEqV for the FFT
 * canvas; it must not run here. */
net.onPoints = (header, f32) => {
    netFps = header.fps;
    ptsLast = header.count;
    renderer.ingestFrame(header, f32);
};
net.onSpectrum = (header, f32) => ui.onSpectrum(header, f32);
net.onState = (st) => ui.onState(st);
net.onStatus = (kind) => {
    ui.setStatus(kind);
    if (kind === 'live') ui.pushBackend();
};
net.connect();

const pgClient = (window.crypto && crypto.randomUUID)
    ? crypto.randomUUID()
    : String(Date.now()) + '-' + Math.random().toString(16).slice(2);
let pgLeaving = false;
let attachTimer = null;

function pgAttach() {
    if (pgLeaving) return;
    fetch('/api/apps/attach?app=phasegaze&client=' + encodeURIComponent(pgClient), {
        method: 'POST',
        keepalive: true
    }).catch(() => {});
}

function pgDetach() {
    if (pgLeaving) return;
    pgLeaving = true;
    if (attachTimer) {
        clearInterval(attachTimer);
        attachTimer = null;
    }
    net.disconnect();
    const url = '/api/apps/detach?app=phasegaze&client=' + encodeURIComponent(pgClient);
    try {
        if (navigator.sendBeacon) navigator.sendBeacon(url);
    } catch (e) {}
    fetch(url, { method: 'POST', keepalive: true }).catch(() => {});
}

function pgResume() {
    if (!pgLeaving && attachTimer) return;
    pgLeaving = false;
    if (!attachTimer)
        attachTimer = setInterval(pgAttach, 1000);
    pgAttach();
    if (!net.ws) net.connect();
}

pgAttach();
attachTimer = setInterval(pgAttach, 1000);
window.addEventListener('pagehide', (e) => {
    if (e.persisted) return;
    pgDetach();
});
window.addEventListener('beforeunload', pgDetach);
window.addEventListener('pageshow', pgResume);
window.addEventListener('resume', pgResume);

let lastT = performance.now();
let fpsEma = 0;
let hudDiv = 0;

function loop(t) {
    requestAnimationFrame(loop);
    const dt = Math.min((t - lastT) / 1000, 0.1);
    lastT = t;
    renderer.render(dt);
    const inst = dt > 0 ? 1 / dt : 0;
    fpsEma = fpsEma * 0.9 + inst * 0.1;
    if (++hudDiv >= 15) { hudDiv = 0; ui.tick(fpsEma, netFps, ptsLast); }
}
requestAnimationFrame(loop);
