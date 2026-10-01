// ui.js — FREQ / COLOR / CONF panels, gain slider, fps line.

import { WIFI_TIERS, WIFI_F0, WIFI_F1, WIFI_SCAN_BANDS, WIFI_HOP_BANDS, WIFI_GAP } from './wifi.js';
import { SCHEMES, schemeCss, lutRgb } from './colors.js';
import { Cam } from './cam.js';
import { applyDrag, defaultCorners, sanitizeCorners, unmapPoint } from './cal.js';

const TARGET_LUTS = ['spectrum', 'iron', 'whitehot', 'greenhot', 'viridis'];

const HW_MIN = 4900, HW_MAX = 6100;
const RF_GAIN_MAX = 63;
const FFT_SPEEDS = ['fast', 'med', 'slow'];
const FFT_SPEED_LAB = { fast: 'FAST', med: 'MED', slow: 'SLOW' };
const WIFI_COLOR = ['band', 'chan', 'full'];
const WIFI_COLOR_LAB = { band: 'BAND', chan: 'CHAN', full: 'FULL' };
/* Video-average time constants (s). Independent of sweep rate. */
const FFT_TAU = { fast: 0.07, med: 0.28, slow: 0.90 };
/* Y-axis AGC (s). p98, not the single hottest bin. Release is long so
 * Wi-Fi bursts do not pump the scale; a plan change snaps instead. */
const FFT_AGC_ATK = 0.18;
const FFT_AGC_REL = 1.10;
/* After the plan or the slider stops moving: snap the Y scale, then
 * let the hop EQ learn again (stale-plan frames are not learned). */
const FFT_PLAN_SNAP_MS = 300;
const FFT_PLAN_LEARN_MS = 400;
const FFT_FLOOR_ATK = 0.10;
const FFT_FLOOR_REL = 0.40;
/* Pad puts the floor a little above the axis. */
const FFT_LO_PAD = 0.10;
/* Keep Y from ranging into the hop scallop when the band is empty. */
const FFT_Y_MIN_RATIO = 1.75;
/* Analog RX is 20 MHz, folded ±10 MHz around each LO. The per-hop
 * residual is not one shape: skirt depth tracks the RF noise share of
 * the floor, so edges dip ~0.15 ln at 4.9 GHz and hump ~0.15 at 6 GHz.
 * Learned per LO as the per-offset median of (hop - hop median) over
 * the hop and its ±FFT_EQ_WIN neighbours; an emitter that fills one or
 * two hops is outvoted and keeps its shape. Across 5.5-5.8 GHz the
 * ±8 MHz lines also alternate with LO mod 80 MHz (two hops on, two
 * off), so hops at the same LO phase vote first when there are enough
 * of them. The hop median (passband)
 * is the zero, so the floor is never pulled down. Banked per gain and
 * server-norm state; short plans and Wi-Fi hops read it interpolated
 * in LO. FFT canvas only, hemisphere points stay on raw CFAR. */
const FFT_HOP_MHZ = 20;
const FFT_EQ_N = 20;
const FFT_EQ_WIN = 3;
const FFT_EQ_MIN_HOPS = 4;
const FFT_EQ_FILL = 15;
const FFT_EQ_TAU = 1.0;
const FFT_EQ_LO0 = 4900;
const FFT_EQ_LO_N = 1201;
/* LO distance (MHz) a learned hop may be borrowed or interpolated over. */
const FFT_EQ_REACH = 40;
const FFT_EQ_PHASE = 80;
const FFT_EQ_PHASE_WIN = 2;
const FFT_EQ_PHASE_MIN = 3;
/* On top of that, skirt and DC taps carry a fixed per-LO pattern
 * (stable to ~0.02 ln over minutes, different hop to hop). Learned per
 * LO only while the hop is quiet: level within FFT_EQ_QUIET_LVL of its
 * neighbours and passband p90-p10 under FFT_EQ_QUIET_IQR. Passband
 * taps never get a per-LO term, so a weak emitter there stays. */
const FFT_EQ_DETAIL = [0, 1, 2, 8, 9, 10, 17, 18, 19];
const FFT_EQ_PASS = [3, 4, 5, 6, 7, 11, 12, 13, 14, 15, 16];
const FFT_EQ_QUIET_LVL = 0.10;
const FFT_EQ_QUIET_IQR = 0.12;
const FFT_EQ_DETAIL_TAU = 2.0;
const FFT_EQ_DETAIL_CLIP = 0.40;
/* Coherent level jump: median shift (ln) and the IQR under it.
 * A gain step moves every bin together; a new emitter does not. */
const FFT_SHIFT_MIN = 0.20;
const FFT_SHIFT_IQR = 0.30;
const FFT_MISS_HOLD = 6;
const LS_KEY = 'phasegaze.settings.v9';
const ACCENT_DEFAULT = '#b8c4b8';
/* Camera mode hides the HUD this long after the last tap. */
const CAM_HIDE_MS = 8000;

const DEFAULTS = {
    size: 15, gain: 4.0, decay: 1, density: 100,
    balanceDb: 10, closureDeg: 0, bgNorm: true, cfarDb: 7,
    pulse: false, flip: false,
    mirrors: true, bottom: false, tiles: true, rings: false,
    scheme: 'spectrum', targetLut: 'iron', targetFreq: 5500, targetWidth: 40,
    freqPin: true, wifiColor: 'band', freqTab: 'range',
    fft: true, fftSpeed: 'fast', fftAgc: true,
    hwGain: 45,
    manualLo: HW_MIN, manualHi: HW_MAX,
    wifiSel: [],
    accent: ACCENT_DEFAULT,
    corners: defaultCorners(),
};

function parseHex(v) {
    if (typeof v !== 'string') return null;
    const m = v.trim().match(/^#?([0-9a-fA-F]{6})$/);
    return m ? ('#' + m[1].toLowerCase()) : null;
}

function hexRgb(hex) {
    return [
        parseInt(hex.slice(1, 3), 16),
        parseInt(hex.slice(3, 5), 16),
        parseInt(hex.slice(5, 7), 16),
    ];
}

function accentInk(hex) {
    const [r, g, b] = hexRgb(hex);
    const lum = (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255;
    return lum > 0.55 ? '#0c110c' : '#e8ece8';
}

/* 0 = one frame, 100 = hold, else time constant in seconds (0.05 .. 20). */
function decayTau(v) {
    if (v <= 0) return -1;
    if (v >= 100) return 0;
    return 0.05 * Math.pow(400, (v - 1) / 98);
}
function decayLabel(v) {
    if (v <= 0) return 'STAMP';
    if (v >= 100) return 'HOLD';
    return `${decayTau(v).toFixed(2)} S`;
}
/* Strongest RX over second strongest at the bin; 0 turns the gate off. */
function balLabel(v) { return v <= 0 ? 'OFF' : `${v.toFixed(0)} DB`; }
/* Max parallelogram closure |phi0 - phi1 + phi2 - phi3|; 0 or 180 is off. */
function closLabel(v) { return (v <= 0 || v >= 180) ? 'OFF' : `${v.toFixed(0)}°`; }
/* CFAR: bin over the local log-power mean. */
function cfarLabel(v) { return `${v.toFixed(1)} DB`; }

const $ = (id) => document.getElementById(id);

export class Ui {
    constructor({ net, renderer }) {
        this.net = net;
        this.renderer = renderer;

        this.s = { ...DEFAULTS, ...this._load() };
        this.s.hwGain = Math.max(0, Math.min(RF_GAIN_MAX, this.s.hwGain | 0));
        if (!Number.isFinite(this.s.balanceDb)) this.s.balanceDb = DEFAULTS.balanceDb;
        if (!Number.isFinite(this.s.closureDeg)) this.s.closureDeg = DEFAULTS.closureDeg;
        if (typeof this.s.bgNorm !== 'boolean') this.s.bgNorm = true;
        if (!Number.isFinite(this.s.cfarDb)) this.s.cfarDb = DEFAULTS.cfarDb;
        delete this.s.spurMask; delete this.s.spurMargin;
        this.s.accent = parseHex(this.s.accent) || ACCENT_DEFAULT;
        if (!TARGET_LUTS.includes(this.s.targetLut)) this.s.targetLut = 'iron';
        if (typeof this.s.fft !== 'boolean') this.s.fft = true;
        if (!FFT_SPEEDS.includes(this.s.fftSpeed)) this.s.fftSpeed = 'fast';
        if (typeof this.s.fftAgc !== 'boolean') this.s.fftAgc = true;
        if (typeof this.s.freqPin !== 'boolean') this.s.freqPin = true;
        if (typeof this.s.mirrors !== 'boolean') this.s.mirrors = true;
        if (typeof this.s.rings !== 'boolean') this.s.rings = false;
        if (this.s.freqTab !== 'wifi') this.s.freqTab = 'range';
        if (!WIFI_COLOR.includes(this.s.wifiColor))
            this.s.wifiColor = this.s.wifiChan === true ? 'chan' : 'band';
        delete this.s.wifiChan;
        this.s.wifiSel = sanitizeWifiSel(this.s.wifiSel);
        this.s.corners = sanitizeCorners(this.s.corners);

        this.spectrum = null;
        this._fftAvg = null;
        this._fftHold = null;
        this._fftMiss = null;
        this._fftScale = 1e-6;
        this._fftLo = 0;
        this._fftScratch = [];
        this._fftDelta = [];
        /* Per-gain IF deviation from that hop's own p20. 64 x 20. */
        /* Per gain, x {raw, server-normalized} x {sweep, parked LO}.
         * Each: LO x offset tables { s, n } plus per-LO detail { d, dn }, lazy. */
        this._fftBanks = new Array(4 * (RF_GAIN_MAX + 1)).fill(null);
        this._fftStatic = false;
        this._fftPlanLo0 = 0;
        this._fftUiSig = '';
        this._fftPlanT = 0;
        this._fftPlanPending = false;
        this._fftHopMap = new Map();
        this._fftHopList = [];
        this._fftCorr = null;
        this._fftCorrKey = -1;
        this._fftSnapped = false;
        this._fftSrvNorm = false;
        this._fftT = 0;
        this._fftDt = 0.03;
        this.state = null;
        this.netFps = 0;
        this.gpuFps = 0;
        this.pts = 0;
        this._gainSendTimer = 0;
        this._rangeSendTimer = 0;
        this._freqTab = this.s.freqTab;

        this.camMode = false;
        this.calOn = false;
        this._camHideTimer = null;
        this._revealTap = false;
        this.cam = new Cam($('cam-feed'));

        this._selectFreqTab = null;
        this._buildWifi();
        this._buildSchemes();
        this._bindPanels();
        this._bindHud();
        this._bindGain();
        this._bindManual();
        this._bindSettings();
        this._bindPointer();
        this._bindHint();
        this._bindCal();
        this._bindCamFade();
        this._applyAll();
        this._setCamMode(true);
        this._resize();
        window.addEventListener('resize', () => this._resize());
        if (window.visualViewport)
            window.visualViewport.addEventListener('resize', () => this._resize());
    }

    _load() {
        try {
            const cur = localStorage.getItem(LS_KEY);
            if (cur) return JSON.parse(cur) || {};
            const v8 = localStorage.getItem('phasegaze.settings.v8');
            if (v8) {
                const prev = JSON.parse(v8) || {};
                prev.rings = false;
                return prev;
            }
            const v7 = localStorage.getItem('phasegaze.settings.v7');
            if (v7) {
                const prev = JSON.parse(v7) || {};
                /* v7 slider 0 was the 0.05 s default. 0 is now one frame. */
                if (prev.decay === 0) prev.decay = 1;
                return prev;
            }
            const v6 = localStorage.getItem('phasegaze.settings.v6');
            if (v6) {
                const prev = JSON.parse(v6) || {};
                /* 23 was the v6 default (~0.20 s). 0 was 0.05 s. */
                if (prev.decay === 23 || prev.decay === 0) prev.decay = 1;
                return prev;
            }
            const v3 = localStorage.getItem('phasegaze.settings.v3');
            if (v3) {
                const prev = JSON.parse(v3) || {};
                prev.mirrors = true;
                if (typeof prev.rings !== 'boolean')
                    prev.rings = false;
                return prev;
            }
            const v2 = localStorage.getItem('phasegaze.settings.v2');
            if (v2) {
                const prev = JSON.parse(v2) || {};
                delete prev.density;
                return prev;
            }
            const v1 = localStorage.getItem('phasegaze.settings.v1');
            if (v1) {
                const prev = JSON.parse(v1) || {};
                delete prev.size;
                delete prev.density;
                return prev;
            }
            return {};
        } catch (_) { return {}; }
    }
    save() { localStorage.setItem(LS_KEY, JSON.stringify(this.s)); }

    /* Push every local setting into the renderer (and backend knobs). */
    _applyAll() {
        const r = this.renderer, s = this.s;
        r.setPointSize(s.size);
        r.setPointGain(s.gain);
        r.setDecayTau(decayTau(s.decay));
        r.setPulse(s.pulse);
        r.setFlipX(s.flip);
        r.setMirrors(s.mirrors);
        r.setBottom(s.bottom);
        r.setTiles(s.tiles);
        r.setRings(s.rings);
        r.setCorners(s.corners);
        this._applySchemeLut();
        this._syncFreqColor();
        r.setTarget(s.targetFreq, s.targetWidth);
        this._applyAccent();
        this._syncSettingUi();
    }

    _applyAccent() {
        const hex = this.s.accent || ACCENT_DEFAULT;
        const [r, g, b] = hexRgb(hex);
        const root = document.documentElement.style;
        root.setProperty('--acc', hex);
        root.setProperty('--acc-ink', accentInk(hex));
        root.setProperty('--acc-soft', `rgba(${r}, ${g}, ${b}, 0.28)`);
        this.renderer.setAccent(hex);
        if ($('freq-pop').classList.contains('open') && this.s.fft)
            this._drawFft();
    }

    _gateMsg() {
        const deg = this.s.closureDeg >= 180 ? 0 : this.s.closureDeg;
        return { balance_db: this.s.balanceDb, closure_max: deg * Math.PI / 180,
            bg_norm: this.s.bgNorm ? 1 : 0, cfar_db: this.s.cfarDb };
    }

    /* Backend re-sync (on connect). */
    pushBackend() {
        this.net.set({ gain: this.s.hwGain, output_fraction: this.s.density / 100,
            spectrum: this.s.fft ? 1 : 0, ...this._gateMsg() });
        if (this.s.scheme === 'target') this._applyTargetSweep(true);
        else if (this._freqTab === 'wifi') this._commitWifiSel();
        else this._applyManualRange(true);
    }

    // ==================================================================
    // HUD / menu
    // ==================================================================

    _bindHud() {
        $('btn-freq').onclick = (e) => {
            e.stopPropagation();
            if ($('freq-pop').classList.contains('open')) this._closeFreq();
            else this._openFreq();
        };
        $('btn-color').onclick = (e) => {
            e.stopPropagation();
            if ($('color-pop').classList.contains('open')) this._closeColor();
            else this._openColor();
        };
        $('btn-set').onclick = (e) => {
            e.stopPropagation();
            if ($('set-pop').classList.contains('open')) this._closeSet();
            else this._openSet();
        };
        $('btn-mirror').onclick = () => {
            this.s.mirrors = !this.s.mirrors;
            $('btn-mirror').classList.toggle('on', this.s.mirrors);
            this.renderer.setMirrors(this.s.mirrors);
            this.save();
        };

        $('btn-view').onclick = () => {
            const next = this.renderer.sphereCam === 'orbit' ? 'inside' : 'orbit';
            this.renderer.setSphereCam(next);
            this._syncViewBtn();
        };
        $('btn-reset-view').onclick = () => this.renderer.resetView();

        $('btn-cam').onclick = () => this._setCamMode(!this.camMode);
        $('btn-cal').onclick = () => this._setCal(!this.calOn);
        $('btn-reset-cal').onclick = () => {
            this.s.corners = defaultCorners();
            this.renderer.setCorners(this.s.corners);
            this._drawCal(null);
            this.save();
        };
        $('btn-flip').onclick = () => this._setFlip(!this.s.flip);
        $('btn-full').onclick = () => {
            if (!document.fullscreenElement)
                document.documentElement.requestFullscreen().catch(() => {});
            else document.exitFullscreen().catch(() => {});
        };
        document.addEventListener('fullscreenchange', () => {
            $('btn-full').classList.toggle('on', !!document.fullscreenElement);
        });
    }

    // ==================================================================
    // Camera (AR) mode
    // ==================================================================

    _setCamMode(on) {
        if (this.camMode === on) return;
        this.camMode = on;
        document.body.classList.toggle('cam', on);
        this._closeFreq();
        this._closeColor();
        this._closeSet();
        this.renderer.setCamMode(on);
        if (on) {
            this.cam.start();
        } else {
            this.cam.stop();
            this._setCal(false);
        }
        this._setHint(false);
        this._syncModeUi();
        this._bumpControls();
    }

    _setCal(on) {
        this.calOn = on && this.camMode;
        document.body.classList.toggle('cal', this.calOn);
        $('btn-cal').classList.toggle('on', this.calOn);
        this._drawCal(null);
        this._bumpControls();
    }

    _setFlip(on) {
        this.s.flip = on;
        this.renderer.setFlipX(on);
        $('t-flip').classList.toggle('on', on);
        $('btn-flip').classList.toggle('on', on);
        this.save();
    }

    /* Button/section visibility for the active mode. */
    _syncModeUi() {
        const cam = this.camMode;
        const show = (id, vis) => { $(id).style.display = vis ? '' : 'none'; };
        $('btn-cam').textContent = cam ? 'SPHERE' : 'CAM';
        $('btn-cam').classList.remove('on');
        this._syncViewBtn();
        show('btn-cal', cam);
        show('btn-mirror', !cam);
        show('btn-view', !cam);
        show('btn-reset-view', !cam);
        show('btn-reset-cal', cam);
        show('btn-flip', cam);
        // Geometry flip lives on the bottom-right button in camera mode, and
        // the sphere shell toggles have nothing to act on.
        for (const id of ['sect-geometry', 't-flip', 'sect-sphere',
                          't-bottom', 't-tiles', 't-rings'])
            show(id, !cam);
        $('btn-flip').classList.toggle('on', !!this.s.flip);
    }

    _syncViewBtn() {
        const inside = this.renderer.sphereCam === 'inside';
        $('btn-view').textContent = (inside ? 'outside' : 'inside').toUpperCase();
    }

    _bindCamFade() {
        // Capture phase: a tap that only brings the HUD back must not also
        // land on a control or start a calibration drag.
        document.addEventListener('pointerdown', () => {
            this._revealTap = this.camMode && document.body.classList.contains('dim');
            this._bumpControls();
        }, { capture: true, passive: true });
        const clear = () => { this._revealTap = false; };
        document.addEventListener('pointerup', clear, { capture: true, passive: true });
        document.addEventListener('pointercancel', clear, { capture: true, passive: true });
    }

    _bumpControls() {
        clearTimeout(this._camHideTimer);
        this._camHideTimer = null;
        document.body.classList.remove('dim');
        if (!this.camMode) return;
        this._camHideTimer = setTimeout(() => {
            if (this._calDrag || $('freq-pop').classList.contains('open') ||
                    $('color-pop').classList.contains('open') ||
                    $('set-pop').classList.contains('open')) {
                this._bumpControls();
                return;
            }
            document.body.classList.add('dim');
        }, CAM_HIDE_MS);
    }

    // ---------- alignment drag ----------

    _bindCal() {
        const cv = $('cal-layer');
        this._calDrag = null;
        const at = (e) => ({ x: e.clientX, y: e.clientY });

        cv.addEventListener('pointerdown', (e) => {
            if (this._revealTap) return;
            const p = at(e);
            // The drag start is the observation: invert the current warp to
            // learn which RF direction is being shown at this pixel.
            const src = unmapPoint(this.s.corners, p.x / innerWidth, p.y / innerHeight);
            if (!src) return;
            this._calDrag = {
                u: src.u, v: src.v, x0: p.x, y0: p.y, x: p.x, y: p.y,
                base: this.s.corners.map(c => ({ u: c.u, v: c.v })),
                moved: false,
            };
            try { cv.setPointerCapture(e.pointerId); } catch (_) {}
            e.preventDefault();
        });

        cv.addEventListener('pointermove', (e) => {
            const d = this._calDrag;
            if (!d) return;
            const p = at(e);
            if (!d.moved && Math.hypot(p.x - d.x0, p.y - d.y0) < 8) return;
            d.moved = true;
            d.x = p.x; d.y = p.y;
            applyDrag(this.s.corners, d.base, d.u, d.v,
                      (p.x - d.x0) / innerWidth, (p.y - d.y0) / innerHeight);
            this.renderer.setCorners(this.s.corners);
            this._drawCal(d);
            this._bumpControls();
            e.preventDefault();
        });

        const end = (e) => {
            const d = this._calDrag;
            if (!d) return;
            if (d.moved) {
                const p = at(e);
                applyDrag(this.s.corners, d.base, d.u, d.v,
                          (p.x - d.x0) / innerWidth, (p.y - d.y0) / innerHeight);
                this.renderer.setCorners(this.s.corners);
                this.save();
            }
            this._calDrag = null;
            this._drawCal(null);
            this._bumpControls();
        };
        cv.addEventListener('pointerup', end);
        cv.addEventListener('pointercancel', end);
    }

    /* Correction vector feedback. The RF itself is the alignment reference,
     * so nothing else is drawn on this layer. */
    _drawCal(d) {
        const cv = $('cal-layer');
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = Math.round(innerWidth * dpr), h = Math.round(innerHeight * dpr);
        if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
        const ctx = cv.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, innerWidth, innerHeight);
        $('cal-tip').classList.toggle('hide', !!(d && d.moved));
        if (!d || !d.moved) return;
        const acc = this.s.accent || ACCENT_DEFAULT;
        ctx.strokeStyle = acc;
        ctx.fillStyle = acc;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(d.x0, d.y0);
        ctx.lineTo(d.x, d.y);
        ctx.stroke();
        ctx.fillRect(d.x0 - 4, d.y0 - 4, 8, 8);
    }

    _openFreq() {
        this._closeColor();
        this._closeSet();
        $('freq-pop').classList.add('open');
        $('btn-freq').classList.add('on');
        if (this._selectFreqTab) this._selectFreqTab(this._freqTab, false);
        this._layoutHudPops();
        this._placeFft();
        this._setHint(false);
    }

    _closeFreq() {
        if (!$('freq-pop').classList.contains('open')) return;
        $('freq-pop').classList.remove('open');
        $('btn-freq').classList.remove('on');
        this._bumpHint();
    }

    _openColor() {
        this._closeFreq();
        this._closeSet();
        $('color-pop').classList.add('open');
        $('btn-color').classList.add('on');
        this._syncSchemeUi();
        this._layoutHudPops();
        this._setHint(false);
    }

    _closeColor() {
        if (!$('color-pop').classList.contains('open')) return;
        $('color-pop').classList.remove('open');
        $('btn-color').classList.remove('on');
        this._bumpHint();
    }

    _openSet() {
        this._closeFreq();
        this._closeColor();
        $('set-pop').classList.add('open');
        $('btn-set').classList.add('on');
        this._layoutHudPops();
        this._setHint(false);
    }

    _closeSet() {
        if (!$('set-pop').classList.contains('open')) return;
        $('set-pop').classList.remove('open');
        $('btn-set').classList.remove('on');
        this._bumpHint();
    }

    _bindPanels() {
        document.addEventListener('pointerdown', (e) => {
            const t = e.target;
            const freq = $('freq-pop');
            if (freq.classList.contains('open') &&
                !freq.contains(t) && !$('btn-freq').contains(t))
                this._closeFreq();
            const color = $('color-pop');
            if (color.classList.contains('open') &&
                !color.contains(t) && !$('btn-color').contains(t))
                this._closeColor();
            const set = $('set-pop');
            if (set.classList.contains('open') &&
                !set.contains(t) && !$('btn-set').contains(t))
                this._closeSet();
        });
    }

    _bindHint() {
        this._hintTimer = null;
        this._hintDrag = null;
        const opts = { passive: true, capture: true };
        document.addEventListener('pointerdown', (e) => {
            this._hintDrag = { x: e.clientX, y: e.clientY, dragging: false };
        }, opts);
        document.addEventListener('pointermove', (e) => {
            const d = this._hintDrag;
            if (!d) return;
            if (!d.dragging && Math.hypot(e.clientX - d.x, e.clientY - d.y) < 8) return;
            d.dragging = true;
            this._bumpHint();
        }, opts);
        const end = () => { this._hintDrag = null; };
        document.addEventListener('pointerup', end, opts);
        document.addEventListener('pointercancel', end, opts);
        this._setHint(true);
    }

    _bumpHint() {
        this._setHint(false);
        clearTimeout(this._hintTimer);
        this._hintTimer = setTimeout(() => this._setHint(true), 10000);
    }

    _setHint(vis) {
        if (vis && this.camMode) return;   // orbit/zoom hint means nothing in AR
        if (vis && ($('freq-pop').classList.contains('open') ||
                    $('color-pop').classList.contains('open') ||
                    $('set-pop').classList.contains('open')))
            return;
        $('hint').classList.toggle('show', vis);
    }

    // ==================================================================
    // Gain slider
    // ==================================================================

    _bindGain() {
        const wrap = $('gain-wrap'), thumb = $('gain-thumb');
        const setFromY = (clientY) => {
            const r = wrap.getBoundingClientRect();
            let t = (clientY - r.top) / r.height;
            t = Math.max(0, Math.min(1, t));
            this.s.hwGain = Math.round((1 - t) * RF_GAIN_MAX);
            this._layoutGain();
            const now = performance.now();
            if (now - this._gainSendTimer > 100) {
                this.net.set({ gain: this.s.hwGain });
                this._gainSendTimer = now;
            }
        };
        let dragging = false;
        wrap.addEventListener('pointerdown', (e) => {
            dragging = true;
            wrap.setPointerCapture(e.pointerId);
            setFromY(e.clientY);
            e.stopPropagation();
        });
        wrap.addEventListener('pointermove', (e) => { if (dragging) setFromY(e.clientY); });
        wrap.addEventListener('pointerup', () => {
            dragging = false;
            this.net.set({ gain: this.s.hwGain });
            this.save();
        });
        this._gainDraggingRef = () => dragging;
        this._layoutGain();
    }

    _layoutGain() {
        const wrap = $('gain-wrap');
        const t = 1 - this.s.hwGain / RF_GAIN_MAX;
        $('gain-thumb').style.top = `calc(${(t * 100).toFixed(1)}% - 7px)`;
        $('gain-label').textContent = `RF ${this.s.hwGain}`;
    }

    // ==================================================================
    // FREQ panel
    // ==================================================================

    _freqToX(f) { return (f - HW_MIN) / (HW_MAX - HW_MIN); }

    /* Piecewise axis: two UNII clusters, compressed hole between them. */
    _wifiToX(f) {
        const a0 = WIFI_SCAN_BANDS[0][0], a1 = WIFI_SCAN_BANDS[0][1];
        const b0 = WIFI_SCAN_BANDS[1][0], b1 = WIFI_SCAN_BANDS[1][1];
        const spanA = a1 - a0, spanB = b1 - b0;
        const usable = 1 - WIFI_GAP;
        const wA = usable * spanA / (spanA + spanB);
        const wB = usable - wA;
        if (f <= a1) return wA * Math.max(0, (f - a0) / spanA);
        if (f < b0) return wA + WIFI_GAP * (f - a1) / (b0 - a1);
        return wA + WIFI_GAP + wB * Math.min(1, Math.max(0, (f - b0) / spanB));
    }

    _wifiXToF(x) {
        const a0 = WIFI_SCAN_BANDS[0][0], a1 = WIFI_SCAN_BANDS[0][1];
        const b0 = WIFI_SCAN_BANDS[1][0], b1 = WIFI_SCAN_BANDS[1][1];
        const spanA = a1 - a0, spanB = b1 - b0;
        const usable = 1 - WIFI_GAP;
        const wA = usable * spanA / (spanA + spanB);
        const wB = usable - wA;
        if (x <= wA) return a0 + (x / Math.max(wA, 1e-9)) * spanA;
        if (x < wA + WIFI_GAP) return a1 + ((x - wA) / WIFI_GAP) * (b0 - a1);
        return b0 + ((x - wA - WIFI_GAP) / Math.max(wB, 1e-9)) * spanB;
    }

    _placeFft() {
        const block = $('fft-block');
        block.classList.toggle('show', !!this.s.fft);
        $('btn-fft').classList.toggle('on', !!this.s.fft);
        $('fft-side').classList.toggle('show', !!this.s.fft);
        this._syncFftOpts();
        if (!this.s.fft) return;
        if (this._freqTab === 'wifi')
            $('freq-wifi').appendChild(block);
        else
            $('freq-range').insertBefore(block, $('manual-wrap'));
        this._drawFft();
    }

    _syncFftOpts() {
        const sp = $('btn-fft-speed'), ag = $('btn-fft-agc');
        if (!sp || !ag) return;
        sp.textContent = FFT_SPEED_LAB[this.s.fftSpeed] || 'FAST';
        ag.textContent = this.s.fftAgc ? 'AGC' : 'HOLD';
        ag.classList.toggle('on', !!this.s.fftAgc);
    }

    _buildWifi() {
        const inner = $('wifi-tiers');
        const gap = document.createElement('div');
        gap.id = 'wifi-gap';
        gap.style.left = (this._wifiToX(WIFI_SCAN_BANDS[0][1]) * 100) + '%';
        gap.style.width = ((this._wifiToX(WIFI_SCAN_BANDS[1][0]) - this._wifiToX(WIFI_SCAN_BANDS[0][1])) * 100) + '%';
        inner.appendChild(gap);
        WIFI_TIERS.forEach((tier, ti) => {
            const row = document.createElement('div');
            row.className = 'tier-row';
            row.style.top = (ti * 32) + 'px';
            for (const t of tier.tiles) {
                const el = document.createElement('div');
                el.className = 'tile';
                const x0 = this._wifiToX(t.f0), x1 = this._wifiToX(t.f1);
                el.style.left = (x0 * 100) + '%';
                el.style.width = ((x1 - x0) * 100) + '%';
                el.textContent = t.label;
                el.dataset.f0 = t.f0;
                el.dataset.f1 = t.f1;
                el.dataset.bw = t.bw;
                row.appendChild(el);
            }
            inner.appendChild(row);
        });

        const selectTab = (tid, apply) => {
            const prev = this._freqTab;
            this._freqTab = tid;
            this.s.freqTab = tid;
            $('ftab-range').classList.toggle('on', tid === 'range');
            $('ftab-wifi').classList.toggle('on', tid === 'wifi');
            $('freq-range').style.display = tid === 'range' ? '' : 'none';
            $('freq-wifi').style.display = tid === 'wifi' ? '' : 'none';
            if (tid === 'wifi') {
                this._restoreWifiTiles();
                this._commitWifiSel();
            } else if (apply !== false && prev === 'wifi') {
                this._applyManualRange(true);
            } else {
                this._syncFreqColor();
            }
            this._placeFft();
            this._layoutHudPops();
        };
        $('ftab-range').onclick = () => { selectTab('range'); this.save(); };
        $('ftab-wifi').onclick = () => { selectTab('wifi'); this.save(); };
        this._selectFreqTab = selectTab;
        $('btn-fft').onclick = (e) => {
            e.stopPropagation();
            this.s.fft = !this.s.fft;
            this.save();
            this.net.set({ spectrum: this.s.fft ? 1 : 0 });
            this._placeFft();
        };
        $('btn-fft-speed').onclick = (e) => {
            e.stopPropagation();
            const i = FFT_SPEEDS.indexOf(this.s.fftSpeed);
            this.s.fftSpeed = FFT_SPEEDS[(i < 0 ? 0 : i + 1) % FFT_SPEEDS.length];
            this.save();
            this._syncFftOpts();
        };
        $('btn-fft-agc').onclick = (e) => {
            e.stopPropagation();
            this.s.fftAgc = !this.s.fftAgc;
            if (!this.s.fftAgc && this._fftAvg)
                this._fftHold = this._fftAvg.slice();
            else
                this._fftHold = null;
            this.save();
            this._syncFftOpts();
            this._drawFft();
        };
        this._bindWifiPaint(inner);
        this._restoreWifiTiles();
        selectTab(this._freqTab, false);
        this._placeFft();
    }

    _restoreWifiTiles() {
        const want = new Set((this.s.wifiSel || []).map(b => `${b[0]}-${b[1]}`));
        for (const el of document.querySelectorAll('#wifi-tiers .tile'))
            el.classList.toggle('on', want.has(`${el.dataset.f0}-${el.dataset.f1}`));
    }

    _commitWifiSel() {
        const sel = [...document.querySelectorAll('#wifi-tiers .tile.on')]
            .map(el => [parseFloat(el.dataset.f0), parseFloat(el.dataset.f1)]);
        this.s.wifiSel = sel;
        this.save();
        if (!sel.length) this._applyWifiScan();
        else this.net.set({ lo_start: WIFI_F0, lo_end: WIFI_F1, bands: mergeBands(sel) });
        this._syncFreqColor();
    }

    _tile20AtFreq(f) {
        for (const el of document.querySelectorAll('#wifi-tiers .tile[data-bw="20"]')) {
            const f0 = parseFloat(el.dataset.f0), f1 = parseFloat(el.dataset.f1);
            if (f >= f0 && f < f1) return el;
        }
        return null;
    }

    _bindWifiPaint(inner) {
        let paint = null;
        let only20 = false;
        const tile20AtX = (clientX) => {
            const cv = $('fft-canvas');
            if (!cv) return null;
            const r = cv.getBoundingClientRect();
            const x = (clientX - r.left) / r.width;
            return this._tile20AtFreq(this._wifiXToF(Math.max(0, Math.min(1, x))));
        };
        const tileAt = (clientX, clientY) => {
            if (only20) return tile20AtX(clientX);
            const el = document.elementFromPoint(clientX, clientY);
            return el && el.closest ? el.closest('#wifi-tiers .tile') : null;
        };
        const paintTile = (el) => {
            if (!el || paint === null) return;
            el.classList.toggle('on', paint);
        };
        const start = (e, el, lock20) => {
            if (!el) return;
            only20 = !!lock20;
            paint = !el.classList.contains('on');
            paintTile(el);
            try { e.currentTarget.setPointerCapture(e.pointerId); } catch (_) {}
            e.preventDefault();
        };
        inner.addEventListener('pointerdown', (e) => {
            const el = e.target.closest('.tile');
            if (!el) return;
            start(e, el, el.dataset.bw === '20');
        });
        const fft = $('fft-wrap');
        if (fft) {
            fft.addEventListener('pointerdown', (e) => {
                if (this._freqTab !== 'wifi') return;
                start(e, tile20AtX(e.clientX), true);
            });
            fft.addEventListener('pointermove', (e) => {
                if (paint === null || !only20) return;
                paintTile(tile20AtX(e.clientX));
            });
        }
        inner.addEventListener('pointermove', (e) => {
            if (paint === null) return;
            paintTile(tileAt(e.clientX, e.clientY));
        });
        const end = () => {
            if (paint === null) return;
            paint = null;
            only20 = false;
            this._commitWifiSel();
        };
        inner.addEventListener('pointerup', end);
        inner.addEventListener('pointercancel', end);
        if (fft) {
            fft.addEventListener('pointerup', end);
            fft.addEventListener('pointercancel', end);
        }
    }

    _applyWifiScan() {
        this.net.set({ lo_start: WIFI_F0, lo_end: WIFI_F1, bands: WIFI_HOP_BANDS });
    }

    onSpectrum(header, f32) {
        const n = f32.length;
        const now = performance.now();
        const dt = this._fftT ? Math.min(0.25, (now - this._fftT) / 1000) : 0.03;
        this._fftT = now;
        this._fftDt = dt;
        const tau = FFT_TAU[this.s.fftSpeed] || FFT_TAU.med;
        const a = 1 - Math.exp(-dt / tau);
        if (!this._fftAvg || this._fftAvg.length !== n) {
            this._fftAvg = Float32Array.from(f32);
            this._fftMiss = new Uint16Array(n);
        } else {
            const avg = this._fftAvg;
            const shift = this._fftCoherentShift(f32);
            if (shift) {
                for (let i = 0; i < n; i++) if (avg[i] > 1e-8) avg[i] += shift;
                this._fftScale += shift;
                if (this._fftLo > 0) this._fftLo += shift;
                this._fftSnapped = true;
            }
            const miss = this._fftMiss;
            for (let i = 0; i < n; i++) {
                const raw = f32[i];
                if (raw <= 1e-8) {
                    if (++miss[i] > FFT_MISS_HOLD) avg[i] = 0;
                    continue;
                }
                miss[i] = 0;
                /* First visit after a wider plan: don't slew up from 0. */
                if (avg[i] <= 1e-8) avg[i] = raw;
                else avg[i] += a * (raw - avg[i]);
            }
        }
        this.spectrum = this._fftAvg;
        this._specF0 = header.loStart;
        this._specF1 = header.loEnd;
        this._fftSrvNorm = !!(header.sweeps & 1);
        const lo0 = (header.sweeps >>> 1) / 10;
        const uiSig = `${this._freqTab}|${this.s.manualLo}|${this.s.manualHi}|${(this.s.wifiSel || []).join(',')}`;
        if (lo0 !== this._fftPlanLo0 || uiSig !== this._fftUiSig) {
            this._fftPlanLo0 = lo0;
            this._fftUiSig = uiSig;
            this._fftPlanT = now;
            this._fftPlanPending = true;
        }
        if (this._fftPlanPending && now - this._fftPlanT > FFT_PLAN_SNAP_MS) {
            this._fftPlanPending = false;
            this._fftSnapped = true;
        }
        if (!this.s.fftAgc && !this._fftHold)
            this._fftHold = this._fftAvg.slice();
        this._updateFftShape(this._fftAvg);
        this._updateFftScale();
        if ($('freq-pop').classList.contains('open') && this.s.fft)
            this._drawFft();
    }

    _fftView() {
        const specF0 = this._specF0 ?? HW_MIN;
        const specF1 = this._specF1 ?? HW_MAX;
        const n = this._fftAvg ? this._fftAvg.length : 0;
        const bin = n ? (specF1 - specF0) / n : 1;
        const view0 = this._freqTab === 'wifi' ? WIFI_F0 : HW_MIN;
        const view1 = this._freqTab === 'wifi' ? WIFI_F1 : HW_MAX;
        const i0 = Math.max(0, Math.floor((view0 - specF0) / bin));
        const i1 = Math.min(n, Math.ceil((view1 - specF0) / bin));
        return { view0, view1, specF0, specF1, bin, i0, i1, n };
    }

    _fftHopLo(f) {
        let start;
        if (this._freqTab === 'wifi') {
            if (f < 5410) start = 5170;
            else if (f < 5732.5) start = 5490;
            else start = 5735;
        } else {
            start = this._fftPlanLo0 > 0 ? this._fftPlanLo0 - FFT_HOP_MHZ / 2 : this.s.manualLo;
        }
        const lo0 = start + FFT_HOP_MHZ / 2;
        return lo0 + FFT_HOP_MHZ * Math.round((f - lo0) / FFT_HOP_MHZ);
    }

    _fftHopIdx(f) {
        const lo = this._fftHopLo(f);
        let i = Math.round((f - lo) + FFT_HOP_MHZ / 2);
        if (i < 0) i = 0;
        if (i >= FFT_EQ_N) i = FFT_EQ_N - 1;
        return i;
    }

    /* Median of (raw-avg) when the move is the same on every filled bin. */
    _fftCoherentShift(raw) {
        const avg = this._fftAvg;
        if (!avg || avg.length !== raw.length) return 0;
        const d = this._fftDelta;
        d.length = 0;
        for (let i = 0; i < raw.length; i++) {
            if (raw[i] > 1e-8 && avg[i] > 1e-8) d.push(raw[i] - avg[i]);
        }
        if (d.length < 40) return 0;
        d.sort((x, y) => x - y);
        const med = d[d.length >> 1];
        const q1 = d[Math.floor(0.25 * (d.length - 1))];
        const q3 = d[Math.floor(0.75 * (d.length - 1))];
        if (Math.abs(med) < FFT_SHIFT_MIN || q3 - q1 > FFT_SHIFT_IQR) return 0;
        return med;
    }

    /* A parked LO has no retune settling in its span, so its skirts
     * differ from the same LO inside a sweep (-0.06 vs -0.12 ln at
     * 5745); it gets its own banks. */
    _fftBank(g) {
        const set = (this._fftSrvNorm ? 1 : 0) + (this._fftStatic ? 2 : 0);
        return set * (RF_GAIN_MAX + 1) + g;
    }

    /* Offsets at `lo` from one bank: exact, else interpolated between
     * learned LOs at the same 80 MHz phase, else linear between the
     * nearest learned LOs within FFT_EQ_REACH. */
    _fftBankAt(b, lo, out) {
        const i = Math.round(lo) - FFT_EQ_LO0;
        if (i < 0 || i >= FFT_EQ_LO_N) return false;
        const s = b.s, n = b.n, N = FFT_EQ_N;
        if (n[i]) {
            for (let k = 0; k < N; k++) out[k] = s[i * N + k];
            return true;
        }
        let il = -1, ih = -1;
        for (let m = 1; m <= FFT_EQ_PHASE_WIN && (il < 0 || ih < 0); m++) {
            const d = m * FFT_EQ_PHASE;
            if (il < 0 && i - d >= 0 && n[i - d]) il = i - d;
            if (ih < 0 && i + d < FFT_EQ_LO_N && n[i + d]) ih = i + d;
        }
        if (il >= 0 || ih >= 0) {
            const j = il < 0 ? ih : ih < 0 ? il : -1;
            if (j >= 0) {
                for (let k = 0; k < N; k++) out[k] = s[j * N + k];
            } else {
                const w = (i - il) / (ih - il);
                for (let k = 0; k < N; k++) out[k] = s[il * N + k] * (1 - w) + s[ih * N + k] * w;
            }
            return true;
        }
        for (let d = 1; d <= FFT_EQ_REACH && (il < 0 || ih < 0); d++) {
            if (il < 0 && i - d >= 0 && n[i - d]) il = i - d;
            if (ih < 0 && i + d < FFT_EQ_LO_N && n[i + d]) ih = i + d;
        }
        if (il < 0 && ih < 0) return false;
        if (il < 0 || ih < 0) {
            const j = il < 0 ? ih : il;
            for (let k = 0; k < N; k++) out[k] = s[j * N + k];
            return true;
        }
        const w = (i - il) / (ih - il);
        for (let k = 0; k < N; k++) out[k] = s[il * N + k] * (1 - w) + s[ih * N + k] * w;
        return true;
    }

    /* This gain's table, else the nearest gain within ±6 steps that has
     * this LO. Offsets are relative to the hop median, so only shape
     * carries across gains. Returns the gain used, or -1. */
    _fftShapeAt(lo, out) {
        const g = this.s.hwGain | 0;
        if (g < 0 || g > RF_GAIN_MAX) return -1;
        const banks = this._fftBanks;
        for (let d = 0; d <= 6; d++) {
            for (const gg of [g + d, g - d]) {
                if (gg < 0 || gg > RF_GAIN_MAX || (d === 0 && gg !== g)) continue;
                const b = banks[this._fftBank(gg)];
                if (!b || !this._fftBankAt(b, lo, out)) continue;
                this._fftDetailAt(b, lo, out);
                return gg;
            }
        }
        return -1;
    }

    _fftBankGet(bi) {
        const N = FFT_EQ_N;
        return this._fftBanks[bi] || (this._fftBanks[bi] = {
            s: new Float32Array(FFT_EQ_LO_N * N), n: new Uint8Array(FFT_EQ_LO_N),
            d: new Float32Array(FFT_EQ_LO_N * N), dn: new Uint8Array(FFT_EQ_LO_N),
            lv: new Float32Array(FFT_EQ_LO_N) });
    }

    /* Parked LO: no neighbours to vote, so learn over time instead, and
     * only while the hop sits at its own floor. A Wi-Fi channel that
     * fills the whole hop has a flat passband too; its level gives it
     * away. The floor tracker drops at once and creeps up 0.02 ln/s. */
    _learnFftParked(h, g) {
        const li = h.lo - FFT_EQ_LO0;
        if (li < 0 || li >= FFT_EQ_LO_N) return;
        const bank = this._fftBankGet(this._fftBank(g));
        const dt = this._fftDt || 0.03;
        if (!bank.lv[li] || h.med < bank.lv[li]) bank.lv[li] = h.med;
        else bank.lv[li] += 0.02 * dt;
        if (h.med - bank.lv[li] > FFT_EQ_QUIET_LVL) return;
        const col = this._fftCol;
        col.length = 0;
        for (const k of FFT_EQ_PASS) if (h.rel[k] === h.rel[k]) col.push(h.rel[k]);
        if (col.length < 8) return;
        col.sort((x, y) => x - y);
        if (col[Math.floor(0.9 * (col.length - 1))] - col[Math.floor(0.1 * (col.length - 1))] > FFT_EQ_QUIET_IQR)
            return;
        const a = 1 - Math.exp(-dt / FFT_EQ_DETAIL_TAU);
        const seed = !bank.n[li];
        for (const k of FFT_EQ_DETAIL) {
            const x = h.rel[k];
            if (x !== x) continue;
            const p = li * FFT_EQ_N + k;
            const r = Math.max(-FFT_EQ_DETAIL_CLIP, Math.min(FFT_EQ_DETAIL_CLIP, x));
            bank.s[p] = seed ? r : bank.s[p] + a * (r - bank.s[p]);
        }
        if (bank.n[li] < 255) bank.n[li]++;
    }

    /* Per-LO skirt/DC term: exact LO only, it does not interpolate. */
    _fftDetailAt(b, lo, out) {
        const i = Math.round(lo) - FFT_EQ_LO0;
        if (i < 0 || i >= FFT_EQ_LO_N || !b.dn[i]) return;
        for (const k of FFT_EQ_DETAIL) out[k] += b.d[i * FFT_EQ_N + k];
    }

    /* FFT-canvas only. Do not use on ingest / sphere intensity. */
    _fftEqI(v, i) {
        const c = this._fftCorr;
        return c && i < c.length ? v - c[i] : v;
    }

    _fftEqV(v, f) {
        const n = this._fftAvg ? this._fftAvg.length : 0;
        if (!n) return v;
        const f0 = this._specF0 ?? HW_MIN, f1 = this._specF1 ?? HW_MAX;
        return this._fftEqI(v, Math.round((f - f0) / ((f1 - f0) / n)));
    }

    _updateFftShape(src) {
        const n = src ? src.length : 0;
        if (!n) return;
        const N = FFT_EQ_N;
        if (!this._fftCorr || this._fftCorr.length !== n) {
            this._fftCorr = new Float32Array(n);
            this._fftBinHop = new Array(n);
            this._fftBinTap = new Uint8Array(n);
        }
        const corr = this._fftCorr, binHop = this._fftBinHop, binTap = this._fftBinTap;
        const specF0 = this._specF0 ?? HW_MIN;
        const specF1 = this._specF1 ?? HW_MAX;
        const bin = (specF1 - specF0) / n;
        const map = this._fftHopMap;
        if (map.size > 256) map.clear();
        for (const h of map.values()) { h.cnt = 0; h.v.fill(NaN); }
        for (let i = 0; i < n; i++) {
            const f = specF0 + i * bin;
            const lo = Math.round(this._fftHopLo(f));
            let h = map.get(lo);
            if (!h) {
                h = { lo, cnt: 0, med: 0, has: false,
                      v: new Float32Array(N).fill(NaN), rel: new Float32Array(N),
                      shape: new Float32Array(N) };
                map.set(lo, h);
            }
            const k = this._fftHopIdx(f);
            binHop[i] = h;
            binTap[i] = k;
            const v = src[i];
            if (v > 1e-8) {
                if (h.v[k] !== h.v[k]) h.cnt++;
                h.v[k] = v;
            }
        }
        const list = this._fftHopList;
        list.length = 0;
        const col = this._fftCol || (this._fftCol = []);
        for (const h of map.values()) {
            if (h.cnt < FFT_EQ_FILL) continue;
            col.length = 0;
            for (let k = 0; k < N; k++) if (h.v[k] === h.v[k]) col.push(h.v[k]);
            col.sort((x, y) => x - y);
            h.med = col[col.length >> 1];
            for (let k = 0; k < N; k++) h.rel[k] = h.v[k] - h.med;
            list.push(h);
        }
        list.sort((x, y) => x.lo - y.lo);
        this._fftStatic = list.length === 1;

        const g = this.s.hwGain | 0;
        const settled = !this._fftPlanT || performance.now() - this._fftPlanT > FFT_PLAN_LEARN_MS;
        const learn = settled && this.s.fftAgc && g >= 0 && g <= RF_GAIN_MAX;
        if (learn && this._fftStatic) this._learnFftParked(list[0], g);
        if (learn && list.length >= FFT_EQ_MIN_HOPS) {
            const bank = this._fftBankGet(this._fftBank(g));
            const dt = this._fftDt || 0.03;
            const a = 1 - Math.exp(-dt / FFT_EQ_TAU);
            const ad = 1 - Math.exp(-dt / FFT_EQ_DETAIL_TAU);
            const reach = FFT_EQ_WIN * FFT_HOP_MHZ;
            const preach = FFT_EQ_PHASE_WIN * FFT_EQ_PHASE;
            const vot = this._fftVot || (this._fftVot = []);
            let j0 = 0, j1 = 0, p0 = 0, p1 = 0;
            for (let i = 0; i < list.length; i++) {
                const h = list[i];
                const li = h.lo - FFT_EQ_LO0;
                while (h.lo - list[j0].lo > reach) j0++;
                if (j1 < i) j1 = i;
                while (j1 + 1 < list.length && list[j1 + 1].lo - h.lo <= reach) j1++;
                while (h.lo - list[p0].lo > preach) p0++;
                if (p1 < i) p1 = i;
                while (p1 + 1 < list.length && list[p1 + 1].lo - h.lo <= preach) p1++;
                if (li < 0 || li >= FFT_EQ_LO_N) continue;
                vot.length = 0;
                for (let j = p0; j <= p1; j++)
                    if ((list[j].lo - h.lo) % FFT_EQ_PHASE === 0) vot.push(list[j]);
                let need = FFT_EQ_PHASE_MIN;
                if (vot.length < need) {
                    vot.length = 0;
                    for (let j = j0; j <= j1; j++) vot.push(list[j]);
                    need = FFT_EQ_MIN_HOPS;
                    if (vot.length < need) continue;
                }
                const seed = !bank.n[li];
                let touched = 0;
                for (let k = 0; k < N; k++) {
                    col.length = 0;
                    for (let j = 0; j < vot.length; j++) {
                        const x = vot[j].rel[k];
                        if (x === x) col.push(x);
                    }
                    if (col.length < need) continue;
                    col.sort((x, y) => x - y);
                    const m = col.length >> 1;
                    const est = col.length & 1 ? col[m] : 0.5 * (col[m - 1] + col[m]);
                    const p = li * N + k;
                    bank.s[p] = seed ? est : bank.s[p] + a * (est - bank.s[p]);
                    touched++;
                }
                if (!touched) continue;
                if (bank.n[li] < 255) bank.n[li]++;

                col.length = 0;
                for (let j = j0; j <= j1; j++) if (j !== i) col.push(list[j].med);
                if (col.length < 2) continue;
                col.sort((x, y) => x - y);
                if (h.med - col[col.length >> 1] > FFT_EQ_QUIET_LVL) continue;
                col.length = 0;
                for (const k of FFT_EQ_PASS) if (h.rel[k] === h.rel[k]) col.push(h.rel[k]);
                if (col.length < 8) continue;
                col.sort((x, y) => x - y);
                const spread = col[Math.floor(0.9 * (col.length - 1))] - col[Math.floor(0.1 * (col.length - 1))];
                if (spread > FFT_EQ_QUIET_IQR) continue;
                bank.lv[li] = bank.lv[li] ? bank.lv[li] + ad * (h.med - bank.lv[li]) : h.med;
                const dseed = !bank.dn[li];
                for (const k of FFT_EQ_DETAIL) {
                    const x = h.rel[k];
                    if (x !== x) continue;
                    const p = li * N + k;
                    const r = Math.max(-FFT_EQ_DETAIL_CLIP, Math.min(FFT_EQ_DETAIL_CLIP, x - bank.s[p]));
                    bank.d[p] = dseed ? r : bank.d[p] + ad * (r - bank.d[p]);
                }
                if (bank.dn[li] < 255) bank.dn[li]++;
            }
        }

        let key = -1;
        for (const h of map.values()) {
            const gu = h.cnt > 0 ? this._fftShapeAt(h.lo, h.shape) : -1;
            h.has = gu >= 0;
            if (h.has && key < 0) key = this._fftBank(gu);
        }
        for (let i = 0; i < n; i++) {
            const h = binHop[i];
            corr[i] = h.has ? h.shape[binTap[i]] : 0;
        }
        this._fftCorrStep = key !== this._fftCorrKey;
        this._fftCorrKey = key;
    }

    _fftPcts(src) {
        const { i0, i1, specF0, bin } = this._fftView();
        const wifi = this._freqTab === 'wifi';
        const a1 = WIFI_SCAN_BANDS[0][1], b0 = WIFI_SCAN_BANDS[1][0];
        const out = this._fftScratch;
        out.length = 0;
        for (let i = i0; i < i1; i++) {
            const raw = src[i];
            if (raw <= 1e-8) continue;
            const f = specF0 + i * bin;
            if (wifi && f >= a1 && f < b0) continue;
            out.push(this._fftEqI(raw, i));
        }
        if (out.length < 8) return { p10: 0, p98: 0, n: out.length };
        out.sort((x, y) => x - y);
        return {
            p10: out[Math.floor(0.10 * (out.length - 1))],
            p98: out[Math.floor(0.98 * (out.length - 1))],
            n: out.length,
        };
    }

    _updateFftScale() {
        if (!this._fftAvg) return;
        const shapeStep = !!this._fftCorrStep;
        this._fftCorrStep = false;
        const { p10, p98, n } = this._fftPcts(this._fftAvg);
        if (n < 8) return;
        if (this._fftScale <= 1e-6) this._fftScale = p98;
        if (this._fftLo <= 0) this._fftLo = p10;
        const snap = !this.s.fftAgc ? false : (this._fftSnapped || shapeStep);
        this._fftSnapped = false;
        if (!this.s.fftAgc) return;
        if (snap) {
            this._fftScale = p98;
            this._fftLo = Math.min(p10, p98 - 0.05);
            return;
        }
        const dt = this._fftDt || 0.03;
        const atk = 1 - Math.exp(-dt / FFT_AGC_ATK);
        const rel = 1 - Math.exp(-dt / FFT_AGC_REL);
        this._fftScale += (p98 > this._fftScale ? atk : rel) * (p98 - this._fftScale);
        const flo = 1 - Math.exp(-dt / (p10 > this._fftLo ? FFT_FLOOR_ATK : FFT_FLOOR_REL));
        this._fftLo += flo * (p10 - this._fftLo);
        if (this._fftLo > this._fftScale - 0.05)
            this._fftLo = this._fftScale - 0.05;
    }

    _drawFft() {
        const cv = $('fft-canvas');
        if (!cv || !this.s.fft) return;
        const w = cv.clientWidth, h = cv.clientHeight || 48;
        if (w < 2) return;
        if (cv.width !== w) cv.width = w;
        if (cv.height !== h) cv.height = h;
        const ctx = cv.getContext('2d');
        ctx.clearRect(0, 0, w, h);
        const src = (!this.s.fftAgc && this._fftHold) ? this._fftHold : this._fftAvg;
        if (!src || !src.length) return;
        const { view0, view1, specF0, bin, i0, i1 } = this._fftView();
        const lo = this._fftLo > 0 ? this._fftLo : 0;
        const hi = Math.max(this._fftScale, lo * FFT_Y_MIN_RATIO, 1e-6);
        const base = lo * (1 - FFT_LO_PAD);
        const span = Math.max(hi - base, 1e-6);
        const wifi = this._freqTab === 'wifi';
        if (wifi) {
            const gx0 = this._wifiToX(WIFI_SCAN_BANDS[0][1]) * w;
            const gx1 = this._wifiToX(WIFI_SCAN_BANDS[1][0]) * w;
            ctx.fillStyle = 'rgba(255,255,255,0.035)';
            ctx.fillRect(gx0, 0, Math.max(gx1 - gx0, 1), h);
        }
        const scheme = this.s.scheme;
        const lut = this._fftLut();
        const inten = (SCHEMES[scheme] || SCHEMES.spectrum).mode === 1;
        if (inten) {
            const g = ctx.createLinearGradient(0, h, 0, 0);
            for (let s = 0; s <= 8; s++) {
                const [cr, cg, cb] = lutRgb(lut, s / 8);
                g.addColorStop(s / 8, `rgba(${cr},${cg},${cb},0.70)`);
            }
            ctx.fillStyle = g;
        }
        const a1 = WIFI_SCAN_BANDS[0][1], b0 = WIFI_SCAN_BANDS[1][0];
        const chan = this._chanRanges;
        for (let i = i0; i < i1; i++) {
            const raw = src[i];
            if (raw <= 0) continue;
            const bf0 = specF0 + i * bin;
            if (wifi && bf0 >= a1 && bf0 < b0) continue;
            const v = this._fftEqI(raw, i);
            const vh = Math.min(1, Math.max(0, (v - base) / span));
            if (vh <= 0) continue;
            const x0 = wifi ? this._wifiToX(bf0) * w : ((bf0 - view0) / (view1 - view0)) * w;
            const x1 = wifi ? this._wifiToX(bf0 + bin) * w : ((bf0 + bin - view0) / (view1 - view0)) * w;
            if (!inten) {
                const col = this._fftBinCss(bf0, lut, chan);
                if (!col) continue;
                ctx.fillStyle = col;
            }
            ctx.fillRect(x0, h * (1 - vh), Math.max(x1 - x0, 0.5), vh * h);
        }
        if (scheme === 'target') this._drawTargetMark(ctx, w, h, wifi, view0, view1);
    }

    _fftLut() {
        if (this.s.scheme === 'target') {
            const n = TARGET_LUTS.includes(this.s.targetLut) ? this.s.targetLut : 'iron';
            return SCHEMES[n].lut;
        }
        return (SCHEMES[this.s.scheme] || SCHEMES.spectrum).lut;
    }

    _fftBinCss(f, lut, chan) {
        if (this.s.scheme === 'target') {
            const d = Math.abs(f - this.s.targetFreq) / Math.max(this.s.targetWidth, 0.1);
            if (d > 1) return 'rgba(71,76,87,0.35)';
            const [r, g, b] = lutRgb(lut, 1 - d);
            return `rgba(${r},${g},${b},0.70)`;
        }
        let t;
        if (chan && chan.length) {
            const hit = chan.find(c => f >= c.f0 && f < c.f1);
            if (!hit) return 'rgba(71,76,87,0.30)';
            t = hit.t;
        } else if (this._freqTab === 'wifi') {
            if (this.s.wifiColor === 'full')
                t = (f - HW_MIN) / (HW_MAX - HW_MIN);
            else
                t = (f - WIFI_F0) / Math.max(WIFI_F1 - WIFI_F0, 1);
        } else if (this.s.freqPin) {
            t = (f - HW_MIN) / (HW_MAX - HW_MIN);
        } else {
            t = (f - this.s.manualLo) / Math.max(this.s.manualHi - this.s.manualLo, 1);
        }
        const [r, g, b] = lutRgb(lut, Math.max(0, Math.min(1, t)));
        return `rgba(${r},${g},${b},0.70)`;
    }

    _drawTargetMark(ctx, w, h, wifi, view0, view1) {
        const f = this.s.targetFreq;
        const x = wifi ? this._wifiToX(f) * w : ((f - view0) / (view1 - view0)) * w;
        if (x < 0 || x > w) return;
        ctx.fillStyle = 'rgba(255,255,255,0.45)';
        ctx.fillRect(x - 0.5, 0, 1, h);
    }

    // ---------- manual dual-thumb range ----------

    _bindManual() {
        const bar = $('range-bar');
        const hit = $('range-hit');
        const thumbs = { lo: $('thumb-lo'), hi: $('thumb-hi') };
        const midEl = $('thumb-mid');
        const layout = () => {
            const tl = this._freqToX(this.s.manualLo), th = this._freqToX(this.s.manualHi);
            const mid = 0.5 * (this.s.manualLo + this.s.manualHi);
            thumbs.lo.style.left = (tl * 100) + '%';
            thumbs.hi.style.left = (th * 100) + '%';
            midEl.style.left = (this._freqToX(mid) * 100) + '%';
            $('range-fill').style.left = (tl * 100) + '%';
            $('range-fill').style.width = ((th - tl) * 100) + '%';
            thumbs.lo.querySelector('.rlab').textContent = `${this.s.manualLo.toFixed(0)} MHZ`;
            thumbs.hi.querySelector('.rlab').textContent = `${this.s.manualHi.toFixed(0)} MHZ`;
            midEl.querySelector('.rlab').textContent = `${mid.toFixed(0)}`;
        };
        this._layoutManual = layout;
        const applyAt = (clientX, key) => {
            const r = bar.getBoundingClientRect();
            let t = (clientX - r.left) / r.width;
            t = Math.max(0, Math.min(1, t));
            const f = Math.round(HW_MIN + t * (HW_MAX - HW_MIN));
            if (key === 'mid' || this.s.scheme === 'target') {
                this._applyRangeKeyed(key, f);
                if (this.s.scheme === 'target') this._syncTargetFromRange();
            } else if (key === 'lo') {
                this.s.manualLo = Math.min(f, this.s.manualHi - 18);
            } else {
                this.s.manualHi = Math.max(f, this.s.manualLo + 18);
            }
            layout();
            this._applyManualRange(false);
        };
        let dragKey = null;
        const move = (ev) => { if (dragKey) applyAt(ev.clientX, dragKey); };
        const up = () => {
            if (!dragKey) return;
            dragKey = null;
            document.removeEventListener('pointermove', move, true);
            document.removeEventListener('mousemove', move, true);
            document.removeEventListener('pointerup', up, true);
            document.removeEventListener('mouseup', up, true);
            this._applyManualRange(true);
            this.save();
        };
        const startDrag = (e, key) => {
            applyAt(e.clientX, key);
            if (dragKey) return;
            dragKey = key;
            try { e.currentTarget.setPointerCapture(e.pointerId); } catch (_) {}
            document.addEventListener('pointermove', move, true);
            document.addEventListener('mousemove', move, true);
            document.addEventListener('pointerup', up, true);
            document.addEventListener('mouseup', up, true);
            e.preventDefault();
            e.stopPropagation();
        };
        const pickKey = (clientX) => {
            const r = bar.getBoundingClientRect();
            let t = (clientX - r.left) / r.width;
            t = Math.max(0, Math.min(1, t));
            const f = Math.round(HW_MIN + t * (HW_MAX - HW_MIN));
            const mid = 0.5 * (this.s.manualLo + this.s.manualHi);
            const dLo = Math.abs(f - this.s.manualLo);
            const dHi = Math.abs(f - this.s.manualHi);
            const dMid = Math.abs(f - mid);
            if (dMid <= dLo && dMid <= dHi) return 'mid';
            return dLo <= dHi ? 'lo' : 'hi';
        };
        for (const key of ['lo', 'hi']) {
            thumbs[key].addEventListener('pointerdown', (e) => startDrag(e, key));
            thumbs[key].addEventListener('mousedown', (e) => startDrag(e, key));
        }
        midEl.addEventListener('pointerdown', (e) => startDrag(e, 'mid'));
        midEl.addEventListener('mousedown', (e) => startDrag(e, 'mid'));
        const onBarDown = (e) => {
            if (e.target.closest('.rthumb') || e.target.closest('.rmid')) return;
            startDrag(e, pickKey(e.clientX));
        };
        hit.addEventListener('pointerdown', onBarDown);
        hit.addEventListener('mousedown', onBarDown);
        $('freq-reset').onclick = () => {
            if (this._freqTab === 'wifi') {
                for (const el of document.querySelectorAll('#wifi-tiers .tile.on'))
                    el.classList.remove('on');
                this._commitWifiSel();
                return;
            }
            this.s.manualLo = HW_MIN;
            this.s.manualHi = HW_MAX;
            if (this.s.scheme === 'target') this._syncTargetFromRange();
            layout();
            this._applyManualRange(true);
            this.save();
        };
        layout();
    }

    _applyRangeKeyed(key, f) {
        const minHalf = 9;
        if (key === 'mid') {
            const half = Math.max(minHalf, 0.5 * (this.s.manualHi - this.s.manualLo));
            let c = f;
            if (c - half < HW_MIN) c = HW_MIN + half;
            if (c + half > HW_MAX) c = HW_MAX - half;
            this.s.manualLo = Math.round(c - half);
            this.s.manualHi = Math.round(c + half);
            return;
        }
        const c = this.s.scheme === 'target'
            ? this.s.targetFreq
            : 0.5 * (this.s.manualLo + this.s.manualHi);
        let half = key === 'lo' ? (c - f) : (f - c);
        half = Math.max(minHalf, half);
        const maxHalf = Math.min(c - HW_MIN, HW_MAX - c);
        if (half > maxHalf) half = maxHalf;
        this.s.manualLo = Math.round(c - half);
        this.s.manualHi = Math.round(c + half);
    }

    _syncTargetFromRange() {
        const mid = 0.5 * (this.s.manualLo + this.s.manualHi);
        const half = 0.5 * (this.s.manualHi - this.s.manualLo);
        this.s.targetFreq = Math.round(mid);
        this.s.targetWidth = Math.max(9, Math.round(half));
        this.renderer.setTarget(this.s.targetFreq, this.s.targetWidth);
        if (!$('target-freq')) return;
        this._clampTargetWidth();
        $('target-freq').value = this.s.targetFreq;
        $('target-freq-val').textContent = `${this.s.targetFreq} MHZ`;
        $('target-width-val').textContent = `\u00b1${this.s.targetWidth} MHZ`;
    }

    _applyManualRange(force) {
        const now = performance.now();
        if (!force && now - this._rangeSendTimer < 80) return;
        this._rangeSendTimer = now;
        this.net.set({ lo_start: this.s.manualLo, lo_end: this.s.manualHi, bands: [] });
        this._syncFreqColor();
    }

    // ==================================================================
    // COLOR panel
    // ==================================================================

    _applySchemeLut() {
        const s = this.s;
        if (s.scheme === 'target') {
            const lutName = TARGET_LUTS.includes(s.targetLut) ? s.targetLut : 'iron';
            this.renderer.setLut(SCHEMES[lutName].lut, 2);
            this._syncFreqColor();
            return;
        }
        const sc = SCHEMES[s.scheme] || SCHEMES.spectrum;
        this.renderer.setLut(sc.lut, sc.mode);
        this._syncFreqColor();
    }

    /* RANGE PIN: HSV locked to 4900–6100. Off: stretch to the thumbs.
     * WIFI BAND / CHAN / FULL: 5170–5895 stretch, per-channel hues, or 4900–6100. */
    _wifiChanRanges() {
        const tiles = [...document.querySelectorAll('#wifi-tiers .tile.on')]
            .map(el => {
                const f0 = parseFloat(el.dataset.f0), f1 = parseFloat(el.dataset.f1);
                return { f0, f1, fc: 0.5 * (f0 + f1) };
            })
            .sort((a, b) => a.fc - b.fc);
        const n = tiles.length;
        if (!n) return null;
        let ts;
        if (n === 1) ts = [0.5];
        else if (n === 2) ts = [0, 1];
        else {
            /* Neighbors at least 60 MHz of hue distance so they stay distinct. */
            const gaps = [];
            for (let i = 0; i < n - 1; i++)
                gaps.push(Math.max(tiles[i + 1].fc - tiles[i].fc, 60));
            let acc = 0;
            const total = gaps.reduce((a, b) => a + b, 0);
            ts = [0];
            for (const g of gaps) { acc += g / total; ts.push(acc); }
        }
        return tiles.map((t, i) => ({ f0: t.f0, f1: t.f1, t: ts[i] }));
    }

    _syncFreqColor() {
        const pin = $('btn-freq-pin'), wcol = $('btn-wifi-chan');
        if (pin) pin.classList.toggle('on', !!this.s.freqPin);
        if (wcol) {
            wcol.textContent = WIFI_COLOR_LAB[this.s.wifiColor] || 'BAND';
            wcol.classList.toggle('on', this.s.wifiColor !== 'band');
        }
        this._chanRanges = null;
        if (this.s.scheme === 'spectrum' && this._freqTab === 'wifi'
                && this.s.wifiColor === 'chan') {
            const ranges = this._wifiChanRanges();
            if (ranges) {
                this._chanRanges = ranges;
                this.renderer.setChanMap(ranges);
                if ($('freq-pop').classList.contains('open') && this.s.fft)
                    this._drawFft();
                return;
            }
        }
        this.renderer.setChanMap(null);
        if (this._freqTab === 'wifi') {
            if (this.s.wifiColor === 'full')
                this.renderer.setFreqSpan(HW_MIN, HW_MAX);
            else
                this.renderer.setFreqSpan(WIFI_F0, WIFI_F1);
        } else if (this.s.freqPin) {
            this.renderer.setFreqSpan(HW_MIN, HW_MAX);
        } else {
            this.renderer.setFreqSpan(this.s.manualLo, this.s.manualHi);
        }
        if ($('freq-pop').classList.contains('open') && this.s.fft)
            this._drawFft();
    }

    _maxTargetWidth(freq) {
        return Math.max(freq - HW_MIN, HW_MAX - freq);
    }

    _clampTargetWidth() {
        const mn = 9;
        const mx = Math.max(mn, this._maxTargetWidth(this.s.targetFreq));
        if (this.s.targetWidth > mx) this.s.targetWidth = mx;
        if (this.s.targetWidth < mn) this.s.targetWidth = mn;
        const tw = $('target-width');
        tw.min = mn;
        tw.max = mx;
        tw.value = this.s.targetWidth;
    }

    _buildSchemes() {
        const list = $('scheme-list');
        for (const [name, sc] of Object.entries(SCHEMES)) {
            const el = document.createElement('div');
            el.className = 'scheme';
            el.dataset.name = name;
            const mode = ['FREQ', 'INTENSITY', 'TARGET PROX'][sc.mode];
            el.innerHTML = `<div class="sw"></div><div class="nm">${sc.label}</div>` +
                (name === 'spectrum'
                    ? `<div class="sopts">` +
                      `<button type="button" class="pbtn" id="btn-freq-pin">PIN</button>` +
                      `<button type="button" class="pbtn" id="btn-wifi-chan">BAND</button>` +
                      `</div>`
                    : `<div class="md">${mode}</div>`);
            el.querySelector('.sw').style.background = name === 'target'
                ? schemeCss(this.s.targetLut)
                : schemeCss(name);
            el.onclick = () => {
                this.s.scheme = name;
                this._applySchemeLut();
                if (name !== 'target') $('lut-pick').classList.remove('open');
                this._syncSchemeUi();
                if (name === 'target') this._applyTargetSweep(true);
                this.save();
            };
            if (name === 'spectrum') {
                el.querySelector('#btn-freq-pin').onclick = (e) => {
                    e.stopPropagation();
                    this.s.freqPin = !this.s.freqPin;
                    this._syncFreqColor();
                    this.save();
                };
                el.querySelector('#btn-wifi-chan').onclick = (e) => {
                    e.stopPropagation();
                    const i = WIFI_COLOR.indexOf(this.s.wifiColor);
                    this.s.wifiColor = WIFI_COLOR[(i < 0 ? 0 : i + 1) % WIFI_COLOR.length];
                    this._syncFreqColor();
                    this.save();
                };
            }
            if (name === 'target') {
                const sw = el.querySelector('.sw');
                sw.onclick = (e) => {
                    e.stopPropagation();
                    if (this.s.scheme !== 'target') {
                        this.s.scheme = 'target';
                        this._applySchemeLut();
                        this._applyTargetSweep(true);
                    }
                    $('lut-pick').classList.toggle('open');
                    this._syncSchemeUi();
                    this.save();
                };
            }
            list.appendChild(el);
        }

        const pick = $('lut-pick');
        for (const name of TARGET_LUTS) {
            const el = document.createElement('div');
            el.className = 'lut';
            el.dataset.name = name;
            el.innerHTML = `<div class="sw" style="background:${schemeCss(name)}"></div>` +
                           `<div class="nm">${SCHEMES[name].label}</div>`;
            el.onclick = (e) => {
                e.stopPropagation();
                this.s.targetLut = name;
                this._applySchemeLut();
                this._syncSchemeUi();
                this.save();
            };
            pick.appendChild(el);
        }

        const tf = $('target-freq'), tw = $('target-width');
        const sync = () => {
            this._clampTargetWidth();
            $('target-freq-val').textContent = `${this.s.targetFreq} MHZ`;
            $('target-width-val').textContent = `\u00b1${this.s.targetWidth} MHZ`;
            this.renderer.setTarget(this.s.targetFreq, this.s.targetWidth);
            if (this.s.scheme === 'target') this._applyTargetSweep(false);
            if ($('freq-pop').classList.contains('open') && this.s.fft)
                this._drawFft();
        };
        tf.oninput = () => { this.s.targetFreq = parseFloat(tf.value); sync(); };
        tw.oninput = () => { this.s.targetWidth = parseFloat(tw.value); sync(); };
        tf.onchange = tw.onchange = () => {
            this._clampTargetWidth();
            if (this.s.scheme === 'target') this._applyTargetSweep(true);
            this.save();
        };
        this._syncSchemeUi();
    }

    _applyTargetSweep(force) {
        let lo = this.s.targetFreq - this.s.targetWidth;
        let hi = this.s.targetFreq + this.s.targetWidth;
        lo = Math.max(HW_MIN, lo);
        hi = Math.min(HW_MAX, hi);
        if (hi - lo < 18) {
            const mid = 0.5 * (lo + hi);
            lo = Math.max(HW_MIN, mid - 9);
            hi = Math.min(HW_MAX, mid + 9);
        }
        this.s.manualLo = lo;
        this.s.manualHi = hi;
        if (this._layoutManual) this._layoutManual();
        this._applyManualRange(force);
    }

    _syncSchemeUi() {
        for (const el of document.querySelectorAll('.scheme'))
            el.classList.toggle('on', el.dataset.name === this.s.scheme);
        const tsw = document.querySelector('.scheme[data-name=target] .sw');
        if (tsw) tsw.style.background = schemeCss(this.s.targetLut);
        for (const el of document.querySelectorAll('#lut-pick .lut'))
            el.classList.toggle('on', el.dataset.name === this.s.targetLut);
        if (this.s.scheme !== 'target') $('lut-pick').classList.remove('open');
        $('target-opts').style.display = this.s.scheme === 'target' ? 'block' : 'none';
        this._clampTargetWidth();
        $('target-freq').value = this.s.targetFreq;
        $('target-freq-val').textContent = `${this.s.targetFreq} MHZ`;
        $('target-width-val').textContent = `\u00b1${this.s.targetWidth} MHZ`;
        this._syncFreqColor();
    }

    // ==================================================================
    // CONF panel
    // ==================================================================

    _bindSettings() {
        const bindRange = (id, key, fmt, apply) => {
            const el = $(`s-${id}`);
            el.oninput = () => {
                this.s[key] = parseFloat(el.value);
                $(`v-${id}`).textContent = fmt(this.s[key]);
                apply(this.s[key]);
            };
            el.onchange = () => this.save();
        };
        bindRange('size', 'size', v => v.toFixed(0), v => this.renderer.setPointSize(v));
        bindRange('gain', 'gain', v => v.toFixed(1), v => this.renderer.setPointGain(v));
        bindRange('decay', 'decay', decayLabel, v => this.renderer.setDecayTau(decayTau(v)));
        bindRange('dens', 'density', v => `${v.toFixed(0)}%`, () => {});
        $('s-dens').onchange = () => {
            this.save();
            this.net.set({ output_fraction: this.s.density / 100 });
        };
        bindRange('bal', 'balanceDb', balLabel, () => {});
        bindRange('clos', 'closureDeg', closLabel, () => {});
        bindRange('cfar', 'cfarDb', cfarLabel, () => {});
        for (const id of ['bal', 'clos', 'cfar'])
            $(`s-${id}`).onchange = () => { this.save(); this.net.set(this._gateMsg()); };

        const bindToggle = (id, key, apply) => {
            const el = $(`t-${id}`);
            el.onclick = () => {
                this.s[key] = !this.s[key];
                el.classList.toggle('on', this.s[key]);
                apply(this.s[key]);
                this.save();
            };
        };
        bindToggle('pulse', 'pulse', v => this.renderer.setPulse(v));
        $('t-flip').onclick = () => this._setFlip(!this.s.flip);
        bindToggle('bottom', 'bottom', v => this.renderer.setBottom(v));
        bindToggle('tiles', 'tiles', v => this.renderer.setTiles(v));
        bindToggle('rings', 'rings', v => this.renderer.setRings(v));
        bindToggle('bgn', 'bgNorm', () => this.net.set(this._gateMsg()));

        const accent = $('s-accent');
        accent.oninput = () => {
            const hex = parseHex(accent.value);
            if (!hex) return;
            this.s.accent = hex;
            $('v-accent').textContent = hex.toUpperCase();
            this._applyAccent();
        };
        accent.onchange = () => this.save();

        /* State is only pushed on changes; poll it for the hit readout. */
        setInterval(() => {
            if ($('set-pop').classList.contains('open')) this.net.send({ type: 'get_state' });
        }, 1000);

        $('s-clear').onclick = () => this.renderer.clearPoints();
        $('s-defaults').onclick = () => this._restoreDefaults();
    }

    _restoreDefaults() {
        this.s = { ...DEFAULTS, wifiSel: [], corners: defaultCorners() };
        this.save();
        this._applyAll();
        this._syncSchemeUi();
        this._restoreWifiTiles();
        if (this._selectFreqTab) this._selectFreqTab(this.s.freqTab, false);
        if (this._layoutManual) this._layoutManual();
        this._layoutGain();
        this._placeFft();
        this._syncModeUi();
        this.pushBackend();
    }

    _syncSettingUi() {
        $('s-size').value = this.s.size; $('v-size').textContent = this.s.size.toFixed(0);
        $('s-gain').value = this.s.gain; $('v-gain').textContent = this.s.gain.toFixed(1);
        $('s-decay').value = this.s.decay; $('v-decay').textContent = decayLabel(this.s.decay);
        $('s-dens').value = this.s.density; $('v-dens').textContent = `${this.s.density.toFixed(0)}%`;
        $('s-bal').value = this.s.balanceDb; $('v-bal').textContent = balLabel(this.s.balanceDb);
        $('s-clos').value = this.s.closureDeg; $('v-clos').textContent = closLabel(this.s.closureDeg);
        $('s-cfar').value = this.s.cfarDb; $('v-cfar').textContent = cfarLabel(this.s.cfarDb);
        $('s-accent').value = this.s.accent;
        $('v-accent').textContent = this.s.accent.toUpperCase();
        const tmap = { pulse: 'pulse', flip: 'flip',
                       bottom: 'bottom', tiles: 'tiles', rings: 'rings', bgn: 'bgNorm' };
        for (const [id, key] of Object.entries(tmap))
            $(`t-${id}`).classList.toggle('on', !!this.s[key]);
        $('btn-mirror').classList.toggle('on', !!this.s.mirrors);
        $('btn-flip').classList.toggle('on', !!this.s.flip);
    }

    // ==================================================================
    // Pointer routing on the GL canvas: inside-view look
    // ==================================================================

    _bindPointer() {
        const gl = $('gl');
        let down = null, lastPos = null;

        gl.addEventListener('pointerdown', (e) => {
            down = true;
            lastPos = { x: e.clientX, y: e.clientY };
        });

        gl.addEventListener('pointermove', (e) => {
            if (!down) return;
            if (this.renderer.morph > 0.5 && this.renderer.sphereCam === 'inside' && lastPos) {
                const dy = (e.clientX - lastPos.x) / innerWidth * 2.2;
                const dp = (e.clientY - lastPos.y) / innerHeight * 1.6;
                this.renderer.insideLook(dy, dp);
            }
            lastPos = { x: e.clientX, y: e.clientY };
        });

        gl.addEventListener('pointerup', () => {
            down = null;
        });
    }

    _layoutHudPops() {
        const right = $('hud-right').getBoundingClientRect().left;
        const w = Math.max(280, right - 8 - 8) + 'px';
        /* Mobile browser toolbars eat into 100vh, which used to clip the
         * bottom action row even when scrolled to the end. */
        const vv = window.visualViewport;
        const h = Math.max(160, (vv ? vv.height : innerHeight) - 64) + 'px';
        for (const id of ['freq-pop', 'color-pop', 'set-pop']) {
            $(id).style.width = w;
            $(id).style.maxHeight = h;
        }
        if (this._layoutManual) this._layoutManual();
        if (this.s.fft) requestAnimationFrame(() => this._drawFft());
    }

    _resize() {
        this._layoutHudPops();
        this._drawFft();
        this._drawCal(this._calDrag);
    }

    // ==================================================================
    // Status / fps line
    // ==================================================================

    onState(st) {
        this._gateRates(st);
        this.state = st;
        if (!this._gainDraggingRef()) {
            // don't fight the user's finger; adopt backend gain otherwise
            const g = Math.max(0, Math.min(RF_GAIN_MAX, st.gain | 0));
            if (g !== this.s.hwGain) {
                this.s.hwGain = g;
                this._layoutGain();
            }
        }
    }

    /* Gate counters are cumulative; show per-second hits and the share each
     * stage rejects since the previous state message. */
    _gateRates(st) {
        const g = st.gates, prev = this.state && this.state.gates;
        const now = performance.now();
        const dt = (now - (this._gateT || now)) / 1000;
        this._gateT = now;
        if (!g || !prev || dt <= 0 || g.hits < prev.hits) return;
        const hits = g.hits - prev.hits;
        const pct = k => hits > 0 ? `${Math.round(100 * (g[k] - prev[k]) / hits)}%` : '-';
        $('v-rej').textContent = `${(hits / dt / 1000).toFixed(1)}K/S  ` +
            `SPUR ${pct('rej_spur')}  BAL ${pct('rej_balance')}  CLOS ${pct('rej_closure')}`;
    }

    setStatus(kind) {
        const el = $('status');
        el.classList.remove('warn', 'lost');
        if (kind === 'live') {
            el.textContent = 'STREAM UP';
        } else if (kind === 'connecting') {
            el.textContent = 'CONNECTING';
            el.classList.add('warn');
        } else {
            el.textContent = 'STREAM LOST';
            el.classList.add('lost');
        }
    }

    tick(gpuFps, netFps, pts) {
        const st = this.state;
        let extra = '';
        if (st && typeof st.adc_peak === 'number') {
            extra = `  ADC:${st.adc_peak | 0}/${(st.adc_rms || 0).toFixed(1)}`;
            if (typeof st.lna_db === 'number' && st.lna_db >= 0)
                extra += `  LNA:${st.lna_db}  VGA:${st.vga_db}`;
        }
        $('fps').textContent =
            `GPU:${gpuFps.toFixed(0)}  NET:${netFps.toFixed(1)}  PTS:${pts}${extra}`;
    }
}

function sanitizeWifiSel(v) {
    if (!Array.isArray(v)) return [];
    return v.filter(b => Array.isArray(b) && b.length >= 2)
        .map(b => [Number(b[0]), Number(b[1])])
        .filter(b => Number.isFinite(b[0]) && Number.isFinite(b[1]));
}

function mergeBands(bands) {
    if (!bands.length) return [];
    const s = bands.map(b => [Math.min(...b), Math.max(...b)]).sort((a, b) => a[0] - b[0]);
    const out = [s[0].slice()];
    for (let i = 1; i < s.length; i++) {
        const last = out[out.length - 1];
        if (s[i][0] <= last[1] + 0.01) last[1] = Math.max(last[1], s[i][1]);
        else out.push(s[i].slice());
    }
    return out;
}
