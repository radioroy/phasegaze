// ui.js — FREQ / COLOR / CONF panels, gain slider, fps line.

import { WIFI_TIERS, WIFI_VIEWS, WIFI_F0, WIFI_F1, WIFI_HOP_BANDS, WIFI_GAP } from './wifi.js?v=pg75';
import { NTSC_CHANNELS, NTSC_ROWS, NTSC_BASE_MHZ, NTSC_F0, NTSC_F1,
    NTSC_SCAN, ntscById, ntscChannelSpan, ntscTrapPoints } from './ntsc.js?v=pg79';
import { SCHEMES, schemeCss, lutRgb } from './colors.js';
import { Cam } from './cam.js';
import { applyDrag, DEFAULT_CORNERS, defaultCorners, mapPoint, sanitizeCorners, unmapPoint } from './cal.js';

const TARGET_LUTS = ['spectrum', 'iron', 'whitehot', 'greenhot', 'viridis'];

const HW_MIN = 4480, HW_MAX = 6740;
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
const FFT_EQ_LO0 = 4480;
const FFT_EQ_LO_N = 2261;
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
const LS_KEY = 'phasegaze.settings.v10';
const ACCENT_DEFAULT = '#b8c4b8';
/* Camera mode hides the HUD this long after the last tap. */
const CAM_HIDE_MS = 8000;

const DEFAULTS = {
    size: 15, gain: 4.0, decay: 1, density: 100,
    balanceDb: 10, closureDeg: 0, bgNorm: true, cfarDb: 7,
    pulse: false, flip: false,
    mirrors: true, bottom: false, tiles: true, rings: false,
    scheme: 'spectrum', targetLut: 'iron', targetFreq: 5500, targetWidth: 40,
    freqPin: true, wifiColor: 'band', freqTab: 'wifi',
    fft: true, fftSpeed: 'fast', fftAgc: true,
    hwGain: 45,
    manualLo: HW_MIN, manualHi: HW_MAX,
    wifiSel: [], wifiView: '5', ntscSel: [],
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
        if (this.s.freqTab !== 'wifi' && this.s.freqTab !== 'ntsc') this.s.freqTab = 'wifi';
        if (!WIFI_COLOR.includes(this.s.wifiColor))
            this.s.wifiColor = this.s.wifiChan === true ? 'chan' : 'band';
        delete this.s.wifiChan;
        this.s.wifiSel = sanitizeWifiSel(this.s.wifiSel);
        this.s.ntscSel = sanitizeNtscSel(this.s.ntscSel);
        if (!WIFI_VIEWS.some(v => v.id === this.s.wifiView)) this.s.wifiView = '5';
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
        this._sweepEdit = false;
        this.boardLo = HW_MIN;
        this.boardHi = HW_MAX;
        this.netFps = 0;
        this.gpuFps = 0;
        this.pts = 0;
        this._gainSendTimer = 0;
        this._rangeSendTimer = 0;
        this._freqTab = this.s.freqTab;
        this._wifiView = this.s.wifiView;

        this.camMode = false;
        this.calOn = false;
        this._camHideTimer = null;
        this._revealTap = false;
        this._hold = null;
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
        this._bindHold();
        this._bindVideo();
        this._bindWifi();
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
        /* Saved 4900–6100 was the old hardware limit, not a chosen sub-band. */
        const widen = (prev) => {
            if (!prev) return prev;
            if (prev.manualLo == null || prev.manualLo <= 4900) prev.manualLo = HW_MIN;
            if (prev.manualHi == null || prev.manualHi >= 6100) prev.manualHi = HW_MAX;
            return prev;
        };
        try {
            const cur = localStorage.getItem(LS_KEY);
            if (cur) return JSON.parse(cur) || {};
            const v9 = localStorage.getItem('phasegaze.settings.v9');
            if (v9) return widen(JSON.parse(v9) || {});
            const v8 = localStorage.getItem('phasegaze.settings.v8');
            if (v8) {
                const prev = JSON.parse(v8) || {};
                prev.rings = false;
                return widen(prev);
            }
            const v7 = localStorage.getItem('phasegaze.settings.v7');
            if (v7) {
                const prev = JSON.parse(v7) || {};
                /* v7 slider 0 was the 0.05 s default. 0 is now one frame. */
                if (prev.decay === 0) prev.decay = 1;
                return widen(prev);
            }
            const v6 = localStorage.getItem('phasegaze.settings.v6');
            if (v6) {
                const prev = JSON.parse(v6) || {};
                /* 23 was the v6 default (~0.20 s). 0 was 0.05 s. */
                if (prev.decay === 23 || prev.decay === 0) prev.decay = 1;
                return widen(prev);
            }
            const v3 = localStorage.getItem('phasegaze.settings.v3');
            if (v3) {
                const prev = JSON.parse(v3) || {};
                prev.mirrors = true;
                if (typeof prev.rings !== 'boolean')
                    prev.rings = false;
                return widen(prev);
            }
            const v2 = localStorage.getItem('phasegaze.settings.v2');
            if (v2) {
                const prev = JSON.parse(v2) || {};
                delete prev.density;
                return widen(prev);
            }
            const v1 = localStorage.getItem('phasegaze.settings.v1');
            if (v1) {
                const prev = JSON.parse(v1) || {};
                delete prev.size;
                delete prev.density;
                return widen(prev);
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
        if ($('freq-pop').classList.contains('open') && this.s.fft) {
            if (!this._fftRafPending) {
                this._fftRafPending = true;
                requestAnimationFrame(() => {
                    this._fftRafPending = false;
                    this._drawFft();
                });
            }
        }
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
        if (this._hold) this._pushHold();
        else if (this.s.scheme === 'target') this._applyTargetSweep(true);
        else if (this._freqTab === 'wifi') this._commitWifiSel();
        else if (this._freqTab === 'ntsc') this._commitNtscSel();
        else this._applyManualRange(true);
    }

    // ==================================================================
    // HUD / menu
    // ==================================================================

    _syncWifiTileTuned(mhz) {
        if (!Number.isFinite(mhz)) return;
        for (const el of document.querySelectorAll('#wifi-tiers .tile')) {
            const f0 = parseFloat(el.dataset.f0), f1 = parseFloat(el.dataset.f1);
            const isMatch = (mhz >= f0 && mhz <= f1);
            el.classList.toggle('tuned', isMatch);
        }
    }

    _syncNtscChTuned(fc) {
        if (!Number.isFinite(fc)) return;
        for (const el of document.querySelectorAll('#ntsc-chart .ntsc-ch')) {
            const elFc = parseFloat(el.dataset.fc);
            const isMatch = Math.abs(elFc - fc) < 1.0;
            el.classList.toggle('tuned', isMatch);
        }
    }

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
        const wifiTop = $('btn-wifi-top');
        if (wifiTop) {
            wifiTop.onclick = (e) => {
                e.stopPropagation();
                if (this._wifing()) {
                    this._stopWifi();
                } else {
                    const mhz = (this._hold && Number.isFinite(this._hold.freq)) ? this._hold.freq : 5180;
                    this._wifiMhz(mhz);
                }
            };
        }
        const ntscTop = $('btn-ntsc-top');
        if (ntscTop) {
            ntscTop.onclick = (e) => {
                e.stopPropagation();
                if (this._watching()) {
                    this._stopVideo();
                } else {
                    const mhz = (this._hold && Number.isFinite(this._hold.freq)) ? this._hold.freq : 5645;
                    this._watchMhz(mhz, true);
                }
            };
        }
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

        // Browsers reject requestFullscreen without a user gesture, so the
        // first click, tap, or key is the earliest call that can succeed.
        // One shot: after that, Esc and FULL leave fullscreen for good.
        const enterFs = (e) => {
            if (e.type === 'keydown') {
                if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
                if (e.key === 'Escape' || e.key === 'Shift' || e.key === 'Control' ||
                    e.key === 'Alt' || e.key === 'Meta') return;
            }
            document.removeEventListener('click', enterFs, true);
            document.removeEventListener('keydown', enterFs, true);
            if (!document.fullscreenElement)
                document.documentElement.requestFullscreen().catch(() => {});
        };
        document.addEventListener('click', enterFs, true);
        document.addEventListener('keydown', enterFs, true);
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
        this._setHint(true);
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

    /* Calibration overlay visualization: shows warp grid, control handles,
     * boresight alignment, calibration state badge, and active drag vectors. */
    _drawCal(d) {
        const cv = $('cal-layer');
        if (!cv) return;
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = Math.round(innerWidth * dpr), h = Math.round(innerHeight * dpr);
        if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
        const ctx = cv.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, innerWidth, innerHeight);

        if (!this.calOn) return;

        $('cal-tip').classList.toggle('hide', !!(d && d.moved));

        const W = innerWidth;
        const H = innerHeight;
        const acc = this.s.accent || ACCENT_DEFAULT;
        const corners = this.s.corners;
        if (!corners || corners.length !== 4) return;

        const hexToRgba = (hex, a) => {
            if (!hex || typeof hex !== 'string') return `rgba(184, 196, 184, ${a})`;
            let c = hex.replace('#', '');
            if (c.length === 3) c = c.split('').map(x => x + x).join('');
            const num = parseInt(c, 16);
            if (isNaN(num)) return `rgba(184, 196, 184, ${a})`;
            return `rgba(${(num >> 16) & 255}, ${(num >> 8) & 255}, ${num & 255}, ${a})`;
        };

        const drawRRect = (x, y, rw, rh, r) => {
            if (ctx.roundRect) ctx.roundRect(x, y, rw, rh, r);
            else ctx.rect(x, y, rw, rh);
        };

        ctx.save();

        // 1. Perspective alignment grid (spans RF field of view across the screen)
        const uSteps = [0.30, 0.40, 0.50, 0.60, 0.70];
        const vSteps = [0.30, 0.40, 0.50, 0.60, 0.70];

        ctx.lineWidth = 1;
        ctx.strokeStyle = hexToRgba(acc, 0.16);
        ctx.setLineDash([4, 6]);

        for (const u of uSteps) {
            if (u === 0.50) continue;
            const p0 = mapPoint(corners, u, 0.27);
            const p1 = mapPoint(corners, u, 0.73);
            ctx.beginPath();
            ctx.moveTo(p0.x * W, p0.y * H);
            ctx.lineTo(p1.x * W, p1.y * H);
            ctx.stroke();
        }
        for (const v of vSteps) {
            if (v === 0.50) continue;
            const p0 = mapPoint(corners, 0.27, v);
            const p1 = mapPoint(corners, 0.73, v);
            ctx.beginPath();
            ctx.moveTo(p0.x * W, p0.y * H);
            ctx.lineTo(p1.x * W, p1.y * H);
            ctx.stroke();
        }

        // Major axes at u=0.50 and v=0.50 (RF coordinate axes)
        ctx.setLineDash([]);
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = hexToRgba(acc, 0.35);

        const vAxis0 = mapPoint(corners, 0.50, 0.27);
        const vAxis1 = mapPoint(corners, 0.50, 0.73);
        ctx.beginPath();
        ctx.moveTo(vAxis0.x * W, vAxis0.y * H);
        ctx.lineTo(vAxis1.x * W, vAxis1.y * H);
        ctx.stroke();

        const hAxis0 = mapPoint(corners, 0.27, 0.50);
        const hAxis1 = mapPoint(corners, 0.73, 0.50);
        ctx.beginPath();
        ctx.moveTo(hAxis0.x * W, hAxis0.y * H);
        ctx.lineTo(hAxis1.x * W, hAxis1.y * H);
        ctx.stroke();

        // 2. Control quad bounding box (the 4 quarter control points in screen space)
        const cp0 = { x: corners[0].u * W, y: corners[0].v * H };
        const cp1 = { x: corners[1].u * W, y: corners[1].v * H };
        const cp2 = { x: corners[2].u * W, y: corners[2].v * H };
        const cp3 = { x: corners[3].u * W, y: corners[3].v * H };

        ctx.strokeStyle = hexToRgba(acc, 0.22);
        ctx.setLineDash([3, 5]);
        ctx.beginPath();
        ctx.moveTo(cp0.x, cp0.y);
        ctx.lineTo(cp1.x, cp1.y);
        ctx.lineTo(cp2.x, cp2.y);
        ctx.lineTo(cp3.x, cp3.y);
        ctx.closePath();
        ctx.stroke();
        ctx.setLineDash([]);

        // 3. Corner control brackets (at the 4 quarter control points)
        const ctrlPoints = [
            { px: cp0.x, py: cp0.y, defX: DEFAULT_CORNERS[0].u * W, defY: DEFAULT_CORNERS[0].v * H, label: 'C0', dirX: 1, dirY: 1 },
            { px: cp1.x, py: cp1.y, defX: DEFAULT_CORNERS[1].u * W, defY: DEFAULT_CORNERS[1].v * H, label: 'C1', dirX: -1, dirY: 1 },
            { px: cp2.x, py: cp2.y, defX: DEFAULT_CORNERS[2].u * W, defY: DEFAULT_CORNERS[2].v * H, label: 'C2', dirX: -1, dirY: -1 },
            { px: cp3.x, py: cp3.y, defX: DEFAULT_CORNERS[3].u * W, defY: DEFAULT_CORNERS[3].v * H, label: 'C3', dirX: 1, dirY: -1 },
        ];

        let maxShift = 0;
        let sumShift = 0;
        const bLen = 14;

        ctx.lineWidth = 2;
        for (let i = 0; i < 4; i++) {
            const cp = ctrlPoints[i];
            const px = cp.px, py = cp.py;
            const defX = cp.defX, defY = cp.defY;
            const shift = Math.hypot(px - defX, py - defY);
            sumShift += shift;
            if (shift > maxShift) maxShift = shift;

            // If corner is shifted from default, draw ghost marker at default position and connector
            if (shift > 2) {
                ctx.save();
                ctx.strokeStyle = hexToRgba(acc, 0.25);
                ctx.setLineDash([2, 3]);
                ctx.beginPath();
                ctx.moveTo(defX, defY);
                ctx.lineTo(px, py);
                ctx.stroke();

                ctx.strokeStyle = hexToRgba(acc, 0.35);
                ctx.strokeRect(defX - 3, defY - 3, 6, 6);
                ctx.restore();
            }

            // Draw corner bracket at active position
            ctx.strokeStyle = shift > 2 ? acc : hexToRgba(acc, 0.7);
            ctx.beginPath();
            ctx.moveTo(px + cp.dirX * bLen, py);
            ctx.lineTo(px, py);
            ctx.lineTo(px, py + cp.dirY * bLen);
            ctx.stroke();

            // Corner small anchor dot
            ctx.fillStyle = acc;
            ctx.beginPath();
            ctx.arc(px, py, 2.5, 0, Math.PI * 2);
            ctx.fill();

            // Small label next to bracket
            ctx.font = '9px ui-monospace, "SF Mono", monospace';
            ctx.fillStyle = hexToRgba(acc, 0.6);
            const tx = px + cp.dirX * (bLen + 4);
            const ty = py + (cp.dirY > 0 ? -4 : 12);
            ctx.fillText(cp.label, tx - (cp.dirX < 0 ? 14 : 0), ty);
        }

        // 4. Center boresight reticle
        const center = mapPoint(corners, 0.50, 0.50);
        const cx = center.x * W, cy = center.y * H;
        const scx = W / 2, scy = H / 2;
        const bDist = Math.hypot(cx - scx, cy - scy);

        if (bDist > 2) {
            ctx.save();
            ctx.strokeStyle = 'rgba(255, 180, 50, 0.4)';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(scx - 6, scy); ctx.lineTo(scx + 6, scy);
            ctx.moveTo(scx, scy - 6); ctx.lineTo(scx, scy + 6);
            ctx.stroke();

            ctx.strokeStyle = 'rgba(255, 180, 50, 0.55)';
            ctx.setLineDash([2, 3]);
            ctx.beginPath();
            ctx.moveTo(scx, scy);
            ctx.lineTo(cx, cy);
            ctx.stroke();
            ctx.restore();
        }

        ctx.strokeStyle = acc;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(cx, cy, 14, 0, Math.PI * 2);
        ctx.stroke();

        const rTick1 = 14, rTick2 = 21;
        ctx.beginPath();
        ctx.moveTo(cx, cy - rTick2); ctx.lineTo(cx, cy - rTick1);
        ctx.moveTo(cx, cy + rTick1); ctx.lineTo(cx, cy + rTick2);
        ctx.moveTo(cx - rTick2, cy); ctx.lineTo(cx - rTick1, cy);
        ctx.moveTo(cx + rTick1, cy); ctx.lineTo(cx + rTick2, cy);
        ctx.stroke();

        ctx.fillStyle = acc;
        ctx.beginPath();
        ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
        ctx.fill();

        // 5. Calibration status HUD badge (visible when not dragging)
        if (!d || !d.moved) {
            const isDef = maxShift < 1.5;
            const topW = Math.hypot(cp1.x - cp0.x, cp1.y - cp0.y);
            const botW = Math.hypot(cp2.x - cp3.x, cp2.y - cp3.y);
            const defW = 0.5 * W;
            const scale = defW > 0 ? ((topW + botW) / 2) / defW : 1.0;

            const badgeY = 74;
            const badgeH = 22;
            const bOffX = Math.round(cx - scx), bOffY = Math.round(cy - scy);
            const statusText = isDef
                ? 'CAL: FACTORY DEFAULT (1.00x)'
                : `CAL: WARP ACTIVE • ΔBORESIGHT: ${bOffX >= 0 ? '+' : ''}${bOffX}, ${bOffY >= 0 ? '+' : ''}${bOffY}px • SCALE: ${scale.toFixed(2)}x`;

            ctx.font = '10px ui-monospace, "SF Mono", monospace';
            const tm = ctx.measureText(statusText);
            const badgeW = tm.width + 32;
            const badgeX = (W - badgeW) / 2;

            ctx.fillStyle = 'rgba(10, 14, 18, 0.85)';
            ctx.strokeStyle = isDef ? hexToRgba(acc, 0.35) : 'rgba(255, 180, 50, 0.6)';
            ctx.lineWidth = 1;

            ctx.beginPath();
            drawRRect(badgeX, badgeY, badgeW, badgeH, 11);
            ctx.fill();
            ctx.stroke();

            // Status indicator dot
            ctx.fillStyle = isDef ? '#50e3c2' : '#f5a623';
            ctx.beginPath();
            ctx.arc(badgeX + 12, badgeY + badgeH / 2, 3.5, 0, Math.PI * 2);
            ctx.fill();

            // Status label
            ctx.fillStyle = isDef ? acc : '#ffffff';
            ctx.textBaseline = 'middle';
            ctx.fillText(statusText, badgeX + 22, badgeY + badgeH / 2);
            ctx.textBaseline = 'alphabetic';
        }

        // 6. Active drag feedback (when dragging)
        if (d && d.moved) {
            ctx.strokeStyle = acc;
            ctx.fillStyle = acc;
            ctx.lineWidth = 2;

            ctx.beginPath();
            ctx.moveTo(d.x0, d.y0);
            ctx.lineTo(d.x, d.y);
            ctx.stroke();

            ctx.fillRect(d.x0 - 4, d.y0 - 4, 8, 8);

            const angle = Math.atan2(d.y - d.y0, d.x - d.x0);
            const arrLen = 12;
            ctx.beginPath();
            ctx.moveTo(d.x, d.y);
            ctx.lineTo(d.x - arrLen * Math.cos(angle - Math.PI / 6),
                       d.y - arrLen * Math.sin(angle - Math.PI / 6));
            ctx.lineTo(d.x - arrLen * Math.cos(angle + Math.PI / 6),
                       d.y - arrLen * Math.sin(angle + Math.PI / 6));
            ctx.closePath();
            ctx.fill();

            const dragDx = Math.round(d.x - d.x0);
            const dragDy = Math.round(d.y - d.y0);
            const deltaStr = `ΔX: ${dragDx > 0 ? '+' : ''}${dragDx}px  ΔY: ${dragDy > 0 ? '+' : ''}${dragDy}px`;
            ctx.font = '10px ui-monospace, "SF Mono", monospace';
            const dtm = ctx.measureText(deltaStr);
            const pillW = dtm.width + 16;
            const pillH = 20;
            const pillX = Math.max(10, Math.min(W - pillW - 10, d.x + 12));
            const pillY = Math.max(10, Math.min(H - pillH - 10, d.y - 28));

            ctx.fillStyle = 'rgba(10, 14, 18, 0.9)';
            ctx.strokeStyle = acc;
            ctx.lineWidth = 1;
            ctx.beginPath();
            drawRRect(pillX, pillY, pillW, pillH, 4);
            ctx.fill();
            ctx.stroke();

            ctx.fillStyle = acc;
            ctx.textBaseline = 'middle';
            ctx.fillText(deltaStr, pillX + 8, pillY + pillH / 2);
            ctx.textBaseline = 'alphabetic';
        }

        ctx.restore();
    }

    _openFreq(tab) {
        this._closeColor();
        this._closeSet();
        $('freq-pop').classList.add('open');
        $('btn-freq').classList.add('on');
        let targetTab = tab;
        if (!targetTab) {
            if (this._wifing()) targetTab = 'wifi';
            else if (this._watching()) targetTab = 'ntsc';
            else targetTab = this._freqTab || 'wifi';
        }
        if (this._selectFreqTab) this._selectFreqTab(targetTab, false);
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
            if (this.camMode) this._bumpHint();
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
        const el = $('hint');
        el.textContent = this.camMode
            ? 'TAP AND HOLD TO DWELL'
            : 'TAP AND HOLD TO DWELL, DRAG TO MOVE, SCROLL TO ZOOM';
        if (vis && ($('freq-pop').classList.contains('open') ||
                    $('color-pop').classList.contains('open') ||
                    $('set-pop').classList.contains('open')))
            return;
        el.classList.toggle('show', vis);
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

    _wifiViewDef() {
        return WIFI_VIEWS.find(v => v.id === this._wifiView) || WIFI_VIEWS[0];
    }

    _wifiViewSpan() {
        const v = this._wifiViewDef();
        return [v.f0, v.f1];
    }

    /* Piecewise axis for the visible band. 5 GHz compresses the 5330–5490 hole. */
    _rebuildWifiAxis() {
        const scan = this._wifiViewDef().scan;
        const nGap = Math.max(0, scan.length - 1);
        const gapW = nGap ? WIFI_GAP : 0;
        const usable = 1 - nGap * gapW;
        let occ = 0;
        for (const b of scan) occ += b[1] - b[0];
        const segs = [];
        let x = 0;
        for (let i = 0; i < scan.length; i++) {
            const a = scan[i][0], b = scan[i][1];
            const w = occ > 0 ? usable * (b - a) / occ : 0;
            segs.push({ a, b, x0: x, x1: x + w, gap: false });
            x += w;
            if (i + 1 < scan.length) {
                segs.push({ a: b, b: scan[i + 1][0], x0: x, x1: x + gapW, gap: true });
                x += gapW;
            }
        }
        this._wifiSegs = segs;
    }

    _wifiToX(f) {
        const segs = this._wifiSegs;
        if (!segs || !segs.length) return 0;
        if (f <= segs[0].a) return 0;
        for (const s of segs) {
            if (f <= s.b) {
                const den = s.b - s.a;
                const u = den > 0 ? (f - s.a) / den : 0;
                return s.x0 + u * (s.x1 - s.x0);
            }
        }
        return 1;
    }

    _wifiXToF(x) {
        const segs = this._wifiSegs;
        if (!segs || !segs.length) return WIFI_F0;
        if (x <= 0) return segs[0].a;
        for (const s of segs) {
            if (x <= s.x1) {
                const den = s.x1 - s.x0;
                const u = den > 0 ? (x - s.x0) / den : 0;
                return s.a + u * (s.b - s.a);
            }
        }
        return segs[segs.length - 1].b;
    }

    _wifiInHole(f) {
        const segs = this._wifiSegs;
        if (!segs) return false;
        for (const s of segs) if (s.gap && f >= s.a && f < s.b) return true;
        return false;
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
        else if (this._freqTab === 'ntsc')
            $('freq-ntsc').insertBefore(block, $('ntsc-chart'));
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
        const views = $('wifi-views');
        for (const v of WIFI_VIEWS) {
            const b = document.createElement('button');
            b.type = 'button';
            b.className = 'tab';
            b.dataset.view = v.id;
            b.textContent = v.label;
            b.onclick = (e) => {
                e.stopPropagation();
                if (this._wifiView === v.id) return;
                this._wifiView = v.id;
                this.s.wifiView = v.id;
                this._layoutWifi();
                this._syncFreqColor();
                this.save();
            };
            views.appendChild(b);
        }
        const inner = $('wifi-tiers');
        WIFI_TIERS.forEach((tier, ti) => {
            const row = document.createElement('div');
            row.className = 'tier-row';
            row.style.top = (ti * 32) + 'px';
            for (const t of tier.tiles) {
                const el = document.createElement('div');
                el.className = 'tile';
                el.textContent = t.label;
                el.title = t.title;
                el.dataset.f0 = t.f0;
                el.dataset.f1 = t.f1;
                el.dataset.bw = t.bw;
                el.dataset.view = t.view;
                row.appendChild(el);
            }
            inner.appendChild(row);
        });
        this._layoutWifi();
        this._buildNtsc();

        const selectTab = (tid, apply) => {
            const prev = this._freqTab;
            this._freqTab = tid;
            this.s.freqTab = tid;
            $('ftab-range').classList.toggle('on', tid === 'range');
            $('ftab-wifi').classList.toggle('on', tid === 'wifi');
            $('ftab-ntsc').classList.toggle('on', tid === 'ntsc');
            $('freq-range').style.display = tid === 'range' ? '' : 'none';
            $('freq-wifi').style.display = tid === 'wifi' ? '' : 'none';
            $('freq-ntsc').style.display = tid === 'ntsc' ? '' : 'none';
            if (tid === 'wifi') {
                this._restoreWifiTiles();
                this._commitWifiSel();
            } else if (tid === 'ntsc') {
                this._restoreNtsc();
                this._commitNtscSel();
            } else if (apply !== false && (prev === 'wifi' || prev === 'ntsc')) {
                this._applyManualRange(true);
            } else {
                this._syncFreqColor();
            }
            this._placeFft();
            this._layoutHudPops();
        };
        $('ftab-range').onclick = () => { selectTab('range'); this.save(); };
        $('ftab-wifi').onclick = () => { selectTab('wifi'); this.save(); };
        $('ftab-ntsc').onclick = () => { selectTab('ntsc'); this.save(); };
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

    _layoutWifi() {
        this._rebuildWifiAxis();
        const view = this._wifiView;
        const inner = $('wifi-tiers');
        for (const el of inner.querySelectorAll('.wifi-gap')) el.remove();
        for (const s of this._wifiSegs) {
            if (!s.gap) continue;
            const gap = document.createElement('div');
            gap.className = 'wifi-gap';
            gap.style.left = (s.x0 * 100) + '%';
            gap.style.width = ((s.x1 - s.x0) * 100) + '%';
            inner.appendChild(gap);
        }
        for (const el of inner.querySelectorAll('.tile')) {
            const show = el.dataset.view === view;
            el.style.display = show ? '' : 'none';
            if (!show) continue;
            const x0 = this._wifiToX(+el.dataset.f0);
            const x1 = this._wifiToX(+el.dataset.f1);
            el.style.left = (x0 * 100) + '%';
            el.style.width = ((x1 - x0) * 100) + '%';
        }
        for (const b of document.querySelectorAll('#wifi-views .tab'))
            b.classList.toggle('on', b.dataset.view === view);
    }

    _restoreWifiTiles() {
        const want = new Set((this.s.wifiSel || []).map(b => `${b[0]}-${b[1]}`));
        for (const el of document.querySelectorAll('#wifi-tiers .tile'))
            el.classList.toggle('on', want.has(`${el.dataset.f0}-${el.dataset.f1}`));
    }

    _commitWifiSel() {
        this._noteSweepEdit();
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
            if (this._wifing()) {
                const fc = (+el.dataset.f0 + +el.dataset.f1) / 2;
                this._wifiMhz(fc);
                this._syncWifiTileTuned(fc);
                e.preventDefault();
                return;
            }
            start(e, el, el.dataset.bw === '20');
        });
        const fft = $('fft-wrap');
        if (fft) {
            fft.addEventListener('pointerdown', (e) => {
                if (this._freqTab !== 'wifi') return;
                const t = tile20AtX(e.clientX);
                if (this._wifing() && t) {
                    const fc = (+t.dataset.f0 + +t.dataset.f1) / 2;
                    this._wifiMhz(fc);
                    this._syncWifiTileTuned(fc);
                    e.preventDefault();
                    return;
                }
                start(e, t, true);
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
        this._noteSweepEdit();
        this.net.set({ lo_start: WIFI_F0, lo_end: WIFI_F1, bands: WIFI_HOP_BANDS });
    }

    _buildNtsc() {
        const chart = $('ntsc-chart');
        const pts = ntscTrapPoints();
        const byBand = new Map();
        for (const c of NTSC_CHANNELS) {
            if (!byBand.has(c.band)) byBand.set(c.band, []);
            byBand.get(c.band).push(c);
        }
        for (const band of NTSC_ROWS) {
            const row = document.createElement('div');
            row.className = 'ntsc-row';
            const lab = document.createElement('div');
            lab.className = 'ntsc-lab';
            lab.textContent = band;
            const axis = document.createElement('div');
            axis.className = 'ntsc-axis';
            axis.dataset.band = band;
            const chans = byBand.get(band).slice().sort((a, b) => a.fc - b.fc);
            for (const c of chans) {
                const el = document.createElement('div');
                el.className = 'ntsc-ch';
                el.dataset.id = c.id;
                el.dataset.fc = String(c.fc);
                el.title = `${c.id}  ${c.fc}`;
                el.innerHTML =
                    `<svg viewBox="0 0 ${NTSC_BASE_MHZ} 16" preserveAspectRatio="none">` +
                    `<polygon points="${pts}" vector-effect="non-scaling-stroke"></polygon></svg>` +
                    `<span class="ntsc-num">${c.n}</span>`;
                axis.appendChild(el);
            }
            row.appendChild(lab);
            row.appendChild(axis);
            chart.appendChild(row);
        }
        this._layoutNtsc();
        this._bindNtscPaint(chart);
    }

    _layoutNtsc() {
        const span = NTSC_F1 - NTSC_F0;
        const half = NTSC_BASE_MHZ / 2;
        for (const el of document.querySelectorAll('#ntsc-chart .ntsc-ch')) {
            const fc = +el.dataset.fc;
            const x0 = (fc - half - NTSC_F0) / span;
            const x1 = (fc + half - NTSC_F0) / span;
            el.style.left = (x0 * 100) + '%';
            el.style.width = Math.max(x1 - x0, 0) * 100 + '%';
        }
    }

    /* Nearest carrier in this band whose 30 MHz base contains f.
     * Gaps (Raceband, the hole in band E) return null so a drag there
     * does not select a channel the pointer is not on. */
    _ntscHit(band, f) {
        const half = NTSC_BASE_MHZ / 2;
        let best = null, bd = Infinity;
        for (const c of NTSC_CHANNELS) {
            if (c.band !== band) continue;
            if (f < c.fc - half || f > c.fc + half) continue;
            const d = Math.abs(f - c.fc);
            if (d < bd || (d === bd && best && c.fc < best.fc)) {
                bd = d;
                best = c;
            }
        }
        return best;
    }

    _ntscChAt(clientX, clientY) {
        let axis = null;
        for (const row of document.querySelectorAll('#ntsc-chart .ntsc-row')) {
            const rr = row.getBoundingClientRect();
            if (clientY >= rr.top && clientY < rr.bottom) {
                axis = row.querySelector('.ntsc-axis');
                break;
            }
        }
        if (!axis) return null;
        const r = axis.getBoundingClientRect();
        if (r.width < 1) return null;
        const u = Math.max(0, Math.min(1, (clientX - r.left) / r.width));
        const ch = this._ntscHit(axis.dataset.band, NTSC_F0 + u * (NTSC_F1 - NTSC_F0));
        axis.title = ch ? `${ch.id}  ${ch.fc}` : '';
        return ch;
    }

    _bindNtscPaint(chart) {
        let paint = null;
        const mark = (ch) => {
            if (!ch || paint === null) return;
            const el = chart.querySelector(`.ntsc-ch[data-id="${ch.id}"]`);
            if (el) el.classList.toggle('on', paint);
        };
        chart.addEventListener('pointerdown', (e) => {
            const ch = this._ntscChAt(e.clientX, e.clientY);
            if (!ch) return;
            if (this._watching()) {
                this._watchMhz(ch.fc, true);
                this._syncNtscChTuned(ch.fc);
                e.preventDefault();
                return;
            }
            const el = chart.querySelector(`.ntsc-ch[data-id="${ch.id}"]`);
            paint = !(el && el.classList.contains('on'));
            mark(ch);
            try { chart.setPointerCapture(e.pointerId); } catch (_) {}
            e.preventDefault();
        });
        chart.addEventListener('pointermove', (e) => {
            if (paint === null) return;
            mark(this._ntscChAt(e.clientX, e.clientY));
        });
        const end = () => {
            if (paint === null) return;
            paint = null;
            this._commitNtscSel();
        };
        chart.addEventListener('pointerup', end);
        chart.addEventListener('pointercancel', end);
    }

    _restoreNtsc() {
        const want = new Set(this.s.ntscSel || []);
        for (const el of document.querySelectorAll('#ntsc-chart .ntsc-ch'))
            el.classList.toggle('on', want.has(el.dataset.id));
        this._syncNtscWatch();
    }

    /* A click or a typed MHz near the FPV list is a bin on a channel,
     * not the carrier. 12 MHz covers the FM skirt and stays short of
     * the next 20 MHz-spaced channel. Farther away, the number stands. */
    _snapMhz(mhz, snap) {
        if (snap) {
            let best = null, bd = 12;
            for (const c of NTSC_CHANNELS) {
                const d = Math.abs(c.fc - mhz);
                if (d <= bd) { bd = d; best = c; }
            }
            if (best) return { mhz: best.fc, id: best.id };
        }
        const r = Math.round(mhz * 10) / 10;
        let id = null, bd = 0.6;
        for (const c of NTSC_CHANNELS) {
            const d = Math.abs(c.fc - r);
            if (d <= bd) { bd = d; id = c.id; }
        }
        return { mhz: r, id };
    }

    _watchMhz(mhz, snap) {
        if (!Number.isFinite(mhz)) return;
        const s = this._snapMhz(mhz, snap);
        if (!(s.mhz >= 1000 && s.mhz <= 8000)) return;
        this._vidClosed = false;
        this.net.send({ type: 'video', freq_mhz: s.mhz, snap: snap ? 1 : 0 });
        if (this._hold) {
            const f = this._holdMhz(s.mhz);
            if (f !== this._hold.freq) {
                this._hold.freq = f;
                this._syncHoldUi();
            }
        }
    }

    _syncNtscWatch() {
        const sel = [...document.querySelectorAll('#ntsc-chart .ntsc-ch.on')];
        const input = $('ntsc-mhz');
        if (!input) return;
        if (sel.length === 1 && document.activeElement !== input)
            input.value = sel[0].dataset.fc;
        const mhz = parseFloat(input.value);
        const btn = $('btn-watch');
        if (btn) btn.disabled = !(mhz >= 1000 && mhz <= 8000);
    }

    _commitNtscSel() {
        this._noteSweepEdit();
        const sel = [...document.querySelectorAll('#ntsc-chart .ntsc-ch.on')]
            .map(el => el.dataset.id);
        this.s.ntscSel = sel;
        this.save();
        if (!sel.length) {
            this.net.set({ lo_start: NTSC_F0, lo_end: NTSC_F1, bands: NTSC_SCAN });
        } else {
            const spans = sel.map(id => ntscChannelSpan(ntscById(id)));
            this.net.set({ lo_start: NTSC_F0, lo_end: NTSC_F1, bands: mergeBands(spans) });
        }
        this._syncNtscWatch();
        this._syncFreqColor();
    }

    /* Selected carriers, nearest one owns each MHz of its 30 MHz base.
     * The slices do not overlap, so the sphere map and the FFT agree. */
    _ntscChanRanges() {
        const sel = (this.s.ntscSel || []).map(ntscById).filter(Boolean)
            .sort((a, b) => a.fc - b.fc);
        const n = sel.length;
        if (!n) return null;
        const ts = chanHueTs(sel.map(c => c.fc));
        const half = NTSC_BASE_MHZ / 2;
        let lo = Infinity, hi = -Infinity;
        for (const c of sel) {
            lo = Math.min(lo, Math.floor(c.fc - half));
            hi = Math.max(hi, Math.ceil(c.fc + half));
        }
        const own = new Array(hi - lo).fill(-1);
        for (let f = lo; f < hi; f++) {
            const mid = f + 0.5;
            let bi = -1, bd = Infinity;
            for (let i = 0; i < n; i++) {
                const fc = sel[i].fc;
                if (mid < fc - half || mid >= fc + half) continue;
                const d = Math.abs(mid - fc);
                if (d < bd) { bd = d; bi = i; }
            }
            own[f - lo] = bi;
        }
        const out = [];
        let i = 0;
        while (i < own.length) {
            if (own[i] < 0) { i++; continue; }
            const t = ts[own[i]];
            let j = i + 1;
            while (j < own.length && own[j] === own[i]) j++;
            out.push({ f0: lo + i, f1: lo + j, t });
            i = j;
        }
        return out.length ? out : null;
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
        /* Raw thumb MHz changes inside a slice do not retune. Keying the
         * plan-snap off them paused hop EQ and jumped the Y scale. */
        const ranged = this._freqTab === 'range';
        const [g0, g1] = ranged ? this._rangeSlices() : [0, 0];
        const sel = this._freqTab === 'wifi' ? (this.s.wifiSel || [])
            : this._freqTab === 'ntsc' ? (this.s.ntscSel || []) : [];
        const uiSig = `${this._freqTab}|${g0}|${g1}|${sel.join(',')}`;
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
        if ($('freq-pop').classList.contains('open') && this.s.fft) {
            if (!this._fftRafPending) {
                this._fftRafPending = true;
                requestAnimationFrame(() => {
                    this._fftRafPending = false;
                    this._drawFft();
                });
            }
        }
    }

    _fftView() {
        const specF0 = this._specF0 ?? HW_MIN;
        const specF1 = this._specF1 ?? HW_MAX;
        const n = this._fftAvg ? this._fftAvg.length : 0;
        const bin = n ? (specF1 - specF0) / n : 1;
        const [view0, view1] = this._freqTab === 'wifi' ? this._wifiViewSpan()
            : this._freqTab === 'ntsc' ? [NTSC_F0, NTSC_F1] : [HW_MIN, HW_MAX];
        const i0 = Math.max(0, Math.floor((view0 - specF0) / bin));
        const i1 = Math.min(n, Math.ceil((view1 - specF0) / bin));
        return { view0, view1, specF0, specF1, bin, i0, i1, n };
    }

    _fftHopLo(f) {
        let start;
        if (this._freqTab === 'wifi') {
            const bands = WIFI_HOP_BANDS;
            start = bands[bands.length - 1][0];
            for (let i = 0; i < bands.length - 1; i++) {
                const mid = 0.5 * (bands[i][1] + bands[i + 1][0]);
                if (f < mid) { start = bands[i][0]; break; }
            }
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
        const out = this._fftScratch;
        out.length = 0;
        for (let i = i0; i < i1; i++) {
            const raw = src[i];
            if (raw <= 1e-8) continue;
            const f = specF0 + i * bin;
            if (wifi && this._wifiInHole(f)) continue;
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
        if (wifi && this._wifiSegs) {
            ctx.fillStyle = 'rgba(255,255,255,0.035)';
            for (const s of this._wifiSegs) {
                if (!s.gap) continue;
                const gx0 = s.x0 * w, gx1 = s.x1 * w;
                ctx.fillRect(gx0, 0, Math.max(gx1 - gx0, 1), h);
            }
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
        const chan = this._chanRanges;
        let lastCol = null;
        for (let i = i0; i < i1; i++) {
            const raw = src[i];
            if (raw <= 0) continue;
            const bf0 = specF0 + i * bin;
            if (wifi && this._wifiInHole(bf0)) continue;
            const v = this._fftEqI(raw, i);
            const vh = Math.min(1, Math.max(0, (v - base) / span));
            if (vh <= 0) continue;
            const x0 = wifi ? this._wifiToX(bf0) * w : ((bf0 - view0) / (view1 - view0)) * w;
            const x1 = wifi ? this._wifiToX(bf0 + bin) * w : ((bf0 + bin - view0) / (view1 - view0)) * w;
            if (!inten) {
                const col = this._fftBinCss(bf0, lut, chan);
                if (!col) continue;
                if (col !== lastCol) {
                    ctx.fillStyle = col;
                    lastCol = col;
                }
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
        } else if (this._freqTab === 'ntsc') {
            if (this.s.wifiColor === 'full')
                t = (f - HW_MIN) / (HW_MAX - HW_MIN);
            else
                t = (f - NTSC_F0) / Math.max(NTSC_F1 - NTSC_F0, 1);
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
        if (wifi) {
            const [a, b] = this._wifiViewSpan();
            if (f < a || f > b) return;
        }
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
            let f = Math.round(HW_MIN + t * (HW_MAX - HW_MIN));
            f = Math.max(this.boardLo, Math.min(this.boardHi, f));
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
            let f = Math.round(HW_MIN + t * (HW_MAX - HW_MIN));
            f = Math.max(this.boardLo, Math.min(this.boardHi, f));
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
            if (this._freqTab === 'ntsc') {
                for (const el of document.querySelectorAll('#ntsc-chart .ntsc-ch.on'))
                    el.classList.remove('on');
                this._commitNtscSel();
                return;
            }
            this.s.manualLo = this.boardLo;
            this.s.manualHi = this.boardHi;
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
            if (c - half < this.boardLo) c = this.boardLo + half;
            if (c + half > this.boardHi) c = this.boardHi - half;
            this.s.manualLo = Math.round(c - half);
            this.s.manualHi = Math.round(c + half);
            return;
        }
        const c = this.s.scheme === 'target'
            ? this.s.targetFreq
            : 0.5 * (this.s.manualLo + this.s.manualHi);
        let half = key === 'lo' ? (c - f) : (f - c);
        half = Math.max(minHalf, half);
        const maxHalf = Math.min(c - this.boardLo, this.boardHi - c);
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

    /* Hardware slices anchored at HW_MIN. Matches snap_hw_slices(). */
    _rangeSlices() {
        const step = FFT_HOP_MHZ;
        const loB = this.boardLo, hiB = this.boardHi;
        let a0 = HW_MIN + step * Math.floor((this.s.manualLo - HW_MIN) / step + 1e-6);
        let b1 = HW_MIN + step * Math.ceil((this.s.manualHi - HW_MIN) / step - 1e-6);
        if (b1 <= a0) b1 = a0 + step;
        if (a0 < loB) a0 = HW_MIN + step * Math.ceil((loB - HW_MIN) / step - 1e-9);
        if (b1 > hiB) b1 = HW_MIN + step * Math.floor((hiB - HW_MIN) / step + 1e-9);
        if (a0 < HW_MIN) a0 = HW_MIN;
        if (b1 > HW_MAX) b1 = HW_MAX;
        if (b1 <= a0) b1 = hiB;
        return [a0, b1];
    }

    _applyManualRange(force) {
        const now = performance.now();
        if (!force && now - this._rangeSendTimer < 80) return;
        this._rangeSendTimer = now;
        this._noteSweepEdit();
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

    /* RANGE PIN: HSV locked to the hardware span. Off: stretch to the thumbs.
     * WIFI / NTSC BAND stretches the catalog. CHAN paints selected carriers.
     * FULL is the hardware span. */
    _wifiChanRanges() {
        const tiles = [...document.querySelectorAll('#wifi-tiers .tile.on')]
            .map(el => {
                const f0 = parseFloat(el.dataset.f0), f1 = parseFloat(el.dataset.f1);
                return { f0, f1, fc: 0.5 * (f0 + f1) };
            })
            .sort((a, b) => a.fc - b.fc);
        if (!tiles.length) return null;
        const ts = chanHueTs(tiles.map(t => t.fc));
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
        if (this.s.scheme === 'spectrum' && this.s.wifiColor === 'chan'
                && (this._freqTab === 'wifi' || this._freqTab === 'ntsc')) {
            const ranges = this._freqTab === 'wifi' ? this._wifiChanRanges() : this._ntscChanRanges();
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
        } else if (this._freqTab === 'ntsc') {
            if (this.s.wifiColor === 'full')
                this.renderer.setFreqSpan(HW_MIN, HW_MAX);
            else
                this.renderer.setFreqSpan(NTSC_F0, NTSC_F1);
        } else if (this.s.freqPin) {
            this.renderer.setFreqSpan(HW_MIN, HW_MAX);
        } else {
            this.renderer.setFreqSpan(this.s.manualLo, this.s.manualHi);
        }
        if ($('freq-pop').classList.contains('open') && this.s.fft) {
            if (!this._fftRafPending) {
                this._fftRafPending = true;
                requestAnimationFrame(() => {
                    this._fftRafPending = false;
                    this._drawFft();
                });
            }
        }
    }

    _maxTargetWidth(freq) {
        return Math.max(freq - this.boardLo, this.boardHi - freq);
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
        lo = Math.max(this.boardLo, lo);
        hi = Math.min(this.boardHi, hi);
        if (hi - lo < 18) {
            const mid = 0.5 * (lo + hi);
            lo = Math.max(this.boardLo, mid - 9);
            hi = Math.min(this.boardHi, mid + 9);
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
        $('s-lock').onclick = () => {
            if (this.state && this.state.lock_busy) return;
            this.net.send({ type: 'lock_cal' });
            const lab = $('v-lock');
            if (lab) lab.textContent = 'MEASURING';
            $('s-lock').disabled = true;
            $('s-sweep-lo').disabled = true;
            $('s-sweep-hi').disabled = true;
        };
        this._bindSweepFields();
    }

    /* The two boxes are the sweep ceiling, not the slider thumbs. A typed
     * value is snapped on the board; don't overwrite the box mid-keystroke. */
    _bindSweepFields() {
        const lo = $('s-sweep-lo'), hi = $('s-sweep-hi');
        const commit = () => {
            this._sweepEdit = false;
            const a = parseInt(lo.value, 10);
            const b = parseInt(hi.value, 10);
            if (!Number.isFinite(a) || !Number.isFinite(b)) return;
            this.net.set({ hw_min: a, hw_max: b });
        };
        for (const el of [lo, hi]) {
            el.addEventListener('input', () => { this._sweepEdit = true; });
            el.addEventListener('change', commit);
            el.addEventListener('keydown', (e) => {
                if (e.key === 'Enter') el.blur();
            });
        }
    }

    _restoreDefaults() {
        this.s = { ...DEFAULTS, wifiSel: [], ntscSel: [], corners: defaultCorners() };
        this.s.manualLo = this.boardLo;
        this.s.manualHi = this.boardHi;
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

    /* A freq-panel edit replaces the dwell. The saved sweep is dropped
     * because the edit is the new plan. */
    _noteSweepEdit() {
        if (!this._hold) return;
        this._hold = null;
        this._syncHoldUi();
    }

    /* One hop, LO on the picked bin rounded to 1 MHz. The 20 MHz keep-band
     * is centered on that LO. Watch snapping is a separate path. */
    _holdLoMin() { return Math.ceil(this.boardLo + 0.5 * FFT_HOP_MHZ); }
    _holdLoMax() { return Math.floor(this.boardHi - 0.5 * FFT_HOP_MHZ); }

    _holdOk(freq) {
        return Number.isFinite(freq) &&
            freq >= this.boardLo - 1 && freq <= this.boardHi + 1;
    }

    _holdMhz(freq) {
        let f = Math.round(freq);
        const lo = this._holdLoMin(), hi = this._holdLoMax();
        if (f < lo) f = lo;
        if (f > hi) f = hi;
        return f;
    }

    _pushHold() {
        const f = this._hold.freq;
        const lo = f - 0.5 * FFT_HOP_MHZ, hi = f + 0.5 * FFT_HOP_MHZ;
        this.net.set({ lo_start: lo, lo_end: hi, bands: [[lo, hi]] });
    }

    _positionHoldRow() {
        const row = $('hold-row');
        if (!row) return;
        const ntsc = $('btn-ntsc-top') || $('hud-left');
        const cam = $('btn-cam') || $('hud-right');
        if (!ntsc || !cam) return;
        const rectN = ntsc.getBoundingClientRect();
        const rectC = cam.getBoundingClientRect();
        if (rectN.width > 0 && rectC.width > 0) {
            const mid = (rectN.right + rectC.left) / 2;
            row.style.left = `${mid}px`;
            row.style.transform = 'translateX(-50%)';
        }
    }

    _syncHoldUi() {
        const row = $('hold-row');
        const input = $('hold-mhz');
        if (!this._hold) {
            row.classList.remove('show');
            if (document.activeElement !== input) input.value = '';
            $('hold-dn').disabled = true;
            $('hold-up').disabled = true;
            return;
        }
        const f = this._hold.freq;
        if (document.activeElement !== input) input.value = String(f);
        $('hold-dn').disabled = f <= this._holdLoMin();
        $('hold-up').disabled = f >= this._holdLoMax();
        this._positionHoldRow();
        row.classList.add('show');
    }

    _watching() {
        return !!(this.state && this.state.mode === 'video');
    }

    _tuneHold(mhz) {
        if (!this._hold || !Number.isFinite(mhz)) return;
        const f = this._holdMhz(mhz);
        if (f === this._hold.freq) {
            this._syncHoldUi();
            return;
        }
        this._hold.freq = f;
        if (this._watching()) this._watchMhz(f, false);
        else if (this._wifing()) this._wifiMhz(f);
        else this._pushHold();
        this._syncHoldUi();
    }

    _beginHold(freq) {
        const f = this._holdMhz(freq);
        if (!this._hold) {
            this._hold = {
                freq: f,
                saved: {
                    manualLo: this.s.manualLo,
                    manualHi: this.s.manualHi,
                    freqTab: this._freqTab,
                    wifiSel: (this.s.wifiSel || []).map(b => [b[0], b[1]]),
                    ntscSel: (this.s.ntscSel || []).slice(),
                },
            };
        } else {
            this._hold.freq = f;
        }
        this._pushHold();
        this._syncHoldUi();
        clearTimeout(this._hintTimer);
        this._setHint(false);
    }

    _endHold(restore) {
        const h = this._hold;
        if (!h) return;
        this._hold = null;
        this._syncHoldUi();
        this._bumpHint();
        if (this._watching() || this._vidMhz != null)
            this._stopVideo();
        if (this._wifing() || this._wifiMhzTuned != null)
            this._stopWifi();
        if (!restore) return;
        const s = h.saved;
        this.s.manualLo = s.manualLo;
        this.s.manualHi = s.manualHi;
        this.s.wifiSel = s.wifiSel;
        this.s.ntscSel = s.ntscSel;
        this._restoreWifiTiles();
        this._restoreNtsc();
        if (this._layoutManual) this._layoutManual();
        if (this._selectFreqTab) this._selectFreqTab(s.freqTab, false);
        if (this.s.scheme === 'target') this._applyTargetSweep(true);
        else if (this._freqTab === 'range') this._applyManualRange(true);
    }

    _bindHold() {
        const gl = $('gl');
        /* A held mouse drifts. A tight slop cancelled the press before the
         * timer, so the ring never got to finish. */
        const slop = 36;
        let arm = null;
        const drop = () => {
            if (!arm) return;
            if (arm.timer) clearTimeout(arm.timer);
            arm = null;
            this._chargeStop();
        };
        gl.addEventListener('pointerdown', (e) => {
            if (e.button !== 0) return;
            if (this.calOn) return;
            drop();
            /* A click during a dwell puts the sweep back. It does not
             * start another one. */
            if (this._hold) {
                arm = { x: e.clientX, y: e.clientY, release: true };
                return;
            }
            const hit = this.renderer.pickFreq(e.clientX, e.clientY);
            if (!hit || !this._holdOk(hit.freq)) return;
            const rgb = this.renderer.pointColor(hit.freq, hit.inten);
            const freq = hit.freq;
            const ms = 500;
            const commit = () => {
                if (!arm || arm.release) return;
                const f = arm.freq;
                drop();
                if (this.state && this.state.mode === 'video') this._watchMhz(f, true);
                else if (this.state && this.state.mode === 'wifi') this._wifiMhz(f);
                else this._beginHold(f);
            };
            arm = { x: e.clientX, y: e.clientY, freq, timer: 0 };
            this._chargeStart(e.clientX, e.clientY, rgb, ms, commit);
        });
        gl.addEventListener('pointermove', (e) => {
            if (!arm) return;
            if (Math.hypot(e.clientX - arm.x, e.clientY - arm.y) > slop) drop();
        });
        gl.addEventListener('pointerup', (e) => {
            if (!arm) return;
            const moved = Math.hypot(e.clientX - arm.x, e.clientY - arm.y) > slop;
            const release = arm.release && !moved;
            drop();
            if (release) this._endHold(true);
        });
        gl.addEventListener('pointercancel', drop);
        gl.addEventListener('contextmenu', (e) => e.preventDefault());
        const resumeBtn = $('hold-resume');
        if (resumeBtn) resumeBtn.onclick = () => this._endHold(true);
        $('hold-dn').onclick = (e) => {
            e.stopPropagation();
            if (!this._hold) return;
            this._tuneHold(this._hold.freq - 1);
        };
        $('hold-up').onclick = (e) => {
            e.stopPropagation();
            if (!this._hold) return;
            this._tuneHold(this._hold.freq + 1);
        };
        const input = $('hold-mhz');
        const commitMhz = () => {
            if (!this._hold) return;
            const mhz = parseFloat(input.value);
            if (Number.isFinite(mhz)) this._tuneHold(mhz);
            else this._syncHoldUi();
        };
        input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                e.preventDefault();
                input.blur();
                commitMhz();
            } else if (e.key === 'Escape') {
                input.blur();
                this._syncHoldUi();
            }
        });
        input.addEventListener('change', commitMhz);
        input.addEventListener('blur', commitMhz);
        const holdWatch = $('hold-watch');
        if (holdWatch) {
            holdWatch.onclick = (e) => {
                e.stopPropagation();
                if (!this._hold || !Number.isFinite(this._hold.freq)) return;
                this._watchMhz(this._hold.freq, true);
            };
        }
        const holdWifi = $('hold-wifi');
        if (holdWifi) {
            holdWifi.onclick = (e) => {
                e.stopPropagation();
                if (!this._hold || !Number.isFinite(this._hold.freq)) return;
                this._wifiMhz(this._hold.freq);
            };
        }
    }

    _bindVideo() {
        const win = $('vid-win');
        const bar = $('vid-bar');
        let drag = null;
        bar.addEventListener('pointerdown', (e) => {
            if (e.target.closest('button') || e.target.closest('#vid-title')) return;
            const r = win.getBoundingClientRect();
            drag = { dx: e.clientX - r.left, dy: e.clientY - r.top };
            try { bar.setPointerCapture(e.pointerId); } catch (_) {}
        });
        bar.addEventListener('pointermove', (e) => {
            if (!drag) return;
            const x = Math.max(0, Math.min(e.clientX - drag.dx, innerWidth - 80));
            const y = Math.max(0, Math.min(e.clientY - drag.dy, innerHeight - 48));
            win.style.left = x + 'px';
            win.style.top = y + 'px';
            win.style.bottom = 'auto';
        });
        const end = () => { drag = null; };
        bar.addEventListener('pointerup', end);
        bar.addEventListener('pointercancel', end);
        $('vid-close').onclick = (e) => {
            e.stopPropagation();
            this._stopVideo();
            if (this._hold) this._pushHold();
        };
        const vidTitle = $('vid-title');
        if (vidTitle) {
            vidTitle.onclick = (e) => {
                e.stopPropagation();
                this._openFreq('ntsc');
            };
        }
        const input = $('ntsc-mhz');
        input.addEventListener('input', () => this._syncNtscWatch());
        input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                const mhz = parseFloat(input.value);
                this._watchMhz(mhz, true);
            }
        });
        $('btn-watch').onclick = () => {
            const mhz = parseFloat($('ntsc-mhz').value);
            this._watchMhz(mhz, true);
        };
        this._vidClosed = false;
        this._vidGot = false;
        this._vidMhz = null;
        this._vidErrHold = '';
        this._vidFps = 0;
        this._vidBusy = false;
        this._vidPending = null;
        this._vidUrl = null;
        this._vidPrevUrl = null;
        const img = $('vid-img');
        if (img) img.style.display = 'none';
    }

    _stopVideo() {
        this._vidClosed = true;
        this._vidGot = false;
        this._vidErrHold = '';
        const win = $('vid-win');
        if (win) win.classList.remove('show');
        const ntscTop = $('btn-ntsc-top');
        if (ntscTop) ntscTop.classList.remove('on');
        const ntscHint = $('ntsc-tune-hint');
        if (ntscHint) ntscHint.style.display = 'none';
        for (const el of document.querySelectorAll('#ntsc-chart .ntsc-ch.tuned')) el.classList.remove('tuned');
        this._dropVidUrl();
        this.renderer.clearAim();
        this.net.send({ type: 'video_stop' });
    }

    _dropVidUrl() {
        if (this._vidPrevUrl) {
            URL.revokeObjectURL(this._vidPrevUrl);
            this._vidPrevUrl = null;
        }
        if (this._vidUrl) {
            URL.revokeObjectURL(this._vidUrl);
            this._vidUrl = null;
        }
        this._vidBusy = false;
        this._vidPending = null;
        const img = $('vid-img');
        if (img) {
            img.onload = null;
            img.onerror = null;
            img.removeAttribute('src');
            img.style.display = 'none';
        }
    }

    _vidMsg(text) {
        const el = $('vid-msg');
        el.textContent = text || '';
        el.style.display = text ? '' : 'none';
    }

    _vidTitle(mhz) {
        const s = this._snapMhz(mhz, false);
        const n = Number.isInteger(s.mhz) ? String(s.mhz) : s.mhz.toFixed(1);
        $('vid-title').textContent = s.id ? `${s.id}  ${n}` : `${n} MHz`;
    }

    _syncVideo(st) {
        const win = $('vid-win');
        const ntscTop = $('btn-ntsc-top');
        if (ntscTop) ntscTop.classList.toggle('on', st.mode === 'video');
        const ntscHint = $('ntsc-tune-hint');
        if (ntscHint) ntscHint.style.display = (st.mode === 'video') ? 'block' : 'none';

        if (st.mode === 'video') {
            if (Number.isFinite(st.video_mhz)) {
                this._syncNtscChTuned(st.video_mhz);
            }
            if (this._vidClosed) return;
            if (st.video_mhz !== this._vidMhz) {
                const isHop = this._vidMhz !== null && Math.abs(st.video_mhz - this._vidMhz) >= 5;
                this._vidMhz = st.video_mhz;
                /* A recenter keeps the picture up until the next frame.
                 * A channel hop drops the frame and shows TUNING. */
                if (isHop || !this._vidGot) {
                    this._vidGot = false;
                    this._dropVidUrl();
                    $('vid-fps').textContent = '';
                }
            }
            this._vidErrHold = '';
            win.classList.add('show');
            this._vidTitle(st.video_mhz);
            if (this._hold && Number.isFinite(st.video_mhz)) {
                const f = this._holdMhz(st.video_mhz);
                if (f !== this._hold.freq) {
                    this._hold.freq = f;
                    this._syncHoldUi();
                }
            }
            if (!this._vidGot)
                this._vidMsg((st.video_err || 'TUNING').toUpperCase());
            return;
        }
        if (st.video_err && !this._vidClosed) {
            this._vidErrHold = st.video_err;
            this._vidGot = false;
            this._dropVidUrl();
            win.classList.add('show');
            this._vidMsg(st.video_err.toUpperCase());
            $('vid-fps').textContent = '';
            this.renderer.clearAim();
            return;
        }
        if (!this._vidErrHold) {
            win.classList.remove('show');
            this._vidGot = false;
            this._vidMhz = null;
            this._dropVidUrl();
        }
        for (const el of document.querySelectorAll('#ntsc-chart .ntsc-ch.tuned')) {
            el.classList.remove('tuned');
        }
        this.renderer.clearAim();
    }

    onVideoFrame(header, u8) {
        if (this._vidClosed) return;
        if (!(this.state && this.state.mode === 'video')) return;
        this._vidFps = header.fps || 0;
        this._vidGot = true;
        const au = header.loEnd, av = header.aimV;
        if (Number.isFinite(au) && Number.isFinite(av) && au * au + av * av <= 1)
            this.renderer.setAim(au, av, header.loStart || 0);
        else
            this.renderer.clearAim();
        $('vid-fps').textContent = this._vidFps ? `${this._vidFps.toFixed(0)} FPS` : '';
        this._vidMsg('');
        const copy = u8.slice();
        if (this._vidBusy) {
            this._vidPending = copy;
            return;
        }
        this._showJpeg(copy);
    }

    _showJpeg(u8) {
        if (this._vidClosed) return;
        this._vidBusy = true;
        const url = URL.createObjectURL(new Blob([u8], { type: 'image/jpeg' }));
        const img = $('vid-img');
        const prev = this._vidUrl;
        this._vidPrevUrl = prev;
        this._vidUrl = url;
        const done = () => {
            if (img.onload === done) img.onload = null;
            if (img.onerror === done) img.onerror = null;
            if (prev) {
                URL.revokeObjectURL(prev);
                if (this._vidPrevUrl === prev) this._vidPrevUrl = null;
            }
            if (img.style.display !== 'block') img.style.display = 'block';
            this._vidBusy = false;
            const next = this._vidPending;
            this._vidPending = null;
            if (next && !this._vidClosed) this._showJpeg(next);
        };
        img.onload = done;
        img.onerror = done;
        img.src = url;
    }


    _bindWifi() {
        const win = $('wifi-win');
        const bar = $('wifi-bar');
        let drag = null;
        if (bar && win) {
            bar.addEventListener('pointerdown', (e) => {
                if (e.target.closest('button') || e.target.closest('#wifi-ch-badge')) return;
                const r = win.getBoundingClientRect();
                drag = { dx: e.clientX - r.left, dy: e.clientY - r.top };
                try { bar.setPointerCapture(e.pointerId); } catch (_) {}
            });
            bar.addEventListener('pointermove', (e) => {
                if (!drag) return;
                const x = Math.max(0, Math.min(e.clientX - drag.dx, innerWidth - 80));
                const y = Math.max(0, Math.min(e.clientY - drag.dy, innerHeight - 48));
                win.style.left = x + 'px';
                win.style.top = y + 'px';
                win.style.bottom = 'auto';
            });
            const end = () => { drag = null; };
            bar.addEventListener('pointerup', end);
            bar.addEventListener('pointercancel', end);
        }

        const closeBtn = $('wifi-close');
        if (closeBtn) {
            closeBtn.onclick = (e) => {
                e.stopPropagation();
                this._stopWifi();
                if (this._hold) this._pushHold();
            };
        }

        const clearBtn = $('wifi-clear');
        if (clearBtn) {
            clearBtn.onclick = (e) => {
                e.stopPropagation();
                this._clearWifi();
            };
        }
        const chBadge = $('wifi-ch-badge');
        if (chBadge) {
            chBadge.onclick = (e) => {
                e.stopPropagation();
                this._openFreq('wifi');
            };
        }





        const tbody = $('wifi-packet-tbody');
        if (tbody) {
            tbody.addEventListener('click', (e) => {
                const tr = e.target.closest('tr');
                if (!tr) return;
                const idx = parseInt(tr.dataset.idx, 10);
                if (!Number.isNaN(idx)) {
                    this._selectWifiPacket(idx);
                }
            });
        }

        this._wifiClosed = false;
        this._wifiMhzTuned = null;
        this._wifiPkts = [];
        this._wifiPktCount = 0;
        this._wifiLastSelectedIdx = -1;
    }

    _wifing() {
        return !!(this.state && this.state.mode === 'wifi');
    }

    _snapWifi(mhz) {
        const lockLo = (this.state && this.state.lock_lo > 0) ? this.state.lock_lo : 4480;
        const lockHi = (this.state && this.state.lock_hi > 0) ? this.state.lock_hi : 6740;

        if (!Number.isFinite(mhz)) return { mhz: 5180, ch: 36, band: '5 GHz' };
        if (mhz >= 5945 && mhz <= 6745) {
            let ch = Math.round((mhz - 5955) / 20) * 4 + 1;
            ch = Math.max(1, Math.min(153, ch));
            const snapMhz = 5955 + (ch - 1) * 5;
            if (snapMhz >= lockLo && snapMhz <= lockHi) {
                return { mhz: snapMhz, ch, band: '6 GHz' };
            }
        }
        const allChs = [
            { ch: 36, mhz: 5180 }, { ch: 40, mhz: 5200 }, { ch: 44, mhz: 5220 }, { ch: 48, mhz: 5240 },
            { ch: 52, mhz: 5260 }, { ch: 56, mhz: 5280 }, { ch: 60, mhz: 5300 }, { ch: 64, mhz: 5320 },
            { ch: 100, mhz: 5500 }, { ch: 104, mhz: 5520 }, { ch: 108, mhz: 5540 }, { ch: 112, mhz: 5560 },
            { ch: 116, mhz: 5580 }, { ch: 120, mhz: 5600 }, { ch: 124, mhz: 5620 }, { ch: 128, mhz: 5640 },
            { ch: 132, mhz: 5660 }, { ch: 136, mhz: 5680 }, { ch: 140, mhz: 5700 }, { ch: 144, mhz: 5720 },
            { ch: 149, mhz: 5745 }, { ch: 153, mhz: 5765 }, { ch: 157, mhz: 5785 }, { ch: 161, mhz: 5805 },
            { ch: 165, mhz: 5825 }, { ch: 169, mhz: 5845 }, { ch: 173, mhz: 5865 }, { ch: 177, mhz: 5885 }
        ];
        const CHS = allChs.filter(c => c.mhz >= lockLo && c.mhz <= lockHi);
        const candidates = CHS.length > 0 ? CHS : allChs;
        let best = candidates[0];
        let minDiff = 1e9;
        for (const c of candidates) {
            const diff = Math.abs(c.mhz - mhz);
            if (diff < minDiff) {
                minDiff = diff;
                best = c;
            }
        }
        return { mhz: best.mhz, ch: best.ch, band: '5 GHz' };
    }

    _wifiMhz(mhz) {
        if (!Number.isFinite(mhz)) return;
        const s = this._snapWifi(mhz);
        this._wifiClosed = false;
        this.net.send({ type: 'wifi', mhz: s.mhz, freq_mhz: s.mhz });
        const win = $('wifi-win');
        if (win) win.classList.add('show');
        this._updateWifiPills(s.mhz);
        const badge = $('wifi-ch-badge');
        if (badge) badge.textContent = `CH ${s.ch} · ${s.mhz} MHz`;
        if (this._hold) {
            const f = this._holdMhz(s.mhz);
            if (f !== this._hold.freq) {
                this._hold.freq = f;
                this._syncHoldUi();
            }
        }
    }

    _stopWifi() {
        this._wifiClosed = true;
        this._wifiMhzTuned = null;
        this._wifiChTuned = null;
        const win = $('wifi-win');
        if (win) win.classList.remove('show');
        const wifiTop = $('btn-wifi-top');
        if (wifiTop) wifiTop.classList.remove('on');
        const wifiHint = $('wifi-tune-hint');
        if (wifiHint) wifiHint.style.display = 'none';
        for (const el of document.querySelectorAll('#wifi-tiers .tile.tuned')) el.classList.remove('tuned');
        this.renderer.clearAim();
        this.net.send({ type: 'wifi_stop' });
    }

    _syncWifi(st) {
        const win = $('wifi-win');
        const wifiTop = $('btn-wifi-top');
        if (wifiTop) wifiTop.classList.toggle('on', st.mode === 'wifi');
        const wifiHint = $('wifi-tune-hint');
        if (wifiHint) wifiHint.style.display = (st.mode === 'wifi') ? 'block' : 'none';

        if (st.mode === 'wifi') {
            if (this._wifiClosed) return;
            if (win) win.classList.add('show');
            if (st.wifi_mhz) {
                this._syncWifiTileTuned(st.wifi_mhz);
                if (st.wifi_mhz !== this._wifiMhzTuned || (st.wifi_ch && st.wifi_ch !== this._wifiChTuned)) {
                    this._wifiMhzTuned = st.wifi_mhz;
                    this._wifiChTuned = st.wifi_ch;
                    const s = this._snapWifi(st.wifi_mhz);
                    const badge = $('wifi-ch-badge');
                    if (badge) badge.textContent = `CH ${s.ch} · ${st.wifi_mhz.toFixed(0)} MHz`;
                }
            }
            if (this._hold && Number.isFinite(st.wifi_mhz)) {
                const f = this._holdMhz(st.wifi_mhz);
                if (f !== this._hold.freq) {
                    this._hold.freq = f;
                    this._syncHoldUi();
                }
            }
        } else {
            if (!this._wifiClosed && win && win.classList.contains('show')) {
                win.classList.remove('show');
            }
            for (const el of document.querySelectorAll('#wifi-tiers .tile.tuned')) {
                el.classList.remove('tuned');
            }
        }
    }

    _updateWifiPills(mhz) {}




    _clearWifi() {
        this._wifiPkts = [];
        this._wifiPktCount = 0;
        this._wifiLastSelectedIdx = -1;
        const countEl = $('wifi-kpi-count');
        if (countEl) countEl.innerHTML = '<b>0</b> PKTS';
        const snrEl = $('wifi-kpi-snr');
        if (snrEl) snrEl.innerHTML = 'SNR: <b>--</b> dB';
        const evmEl = $('wifi-kpi-evm');
        if (evmEl) evmEl.innerHTML = 'EVM: <b>--</b> dB';
        const tbody = $('wifi-packet-tbody');
        if (tbody) tbody.innerHTML = '';
        const detail = $('wifi-detail-content');
        if (detail) detail.textContent = 'Select a packet from the table to inspect 802.11 MAC headers, Frame Control fields, and Management IEs.';
    }



    onWifiPacket(header, pkt) {
        if (this._wifiClosed) return;
        this._wifiPktCount++;

        const au = header.loEnd, av = header.aimV;
        if (Number.isFinite(au) && Number.isFinite(av) && au * au + av * av <= 1) {
            this.renderer.setAim(au, av, header.loStart || 0);
        } else {
            this.renderer.clearAim();
        }

        const countEl = $('wifi-kpi-count');
        if (countEl) countEl.innerHTML = `<b>${this._wifiPktCount}</b> PKTS`;
        if (Number.isFinite(pkt.snr_db)) {
            const snrEl = $('wifi-kpi-snr');
            if (snrEl) snrEl.innerHTML = `SNR: <b>${pkt.snr_db.toFixed(1)}</b> dB`;
        }
        if (Number.isFinite(pkt.evm_db)) {
            const evmEl = $('wifi-kpi-evm');
            if (evmEl) evmEl.innerHTML = `EVM: <b>${pkt.evm_db.toFixed(1)}</b> dB`;
        }

        this._wifiPkts.push(pkt);
        const idx = this._wifiPkts.length - 1;
        if (this._wifiPkts.length > 500) {
            this._wifiPkts.shift();
        }

        const tbody = $('wifi-packet-tbody');
        if (tbody) {
            const tr = document.createElement('tr');
            tr.dataset.idx = String(idx);

            const now = new Date();
            const timeStr = now.toTimeString().split(' ')[0] + '.' + String(now.getMilliseconds()).padStart(3, '0');

            let cleanSubtype = pkt.subtype || '';
            if (pkt.pkt_type && cleanSubtype.toLowerCase().startsWith(pkt.pkt_type.toLowerCase())) {
                cleanSubtype = cleanSubtype.substring(pkt.pkt_type.length).replace(/^[:\s\-]+/, '');
            }
            const typeStr = (pkt.pkt_type ? `${pkt.pkt_type}` : '') + (cleanSubtype ? `: ${cleanSubtype}` : '');
            const ssidStr = pkt.ssid || (pkt.pkt_type === 'Mgmt' && pkt.subtype === 'Beacon' ? '<hidden>' : '');
            const bssidStr = pkt.bssid || '';
            const srcStr = pkt.sa || (pkt.is_ipv4 ? pkt.ip_src : '');
            const dstStr = pkt.da || (pkt.is_ipv4 ? pkt.ip_dst : '');
            const rateStr = pkt.rate || '';
            const snrStr = Number.isFinite(pkt.snr_db) ? `${pkt.snr_db.toFixed(0)} dB` : '';
            const lenStr = `${pkt.len || 0} B`;

            const typeClass = pkt.subtype === 'Beacon' ? 'type-beacon'
                            : (pkt.pkt_type === 'Data' ? 'type-data'
                            : (pkt.pkt_type === 'Ctrl' ? 'type-ctrl' : 'type-probe'));

            tr.innerHTML = `
                <td>${this._wifiPktCount}</td>
                <td>${timeStr}</td>
                <td><span class="${typeClass}">${typeStr}</span></td>
                <td title="${bssidStr}">${bssidStr}</td>
                <td title="${srcStr}">${srcStr}</td>
                <td title="${dstStr}">${dstStr}</td>
                <td>${rateStr}</td>
                <td>${snrStr}</td>
                <td>${lenStr}</td>
            `;

            tbody.appendChild(tr);

            while (tbody.children.length > 200) {
                tbody.removeChild(tbody.firstChild);
            }

            const container = $('wifi-table-container');
            if (container) {
                const atBottom = container.scrollHeight - container.scrollTop - container.clientHeight < 70;
                if (atBottom || this._wifiPktCount <= 5) {
                    container.scrollTop = container.scrollHeight;
                }
            }
        }

    }

    _selectWifiPacket(idx) {
        const tbody = $('wifi-packet-tbody');
        if (tbody) {
            tbody.querySelectorAll('tr').forEach(tr => {
                tr.classList.toggle('selected', tr.dataset.idx === String(idx));
            });
        }
        const pkt = this._wifiPkts[idx] || this._wifiPkts[this._wifiPkts.length - 1];
        if (!pkt) return;
        this._wifiLastSelectedIdx = idx;

        const detail = $('wifi-detail-content');
        if (!detail) return;

        let html = '';

        // 1. PHY Layer
        html += `<div class="tree-node">&#9656; <b>PHY LAYER</b></div>`;
        html += `<div class="tree-leaf">Modulation / Rate: <span>${pkt.rate || '--'} (${pkt.rate_val ? pkt.rate_val + ' Mbps' : '--'})</span> · Len: <span>${pkt.len || 0} bytes</span></div>`;
        html += `<div class="tree-leaf">SNR: <span>${Number.isFinite(pkt.snr_db) ? pkt.snr_db.toFixed(1) + ' dB' : '--'}</span> · EVM: <span>${Number.isFinite(pkt.evm_db) ? pkt.evm_db.toFixed(1) + ' dB' : '--'}</span> · CFO: <span>${Number.isFinite(pkt.cfo_hz) ? (pkt.cfo_hz / 1000).toFixed(1) + ' kHz' : '--'}</span></div>`;

        // 2. MAC Frame
        html += `<div class="tree-node">&#9656; <b>IEEE 802.11 MAC Frame</b> (${pkt.pkt_type || ''}: ${pkt.subtype || ''})</div>`;
        html += `<div class="tree-leaf">Frame Control: <span>${pkt.fc || '0x0000'}</span> · Seq: <span>${pkt.seq !== undefined ? pkt.seq : '--'}</span> (Frag: ${pkt.frag || 0})</div>`;
        html += `<div class="tree-leaf">Flags: <span>ToDS=${pkt.to_ds ? 1 : 0} FromDS=${pkt.from_ds ? 1 : 0} Retry=${pkt.retry ? 1 : 0} Prot=${pkt.protected ? 1 : 0}</span></div>`;
        html += `<div class="tree-leaf">Destination (DA): <span>${pkt.da || '--'}</span></div>`;
        html += `<div class="tree-leaf">Source (SA): <span>${pkt.sa || '--'}</span></div>`;
        html += `<div class="tree-leaf">BSSID: <span>${pkt.bssid || '--'}</span></div>`;

        // 3. Management
        if (pkt.ssid || pkt.channel || pkt.beacon_int_tu) {
            html += `<div class="tree-node">&#9656; <b>Management Parameters</b></div>`;
            if (pkt.ssid) html += `<div class="tree-leaf">SSID: <span>"${pkt.ssid}"</span></div>`;
            if (pkt.channel) html += `<div class="tree-leaf">Channel IE: <span>${pkt.channel}</span></div>`;
            if (pkt.beacon_int_tu) html += `<div class="tree-leaf">Beacon Interval: <span>${pkt.beacon_int_tu} TU (${(pkt.beacon_int_tu * 1.024).toFixed(1)} ms)</span></div>`;
        }

        // 4. LLC / Upper Layer
        if (pkt.has_llc || pkt.is_ipv4) {
            html += `<div class="tree-node">&#9656; <b>LLC / Network Decapsulation</b></div>`;
            if (pkt.ethertype) html += `<div class="tree-leaf">EtherType: <span>${pkt.ethertype} (${pkt.ethertype_name || ''})</span></div>`;
            if (pkt.is_ipv4) html += `<div class="tree-leaf">IPv4: <span>${pkt.ip_src} &rarr; ${pkt.ip_dst}</span> (Proto: ${pkt.ip_proto || 'IPv4'})</div>`;
            if (pkt.port_src || pkt.port_dst) html += `<div class="tree-leaf">Ports: <span>${pkt.port_src} &rarr; ${pkt.port_dst}</span></div>`;
        }

        // 5. Summary
        if (pkt.summary) {
            html += `<div class="tree-node">&#9656; <b>Summary</b></div>`;
            html += `<div class="tree-leaf"><span>${pkt.summary}</span></div>`;
        }

        detail.innerHTML = html;
    }

    /* Ring at the press. Color and bin are fixed from the point under
     * the cursor when the press started, and the sweep always takes `ms`. */
    _chargeStart(x, y, rgb, ms, onDone) {
        this._chargeStop();
        const el = $('hold-charge');
        const [r, g, b] = rgb;
        const css = `rgb(${r},${g},${b})`;
        const dim = `rgba(${r},${g},${b},0.28)`;
        el.style.left = x + 'px';
        el.style.top = y + 'px';
        const t0 = performance.now();
        const token = (this._chargeTok = (this._chargeTok || 0) + 1);
        const paint = (p) => {
            el.style.background = `conic-gradient(from -90deg, ${css} ${(p * 100).toFixed(1)}%, ${dim} 0)`;
        };
        paint(0);
        el.classList.add('on');
        const tick = () => {
            if (this._chargeTok !== token) return;
            const p = Math.min(1, (performance.now() - t0) / ms);
            paint(p);
            if (p < 1) this._chargeTimer = setTimeout(tick, 16);
            else if (onDone) onDone();
        };
        this._chargeTimer = setTimeout(tick, 16);
    }

    _chargeStop() {
        this._chargeTok = (this._chargeTok || 0) + 1;
        if (this._chargeTimer) clearTimeout(this._chargeTimer);
        this._chargeTimer = 0;
        const el = $('hold-charge');
        if (el) el.classList.remove('on');
    }

    // ==================================================================
    // Pointer routing on the GL canvas: inside-view look
    // ==================================================================

    _bindPointer() {
        const gl = $('gl');
        let down = null, lastPos = null;
        const activePointers = new Map();
        let prevPinchDist = null;

        gl.addEventListener('pointerdown', (e) => {
            activePointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
            if (activePointers.size === 2) {
                const pts = Array.from(activePointers.values());
                prevPinchDist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
            } else {
                prevPinchDist = null;
            }
            down = true;
            lastPos = { x: e.clientX, y: e.clientY };
        });

        gl.addEventListener('pointermove', (e) => {
            if (activePointers.has(e.pointerId)) {
                activePointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
            }
            if (activePointers.size >= 2 && this.renderer.morph > 0.5 && this.renderer.sphereCam === 'inside') {
                const pts = Array.from(activePointers.values());
                const dist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
                if (prevPinchDist !== null && prevPinchDist > 0) {
                    const diff = prevPinchDist - dist;
                    this.renderer.insideZoom(diff * 0.15);
                }
                prevPinchDist = dist;
                return;
            }
            if (!down) return;
            if (this.renderer.morph > 0.5 && this.renderer.sphereCam === 'inside' && lastPos && activePointers.size <= 1) {
                const dy = (e.clientX - lastPos.x) / innerWidth * 2.2;
                const dp = (e.clientY - lastPos.y) / innerHeight * 1.6;
                this.renderer.insideLook(dy, dp);
            }
            lastPos = { x: e.clientX, y: e.clientY };
        });

        const pointerEnd = (e) => {
            activePointers.delete(e.pointerId);
            if (activePointers.size < 2) prevPinchDist = null;
            if (activePointers.size === 0) down = null;
        };
        gl.addEventListener('pointerup', pointerEnd);
        gl.addEventListener('pointercancel', pointerEnd);

        gl.addEventListener('wheel', (e) => {
            if (this.renderer.morph > 0.5 && this.renderer.sphereCam === 'inside') {
                e.preventDefault();
                this.renderer.insideZoom(e.deltaY * 0.04);
            }
        }, { passive: false });
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
        this._positionHoldRow();
        this._drawFft();
        this._drawCal(this._calDrag);
    }

    // ==================================================================
    // Status / fps line
    // ==================================================================

    _lockLabel(st) {
        if (!st || st.lock_busy) return 'MEASURING';
        if (st.lock_err) return 'FAILED';
        if (!(st.lock_lo > 0)) return 'NOT MEASURED';
        return `${st.lock_lo}–${st.lock_hi}`;
    }

    _syncSweepFields(st) {
        const lo = $('s-sweep-lo'), hi = $('s-sweep-hi');
        if (!lo || !hi || !st) return;
        const busy = !!st.lock_busy;
        lo.disabled = busy;
        hi.disabled = busy;
        if (this._sweepEdit) return;
        if (st.hw_min != null) lo.value = String(Math.round(st.hw_min));
        if (st.hw_max != null) hi.value = String(Math.round(st.hw_max));
    }

    onState(st) {
        this._gateRates(st);
        this.state = st;
        if (st.hw_min != null && st.hw_max != null) {
            const lo = st.hw_min, hi = st.hw_max;
            const prevLo = this.boardLo, prevHi = this.boardHi;
            const limChanged = lo !== prevLo || hi !== prevHi;
            this.boardLo = lo;
            this.boardHi = hi;
            const tf = $('target-freq');
            if (tf) { tf.min = String(lo); tf.max = String(hi); }
            if (limChanged) {
                let changed = false;
                /* A thumb parked on the old ceiling follows it. A window
                 * the user placed inside only moves when it would stick out. */
                if (this.s.manualLo <= prevLo + 0.5) { this.s.manualLo = lo; changed = true; }
                else if (this.s.manualLo < lo) { this.s.manualLo = lo; changed = true; }
                if (this.s.manualHi >= prevHi - 0.5) { this.s.manualHi = hi; changed = true; }
                else if (this.s.manualHi > hi) { this.s.manualHi = hi; changed = true; }
                if (this.s.targetFreq < lo) this.s.targetFreq = lo;
                if (this.s.targetFreq > hi) this.s.targetFreq = hi;
                if (changed) {
                    if (this._layoutManual) this._layoutManual();
                    this._applyManualRange(true);
                    this.save();
                }
            }
        }
        const lab = $('v-lock');
        if (lab) lab.textContent = this._lockLabel(st);
        this._syncSweepFields(st);
        const lockBtn = $('s-lock');
        if (lockBtn) lockBtn.disabled = !!st.lock_busy || st.mode === 'video' || st.mode === 'wifi';
        const gainWrap = $('gain-wrap');
        if (gainWrap) {
            gainWrap.classList.toggle('agc', st.mode === 'video' || st.mode === 'wifi');
            gainWrap.title = (st.mode === 'video' || st.mode === 'wifi')
                ? 'Decoder AGC is running. This slider returns with the sweep.'
                : '';
        }
        this._syncVideo(st);
        this._syncWifi(st);
        if (st.mode === 'video' || st.mode === 'wifi') {
            $('gain-label').textContent = 'AGC';
        } else if (!this._gainDraggingRef()) {
            // don't fight the user's finger; adopt backend gain otherwise
            const g = Math.max(0, Math.min(RF_GAIN_MAX, st.gain | 0));
            if (g !== this.s.hwGain) this.s.hwGain = g;
            this._layoutGain();
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
            el.textContent = '';
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
        if (st && st.mode === 'video') {
            $('fps').textContent =
                `FPS:${gpuFps.toFixed(0)}  VID:${(this._vidFps || 0).toFixed(1)}`;
            return;
        }
        if (st && st.mode === 'wifi') {
            $('fps').textContent =
                `FPS:${gpuFps.toFixed(0)}  WIFI PKTS:${this._wifiPktCount}`;
            return;
        }
        let extra = '';
        if (st && typeof st.lna_db === 'number' && st.lna_db >= 0) {
            extra = `  LNA:${st.lna_db}  VGA:${st.vga_db}`;
        }
        $('fps').textContent =
            `FPS:${gpuFps.toFixed(0)}  SWEEP/S:${netFps.toFixed(1)}  PTS:${pts}${extra}`;
    }
}

function sanitizeNtscSel(v) {
    if (!Array.isArray(v)) return [];
    const out = [];
    for (const id of v) {
        if (typeof id === 'string' && ntscById(id) && !out.includes(id)) out.push(id);
    }
    return out;
}

function sanitizeWifiSel(v) {
    if (!Array.isArray(v)) return [];
    return v.filter(b => Array.isArray(b) && b.length >= 2)
        .map(b => [Number(b[0]), Number(b[1])])
        .filter(b => Number.isFinite(b[0]) && Number.isFinite(b[1]));
}

/* Neighbors at least 60 MHz of hue distance so they stay distinct. */
function chanHueTs(fcs) {
    const n = fcs.length;
    if (n <= 1) return [0.5];
    if (n === 2) return [0, 1];
    const gaps = [];
    for (let i = 0; i < n - 1; i++)
        gaps.push(Math.max(fcs[i + 1] - fcs[i], 60));
    let acc = 0;
    const total = gaps.reduce((a, b) => a + b, 0);
    const ts = [0];
    for (const g of gaps) { acc += g / total; ts.push(acc); }
    return ts;
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
