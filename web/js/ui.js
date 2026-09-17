// ui.js — FREQ / COLOR / CONF panels, gain slider, fps line.

import { WIFI_TIERS, WIFI_F0, WIFI_F1, WIFI_SCAN_BANDS, WIFI_HOP_BANDS, WIFI_GAP } from './wifi.js';
import { SCHEMES, schemeCss, lutRgb } from './colors.js';

const TARGET_LUTS = ['spectrum', 'iron', 'whitehot', 'greenhot', 'viridis'];

const HW_MIN = 4900, HW_MAX = 6100;
const RF_GAIN_MAX = 63;
const FFT_SPEEDS = ['fast', 'med', 'slow'];
const FFT_SPEED_LAB = { fast: 'FAST', med: 'MED', slow: 'SLOW' };
const WIFI_COLOR = ['band', 'chan', 'full'];
const WIFI_COLOR_LAB = { band: 'BAND', chan: 'CHAN', full: 'FULL' };
/* Video-average time constants (s). Independent of sweep rate. */
const FFT_TAU = { fast: 0.07, med: 0.28, slow: 0.90 };
/* AGC attack / release (s). Attack follows a new peak; release holds the scale. */
const FFT_AGC_ATK = 0.18;
const FFT_AGC_REL = 1.10;
/* Floor = live min. Pad puts that trough a little above the axis. */
const FFT_LO_PAD = 0.10;
/* Keep Y from ranging into the hop scallop when the band is empty. */
const FFT_Y_MIN_RATIO = 1.75;
/* Analog RX is 20 MHz. Fold is ±10 MHz around each LO. Classify that
 * IF shape (p20 per offset) and subtract it from the FFT canvas only.
 * Hemisphere points stay on raw CFAR intensity — this mask does not
 * scale, gate, or recolor them. */
const FFT_HOP_MHZ = 20;
const FFT_EQ_N = 20;
const FFT_EQ_PCT = 0.20;
const FFT_EQ_TAU = 0.40;
/* One 20 MHz hop has one sample per IF offset — p20 is the spectrum
 * itself. Need ≥2 hops to classify the analog shape. */
const FFT_EQ_MIN_HOPS = 2;
const LS_KEY = 'phasegaze.settings.v6';
const ACCENT_DEFAULT = '#b8c4b8';

const DEFAULTS = {
    size: 15, gain: 4.0, decay: 23, density: 100,
    pulse: false, flip: false,
    mirrors: true, bottom: false, tiles: true, rings: true,
    scheme: 'spectrum', targetLut: 'iron', targetFreq: 5500, targetWidth: 40,
    freqPin: true, wifiColor: 'band',
    fft: true, fftSpeed: 'fast', fftAgc: true,
    hwGain: 45,
    manualLo: HW_MIN, manualHi: HW_MAX,
    wifiSel: [],
    accent: ACCENT_DEFAULT,
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

/* decay slider (0..100) -> time constant in seconds; 100 = hold forever */
function decayTau(v) {
    if (v >= 100) return 0;
    return 0.05 * Math.pow(400, v / 100);   // 0.05 s .. 20 s
}
function decayLabel(v) {
    return v >= 100 ? 'HOLD' : `${decayTau(v).toFixed(2)} S`;
}

const $ = (id) => document.getElementById(id);

export class Ui {
    constructor({ net, renderer }) {
        this.net = net;
        this.renderer = renderer;

        this.s = { ...DEFAULTS, ...this._load() };
        this.s.hwGain = Math.max(0, Math.min(RF_GAIN_MAX, this.s.hwGain | 0));
        this.s.accent = parseHex(this.s.accent) || ACCENT_DEFAULT;
        if (!TARGET_LUTS.includes(this.s.targetLut)) this.s.targetLut = 'iron';
        if (typeof this.s.fft !== 'boolean') this.s.fft = true;
        if (!FFT_SPEEDS.includes(this.s.fftSpeed)) this.s.fftSpeed = 'fast';
        if (typeof this.s.fftAgc !== 'boolean') this.s.fftAgc = true;
        if (typeof this.s.freqPin !== 'boolean') this.s.freqPin = true;
        if (typeof this.s.mirrors !== 'boolean') this.s.mirrors = true;
        if (typeof this.s.rings !== 'boolean') this.s.rings = this.s.tiles !== false;
        if (!WIFI_COLOR.includes(this.s.wifiColor))
            this.s.wifiColor = this.s.wifiChan === true ? 'chan' : 'band';
        delete this.s.wifiChan;
        this.s.wifiSel = sanitizeWifiSel(this.s.wifiSel);

        this.spectrum = null;
        this._fftAvg = null;
        this._fftHold = null;
        this._fftScale = 1e-6;
        this._fftLo = 0;
        this._fftScratch = [];
        this._fftEq = new Float32Array(FFT_EQ_N);
        this._fftEqMean = 0;
        this._fftEqBkt = Array.from({ length: FFT_EQ_N }, () => []);
        this._fftHops = 0;
        this._fftT = 0;
        this._fftDt = 0.03;
        this.state = null;
        this.netFps = 0;
        this.gpuFps = 0;
        this.pts = 0;
        this._gainSendTimer = 0;
        this._rangeSendTimer = 0;
        this._freqTab = 'range';

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
        this._applyAll();
        this._resize();
        window.addEventListener('resize', () => this._resize());
    }

    _load() {
        try {
            const cur = localStorage.getItem(LS_KEY);
            if (cur) return JSON.parse(cur) || {};
            const v3 = localStorage.getItem('phasegaze.settings.v3');
            if (v3) {
                const prev = JSON.parse(v3) || {};
                prev.mirrors = true;
                if (typeof prev.rings !== 'boolean')
                    prev.rings = prev.tiles !== false;
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

    /* Backend re-sync (on connect). */
    pushBackend() {
        this.net.set({ gain: this.s.hwGain, output_fraction: this.s.density / 100,
            spectrum: this.s.fft ? 1 : 0 });
        if (this.s.scheme === 'target') this._applyTargetSweep(true);
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
            $('btn-view').textContent = (next === 'orbit' ? 'inside' : 'orbit').toUpperCase();
        };
        $('btn-reset-view').onclick = () => this.renderer.resetView();
    }

    _openFreq() {
        this._closeColor();
        this._closeSet();
        $('freq-pop').classList.add('open');
        $('btn-freq').classList.add('on');
        if (this._selectFreqTab) this._selectFreqTab('range', false);
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
        $('ftab-range').onclick = () => selectTab('range');
        $('ftab-wifi').onclick = () => selectTab('wifi');
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
        selectTab('range', false);
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
        } else {
            const avg = this._fftAvg;
            for (let i = 0; i < n; i++) {
                if (f32[i] <= 1e-8) avg[i] = 0;
                else avg[i] += a * (f32[i] - avg[i]);
            }
        }
        this.spectrum = this._fftAvg;
        this._specF0 = header.loStart;
        this._specF1 = header.loEnd;
        if (!this.s.fftAgc && !this._fftHold)
            this._fftHold = this._fftAvg.slice();
        this._updateFftEq(f32);
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
            start = this.s.manualLo;
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

    /* FFT-canvas only. Do not use on ingest / sphere intensity. */
    _fftEqV(v, f) {
        if (this._fftHops < FFT_EQ_MIN_HOPS || this._fftEqMean <= 0) return v;
        return v - (this._fftEq[this._fftHopIdx(f)] - this._fftEqMean);
    }

    _updateFftEq(raw) {
        if (!raw) return;
        const specF0 = this._specF0 ?? HW_MIN;
        const specF1 = this._specF1 ?? HW_MAX;
        const n = raw.length;
        if (!n) return;
        const bin = (specF1 - specF0) / n;
        const bkt = this._fftEqBkt;
        const hops = new Set();
        for (let k = 0; k < FFT_EQ_N; k++) bkt[k].length = 0;
        for (let i = 0; i < n; i++) {
            const v = raw[i];
            if (v <= 1e-8) continue;
            const f = specF0 + i * bin;
            hops.add(Math.round(this._fftHopLo(f)));
            bkt[this._fftHopIdx(f)].push(v);
        }
        this._fftHops = hops.size;
        if (!this.s.fftAgc || this._fftHops < FFT_EQ_MIN_HOPS) return;
        const dt = this._fftDt || 0.03;
        const a = 1 - Math.exp(-dt / FFT_EQ_TAU);
        const eq = this._fftEq;
        let sum = 0, ntap = 0;
        for (let k = 0; k < FFT_EQ_N; k++) {
            const b = bkt[k];
            if (!b.length) {
                if (eq[k] > 0) { sum += eq[k]; ntap++; }
                continue;
            }
            b.sort((x, y) => x - y);
            const p = b[Math.min(b.length - 1, Math.floor(b.length * FFT_EQ_PCT))];
            if (eq[k] <= 0) eq[k] = p;
            else eq[k] += a * (p - eq[k]);
            sum += eq[k];
            ntap++;
        }
        this._fftEqMean = ntap ? sum / ntap : 0;
    }

    _fftLiveVals(src) {
        const { i0, i1, specF0, bin } = this._fftView();
        const wifi = this._freqTab === 'wifi';
        const a1 = WIFI_SCAN_BANDS[0][1], b0 = WIFI_SCAN_BANDS[1][0];
        const out = this._fftScratch;
        out.length = 0;
        let mx = 1e-6, mn = Infinity;
        for (let i = i0; i < i1; i++) {
            const raw = src[i];
            if (raw <= 1e-8) continue;
            const f = specF0 + i * bin;
            if (wifi && f >= a1 && f < b0) continue;
            const v = this._fftEqV(raw, f);
            out.push(v);
            if (v > mx) mx = v;
            if (v < mn) mn = v;
        }
        return { mx, mn: Number.isFinite(mn) ? mn : mx, n: out.length };
    }

    _updateFftScale() {
        if (!this._fftAvg) return;
        const { mx, mn } = this._fftLiveVals(this._fftAvg);
        const lo = mn;
        if (this._fftScale <= 1e-6) this._fftScale = mx;
        if (this._fftLo <= 0) this._fftLo = lo;
        if (!this.s.fftAgc) return;
        if (this._fftHops < FFT_EQ_MIN_HOPS) {
            this._fftScale = mx;
            this._fftLo = lo;
            if (this._fftLo > this._fftScale * 0.95)
                this._fftLo = this._fftScale * 0.95;
            return;
        }
        const dt = this._fftDt || 0.03;
        const pk = 1 - Math.exp(-dt / (mx > this._fftScale ? FFT_AGC_ATK : FFT_AGC_REL));
        this._fftScale += pk * (mx - this._fftScale);
        this._fftLo = lo;
        if (this._fftLo > this._fftScale * 0.95)
            this._fftLo = this._fftScale * 0.95;
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
        const hi = this._fftHops < FFT_EQ_MIN_HOPS
            ? Math.max(this._fftScale, 1e-6)
            : Math.max(this._fftScale, lo * FFT_Y_MIN_RATIO, 1e-6);
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
            const v = this._fftEqV(raw, bf0);
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
        bindToggle('flip', 'flip', v => this.renderer.setFlipX(v));
        bindToggle('bottom', 'bottom', v => this.renderer.setBottom(v));
        bindToggle('tiles', 'tiles', v => this.renderer.setTiles(v));
        bindToggle('rings', 'rings', v => this.renderer.setRings(v));

        const accent = $('s-accent');
        accent.oninput = () => {
            const hex = parseHex(accent.value);
            if (!hex) return;
            this.s.accent = hex;
            $('v-accent').textContent = hex.toUpperCase();
            this._applyAccent();
        };
        accent.onchange = () => this.save();

        $('s-fullscreen').onclick = () => {
            if (!document.fullscreenElement)
                document.documentElement.requestFullscreen().catch(() => {});
            else document.exitFullscreen();
        };
        $('s-clear').onclick = () => this.renderer.clearPoints();
        $('s-defaults').onclick = () => this._restoreDefaults();
    }

    _restoreDefaults() {
        this.s = { ...DEFAULTS, wifiSel: [] };
        this.save();
        this._applyAll();
        this._syncSchemeUi();
        this._restoreWifiTiles();
        if (this._layoutManual) this._layoutManual();
        this._layoutGain();
        this._placeFft();
        this.pushBackend();
    }

    _syncSettingUi() {
        $('s-size').value = this.s.size; $('v-size').textContent = this.s.size.toFixed(0);
        $('s-gain').value = this.s.gain; $('v-gain').textContent = this.s.gain.toFixed(1);
        $('s-decay').value = this.s.decay; $('v-decay').textContent = decayLabel(this.s.decay);
        $('s-dens').value = this.s.density; $('v-dens').textContent = `${this.s.density.toFixed(0)}%`;
        $('s-accent').value = this.s.accent;
        $('v-accent').textContent = this.s.accent.toUpperCase();
        const tmap = { pulse: 'pulse', flip: 'flip',
                       bottom: 'bottom', tiles: 'tiles', rings: 'rings' };
        for (const [id, key] of Object.entries(tmap))
            $(`t-${id}`).classList.toggle('on', !!this.s[key]);
        $('btn-mirror').classList.toggle('on', !!this.s.mirrors);
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
        $('freq-pop').style.width = w;
        $('color-pop').style.width = w;
        $('set-pop').style.width = w;
        if (this._layoutManual) this._layoutManual();
        if (this.s.fft) requestAnimationFrame(() => this._drawFft());
    }

    _resize() {
        this._layoutHudPops();
        this._drawFft();
    }

    // ==================================================================
    // Status / fps line
    // ==================================================================

    onState(st) {
        this.state = st;
        if (!this._gainDraggingRef()) {
            // don't fight the user's finger; adopt backend gain otherwise
            this.s.hwGain = Math.max(0, Math.min(RF_GAIN_MAX, st.gain | 0));
            this._layoutGain();
        }
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
