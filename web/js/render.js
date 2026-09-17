// render.js — single WebGL scene for both views.
//
// One preallocated point cloud ring buffer. The vertex shader computes both
// the AR screen mapping (direction cosines -> draggable quad homography) and
// the sphere projection, then mixes the two NDC positions with uMorph. Decay
// is fully shader-side from per-point birth timestamps, so idle frames cost
// no CPU. Mirror tiles are extra draw calls of the same geometry with a
// gradient-space lattice offset.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export const SF_PER_MHZ = 2 * Math.PI * 0.0455 / 299.792458; // 2*pi*d/lambda per MHz
const ORBIT_HOME = 2.45;      // default orbit radius (scroll zoom)
const CAPACITY = 65536;

// Reciprocal lattice basis (see phasegaze rf_math.js).
const R1X = 4 * Math.PI / Math.sqrt(3);
const R2X = 2 * Math.PI / Math.sqrt(3);
const R2Y = 2 * Math.PI;

function mirrorOffsets() {
    const out = [];
    for (let n1 = -2; n1 <= 2; n1++)
        for (let n2 = -2; n2 <= 2; n2++) {
            if (n1 === 0 && n2 === 0) continue;
            const ox = n1 * R1X + n2 * R2X, oy = n2 * R2Y;
            if (Math.hypot(ox, oy) <= 10.5) out.push([ox, oy]);
        }
    return out;
}

const POINT_VS = /* glsl */`
attribute vec2 aGrad;
attribute vec4 aAux;    // freq_mhz, intensity, birth_sec, _
attribute vec4 aQuat;   // device->world at ingest

uniform float uNow, uDecayTau, uGain, uPointSize, uPixelRatio, uPulse;
uniform float uMorph, uFlipX, uFlipW, uSfK, uExtrap;
uniform vec2  uMirror;
uniform vec4  uQuatView;      // conj(current device->world)
uniform vec2  uC0, uC1, uC2, uC3;   // quad corners, screen uv (y down)
uniform int   uColorMode;
uniform sampler2D uLut;
uniform sampler2D uFreqT;
uniform float uChanMap;
uniform float uFreqLo, uFreqHi, uTargetFreq, uTargetWidth;

varying vec4 vColor;
varying float vPulse;

vec3 qrot(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

vec2 quadMap(vec2 t) {
    vec2 tm = (t - 0.5) * uExtrap + 0.5;
    vec2 top = mix(uC0, uC1, tm.x);
    vec2 bot = mix(uC3, uC2, tm.x);
    return mix(top, bot, tm.y);
}

void kill() { gl_Position = vec4(0.0, 0.0, 2.0, 1.0); gl_PointSize = 0.0; vColor = vec4(0.0); }

void main() {
    float freq = aAux.x;
    float inten = aAux.y;
    float sf = uSfK * freq;

    vec2 g = aGrad + uMirror;
    vec2 uv = g / sf;
    float r2 = dot(uv, uv);
    if (r2 > 1.0) { kill(); return; }
    float w = sqrt(1.0 - r2) * uFlipW;

    // decay + intensity curve (uGain is a power-law exponent; high exponent
    // suppresses the noise floor, matching the proven AR tool response)
    float age = max(uNow - aAux.z, 0.0);
    float decay = (uDecayTau > 0.0) ? exp(-age / uDecayTau) : 1.0;
    float lvl = pow(clamp(inten, 0.0, 1.0), uGain);
    float alpha = lvl * 0.55 * decay;

    // device frame: X right, Y up, boresight along +Z
    vec3 dirDev = vec3(uv.x, uv.y, w);
    vec3 dirWorld = qrot(aQuat, dirDev);

    // --- AR path ---
    vec3 dirNow = qrot(uQuatView, dirWorld);
    vec2 aruv = vec2(dirNow.x * uFlipX * 0.5 + 0.5, 0.5 - dirNow.y * 0.5);
    vec2 spx = quadMap(aruv);
    vec3 ndcAR = vec3(spx.x * 2.0 - 1.0, 1.0 - spx.y * 2.0, 0.0);
    if (dirNow.z < 0.0) alpha *= uMorph;   // behind the phone: invisible in AR

    // --- sphere path ---
    vec4 clipS = projectionMatrix * modelViewMatrix * vec4(dirWorld * 1.005, 1.0);
    vec3 ndcS = (clipS.w > 0.0) ? clipS.xyz / clipS.w : vec3(0.0, 0.0, 2.0);
    if (clipS.w <= 0.0 && uMorph > 0.5) { kill(); return; }

    // color
    float t;
    vec3 col;
    if (uColorMode == 0) {
        if (uChanMap > 0.5) {
            float u = clamp((freq - 4900.0) / 1200.0, 0.0, 1.0);
            vec4 cm = texture2D(uFreqT, vec2(u, 0.5));
            if (cm.a < 0.5) { col = vec3(0.28, 0.30, 0.34); alpha *= 0.3; }
            else col = texture2D(uLut, vec2(cm.r, 0.5)).rgb;
        } else {
            t = clamp((freq - uFreqLo) / max(uFreqHi - uFreqLo, 1.0), 0.0, 1.0);
            col = texture2D(uLut, vec2(t, 0.5)).rgb;
        }
    } else if (uColorMode == 1) {
        col = texture2D(uLut, vec2(lvl, 0.5)).rgb;
    } else {
        float d = abs(freq - uTargetFreq) / max(uTargetWidth, 0.1);
        if (d > 1.0) { col = vec3(0.28, 0.30, 0.34); alpha *= 0.3; }
        else col = texture2D(uLut, vec2(clamp(1.0 - d, 0.0, 1.0), 0.5)).rgb;
    }

    // Facing check: points on the near hemisphere face the camera directly.
    // Points on the far side (seen through the see-through sphere) are smoothly
    // dimmed and scaled down so they read as background depth without clashing with the foreground.
    float szMod = 1.0;
    if (uMorph > 0.5 && length(cameraPosition) > 0.1) {
        vec3 toCam = normalize(cameraPosition - dirWorld);
        float facing = dot(dirWorld, toCam);
        float tf = smoothstep(-0.15, 0.15, facing);
        alpha *= mix(0.18, 1.0, tf);
        col   *= mix(0.45, 1.0, tf);
        szMod  = mix(0.80, 1.0, tf);
    }

    if (alpha < 0.004) { kill(); return; }

    vPulse = uPulse * (1.0 - decay);
    float sz = uPointSize * uPixelRatio * (0.35 + 0.65 * lvl) * szMod;
    sz *= 1.0 + vPulse * 2.2;

    gl_Position = vec4(mix(ndcAR, ndcS, uMorph), 1.0);
    gl_PointSize = sz;
    vColor = vec4(col, alpha);
}
`;

const POINT_FS = /* glsl */`
precision mediump float;
varying vec4 vColor;
varying float vPulse;
void main() {
    vec2 pc = gl_PointCoord - vec2(0.5);
    float r2 = dot(pc, pc);
    if (r2 > 0.25) discard;
    // Gaussian falloff (sigma = 0.20 in [0, 0.5] point sprite coords)
    float g = exp(-r2 / (2.0 * 0.20 * 0.20));
    float r = sqrt(r2);
    float ring = smoothstep(0.50, 0.40, r) * smoothstep(0.28, 0.38, r);
    float a = vColor.a * mix(g, ring, clamp(vPulse * 1.4, 0.0, 1.0));
    if (a < 0.004) discard;
    gl_FragColor = vec4(vColor.rgb, a);
}
`;

// Hemisphere shell: near-black fill, Voronoi tile boundaries, deterministic
// low/high frequency rings (port of the phasegaze shell shader).
const HEMI_VS = /* glsl */`
varying vec3 vPosition;
varying vec3 vNormal;
void main() {
    vPosition = position;
    vNormal = normalize(normalMatrix * normal);
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}
`;

const HEMI_FS = /* glsl */`
precision mediump float;
#define PI 3.1415926535897932
varying vec3 vPosition;
varying vec3 vNormal;
uniform float uScaleFactor, uScaleFactorLow, uScaleFactorHigh, uOpacity;
uniform float uShowTiles, uShowRings, uShowBottom;
uniform sampler2D uLut, uFreqT;
uniform float uChanMap, uFreqLo, uFreqHi, uSweepLo, uSweepHi;

vec3 freqColor(float freq) {
    if (uChanMap > 0.5) {
        float u = clamp((freq - 4900.0) / 1200.0, 0.0, 1.0);
        vec4 cm = texture2D(uFreqT, vec2(u, 0.5));
        if (cm.a < 0.5) return vec3(0.28, 0.30, 0.34);
        return texture2D(uLut, vec2(cm.r, 0.5)).rgb;
    }
    float t = clamp((freq - uFreqLo) / max(uFreqHi - uFreqLo, 1.0), 0.0, 1.0);
    return texture2D(uLut, vec2(t, 0.5)).rgb;
}

void main() {
    bool isFront = (vPosition.z >= -0.001);

    // Subtle edge rim so the silhouette reads cleanly without coloring in the face
    vec3 n = normalize(vNormal);
    float ndotv = abs(n.z);
    float rim = pow(1.0 - ndotv, 3.5);
    vec3 color = vec3(0.06 * rim);

    float lineMix = 0.0;
    float lL = 0.0, lH = 0.0;

    // Voronoi: front tiles if uShowTiles; back tiles if showBottom.
    // Frequency rings are a separate switch (same lattice, boresight cell).
    bool wantTiles = (isFront && uShowTiles > 0.5) || (!isFront && uShowBottom > 0.5);
    bool wantRings = isFront && uShowRings > 0.5;
    if (wantTiles || wantRings) {
        vec2 uv = vPosition.xy;
        float gx = uv.x * uScaleFactor;
        float gy = uv.y * uScaleFactor;

        vec2 r1 = vec2(4.0 * PI / sqrt(3.0), 0.0);
        vec2 r2 = vec2(2.0 * PI / sqrt(3.0), 2.0 * PI);

        float minDist1 = 1e20, minDist2 = 1e20;
        float minDistSqLow = 1e20, minDistSqHigh = 1e20;
        vec2 best_n = vec2(0.0);
        for (float n1 = -3.0; n1 <= 3.0; n1++) {
            for (float n2 = -3.0; n2 <= 3.0; n2++) {
                vec2 lattice = n1 * r1 + n2 * r2;
                vec2 diffPos = vec2(gx, gy) - lattice;
                float dist = dot(diffPos, diffPos);
                if (dist < minDist1) { minDist2 = minDist1; minDist1 = dist; best_n = vec2(n1, n2); }
                else if (dist < minDist2) { minDist2 = dist; }

                if (wantRings && !(n1 == 0.0 && n2 == 0.0)) {
                    vec2 diffL = uv - lattice / uScaleFactorLow;
                    minDistSqLow = min(minDistSqLow, dot(diffL, diffL));
                    vec2 diffH = uv - lattice / uScaleFactorHigh;
                    minDistSqHigh = min(minDistSqHigh, dot(diffH, diffH));
                }
            }
        }

        if (wantTiles) {
            float edgeDist = sqrt(minDist2) - sqrt(minDist1);
            float lineSmooth = fwidth(edgeDist) * 1.2;
            lineMix = 1.0 - smoothstep(0.035 - lineSmooth, 0.035 + lineSmooth, edgeDist);

            vec3 lineColor = (isFront && best_n.x == 0.0 && best_n.y == 0.0)
                ? vec3(0.35, 0.65, 0.35)
                : vec3(0.20, 0.20, 0.20);
            color = mix(color, lineColor, lineMix);
        }

        if (wantRings && best_n.x == 0.0 && best_n.y == 0.0) {
            float dL = abs(sqrt(minDistSqLow) - 1.0);
            float smL = fwidth(dL) * 1.2;
            lL = 1.0 - smoothstep(0.0018 - smL, 0.0018 + smL, dL);
            color = mix(color, freqColor(uSweepLo), lL * 0.9);

            float dH = abs(sqrt(minDistSqHigh) - 1.0);
            float smH = fwidth(dH) * 1.2;
            lH = 1.0 - smoothstep(0.0018 - smH, 0.0018 + smH, dH);
            color = mix(color, freqColor(uSweepHi), lH * 0.9);
        }
    }

    // Subtle equator line so the hemisphere boundary reads cleanly
    float eqDist = abs(vPosition.z);
    float eqLine = 1.0 - smoothstep(0.002, 0.015, eqDist);
    color = mix(color, vec3(0.18, 0.22, 0.18), eqLine * 0.4);

    float alpha = max(lineMix * 0.85, max(lL * 0.9, max(lH * 0.9, eqLine * 0.4)));
    alpha = max(alpha, rim * 0.30) * uOpacity;
    gl_FragColor = vec4(color, alpha);
}
`;

function buildHemiGeometry(front) {
    // Device frame shell: boresight along +Z. front=true covers z >= 0.
    const nlat = 40, nlon = 80;
    const verts = [];
    for (let lat = 0; lat <= nlat; lat++) {
        const theta = (lat / nlat) * (Math.PI * 0.5);
        const st = Math.sin(theta), ct = Math.cos(theta);
        for (let lon = 0; lon <= nlon; lon++) {
            const phi = (lon / nlon) * Math.PI * 2;
            verts.push(st * Math.cos(phi), st * Math.sin(phi), front ? ct : -ct);
        }
    }
    const idx = [];
    for (let lat = 0; lat < nlat; lat++)
        for (let lon = 0; lon < nlon; lon++) {
            const a = lat * (nlon + 1) + lon, b = a + nlon + 1;
            // Outward-facing normals for both caps (CCW when viewed from outside).
            if (front) idx.push(a, b, a + 1, a + 1, b, b + 1);
            else idx.push(a, a + 1, b, a + 1, b + 1, b);
        }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    geo.setIndex(idx);
    geo.computeVertexNormals();
    return geo;
}

const IDENT_Q = [0, 0, 0, 1];

export class VrfRenderer {
    constructor(canvas) {
        this.renderer = new THREE.WebGLRenderer({ canvas, alpha: false, antialias: false });
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setClearColor(0x000000, 1);

        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.05, 100);
        this.controls = new OrbitControls(this.camera, canvas);
        this.controls.enableDamping = true;
        this.controls.enablePan = false;
        this.controls.minDistance = 0.4;
        this.controls.maxDistance = 8;
        this.controls.enabled = false;
        this._orbitPos = new THREE.Vector3(0, 0, ORBIT_HOME);
        this._frontOrbitCam();
        this.controls.saveState();

        // ---- state ----
        this.morph = 1; this.morphTarget = 1;
        this.sphereCam = 'orbit';          // 'orbit' | 'inside'
        this.imuQuat = new THREE.Quaternion();
        this.imuEnabled = false;
        this.insideYaw = 0; this.insidePitch = 0;
        this.freqLo = 4900; this.freqHi = 6100;
        this.showMirrors = false;
        this.showBottom = false;
        this.onMorph = null;
        this._clock = performance;
        this._t0 = performance.now();
        this._fps = 0; this._lastT = 0;

        this._buildPoints();
        this._buildShell();

        window.addEventListener('resize', () => this._resize());
    }

    now() { return (performance.now() - this._t0) / 1000; }

    _buildPoints() {
        const geo = new THREE.BufferGeometry();
        this.aGrad = new THREE.BufferAttribute(new Float32Array(CAPACITY * 2), 2);
        this.aAux = new THREE.BufferAttribute(new Float32Array(CAPACITY * 4), 4);
        this.aQuat = new THREE.BufferAttribute(new Float32Array(CAPACITY * 4), 4);
        // birth defaults to -1e9 so untouched slots decay to invisible
        for (let i = 0; i < CAPACITY; i++) this.aAux.setZ(i, -1e9);
        geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(CAPACITY * 3), 3));
        geo.setAttribute('aGrad', this.aGrad);
        geo.setAttribute('aAux', this.aAux);
        geo.setAttribute('aQuat', this.aQuat);
        geo.setDrawRange(0, 0);
        this.pointGeo = geo;
        this.head = 0;
        this.used = 0;

        const lutTex = new THREE.DataTexture(new Uint8Array(256 * 4).fill(255), 256, 1);
        lutTex.needsUpdate = true;
        this._chanData = new Uint8Array(1200 * 4);
        this._chanTex = new THREE.DataTexture(this._chanData, 1200, 1);
        this._chanTex.magFilter = THREE.NearestFilter;
        this._chanTex.minFilter = THREE.NearestFilter;
        this._chanTex.flipY = false;
        this._chanTex.needsUpdate = true;

        this.pointUniforms = {
            uNow: { value: 0 }, uDecayTau: { value: 0.8 }, uGain: { value: 4.0 },
            uPointSize: { value: 10 }, uPixelRatio: { value: this.renderer.getPixelRatio() },
            uPulse: { value: 0 },
            uMorph: { value: 0 }, uFlipX: { value: 1 }, uFlipW: { value: 1 },
            uSfK: { value: SF_PER_MHZ }, uExtrap: { value: 4.0 },
            uMirror: { value: new THREE.Vector2(0, 0) },
            uQuatView: { value: new THREE.Vector4(0, 0, 0, 1) },
            uC0: { value: new THREE.Vector2(0.25, 0.25) },
            uC1: { value: new THREE.Vector2(0.75, 0.25) },
            uC2: { value: new THREE.Vector2(0.75, 0.75) },
            uC3: { value: new THREE.Vector2(0.25, 0.75) },
            uColorMode: { value: 0 },
            uLut: { value: lutTex },
            uFreqT: { value: this._chanTex },
            uChanMap: { value: 0 },
            uFreqLo: { value: 4900 }, uFreqHi: { value: 6100 },
            uTargetFreq: { value: 5500 }, uTargetWidth: { value: 40 },
        };

        this.pointMat = new THREE.ShaderMaterial({
            uniforms: this.pointUniforms,
            vertexShader: POINT_VS,
            fragmentShader: POINT_FS,
            transparent: true,
            depthTest: false,
            depthWrite: false,
            blending: THREE.AdditiveBlending,
        });

        // Draw pool: primary + mirrors, each optionally flipped to the back
        // hemisphere. onBeforeRender patches the per-draw uniforms.
        this.mirrorOffs = mirrorOffsets();
        this.drawPool = [];
        const mkDraw = (ox, oy, flip) => {
            const p = new THREE.Points(this.pointGeo, this.pointMat);
            p.frustumCulled = false;
            p.renderOrder = 2;
            p.onBeforeRender = () => {
                this.pointUniforms.uMirror.value.set(ox, oy);
                this.pointUniforms.uFlipW.value = flip;
                // shared material: force re-upload of uniforms for this draw
                this.pointMat.uniformsNeedUpdate = true;
            };
            p.visible = false;
            this.scene.add(p);
            this.drawPool.push({ mesh: p, mirror: (ox !== 0 || oy !== 0), flip });
            return p;
        };
        mkDraw(0, 0, 1);
        mkDraw(0, 0, -1);
        for (const [ox, oy] of this.mirrorOffs) { mkDraw(ox, oy, 1); mkDraw(ox, oy, -1); }
        this._updateDrawPool();
    }

    _updateDrawPool() {
        for (const d of this.drawPool) {
            let vis = true;
            if (d.mirror && !(this.showMirrors && this.morph > 0.02)) vis = false;
            if (d.flip < 0 && !(this.showBottom && this.morph > 0.02)) vis = false;
            d.mesh.visible = vis;
        }
    }

    _buildShell() {
        this.shellUniforms = {
            uScaleFactor: { value: SF_PER_MHZ * 5500 },
            uScaleFactorLow: { value: SF_PER_MHZ * 4900 },
            uScaleFactorHigh: { value: SF_PER_MHZ * 6100 },
            uOpacity: { value: 0 },
            uShowTiles: { value: 1 },
            uShowRings: { value: 1 },
            uShowBottom: { value: 0 },
            uLut: { value: this.pointUniforms.uLut.value },
            uFreqT: { value: this._chanTex },
            uChanMap: { value: 0 },
            uFreqLo: { value: 4900 },
            uFreqHi: { value: 6100 },
            uSweepLo: { value: 4900 },
            uSweepHi: { value: 6100 },
        };
        const mkMat = () => new THREE.ShaderMaterial({
            uniforms: this.shellUniforms,
            vertexShader: HEMI_VS,
            fragmentShader: HEMI_FS,
            transparent: true,
            side: THREE.FrontSide,
            depthTest: false,
            depthWrite: false,
        });
        this.shellGroup = new THREE.Group();
        this.shellFront = new THREE.Mesh(buildHemiGeometry(true), mkMat());
        this.shellBack = new THREE.Mesh(buildHemiGeometry(false), mkMat());
        this.shellFront.renderOrder = 0;
        this.shellBack.renderOrder = 0;
        this.shellGroup.add(this.shellFront);
        this.shellGroup.add(this.shellBack);
        this.scene.add(this.shellGroup);

        const tick = 0.028;
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.Float32BufferAttribute(
            [-tick, 0, 1.004, tick, 0, 1.004, 0, -tick, 1.004, 0, tick, 1.004], 3));
        this.boresight = new THREE.LineSegments(g,
            new THREE.LineBasicMaterial({
                color: 0xb8c4b8, transparent: true, opacity: 0.45,
                depthTest: false, depthWrite: false,
            }));
        this.boresight.renderOrder = 1;
        this.shellGroup.add(this.boresight);
    }

    // ------------------------------------------------------------------
    // Data ingest
    // ------------------------------------------------------------------

    ingestFrame(header, f32) {
        /* Occupied keep-band. Drives the boresight hex (λ at mid RF). */
        this.freqLo = header.loStart;
        this.freqHi = header.loEnd;
        const n = header.count;
        if (!n) return;

        const q = this.imuEnabled
            ? [this.imuQuat.x, this.imuQuat.y, this.imuQuat.z, this.imuQuat.w]
            : IDENT_Q;
        const birth = this.now();

        let lo = this.head, wrapped = false;
        for (let i = 0; i < n; i++) {
            const u = f32[i * 4], v = f32[i * 4 + 1];
            const freq = f32[i * 4 + 2], inten = f32[i * 4 + 3];
            const sf = SF_PER_MHZ * freq;
            const j = this.head;
            this.aGrad.setXY(j, u * sf, v * sf);
            this.aAux.setXYZW(j, freq, inten, birth, 0);
            this.aQuat.setXYZW(j, q[0], q[1], q[2], q[3]);
            this.head = (this.head + 1) % CAPACITY;
            if (this.head === 0) wrapped = true;
            if (this.used < CAPACITY) this.used++;
        }

        // Upload just the touched span (full upload on wrap, which is rare).
        for (const attr of [this.aGrad, this.aAux, this.aQuat]) {
            attr.needsUpdate = true;
            if (!wrapped && attr.updateRanges !== undefined) {
                attr.clearUpdateRanges();
                attr.addUpdateRange(lo * attr.itemSize, n * attr.itemSize);
            }
        }
        this.pointGeo.setDrawRange(0, this.used);
    }

    clearPoints() {
        for (let i = 0; i < CAPACITY; i++) this.aAux.setZ(i, -1e9);
        this.aAux.needsUpdate = true;
        this.head = 0; this.used = 0;
        this.pointGeo.setDrawRange(0, 0);
    }

    // ------------------------------------------------------------------
    // Settings
    // ------------------------------------------------------------------

    setLut(rgba, mode) {
        const tex = this.pointUniforms.uLut.value;
        tex.image.data.set(rgba);
        tex.needsUpdate = true;
        this.pointUniforms.uColorMode.value = mode;
    }
    setFreqSpan(lo, hi) {
        this.pointUniforms.uFreqLo.value = lo;
        this.pointUniforms.uFreqHi.value = hi;
    }
    /* ranges: [{f0,f1,t}] or null. t is the LUT coordinate for that channel. */
    setChanMap(ranges) {
        const data = this._chanData;
        data.fill(0);
        if (!ranges || !ranges.length) {
            this.pointUniforms.uChanMap.value = 0;
            this._chanTex.needsUpdate = true;
            return;
        }
        for (const r of ranges) {
            const i0 = Math.max(0, Math.floor(r.f0 - 4900));
            const i1 = Math.min(1200, Math.ceil(r.f1 - 4900));
            const t8 = Math.round(Math.max(0, Math.min(1, r.t)) * 255);
            for (let i = i0; i < i1; i++) {
                data[i * 4] = t8;
                data[i * 4 + 3] = 255;
            }
        }
        this._chanTex.needsUpdate = true;
        this.pointUniforms.uChanMap.value = 1;
    }
    setTarget(freq, width) {
        this.pointUniforms.uTargetFreq.value = freq;
        this.pointUniforms.uTargetWidth.value = width;
    }
    setDecayTau(tau) { this.pointUniforms.uDecayTau.value = tau; }
    setPointSize(s) { this.pointUniforms.uPointSize.value = s; }
    setPointGain(g) { this.pointUniforms.uGain.value = g; }
    setPulse(on) { this.pointUniforms.uPulse.value = on ? 1 : 0; }
    setFlipX(f) { this.pointUniforms.uFlipX.value = f ? -1 : 1; }
    setMirrors(on) { this.showMirrors = on; this._updateDrawPool(); }
    setBottom(on) { this.showBottom = on; this._updateDrawPool(); }
    setTiles(on) { this.shellUniforms.uShowTiles.value = on ? 1 : 0; }
    setRings(on) { this.shellUniforms.uShowRings.value = on ? 1 : 0; }
    setAccent(hex) {
        const m = /^#?([0-9a-fA-F]{6})$/.exec(hex || '');
        if (!m || !this.boresight) return;
        const n = parseInt(m[1], 16);
        this.boresight.material.color.setRGB(
            ((n >> 16) & 255) / 255,
            ((n >> 8) & 255) / 255,
            (n & 255) / 255);
    }
    setCorners(c) {
        this.pointUniforms.uC0.value.set(c[0].u, c[0].v);
        this.pointUniforms.uC1.value.set(c[1].u, c[1].v);
        this.pointUniforms.uC2.value.set(c[2].u, c[2].v);
        this.pointUniforms.uC3.value.set(c[3].u, c[3].v);
    }
    setImuEnabled(on) {
        if (this.imuEnabled !== on) this.clearPoints();
        this.imuEnabled = on;
        if (!on) this.imuQuat.identity();
    }
    setMorphTarget(t) { this.morphTarget = t; }
    // Burn leftover orbit damping/zoom without a visible jump. update()
    // applies sphericalDelta then zeros it; we restore the pose we want
    // and sync internals from that.
    _clearOrbitInertia() {
        const pos = this.camera.position.clone();
        const quat = this.camera.quaternion.clone();
        const target = this.controls.target.clone();
        const damp = this.controls.enableDamping;
        this.controls.enableDamping = false;
        this.controls.update();
        this.camera.position.copy(pos);
        this.camera.quaternion.copy(quat);
        this.controls.target.copy(target);
        this.controls.update();
        this.controls.enableDamping = damp;
    }

    _frontOrbitCam() {
        // Boresight / main hexagon is the +Z pole.
        this.camera.up.set(0, 1, 0);
        this.camera.position.set(0, 0, ORBIT_HOME);
        this.camera.lookAt(0, 0, 0);
        this.controls.target.set(0, 0, 0);
        this._clearOrbitInertia();
        this._orbitPos.copy(this.camera.position);
    }

    _saveOrbitCam() {
        this._clearOrbitInertia();
        this._orbitPos.copy(this.camera.position);
    }

    _restoreOrbitCam() {
        this.camera.up.set(0, 1, 0);
        this.camera.position.copy(this._orbitPos);
        this.controls.target.set(0, 0, 0);
        this.camera.lookAt(0, 0, 0);
        this._clearOrbitInertia();
    }

    setSphereCam(mode) {
        if (this.sphereCam === 'orbit' && mode === 'inside') this._saveOrbitCam();
        this.sphereCam = mode;
        this._viewReset = null;
        if (mode === 'orbit') this._restoreOrbitCam();
    }

    resetView() {
        this.controls.enabled = false;
        this._clearOrbitInertia();
        const offset = this.camera.position.clone().sub(this.controls.target);
        const sph = new THREE.Spherical().setFromVector3(offset);
        let theta = sph.theta;
        theta = ((theta + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
        this._viewReset = {
            t0: performance.now(),
            dur: 500,
            fromTheta: theta,
            fromPhi: sph.phi,
            fromRadius: Math.max(sph.radius, 0.05),
            fromYaw: this.insideYaw,
            fromPitch: this.insidePitch,
        };
    }

    /* Manual look-around for inside view (radians deltas). */
    insideLook(dyaw, dpitch) {
        this.insideYaw += dyaw;
        this.insidePitch = Math.max(-1.5, Math.min(1.5, this.insidePitch + dpitch));
    }

    // ------------------------------------------------------------------
    // Frame loop
    // ------------------------------------------------------------------

    render(dt) {
        // morph tween
        const rate = dt / 0.7;
        if (this.morph < this.morphTarget) this.morph = Math.min(this.morphTarget, this.morph + rate);
        else if (this.morph > this.morphTarget) this.morph = Math.max(this.morphTarget, this.morph - rate);
        this.pointUniforms.uMorph.value = this.morph;
        this.shellUniforms.uOpacity.value = this.morph;
        this.shellGroup.visible = this.morph > 0.02;
        this._updateDrawPool();
        if (this.onMorph) this.onMorph(this.morph);

        /* Boresight hex = unambiguous cell at mid occupied RF (2πd/λ).
         * Uses the keep-band header, not the WIFI 5170–5895 catalog. */
        const sfC = SF_PER_MHZ * 0.5 * (this.freqLo + this.freqHi);
        this.shellUniforms.uScaleFactor.value = sfC;
        this.shellUniforms.uScaleFactorLow.value = SF_PER_MHZ * this.freqLo;
        this.shellUniforms.uScaleFactorHigh.value = SF_PER_MHZ * this.freqHi;
        this.shellUniforms.uShowBottom.value = this.showBottom ? 1 : 0;
        this.shellUniforms.uSweepLo.value = this.freqLo;
        this.shellUniforms.uSweepHi.value = this.freqHi;
        this.shellUniforms.uFreqLo.value = this.pointUniforms.uFreqLo.value;
        this.shellUniforms.uFreqHi.value = this.pointUniforms.uFreqHi.value;
        this.shellUniforms.uChanMap.value = this.pointUniforms.uChanMap.value;

        // stabilization view quat (AR path)
        const qv = this.pointUniforms.uQuatView.value;
        if (this.imuEnabled) {
            const q = this.imuQuat.clone().invert();
            qv.set(q.x, q.y, q.z, q.w);
        } else qv.set(0, 0, 0, 1);

        // shell follows the device orientation so the tile pattern stays
        // aligned with the physical array
        if (this.imuEnabled) this.shellGroup.quaternion.copy(this.imuQuat);
        else this.shellGroup.quaternion.identity();

        // cameras
        if (this._viewReset) {
            const u = Math.min(1, (performance.now() - this._viewReset.t0) / this._viewReset.dur);
            // cubic ease-in-out: slow start, faster mid, settle at the end
            const e = u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2;
            const vr = this._viewReset;
            this.insideYaw = vr.fromYaw * (1 - e);
            this.insidePitch = vr.fromPitch * (1 - e);
            if (this.sphereCam !== 'inside') {
                const theta = vr.fromTheta * (1 - e);
                const phi = vr.fromPhi + (Math.PI / 2 - vr.fromPhi) * e;
                const radius = vr.fromRadius + (ORBIT_HOME - vr.fromRadius) * e;
                this.camera.position.setFromSphericalCoords(radius, phi, theta);
                this.camera.up.set(0, 1, 0);
                this.controls.target.set(0, 0, 0);
                this.camera.lookAt(0, 0, 0);
            }
            if (u >= 1) {
                this.insideYaw = 0;
                this.insidePitch = 0;
                this._viewReset = null;
                if (this.sphereCam === 'orbit') this._frontOrbitCam();
            }
        }

        if (this.morph > 0.02 && this.sphereCam === 'inside') {
            this.controls.enabled = false;
            this.camera.position.set(0, 0, 0);
            const q = new THREE.Quaternion();
            if (this.imuEnabled) q.copy(this.imuQuat);
            const qFwd = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);
            const qYaw = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), this.insideYaw);
            const qPitch = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), this.insidePitch);
            this.camera.quaternion.copy(q).multiply(qFwd).multiply(qYaw).multiply(qPitch);
        } else {
            this.controls.enabled = this.morph > 0.5 && !this._viewReset;
            if (this.controls.enabled) this.controls.update();
        }

        // Orbit sees the outer surface; inside-cam looks at the inner one.
        const hemiSide = (this.morph > 0.02 && this.sphereCam === 'inside')
            ? THREE.BackSide : THREE.FrontSide;
        this.shellFront.material.side = hemiSide;
        this.shellBack.material.side = hemiSide;
        this.pointMat.depthTest = false;

        this.pointUniforms.uNow.value = this.now();
        this.renderer.render(this.scene, this.camera);
    }

    _resize() {
        this.renderer.setSize(innerWidth, innerHeight);
        this.camera.aspect = innerWidth / innerHeight;
        this.camera.updateProjectionMatrix();
    }
}
