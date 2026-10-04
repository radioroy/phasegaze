// web_ingest_bench.js
// Paste into the PhaseGaze page (or Runtime.evaluate it). Times the point
// ingest and the render call on the live stream, then on a synthetic
// worst-case coalesced packet (16384 points). Resolves to a JSON summary.
//
//   await pgIngestBench({ liveSecs: 8, plan: 'single' })

window.pgIngestBench = async function ({ liveSecs = 8, plan = null, synthReps = 200 } = {}) {
    const r = window.__renderer, ui = window.__ui;
    const PLANS = {
        full:   { lo_start: 4480, lo_end: 6740, bands: [] },
        single: { lo_start: 5170, lo_end: 5895, bands: [[5735, 5755]] },
    };
    if (plan) {
        ui.net.set(PLANS[plan]);
        await new Promise(res => setTimeout(res, 2000));
    }

    const med = a => { const s = [...a].sort((x, y) => x - y); return s.length ? s[s.length >> 1] : 0; };
    const p99 = a => { const s = [...a].sort((x, y) => x - y); return s.length ? s[Math.min(s.length - 1, Math.floor(s.length * 0.99))] : 0; };

    const ing = [], ingPts = [], rend = [];
    const oIngest = r.ingestFrame, oRender = r.render;
    r.ingestFrame = function (h, f) {
        const t = performance.now();
        oIngest.call(this, h, f);
        ing.push(performance.now() - t);
        ingPts.push(h.count);
    };
    r.render = function (dt) {
        const t = performance.now();
        oRender.call(this, dt);
        rend.push(performance.now() - t);
    };
    await new Promise(res => setTimeout(res, liveSecs * 1000));
    r.ingestFrame = oIngest;
    r.render = oRender;

    const totPts = ingPts.reduce((a, b) => a + b, 0);
    const totIng = ing.reduce((a, b) => a + b, 0);

    // Synthetic worst case: one full coalesced packet.
    const N = 16384;
    const f32 = new Float32Array(N * 4);
    for (let i = 0; i < N; i++) {
        const a = (i * 2.399963) % (2 * Math.PI), rr = Math.sqrt((i % 997) / 997);
        f32[i * 4] = rr * Math.cos(a);
        f32[i * 4 + 1] = rr * Math.sin(a);
        f32[i * 4 + 2] = 5170 + (i % 725);
        f32[i * 4 + 3] = (i % 101) / 100;
    }
    const hdr = { count: N, loStart: 5170, loEnd: 5895 };
    const syn = [];
    for (let k = 0; k < synthReps; k++) {
        const t = performance.now();
        oIngest.call(r, hdr, f32);
        syn.push(performance.now() - t);
    }
    return JSON.stringify({
        live_packets: ing.length,
        live_packets_s: +(ing.length / liveSecs).toFixed(1),
        live_points_s: Math.round(totPts / liveSecs),
        live_ingest_ms_med: +med(ing).toFixed(3),
        live_ingest_ms_p99: +p99(ing).toFixed(3),
        live_ingest_ms_per_s: +(totIng / liveSecs).toFixed(2),
        live_ns_per_point: totPts ? Math.round(totIng / totPts * 1e6) : 0,
        render_ms_med: +med(rend).toFixed(3),
        render_ms_p99: +p99(rend).toFixed(3),
        frames_s: +(rend.length / liveSecs).toFixed(1),
        synth16k_ms_med: +med(syn).toFixed(3),
        synth16k_ms_p99: +p99(syn).toFixed(3),
        synth_ns_per_point: Math.round(med(syn) / N * 1e6),
    });
};
