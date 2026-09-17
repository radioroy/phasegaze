// net.js — WebSocket stream client with auto-reconnect.
//
// Binary frames: 32-byte header (see backend/pg_stream.h) + payload.
// Text frames: JSON state / control.

const PG_MAGIC = 0x315A4750;
const HDR_BYTES = 32;

export class Net {
    constructor() {
        this.ws = null;
        this.onPoints = null;
        this.onSpectrum = null;
        this.onState = null;
        this.onStatus = null;
        this._timer = null;
        this._closed = false;
    }

    connect() {
        this._closed = false;
        const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
        let base = location.pathname;
        if (!base.endsWith('/')) base = base.replace(/[^/]*$/, '');
        const url = `${proto}//${location.host}${base}ws`;

        const ws = new WebSocket(url);
        ws.binaryType = 'arraybuffer';
        this.ws = ws;

        ws.onopen = () => {
            if (this.onStatus) this.onStatus('live');
            this.send({ type: 'get_state' });
        };
        ws.onclose = () => {
            this.ws = null;
            if (this._closed) return;
            if (this.onStatus) this.onStatus('lost');
            clearTimeout(this._timer);
            this._timer = setTimeout(() => this.connect(), 1000);
        };
        ws.onerror = () => { try { ws.close(); } catch (_) {} };
        ws.onmessage = (ev) => {
            if (typeof ev.data === 'string') {
                try {
                    const j = JSON.parse(ev.data);
                    if (j.type === 'state' && this.onState) this.onState(j);
                } catch (_) {}
                return;
            }
            this._onBinary(ev.data);
        };
    }

    _onBinary(buf) {
        if (buf.byteLength < HDR_BYTES) return;
        const dv = new DataView(buf);
        if (dv.getUint32(0, true) !== PG_MAGIC) return;
        const header = {
            version: dv.getUint16(4, true),
            type: dv.getUint16(6, true),
            count: dv.getUint32(8, true),
            loStart: dv.getFloat32(12, true),
            loEnd: dv.getFloat32(16, true),
            fps: dv.getFloat32(20, true),
            seq: dv.getUint32(24, true),
        };
        if (header.type === 0 && this.onPoints) {
            const need = HDR_BYTES + header.count * 16;
            if (buf.byteLength < need) return;
            this.onPoints(header, new Float32Array(buf, HDR_BYTES, header.count * 4));
        } else if (header.type === 1 && this.onSpectrum) {
            const need = HDR_BYTES + header.count * 4;
            if (buf.byteLength < need) return;
            this.onSpectrum(header, new Float32Array(buf, HDR_BYTES, header.count));
        }
    }

    disconnect() {
        this._closed = true;
        clearTimeout(this._timer);
        this._timer = null;
        if (this.ws) {
            try { this.ws.close(); } catch (_) {}
            this.ws = null;
        }
    }

    send(obj) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN)
            this.ws.send(JSON.stringify(obj));
    }

    set(patch) { this.send({ type: 'set', ...patch }); }
}
