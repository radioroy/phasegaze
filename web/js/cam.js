// cam.js — rear camera feed for AR mode.
//
// Same capture request as the Spatial RF Vision tool: environment facing,
// 1080p ideal, and zoom pinned to the widest the sensor allows. Digital zoom
// would scale the image without changing the RF projection, so the alignment
// calibration only holds at minimum zoom.

const $ = (id) => document.getElementById(id);

export class Cam {
    constructor(video) {
        this.video = video;
        this.stream = null;
        this._bindNote();
        window.addEventListener('pagehide', () => this.stop());
    }

    _bindNote() {
        const close = $('cam-note-close');
        if (close) close.onclick = () => this.hideNote();
    }

    async start() {
        if (this.stream) return true;
        this.hideNote();
        const md = navigator.mediaDevices;
        if (!md || !md.getUserMedia) {
            this.showNote({ name: 'NotSupportedError' });
            return false;
        }
        try {
            const stream = await md.getUserMedia({
                video: {
                    facingMode: 'environment',
                    width: { ideal: 1920 },
                    height: { ideal: 1080 },
                },
            });
            this.stream = stream;
            this.video.srcObject = stream;
            const [track] = stream.getVideoTracks();
            const caps = track.getCapabilities ? track.getCapabilities() : {};
            if (caps && 'zoom' in caps) {
                track.applyConstraints({ advanced: [{ zoom: caps.zoom.min }] })
                    .catch(() => {});
            }
            return true;
        } catch (err) {
            this.showNote(err);
            return false;
        }
    }

    stop() {
        if (!this.stream) return;
        for (const t of this.stream.getTracks()) {
            try { t.stop(); } catch (_) {}
        }
        this.stream = null;
        this.video.srcObject = null;
        this.hideNote();
    }

    hideNote() { $('cam-note').classList.remove('show'); }

    showNote(err) {
        const insecure = !window.isSecureContext;
        const name = (err && err.name) || 'NotSupportedError';
        let title = 'CAMERA ERROR';
        let msg = (err && err.message) || 'This browser does not provide camera access.';

        if (insecure) {
            // Every current browser gates getUserMedia on a secure context;
            // localhost is the only insecure origin they trust.
            title = 'HTTPS REQUIRED';
            msg = 'This page is on HTTP, so the camera feed is not ' +
                  'available. Install the QuadRF certificate from the ' +
                  'setup page, then reopen PhaseGaze over HTTPS.';
        } else if (name === 'NotAllowedError' || name === 'SecurityError') {
            title = 'CAMERA DENIED';
            msg = `Allow camera for ${location.host} in this browser's site ` +
                  'settings, then tap SPHERE and CAM.';
        } else if (name === 'NotFoundError') {
            title = 'NO CAMERA';
            msg = 'No usable camera on this device.';
        } else if (name === 'NotReadableError') {
            title = 'CAMERA BUSY';
            msg = 'Another app holds the camera. Close it, tap SPHERE, then CAM.';
        }

        $('cam-note-title').textContent = title;
        $('cam-note-msg').textContent = msg;
        $('cam-note-link').style.display = insecure ? '' : 'none';
        $('cam-note').classList.add('show');
    }
}
