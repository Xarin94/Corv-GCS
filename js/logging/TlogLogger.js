/**
 * TlogLogger.js - TLOG flight recording controller (renderer side)
 * Controls start/stop of TLOG recording via IPC to the main process.
 * The actual packet capture and file writing happens in main-mavlink.js.
 * Auto-starts on MAVLink connection, auto-stops on disconnect.
 */

export class TlogLogger {
    constructor() {
        this.state = 'idle'; // 'idle' | 'recording'
        this.filePath = null;

        this._initConnectionListener();
    }

    _initConnectionListener() {
        window.addEventListener('mavlinkConnectionState', (e) => {
            const connState = e.detail && e.detail.state;
            if (connState === 'CONNECTED' && this.state === 'idle') {
                setTimeout(() => this.startRecording(), 500);
            } else if (connState === 'DISCONNECTED' && this.state === 'recording') {
                this.stopRecording();
            }
        });
    }

    async startRecording() {
        if (this.state === 'recording') return false;

        const result = await window.tlogLogger.startRecording();
        if (!result.success) return false;

        this.filePath = result.filePath;
        this.state = 'recording';
        this._updateUI(true);
        console.log(`[tlog] recording started → ${this.filePath}`);
        return true;
    }

    async stopRecording() {
        if (this.state !== 'recording') return;

        this.state = 'idle';
        await window.tlogLogger.stopRecording();
        this._updateUI(false);
        console.log('[tlog] recording stopped');
    }

    async toggleRecording() {
        if (this.state === 'idle') {
            await this.startRecording();
        } else {
            await this.stopRecording();
        }
    }

    _updateUI(recording) {
        const btn = document.getElementById('btn-rec');
        if (!btn) return;

        if (recording) {
            btn.textContent = 'STOP REC';
            btn.classList.add('recording');
        } else {
            btn.textContent = 'REC';
            btn.classList.remove('recording');
        }
    }
}
