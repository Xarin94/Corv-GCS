/**
 * CRVLogger.js - Binary telemetry recorder (.CRV format)
 * Records STATE telemetry at 10 Hz into a compact binary log file.
 * Auto-starts on connection, auto-stops on disconnect.
 * Logs are saved to the default logs directory (userData/logs/).
 * See CRV_FORMAT.md for the full specification.
 */

import { STATE } from '../core/state.js';
import { calculateCRC16 } from '../core/utils.js';

// Packet type IDs
const PKT_FILE_HEADER = 0x10;
const PKT_NAVIGATION  = 0x11;
const PKT_SYS_STATUS  = 0x12;
const PKT_EVENT       = 0x13;

// Sync bytes
const SYNC_0 = 0xA5;
const SYNC_1 = 0x5A;

// Recording interval (ms)
const SAMPLE_INTERVAL = 100; // 10 Hz

// Flush threshold (bytes)
const FLUSH_THRESHOLD = 8192;

// Connection type encoding
const CONNECTION_TYPE_MAP = {
    'none': 0,
    'corv-binary': 1,
    'mavlink-serial': 2,
    'mavlink-udp': 3,
    'mavlink-tcp': 4
};

// Event types
const EVT_ARM_CHANGE  = 0;
const EVT_MODE_CHANGE = 1;
const EVT_STATUS_TEXT = 2;

export class CRVLogger {
    constructor() {
        this.state = 'idle'; // 'idle' | 'recording'
        this.startTimeMs = 0;
        this.sequenceCounter = 0;
        this.intervalId = null;
        this.navCount = 0; // counts nav packets for 1 Hz status timing
        this.filePath = null;

        // Event detection state
        this.lastArmed = false;
        this.lastFlightModeNum = -1;
        this.lastStatusText = '';

        // Write buffer
        this.chunkBuffer = [];
        this.chunkSize = 0;

        // Auto-start/stop: listen for connection state changes
        this._initConnectionListener();
    }

    /**
     * Listen for mavlinkConnectionState events to auto-start/stop recording
     */
    _initConnectionListener() {
        window.addEventListener('mavlinkConnectionState', (e) => {
            const connState = e.detail && e.detail.state;
            if (connState === 'CONNECTED' && this.state === 'idle') {
                // Small delay to let connectionType be set in STATE
                setTimeout(() => this.startRecording(), 500);
            } else if (connState === 'DISCONNECTED' && this.state === 'recording') {
                this.stopRecording();
            }
        });
    }

    /**
     * Start recording to a new .crv file (auto-named in logs directory)
     */
    async startRecording() {
        if (this.state === 'recording') return false;

        const result = await window.crvLogger.startRecording();
        if (!result.success) return false;

        this.filePath = result.filePath;
        this.startTimeMs = Date.now();
        this.sequenceCounter = 0;
        this.navCount = 0;
        this.chunkBuffer = [];
        this.chunkSize = 0;

        // Snapshot event state
        this.lastArmed = STATE.armed;
        this.lastFlightModeNum = STATE.flightModeNum;
        this.lastStatusText = STATE.statusText;

        // Write file header packet
        this._writeFileHeader();

        // Write initial system status
        this._writeSysStatus();

        // Start 10 Hz sampling
        this.state = 'recording';
        this.intervalId = setInterval(() => this._sample(), SAMPLE_INTERVAL);

        this._updateUI(true);
        console.log(`[CRV] recording started → ${this.filePath}`);
        return true;
    }

    /**
     * Stop recording and close the file
     */
    async stopRecording() {
        if (this.state !== 'recording') return;

        clearInterval(this.intervalId);
        this.intervalId = null;
        this.state = 'idle';

        // Flush remaining data
        await this._flush();
        await window.crvLogger.stopRecording();

        this._updateUI(false);
        console.log(`[CRV] recording stopped`);
    }

    /**
     * Toggle recording on/off (for manual REC button)
     */
    async toggleRecording() {
        if (this.state === 'idle') {
            await this.startRecording();
        } else {
            await this.stopRecording();
        }
    }

    // ── Internal ────────────────────────────────────────────────────────

    /**
     * Called at 10 Hz — write nav packet + check for events/status
     */
    _sample() {
        this._writeNavigation();
        this.navCount++;

        // System status at 1 Hz (every 10th nav packet)
        if (this.navCount % 10 === 0) {
            this._writeSysStatus();
        }

        // Event detection
        this._checkEvents();

        // Flush if buffer is large enough
        if (this.chunkSize >= FLUSH_THRESHOLD) {
            this._flush();
        }
    }

    /**
     * Elapsed ms since recording start
     */
    _elapsed() {
        return Date.now() - this.startTimeMs;
    }

    // ── Packet builders ─────────────────────────────────────────────────

    /**
     * Build a complete packet: sync + type + length + seq + payload + CRC
     */
    _buildPacket(type, payloadBytes) {
        const payloadLen = payloadBytes.length;
        const total = 6 + payloadLen + 2; // 2 sync + 1 type + 2 len + 1 seq + payload + 2 CRC
        const pkt = new Uint8Array(total);

        // Sync
        pkt[0] = SYNC_0;
        pkt[1] = SYNC_1;

        // Type
        pkt[2] = type;

        // Payload length (little-endian uint16)
        pkt[3] = payloadLen & 0xFF;
        pkt[4] = (payloadLen >> 8) & 0xFF;

        // Sequence
        pkt[5] = this.sequenceCounter & 0xFF;
        this.sequenceCounter = (this.sequenceCounter + 1) & 0xFF;

        // Payload
        pkt.set(payloadBytes, 6);

        // CRC-16 over [Type .. end of payload] = bytes [2 .. 6+payloadLen-1]
        const crcStart = 2;
        const crcLen = 4 + payloadLen; // type(1) + len(2) + seq(1) + payload
        const crc = calculateCRC16(pkt, crcStart, crcLen);
        const crcOffset = 6 + payloadLen;
        pkt[crcOffset] = crc & 0xFF;
        pkt[crcOffset + 1] = (crc >> 8) & 0xFF;

        return pkt;
    }

    /**
     * Append a packet to the write buffer
     */
    _enqueue(packet) {
        this.chunkBuffer.push(packet);
        this.chunkSize += packet.length;
    }

    /**
     * Flush buffered packets to main process
     */
    async _flush() {
        if (this.chunkBuffer.length === 0) return;

        // Concatenate all buffered packets
        const combined = new Uint8Array(this.chunkSize);
        let offset = 0;
        for (const chunk of this.chunkBuffer) {
            combined.set(chunk, offset);
            offset += chunk.length;
        }

        this.chunkBuffer = [];
        this.chunkSize = 0;

        await window.crvLogger.writeChunk(combined.buffer);
    }

    // ── Packet type 0x10: File Header ───────────────────────────────────

    _writeFileHeader() {
        const payload = new Uint8Array(32);
        const dv = new DataView(payload.buffer);

        // Magic "CRV\0"
        payload[0] = 0x43; // C
        payload[1] = 0x52; // R
        payload[2] = 0x56; // V
        payload[3] = 0x00;

        // Format version
        payload[4] = 1;

        // Start timestamp (uint64 as two uint32)
        const ts = this.startTimeMs;
        dv.setUint32(6, ts & 0xFFFFFFFF, true);
        dv.setUint32(10, Math.floor(ts / 0x100000000), true);

        // Sample rate
        payload[14] = 10;

        // Connection type
        payload[15] = CONNECTION_TYPE_MAP[STATE.connectionType] || 0;

        // Vehicle/autopilot
        payload[16] = STATE.vehicleType || 0;
        payload[17] = STATE.autopilotType || 0;
        payload[18] = STATE.systemId || 1;
        payload[19] = STATE.componentId || 1;

        // reserved2[12] already zero

        this._enqueue(this._buildPacket(PKT_FILE_HEADER, payload));
    }

    // ── Packet type 0x11: Navigation ────────────────────────────────────

    _writeNavigation() {
        const payload = new Uint8Array(80);
        const dv = new DataView(payload.buffer);

        let o = 0;

        // timestampMs
        dv.setUint32(o, this._elapsed(), true); o += 4;

        // roll, pitch, yaw (int16, x1000 = milliradians)
        dv.setInt16(o, Math.round(STATE.roll * 1000), true);  o += 2;
        dv.setInt16(o, Math.round(STATE.pitch * 1000), true); o += 2;
        dv.setInt16(o, Math.round(STATE.yaw * 1000), true);   o += 2;

        // lat, lon (float64)
        dv.setFloat64(o, STATE.lat, true);  o += 8;
        dv.setFloat64(o, STATE.lon, true);  o += 8;

        // rawAlt, offsetAlt (float32)
        dv.setFloat32(o, STATE.rawAlt, true);    o += 4;
        dv.setFloat32(o, STATE.offsetAlt, true); o += 4;

        // airspeed, groundspeed, vs (float32)
        dv.setFloat32(o, STATE.as, true); o += 4;
        dv.setFloat32(o, STATE.gs, true); o += 4;
        dv.setFloat32(o, STATE.vs, true); o += 4;

        // vn, ve, vd (float32)
        dv.setFloat32(o, STATE.vn, true); o += 4;
        dv.setFloat32(o, STATE.ve, true); o += 4;
        dv.setFloat32(o, STATE.vd, true); o += 4;

        // ax, ay, az (int16, x100 = cm/s^2)
        dv.setInt16(o, Math.round(STATE.ax * 100), true); o += 2;
        dv.setInt16(o, Math.round(STATE.ay * 100), true); o += 2;
        dv.setInt16(o, Math.round(STATE.az * 100), true); o += 2;

        // aoa, ssa, gamma, track (int16, x1000 = milliradians)
        dv.setInt16(o, Math.round(STATE.aoa * 1000), true);   o += 2;
        dv.setInt16(o, Math.round(STATE.ssa * 1000), true);   o += 2;
        dv.setInt16(o, Math.round(STATE.gamma * 1000), true); o += 2;
        dv.setInt16(o, Math.round(STATE.track * 1000), true); o += 2;

        // terrainHeight, rangefinderDist (float32, NaN if null)
        dv.setFloat32(o, STATE.terrainHeight != null ? STATE.terrainHeight : NaN, true); o += 4;
        dv.setFloat32(o, STATE.rangefinderDist != null ? STATE.rangefinderDist : NaN, true);

        this._enqueue(this._buildPacket(PKT_NAVIGATION, payload));
    }

    // ── Packet type 0x12: System Status ─────────────────────────────────

    _writeSysStatus() {
        const payload = new Uint8Array(52);
        const dv = new DataView(payload.buffer);

        let o = 0;

        // timestampMs
        dv.setUint32(o, this._elapsed(), true); o += 4;

        // battery
        dv.setUint16(o, Math.round(STATE.batteryVoltage * 100), true); o += 2;
        dv.setInt16(o, Math.round(STATE.batteryCurrent * 100), true);  o += 2;
        dv.setInt8(o, STATE.batteryRemaining); o += 1;

        // GPS
        dv.setUint8(o, STATE.gpsFix);    o += 1;
        dv.setUint8(o, STATE.gpsNumSat); o += 1;
        dv.setUint16(o, Math.round(STATE.gpsHdop * 100), true); o += 2;

        // armed, mode, baseMode, linkQuality
        dv.setUint8(o, STATE.armed ? 1 : 0);  o += 1;
        dv.setUint8(o, STATE.flightModeNum);   o += 1;
        dv.setUint8(o, STATE.baseMode);        o += 1;
        dv.setUint8(o, STATE.linkQuality);     o += 1;

        // RTK
        dv.setUint16(o, STATE.rtkIar, true);      o += 2;
        dv.setUint32(o, STATE.rtkBaseline, true);  o += 4;
        dv.setUint16(o, STATE.rtkAccuracy, true);  o += 2;

        // Vibration
        dv.setFloat32(o, STATE.vibX, true); o += 4;
        dv.setFloat32(o, STATE.vibY, true); o += 4;
        dv.setFloat32(o, STATE.vibZ, true); o += 4;

        // Home position
        dv.setFloat32(o, STATE.homeLat != null ? STATE.homeLat : NaN, true); o += 4;
        dv.setFloat32(o, STATE.homeLon != null ? STATE.homeLon : NaN, true); o += 4;
        dv.setFloat32(o, STATE.homeAlt != null ? STATE.homeAlt : NaN, true); o += 4;

        // reserved[3] already zero

        this._enqueue(this._buildPacket(PKT_SYS_STATUS, payload));
    }

    // ── Packet type 0x13: Events ────────────────────────────────────────

    _checkEvents() {
        // Arm/disarm change
        if (STATE.armed !== this.lastArmed) {
            this.lastArmed = STATE.armed;
            this._writeEvent(EVT_ARM_CHANGE, STATE.armed ? 1 : 0, 0, '');
        }

        // Flight mode change
        if (STATE.flightModeNum !== this.lastFlightModeNum) {
            this.lastFlightModeNum = STATE.flightModeNum;
            this._writeEvent(EVT_MODE_CHANGE, STATE.flightModeNum, 0, STATE.flightMode || '');
        }

        // Status text change
        if (STATE.statusText && STATE.statusText !== this.lastStatusText) {
            this.lastStatusText = STATE.statusText;
            this._writeEvent(EVT_STATUS_TEXT, 0, STATE.statusSeverity || 0, STATE.statusText);
        }
    }

    _writeEvent(eventType, eventData, severity, text) {
        // Truncate text to 50 chars
        const textStr = (text || '').substring(0, 50);
        const textBytes = new TextEncoder().encode(textStr);
        const textLen = textBytes.length;

        const payload = new Uint8Array(8 + textLen);
        const dv = new DataView(payload.buffer);

        dv.setUint32(0, this._elapsed(), true);
        payload[4] = eventType;
        payload[5] = eventData;
        payload[6] = severity;
        payload[7] = textLen;
        if (textLen > 0) {
            payload.set(textBytes, 8);
        }

        this._enqueue(this._buildPacket(PKT_EVENT, payload));
    }

    // ── UI ──────────────────────────────────────────────────────────────

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
