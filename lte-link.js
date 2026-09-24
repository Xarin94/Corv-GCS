/**
 * lte-link.js - Cellular link to an LTEtelem module through a CRV2 relay
 *
 * The module (Teensy + cellular modem on the vehicle) and this GCS both dial
 * out to a relay server; the relay pairs them by module ID. Everything after
 * the handshake is end-to-end encrypted between module and GCS, so the relay
 * forwards records it can neither read nor modify.
 *
 * Handshake (TLS to the relay's GCS port):
 *   relay -> "CRV2-GCS <challenge hex>\n"
 *   GCS   -> "<module id> <hex HMAC-SHA256(authKey, 'CRV2-GCS-AUTH ' || challenge || id)>\n"
 *   relay -> "OK <module id> air=online|offline\n"  or  "ERR auth\n"
 * Records (both directions):
 *   C7 | type | length (u16 LE) | body
 *   DATA (0x01):      session id (8) | counter (4, BE) | ciphertext | tag (16)
 *   KEEPALIVE (0x02): empty, relay -> module only
 * AES-256-GCM, nonce = session id || counter. From the 256-bit module key:
 *   up   = HMAC-SHA256(key, "CRV2 air->gcs")  telemetry, module -> GCS
 *   down = HMAC-SHA256(key, "CRV2 gcs->air")  commands, GCS -> module
 *   auth = HMAC-SHA256(key, "CRV2 gcs-auth")  the only value the relay holds
 * AAD is the record header; GCS -> module records also bind the module's
 * current session id, so a command captured in an earlier session is refused.
 * Replay: per session id the counter must strictly increase.
 *
 * Reference implementation and tests: LTEtelem tools/relay/e2e.py.
 */

const tls = require('tls');
const net = require('net');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const { EventEmitter } = require('events');
const { ipcMain, safeStorage, app } = require('electron');

const MAGIC = 0xC7;
const T_DATA = 0x01;
const T_KEEPALIVE = 0x02;
const HDR = 4;
const SID = 8;
const TAG = 16;
const MAX_BODY = 1400;
const MAX_PLAIN = MAX_BODY - SID - 4 - TAG;
// MAVLink frames are packed into one record: closed at BATCH_BYTES or
// BATCH_MS after the first frame, whichever comes first
const BATCH_BYTES = 800;
const BATCH_MS = 20;
const HANDSHAKE_TIMEOUT_MS = 20000;
const RECONNECT_MS = [1000, 2000, 5000, 10000];

const hmac = (key, data) => crypto.createHmac('sha256', key).update(data).digest();

function deriveKeys(key) {
    return {
        up: hmac(key, 'CRV2 air->gcs'),
        down: hmac(key, 'CRV2 gcs->air'),
        auth: hmac(key, 'CRV2 gcs-auth')
    };
}

function header(type, bodyLen) {
    const h = Buffer.alloc(HDR);
    h[0] = MAGIC;
    h[1] = type;
    h.writeUInt16LE(bodyLen, 2);
    return h;
}

/** Normalises "AA:BB:.." / "aabb.." to lowercase hex without separators. */
function normFingerprint(fp) {
    return String(fp || '').replace(/[^0-9a-fA-F]/g, '').toLowerCase();
}

/** Splits the byte stream into whole records without looking inside them. */
class RecordFramer {
    constructor() { this.buf = Buffer.alloc(0); this.garbage = 0; }

    feed(data) {
        this.buf = this.buf.length ? Buffer.concat([this.buf, data]) : data;
        const out = [];
        while (this.buf.length) {
            if (this.buf[0] !== MAGIC) {
                let j = this.buf.indexOf(MAGIC);
                if (j < 0) j = this.buf.length;
                this.garbage += j;
                this.buf = this.buf.subarray(j);
                continue;
            }
            if (this.buf.length < HDR) break;
            const type = this.buf[1];
            const len = this.buf.readUInt16LE(2);
            if ((type !== T_DATA && type !== T_KEEPALIVE) || len > MAX_BODY) {
                this.garbage += 1;
                this.buf = this.buf.subarray(1);
                continue;
            }
            if (this.buf.length < HDR + len) break;
            out.push(Buffer.from(this.buf.subarray(0, HDR + len)));
            this.buf = this.buf.subarray(HDR + len);
        }
        return out;
    }
}

class LteLink extends EventEmitter {
    /**
     * @param {object} opts
     * @param {string} opts.host        relay host
     * @param {number} opts.port        relay GCS port
     * @param {string} opts.moduleId    module (vehicle) ID registered on the relay
     * @param {string} opts.keyHex      256-bit module key, 64 hex chars
     * @param {string} [opts.certSha256] relay certificate SHA-256 to pin (empty = not verified)
     */
    constructor(opts) {
        super();
        const key = Buffer.from(String(opts.keyHex || '').trim(), 'hex');
        if (key.length !== 32) throw new Error('AES key must be 64 hex characters (256 bit)');
        if (!opts.moduleId || /\s/.test(opts.moduleId)) throw new Error('Invalid module ID');
        if (!opts.host) throw new Error('Relay host missing');
        this.opts = { ...opts, port: Number(opts.port) || 5765, pin: normFingerprint(opts.certSha256) };
        this.keys = deriveKeys(key);
        key.fill(0);
        this.socket = null;
        this.stopped = true;
        this.backoff = 0;
        this.reconnectTimer = null;
        this.statsTimer = null;
        this.batch = [];
        this.batchLen = 0;
        this.batchTimer = null;
        this.status = {
            state: 'idle', moduleId: opts.moduleId, relay: `${opts.host}:${this.opts.port}`,
            airOnline: false, vehicleSession: null, tls: null, lastError: null,
            bytesRx: 0, bytesTx: 0, recordsIn: 0, recordsOut: 0,
            authFail: 0, replay: 0, droppedNoSession: 0, reconnects: 0, lastRxAge: null
        };
    }

    // ── lifecycle ──────────────────────────────────────────────────────────

    /** Resolves on the first successful handshake; after that the link reconnects by itself. */
    start() {
        this.stopped = false;
        this.statsTimer = setInterval(() => this._emitStatus(), 1000);
        return new Promise((resolve, reject) => {
            this._firstResult = { resolve, reject };
            this._connect();
        });
    }

    stop() {
        this.stopped = true;
        clearTimeout(this.reconnectTimer);
        clearTimeout(this.batchTimer);
        clearInterval(this.statsTimer);
        this.batch = [];
        this.batchLen = 0;
        if (this.socket) this.socket.destroy();
        this.socket = null;
        this._setState('stopped');
    }

    _connect() {
        this._resetSession();
        this._setState('connecting');
        const { host, port } = this.opts;
        const socket = tls.connect({
            host, port,
            servername: net.isIP(host) ? undefined : host,
            // The relay certificate is pinned by fingerprint below (it is usually
            // self-signed). Without a pin the TLS session is still encrypted but
            // the relay is not authenticated; the E2E layer protects the content.
            rejectUnauthorized: false,
            minVersion: 'TLSv1.2'
        });
        this.socket = socket;
        socket.setNoDelay(true);
        socket.setKeepAlive(true, 10000);

        let handshakeDone = false;
        let lineBuf = Buffer.alloc(0);
        let stage = 'banner';
        const hsTimer = setTimeout(() => this._fail(socket, 'Relay handshake timeout'), HANDSHAKE_TIMEOUT_MS);

        socket.once('secureConnect', () => {
            const cert = socket.getPeerCertificate();
            const fp = normFingerprint(cert && cert.fingerprint256);
            this.status.tls = {
                version: socket.getProtocol(),
                cipher: socket.getCipher()?.name,
                fingerprint: cert && cert.fingerprint256,
                verified: !!this.opts.pin && fp === this.opts.pin
            };
            if (this.opts.pin && fp !== this.opts.pin) {
                clearTimeout(hsTimer);
                this._fail(socket, 'Relay certificate does not match the pinned fingerprint', true);
                return;
            }
            this._setState('handshake');
        });

        socket.on('data', (data) => {
            this.status.bytesRx += data.length;
            this.lastRx = Date.now();
            if (handshakeDone) { this._onRecords(data); return; }

            lineBuf = Buffer.concat([lineBuf, data]);
            let nl;
            while (!handshakeDone && (nl = lineBuf.indexOf(0x0a)) >= 0) {
                const line = lineBuf.subarray(0, nl).toString('ascii').trim();
                lineBuf = lineBuf.subarray(nl + 1);
                if (stage === 'banner') {
                    const m = /^CRV2-GCS ([0-9a-f]{32})$/i.exec(line);
                    if (!m) { clearTimeout(hsTimer); this._fail(socket, `Unexpected relay banner: ${line}`, true); return; }
                    const challenge = Buffer.from(m[1], 'hex');
                    const proof = hmac(this.keys.auth, Buffer.concat([
                        Buffer.from('CRV2-GCS-AUTH '), challenge, Buffer.from(this.opts.moduleId)
                    ])).toString('hex');
                    this._write(Buffer.from(`${this.opts.moduleId} ${proof}\n`));
                    stage = 'reply';
                } else {
                    clearTimeout(hsTimer);
                    if (!line.startsWith('OK')) {
                        // wrong module ID or key: retrying cannot help
                        this._fail(socket, line.startsWith('ERR') ? 'Relay refused: wrong module ID or AES key' : `Relay: ${line}`, true);
                        return;
                    }
                    handshakeDone = true;
                    this.status.airOnline = /air=online/.test(line);
                    this.backoff = 0;
                    this.status.lastError = null;
                    this._setState('online');
                    if (this._firstResult) { this._firstResult.resolve(); this._firstResult = null; }
                    if (lineBuf.length) this._onRecords(lineBuf);
                }
            }
        });

        socket.on('error', (err) => this._fail(socket, err.message));
        socket.on('close', () => {
            clearTimeout(hsTimer);
            if (this.socket === socket) this._scheduleReconnect();
        });
    }

    _fail(socket, message, fatal = false) {
        this.status.lastError = message;
        if (fatal) this._fatal = true;
        if (this._firstResult) {           // first attempt: report to the caller, no retry loop
            this._firstResult.reject(new Error(message));
            this._firstResult = null;
            this._fatal = true;
        }
        socket.destroy();
    }

    _scheduleReconnect() {
        this.socket = null;
        if (this.stopped) return;
        if (this._fatal) { this._fatal = false; this.stopped = true; clearInterval(this.statsTimer); this._setState('error'); return; }
        const delay = RECONNECT_MS[Math.min(this.backoff++, RECONNECT_MS.length - 1)];
        this.status.reconnects++;
        this._setState('reconnecting');
        this.reconnectTimer = setTimeout(() => this._connect(), delay);
    }

    _resetSession() {
        this.framer = new RecordFramer();
        this.mySid = crypto.randomBytes(SID);   // new session id: the nonce never repeats
        this.myCtr = 0;
        this.seen = new Map();                  // module session id (hex) -> last counter
        this.airSid = null;
    }

    // ── receive ────────────────────────────────────────────────────────────

    _onRecords(data) {
        for (const rec of this.framer.feed(data)) {
            if (rec[1] !== T_DATA) continue;    // keepalives only go to the module
            const plain = this._open(rec);
            if (plain) this.emit('data', plain);
        }
    }

    _open(rec) {
        if (rec.length < HDR + SID + 4 + TAG) { this.status.authFail++; return null; }
        const hdr = rec.subarray(0, HDR);
        const nonce = rec.subarray(HDR, HDR + SID + 4);
        const ct = rec.subarray(HDR + SID + 4, rec.length - TAG);
        const tag = rec.subarray(rec.length - TAG);
        let plain;
        try {
            const d = crypto.createDecipheriv('aes-256-gcm', this.keys.up, nonce);
            d.setAAD(hdr);
            d.setAuthTag(tag);
            plain = Buffer.concat([d.update(ct), d.final()]);
        } catch (e) {
            this.status.authFail++;
            return null;
        }
        const sid = nonce.subarray(0, SID);
        const sidHex = sid.toString('hex');
        const ctr = nonce.readUInt32BE(SID);
        if (ctr <= (this.seen.get(sidHex) || 0)) { this.status.replay++; return null; }
        this.seen.set(sidHex, ctr);
        this.airSid = Buffer.from(sid);         // commands are bound to this session
        this.status.vehicleSession = sidHex;
        this.status.airOnline = true;
        this.status.recordsIn++;
        return plain;
    }

    // ── send ───────────────────────────────────────────────────────────────

    /** Queue MAVLink bytes for the module; they go out in the next encrypted record. */
    write(buf) {
        if (this.stopped || !this.socket) return;
        this.batch.push(Buffer.from(buf));
        this.batchLen += buf.length;
        if (this.batchLen >= BATCH_BYTES) this._flush();
        else if (!this.batchTimer) this.batchTimer = setTimeout(() => this._flush(), BATCH_MS);
    }

    _flush() {
        clearTimeout(this.batchTimer);
        this.batchTimer = null;
        if (!this.batchLen) return;
        const plain = Buffer.concat(this.batch);
        this.batch = [];
        this.batchLen = 0;
        if (!this.airSid) {                     // no telemetry yet: module session unknown
            this.status.droppedNoSession++;
            return;
        }
        for (let off = 0; off < plain.length; off += MAX_PLAIN) {
            this._write(this._seal(plain.subarray(off, off + MAX_PLAIN)));
        }
    }

    _seal(plain) {
        this.myCtr++;
        const nonce = Buffer.alloc(SID + 4);
        this.mySid.copy(nonce, 0);
        nonce.writeUInt32BE(this.myCtr, SID);
        const hdr = header(T_DATA, SID + 4 + plain.length + TAG);
        const c = crypto.createCipheriv('aes-256-gcm', this.keys.down, nonce);
        c.setAAD(Buffer.concat([hdr, this.airSid]));
        const ct = Buffer.concat([c.update(plain), c.final()]);
        this.status.recordsOut++;
        return Buffer.concat([hdr, nonce, ct, c.getAuthTag()]);
    }

    _write(buf) {
        if (!this.socket) return;
        this.status.bytesTx += buf.length;
        this.socket.write(buf);
    }

    // ── status ─────────────────────────────────────────────────────────────

    _setState(state) {
        this.status.state = state;
        this._emitStatus();
    }

    _emitStatus() {
        this.status.lastRxAge = this.lastRx ? Date.now() - this.lastRx : null;
        this.emit('status', { ...this.status });
    }
}

// ── remembered key (encrypted with the OS keychain via safeStorage) ─────────

function keyFile() {
    return path.join(app.getPath('userData'), 'lte-link-key.bin');
}

function initLteKeyHandlers() {
    ipcMain.handle('lte-key-available', () => safeStorage.isEncryptionAvailable());
    ipcMain.handle('lte-key-save', (event, keyHex) => {
        if (!safeStorage.isEncryptionAvailable()) return false;
        fs.writeFileSync(keyFile(), safeStorage.encryptString(String(keyHex)), { mode: 0o600 });
        return true;
    });
    ipcMain.handle('lte-key-load', () => {
        try {
            if (!safeStorage.isEncryptionAvailable() || !fs.existsSync(keyFile())) return '';
            return safeStorage.decryptString(fs.readFileSync(keyFile()));
        } catch (e) {
            return '';
        }
    });
    ipcMain.handle('lte-key-forget', () => {
        try { fs.unlinkSync(keyFile()); } catch (e) { /* not saved */ }
        return true;
    });
}

module.exports = { LteLink, RecordFramer, deriveKeys, initLteKeyHandlers };
