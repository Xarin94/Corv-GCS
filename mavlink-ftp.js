/**
 * mavlink-ftp.js - MAVLink FTP client (main process) and the ArduPilot packed
 * parameter file decoder.
 *
 * ArduPilot serves its parameters as one file, "@PARAM/param.pck", over the
 * MAVLink file transfer protocol (FILE_TRANSFER_PROTOCOL, msg 110). Reading it
 * is a few dozen packets of 239 bytes instead of one PARAM_VALUE per parameter
 * (1000-1500 of them, 25 bytes each, each a separate message), so a full
 * parameter download takes seconds on a telemetry radio instead of minutes.
 * The protocol and the file format follow the firmware: GCS_MAVLink/GCS_FTP.cpp
 * and AP_Filesystem/AP_Filesystem_Param.cpp.
 *
 * The client here is deliberately small: open, burst read, fill the gaps with
 * single reads, terminate. It is used only for the parameter file; the renderer
 * keeps PARAM_REQUEST_LIST and the single PARAM_REQUEST_READs as the fallback
 * for a firmware that does not answer FTP.
 *
 * FTP payload (251 bytes inside the MAVLink message):
 *   0  u16 seq_number        3  u8 opcode     5  u8 req_opcode   8  u32 offset
 *   2  u8  session           4  u8 size       6  u8 burst_complete 12 data[239]
 */

'use strict';

const FTP_PAYLOAD_LEN = 251;
const FTP_HEADER_LEN = 12;
const FTP_DATA_LEN = FTP_PAYLOAD_LEN - FTP_HEADER_LEN;   // 239

const FTP_OP = Object.freeze({
    None: 0, TerminateSession: 1, ResetSessions: 2, ListDirectory: 3,
    OpenFileRO: 4, ReadFile: 5, CreateFile: 6, WriteFile: 7, RemoveFile: 8,
    CreateDirectory: 9, RemoveDirectory: 10, OpenFileWO: 11, TruncateFile: 12,
    Rename: 13, CalcFileCRC32: 14, BurstReadFile: 15,
    Ack: 128, Nak: 129
});

const FTP_ERROR = Object.freeze({
    None: 0, Fail: 1, FailErrno: 2, InvalidDataSize: 3, InvalidSession: 4,
    NoSessionsAvailable: 5, EndOfFile: 6, UnknownCommand: 7, FileExists: 8,
    FileProtected: 9, FileNotFound: 10
});

const ERROR_NAMES = Object.fromEntries(Object.entries(FTP_ERROR).map(([k, v]) => [v, k]));

/** Build one FTP payload (a 251-byte Buffer). `data` is a Buffer or omitted. */
function buildPayload({ seq, session, opcode, size, reqOpcode = 0, burstComplete = 0, offset = 0, data = null }) {
    const p = Buffer.alloc(FTP_PAYLOAD_LEN);
    p.writeUInt16LE(seq & 0xFFFF, 0);
    p[2] = session & 0xFF;
    p[3] = opcode & 0xFF;
    p[5] = reqOpcode & 0xFF;
    p[6] = burstComplete ? 1 : 0;
    p.writeUInt32LE(offset >>> 0, 8);
    let n = 0;
    if (data && data.length) {
        n = Math.min(data.length, FTP_DATA_LEN);
        Buffer.from(data).copy(p, FTP_HEADER_LEN, 0, n);
    }
    p[4] = size !== undefined ? (size & 0xFF) : n;
    return p;
}

/** Parse an FTP payload (Buffer or array of numbers) into its fields. */
function parsePayload(raw) {
    const p = Buffer.isBuffer(raw) ? raw : Buffer.from(raw || []);
    if (p.length < FTP_HEADER_LEN) return null;
    const size = Math.min(p[4], FTP_DATA_LEN, Math.max(0, p.length - FTP_HEADER_LEN));
    return {
        seq: p.readUInt16LE(0),
        session: p[2],
        opcode: p[3],
        size: p[4],
        reqOpcode: p[5],
        burstComplete: p[6] !== 0,
        offset: p.readUInt32LE(8),
        data: p.subarray(FTP_HEADER_LEN, FTP_HEADER_LEN + size)
    };
}

class FtpError extends Error {
    constructor(message, code) {
        super(message);
        this.name = 'FtpError';
        this.code = code;   // 'NOREPLY' | 'NAK' | 'CANCELLED' | 'BUSY' | 'TIMEOUT'
    }
}

/**
 * One FTP client per link. `send(payloadBuffer, targetSystem, targetComponent)`
 * must put a FILE_TRANSFER_PROTOCOL message on the wire; `onPacket()` must be
 * called with the payload of every FILE_TRANSFER_PROTOCOL received.
 */
class MavFtpClient {
    constructor({ send, log = () => {} }) {
        this._send = send;
        this._log = log;
        this._seq = Math.floor(Math.random() * 0xFFFF);
        this._session = 0;
        this._active = null;      // the transfer in progress, if any
    }

    get busy() { return !!this._active; }

    /** Feed an incoming FILE_TRANSFER_PROTOCOL payload (array or Buffer). */
    onPacket(payload) {
        if (!this._active) return;
        const op = parsePayload(payload);
        if (!op) return;
        this._active.handle(op);
    }

    /** Abort the transfer in progress; its promise rejects with code CANCELLED. */
    cancel(reason = 'cancelled') {
        if (this._active) this._active.abort(new FtpError(reason, 'CANCELLED'));
    }

    /**
     * Read a whole file from the vehicle.
     * @param {string} path - e.g. "@PARAM/param.pck"
     * @param {object} [opts]
     * @param {number} [opts.targetSystem=1]
     * @param {number} [opts.targetComponent=1]
     * @param {number} [opts.timeoutMs=3000] - silence before a request is repeated
     * @param {number} [opts.retries=4] - repeats of one request before giving up
     * @param {function} [opts.onProgress] - ({bytes, sizeHint, packets}) as data lands
     * @returns {Promise<Buffer>}
     */
    async readFile(path, opts = {}) {
        if (this._active) throw new FtpError('an FTP transfer is already running', 'BUSY');
        const transfer = new ReadTransfer(this, path, opts);
        this._active = transfer;
        try {
            return await transfer.run();
        } finally {
            if (this._active === transfer) this._active = null;
        }
    }

    _nextSeq() { this._seq = (this._seq + 1) & 0xFFFF; return this._seq; }
    _nextSession() { this._session = (this._session + 1) & 0xFF; return this._session; }
}

/**
 * The state machine of one file read. Requests are serialized: one outstanding
 * request at a time, repeated after `timeoutMs` of silence, `retries` times.
 */
class ReadTransfer {
    constructor(client, path, opts) {
        this.client = client;
        this.path = path;
        this.sys = opts.targetSystem ?? 1;
        this.comp = opts.targetComponent ?? 1;
        this.timeoutMs = opts.timeoutMs ?? 3000;
        this.retries = opts.retries ?? 4;
        this.onProgress = typeof opts.onProgress === 'function' ? opts.onProgress : null;
        this.session = client._nextSession();
        this.chunks = new Map();     // offset -> Buffer
        this.gaps = [];              // [offset, length] still to read
        this.expected = 0;           // next offset the burst should deliver
        this.eof = null;             // file length once known
        this.sizeHint = 0;           // what OpenFileRO said (ArduPilot: an estimate)
        this.bytes = 0;
        this.packets = 0;
        this.pending = null;         // { op, seq, offset, tries, timer }
        this.phase = 'open';         // open | burst | gaps | done
        this.resolveRun = null;
        this.rejectRun = null;
        this.finished = false;
        this.resetDone = false;
    }

    run() {
        return new Promise((resolve, reject) => {
            this.resolveRun = resolve;
            this.rejectRun = reject;
            this.sendOpen();
        });
    }

    abort(err) {
        if (this.finished) return;
        this.finished = true;
        this.clearPending();
        this.client._log(`[ftp] ${this.path}: ${err.message}`);
        this.rejectRun(err);
    }

    finish() {
        if (this.finished) return;
        this.finished = true;
        this.clearPending();
        const out = Buffer.alloc(this.eof);
        for (const [ofs, buf] of this.chunks) {
            if (ofs >= this.eof) continue;
            buf.copy(out, ofs, 0, Math.min(buf.length, this.eof - ofs));
        }
        this.phase = 'done';
        // Close the session; the ACK is not worth waiting for.
        this.sendRaw({ opcode: FTP_OP.TerminateSession });
        this.resolveRun(out);
    }

    // ── sending ─────────────────────────────────────────────────────────

    clearPending() {
        if (this.pending && this.pending.timer) clearTimeout(this.pending.timer);
        this.pending = null;
    }

    sendRaw(fields) {
        const seq = this.client._nextSeq();
        const payload = buildPayload({ seq, session: this.session, ...fields });
        Promise.resolve(this.client._send(payload, this.sys, this.comp))
            .catch(e => this.abort(new FtpError(`send failed: ${e.message}`, 'NOREPLY')));
        return seq;
    }

    /** Send a request that expects a reply; repeat it on silence. */
    request(fields, tries = 0) {
        this.clearPending();
        const seq = this.sendRaw(fields);
        const timer = setTimeout(() => this.onTimeout(), this.timeoutMs);
        this.pending = { opcode: fields.opcode, offset: fields.offset || 0, seq, tries, timer, fields };
    }

    onTimeout() {
        const p = this.pending;
        if (!p || this.finished) return;
        if (p.tries >= this.retries) {
            if (this.phase === 'open') {
                this.abort(new FtpError(`no FTP reply from the vehicle after ${this.retries + 1} attempts`, 'NOREPLY'));
            } else {
                this.abort(new FtpError(`FTP transfer stalled at ${this.bytes} bytes`, 'TIMEOUT'));
            }
            return;
        }
        // A burst that went quiet is asked again from the first byte still
        // missing, so nothing already received is sent twice.
        if (p.opcode === FTP_OP.BurstReadFile) {
            const from = this.gaps.length ? this.gaps[0][0] : this.expected;
            this.request({ opcode: FTP_OP.BurstReadFile, offset: from, size: FTP_DATA_LEN }, p.tries + 1);
        } else {
            this.request(p.fields, p.tries + 1);
        }
    }

    sendOpen() {
        this.phase = 'open';
        const data = Buffer.from(this.path, 'utf8');
        this.request({ opcode: FTP_OP.OpenFileRO, data, size: data.length });
    }

    sendBurst(offset) {
        this.phase = 'burst';
        this.request({ opcode: FTP_OP.BurstReadFile, offset, size: FTP_DATA_LEN });
    }

    sendNextGap() {
        if (!this.gaps.length) {
            if (this.eof !== null) this.finish();
            else this.sendBurst(this.expected);
            return;
        }
        this.phase = 'gaps';
        const [ofs, len] = this.gaps[0];
        this.request({ opcode: FTP_OP.ReadFile, offset: ofs, size: Math.min(len, FTP_DATA_LEN) });
    }

    // ── receiving ───────────────────────────────────────────────────────

    handle(op) {
        if (this.finished) return;
        if (op.session !== this.session) return;              // another client, or an old session
        if (op.opcode !== FTP_OP.Ack && op.opcode !== FTP_OP.Nak) return;
        this.packets++;

        switch (op.reqOpcode) {
            case FTP_OP.OpenFileRO:    return this.onOpenReply(op);
            case FTP_OP.ResetSessions: return this.onResetReply(op);
            case FTP_OP.BurstReadFile: return this.onBurstReply(op);
            case FTP_OP.ReadFile:      return this.onReadReply(op);
            default: return;
        }
    }

    nakError(op) {
        const code = op.data.length ? op.data[0] : FTP_ERROR.Fail;
        const errno = code === FTP_ERROR.FailErrno && op.data.length > 1 ? ` errno ${op.data[1]}` : '';
        return { code, text: `${ERROR_NAMES[code] || 'error ' + code}${errno}` };
    }

    onOpenReply(op) {
        if (this.phase !== 'open') return;
        if (op.opcode === FTP_OP.Nak) {
            const { code, text } = this.nakError(op);
            if (code === FTP_ERROR.NoSessionsAvailable && !this.resetDone) {
                // A session left open by an interrupted transfer: free them all and try once more
                this.resetDone = true;
                this.client._log('[ftp] no session available, resetting the vehicle sessions');
                this.request({ opcode: FTP_OP.ResetSessions });
                return;
            }
            this.abort(new FtpError(`open ${this.path}: ${text}`, 'NAK'));
            return;
        }
        this.sizeHint = op.data.length >= 4 ? op.data.readUInt32LE(0) : 0;
        this.client._log(`[ftp] ${this.path} open, size hint ${this.sizeHint} B`);
        this.sendBurst(0);
    }

    onResetReply() {
        if (this.phase !== 'open') return;
        this.sendOpen();
    }

    storeChunk(op) {
        if (!op.data.length) return;
        const len = op.data.length;
        const end = op.offset + len;
        const known = this.chunks.get(op.offset);
        if (known && known.length >= len) return;           // duplicate
        this.chunks.set(op.offset, Buffer.from(op.data));
        this.bytes += len - (known ? known.length : 0);
        // Take it out of the gaps, whichever gap it falls in
        const gaps = [];
        for (const [gofs, glen] of this.gaps) {
            const gend = gofs + glen;
            if (end <= gofs || op.offset >= gend) { gaps.push([gofs, glen]); continue; }
            if (op.offset > gofs) gaps.push([gofs, op.offset - gofs]);
            if (end < gend) gaps.push([end, gend - end]);
        }
        this.gaps = gaps;
        if (this.onProgress) {
            try { this.onProgress({ bytes: this.bytes, sizeHint: this.sizeHint, packets: this.packets, eof: this.eof }); } catch (e) { /* ignore */ }
        }
    }

    onBurstReply(op) {
        if (this.phase === 'gaps' && op.opcode === FTP_OP.Ack) {
            // A straggler from the burst: still useful if it fills the gap being read
            const want = this.pending ? this.pending.offset : -1;
            this.storeChunk(op);
            if (want >= 0 && !this.gaps.some(g => g[0] === want)) { this.clearPending(); this.sendNextGap(); }
            return;
        }
        if (this.phase !== 'burst') return;
        if (op.opcode === FTP_OP.Nak) {
            const { code, text } = this.nakError(op);
            if (code === FTP_ERROR.EndOfFile) {
                this.eof = this.expected;
                this.clearPending();
                this.sendNextGap();
                return;
            }
            this.abort(new FtpError(`burst read: ${text}`, 'NAK'));
            return;
        }
        // Any ACK keeps the burst alive
        if (this.pending) { clearTimeout(this.pending.timer); this.pending.timer = setTimeout(() => this.onTimeout(), this.timeoutMs); }

        if (op.offset > this.expected) {
            // Packets lost between expected and this one
            this.gaps.push([this.expected, op.offset - this.expected]);
            this.expected = op.offset;
        }
        this.storeChunk(op);
        if (op.offset === this.expected) this.expected = op.offset + op.data.length;

        if (op.burstComplete) {
            if (op.data.length < FTP_DATA_LEN) {
                // A short last packet is the end of the file
                this.eof = Math.max(this.expected, op.offset + op.data.length);
                this.clearPending();
                this.sendNextGap();
            } else {
                // The firmware's burst is over (2000 packets); ask for the rest
                this.sendBurst(this.expected);
            }
        }
    }

    onReadReply(op) {
        if (this.phase !== 'gaps') return;
        if (op.opcode === FTP_OP.Nak) {
            const { code, text } = this.nakError(op);
            if (code === FTP_ERROR.EndOfFile) {
                // The gap lies past the end of the file: nothing to fetch
                if (this.pending) this.gaps = this.gaps.filter(g => g[0] !== this.pending.offset);
                this.clearPending();
                this.sendNextGap();
                return;
            }
            this.abort(new FtpError(`read at ${op.offset}: ${text}`, 'NAK'));
            return;
        }
        this.storeChunk(op);
        this.clearPending();
        this.sendNextGap();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// @PARAM/param.pck
//
//   header: u16 magic (0x671b, 0x671c with defaults), u16 num_params, u16 total_params
//   entry:  u8  type:4 (1 INT8, 2 INT16, 3 INT32, 4 FLOAT) | flags:4 (bit 0: a default follows)
//           u8  common_len:4 (name bytes shared with the previous entry) | name_len-1:4
//           name[name_len], value[type size], [default[type size]]
//   Zero bytes between entries are padding (an entry never straddles a 239-byte packet).
// ═══════════════════════════════════════════════════════════════════════════

const PCK_MAGIC = 0x671b;
const PCK_MAGIC_DEFAULTS = 0x671c;
const PCK_TYPE_SIZE = { 1: 1, 2: 2, 3: 4, 4: 4 };
// AP_Param type -> MAV_PARAM_TYPE, as PARAM_VALUE reports it
const PCK_TO_MAV_TYPE = { 1: 2, 2: 4, 3: 6, 4: 9 };

function readValue(buf, ofs, ptype) {
    switch (ptype) {
        case 1: return buf.readInt8(ofs);
        case 2: return buf.readInt16LE(ofs);
        case 3: return buf.readInt32LE(ofs);
        default: return buf.readFloatLE(ofs);
    }
}

/**
 * Decode a packed parameter file, whole or the part received so far.
 * @param {Buffer|Uint8Array} raw
 * @returns {{ ok:boolean, error?:string, total:number, count:number, complete:boolean,
 *            params:Array<{name:string,value:number,type:number,default?:number}> }}
 */
function decodeParamPack(raw) {
    const buf = Buffer.isBuffer(raw) ? raw : Buffer.from(raw || []);
    const out = { ok: false, total: 0, count: 0, complete: false, params: [] };
    if (buf.length < 6) { out.error = 'short header'; return out; }
    const magic = buf.readUInt16LE(0);
    if (magic !== PCK_MAGIC && magic !== PCK_MAGIC_DEFAULTS) {
        out.error = `bad magic 0x${magic.toString(16)}`;
        return out;
    }
    const withDefaults = magic === PCK_MAGIC_DEFAULTS;
    out.total = buf.readUInt16LE(4);
    const numParams = buf.readUInt16LE(2);
    out.ok = true;

    let pos = 6;
    let lastName = '';
    while (pos < buf.length) {
        while (pos < buf.length && buf[pos] === 0) pos++;      // padding
        if (pos + 2 > buf.length) break;
        const ptype = buf[pos] & 0x0F;
        const flags = buf[pos] >> 4;
        const commonLen = buf[pos + 1] & 0x0F;
        const nameLen = (buf[pos + 1] >> 4) + 1;
        const typeLen = PCK_TYPE_SIZE[ptype];
        if (!typeLen) { out.ok = false; out.error = `bad type ${ptype} at ${pos}`; break; }
        if (commonLen > lastName.length) { out.ok = false; out.error = `bad common length at ${pos}`; break; }
        const hasDefault = withDefaults && (flags & 1) !== 0;
        const entryLen = 2 + nameLen + typeLen + (hasDefault ? typeLen : 0);
        if (pos + entryLen > buf.length) break;                 // the rest has not arrived yet
        const name = lastName.slice(0, commonLen) + buf.toString('latin1', pos + 2, pos + 2 + nameLen);
        const value = readValue(buf, pos + 2 + nameLen, ptype);
        const p = { name, value, type: PCK_TO_MAV_TYPE[ptype] };
        if (hasDefault) p.default = readValue(buf, pos + 2 + nameLen + typeLen, ptype);
        else if (withDefaults) p.default = value;
        out.params.push(p);
        lastName = name;
        pos += entryLen;
    }
    out.count = out.params.length;
    out.complete = out.ok && out.count === numParams && numParams > 0;
    return out;
}

/**
 * Encode a parameter list the way the firmware does (used by the tests and
 * by anything that wants to fake a vehicle). `params`: [{name, value, ptype}]
 * with ptype 1..4. Entries that would straddle a `packetLen` boundary are
 * preceded by zero padding, exactly as AP_Filesystem_Param does.
 */
function encodeParamPack(params, { packetLen = FTP_DATA_LEN } = {}) {
    const parts = [];
    const header = Buffer.alloc(6);
    header.writeUInt16LE(PCK_MAGIC, 0);
    header.writeUInt16LE(params.length, 2);
    header.writeUInt16LE(params.length, 4);
    parts.push(header);
    let pos = 6;
    let lastName = '';
    for (const p of params) {
        const name = String(p.name);
        let common = 0;
        while (common < Math.min(15, lastName.length, name.length) && lastName[common] === name[common]) common++;
        let tail = name.slice(common);
        if (!tail.length) { common--; tail = name.slice(common); }
        const typeLen = PCK_TYPE_SIZE[p.ptype];
        const len = 2 + tail.length + typeLen;
        const boundary = Math.floor(pos / packetLen);
        if (Math.floor((pos + len - 1) / packetLen) !== boundary) {
            const pad = (boundary + 1) * packetLen - pos;
            parts.push(Buffer.alloc(pad));
            pos += pad;
        }
        const e = Buffer.alloc(len);
        e[0] = p.ptype & 0x0F;
        e[1] = (common & 0x0F) | ((tail.length - 1) << 4);
        e.write(tail, 2, 'latin1');
        const vofs = 2 + tail.length;
        switch (p.ptype) {
            case 1: e.writeInt8(p.value, vofs); break;
            case 2: e.writeInt16LE(p.value, vofs); break;
            case 3: e.writeInt32LE(p.value, vofs); break;
            default: e.writeFloatLE(p.value, vofs); break;
        }
        parts.push(e);
        pos += len;
        lastName = name;
    }
    return Buffer.concat(parts);
}

module.exports = {
    FTP_OP, FTP_ERROR, FTP_PAYLOAD_LEN, FTP_DATA_LEN,
    FtpError, MavFtpClient,
    buildPayload, parsePayload,
    decodeParamPack, encodeParamPack,
    PCK_TO_MAV_TYPE
};
