/**
 * link-demux.js - MAVLink and CORV-2 binary on one byte stream
 *
 * The cellular link carries whatever the flight controller sends: MAVLink from
 * an ArduPilot board, or CORV-2 binary telemetry from a CORV board. Either way
 * the LTEtelem module mixes in MAVLink of its own (RADIO_STATUS and
 * CORV_CELL_INFO, sysid 51), so a CORV stream is never pure. LinkDemux takes
 * the bytes as they come and hands out, in order:
 *   - every MAVLink v1/v2 frame whose CRC checks out (CRC_EXTRA known)
 *   - every CORV-2 packet (0xA5 0x5A, type, length, seq, payload, CRC-16-CCITT)
 *     whose CRC checks out
 * Nothing is taken without its CRC, so a 0xFD or 0xA5 0x5A inside the other
 * protocol's payload is just a byte to step over. Frames of a MAVLink message
 * the GCS cannot check (unknown msgid) are dropped, as node-mavlink does.
 * A stray start (a byte lost on the way) is given up when the bytes its length
 * promises have arrived and the CRC fails: the stream behind it waits at most
 * one frame, 280 bytes.
 */

const MAV1 = 0xFE, MAV2 = 0xFD;
const CORV_SYNC0 = 0xA5, CORV_SYNC1 = 0x5A;
const MAV2_SIGNED = 0x01;

/** CRC-16/MCRF4XX (X.25) as MAVLink uses it. */
function x25(buf, start, end, crc = 0xFFFF) {
    for (let i = start; i < end; i++) {
        let tmp = buf[i] ^ (crc & 0xFF);
        tmp = (tmp ^ (tmp << 4)) & 0xFF;
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
    }
    return crc;
}

/** CRC-16-CCITT (poly 0x1021, init 0xFFFF), the CORV-2 packet checksum. */
function ccitt(buf, start, end) {
    let crc = 0xFFFF;
    for (let i = start; i < end; i++) {
        crc ^= buf[i] << 8;
        for (let b = 0; b < 8; b++) crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
    return crc;
}

class LinkDemux {
    /**
     * @param {object} opts
     * @param {Object<number, number>} opts.magicNumbers msgid → CRC_EXTRA
     * @param {(frame: Buffer) => void} opts.onMavlink a whole MAVLink frame
     * @param {(type: number, payload: Buffer) => void} opts.onCorv a CORV-2 packet
     */
    constructor({ magicNumbers, onMavlink, onCorv }) {
        this.magic = magicNumbers;
        this.onMavlink = onMavlink;
        this.onCorv = onCorv;
        this.buf = Buffer.alloc(0);
        this.stats = { mavlink: 0, corv: 0, skipped: 0 };
    }

    /** Frame starting at i: its length, 0 = not one, -1 = need more bytes. */
    _mavlinkAt(b, i) {
        const left = b.length - i;
        let hdr, len, msgid;
        if (b[i] === MAV2) {
            if (left < 10) return -1;
            hdr = 10;
            len = b[i + 1] + 12 + ((b[i + 2] & MAV2_SIGNED) ? 13 : 0);
            msgid = b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16);
        } else if (b[i] === MAV1) {
            if (left < 6) return -1;
            hdr = 6;
            len = b[i + 1] + 8;
            msgid = b[i + 5];
        } else {
            return 0;
        }
        if (left < len) return -1;
        const magic = this.magic[msgid];
        if (magic === undefined || magic === null) return 0;
        const end = i + hdr + b[i + 1];                  // CRC follows the payload
        const crc = x25(Buffer.of(magic), 0, 1, x25(b, i + 1, end));
        return crc === (b[end] | (b[end + 1] << 8)) ? len : 0;
    }

    /** CORV-2 packet starting at i: its length, 0 = not one, -1 = need more bytes. */
    _corvAt(b, i) {
        if (b[i] !== CORV_SYNC0) return 0;
        const left = b.length - i;
        if (left < 2) return -1;
        if (b[i + 1] !== CORV_SYNC1) return 0;
        if (left < 5) return -1;
        const len = 5 + b[i + 3] + 2;
        if (left < len) return -1;
        const crc = ccitt(b, i + 2, i + 5 + b[i + 3]);
        return crc === (b[i + len - 2] | (b[i + len - 1] << 8)) ? len : 0;
    }

    /** Feed received bytes; frames and packets are handed out as they complete. */
    push(chunk) {
        let b = this.buf.length ? Buffer.concat([this.buf, chunk]) : Buffer.from(chunk);
        let i = 0;
        while (i < b.length) {
            const m = this._mavlinkAt(b, i);
            if (m > 0) {
                this.stats.mavlink++;
                this.onMavlink(b.subarray(i, i + m));
                i += m;
                continue;
            }
            const c = this._corvAt(b, i);
            if (c > 0) {
                this.stats.corv++;
                this.onCorv(b[i + 2], Buffer.from(b.subarray(i + 5, i + c - 2)));
                i += c;
                continue;
            }
            if (m < 0 || c < 0) break;          // a start that is still arriving
            this.stats.skipped++;
            i++;
        }
        // what is left is a start still arriving: at most one frame (280 bytes)
        this.buf = Buffer.from(b.subarray(i));
    }
}

module.exports = { LinkDemux, x25, ccitt };
