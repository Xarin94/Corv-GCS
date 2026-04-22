/**
 * log-replay-bin-parser.js
 *
 * Minimal ArduPilot DataFlash (.bin) log parser that converts a whitelisted set
 * of messages into MAVLink-shaped records suitable for replay through the
 * existing renderer telemetry pipeline.
 *
 * Format reference:
 *   - Record: 0xA3 0x95 <msgType:u8> <payload...>
 *   - Self-describing via FMT records (msgType=128): each FMT declares
 *     { type:u8, length:u8, name:char[4], format:char[16], labels:char[64] }
 *
 * Scope: we only decode the messages the GCS UI consumes (ATT/GPS/BARO/ARSP/
 * BAT/MODE/MSG/RCIN/RCOU/VIBE/ORGN) and produce MAVLink message IDs that
 * MAVLinkStateMapper already handles (30/33/24/74/1/0/253/35/36/241/242).
 */

const fs = require('fs');

const HEAD0 = 0xA3;
const HEAD1 = 0x95;
const FMT_MSG_TYPE = 128;
const FMT_RECORD_LENGTH = 89;

const WHITELIST = new Set([
    'ATT', 'GPS', 'BARO', 'ARSP', 'BAT', 'MODE', 'MSG',
    'RCIN', 'RCOU', 'VIBE', 'ORGN'
]);

function typeSize(c) {
    switch (c) {
        case 'b': case 'B': case 'M': return 1;
        case 'h': case 'H': case 'c': case 'C': return 2;
        case 'i': case 'I': case 'f': case 'e': case 'E': case 'L': return 4;
        case 'd': case 'q': case 'Q': return 8;
        case 'n': return 4;
        case 'N': return 16;
        case 'Z': return 64;
        case 'a': return 64;  // int16[32]
        default: return 0;
    }
}

function readField(buf, offset, c) {
    switch (c) {
        case 'b': return buf.readInt8(offset);
        case 'B': case 'M': return buf.readUInt8(offset);
        case 'h': return buf.readInt16LE(offset);
        case 'H': return buf.readUInt16LE(offset);
        case 'i': case 'L': return buf.readInt32LE(offset);
        case 'I': return buf.readUInt32LE(offset);
        case 'f': return buf.readFloatLE(offset);
        case 'd': return buf.readDoubleLE(offset);
        case 'q': return Number(buf.readBigInt64LE(offset));
        case 'Q': return Number(buf.readBigUInt64LE(offset));
        case 'c': return buf.readInt16LE(offset) * 0.01;
        case 'C': return buf.readUInt16LE(offset) * 0.01;
        case 'e': return buf.readInt32LE(offset) * 0.01;
        case 'E': return buf.readUInt32LE(offset) * 0.01;
        case 'n': return buf.subarray(offset, offset + 4).toString('ascii').replace(/\0.*$/, '');
        case 'N': return buf.subarray(offset, offset + 16).toString('ascii').replace(/\0.*$/, '');
        case 'Z': return buf.subarray(offset, offset + 64).toString('ascii').replace(/\0.*$/, '');
        default: return null;
    }
}

/**
 * Seed the FMT table with the FMT message's own description so Pass 1 can
 * deserialize the first FMT record it encounters.
 */
function seedFmtTable() {
    const fieldOffsets = [
        { name: 'Type',   typeChar: 'B', offset: 0 },
        { name: 'Length', typeChar: 'B', offset: 1 },
        { name: 'Name',   typeChar: 'n', offset: 2 },
        { name: 'Format', typeChar: 'N', offset: 6 },
        { name: 'Labels', typeChar: 'Z', offset: 22 }
    ];
    const table = new Map();
    table.set(FMT_MSG_TYPE, {
        length: FMT_RECORD_LENGTH,
        name: 'FMT',
        format: 'BBnNZ',
        labels: ['Type', 'Length', 'Name', 'Format', 'Labels'],
        fieldOffsets
    });
    return table;
}

/**
 * Resync to the next HEAD marker starting from offset. Returns the new offset
 * (>= start) of a valid HEAD, or buf.length if none found.
 */
function findNextHead(buf, start) {
    for (let i = start; i + 1 < buf.length; i++) {
        if (buf[i] === HEAD0 && buf[i + 1] === HEAD1) return i;
    }
    return buf.length;
}

/**
 * Parse the file and return { index, totalMs, totalMessages }.
 * index entries: { tsMs, msgId, data, sysId, compId }
 */
function indexBinFile(filePath) {
    const buf = fs.readFileSync(filePath);
    const fmtTable = seedFmtTable();

    // Single-pass scan: FMT records always precede the messages they describe.
    // Unknown msgType => resync by scanning for next HEAD.
    const items = [];
    let minUs = null;

    // Carry-over state for messages that contribute multiple fields to one
    // MAVLink message (VFR_HUD combines BARO + ARSP).
    let vfrAirspeed = 0;
    let vfrClimb = 0;
    let vfrAlt = 0;
    let gpsFix = 0;
    let gpsNumSat = 0;
    let gpsHdop = 99.9;

    let offset = 0;
    while (offset + 3 <= buf.length) {
        if (buf[offset] !== HEAD0 || buf[offset + 1] !== HEAD1) {
            offset = findNextHead(buf, offset + 1);
            continue;
        }
        const msgType = buf[offset + 2];
        const entry = fmtTable.get(msgType);
        if (!entry) {
            offset = findNextHead(buf, offset + 1);
            continue;
        }
        const len = entry.length;
        if (offset + len > buf.length) break;

        // FMT record: expand the table
        if (msgType === FMT_MSG_TYPE) {
            const p = buf.subarray(offset + 3, offset + len);
            try {
                const type = p.readUInt8(0);
                const length = p.readUInt8(1);
                const name = p.subarray(2, 6).toString('ascii').replace(/\0.*$/, '').trim();
                const format = p.subarray(6, 22).toString('ascii').replace(/\0.*$/, '').trim();
                const labelsStr = p.subarray(22, 86).toString('ascii').replace(/\0.*$/, '').trim();
                const labels = labelsStr.length ? labelsStr.split(',') : [];
                const fieldOffsets = [];
                let foff = 0;
                for (let i = 0; i < labels.length && i < format.length; i++) {
                    fieldOffsets.push({ name: labels[i], typeChar: format[i], offset: foff });
                    foff += typeSize(format[i]);
                }
                if (!fmtTable.has(type) || type !== FMT_MSG_TYPE) {
                    fmtTable.set(type, { length, name, format, labels, fieldOffsets });
                }
            } catch (e) {
                // Corrupted FMT — skip silently
            }
            offset += len;
            continue;
        }

        if (!WHITELIST.has(entry.name)) {
            offset += len;
            continue;
        }

        const payload = buf.subarray(offset + 3, offset + len);
        const fm = {};
        for (const f of entry.fieldOffsets) {
            try {
                fm[f.name] = readField(payload, f.offset, f.typeChar);
            } catch (e) {
                // Skip field if out-of-bounds; continue best-effort decoding
            }
        }

        const tsUs = typeof fm.TimeUS === 'number' ? fm.TimeUS : null;
        if (tsUs === null) { offset += len; continue; }
        if (minUs === null || tsUs < minUs) minUs = tsUs;

        switch (entry.name) {
            case 'ATT': {
                const D = Math.PI / 180;
                items.push({
                    tsUs, msgId: 30, sysId: 1, compId: 1,
                    data: {
                        roll: (fm.Roll || 0) * D,
                        pitch: (fm.Pitch || 0) * D,
                        yaw: (fm.Yaw || 0) * D,
                        rollspeed: 0, pitchspeed: 0, yawspeed: 0,
                        timeBootMs: Math.floor(tsUs / 1000)
                    }
                });
                break;
            }
            case 'GPS': {
                gpsFix = (fm.Status !== undefined) ? fm.Status : gpsFix;
                gpsNumSat = (fm.NSats !== undefined) ? fm.NSats : gpsNumSat;
                gpsHdop = (fm.HDop !== undefined) ? fm.HDop : gpsHdop;

                const lat = fm.Lat;
                const lon = fm.Lng;
                const alt_m = fm.Alt || 0;
                const spd = fm.Spd || 0;
                const gcrs = fm.GCrs || 0;
                const vx = spd * Math.cos(gcrs * Math.PI / 180);
                const vy = spd * Math.sin(gcrs * Math.PI / 180);

                if (lat !== undefined && lon !== undefined) {
                    items.push({
                        tsUs, msgId: 33, sysId: 1, compId: 1,
                        data: {
                            lat: Math.round(lat),
                            lon: Math.round(lon),
                            alt: Math.round(alt_m * 1000),
                            relativeAlt: Math.round(alt_m * 1000),
                            vx: Math.round(vx * 100),
                            vy: Math.round(vy * 100),
                            vz: 0,
                            hdg: Math.round(gcrs * 100)
                        }
                    });
                }
                items.push({
                    tsUs, msgId: 24, sysId: 1, compId: 1,
                    data: {
                        fixType: gpsFix || 0,
                        satellitesVisible: gpsNumSat || 0,
                        eph: Math.round((gpsHdop || 99.9) * 100),
                        lat: (lat !== undefined) ? Math.round(lat) : 0,
                        lon: (lon !== undefined) ? Math.round(lon) : 0,
                        alt: Math.round(alt_m * 1000),
                        vel: Math.round(spd * 100),
                        cog: Math.round(gcrs * 100)
                    }
                });
                break;
            }
            case 'BARO': {
                vfrAlt = (fm.Alt !== undefined) ? fm.Alt : vfrAlt;
                vfrClimb = (fm.CRt !== undefined) ? fm.CRt : vfrClimb;
                items.push({
                    tsUs, msgId: 74, sysId: 1, compId: 1,
                    data: {
                        airspeed: vfrAirspeed,
                        groundspeed: vfrAirspeed, // best-effort until GPS fills gs
                        climb: vfrClimb,
                        altitude: vfrAlt,
                        heading: 0,
                        throttle: 0
                    }
                });
                break;
            }
            case 'ARSP': {
                vfrAirspeed = (fm.Airspeed !== undefined) ? fm.Airspeed : vfrAirspeed;
                items.push({
                    tsUs, msgId: 74, sysId: 1, compId: 1,
                    data: {
                        airspeed: vfrAirspeed,
                        groundspeed: vfrAirspeed,
                        climb: vfrClimb,
                        altitude: vfrAlt,
                        heading: 0,
                        throttle: 0
                    }
                });
                break;
            }
            case 'BAT': {
                const volt = fm.Volt || 0;
                const curr = fm.Curr || 0;
                items.push({
                    tsUs, msgId: 1, sysId: 1, compId: 1,
                    data: {
                        voltageBattery: Math.round(volt * 1000),
                        currentBattery: Math.round(curr * 100),
                        batteryRemaining: -1,
                        dropRateComm: 0,
                        errorsComm: 0
                    }
                });
                break;
            }
            case 'MODE': {
                const modeNum = fm.Mode || 0;
                items.push({
                    tsUs, msgId: 0, sysId: 1, compId: 1,
                    data: {
                        type: 1,          // MAV_TYPE_FIXED_WING (ArduPlane)
                        autopilot: 3,     // MAV_AUTOPILOT_ARDUPILOTMEGA
                        baseMode: 209,    // armed + custom mode + auto
                        customMode: modeNum,
                        systemStatus: 4,  // MAV_STATE_ACTIVE
                        mavlinkVersion: 3
                    }
                });
                break;
            }
            case 'MSG': {
                const text = fm.Message || '';
                if (text) {
                    items.push({
                        tsUs, msgId: 253, sysId: 1, compId: 1,
                        data: { severity: 6, text }
                    });
                }
                break;
            }
            case 'RCIN': {
                items.push({
                    tsUs, msgId: 35, sysId: 1, compId: 1,
                    data: {
                        chan1Raw: fm.C1 || 0, chan2Raw: fm.C2 || 0,
                        chan3Raw: fm.C3 || 0, chan4Raw: fm.C4 || 0,
                        chan5Raw: fm.C5 || 0, chan6Raw: fm.C6 || 0,
                        chan7Raw: fm.C7 || 0, chan8Raw: fm.C8 || 0,
                        rssi: 0
                    }
                });
                break;
            }
            case 'RCOU': {
                items.push({
                    tsUs, msgId: 36, sysId: 1, compId: 1,
                    data: {
                        servo1Raw: fm.C1 || 0, servo2Raw: fm.C2 || 0,
                        servo3Raw: fm.C3 || 0, servo4Raw: fm.C4 || 0,
                        servo5Raw: fm.C5 || 0, servo6Raw: fm.C6 || 0,
                        servo7Raw: fm.C7 || 0, servo8Raw: fm.C8 || 0
                    }
                });
                break;
            }
            case 'VIBE': {
                items.push({
                    tsUs, msgId: 241, sysId: 1, compId: 1,
                    data: {
                        vibrationX: fm.VibeX || 0,
                        vibrationY: fm.VibeY || 0,
                        vibrationZ: fm.VibeZ || 0,
                        clipping_0: fm.Clip0 || 0,
                        clipping_1: fm.Clip1 || 0,
                        clipping_2: fm.Clip2 || 0
                    }
                });
                break;
            }
            case 'ORGN': {
                if (fm.Lat !== undefined && fm.Lng !== undefined) {
                    items.push({
                        tsUs, msgId: 242, sysId: 1, compId: 1,
                        data: {
                            latitude: Math.round(fm.Lat),
                            longitude: Math.round(fm.Lng),
                            altitude: Math.round((fm.Alt || 0) * 1000)
                        }
                    });
                }
                break;
            }
        }

        offset += len;
    }

    if (minUs === null) minUs = 0;

    // Normalize to start at 0 and sort stably by tsMs
    const index = items.map(it => ({
        tsMs: Math.max(0, Math.floor((it.tsUs - minUs) / 1000)),
        msgId: it.msgId,
        data: it.data,
        sysId: it.sysId,
        compId: it.compId
    }));
    index.sort((a, b) => a.tsMs - b.tsMs);

    const totalMs = index.length ? index[index.length - 1].tsMs : 0;
    return { index, totalMs, totalMessages: index.length };
}

module.exports = { indexBinFile };
