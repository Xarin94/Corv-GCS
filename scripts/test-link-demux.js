#!/usr/bin/env node
/**
 * test-link-demux.js - offline checks for link-demux.js
 *
 * A synthetic cellular-link stream, no relay and no module: CORV-2 binary
 * packets from a CORV board with the LTEtelem module's own MAVLink frames
 * (RADIO_STATUS, CORV_CELL_INFO) mixed in, cut into chunks of random size as
 * the link records arrive. Payloads carry 0xFD and 0xA5 0x5A on purpose.
 *
 *   node scripts/test-link-demux.js
 */

const { PassThrough } = require('stream');
const { MavLinkPacketParser } = require('node-mavlink');
const { LinkDemux, x25, ccitt } = require('../link-demux');
const corv = require('../corv-mavlink');

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}

// Deterministic pseudo-random numbers (the test must not be flaky)
let seed = 12345;
const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
const rndInt = (lo, hi) => lo + Math.floor(rnd() * (hi - lo + 1));

/** CORV-2 packet: A5 5A, type, length, sequence, payload, CRC-16-CCITT (LE) over type..payload. */
let corvSeq = 0;
function corvPacket(type, payload) {
    const p = Buffer.alloc(7 + payload.length);
    p[0] = 0xA5; p[1] = 0x5A; p[2] = type; p[3] = payload.length; p[4] = corvSeq++ & 0xFF;
    payload.copy(p, 5);
    const crc = ccitt(p, 2, 5 + payload.length);
    p[5 + payload.length] = crc & 0xFF;
    p[6 + payload.length] = crc >> 8;
    return p;
}

/** MAVLink2 frame from sysid 51 / compid 68 (the module's "radio"). */
let mavSeq = 0;
function mav2(msgid, payload, crcExtra) {
    const f = Buffer.alloc(12 + payload.length);
    f[0] = 0xFD; f[1] = payload.length; f[2] = 0; f[3] = 0; f[4] = mavSeq++ & 0xFF; f[5] = 51; f[6] = 68;
    f[7] = msgid & 0xFF; f[8] = (msgid >> 8) & 0xFF; f[9] = (msgid >> 16) & 0xFF;
    payload.copy(f, 10);
    const crc = x25(Buffer.of(crcExtra), 0, 1, x25(f, 1, 10 + payload.length));
    f[10 + payload.length] = crc & 0xFF;
    f[11 + payload.length] = crc >> 8;
    return f;
}

function payloadWithTraps(n) {
    const p = Buffer.alloc(n);
    for (let i = 0; i < n; i++) p[i] = rndInt(0, 255);
    // a MAVLink start and a CORV sync inside the payload
    if (n > 12) { p[3] = 0xFD; p[4] = 40; p[8] = 0xA5; p[9] = 0x5A; p[10] = 0x01; p[11] = 20; }
    return p;
}

// CORV_CELL_INFO as packed by pymavlink from LTEtelem tools/mavlink/corv.xml
const CELL_INFO = Buffer.from('fd3f0000003344e8cb0009de571b04b500009c180000030000009c180000640000003a07000000000000'
    + 'de0032006001a1ffbaffd2003f006a01ffff9bff96ff92ffff7ff70c1404070a38', 'hex');
const radioStatus = () => mav2(109, Buffer.from([3, 0, 0, 0, 131, 255, 100, 0, 0]), 185);

function run(stream, chunkMax) {
    const out = [];
    const d = new LinkDemux({
        magicNumbers: corv.MAGIC_NUMBERS,
        onMavlink: (f) => out.push({ kind: 'mav', bytes: Buffer.from(f) }),
        onCorv: (type, payload) => out.push({ kind: 'corv', type, payload }),
    });
    for (let i = 0; i < stream.length;) {
        const n = Math.min(stream.length - i, rndInt(1, chunkMax));
        d.push(stream.subarray(i, i + n));
        i += n;
    }
    return { out, stats: d.stats };
}

(async () => {
    // ── 1. CORV board + module frames, chunked like link records ──────────────
    const expected = [];
    const parts = [];
    for (let k = 0; k < 400; k++) {
        const type = [0x01, 0x01, 0x01, 0x02, 0x03][k % 5];
        const len = type === 0x01 ? 104 : type === 0x02 ? 92 : 74;
        const payload = payloadWithTraps(len);
        parts.push(corvPacket(type, payload));
        expected.push({ kind: 'corv', type, payload });
        if (k % 50 === 7) { const f = radioStatus(); parts.push(f); expected.push({ kind: 'mav', bytes: f }); }
        if (k % 100 === 33) { parts.push(CELL_INFO); expected.push({ kind: 'mav', bytes: CELL_INFO }); }
    }
    const stream = Buffer.concat(parts);
    for (const chunkMax of [1, 7, 64, 800]) {
        const { out, stats } = run(stream, chunkMax);
        const same = out.length === expected.length && out.every((o, i) => {
            const e = expected[i];
            return o.kind === e.kind && (o.kind === 'mav' ? o.bytes.equals(e.bytes)
                : o.type === e.type && o.payload.equals(e.payload));
        });
        check(`mixed stream, chunks up to ${chunkMax} B: everything out, in order`, same,
            `${stats.corv} CORV, ${stats.mavlink} MAVLink, ${stats.skipped} bytes skipped`);
        check(`mixed stream, chunks up to ${chunkMax} B: nothing skipped`, stats.skipped === 0);
    }

    // ── 2. Module frames come out decodable by the normal MAVLink pipeline ────
    {
        const { out } = run(stream, 300);
        const input = new PassThrough();
        const parser = input.pipe(corv.createSplitter()).pipe(new MavLinkPacketParser());
        const got = [];
        parser.on('data', (p) => got.push(p));
        for (const o of out) if (o.kind === 'mav') input.write(o.bytes);
        await new Promise((r) => setTimeout(r, 100));
        const cells = got.filter((p) => p.header.msgid === 52200);
        const radios = got.filter((p) => p.header.msgid === 109);
        check('RADIO_STATUS and CORV_CELL_INFO reach the MAVLink parser', cells.length === 4 && radios.length === 8,
            `${radios.length} RADIO_STATUS, ${cells.length} CORV_CELL_INFO`);
        const m = cells.length && cells[0].protocol.data(cells[0].payload, corv.CorvCellInfo);
        check('CORV_CELL_INFO decodes', !!m && m.eci === 0x1B57DE09 && m.mnc === 50 && m.rsrp === -95);
    }

    // ── 3. Damage: a flipped byte and stray bytes cost that packet only ───────
    // (the stream goes on afterwards, as a live link does: a stray start is
    // only given up once a whole frame's worth of bytes has arrived)
    {
        const good = [corvPacket(0x01, payloadWithTraps(104)), radioStatus(), corvPacket(0x02, payloadWithTraps(92))];
        const bad = Buffer.from(corvPacket(0x01, payloadWithTraps(104)));
        bad[50] ^= 0x40;
        const tail = [];
        for (let k = 0; k < 4; k++) tail.push(corvPacket(0x03, payloadWithTraps(74)));
        const s = Buffer.concat([good[0], bad, good[1], Buffer.from([0x13, 0xFD, 0xA5]), good[2], ...tail]);
        const { out } = run(s, 40);
        const kinds = out.map((o) => o.kind === 'mav' ? 'M' : o.type).join(' ');
        check('corrupted packet and stray bytes dropped, the rest kept', kinds === '1 M 2 3 3 3 3', kinds);
    }

    // ── 4. A stray MAVLink start with a long length does not swallow packets ──
    {
        const after = [];
        for (let k = 0; k < 6; k++) after.push(corvPacket(0x01, payloadWithTraps(104)));
        const s = Buffer.concat([Buffer.from([0xFD, 0xF0, 0x00]), ...after]);
        const { out } = run(s, 1000);
        check('stray 0xFD: every following packet still decoded', out.filter((o) => o.kind === 'corv').length === 6);
    }

    // ── 5. Plain MAVLink (an ArduPilot board) passes untouched ────────────────
    {
        const frames = [];
        for (let k = 0; k < 300; k++) frames.push(k % 3 ? radioStatus() : mav2(0, Buffer.from([0, 0, 0, 0, 1, 3, 0x51, 4, 3]), 50));
        const { out, stats } = run(Buffer.concat(frames), 500);
        check('plain MAVLink: every frame, nothing skipped',
            out.length === 300 && stats.skipped === 0 && out.every((o, i) => o.kind === 'mav' && o.bytes.equals(frames[i])));
    }

    console.log(failures ? `\n${failures} FAILED` : '\nall passed');
    process.exit(failures ? 1 : 0);
})();
