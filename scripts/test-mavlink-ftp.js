#!/usr/bin/env node
/**
 * test-mavlink-ftp.js - MAVLink FTP client + packed parameter file decoder
 *
 * A fake vehicle answers like GCS_FTP.cpp does (open with a size estimate,
 * bursts of 239-byte packets, short last packet or NAK EOF, single reads,
 * busy sessions) with configurable packet loss, and the client has to come
 * back with the exact file. Run: npm run test:ftp
 */
'use strict';
const assert = require('assert');
const {
    FTP_OP, FTP_ERROR, FTP_DATA_LEN, MavFtpClient,
    buildPayload, parsePayload, decodeParamPack, encodeParamPack
} = require('../mavlink-ftp');

let passed = 0;
function ok(name, fn) {
    return Promise.resolve().then(fn).then(() => { passed++; console.log(`  ok   ${name}`); },
        e => { console.error(`  FAIL ${name}\n       ${e.stack || e}`); process.exitCode = 1; });
}

// ── a parameter list that looks like a real one ───────────────────────────
function sampleParams(n = 1300) {
    const groups = ['ATC_RAT_RLL_', 'ATC_RAT_PIT_', 'BATT_', 'COMPASS_OFS_', 'EK3_', 'INS_', 'SERVO', 'RC', 'GPS_', 'WPNAV_'];
    const suff = ['P', 'I', 'D', 'FF', 'FLTD', 'MAX', 'MIN', 'TYPE', 'ENABLE', 'X', 'Y', 'Z', 'OPTIONS'];
    const out = [];
    for (let i = 0; i < n; i++) {
        const name = (groups[i % groups.length] + (i % 7 ? '' : (i % 16) + '_') + suff[(i * 7) % suff.length] + (i % 3 ? '' : (i % 9)))
            .slice(0, 16);
        const ptype = 1 + (i % 4);
        let value;
        if (ptype === 1) value = (i % 200) - 100;
        else if (ptype === 2) value = (i * 37) % 60000 - 30000;
        else if (ptype === 3) value = (i * 100003) % 2000000000 - 1000000000;
        else value = Math.fround(i * 0.137 - 50.5);
        out.push({ name, value, ptype });
    }
    return out;
}

// ── fake vehicle ─────────────────────────────────────────────────────────
class FakeVehicle {
    constructor(file, opts = {}) {
        this.file = file;
        this.client = null;
        this.dropEvery = opts.dropEvery || 0;      // drop every n-th burst packet
        this.dropReadOnce = !!opts.dropReadOnce;   // lose the first single-read reply
        this.busyFirst = !!opts.busyFirst;         // NAK NoSessionsAvailable until a reset
        this.silent = !!opts.silent;
        this.burstLimit = opts.burstLimit || 2000;
        this.delayMs = opts.delayMs ?? 0;
        this.sent = 0; this.burstSent = 0; this.reads = 0; this.opens = 0; this.resets = 0; this.terminates = 0;
        this.fd = false;
    }
    reply(fields) {
        this.sent++;
        const p = buildPayload(fields);
        setTimeout(() => this.client.onPacket(Array.from(p)), this.delayMs);
    }
    nak(req, code, extra = {}) {
        this.reply({ seq: req.seq + 1, session: req.session, opcode: FTP_OP.Nak, reqOpcode: req.opcode, data: Buffer.from([code]), ...extra });
    }
    onRequest(payload) {
        if (this.silent) return;
        const req = parsePayload(payload);
        const base = { seq: req.seq + 1, session: req.session, opcode: FTP_OP.Ack, reqOpcode: req.opcode };
        switch (req.opcode) {
            case FTP_OP.ResetSessions:
                this.resets++; this.busyFirst = false; this.fd = false;
                return this.reply(base);
            case FTP_OP.TerminateSession:
                this.terminates++; this.fd = false;
                return this.reply(base);
            case FTP_OP.OpenFileRO: {
                this.opens++;
                if (this.busyFirst) return this.nak(req, FTP_ERROR.NoSessionsAvailable);
                const path = req.data.toString('utf8');
                if (path !== '@PARAM/param.pck') return this.nak(req, FTP_ERROR.FileNotFound);
                this.fd = true;
                const d = Buffer.alloc(4); d.writeUInt32LE(this.file.length + 400, 0);   // an estimate, as the firmware gives
                return this.reply({ ...base, data: d });
            }
            case FTP_OP.ReadFile: {
                this.reads++;
                if (!this.fd) return this.nak(req, FTP_ERROR.FileNotFound);
                if (req.offset >= this.file.length) return this.nak(req, FTP_ERROR.EndOfFile);
                const d = this.file.subarray(req.offset, Math.min(this.file.length, req.offset + req.size));
                if (this.dropReadOnce) { this.dropReadOnce = false; return; }
                return this.reply({ ...base, offset: req.offset, data: d });
            }
            case FTP_OP.BurstReadFile: {
                if (!this.fd) return this.nak(req, FTP_ERROR.FileNotFound);
                let ofs = req.offset; let seq = req.seq + 1;
                const max = req.size || FTP_DATA_LEN;
                for (let i = 0; i < this.burstLimit; i++) {
                    if (ofs >= this.file.length) {
                        return this.reply({ seq, session: req.session, opcode: FTP_OP.Nak, reqOpcode: req.opcode, burstComplete: 1, offset: ofs, data: Buffer.from([FTP_ERROR.EndOfFile]) });
                    }
                    const d = this.file.subarray(ofs, Math.min(this.file.length, ofs + max));
                    const last = i === this.burstLimit - 1 || d.length < max;
                    this.burstSent++;
                    const drop = this.dropEvery && (this.burstSent % this.dropEvery === 0) && !last;
                    if (!drop) this.reply({ seq, session: req.session, opcode: FTP_OP.Ack, reqOpcode: req.opcode, burstComplete: last ? 1 : 0, offset: ofs, data: d });
                    seq = (seq + 1) & 0xFFFF; ofs += d.length;
                    if (last) return;
                }
                return;
            }
            default:
                return this.nak(req, FTP_ERROR.UnknownCommand);
        }
    }
}

function makePair(file, opts) {
    const veh = new FakeVehicle(file, opts);
    const client = new MavFtpClient({ send: (payload) => { veh.onRequest(payload); } });
    veh.client = client;
    return { veh, client };
}

(async () => {
    console.log('mavlink-ftp');

    const params = sampleParams(1300);
    const file = encodeParamPack(params);

    await ok('pck encode → decode round trip (1300 params, 4 types, padding)', () => {
        const d = decodeParamPack(file);
        assert.ok(d.ok, d.error);
        assert.ok(d.complete);
        assert.strictEqual(d.total, params.length);
        assert.strictEqual(d.params.length, params.length);
        for (let i = 0; i < params.length; i++) {
            assert.strictEqual(d.params[i].name, params[i].name, `name ${i}`);
            assert.strictEqual(d.params[i].value, params[i].value, `value ${i} ${params[i].name}`);
            assert.strictEqual(d.params[i].type, [null, 2, 4, 6, 9][params[i].ptype]);
        }
        // every entry sits inside one 239-byte packet
        assert.ok(file.length > 239 * 20);
    });

    await ok('partial file decodes to a prefix and says it is incomplete', () => {
        const d = decodeParamPack(file.subarray(0, 239 * 5));
        assert.ok(d.ok, d.error);
        assert.ok(!d.complete);
        assert.ok(d.count > 50 && d.count < params.length, `count ${d.count}`);
        assert.strictEqual(d.total, params.length);
        assert.strictEqual(d.params[0].name, params[0].name);
        assert.strictEqual(d.params[d.count - 1].name, params[d.count - 1].name);
    });

    await ok('bad magic is refused', () => {
        const d = decodeParamPack(Buffer.from([1, 2, 3, 4, 5, 6, 7]));
        assert.ok(!d.ok);
    });

    await ok('lossless burst read returns the exact file', async () => {
        const { veh, client } = makePair(file);
        let progress = 0;
        const out = await client.readFile('@PARAM/param.pck', { timeoutMs: 200, onProgress: () => progress++ });
        assert.ok(out.equals(file));
        assert.ok(progress > 10);
        assert.strictEqual(veh.reads, 0, 'no single reads needed');
        assert.strictEqual(veh.opens, 1);
        await new Promise(r => setTimeout(r, 5));
        assert.strictEqual(veh.terminates, 1);
    });

    await ok('lost burst packets are refetched with single reads', async () => {
        const { veh, client } = makePair(file, { dropEvery: 5, dropReadOnce: true });
        const out = await client.readFile('@PARAM/param.pck', { timeoutMs: 100 });
        assert.ok(out.equals(file));
        assert.ok(veh.reads >= Math.floor(veh.burstSent / 5), `reads ${veh.reads} bursts ${veh.burstSent}`);
    });

    await ok('firmware burst limit: the client asks for the next burst', async () => {
        const { veh, client } = makePair(file, { burstLimit: 10 });
        const out = await client.readFile('@PARAM/param.pck', { timeoutMs: 200 });
        assert.ok(out.equals(file));
        assert.ok(veh.burstSent >= Math.ceil(file.length / 239));
    });

    await ok('file an exact multiple of 239 bytes ends with NAK EOF', async () => {
        const exact = Buffer.alloc(239 * 7, 0xA5);
        const { client } = makePair(exact);
        const out = await client.readFile('@PARAM/param.pck', { timeoutMs: 200 });
        assert.ok(out.equals(exact));
    });

    await ok('tiny file: one short packet', async () => {
        const tiny = Buffer.from('hello');
        const { client } = makePair(tiny);
        const out = await client.readFile('@PARAM/param.pck', { timeoutMs: 200 });
        assert.ok(out.equals(tiny));
    });

    await ok('busy sessions: reset, then open again', async () => {
        const { veh, client } = makePair(file, { busyFirst: true });
        const out = await client.readFile('@PARAM/param.pck', { timeoutMs: 200 });
        assert.ok(out.equals(file));
        assert.strictEqual(veh.resets, 1);
        assert.strictEqual(veh.opens, 2);
    });

    await ok('a vehicle without FTP: NOREPLY after the retries, in bounded time', async () => {
        const { client } = makePair(file, { silent: true });
        const t0 = Date.now();
        await assert.rejects(client.readFile('@PARAM/param.pck', { timeoutMs: 50, retries: 2 }),
            e => e.code === 'NOREPLY');
        const dt = Date.now() - t0;
        assert.ok(dt >= 140 && dt < 600, `took ${dt} ms`);
    });

    await ok('unknown path: NAK FileNotFound', async () => {
        const { client } = makePair(file);
        await assert.rejects(client.readFile('@PARAM/nothing', { timeoutMs: 100 }), e => e.code === 'NAK' && /FileNotFound/.test(e.message));
    });

    await ok('cancel rejects the transfer in progress', async () => {
        const { client } = makePair(file, { delayMs: 20 });
        const p = client.readFile('@PARAM/param.pck', { timeoutMs: 500 });
        setTimeout(() => client.cancel('link closed'), 15);
        await assert.rejects(p, e => e.code === 'CANCELLED');
        assert.ok(!client.busy);
    });

    await ok('a second read while one runs is refused', async () => {
        const { client } = makePair(file, { delayMs: 10 });
        const p = client.readFile('@PARAM/param.pck', { timeoutMs: 500 });
        await assert.rejects(client.readFile('@PARAM/param.pck'), e => e.code === 'BUSY');
        await p;
    });

    console.log(process.exitCode ? 'FAILED' : `all ${passed} passed`);
})();
