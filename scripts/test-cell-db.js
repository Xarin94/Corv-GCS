#!/usr/bin/env node
/**
 * test-cell-db.js - checks for cell-db.js and the CORV_CELL_INFO decoder
 *
 *   node scripts/test-cell-db.js            offline checks
 *   node scripts/test-cell-db.js --online   also reads the real archive around
 *                                           Modena into a temp folder (~2 MB)
 */

const fs = require('fs');
const os = require('os');
const path = require('path');
const { PassThrough } = require('stream');
const { MavLinkPacketParser } = require('node-mavlink');
const cdb = require('../cell-db');
const corv = require('../corv-mavlink');

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}

// ── Tile ids ─────────────────────────────────────────────────────────────────
{
    let ok = true;
    for (const [z, x, y] of [[0, 0, 0], [1, 1, 0], [3, 5, 2], [8, 136, 91], [14, 8740, 5880], [14, 16383, 0]]) {
        const [z2, x2, y2] = cdb.tileIdToZxy(cdb.zxyToTileId(z, x, y));
        if (z2 !== z || x2 !== x || y2 !== y) ok = false;
    }
    check('tile id round trip', ok);
    check('zoom 1 first id', cdb.zxyToTileId(1, 0, 0) === 1);

    // every zoom-14 child of a block falls inside its run, and the run has no gaps
    const bx = cdb.lonToX(10.93, 8), by = cdb.latToY(44.65, 8);
    const { lo, hi } = cdb.blockTileRange(bx, by);
    const sub = 1 << (cdb.TILE_Z - cdb.BLOCK_Z);
    const seen = new Set();
    for (let x = 0; x < sub; x++) for (let y = 0; y < sub; y++) {
        const id = cdb.zxyToTileId(14, bx * sub + x, by * sub + y);
        if (id >= lo && id <= hi) seen.add(id);
    }
    check('block = one contiguous run of zoom-14 ids', seen.size === sub * sub && hi - lo + 1 === sub * sub, `${seen.size}/${sub * sub}`);
    check('distance Modena-Bologna', Math.abs(cdb.distanceKm(44.6471, 10.9252, 44.4949, 11.3426) - 37) < 2);
}

// ── CORV_CELL_INFO: frame packed by pymavlink from LTEtelem tools/mavlink/corv.xml ──
(async () => {
    const hex = 'fd3f0000003344e8cb0009de571b04b500009c180000030000009c180000640000003a07000000000000'
        + 'de0032006001a1ffbaffd2003f006a01ffff9bff96ff92ffff7ff70c1404070a38';
    const input = new PassThrough();
    const parser = input.pipe(corv.createSplitter()).pipe(new MavLinkPacketParser());
    const packet = await new Promise((resolve) => {
        parser.once('data', resolve);
        setTimeout(() => resolve(null), 500);
        input.write(Buffer.from(hex, 'hex'));
    });
    check('CORV_CELL_INFO passes the splitter', !!packet && packet.header.msgid === 52200);
    if (packet) {
        const m = packet.protocol.data(packet.payload, corv.CorvCellInfo);
        check('cell id', m.eci === 0x1B57DE09 && m.mcc === 222 && m.mnc === 50 && m.tac === 0xB504);
        check('radio', m.pci === 352 && m.earfcn === 6300 && m.band === 20 && m.state === 4);
        check('signal', m.rsrp === -95 && m.rssi === -70 && m.rsrq === -9 && m.sinr === 12);
        check('neighbours', m.nNeighbours === 7 && m.nbPci.join() === '210,63,362,65535'
            && m.nbRsrp.join() === '-101,-106,-110,32767' && m.nbEarfcn.join() === '6300,100,1850,0');
        check('handovers', m.handovers === 3);
    }

    if (process.argv.includes('--online')) {
        const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-celldb-'));
        const db = cdb.createCellDb({ dir });
        const t0 = Date.now();
        const r = await db.query({ lat: 44.65, lon: 10.93, radiusKm: 100, mcc: 222 });
        const cells = r.sites.reduce((n, s) => n + s.cells.length, 0);
        console.log(`      ${r.sites.length} sites, ${cells} cells, ${db.stats.requests} requests, `
            + `${(db.stats.bytes / 1e6).toFixed(2)} MB, ${((Date.now() - t0) / 1000).toFixed(1)} s, data ${r.date}`);
        // the Python prototype (own tile reader, same 100 km by haversine) found
        // 9473 LTE cells of MCC 222 on 3405 eNB sites; it read a slightly smaller
        // box of tiles, so a few cells on the edge are missing there
        check('Modena 100 km: cells as the prototype', cells >= 9473 && cells < 9473 * 1.01, String(cells));
        check('Modena 100 km: sites as the prototype', r.sites.length >= 3405 && r.sites.length < 3405 * 1.01, String(r.sites.length));
        check('no missing block', r.missing === 0);
        const iliad = r.sites.filter((s) => s.mnc === 50).length;
        check('Iliad sites present', iliad > 1000, String(iliad));

        // second query: from disk, no network
        const before = db.stats.requests;
        const db2 = cdb.createCellDb({ dir, fetch: () => { throw new Error('network used'); } });
        const r2 = await db2.query({ lat: 44.65, lon: 10.93, radiusKm: 100, mcc: 222, mnc: 50 });
        check('cached query works offline', r2.missing === 0 && r2.sites.length === iliad && db.stats.requests === before);
        const st = db2.status();
        console.log(`      cache: ${st.blocks} blocks, ${st.cells} cells, ${(st.bytes / 1e6).toFixed(1)} MB`);
        fs.rmSync(dir, { recursive: true, force: true });
    }

    console.log(failures ? `\n${failures} FAILED` : '\nall passed');
    process.exit(failures ? 1 : 0);
})();
