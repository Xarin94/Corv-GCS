#!/usr/bin/env node
/**
 * test-cell-db.js - checks for cell-db.js and the CORV_CELL_INFO decoder
 *
 *   node scripts/test-cell-db.js            offline checks
 *   node scripts/test-cell-db.js --online   also reads the real archive around
 *                                           Modena into a temp folder (~2 MB)
 *                                           and asks beaconDB for one cell
 */

const fs = require('fs');
const os = require('os');
const path = require('path');
const zlib = require('zlib');
const { PassThrough } = require('stream');
const { pathToFileURL } = require('url');
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

    // ── CSV layers, priority, shared cells, lookups (no network) ─────────────
    {
        const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-celldb-'));
        const bundleDir = path.join(dir, 'bundle');
        fs.mkdirSync(bundleDir);
        // bundle layout: eNB 1000 of 222-88 (two sectors), eNB 2000 of 222-1, a cell of 228 (Switzerland)
        const bundle = 'mcc,mnc,tac,eci,lat,lon,range,samples,updated,src\n'
            + '222,88,100,256001,44.6500,10.9300,500,10,1700000000,mls\n'
            + '222,88,100,256002,44.6520,10.9320,500,10,1700000000,ocid\n'
            + '222,1,200,512001,44.7000,10.9000,800,5,1700000000,ocid\n'
            + '228,1,300,700001,46.0000,8.9500,800,5,1700000000,ocid\n';
        fs.writeFileSync(path.join(bundleDir, 'cells-lte.csv.gz'), zlib.gzipSync(bundle));
        fs.writeFileSync(path.join(bundleDir, 'manifest.json'), JSON.stringify({ file: 'cells-lte.csv.gz', mcc: [222], cells: 4, sources: [{ src: 'ocid', date: '2026-10-05' }] }));
        // user file, OpenCelliD layout: moves sector 1 of eNB 1000, adds a UMTS row to skip
        const user = 'radio,mcc,net,area,cell,unit,lon,lat,range,samples,changeable,created,updated,averageSignal\n'
            + 'LTE,222,88,100,256001,0,10.9400,44.6600,300,40,1,1600000000,1790000000,0\n'
            + 'UMTS,222,88,100,999,0,10.9,44.6,300,40,1,1600000000,1790000000,0\n';
        const userFile = path.join(dir, 'user-222.csv');
        fs.writeFileSync(userFile, user);

        const calls = [];
        const fakeFetch = async (u, init) => {
            calls.push(String(u));
            if (String(u).includes('source.coop')) throw new Error('PMTiles read where the bundle has cells');
            const body = JSON.parse(init.body);
            const eci = body.cellTowers[0].cellId;
            const json = eci === 768001
                ? { status: 200, body: { location: { lat: 44.66, lng: 10.95 }, accuracy: 900 } }
                : { status: 404, body: { error: { code: 404 } } };
            return { status: json.status, text: async () => JSON.stringify(json.body) };
        };
        const db = cdb.createCellDb({ dir: path.join(dir, 'cache'), bundleDir, fetch: fakeFetch });
        let r = await db.query({ lat: 44.65, lon: 10.93, radiusKm: 3, mcc: 222, mnc: 88 });
        check('bundle: one site of the network', r.sites.length === 1 && r.sites[0].enb === 1000 && r.sites[0].cells.length === 2, JSON.stringify(r.sites.map((s) => s.enb)));
        check('bundle: no PMTiles read for a bundled country', !calls.some((u) => u.includes('source.coop')));
        check('bundle: date from the manifest', r.date === '2026-10-05', r.date);

        r = await db.query({ lat: 44.65, lon: 10.93, radiusKm: 3, mcc: 222, mnc: 50, eci: 256002 });
        check('shared cell found by ECI under another MNC', r.sites.length === 1 && r.sites[0].mnc === 88
            && r.sites[0].cells.length === 1 && r.sites[0].cells[0].eci === 256002);

        await db.importCsv(userFile);
        r = await db.query({ lat: 44.65, lon: 10.93, radiusKm: 3, mcc: 222, mnc: 88 });
        const c1 = r.sites[0].cells.find((c) => c.eci === 256001);
        check('imported file wins over the bundle', c1 && c1.lat === 44.66 && c1.src === 'user-222.csv', JSON.stringify(c1));
        check('imported file: UMTS skipped', r.sites[0].cells.length === 2);
        const st = await db.status();
        check('status lists bundle and file', st.bundle.cells === 4 && st.files.length === 1 && st.files[0].cells === 1, JSON.stringify(st.files));

        // lookups: beaconDB found / not found, remembered, then part of the queries
        const hit = await db.lookup({ mcc: 222, mnc: 50, tac: 7, eci: 768001 });
        check('lookup found', hit && hit.lat === 44.66 && hit.src === 'beacondb' && hit.range === 900, JSON.stringify(hit));
        const miss = await db.lookup({ mcc: 222, mnc: 50, tac: 7, eci: 768002 });
        const n = calls.length;
        await db.lookup({ mcc: 222, mnc: 50, tac: 7, eci: 768001 });
        await db.lookup({ mcc: 222, mnc: 50, tac: 7, eci: 768002 });
        check('lookup not found, both remembered', miss === null && calls.length === n);
        r = await db.query({ lat: 44.65, lon: 10.93, radiusKm: 3, mcc: 222, mnc: 50 });
        check('looked-up cell in the next queries', r.sites.some((s) => s.enb === 3000 && s.cells[0].src === 'beacondb'));
        db.setSettings({ online: false });
        const m = calls.length;
        check('lookup off: no request', (await db.lookup({ mcc: 222, mnc: 50, tac: 7, eci: 768003 })) === null && calls.length === m);
        db.clear();
        check('clear keeps the bundle', (await db.status()).bundle.cells === 4 && (await db.status()).files.length === 0);
        fs.rmSync(dir, { recursive: true, force: true });
    }

    // ── EARFCN → frequency (renderer module) ─────────────────────────────────
    {
        // a CommonJS package: the ES module is loaded from a .mjs copy
        const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-lte-'));
        fs.copyFileSync(path.join(__dirname, '../js/core/lte-bands.js'), path.join(tmp, 'lte-bands.mjs'));
        const { earfcnToFrequency, frequencyText } = await import(pathToFileURL(path.join(tmp, 'lte-bands.mjs')));
        fs.rmSync(tmp, { recursive: true, force: true });
        const f = (e) => earfcnToFrequency(e);
        check('EARFCN 6300 = B20 806.0 MHz', f(6300).band === 20 && f(6300).mhz === 806);
        check('EARFCN 1850 = B3 1870.0 MHz', f(1850).band === 3 && f(1850).mhz === 1870);
        check('EARFCN 100 = B1 2120.0 MHz', f(100).band === 1 && f(100).mhz === 2120);
        check('EARFCN 3350 = B7 2680.0 MHz', f(3350).band === 7 && f(3350).mhz === 2680);
        check('EARFCN 9260 = B28 763.0 MHz', f(9260).band === 28 && f(9260).mhz === 763);
        check('EARFCN 3750 = B8 955.0 MHz', f(3750).band === 8 && f(3750).mhz === 955);
        check('frequency text', frequencyText(6300, 20) === '806.0 MHz' && frequencyText(0, 20) === 'B20'
            && frequencyText(0, 0, '-') === '-' && frequencyText(0, 1) === '2110.0 MHz' && frequencyText(99999, 3) === 'B3');
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
        // box of tiles, so a few cells on the edge are missing there. ~930 of
        // its rows repeat a cell (same ECI, another TAC): counted once now
        check('Modena 100 km: cells as the prototype, repeats once', cells >= 8530 && cells < 8530 * 1.01, String(cells));
        check('Modena 100 km: sites as the prototype', r.sites.length >= 3405 && r.sites.length < 3405 * 1.01, String(r.sites.length));
        check('no missing block', r.missing === 0);
        const iliad = r.sites.filter((s) => s.mnc === 50).length;
        check('Iliad sites present', iliad > 1000, String(iliad));

        // second query: from disk, no network
        const before = db.stats.requests;
        const db2 = cdb.createCellDb({ dir, fetch: () => { throw new Error('network used'); } });
        const r2 = await db2.query({ lat: 44.65, lon: 10.93, radiusKm: 100, mcc: 222, mnc: 50 });
        check('cached query works offline', r2.missing === 0 && r2.sites.length === iliad && db.stats.requests === before);
        const st = await db2.status();
        console.log(`      cache: ${st.blocks} blocks, ${st.cells} cells, ${(st.bytes / 1e6).toFixed(1)} MB`);
        fs.rmSync(dir, { recursive: true, force: true });

        // beaconDB: a TIM cell in Modena, found; the same eNB with a sector that does not exist, not found
        const dir2 = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-celldb-'));
        const db3 = cdb.createCellDb({ dir: dir2 });
        const hit = await db3.lookup({ mcc: 222, mnc: 1, tac: 18201, eci: 204853289 });
        check('beaconDB: known cell', hit && cdb.distanceKm(hit.lat, hit.lon, 44.6691, 10.9054) < 3, JSON.stringify(hit));
        check('beaconDB: unknown cell', (await db3.lookup({ mcc: 222, mnc: 1, tac: 18201, eci: 999999999 })) === null);
        fs.rmSync(dir2, { recursive: true, force: true });
    }

    console.log(failures ? `\n${failures} FAILED` : '\nall passed');
    process.exit(failures ? 1 : 0);
})();
