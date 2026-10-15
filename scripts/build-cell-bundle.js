#!/usr/bin/env node
/**
 * build-cell-bundle.js - the LTE cell database shipped with the installer
 *
 *   node scripts/build-cell-bundle.js --ocid cell_towers.csv.gz \
 *       [--pmtiles <cell-db cache folder>] [--mls MLS-full-cell-export-final.csv.gz] \
 *       [--mcc 222,292,225] [--out assets/celldb/cells-lte.csv.gz]
 *
 * Sources, in order of preference when one cell is in more than one:
 *  --ocid     an OpenCelliD export (world file or a per-MCC file): cells seen
 *             in the last 18 months. Downloaded with an OpenCelliD token, or
 *             the dated copies at huggingface.co/datasets/yuiseki/opencellid
 *  --pmtiles  a cell-db.js cache folder holding the blocks of the countries
 *             (DOWNLOAD in SETUP, or createCellDb().downloadBox): the June
 *             2024 OpenCelliD copy, with sites unseen since 2024
 *  --mls      the final Mozilla Location Service export (March 2024, public
 *             domain; archive.org item MLS-full-cell-export-final.csv.gz):
 *             many sites OpenCelliD never had
 *
 * Output: gzipped CSV, one LTE cell per row, sorted by mcc, mnc, eci:
 *   mcc,mnc,tac,eci,lat,lon,range,samples,updated,src
 * updated in Unix seconds (0 = unknown), src one of ocid / ocid-2024 / mls.
 * Next to it manifest.json (sources, dates, counts), read by cell-db.js.
 */

const fs = require('fs');
const path = require('path');
const zlib = require('zlib');
const readline = require('readline');

function arg(name, def = null) {
    const i = process.argv.indexOf(`--${name}`);
    return i > 0 ? process.argv[i + 1] : def;
}

const OUT = arg('out', path.join(__dirname, '..', 'assets', 'celldb', 'cells-lte.csv.gz'));
const MCCS = new Set(arg('mcc', '222,292,225').split(',').map(Number));
const PMTILES_DATE = Date.parse('2024-06-15') / 1000;

const cells = new Map();     // "mcc-mnc-eci" → row
const counts = {};

function add(src, mcc, mnc, tac, eci, lat, lon, range, samples, updated) {
    if (!MCCS.has(mcc) || !(eci > 0) || !Number.isFinite(lat) || !Number.isFinite(lon)) return;
    if (lat === 0 && lon === 0) return;
    const key = `${mcc}-${mnc}-${eci}`;
    if (cells.has(key)) return;
    cells.set(key, [mcc, mnc, tac, eci, +lat.toFixed(5), +lon.toFixed(5), range | 0, samples | 0, updated | 0, src]);
    counts[src] = (counts[src] || 0) + 1;
}

/** OpenCelliD / MLS layout: radio,mcc,net,area,cell,unit,lon,lat,range,samples,changeable,created,updated,averageSignal */
async function readExport(file, src) {
    const rl = readline.createInterface({ input: fs.createReadStream(file).pipe(zlib.createGunzip()), crlfDelay: Infinity });
    let col = null, newest = 0;
    for await (const line of rl) {
        const f = line.split(',');
        if (!col) { col = Object.fromEntries(f.map((n, i) => [n.trim(), i])); continue; }
        if (f[col.radio] !== 'LTE') continue;
        const updated = +f[col.updated];
        if (updated > newest) newest = updated;
        add(src, +f[col.mcc], +f[col.net], +f[col.area], +f[col.cell], +f[col.lat], +f[col.lon],
            +f[col.range], +f[col.samples], updated);
    }
    return newest ? new Date(newest * 1000).toISOString().slice(0, 10) : null;
}

function readPmtilesCache(dir) {
    const blocks = path.join(dir, 'blocks');
    for (const f of fs.readdirSync(blocks)) {
        for (const [lat, lon, mcc, mnc, tac, eci, range, samples] of JSON.parse(fs.readFileSync(path.join(blocks, f), 'utf8'))) {
            add('ocid-2024', mcc, mnc, tac, eci, lat, lon, range, samples, PMTILES_DATE);
        }
    }
}

(async () => {
    const sources = [];
    const ocid = arg('ocid');
    if (!ocid) throw new Error('--ocid is required');
    const ocidDate = await readExport(ocid, 'ocid');
    sources.push({ src: 'ocid', name: 'OpenCelliD', date: ocidDate, licence: 'CC BY-SA 4.0', url: 'https://opencellid.org' });
    if (arg('pmtiles')) {
        readPmtilesCache(arg('pmtiles'));
        sources.push({ src: 'ocid-2024', name: 'OpenCelliD (Source Cooperative PMTiles)', date: '2024-06-15', licence: 'CC BY-SA 4.0', url: 'https://source.coop/smartmaps/opencellid' });
    }
    if (arg('mls')) {
        const mlsDate = await readExport(arg('mls'), 'mls');
        sources.push({ src: 'mls', name: 'Mozilla Location Service, final export', date: mlsDate, licence: 'Public domain (CC0)', url: 'https://archive.org/details/MLS-full-cell-export-final.csv.gz' });
    }

    const rows = [...cells.values()].sort((a, b) => a[0] - b[0] || a[1] - b[1] || a[3] - b[3]);
    const csv = 'mcc,mnc,tac,eci,lat,lon,range,samples,updated,src\n' + rows.map((r) => r.join(',')).join('\n') + '\n';
    fs.mkdirSync(path.dirname(OUT), { recursive: true });
    fs.writeFileSync(OUT, zlib.gzipSync(csv, { level: 9 }));
    const manifest = {
        file: path.basename(OUT), built: new Date().toISOString().slice(0, 10),
        mcc: [...MCCS], cells: rows.length, bySource: counts, sources,
    };
    fs.writeFileSync(path.join(path.dirname(OUT), 'manifest.json'), JSON.stringify(manifest, null, 2) + '\n');
    console.log(`${rows.length} LTE cells -> ${OUT} (${(fs.statSync(OUT).size / 1048576).toFixed(1)} MB)`, counts);
})().catch((e) => { console.error(e.message); process.exit(1); });
