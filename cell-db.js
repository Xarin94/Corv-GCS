/**
 * cell-db.js - LTE cell sites around the vehicle
 *
 * Layers, first wins when a cell (mcc, mnc, ECI) is in more than one:
 *  1. CSV files the user added (<data root>/celldb/sources): an OpenCelliD
 *     country file downloaded with the user's token (DOWNLOAD FROM
 *     OPENCELLID), or any OpenCelliD / MLS layout CSV (IMPORT CSV)
 *  2. the CSV shipped with the installer (assets/celldb, built by
 *     scripts/build-cell-bundle.js): OpenCelliD of the build date, the June
 *     2024 OpenCelliD copy and the final Mozilla Location Service export
 *  3. the OpenCelliD copy published as one PMTiles file by UN Smart Maps on
 *     Source Cooperative (June 2024; public, no account), read around the
 *     vehicle with HTTP range requests, never the whole 560 MB file
 *  4. single cells found online (beaconDB, or the OpenCelliD API with a
 *     token) when the serving cell is in none of the above (lookup)
 *
 * PMTiles storage unit: one zoom-8 tile (a "block", ~110 km at 45° N). Its
 * zoom-14 tiles — the only zoom that holds every cell — are one contiguous run
 * of tile ids in the Hilbert order of PMTiles, and contiguous on disk because
 * the archive is clustered, so a block costs a directory lookup and one or
 * two range requests. LTE cells of a block are kept as JSON under
 * <data root>/celldb/blocks; once there they work offline. CSV layers are
 * indexed by the same blocks.
 *
 * Positions are estimates (averaged crowd measurements), not the surveyed
 * mast position: off by hundreds of metres, sometimes more. OpenCelliD is
 * CC BY-SA 4.0 (attribution "OpenCelliD"), MLS and beaconDB public domain.
 */

const fs = require('fs');
const path = require('path');
const zlib = require('zlib');
const readline = require('readline');

const SOURCE_URL = 'https://data.source.coop/smartmaps/opencellid/cellid.pmtiles';
const USER_AGENT = 'Corv-GCS cell-db';   // the CDN refuses requests without one; beaconDB asks for one
const BEACONDB_URL = 'https://api.beacondb.net/v1/geolocate';
const OCID_CELL_URL = 'https://opencellid.org/cell/get';
const OCID_DOWNLOAD_URL = 'https://opencellid.org/ocid/downloads';
const LOOKUP_MISS_MS = 6 * 3600 * 1000;  // a cell not found online is asked again after this
const LOOKUP_TIMEOUT_MS = 10000;
const TILE_Z = 14;            // maximum zoom of the archive: no feature dropped
const BLOCK_Z = 8;
const SUB = 1 << (TILE_Z - BLOCK_Z);     // zoom-14 tiles per block side (64)
const MERGE_GAP = 256 * 1024;            // read across gaps smaller than this...
const MAX_RANGE = 16 * 1024 * 1024;      // ...up to this many bytes per request
const MEM_BLOCKS = 48;                   // decoded blocks kept in memory

// ── Tiles ────────────────────────────────────────────────────────────────────

function lonToX(lon, z) { return Math.floor((lon + 180) / 360 * (1 << z)); }
function latToY(lat, z) {
    const r = lat * Math.PI / 180;
    return Math.floor((1 - Math.asinh(Math.tan(r)) / Math.PI) / 2 * (1 << z));
}
function tileLon(x, z) { return x / (1 << z) * 360 - 180; }
function tileLat(y, z) { return Math.atan(Math.sinh(Math.PI * (1 - 2 * y / (1 << z)))) * 180 / Math.PI; }

function rotate(n, x, y, rx, ry) {
    if (ry === 0) {
        if (rx === 1) { x = n - 1 - x; y = n - 1 - y; }
        return [y, x];
    }
    return [x, y];
}

/** PMTiles v3 tile id: tiles of all lower zooms first, then the Hilbert index. */
function zxyToTileId(z, x, y) {
    let acc = 0;
    for (let t = 0; t < z; t++) acc += 4 ** t;
    const n = 1 << z;
    let d = 0;
    for (let s = n >> 1; s > 0; s >>= 1) {
        const rx = (x & s) ? 1 : 0;
        const ry = (y & s) ? 1 : 0;
        d += s * s * ((3 * rx) ^ ry);
        [x, y] = rotate(n, x, y, rx, ry);
    }
    return acc + d;
}

function tileIdToZxy(id) {
    let acc = 0;
    for (let z = 0; z < 27; z++) {
        const count = 4 ** z;
        if (id < acc + count) {
            let t = id - acc, x = 0, y = 0;
            for (let s = 1; s < (1 << z); s *= 2) {
                const rx = 1 & (t / 2);
                const ry = 1 & (t ^ rx);
                [x, y] = rotate(s, x, y, rx, ry);
                x += s * rx;
                y += s * ry;
                t = Math.floor(t / 4);
            }
            return [z, x, y];
        }
        acc += count;
    }
    throw new Error(`tile id ${id} out of range`);
}

const firstId = (z) => (4 ** z - 1) / 3;   // tiles of all lower zooms

/**
 * Zoom-14 tile ids of a block: the Hilbert index of a child tile is the
 * parent's index followed by the index inside the parent, so they are one run.
 */
function blockTileRange(bx, by) {
    const d = zxyToTileId(BLOCK_Z, bx, by) - firstId(BLOCK_Z);
    const lo = firstId(TILE_Z) + d * SUB * SUB;
    return { lo, hi: lo + SUB * SUB - 1 };
}

// ── PMTiles directories ──────────────────────────────────────────────────────

function readVarint(buf, pos) {
    let result = 0, shift = 1, b;
    do {
        b = buf[pos.i++];
        result += (b & 0x7f) * shift;   // arithmetic, not <<: offsets pass 2^31
        shift *= 128;
    } while (b & 0x80);
    return result;
}

function parseDirectory(gz) {
    const buf = zlib.gunzipSync(gz);
    const pos = { i: 0 };
    const n = readVarint(buf, pos);
    const entries = new Array(n);
    let last = 0;
    for (let k = 0; k < n; k++) { last += readVarint(buf, pos); entries[k] = { tileId: last, runLength: 0, length: 0, offset: 0 }; }
    for (let k = 0; k < n; k++) entries[k].runLength = readVarint(buf, pos);
    for (let k = 0; k < n; k++) entries[k].length = readVarint(buf, pos);
    for (let k = 0; k < n; k++) {
        const v = readVarint(buf, pos);
        entries[k].offset = (v === 0 && k > 0) ? entries[k - 1].offset + entries[k - 1].length : v - 1;
    }
    return entries;
}

function parseHeader(buf) {
    if (buf.toString('latin1', 0, 7) !== 'PMTiles' || buf[7] !== 3) throw new Error('not a PMTiles v3 archive');
    const u64 = (o) => Number(buf.readBigUInt64LE(o));
    return {
        rootOffset: u64(8), rootLength: u64(16),
        leafOffset: u64(40), leafLength: u64(48),
        dataOffset: u64(56), dataLength: u64(64),
        clustered: buf[96] === 1, internalCompression: buf[97], tileCompression: buf[98],
        tileType: buf[99], minZoom: buf[100], maxZoom: buf[101],
    };
}

// ── Vector tile (MVT) decoding: just enough for point features ────────────────

function pbfFields(buf, start, end, cb) {
    const pos = { i: start };
    while (pos.i < end) {
        const key = readVarint(buf, pos);
        const field = Math.floor(key / 8), wire = key & 7;
        if (wire === 0) cb(field, wire, readVarint(buf, pos));
        else if (wire === 2) { const len = readVarint(buf, pos); cb(field, wire, pos.i, pos.i + len); pos.i += len; }
        else if (wire === 1) { cb(field, wire, pos.i); pos.i += 8; }
        else if (wire === 5) { cb(field, wire, pos.i); pos.i += 4; }
        else throw new Error(`unsupported protobuf wire type ${wire}`);
    }
}

function packedVarints(buf, start, end) {
    const out = [], pos = { i: start };
    while (pos.i < end) out.push(readVarint(buf, pos));
    return out;
}

const zigzag = (v) => (v % 2 ? -(v + 1) / 2 : v / 2);

function decodeValue(buf, start, end) {
    let value = null;
    pbfFields(buf, start, end, (f, w, a, b) => {
        if (f === 1) value = buf.toString('utf8', a, b);
        else if (f === 2) value = buf.readFloatLE(a);
        else if (f === 3) value = buf.readDoubleLE(a);
        else if (f === 4 || f === 5) value = a;
        else if (f === 6) value = zigzag(a);
        else if (f === 7) value = !!a;
    });
    return value;
}

/** LTE cells of one zoom-14 tile as rows [lat, lon, mcc, mnc, tac, eci, range, samples]. */
function decodeTile(buf, x, y, out) {
    pbfFields(buf, 0, buf.length, (field, wire, ls, le) => {
        if (field !== 3 || wire !== 2) return;            // layers
        const keys = [], values = [], features = [];
        let extent = 4096;
        pbfFields(buf, ls, le, (f, w, a, b) => {
            if (f === 3) keys.push(buf.toString('utf8', a, b));
            else if (f === 4) values.push(decodeValue(buf, a, b));
            else if (f === 2) features.push([a, b]);
            else if (f === 5) extent = a;
        });
        for (const [fs0, fe] of features) {
            let tags = [], geom = [];
            pbfFields(buf, fs0, fe, (f, w, a, b) => {
                if (f === 2) tags = packedVarints(buf, a, b);
                else if (f === 4) geom = packedVarints(buf, a, b);
            });
            if (geom.length < 3) continue;
            const p = {};
            for (let k = 0; k + 1 < tags.length; k += 2) p[keys[tags[k]]] = values[tags[k + 1]];
            if (p.radio !== 'LTE' || !p.cell) continue;
            const n = 1 << TILE_Z;
            const px = x + zigzag(geom[1]) / extent, py = y + zigzag(geom[2]) / extent;
            const lon = px / n * 360 - 180;
            const lat = Math.atan(Math.sinh(Math.PI * (1 - 2 * py / n))) * 180 / Math.PI;
            out.push([+lat.toFixed(6), +lon.toFixed(6), p.mcc | 0, p.net | 0, p.area | 0, p.cell, p.range | 0, p.samples | 0]);
        }
    });
}

// ── CSV layers ───────────────────────────────────────────────────────────────

const blockKey = (bx, by) => bx * (1 << BLOCK_Z) + by;
// header names of the bundle layout and of the OpenCelliD / MLS one
const CSV_COLUMNS = {
    mcc: ['mcc'], mnc: ['mnc', 'net'], tac: ['tac', 'area', 'lac'], eci: ['eci', 'cell', 'cellid'],
    lat: ['lat'], lon: ['lon'], range: ['range'], samples: ['samples'], updated: ['updated'],
};

/**
 * LTE cells of a CSV file (gzipped or not) in columns, ordered by block so a
 * block is one slice: { at: Map(blockKey → [start, end]), lat, lon, ... }.
 * Read as a stream: a 40 MB file does not stall the main process.
 * @param {string} file
 * @param {string} name   source name for cells without a src column
 */
async function loadCsvLayer(file, name) {
    const head = Buffer.alloc(2);
    const fd = fs.openSync(file, 'r');
    try { fs.readSync(fd, head, 0, 2, 0); } finally { fs.closeSync(fd); }
    let input = fs.createReadStream(file);
    if (head[0] === 0x1f && head[1] === 0x8b) input = input.pipe(zlib.createGunzip());
    const rl = readline.createInterface({ input, crlfDelay: Infinity });

    let cap = 1 << 16, n = 0;
    let lat = new Float32Array(cap), lon = new Float32Array(cap), num = new Int32Array(cap * 6), key = new Int32Array(cap);
    const srcNames = [name], srcIndex = new Map([[name, 0]]);
    let src = new Uint8Array(cap);
    let col = null, newest = 0;
    for await (const line of rl) {
        if (!line) continue;
        const f = line.split(',');
        if (!col) {
            const names = f.map((s) => s.trim().toLowerCase());
            col = {};
            for (const [k, alts] of Object.entries(CSV_COLUMNS)) col[k] = names.findIndex((h) => alts.includes(h));
            col.radio = names.indexOf('radio');
            col.src = names.indexOf('src');
            for (const k of ['mcc', 'mnc', 'eci', 'lat', 'lon']) {
                if (col[k] < 0) throw new Error(`${path.basename(file)}: no "${CSV_COLUMNS[k][0]}" column`);
            }
            continue;
        }
        if (col.radio >= 0 && f[col.radio] !== 'LTE') continue;
        const la = +f[col.lat], lo = +f[col.lon], eci = +f[col.eci];
        if (!Number.isFinite(la) || !Number.isFinite(lo) || (la === 0 && lo === 0) || !(eci > 0) || Math.abs(la) > 85) continue;
        if (n === cap) {
            cap *= 2;
            const grow = (a, k = 1) => { const b = new a.constructor(cap * k); b.set(a); return b; };
            lat = grow(lat); lon = grow(lon); num = grow(num, 6); key = grow(key); src = grow(src);
        }
        const updated = col.updated >= 0 ? +f[col.updated] || 0 : 0;
        if (updated > newest) newest = updated;
        lat[n] = la; lon[n] = lo;
        num.set([+f[col.mcc] | 0, +f[col.mnc] | 0, col.tac >= 0 ? +f[col.tac] | 0 : 0, eci,
            col.range >= 0 ? +f[col.range] | 0 : 0, col.samples >= 0 ? +f[col.samples] | 0 : 0], n * 6);
        key[n] = blockKey(lonToX(lo, BLOCK_Z), latToY(la, BLOCK_Z));
        if (col.src >= 0 && f[col.src]) {
            let s = srcIndex.get(f[col.src]);
            if (s === undefined) { s = srcNames.length; srcNames.push(f[col.src]); srcIndex.set(f[col.src], s); }
            src[n] = s;
        }
        n++;
    }
    // counting sort by block
    const blocks = 1 << (2 * BLOCK_Z);
    const start = new Int32Array(blocks + 1);
    for (let i = 0; i < n; i++) start[key[i] + 1]++;
    for (let b = 0; b < blocks; b++) start[b + 1] += start[b];
    const fill = start.slice(0, blocks), order = new Int32Array(n);
    for (let i = 0; i < n; i++) order[fill[key[i]]++] = i;
    const layer = {
        name, file, cells: n, newest: newest ? new Date(newest * 1000).toISOString().slice(0, 10) : null,
        lat: new Float32Array(n), lon: new Float32Array(n), num: new Int32Array(n * 6), src: new Uint8Array(n), srcNames,
        at: new Map(),
    };
    for (let j = 0; j < n; j++) {
        const i = order[j];
        layer.lat[j] = lat[i]; layer.lon[j] = lon[i]; layer.src[j] = src[i];
        layer.num.set(num.subarray(i * 6, i * 6 + 6), j * 6);
    }
    for (let b = 0; b < blocks; b++) if (start[b + 1] > start[b]) layer.at.set(b, [start[b], start[b + 1]]);
    return layer;
}

/** Cells of one block of a CSV layer as rows [lat, lon, mcc, mnc, tac, eci, range, samples, src]. */
function layerRows(layer, bx, by) {
    const r = layer.at.get(blockKey(bx, by));
    if (!r) return [];
    const out = new Array(r[1] - r[0]);
    for (let j = r[0]; j < r[1]; j++) {
        const k = j * 6, num = layer.num;
        // float32 keeps ~0.3 m: round back to the 5 decimals of the files
        out[j - r[0]] = [Math.round(layer.lat[j] * 1e5) / 1e5, Math.round(layer.lon[j] * 1e5) / 1e5, num[k], num[k + 1], num[k + 2], num[k + 3], num[k + 4], num[k + 5], layer.srcNames[layer.src[j]]];
    }
    return out;
}

// ── Database ─────────────────────────────────────────────────────────────────

function distanceKm(lat1, lon1, lat2, lon2) {
    const r = Math.PI / 180;
    const a = Math.sin((lat2 - lat1) * r / 2) ** 2
        + Math.cos(lat1 * r) * Math.cos(lat2 * r) * Math.sin((lon2 - lon1) * r / 2) ** 2;
    return 12742 * Math.asin(Math.sqrt(a));
}

/**
 * @param {object} opts
 * @param {string} opts.dir   cache folder
 * @param {string} [opts.bundleDir] folder of the CSV shipped with the installer
 * @param {string} [opts.url] PMTiles archive
 * @param {Function} [opts.fetch] fetch implementation (tests)
 */
function createCellDb({ dir, bundleDir = null, url = SOURCE_URL, fetch: fetchImpl = globalThis.fetch }) {
    const blocksDir = path.join(dir, 'blocks');
    const leafDir = path.join(dir, 'leaves');
    const sourcesDir = path.join(dir, 'sources');
    const indexFile = path.join(dir, 'index.json');
    const lookupFile = path.join(dir, 'lookups.json');
    const settingsFile = path.join(dir, 'settings.json');
    let index = null;           // { etag, date, blocks: { "x_y": cells } }
    let layers = null;          // Promise of the CSV layers, user files first
    let lookups = null;         // "mcc-mnc-eci" → { lat, lon, range, src, tac, at } | { miss: true, at }
    const lookupsInFlight = new Map();
    let header = null, root = null;
    const leaves = new Map();   // leaf offset → entries
    const memBlocks = new Map(); // "x_y" → rows (insertion order = LRU)
    const inFlight = new Map(); // "x_y" → Promise
    let stats = { requests: 0, bytes: 0 };

    function loadIndex() {
        if (index) return index;
        try { index = JSON.parse(fs.readFileSync(indexFile, 'utf8')); } catch (e) { index = null; }
        if (!index || typeof index.blocks !== 'object') index = { etag: null, date: null, blocks: {} };
        return index;
    }
    function writeJson(file, value) {
        fs.mkdirSync(dir, { recursive: true });
        const tmp = file + '.tmp';
        fs.writeFileSync(tmp, JSON.stringify(value));
        fs.renameSync(tmp, file);
    }
    function readJson(file) {
        try { return JSON.parse(fs.readFileSync(file, 'utf8')); } catch (e) { return null; }
    }
    function saveIndex() { writeJson(indexFile, index); }

    // online: look the serving cell up when no layer has it; token: OpenCelliD API key
    function getSettings() { return { online: true, token: '', ...readJson(settingsFile) }; }

    function bundleManifest() {
        return bundleDir ? readJson(path.join(bundleDir, 'manifest.json')) : null;
    }

    function sourceFiles() {
        try {
            return fs.readdirSync(sourcesDir).filter((f) => /\.csv(\.gz)?$/i.test(f))
                .map((f) => ({ f, mtime: fs.statSync(path.join(sourcesDir, f)).mtimeMs }))
                .sort((a, b) => b.mtime - a.mtime).map((x) => x.f);
        } catch (e) { return []; }
    }

    /** CSV layers, loaded once: the user's files (newest first), then the bundle. */
    function loadLayers() {
        if (layers) return layers;
        layers = (async () => {
            const out = [];
            const t0 = Date.now();
            for (const f of sourceFiles()) {
                try { out.push(await loadCsvLayer(path.join(sourcesDir, f), f)); } catch (e) {
                    console.warn(`[celldb] ${f}: ${e.message}`);
                }
            }
            const manifest = bundleManifest();
            if (manifest) {
                try {
                    const layer = await loadCsvLayer(path.join(bundleDir, manifest.file), 'bundle');
                    layer.bundle = true;
                    out.push(layer);
                } catch (e) { console.warn(`[celldb] bundled cells: ${e.message}`); }
            }
            if (out.length) console.log(`[celldb] ${out.map((l) => `${l.name} ${l.cells}`).join(', ')} LTE cells loaded in ${Date.now() - t0} ms`);
            return out;
        })();
        return layers;
    }

    function loadLookups() {
        if (!lookups) lookups = readJson(lookupFile) || {};
        return lookups;
    }

    async function fetchJson(u, init) {
        const ctl = new AbortController();
        const timer = setTimeout(() => ctl.abort(), LOOKUP_TIMEOUT_MS);
        try {
            const res = await fetchImpl(u, { ...init, signal: ctl.signal });
            const text = await res.text();
            let body = null;
            try { body = JSON.parse(text); } catch (e) { /* not JSON */ }
            return { status: res.status, body, text };
        } finally { clearTimeout(timer); }
    }

    /** Cell position from the OpenCelliD API (token) or beaconDB; null if unknown, throws when offline. */
    async function lookupOnline({ mcc, mnc, tac, eci }, token) {
        if (token) {
            const q = new URLSearchParams({ key: token, mcc, mnc, lac: tac, cellid: eci, radio: 'LTE', format: 'json' });
            const r = await fetchJson(`${OCID_CELL_URL}?${q}`, { headers: { 'User-Agent': USER_AGENT } });
            const b = r.body;
            if (b && Number.isFinite(b.lat) && Number.isFinite(b.lon) && (b.lat || b.lon)) {
                return { lat: b.lat, lon: b.lon, range: b.range | 0, samples: b.samples | 0, src: 'ocid-api' };
            }
            // anything but "not found" (bad token, daily credits used up): say so, then try beaconDB
            if (r.status !== 200 || (b && b.error && !/not\s*found/i.test(b.error))) {
                console.warn(`[celldb] OpenCelliD API: HTTP ${r.status} ${b && b.error ? b.error : r.text.slice(0, 80)}`);
            }
        }
        const r = await fetchJson(BEACONDB_URL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json', 'User-Agent': USER_AGENT },
            // no IP or area fallback: only the cell itself, else beaconDB answers with the GCS position
            body: JSON.stringify({
                considerIp: false, fallbacks: { lacf: false, ipf: false },
                cellTowers: [{ radioType: 'lte', mobileCountryCode: mcc, mobileNetworkCode: mnc, locationAreaCode: tac, cellId: eci }],
            }),
        });
        const loc = r.body && r.body.location;
        if (r.status === 200 && loc && Number.isFinite(loc.lat) && Number.isFinite(loc.lng)) {
            return { lat: loc.lat, lon: loc.lng, range: Math.round(r.body.accuracy || 0), samples: 0, src: 'beacondb' };
        }
        if (r.status === 404) return null;
        throw new Error(`beaconDB: HTTP ${r.status}`);
    }

    async function range(offset, length) {
        const res = await fetchImpl(url, { headers: { Range: `bytes=${offset}-${offset + length - 1}`, 'User-Agent': USER_AGENT } });
        // 200 = the server ignored Range and is sending the whole archive
        if (res.status !== 206) {
            if (res.body && res.body.cancel) res.body.cancel().catch(() => {});
            throw new Error(`cell database: HTTP ${res.status}`);
        }
        const buf = Buffer.from(await res.arrayBuffer());
        stats.requests++;
        stats.bytes += buf.length;
        return { buf, res };
    }

    /** Header and root directory, once per session; a new archive drops the leaf cache. */
    async function open() {
        if (header) return;
        const r = await range(0, 16384);
        const buf = r.buf;
        header = parseHeader(buf);
        if (header.tileType !== 1 || header.tileCompression !== 2 || header.internalCompression !== 2 || !header.clustered) {
            throw new Error('cell database: unexpected archive layout');
        }
        if (header.maxZoom !== TILE_Z) throw new Error(`cell database: max zoom ${header.maxZoom}, expected ${TILE_Z}`);
        const rootBytes = header.rootOffset + header.rootLength <= buf.length
            ? buf.subarray(header.rootOffset, header.rootOffset + header.rootLength)
            : (await range(header.rootOffset, header.rootLength)).buf;
        root = parseDirectory(rootBytes);
        const etag = r.res.headers.get('etag');
        const modified = r.res.headers.get('last-modified');
        loadIndex();
        if (etag && index.etag && etag !== index.etag) {
            fs.rmSync(leafDir, { recursive: true, force: true });
            leaves.clear();
            console.log('[celldb] the archive changed: blocks downloaded from now on are from the new copy');
        }
        index.etag = etag || index.etag;
        if (modified) index.date = new Date(modified).toISOString().slice(0, 10);
        saveIndex();
    }

    async function leaf(offset, length) {
        if (leaves.has(offset)) return leaves.get(offset);
        const file = path.join(leafDir, `${offset}.gz`);
        let gz = null;
        try { gz = fs.readFileSync(file); } catch (e) { /* not cached */ }
        if (!gz || gz.length !== length) {
            gz = (await range(header.leafOffset + offset, length)).buf;
            fs.mkdirSync(leafDir, { recursive: true });
            fs.writeFileSync(file, gz);
        }
        const entries = parseDirectory(gz);
        leaves.set(offset, entries);
        return entries;
    }

    /** Directory entries with tile data that overlap tile ids lo..hi. */
    async function entriesIn(entries, lo, hi, out, depth = 0) {
        if (depth > 4) throw new Error('cell database: directory too deep');
        for (let k = 0; k < entries.length; k++) {
            const e = entries[k];
            const end = e.runLength > 0 ? e.tileId + e.runLength
                : (k + 1 < entries.length ? entries[k + 1].tileId : Infinity);
            if (end <= lo || e.tileId > hi) continue;
            if (e.runLength === 0) await entriesIn(await leaf(e.offset, e.length), lo, hi, out, depth + 1);
            else out.push(e);
        }
        return out;
    }

    async function downloadBlock(bx, by) {
        await open();
        const { lo, hi } = blockTileRange(bx, by);
        const entries = await entriesIn(root, lo, hi, []);
        // tile data → the tile ids that use it (a run repeats one tile)
        const pieces = [];
        for (const e of entries) {
            const ids = [];
            for (let t = Math.max(e.tileId, lo); t < Math.min(e.tileId + e.runLength, hi + 1); t++) ids.push(t);
            if (ids.length) pieces.push({ offset: e.offset, length: e.length, ids });
        }
        pieces.sort((a, b) => a.offset - b.offset);
        const rows = [];
        for (let i = 0; i < pieces.length;) {
            const start = pieces[i].offset;
            let end = start + pieces[i].length, j = i + 1;
            while (j < pieces.length && pieces[j].offset - end < MERGE_GAP && pieces[j].offset + pieces[j].length - start <= MAX_RANGE) {
                end = Math.max(end, pieces[j].offset + pieces[j].length);
                j++;
            }
            const { buf } = await range(header.dataOffset + start, end - start);
            for (let k = i; k < j; k++) {
                const p = pieces[k];
                const tile = zlib.gunzipSync(buf.subarray(p.offset - start, p.offset - start + p.length));
                for (const id of p.ids) {
                    const [, x, y] = tileIdToZxy(id);
                    decodeTile(tile, x, y, rows);
                }
            }
            i = j;
        }
        fs.mkdirSync(blocksDir, { recursive: true });
        fs.writeFileSync(path.join(blocksDir, `${bx}_${by}.json`), JSON.stringify(rows));
        loadIndex().blocks[`${bx}_${by}`] = rows.length;
        saveIndex();
        return rows;
    }

    function remember(key, rows) {
        memBlocks.delete(key);
        memBlocks.set(key, rows);
        while (memBlocks.size > MEM_BLOCKS) memBlocks.delete(memBlocks.keys().next().value);
        return rows;
    }

    /** Rows of a block: memory, then disk, then (download = true) the network; null if missing. */
    async function block(bx, by, download) {
        const key = `${bx}_${by}`;
        if (memBlocks.has(key)) return remember(key, memBlocks.get(key));
        if (loadIndex().blocks[key] !== undefined) {
            try {
                return remember(key, JSON.parse(fs.readFileSync(path.join(blocksDir, `${key}.json`), 'utf8')));
            } catch (e) { /* listed but unreadable: fetch again */ }
        }
        if (!download) return null;
        if (!inFlight.has(key)) {
            inFlight.set(key, downloadBlock(bx, by).finally(() => inFlight.delete(key)));
        }
        return remember(key, await inFlight.get(key));
    }

    function blocksForBox(south, west, north, east) {
        const out = [];
        const x0 = lonToX(west, BLOCK_Z), x1 = lonToX(east, BLOCK_Z);
        const y0 = latToY(north, BLOCK_Z), y1 = latToY(south, BLOCK_Z);
        for (let x = x0; x <= x1; x++) for (let y = y0; y <= y1; y++) out.push([x, y]);
        return out;
    }

    function blocksForRadius(lat, lon, radiusKm) {
        const dLat = radiusKm / 111.2;
        const dLon = radiusKm / (111.2 * Math.max(0.05, Math.cos(lat * Math.PI / 180)));
        return blocksForBox(lat - dLat, lon - dLon, lat + dLat, lon + dLon);
    }

    return {
        /**
         * Cell sites (cells of one eNB, by operator) within radiusKm. With eci,
         * the cell of that ECI in the same country is kept whatever its network:
         * a cell shared by two operators (RAN sharing) is often in the
         * databases under the other one's MNC.
         * The PMTiles copy is read (and downloaded) only where no CSV layer
         * has cells: it is older than the bundle, which already holds it.
         * @returns {{sites: object[], missing: number, date: string|null}}
         */
        async query({ lat, lon, radiusKm = 100, mcc = null, mnc = null, eci: servingEci = null, download = true }) {
            const csv = await loadLayers();
            const looked = Object.entries(loadLookups()).filter(([, v]) => !v.miss)
                .map(([k, v]) => { const [m, n, e] = k.split('-').map(Number); return [v.lat, v.lon, m, n, v.tac | 0, e, v.range, v.samples, v.src]; });
            const sites = new Map();
            const seen = new Set();
            let missing = 0;
            for (const [bx, by] of blocksForRadius(lat, lon, radiusKm)) {
                const sets = csv.map((l) => layerRows(l, bx, by)).filter((r) => r.length);
                const bundled = csv.some((l) => l.bundle && l.at.has(blockKey(bx, by)))
                    && mcc !== null && ((bundleManifest() || {}).mcc || []).includes(mcc);
                if (!bundled) {
                    let rows;
                    try { rows = await block(bx, by, download); } catch (e) {
                        console.warn(`[celldb] block ${bx}/${by}: ${e.message}`);
                        rows = null;
                    }
                    if (rows) sets.push(rows.map((r) => [...r, 'ocid-2024']));
                    else if (!sets.length) missing++;
                }
                sets.push(looked.filter(([cLat, cLon]) => lonToX(cLon, BLOCK_Z) === bx && latToY(cLat, BLOCK_Z) === by));
                for (const rows of sets) {
                    for (const [cLat, cLon, cMcc, cMnc, tac, eci, rng, samples, src] of rows) {
                        if ((mcc !== null && cMcc !== mcc) || (mnc !== null && cMnc !== mnc && eci !== servingEci)) continue;
                        const cellKey = `${cMcc}-${cMnc}-${eci}`;
                        if (seen.has(cellKey)) continue;
                        seen.add(cellKey);
                        if (distanceKm(lat, lon, cLat, cLon) > radiusKm) continue;
                        const enb = Math.floor(eci / 256);
                        const key = `${cMcc}-${cMnc}-${enb}`;
                        let s = sites.get(key);
                        if (!s) sites.set(key, s = { mcc: cMcc, mnc: cMnc, enb, lat: 0, lon: 0, w: 0, cells: [] });
                        const w = Math.max(1, samples);
                        s.lat += cLat * w; s.lon += cLon * w; s.w += w;
                        s.cells.push({ eci, tac, lat: cLat, lon: cLon, range: rng, samples, src });
                    }
                }
            }
            const list = [];
            for (const s of sites.values()) {
                list.push({ mcc: s.mcc, mnc: s.mnc, enb: s.enb, lat: s.lat / s.w, lon: s.lon / s.w, cells: s.cells });
            }
            const user = csv.find((l) => !l.bundle);
            const date = (user && user.newest) || (bundleManifest() || {}).sources?.[0]?.date || loadIndex().date;
            return { sites: list, missing, date };
        },

        /**
         * One cell looked up online (beaconDB; the OpenCelliD API first with a
         * token), remembered on disk: found for good, not found for 6 hours.
         * @returns {Promise<object|null>} { lat, lon, range, src } or null
         */
        async lookup({ mcc, mnc, tac, eci }) {
            const key = `${mcc}-${mnc}-${eci}`;
            const hit = loadLookups()[key];
            if (hit && !hit.miss) return hit;
            if (hit && Date.now() - hit.at < LOOKUP_MISS_MS) return null;
            const { online, token } = getSettings();
            if (!online) return null;
            if (!lookupsInFlight.has(key)) {
                lookupsInFlight.set(key, (async () => {
                    const found = await lookupOnline({ mcc, mnc, tac, eci }, token);
                    const entry = found ? { ...found, tac, at: Date.now() } : { miss: true, at: Date.now() };
                    loadLookups()[key] = entry;
                    writeJson(lookupFile, lookups);
                    console.log(`[celldb] lookup ${key}: ${found ? `${found.src} ${found.lat.toFixed(5)}, ${found.lon.toFixed(5)} ±${found.range} m` : 'not found'}`);
                    return found ? entry : null;
                })().finally(() => lookupsInFlight.delete(key)));
            }
            return lookupsInFlight.get(key);
        },

        /**
         * The OpenCelliD file of one country (MCC), with the user's token:
         * cells seen in the last 18 months, refreshed every day by OpenCelliD
         * (each file can be downloaded twice a day). Replaces the bundle
         * there, as the newest layer.
         */
        async downloadOpenCellid(mcc) {
            const { token } = getSettings();
            if (!token) throw new Error('no OpenCelliD token');
            if (!/^\d{3}$/.test(String(mcc))) throw new Error(`bad MCC ${mcc}`);
            const q = new URLSearchParams({ token, type: 'mcc', file: `${mcc}.csv.gz` });
            const res = await fetchImpl(`${OCID_DOWNLOAD_URL}?${q}`, { headers: { 'User-Agent': USER_AGENT } });
            const buf = Buffer.from(await res.arrayBuffer());
            if (res.status !== 200 || buf[0] !== 0x1f || buf[1] !== 0x8b) {
                // OpenCelliD answers errors (bad token, limit reached) as text
                throw new Error(`OpenCelliD: ${buf.toString('utf8', 0, 200).replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').trim() || `HTTP ${res.status}`}`);
            }
            fs.mkdirSync(sourcesDir, { recursive: true });
            const file = path.join(sourcesDir, `opencellid-${mcc}.csv.gz`);
            fs.writeFileSync(file + '.tmp', buf);
            fs.renameSync(file + '.tmp', file);
            layers = null;
            const loaded = await loadLayers();
            const layer = loaded.find((l) => l.file === file);
            if (!layer) throw new Error('the downloaded file could not be read');
            return { cells: layer.cells, bytes: buf.length };
        },

        /** Adds a CSV (OpenCelliD / MLS layout, or the bundle's) as the newest layer. */
        async importCsv(srcFile) {
            const probe = await loadCsvLayer(srcFile, path.basename(srcFile));
            if (!probe.cells) throw new Error('no LTE cells in the file');
            fs.mkdirSync(sourcesDir, { recursive: true });
            fs.copyFileSync(srcFile, path.join(sourcesDir, path.basename(srcFile)));
            layers = null;
            return { cells: probe.cells };
        },

        getSettings() {
            const s = getSettings();
            return { online: s.online, hasToken: !!s.token };
        },
        setSettings(patch) {
            const s = getSettings();
            if (typeof patch.online === 'boolean') s.online = patch.online;
            if (typeof patch.token === 'string') s.token = patch.token.trim();
            writeJson(settingsFile, s);
            // a new token may find cells that were not found before
            if (typeof patch.token === 'string' && lookups) {
                for (const k of Object.keys(lookups)) if (lookups[k].miss) delete lookups[k];
            }
            return this.getSettings();
        },

        /** Download every block of a box (e.g. a whole country) ahead of time. */
        async downloadBox({ south, west, north, east }, onProgress = () => {}) {
            const todo = blocksForBox(south, west, north, east);
            let done = 0, cells = 0, failed = 0;
            for (const [bx, by] of todo) {
                try {
                    const rows = await block(bx, by, true);
                    cells += rows ? rows.length : 0;
                } catch (e) {
                    failed++;
                    console.warn(`[celldb] block ${bx}/${by}: ${e.message}`);
                }
                onProgress({ done: ++done, total: todo.length, cells, failed });
            }
            return { blocks: todo.length, cells, failed };
        },

        async status() {
            const idx = loadIndex();
            const counts = Object.values(idx.blocks);
            let size = 0;
            try { for (const f of fs.readdirSync(blocksDir)) size += fs.statSync(path.join(blocksDir, f)).size; } catch (e) { /* empty */ }
            const csv = await loadLayers();
            const lk = Object.values(loadLookups());
            return {
                blocks: counts.length, cells: counts.reduce((a, b) => a + b, 0), bytes: size, date: idx.date, source: url,
                bundle: bundleManifest(),
                files: csv.filter((l) => !l.bundle).map((l) => ({ name: l.name, cells: l.cells, newest: l.newest })),
                lookups: { found: lk.filter((v) => !v.miss).length, missed: lk.filter((v) => v.miss).length },
                settings: this.getSettings(),
            };
        },

        /** Deletes what was downloaded or imported; the bundle and the settings stay. */
        clear() {
            for (const f of [blocksDir, leafDir, sourcesDir, indexFile, lookupFile]) fs.rmSync(f, { recursive: true, force: true });
            index = null;
            header = root = null;
            leaves.clear();
            memBlocks.clear();
            layers = null;
            lookups = null;
        },

        get stats() { return stats; },
    };
}

// ── IPC ──────────────────────────────────────────────────────────────────────

let db = null;

function initCellDbHandlers(win) {
    const { ipcMain } = require('electron');
    const dir = path.join(require('./mission-store').getRoot(), 'celldb');
    db = createCellDb({ dir, bundleDir: path.join(__dirname, 'assets', 'celldb') });
    ipcMain.handle('celldb-query', (event, q) => db.query(q || {}));
    ipcMain.handle('celldb-lookup', (event, cell) => db.lookup(cell));
    ipcMain.handle('celldb-status', () => db.status());
    ipcMain.handle('celldb-clear', () => { db.clear(); return db.status(); });
    ipcMain.handle('celldb-settings', (event, patch) => (patch ? db.setSettings(patch) : db.getSettings()));
    ipcMain.handle('celldb-download-ocid', async (event, mcc) => ({ ...(await db.downloadOpenCellid(mcc)), status: await db.status() }));
    ipcMain.handle('celldb-import', async () => {
        const { dialog } = require('electron');
        const r = await dialog.showOpenDialog(win, {
            title: 'Import LTE cells (OpenCelliD or MLS CSV)',
            filters: [{ name: 'Cell CSV', extensions: ['csv', 'gz'] }],
            properties: ['openFile'],
        });
        if (r.canceled || !r.filePaths.length) return null;
        return { ...(await db.importCsv(r.filePaths[0])), status: await db.status() };
    });
    ipcMain.handle('celldb-download-box', async (event, box) => {
        const t0 = Date.now();
        const r = await db.downloadBox(box, (p) => {
            if (win && !win.isDestroyed()) win.webContents.send('celldb-progress', p);
        });
        console.log(`[celldb] box downloaded: ${r.blocks} blocks, ${r.cells} LTE cells, ${r.failed} failed, ${((Date.now() - t0) / 1000).toFixed(1)} s`);
        return { ...r, status: await db.status() };
    });
}

module.exports = {
    createCellDb, initCellDbHandlers,
    // exported for scripts/test-cell-db.js
    zxyToTileId, tileIdToZxy, blockTileRange, parseDirectory, decodeTile, distanceKm,
    lonToX, latToY, tileLon, tileLat, BLOCK_Z, TILE_Z,
};
