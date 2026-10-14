/**
 * cell-db.js - LTE cell sites around the vehicle, from OpenCelliD
 *
 * Source: the OpenCelliD database published as one PMTiles file by UN Smart
 * Maps on Source Cooperative (public, no account; CC BY-SA 4.0, attribution
 * "OpenCelliD"). Only the parts that are needed are read, with HTTP range
 * requests: never the whole 560 MB file.
 *
 * Storage unit: one zoom-8 tile (a "block", ~110 km at 45° N). Its zoom-14
 * tiles — the only zoom that holds every cell — are one contiguous run of
 * tile ids in the Hilbert order of PMTiles, and contiguous on disk because
 * the archive is clustered, so a block costs a directory lookup and one or
 * two range requests. LTE cells of a block are kept as JSON under
 * <data root>/celldb/blocks; once there they work offline.
 *
 * Positions are OpenCelliD estimates (averaged crowd measurements), not the
 * surveyed mast position: off by hundreds of metres, sometimes more.
 */

const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

const SOURCE_URL = 'https://data.source.coop/smartmaps/opencellid/cellid.pmtiles';
const USER_AGENT = 'Corv-GCS cell-db';   // the CDN refuses requests without one
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
 * @param {string} [opts.url] PMTiles archive
 * @param {Function} [opts.fetch] fetch implementation (tests)
 */
function createCellDb({ dir, url = SOURCE_URL, fetch: fetchImpl = globalThis.fetch }) {
    const blocksDir = path.join(dir, 'blocks');
    const leafDir = path.join(dir, 'leaves');
    const indexFile = path.join(dir, 'index.json');
    let index = null;           // { etag, date, blocks: { "x_y": cells } }
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
    function saveIndex() {
        fs.mkdirSync(dir, { recursive: true });
        const tmp = indexFile + '.tmp';
        fs.writeFileSync(tmp, JSON.stringify(index));
        fs.renameSync(tmp, indexFile);
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
         * Cell sites (cells of one eNB, by operator) within radiusKm.
         * @returns {{sites: object[], missing: number, date: string|null}}
         */
        async query({ lat, lon, radiusKm = 100, mcc = null, mnc = null, download = true }) {
            const sites = new Map();
            let missing = 0;
            for (const [bx, by] of blocksForRadius(lat, lon, radiusKm)) {
                let rows;
                try { rows = await block(bx, by, download); } catch (e) {
                    console.warn(`[celldb] block ${bx}/${by}: ${e.message}`);
                    rows = null;
                }
                if (!rows) { missing++; continue; }
                for (const [cLat, cLon, cMcc, cMnc, tac, eci, rng, samples] of rows) {
                    if ((mcc !== null && cMcc !== mcc) || (mnc !== null && cMnc !== mnc)) continue;
                    if (distanceKm(lat, lon, cLat, cLon) > radiusKm) continue;
                    const enb = Math.floor(eci / 256);
                    const key = `${cMcc}-${cMnc}-${enb}`;
                    let s = sites.get(key);
                    if (!s) sites.set(key, s = { mcc: cMcc, mnc: cMnc, enb, lat: 0, lon: 0, w: 0, cells: [] });
                    const w = Math.max(1, samples);
                    s.lat += cLat * w; s.lon += cLon * w; s.w += w;
                    s.cells.push({ eci, tac, lat: cLat, lon: cLon, range: rng, samples });
                }
            }
            const list = [];
            for (const s of sites.values()) {
                list.push({ mcc: s.mcc, mnc: s.mnc, enb: s.enb, lat: s.lat / s.w, lon: s.lon / s.w, cells: s.cells });
            }
            return { sites: list, missing, date: loadIndex().date };
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

        status() {
            const idx = loadIndex();
            const counts = Object.values(idx.blocks);
            let size = 0;
            try { for (const f of fs.readdirSync(blocksDir)) size += fs.statSync(path.join(blocksDir, f)).size; } catch (e) { /* empty */ }
            return { blocks: counts.length, cells: counts.reduce((a, b) => a + b, 0), bytes: size, date: idx.date, source: url };
        },

        clear() {
            fs.rmSync(dir, { recursive: true, force: true });
            index = null;
            header = root = null;
            leaves.clear();
            memBlocks.clear();
        },

        get stats() { return stats; },
    };
}

// ── IPC ──────────────────────────────────────────────────────────────────────

let db = null;

function initCellDbHandlers(win) {
    const { ipcMain } = require('electron');
    const dir = path.join(require('./mission-store').getRoot(), 'celldb');
    db = createCellDb({ dir });
    ipcMain.handle('celldb-query', (event, q) => db.query(q || {}));
    ipcMain.handle('celldb-status', () => db.status());
    ipcMain.handle('celldb-clear', () => { db.clear(); return db.status(); });
    ipcMain.handle('celldb-download-box', async (event, box) => {
        const t0 = Date.now();
        const r = await db.downloadBox(box, (p) => {
            if (win && !win.isDestroyed()) win.webContents.send('celldb-progress', p);
        });
        console.log(`[celldb] box downloaded: ${r.blocks} blocks, ${r.cells} LTE cells, ${r.failed} failed, ${((Date.now() - t0) / 1000).toFixed(1)} s`);
        return { ...r, status: db.status() };
    });
}

module.exports = {
    createCellDb, initCellDbHandlers,
    // exported for scripts/test-cell-db.js
    zxyToTileId, tileIdToZxy, blockTileRange, parseDirectory, decodeTile, distanceKm,
    lonToX, latToY, tileLon, tileLat, BLOCK_Z, TILE_Z,
};
