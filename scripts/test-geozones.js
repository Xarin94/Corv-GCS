#!/usr/bin/env node
/**
 * test-geozones.js - checks for the UAS geographical zones
 *
 *   node scripts/test-geozones.js            offline checks: ED-269 / ED-318
 *                                            parsing, source mapping, tile
 *                                            cache and source choice (fake
 *                                            fetch), regions, evaluation, the
 *                                            terrain raster
 *   node scripts/test-geozones.js --online   also asks the real services
 *                                            around Munich, Los Angeles,
 *                                            Paris, Madrid and Zurich
 */

const fs = require('fs');
const os = require('os');
const path = require('path');
const { pathToFileURL } = require('url');
const gz = require('../geozones');

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}

const square = (lon, lat, d) => [[[lon - d, lat - d], [lon + d, lat - d], [lon + d, lat + d], [lon - d, lat + d], [lon - d, lat - d]]];

// ── ED-269 / ED-318 parsing ──────────────────────────────────────────────────
{
    const ed269 = {
        title: 'test', features: [
            {
                identifier: 'A1', name: 'Airport CTR', restriction: 'REQ_AUTHORISATION', message: 'Ask ATC',
                applicability: [{ permanent: 'YES' }],
                geometry: [
                    { uomDimensions: 'M', lowerLimit: 0, lowerVerticalReference: 'AGL', upperLimit: 99999, upperVerticalReference: 'AGL',
                        horizontalProjection: { type: 'Polygon', coordinates: square(11, 45, 0.01) } },
                    { uomDimensions: 'FT', lowerLimit: 200, lowerVerticalReference: 'AGL', upperLimit: 5000, upperVerticalReference: 'AMSL',
                        horizontalProjection: { type: 'Circle', center: [11.1, 45], radius: 1000 } },
                ],
            },
            { identifier: 'OLD', restriction: 'PROHIBITED', applicability: [{ permanent: 'NO', endDateTime: '2020-01-01T00:00:00Z' }],
                geometry: [{ uomDimensions: 'M', lowerLimit: 0, lowerVerticalReference: 'AGL', upperLimit: 120, upperVerticalReference: 'AGL',
                    horizontalProjection: { type: 'Polygon', coordinates: square(12, 45, 0.01) } }] },
            { identifier: 'FREE', restriction: 'NO_RESTRICTION',
                geometry: [{ uomDimensions: 'M', lowerLimit: 0, lowerVerticalReference: 'AGL', upperLimit: 120, upperVerticalReference: 'AGL',
                    horizontalProjection: { type: 'Polygon', coordinates: square(13, 45, 0.01) } }] },
        ],
    };
    const z = gz.parseEdZones(ed269, 'imp');
    check('ED-269: one zone per volume, expired and unrestricted ones dropped', z.length === 2, `${z.length}`);
    check('ED-269: authorisation is a no-fly zone, 99999 is no top', z[0].kind === 'prohibited' && z[0].top === null && z[0].floor === 0 && z[0].id === 'imp:A1#0');
    check('ED-269: feet and AMSL', Math.abs(z[1].floor - 60.96) < 0.1 && z[1].floorRef === 'AGL' && z[1].topRef === 'MSL' && Math.abs(z[1].top - 1524) < 0.01);
    const ring = z[1].polys[0][0];
    const dLat = (Math.max(...ring.filter((v, i) => i % 2)) - Math.min(...ring.filter((v, i) => i % 2))) * 111320;
    check('ED-269: a circle becomes a polygon of its radius', Math.abs(dLat - 2000) < 5, `${dLat.toFixed(0)} m across`);

    const ed318 = { type: 'FeatureCollection', features: [
        { type: 'Feature', geometry: { type: 'Polygon', coordinates: square(-3.7, 40.4, 0.01) },
            properties: { identifier: 'ES1', name: 'Urban', type: 'CONDITIONAL', lower: 0, lowerReference: 'AGL', upper: 120, upperReference: 'AGL', uom: 'M' } },
        { type: 'Feature', geometry: { type: 'MultiPolygon', coordinates: [square(-3.6, 40.4, 0.01), square(-3.5, 40.4, 0.01)] },
            properties: { identifier: 'ES2', restriction: 'PROHIBITED', verticalLayer: { lower: 45, lowerReference: 'AGL', upper: 900, upperReference: 'AMSL', uom: 'M' } } },
    ] };
    const e = gz.parseEdZones(ed318, 'es');
    check('ED-318: properties and verticalLayer', e.length === 2 && e[0].kind === 'conditional' && e[1].kind === 'prohibited' && e[1].floor === 45 && e[1].polys.length === 2);
}

// ── Simplification ───────────────────────────────────────────────────────────
{
    const ring = [];
    for (let i = 0; i <= 1000; i++) ring.push([10 + i * 1e-5, 45 + (i % 2) * 1e-7]);   // a wobbly 800 m line...
    ring.push([10.01, 45.01], [10, 45.01], [10, 45]);
    const flat = gz.simplifyRing(ring);
    check('simplify: a wiggle under 3 m goes, the corners stay', flat.length / 2 === 5, `${flat.length / 2} vertices`);
}

// ── FAA mapping ──────────────────────────────────────────────────────────────
{
    const uasfm = gz.FAA_LAYERS.find(l => l.id === 'uasfm');
    const a = uasfm.zone({ OBJECTID: 1, CEILING: 0, UNIT: 'Feet', APT1_ICAO: 'KLAX', AIRSPACE_1: 'B' });
    const b = uasfm.zone({ OBJECTID: 2, CEILING: 100, UNIT: 'Feet', APT1_ICAO: 'KSMO', AIRSPACE_1: 'D' });
    check('FAA facility map: 0 ft is no-fly, 100 ft a 30.5 m ceiling', a.floor === 0 && Math.abs(b.floor - 30.48) < 0.01 && b.floorRef === 'AGL');
    const ns = gz.FAA_LAYERS.find(l => l.id === 'nsufr').zone({ OBJECTID: 3, Floor: 'Surface', Ceiling: "400' AGL" });
    check('FAA national security: surface to 400 ft AGL', ns.floor === 0 && Math.abs(ns.top - 121.92) < 0.01 && ns.topRef === 'AGL');
    const fl = gz.faaLimit('180', 'FL', 'STD');
    check('FAA limits: flight level, SFC', Math.abs(fl.value - 18000 * 0.3048) < 0.1 && fl.ref === 'MSL' && gz.faaLimit('0', 'FT', 'SFC').ref === 'AGL');
}

// ── Busy servers ─────────────────────────────────────────────────────────────
async function busyChecks() {
    let n = 0;
    const answers = [
        { status: 200, body: { error: { code: 429, message: 'Unable to perform query. Too many requests.' } } },
        { status: 429, body: {} },
        { status: 200, body: { features: [1] } },
    ];
    const fake = async () => { const a = answers[Math.min(n++, answers.length - 1)]; return { ok: a.status < 400, status: a.status, text: async () => JSON.stringify(a.body) }; };
    const json = await gz.fetchJson(fake, 'https://x.test/q', 1000, 5);
    check('busy server: asked again until it answers', n === 3 && json.features.length === 1);
    n = 0;
    const always = async () => { n++; return { ok: true, status: 200, text: async () => JSON.stringify(answers[0].body) }; };
    let err = null;
    try { await gz.fetchJson(always, 'https://x.test/q', 1000, 1); } catch (e) { err = e; }
    check('busy server: gives up after 4 retries with its message', n === 5 && /Too many requests/.test(err?.message));
}

// ── Store: sources by country, tiles, cache, imports (fake network) ──────────
async function storeChecks() {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-geozones-'));
    const asked = [];
    const fakeFetch = async (url) => {
        asked.push(url);
        const u = new URL(url);
        const body = u.hostname === 'services6.arcgis.com' && url.includes('FAA_UAS_FacilityMap_Data')
            ? { type: 'FeatureCollection', features: [{ type: 'Feature', id: 7, geometry: { type: 'Polygon', coordinates: square(-118.4, 33.95, 0.004) },
                properties: { OBJECTID: 7, CEILING: 50, UNIT: 'Feet', APT1_ICAO: 'KLAX', AIRSPACE_1: 'B' } }] }
            : u.hostname === 'uas-betrieb.de' && u.searchParams.get('TYPENAMES') === 'dipul:kontrollzonen'
                ? { type: 'FeatureCollection', features: [{ type: 'Feature', id: 'kontrollzonen.1', geometry: { type: 'Polygon', coordinates: square(11.78, 48.35, 0.05) },
                    properties: { name: 'EDDM Zone 2', lower_limit_altitude: 1569, lower_limit_unit: 'ft', lower_limit_alt_ref: 'MSL',
                        upper_limit_altitude: 3500, upper_limit_unit: 'ft', upper_limit_alt_ref: 'MSL', type_code: 'KONTROLLZONE' } }] }
                : { type: 'FeatureCollection', features: [] };
        return { ok: true, status: 200, text: async () => JSON.stringify(body) };
    };
    let changes = 0;
    const store = gz.createGeoZones({ dir, fetch: fakeFetch, onChange: () => changes++ });
    const settle = () => new Promise(r => setTimeout(r, 700));

    let r = store.query({ circles: [{ lat: 41.9, lon: 12.5, radiusKm: 50 }] });
    check('Rome: no online source asked (import the D-Flight file)', r.sources.length === 0 && asked.length === 0);

    r = store.query({ circles: [{ lat: 33.95, lon: -118.4, radiusKm: 50 }] });
    check('Los Angeles: only the FAA', r.sources.length === 1 && r.sources[0].id === 'faa' && r.sources[0].tiles >= 4, `${r.sources.map(s => `${s.id}:${s.tiles}`)}`);
    await settle(); await settle();
    r = store.query({ circles: [{ lat: 33.95, lon: -118.4, radiusKm: 50 }] });
    check('Los Angeles: tiles fetched in the background, onChange called', changes > 0 && r.zones.length === 1 && r.zones[0].floorRef === 'AGL' && r.pending === 0);
    const n = asked.length;
    store.query({ circles: [{ lat: 33.95, lon: -118.4, radiusKm: 50 }] });
    check('Los Angeles: fresh tiles are not asked again', asked.length === n);

    r = store.query({ circles: [{ lat: 48.35, lon: 11.78, radiusKm: 50 }] });
    check('Munich: only DIPUL', r.sources.length === 1 && r.sources[0].id === 'dipul');
    await settle(); await settle();
    r = store.query({ circles: [{ lat: 48.35, lon: 11.78, radiusKm: 50 }] });
    const ctr = r.zones.find(z => z.name === 'EDDM Zone 2');
    check('Munich: control zone 2 with its floor above sea level', ctr && ctr.floorRef === 'MSL' && Math.abs(ctr.floor - 478.2) < 0.1);
    r = store.query({ circles: [{ lat: 49.5, lon: 11.78, radiusKm: 50 }] });
    check('A zone further than 50 km is left out', !r.zones.some(z => z.name === 'EDDM Zone 2'));

    const store2 = gz.createGeoZones({ dir, fetch: async () => { throw new Error('offline'); } });
    r = store2.query({ circles: [{ lat: 48.35, lon: 11.78, radiusKm: 50 }] });
    check('Offline: the tiles on disk still answer', r.zones.some(z => z.name === 'EDDM Zone 2'));

    const file = path.join(dir, 'dflight.json');
    fs.writeFileSync(file, JSON.stringify({ features: [{ identifier: 'IT1', name: 'Urbe', restriction: 'PROHIBITED',
        geometry: [{ uomDimensions: 'M', lowerLimit: 25, lowerVerticalReference: 'AGL', upperLimit: 120, upperVerticalReference: 'AGL',
            horizontalProjection: { type: 'Polygon', coordinates: square(12.5, 41.95, 0.02) } }] }] }));
    store.importFile(file);
    r = store.query({ circles: [{ lat: 41.9, lon: 12.5, radiusKm: 50 }] });
    check('Rome: an imported ED-269 file answers', r.zones.length === 1 && r.zones[0].floor === 25 && r.imports.length === 1);
    store.removeImport(r.imports[0].file);
    r = store.query({ circles: [{ lat: 41.9, lon: 12.5, radiusKm: 50 }] });
    check('Removing the import removes its zones', r.zones.length === 0 && r.imports.length === 0);
    fs.rmSync(dir, { recursive: true, force: true });
}

// ── Renderer modules (ES modules, run from a temp copy) ──────────────────────
async function rendererChecks() {
    const root = path.join(__dirname, '..');
    const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-geozones-es-'));
    fs.writeFileSync(path.join(tmp, 'package.json'), '{"type":"module"}');
    for (const rel of ['js/safety/GeoZoneRules.js', 'js/safety/GeoZoneStyle.js', 'js/terrain/GeoZoneTerrain.js',
        'js/core/utils.js', 'js/core/constants.js', 'vendor/three/three.core.js']) {
        fs.mkdirSync(path.dirname(path.join(tmp, rel)), { recursive: true });
        fs.copyFileSync(path.join(root, rel), path.join(tmp, rel));
    }
    const load = rel => import(pathToFileURL(path.join(tmp, rel)));
    const R = await load('js/safety/GeoZoneRules.js');

    const places = {
        EU: { Rome: [41.9, 12.5], Munich: [48.14, 11.58], London: [51.5, -0.12], Reykjavik: [64.15, -21.94], Lampedusa: [35.5, 12.6],
            Rhodes: [36.43, 28.22], Kos: [36.89, 27.29], Karpathos: [35.5, 27.2], Nicosia: [35.17, 33.36], 'Las Palmas': [28.1, -15.42], Helsinki: [60.17, 24.94], Athens: [37.98, 23.73],
            Lisbon: [38.72, -9.14], Valletta: [35.9, 14.51], Bucharest: [44.43, 26.1], Tromso: [69.65, 18.96], Madeira: [32.65, -16.9] },
        US: { 'Los Angeles': [34.05, -118.24], Anchorage: [61.22, -149.9], Honolulu: [21.31, -157.86], 'San Juan': [18.47, -66.1] },
        null: { Istanbul: [41.0, 28.97], Antalya: [36.9, 30.7], Bodrum: [37.03, 27.43], Marmaris: [36.85, 28.27], Fethiye: [36.62, 29.12], Tunis: [36.8, 10.18], Algiers: [36.75, 3.06],
            Casablanca: [33.57, -7.59], Tangier: [35.77, -5.8], Kyiv: [50.45, 30.52], Minsk: [53.9, 27.56], Moscow: [55.75, 37.62],
            'St Petersburg': [59.94, 30.31], Tripoli: [32.89, 13.19], Nuuk: [64.18, -51.72] },
    };
    for (const [want, list] of Object.entries(places)) {
        const wrong = Object.entries(list).filter(([, [lat, lon]]) => String(R.regionAt(lat, lon)) !== want).map(([n]) => n);
        check(`regions: ${want === 'null' ? 'no general limit' : want} for ${Object.keys(list).length} places`, !wrong.length, wrong.join(', '));
    }

    const zone = (o) => ({ id: o.id, name: o.id, kind: 'prohibited', floor: 0, floorRef: 'AGL', top: null, topRef: 'AGL', info: '', ...o,
        polys: [[[o.lon - 0.01, o.lat - 0.01, o.lon + 0.01, o.lat - 0.01, o.lon + 0.01, o.lat + 0.01, o.lon - 0.01, o.lat + 0.01, o.lon - 0.01, o.lat - 0.01]]],
        bbox: [o.lon - 0.01, o.lat - 0.01, o.lon + 0.01, o.lat + 0.01] });
    const holed = zone({ id: 'H', lat: 45, lon: 11 });
    holed.polys[0].push([10.995, 44.995, 11.005, 44.995, 11.005, 45.005, 10.995, 45.005, 10.995, 44.995]);
    check('point in zone: inside, in the hole, outside', R.pointInZone(holed, 45.008, 11) && !R.pointInZone(holed, 45, 11) && !R.pointInZone(holed, 45.02, 11));

    const ids = r => r.alerts.map(a => a.id).sort().join(',');
    const nf = zone({ id: 'NF', lat: 45, lon: 11 });
    let r = R.evaluateGeoZones({ zones: [nf], lat: 45, lon: 11, agl: 30, terrain: 100 });
    check('no-fly zone, flying: NO-FLY warning', ids(r) === 'noFly' && r.alerts[0].level === 'warn' && !r.overHeight);
    r = R.evaluateGeoZones({ zones: [nf], lat: 45, lon: 11, agl: 0, terrain: 100, airborne: false });
    check('no-fly zone, on the ground: nothing', ids(r) === '');
    const c60 = zone({ id: 'C60', lat: 45, lon: 11, floor: 60 });
    r = R.evaluateGeoZones({ zones: [c60], lat: 45, lon: 11, agl: 40, terrain: 100 });
    check('under a 60 m ceiling: MAX 60 m caution', ids(r) === 'maxHere' && r.alerts[0].label === 'MAX 60 m' && r.alerts[0].level === 'caut');
    r = R.evaluateGeoZones({ zones: [c60], lat: 45, lon: 11, agl: 70, terrain: 100 });
    check('over a 60 m ceiling: MAX 60 m warning, TERR ALT flashes', ids(r) === 'ceiling' && r.alerts[0].level === 'warn' && r.overHeight);
    const msl = zone({ id: 'MSL', lat: 45, lon: 11, floor: 478, floorRef: 'MSL' });
    r = R.evaluateGeoZones({ zones: [msl], lat: 45, lon: 11, agl: 40, terrain: 450 });
    check('a floor above sea level is taken over the ground under the vehicle', ids(r) === 'ceiling' && Math.round(r.alerts[0].metres) === 28);
    r = R.evaluateGeoZones({ zones: [], lat: 45, lon: 11, agl: 125, terrain: 100 });
    check('Europe over 120 m, no zone: ALT LIMIT warning, TERR ALT flashes', ids(r) === 'altLimit' && r.overHeight && r.alerts[0].label === 'MAX 120 m');
    r = R.evaluateGeoZones({ zones: [], lat: 45, lon: 11, agl: 115, terrain: 100 });
    check('Europe at 115 m: nothing', ids(r) === '' && !r.overHeight);
    r = R.evaluateGeoZones({ zones: [], lat: 34, lon: -118, agl: 122, terrain: 0 });
    check('US at 122 m: within 400 ft (+1 m)', !r.overHeight);
    r = R.evaluateGeoZones({ zones: [], lat: 34, lon: -118, agl: 130, terrain: 0 });
    check('US at 130 m: MAX 400 ft', ids(r) === 'altLimit' && r.alerts[0].label === 'MAX 400 ft');
    const cond = zone({ id: 'COND', lat: 45, lon: 11, kind: 'conditional' });
    const top = zone({ id: 'TOP', lat: 45, lon: 11, top: 50 });
    r = R.evaluateGeoZones({ zones: [cond, top], lat: 45, lon: 11, agl: 80, terrain: 100 });
    check('conditions: ZONE caution; over a zone\'s top: not in it', ids(r) === 'cond');

    // Arming alarm: no-fly first, then conditions, then the lowest ceiling under the general limit
    const lim = R.GENERAL_LIMITS.EU;
    let a = R.armingAlert([c60, cond, nf], 100, lim);
    check('arming: in a no-fly zone, red', a && a.level === 'warn' && a.title === 'ARMED IN A NO-FLY ZONE' && a.zone === nf);
    a = R.armingAlert([c60, cond], 100, lim);
    check('arming: where conditions apply, amber', a && a.level === 'caut' && a.zone === cond);
    a = R.armingAlert([zone({ id: 'C90', lat: 45, lon: 11, floor: 90 }), c60, msl], 450, lim);
    check('arming: under ceilings, the lowest (MSL over the ground)', a && a.level === 'caut' && a.title === 'ARMED UNDER A 28 m CEILING' && a.zone === msl);
    check('arming: a ceiling at the general limit is no alarm, nor is no zone',
        R.armingAlert([zone({ id: 'C120', lat: 45, lon: 11, floor: 120 })], 100, lim) === null && R.armingAlert([], 100, lim) === null);

    // Terrain raster: a zone over the north half of the window, its floor where the shader reads it
    global.THREE = await load('vendor/three/three.core.js');
    global.performance = global.performance || { now: () => Date.now() };
    const T = await load('js/terrain/GeoZoneTerrain.js');
    const north = { ...zone({ id: 'N', lat: 45.05, lon: 11, floor: 30 }), polys: [[[10.8, 45.0, 11.2, 45.0, 11.2, 45.2, 10.8, 45.2, 10.8, 45.0]]], bbox: [10.8, 45, 11.2, 45.2] };
    const nfz = { ...zone({ id: 'S', lat: 44.95, lon: 11, floor: 0 }), polys: [[[10.99, 44.94, 11.01, 44.94, 11.01, 44.96, 10.99, 44.96, 10.99, 44.94]]], bbox: [10.99, 44.94, 11.01, 44.96] };
    T.setGeoZoneTerrain([north, nfz], 1, true);
    console.log = ((log) => (...a) => { if (!String(a[0]).startsWith('[geozones] terrain')) log(...a); })(console.log);
    T.updateGeoZoneTerrain(45, 11);
    const u = T.geoZoneTerrainUniforms;
    const S = u.uZoneCov.value.image.width;
    const cv = u.uZoneCov.value.image.data, fl = u.uZoneFloor.value.image.data;
    const half = v => global.THREE.DataUtils.fromHalfFloat(v);
    const covAt = (row, col, ch) => cv[(Math.round(row) * S + Math.round(col)) * 4 + ch];
    const floorAt = (row, col, ch) => half(fl[(Math.round(row) * S + Math.round(col)) * 2 + ch]);
    check('raster: zone on, the north half is a 30 m floor, the south none',
        u.uZoneOn.value === 1 && covAt(S * 0.25, S / 2, 2) === 255 && floorAt(S * 0.25, S / 2, 0) === 30
        && covAt(S * 0.98, S / 2, 2) === 0 && floorAt(S * 0.98, S / 2, 0) > 50000 && covAt(S * 0.25, S / 2, 0) === 0);
    check('raster: a floor spreads one texel past the zone', floorAt(S / 2, S / 2, 0) === 30 && floorAt(S / 2 + 2, S / 2, 0) > 50000);
    // 44.95 N is 5.56 km south of the centre, in a 20 km window: row 0.778 S
    const row = Math.round(S * (0.5 + 5.56 / 20));
    const partial = Array.from({ length: 120 }, (_, i) => covAt(row, S / 2 - 60 + i, 0)).filter(v => v > 0 && v < 255);
    check('raster: the no-fly square is covered where it lies, partly on its edges, and only there',
        covAt(row, S / 2, 0) === 255 && covAt(row, S / 2 + 58, 0) === 0 && partial.length === 2, `row ${row}, partial ${partial}`);
    const m = u.uZoneMap.value;
    check('raster: world rect from its north-west corner, 20 km north to south', 1 / m.z > 0 && Math.abs(1 / m.w - 20000) < 50);
    fs.rmSync(tmp, { recursive: true, force: true });
}

async function onlineChecks() {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-geozones-online-'));
    const store = gz.createGeoZones({ dir });
    for (const [name, lat, lon, src] of [['Munich', 48.35, 11.78, 'dipul'], ['Los Angeles', 33.94, -118.4, 'faa'],
        ['Paris', 48.85, 2.35, 'ign'], ['Madrid', 40.42, -3.7, 'enaire'], ['Zurich', 47.45, 8.56, 'bazl']]) {
        const q = { circles: [{ lat, lon, radiusKm: 30 }] };
        let r = store.query(q);
        const t0 = Date.now();
        while (r.pending && Date.now() - t0 < 300000) { await new Promise(res => setTimeout(res, 2000)); r = store.query(q); }
        const n = r.zones.filter(z => z.src === src).length;
        check(`online ${name}: ${src} zones`, n > 0, `${n} zones, ${r.sources.map(s => `${s.id} ${s.ready}/${s.tiles}${s.error ? ' ' + s.error : ''}`).join(', ')}`);
    }
    fs.rmSync(dir, { recursive: true, force: true });
}

(async () => {
    await busyChecks();
    await storeChecks();
    await rendererChecks();
    if (process.argv.includes('--online')) await onlineChecks();
    console.log(failures ? `\n${failures} FAILED` : '\nall passed');
    process.exit(failures ? 1 : 0);
})();
