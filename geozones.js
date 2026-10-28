/**
 * geozones.js - UAS geographical zones around the mission and the vehicle
 *
 * There is no EASA service for the whole of Europe: under Regulation (EU)
 * 2019/947 art. 15 every state publishes its own zones, in the EUROCAE ED-269
 * (or the newer ED-318) format where it follows the AMC. The FAA publishes the
 * US data on ArcGIS. Sources, each asked only where it can have data (its
 * country's box), so a flight in Italy never queries the FAA:
 *
 *   faa     FAA (US) - ArcGIS FeatureServers, no key: UAS Facility Map grid
 *           (the ceiling LAANC can authorise, 0-400 ft AGL), national security
 *           UAS flight restrictions (full time and part time), prohibited
 *           areas, restricted areas (SUA type R), national defense TFR areas
 *   dipul   DFS DIPUL (DE) - WFS, one request per layer: airports, control
 *           zones, restricted areas, temporary restrictions, sensitive sites
 *           (military, prisons, power plants, police, hospitals...), nature
 *           reserves. Not residential plots, roads, railways or waterways:
 *           megabytes per tile, and the rule there is a lateral distance
 *   ign     IGN / DGAC (FR) - Géoplateforme WFS "restrictions UAS"
 *           ("Vol interdit" or "Hauteur maximale de vol de N m")
 *   enaire  ENAIRE (ES) - ArcGIS MapServer, ED-318 attributes
 *   bazl    FOCA / BAZL (CH) - the whole country as one ED-269 file
 *   imports ED-269 / ED-318 files the user loads: D-Flight (IT, download from
 *           the d-flight.it profile page), Austro Control, any other state
 *
 * Online sources are read by tiles (TILE_DEG, DIPUL 1°), kept on disk under
 * <data root>/geozones and asked again after TTL_MS so the zones stay current
 * while the GCS runs; a tile that cannot be fetched keeps serving its last
 * copy. A query is a list of circles (the vehicle, the mission's points, 50 km
 * each): it answers at once with what is on disk, fetches the missing or stale
 * tiles in the background and calls onChange when they arrive.
 *
 * Every zone comes out the same way: a restricted volume (floor, top, each AGL
 * or MSL) over one or more polygons, kind 'prohibited' (PROHIBITED or
 * REQ_AUTHORISATION) or 'conditional' (CONDITIONAL, part time, by NOTAM). A
 * floor above the ground is the height that may be flown up to.
 */

const fs = require('fs');
const path = require('path');

const USER_AGENT = 'Corv-GCS geozones';
const TTL_MS = 6 * 3600 * 1000;           // a tile is fetched again after this...
const RETRY_MS = 10 * 60 * 1000;          // ...a failed one after this
const FILE_TTL_MS = 7 * 24 * 3600 * 1000; // a whole-country file
const TIMEOUT_MS = 30000;
const FILE_TIMEOUT_MS = 180000;
const CONCURRENCY = 4;
const BUSY_RETRIES = 4;                   // a "too many requests" is asked again after 2, 4, 8, 16 s
const MAX_TILES = 160;                    // per source and query
const SIMPLIFY_DEG = 3e-5;                // ~3 m: vertices closer than this to the outline go
const FT = 0.3048;
const UNLIMITED_M = 30000;                // a top above this is no top ("99999", FL999)

// ── Geometry ─────────────────────────────────────────────────────────────────

/** Douglas-Peucker on a [[lon, lat], ...] ring, lon scaled by cos(lat). Returns a flat [lon, lat, ...]. */
function simplifyRing(coords, tol = SIMPLIFY_DEG) {
    const n = coords.length;
    if (n < 4) return null;
    const k = Math.cos((coords[0][1] || 0) * Math.PI / 180);
    const keep = new Uint8Array(n);
    keep[0] = keep[n - 1] = 1;
    const stack = [[0, n - 1]];
    while (stack.length) {
        const [a, b] = stack.pop();
        const ax = coords[a][0] * k, ay = coords[a][1];
        const dx = coords[b][0] * k - ax, dy = coords[b][1] - ay;
        const len2 = dx * dx + dy * dy;
        let worst = -1, worstD = tol * tol;
        for (let i = a + 1; i < b; i++) {
            const px = coords[i][0] * k - ax, py = coords[i][1] - ay;
            let d2;
            if (len2 === 0) d2 = px * px + py * py;
            else {
                const t = Math.max(0, Math.min(1, (px * dx + py * dy) / len2));
                const ex = px - t * dx, ey = py - t * dy;
                d2 = ex * ex + ey * ey;
            }
            if (d2 > worstD) { worstD = d2; worst = i; }
        }
        if (worst > 0) {
            keep[worst] = 1;
            stack.push([a, worst], [worst, b]);
        }
    }
    const out = [];
    for (let i = 0; i < n; i++) {
        if (keep[i]) out.push(+coords[i][0].toFixed(6), +coords[i][1].toFixed(6));
    }
    return out.length >= 8 ? out : null;      // a ring needs 3 corners and the closing point
}

/** GeoJSON Polygon / MultiPolygon → polygons of flat rings (outer first, then holes). */
function geojsonPolys(geometry) {
    if (!geometry) return [];
    const list = geometry.type === 'Polygon' ? [geometry.coordinates]
        : geometry.type === 'MultiPolygon' ? geometry.coordinates : [];
    const polys = [];
    for (const rings of list) {
        const flat = [];
        for (const ring of rings || []) {
            const r = Array.isArray(ring) ? simplifyRing(ring) : null;
            if (r) flat.push(r);
            else if (!flat.length) break;   // no outer ring, no polygon
        }
        if (flat.length) polys.push(flat);
    }
    return polys;
}

function circlePoly(lon, lat, radiusM, n = 48) {
    const ring = [];
    const dLat = radiusM / 111320, dLon = dLat / Math.max(0.01, Math.cos(lat * Math.PI / 180));
    for (let i = 0; i <= n; i++) {
        const a = (i % n) / n * 2 * Math.PI;
        ring.push(+(lon + dLon * Math.sin(a)).toFixed(6), +(lat + dLat * Math.cos(a)).toFixed(6));
    }
    return [[ring]];
}

function bboxOf(polys) {
    let w = 180, s = 90, e = -180, n = -90;
    for (const rings of polys) {
        const r = rings[0];
        for (let i = 0; i < r.length; i += 2) {
            if (r[i] < w) w = r[i]; if (r[i] > e) e = r[i];
            if (r[i + 1] < s) s = r[i + 1]; if (r[i + 1] > n) n = r[i + 1];
        }
    }
    return [w, s, e, n];
}

function boxesIntersect(a, b) {
    return a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1];
}

/** Bounding box of a circle of radiusKm around lat/lon. */
function circleBox(c) {
    const dLat = c.radiusKm / 111.32;
    const dLon = dLat / Math.max(0.05, Math.cos(c.lat * Math.PI / 180));
    return [c.lon - dLon, c.lat - dLat, c.lon + dLon, c.lat + dLat];
}

/** Distance in km from lat/lon to the nearest point of a box (0 inside). */
function distanceToBoxKm(lat, lon, box) {
    const cLat = Math.max(box[1], Math.min(box[3], lat));
    const cLon = Math.max(box[0], Math.min(box[2], lon));
    const k = Math.cos(lat * Math.PI / 180);
    return Math.hypot((cLat - lat) * 111.32, (cLon - lon) * 111.32 * k);
}

function nearAnyCircle(zone, circles) {
    return circles.some(c => distanceToBoxKm(c.lat, c.lon, zone.bbox) <= c.radiusKm);
}

// ── Normalisation ────────────────────────────────────────────────────────────

/** Metres from a value and its unit (M, FT, Feet, ft...). */
function toMetres(value, uom) {
    const v = Number(value);
    if (!isFinite(v)) return null;
    return /^f/i.test(String(uom || 'M')) ? v * FT : v;
}

/** AGL (above the ground, the surface) or MSL (AMSL, WGS84, a flight level). */
function verticalRef(ref) {
    return /^(AGL|SFC|GND)/i.test(String(ref || 'AGL')) ? 'AGL' : 'MSL';
}

/** ED-269 / ED-318 restriction → kind; null for zones that restrict nothing. */
function restrictionKind(r) {
    const v = String(r || '').toUpperCase();
    if (v === 'PROHIBITED' || v === 'REQ_AUTHORISATION' || v === 'REQ_AUTHORIZATION') return 'prohibited';
    if (v === 'NO_RESTRICTION') return null;
    return 'conditional';
}

function stripHtml(s) {
    return String(s || '').replace(/<[^>]*>/g, ' ').replace(/&nbsp;/g, ' ').replace(/\s+/g, ' ').trim().slice(0, 300);
}

function makeZone({ src, key, name, kind, floor = 0, floorRef = 'AGL', top = null, topRef = 'AGL', polys, info = '' }) {
    if (!kind || !polys || !polys.length) return null;
    if (top !== null && top >= UNLIMITED_M) top = null;
    return {
        id: `${src}:${key}`, src, name: String(name || key).trim().slice(0, 120), kind,
        floor: Math.max(0, +(floor || 0).toFixed(1)), floorRef,
        top: top === null ? null : +top.toFixed(1), topRef,
        polys, bbox: bboxOf(polys), info: stripHtml(info),
    };
}

/** An ED-269 applicability (or ED-318 limitedApplicability) that has already ended. */
function expired(applicability, now = Date.now()) {
    const list = Array.isArray(applicability) ? applicability : applicability ? [applicability] : [];
    if (!list.length) return false;
    return list.every(a => {
        if (String(a.permanent || '').toUpperCase() === 'YES') return false;
        const end = Date.parse(a.endDateTime || a.endDate || '');
        return isFinite(end) && end < now;
    });
}

/**
 * Zones of an ED-269 file ({ features: [{ restriction, geometry: [{ lowerLimit,
 * horizontalProjection }] }] }) or an ED-318 / GeoJSON one (FeatureCollection,
 * vertical limits and restriction in the properties, possibly in a
 * verticalLayer object).
 */
function parseEdZones(json, src) {
    const features = Array.isArray(json) ? json : (json && Array.isArray(json.features) ? json.features : []);
    const zones = [];
    let n = 0;
    for (const f of features) {
        n++;
        if (!f || typeof f !== 'object') continue;
        if (Array.isArray(f.geometry)) {
            // ED-269: one volume per geometry element
            if (expired(f.applicability)) continue;
            const kind = restrictionKind(f.restriction);
            const base = f.identifier || f.name || `z${n}`;
            f.geometry.forEach((g, i) => {
                const hp = g.horizontalProjection || {};
                let polys = [];
                if (hp.type === 'Circle' && Array.isArray(hp.center)) polys = circlePoly(+hp.center[0], +hp.center[1], +hp.radius || 0);
                else polys = geojsonPolys(hp);
                const z = makeZone({
                    src, key: f.geometry.length > 1 ? `${base}#${i}` : base, name: f.name || f.identifier, kind,
                    floor: toMetres(g.lowerLimit, g.uomDimensions) || 0, floorRef: verticalRef(g.lowerVerticalReference),
                    top: toMetres(g.upperLimit, g.uomDimensions), topRef: verticalRef(g.upperVerticalReference),
                    polys, info: f.message || (Array.isArray(f.restrictionConditions) ? f.restrictionConditions.join(' ') : ''),
                });
                if (z) zones.push(z);
            });
        } else {
            // ED-318 / GeoJSON
            const p = f.properties || {};
            if (expired(p.limitedApplicability || p.applicability)) continue;
            const v = p.verticalLayer || p;
            const uom = v.uom || v.uomDimensions || 'M';
            const z = makeZone({
                src, key: p.identifier || f.id || `z${n}`, name: p.name || p.identifier,
                kind: restrictionKind(p.restriction || p.type),
                floor: toMetres(v.lower ?? v.lowerLimit, uom) || 0, floorRef: verticalRef(v.lowerReference ?? v.lowerVerticalReference),
                top: toMetres(v.upper ?? v.upperLimit, uom), topRef: verticalRef(v.upperReference ?? v.upperVerticalReference),
                polys: geojsonPolys(f.geometry), info: p.message || p.otherReasonInfo || '',
            });
            if (z) zones.push(z);
        }
    }
    return zones;
}

// ── Sources ──────────────────────────────────────────────────────────────────

const sleep = ms => new Promise(r => setTimeout(r, ms));
const busy = (status, json) => status === 429 || status === 503 || json?.error?.code === 429 || /too many requests/i.test(json?.error?.message || '');

/**
 * GET a JSON document. A server that says it is busy (HTTP 429 / 503, or an
 * ArcGIS error 429 in a 200 answer: the FAA's limits several requests at
 * once) is asked again after 2, 4, 8 and 16 s.
 */
async function fetchJson(fetchImpl, url, timeoutMs = TIMEOUT_MS, retryMs = 2000) {
    for (let attempt = 0; ; attempt++) {
        const ctl = new AbortController();
        const timer = setTimeout(() => ctl.abort(), timeoutMs);
        let status, json;
        try {
            const r = await fetchImpl(url, { headers: { 'User-Agent': USER_AGENT, Accept: 'application/json' }, signal: ctl.signal });
            status = r.status;
            const text = await r.text();
            try { json = JSON.parse(text); }
            catch (e) { if (r.ok) throw new Error(`not JSON: ${text.slice(0, 120).replace(/\s+/g, ' ')}`); }
            if (!busy(status, json) && !r.ok) throw new Error(`HTTP ${status}`);
        } finally {
            clearTimeout(timer);
        }
        if (!busy(status, json)) return json;
        if (attempt >= BUSY_RETRIES) throw new Error(json?.error?.message || `HTTP ${status}`);
        await sleep(retryMs * 2 ** attempt);
    }
}

/** Every feature of an ArcGIS layer in a box, as GeoJSON, page by page. */
async function arcgisFeatures(fetchImpl, layerUrl, box, { where = '1=1', outFields = '*', pageSize = 2000 } = {}) {
    const out = [];
    for (let offset = 0; offset < 100000;) {
        const q = new URLSearchParams({
            where, outFields, f: 'geojson', returnGeometry: 'true',
            geometry: box.join(','), geometryType: 'esriGeometryEnvelope', inSR: '4326', outSR: '4326',
            spatialRel: 'esriSpatialRelIntersects', resultOffset: String(offset), resultRecordCount: String(pageSize),
            maxAllowableOffset: String(SIMPLIFY_DEG), geometryPrecision: '6',
        });
        const json = await fetchJson(fetchImpl, `${layerUrl}/query?${q}`);
        if (json.error) throw new Error(json.error.message || 'ArcGIS error');
        const feats = json.features || [];
        out.push(...feats);
        const more = json.exceededTransferLimit || json.properties?.exceededTransferLimit;
        if (!more || !feats.length) break;
        offset += feats.length;
    }
    return out;
}

/** "Surface", "400' AGL", "1200 MSL" → { value m, ref } (FAA national security restrictions). */
function faaLimitText(s, fallbackFt) {
    const t = String(s || '');
    if (/surface|sfc|gnd/i.test(t)) return { value: 0, ref: 'AGL' };
    const m = t.match(/([\d,.]+)/);
    const ft = m ? parseFloat(m[1].replace(/,/g, '')) : fallbackFt;
    return { value: ft * FT, ref: /msl/i.test(t) ? 'MSL' : 'AGL' };
}

/** FAA airspace limits: VAL, UOM, CODE (SFC, AGL, MSL, STD = flight level). */
function faaLimit(val, uom, code) {
    const c = String(code || '').toUpperCase();
    if (c === 'SFC') return { value: 0, ref: 'AGL' };
    let v = Number(val);
    if (!isFinite(v)) return { value: null, ref: 'MSL' };
    if (c === 'STD' || /^FL$/i.test(uom)) v *= 100;
    return { value: toMetres(v, uom || 'FT'), ref: c === 'AGL' ? 'AGL' : 'MSL' };
}

const FAA_BASE = 'https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/ArcGIS/rest/services';
const FAA_LAYERS = [
    {
        id: 'uasfm', url: `${FAA_BASE}/FAA_UAS_FacilityMap_Data/FeatureServer/0`,
        outFields: 'OBJECTID,CEILING,UNIT,APT1_ICAO,APT1_FAAID,APT1_NAME,AIRSPACE_1',
        zone: (p) => {
            const ft = Number(p.CEILING) || 0;
            return {
                key: p.OBJECTID, name: `UAS facility map · ${p.APT1_ICAO || p.APT1_FAAID || p.APT1_NAME || ''} class ${p.AIRSPACE_1 || '?'}`,
                kind: 'prohibited', floor: toMetres(ft, p.UNIT || 'Feet'), floorRef: 'AGL', top: null,
                info: ft > 0 ? `Controlled airspace: LAANC authorisation up to ${ft} ft AGL` : 'Controlled airspace: no LAANC authorisation, ask the FAA (DroneZone)',
            };
        },
    },
    {
        id: 'nsufr', url: `${FAA_BASE}/DoD_Mar_13/FeatureServer/0`,
        outFields: 'OBJECTID,Facility,Base,Reason,Floor,Ceiling',
        zone: (p) => {
            const lo = faaLimitText(p.Floor, 0), hi = faaLimitText(p.Ceiling, 400);
            return { key: p.OBJECTID, name: p.Facility || p.Base, kind: 'prohibited', floor: lo.value, floorRef: lo.ref, top: hi.value, topRef: hi.ref,
                info: `${p.Reason || 'National security'} UAS flight restriction (14 CFR 99.7)` };
        },
    },
    {
        id: 'nsufr-pt', url: `${FAA_BASE}/Part_Time_National_Security_UAS_Flight_Restrictions/FeatureServer/0`,
        outFields: 'OBJECTID,Facility,Base,Reason,Floor,Ceiling,ALERTYPE',
        zone: (p) => {
            const lo = faaLimitText(p.Floor, 0), hi = faaLimitText(p.Ceiling, 400);
            return { key: p.OBJECTID, name: p.Facility || p.Base, kind: 'conditional', floor: lo.value, floorRef: lo.ref, top: hi.value, topRef: hi.ref,
                info: `Part-time national security restriction (${p.ALERTYPE || 'intermittent'}): active when published by NOTAM` };
        },
    },
    {
        id: 'p', url: `${FAA_BASE}/Prohibited_Areas/FeatureServer/0`,
        outFields: 'OBJECTID,NAME,CITY,UPPER_VAL,UPPER_UOM,UPPER_CODE,LOWER_VAL,LOWER_UOM,LOWER_CODE,TIMESOFUSE',
        zone: (p) => {
            const lo = faaLimit(p.LOWER_VAL, p.LOWER_UOM, p.LOWER_CODE), hi = faaLimit(p.UPPER_VAL, p.UPPER_UOM, p.UPPER_CODE);
            return { key: p.OBJECTID, name: `${p.NAME} ${p.CITY || ''}`, kind: 'prohibited', floor: lo.value || 0, floorRef: lo.ref, top: hi.value, topRef: hi.ref,
                info: `Prohibited area · ${p.TIMESOFUSE || ''}` };
        },
    },
    {
        id: 'r', url: `${FAA_BASE}/Special_Use_Airspace/FeatureServer/0`, where: "TYPE_CODE='R'",
        outFields: 'OBJECTID,NAME,UPPER_VAL,UPPER_UOM,UPPER_CODE,LOWER_VAL,LOWER_UOM,LOWER_CODE,TIMESOFUSE',
        zone: (p) => {
            const lo = faaLimit(p.LOWER_VAL, p.LOWER_UOM, p.LOWER_CODE), hi = faaLimit(p.UPPER_VAL, p.UPPER_UOM, p.UPPER_CODE);
            return { key: p.OBJECTID, name: p.NAME, kind: 'conditional', floor: lo.value || 0, floorRef: lo.ref, top: hi.value, topRef: hi.ref,
                info: `Restricted area, when active · ${p.TIMESOFUSE || ''}` };
        },
    },
    {
        id: 'nda', url: `${FAA_BASE}/National_Defense_Airspace_TFR_Areas/FeatureServer/0`,
        outFields: 'OBJECTID,NAME,WKHR_RMK',
        zone: (p) => ({ key: p.OBJECTID, name: p.NAME, kind: 'conditional', floor: 0, floorRef: 'AGL', top: null,
            info: `National defense airspace TFR · ${p.WKHR_RMK || 'by NOTAM'}` }),
    },
];

async function fetchFaaTile(fetchImpl, box) {
    const zones = [];
    for (const layer of FAA_LAYERS) {
        const feats = await arcgisFeatures(fetchImpl, layer.url, box, { where: layer.where, outFields: layer.outFields });
        for (const f of feats) {
            const d = layer.zone(f.properties || {});
            const z = makeZone({ src: 'faa', ...d, key: `${layer.id}-${d.key ?? f.id}`, polys: geojsonPolys(f.geometry) });
            if (z) zones.push(z);
        }
    }
    return zones;
}

const ENAIRE_URL = 'https://servais.enaire.es/insignia/rest/services/NSF_SRV/SRV_UAS_ZG_V1/MapServer';
const ENAIRE_LAYERS = [0, 2, 3];   // infrastructure, aeronautical, urban

async function fetchEnaireTile(fetchImpl, box) {
    const zones = [];
    for (const layer of ENAIRE_LAYERS) {
        const feats = await arcgisFeatures(fetchImpl, `${ENAIRE_URL}/${layer}`, box, { pageSize: 1000 });
        for (const f of feats) {
            const p = f.properties || {};
            const z = makeZone({
                src: 'enaire', key: `${layer}-${p.GFID || p.OBJECTID || f.id}`, name: p.name || p.identifier || p.otherReasonInfo,
                kind: restrictionKind(p.type), floor: toMetres(p.lower, p.uom) || 0, floorRef: verticalRef(p.lowerReference),
                top: toMetres(p.upper, p.uom), topRef: verticalRef(p.upperReference),
                polys: geojsonPolys(f.geometry), info: p.otherReasonInfo ? `${p.otherReasonInfo}. ${p.message || ''}` : p.message,
            });
            if (z) zones.push(z);
        }
    }
    return zones;
}

const DIPUL_URL = 'https://uas-betrieb.de/geoservices/dipul/wfs';
// Layer → kind. §21h LuftVO: flight prohibited unless permitted (airports,
// control zones, sensitive sites) or under conditions set by the state (nature)
const DIPUL_LAYERS = {
    flughaefen: 'prohibited', flugplaetze: 'prohibited', kontrollzonen: 'prohibited',
    flugbeschraenkungsgebiete: 'prohibited', temporaere_betriebseinschraenkungen: 'prohibited',
    militaerische_anlagen: 'prohibited', justizvollzugsanstalten: 'prohibited', kraftwerke: 'prohibited',
    umspannwerke: 'prohibited', industrieanlagen: 'prohibited', sicherheitsbehoerden: 'prohibited',
    polizei: 'prohibited', behoerden: 'prohibited', diplomatische_vertretungen: 'prohibited',
    internationale_organisationen: 'prohibited', labore: 'prohibited', krankenhaeuser: 'prohibited',
    naturschutzgebiete: 'conditional', nationalparks: 'conditional', 'ffh-gebiete': 'conditional',
    vogelschutzgebiete: 'conditional', freibaeder: 'conditional',
};

async function fetchDipulTile(fetchImpl, box) {
    const zones = [];
    for (const [layer, kind] of Object.entries(DIPUL_LAYERS)) {
        const q = new URLSearchParams({
            SERVICE: 'WFS', VERSION: '2.0.0', REQUEST: 'GetFeature', TYPENAMES: `dipul:${layer}`,
            outputFormat: 'application/json', srsName: 'EPSG:4326', BBOX: `${box.join(',')},EPSG:4326`, count: '20000',
        });
        const json = await fetchJson(fetchImpl, `${DIPUL_URL}?${q}`);
        for (const f of json.features || []) {
            const p = f.properties || {};
            const z = makeZone({
                src: 'dipul', key: f.id || `${layer}-${p.external_reference}`, name: p.name || p.generated_name_EN, kind,
                floor: toMetres(p.lower_limit_altitude, p.lower_limit_unit) || 0, floorRef: verticalRef(p.lower_limit_alt_ref),
                top: p.upper_limit_altitude == null ? null : toMetres(p.upper_limit_altitude, p.upper_limit_unit), topRef: verticalRef(p.upper_limit_alt_ref),
                polys: geojsonPolys(f.geometry), info: [p.type_code_detail || p.type_code, p.legal_ref].filter(Boolean).join(' · '),
            });
            if (z) zones.push(z);
        }
    }
    return zones;
}

const IGN_URL = 'https://data.geopf.fr/wfs/ows';

async function fetchIgnTile(fetchImpl, box) {
    const q = new URLSearchParams({
        SERVICE: 'WFS', VERSION: '2.0.0', REQUEST: 'GetFeature', TYPENAMES: 'TRANSPORTS.DRONES.RESTRICTIONS:carte_restriction_drones_lf',
        outputFormat: 'application/json', srsName: 'EPSG:4326', BBOX: `${box.join(',')},EPSG:4326`, count: '20000',
    });
    const json = await fetchJson(fetchImpl, `${IGN_URL}?${q}`);
    const zones = [];
    for (const f of json.features || []) {
        const p = f.properties || {};
        const limit = String(p.limite || '');
        const m = limit.match(/(\d+)\s*m/);
        if (!m && !/interdit/i.test(limit)) continue;
        const z = makeZone({
            src: 'ign', key: f.id, name: limit.replace(/\s*\*$/, ''), kind: 'prohibited',
            floor: m ? +m[1] : 0, floorRef: 'AGL', top: null, polys: geojsonPolys(f.geometry), info: p.remarque,
        });
        if (z) zones.push(z);
    }
    return zones;
}

const BAZL_URL = 'https://data.geo.admin.ch/ch.bazl.einschraenkungen-drohnen/einschraenkungen-drohnen/einschraenkungen-drohnen_4326.json';

/** The sources, with the boxes ([w, s, e, n]) of the countries they cover. */
const PROVIDERS = [
    { id: 'faa', name: 'FAA', country: 'US', tileDeg: 0.5, fetchTile: fetchFaaTile, parallel: 2,
        coverage: [[-125, 24, -66.5, 49.5], [-170, 51, -129, 72], [-161, 18.5, -154, 22.5], [-68, 17.5, -65, 18.7]] },
    { id: 'dipul', name: 'DFS DIPUL', country: 'DE', tileDeg: 1, fetchTile: fetchDipulTile, coverage: [[5.8, 47.2, 15.1, 55.1]] },
    { id: 'ign', name: 'IGN / DGAC', country: 'FR', tileDeg: 0.5, fetchTile: fetchIgnTile, coverage: [[-5.3, 41.3, 9.7, 51.2]] },
    { id: 'enaire', name: 'ENAIRE', country: 'ES', tileDeg: 0.5, fetchTile: fetchEnaireTile,
        coverage: [[-9.5, 35.9, 4.4, 43.9], [-18.2, 27.6, -13.3, 29.5]] },
    { id: 'bazl', name: 'FOCA / BAZL', country: 'CH', file: BAZL_URL, coverage: [[5.9, 45.8, 10.5, 47.9]] },
];

// ── Store ────────────────────────────────────────────────────────────────────

function createGeoZones({ dir, fetch: fetchImpl = globalThis.fetch, onChange = () => {} }) {
    const importDir = path.join(dir, 'import');
    const mem = new Map();          // "provider/tile" → { at, zones } | { failedAt, error }
    const files = new Map();        // provider id / import name → { at, zones }
    const queue = [];               // tiles waiting to be fetched
    const inFlight = new Set();
    const status = {};              // provider id → { lastFetch, lastError, fetched, failed }
    for (const p of PROVIDERS) status[p.id] = { lastFetch: null, lastError: null, fetched: 0, failed: 0 };
    let active = 0, changeTimer = null;

    function readJson(file) { try { return JSON.parse(fs.readFileSync(file, 'utf8')); } catch (e) { return null; } }
    function writeJson(file, value) {
        fs.mkdirSync(path.dirname(file), { recursive: true });
        const tmp = `${file}.tmp`;
        fs.writeFileSync(tmp, JSON.stringify(value));
        fs.renameSync(tmp, file);
    }
    function changed() {
        clearTimeout(changeTimer);
        changeTimer = setTimeout(() => onChange(), 400);
    }

    function tileFile(p, key) { return path.join(dir, p.id, `${key}.json`); }
    function tileKey(p, tx, ty) { return `${p.tileDeg}_${tx}_${ty}`; }
    function tileBox(p, tx, ty) { return [tx * p.tileDeg, ty * p.tileDeg, (tx + 1) * p.tileDeg, (ty + 1) * p.tileDeg]; }

    function tile(p, key) {
        const id = `${p.id}/${key}`;
        let t = mem.get(id);
        if (!t) {
            t = readJson(tileFile(p, key)) || {};
            mem.set(id, t);
        }
        return t;
    }

    const activeBy = {};            // provider id → tiles being fetched
    function pump() {
        while (active < CONCURRENCY) {
            const i = queue.findIndex(j => (activeBy[j.p.id] || 0) < (j.p.parallel || CONCURRENCY));
            if (i < 0) break;
            const job = queue.splice(i, 1)[0];
            active++;
            activeBy[job.p.id] = (activeBy[job.p.id] || 0) + 1;
            const t0 = Date.now();
            job.p.fetchTile(fetchImpl, job.box).then((zones) => {
                const t = { at: Date.now(), zones };
                mem.set(job.id, t);
                writeJson(tileFile(job.p, job.key), t);
                Object.assign(status[job.p.id], { lastFetch: t.at, lastError: null });
                status[job.p.id].fetched++;
                console.log(`[geozones] ${job.id}: ${zones.length} zones, ${((Date.now() - t0) / 1000).toFixed(1)} s`);
            }).catch((e) => {
                const t = mem.get(job.id) || {};
                t.failedAt = Date.now();
                t.error = String(e.message || e);
                mem.set(job.id, t);
                Object.assign(status[job.p.id], { lastError: t.error });
                status[job.p.id].failed++;
                console.warn(`[geozones] ${job.id} failed: ${t.error}`);
            }).finally(() => {
                active--;
                activeBy[job.p.id]--;
                inFlight.delete(job.id);
                changed();
                pump();
            });
        }
    }

    function want(p, tx, ty, now) {
        const key = tileKey(p, tx, ty), id = `${p.id}/${key}`;
        const t = tile(p, key);
        const stale = !t.at || now - t.at > TTL_MS;
        const backoff = t.failedAt && now - t.failedAt < RETRY_MS;
        if (stale && !backoff && !inFlight.has(id)) {
            inFlight.add(id);
            queue.push({ p, key, id, box: tileBox(p, tx, ty) });
        }
        return t;
    }

    /** A whole-country file: from memory, disk, or downloaded (in the background). */
    function countryFile(p, now) {
        let f = files.get(p.id);
        if (!f) {
            f = readJson(path.join(dir, `${p.id}.json`)) || {};
            files.set(p.id, f);
        }
        const stale = !f.at || now - f.at > FILE_TTL_MS;
        const backoff = f.failedAt && now - f.failedAt < RETRY_MS;
        if (stale && !backoff && !inFlight.has(p.id)) {
            inFlight.add(p.id);
            const t0 = Date.now();
            fetchJson(fetchImpl, p.file, FILE_TIMEOUT_MS).then((json) => {
                const next = { at: Date.now(), zones: parseEdZones(json, p.id) };
                files.set(p.id, next);
                writeJson(path.join(dir, `${p.id}.json`), next);
                Object.assign(status[p.id], { lastFetch: next.at, lastError: null });
                status[p.id].fetched++;
                console.log(`[geozones] ${p.id}: ${next.zones.length} zones, ${((Date.now() - t0) / 1000).toFixed(1)} s`);
            }).catch((e) => {
                f.failedAt = Date.now();
                Object.assign(status[p.id], { lastError: String(e.message || e) });
                status[p.id].failed++;
                console.warn(`[geozones] ${p.id} failed: ${e.message || e}`);
            }).finally(() => { inFlight.delete(p.id); changed(); });
        }
        return f;
    }

    function loadImports() {
        if (files.has('import:loaded')) return;
        files.set('import:loaded', true);
        let names = [];
        try { names = fs.readdirSync(importDir).filter(n => n.endsWith('.json')); } catch (e) { /* none yet */ }
        for (const n of names) {
            const f = readJson(path.join(importDir, n));
            if (f && Array.isArray(f.zones)) files.set(`import:${n}`, f);
        }
    }

    function imports() {
        loadImports();
        return [...files.entries()].filter(([k]) => k.startsWith('import:') && k !== 'import:loaded')
            .map(([k, f]) => ({ file: k.slice(7), name: f.name, at: f.at, count: f.zones.length }));
    }

    /**
     * Zones within the circles ([{ lat, lon, radiusKm }]): what is on disk now;
     * missing or stale tiles are fetched in the background (onChange follows).
     */
    function query({ circles = [] } = {}) {
        const now = Date.now();
        circles = circles.filter(c => isFinite(c.lat) && isFinite(c.lon) && c.radiusKm > 0);
        const byId = new Map();
        const sources = [];
        const add = (zones) => {
            for (const z of zones || []) if (!byId.has(z.id) && nearAnyCircle(z, circles)) byId.set(z.id, z);
        };
        if (circles.length) {
            const boxes = circles.map(circleBox);
            for (const p of PROVIDERS) {
                const cover = p.coverage.filter(c => boxes.some(b => boxesIntersect(b, c)));
                if (!cover.length) continue;
                const src = { id: p.id, name: p.name, country: p.country, tiles: 0, ready: 0, pending: 0, error: status[p.id].lastError };
                sources.push(src);
                if (p.file) {
                    const f = countryFile(p, now);
                    src.tiles = 1;
                    if (f.zones) { src.ready = 1; add(f.zones); }
                    if (inFlight.has(p.id)) src.pending = 1;
                    continue;
                }
                const seen = new Set();
                for (const b of boxes) {
                    for (const c of cover) {
                        if (!boxesIntersect(b, c)) continue;
                        const w = Math.max(b[0], c[0]), s = Math.max(b[1], c[1]), e = Math.min(b[2], c[2]), n = Math.min(b[3], c[3]);
                        for (let ty = Math.floor(s / p.tileDeg); ty <= Math.floor(n / p.tileDeg); ty++) {
                            for (let tx = Math.floor(w / p.tileDeg); tx <= Math.floor(e / p.tileDeg); tx++) {
                                const key = `${tx}_${ty}`;
                                if (seen.has(key) || seen.size >= MAX_TILES) continue;
                                seen.add(key);
                                const t = want(p, tx, ty, now);
                                src.tiles++;
                                if (t.zones) { src.ready++; add(t.zones); }
                                if (inFlight.has(`${p.id}/${tileKey(p, tx, ty)}`)) src.pending++;
                            }
                        }
                    }
                }
            }
            loadImports();
            for (const [k, f] of files) {
                if (k.startsWith('import:') && k !== 'import:loaded') add(f.zones);
            }
            pump();
        }
        return { zones: [...byId.values()], sources, imports: imports(), pending: inFlight.size };
    }

    /** Read an ED-269 / ED-318 file and keep its zones. */
    function importFile(file) {
        const json = JSON.parse(fs.readFileSync(file, 'utf8').replace(/^﻿/, ''));
        const base = path.basename(file).replace(/\.[^.]+$/, '').replace(/[^\w.-]+/g, '_');
        const zones = parseEdZones(json, `import-${base}`);
        if (!zones.length) throw new Error('No ED-269 / ED-318 zones in this file');
        const f = { name: path.basename(file), at: Date.now(), zones };
        writeJson(path.join(importDir, `${base}.json`), f);
        files.set(`import:${base}.json`, f);
        changed();
        return { name: f.name, count: zones.length };
    }

    function removeImport(name) {
        try { fs.unlinkSync(path.join(importDir, path.basename(name))); } catch (e) { /* gone */ }
        files.delete(`import:${path.basename(name)}`);
        changed();
    }

    /** Forget every downloaded tile and country file (not the imports): fetched again on the next query. */
    function clear() {
        for (const p of PROVIDERS) {
            try { fs.rmSync(path.join(dir, p.id), { recursive: true, force: true }); } catch (e) { /* ignore */ }
            try { fs.rmSync(path.join(dir, `${p.id}.json`), { force: true }); } catch (e) { /* ignore */ }
            files.delete(p.id);
        }
        mem.clear();
        changed();
    }

    function getStatus() {
        return {
            providers: PROVIDERS.map(p => ({ id: p.id, name: p.name, country: p.country, ...status[p.id] })),
            imports: imports(), pending: inFlight.size,
        };
    }

    return { query, importFile, removeImport, clear, status: getStatus };
}

// ── IPC ──────────────────────────────────────────────────────────────────────

function initGeoZonesHandlers(win) {
    const { ipcMain, dialog } = require('electron');
    const dir = path.join(require('./mission-store').getRoot(), 'geozones');
    const store = createGeoZones({
        dir,
        onChange: () => { if (win && !win.isDestroyed()) win.webContents.send('geozones-changed'); },
    });
    ipcMain.handle('geozones-query', (event, q) => store.query(q || {}));
    ipcMain.handle('geozones-status', () => store.status());
    ipcMain.handle('geozones-clear', () => { store.clear(); return store.status(); });
    ipcMain.handle('geozones-remove-import', (event, name) => { store.removeImport(name); return store.status(); });
    ipcMain.handle('geozones-import', async () => {
        const r = await dialog.showOpenDialog(win, {
            title: 'Import UAS geographical zones (ED-269 / ED-318)',
            filters: [{ name: 'ED-269 / ED-318 JSON', extensions: ['json', 'geojson'] }],
            properties: ['openFile'],
        });
        if (r.canceled || !r.filePaths.length) return null;
        try {
            return { ...store.importFile(r.filePaths[0]), status: store.status() };
        } catch (e) {
            return { error: String(e.message || e), status: store.status() };
        }
    });
}

module.exports = {
    createGeoZones, initGeoZonesHandlers,
    // exported for scripts/test-geozones.js
    parseEdZones, geojsonPolys, simplifyRing, restrictionKind, faaLimit, faaLimitText, fetchJson, PROVIDERS, FAA_LAYERS,
};
