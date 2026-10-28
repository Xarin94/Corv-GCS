/**
 * GeoZones.js - UAS geographical zones around the mission and the vehicle
 *
 * GCS OPTIONS → RESTRICTED ZONES (off by default, remembered). On, the zones
 * within RADIUS_KM of the connected vehicle and of every point of the mission
 * are asked of geozones.js (main process: FAA, DFS DIPUL, IGN, ENAIRE, BAZL,
 * imported ED-269 files — only the sources whose country is in reach). Asked
 * again when the vehicle has moved REQUERY_KM, when the mission changes, when
 * the main process has fetched new tiles, and every REFRESH_MS so tiles past
 * their age are fetched again during a long session. No vehicle position is
 * taken from the demo flight.
 *
 * The zones go to the 2D maps (maps/GeoZoneLayer.js) and the 3D terrain
 * (terrain/GeoZoneTerrain.js). For a flying vehicle (copter, plane) with a
 * position, geoZoneStatus says what the zones and the region's general limit
 * (120 m in Europe, 400 ft in the US) make of its height above the ground —
 * the TERR ALT figure — read by the zone annunciators beside the altitude
 * tape (ui/GeoZonePanel.js) and by the TERR ALT cell, which flashes when a
 * height limit is broken. Arming inside a zone raises geoZoneStatus.armAlert,
 * a banner of its own (no-fly red, conditions or a ceiling amber).
 */

import { STATE, isDemoMode } from '../core/state.js';
import { isRelativeMode } from '../core/RelativeNav.js';
import { streamHasFix } from '../core/SceneOrigin.js';
import { getRoute } from '../mission/RouteModel.js';
import { getVehicleTypeName } from '../mavlink/MAVLinkStateMapper.js';
import { evaluateGeoZones, armingAlert } from './GeoZoneRules.js';
import { setGeoZoneTerrain } from '../terrain/GeoZoneTerrain.js';

const STORAGE_KEY = 'geozones-enabled';
export const RADIUS_KM = 50;
const REQUERY_KM = 5;
const MISSION_SPACING_KM = 10;       // mission points closer than this to one already asked share its circle
const REFRESH_MS = 10 * 60 * 1000;
const EVALUATE_MS = 250;
const AIRBORNE_AGL = 3;

let enabled = false;
let data = { zones: [], sources: [], imports: [], pending: 0 };
let version = 0;
let lastQueryAt = 0, vehicleCentre = null, querying = false, queryAgain = false;
let lastEvalAt = 0, missionTimer = null;
let wasArmed = null;                 // null until the first evaluation of a link: an armed vehicle met on connecting is not an arming
const listeners = new Set();

export const geoZoneStatus = {
    active: false, region: null, limit: null, agl: null, alerts: [], zonesHere: [], overHeight: false,
    armAlert: null,                  // { level, title, detail, zone, at } on arming inside a zone
};

function loadEnabled() {
    try { return localStorage.getItem(STORAGE_KEY) === '1'; } catch (e) { return false; }
}

function kmBetween(a, b) {
    const k = Math.cos(a.lat * Math.PI / 180);
    return Math.hypot((a.lat - b.lat) * 111.32, (a.lon - b.lon) * 111.32 * k);
}

/** The vehicle's position, when there is a real one (a link or a log, a fix, not the relative frame). */
function vehiclePosition() {
    if (isDemoMode() || isRelativeMode() || !streamHasFix()) return null;
    if (!isFinite(STATE.lat) || !isFinite(STATE.lon) || (STATE.lat === 0 && STATE.lon === 0)) return null;
    return { lat: STATE.lat, lon: STATE.lon };
}

/** One circle per mission point not already within MISSION_SPACING_KM of an earlier one. */
function missionCentres() {
    const pts = [];
    for (const seg of getRoute().segments) for (const p of seg.points) pts.push({ lat: p.lat, lon: p.lng });
    for (const it of STATE.missionItems) if (it.lat || it.lng) pts.push({ lat: it.lat, lon: it.lng });
    const out = [];
    for (const p of pts) {
        if (!isFinite(p.lat) || !isFinite(p.lon)) continue;
        if (!out.some(c => kmBetween(c, p) < MISSION_SPACING_KM)) out.push(p);
    }
    return out;
}

function notify() {
    version++;
    setGeoZoneTerrain(data.zones, version, enabled && !isRelativeMode());
    for (const cb of listeners) cb();
}

async function query() {
    if (!enabled || !window.geoZones) return;
    if (querying) { queryAgain = true; return; }
    querying = true;
    lastQueryAt = Date.now();
    try {
        const v = vehiclePosition();
        vehicleCentre = v;
        const centres = [...(v ? [v] : []), ...missionCentres()];
        const r = centres.length
            ? await window.geoZones.query({ circles: centres.map(c => ({ ...c, radiusKm: RADIUS_KM })) })
            : { zones: [], sources: [], imports: data.imports, pending: 0 };
        if (enabled) {
            data = r;
            notify();
        }
    } catch (e) {
        console.warn('[geozones] query failed:', e);
    } finally {
        querying = false;
        if (queryAgain) { queryAgain = false; query(); }
    }
}

export function initGeoZones() {
    enabled = loadEnabled();
    window.geoZones?.onChanged(() => query());
    window.addEventListener('missionUpdated', () => {
        clearTimeout(missionTimer);
        missionTimer = setTimeout(query, 1500);
    });
    window.addEventListener('relNavChanged', () => notify());
    window.addEventListener('vehicleConnected', () => { wasArmed = null; });
    if (enabled) query();
    notify();
}

export function geoZonesEnabled() { return enabled; }

export function setGeoZonesEnabled(on) {
    enabled = !!on;
    try { localStorage.setItem(STORAGE_KEY, enabled ? '1' : '0'); } catch (e) { /* ignore */ }
    console.log(`[geozones] ${enabled ? 'on' : 'off'}`);
    if (enabled) query();
    else data = { ...data, zones: [], sources: [], pending: 0 };
    notify();
}

/** Ask the main process again now (after an import, a removal, a cache clear). */
export function refreshGeoZones() { query(); }

export function onGeoZonesChange(cb) { listeners.add(cb); }

/** What the maps and the settings panel show. */
export function getGeoZoneData() {
    return { enabled, version, ...data, radiusKm: RADIUS_KM, centres: vehicleCentre ? 1 + missionCentres().length : missionCentres().length };
}

function clearStatus() {
    if (!geoZoneStatus.active && !geoZoneStatus.alerts.length) return;
    Object.assign(geoZoneStatus, { active: false, region: null, limit: null, agl: null, alerts: [], zonesHere: [], overHeight: false });
}

/** 10 Hz from the main loop: follow the vehicle, refresh, evaluate. */
export function updateGeoZones() {
    if (!enabled) { clearStatus(); wasArmed = null; return; }
    const now = Date.now();
    const v = vehiclePosition();
    if (v ? (!vehicleCentre || kmBetween(v, vehicleCentre) > REQUERY_KM) : !!vehicleCentre) query();
    else if (now - lastQueryAt > REFRESH_MS) query();

    if (now - lastEvalAt < EVALUATE_MS) return;
    lastEvalAt = now;
    const type = getVehicleTypeName(STATE.vehicleType);
    if (!v || (type !== 'Copter' && type !== 'Plane')) { clearStatus(); wasArmed = null; return; }
    const alt = STATE.rawAlt + STATE.offsetAlt;
    const terrain = STATE.terrainHeight;
    const agl = terrain !== null ? alt - terrain : (STATE.homeAlt !== null ? alt - STATE.homeAlt : null);
    // A floor above sea level needs the ground under the vehicle: home's when the terrain is not loaded
    const r = evaluateGeoZones({
        zones: data.zones, lat: v.lat, lon: v.lon, agl, terrain: terrain ?? STATE.homeAlt,
        airborne: STATE.armed || (agl !== null && agl > AIRBORNE_AGL),
    });
    Object.assign(geoZoneStatus, r, { active: true, agl });
    if (wasArmed === false && STATE.armed) {
        const a = armingAlert(r.zonesHere, terrain ?? STATE.homeAlt, r.limit);
        if (a) {
            geoZoneStatus.armAlert = { ...a, at: now };
            (a.level === 'warn' ? console.warn : console.log)(`[geozones] ${a.title}: ${a.detail}`);
        }
    }
    wasArmed = STATE.armed;
}
