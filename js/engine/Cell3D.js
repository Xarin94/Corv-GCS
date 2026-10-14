/**
 * Cell3D.js - LTE cell sites and the link to the serving cell in the 3D view
 *
 * Same sites as the mini-map overlay (CellTowerLayer.js, live only while an
 * LTEtelem module reports its cell): the serving site and the others within
 * 10 km of the vehicle, each a thin red pole 10 m tall standing on the
 * terrain, and a dashed line from the vehicle to the top of the serving
 * site's pole — green, orange or red by the link quality, the same as on the
 * mini-map — dimmer where a ridge hides it. The line is drawn in the chase
 * view only: in the first-person view it starts at the camera and would
 * shrink to a point (the mini-map always has it).
 *
 * All poles are one ThickLine in 'pairs' mode (one draw call). A pole needs
 * the terrain height under it: sites over HGT tiles that are not loaded yet
 * are left out and tried again every few seconds.
 */

import { ThickLine } from './ThickLine.js';
import { latLonToMeters } from '../core/utils.js';
import { STATE } from '../core/state.js';
import { isRelativeMode } from '../core/RelativeNav.js';
import { getTerrainElevationFromHGT } from '../terrain/TerrainManager.js';
import { getCellOverlay, linkQuality } from '../maps/CellTowerLayer.js';

const POLE_M = 10;
const POLE_COLOR = 0xff2020;
const RETRY_MS = 3000;           // poles without terrain height: try again

let poles = null;
let link = null;
let linkColor = null;
let firstPerson = true;

/** First-person (cockpit) view or chase view: the link line shows in the chase view only. */
export function setCell3DFirstPerson(on) {
    firstPerson = !!on;
}
let builtVersion = -1;
let builtOffset = null;
let builtAt = 0;
let missingGround = false;
const servingTop = { x: 0, y: 0, z: 0, valid: false };
let servingRef = null;

/** @param {THREE.Scene} scene */
export function initCell3D(scene) {
    poles = new ThickLine({ mode: 'pairs', color: POLE_COLOR, width: 2, renderOrder: 5 }).addTo(scene);
    link = new ThickLine({
        color: 0xffffff, width: 2, dash: [10, 7], ghost: { width: 1.5, opacity: 0.35 }, renderOrder: 6
    }).addTo(scene);
    poles.setVisible(false);
    link.setVisible(false);
}

function hide() {
    if (poles.visible) poles.setVisible(false);
    if (link.visible) link.setVisible(false);
    builtVersion = -1;
}

function buildPoles(sites, offset) {
    const flat = new Float32Array(sites.length * 6);
    let n = 0;
    missingGround = false;
    for (const s of sites) {
        const ground = getTerrainElevationFromHGT(s.lat, s.lon);
        if (ground === null) { missingGround = true; continue; }
        const p = latLonToMeters(s.lat, s.lon);
        const base = ground + offset;
        flat[n * 6] = p.x; flat[n * 6 + 1] = base; flat[n * 6 + 2] = p.z;
        flat[n * 6 + 3] = p.x; flat[n * 6 + 4] = base + POLE_M; flat[n * 6 + 5] = p.z;
        n++;
    }
    if (n) poles.setPoints(flat, n * 2);
    else poles.clear();
}

function placeServing(site, offset) {
    servingRef = site;
    servingTop.valid = false;
    if (!site) return;
    const ground = getTerrainElevationFromHGT(site.lat, site.lon);
    if (ground === null) return;
    const p = latLonToMeters(site.lat, site.lon);
    servingTop.x = p.x; servingTop.y = ground + offset + POLE_M; servingTop.z = p.z;
    servingTop.valid = true;
}

/** Every frame, before the lines are flushed. */
export function animateCell3D() {
    if (!poles) return;
    const ov = getCellOverlay();
    if (!ov.live || isRelativeMode()) { hide(); return; }

    const offset = STATE.offsetAlt || 0;
    const now = performance.now();
    const rebuild = ov.version !== builtVersion || offset !== builtOffset || (missingGround && now - builtAt > RETRY_MS);
    if (rebuild) {
        builtVersion = ov.version;
        builtOffset = offset;
        builtAt = now;
        buildPoles(ov.sites, offset);
        servingRef = undefined;          // placed again below with the new ground
    }
    if (!poles.visible) poles.setVisible(true);

    if (ov.serving !== servingRef || (ov.serving && !servingTop.valid && rebuild)) placeServing(ov.serving, offset);
    if (!servingTop.valid || firstPerson) {
        if (link.visible) link.setVisible(false);
        return;
    }
    const { color } = linkQuality();
    if (color !== linkColor) {
        linkColor = color;
        link.setColor(color);
    }
    const v = latLonToMeters(STATE.lat, STATE.lon);
    link.setPoints([{ x: v.x, y: STATE.rawAlt + offset, z: v.z }, servingTop]);
    if (!link.visible) link.setVisible(true);
}
