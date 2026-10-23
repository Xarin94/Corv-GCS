/**
 * SceneOrigin.js - where the 3D world metres count from
 *
 * latLonToMeters() measures from ORIGIN. The GPU holds vertex positions in
 * float32: 10 000 km from the origin (a SITL home in Australia seen from the
 * default origin) they are 1 m apart, and the trail and the route are drawn in
 * north-south / east-west steps. So the origin goes where the vehicle is:
 *  - every data stream that opens (serial, UDP, TCP, LTE, MSP, a log) takes
 *    its first valid position, again after it is closed and reopened: the
 *    vehicle may already be flying, and HOME_POSITION may never come (the
 *    request goes unanswered on a one-way link);
 *  - a HOME_POSITION moves it to that home.
 * What is held in world metres is rebuilt by its owner through
 * onSceneOriginChange().
 */

import { ORIGIN } from './constants.js';

const listeners = [];
let awaitingFix = false;

/**
 * fn(origin) runs after the origin moved: terrain, trails, route and markers
 * in the old world metres are rebuilt or dropped there.
 * @param {Function} fn
 */
export function onSceneOriginChange(fn) {
    listeners.push(fn);
}

/**
 * Move the origin to a point.
 * @returns {boolean} true when the origin moved
 */
export function setSceneOrigin(lat, lon) {
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return false;
    if (lat === ORIGIN.lat && lon === ORIGIN.lon) return false;
    ORIGIN.lat = lat;
    ORIGIN.lon = lon;
    console.log(`[origin] scene origin moved to ${lat.toFixed(7)}, ${lon.toFixed(7)}`);
    for (const fn of listeners) {
        try {
            fn(ORIGIN);
        } catch (e) {
            console.error('[origin] rebuild after the origin move failed:', e);
        }
    }
    return true;
}

/** A data stream opened: its next valid position becomes the origin. */
export function awaitSceneOriginFix() {
    awaitingFix = true;
}

/** A valid position from the stream: the first one after it opened moves the origin there. */
export function sceneOriginFix(lat, lon) {
    if (!awaitingFix || !Number.isFinite(lat) || !Number.isFinite(lon)) return;
    awaitingFix = false;
    setSceneOrigin(lat, lon);
}

/** HOME_POSITION from the vehicle: the origin goes to that home. */
export function sceneOriginHome(lat, lon) {
    awaitingFix = false;
    setSceneOrigin(lat, lon);
}
