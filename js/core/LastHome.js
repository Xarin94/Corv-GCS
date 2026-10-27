/**
 * LastHome.js - where the last vehicle was based, kept across sessions
 *
 * The flight plan opens where the vehicle is. With no vehicle on the link it
 * opens at the last home a vehicle reported (for a link that sends no
 * HOME_POSITION: where the vehicle was when the link opened), not at the
 * demo's Innsbruck.
 */

const LS_KEY = 'corv.lastHome';

/** A home from a live link (not a replayed log). */
export function rememberLastHome(lat, lon) {
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const last = lastHome();
    if (last && Math.abs(last.lat - lat) < 1e-7 && Math.abs(last.lng - lon) < 1e-7) return;
    try { localStorage.setItem(LS_KEY, JSON.stringify({ lat, lng: lon })); } catch { /* storage unavailable */ }
}

/** @returns {{lat:number, lng:number}|null} */
export function lastHome() {
    try {
        const p = JSON.parse(localStorage.getItem(LS_KEY) || 'null');
        if (p && Number.isFinite(p.lat) && Number.isFinite(p.lng)) return { lat: p.lat, lng: p.lng };
    } catch { /* storage unavailable or corrupt */ }
    return null;
}
