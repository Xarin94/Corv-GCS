/**
 * RadioLink.js - Ground ↔ aircraft radio link over the terrain
 *
 * Pure link-budget and geometry maths, no DOM: the Flight Plan page asks it
 * three questions.
 *   evaluateLink()    – one point: received power, margin, line of sight and
 *                       first-Fresnel-zone clearance against the SRTM terrain
 *                       between the ground antenna and the aircraft
 *   analyzeRoute()    – the same along the calculated flight path, as runs of
 *                       equal link quality for the map halo and the profile band
 *   computeCoverage() – a polar raster around the operator at the planned
 *                       altitude, for the coverage overlay on the map
 *
 * Model: free-space path loss, single knife-edge diffraction (ITU-R P.526) on
 * the terrain sample that intrudes most into the first Fresnel zone, 4/3 earth
 * radius for the curvature bulge, omnidirectional antennas. Good enough to tell
 * "the ridge at 3 km shadows the far end of the survey" — not a propagation
 * study.
 *
 * Link classes, from the operator's point of view:
 *   GOOD      margin ≥ safety margin and the 60 % Fresnel zone is clear
 *   DEGRADED  margin still ≥ safety margin but the terrain intrudes into the
 *             Fresnel zone or blocks the line of sight (diffraction path)
 *   MARGINAL  link possible (above the sensitivity) but below the safety margin
 *   NONE      received power under the receiver sensitivity
 *   UNKNOWN   no elevation data along the path
 */

import { localFrame } from './RouteModel.js';

export const LINK = { NONE: 0, MARGINAL: 1, DEGRADED: 2, GOOD: 3, UNKNOWN: 4 };

export const LINK_STYLE = {
    [LINK.GOOD]:     { label: 'good link',           short: 'GOOD',     color: '#60ff80', fill: 'rgba(96, 255, 128, 0.26)' },
    [LINK.DEGRADED]: { label: 'degraded link',       short: 'DEGRADED', color: '#ffaa00', fill: 'rgba(255, 170, 0, 0.30)' },
    [LINK.MARGINAL]: { label: 'below safety margin', short: 'MARGINAL', color: '#ff3b3b', fill: 'rgba(255, 60, 60, 0.32)' },
    [LINK.NONE]:     { label: 'no link',             short: 'NO LINK',  color: '#9aa4b0', fill: 'rgba(0, 0, 0, 0)' },
    [LINK.UNKNOWN]:  { label: 'no terrain data',     short: 'UNKNOWN',  color: '#4a5560', fill: 'rgba(0, 0, 0, 0)' },
};

const C_LIGHT = 299792458;
const K_EARTH = (4 / 3) * 6371000;   // effective earth radius (standard refraction)
const NU_CLEAR = -0.78;              // ITU-R P.526: J(ν) ≈ 0 dB below this — the 0.6·F1 rule of thumb
const D2R = Math.PI / 180;

// ── Link budget primitives ────────────────────────────────────────────────────

export function wavelength(fMHz) {
    return C_LIGHT / (Math.max(1, +fMHz || 1) * 1e6);
}

/** Free-space path loss in dB for a distance in metres. */
export function fspl(dM, fMHz) {
    const km = Math.max(1, dM) / 1000;
    return 20 * Math.log10(km) + 20 * Math.log10(Math.max(1, +fMHz || 1)) + 32.44;
}

/** First Fresnel zone radius (m) at d1 from one end and d2 from the other. */
export function fresnelRadius(d1, d2, fMHz) {
    if (d1 <= 0 || d2 <= 0) return 0;
    return Math.sqrt(wavelength(fMHz) * d1 * d2 / (d1 + d2));
}

/** Knife-edge diffraction loss J(ν) in dB, ITU-R P.526 approximation. */
export function knifeEdgeLoss(nu) {
    if (!(nu > NU_CLEAR)) return 0;
    const t = nu - 0.1;
    return 6.9 + 20 * Math.log10(Math.sqrt(t * t + 1) + t);
}

/** Everything in the budget except the path: EIRP plus receive gain minus losses. */
export function systemGain(radio) {
    return (+radio.txPower || 0) + (+radio.gainGround || 0) + (+radio.gainAir || 0) - (+radio.losses || 0);
}

/** Distance (m) at which a free-space path leaves exactly `marginDb` above the sensitivity. */
export function freeSpaceRange(radio, marginDb = 0) {
    const budget = systemGain(radio) - (+radio.minRssi || 0) - marginDb;
    const km = Math.pow(10, (budget - 20 * Math.log10(Math.max(1, +radio.freq || 1)) - 32.44) / 20);
    return km * 1000;
}

export function classifyLink(radio, margin, nu) {
    if (!Number.isFinite(margin)) return LINK.UNKNOWN;
    if (margin < 0) return LINK.NONE;
    if (margin < (+radio.minMargin || 0)) return LINK.MARGINAL;
    if (nu > NU_CLEAR) return LINK.DEGRADED;
    return LINK.GOOD;
}

/** The ground antenna: operator position, ground elevation and antenna height above it. */
export function groundStation(radio, pos, terrain) {
    const elev = terrain(pos.lat, pos.lng);
    const height = Math.max(0, +radio.groundHeight || 0);
    return { lat: pos.lat, lng: pos.lng, elev, height, antennaMsl: elev === null ? null : elev + height };
}

// ── One path ──────────────────────────────────────────────────────────────────

/**
 * Worst knife edge along a path: the sample whose height above the straight
 * line, normalised by the Fresnel radius there, is largest. `terr` holds the
 * ground elevation every `step` metres from the ground antenna (index 0) to
 * the aircraft (index n); samples strictly between the two are candidates.
 * The earth bulge is added here since it depends on the path length.
 */
function worstKnifeEdge(terr, n, step, D, h0, h1, lambda) {
    const slope = (h1 - h0) / D;
    const c = 2 / lambda;
    const bulgeK = 1 / (2 * K_EARTH);
    let best = -Infinity, bj = -1;
    for (let j = 1; j < n; j++) {
        const d1 = j * step, d2 = D - d1;
        if (d2 <= 0) break;
        const h = terr[j] + d1 * d2 * bulgeK - (h0 + slope * d1);
        const nu = h * Math.sqrt(c * (1 / d1 + 1 / d2));
        if (nu > best) { best = nu; bj = j; }
    }
    return { nu: best, j: bj };
}

/**
 * Link between the ground antenna and one point in the air.
 * @param {object} radio   route radio profile (see RADIO_FIELDS)
 * @param {object} gs      groundStation()
 * @param {object} pt      { lat, lng, altMsl }
 * @param {function} terrain (lat, lng) → m | null
 * @param {object} opts    { step: sample spacing m, maxSamples, profile: keep the terrain cut }
 */
export function evaluateLink(radio, gs, pt, terrain, opts = {}) {
    const step = opts.step || 30;
    const maxSamples = opts.maxSamples || 600;
    const f = localFrame(gs);
    const xy = f.toXY(pt);
    const D = Math.hypot(xy.x, xy.y);
    const h0 = gs.antennaMsl, h1 = +pt.altMsl;
    const n = Math.max(2, Math.min(maxSamples, Math.ceil(D / step)));
    const ds = D / n;
    const terr = new Float64Array(n + 1);
    const unknown = { dist: D, clazz: LINK.UNKNOWN, rssi: null, margin: null, nu: null, loss: 0, fspl: null, obstructed: false, fresnelClear: false, worst: null };
    if (h0 === null || h0 === undefined || !Number.isFinite(h1)) return unknown;
    for (let j = 0; j <= n; j++) {
        const t = j / n;
        const ll = f.toLL({ x: xy.x * t, y: xy.y * t });
        const g = terrain(ll.lat, ll.lng);
        if (g === null || g === undefined) return unknown;
        terr[j] = g;
    }
    const lambda = wavelength(radio.freq);
    const { nu, j } = D > 0 ? worstKnifeEdge(terr, n, ds, D, h0, h1, lambda) : { nu: -Infinity, j: -1 };
    const loss = knifeEdgeLoss(nu);
    const pathLoss = fspl(D, radio.freq);
    const rssi = systemGain(radio) - pathLoss - loss;
    const margin = rssi - (+radio.minRssi || 0);
    const clazz = classifyLink(radio, margin, nu);
    const res = { dist: D, rssi, margin, nu, loss, fspl: pathLoss, obstructed: nu > 0, fresnelClear: !(nu > NU_CLEAR), clazz, worst: null };
    if (j >= 0) {
        const d1 = j * ds, d2 = D - d1;
        const line = h0 + (h1 - h0) * (d1 / D);
        res.worst = { dist: d1, elev: terr[j], line, clearance: line - terr[j] - d1 * d2 / (2 * K_EARTH), r1: fresnelRadius(d1, d2, radio.freq) };
    }
    if (opts.profile) res.profile = { n, step: ds, terr, h0, h1, D, lambda, freq: +radio.freq };
    return res;
}

// ── Along the route ───────────────────────────────────────────────────────────

/**
 * Link quality along the calculated path. Samples the path every `stepM`
 * (plus every waypoint), evaluates each sample, and groups consecutive samples
 * of the same class into runs for drawing.
 * @returns {{ samples, runs, lengths, worst, total }}
 */
export function analyzeRoute(radio, gs, navPath, terrain, opts = {}) {
    if (!navPath || navPath.length < 2) return null;
    const total = navPath[navPath.length - 1].dist || 0;
    const stepM = opts.stepM || Math.max(20, Math.min(100, total / 600));
    const profileStep = opts.profileStep || 30;
    const samples = [];
    const eval1 = (lat, lng, altMsl, dist, segId) => {
        const ev = evaluateLink(radio, gs, { lat, lng, altMsl }, terrain, { step: profileStep });
        samples.push({ dist, lat, lng, altMsl, segId, clazz: ev.clazz, margin: ev.margin, rssi: ev.rssi, nu: ev.nu, loss: ev.loss, gsDist: ev.dist });
    };
    for (let i = 1; i < navPath.length; i++) {
        const a = navPath[i - 1], b = navPath[i];
        const len = b.dist - a.dist;
        const nSub = Math.max(1, Math.ceil(len / stepM));
        for (let s = 0; s < nSub; s++) {
            const t = s / nSub;
            eval1(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t, a.altMsl + (b.altMsl - a.altMsl) * t, a.dist + len * t, b.segId || a.segId);
        }
    }
    const last = navPath[navPath.length - 1];
    eval1(last.lat, last.lng, last.altMsl, last.dist, last.segId);

    // Runs of equal class; each run repeats the first point of the next so the halo is continuous
    const runs = [];
    let run = null;
    for (const s of samples) {
        if (!run || run.clazz !== s.clazz) {
            if (run) { run.points.push([s.lat, s.lng]); run.to = s.dist; }
            run = { clazz: s.clazz, from: s.dist, to: s.dist, points: [[s.lat, s.lng]] };
            runs.push(run);
        } else {
            run.points.push([s.lat, s.lng]);
            run.to = s.dist;
        }
    }
    const lengths = { [LINK.GOOD]: 0, [LINK.DEGRADED]: 0, [LINK.MARGINAL]: 0, [LINK.NONE]: 0, [LINK.UNKNOWN]: 0 };
    for (const r of runs) lengths[r.clazz] += r.to - r.from;

    // Worst point: lowest class first, then the smallest margin
    let worst = null;
    for (const s of samples) {
        if (s.clazz === LINK.UNKNOWN) continue;
        if (!worst || s.clazz < worst.clazz || (s.clazz === worst.clazz && s.margin < worst.margin)) worst = s;
    }
    const maxGsDist = samples.reduce((m, s) => Math.max(m, s.gsDist), 0);
    return { samples, runs, lengths, worst, total, maxGsDist };
}

/** Human summary of a route analysis, e.g. "link lost 1.2 km · marginal 400 m". */
export function summarizeLink(analysis) {
    if (!analysis) return '';
    const fmt = m => m >= 1000 ? `${(m / 1000).toFixed(1)} km` : `${Math.round(m)} m`;
    const L = analysis.lengths;
    const parts = [];
    if (L[LINK.NONE] > 0) parts.push(`no link ${fmt(L[LINK.NONE])}`);
    if (L[LINK.MARGINAL] > 0) parts.push(`marginal ${fmt(L[LINK.MARGINAL])}`);
    if (L[LINK.DEGRADED] > 0) parts.push(`degraded ${fmt(L[LINK.DEGRADED])}`);
    if (L[LINK.UNKNOWN] > 0) parts.push(`no terrain ${fmt(L[LINK.UNKNOWN])}`);
    return parts.length ? parts.join(' · ') : 'good link along the whole route';
}

// ── Coverage raster ───────────────────────────────────────────────────────────

/**
 * Polar coverage around the ground antenna: `rays` azimuths, `count` cells
 * along each up to `range` metres, one class per cell for an aircraft at the
 * altitude `altAt(terrainElev)` returns over that cell.
 *
 * The knife-edge search is O(cells²) per ray; the loop yields to the event
 * loop every `budgetMs` so the map keeps responding while it runs, and a
 * `token.cancelled` set by the caller abandons the run.
 */
export async function computeCoverage(radio, gs, altAt, terrain, opts = {}) {
    const range = Math.max(200, +opts.range || 15000);
    const rays = opts.rays || 720;
    const count = Math.max(8, Math.min(opts.maxCount || 400, Math.ceil(range / 30)));
    const step = range / count;
    const budgetMs = opts.budgetMs ?? 8;
    const token = opts.token || {};
    const classes = new Uint8Array(rays * count).fill(LINK.UNKNOWN);
    if (gs.antennaMsl === null || gs.antennaMsl === undefined) return { rays, count, step, range, classes, gs: { lat: gs.lat, lng: gs.lng } };

    const f = localFrame(gs);
    const h0 = gs.antennaMsl;
    const lambda = wavelength(radio.freq);
    const c = 2 / lambda;
    const bulgeK = 1 / (2 * K_EARTH);
    const gain = systemGain(radio);
    const minRssi = +radio.minRssi || 0;
    const minMargin = +radio.minMargin || 0;
    const terr = new Float64Array(count + 1);
    const invD = new Float64Array(count + 1);
    for (let j = 1; j <= count; j++) invD[j] = 1 / (j * step);
    const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
    let t0 = now();

    for (let r = 0; r < rays; r++) {
        const az = (r / rays) * 2 * Math.PI;
        const sx = Math.sin(az), cy = Math.cos(az);
        terr[0] = gs.elev;
        let known = count;                       // first unknown sample ends the ray
        for (let j = 1; j <= count; j++) {
            const d = j * step;
            const ll = f.toLL({ x: d * sx, y: d * cy });
            const g = terrain(ll.lat, ll.lng);
            if (g === null || g === undefined) { known = j - 1; break; }
            terr[j] = g;
        }
        const base = r * count;
        for (let k = 1; k <= known; k++) {
            const D = k * step;
            const h1 = altAt(terr[k]);
            if (h1 === null || !Number.isFinite(h1)) continue;
            let clazz;
            if (h1 < terr[k]) clazz = LINK.NONE;      // planned altitude is under the ground here
            else {
                const slope = (h1 - h0) / D;
                let best = -Infinity;
                for (let j = 1; j < k; j++) {
                    const d1 = j * step, d2 = D - d1;
                    const h = terr[j] + d1 * d2 * bulgeK - (h0 + slope * d1);
                    const nu = h * Math.sqrt(c * (invD[j] + 1 / d2));
                    if (nu > best) best = nu;
                }
                const margin = gain - fspl(D, radio.freq) - knifeEdgeLoss(best) - minRssi;
                clazz = margin < 0 ? LINK.NONE : margin < minMargin ? LINK.MARGINAL : best > NU_CLEAR ? LINK.DEGRADED : LINK.GOOD;
            }
            classes[base + k - 1] = clazz;
        }
        if (now() - t0 > budgetMs) {
            await new Promise(res => setTimeout(res, 0));
            if (token.cancelled) return null;
            t0 = now();
        }
    }
    return { rays, count, step, range, classes, gs: { lat: gs.lat, lng: gs.lng } };
}

/** Class of the coverage cell under a ground offset (metres east / north of the antenna). */
export function coverageClassAt(cov, xEast, yNorth) {
    const d = Math.hypot(xEast, yNorth);
    if (d > cov.range) return LINK.UNKNOWN;
    let az = Math.atan2(xEast, yNorth);
    if (az < 0) az += 2 * Math.PI;
    const r = Math.round(az / (2 * Math.PI) * cov.rays) % cov.rays;
    const k = Math.min(cov.count - 1, Math.floor(d / cov.step));
    return cov.classes[r * cov.count + k];
}

export const _test = { worstKnifeEdge, NU_CLEAR, K_EARTH, D2R };
