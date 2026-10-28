/**
 * GeoZoneRules.js - Where a UAS may fly: the general height limit of the
 * region and the zones over a point
 *
 * Pure functions, no DOM and no STATE (scripts/test-geozones.js runs them in
 * Node). Zones are the records of geozones.js: a restricted volume — floor and
 * top, each AGL or MSL — over polygons of flat [lon, lat, ...] rings, kind
 * 'prohibited' or 'conditional'.
 *
 * General limit: 120 m above the ground in the EASA states (Regulation (EU)
 * 2019/947, open category; the UK, Switzerland and Norway the same), 400 ft in
 * the United States (14 CFR 107.51). The regions are coarse outlines, good to
 * a few km at the borders with the rest of the world, not between member
 * states.
 */

export const GENERAL_LIMITS = {
    EU: { metres: 120, label: '120 m', rule: 'EU open category (Reg. 2019/947): 120 m above the ground' },
    US: { metres: 400 * 0.3048, label: '400 ft', rule: 'FAA Part 107 / recreational: 400 ft above the ground' },
};

// EASA member states with the UK, Switzerland, Norway and Iceland (and the
// Balkans, on their way to the same rules), [lon, lat]
const EUROPE = [
    [-32, 32], [-12, 32], [-9.5, 35.5], [-6.2, 35.85], [-5.3, 35.95], [-2, 36.25], [8, 38], [10.5, 37.7],
    [11.6, 36.95], [11.8, 36.6], [12.2, 35.3], [15.5, 35.3], [23, 34.6], [26.5, 34.8], [32, 34.4], [34.7, 34.4],
    [34.7, 35.75], [32.5, 35.75], [28.6, 36.2], [28.45, 36.55], [27.5, 36.72], [27.4, 36.95], [27, 37.75], [26.25, 38.3], [26.62, 39.05],
    [25.95, 39.7], [25.9, 40.3], [26.05, 40.75], [26.35, 41.25], [26.6, 41.7], [28, 42], [29.8, 45.2],
    [28.2, 45.45], [26.6, 48.25], [22.9, 48], [22.5, 49], [24.1, 50.6], [23.6, 51.5], [23.6, 53.9], [25, 54.2],
    [26.6, 55.6], [28.2, 56.2], [27.7, 57.4], [28.2, 59.45], [27.7, 60.5], [29.5, 61.5], [31.6, 62.9],
    [30, 65.5], [29, 67], [28.6, 68.5], [29.3, 69.7], [31.5, 69.9], [32, 72], [-12, 72], [-12, 67.5],
    [-25, 67.5], [-25, 63], [-32, 40],
].flat();
const CANARIES = [-18.5, 27.5, -13.3, 29.5];
const US_BOXES = [[-125, 24.4, -66.8, 49.4], [-170, 51, -129, 71.5], [-161, 18.8, -154.7, 22.3], [-67.4, 17.8, -65.2, 18.6]];

/** Ray casting on a flat [lon, lat, ...] ring. */
export function pointInRing(ring, lon, lat) {
    let inside = false;
    for (let i = 0, j = ring.length - 2; i < ring.length; j = i, i += 2) {
        const xi = ring[i], yi = ring[i + 1], xj = ring[j], yj = ring[j + 1];
        if ((yi > lat) !== (yj > lat) && lon < (xj - xi) * (lat - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}

const inBox = (b, lon, lat) => lon >= b[0] && lon <= b[2] && lat >= b[1] && lat <= b[3];

/** 'EU', 'US' or null (no general limit known here). */
export function regionAt(lat, lon) {
    if (US_BOXES.some(b => inBox(b, lon, lat))) return 'US';
    if (inBox(CANARIES, lon, lat) || pointInRing(EUROPE, lon, lat)) return 'EU';
    return null;
}

/** True when lat/lon lies in one of the zone's polygons (and in none of its holes). */
export function pointInZone(zone, lat, lon) {
    if (!inBox(zone.bbox, lon, lat)) return false;
    for (const rings of zone.polys) {
        if (!pointInRing(rings[0], lon, lat)) continue;
        let hole = false;
        for (let k = 1; k < rings.length && !hole; k++) hole = pointInRing(rings[k], lon, lat);
        if (!hole) return true;
    }
    return false;
}

/** A limit of the zone in metres above the ground at a point of height terrain (null: unknown or no top). */
export function heightAgl(value, ref, terrain) {
    if (value === null || value === undefined) return null;
    if (ref === 'AGL') return value;
    return terrain === null || terrain === undefined ? null : value - terrain;
}

const HYSTERESIS_M = 1;   // a ceiling counts as broken this far above it

const shortName = (z) => (z.name || '').length > 18 ? `${z.name.slice(0, 17)}…` : (z.name || '');
const fmtM = (m) => `${Math.round(m)} m`;

/**
 * What the zones and the general limit say about a vehicle at lat/lon, agl
 * metres above ground of height terrain (MSL; null when unknown).
 * airborne: false on the ground, where only ceilings are reported.
 *
 * alerts: up to one per id, warnings first —
 *   noFly      warn  in a zone restricted from the surface, airborne
 *   ceiling    warn  above a zone's floor (the height that may be flown)
 *   altLimit   warn  above the region's general limit
 *   cond       caut  in a zone where conditions apply
 *   maxHere    caut  under a zone's ceiling: the height that may be flown here
 * overHeight: a ceiling or the general limit is broken (TERR ALT flashes)
 */
export function evaluateGeoZones({ zones, lat, lon, agl, terrain, airborne = true }) {
    const region = regionAt(lat, lon);
    const limit = region ? GENERAL_LIMITS[region] : null;
    const here = zones.filter(z => pointInZone(z, lat, lon));
    let noFly = null, ceiling = null, cond = null, maxHere = null;
    for (const z of here) {
        const floor = heightAgl(z.floor, z.floorRef, terrain);
        const top = heightAgl(z.top, z.topRef, terrain);
        if (floor === null) continue;
        if (agl !== null && top !== null && agl > top) continue;     // over the zone
        if (floor <= 0.5) {
            if (!airborne) continue;
            if (z.kind === 'prohibited') { if (!noFly) noFly = { zone: z }; }
            else if (!cond) cond = { zone: z };
        } else if (agl !== null && agl > floor + HYSTERESIS_M) {
            // The broken ceiling shown: a prohibited zone before a conditional one, then the lowest
            const p = z.kind === 'prohibited', cp = ceiling?.zone.kind === 'prohibited';
            if (!ceiling || (p && !cp) || (p === cp && floor < ceiling.floor)) ceiling = { zone: z, floor };
        } else if (!maxHere || floor < maxHere.floor) {
            maxHere = { zone: z, floor };
        }
    }
    const overLimit = !!(limit && agl !== null && agl > limit.metres + HYSTERESIS_M);
    const alerts = [];
    if (noFly) alerts.push({ id: 'noFly', level: 'warn', icon: 'zone', label: 'NO-FLY', value: shortName(noFly.zone), zone: noFly.zone });
    if (ceiling) {
        alerts.push({
            id: 'ceiling', level: ceiling.zone.kind === 'prohibited' ? 'warn' : 'caut', icon: 'ceiling',
            label: `MAX ${fmtM(ceiling.floor)}`, value: shortName(ceiling.zone), zone: ceiling.zone, metres: ceiling.floor,
        });
    }
    if (overLimit) alerts.push({ id: 'altLimit', level: 'warn', icon: 'ceiling', label: `MAX ${limit.label}`, value: 'ALT LIMIT', rule: limit.rule });
    if (cond) alerts.push({ id: 'cond', level: 'caut', icon: 'zone', label: 'ZONE', value: shortName(cond.zone), zone: cond.zone });
    if (maxHere && !ceiling && (!limit || maxHere.floor < limit.metres)) {
        alerts.push({ id: 'maxHere', level: 'caut', icon: 'ceiling', label: `MAX ${fmtM(maxHere.floor)}`, value: shortName(maxHere.zone), zone: maxHere.zone, metres: maxHere.floor });
    }
    return {
        region, limit, zonesHere: here, alerts,
        overHeight: overLimit || !!(ceiling && ceiling.zone.kind === 'prohibited'),
    };
}

/**
 * The alarm on arming among the zones over the vehicle (zonesHere of
 * evaluateGeoZones), null when none restricts it: a no-fly zone from the
 * surface (warn), else one where conditions apply (caut), else the lowest
 * ceiling under the general limit (caut).
 */
export function armingAlert(zonesHere, terrain, limit = null) {
    let noFly = null, cond = null, ceiling = null;
    for (const z of zonesHere) {
        const floor = heightAgl(z.floor, z.floorRef, terrain);
        if (floor === null) continue;
        if (floor <= 0.5) {
            if (z.kind === 'prohibited') noFly = noFly || z;
            else cond = cond || z;
        } else if ((!limit || floor < limit.metres) && (!ceiling || floor < ceiling.floor)) {
            ceiling = { zone: z, floor };
        }
    }
    if (noFly) return { level: 'warn', title: 'ARMED IN A NO-FLY ZONE', detail: `${noFly.name} — no flight without authorisation`, zone: noFly };
    if (cond) return { level: 'caut', title: 'ARMED IN A RESTRICTED ZONE', detail: `${cond.name} — conditions apply`, zone: cond };
    if (ceiling) {
        return { level: 'caut', title: `ARMED UNDER A ${fmtM(ceiling.floor)} CEILING`,
            detail: `${ceiling.zone.name} — max ${fmtM(ceiling.floor)} above the ground`, zone: ceiling.zone };
    }
    return null;
}
