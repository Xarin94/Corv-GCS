/**
 * GeoZoneLayer.js - UAS geographical zones on a 2D map (mini-map, FLIGHT PLAN)
 *
 * The zones of safety/GeoZones.js in the palette of safety/GeoZoneStyle.js:
 * no-fly red with black bands, conditions violet with bands, a ceiling
 * coloured by the height that may be flown (its floor above the ground; for
 * a floor given above sea level, at the ground under the zone's centre).
 * Ceilings are drawn highest first, so a lower one covers it, and no-fly zones
 * last, on top.
 *
 * SVG paths in a pane of their own, under the track and the markers: the
 * bands are an SVG pattern (fill="url(#...)"), which Leaflet's canvas
 * renderer cannot draw. Only the zones within three times the view are
 * built, again when the map leaves them or the zones change. On the FLIGHT
 * PLAN map (interactive) a zone shows its name, limits and source on hover,
 * and a legend sits in the corner while the zones are on.
 */

import { STATE } from '../core/state.js';
import { getGeoZoneData, onGeoZonesChange } from '../safety/GeoZones.js';
import { PROHIBITED_RGB, CONDITIONAL_RGB, CEILING_MAX_M, CEILING_STOPS, ceilingRgb, rgbCss } from '../safety/GeoZoneStyle.js';
import { getTerrainElevationFromHGT } from '../terrain/TerrainManager.js';

const MAX_POLYGONS = 6000;
const SOURCE_NAMES = { faa: 'FAA', dipul: 'DFS DIPUL', ign: 'IGN / DGAC', enaire: 'ENAIRE', bazl: 'FOCA / BAZL' };

export function geoZoneSourceName(src) {
    return SOURCE_NAMES[src] || (src.startsWith('import-') ? `imported ${src.slice(7)}` : src);
}

let patternsAdded = false;
function ensurePatterns() {
    if (patternsAdded) return;
    patternsAdded = true;
    const hatch = (id, rgb, dark) => `<pattern id="${id}" patternUnits="userSpaceOnUse" width="10" height="10" patternTransform="rotate(45)">`
        + `<rect width="10" height="10" fill="${rgbCss(rgb)}"/><rect width="${dark}" height="10" fill="#000"/></pattern>`;
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    svg.setAttribute('width', '0');
    svg.setAttribute('height', '0');
    svg.style.position = 'absolute';
    svg.innerHTML = `<defs>${hatch('gz-hatch-noflyzone', PROHIBITED_RGB, 5)}${hatch('gz-hatch-condzone', CONDITIONAL_RGB, 3.5)}</defs>`;
    document.body.appendChild(svg);
}

const fmtLimit = (v, ref) => v === null ? 'unlimited' : `${Math.round(v)} m ${ref}`;

/** Floor above the ground (m), for the colour; null when not drawn. */
function floorAgl(z) {
    if (z.floorRef === 'AGL') return z.floor;
    const lat = (z.bbox[1] + z.bbox[3]) / 2, lon = (z.bbox[0] + z.bbox[2]) / 2;
    const ground = getTerrainElevationFromHGT(lat, lon) ?? STATE.terrainHeight ?? STATE.homeAlt;
    return ground === null || ground === undefined ? null : z.floor - ground;
}

/** Leaflet style and draw order (higher first) of a zone; null: not drawn. */
function zoneStyle(z) {
    const floor = floorAgl(z);
    if (floor === null) return null;
    const faaGrid = z.id.startsWith('faa:uasfm');
    if (floor <= 0.5) {
        const p = z.kind === 'prohibited';
        return {
            order: p ? -2 : -1, floor,
            style: {
                fillColor: p ? 'url(#gz-hatch-noflyzone)' : 'url(#gz-hatch-condzone)', fillOpacity: p ? 0.45 : 0.35,
                color: rgbCss(p ? PROHIBITED_RGB : CONDITIONAL_RGB), weight: faaGrid ? 0.5 : 1.2, opacity: 0.9,
                dashArray: p ? null : '5 4',
            },
        };
    }
    if (floor >= CEILING_MAX_M) return null;
    const c = rgbCss(ceilingRgb(floor));
    return {
        order: floor, floor,
        style: { fillColor: c, fillOpacity: 0.4, color: c, weight: faaGrid ? 0.5 : 1.2, opacity: 0.85, dashArray: z.kind === 'prohibited' ? null : '5 4' },
    };
}

function tooltipHtml(z, floor) {
    const what = floor <= 0.5
        ? (z.kind === 'prohibited' ? 'No-fly or authorisation required from the surface' : 'Conditions apply from the surface')
        : `Max <b>${Math.round(floor)} m AGL</b>${z.kind === 'prohibited' ? '' : ' (conditions above)'}`;
    const esc = s => String(s).replace(/[&<>"]/g, ch => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[ch]));
    return `<b>${esc(z.name)}</b><br>${what}<br>`
        + `<span class="gz-tip-dim">Zone ${fmtLimit(z.floor, z.floorRef)} – ${fmtLimit(z.top, z.topRef)} · ${esc(geoZoneSourceName(z.src))}</span>`
        + (z.info ? `<br><span class="gz-tip-dim">${esc(z.info.length > 160 ? `${z.info.slice(0, 159)}…` : z.info)}</span>` : '');
}

function legendControl() {
    const ctl = L.control({ position: 'bottomleft' });
    ctl.onAdd = () => {
        const el = L.DomUtil.create('div', 'gz-legend');
        const swatch = (bg, label) => `<div class="gz-legend-row"><span class="gz-swatch" style="background:${bg}"></span>${label}</div>`;
        const band = (rgb) => `repeating-linear-gradient(45deg, ${rgbCss(rgb)} 0 4px, #000 4px 7px)`;
        el.innerHTML = '<div class="gz-legend-title">UAS ZONES</div>'
            + swatch(band(PROHIBITED_RGB), 'No-fly / authorisation')
            + swatch(band(CONDITIONAL_RGB), 'Conditions apply')
            + `<div class="gz-legend-row"><span class="gz-ramp">${CEILING_STOPS.slice(1).map(([m, rgb]) =>
                `<span style="background:${rgbCss(rgb)}">${m}</span>`).join('')}</span>max m AGL</div>`;
        L.DomEvent.disableClickPropagation(el);
        return el;
    };
    return ctl;
}

/**
 * @param map Leaflet map
 * @param {{interactive?: boolean}} opts interactive: tooltips and a legend (FLIGHT PLAN)
 */
export function createGeoZoneLayer(map, { interactive = false } = {}) {
    ensurePatterns();
    const pane = map.createPane('geoZones');
    pane.style.zIndex = 360;            // over the tiles, under the cell sites, the track and the markers
    if (!interactive) pane.style.pointerEvents = 'none';
    const renderer = L.svg({ pane: 'geoZones', padding: 0.5 });
    const group = L.layerGroup().addTo(map);
    const legend = interactive ? legendControl() : null;
    let builtVersion = -1, builtBounds = null, legendShown = false;

    function rebuild(force) {
        const d = getGeoZoneData();
        const show = d.enabled && d.zones.length > 0;
        if (legend && show !== legendShown) {
            legendShown = show;
            if (show) legend.addTo(map); else legend.remove();
        }
        if (!show) {
            if (builtBounds) group.clearLayers();
            builtBounds = null;
            builtVersion = d.version;
            return;
        }
        let view;
        try { view = map.getBounds(); } catch (e) { return; }   // map not laid out yet
        if (!force && builtVersion === d.version && builtBounds && builtBounds.contains(view)) return;
        builtBounds = view.pad(1);
        builtVersion = d.version;
        const box = [builtBounds.getWest(), builtBounds.getSouth(), builtBounds.getEast(), builtBounds.getNorth()];
        const list = [];
        for (const z of d.zones) {
            const b = z.bbox;
            if (b[0] > box[2] || b[2] < box[0] || b[1] > box[3] || b[3] < box[1]) continue;
            const s = zoneStyle(z);
            if (s) list.push({ z, s });
        }
        list.sort((a, b) => b.s.order - a.s.order);
        group.clearLayers();
        let n = 0;
        for (const { z, s } of list) {
            for (const rings of z.polys) {
                if (n++ >= MAX_POLYGONS) break;
                const latlngs = rings.map((ring) => {
                    const out = [];
                    for (let i = 0; i < ring.length; i += 2) out.push([ring[i + 1], ring[i]]);
                    return out;
                });
                const poly = L.polygon(latlngs, { ...s.style, renderer, interactive, bubblingMouseEvents: true });
                if (interactive) poly.bindTooltip(() => tooltipHtml(z, s.floor), { sticky: true, className: 'gz-tooltip' });
                group.addLayer(poly);
            }
        }
    }

    map.on('moveend', () => rebuild(false));
    onGeoZonesChange(() => rebuild(true));
    rebuild(true);
    return { refresh: () => rebuild(true) };
}
