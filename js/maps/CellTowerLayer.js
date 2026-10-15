/**
 * CellTowerLayer.js - LTE cell sites and the serving cell on the mini-map
 *
 * Live only while CORV_CELL_INFO arrives from an LTEtelem cellular module
 * (STATE.cell, last message under 10 s old): the serving site highlighted, a
 * dashed line from the vehicle to it — green, orange or red by linkQuality()
 * — so the operator sees which mast carries the link and how well, and the
 * other sites within 10 km of the vehicle. The cell
 * figures go in the position panel (UIController), not on the map.
 *
 * Sites come from the cell database (cell-db.js, through window.cellDb:
 * the cells shipped with the installer, files the user added, the OpenCelliD
 * PMTiles copy) within 100 km — wide, so the serving site is found even when
 * far — and are fetched again when the vehicle has moved 10 km from the last
 * query centre, when the serving network changes, or when the settings
 * change. The ones drawn are picked again every 250 m and at every cell change.
 * A serving cell missing from the sites is asked again with its ECI (found
 * under another operator's code when the cell is shared), then looked up
 * online (beaconDB / OpenCelliD API) and added as a site of its own.
 * Settings persist in localStorage under 'lte-cell-map'.
 */

import { STATE } from '../core/state.js';

const STORAGE_KEY = 'lte-cell-map';
const RADIUS_KM = 100;           // sites fetched around the vehicle
const REQUERY_KM = 10;
const SHOW_KM = 10;              // sites drawn besides the serving one
const REFILTER_KM = 0.25;
const STALE_MS = 10000;          // no CORV_CELL_INFO for this long: overlay off
// Link colour by quality (linkQuality): good / fair / poor
const QUALITY_COLORS = { good: '#3dff6e', fair: '#ffaa00', poor: '#ff3030' };
const STALE_LINK_MS = 5000;      // no CORV_CELL_INFO for this long: poor

/**
 * Quality of the cellular link from the serving cell: good with RSRP from
 * -100 dBm and an average SINR from 5 dB, poor under -110 dBm or 0 dB (or
 * no report for 5 s), fair in between.
 * @returns {{level: 'good'|'fair'|'poor', color: string}}
 */
export function linkQuality() {
    const c = STATE.cell;
    let level = 'fair';
    if (!c || !c.eci || Date.now() - c.receivedAt > STALE_LINK_MS
        || (c.rsrp !== null && c.rsrp < -110) || (c.sinrAvg !== null && c.sinrAvg < 0)) level = 'poor';
    else if (c.rsrp !== null && c.rsrp >= -100 && c.sinrAvg !== null && c.sinrAvg >= 5) level = 'good';
    return { level, color: QUALITY_COLORS[level] };
}

// Where a cell position comes from (cell-db.js src)
export const SOURCE_NAMES = {
    'ocid': 'OpenCelliD', 'ocid-2024': 'OpenCelliD 2024', 'mls': 'Mozilla MLS 2024',
    'beacondb': 'beaconDB (online)', 'ocid-api': 'OpenCelliD API (online)', 'bundle': 'built-in',
};

// Italian networks by MNC (MCC 222); elsewhere the PLMN code is shown
const OPERATORS_222 = { 1: 'TIM', 10: 'Vodafone', 50: 'Iliad', 88: 'WindTre', 99: 'WindTre (3)', 43: 'TIM', 48: 'TIM' };
export function operatorName(mcc, mnc) {
    const plmn = `${mcc}-${String(mnc).padStart(2, '0')}`;
    return mcc === 222 && OPERATORS_222[mnc] ? `${OPERATORS_222[mnc]} (${plmn})` : plmn;
}
/** Operator name alone, or the PLMN code when unknown. */
export function operatorShort(mcc, mnc) {
    return (mcc === 222 && OPERATORS_222[mnc]) || `${mcc}-${String(mnc).padStart(2, '0')}`;
}

// show: sites and link on the maps; allOperators: every network, not just the
// serving one; table: cell rows of the position panel (LTE OVERLAY)
const DEFAULT_SETTINGS = { show: true, allOperators: false, table: true };
let settings = loadSettings();
function loadSettings() {
    try { return { ...DEFAULT_SETTINGS, ...JSON.parse(localStorage.getItem(STORAGE_KEY)) }; }
    catch (e) { return { ...DEFAULT_SETTINGS }; }
}
const listeners = new Set();
export function getCellMapSettings() { return { ...settings }; }
export function setCellMapSettings(patch) {
    settings = { ...settings, ...patch };
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(settings)); } catch (e) { /* ignore */ }
    for (const cb of listeners) cb();
}

/**
 * Serving site as last seen by the layer, for the position panel and the
 * settings page: servingFound is null until the sites around the vehicle are
 * known, then whether the serving eNB is among them; distanceKm to its mast.
 */
export const cellMapInfo = { servingSite: null, servingFound: null, lookingUp: false, distanceKm: null, sites: 0, date: null, missing: 0, error: null };

function distanceKm(a, b) {
    const r = Math.PI / 180;
    const h = Math.sin((b.lat - a.lat) * r / 2) ** 2
        + Math.cos(a.lat * r) * Math.cos(b.lat * r) * Math.sin((b.lng - a.lng) * r / 2) ** 2;
    return 12742 * Math.asin(Math.sqrt(h));
}

/**
 * What the map overlay shows, for the 3D view (Cell3D.js): live, the sites
 * drawn (serving one included; a new array, and a new version, whenever they
 * are picked again) and the serving site.
 */
const overlay = { live: false, sites: [], serving: null, version: 0 };
export function getCellOverlay() { return overlay; }

export function cellIsLive() {
    return !!STATE.cell && Date.now() - STATE.cell.receivedAt < STALE_MS;
}

export function createCellTowerLayer(map) {
    const pane = map.createPane('cellSites');
    pane.style.zIndex = 380;            // over the tiles, under the track and the vehicle
    const renderer = L.canvas({ pane: 'cellSites', padding: 0.3 });
    const sitesLayer = L.layerGroup();
    const line = L.polyline([], { color: QUALITY_COLORS.good, weight: 2, opacity: 0.95, dashArray: '8 6', interactive: false });
    const servingMarker = L.circleMarker([0, 0], {
        pane: 'cellSites', renderer, radius: 7, color: QUALITY_COLORS.good, weight: 2, fillColor: QUALITY_COLORS.good, fillOpacity: 0.35,
    });
    let lineColor = QUALITY_COLORS.good;

    let shown = false, relative = false;
    let sites = [];                 // last query result
    let filterCentre = null, filterServing = null, filterSites = null;
    let queryKey = null, queryCentre = null, querying = false;
    let servingSite = null, servingEci = null, tooltipSite = null;
    let requeriedEci = null, lookupEci = null, lookingUp = false;

    function siteTooltip(s, serving) {
        const cells = s.cells.map((c) => c.eci & 255).sort((a, b) => a - b).join(', ');
        const src = [...new Set(s.cells.map((c) => SOURCE_NAMES[c.src] || c.src))].join(', ');
        return `${serving ? '<b>SERVING</b> · ' : ''}${operatorName(s.mcc, s.mnc)}<br>eNB ${s.enb} · sectors ${cells}<br>${src}`;
    }

    /** The serving site and the sites within SHOW_KM of the vehicle. */
    function pickSites(vehicle) {
        filterCentre = vehicle;
        filterServing = servingSite;
        filterSites = sites;
        const shown = vehicle
            ? sites.filter((s) => s === servingSite || distanceKm(vehicle, { lat: s.lat, lng: s.lon }) <= SHOW_KM)
            : (servingSite ? [servingSite] : []);
        overlay.sites = shown;
        overlay.version++;
        drawSites(shown);
    }

    function drawSites(list) {
        sitesLayer.clearLayers();
        const own = STATE.cell ? `${STATE.cell.mcc}-${STATE.cell.mnc}` : '';
        for (const s of list) {
            if (s === servingSite) continue;      // the green ring
            const ownNet = `${s.mcc}-${s.mnc}` === own;
            L.circleMarker([s.lat, s.lon], {
                pane: 'cellSites', renderer, radius: ownNet ? 3 : 2.5, weight: 1,
                color: ownNet ? '#e8e8e8' : '#7a8ca0', fillColor: ownNet ? '#ffffff' : '#7a8ca0',
                fillOpacity: ownNet ? 0.8 : 0.5, opacity: 0.9,
            }).bindTooltip(() => siteTooltip(s, false), { direction: 'top', className: 'traffic-tooltip' }).addTo(sitesLayer);
        }
    }

    function findServing() {
        const c = STATE.cell;
        servingEci = c ? c.eci : null;
        servingSite = null;
        if (!c || !c.eci) return;
        // the eNB is found even when this sector is missing from the database
        servingSite = sites.find((s) => s.mcc === c.mcc && s.mnc === c.mnc && s.enb === c.enb)
            // a shared cell, in the database under the partner's network code
            || sites.find((s) => s.mcc === c.mcc && s.cells.some((x) => x.eci === c.eci))
            || null;
        if (servingSite || querying || !queryCentre) return;
        if (requeriedEci !== c.eci) {
            requeriedEci = c.eci;
            queryCentre = null;           // the next update asks again with this ECI
        } else lookupServing(c);
    }

    /** The serving cell from beaconDB / the OpenCelliD API, added to the sites. */
    function lookupServing(c) {
        if (lookupEci === c.eci) return;
        lookupEci = c.eci;
        lookingUp = true;
        window.cellDb.lookup({ mcc: c.mcc, mnc: c.mnc, tac: c.tac, eci: c.eci }).then((hit) => {
            if (!hit || !STATE.cell || STATE.cell.eci !== c.eci) return;
            const cell = { eci: c.eci, tac: c.tac, lat: hit.lat, lon: hit.lon, range: hit.range, samples: hit.samples, src: hit.src };
            sites = [...sites, { mcc: c.mcc, mnc: c.mnc, enb: c.enb, lat: hit.lat, lon: hit.lon, cells: [cell] }];
            findServing();
        }).catch((e) => {
            lookupEci = null;              // offline: try again at the next cell change
            console.warn('[cell] lookup:', e.message || e);
        }).finally(() => { lookingUp = false; });
    }

    async function query(centre) {
        const c = STATE.cell;
        querying = true;
        queryCentre = centre;
        queryKey = `${c.mcc}-${c.mnc}-${settings.allOperators}`;
        try {
            const r = await window.cellDb.query({
                lat: centre.lat, lon: centre.lng, radiusKm: RADIUS_KM,
                mcc: settings.allOperators ? null : c.mcc, mnc: settings.allOperators ? null : c.mnc, eci: c.eci,
            });
            sites = r.sites;
            cellMapInfo.sites = sites.length;
            cellMapInfo.date = r.date;
            cellMapInfo.missing = r.missing;
            cellMapInfo.error = null;
            querying = false;
            findServing();
        } catch (e) {
            cellMapInfo.error = String(e.message || e);
            queryCentre = null;   // try again on the next update
        } finally {
            querying = false;
        }
    }

    function show(on) {
        if (on === shown) return;
        shown = on;
        overlay.live = on;
        if (!on) overlay.serving = null;
        if (on) {
            sitesLayer.addTo(map);
            line.addTo(map);
        } else {
            map.removeLayer(sitesLayer);
            map.removeLayer(line);
            map.removeLayer(servingMarker);
            servingSite = null;
            filterSites = null;           // drawn again when the overlay comes back
            cellMapInfo.servingSite = cellMapInfo.servingFound = cellMapInfo.distanceKm = null;
        }
    }

    return {
        /** Called by MapEngine.updateMap (10 Hz) with the vehicle position. */
        update(vehicle) {
            const valid = vehicle && (vehicle.lat !== 0 || vehicle.lng !== 0);
            const live = cellIsLive() && settings.show && !relative && !!window.cellDb;
            show(live);
            if (!live) return;

            const c = STATE.cell;
            const key = `${c.mcc}-${c.mnc}-${settings.allOperators}`;
            if (valid && c.mcc && !querying
                && (key !== queryKey || !queryCentre || distanceKm(queryCentre, vehicle) > REQUERY_KM)) {
                query({ lat: vehicle.lat, lng: vehicle.lng });
            }
            if (c.eci !== servingEci) findServing();
            if (sites !== filterSites || servingSite !== filterServing || (valid && !filterCentre)
                || (valid && filterCentre && distanceKm(filterCentre, vehicle) > REFILTER_KM)) {
                pickSites(valid ? { lat: vehicle.lat, lng: vehicle.lng } : null);
            }

            const { color } = linkQuality();
            if (color !== lineColor) {
                lineColor = color;
                line.setStyle({ color });
                servingMarker.setStyle({ color, fillColor: color });
            }
            if (servingSite && valid) {
                const mast = [servingSite.lat, servingSite.lon];
                line.setLatLngs([[vehicle.lat, vehicle.lng], mast]);
                servingMarker.setLatLng(mast);
                if (!map.hasLayer(servingMarker)) servingMarker.addTo(map);
                if (tooltipSite !== servingSite) {
                    tooltipSite = servingSite;
                    servingMarker.unbindTooltip().bindTooltip(siteTooltip(servingSite, true), { direction: 'top', className: 'traffic-tooltip' });
                }
            } else {
                line.setLatLngs([]);
                if (map.hasLayer(servingMarker)) map.removeLayer(servingMarker);
            }
            cellMapInfo.servingSite = servingSite;
            overlay.serving = servingSite;
            cellMapInfo.lookingUp = lookingUp;
            cellMapInfo.servingFound = querying || !queryCentre || lookingUp ? null : !!servingSite;
            cellMapInfo.distanceKm = servingSite && valid
                ? distanceKm(vehicle, { lat: servingSite.lat, lng: servingSite.lon }) : null;
        },

        /** Relative navigation frame: real-world overlays would lie, hide them. */
        setRelative(on) {
            relative = !!on;
        },

        /** Settings changed: query again with the new filter. */
        refresh() {
            queryKey = null;
        },
    };
}

export function onCellMapSettings(cb) { listeners.add(cb); }
