/**
 * MapEngine.js - Leaflet Mini-Map Integration
 * Handles the small overview map in the corner
 */

import { STATE } from '../core/state.js';
import { RAD } from '../core/constants.js';
import { cachedTileLayer } from './CachedTileLayer.js';

let map = null;
let marker = null;
let satelliteLayer = null;
let pathLine = null;
let livePathPoints = [];
let lastLiveLatLng = null;
// When true, updateMap() leaves the trail polyline untouched — used during log
// replay so a pre-drawn full-flight trajectory stays visible and is not
// clobbered by the live-path append logic.
let trailFrozen = false;

const MAX_MAP_PATH_POINTS = 3000;

// Mission overlay on mini-map
let missionMarkers = [];
let missionPolyline = null;
let homeMarker = null;
let targetMarkerInner = null;
let targetMarkerOuter = null;

// ADS-B traffic markers on mini-map
const trafficMarkersMap = new Map(); // icao24 → L.CircleMarker

/**
 * Initialize the mini-map
 * @param {string} containerId - DOM element ID for map container
 */
export function initMap(containerId) {
    map = L.map(containerId, {
        zoomControl: false,
        attributionControl: false,
        keyboard: false
    }).setView([STATE.lat, STATE.lon], 13);

    // Prevent map container from stealing focus from input fields
    map.getContainer().setAttribute('tabindex', '-1');

    // Satellite layer (with offline cache)
    satelliteLayer = cachedTileLayer(
        'https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        {
            maxZoom: 20,
            opacity: 0.6,
            subdomains: ['0','1','2','3'],
            attribution: 'Google',
            provider: 'esri'
        }
    );
    satelliteLayer.addTo(map);

    // Plane marker
    const planeIcon = L.divIcon({
        html: `<svg viewBox="0 0 24 24" fill="#ffffff" style="filter:drop-shadow(0 0 4px #ffffff); width:100%; height:100%;">
            <path d="M21,16V14L13,9V3.5A1.5,1.5 0 0,0 11.5,2A1.5,1.5 0 0,0 10,3.5V9L2,14V16L10,13.5V19L8,20.5V22L11.5,21L15,22V20.5L13,19V13.5L21,16Z"/>
        </svg>`,
        className: 'plane-marker-icon',
        iconSize: [40, 40],
        iconAnchor: [20, 20]
    });
    marker = L.marker([STATE.lat, STATE.lon], { icon: planeIcon }).addTo(map);

    // Path line (red)
    pathLine = L.polyline([], {
        color: '#ff0000',
        weight: 2,
        opacity: 0.85
    }).addTo(map);
}

/**
 * Update mini-map position and marker rotation
 */
export function updateMap() {
    if (!map) return;

    const newLatLng = new L.LatLng(STATE.lat, STATE.lon);
    marker.setLatLng(newLatLng);
    map.panTo(newLatLng);

    const iconElement = marker.getElement();
    if (iconElement) {
        const deg = STATE.yaw * RAD;
        iconElement.style.transformOrigin = "center center";
        const svg = iconElement.querySelector('svg');
        if (svg) svg.style.transform = `rotate(${deg}deg)`;
    }

    // Update path (skip while a frozen replay trail is on display)
    if (pathLine && !trailFrozen) {
        const cur = newLatLng;
        if (!lastLiveLatLng) {
            lastLiveLatLng = cur;
            livePathPoints = [cur];
        } else {
            const moved = map.distance(lastLiveLatLng, cur);
            if (moved >= 5) {
                livePathPoints.push(cur);
                lastLiveLatLng = cur;
            }
        }
        if (livePathPoints.length > MAX_MAP_PATH_POINTS) {
            livePathPoints = livePathPoints.slice(livePathPoints.length - MAX_MAP_PATH_POINTS);
        }
        pathLine.setLatLngs(livePathPoints);
    }

    map.invalidateSize();

    // Update ADS-B traffic dots
    updateTrafficOverlay();
}

/**
 * Update ADS-B traffic markers on mini-map
 */
export function updateTrafficOverlay() {
    if (!map) return;

    if (!STATE.traffic || STATE.traffic.length === 0) {
        // Remove all markers
        for (const m of trafficMarkersMap.values()) map.removeLayer(m);
        trafficMarkersMap.clear();
        return;
    }

    const activeIcaos = new Set();
    for (const tfc of STATE.traffic) {
        if (tfc.lat == null || tfc.lon == null || !tfc.icao24) continue;
        activeIcaos.add(tfc.icao24);

        const existing = trafficMarkersMap.get(tfc.icao24);
        if (existing) {
            // Update position of existing marker
            existing.setLatLng([tfc.lat, tfc.lon]);
        } else {
            // Create new marker
            const m = L.circleMarker([tfc.lat, tfc.lon], {
                radius: 5,
                color: '#ff0000',
                fillColor: '#ff0000',
                fillOpacity: 0.9,
                weight: 1
            }).addTo(map);
            if (tfc.callsign) {
                m.bindTooltip(tfc.callsign, {
                    permanent: false,
                    direction: 'top',
                    className: 'traffic-tooltip'
                });
            }
            trafficMarkersMap.set(tfc.icao24, m);
        }
    }

    // Remove stale markers
    for (const [icao, m] of trafficMarkersMap) {
        if (!activeIcaos.has(icao)) {
            map.removeLayer(m);
            trafficMarkersMap.delete(icao);
        }
    }
}

/**
 * Invalidate map size (for resize events)
 */
export function invalidateSize() {
    if (map) map.invalidateSize();
}

/**
 * Update mission waypoints on mini-map (home + WPs as small markers)
 */
export function updateMissionOverlay() {
    if (!map) return;

    // Clear previous
    for (const m of missionMarkers) map.removeLayer(m);
    missionMarkers = [];
    if (missionPolyline) { map.removeLayer(missionPolyline); missionPolyline = null; }
    if (homeMarker) { map.removeLayer(homeMarker); homeMarker = null; }

    const items = STATE.missionItems;
    if (!items || items.length === 0) return;

    const latLngs = [];

    for (let i = 0; i < items.length; i++) {
        const it = items[i];
        if (!it.lat && !it.lng) continue;
        const ll = [it.lat, it.lng];
        latLngs.push(ll);

        if (i === 0 && it.frame === 0) {
            // Home marker - small orange circle
            homeMarker = L.circleMarker(ll, {
                radius: 5, color: '#ff8800', fillColor: '#ff8800',
                fillOpacity: 0.9, weight: 1
            }).addTo(map);
        } else {
            // WP marker - small green dot
            const m = L.circleMarker(ll, {
                radius: 3, color: '#44ff44', fillColor: '#44ff44',
                fillOpacity: 0.8, weight: 1
            }).addTo(map);
            missionMarkers.push(m);
        }
    }

    // Thin green polyline connecting WPs
    if (latLngs.length >= 2) {
        missionPolyline = L.polyline(latLngs, {
            color: '#44ff44', weight: 1, opacity: 0.5, dashArray: '4,4'
        }).addTo(map);
    }
}

/**
 * Set or update target marker on mini-map
 */
export function setTargetMarker(lat, lon) {
    if (!map) return;
    clearTargetMarker();
    const ll = [lat, lon];
    // Outer glow ring
    targetMarkerOuter = L.circleMarker(ll, {
        radius: 14, color: '#ff0000', fillColor: '#ff0000',
        fillOpacity: 0.15, weight: 2, opacity: 0.5
    }).addTo(map);
    // Inner solid dot
    targetMarkerInner = L.circleMarker(ll, {
        radius: 6, color: '#ff0000', fillColor: '#ff0000',
        fillOpacity: 0.9, weight: 2
    }).addTo(map);
}

/**
 * Remove target marker from mini-map
 */
export function clearTargetMarker() {
    if (targetMarkerOuter) { map.removeLayer(targetMarkerOuter); targetMarkerOuter = null; }
    if (targetMarkerInner) { map.removeLayer(targetMarkerInner); targetMarkerInner = null; }
}

export function resetMapTrail() {
    livePathPoints = [];
    lastLiveLatLng = null;
    if (pathLine) pathLine.setLatLngs([]);
}

/**
 * Draw a static full-flight trail on the mini-map from a pre-computed list of
 * {lat, lon} (or {lat, lng}) points and freeze live path updates so the line
 * stays intact during replay. If points is empty, trail is cleared and live
 * updates resume.
 */
export function setMapTrail(points) {
    trailFrozen = false;
    livePathPoints = [];
    lastLiveLatLng = null;
    if (!pathLine) return;
    if (!Array.isArray(points) || points.length === 0) {
        pathLine.setLatLngs([]);
        return;
    }
    const latLngs = points.map(p => [p.lat, p.lon !== undefined ? p.lon : p.lng]);
    pathLine.setLatLngs(latLngs);
    trailFrozen = true;

    // Fit the map view to show the full track
    if (map && latLngs.length >= 2) {
        try { map.fitBounds(L.latLngBounds(latLngs), { padding: [20, 20] }); } catch {}
    }
}

export function unfreezeMapTrail() {
    trailFrozen = false;
    livePathPoints = [];
    lastLiveLatLng = null;
    if (pathLine) pathLine.setLatLngs([]);
}

