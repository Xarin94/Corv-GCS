/**
 * NDView.js
 *
 * Airbus A350-style Navigation Display (ND)
 * Supports ARC, ROSE NAV, ROSE VOR, ROSE ILS, and PLAN modes
 */

import { STATE } from '../core/state.js';
import { computePredictedPath2D } from '../engine/TrajectoryPredictor.js';

// Visibility check function (will be set by SplitView)
let checkNDVisible = () => true;

let canvas = null;
let ctx = null;
let dpr = 1;

// ND Configuration State
export const ndConfig = {
    mode: 'ARC',           // ARC, ROSE_NAV, ROSE_VOR, ROSE_ILS, PLAN
    range: 40,             // NM: 10, 20, 40, 80, 160, 320
    showTerrain: true,
    showWxr: false,        // Weather radar
    showTfc: true,         // Traffic (TCAS/ADS-B)
    showWpt: true,         // Waypoints
    showVorDme: true,      // VOR/DME stations
    showNdb: true,         // NDB stations
    showArpt: true,        // Airports
    showCstr: false,       // Constraints
    showPredictedPath: false, // Predicted trajectory line
    navaidFilter: 'AUTO',  // AUTO, VOR, NDB, WPT
    
    // Flight plan / navigation data
    flightPlan: [],        // Array of waypoints
    currentWptIndex: 0,
    
    // Selected navigation data
    selectedHdg: null,     // Selected heading (null = managed)
    selectedTrk: null,     // Selected track
    selectedCrs: null,     // Selected course (VOR/ILS)
    
    // VOR/ILS data
    vor1: { id: '---', freq: '---.-', crs: 0, dme: null, active: false },
    vor2: { id: '---', freq: '---.-', crs: 0, dme: null, active: false },
    ils: { id: '---', freq: '---.-', crs: 0, active: false },
    
    // Wind data
    windDir: 0,
    windSpd: 0,
    
    // Ground speed / True airspeed
    gs: 0,
    tas: 0,
    
    // ETA/Distance to next waypoint
    etaNext: '---',
    distNext: 0,
    nextWpt: '-----'
};

// Predefined flight plans
export const FLIGHT_PLANS = {
    'IBK-BLQ': {
        name: 'Innsbruck → Bologna',
        departure: 'LOWI',
        arrival: 'LIPE',
        waypoints: [
            { id: 'LOWI', lat: 47.2602, lon: 11.3439 },   // Innsbruck Airport
            { id: 'RTT', lat: 47.0833, lon: 11.3167 },    // Rattenberg VOR
            { id: 'BZO', lat: 46.4603, lon: 11.3264 },    // Bolzano VOR
            { id: 'EVANO', lat: 46.2167, lon: 11.3000 },  // EVANO waypoint
            { id: 'RONAG', lat: 45.8500, lon: 11.2833 },  // RONAG waypoint
            { id: 'VIC', lat: 45.5722, lon: 11.5297 },    // Vicenza VOR
            { id: 'PERON', lat: 45.2833, lon: 11.4500 },  // PERON waypoint
            { id: 'RIDGO', lat: 44.8000, lon: 11.3500 },  // RIDGO waypoint
            { id: 'BOA', lat: 44.5353, lon: 11.2886 },    // Bologna VOR
            { id: 'LIPE', lat: 44.5354, lon: 11.2887 }    // Bologna Airport
        ]
    },
    'DEMO': {
        name: 'Demo Route (Alps)',
        departure: '---',
        arrival: '---',
        waypoints: [
            { id: 'START', lat: 46.0, lon: 11.0 },
            { id: 'WPT01', lat: 46.2, lon: 11.3 },
            { id: 'WPT02', lat: 46.4, lon: 11.1 },
            { id: 'WPT03', lat: 46.6, lon: 11.4 },
            { id: 'END', lat: 46.8, lon: 11.2 }
        ]
    },
    'BLQ-MXP': {
        name: 'Bologna → Milan Malpensa',
        departure: 'LIPE',
        arrival: 'LIMC',
        waypoints: [
            { id: 'LIPE', lat: 44.5354, lon: 11.2887 },   // Bologna Airport
            { id: 'BOA', lat: 44.5353, lon: 11.2886 },    // Bologna VOR
            { id: 'TIMPY', lat: 44.7500, lon: 10.6667 },  // TIMPY waypoint
            { id: 'NIKMO', lat: 44.9333, lon: 10.2167 },  // NIKMO waypoint
            { id: 'TOP', lat: 45.1500, lon: 9.7167 },     // TOP VOR
            { id: 'LAGEN', lat: 45.3833, lon: 9.1333 },   // LAGEN waypoint
            { id: 'MLP', lat: 45.6300, lon: 8.7231 },     // Milan VOR
            { id: 'LIMC', lat: 45.6306, lon: 8.7231 }     // Malpensa Airport
        ]
    }
};

// Colors - Airbus standard
const COLORS = {
    background: '#0a0a12',
    green: '#00ff00',
    cyan: '#00ffff',
    magenta: '#ff00ff',
    white: '#ffffff',
    yellow: '#ffff00',
    amber: '#ff9900',
    red: '#ff0000',
    blue: '#0088ff',
    dimGreen: '#008800',
    dimCyan: '#006666',
    terrainYellow: '#aa8800',
    terrainRed: '#880000',
    skyBlue: '#001133'
};

export function initND(canvasEl) {
    canvas = canvasEl;
    if (!canvas) return false;

    ctx = canvas.getContext('2d');
    if (!ctx) return false;

    dpr = Math.max(1, Math.floor(window.devicePixelRatio || 1));
    resizeND();
    
    // Start animation loop
    requestAnimationFrame(animateND);
    return true;
}

function animateND() {
    // Only render if ND is visible to save GPU/CPU resources
    if (checkNDVisible()) {
        drawND();
    }
    requestAnimationFrame(animateND);
}

/**
 * Set the visibility check function (called by SplitView)
 * @param {Function} fn - Function that returns true if ND is visible
 */
export function setNDVisibilityCheck(fn) {
    if (typeof fn === 'function') {
        checkNDVisible = fn;
    }
}

export function resizeND() {
    if (!canvas || !ctx) return;

    const rect = canvas.getBoundingClientRect();
    const w = Math.max(1, Math.floor(rect.width * dpr));
    const h = Math.max(1, Math.floor(rect.height * dpr));

    if (canvas.width !== w) canvas.width = w;
    if (canvas.height !== h) canvas.height = h;

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.imageSmoothingEnabled = true;
}

export function drawND() {
    if (!canvas || !ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    if (w <= 1 || h <= 1) return;

    // Clear with background (theme-aware)
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--bg-nd').trim() || COLORS.background;
    ctx.fillRect(0, 0, w, h);
    
    // Get current aircraft state
    const hdg = ((STATE.yaw * 180 / Math.PI) + 360) % 360;
    const trk = ((STATE.track * 180 / Math.PI) + 360) % 360;
    
    // Update config from state
    ndConfig.gs = STATE.gs || 0;
    ndConfig.tas = STATE.as || 0;
    
    // Determine center and radius based on mode
    const size = Math.min(w, h);
    const cx = w / 2;
    let cy, radius;
    
    if (ndConfig.mode === 'ARC') {
        cy = h * 0.75;
        radius = size * 0.6;
    } else {
        cy = h / 2;
        radius = size * 0.4;
    }
    
    ctx.save();
    ctx.scale(dpr, dpr);
    const sw = w / dpr;
    const sh = h / dpr;
    const scx = cx / dpr;
    const scy = cy / dpr;
    const sr = radius / dpr;
    
    // Draw based on mode
    switch (ndConfig.mode) {
        case 'ARC':
            drawArcMode(scx, scy, sr, sw, sh, hdg, trk);
            break;
        case 'ROSE_NAV':
            drawRoseNavMode(scx, sh / 2, sr * 0.85, sw, sh, hdg, trk);
            break;
        case 'ROSE_VOR':
            drawRoseVorMode(scx, sh / 2, sr * 0.85, sw, sh, hdg, trk);
            break;
        case 'ROSE_ILS':
            drawRoseIlsMode(scx, sh / 2, sr * 0.85, sw, sh, hdg, trk);
            break;
        case 'PLAN':
            drawPlanMode(scx, sh / 2, sr, sw, sh, hdg, trk);
            break;
    }
    
    // Draw common overlays
    drawTopInfoBar(sw, sh, hdg, trk);
    drawBottomInfoBar(sw, sh);
    drawSideInfoPanels(sw, sh);
    
    ctx.restore();
}

// ============= ARC MODE =============
function drawArcMode(cx, cy, r, w, h, hdg, trk) {
    // Draw range arc segments (fixed, don't rotate - they are distance markers)
    drawRangeArcs(cx, cy, r, hdg, 'ARC');
    
    // Draw compass arc (120 degrees) - rotates to show current heading at top
    drawCompassArc(cx, cy, r, hdg);
    
    // Draw aircraft symbol at center (fixed)
    drawAircraftSymbol(cx, cy, 0);

    // Predicted trajectory line
    drawPredictedPath(cx, cy, r, hdg, 'ARC');

    // Draw heading bug
    if (ndConfig.selectedHdg !== null) {
        drawHeadingBug(cx, cy, r, hdg, ndConfig.selectedHdg);
    }

    // Draw flight plan route
    drawFlightPlanRoute(cx, cy, r, hdg, 'ARC');

    // Draw ADS-B traffic
    drawTraffic(cx, cy, r, hdg, 'ARC');

    // Draw wind arrow
    drawWindArrow(w - 80, 120, ndConfig.windDir, ndConfig.windSpd, hdg);
}

// ============= ROSE NAV MODE =============
function drawRoseNavMode(cx, cy, r, w, h, hdg, trk) {
    // Full compass rose (rotates with heading)
    drawCompassRose(cx, cy, r, hdg);
    
    // Range rings (fixed, don't rotate - they are distance markers)
    drawRangeRings(cx, cy, r, true, hdg);
    
    // Aircraft symbol (fixed at center)
    drawAircraftSymbol(cx, cy, 0);

    // Predicted trajectory line
    drawPredictedPath(cx, cy, r, hdg, 'ROSE');

    // Heading bug
    if (ndConfig.selectedHdg !== null) {
        drawHeadingBug(cx, cy, r, hdg, ndConfig.selectedHdg);
    }

    // Flight plan route
    drawFlightPlanRoute(cx, cy, r, hdg, 'ROSE');

    // ADS-B traffic
    drawTraffic(cx, cy, r, hdg, 'ROSE');

    // Wind arrow
    drawWindArrow(w - 80, 80, ndConfig.windDir, ndConfig.windSpd, hdg);
}

// ============= ROSE VOR MODE =============
function drawRoseVorMode(cx, cy, r, w, h, hdg, trk) {
    // Full compass rose
    drawCompassRose(cx, cy, r, hdg);
    
    // VOR/DME deviation indicator
    drawVorDeviation(cx, cy, r * 0.7, ndConfig.vor1);
    
    // Aircraft symbol
    drawAircraftSymbol(cx, cy, 0);
    
    // Heading bug
    if (ndConfig.selectedHdg !== null) {
        drawHeadingBug(cx, cy, r, hdg, ndConfig.selectedHdg);
    }
    
    // TO/FROM indicator
    drawToFromIndicator(cx, cy - r * 0.3, ndConfig.vor1);
    
    // Wind arrow
    drawWindArrow(w - 80, 80, ndConfig.windDir, ndConfig.windSpd, hdg);
}

// ============= ROSE ILS MODE =============
function drawRoseIlsMode(cx, cy, r, w, h, hdg, trk) {
    // Full compass rose
    drawCompassRose(cx, cy, r, hdg);
    
    // ILS localizer and glideslope
    drawIlsDeviation(cx, cy, r * 0.7, ndConfig.ils);
    
    // Aircraft symbol
    drawAircraftSymbol(cx, cy, 0);
    
    // Heading bug
    if (ndConfig.selectedHdg !== null) {
        drawHeadingBug(cx, cy, r, hdg, ndConfig.selectedHdg);
    }
    
    // Glideslope indicator on right side
    drawGlideslopeIndicator(cx + r + 30, cy, r * 0.6);
    
    // Wind arrow
    drawWindArrow(w - 80, 80, ndConfig.windDir, ndConfig.windSpd, hdg);
}

// ============= PLAN MODE =============
function drawPlanMode(cx, cy, r, w, h, hdg, trk) {
    // North-up display
    ctx.fillStyle = COLORS.white;
    ctx.font = 'bold 14px "Roboto Mono"';
    ctx.textAlign = 'center';
    ctx.fillText('N', cx, 35);
    ctx.fillText('▲', cx, 50);
    
    // Range rings (fixed north-up)
    drawRangeRings(cx, cy, r, true, hdg);
    
    // Flight plan route (north-up)
    drawFlightPlanRoute(cx, cy, r, 0, 'PLAN');

    // Aircraft symbol rotated by heading (north-up background)
    drawAircraftSymbol(cx, cy, hdg * Math.PI / 180);

    // Predicted trajectory line
    drawPredictedPath(cx, cy, r, 0, 'PLAN');

    // ADS-B traffic
    drawTraffic(cx, cy, r, 0, 'PLAN');
}

// ============= DRAWING HELPERS =============

function drawCompassArc(cx, cy, r, hdg) {
    // Save context for clipping
    ctx.save();
    
    // Create clipping region for 120-degree arc at top
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, r + 50, Math.PI + Math.PI/6, -Math.PI/6);
    ctx.closePath();
    ctx.clip();
    
    // Now draw the full compass rose, rotated by heading
    // The compass rotates so current heading appears at top
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-hdg * Math.PI / 180);
    
    // Draw outer circle
    ctx.strokeStyle = COLORS.white;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.stroke();
    
    // Draw all tick marks and numbers around the full 360 degrees
    for (let deg = 0; deg < 360; deg += 5) {
        const angle = deg * Math.PI / 180 - Math.PI / 2;  // 0 degrees at top
        
        let tickLen = 8;
        let showNumber = false;
        
        if (deg % 30 === 0) {
            tickLen = 20;
            showNumber = true;
        } else if (deg % 10 === 0) {
            tickLen = 15;
            showNumber = true;
        }
        
        const x1 = Math.cos(angle) * r;
        const y1 = Math.sin(angle) * r;
        const x2 = Math.cos(angle) * (r - tickLen);
        const y2 = Math.sin(angle) * (r - tickLen);
        
        ctx.strokeStyle = COLORS.white;
        ctx.lineWidth = deg % 10 === 0 ? 2 : 1;
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
        
        // Draw numbers for every 10 degrees
        if (showNumber && deg % 10 === 0) {
            const labelR = r - tickLen - 15;
            const lx = Math.cos(angle) * labelR;
            const ly = Math.sin(angle) * labelR;
            
            // Counter-rotate the text so it stays upright
            ctx.save();
            ctx.translate(lx, ly);
            ctx.rotate(hdg * Math.PI / 180);
            
            ctx.fillStyle = COLORS.white;
            ctx.font = 'bold 14px "Roboto Mono"';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            
            let label = (deg / 10).toString();
            if (deg === 0) label = 'N';
            else if (deg === 90) label = 'E';
            else if (deg === 180) label = 'S';
            else if (deg === 270) label = 'W';
            
            ctx.fillText(label, 0, 0);
            ctx.restore();
        }
    }
    
    ctx.restore();  // Restore from rotation
    ctx.restore();  // Restore from clipping
    
    // Draw current heading indicator (fixed yellow triangle at top)
    ctx.fillStyle = COLORS.yellow;
    ctx.beginPath();
    ctx.moveTo(cx, cy - r - 5);
    ctx.lineTo(cx - 10, cy - r - 20);
    ctx.lineTo(cx + 10, cy - r - 20);
    ctx.closePath();
    ctx.fill();
    
    // Heading readout
    ctx.fillStyle = COLORS.green;
    ctx.font = 'bold 24px "Roboto Mono"';
    ctx.textAlign = 'center';
    ctx.fillText(Math.round(hdg).toString().padStart(3, '0') + '°', cx, cy - r - 35);
}

function drawCompassRose(cx, cy, r, hdg) {
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-hdg * Math.PI / 180);
    
    // Draw circle
    ctx.strokeStyle = COLORS.white;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.stroke();
    
    // Draw tick marks
    for (let deg = 0; deg < 360; deg += 5) {
        const angle = deg * Math.PI / 180 - Math.PI / 2;
        let tickLen = 8;
        let showNumber = false;
        
        if (deg % 30 === 0) {
            tickLen = 20;
            showNumber = true;
        } else if (deg % 10 === 0) {
            tickLen = 12;
        }
        
        const x1 = Math.cos(angle) * r;
        const y1 = Math.sin(angle) * r;
        const x2 = Math.cos(angle) * (r - tickLen);
        const y2 = Math.sin(angle) * (r - tickLen);
        
        ctx.strokeStyle = COLORS.white;
        ctx.lineWidth = deg % 10 === 0 ? 2 : 1;
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
        
        if (showNumber) {
            const labelR = r - tickLen - 15;
            const lx = Math.cos(angle) * labelR;
            const ly = Math.sin(angle) * labelR;
            
            ctx.save();
            ctx.translate(lx, ly);
            ctx.rotate(hdg * Math.PI / 180);
            
            ctx.fillStyle = COLORS.white;
            ctx.font = 'bold 14px "Roboto Mono"';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            
            let label = (deg / 10).toString();
            if (deg === 0) label = 'N';
            else if (deg === 90) label = 'E';
            else if (deg === 180) label = 'S';
            else if (deg === 270) label = 'W';
            
            ctx.fillText(label, 0, 0);
            ctx.restore();
        }
    }
    
    ctx.restore();
    
    // Draw fixed heading indicator at top
    ctx.fillStyle = COLORS.yellow;
    ctx.beginPath();
    ctx.moveTo(cx, cy - r - 5);
    ctx.lineTo(cx - 8, cy - r - 18);
    ctx.lineTo(cx + 8, cy - r - 18);
    ctx.closePath();
    ctx.fill();
}

function drawRangeArcs(cx, cy, r, hdg, mode) {
    const ranges = [0.25, 0.5, 0.75, 1.0];
    
    // Range arcs are FIXED (don't rotate) - they represent distance from aircraft
    ctx.strokeStyle = COLORS.dimCyan;
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);
    
    ranges.forEach((ratio, idx) => {
        const arcR = r * ratio;
        const isOuterRing = ratio === 1.0;
        
        if (isOuterRing) {
            // Outer ring: only draw a dot at the top (fixed position)
            ctx.setLineDash([]);
            ctx.fillStyle = COLORS.dimCyan;
            ctx.beginPath();
            ctx.arc(cx, cy - arcR, 4, 0, Math.PI * 2);  // Dot at top
            ctx.fill();
        } else {
            // Inner arcs: draw arc (fixed)
            if (mode === 'ARC') {
                ctx.beginPath();
                ctx.arc(cx, cy, arcR, Math.PI + Math.PI/6, -Math.PI/6);
                ctx.stroke();
            } else {
                ctx.beginPath();
                ctx.arc(cx, cy, arcR, 0, Math.PI * 2);
                ctx.stroke();
            }
        }
        
        // Range label on the arc line (at top of arc, fixed position)
        if (idx > 0 && !isOuterRing) {
            const rangeNm = ndConfig.range * ratio;
            ctx.setLineDash([]);
            ctx.fillStyle = COLORS.cyan;
            ctx.font = '11px "Roboto Mono"';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(rangeNm.toFixed(0), cx, cy - arcR);
            ctx.setLineDash([5, 5]);
        }
    });
    
    ctx.setLineDash([]);
}

function drawRangeRings(cx, cy, r, northUp = false, hdg = 0) {
    const rings = [0.5, 1.0];
    
    // Range rings are FIXED (don't rotate) - they represent distance from aircraft
    ctx.strokeStyle = COLORS.dimCyan;
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);
    
    rings.forEach((ratio) => {
        const ringR = r * ratio;
        const isOuterRing = ratio === 1.0;
        
        if (isOuterRing) {
            // Outer ring: only draw a dot at the top (fixed position)
            ctx.setLineDash([]);
            ctx.fillStyle = COLORS.dimCyan;
            ctx.beginPath();
            ctx.arc(cx, cy - ringR, 4, 0, Math.PI * 2);  // Dot at top
            ctx.fill();
        } else {
            // Inner rings: draw full circle (fixed)
            ctx.beginPath();
            ctx.arc(cx, cy, ringR, 0, Math.PI * 2);
            ctx.stroke();
            
            // Range label on the ring (at top, fixed position)
            const rangeNm = ndConfig.range * ratio;
            ctx.setLineDash([]);
            ctx.fillStyle = COLORS.cyan;
            ctx.font = '11px "Roboto Mono"';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(rangeNm.toFixed(0), cx, cy - ringR);
            ctx.setLineDash([5, 5]);
        }
    });
    
    ctx.setLineDash([]);
}

function drawAircraftSymbol(cx, cy, rotation) {
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(rotation);
    
    ctx.strokeStyle = COLORS.yellow;
    ctx.fillStyle = COLORS.yellow;
    ctx.lineWidth = 2;
    
    // Fuselage
    ctx.beginPath();
    ctx.moveTo(0, -20);
    ctx.lineTo(0, 15);
    ctx.stroke();
    
    // Wings
    ctx.beginPath();
    ctx.moveTo(-25, 0);
    ctx.lineTo(25, 0);
    ctx.stroke();
    
    // Horizontal stabilizer
    ctx.beginPath();
    ctx.moveTo(-10, 12);
    ctx.lineTo(10, 12);
    ctx.stroke();
    
    // Nose
    ctx.beginPath();
    ctx.arc(0, -20, 3, 0, Math.PI * 2);
    ctx.fill();
    
    ctx.restore();
}

function drawHeadingBug(cx, cy, r, currentHdg, selectedHdg) {
    const relAngle = ((selectedHdg - currentHdg) + 360) % 360;
    const angle = (90 - relAngle) * Math.PI / 180;
    
    const x = cx + Math.cos(angle) * r;
    const y = cy - Math.sin(angle) * r;
    
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-angle + Math.PI/2);
    
    ctx.fillStyle = COLORS.cyan;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(-8, -15);
    ctx.lineTo(-8, -20);
    ctx.lineTo(8, -20);
    ctx.lineTo(8, -15);
    ctx.closePath();
    ctx.fill();
    
    ctx.restore();
}

function drawTrackLine(cx, cy, r, hdg, trk) {
    const relAngle = ((trk - hdg) + 360) % 360;
    const angle = (90 - relAngle) * Math.PI / 180;
    
    ctx.strokeStyle = COLORS.green;
    ctx.lineWidth = 2;
    ctx.setLineDash([10, 5]);
    
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(angle) * r, cy - Math.sin(angle) * r);
    ctx.stroke();
    
    ctx.setLineDash([]);
    
    // Track diamond at end
    const dx = cx + Math.cos(angle) * (r - 10);
    const dy = cy - Math.sin(angle) * (r - 10);
    
    ctx.fillStyle = COLORS.green;
    ctx.beginPath();
    ctx.moveTo(dx, dy - 6);
    ctx.lineTo(dx + 5, dy);
    ctx.lineTo(dx, dy + 6);
    ctx.lineTo(dx - 5, dy);
    ctx.closePath();
    ctx.fill();
}

function drawFlightPlanRoute(cx, cy, r, hdg, mode) {
    if (ndConfig.flightPlan.length < 2) return;
    
    const nmPerPixel = ndConfig.range / r;
    const acLat = STATE.lat;
    const acLon = STATE.lon;
    
    ctx.strokeStyle = COLORS.magenta;
    ctx.lineWidth = 2;
    
    const points = ndConfig.flightPlan.map((wpt, idx) => {
        // Convert lat/lon to relative NM
        const dLat = (wpt.lat - acLat) * 60;
        const dLon = (wpt.lon - acLon) * 60 * Math.cos(acLat * Math.PI / 180);
        
        // Distance and bearing
        const distNm = Math.sqrt(dLat * dLat + dLon * dLon);
        let bearing = Math.atan2(dLon, dLat) * 180 / Math.PI;
        
        // Adjust for heading in non-PLAN modes
        if (mode !== 'PLAN') {
            bearing = bearing - hdg;
        }
        
        const bearingRad = bearing * Math.PI / 180;
        const pixelDist = Math.min(distNm / nmPerPixel, r);
        
        return {
            x: cx + Math.sin(bearingRad) * pixelDist,
            y: cy - Math.cos(bearingRad) * pixelDist,
            id: wpt.id,
            active: idx === ndConfig.currentWptIndex
        };
    });
    
    // Draw route line
    ctx.beginPath();
    points.forEach((pt, idx) => {
        if (idx === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
    });
    ctx.stroke();
    
    // Draw waypoint symbols
    points.forEach((pt) => {
        ctx.fillStyle = pt.active ? COLORS.white : COLORS.magenta;
        ctx.strokeStyle = pt.active ? COLORS.white : COLORS.magenta;
        ctx.lineWidth = 2;
        
        // Waypoint diamond
        ctx.beginPath();
        ctx.moveTo(pt.x, pt.y - 8);
        ctx.lineTo(pt.x + 6, pt.y);
        ctx.lineTo(pt.x, pt.y + 8);
        ctx.lineTo(pt.x - 6, pt.y);
        ctx.closePath();
        ctx.stroke();
        
        // Waypoint label
        ctx.fillStyle = pt.active ? COLORS.white : COLORS.cyan;
        ctx.font = '11px "Roboto Mono"';
        ctx.textAlign = 'center';
        ctx.fillText(pt.id, pt.x, pt.y - 12);
    });
}

function drawPredictedPath(cx, cy, r, hdg, mode) {
    if (!ndConfig.showPredictedPath) return;
    if ((STATE.gs || 0) < 2) return;

    const points = computePredictedPath2D(STATE, 30, 15);
    if (points.length < 2) return;

    const nmPerPixel = ndConfig.range / r;
    const acLat = STATE.lat;
    const acLon = STATE.lon;
    const cosAcLat = Math.cos(acLat * Math.PI / 180);

    ctx.save();
    ctx.lineWidth = 2.5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    // Convert all points to pixel coords
    const pxPoints = [];
    for (let i = 0; i < points.length; i++) {
        const dLat = (points[i].lat - acLat) * 60;
        const dLon = (points[i].lon - acLon) * 60 * cosAcLat;
        const distNm = Math.sqrt(dLat * dLat + dLon * dLon);
        let bearing = Math.atan2(dLon, dLat) * 180 / Math.PI;
        if (mode !== 'PLAN') bearing -= hdg;
        const bearingRad = bearing * Math.PI / 180;
        const pixelDist = distNm / nmPerPixel;

        pxPoints.push({
            x: cx + Math.sin(bearingRad) * pixelDist,
            y: cy - Math.cos(bearingRad) * pixelDist
        });
    }

    // Draw segments with fading alpha
    for (let i = 0; i < pxPoints.length - 1; i++) {
        const alpha = 0.7 * (1 - i / (pxPoints.length - 1));
        ctx.globalAlpha = alpha;
        ctx.strokeStyle = COLORS.cyan;
        ctx.beginPath();
        ctx.moveTo(pxPoints[i].x, pxPoints[i].y);
        ctx.lineTo(pxPoints[i + 1].x, pxPoints[i + 1].y);
        ctx.stroke();
    }

    ctx.globalAlpha = 1;
    ctx.restore();
}

// ============= TRAFFIC (TCAS/ADS-B) =============
function drawTraffic(cx, cy, r, hdg, mode) {
    if (!ndConfig.showTfc) return;
    if (!STATE.traffic || STATE.traffic.length === 0) return;

    const nmPerPixel = ndConfig.range / r;
    const acLat = STATE.lat;
    const acLon = STATE.lon;
    const cosLat = Math.cos(acLat * Math.PI / 180);

    for (const tfc of STATE.traffic) {
        if (tfc.lat == null || tfc.lon == null) continue;

        const dLat = (tfc.lat - acLat) * 60;
        const dLon = (tfc.lon - acLon) * 60 * cosLat;
        const distNm = Math.sqrt(dLat * dLat + dLon * dLon);
        if (distNm > ndConfig.range) continue;

        let bearing = Math.atan2(dLon, dLat) * 180 / Math.PI;
        if (mode !== 'PLAN') bearing -= hdg;
        const bearingRad = bearing * Math.PI / 180;
        const pixelDist = distNm / nmPerPixel;

        const tx = cx + Math.sin(bearingRad) * pixelDist;
        const ty = cy - Math.cos(bearingRad) * pixelDist;

        // Red dot
        ctx.fillStyle = COLORS.red;
        ctx.beginPath();
        ctx.arc(tx, ty, 5, 0, Math.PI * 2);
        ctx.fill();

        // Callsign label
        if (tfc.callsign) {
            ctx.fillStyle = COLORS.white;
            ctx.font = '9px "Roboto Mono"';
            ctx.textAlign = 'center';
            ctx.fillText(tfc.callsign, tx, ty - 9);
        }
    }
}

function drawVorDeviation(cx, cy, r, vorData) {
    // Course deviation bar
    ctx.strokeStyle = COLORS.cyan;
    ctx.lineWidth = 2;
    
    // Center line
    ctx.beginPath();
    ctx.moveTo(cx, cy - r);
    ctx.lineTo(cx, cy + r);
    ctx.stroke();
    
    // Deviation dots (5 each side)
    const dotSpacing = r * 0.15;
    ctx.fillStyle = COLORS.white;
    for (let i = -2; i <= 2; i++) {
        if (i === 0) continue;
        const x = cx + i * dotSpacing;
        ctx.beginPath();
        ctx.arc(x, cy, 4, 0, Math.PI * 2);
        ctx.fill();
    }
    
    // CDI needle (simulated deviation)
    const deviation = 0; // Would come from VOR receiver
    ctx.fillStyle = COLORS.cyan;
    ctx.fillRect(cx + deviation * dotSpacing - 3, cy - r * 0.8, 6, r * 1.6);
}

function drawIlsDeviation(cx, cy, r, ilsData) {
    // Draw VOR-style deviation first
    drawVorDeviation(cx, cy, r, { crs: ilsData.crs });
    
    // Extended centerline for localizer
    ctx.strokeStyle = COLORS.magenta;
    ctx.lineWidth = 2;
    ctx.setLineDash([10, 5]);
    ctx.beginPath();
    ctx.moveTo(cx, cy - r * 1.3);
    ctx.lineTo(cx, cy - r);
    ctx.stroke();
    ctx.setLineDash([]);
}

function drawGlideslopeIndicator(x, cy, height) {
    ctx.strokeStyle = COLORS.white;
    ctx.lineWidth = 2;
    
    // Vertical scale
    ctx.beginPath();
    ctx.moveTo(x, cy - height/2);
    ctx.lineTo(x, cy + height/2);
    ctx.stroke();
    
    // Deviation dots
    const dotSpacing = height / 5;
    ctx.fillStyle = COLORS.white;
    for (let i = -2; i <= 2; i++) {
        if (i === 0) continue;
        const y = cy + i * dotSpacing;
        ctx.beginPath();
        ctx.arc(x, y, 4, 0, Math.PI * 2);
        ctx.fill();
    }
    
    // Center marker
    ctx.strokeStyle = COLORS.yellow;
    ctx.beginPath();
    ctx.moveTo(x - 10, cy);
    ctx.lineTo(x + 10, cy);
    ctx.stroke();
    
    // GS diamond (simulated)
    const gsDeviation = 0;
    ctx.fillStyle = COLORS.magenta;
    ctx.beginPath();
    ctx.moveTo(x, cy + gsDeviation * dotSpacing - 8);
    ctx.lineTo(x + 8, cy + gsDeviation * dotSpacing);
    ctx.lineTo(x, cy + gsDeviation * dotSpacing + 8);
    ctx.lineTo(x - 8, cy + gsDeviation * dotSpacing);
    ctx.closePath();
    ctx.fill();
    
    // Label
    ctx.fillStyle = COLORS.white;
    ctx.font = 'bold 12px "Roboto Mono"';
    ctx.textAlign = 'center';
    ctx.fillText('G/S', x, cy - height/2 - 10);
}

function drawToFromIndicator(x, y, vorData) {
    ctx.fillStyle = COLORS.white;
    ctx.font = 'bold 14px "Roboto Mono"';
    ctx.textAlign = 'center';
    
    // Triangle pointing up (TO) or down (FROM)
    const toFrom = 'TO'; // Would come from VOR receiver
    
    if (toFrom === 'TO') {
        ctx.beginPath();
        ctx.moveTo(x, y - 10);
        ctx.lineTo(x - 8, y + 5);
        ctx.lineTo(x + 8, y + 5);
        ctx.closePath();
        ctx.fill();
        ctx.fillText('TO', x, y + 20);
    } else {
        ctx.beginPath();
        ctx.moveTo(x, y + 10);
        ctx.lineTo(x - 8, y - 5);
        ctx.lineTo(x + 8, y - 5);
        ctx.closePath();
        ctx.fill();
        ctx.fillText('FROM', x, y + 20);
    }
}

function drawWindArrow(x, y, windDir, windSpd, hdg) {
    ctx.save();
    ctx.translate(x, y);
    
    // Wind direction relative to heading
    const relWindDir = ((windDir - hdg) + 360) % 360;
    
    // Wind arrow
    ctx.rotate(relWindDir * Math.PI / 180);
    ctx.strokeStyle = COLORS.cyan;
    ctx.fillStyle = COLORS.cyan;
    ctx.lineWidth = 2;
    
    // Arrow shaft
    ctx.beginPath();
    ctx.moveTo(0, -25);
    ctx.lineTo(0, 25);
    ctx.stroke();
    
    // Arrow head (pointing down = wind from that direction)
    ctx.beginPath();
    ctx.moveTo(0, 25);
    ctx.lineTo(-6, 15);
    ctx.lineTo(6, 15);
    ctx.closePath();
    ctx.fill();
    
    ctx.restore();
    
    // Wind data text
    ctx.fillStyle = COLORS.cyan;
    ctx.font = '12px "Roboto Mono"';
    ctx.textAlign = 'center';
    ctx.fillText(Math.round(windDir).toString().padStart(3, '0') + '°', x, y + 45);
    ctx.fillText(Math.round(windSpd) + ' KT', x, y + 60);
}

function drawTopInfoBar(w, h, hdg, trk) {
    const barY = 20;
    
    // Ground speed
    ctx.fillStyle = COLORS.green;
    ctx.font = 'bold 16px "Roboto Mono"';
    ctx.textAlign = 'left';
    ctx.fillText('GS', 15, barY);
    ctx.fillStyle = COLORS.green;
    ctx.font = 'bold 20px "Roboto Mono"';
    ctx.fillText(Math.round(ndConfig.gs * 1.944).toString(), 50, barY); // m/s to kt
    
    // True airspeed
    ctx.fillStyle = COLORS.white;
    ctx.font = 'bold 16px "Roboto Mono"';
    ctx.fillText('TAS', 15, barY + 25);
    ctx.font = 'bold 20px "Roboto Mono"';
    ctx.fillText(Math.round(ndConfig.tas * 1.944).toString(), 60, barY + 25);
    
    // Mode annunciation (center top)
    ctx.fillStyle = COLORS.white;
    ctx.font = 'bold 18px "Roboto Mono"';
    ctx.textAlign = 'center';
    ctx.fillText(ndConfig.mode.replace('_', ' '), w/2, barY);
    
    // Range
    ctx.fillStyle = COLORS.cyan;
    ctx.font = 'bold 14px "Roboto Mono"';
    ctx.fillText(ndConfig.range + ' NM', w/2, barY + 20);
    
    // HDG/TRK display (top right)
    ctx.textAlign = 'right';
    ctx.fillStyle = COLORS.white;
    ctx.font = '12px "Roboto Mono"';
    ctx.fillText('HDG', w - 80, barY - 5);
    ctx.fillStyle = COLORS.green;
    ctx.font = 'bold 20px "Roboto Mono"';
    ctx.fillText(Math.round(hdg).toString().padStart(3, '0') + '°', w - 15, barY);
    
    ctx.fillStyle = COLORS.white;
    ctx.font = '12px "Roboto Mono"';
    ctx.fillText('TRK', w - 80, barY + 20);
    ctx.fillStyle = COLORS.green;
    ctx.font = 'bold 20px "Roboto Mono"';
    ctx.fillText(Math.round(trk).toString().padStart(3, '0') + '°', w - 15, barY + 25);
}

function drawBottomInfoBar(w, h) {
    const barY = h - 15;  // Original position - dropdown now hidden by default
    
    // Next waypoint info
    ctx.fillStyle = COLORS.white;
    ctx.font = '12px "Roboto Mono"';
    ctx.textAlign = 'left';
    ctx.fillText('TO', 15, barY - 20);
    
    ctx.fillStyle = COLORS.magenta;
    ctx.font = 'bold 16px "Roboto Mono"';
    ctx.fillText(ndConfig.nextWpt, 40, barY - 20);
    
    // Distance
    ctx.fillStyle = COLORS.white;
    ctx.font = '12px "Roboto Mono"';
    ctx.fillText('DIST', 15, barY);
    ctx.fillStyle = COLORS.green;
    ctx.font = 'bold 16px "Roboto Mono"';
    ctx.fillText(ndConfig.distNext.toFixed(1) + ' NM', 55, barY);
    
    // ETA
    ctx.fillStyle = COLORS.white;
    ctx.font = '12px "Roboto Mono"';
    ctx.textAlign = 'right';
    ctx.fillText('ETA', w - 100, barY);
    ctx.fillStyle = COLORS.green;
    ctx.font = 'bold 16px "Roboto Mono"';
    ctx.fillText(ndConfig.etaNext, w - 15, barY);
}

function drawSideInfoPanels(w, h) {
    // VOR 1 info (bottom left) - original position
    if (ndConfig.vor1.active) {
        ctx.fillStyle = COLORS.cyan;
        ctx.font = '12px "Roboto Mono"';
        ctx.textAlign = 'left';
        ctx.fillText('VOR1', 15, h - 80);
        ctx.fillText(ndConfig.vor1.id, 15, h - 65);
        ctx.fillText(ndConfig.vor1.freq, 15, h - 50);
        if (ndConfig.vor1.dme !== null) {
            ctx.fillText('DME ' + ndConfig.vor1.dme.toFixed(1), 15, h - 35);
        }
    }
    
    // VOR 2 info (bottom right) - original position
    if (ndConfig.vor2.active) {
        ctx.fillStyle = COLORS.cyan;
        ctx.font = '12px "Roboto Mono"';
        ctx.textAlign = 'right';
        ctx.fillText('VOR2', w - 15, h - 80);
        ctx.fillText(ndConfig.vor2.id, w - 15, h - 65);
        ctx.fillText(ndConfig.vor2.freq, w - 15, h - 50);
        if (ndConfig.vor2.dme !== null) {
            ctx.fillText('DME ' + ndConfig.vor2.dme.toFixed(1), w - 15, h - 35);
        }
    }
}

// ============= PUBLIC API =============

export function setNDMode(mode) {
    const validModes = ['ARC', 'ROSE_NAV', 'ROSE_VOR', 'ROSE_ILS', 'PLAN'];
    if (validModes.includes(mode)) {
        ndConfig.mode = mode;
    }
}

export function setNDRange(range) {
    const validRanges = [10, 20, 40, 80, 160, 320];
    if (validRanges.includes(range)) {
        ndConfig.range = range;
    }
}

export function setSelectedHeading(hdg) {
    ndConfig.selectedHdg = hdg !== null ? ((hdg % 360) + 360) % 360 : null;
}

export function setFlightPlan(waypoints) {
    ndConfig.flightPlan = waypoints;
}

export function setCurrentWaypoint(index) {
    ndConfig.currentWptIndex = index;
}

export function setWindData(dir, spd) {
    ndConfig.windDir = dir;
    ndConfig.windSpd = spd;
}

export function setVOR1(id, freq, crs, dme) {
    ndConfig.vor1 = { id, freq, crs, dme, active: true };
}

export function setVOR2(id, freq, crs, dme) {
    ndConfig.vor2 = { id, freq, crs, dme, active: true };
}

export function setILS(id, freq, crs) {
    ndConfig.ils = { id, freq, crs, active: true };
}

export function setNextWaypoint(id, dist, eta) {
    ndConfig.nextWpt = id;
    ndConfig.distNext = dist;
    ndConfig.etaNext = eta;
}

export function toggleNDOption(option) {
    if (option in ndConfig && typeof ndConfig[option] === 'boolean') {
        ndConfig[option] = !ndConfig[option];
    }
}

