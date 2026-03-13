/**
 * utils.js - Utility Functions
 * Math helpers, coordinate conversions, and general utilities
 */

import { ORIGIN } from './constants.js';

/**
 * Convert latitude/longitude to meters relative to origin
 * @param {number} lat - Latitude in degrees
 * @param {number} lon - Longitude in degrees
 * @returns {{x: number, z: number}} Position in meters
 */
export function latLonToMeters(lat, lon) {
    const metersPerLat = 111320;
    const metersPerLon = 111320 * Math.cos(ORIGIN.lat * (Math.PI / 180));
    const z = -(lat - ORIGIN.lat) * metersPerLat;
    const x = (lon - ORIGIN.lon) * metersPerLon;
    return { x, z };
}

/**
 * Calculate distance between two coordinates in meters
 * @param {number} lat1 - First latitude
 * @param {number} lon1 - First longitude
 * @param {number} lat2 - Second latitude
 * @param {number} lon2 - Second longitude
 * @returns {number} Distance in meters
 */
export function calculateDistance(lat1, lon1, lat2, lon2) {
    const metersPerLat = 111320;
    const metersPerLon = 111320 * Math.cos(lat1 * (Math.PI / 180));
    const dLat = (lat2 - lat1) * metersPerLat;
    const dLon = (lon2 - lon1) * metersPerLon;
    return Math.sqrt(dLat * dLat + dLon * dLon);
}

/**
 * Interpolate between two hex colors
 * @param {number} color1 - Start color (hex)
 * @param {number} color2 - End color (hex)
 * @param {number} t - Interpolation factor (0-1)
 * @returns {number} Interpolated color (hex)
 */
export function lerpColor(color1, color2, t) {
    const r1 = (color1 >> 16) & 0xff;
    const g1 = (color1 >> 8) & 0xff;
    const b1 = color1 & 0xff;
    const r2 = (color2 >> 16) & 0xff;
    const g2 = (color2 >> 8) & 0xff;
    const b2 = color2 & 0xff;
    const r = Math.round(r1 + (r2 - r1) * t);
    const g = Math.round(g1 + (g2 - g1) * t);
    const b = Math.round(b1 + (b2 - b1) * t);
    return (r << 16) | (g << 8) | b;
}

/**
 * Get terrain height color based on elevation
 * @param {number} height - Elevation in meters
 * @returns {{r: number, g: number, b: number}} RGB color (0-1 range)
 */
export function getHeightColor(height) {
    if (height <= -100) return { r: 0, g: 0, b: 0 }; // voids / sea

    // Color palette (normalized 0..1)
    const G = [0.0431372549, 0.4, 0.137254902]; // green (#0B6623)
    const Y = [0.902, 0.7647058824, 0.3529411765]; // warmer yellow/sand (#E6C35A)
    const O = [0.902, 0.494, 0.133]; // orange
    const R = [0.906, 0.298, 0.235]; // red
    const P = [0.608, 0.349, 0.713]; // purple
    const B = [0.204, 0.596, 0.858]; // blue

    function lerp(a, b, t) { return a + (b - a) * t; }
    function lerpColorArr(c1, c2, t) {
        return { r: lerp(c1[0], c2[0], t), g: lerp(c1[1], c2[1], t), b: lerp(c1[2], c2[2], t) };
    }

    if (height <= 700) return { r: G[0], g: G[1], b: G[2] };
    if (height <= 1400) {
        const t = (height - 700) / (1400 - 700);
        return lerpColorArr(G, Y, t);
    }
    if (height <= 2100) {
        const t = (height - 1400) / (2100 - 1400);
        return lerpColorArr(Y, O, t);
    }
    if (height <= 2800) {
        const t = (height - 2100) / (2800 - 2100);
        return lerpColorArr(O, R, t);
    }
    if (height <= 3500) {
        const t = (height - 2800) / (3500 - 2800);
        return lerpColorArr(R, P, t);
    }
    if (height <= 4000) {
        const t = (height - 3500) / (4000 - 3500);
        return lerpColorArr(P, B, t);
    }
    return { r: B[0], g: B[1], b: B[2] };
}

/**
 * Calculate CRC16 checksum for serial packets
 * @param {Uint8Array} buffer - Data buffer
 * @param {number} start - Start index
 * @param {number} length - Length of data
 * @returns {number} CRC16 value
 */
export function calculateCRC16(buffer, start, length) {
    let crc = 0xFFFF;
    const end = start + length;
    for (let i = start; i < end; i++) {
        crc ^= (buffer[i] << 8);
        for (let j = 0; j < 8; j++) {
            if ((crc & 0x8000) !== 0) {
                crc = ((crc << 1) ^ 0x1021);
            } else {
                crc = (crc << 1);
            }
        }
        crc = crc & 0xFFFF;
    }
    return crc;
}

/**
 * Convert lat/lon to tile coordinates
 * @param {number} lat - Latitude
 * @param {number} lon - Longitude
 * @param {number} zoom - Zoom level
 * @returns {{x: number, y: number, z: number}} Tile coordinates
 */
export function latLonToTile(lat, lon, zoom) {
    const n = Math.pow(2, zoom);
    const x = Math.floor((lon + 180) / 360 * n);
    const latRad = lat * Math.PI / 180;
    const y = Math.floor((1 - Math.asinh(Math.tan(latRad)) / Math.PI) / 2 * n);
    return { x, y, z: zoom };
}

/**
 * Convert tile coordinates to lat/lon bounds
 * @param {number} x - Tile X
 * @param {number} y - Tile Y
 * @param {number} z - Zoom level
 * @returns {{latTop: number, latBottom: number, lonLeft: number, lonRight: number}}
 */
export function tileToBounds(x, y, z) {
    const n = Math.pow(2, z);
    const lonLeft = x / n * 360 - 180;
    const lonRight = (x + 1) / n * 360 - 180;
    const latTopRad = Math.atan(Math.sinh(Math.PI * (1 - 2 * y / n)));
    const latBottomRad = Math.atan(Math.sinh(Math.PI * (1 - 2 * (y + 1) / n)));
    return {
        latTop: latTopRad * 180 / Math.PI,
        latBottom: latBottomRad * 180 / Math.PI,
        lonLeft,
        lonRight
    };
}
