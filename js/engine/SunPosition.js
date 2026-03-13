/**
 * SunPosition.js - Solar Position Calculator
 * Calculates sun position for realistic lighting and hillshading
 */

/**
 * Calculate sun position based on date/time and location
 * @param {Date} date - Date/time for calculation
 * @param {number} lat - Latitude in degrees
 * @param {number} lon - Longitude in degrees
 * @returns {Object} Sun position {azimuth, elevation} in radians
 */
export function calculateSunPosition(date, lat, lon) {
    const RAD = Math.PI / 180;
    
    // Calculate day of year
    const start = new Date(date.getFullYear(), 0, 0);
    const diff = date - start;
    const oneDay = 1000 * 60 * 60 * 24;
    const dayOfYear = Math.floor(diff / oneDay);
    
    // Calculate solar declination
    const declination = -23.45 * Math.cos(RAD * (360 / 365) * (dayOfYear + 10));
    
    // Calculate hour angle
    const hour = date.getHours() + date.getMinutes() / 60;
    const solarNoon = 12 - lon / 15; // Approximate
    const hourAngle = 15 * (hour - solarNoon);
    
    // Calculate elevation
    const latRad = lat * RAD;
    const decRad = declination * RAD;
    const hourRad = hourAngle * RAD;
    
    const sinElevation = Math.sin(latRad) * Math.sin(decRad) + 
                         Math.cos(latRad) * Math.cos(decRad) * Math.cos(hourRad);
    const elevation = Math.asin(sinElevation);
    
    // Calculate azimuth
    const cosAzimuth = (Math.sin(decRad) - Math.sin(latRad) * sinElevation) / 
                       (Math.cos(latRad) * Math.cos(elevation));
    let azimuth = Math.acos(Math.max(-1, Math.min(1, cosAzimuth)));
    
    if (hourAngle > 0) {
        azimuth = 2 * Math.PI - azimuth;
    }
    
    return {
        azimuth: azimuth,
        elevation: elevation
    };
}

/**
 * Get light direction vector from sun position
 * @param {Object} sunPos - Sun position {azimuth, elevation}
 * @returns {Object} Light direction {x, y, z}
 */
export function getSunLightDirection(sunPos) {
    const x = Math.cos(sunPos.elevation) * Math.sin(sunPos.azimuth);
    const y = Math.sin(sunPos.elevation);
    const z = Math.cos(sunPos.elevation) * Math.cos(sunPos.azimuth);
    return { x, y, z };
}

/**
 * Calculate hillshade value for terrain
 * @param {Object} sunDir - Sun direction vector
 * @param {Object} normal - Surface normal vector
 * @returns {number} Hillshade value 0-1
 */
export function calculateHillshade(sunDir, normal) {
    // Normalize vectors
    const normLen = Math.sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    const nx = normal.x / normLen;
    const ny = normal.y / normLen;
    const nz = normal.z / normLen;
    
    // Dot product for illumination
    const dot = nx * sunDir.x + ny * sunDir.y + nz * sunDir.z;
    
    // Clamp and apply ambient
    const ambient = 0.3;
    const diffuse = 0.7;
    return Math.max(0, Math.min(1, ambient + diffuse * Math.max(0, dot)));
}

/**
 * Apply hillshade to a color
 * @param {Object} color - RGB color {r, g, b} 0-255
 * @param {number} shade - Shade value 0-1
 * @returns {Object} Shaded color {r, g, b}
 */
export function applyHillshade(color, shade) {
    return {
        r: Math.round(color.r * shade),
        g: Math.round(color.g * shade),
        b: Math.round(color.b * shade)
    };
}
