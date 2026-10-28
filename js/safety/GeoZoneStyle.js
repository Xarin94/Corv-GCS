/**
 * GeoZoneStyle.js - Colours of the UAS geographical zones, one palette for
 * the 2D maps (GeoZoneLayer) and the 3D terrain (GeoZoneTerrain)
 *
 *   no-fly (prohibited, or authorisation needed) from the surface:
 *                       red with black bands
 *   conditions apply from the surface (CONDITIONAL, part time, by NOTAM):
 *                       violet with black bands
 *   a ceiling (the restricted volume starts above the ground): coloured by
 *   the height that may be flown, red-orange at 15 m to yellow-green at
 *   120 m; zones whose floor is CEILING_MAX_M or more above the ground are
 *   not drawn, the general limit (120 m / 400 ft) is lower.
 *
 * No DOM, no THREE: also loaded by the render architecture test.
 */

export const PROHIBITED_RGB = [232, 35, 29];
export const CONDITIONAL_RGB = [176, 96, 255];
export const CEILING_MAX_M = 150;
// Height that may be flown (m AGL) → colour, interpolated between stops
export const CEILING_STOPS = [
    [0, [255, 64, 24]],
    [15, [255, 84, 26]],
    [30, [255, 140, 26]],
    [60, [255, 196, 26]],
    [90, [226, 232, 30]],
    [120, [140, 222, 60]],
];

export function ceilingRgb(m) {
    const s = CEILING_STOPS;
    if (m <= s[0][0]) return s[0][1];
    for (let i = 1; i < s.length; i++) {
        if (m <= s[i][0]) {
            const t = (m - s[i - 1][0]) / (s[i][0] - s[i - 1][0]);
            return s[i - 1][1].map((c, k) => Math.round(c + (s[i][1][k] - c) * t));
        }
    }
    return s[s.length - 1][1];
}

export function rgbCss(rgb, alpha = 1) {
    return alpha >= 1 ? `rgb(${rgb.join(',')})` : `rgba(${rgb.join(',')},${alpha})`;
}

const glslVec = rgb => `vec3(${rgb.map(c => (c / 255).toFixed(4)).join(', ')})`;

/** GLSL: the same ramp, geoZoneCeilingColor(m), and the two banded colours. */
export const GEOZONE_STYLE_GLSL = `
const vec3 GZ_PROHIBITED = ${glslVec(PROHIBITED_RGB)};
const vec3 GZ_CONDITIONAL = ${glslVec(CONDITIONAL_RGB)};
const float GZ_CEILING_MAX = ${CEILING_MAX_M.toFixed(1)};
vec3 geoZoneCeilingColor(float m) {
${CEILING_STOPS.slice(1).map((stop, i) => {
    const [m0, c0] = CEILING_STOPS[i], [m1, c1] = stop;
    return `    if (m <= ${m1.toFixed(1)}) return mix(${glslVec(c0)}, ${glslVec(c1)}, clamp((m - ${m0.toFixed(1)}) / ${(m1 - m0).toFixed(1)}, 0.0, 1.0));`;
}).join('\n')}
    return ${glslVec(CEILING_STOPS[CEILING_STOPS.length - 1][1])};
}
`;
