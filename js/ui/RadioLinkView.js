/**
 * RadioLinkView.js - Radio link drawing for the Flight Plan page
 *
 * Everything visual about the ground ↔ aircraft link, fed by RadioLink.js:
 *   coverage  – the polar raster rendered to a georeferenced image overlay
 *               (light green / orange / red, transparent where there is no link)
 *   halo      – a wide translucent band under the calculated route, coloured
 *               by the link class of every stretch
 *   LOS line  – operator → hovered route point while the profile is hovered
 *   profile   – the link band under the elevation profile header and the
 *               "LINK PROFILE" inset: the terrain cut between the operator and
 *               a route point with the line of sight and the first Fresnel zone
 *
 * The Leaflet panes sit between the tiles and the route drawing so the overlay
 * never hides a marker or steals a click.
 */

import { LINK, LINK_STYLE, coverageClassAt, fresnelRadius } from '../mission/RadioLink.js';
import { localFrame } from '../mission/RouteModel.js';

const D2R = Math.PI / 180;
const R_MERC = 6378137;
const IMG_PX = 720;

// RGBA per class for the coverage image — NONE and UNKNOWN stay transparent
const PIXEL = {
    [LINK.GOOD]:     [96, 255, 128, 66],
    [LINK.DEGRADED]: [255, 170, 0, 78],
    [LINK.MARGINAL]: [255, 60, 60, 84],
    [LINK.NONE]:     [0, 0, 0, 0],
    [LINK.UNKNOWN]:  [0, 0, 0, 0],
};

export function createRadioLinkView(map) {
    const covPane = map.createPane('fpCoverage');
    covPane.style.zIndex = 350;
    covPane.style.pointerEvents = 'none';
    const haloPane = map.createPane('fpLinkHalo');
    haloPane.style.zIndex = 390;
    haloPane.style.pointerEvents = 'none';

    const halo = L.layerGroup().addTo(map);
    const los = L.layerGroup().addTo(map);
    let overlay = null;
    let visible = false;

    return {
        setVisible(v) {
            visible = !!v;
            if (overlay) { if (visible) overlay.addTo(map); else map.removeLayer(overlay); }
            if (!visible) { halo.clearLayers(); los.clearLayers(); }
        },
        /** Replace the coverage image with a freshly rendered raster (null clears it). */
        setCoverage(cov) {
            if (overlay) { map.removeLayer(overlay); overlay = null; }
            if (!cov) return;
            const { url, bounds } = renderCoverage(cov);
            overlay = L.imageOverlay(url, bounds, { pane: 'fpCoverage', interactive: false, className: 'fp-coverage-img' });
            if (visible) overlay.addTo(map);
        },
        /** Colour band under the route from analyzeRoute() runs. */
        setHalo(analysis) {
            halo.clearLayers();
            if (!visible || !analysis) return;
            for (const run of analysis.runs) {
                if (run.points.length < 2) continue;
                const st = LINK_STYLE[run.clazz];
                const none = run.clazz === LINK.NONE || run.clazz === LINK.UNKNOWN;
                L.polyline(run.points, {
                    pane: 'fpLinkHalo', interactive: false, color: st.color, weight: none ? 5 : 10,
                    opacity: none ? 0.7 : 0.45, lineCap: 'butt', dashArray: none ? '2 6' : null,
                }).addTo(halo);
            }
        },
        /** Dashed line of sight from the operator to a point, in the class colour. */
        setLos(gs, pt, clazz) {
            los.clearLayers();
            if (!visible || !gs || !pt) return;
            L.polyline([[gs.lat, gs.lng], [pt.lat, pt.lng]], {
                pane: 'fpLinkHalo', interactive: false, color: LINK_STYLE[clazz]?.color || '#fff', weight: 1.5, opacity: 0.9, dashArray: '5 5',
            }).addTo(los);
        },
        clearLos() { los.clearLayers(); },
        clear() { halo.clearLayers(); los.clearLayers(); if (overlay) { map.removeLayer(overlay); overlay = null; } },
    };
}

/**
 * Rasterise the polar coverage into a Web-Mercator image: pixels map
 * linearly onto EPSG:3857, which is exactly how Leaflet stretches an image
 * overlay between two corners, so the picture lands on the terrain it was
 * computed for at every zoom.
 */
function renderCoverage(cov) {
    const lat0 = cov.gs.lat * D2R;
    const mx = R_MERC * cov.gs.lng * D2R;
    const my = R_MERC * Math.log(Math.tan(Math.PI / 4 + lat0 / 2));
    const Rp = cov.range / Math.cos(lat0);          // ground metres → projected metres at this latitude
    const x0 = mx - Rp, yTop = my + Rp;
    const px = (2 * Rp) / IMG_PX;
    const frame = localFrame(cov.gs);

    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = IMG_PX;
    const ctx = canvas.getContext('2d');
    const img = ctx.createImageData(IMG_PX, IMG_PX);
    const data = img.data;
    for (let j = 0; j < IMG_PX; j++) {
        const y = yTop - (j + 0.5) * px;
        const lat = (2 * Math.atan(Math.exp(y / R_MERC)) - Math.PI / 2) / D2R;
        for (let i = 0; i < IMG_PX; i++) {
            const x = x0 + (i + 0.5) * px;
            const lng = (x / R_MERC) / D2R;
            const q = frame.toXY({ lat, lng });
            const c = PIXEL[coverageClassAt(cov, q.x, q.y)];
            if (!c || !c[3]) continue;
            const o = (j * IMG_PX + i) * 4;
            data[o] = c[0]; data[o + 1] = c[1]; data[o + 2] = c[2]; data[o + 3] = c[3];
        }
    }
    ctx.putImageData(img, 0, 0);
    // A touch of blur hides the polar cell edges at the far end of the rays
    const out = document.createElement('canvas');
    out.width = out.height = IMG_PX;
    const octx = out.getContext('2d');
    octx.filter = 'blur(1.2px)';
    octx.drawImage(canvas, 0, 0);

    const unproject = (x, y) => [(2 * Math.atan(Math.exp(y / R_MERC)) - Math.PI / 2) / D2R, (x / R_MERC) / D2R];
    const bounds = L.latLngBounds(unproject(x0, yTop - 2 * Rp), unproject(x0 + 2 * Rp, yTop));
    return { url: out.toDataURL('image/png'), bounds };
}

// ── Elevation profile ─────────────────────────────────────────────────────────

/** Link class band along the route: one rectangle per stretch between analysis samples. */
export function drawLinkBand(ctx, analysis, X, y, h, xMin, xMax) {
    if (!analysis) return;
    const s = analysis.samples;
    for (let i = 1; i < s.length; i++) {
        const a = s[i - 1], b = s[i];
        let x1 = X(a.dist), x2 = X(b.dist);
        if (x2 < xMin || x1 > xMax) continue;
        x1 = Math.max(xMin, x1); x2 = Math.min(xMax, x2);
        const cls = a.clazz;
        if (cls === LINK.NONE) {
            ctx.fillStyle = 'rgba(154, 164, 176, 0.25)';
            ctx.fillRect(x1, y, Math.max(1, x2 - x1), h);
            ctx.fillStyle = '#9aa4b0';
            for (let x = Math.ceil(x1 / 4) * 4; x < x2; x += 4) ctx.fillRect(x, y + h / 2 - 0.5, 1.5, 1);
            continue;
        }
        if (cls === LINK.UNKNOWN) { ctx.fillStyle = 'rgba(74, 85, 96, 0.35)'; ctx.fillRect(x1, y, Math.max(1, x2 - x1), h); continue; }
        ctx.fillStyle = LINK_STYLE[cls].color;
        ctx.globalAlpha = cls === LINK.GOOD ? 0.55 : 0.85;
        ctx.fillRect(x1, y, Math.max(1, x2 - x1 + 0.5), h);
        ctx.globalAlpha = 1;
    }
}

/**
 * The terrain cut from the ground antenna to one route point: ground, earth
 * bulge, the line of sight, the first Fresnel zone (outline) with its 60 %
 * core (filled), the worst knife edge and the numbers. `ev` comes from
 * evaluateLink(..., { profile: true }).
 */
export function drawLinkInset(ctx, rect, ev, opts = {}) {
    const { x: rx, y: ry, w, h } = rect;
    const mono = '"Roboto Mono", monospace';
    ctx.save();
    ctx.beginPath(); ctx.rect(rx, ry, w, h); ctx.clip();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.22)';
    ctx.fillRect(rx, ry, w, h);
    ctx.strokeStyle = 'rgba(0, 210, 255, 0.25)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(rx + 0.5, ry); ctx.lineTo(rx + 0.5, ry + h); ctx.stroke();

    ctx.textAlign = 'left'; ctx.fillStyle = '#00d2ff'; ctx.font = 'bold 10px "Rajdhani", sans-serif';
    ctx.fillText('LINK PROFILE', rx + 10, ry + 15);
    if (opts.title) { ctx.fillStyle = '#889999'; ctx.font = `9px ${mono}`; ctx.fillText(opts.title, rx + 96, ry + 15); }

    const pad = { l: 12, r: 12, t: 24, b: 26 };
    const pw = w - pad.l - pad.r, ph = h - pad.t - pad.b;
    const p = ev?.profile;
    if (!p || !(p.D > 0) || ev.clazz === LINK.UNKNOWN) {
        ctx.fillStyle = '#6e7f8d'; ctx.font = `9.5px ${mono}`; ctx.textAlign = 'center';
        ctx.fillText(ev?.clazz === LINK.UNKNOWN ? 'no elevation data along the path' : 'hover the profile to see the line of sight', rx + w / 2, ry + h / 2 + 3);
        ctx.restore();
        return;
    }
    const { n, step, terr, h0, h1, D, freq } = p;
    const bulge = d => d * (D - d) / (2 * (4 / 3) * 6371000);
    const line = d => h0 + (h1 - h0) * (d / D);
    let lo = Math.min(h0, h1), hi = Math.max(h0, h1);
    for (let j = 0; j <= n; j++) {
        const d = j * step, r = fresnelRadius(d, D - d, freq);
        lo = Math.min(lo, terr[j] + bulge(d), line(d) - r);
        hi = Math.max(hi, terr[j] + bulge(d), line(d) + r);
    }
    const range = Math.max(20, hi - lo);
    lo -= range * 0.08; hi += range * 0.12;
    const X = d => rx + pad.l + (d / D) * pw;
    const Y = a => ry + pad.t + ph - ((a - lo) / (hi - lo)) * ph;
    const st = LINK_STYLE[ev.clazz];

    // First Fresnel zone: outline of F1, filled 60 % core (the clearance that matters)
    const zone = (k, fill, stroke, dash) => {
        ctx.beginPath();
        for (let j = 0; j <= n; j++) { const d = j * step; const y = Y(line(d) + k * fresnelRadius(d, D - d, freq)); j ? ctx.lineTo(X(d), y) : ctx.moveTo(X(d), y); }
        for (let j = n; j >= 0; j--) { const d = j * step; ctx.lineTo(X(d), Y(line(d) - k * fresnelRadius(d, D - d, freq))); }
        ctx.closePath();
        if (fill) { ctx.fillStyle = fill; ctx.fill(); }
        if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]); }
    };
    zone(1, null, 'rgba(0, 210, 255, 0.35)', [3, 3]);
    zone(0.6, 'rgba(0, 210, 255, 0.10)', null);

    // Terrain with the earth bulge folded in — what the wave actually sees
    ctx.beginPath();
    ctx.moveTo(X(0), Y(lo));
    for (let j = 0; j <= n; j++) ctx.lineTo(X(j * step), Y(terr[j] + bulge(j * step)));
    ctx.lineTo(X(D), Y(lo));
    ctx.closePath();
    const g = ctx.createLinearGradient(0, Y(hi), 0, Y(lo));
    g.addColorStop(0, 'rgba(96, 128, 72, 0.7)');
    g.addColorStop(1, 'rgba(40, 56, 32, 0.35)');
    ctx.fillStyle = g; ctx.fill();
    ctx.beginPath();
    for (let j = 0; j <= n; j++) { const y = Y(terr[j] + bulge(j * step)); j ? ctx.lineTo(X(j * step), y) : ctx.moveTo(X(0), y); }
    ctx.strokeStyle = 'rgba(140, 190, 110, 0.8)'; ctx.lineWidth = 1.2; ctx.stroke();

    // Where the ground enters the 60 % zone: the intruding slice of the zone in red
    // (never above the ray — the mountain itself stays terrain-coloured) and the
    // ground line painted red along that stretch
    const cw = Math.max(1.5, pw / n);
    ctx.fillStyle = 'rgba(255, 60, 60, 0.35)';
    for (let j = 1; j < n; j++) {
        const d = j * step;
        const top = line(d) - 0.6 * fresnelRadius(d, D - d, freq);
        const ground = terr[j] + bulge(d);
        if (ground <= top) continue;
        const yTop = Y(Math.min(ground, line(d)));
        ctx.fillRect(X(d) - cw / 2, yTop, cw, Math.max(1, Y(top) - yTop));
    }
    ctx.strokeStyle = 'rgba(255, 70, 70, 0.95)'; ctx.lineWidth = 2;
    let inRun = false;
    ctx.beginPath();
    for (let j = 1; j < n; j++) {
        const d = j * step;
        const intrudes = terr[j] + bulge(d) > line(d) - 0.6 * fresnelRadius(d, D - d, freq);
        const y = Y(terr[j] + bulge(d));
        if (intrudes) { inRun ? ctx.lineTo(X(d), y) : ctx.moveTo(X(d), y); inRun = true; }
        else inRun = false;
    }
    ctx.stroke();

    // Line of sight
    ctx.beginPath(); ctx.moveTo(X(0), Y(h0)); ctx.lineTo(X(D), Y(h1));
    ctx.strokeStyle = st.color; ctx.lineWidth = 1.6; ctx.stroke();

    // Mast and aircraft
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(X(0) + 0.5, Y(terr[0])); ctx.lineTo(X(0) + 0.5, Y(h0)); ctx.stroke();
    ctx.beginPath(); ctx.arc(X(0) + 0.5, Y(h0), 2.5, 0, Math.PI * 2); ctx.fillStyle = '#fff'; ctx.fill();
    ctx.beginPath(); ctx.arc(X(D), Y(h1), 3.5, 0, Math.PI * 2); ctx.fillStyle = st.color; ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1; ctx.stroke();

    // Worst knife edge
    if (ev.worst && ev.nu > -0.78) {
        const wx = X(ev.worst.dist), wy = Y(ev.worst.elev + bulge(ev.worst.dist));
        ctx.beginPath(); ctx.moveTo(wx, wy - 9); ctx.lineTo(wx - 4, wy - 2); ctx.lineTo(wx + 4, wy - 2); ctx.closePath();
        ctx.fillStyle = ev.obstructed ? '#ff3b3b' : '#ffaa00'; ctx.fill();
        ctx.font = `bold 8.5px ${mono}`; ctx.fillStyle = ev.obstructed ? '#ff6060' : '#ffc040'; ctx.textAlign = 'center';
        ctx.fillText(`−${ev.loss.toFixed(1)} dB`, Math.min(rx + w - 26, Math.max(rx + 26, wx)), wy - 12);
    }

    // Axis text
    ctx.fillStyle = '#6e7f8d'; ctx.font = `8.5px ${mono}`;
    ctx.textAlign = 'left'; ctx.fillText('GS', X(0) + 4, ry + pad.t + ph + 10);
    ctx.textAlign = 'right'; ctx.fillText(fmtDist(D), X(D), ry + pad.t + ph + 10);
    ctx.textAlign = 'left'; ctx.fillStyle = '#556';
    ctx.fillText(`${Math.round(lo)} m`, rx + pad.l, Y(lo) - 2);

    // Numbers
    const status = ev.obstructed ? 'NLOS · diffraction' : ev.fresnelClear ? 'LOS · Fresnel clear' : 'LOS · Fresnel intruded';
    ctx.font = `9px ${mono}`; ctx.textAlign = 'left';
    ctx.fillStyle = st.color;
    ctx.fillText(st.short, rx + pad.l, ry + h - 6);
    ctx.fillStyle = '#cfd8dd';
    ctx.fillText(`${ev.rssi.toFixed(0)} dBm · margin ${ev.margin >= 0 ? '+' : ''}${ev.margin.toFixed(0)} dB · ${status}`, rx + pad.l + 62, ry + h - 6);
    ctx.restore();
}

function fmtDist(m) { return m >= 1000 ? `${(m / 1000).toFixed(m >= 10000 ? 0 : 1)} km` : `${Math.round(m)} m`; }
