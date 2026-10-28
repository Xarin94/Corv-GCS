/**
 * GeoZonePanel.js - What the UAS geographical zones say, on screen
 *
 * - Zone annunciators, beside the altitude tape on its inboard side, drawn
 *   like the health annunciators (AnnunciatorPanel.js): red flashing NO-FLY
 *   in a zone restricted from the surface, MAX n m above a zone's ceiling or
 *   above the region's general limit (120 m / 400 ft: ALT LIMIT); amber ZONE
 *   where conditions apply and MAX n m while under a zone's ceiling. A click
 *   silences one until the next connection, as on the other strip.
 * - The arming alarm: armed inside a zone, a banner under the mode banner
 *   for ARM_ALERT_MS (red in a no-fly zone, amber in a zone with conditions
 *   or a ceiling; a click closes it), and the same line in the HUD messages.
 * - The RESTRICTED ZONES checkboxes (side panel GCS OPTIONS, SYS CONFIG → UAS
 *   GEO ZONES) kept in step.
 * - SYS CONFIG → UAS GEO ZONES: zones in reach, the sources asked and their
 *   tiles, imported ED-269 / ED-318 files (import, remove), refresh.
 *
 * The figures come from safety/GeoZones.js (geoZoneStatus); the TERR ALT cell
 * flashes by itself (UIController) when a height limit is broken.
 */

import { geoZoneStatus, geoZonesEnabled, getGeoZoneData, onGeoZonesChange, refreshGeoZones } from '../safety/GeoZones.js';
import { geoZoneSourceName } from '../maps/GeoZoneLayer.js';
import { pushHudMessage } from '../hud/HUDRenderer.js';

const ARM_ALERT_MS = 10000;

let col = null, armEl = null;
let lastKey = '';
let armShownAt = null, armHideTimer = null;
const silenced = new Set();

function esc(s) {
    return String(s).replace(/[&<>"]/g, ch => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[ch]));
}

function alertTitle(a) {
    const z = a.zone;
    const lines = [];
    if (z) {
        lines.push(z.name);
        if (a.id === 'noFly') lines.push('No-fly or authorisation required from the surface');
        else if (a.id === 'cond') lines.push('Conditions apply in this zone');
        else lines.push(`Height that may be flown here: ${Math.round(a.metres)} m above the ground`);
        lines.push(`Source: ${geoZoneSourceName(z.src)}`);
        if (z.info) lines.push(z.info.length > 200 ? `${z.info.slice(0, 199)}…` : z.info);
    } else if (a.rule) {
        lines.push(a.rule);
    }
    lines.push('Click to silence until the next connection');
    return lines.join('\n');
}

function buildItem(a) {
    const item = document.createElement('div');
    item.className = `annun annun-${a.level}`;
    item.dataset.id = a.id;
    item.title = alertTitle(a);
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    const use = document.createElementNS('http://www.w3.org/2000/svg', 'use');
    use.setAttribute('href', `#an-${a.icon}`);
    svg.appendChild(use);
    const lbl = document.createElement('span');
    lbl.className = 'annun-lbl';
    lbl.textContent = a.label;
    const val = document.createElement('span');
    val.className = 'annun-val';
    val.textContent = a.value || '';
    item.append(svg, lbl, val);
    return item;
}

function syncChecks() {
    for (const id of ['chk-geozones', 'syscfg-geozones-enable']) {
        const el = document.getElementById(id);
        if (el) el.checked = geoZonesEnabled();
    }
}

function ago(t) {
    if (!t) return '';
    const min = Math.round((Date.now() - t) / 60000);
    return min < 1 ? 'just now' : min < 120 ? `${min} min ago` : `${Math.round(min / 60)} h ago`;
}

function renderSettings() {
    const d = getGeoZoneData();
    const zonesEl = document.getElementById('gz-st-zones');
    const srcEl = document.getElementById('gz-st-sources');
    const impEl = document.getElementById('gz-st-imports');
    if (!zonesEl || !srcEl || !impEl) return;
    if (!d.enabled) zonesEl.textContent = 'Off';
    else if (!d.centres) zonesEl.textContent = 'No vehicle position and no mission to look around';
    else zonesEl.textContent = `${d.zones.length} zones within ${d.radiusKm} km of ${d.centres} point${d.centres > 1 ? 's' : ''}${d.pending ? ` · fetching ${d.pending}…` : ''}`;
    srcEl.innerHTML = !d.enabled || !d.sources.length
        ? (d.enabled && d.centres ? 'None in reach: no online source here, import the national ED-269 file' : '--')
        : d.sources.map(s => `${esc(s.name)} (${s.country}) ${s.ready}/${s.tiles}${s.pending ? ' …' : ''}`
            + (s.error ? ` <span class="gz-err">${esc(s.error)}</span>` : '')).join('<br>');
    impEl.innerHTML = d.imports && d.imports.length
        ? d.imports.map(f => `${esc(f.name)} · ${f.count} zones · ${ago(f.at)} `
            + `<button class="gz-remove" data-file="${esc(f.file)}" title="Remove this file's zones">✕</button>`).join('<br>')
        : 'none';
}

function showArmAlert(a) {
    pushHudMessage(`${a.title}: ${a.detail}`, a.level === 'warn' ? 'error' : 'warning', { log: false });
    if (!armEl) return;
    armEl.className = `zone-arm-alert zone-arm-${a.level}`;
    armEl.querySelector('.zaa-title').textContent = a.title;
    armEl.querySelector('.zaa-detail').textContent = `${a.detail} · ${geoZoneSourceName(a.zone.src)}`;
    clearTimeout(armHideTimer);
    armHideTimer = setTimeout(() => armEl.classList.add('hidden'), ARM_ALERT_MS);
}

export function initGeoZonePanel() {
    col = document.getElementById('zone-annun');
    armEl = document.getElementById('zone-arm-alert');
    armEl?.addEventListener('click', () => { clearTimeout(armHideTimer); armEl.classList.add('hidden'); });
    col?.addEventListener('click', (e) => {
        const item = e.target.closest('.annun');
        if (!item) return;
        console.log(`[geozones] silenced by the operator: ${item.dataset.id}`);
        silenced.add(item.dataset.id);
        lastKey = '';
        updateGeoZonePanel();
    });
    window.addEventListener('vehicleConnected', () => { silenced.clear(); lastKey = ''; });

    document.getElementById('gz-import')?.addEventListener('click', async () => {
        const r = await window.geoZones?.importFile();
        if (!r) return;
        if (r.error) alert(`Import failed: ${r.error}`);
        else console.log(`[geozones] imported ${r.name}: ${r.count} zones`);
        refreshGeoZones();
    });
    document.getElementById('gz-refresh')?.addEventListener('click', async () => {
        await window.geoZones?.clear();
        refreshGeoZones();
    });
    document.getElementById('gz-st-imports')?.addEventListener('click', async (e) => {
        const b = e.target.closest('.gz-remove');
        if (!b) return;
        await window.geoZones?.removeImport(b.dataset.file);
        refreshGeoZones();
    });

    syncChecks();
    renderSettings();
    onGeoZonesChange(() => { syncChecks(); renderSettings(); });
}

/** 10 Hz from the main loop. */
export function updateGeoZonePanel() {
    const arm = geoZoneStatus.armAlert;
    if (arm && arm.at !== armShownAt) {
        armShownAt = arm.at;
        showArmAlert(arm);
    }
    if (!col) return;
    const alerts = geoZoneStatus.alerts.filter(a => !silenced.has(a.id));
    const key = alerts.map(a => `${a.id}:${a.label}:${a.value}:${a.zone?.id || ''}`).join('|');
    if (key === lastKey) return;
    if (alerts.length || lastKey) {
        const was = new Set(lastKey ? lastKey.split('|').map(k => k.split(':')[0]) : []);
        for (const a of alerts) {
            if (!was.has(a.id)) (a.level === 'warn' ? console.warn : console.log)(`[geozones] ${a.level === 'warn' ? 'WARNING' : 'caution'}: ${a.label} ${a.value || ''}`);
        }
    }
    lastKey = key;
    col.replaceChildren(...alerts.map(buildItem));
}
