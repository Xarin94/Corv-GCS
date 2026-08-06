/**
 * MissionLibrary.js - Recall, save and overwrite missions stored on disk
 *
 * Files live under the installation data root (see mission-store.js in the main
 * process); this module is only the renderer half: a modal listing what is on disk
 * and the save/overwrite flow.
 *
 * The "current mission" is remembered after a load or a save so that SAVE overwrites
 * the same file instead of piling up copies — SAVE AS is the explicit way to branch.
 */

import { STATE } from '../core/state.js';

let initialized = false;
let onMissionLoaded = null;   // callback into TabController
let currentId = null;         // id of the saved mission currently open
let currentName = null;
let cachedIndex = null;

export function getCurrentMissionName() {
    return currentName;
}

export function getCurrentMissionId() {
    return currentId;
}

/**
 * In-page text prompt.
 * Chromium in Electron does not implement window.prompt() at all, and native dialogs
 * steal focus permanently in a frameless window — which is why index.html already
 * replaces alert() and confirm() with in-page equivalents. This is the missing third.
 * @returns {Promise<string|null>} trimmed text, or null if cancelled/empty
 */
function promptText(message, defaultValue = '') {
    return new Promise(resolve => {
        const overlay = document.createElement('div');
        overlay.className = 'gcs-prompt-overlay';
        overlay.innerHTML = `
            <div class="gcs-prompt-box">
                <div class="gcs-prompt-msg"></div>
                <input type="text" class="gcs-prompt-input" spellcheck="false">
                <div class="gcs-prompt-btns">
                    <button class="gcs-prompt-btn" data-act="cancel">Cancel</button>
                    <button class="gcs-prompt-btn is-accent" data-act="ok">OK</button>
                </div>
            </div>`;
        overlay.querySelector('.gcs-prompt-msg').textContent = message;
        const input = overlay.querySelector('.gcs-prompt-input');
        input.value = defaultValue;
        document.body.appendChild(overlay);

        const close = (value) => {
            document.removeEventListener('keydown', onKey, true);
            overlay.remove();
            resolve(value);
        };
        const accept = () => {
            const v = input.value.trim();
            close(v || null);
        };
        const onKey = (e) => {
            if (e.key === 'Enter') { e.preventDefault(); e.stopPropagation(); accept(); }
            if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); close(null); }
        };
        document.addEventListener('keydown', onKey, true);
        overlay.addEventListener('click', (e) => {
            if (e.target === overlay) close(null);
            const act = e.target.dataset?.act;
            if (act === 'ok') accept();
            if (act === 'cancel') close(null);
        });
        input.focus();
        input.select();
    });
}

function escapeHtml(s) {
    return String(s ?? '').replace(/[&<>"']/g, c => (
        { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]
    ));
}

function formatDistance(m) {
    if (!m) return '—';
    return m >= 1000 ? `${(m / 1000).toFixed(1)} km` : `${Math.round(m)} m`;
}

function formatDate(iso) {
    if (!iso) return '';
    const d = new Date(iso);
    if (Number.isNaN(d.getTime())) return '';
    return d.toLocaleString(undefined, {
        year: 'numeric', month: '2-digit', day: '2-digit',
        hour: '2-digit', minute: '2-digit'
    });
}

function formatSize(bytes) {
    if (bytes >= 1048576) return `${(bytes / 1048576).toFixed(1)} MB`;
    if (bytes >= 1024) return `${Math.round(bytes / 1024)} kB`;
    return `${bytes} B`;
}

/**
 * @param {Function} loadedCallback - invoked after STATE.missionItems is replaced
 */
export function initMissionLibrary(loadedCallback) {
    onMissionLoaded = loadedCallback;
    if (initialized) return;
    initialized = true;

    if (!window.missionStore) {
        console.warn('[MissionLibrary] missionStore bridge missing — library disabled');
        return;
    }

    const closeBtn = document.getElementById('mission-lib-close');
    if (closeBtn) closeBtn.addEventListener('click', closeMissionLibrary);

    const saveAsBtn = document.getElementById('mission-lib-save-as');
    if (saveAsBtn) saveAsBtn.addEventListener('click', () => saveCurrentMission({ forceNew: true }).then(refresh));

    const revealBtn = document.getElementById('mission-lib-reveal');
    if (revealBtn) revealBtn.addEventListener('click', () => window.missionStore.reveal('root'));

    const list = document.getElementById('mission-lib-list');
    if (list) list.addEventListener('click', onListClick);

    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && isOpen()) closeMissionLibrary();
    });
}

function isOpen() {
    return document.getElementById('mission-library-page')?.classList.contains('open');
}

export async function openMissionLibrary() {
    const page = document.getElementById('mission-library-page');
    if (!page) return;
    page.classList.add('open');
    await refresh();
}

export function closeMissionLibrary() {
    document.getElementById('mission-library-page')?.classList.remove('open');
}

async function refresh() {
    if (!window.missionStore) return;
    try {
        cachedIndex = await window.missionStore.list();
    } catch (e) {
        cachedIndex = null;
        console.error('[MissionLibrary] list failed:', e);
    }
    renderList();
    renderRoot();
    renderLogs();
}

function renderRoot() {
    const el = document.getElementById('mission-lib-root');
    if (!el || !cachedIndex) return;
    el.textContent = cachedIndex.root;
    el.title = cachedIndex.portable
        ? 'Stored inside the installation folder (portable)'
        : 'Installation folder is not writable — stored in the user data folder instead';
    el.classList.toggle('is-fallback', !cachedIndex.portable);
}

function renderList() {
    const list = document.getElementById('mission-lib-list');
    if (!list) return;
    const missions = cachedIndex?.missions || [];

    if (!missions.length) {
        list.innerHTML = '<div class="mission-lib-empty">No saved missions yet.<br>Build a plan and press SAVE.</div>';
        return;
    }

    list.innerHTML = missions.map(m => `
        <div class="mission-lib-row${m.id === currentId ? ' is-current' : ''}" data-id="${escapeHtml(m.id)}">
            <div class="mission-lib-main">
                <span class="mission-lib-name">${escapeHtml(m.name)}</span>
                <span class="mission-lib-meta">${m.waypoints} WP · ${formatDistance(m.distanceM)} · ${escapeHtml(formatDate(m.modified))}</span>
                ${m.notes ? `<span class="mission-lib-notes">${escapeHtml(m.notes)}</span>` : ''}
            </div>
            <div class="mission-lib-actions">
                <button class="gcs-btn-sm" data-act="load" title="Replace the current plan with this one">LOAD</button>
                <button class="gcs-btn-sm" data-act="overwrite" title="Overwrite this mission with the current plan">OVERWRITE</button>
                <button class="gcs-btn-sm" data-act="rename" title="Rename">REN</button>
                <button class="gcs-btn-sm mission-lib-del" data-act="delete" title="Delete">DEL</button>
            </div>
        </div>`).join('');
}

function renderLogs() {
    const el = document.getElementById('mission-lib-logs');
    if (!el) return;
    const logs = cachedIndex?.logs || [];
    if (!logs.length) {
        el.innerHTML = '<div class="mission-lib-empty">No flight logs recorded yet.</div>';
        return;
    }
    el.innerHTML = logs.slice(0, 50).map(l => `
        <div class="mission-lib-logrow">
            <span class="mission-lib-name">${escapeHtml(l.name)}</span>
            <span class="mission-lib-meta">${formatSize(l.size)} · ${escapeHtml(formatDate(l.modified))}</span>
        </div>`).join('')
        + (logs.length > 50 ? `<div class="mission-lib-empty">… ${logs.length - 50} more in the logs folder</div>` : '');
}

async function onListClick(e) {
    const btn = e.target.closest('button[data-act]');
    if (!btn) return;
    const row = btn.closest('.mission-lib-row');
    const id = row?.dataset.id;
    if (!id) return;
    const entry = (cachedIndex?.missions || []).find(m => m.id === id);

    try {
        switch (btn.dataset.act) {
            case 'load':
                await loadMission(id);
                break;

            case 'overwrite': {
                if (!STATE.missionItems.length) {
                    alert('The current plan is empty — nothing to write over it with.');
                    return;
                }
                if (!await confirm(`Overwrite "${entry?.name || id}" with the current plan (${STATE.missionItems.length} WP)?`)) return;
                await window.missionStore.save({
                    id,
                    name: entry?.name || id,
                    notes: entry?.notes || '',
                    items: serializeItems(),
                    vehicleType: STATE.vehicleType,
                });
                currentId = id;
                currentName = entry?.name || id;
                await refresh();
                break;
            }

            case 'rename': {
                const name = await promptText('New mission name:', entry?.name || id);
                if (!name || name === entry?.name) return;
                await window.missionStore.rename(id, name);
                if (currentId === id) currentName = name;
                await refresh();
                break;
            }

            case 'delete':
                if (!await confirm(`Delete "${entry?.name || id}" permanently?`)) return;
                await window.missionStore.remove(id);
                if (currentId === id) { currentId = null; currentName = null; }
                await refresh();
                break;
        }
    } catch (err) {
        alert('Mission library error: ' + err.message);
    }
}

/** Strip runtime-only fields; seq is recomputed on load anyway. */
function serializeItems() {
    return STATE.missionItems.map(it => ({
        seq: it.seq,
        command: it.command,
        lat: it.lat,
        lng: it.lng,
        alt: it.alt,
        frame: it.frame,
        param1: it.param1 ?? 0,
        param2: it.param2 ?? 0,
        param3: it.param3 ?? 0,
        param4: it.param4 ?? 0,
        ...(it.isHome ? { isHome: true } : {}),
    }));
}

async function loadMission(id) {
    if (STATE.missionItems.length &&
        !await confirm('Replace the current plan? Unsaved changes will be lost.')) {
        return;
    }
    const data = await window.missionStore.load(id);
    STATE.missionItems.length = 0;
    data.items.forEach((it, i) => {
        STATE.missionItems.push({ ...it, seq: i });
    });
    currentId = id;
    currentName = data.name || id;
    closeMissionLibrary();
    if (onMissionLoaded) onMissionLoaded();
}

/**
 * Save the current plan. Overwrites the mission it was loaded from unless
 * `forceNew` is set or nothing is currently open.
 * @returns {Promise<object|null>} the saved entry, or null if the operator cancelled
 */
export async function saveCurrentMission(opts = {}) {
    if (!window.missionStore) throw new Error('Mission store not available');
    if (!STATE.missionItems.length) {
        alert('No waypoints to save.');
        return null;
    }

    let id = opts.forceNew ? null : currentId;
    let name = currentName;

    if (!id) {
        const suggestion = name || `Mission ${new Date().toISOString().slice(0, 16).replace('T', ' ')}`;
        name = await promptText('Mission name:', suggestion);
        if (!name) return null;
    }

    const saved = await window.missionStore.save({
        id,
        name,
        items: serializeItems(),
        vehicleType: STATE.vehicleType,
    });

    currentId = saved.id;
    currentName = saved.name;
    if (isOpen()) await refresh();
    return saved;
}
