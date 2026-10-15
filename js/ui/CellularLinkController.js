/**
 * CellularLinkController.js - SETUP → COMMS → CELLULAR LINK
 *
 * Connects to one LTEtelem cellular module through a relay: the operator
 * gives the module ID and its AES-256 key, the relay address is entered once.
 * The relay knows no keys (CRV3 access), so a wrong key is not refused: the
 * module just stays offline. The key fingerprint shown then is the one to
 * compare with e2e_key_fp on the module.
 * The link itself lives in the main process (lte-link.js) and feeds the normal
 * MAVLink pipeline, so everything else in the GCS works as on any other link.
 *
 * Module ID and relay settings persist in localStorage under 'lte-link-config';
 * the relay host defaults to the CORV relay.
 * The key is never written there: with REMEMBER KEY it is stored encrypted by
 * the OS keychain (Electron safeStorage) through window.lteKey.
 */

import { STATE } from '../core/state.js';
import { connect, disconnect } from '../mavlink/ConnectionManager.js';
import { requestAllDataStreams } from '../mavlink/CommandSender.js';
import { setNavDot } from './TabController.js';
import { getCellMapSettings, setCellMapSettings, cellIsLive, cellMapInfo, operatorName, SOURCE_NAMES } from '../maps/CellTowerLayer.js';
import { frequencyText } from '../core/lte-bands.js';

const STORAGE_KEY = 'lte-link-config';
const DEFAULT_RELAY_HOST = '94.177.202.247';   // CORV relay (Aruba VPS)
const $ = (id) => document.getElementById(id);

const STATE_LABELS = {
    idle: 'Not connected',
    connecting: 'Connecting to relay…',
    handshake: 'Authenticating…',
    online: 'ONLINE',
    reconnecting: 'Link lost — reconnecting…',
    error: 'ERROR',
    stopped: 'Not connected'
};

let lastFingerprint = null;
// A new module session means the module restarted, usually with the flight
// controller (the drone was powered up): the autopilot is back on its default
// stream rates, so ask for them again. The first session is covered by the
// request ConnectionManager sends after connecting.
let lastModuleSession = null;

function loadConfig() {
    try { return JSON.parse(localStorage.getItem(STORAGE_KEY)) || {}; } catch (e) { return {}; }
}

function saveConfig(cfg) {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(cfg)); } catch (e) { /* ignore */ }
}

function readForm() {
    return {
        moduleId: $('lte-module-id').value.trim(),
        keyHex: $('lte-key').value.replace(/[\s:]/g, ''),
        host: $('lte-relay-host').value.trim(),
        port: parseInt($('lte-relay-port').value, 10) || 5765,
        certSha256: $('lte-relay-cert').value.trim(),
        rememberKey: $('lte-key-remember').checked
    };
}

function validate(f) {
    if (!f.moduleId || /\s/.test(f.moduleId)) return 'Enter the module ID';
    if (!/^[0-9a-fA-F]{64}$/.test(f.keyHex)) return 'The AES-256 key must be 64 hexadecimal characters';
    if (!f.host) return 'Enter the relay host (RELAY panel)';
    return null;
}

function formatBytes(n) {
    if (n >= 1e6) return `${(n / 1e6).toFixed(1)} MB`;
    if (n >= 1e3) return `${(n / 1e3).toFixed(1)} kB`;
    return `${n} B`;
}

function render(st) {
    const online = st.state === 'online';
    $('lte-st-state').textContent = STATE_LABELS[st.state] || st.state;
    $('lte-st-state').style.color = online ? 'var(--accent-green, #3c3)' :
        (st.state === 'error' || st.state === 'reconnecting') ? 'var(--accent-red, #e44)' : '';

    const receiving = online && st.recordsIn > 0 && st.lastRxAge !== null && st.lastRxAge < 3000;
    $('lte-st-air').textContent = !online ? '--' :
        receiving ? `${st.moduleId} — receiving` :
        st.airOnline ? `${st.moduleId} — online, waiting for data` :
        `${st.moduleId} — offline on the relay (key fingerprint ${st.keyFingerprint}: same as e2e_key_fp on the module?)`;
    $('lte-st-session').textContent = st.vehicleSession || '--';
    if (st.vehicleSession && st.vehicleSession !== lastModuleSession) {
        if (lastModuleSession) setTimeout(() => requestAllDataStreams().catch(() => {}), 1000);
        lastModuleSession = st.vehicleSession;
    }

    if (st.tls) {
        lastFingerprint = st.tls.fingerprint || null;
        $('lte-st-tls').textContent = `${st.tls.version || '?'} ${st.tls.cipher || ''} — ` +
            (st.tls.verified ? 'certificate pinned' : 'certificate NOT verified');
    }
    $('lte-st-records').textContent = `${st.recordsIn} / ${st.recordsOut}`;
    $('lte-st-rejected').textContent =
        `${st.authFail} not authentic, ${st.replay} replayed, ${st.droppedNoSession} unsent (no module session yet)`;
    $('lte-st-traffic').textContent = `↓ ${formatBytes(st.bytesRx)}  ↑ ${formatBytes(st.bytesTx)}`;
    $('lte-st-age').textContent = st.lastRxAge === null ? '--' : `${(st.lastRxAge / 1000).toFixed(1)} s ago`;
    $('lte-st-reconnects').textContent = String(st.reconnects);
    $('lte-st-error').textContent = st.lastError || '--';

    setNavDot('lte-link', online);
}

export async function initCellularLink() {
    if (!$('subtab-lte-link') || !window.mavlink?.connectLTE) return;

    const cfg = loadConfig();
    $('lte-module-id').value = cfg.moduleId || '';
    $('lte-relay-host').value = cfg.host || DEFAULT_RELAY_HOST;
    $('lte-relay-port').value = cfg.port || 5765;
    $('lte-relay-cert').value = cfg.certSha256 || '';
    $('lte-key-remember').checked = !!cfg.rememberKey;

    const keychain = window.lteKey ? await window.lteKey.available().catch(() => false) : false;
    if (!keychain) {
        $('lte-key-remember').checked = false;
        $('lte-key-remember').disabled = true;
        $('lte-key-remember').parentElement.title = 'No OS keychain available: the key must be entered every time';
    } else if (cfg.rememberKey) {
        $('lte-key').value = await window.lteKey.load().catch(() => '') || '';
    }

    $('lte-key-show').addEventListener('click', () => {
        const input = $('lte-key');
        input.type = input.type === 'password' ? 'text' : 'password';
        $('lte-key-show').textContent = input.type === 'password' ? 'SHOW' : 'HIDE';
    });

    $('lte-cert-pin').addEventListener('click', () => {
        if (!lastFingerprint) { alert('Connect once to read the relay certificate, then pin it.'); return; }
        $('lte-relay-cert').value = lastFingerprint;
        saveConfig({ ...loadConfig(), certSha256: lastFingerprint });
    });

    $('lte-connect').addEventListener('click', async () => {
        const f = readForm();
        const problem = validate(f);
        if (problem) { alert(problem); return; }
        lastModuleSession = null;   // a new link: its first session is not a restart

        saveConfig({ moduleId: f.moduleId, host: f.host, port: f.port,
                     certSha256: f.certSha256, rememberKey: f.rememberKey && keychain });
        if (window.lteKey && keychain) {
            if (f.rememberKey) await window.lteKey.save(f.keyHex);
            else await window.lteKey.forget();
        }

        $('lte-connect').disabled = true;
        $('lte-st-state').textContent = STATE_LABELS.connecting;
        try {
            await connect('mavlink-lte', {
                moduleId: f.moduleId, keyHex: f.keyHex,
                host: f.host, port: f.port, certSha256: f.certSha256
            });
        } catch (e) {
            // Electron wraps main-process errors: keep only the message
            alert('Cellular link failed: ' + String(e.message || e).replace(/^Error invoking remote method '[^']+': (Error: )?/, ''));
        } finally {
            $('lte-connect').disabled = false;
        }
    });

    $('lte-disconnect').addEventListener('click', async () => {
        if (STATE.connectionType === 'mavlink-lte') await disconnect();
    });

    window.mavlink.onLteStatus(render);
    initCellMap();
}

// ── LTE cell map (CORV_CELL_INFO + OpenCelliD cache) ─────────────────────────

// Boxes for DOWNLOAD 2024 COPY (south, west, north, east) and the MCC for DOWNLOAD FROM OPENCELLID
const COUNTRIES = {
    'Italy': { box: [35.4, 6.6, 47.1, 18.6], mcc: 222 },
    'Switzerland': { box: [45.8, 5.9, 47.9, 10.5], mcc: 228 },
    'Austria': { box: [46.4, 9.5, 49.0, 17.2], mcc: 232 },
    'Slovenia': { box: [45.4, 13.4, 46.9, 16.6], mcc: 293 },
    'France': { box: [42.3, -4.8, 51.1, 8.2], mcc: 208 },
    'Germany': { box: [47.3, 5.9, 55.1, 15.0], mcc: 262 },
    'Spain': { box: [36.0, -9.3, 43.8, 3.3], mcc: 214 },
};

const ipcMessage = (e) => String(e.message || e).replace(/^Error invoking remote method '[^']+': (Error: )?/, '');

function renderCellDbStatus(st) {
    if (!st) return;
    const b = st.bundle;
    $('lte-celldb-bundle').textContent = b
        ? `${b.cells.toLocaleString()} LTE cells (MCC ${b.mcc.join(', ')}): `
            + b.sources.map((s) => `${SOURCE_NAMES[s.src] || s.name} ${s.date}`).join(', ')
        : 'none';
    $('lte-celldb-files').textContent = st.files && st.files.length
        ? st.files.map((f) => `${f.name}: ${f.cells.toLocaleString()} cells${f.newest ? ` (to ${f.newest})` : ''}`).join(', ')
        : 'none';
    $('lte-celldb-status').textContent = st.blocks
        ? `${st.cells.toLocaleString()} LTE cells in ${st.blocks} areas, ${formatBytes(st.bytes)}${st.date ? `, data of ${st.date}` : ''}`
        : 'none';
    const lk = st.lookups || { found: 0, missed: 0 };
    $('lte-celldb-lookups').textContent = `${lk.found} found, ${lk.missed} not found`;
    if (st.settings) {
        $('lte-celldb-online').checked = st.settings.online;
        $('lte-celldb-token').placeholder = st.settings.hasToken ? 'saved (type a new one to replace it)' : 'free at opencellid.org';
    }
}

function renderCell() {
    const c = STATE.cell;
    if (!c || !cellIsLive()) {
        for (const id of ['serving', 'signal', 'neighbours', 'changes', 'mast']) $(`lte-cell-${id}`).textContent = '--';
        if (c) $('lte-cell-serving').textContent = 'no CORV_CELL_INFO for 10 s';
        return;
    }
    $('lte-cell-serving').textContent = c.eci
        ? `${operatorName(c.mcc, c.mnc)} eNB ${c.enb} sector ${c.sector} (ECI ${c.eci}), TAC ${c.tac}, PCI ${c.pci}, `
            + `EARFCN ${c.earfcn} (band ${c.band}, ${frequencyText(c.earfcn, c.band, '?')} downlink), ${c.state}`
        : `no cell (${c.state})`;
    const v = (x, unit) => (x === null ? '-' : `${x} ${unit}`);
    $('lte-cell-signal').textContent = `RSRP ${v(c.rsrp, 'dBm')}, RSRQ ${v(c.rsrq, 'dB')}, SINR ${v(c.sinr, 'dB')}, RSSI ${v(c.rssi, 'dBm')}`;
    $('lte-cell-neighbours').textContent = c.neighbours
        ? `${c.neighbours}: ` + c.nb.map((n) => `PCI ${n.pci}${n.earfcn !== c.earfcn ? `/${n.earfcn}` : ''} ${v(n.rsrp, 'dBm')}`).join(', ')
        : '0';
    $('lte-cell-changes').textContent = `${c.handovers}` + (c.changedAt ? `, last ${Math.round((Date.now() - c.changedAt) / 1000)} s ago` : '');
    const site = cellMapInfo.servingSite;
    let mast = 'shown when the map is open';
    if (cellMapInfo.error) mast = `database: ${cellMapInfo.error}`;
    else if (site) {
        const src = [...new Set(site.cells.map((x) => SOURCE_NAMES[x.src] || x.src))].join(', ');
        mast = `${site.lat.toFixed(5)}, ${site.lon.toFixed(5)} (estimate from ${site.cells.length} cell${site.cells.length > 1 ? 's' : ''} of the eNB, ${src})`
            + (site.mnc !== c.mnc ? `; shared cell, listed under ${operatorName(site.mcc, site.mnc)}` : '');
    } else if (cellMapInfo.lookingUp) mast = 'not in the database: looking it up online';
    else if (cellMapInfo.servingFound === false) mast = 'not in the database nor online (site never measured, or newer than the data)';
    $('lte-cell-mast').textContent = mast;
}

async function initCellMap() {
    const s = getCellMapSettings();
    $('lte-cell-show').checked = s.show;
    $('lte-cell-all').checked = s.allOperators;
    $('lte-cell-show').addEventListener('change', (e) => setCellMapSettings({ show: e.target.checked }));
    $('lte-cell-all').addEventListener('change', (e) => setCellMapSettings({ allOperators: e.target.checked }));
    // GCS OPTIONS (sidebar) → LTE OVERLAY: the cell rows of the position panel
    const overlay = $('chk-lte-overlay');
    if (overlay) {
        overlay.checked = s.table;
        overlay.addEventListener('change', (e) => setCellMapSettings({ table: e.target.checked }));
    }
    setInterval(renderCell, 1000);

    const sel = $('lte-celldb-country');
    for (const name of Object.keys(COUNTRIES)) sel.add(new Option(name, name));
    if (!window.cellDb) {
        $('lte-celldb-status').textContent = 'not available';
        return;
    }
    const refreshStatus = async () => renderCellDbStatus(await window.cellDb.status().catch(() => null));
    await refreshStatus();
    $('lte-celldb-online').addEventListener('change', async (e) => {
        await window.cellDb.settings({ online: e.target.checked });
    });
    $('lte-celldb-token-save').addEventListener('click', async () => {
        const token = $('lte-celldb-token').value.trim();
        await window.cellDb.settings({ token });
        $('lte-celldb-token').value = '';
        await refreshStatus();
    });
    $('lte-celldb-ocid').addEventListener('click', async () => {
        const { mcc } = COUNTRIES[sel.value];
        const btn = $('lte-celldb-ocid');
        btn.disabled = true;
        $('lte-celldb-files').textContent = `downloading the OpenCelliD file of MCC ${mcc}...`;
        try {
            await window.cellDb.downloadOpenCellid(mcc);
            setCellMapSettings({});   // the map queries again
        } catch (e) {
            alert('OpenCelliD download failed: ' + ipcMessage(e));
        } finally {
            btn.disabled = false;
            await refreshStatus();
        }
    });
    $('lte-celldb-import').addEventListener('click', async () => {
        try {
            if (await window.cellDb.importCsv()) setCellMapSettings({});
        } catch (e) {
            alert('Import failed: ' + ipcMessage(e));
        }
        await refreshStatus();
    });
    window.cellDb.onProgress((p) => {
        $('lte-celldb-status').textContent = `downloading: area ${p.done} of ${p.total}, ${p.cells.toLocaleString()} cells`
            + (p.failed ? `, ${p.failed} failed` : '');
    });
    $('lte-celldb-download').addEventListener('click', async () => {
        const [south, west, north, east] = COUNTRIES[sel.value].box;
        $('lte-celldb-download').disabled = true;
        try {
            const r = await window.cellDb.downloadBox({ south, west, north, east });
            renderCellDbStatus(r.status);
            if (r.failed) alert(`${r.failed} of ${r.blocks} areas could not be downloaded: try again later.`);
        } catch (e) {
            alert('Cell download failed: ' + ipcMessage(e));
        } finally {
            $('lte-celldb-download').disabled = false;
        }
    });
    $('lte-celldb-clear').addEventListener('click', async () => {
        if (!confirm('Delete the downloaded, imported and looked-up cells? The built-in ones stay.')) return;
        renderCellDbStatus(await window.cellDb.clear());
        setCellMapSettings({});   // the map queries again
    });
}
