/**
 * CellularLinkController.js - SETUP → COMMS → CELLULAR LINK
 *
 * Connects to one LTEtelem cellular module through a CRV2 relay: the operator
 * gives the module ID and its AES-256 key, the relay address is entered once.
 * The link itself lives in the main process (lte-link.js) and feeds the normal
 * MAVLink pipeline, so everything else in the GCS works as on any other link.
 *
 * Module ID and relay settings persist in localStorage under 'lte-link-config'.
 * The key is never written there: with REMEMBER KEY it is stored encrypted by
 * the OS keychain (Electron safeStorage) through window.lteKey.
 */

import { STATE } from '../core/state.js';
import { connect, disconnect } from '../mavlink/ConnectionManager.js';
import { setNavDot } from './TabController.js';

const STORAGE_KEY = 'lte-link-config';
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
        st.airOnline ? `${st.moduleId} — online, waiting for data` : `${st.moduleId} — offline on the relay`;
    $('lte-st-session').textContent = st.vehicleSession || '--';

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
    $('lte-relay-host').value = cfg.host || '';
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
}
