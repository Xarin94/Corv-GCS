#!/usr/bin/env node
// Electron smoke for the real joystick markup, styles and event handlers.
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const root = path.resolve(__dirname, '..');

if (!process.versions.electron) {
    const { spawn } = require('node:child_process');
    const env = { ...process.env }; delete env.ELECTRON_RUN_AS_NODE;
    const child = spawn(require('electron'), [__filename, ...process.argv.slice(2)], {
        cwd: root, env, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe']
    });
    child.stdout.pipe(process.stdout);
    child.stderr.pipe(process.stderr);
    const timeout = setTimeout(() => { child.kill(); process.exitCode = 1; }, 30000);
    child.on('error', error => { console.error(error); process.exitCode = 1; });
    child.on('exit', code => { clearTimeout(timeout); process.exitCode = code || 0; });
} else {
    run().catch(error => { console.error(error); require('electron').app.exit(1); });
}

async function run() {
    const { app, BrowserWindow } = require('electron');
    const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-joystick-ui-'));
    app.setPath('userData', path.join(tmp, 'profile'));
    await app.whenReady();
    const html = fs.readFileSync(path.join(root, 'html/index.html'), 'utf8');
    const panel = html.slice(html.indexOf('<div class="sub-tab-content" id="subtab-joystick">'), html.indexOf('<!-- Sub-tab: Radio Calibration -->'))
        .replace('class="sub-tab-content"', 'class="sub-tab-content active"');
    const base = pathToFileURL(root + path.sep).href;
    const harness = `<!doctype html><html><head><meta charset="utf-8"><base href="${base}">
        <link rel="stylesheet" href="vendor/fonts/fonts.css"><link rel="stylesheet" href="css/style.css">
        <style>body { padding: 20px; overflow: auto; } #subtab-joystick { display: block; }</style></head><body>
        <div id="tab-flight-data" class="active"></div>${panel}
        <script type="module">
        window.__pad = { id: 'UI simulated pad', index: 0, axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => ({ value: 0, pressed: false })) };
        Object.defineProperty(navigator, 'getGamepads', { value: () => [window.__pad] });
        document.hasFocus = () => true;
        window.__commands = []; window.__messages = [];
        window.mavlink = {
            sendCommand: async cmd => { window.__commands.push(cmd); return { success: true }; },
            sendMessage: async msg => { window.__messages.push(msg); return { success: true }; }
        };
        window.addEventListener('error', e => { window.__error = e.message; });
        const { STATE } = await import('./js/core/state.js'); STATE.connected = true;
        const { initJoystick } = await import('./js/joystick/JoystickUI.js');
        const { lookAroundInput } = await import('./js/input/LookAroundInput.js');
        window.__look = lookAroundInput;
        initJoystick(); window.__ready = true;
        </script></body></html>`;
    const page = path.join(tmp, 'joystick.html');
    fs.writeFileSync(page, harness);
    const win = new BrowserWindow({ show: false, width: 1080, height: 1180,
        webPreferences: { nodeIntegration: false, contextIsolation: true, backgroundThrottling: false } });
    const errors = [];
    win.webContents.on('console-message', event => {
        if (event.level === 'error') { errors.push(event.message); console.error(event.message); }
    });
    await win.loadFile(page);
    const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
    await wait(700);
    const result = await win.webContents.executeJavaScript(`(async () => {
        const assert = (ok, text) => { if (!ok) throw new Error(text); };
        const wait = ms => new Promise(r => setTimeout(r, ms));
        const change = (selector, value) => {
            const el = document.querySelector(selector); assert(el, selector);
            if (el.type === 'checkbox') el.checked = value; else el.value = String(value);
            el.dispatchEvent(new Event('change', { bubbles: true }));
        };
        assert(window.__ready, 'UI initialization');
        document.getElementById('joystick-scan').click();
        change('#joystick-gamepad-select', 0);
        assert(document.querySelectorAll('.joystick-axis-row').length === 8, 'eight axis rows');
        assert(document.querySelectorAll('[data-servo-button]').length === 2, 'two servo mappings');
        assert(document.querySelectorAll('[data-view-button]').length === 4, 'four FPV mappings');
        change('.joystick-source-select[data-axis="7"]', 'button:7');
        change('.joystick-ch-select[data-axis="7"]', 8);
        window.__pad.buttons[7] = { value: 1, pressed: true }; await wait(80);
        assert(document.querySelector('[data-ch-val="8"]').textContent === '2000', 'live eighth RC channel');
        assert(window.__messages.length === 0, 'preview sends no MAVLink');
        change('[data-servo-button="0"]', 0); change('[data-servo-output="0"]', 11);
        change('[data-servo-percent="0"]', 63); change('#joystick-servo-enable', true); await wait(40);
        window.__pad.buttons[0] = { value: 1, pressed: true }; await wait(120);
        assert(window.__commands.length === 1, 'one command per press');
        assert(window.__commands[0].param1 === 11 && window.__commands[0].param2 === 1630, 'servo output / percent');
        change('[data-view-button="left"]', 14); document.activeElement?.blur();
        window.__pad.buttons[14] = { value: 1, pressed: true }; await wait(40);
        assert(window.__look.read().left, 'FPV input while RC disabled');
        window.__pad.buttons[14] = { value: 0, pressed: false }; await wait(40);
        assert(!window.__look.read().left, 'FPV release centers');
        const cfg = JSON.parse(localStorage.getItem('datad-joystick-config'));
        assert(cfg.axisMap[7].sourceType === 'button' && cfg.servoMap[0].percent === 63 && cfg.viewMap.left === 14, 'persist mappings');
        const panel = document.getElementById('joystick-panel');
        assert(panel.scrollWidth <= panel.clientWidth, 'panel has no horizontal overflow');
        assert(!window.__error, window.__error);
        return { axes: 8, servos: 2, fpv: 4, servoPwm: window.__commands[0].param2, width: panel.clientWidth };
    })().catch(error => ({ error: error.message, stack: error.stack }))`);
    if (result.error) throw new Error(result.stack || result.error);
    const screenshotAt = process.argv.indexOf('--screenshot');
    if (screenshotAt >= 0) fs.writeFileSync(path.resolve(process.argv[screenshotAt + 1]), (await win.webContents.capturePage()).toPNG());
    await win.loadFile(page); await wait(700);
    await win.webContents.executeJavaScript(`(() => {
        const assert = (ok, text) => { if (!ok) throw new Error(text); };
        assert(document.querySelector('.joystick-source-select[data-axis="7"]').value === 'button:7', 'reload axis source');
        assert(document.querySelector('[data-servo-percent="0"]').value === '63', 'reload servo percent');
        assert(document.querySelector('[data-view-button="left"]').value === '14', 'reload FPV button');
        assert(!document.getElementById('joystick-servo-enable').checked, 'reload does not auto-enable servos');
        const select = document.getElementById('joystick-gamepad-select'); select.value = '';
        select.dispatchEvent(new Event('change', { bubbles: true }));
        assert(select.value === '', 'deselect actually clears controller');
    })()`);
    win.setSize(650, 1100); await wait(100);
    await win.webContents.executeJavaScript(`(() => {
        const panel = document.getElementById('joystick-panel');
        if (panel.scrollWidth > panel.clientWidth) throw new Error('narrow panel overflow');
    })()`);
    if (errors.length) throw new Error(errors.join('\n'));
    console.log('PASS Electron joystick UI:', JSON.stringify(result), 'reload, deselect and narrow layout');
    win.destroy();
    // Chromium can retain profile file handles until process exit. Let the OS
    // temp directory own the isolated browser profile; no repo artifacts.
    app.exit(0);
}
