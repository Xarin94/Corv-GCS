#!/usr/bin/env node
// Exercise the real input manager and CommandSender with a simulated Gamepad
// and IPC transport. No connection to a vehicle or physical servo is opened.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const root = path.join(__dirname, '..');
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-joystick-'));
fs.writeFileSync(path.join(tmp, 'package.json'), '{"type":"module"}');
for (const relative of ['js/joystick', 'js/input', 'js/mavlink', 'js/core', 'js/adsb']) {
    fs.cpSync(path.join(root, relative), path.join(tmp, relative), { recursive: true });
}
const load = relative => import(pathToFileURL(path.join(tmp, relative)));
const storage = new Map();
global.localStorage = { getItem: key => storage.get(key) ?? null, setItem: (key, value) => storage.set(key, value) };
global.window = new EventTarget();
let focused = true;
let flightTab = true;
global.document = Object.assign(new EventTarget(), {
    hidden: false, activeElement: null, hasFocus: () => focused,
    getElementById: id => id === 'tab-flight-data' ? { classList: { contains: () => flightTab } } : null
});
const pad = { id: 'Simulated gamepad', index: 0, axes: [-1, 0, 1, 0.5], buttons: Array.from({ length: 17 }, () => ({ value: 0, pressed: false })) };
let pads = [pad];
Object.defineProperty(global, 'navigator', { configurable: true, value: { getGamepads: () => pads } });
const button = (i, value) => { pad.buttons[i] = { value, pressed: value > 0.5 }; };
const timers = new Map();
let timerId = 0;
global.setInterval = (fn, ms) => { timers.set(++timerId, { fn, ms }); return timerId; };
global.clearInterval = id => timers.delete(id);
const messages = [];
const commands = [];
let incoming;
let rejectCommand = false;
window.mavlink = {
    onMessage: fn => { incoming = fn; }, onConnectionState() {},
    sendMessage: async msg => {
        messages.push(structuredClone(msg));
        if (msg.type === 'PARAM_SET') setTimeout(() => incoming({ msgId: 22, sysId: 1, compId: 1,
            data: { paramId: msg.paramId, paramValue: msg.paramValue, paramType: 9 } }), 0);
        return { success: true };
    },
    sendCommand: async cmd => {
        if (rejectCommand) throw new Error('simulated transport failure');
        commands.push(structuredClone(cmd));
        return { success: true };
    }
};
const settle = () => new Promise(resolve => setTimeout(resolve, 10));
let manager;
(async () => {
    try {
        const { STATE } = await load('js/core/state.js');
        const { initMAVLink } = await load('js/mavlink/MAVLinkManager.js');
        const { JoystickManager } = await load('js/joystick/JoystickManager.js');
        const { lookAroundInput } = await load('js/input/LookAroundInput.js');
        const { sendRCChannelsOverride } = await load('js/mavlink/CommandSender.js');
        initMAVLink();
        STATE.connected = true;
        STATE.systemId = 1; STATE.componentId = 1;

        localStorage.setItem('datad-joystick-config', JSON.stringify({ gamepadId: pad.id, sendRateHz: 25,
            axisMap: [1, 2, 4, 3].map(channel => ({ channel, inverted: true, deadzone: 0.1, expo: 0.2 })) }));
        manager = new JoystickManager();
        assert.equal(manager.axisMap.length, 8);
        assert.deepEqual(manager.axisMap.slice(0, 4).map(m => m.channel), [1, 2, 4, 3]);
        assert.equal(manager.axisMap[0].inverted, true);
        assert.equal(manager.axisMap[0].sourceIndex, 0);
        manager.selectGamepad(0);
        assert.equal(manager.axisMap.length, 8, 'four-axis pad still offers eight mappings');
        for (let i = 0; i < 8; i++) manager.setAxisConfig(i, {
            channel: i + 1, sourceType: i < 4 ? 'axis' : 'button', sourceIndex: i < 4 ? i : i + 2,
            inverted: false, deadzone: 0, expo: 0
        });
        button(6, 0); button(7, 0.5); button(8, 1); button(9, 0.25);
        manager._poll();
        assert.deepEqual(manager.channelValues.slice(0, 8), [1000, 1500, 2000, 1750, 1000, 1500, 2000, 1250]);
        assert.equal(messages.length, 0, 'preview never sends RC or changes parameters');
        manager.setAxisConfig(7, { channel: 18 });
        manager.enabled = true;
        manager._poll(); manager._sendOverride(); manager.enabled = false;
        assert.equal(messages.at(-1).channels[17], 1250, 'eighth input reaches CH18');
        assert.equal(messages.at(-1).channels[7], 0);
        manager.setAxisConfig(7, { sourceIndex: 99 }); manager._poll();
        assert.equal(manager.channelValues[17], 0, 'missing source releases rather than centers');
        manager.setAxisConfig(7, { sourceType: 'axis', sourceIndex: 7, channel: 8 });
        pad.axes.push(-0.5, 0.25, -0.25, 1);
        for (let i = 4; i < 8; i++) manager.setAxisConfig(i, { sourceType: 'axis', sourceIndex: i, channel: i + 1 });
        manager._poll();
        assert.deepEqual(manager.channelValues.slice(4, 8), [1250, 1625, 1375, 2000]);
        manager.setAxisConfig(4, { inverted: true }); manager._poll();
        assert.equal(manager.channelValues[4], 1750);
        pad.axes[4] = NaN; manager._poll(); assert.equal(manager.channelValues[4], 0);
        pad.axes[4] = 0.05; manager.setAxisConfig(4, { deadzone: 0.1 }); manager._poll();
        assert.equal(manager.channelValues[4], 1500);
        console.log('PASS old config migration, eight physical/virtual inputs, CH18, inversion, deadzone and missing/invalid sources');

        await sendRCChannelsOverride(Array(18).fill(0));
        assert.deepEqual(messages.at(-1).channels, [...Array(8).fill(0), ...Array(10).fill(65534)]);
        await sendRCChannelsOverride(Array(18).fill(65535));
        assert(messages.at(-1).channels.every(v => v === 65535), 'ignore sentinel is preserved');
        manager.enabled = true;
        manager.lastGamepadTimestamp = Date.now() - 600;
        manager._sendOverride();
        assert.equal(manager.suspended, true);
        assert.equal(STATE.rcOverrideActive, false);
        assert.equal(messages.at(-1).channels[17], 65534);
        manager._poll(); assert.equal(manager.suspended, false);
        manager._handleLostGamepad();
        assert.equal(messages.at(-1).channels[17], 65534);
        pads = [{ ...pad, id: 'Different controller' }];
        assert.equal(manager.reselectGamepad(), false, 'do not silently switch controller after disconnect');
        pads = [pad]; assert.equal(manager.reselectGamepad(), true);
        manager.enabled = false;
        console.log('PASS MAVLink release/ignore for all 18 channels, stale input and reconnect identity');

        manager.setServoConfig(0, { button: 0, servo: 9, percent: 0 });
        manager.setServoConfig(1, { button: 1, servo: 10, percent: 100 });
        button(0, 1); manager.setServoButtonsEnabled(true); manager._poll();
        assert.equal(commands.length, 0, 'held button on enable is not a press');
        button(0, 0); manager._poll(); button(0, 1); manager._poll();
        await settle();
        assert.equal(commands.length, 1);
        assert.equal(commands[0].command, 183);
        assert.equal(commands[0].param1, 9); assert.equal(commands[0].param2, 1000);
        for (let i = 0; i < 100; i++) manager._poll();
        await settle(); assert.equal(commands.length, 1, 'holding does not repeat');
        button(1, 1); manager._poll(); await settle();
        assert.equal(commands.at(-1).param1, 10); assert.equal(commands.at(-1).param2, 2000);
        button(1, 0); manager._poll(); await settle();
        assert.equal(commands.length, 2, 'release sends nothing');
        manager.setServoConfig(1, { percent: 37 }); manager._poll();
        button(1, 1); manager._poll(); await settle(); assert.equal(commands.at(-1).param2, 1370);
        STATE.connected = false; button(1, 0); manager._poll(); button(1, 1); manager._poll();
        STATE.connected = true; manager._poll(); await settle();
        assert.equal(commands.length, 3, 'reconnection never replays a held command');
        focused = false; window.dispatchEvent(new Event('blur'));
        button(0, 0); manager._poll(); button(0, 1); manager._poll();
        focused = true; manager._poll(); await settle(); assert.equal(commands.length, 3);
        manager.setServoButtonsEnabled(false); button(0, 0); manager._poll(); button(0, 1); manager._poll();
        assert.equal(commands.length, 3);
        manager.setServoButtonsEnabled(true); button(0, 0); manager._poll();
        rejectCommand = true; button(0, 1); manager._poll(); await settle();
        assert.match(manager.servoStatus, /simulated transport failure/);
        rejectCommand = false;
        console.log('PASS two independent servo commands, 0/37/100%, press edges, error display, disable/focus/reconnect guards');

        for (const [dir, b] of Object.entries({ up: 12, down: 13, left: 14, right: 15 })) manager.setViewButton(dir, b);
        const before = messages.length;
        manager.enabled = false; manager.setServoButtonsEnabled(false);
        button(12, 1); button(14, 1); manager._poll();
        assert.deepEqual(lookAroundInput.read(), { up: true, down: false, left: true, right: false });
        assert.equal(messages.length, before, 'FPV input sends no vehicle traffic');
        lookAroundInput.setKey('left', true); button(12, 0); button(14, 0); manager._poll();
        assert.equal(lookAroundInput.read().left, true, 'gamepad release does not release keyboard');
        lookAroundInput.setKey('left', false); assert.equal(lookAroundInput.read().left, false);
        button(15, 1); manager._poll(); assert.equal(lookAroundInput.read().right, true);
        document.activeElement = { tagName: 'INPUT' }; manager._poll(); assert.equal(lookAroundInput.read().right, false);
        document.activeElement = null; flightTab = false; manager._poll(); assert.equal(lookAroundInput.read().right, false);
        flightTab = true; manager._poll(); assert.equal(lookAroundInput.read().right, true);
        pads = []; manager._poll(); assert.equal(lookAroundInput.read().right, false);
        pads = [pad]; manager._poll(); document.hidden = true;
        document.dispatchEvent(new Event('visibilitychange')); assert.equal(lookAroundInput.read().right, false);
        document.hidden = false;
        console.log('PASS FPV diagonals, independent keyboard release, typing/tab/focus/unplug guards with RC disabled');

        STATE.parameters.set('SYSID_MYGCS', { value: 42 });
        STATE.parameters.set('RC_OVERRIDE_TIME', { value: 2 });
        STATE.parameters.set('RC_OPTIONS', { value: 258 });
        STATE.parameters.set('FS_THR_ENABLE', { value: 1 });
        await manager.enable();
        assert.equal(STATE.parameters.get('RC_OPTIONS').value, 256, 'clear bit 1, keep CRSF passthrough bit 8');
        manager.disable(); await settle();
        assert.equal(STATE.parameters.get('RC_OPTIONS').value, 258, 'restore original RC options');
        manager.saveConfig();
        const persisted = new JoystickManager();
        assert.deepEqual(persisted.axisMap, manager.axisMap);
        assert.deepEqual(persisted.servoMap, manager.servoMap);
        assert.deepEqual(persisted.viewMap, manager.viewMap);
        assert.equal(persisted.servoButtonsEnabled, false);
        assert.equal(persisted.enabled, false);
        persisted.destroy();
        manager.clearGamepad();
        assert.equal(manager.gamepadIndex, null); assert.equal(manager.pollHandle, null);
        assert.equal(timers.size, 0);
        console.log('PASS correct RC_OPTIONS bit/restoration, mapping persistence without auto-activation, timer cleanup');
    } finally {
        manager?.destroy();
        fs.rmSync(tmp, { recursive: true, force: true });
    }
})().catch(error => { console.error(error); process.exitCode = 1; });
