/**
 * JoystickManager.js - Gamepad polling, axis mapping, and RC override sending
 */

import { STATE } from '../core/state.js';
import { sendRCChannelsOverride, sendServoTest, setParameter } from '../mavlink/CommandSender.js';
import { LOOK_DIRECTIONS, lookAroundInput, canLookAround } from '../input/LookAroundInput.js';

const STORAGE_KEY = 'datad-joystick-config';
const RELEASE = 0; // 0 = release channel back to RC receiver
const MIN_INPUTS = 8;
const validIndex = value => Number.isInteger(value) && value >= 0 && value < 256;
const clamp = (value, min, max, fallback) => Number.isFinite(value) ? Math.max(min, Math.min(max, value)) : fallback;
const buttonIndex = value => validIndex(value) ? value : -1;

function normalizeAxis(cfg, index) {
    return {
        sourceType: ['axis', 'button', 'none'].includes(cfg?.sourceType) ? cfg.sourceType : 'axis',
        sourceIndex: validIndex(cfg?.sourceIndex) ? cfg.sourceIndex : index,
        channel: Number.isInteger(cfg?.channel) && cfg.channel >= 0 && cfg.channel <= 18 ? cfg.channel : 0,
        inverted: !!cfg?.inverted,
        deadzone: clamp(cfg?.deadzone, 0, 0.9, 0.05),
        expo: clamp(cfg?.expo, 0, 1, 0)
    };
}

function normalizeServo(cfg, index) {
    return {
        button: buttonIndex(cfg?.button),
        servo: Number.isInteger(cfg?.servo) && cfg.servo >= 1 && cfg.servo <= 16 ? cfg.servo : 9 + index,
        percent: clamp(cfg?.percent, 0, 100, 100)
    };
}

/**
 * Default axis-to-channel mapping (standard RC order)
 * Axis 0 → CH1 (Roll), Axis 1 → CH2 (Pitch), Axis 2 → CH4 (Yaw), Axis 3 → CH3 (Throttle)
 */
const DEFAULT_AXIS_MAP = [
    { channel: 1, inverted: false, deadzone: 0.05, expo: 0.3 },  // Axis 0 → Roll
    { channel: 2, inverted: false, deadzone: 0.05, expo: 0.3 },  // Axis 1 → Pitch
    { channel: 4, inverted: false, deadzone: 0.05, expo: 0.3 },  // Axis 2 → Yaw
    { channel: 3, inverted: false, deadzone: 0.05, expo: 0.0 },  // Axis 3 → Throttle
];

export class JoystickManager {
    constructor() {
        this.gamepadIndex = null;
        this.gamepadId = null;      // id of the selected pad — used to re-select it after a replug
        this.enabled = false;
        // enabled but no usable gamepad data (unplugged, or the window lost
        // focus and Chromium hides the pad): channels released, waiting for
        // the pad to come back. Cleared automatically when data resumes.
        this.suspended = false;
        this.onStateChange = null;  // UI hook: enable/disable/suspend/resume/select
        this._lastNullWarn = 0;
        this.axisMap = Array.from({ length: MIN_INPUTS }, (_, i) => normalizeAxis(DEFAULT_AXIS_MAP[i], i));
        this.servoMap = [normalizeServo(null, 0), normalizeServo(null, 1)];
        this.viewMap = Object.fromEntries(LOOK_DIRECTIONS.map(dir => [dir, -1]));
        this.servoButtonsEnabled = false; // Deliberate activation each session, never persisted.
        this.servoStatus = '';
        this.pressedButtons = [];
        this._previousButtons = null;
        this._buttonTarget = null;
        this._servoPending = new Set();
        this._lastPreview = 0;
        this.channelValues = new Array(18).fill(RELEASE);
        this.rawAxisValues = [];
        this.pollHandle = null;
        this.sendInterval = null;
        this.sendRateHz = 25;
        this.onUpdate = null;
        this.lastGamepadTimestamp = 0;

        // Gamepad events
        this._onConnected = (e) => this._handleGamepadConnected(e);
        this._onDisconnected = (e) => this._handleGamepadDisconnected(e);
        window.addEventListener('gamepadconnected', this._onConnected);
        window.addEventListener('gamepaddisconnected', this._onDisconnected);
        this._onBlur = () => this._resetButtons();
        this._onVisibility = () => { if (document.hidden) this._resetButtons(); };
        window.addEventListener('blur', this._onBlur);
        window.addEventListener('mavlinkConnectionState', this._onBlur);
        document.addEventListener('visibilitychange', this._onVisibility);

        this.loadConfig();
    }

    /**
     * Detect all currently connected gamepads
     */
    detectGamepads() {
        const gamepads = navigator.getGamepads();
        const result = [];
        for (let i = 0; i < gamepads.length; i++) {
            if (gamepads[i]) {
                result.push({ index: i, id: gamepads[i].id, axes: gamepads[i].axes.length, buttons: gamepads[i].buttons.length });
            }
        }
        return result;
    }

    /**
     * Select a gamepad by index
     */
    selectGamepad(index) {
        const gp = navigator.getGamepads()[index];
        if (!gp) return false;

        if (this.gamepadIndex !== index || this.gamepadId !== gp.id) {
            if (this.enabled) this._releaseAllChannels();
            this.channelValues.fill(RELEASE);
            this._resetButtons();
        }
        this.gamepadIndex = index;
        this.gamepadId = gp.id;
        STATE.joystickConnected = true;

        // Build axis map for this gamepad's axes count
        const axisCount = Math.max(MIN_INPUTS, gp.axes.length, this.axisMap.length);
        this.rawAxisValues = new Array(axisCount).fill(null);

        // Preserve existing mappings, fill defaults for new axes
        const newMap = [];
        for (let i = 0; i < axisCount; i++) {
            if (this.axisMap[i]) {
                newMap.push({ ...this.axisMap[i] });
            } else if (DEFAULT_AXIS_MAP[i]) {
                newMap.push(normalizeAxis(DEFAULT_AXIS_MAP[i], i));
            } else {
                newMap.push(normalizeAxis(null, i));
            }
        }
        this.axisMap = newMap;
        this._startPolling(); // Preview and local FPV work without RC override.
        this.saveConfig();
        this._notify();
        return true;
    }

    /**
     * Re-select the pad we were using if it is present again (same id), or
     * the only pad connected when nothing was selected. Returns true if a
     * pad is selected afterwards.
     */
    reselectGamepad() {
        if (this.gamepadIndex !== null && navigator.getGamepads()[this.gamepadIndex]?.id === this.gamepadId) return true;
        const pads = this.detectGamepads();
        let pick = this.gamepadId ? pads.find(p => p.id === this.gamepadId) : null;
        if (!pick && !this.gamepadId && pads.length === 1) pick = pads[0];
        if (!pick) return false;
        return this.selectGamepad(pick.index);
    }

    _notify() {
        if (this.onStateChange) {
            try { this.onStateChange(); } catch (e) { /* UI must not break the manager */ }
        }
    }

    /**
     * Enable joystick - start polling and sending
     * Disables RC throttle failsafe so ArduPilot won't disarm without a physical receiver
     */
    async enable() {
        if (this.enabled) return true;
        if (this.gamepadIndex === null && !this.reselectGamepad()) return false;
        this.enabled = true;
        this.suspended = false;
        this.lastGamepadTimestamp = Date.now(); // grace period before the stale-data failsafe
        STATE.joystickEnabled = true;
        this._notify();

        // Configure ArduPilot to accept RC overrides from this GCS
        try {
            // Set SYSID_MYGCS to match our GCS sysid (255)
            // ArduPilot ONLY accepts RC_CHANNELS_OVERRIDE from this sysid.
            // Must match the sysid in MavLinkProtocolV2 in main-mavlink.js.
            this._savedSysidMygcs = (STATE.parameters.get('SYSID_MYGCS') || {}).value;
            await setParameter('SYSID_MYGCS', 255);
            console.log('[Joystick] Set SYSID_MYGCS = 255');
        } catch (e) {
            console.warn('[Joystick] Could not set SYSID_MYGCS:', e.message);
        }

        try {
            // Enable RC override timeout (3 seconds) - 0 means overrides are DISABLED
            this._savedRcOverrideTime = (STATE.parameters.get('RC_OVERRIDE_TIME') || {}).value;
            await setParameter('RC_OVERRIDE_TIME', 3.0);
            console.log('[Joystick] Set RC_OVERRIDE_TIME = 3.0');
        } catch (e) {
            console.warn('[Joystick] Could not set RC_OVERRIDE_TIME:', e.message);
        }

        try {
            // RC_OPTIONS bit 1 (2) ignores GCS overrides. Bit 8 is CRSF passthrough.
            this._savedRcOptions = (STATE.parameters.get('RC_OPTIONS') || {}).value;
            const current = this._savedRcOptions || 0;
            const cleared = current & ~2;
            if (cleared !== current) {
                await setParameter('RC_OPTIONS', cleared);
                console.log('[Joystick] Cleared RC_OPTIONS ignore-override bit:', current, '->', cleared);
            }
        } catch (e) {
            console.warn('[Joystick] Could not set RC_OPTIONS:', e.message);
        }

        try {
            // Disable RC throttle failsafe (no physical receiver when using joystick)
            this._savedFsThrEnable = (STATE.parameters.get('FS_THR_ENABLE') || {}).value;
            await setParameter('FS_THR_ENABLE', 0);
            console.log('[Joystick] Disabled FS_THR_ENABLE');
        } catch (e) {
            console.warn('[Joystick] Could not disable FS_THR_ENABLE:', e.message);
        }

        // The user may have switched it off while the parameter writes above
        // were in flight — don't start the loops under their feet.
        if (!this.enabled) return false;
        this._startPolling();
        this._startSending();
        this._notify();
        return true;
    }

    /**
     * Disable RC sending and release channels; local inputs keep polling.
     * Restores RC throttle failsafe
     */
    disable() {
        const wasEnabled = this.enabled;
        this.enabled = false;
        this.suspended = false;
        STATE.joystickEnabled = false;
        STATE.rcOverrideActive = false;
        this._stopSending();
        // Release all channels
        if (wasEnabled) this._releaseAllChannels();
        this.channelValues.fill(RELEASE);
        this.rawAxisValues.fill(0);
        if (wasEnabled) this._notify();

        // Restore original parameters only after an active RC session.
        if (!wasEnabled) return;
        if (this._savedSysidMygcs !== undefined && this._savedSysidMygcs !== null) {
            setParameter('SYSID_MYGCS', this._savedSysidMygcs).catch(() => {});
            console.log('[Joystick] Restored SYSID_MYGCS to', this._savedSysidMygcs);
        }
        if (this._savedRcOverrideTime !== undefined && this._savedRcOverrideTime !== null) {
            setParameter('RC_OVERRIDE_TIME', this._savedRcOverrideTime).catch(() => {});
            console.log('[Joystick] Restored RC_OVERRIDE_TIME to', this._savedRcOverrideTime);
        }
        if (this._savedRcOptions !== undefined && this._savedRcOptions !== null) {
            setParameter('RC_OPTIONS', this._savedRcOptions).catch(() => {});
            console.log('[Joystick] Restored RC_OPTIONS to', this._savedRcOptions);
        }
        if (this._savedFsThrEnable !== undefined && this._savedFsThrEnable !== null) {
            setParameter('FS_THR_ENABLE', this._savedFsThrEnable).catch(() => {});
            console.log('[Joystick] Restored FS_THR_ENABLE to', this._savedFsThrEnable);
        }
    }

    /**
     * Set send rate in Hz
     */
    setSendRate(hz) {
        // Clamp: hz <= 0 would produce a 0 ms setInterval (RC override flood)
        this.sendRateHz = Number.isFinite(hz) ? Math.max(1, Math.min(50, hz)) : 25;
        if (this.sendInterval !== null) {
            this._stopSending();
            this._startSending();
        }
        this.saveConfig();
    }

    /**
     * Update axis mapping config
     */
    setAxisConfig(axisIndex, config) {
        if (!this.axisMap[axisIndex]) return;
        this.axisMap[axisIndex] = normalizeAxis({ ...this.axisMap[axisIndex], ...config }, axisIndex);
        this.saveConfig();
    }

    setServoConfig(index, config) {
        if (!this.servoMap[index]) return;
        this.servoMap[index] = normalizeServo({ ...this.servoMap[index], ...config }, index);
        this._resetButtons();
        this.saveConfig();
    }

    setViewButton(direction, button) {
        if (!LOOK_DIRECTIONS.includes(direction)) return;
        this.viewMap[direction] = buttonIndex(button);
        this._resetButtons();
        this.saveConfig();
    }

    setServoButtonsEnabled(enabled) {
        this.servoButtonsEnabled = !!enabled;
        this.servoStatus = enabled ? 'Ready - release held buttons before commanding' : 'Disabled';
        this._resetButtons();
        this._notify();
    }

    clearGamepad() {
        this.gamepadIndex = null;
        this.gamepadId = null;
        this.servoButtonsEnabled = false;
        this.servoStatus = 'Disabled';
        STATE.joystickConnected = false;
        this.disable();
        this._stopPolling();
        this._resetButtons();
        this.saveConfig();
        this._notify();
    }

    _resetButtons() {
        this._previousButtons = null;
        this._buttonTarget = null;
        this.pressedButtons = [];
        lookAroundInput.release('gamepad');
    }

    _readSource(gp, cfg) {
        const raw = cfg.sourceType === 'axis' ? gp.axes[cfg.sourceIndex]
            : cfg.sourceType === 'button' ? gp.buttons[cfg.sourceIndex]?.value : null;
        if (!Number.isFinite(raw)) return null;
        return cfg.sourceType === 'button' ? clamp(raw, 0, 1, 0) * 2 - 1 : clamp(raw, -1, 1, 0);
    }

    _pollButtons(gp) {
        const buttons = gp.buttons.map(b => !!b.pressed || b.value > 0.5);
        this.pressedButtons = buttons.flatMap((pressed, i) => pressed ? [i] : []);
        if (document.hidden || !document.hasFocus()) {
            this._resetButtons();
            return;
        }
        const view = {};
        const viewAllowed = canLookAround();
        for (const dir of LOOK_DIRECTIONS) view[dir] = viewAllowed && buttons[this.viewMap[dir]];
        lookAroundInput.setGamepad(view);

        // Prime edges again after reconnect/focus/config changes: a held button
        // must never become a new servo command on a different connection.
        const target = STATE.connected ? `${STATE.connectionType}:${STATE.systemId}:${STATE.componentId}` : null;
        if (this._previousButtons && target !== null && target === this._buttonTarget && this.servoButtonsEnabled) {
            this.servoMap.forEach((cfg, index) => {
                if (buttons[cfg.button] && this._previousButtons[cfg.button] === false) this._sendServo(index, cfg);
            });
        }
        this._previousButtons = buttons;
        this._buttonTarget = target;
    }

    async _sendServo(index, cfg) {
        if (this._servoPending.has(index)) return;
        this._servoPending.add(index);
        // Match the existing servo test: 0..100% represents 1000..2000 us.
        const pwm = Math.round(1000 + cfg.percent * 10);
        try {
            const result = await sendServoTest(cfg.servo, pwm);
            if (result?.success === false) throw new Error(result.error || 'Send failed');
            this.servoStatus = `Sent S${cfg.servo}: ${cfg.percent}% (${pwm} us)`;
        } catch (error) {
            this.servoStatus = `S${cfg.servo}: ${error.message}`;
            console.warn('[Joystick] Servo command:', error.message);
        } finally {
            this._servoPending.delete(index);
            this._notify();
        }
    }

    // --- Polling ---
    // Uses setInterval instead of requestAnimationFrame so gamepad polling
    // is independent of render scheduling. Both still share the renderer thread;
    // the stale-data check releases RC channels after a polling interruption.

    _startPolling() {
        if (this.pollHandle !== null) return;
        // Poll at 100Hz (10ms) for responsive input — fast enough for RC control
        this.pollHandle = setInterval(() => this._poll(), 10);
    }

    _stopPolling() {
        if (this.pollHandle !== null) {
            clearInterval(this.pollHandle);
            this.pollHandle = null;
        }
    }

    _poll() {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[this.gamepadIndex];

        if (!gp || gp.id !== this.gamepadId) {
            this._resetButtons();
            this.rawAxisValues.fill(null);
            // Gamepad API returns null while the window has no focus (Chromium
            // hides pads from unfocused pages) or right after an unplug. The
            // 500 ms failsafe in _sendOverride releases the channels; here we
            // only wait for the pad to come back (same index, or same id on a
            // replug).
            const now = Date.now();
            if (this.enabled && now - this._lastNullWarn > 2000) {
                this._lastNullWarn = now;
                console.warn('[Joystick] No gamepad data (window unfocused or pad unplugged)');
            }
            if (this.reselectGamepad()) {
                // reselect swapped the index; data resumes on the next tick
            }
            return;
        }

        this.lastGamepadTimestamp = Date.now();
        if (this.suspended) {
            this.suspended = false;
            console.log('[Joystick] Gamepad data resumed — override active again');
            this._notify();
        }

        // Reset channel values
        this.channelValues.fill(RELEASE);

        // Process each axis
        for (let i = 0; i < this.axisMap.length; i++) {
            const cfg = this.axisMap[i];
            let value = this._readSource(gp, cfg);
            this.rawAxisValues[i] = value;

            if (cfg.channel === 0 || value === null) continue; // Unmapped/unavailable

            // Inversion
            if (cfg.inverted) value = -value;

            // Deadzone
            value = this._applyDeadzone(value, cfg.deadzone);

            // Expo
            value = this._applyExpo(value, cfg.expo);

            // Convert to PWM (1000-2000, center 1500)
            const pwm = this._axisToPWM(value);

            // Map to channel (1-indexed to 0-indexed)
            const chIdx = cfg.channel - 1;
            if (chIdx >= 0 && chIdx < 18) {
                this.channelValues[chIdx] = pwm;
            }
        }

        this._pollButtons(gp);
        // The controls run at 100 Hz; DOM previews need at most 25 Hz.
        if (this.onUpdate && Date.now() - this._lastPreview >= 40) {
            this._lastPreview = Date.now();
            this.onUpdate();
        }
    }

    _applyDeadzone(value, deadzone) {
        if (Math.abs(value) < deadzone) return 0;
        // Rescale remaining range to 0..1
        const sign = value > 0 ? 1 : -1;
        return sign * (Math.abs(value) - deadzone) / (1 - deadzone);
    }

    _applyExpo(value, expo) {
        if (expo === 0) return value;
        const sign = value > 0 ? 1 : -1;
        const abs = Math.abs(value);
        return sign * (expo * abs * abs * abs + (1 - expo) * abs);
    }

    _axisToPWM(value) {
        // Clamp to -1..1
        value = Math.max(-1, Math.min(1, value));
        return Math.round(1500 + value * 500);
    }

    // --- Sending ---

    _startSending() {
        if (this.sendInterval !== null) return;
        const intervalMs = Math.round(1000 / this.sendRateHz);
        this.sendInterval = setInterval(() => this._sendOverride(), intervalMs);
    }

    _stopSending() {
        if (this.sendInterval !== null) {
            clearInterval(this.sendInterval);
            this.sendInterval = null;
        }
    }

    _sendOverride() {
        if (!this.enabled || !STATE.connected) {
            STATE.rcOverrideActive = false;
            return;
        }

        // Failsafe: no gamepad data for 500 ms → release the channels to the
        // RC receiver and suspend. Stay enabled: the user did not switch the
        // override off, so sending resumes as soon as the pad is readable
        // again (focus back, pad replugged).
        if (this.suspended || Date.now() - this.lastGamepadTimestamp > 500) {
            if (!this.suspended) {
                this.suspended = true;
                STATE.rcOverrideActive = false;
                this.channelValues.fill(RELEASE);
                this._releaseAllChannels();
                console.warn('[Joystick] Gamepad data stale — channels released, override suspended');
                this._notify();
            }
            return;
        }

        STATE.rcOverrideActive = true;
        sendRCChannelsOverride(this.channelValues).catch(err => {
            console.warn('[Joystick] RC override send error:', err.message);
        });
    }

    _releaseAllChannels() {
        if (STATE.connected) {
            const release = new Array(18).fill(0);
            sendRCChannelsOverride(release).catch(() => {});
        }
    }

    // --- Gamepad events ---

    _handleGamepadConnected(e) {
        console.log('[Joystick] Gamepad connected:', e.gamepad.id);
        STATE.joystickConnected = true;
        // Replug of the pad we were using (or first pad seen while nothing is
        // selected): pick it up again so the override can simply be re-enabled
        // — or resumes by itself if it is still enabled.
        if (this.gamepadIndex === null) this.reselectGamepad();
        this._notify();
        if (this.onUpdate) this.onUpdate();
    }

    _handleGamepadDisconnected(e) {
        console.log('[Joystick] Gamepad disconnected:', e.gamepad.id);
        if (e.gamepad.index === this.gamepadIndex) {
            this._handleLostGamepad();
        }
    }

    _handleLostGamepad() {
        this._resetButtons();
        this.rawAxisValues.fill(null);
        // Keep gamepadId so the same pad is re-selected on replug.
        this.gamepadIndex = null;
        STATE.joystickConnected = false;
        if (this.enabled && !this.suspended) {
            this.suspended = true;
            STATE.rcOverrideActive = false;
            this.channelValues.fill(RELEASE);
            this._releaseAllChannels();
            console.warn('[Joystick] Gamepad lost — channels released, override suspended');
        }
        this._notify();
        if (this.onUpdate) this.onUpdate();
    }

    // --- Persistence ---

    saveConfig() {
        const config = {
            gamepadIndex: this.gamepadIndex,
            gamepadId: this.gamepadId,
            sendRateHz: this.sendRateHz,
            axisMap: this.axisMap,
            servoMap: this.servoMap,
            viewMap: this.viewMap
        };
        try {
            localStorage.setItem(STORAGE_KEY, JSON.stringify(config));
        } catch (e) { /* ignore */ }
    }

    loadConfig() {
        try {
            const raw = localStorage.getItem(STORAGE_KEY);
            if (!raw) return;
            const config = JSON.parse(raw);
            // Sanitize persisted values: corrupted/hand-edited config must not
            // produce a 0 ms send interval or NaN PWM (deadzone >= 1 divides by 0)
            if (typeof config.gamepadId === 'string') this.gamepadId = config.gamepadId;
            if (Number.isFinite(config.sendRateHz)) {
                this.sendRateHz = Math.max(1, Math.min(50, config.sendRateHz));
            }
            if (Array.isArray(config.axisMap)) {
                this.axisMap = Array.from({ length: Math.max(MIN_INPUTS, Math.min(64, config.axisMap.length)) },
                    (_, i) => normalizeAxis(config.axisMap[i] ?? DEFAULT_AXIS_MAP[i], i));
            }
            if (Array.isArray(config.servoMap)) {
                this.servoMap = this.servoMap.map((cfg, i) => normalizeServo(config.servoMap[i], i));
            }
            for (const dir of LOOK_DIRECTIONS) this.viewMap[dir] = buttonIndex(config.viewMap?.[dir]);
        } catch (e) { /* ignore */ }
    }

    /**
     * Cleanup
     */
    destroy() {
        this.disable();
        this.servoButtonsEnabled = false;
        this._stopPolling();
        this._resetButtons();
        window.removeEventListener('gamepadconnected', this._onConnected);
        window.removeEventListener('gamepaddisconnected', this._onDisconnected);
        window.removeEventListener('blur', this._onBlur);
        window.removeEventListener('mavlinkConnectionState', this._onBlur);
        document.removeEventListener('visibilitychange', this._onVisibility);
    }
}
