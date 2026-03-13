/**
 * JoystickManager.js - Gamepad polling, axis mapping, and RC override sending
 */

import { STATE } from '../core/state.js';
import { sendRCChannelsOverride, setParameter } from '../mavlink/CommandSender.js';

const STORAGE_KEY = 'datad-joystick-config';
const RELEASE = 0; // 0 = release channel back to RC receiver

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
        this.enabled = false;
        this.axisMap = [];
        this.channelValues = new Array(18).fill(RELEASE);
        this.rawAxisValues = [];
        this.pollHandle = null;
        this.sendInterval = null;
        this.sendRateHz = 25;
        this.onUpdate = null;
        this.lastGamepadTimestamp = 0;
        this._boundPoll = this._poll.bind(this);

        // Gamepad events
        this._onConnected = (e) => this._handleGamepadConnected(e);
        this._onDisconnected = (e) => this._handleGamepadDisconnected(e);
        window.addEventListener('gamepadconnected', this._onConnected);
        window.addEventListener('gamepaddisconnected', this._onDisconnected);

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

        this.gamepadIndex = index;
        STATE.joystickConnected = true;

        // Build axis map for this gamepad's axes count
        const axisCount = gp.axes.length;
        this.rawAxisValues = new Array(axisCount).fill(0);

        // Preserve existing mappings, fill defaults for new axes
        const newMap = [];
        for (let i = 0; i < axisCount; i++) {
            if (this.axisMap[i]) {
                newMap.push({ ...this.axisMap[i] });
            } else if (DEFAULT_AXIS_MAP[i]) {
                newMap.push({ ...DEFAULT_AXIS_MAP[i] });
            } else {
                newMap.push({ channel: 0, inverted: false, deadzone: 0.05, expo: 0.0 });
            }
        }
        this.axisMap = newMap;
        this.saveConfig();
        return true;
    }

    /**
     * Enable joystick - start polling and sending
     * Disables RC throttle failsafe so ArduPilot won't disarm without a physical receiver
     */
    async enable() {
        if (this.gamepadIndex === null) return false;
        this.enabled = true;
        STATE.joystickEnabled = true;

        // Configure ArduPilot to accept RC overrides from this GCS
        try {
            // Set SYSID_MYGCS to match our GCS sysid (254)
            // ArduPilot ONLY accepts RC_CHANNELS_OVERRIDE from this sysid
            this._savedSysidMygcs = (STATE.parameters.get('SYSID_MYGCS') || {}).value;
            await setParameter('SYSID_MYGCS', 254);
            console.log('[Joystick] Set SYSID_MYGCS = 254');
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
            // Clear bit 8 (256) in RC_OPTIONS to allow overrides
            this._savedRcOptions = (STATE.parameters.get('RC_OPTIONS') || {}).value;
            const current = this._savedRcOptions || 0;
            const cleared = current & ~256; // Clear "Ignore Overrides" bit
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

        this._startPolling();
        this._startSending();
        return true;
    }

    /**
     * Disable joystick - stop polling, release all channels
     * Restores RC throttle failsafe
     */
    disable() {
        this.enabled = false;
        STATE.joystickEnabled = false;
        STATE.rcOverrideActive = false;
        this._stopSending();
        this._stopPolling();
        // Release all channels
        this._releaseAllChannels();
        this.channelValues.fill(RELEASE);

        // Restore original parameters
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
        this.sendRateHz = hz;
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
        Object.assign(this.axisMap[axisIndex], config);
        this.saveConfig();
    }

    // --- Polling ---
    // Uses setInterval instead of requestAnimationFrame so gamepad polling
    // runs at a guaranteed fixed rate, independent of the 3D render loop.
    // This ensures RC input is never dropped or delayed by heavy rendering.

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
        if (!this.enabled) return;

        const gamepads = navigator.getGamepads();
        const gp = gamepads[this.gamepadIndex];

        if (!gp) {
            // Gamepad API can return null momentarily (focus loss, GC, etc.)
            // Don't immediately disconnect - let the 500ms failsafe in _sendOverride handle it
            console.warn('[Joystick] Gamepad read returned null (transient)');
            return;
        }

        this.lastGamepadTimestamp = Date.now();

        // Reset channel values
        this.channelValues.fill(RELEASE);

        // Process each axis
        for (let i = 0; i < gp.axes.length && i < this.axisMap.length; i++) {
            let value = gp.axes[i];
            this.rawAxisValues[i] = value;

            const cfg = this.axisMap[i];
            if (cfg.channel === 0) continue; // Unmapped

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

        // Update callback for UI
        if (this.onUpdate) this.onUpdate();
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

        // Failsafe: if no gamepad data for 500ms, release
        if (Date.now() - this.lastGamepadTimestamp > 500) {
            this._releaseAllChannels();
            this.disable();
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
        if (this.onUpdate) this.onUpdate();
    }

    _handleGamepadDisconnected(e) {
        console.log('[Joystick] Gamepad disconnected:', e.gamepad.id);
        if (e.gamepad.index === this.gamepadIndex) {
            this._handleLostGamepad();
        }
    }

    _handleLostGamepad() {
        this.disable();
        this.gamepadIndex = null;
        STATE.joystickConnected = false;
        if (this.onUpdate) this.onUpdate();
    }

    // --- Persistence ---

    saveConfig() {
        const config = {
            gamepadIndex: this.gamepadIndex,
            sendRateHz: this.sendRateHz,
            axisMap: this.axisMap
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
            if (config.sendRateHz) this.sendRateHz = config.sendRateHz;
            if (Array.isArray(config.axisMap)) this.axisMap = config.axisMap;
        } catch (e) { /* ignore */ }
    }

    /**
     * Cleanup
     */
    destroy() {
        this.disable();
        window.removeEventListener('gamepadconnected', this._onConnected);
        window.removeEventListener('gamepaddisconnected', this._onDisconnected);
    }
}
