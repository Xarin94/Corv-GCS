#!/usr/bin/env node
/**
 * test-ekf-monitor.js - offline checks for js/core/EkfMonitor.js
 *
 * Feeds the EKF STATUS data module the messages an ArduPilot sends and the
 * packets a CORV board sends (built byte by byte from BINARY_TELEMETRY.md),
 * and checks the series the charts draw: units, the EKF − DCM comparison at
 * the same instant, the GPS residual at the same board time, resample counts
 * across the counter wrap, the 20 s window and the source switch.
 *
 *   node scripts/test-ekf-monitor.js
 */
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { pathToFileURL } = require('node:url');

const root = path.join(__dirname, '..');
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-ekf-'));
fs.writeFileSync(path.join(tmp, 'package.json'), '{"type":"module"}');
fs.copyFileSync(path.join(root, 'js/core/EkfMonitor.js'), path.join(tmp, 'EkfMonitor.js'));

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}
const near = (a, b, tol) => a !== null && Math.abs(a - b) <= tol;
const RAD = Math.PI / 180;

/** CORV Debug payload (92 B). */
function debugPacket({ gyroBias = [0.01, -0.02, 0.005], magQ = 0.9, hAcc = 1.2, vAcc = 2.5, ess = 300, spread = 1.8, resamples = 0, n = 600 } = {}) {
    const b = Buffer.alloc(92);
    gyroBias.forEach((v, i) => b.writeFloatLE(v, 4 + i * 4));
    b.writeFloatLE(magQ, 32); b.writeFloatLE(hAcc, 56); b.writeFloatLE(vAcc, 60);
    b.writeFloatLE(ess, 76); b.writeFloatLE(spread, 80);
    b.writeUInt16LE(resamples, 84); b.writeUInt16LE(n, 86);
    return Array.from(b);
}

/** CORV Raw Sensor payload (74 B). */
function rawPacket({ ts, lat, lon, alt = 100, fix = 3, hAcc = 1.5 }) {
    const b = Buffer.alloc(74);
    b.writeUInt32LE(ts, 0); b.writeDoubleLE(lat, 4); b.writeDoubleLE(lon, 12); b.writeFloatLE(alt, 20);
    b.writeUInt8(fix, 36); b.writeUInt8(12, 37); b.writeFloatLE(hAcc, 38);
    return Array.from(b);
}

(async () => {
    const { EkfMonitor, EKF_WINDOW_MS } = await import(pathToFileURL(path.join(tmp, 'EkfMonitor.js')));

    // ---------- ArduPilot ----------
    {
        const m = new EkfMonitor();
        check('no data: no source', m.source(0) === null && !m.live(0));
        m.ingestMavlink(193, { flags: 0x37f, velocityVariance: 0.12, posHorizVariance: 0.3, posVertVariance: 0.05, compassVariance: 0.2, terrainAltVariance: 0, airspeedVariance: 0 }, 1000);
        check('EKF_STATUS_REPORT: ArduPilot source, ratios stored', m.source(1000) === 'ardupilot' && m.get('apPosH').last() === 0.3 && m.ap.flags === 0x37f);
        check('no airspeed fusion: TAS not drawn', m.get('apTas').length === 0);
        m.ingestMavlink(193, { flags: 0x37f, velocityVariance: 0.1, posHorizVariance: 0.3, posVertVariance: 0.05, compassVariance: 0.2, airspeedVariance: 0.4 }, 1200);
        check('airspeed fusion: TAS drawn', m.get('apTas').last() === 0.4);

        m.ingestMavlink(24, { fixType: 3, satellitesVisible: 14, eph: 80, hAcc: 1500, vAcc: 2200, velAcc: 300 }, 1300);
        check('GPS_RAW_INT: accuracies in metres', m.get('apHAcc').last() === 1.5 && m.get('apVAcc').last() === 2.2 && m.get('apVelAcc').last() === 0.3 && m.ap.hdop === 0.8);
        m.ingestMavlink(24, { fixType: 3, satellitesVisible: 14, eph: 80, hAcc: 0, vAcc: 0, velAcc: 0 }, 1400);
        check('GPS_RAW_INT: accuracy 0 = not reported, not plotted', m.get('apHAcc').length === 1);
        m.ingestMavlink(24, { fixType: 1, satellitesVisible: 0, eph: 9999, hAcc: 300, vAcc: 300, velAcc: 0 }, 1450);
        check('GPS_RAW_INT: no fix, the last accuracy it quotes is not plotted', m.get('apHAcc').length === 1);

        // EKF ATTITUDE at 0 and 40 ms, rolling 100°/s; DCM at 20 ms one degree behind
        m.ingestMavlink(30, { roll: 0, pitch: 0, yaw: 359 * RAD }, 2000);
        m.ingestMavlink(178, { roll: 1 * RAD, pitch: 0, yaw: 1 * RAD }, 2020);
        check('AHRS2 waits for the next ATTITUDE', m.get('apDRoll').length === 0);
        m.ingestMavlink(30, { roll: 4 * RAD, pitch: 0, yaw: 359 * RAD }, 2040);
        check('EKF − DCM interpolated at the AHRS2 instant', near(m.get('apDRoll').last(), 1, 1e-9), m.get('apDRoll').last().toFixed(3) + '° (nearest sample alone: 3° or -1°)');
        check('EKF − DCM yaw across north', near(m.get('apDYaw').last(), -2, 1e-9), m.get('apDYaw').last().toFixed(3) + '°');
        m.ingestMavlink(178, { roll: 0, pitch: 0, yaw: 0 }, 2100);
        m.ingestMavlink(178, { roll: 0, pitch: 0, yaw: 0 }, 2500);
        check('ATTITUDE stream gone: AHRS2 compared with the latest one', m.get('apDRoll').length === 2);

        m.ingestMavlink(241, { vibrationX: 12, vibrationY: 15, vibrationZ: 31, clipping0: 0, clipping1: 2, clipping2: 0 }, 2600);
        check('VIBRATION kept for the chips', m.ap.vibe[2] === 31 && m.ap.clip === 2);

        check('silent 3 s: shown as not live, still ArduPilot', m.source(6000) === 'ardupilot' && !m.live(6000));
        m.ingestMavlink(193, { flags: 0x37f, velocityVariance: 0.1, posHorizVariance: 0.3, posVertVariance: 0.05, compassVariance: 0.2 }, 1000 + EKF_WINDOW_MS + 2500);
        check('20 s window: older samples dropped', m.get('apPosH').t[0] > 1000);
    }

    // ---------- CORV ----------
    {
        const m = new EkfMonitor();
        const lat0 = 45, lon0 = 10, mLon = 111320 * Math.cos(lat0 * RAD);
        m.ingestCorvNav({ ts: 10000, lat: lat0, lon: lon0, alt: 100, confidence: 0.92, covTrace: 0.004, flags: 0xff, fixType: 3, numSat: 15, hdop: 0.7 }, 500);
        check('Navigation: CORV source, confidence in %', m.source(500) === 'corv' && near(m.get('corvConf').last(), 92, 1e-4) && m.get('corvTrP').last() === 0.004);
        m.ingestCorvNav({ ts: 10040, lat: lat0 + 4 / 111320, lon: lon0, alt: 101, confidence: 0.92, covTrace: 0.004, flags: 0xff, fixType: 3 }, 540);
        // Raw fix at 10020: the filter is 2 m north there; the GPS says 2 m north and 3 m east
        m.ingestCorvRaw(rawPacket({ ts: 10020, lat: lat0 + 2 / 111320, lon: lon0 + 3 / mLon, alt: 104.5 }), 560);
        check('GPS − PF at the same board time', near(m.get('corvResH').last(), 3, 0.01), m.get('corvResH').last().toFixed(3) + ' m');
        check('GPS − PF vertical', near(m.get('corvResV').last(), 4, 0.01));
        check('raw GPS accuracy', near(m.get('corvGpsH').last(), 1.5, 1e-6));
        m.ingestCorvRaw(rawPacket({ ts: 10020, lat: lat0, lon: lon0, fix: 1 }), 600);
        check('no fix: no residual', m.get('corvResH').length === 1);

        m.ingestCorvDebug(debugPacket({ ess: 300, n: 600, spread: 1.8, resamples: 65534 }), 1000);
        check('Debug: ESS / N in %, spread, GPS V accuracy', near(m.get('corvEss').last(), 50, 1e-6) && near(m.get('corvSpread').last(), 1.8, 1e-6) && near(m.get('corvGpsV').last(), 2.5, 1e-6));
        m.ingestCorvDebug(debugPacket({ ess: 540, n: 600, resamples: 2 }), 2000);
        check('resamples counted across the wrap', m.get('corvResample').last() === 4 && near(m.corv.resampleRate, 4, 1e-9), m.corv.resampleRate + '/s');
        check('gyro bias and magnetometer quality kept', near(m.corv.gyroBias[1], -0.02, 1e-6) && near(m.corv.magQuality, 0.9, 1e-6));

        m.ingestMavlink(193, { flags: 0x37f, velocityVariance: 0.1, posHorizVariance: 0.3, posVertVariance: 0.05, compassVariance: 0.2 }, 2100);
        check('CORV reporting: CORV charts even with MAVLink about', m.source(2100) === 'corv');
        check('CORV silent, ArduPilot live: ArduPilot charts', m.source(4500) === 'ardupilot');

        m.ingestCorvNav({ ts: 50, lat: lat0, lon: lon0, alt: 100, confidence: 0.5, covTrace: 0.01, flags: 0xff }, 5000);
        check('board rebooted: its old positions forgotten', m.nav.length === 1 && m.nav[0].ts === 50);
    }

    fs.rmSync(tmp, { recursive: true, force: true });
    console.log(failures ? `\n${failures} FAILED` : '\nall passed');
    process.exitCode = failures ? 1 : 0;
})().catch(error => { console.error(error); fs.rmSync(tmp, { recursive: true, force: true }); process.exitCode = 1; });
