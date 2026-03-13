/**
 * sitl-manager.js - ArduPilot SITL launcher for the Electron main process
 * Downloads pre-built SITL binaries from firmware.ardupilot.org and manages the SITL process.
 */

const { ipcMain, app } = require('electron');
const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');
const https = require('https');
const http = require('http');

// SITL binary info per vehicle type
const VEHICLE_MAP = {
    copter:  { binary: 'arducopter',  model: 'quad' },
    plane:   { binary: 'arduplane',   model: 'plane' },
    rover:   { binary: 'ardurover',   model: 'rover' },
    sub:     { binary: 'ardusub',     model: 'vectored' },
    heli:    { binary: 'arducopter',  model: 'heli' },
    quadplane: { binary: 'arduplane', model: 'quadplane' }
};

const VERSIONS = ['stable', 'beta', 'latest'];

const isWindows = process.platform === 'win32';

function getFirmwareUrl(vehicle, version) {
    const info = VEHICLE_MAP[vehicle];
    if (!info) return null;
    const pathMap = {
        copter: 'Copter', plane: 'Plane', rover: 'Rover', sub: 'Sub',
        heli: 'Copter', quadplane: 'Plane'
    };
    const fwPath = pathMap[vehicle] || 'Copter';
    // ArduPilot only provides Linux x86_64 SITL binaries
    // On Windows we download the same binary and run it via WSL
    return `https://firmware.ardupilot.org/${fwPath}/${version}/SITL_x86_64_linux_gnu/${info.binary}`;
}

// Storage directory for SITL binaries
function getSitlDir() {
    const dir = path.join(app.getPath('userData'), 'sitl');
    if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
    return dir;
}

function getBinaryPath(vehicle, version) {
    const info = VEHICLE_MAP[vehicle];
    if (!info) return null;
    // Always Linux binary (no .exe), even on Windows (runs via WSL)
    return path.join(getSitlDir(), `${info.binary}_${version}`);
}

// State
let sitlProcess = null;
let mainWindow = null;

/**
 * Download a file with redirect support
 */
function downloadFile(url, destPath, onProgress) {
    return new Promise((resolve, reject) => {
        const file = fs.createWriteStream(destPath);
        const doRequest = (reqUrl) => {
            const mod = reqUrl.startsWith('https') ? https : http;
            mod.get(reqUrl, (response) => {
                // Handle redirects
                if (response.statusCode >= 300 && response.statusCode < 400 && response.headers.location) {
                    file.close();
                    fs.unlinkSync(destPath);
                    const newFile = fs.createWriteStream(destPath);
                    const redirectUrl = response.headers.location;
                    const mod2 = redirectUrl.startsWith('https') ? https : http;
                    mod2.get(redirectUrl, (res2) => {
                        if (res2.statusCode !== 200) {
                            newFile.close();
                            fs.unlinkSync(destPath);
                            reject(new Error(`HTTP ${res2.statusCode}`));
                            return;
                        }
                        const total = parseInt(res2.headers['content-length'] || '0');
                        let downloaded = 0;
                        res2.on('data', (chunk) => {
                            downloaded += chunk.length;
                            if (onProgress && total > 0) onProgress(downloaded, total);
                        });
                        res2.pipe(newFile);
                        newFile.on('finish', () => { newFile.close(); resolve(); });
                    }).on('error', (e) => { newFile.close(); fs.unlinkSync(destPath); reject(e); });
                    return;
                }
                if (response.statusCode !== 200) {
                    file.close();
                    fs.unlinkSync(destPath);
                    reject(new Error(`HTTP ${response.statusCode}`));
                    return;
                }
                const total = parseInt(response.headers['content-length'] || '0');
                let downloaded = 0;
                response.on('data', (chunk) => {
                    downloaded += chunk.length;
                    if (onProgress && total > 0) onProgress(downloaded, total);
                });
                response.pipe(file);
                file.on('finish', () => { file.close(); resolve(); });
            }).on('error', (e) => { file.close(); fs.unlinkSync(destPath); reject(e); });
        };
        doRequest(url);
    });
}

/**
 * Initialize SITL IPC handlers
 */
function initSITLHandlers(win) {
    mainWindow = win;

    // Get available vehicle types and versions
    ipcMain.handle('sitl-get-options', () => {
        return {
            vehicles: Object.keys(VEHICLE_MAP),
            versions: VERSIONS
        };
    });

    // Check if a SITL binary is already downloaded
    ipcMain.handle('sitl-check-binary', (event, vehicle, version) => {
        const binPath = getBinaryPath(vehicle, version);
        return binPath && fs.existsSync(binPath);
    });

    // Download SITL binary
    ipcMain.handle('sitl-download', async (event, vehicle, version) => {
        const url = getFirmwareUrl(vehicle, version);
        if (!url) throw new Error(`Unknown vehicle: ${vehicle}`);

        const binPath = getBinaryPath(vehicle, version);
        console.log(`[sitl] Downloading ${vehicle}/${version} from ${url}`);

        sendSitlStatus('DOWNLOADING', `Downloading ${vehicle} ${version}...`);

        await downloadFile(url, binPath, (downloaded, total) => {
            const pct = Math.round((downloaded / total) * 100);
            sendSitlStatus('DOWNLOADING', `Downloading... ${pct}%`);
        });

        // Make executable
        fs.chmodSync(binPath, 0o755);
        console.log(`[sitl] Downloaded to ${binPath}`);
        sendSitlStatus('READY', `${vehicle} ${version} ready`);
        return true;
    });

    // Launch SITL
    ipcMain.handle('sitl-launch', async (event, vehicle, version, options = {}) => {
        // Kill existing SITL if running
        await killSitl();

        const info = VEHICLE_MAP[vehicle];
        if (!info) throw new Error(`Unknown vehicle: ${vehicle}`);

        const binPath = getBinaryPath(vehicle, version);
        if (!binPath || !fs.existsSync(binPath)) {
            throw new Error('SITL binary not found. Download it first.');
        }

        const model = options.model || info.model;
        const homeLat = options.homeLat || 47.2603;
        const homeLon = options.homeLon || 11.3439;
        const homeAlt = options.homeAlt || 0;
        const homeHdg = options.homeHdg || 0;
        const speedup = options.speedup || 1;

        const args = [
            '--model', model,
            '--home', `${homeLat},${homeLon},${homeAlt},${homeHdg}`,
            '--speedup', String(speedup),
            '-I0'
        ];

        // Add default params file if it exists
        const defaultParams = path.join(getSitlDir(), `default_params_${vehicle}.parm`);
        if (fs.existsSync(defaultParams)) {
            args.push('--defaults', defaultParams);
        }

        sendSitlStatus('LAUNCHING', `Starting ${vehicle} SITL...`);

        let spawnCmd, spawnArgs, spawnOpts;
        if (isWindows) {
            // On Windows: run Linux binary via WSL
            // Convert Windows path to WSL path: C:\Users\... -> /mnt/c/Users/...
            const wslBin = binPath.replace(/\\/g, '/').replace(/^([A-Za-z]):/, (_, d) => `/mnt/${d.toLowerCase()}`);
            const wslCwd = getSitlDir().replace(/\\/g, '/').replace(/^([A-Za-z]):/, (_, d) => `/mnt/${d.toLowerCase()}`);

            // Resolve Windows host IP from WSL before launching
            // (doing it in Node to avoid shell escaping issues with $)
            let winHostIp = '127.0.0.1';
            try {
                const { execSync } = require('child_process');
                const resolv = execSync('wsl cat /etc/resolv.conf', { encoding: 'utf8', timeout: 5000 });
                const match = resolv.match(/nameserver\s+(\d+\.\d+\.\d+\.\d+)/);
                if (match) winHostIp = match[1];
            } catch (e) {
                console.log('[sitl] Could not resolve WSL host IP, using 127.0.0.1');
            }
            console.log(`[sitl] Windows host IP for WSL: ${winHostIp}`);

            // Copy binary to native WSL dir (/tmp/sitl) to avoid /mnt/c permission issues
            // SITL needs to write eeprom.bin and log files in its working directory
            const wslRunDir = '/tmp/sitl_run';
            // On Windows/WSL we connect to serial0 (TCP 5760) which IS a MAVLink port.
            // serial0 is what MAVProxy also connects to - it speaks MAVLink, not console.
            // --wipe clears eeprom.bin to avoid crashes from corrupted state
            const allArgs = [...args, '--wipe'];
            const bashScript = [
                `mkdir -p ${wslRunDir}`,
                `cp "${wslBin}" ${wslRunDir}/sitl_bin`,
                `chmod +x ${wslRunDir}/sitl_bin`,
                `cd ${wslRunDir}`,
                `./sitl_bin ${allArgs.join(' ')} 2>&1`
            ].join(' && ');
            spawnCmd = 'wsl';
            spawnArgs = ['bash', '-c', bashScript];
            spawnOpts = { stdio: ['pipe', 'pipe', 'pipe'] };
            console.log(`[sitl] Launching via WSL (native dir): ${allArgs.join(' ')}`);
        } else {
            // On Linux: run directly, connect via TCP serial0 (port 5760)
            spawnCmd = binPath;
            spawnArgs = args;
            spawnOpts = { cwd: getSitlDir(), stdio: ['pipe', 'pipe', 'pipe'] };
            console.log(`[sitl] Launching: ${binPath} ${args.join(' ')}`);
        }

        sitlProcess = spawn(spawnCmd, spawnArgs, spawnOpts);

        sitlProcess.stdout.on('data', (data) => {
            const text = data.toString().trim();
            if (text) console.log(`[sitl:stdout] ${text}`);
        });

        sitlProcess.stderr.on('data', (data) => {
            const text = data.toString().trim();
            if (text) console.log(`[sitl:stderr] ${text}`);
        });

        sitlProcess.on('error', (err) => {
            console.error(`[sitl] Process error:`, err.message);
            sendSitlStatus('ERROR', err.message);
            sitlProcess = null;
        });

        sitlProcess.on('exit', (code, signal) => {
            console.log(`[sitl] Process exited: code=${code} signal=${signal}`);
            sendSitlStatus('STOPPED', `SITL stopped (code ${code})`);
            sitlProcess = null;
        });

        // Wait for SITL to fully initialize (TCP listeners need time to bind)
        await new Promise(r => setTimeout(r, 5000));
        if (sitlProcess && !sitlProcess.killed) {
            sendSitlStatus('RUNNING', `${vehicle} SITL running`);
            if (isWindows) {
                // On Windows/WSL: connect via TCP to SITL's serial0 on port 5760
                // serial0 speaks MAVLink (same port MAVProxy connects to)
                return { success: true, connectionType: 'mavlink-tcp', host: '127.0.0.1', port: 5760 };
            } else {
                // On Linux: connect via TCP to SITL's serial0 on port 5760
                return { success: true, connectionType: 'mavlink-tcp', host: '127.0.0.1', port: 5760 };
            }
        } else {
            throw new Error('SITL process failed to start');
        }
    });

    // Stop SITL
    ipcMain.handle('sitl-stop', async () => {
        await killSitl();
        return true;
    });

    // Get SITL status
    ipcMain.handle('sitl-status', () => {
        return {
            running: sitlProcess !== null && !sitlProcess.killed
        };
    });
}

async function killSitl() {
    if (sitlProcess && !sitlProcess.killed) {
        console.log('[sitl] Killing SITL process');
        sitlProcess.kill('SIGTERM');
        await new Promise(r => setTimeout(r, 500));
        if (sitlProcess && !sitlProcess.killed) {
            sitlProcess.kill('SIGKILL');
        }
    }
    // On Windows, also kill any lingering SITL processes inside WSL
    if (isWindows) {
        try {
            const { execSync } = require('child_process');
            execSync('wsl bash -c "pkill -9 -f sitl_bin 2>/dev/null; pkill -9 -f arducopter 2>/dev/null"', { timeout: 3000 });
        } catch (e) { /* ignore */ }
    }
    sitlProcess = null;
    sendSitlStatus('STOPPED', 'SITL stopped');
}

function sendSitlStatus(state, message) {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('sitl-status-update', { state, message });
    }
}

function cleanup() {
    if (sitlProcess && !sitlProcess.killed) {
        sitlProcess.kill('SIGKILL');
        sitlProcess = null;
    }
}

module.exports = { initSITLHandlers, cleanup: cleanup };
