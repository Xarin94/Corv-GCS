/**
 * dialog-dirs.js - Folder each open dialog starts in
 *
 * Since Electron 43 a dialog without defaultPath opens in Downloads instead of
 * the folder Windows remembered for the app. The last folder of each kind of
 * dialog is kept here instead, per user, in <userData>/dialog-dirs.json as
 * { kind: dir }; a folder that no longer exists falls back to the caller's.
 */

const { app } = require('electron');
const fs = require('fs');
const path = require('path');

let dirs = null;

function dirsFile() {
    return path.join(app.getPath('userData'), 'dialog-dirs.json');
}

function load() {
    if (!dirs) {
        try { dirs = JSON.parse(fs.readFileSync(dirsFile(), 'utf8')) || {}; } catch (e) { dirs = {}; }
    }
    return dirs;
}

/**
 * Folder to open a dialog of this kind in.
 * @param {string} kind
 * @param {string} [fallback]  used until a file has been picked (undefined = Downloads)
 */
function lastDir(kind, fallback) {
    const dir = load()[kind];
    return dir && fs.existsSync(dir) ? dir : fallback;
}

/** Remember the folder of the file just picked in a dialog of this kind */
function rememberDir(kind, filePath) {
    load()[kind] = path.dirname(filePath);
    try { fs.writeFileSync(dirsFile(), JSON.stringify(dirs, null, 2)); } catch (e) { /* kept for this session */ }
}

module.exports = { lastDir, rememberDir };
