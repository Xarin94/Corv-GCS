/**
 * InavTransfer.js - The INAV mission channel (MSP), renderer side
 *
 * The mirror of MissionTransfer.js for the other protocol. MAVLink mission
 * transfer is a conversation the renderer can hold on its own; MSP is not —
 * there is one request in flight at a time and the scheduler that owns the
 * link lives in the main process, so the whole transfer happens there and this
 * module is only the doorway to it (msp-manager.js: missionInfo / missionUpload
 * / missionDownload).
 *
 * Progress arrives as events rather than as promise results, because a 60
 * waypoint upload over a 57k radio link is slow enough that the button has to
 * count.
 */

let progressCb = null;
let bound = false;

function bridge() {
    if (!window.msp) throw new Error('MSP bridge not available');
    return window.msp;
}

function bindProgress() {
    if (bound) return;
    bound = true;
    window.msp?.onMissionProgress?.((p) => { if (progressCb) progressCb(p); });
}

/** What the board can hold and what it holds right now. */
export async function inavMissionInfo() {
    return bridge().missionInfo();
}

/**
 * Write a waypoint list to the flight controller.
 * @param {Array} wps      records from InavMission.toInavMission()
 * @param {object} opts    { save:boolean } — also store it in the board EEPROM
 * @param {(p:{phase:string,done:number,total:number})=>void} onProgress
 */
export async function uploadInavMission(wps, opts = {}, onProgress = null) {
    if (!wps?.length) throw new Error('No waypoints to upload');
    bindProgress();
    progressCb = onProgress;
    try {
        return await bridge().missionUpload(wps, { save: !!opts.save });
    } finally {
        progressCb = null;
    }
}

/**
 * Read the waypoint list stored on the flight controller.
 * @returns {Promise<{wps:Array, count:number, maxWaypoints:number, valid:boolean}>}
 */
export async function downloadInavMission(onProgress = null) {
    bindProgress();
    progressCb = onProgress;
    try {
        return await bridge().missionDownload();
    } finally {
        progressCb = null;
    }
}
