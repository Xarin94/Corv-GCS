/**
 * lidar-worker.js - worker_threads host for lidar-core.js
 *
 * Message protocol with lidar-manager.js (main thread):
 *   main → worker  { id, cmd, args }          request (cmd = key of core.commands)
 *                  { mav: true, msgId, data } decoded MAVLink message (pose tap)
 *   worker → main  { id, result | error }     reply
 *                  { ch, d }                  event for the renderer ('lidar-points', ...)
 */

const { parentPort, workerData } = require('worker_threads');
const core = require('./lidar-core');

core.bind({
    dataRoot: workerData && workerData.dataRoot,
    emit: (ch, d, transfer) => parentPort.postMessage({ ch, d }, transfer || [])
});

parentPort.on('message', async (m) => {
    if (m.mav) { core.onMavlinkMessage(m.msgId, m.data); return; }
    const fn = core.commands[m.cmd];
    if (!fn) { parentPort.postMessage({ id: m.id, error: `unknown command ${m.cmd}` }); return; }
    try {
        const result = await fn(...(m.args || []));
        parentPort.postMessage({ id: m.id, result });
    } catch (e) {
        parentPort.postMessage({ id: m.id, error: e.message });
    }
});
