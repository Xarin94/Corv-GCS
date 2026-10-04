/** Livox controller: coordinate transforms and data updates, independent of Three resources. */
import { STATE } from '../core/state.js';
import { ORIGIN } from '../core/constants.js';
import { latLonToMeters } from '../core/utils.js';
import { PointCloudData, setYXZQuaternion } from '../render/PointCloudData.js';
import { renderWorld } from '../render/RenderState.js';

let cloud = null, view = null;
export function initLidarCloud(scene, renderer, backend) {
    if (!backend?.createPointCloudView) throw new Error('LiDAR requires a rendering backend');
    view?.dispose();
    cloud = new PointCloudData();
    renderWorld.pointClouds.set('livox', cloud);
    view = backend.createPointCloudView({ scene, model: cloud });
}
export function setLidarOrigin(origin) {
    if (!cloud || !origin) return;
    cloud.setOrigin(origin);
    const p = latLonToMeters(origin.lat, origin.lon), t = cloud.mapTransform;
    t.position[0] = p.x; t.position[1] = origin.alt + (STATE.offsetAlt || 0); t.position[2] = p.z;
    t.scale[0] = 111320 * Math.cos(ORIGIN.lat * Math.PI / 180) / origin.mPerLon;
    t.scale[1] = 1; t.scale[2] = 111320 / origin.mPerLat;
    view.updateFrame();
}
export function appendLidarPoints(batch) {
    if (!cloud || !batch?.enu) return;
    view.applyMapped(cloud.appendMapped(batch));
}
export function appendLiveLidarPoints(batch) {
    if (!cloud || !batch?.xyz) return;
    view.applyLive(cloud.appendLive(batch, performance.now() / 1000));
}
export function clearLiveLidarPoints() { if (cloud) { cloud.clearLive(); view.updateFrame(); } }
export function clearLidarCloud() { if (cloud) { cloud.clear(); view.clearHistory(); } }
export function setLidarCloudVisible(value) {
    if (!cloud) return;
    cloud.visible = !!value;
    if (!cloud.visible) cloud.clearLive();
    view.updateFrame();
}
export function setLidarColorMode(value) {
    if (cloud) { cloud.colorMode = value === 'intensity' ? 'intensity' : 'height'; view.updateFrame(); }
}
export function setLidarPointSize(px) {
    if (cloud) { cloud.pointSize = Math.max(1, px || 2); view.updateFrame(); }
}
export function setLidarMaxPoints(n) { if (cloud) cloud.maxPoints = Math.max(1000, n | 0); }
export function setLidarOverTerrain(value) { if (cloud) { cloud.overTerrain = !!value; view.updateFrame(); } }
export function getLidarPointCount() { return cloud?.total || 0; }
export function updateLidarCloud() {
    if (!cloud?.visible) return;
    if (cloud.live.count > 0) {
        const p = latLonToMeters(STATE.lat, STATE.lon), t = cloud.liveTransform;
        t.position[0] = p.x; t.position[1] = Math.max(STATE.rawAlt + (STATE.offsetAlt || 0), 1); t.position[2] = p.z;
        if (cloud.live.frame === 'body') setYXZQuaternion(t.quaternion, STATE.pitch, -STATE.yaw, -STATE.roll);
        else { t.quaternion[0] = t.quaternion[1] = t.quaternion[2] = 0; t.quaternion[3] = 1; }
    }
    if (cloud.origin) {
        cloud.mapTransform.position[1] = cloud.origin.alt + (STATE.offsetAlt || 0);
        cloud.updateRange(performance.now());
    }
    view.updateFrame();
}
