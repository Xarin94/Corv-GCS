/** The legacy camera stays mutable for orbit/FPV controls; capture after controls. */
const position = [0, 0, 0], quaternion = [0, 0, 0, 1], viewport = [1, 1, 1];
let worldQuaternion;
export function captureThreeCamera(camera, target, renderer) {
    camera.updateMatrixWorld();
    const m = camera.matrixWorld.elements;
    position[0] = m[12]; position[1] = m[13]; position[2] = m[14];
    worldQuaternion ||= camera.quaternion.clone();
    camera.getWorldQuaternion(worldQuaternion);
    quaternion[0] = worldQuaternion.x; quaternion[1] = worldQuaternion.y;
    quaternion[2] = worldQuaternion.z; quaternion[3] = worldQuaternion.w;
    if (renderer) {
        viewport[0] = renderer.domElement.width / renderer.getPixelRatio();
        viewport[1] = renderer.domElement.height / renderer.getPixelRatio();
        viewport[2] = renderer.getPixelRatio();
    }
    target.set({ position, quaternion, viewport, projection: camera.projectionMatrix.elements,
        view: camera.matrixWorldInverse.elements, world: m, near: camera.near, far: camera.far, fov: camera.fov });
}
