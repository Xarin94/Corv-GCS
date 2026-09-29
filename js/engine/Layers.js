/**
 * Layers.js - Render layers of the 3D scene
 *
 * The schematic view renders in passes (see Scene3D.renderSchematic): the
 * world — terrain, runways, vehicle — first, so its depth can be outlined,
 * then the overlays on top. Overlays sit on their own layer so they are
 * neither outlined nor hidden by the outline pass: trail, route, waypoint and
 * home symbols, labels, the predicted corridor, target and traffic markers,
 * the LiDAR cloud. Layers are not inherited: every drawn object sets its own.
 */

export const WORLD_LAYER = 0;
export const OVERLAY_LAYER = 1;

/** Put an object and everything under it on the overlay layer. */
export function toOverlayLayer(object) {
    object.traverse(o => o.layers.set(OVERLAY_LAYER));
    return object;
}
