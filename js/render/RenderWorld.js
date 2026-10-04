import { CameraData } from './CameraData.js';
import { TerrainData } from './TerrainData.js';

export const RENDER_SCHEMA_VERSION = 1;
export class RenderWorld {
    constructor(origin) {
        this.schemaVersion = RENDER_SCHEMA_VERSION;
        this.origin = { ...origin };
        this.axes = 'east-up-south';
        this.units = 'metres';
        this.camera = new CameraData();
        this.terrain = new TerrainData(origin);
        this.pointClouds = new Map();
        this.terrainStyle = { visible: true, schematic: false, light: false, brightness: 0.85,
            sunlight: true, sunDirection: [0, 1, 0], schematicRadius: 30000,
            gridStrength: 0, gridCenter: { x: 0, y: 0 }, subWater: [0, 0, 0, 0] };
    }
    getStats() {
        return { schemaVersion: this.schemaVersion, cameraRevision: this.camera.revision,
            terrain: this.terrain.getStats(), pointClouds: this.pointClouds.size };
    }
}
