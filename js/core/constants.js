/**
 * constants.js - Application Constants and Configuration
 * All global constants, settings, and initial values
 */

export const ORIGIN = { lat: 47.2603, lon: 11.3439 };
export const CAMERA_FOV = 60;
// 35 km: oltre questa distanza il FogExp2 della scena (densità 0.00005) lascia
// visibile meno del ~5% del terreno — generare chunk più lontani è lavoro sprecato.
export const VISIBILITY_RADIUS = 35000;
export const RELOAD_DISTANCE = 5000;
export const RAD = 180 / Math.PI;

// Chunk system
export const CHUNKS_PER_FRAME = 3;
export const CLEANUP_RADIUS = VISIBILITY_RADIUS * 1.2;
export const MAX_ACTIVE_CHUNKS = 80;
export const SHADOW_CHUNK_SIZE = 5000;

// Tile system - zoom levels defined in TerrainManager.js (dual-zoom LOD)
export const MAX_CONCURRENT_LOADS = 6;

// Demo mode - fixed-wing survey drone
export const DEMO_TARGET_INTERVAL = 12000;
export const DEMO_SMOOTHING = 0.005;
export const DEMO_BASE_SPEED = 25;       // m/s typical mapping drone
export const DEMO_SPEED_VARIANCE = 3;    // m/s
export const DEMO_ALT_AGL = 120;         // meters above ground level
export const DEMO_PITCH_RANGE = 0.07;    // ~4 degrees
export const DEMO_ROLL_RANGE = 0.26;     // ~15 degrees
export const DEMO_LEG_LENGTH = 800;      // meters before turning
export const DEMO_LEG_SPACING = 60;      // meters between survey legs

// Loading overlay
export const INITIAL_MIN_VISIBLE_MS = 600;
export const POST_COMPLETE_MS = 3000;

