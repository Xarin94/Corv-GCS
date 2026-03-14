/**
 * constants.js - Application Constants and Configuration
 * All global constants, settings, and initial values
 */

export const ORIGIN = { lat: 47.2603, lon: 11.3439 };
export const CAMERA_FOV = 60;
export const VISIBILITY_RADIUS = 50000;
export const RELOAD_DISTANCE = 5000;
export const RAD = 180 / Math.PI;

// Chunk system
export const CHUNKS_PER_FRAME = 3;
export const CLEANUP_RADIUS = VISIBILITY_RADIUS * 1.2;
export const MAX_ACTIVE_CHUNKS = 80;
export const SHADOW_CHUNK_SIZE = 5000;

// Tile system - zoom levels defined in TerrainManager.js (dual-zoom LOD)
export const MAX_CONCURRENT_LOADS = 6;

// Plotly
export const BUFFER_SIZE = 1200;
export const SAMPLE_INTERVAL = 100; // ms

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

// Plotly trace configuration
export const TRACE_CONFIG = {
    as:     { name: 'IAS (m/s)', color: '#00FF00', yaxis: 'y' },
    gs:     { name: 'GS (m/s)', color: '#00CC00', yaxis: 'y' },
    vs:     { name: 'VS (m/s)', color: '#00FFFF', yaxis: 'y' },
    rawAlt: { name: 'Alt (m)', color: '#FFFF00', yaxis: 'y2' },
    roll:   { name: 'Roll (°)', color: '#FF00FF', yaxis: 'y3' },
    pitch:  { name: 'Pitch (°)', color: '#FF66FF', yaxis: 'y3' },
    az:     { name: 'G-Load', color: '#FF3300', yaxis: 'y' }
};
