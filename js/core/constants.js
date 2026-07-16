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

// Demo mode - realistic fixed-wing patrol circuit (coordinated turns)
export const DEMO_CRUISE_SPEED = 28;              // m/s cruise airspeed
export const DEMO_SPEED_VARIANCE = 4;             // m/s gentle airspeed variation
export const DEMO_CRUISE_AGL = 150;               // m cruise height above circuit-centre terrain
export const DEMO_MIN_CLEARANCE = 90;             // m minimum terrain clearance (climb trigger)
export const DEMO_MAX_VS = 3.5;                   // m/s climb/descent limit — gentle terrain follow
export const DEMO_MAX_BANK = 28 * Math.PI / 180;  // rad, max bank in a coordinated turn
export const DEMO_ROLL_RATE = 22 * Math.PI / 180; // rad/s, max roll rate (aileron authority)
export const DEMO_HDG_GAIN = 2.2;                 // commanded bank per rad of heading error
export const DEMO_STRAIGHT = 1100;                // m straight-leg length of the racetrack
export const DEMO_TURN_RADIUS = 340;              // m radius of the racetrack turns
export const DEMO_CAPTURE_R = 130;                // m waypoint capture radius

// Loading overlay
export const INITIAL_MIN_VISIBLE_MS = 600;
export const POST_COMPLETE_MS = 3000;

