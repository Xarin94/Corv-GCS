# CORV GCS — Feature Description

> Version 1.2.25 | Ground Control Station for ArduPilot

---

## 1. Overview

CORV GCS is a desktop Ground Control Station for ArduPilot-based UAVs. It provides real-time 3D terrain visualization, flight instruments, mission planning, and full vehicle control via MAVLink v2. Built on Electron with Three.js, Leaflet, and Plotly.

---

## 2. Connection & Communication

### 2.1 MAVLink v2 Transport
- **Serial** — USB/radio modem (configurable baud rate)
- **UDP** — Network telemetry (default port 14550)
- **TCP** — Direct TCP connection (e.g., SITL on port 5760)
- **Legacy binary** — Custom CORV binary protocol via WebSerial (460800 baud)

### 2.2 GCS Identity
- **System ID:** 255 (Mission Planner compatible)
- **Component ID:** 190 (MAV_COMP_ID_MISSIONPLANNER)
- **Heartbeat:** 1 Hz, MAV_TYPE_GCS, MAV_STATE_ACTIVE
- **GCS Mute:** Toggle to suppress ALL outgoing messages (heartbeat, commands, RTK, RC override)

### 2.3 Home Position
- Automatically requested after connection and on vehicle arm
- Polls every 5 seconds until a valid home is received
- Stops polling once home position is set
- Re-requests on re-arm (home is reset at arm time)

### 2.4 Supported Vehicles
| Type | Mode Tables |
|------|------------|
| Copter | STABILIZE, ALT_HOLD, LOITER, GUIDED, AUTO, RTL, LAND, POSHOLD, etc. |
| Plane | MANUAL, FBWA, FBWB, CRUISE, AUTO, RTL, LOITER, GUIDED, etc. |
| Rover | MANUAL, HOLD, AUTO, GUIDED, RTL, LOITER, etc. |
| Sub | STABILIZE, ALT_HOLD, MANUAL, GUIDED, POSHOLD, etc. |
| QuadPlane | All Plane + VTOL modes (QSTABILIZE, QHOVER, QLOITER, QRTL) |

---

## 3. 3D Terrain Engine

### 3.1 Terrain Rendering
- **SRTM elevation data** — HGT files (SRTM1: 1 arc-second, ~30m resolution)
- **Chunked mesh generation** — 5000m x 5000m chunks via Web Workers
- **Visibility radius** — 50 km around aircraft position
- **Satellite textures** — Google Satellite imagery at zoom level 15
- **Dual-zoom LOD** — Higher detail near aircraft, lower detail at distance
- **Hillshading** — Solar position-based dynamic lighting and terrain shading
- **Frustum culling** — Only loads textures visible to the camera
- **LRU texture cache** — 1500 capacity, auto-evicts least-recently-used textures
- **Wireframe proximity** — Terrain mesh wireframe overlay near aircraft for depth reference

### 3.2 3D Scene Elements
- **Aircraft model** — GLB 3D model positioned by GPS + attitude
- **Flight trail** — 50,000-point BufferGeometry trail with color coding
- **Home marker** — Orange pole + sphere at ground level (always at terrain elevation)
- **Target marker** — Red marker at guided target location
- **Mission trajectory** — 3D path visualization of uploaded mission waypoints
- **Trajectory prediction** — Physics-based 5–20s flight path prediction (bank angle, vertical speed)
- **Safety corridor** — Translucent corridor around predicted path
- **ADS-B traffic** — 3D markers for nearby aircraft
- **Sun lighting** — Directional light with 4096x4096 shadow map, real sun position

### 3.3 Web Workers (4 dedicated)
| Worker | Function |
|--------|----------|
| TerrainWorker | Mesh geometry generation from HGT elevation data |
| TileWorker | Satellite tile download and image decoding |
| HillshadeWorker | Normal computation + sun position shading |
| TextureCullWorker | Camera frustum culling for texture load priority |

---

## 4. HUD — Head-Up Display

### 4.1 Flight Instruments (Canvas 2D overlay)
- **Artificial horizon** — Pitch/roll ladder with sky/ground coloring
- **Airspeed tape** — Left side, graduated scale
- **Altitude tape** — Right side, graduated scale with terrain awareness
- **Compass rose** — Heading indicator with track marker
- **Vertical speed indicator** — Rate of climb/descent
- **G-load widget** — 350-element history graph with peak tracking
- **Disarmed label** — Red-orange iridescent HUD overlay when disarmed
- **Status messages** — Max 5 messages, 5-second duration, color-coded by severity
- **Command ACK toast** — Brief overlay showing command results (ARM: ACCEPTED, etc.)

### 4.2 Data Cells (configurable 2x3 grid)
- User-configurable telemetry cells on the HUD
- Selectable fields: airspeed, groundspeed, altitude, vertical speed, battery, GPS, etc.
- Configuration persisted to localStorage

---

## 5. 2D Maps

### 5.1 Mini-Map (Main Screen)
- **Leaflet** with Google Satellite imagery (cache-first loading)
- Aircraft SVG marker with heading rotation
- Red flight trail polyline (3000 points max, downsampled)
- Mission waypoints overlay (green dots + dashed path)
- Home position marker (orange circle)
- ADS-B traffic dots (red circles with callsign tooltips)
- Click-to-go: click on map to set GUIDED target

### 5.2 Mission Map (Flight Plan Tab)
- Full-screen Leaflet map with OpenStreetMap base
- Satellite imagery toggle (Google Satellite layer)
- Waypoint editing: click to add, drag to move, right-click to delete
- Survey grid planner with camera FOV, overlap, altitude parameters
- Elevation profile chart below map (terrain + mission altitude)
- Home position marker with altitude popup

### 5.3 Offline Tile Cache
- **IndexedDB** persistent tile storage (`datad-tile-cache`)
- Cache-first strategy: check IndexedDB → network fallback → cache result
- Opportunistic caching: tiles loaded online are automatically cached for offline use
- CORS-safe loading via native `<img>` tags (avoids Electron fetch restrictions)

---

## 6. Navigation Display (Airbus A350 Style)

### 6.1 Display Modes
| Mode | Description |
|------|-------------|
| ARC | Forward-looking arc with heading, 60° sweep |
| ROSE NAV | Full 360° compass rose with navigation data |
| ROSE VOR | VOR tracking display with CDI |
| ROSE ILS | ILS approach display with localizer + glideslope |
| PLAN | Top-down flight plan view (north-up) |

### 6.2 Features
- Range selection: 10, 20, 40, 80, 160, 320 NM
- Wind vector display (speed + direction)
- Terrain overlay with elevation coloring
- TCAS traffic overlay from ADS-B
- Waypoint tracking with ETA and distance
- VOR1/VOR2 needle displays
- ILS localizer and glideslope deviation

---

## 7. Telemetry & Charting

### 7.1 Real-Time Plotly Charts
- **Predefined traces:** Airspeed, Groundspeed, Vertical Speed, Altitude, Roll, Pitch, Az
- **Custom formula traces** — Safe expression parser (whitelist-based, no eval)
  - Example: `state.gs * 3.6` (groundspeed in km/h)
  - Available fields: all STATE properties
- RingBuffer-backed time-series (1200 samples, Float64Array)
- 8 synchronized channels: timestamp, as, gs, vs, rawAlt, roll, pitch, az

### 7.2 Telemetry Panel
- Raw telemetry stream viewer
- 40+ updatable fields
- FPS counter

---

## 8. Mission Planning

### 8.1 Mission Editor
- Click-to-add waypoints on the mission map
- Drag waypoints to reposition
- Right-click to delete waypoints
- Per-waypoint altitude setting
- Command type selection (100+ MAVLink commands)
- Command categories: Navigation, Condition, DO, Camera/Gimbal

### 8.2 Mission Upload
- Full MISSION_COUNT → MISSION_ITEM_INT protocol
- Waypoint-by-waypoint upload with ACK verification
- Home position (WP#0) handling

### 8.3 Survey Grid Planner
- Draw polygon survey area (click vertices, double-click to close)
- Configurable parameters: altitude, camera FOV, overlap %, ground spacing
- Auto-calculated grid pattern
- Real-time preview on mission map

### 8.4 Elevation Profile
- Side-view terrain cross-section under mission path
- Interpolated terrain elevation between waypoints
- Mission altitude line overlaid on terrain
- Ground clearance visualization

---

## 9. Vehicle Control

### 9.1 Command Bar (Bottom)
- **ARM/DISARM** button with confirmation
- **Flight mode** dropdown (color-coded: yellow=manual, cyan=assisted, blue=auto, orange=RTL)
- Battery voltage/current/remaining indicators
- GPS fix type + satellite count
- Link quality indicator
- Flight timer (auto-start on arm)

### 9.2 Available Commands
| Command | Description |
|---------|-------------|
| ARM / DISARM | Vehicle arming with safety confirmation |
| Set Flight Mode | Mode change (FBWA, AUTO, GUIDED, RTL, LOITER, etc.) |
| Takeoff | Guided takeoff to specified altitude |
| Land | Initiate landing |
| RTL | Return to launch |
| Set Home | Set home position to current location |
| Guided Target | Click-to-go on map (lat/lon/alt) |
| Change Altitude | Adjust target altitude in guided/auto |
| Change Speed | Adjust target airspeed/groundspeed |
| Reboot | Autopilot reboot |
| Calibration | Accelerometer, compass, gyro calibration |

### 9.3 Joystick RC Override
- Gamepad API polling at 25 Hz
- 4-axis mapping: Roll, Pitch, Yaw, Throttle
- Configurable deadzone, expo curve, channel inversion
- RC_CHANNELS_OVERRIDE output (1000–2000 PWM)
- Configuration persisted to localStorage

---

## 10. RTK GNSS Corrections

- **Base station support** — u-blox F9P via serial (115200 baud)
- **RTCM3 frame parsing** — Extracts correction data from serial stream
- **GPS_RTCM_DATA injection** — MAVLink message ID 233
- **Fragmentation** — Splits corrections >180 bytes into multiple packets
- **Status indicators** — RTK fix type (None, Float, Fixed) displayed in UI

---

## 11. ADS-B Traffic Awareness

- **OpenSky Network** — HTTP polling for nearby traffic (50 km radius)
- **Rate limited** — 10-second polling interval
- **Stale removal** — Entries older than 60 seconds automatically removed
- **Display layers:**
  - 2D mini-map: red circle markers with callsign tooltips
  - 3D scene: positioned markers with altitude
  - Navigation Display: TCAS-style overlay
- **CSV export** — Download current traffic data

---

## 12. FPV Camera

- **RTSP streaming** — Supports SIYI HM30 and generic RTSP cameras
- **ffmpeg backend** — RTSP → MJPEG conversion in main process
- **Frame extraction** — SOI/EOI JPEG frame parsing
- **Overlay display** — Semi-transparent overlay on 3D view
- **Settings dialog** — Camera IP, port, stream path configuration

---

## 13. Telemetry Forwarding

- **LTM protocol** — Lightweight Telemetry (G/A/S/O frames) to external serial port
- **MAVLink passthrough** — Forward all MAVLink packets to secondary connection
- **Use case** — Feed OSD, antenna tracker, or secondary GCS

---

## 14. Flight Logging

### 14.1 Recording (.CRV format)
- Binary format at 10 Hz sample rate
- Packet types: FILE_HEADER (0x10), NAVIGATION (0x11), SYS_STATUS (0x12), EVENT (0x13)
- Auto-starts on MAVLink connect
- 8 KB flush buffer for write efficiency

### 14.2 Playback
- Variable speed: 0.1x to 4x
- Timeline scrubber with seek
- Full 3D/HUD/map replay
- Auto UI mode switching to playback layout

---

## 15. Offline Data Download

### 15.1 UI Panel (Sys Config Tab)
- **Bounding box input** — North/South latitude, West/East longitude
- **Default example** — Italy (36–47 N, 6–19 E)
- **Zoom level selector** — Max satellite tile zoom (1–18)
- **Download type** — Satellite tiles, SRTM1 elevation, or both
- **Live tile count** — Estimated number of tiles before download
- **Progress bar** — Real-time status with abort capability

### 15.2 Satellite Tile Download
- Enumerates all tile coordinates from zoom 1 to max zoom
- Downloads from Google Satellite (mt0-3.google.com)
- Stores in IndexedDB via TileCache
- 6 concurrent downloads
- Skips already-cached tiles

### 15.3 SRTM1 Elevation Download
- Enumerates all 1°×1° HGT files covering the bounding box
- Source: AWS Mapzen (`elevation-tiles-prod.s3.amazonaws.com/skadi/`)
- Free, no authentication required
- Gzip decompression via browser DecompressionStream API
- Validates file size (3601 × 3601 × 2 = 25,934,402 bytes)
- Saves to disk (`topo/` folder) via IPC
- Registers in terrain engine immediately for use

---

## 16. SITL Simulator

- **ArduPilot SITL** — Software-in-the-loop simulator
- **Auto-download** — Downloads SITL binaries automatically
- **Platform support:**
  - Linux: native SITL process
  - Windows: WSL-based SITL execution
- **Connection** — TCP 5760 to simulated vehicle
- **Vehicle types** — Copter, Plane, Rover, Sub

---

## 17. Parameter Editor

- Full ArduPilot parameter list (1000+ parameters)
- Search/filter by name or description
- Inline value editing with save
- Parameter descriptions from ArduPilot metadata

---

## 18. UI Layout

### 18.1 View Modes
- **Fullscreen** — Single view (3D, 2D, ND, or Chart)
- **Split** — Multi-pane layout with selectable combinations
- **Pane selection** — Click pane header to swap content
- **Keyboard shortcuts** — Ctrl+1-4 for tab switching

### 18.2 Tabs
| Tab | Content |
|-----|---------|
| Flight Data | 3D terrain + HUD + mini-map + charts |
| Flight Plan | Mission editor + full map + elevation profile |
| Setup | Parameter editor |
| Sys Config | Connection settings, RTK, FPV, joystick, offline download |

### 18.3 Window
- Frameless Electron window with custom title bar
- Dark theme (glass-effect panels)
- Animated splash screen with terrain loading progress

---

## 19. Build & Platforms

```
npm install          # Install dependencies
npm start            # Development mode
npm run build:win    # Windows installer (NSIS)
npm run build:linux  # Linux (AppImage, deb)
```

### Installer Excludes
- `topo/` — Downloaded SRTM elevation files (user-specific, large)
- `topography/` — Alternative terrain folder
- `scripts/` — Development scripts

### Dependencies
| Package | Role |
|---------|------|
| Electron 39.x | Desktop runtime (Chromium + Node.js) |
| electron-builder | NSIS/AppImage/deb packaging |
| node-mavlink 2.x | MAVLink v2 protocol (ardupilotmega dialect) |
| serialport 13.x | Native serial port access |
| Three.js r128 | 3D rendering (CDN) |
| Leaflet 1.9 | 2D mapping (CDN) |
| Plotly.js 2.27 | Telemetry charts (CDN) |
