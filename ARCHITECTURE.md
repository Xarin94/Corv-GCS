# Corv-GCS Architecture

> Desktop Ground Control Station for ArduPilot — Electron + Three.js + Leaflet

**Version:** 1.2.2 | **License:** Apache-2.0 | **Repository:** [github.com/Xarin94/Corv-GCS](https://github.com/Xarin94/Corv-GCS)

Corv-GCS is a frameless Electron desktop application providing 3D terrain visualization, 2D mapping, HUD flight instruments, an Airbus-style Navigation Display, telemetry charting, mission planning, FPV camera, RTK corrections, ADS-B traffic awareness, joystick RC override, and flight log recording/playback. It supports ArduPilot vehicles (Copter, Plane, Rover, Sub, Heli, QuadPlane) via MAVLink v2.

---

## 1. High-Level Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                       ELECTRON MAIN PROCESS                          │
│                            (Node.js / CommonJS)                      │
│                                                                      │
│  main.js ── Window lifecycle, IPC handlers, file I/O                 │
│     │                                                                │
│     ├── main-mavlink.js ── Serial/UDP/TCP + MAVLink v2 parse/send    │
│     ├── sitl-manager.js ── SITL binary download & process spawn      │
│     ├── rtk-manager.js  ── RTCM3 parse + GPS_RTCM_DATA injection    │
│     ├── fpv-manager.js  ── ffmpeg RTSP → MJPEG frame extraction      │
│     └── telforward-manager.js ── LTM / MAVLink passthrough output    │
│                                                                      │
└──────────────────────────┬───────────────────────────────────────────┘
                           │
                     preload.js
                (contextBridge IPC APIs)
                           │
┌──────────────────────────┴───────────────────────────────────────────┐
│                     ELECTRON RENDERER PROCESS                        │
│                       (Chromium / ES Modules)                        │
│                                                                      │
│  js/main.js ── Init + 60 FPS animation loop                         │
│     │                                                                │
│     ├── core/      STATE, constants, utils, RingBuffer, LRUCache     │
│     ├── mavlink/   MAVLinkManager, StateMapper, CommandSender,       │
│     │              ConnectionManager                                 │
│     ├── engine/    Scene3D, TrajectoryPredictor, TrajectoryCorridor, │
│     │              SunPosition                                       │
│     ├── terrain/   TerrainManager + 4 Web Workers                    │
│     ├── hud/       HUDRenderer (Canvas 2D)                           │
│     ├── maps/      MapEngine (Leaflet 2D)                            │
│     ├── ui/        TabController, SplitView, NDView, NDController,   │
│     │              UIController, CommandBarController,                │
│     │              GCSSidebarController, ParametersPageController,    │
│     │              FPVController, TraceManager, LoadingOverlay        │
│     ├── adsb/      ADSBManager                                       │
│     ├── joystick/  JoystickManager, JoystickUI                       │
│     ├── logging/   CRVLogger                                         │
│     ├── playback/  LogPlayer                                         │
│     ├── mission/   MissionCommands                                   │
│     └── serial/    SerialHandler (legacy binary protocol)            │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 2. Directory Structure

```
Corv-GCS/
├── main.js                     Electron main process entry point
├── preload.js                  Context bridge (10 IPC API namespaces)
├── main-mavlink.js             MAVLink serial/UDP/TCP connections + parsing
├── sitl-manager.js             ArduPilot SITL simulator launcher
├── rtk-manager.js              RTK GNSS base station (RTCM3)
├── fpv-manager.js              FPV camera stream (ffmpeg RTSP→MJPEG)
├── telforward-manager.js       Telemetry forwarding (LTM / MAVLink)
├── package.json                Dependencies & build config
│
├── js/                         Renderer process modules (ES Modules)
│   ├── main.js                 Entry point + 60 FPS animation loop
│   ├── core/                   State management & utilities
│   │   ├── state.js            Global STATE object (~100+ properties)
│   │   ├── constants.js        Configuration constants
│   │   ├── utils.js            Math helpers (coordinates, colors, CRC)
│   │   ├── RingBuffer.js       O(1) circular buffer (Float64Array)
│   │   ├── LRUCache.js         Least-recently-used cache
│   │   └── ExpressionParser.js Safe formula evaluator for custom traces
│   ├── engine/                 3D rendering engine
│   │   ├── Scene3D.js          Three.js scene, camera, lighting, trail
│   │   ├── TrajectoryPredictor.js  Physics-based flight path prediction
│   │   ├── TrajectoryCorridor3D.js Visual safety corridor
│   │   └── SunPosition.js      Solar position for lighting & hillshade
│   ├── terrain/                Terrain elevation & mesh generation
│   │   ├── TerrainManager.js   HGT loading, chunked 3D mesh, LOD textures
│   │   ├── TerrainWorker.js    Web Worker: mesh geometry generation
│   │   ├── TileWorker.js       Web Worker: satellite tile download
│   │   ├── HillshadeWorker.js  Web Worker: hillshade computation
│   │   └── TextureCullWorker.js Web Worker: frustum culling
│   ├── hud/
│   │   └── HUDRenderer.js      Canvas 2D flight instruments overlay
│   ├── maps/
│   │   └── MapEngine.js         Leaflet 2D mini-map
│   ├── mavlink/                 MAVLink protocol layer (renderer side)
│   │   ├── MAVLinkManager.js    Message dispatcher & connection bridge
│   │   ├── MAVLinkStateMapper.js Message → STATE field mapping
│   │   ├── CommandSender.js     High-level autopilot commands
│   │   └── ConnectionManager.js Connection lifecycle orchestrator
│   ├── ui/                      UI controllers
│   │   ├── UIController.js      Telemetry display, HUD cell config
│   │   ├── TabController.js     Tab navigation, mission editor, survey
│   │   ├── SplitView.js         Multi-pane layout + Plotly charts
│   │   ├── NDView.js            Navigation Display (Airbus A350 style)
│   │   ├── NDController.js      ND sidebar controls
│   │   ├── CommandBarController.js Bottom bar (ARM/mode/status)
│   │   ├── GCSSidebarController.js Right sidebar (connections, SITL, RTK)
│   │   ├── ParametersPageController.js Full parameter editor
│   │   ├── FPVController.js     FPV camera overlay & settings
│   │   ├── TraceManager.js      Plotly trace config + custom formulas
│   │   └── LoadingOverlay.js    Splash screen with loading progress
│   ├── adsb/
│   │   └── ADSBManager.js       OpenSky Network ADS-B traffic
│   ├── joystick/
│   │   ├── JoystickManager.js   Gamepad RC override (25 Hz)
│   │   └── JoystickUI.js        Joystick configuration UI
│   ├── logging/
│   │   └── CRVLogger.js         Binary .CRV telemetry recording (10 Hz)
│   ├── playback/
│   │   └── LogPlayer.js         Flight log playback (variable speed)
│   ├── mission/
│   │   └── MissionCommands.js   MAVLink command catalog (100+ commands)
│   └── serial/
│       └── SerialHandler.js     Legacy binary protocol (WebSerial)
│
├── html/
│   ├── index.html               Main page (loads all modules)
│   └── components/              HTML template fragments
│       ├── header.html           Brand + view mode buttons
│       ├── title-bar.html        Frameless window controls
│       ├── bottom-bar.html       LAT/LON/RADAR ALT status
│       ├── config-panel.html     Settings panel (alt offset, terrain, time)
│       ├── telemetry-panel.html  Raw telemetry stream viewer
│       ├── tape-left.html        Airspeed / GS cells
│       ├── tape-right.html       Altitude / terrain cells
│       ├── plotly-container.html Trace checkboxes + chart
│       ├── loading-overlay.html  Animated splash screen
│       └── storyline-player.html Log playback controls
│
├── css/                         17 modular CSS files
│   ├── style.css                Master import file
│   ├── variables.css            Design tokens (dark/light themes)
│   ├── base.css                 Resets & scrollbar styles
│   ├── layout.css               Grid layout & view modes
│   ├── components.css           Buttons, data cells, controls
│   ├── panels.css               Floating panels (glass effect)
│   ├── title-bar.css            Custom window title bar
│   ├── tabs.css                 Tab navigation
│   ├── nd-panel.css             Navigation Display + sidebar
│   ├── command-bar.css          Bottom command bar
│   ├── gcs-sidebar.css          Right sidebar
│   ├── plotly.css               Chart controls & trace config
│   ├── loading.css              Loading overlay
│   ├── animations.css           Keyframe animations
│   ├── fpv.css                  FPV camera panel
│   ├── joystick.css             Joystick config panel
│   └── vendors.css              Leaflet overrides
│
├── assets/icons/                App icons (16x16 to 512x512, ICO, PNG)
├── models/                      3D aircraft models (GLB)
├── topo/                        SRTM .hgt terrain elevation files
├── flightplans/                 Sample flight plans (JSON)
├── build/                       Build artifacts & installer icons
├── docs/                        Additional documentation
└── screenshots/                 Project screenshots
```

---

## 3. Functional Map

### 3.1 Main Process (Root)

| File | Key Functions | Purpose |
|------|---------------|---------|
| `main.js` | `createWindow()`, IPC handlers for models/topography/ADS-B/CRV | Electron app lifecycle, window management, file I/O |
| `main-mavlink.js` | `initMAVLinkHandlers()`, `connectSerial/UDP/TCP()`, `handlePacket()`, `sendMAVLinkCommand()`, `sendMAVLinkMessage()`, `startHeartbeat()`, `disconnectCurrent()`, `sendRawBuffer()` | MAVLink v2 connection pipeline: serial/UDP/TCP transport, packet splitting/parsing/deserialization, 1 Hz heartbeat, command encoding |
| `preload.js` | Context bridge: `mavlink`, `sitl`, `rtk`, `fpv`, `telForward`, `adsb`, `crvLogger`, `topography`, `models`, `windowControls`, `devtools` | Secure IPC bridge between main and renderer processes (10 namespaced APIs) |
| `sitl-manager.js` | `initSITLHandlers()`, `cleanup()` | Download ArduPilot SITL binaries, spawn process (native Linux or WSL on Windows), TCP 5760 |
| `rtk-manager.js` | `initRTKHandlers()`, `cleanup()` | RTCM3 frame parsing from serial GPS base station, GPS_RTCM_DATA (ID 233) injection to drone via raw MAVLink v2 packets |
| `fpv-manager.js` | `initFPVHandlers()`, `cleanupFPV()` | Spawn ffmpeg for RTSP-to-MJPEG conversion, extract JPEG frames (SOI/EOI markers), send base64 frames via IPC |
| `telforward-manager.js` | `initTelForwardHandlers()`, `cleanup()` | Forward telemetry as LTM protocol (G/A/S/O frames) or MAVLink passthrough to external serial port |

### 3.2 Core (`js/core/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `state.js` | `STATE`, `dataBuffer`, `pushGHistory()`, `demoAttitude`, `demoSurveyState`, `activeTraces`, `viewMode`, `resetDataBuffer()` | Global mutable state (~100+ properties): attitude, position, velocity, MAVLink state, battery, GPS, RTK, vibration, mission, traffic. RingBuffer-backed time-series via `dataBuffer` proxy |
| `constants.js` | `ORIGIN`, `CAMERA_FOV`, `VISIBILITY_RADIUS`, `BUFFER_SIZE`, `SAMPLE_INTERVAL`, `TRACE_CONFIG`, demo constants | All configuration constants (reference origin, camera, terrain chunks, demo mode) |
| `utils.js` | `latLonToMeters()`, `calculateDistance()`, `lerpColor()`, `getHeightColor()`, `calculateCRC16()`, `latLonToTile()`, `tileToBounds()` | Coordinate conversion (WGS84 → local meters), Haversine distance, color interpolation, terrain palette, CRC-16, tile math |
| `RingBuffer.js` | `RingBuffer`, `MultiChannelRingBuffer` | O(1) circular buffer (Float64Array) with binary search (`lowerBound`), array export, clear. Used for telemetry time-series |
| `LRUCache.js` | `LRUCache` | Least-recently-used eviction cache for terrain satellite textures. Prevents GPU memory exhaustion |
| `ExpressionParser.js` | `compileExpression()`, `validateExpression()`, `getAvailableFields()`, `ExpressionError` | Safe math expression evaluator (whitelist-based, no eval) for custom Plotly trace formulas |

### 3.3 MAVLink (`js/mavlink/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `MAVLinkManager.js` | `initMAVLink()`, `onMessage(msgId, handler)`, `offMessage()`, `connectMAVLinkSerial/UDP/TCP()`, `disconnectMAVLink()`, `listSerialPorts()` | Renderer-side message router. Registers IPC listeners, dispatches messages to handlers, calls `mapMessageToState()`, fires `serialUpdate` CustomEvent |
| `MAVLinkStateMapper.js` | `mapMessageToState(msgId, data)`, `getFlightModeName()`, `getFlightModeNumber()`, `getAvailableFlightModes()`, `getGPSFixName()`, `getVehicleTypeName()`, `computeAeroAngles()` | Decodes 17+ MAVLink message types into STATE fields. Maintains ArduPilot mode tables for Copter/Plane/Rover/Sub. Computes AoA/SSA from NED velocity |
| `CommandSender.js` | `armVehicle()`, `disarmVehicle()`, `setFlightMode()`, `takeoff()`, `land()`, `returnToLaunch()`, `setGuidedTarget()`, `setParameter()`, `requestAllParameters()`, `requestAllDataStreams()`, `uploadMission()`, `sendRCChannelsOverride()`, `changeAltitude()`, `calibrateAccel/Compass/Gyro()`, `rebootAutopilot()` | High-level autopilot command abstraction with retry/ACK logic. Covers arming, modes, navigation, parameters, mission upload protocol, RC override, calibration |
| `ConnectionManager.js` | `connect(type, options)`, `disconnect()`, `getAvailablePorts()`, `isHeartbeatAlive()`, `getConnectionInfo()` | Connection lifecycle orchestrator. Auto-requests data streams and home position on MAVLink connect. Supports serial, UDP, TCP, legacy corv-binary |

### 3.4 3D Engine (`js/engine/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `Scene3D.js` | `init3D()`, `render()`, `updateCamera()`, `updateTrail()`, `resetTrail()`, `setTrailPoints()`, `resize()`, `updateMissionTrajectory()`, `clearMissionTrajectory()`, `updateHomeMarker3D()`, `updateTargetMarker3D()`, `updateTrafficMarkers3D()`, `getScene/Camera/Renderer/SunLight()`, `setSunlightEnabled()` | Three.js scene setup (FOV 60°, exponential fog), camera follow modes, directional sun with 4096² shadow map, flight trail (BufferGeometry, 50k points), 3D mission path, home/target/traffic markers |
| `TrajectoryPredictor.js` | `computePredictedPath()`, `computePredictedPath2D()` | Physics-based flight path prediction (5–20s ahead). Low-pass filter (α=0.88) on speed/roll/VS, turn radius from bank angle (R = V²/g·tan(roll)), vertical acceleration from NED |
| `TrajectoryCorridor3D.js` | `initCorridor()`, `updateCorridor()`, `setCorridorVisible()`, `disposeCorridor()`, `getPredictionTime()` | Visual "safety corridor" around predicted path (two border lines + translucent fill, ~1.2m width, green with alpha fade) |
| `SunPosition.js` | `calculateSunPosition()`, `getSunLightDirection()`, `calculateHillshade()`, `applyHillshade()` | Solar almanac for realistic dynamic lighting and terrain hillshading based on date/time/location |

### 3.5 Terrain (`js/terrain/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `TerrainManager.js` | `initTerrain()`, `updateTerrainChunks()`, `getTerrainElevationCached()`, `getTerrainElevationFromHGT()`, `addHGTFile()`, `updateTerrainHillshading()`, `setMapBrightness()`, `setTerrainSatelliteEnabled()`, `updateWireframeProximity()`, `getMemoryStats()` | Main terrain engine (58KB). Loads SRTM HGT elevation data, generates chunked 3D meshes (5000m × 5000m, 50km visibility radius), dual-zoom LOD satellite textures (zoom 15), frustum culling, LRU texture cache (1500 capacity), max 24 concurrent tile loads |
| `TerrainWorker.js` | Web Worker | Background mesh geometry generation from elevation data |
| `TileWorker.js` | Web Worker | Satellite tile downloading and image decoding |
| `HillshadeWorker.js` | Web Worker | Hillshade normal computation from elevation + sun position |
| `TextureCullWorker.js` | Web Worker | Camera frustum culling for texture loading priority |

### 3.6 HUD & Maps

| File | Key Exports | Purpose |
|------|-------------|---------|
| `hud/HUDRenderer.js` | `initHUD()`, `drawHUD()`, `resizeHUD()`, `pushHudMessage()`, `initGLoadWidget()`, `drawGLoadWidget()`, `setViewMode()` | Canvas 2D flight instruments overlay: artificial horizon, altitude/speed tapes, compass rose, G-load graph (350-element object pool), vertical speed indicator, status messages (max 5, 5s duration) |
| `maps/MapEngine.js` | `initMap()`, `updateMap()`, `invalidateSize()`, `updateMissionOverlay()`, `setTargetMarker()`, `clearTargetMarker()` | Leaflet 2D mini-map with Esri World Imagery. Aircraft SVG marker, red trail polyline (3000 points max, downsampled), mission waypoint circles, home marker, click-to-go guided target |

### 3.7 UI Controllers (`js/ui/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `UIController.js` | `updateUI()`, `initHudCells()`, `toggleConfig()`, `toggleTelemetry()`, `updateOffset()`, `updateAGLDisplay()`, `setStatusMessage()`, `updateFPSDisplay()`, `initMoreMenu()`, `initConfigAutoClose()` | Telemetry display updates (40+ fields), configurable 2×3 HUD cell grid, config/telemetry panel toggles, FPS counter. Persists cell config to localStorage |
| `TabController.js` | `initTabs()`, `getCurrentTab()`, `initSurveyGrid()` | Tab-based page navigation (Flight Data, Flight Plan, Setup, Sys Config), mission editor UI, survey grid planner |
| `SplitView.js` | `toggleViewMode()`, `setViewMode()`, `getViewMode()`, `sampleDataPoint()`, `updatePlotly()`, `resizeSplitView()`, `updateND()`, `recordLivePathPoint()`, `is3DVisible()`, `is2DMapVisible()`, `isNDVisible()` | Multi-pane layout manager (3D / 2D / ND / Plotly). Handles pane selection, swap logic, Plotly real-time trace updates, view mode cycling (FULLSCREEN / SPLIT) |
| `NDView.js` | `initND()`, `drawND()`, `resizeND()`, `setNDMode()`, `setNDRange()`, `setFlightPlan()`, `setWindData()`, `setVOR1/2()`, `setILS()`, `ndConfig`, `FLIGHT_PLANS` | Navigation Display rendering (Airbus A350 style): ARC/ROSE_NAV/ROSE_VOR/ROSE_ILS/PLAN modes, range 10–320 NM, terrain/weather/TCAS overlays, wind vector, waypoint tracking |
| `NDController.js` | `initNDControls()`, `startWaypointTracking()`, `stopWaypointTracking()`, `loadSampleFlightPlan()` | ND sidebar controls: mode/range selectors, HDG/VOR/ILS inputs, flight plan loading |
| `CommandBarController.js` | `initCommandBar()`, `updateCommandBar()` | Bottom command bar: ARM/DISARM button, flight mode dropdown (color-coded: yellow=manual, cyan=assisted, blue=auto, orange=RTL), battery/GPS/link indicators, flight timer |
| `GCSSidebarController.js` | `initGCSSidebar()`, `updateGCSSidebar()`, `getTargetCoords()` | Right sidebar: connection panel (serial/UDP/TCP port selection), SITL launcher, RTK base station, telemetry forwarding config |
| `ParametersPageController.js` | `initParamsPage()`, `toggleParamsPage()`, `formatParamValue()` | Full ArduPilot parameter editor with search, inline edit, save |
| `FPVController.js` | `initFPV()`, `onFPVButtonClick()`, `setFPVActive()`, `stopFPVStream()`, `resizeFPV()`, `openFPVSettings()` | FPV camera overlay on 3D view. ffmpeg stream controls, SIYI HM30 / generic RTSP settings dialog |
| `TraceManager.js` | `initTraceManager()`, `evaluateTraces()`, `sampleFormulaDataPoint()`, `getActiveTraceConfigs()`, `buildLiveEntry()`, `formulaDataBuffer` | Custom Plotly trace manager. Predefined traces (as, gs, vs, rawAlt, roll, pitch, az) + custom formula traces via ExpressionParser |
| `LoadingOverlay.js` | `showLoadingOverlay()`, `hideLoadingOverlay()`, `checkInitialLoadComplete()`, `scheduleHideLoadingOverlaySoon()` | Animated splash screen with cloud parallax and plane animation, terrain loading progress bar |

### 3.8 Other Modules

| File | Key Exports | Purpose |
|------|-------------|---------|
| `adsb/ADSBManager.js` | `fetchADSBData()`, `getNearestTraffic(n)`, `downloadTrafficCSV()` | OpenSky Network ADS-B traffic polling (50km radius, via main process for CORS bypass). Rate limited (10s), stale entry removal (60s), CSV export |
| `joystick/JoystickManager.js` | `JoystickManager` class | Gamepad API polling at 25 Hz. Axis mapping (roll/pitch/yaw/throttle), deadzone, expo, inversion config. Sends RC_CHANNELS_OVERRIDE (1000–2000 PWM). Config persisted to localStorage |
| `joystick/JoystickUI.js` | `initJoystick()` | Joystick configuration UI: gamepad selection, axis live display, channel mapping |
| `logging/CRVLogger.js` | `CRVLogger` class | Binary .CRV telemetry recording at 10 Hz. Packet types: FILE_HEADER (0x10), NAVIGATION (0x11), SYS_STATUS (0x12), EVENT (0x13). Auto-starts on MAVLink connect, 8KB flush buffer |
| `playback/LogPlayer.js` | `initPlaybackControls()`, `tickPlayback()`, `togglePlay()`, `seekTo()`, `setPlaybackSpeed()`, `updateFromLog()` | Flight log playback with variable speed (0.1x–4x), timeline scrubber, auto UI mode switching |
| `mission/MissionCommands.js` | `MISSION_COMMANDS`, `getCmdDef()`, `getCmdName()`, `getCmdParams()`, `isNavCmd()`, `getGroupedCommands()` | MAVLink mission command catalog (100+ commands). Categories: Navigation, Condition, DO, Camera/Gimbal. Used by mission planner UI and CommandSender.uploadMission() |
| `serial/SerialHandler.js` | `connectSerial()` | Legacy binary protocol via WebSerial API (460800 baud). Custom packets: [0xA5, 0x5A, TYPE, LEN, PAYLOAD, CRC16]. Parses navigation packets directly to STATE |

---

## 4. Data Flow Diagrams

### 4.1 Telemetry Ingest (MAVLink → Display)

```
Aircraft / SITL
    │ MAVLink v2 packets (serial / UDP / TCP)
    ▼
main-mavlink.js
    │ MavLinkPacketSplitter → MavLinkPacketParser → deserialize
    ▼
IPC: 'mavlink-message' { msgId, data, sysId, compId }
    │
    ▼
preload.js (contextBridge)
    │
    ▼
MAVLinkManager.js
    │ handleMessage()
    ├── mapMessageToState(msgId, data)  ──→ STATE updated
    ├── registered handlers (onMessage callbacks)
    └── CustomEvent('serialUpdate') dispatched
         │
         ▼
js/main.js  (60 FPS animation loop)
    │ reads STATE
    │
    ├──→ Scene3D.render()              3D terrain + aircraft
    ├──→ HUDRenderer.drawHUD()         flight instruments
    ├──→ MapEngine.updateMap()         2D mini-map
    ├──→ NDView.drawND()               navigation display
    ├──→ SplitView.updatePlotly()      telemetry charts
    └──→ CommandBar.updateCommandBar() status indicators
```

### 4.2 Command Send (UI → Aircraft)

```
User Action (button click / mode selector / joystick)
    │
    ▼
CommandSender.js
    │ e.g. armVehicle(), setFlightMode(), uploadMission()
    ▼
IPC: 'mavlink-send-command' or 'mavlink-send-message'
    │
    ▼
preload.js (contextBridge)
    │
    ▼
main-mavlink.js
    │ sendMAVLinkCommand() / sendMAVLinkMessage()
    │ serialize via MavLinkProtocolV2
    ▼
Serial port / UDP socket / TCP socket ──→ Aircraft
```

### 4.3 RTK Correction Flow

```
GPS Base Station (u-blox F9P)
    │ Serial (115200 baud)
    ▼
rtk-manager.js
    │ RTCM3Parser.parse() ── extract RTCM frames
    │ forwardRTCMtoDrone() ── build GPS_RTCM_DATA (ID 233)
    │ fragment if > 180 bytes
    ▼
main-mavlink.js
    │ sendRawBuffer()
    ▼
Active connection ──→ Drone (injects corrections into GPS)
```

### 4.4 Terrain Loading Pipeline

```
STATE.lat, STATE.lon (aircraft position)
    │
    ▼
TerrainManager.updateTerrainChunks()
    │ determine chunks needed (50km visibility radius)
    │ queue chunk creation
    │
    ├── TerrainWorker.js       generate mesh geometry from HGT
    ├── TileWorker.js          download satellite tiles
    ├── HillshadeWorker.js     compute normals + sun shading
    └── TextureCullWorker.js   frustum culling for load priority
         │
         ▼
Three.js Scene ── Mesh(geometry, texture) per chunk
    │ LRUCache manages texture memory (cap: 1500)
    ▼
Rendered at 60 FPS
```

### 4.5 FPV Camera Pipeline

```
Camera (SIYI HM30 or RTSP source)
    │ RTSP stream (H.264)
    ▼
fpv-manager.js
    │ spawn ffmpeg: RTSP → MJPEG pipe
    │ MJPEGParser: extract JPEG frames (SOI/EOI markers)
    │ frame.toString('base64')
    ▼
IPC: 'fpv-frame' (base64 JPEG string)
    │
    ▼
FPVController.js
    │ set <img>.src = 'data:image/jpeg;base64,...'
    ▼
Rendered as overlay on 3D view
```

---

## 5. Key Integration Patterns

### 5.1 Single Source of Truth (STATE)
All telemetry flows through the global `STATE` object in `core/state.js`. The 60 FPS render loop reads STATE — no UI component queries the autopilot directly. `MAVLinkStateMapper` writes to STATE; all rendering and UI modules read from it.

### 5.2 IPC Bridge Architecture
`preload.js` exposes 10 namespaced APIs via `contextBridge.exposeInMainWorld()`. All IPC uses `invoke`/`handle` (request-response) or `send`/`on` (events). Security: `contextIsolation: true`, no `nodeIntegration`.

### 5.3 Web Workers for Heavy Computation
4 dedicated Web Workers handle terrain processing: mesh generation, tile download, hillshade, frustum culling. Workers communicate via `postMessage` with transferable ArrayBuffers. This keeps the main thread free for 60 FPS rendering.

### 5.4 RingBuffer for Time-Series
`dataBuffer` uses RingBuffer (Float64Array, capacity 1200) instead of Array.push/shift. O(1) push, zero GC pressure, binary search for time windows. 8 synchronized channels: timestamps, as, gs, vs, rawAlt, roll, pitch, az.

### 5.5 LRU Cache for Textures
`LRUCache` (capacity 1500) auto-evicts least-recently-used satellite tile textures. Prevents GPU memory exhaustion during long flights across large terrain areas.

### 5.6 Event-Driven Message Handling
- `MAVLinkManager.onMessage(msgId, handler)` — pub/sub per MAVLink message ID
- `CustomEvent('serialUpdate')` — global render trigger on new telemetry
- `CustomEvent('commandAck')` — HUD displays command results
- `CustomEvent('mavlinkConnectionState')` — UI connection indicators, CRV auto-record

### 5.7 ArduPilot Vehicle Abstraction
`MAVLinkStateMapper` maintains mode tables for Copter, Plane, Rover, and Sub. Vehicle type is auto-detected from the HEARTBEAT message `type` field. Mode names and available modes adapt per vehicle type.

---

## 6. Dependencies

| Package | Version | Role |
|---------|---------|------|
| `electron` | ^39.2.7 | Desktop app runtime (Chromium + Node.js) |
| `electron-builder` | ^26.8.1 | Build & packaging (NSIS, AppImage, deb) |
| `node-mavlink` | ^2.3.0 | MAVLink v2 protocol parse/serialize (ardupilotmega dialect) |
| `serialport` | ^13.0.0 | Native serial port access |
| Three.js | r128 | 3D rendering (loaded via CDN in HTML) |
| Leaflet | 1.9.4 | 2D map tiles (loaded via CDN in HTML) |
| Plotly.js | 2.27.0 | Telemetry charting (loaded via CDN in HTML) |

---

## 7. Build & Run

```bash
# Install dependencies
npm install

# Run in development mode
npm start

# Build for Windows + Linux
npm run build

# Build for Windows only
npm run build:win

# Build for Linux only
npm run build:linux
```

**Platforms:** Windows (NSIS installer), Linux (AppImage, deb)

**Note:** SITL on Windows runs via WSL. Native SITL is supported on Linux.
