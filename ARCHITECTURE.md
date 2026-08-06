# Corv-GCS Architecture

> Desktop Ground Control Station for ArduPilot — Electron + Three.js + Leaflet

**Version:** 1.5.0 | **License:** Apache-2.0 | **Repository:** [github.com/Xarin94/Corv-GCS](https://github.com/Xarin94/Corv-GCS)

Corv-GCS is a frameless Electron desktop application providing 3D terrain visualization, 2D mapping, HUD flight instruments, mission planning with undo/redo and a local mission library, FPV camera, RTK/NTRIP corrections, ADS-B traffic awareness, joystick RC override, `.tlog` flight recording, `.tlog` and ArduPilot `.bin` log replay, and offline map/elevation caching.

Three link protocols are supported, all normalised to MAVLink before they reach the renderer:

| Protocol | Vehicles | Direction |
|----------|----------|-----------|
| **MAVLink v2** (GCS sysid 255, Mission Planner compatible) | ArduPilot Copter, Plane, Rover, Sub, Heli, QuadPlane | full duplex — telemetry, commands, parameters, missions |
| **CORV binary v7/v8** | onboard CORV autopilot | telemetry in, config out (USB only) |
| **MSP / MSP2** | INAV, Betaflight | telemetry only (polled) |

---

## 1. High-Level Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                       ELECTRON MAIN PROCESS                          │
│                            (Node.js / CommonJS)                      │
│                                                                      │
│  main.js ── Window lifecycle, IPC handlers, file I/O                 │
│     │                                                                │
│     ├── main-mavlink.js ── Serial/UDP/TCP + MAVLink v2 parse/send +  │
│     │                       CORV binary protocol + .tlog recording   │
│     ├── log-replay-manager.js ── .tlog/.bin replay engine (20 Hz)    │
│     ├── log-replay-bin-parser.js ── ArduPilot DataFlash .bin parser  │
│     ├── sitl-manager.js ── SITL binary download & process spawn      │
│     ├── rtk-manager.js  ── RTCM3 parse + GPS_RTCM_DATA injection     │
│     ├── fpv-manager.js  ── ffmpeg RTSP → MJPEG frame extraction      │
│     ├── telforward-manager.js ── LTM / MAVLink / UDP mirror output    │
│     ├── msp-manager.js  ── MSP/MSP2 (INAV/Betaflight) → MAVLink      │
│     └── mission-store.js ── data root, mission library, index.json   │
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
│     ├── maps/      MapEngine, CachedTileLayer, TileCache,            │
│     │              OfflineDownloader                                  │
│     ├── ui/        TabController, UIController, CommandBarController, │
│     │              GCSSidebarController, ParametersPageController,    │
│     │              ParamCatalog, FPVController, RotorLoadPanel,       │
│     │              LoadingOverlay                                     │
│     ├── adsb/      ADSBManager                                       │
│     ├── joystick/  JoystickManager, JoystickUI                       │
│     ├── logging/   TlogLogger, LogReplayController                   │
│     ├── mission/   MissionCommands, MissionHistory, MissionLibrary   │
│     └── serial/    SerialHandler (CORV binary protocol v7/v8)        │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 2. Directory Structure

```
Corv-GCS/
├── main.js                     Electron main process entry point
├── preload.js                  Context bridge (15 IPC API namespaces)
├── main-mavlink.js             MAVLink serial/UDP/TCP + CORV binary + .tlog rec
├── log-replay-manager.js       .tlog / .bin replay engine (main process)
├── log-replay-bin-parser.js    ArduPilot DataFlash .bin parser
├── sitl-manager.js             ArduPilot SITL simulator launcher
├── rtk-manager.js              RTK GNSS base station (RTCM3) + NTRIP client
├── fpv-manager.js              FPV camera stream (ffmpeg RTSP→MJPEG)
├── telforward-manager.js       Telemetry forwarding (LTM / MAVLink / UDP mirror)
├── msp-manager.js              MSP/MSP2 adapter (INAV, Betaflight) → MAVLink
├── mission-store.js            Data root: missions/, logs/, index.json
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
│   │   └── ExpressionParser.js Safe formula evaluator (whitelist, no eval)
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
│   │   ├── MapEngine.js         Leaflet 2D mini-map
│   │   ├── CachedTileLayer.js   Cache-first Leaflet tile layer
│   │   ├── TileCache.js         IndexedDB tile cache + bulk download
│   │   └── OfflineDownloader.js Offline satellite + SRTM1 download engine
│   ├── mavlink/                 MAVLink protocol layer (renderer side)
│   │   ├── MAVLinkManager.js    Message dispatcher & connection bridge
│   │   ├── MAVLinkStateMapper.js Message → STATE field mapping
│   │   ├── CommandSender.js     High-level autopilot commands
│   │   └── ConnectionManager.js Connection lifecycle orchestrator
│   ├── ui/                      UI controllers
│   │   ├── UIController.js      Telemetry display, HUD cell config
│   │   ├── TabController.js     Tab navigation, mission editor, survey
│   │   ├── CommandBarController.js Bottom bar (ARM/mode/status)
│   │   ├── GCSSidebarController.js Right sidebar (connections, SITL, RTK)
│   │   ├── ParametersPageController.js Full parameter editor
│   │   ├── RotorLoadPanel.js    ROTOR LOAD schematic
│   │   ├── ParamCatalog.js      Known-param name catalog (on-demand reads)
│   │   ├── FPVController.js     FPV camera overlay & settings
│   │   └── LoadingOverlay.js    Splash screen with loading progress
│   ├── adsb/
│   │   └── ADSBManager.js       OpenSky Network ADS-B traffic
│   ├── joystick/
│   │   ├── JoystickManager.js   Gamepad RC override (25 Hz)
│   │   └── JoystickUI.js        Joystick configuration UI
│   ├── logging/
│   │   ├── TlogLogger.js        .tlog auto-recording (start/stop bridge to main)
│   │   └── LogReplayController.js Log Replay UI: open file, timeline scrub, full-track trail
│   ├── mission/
│   │   ├── MissionCommands.js   MAVLink command catalog (100+ commands)
│   │   ├── MissionHistory.js    Undo/redo (snapshot based)
│   │   └── MissionLibrary.js    Saved-mission browser (load/save/overwrite)
│   └── serial/
│       └── SerialHandler.js     CORV binary protocol v7/v8 (WebSerial)
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
├── build/                       Build artifacts & installer icons
├── docs/                        Additional documentation
└── screenshots/                 Project screenshots
```

---

## 3. Functional Map

### 3.1 Main Process (Root)

| File | Key Functions | Purpose |
|------|---------------|---------|
| `main.js` | `createWindow()`, IPC handlers for models/topography/ADS-B/tlog/topography-save | Electron app lifecycle, window management, file I/O, HGT file persistence |
| `main-mavlink.js` | `initMAVLinkHandlers()`, `connectSerial/UDP/TCP()`, `handlePacket()`, `sendMAVLinkCommand()`, `sendMAVLinkMessage()`, `startHeartbeat()`, `disconnectCurrent()`, `sendRawBuffer()`, `corvEmitNavigation()`, `corvEmitDebug()`, `corvEmitRawSensor()`, `emitFakeMavlinkMessage()`, `getReplayParserBuilder()` | MAVLink v2 connection pipeline: serial/UDP/TCP transport, packet splitting/parsing/deserialization, 1 Hz GCS heartbeat (sysid 255, compid 190, MAV_TYPE_GCS), command encoding, GCS output mute toggle. Also hosts the CORV binary protocol v7/v8 decoder (re-emits as synthetic MAVLink to share the renderer pipeline) and the .tlog raw-packet recorder (auto-start on connect). Exports replay hooks consumed by `log-replay-manager.js` |
| `log-replay-manager.js` | `initLogReplay()`, internal: `loadFile()`, `play()/pause()`, `seek()`, `tick()`, sticky-message re-emit, 20 Hz emitter + 10 Hz UI tick | Main-process replay engine. Indexes a .tlog or .bin file once, then streams its MAVLink messages into the renderer via the same `mavlink-message` IPC channel live connections use — the UI is animated without awareness of the source. .tlog uses a replay-scoped `MavLinkPacketSplitter`/`MavLinkPacketParser` from node-mavlink; .bin uses a custom minimal parser. Sticky whitelist re-emits the last HOME/MODE/etc. on seek so the UI stays coherent |
| `log-replay-bin-parser.js` | `parseBinLog()` | Minimal ArduPilot DataFlash (.bin) parser. Decodes a whitelisted subset (ATT/GPS/AHR2/BARO/ARSP/BAT/MODE/MSG/RCIN/RCOU/VIBE/ORGN) and synthesizes MAVLink-shaped records for IDs 30/33/24/74/1/0/253/35/36/241/242. Trajectory comes from AHR2 (EKF-smoothed), velocity carries over from the most recent GPS sample; VFR_HUD is synthesized from GPS speed + (barometer climb when present) so the HUD shows real values on logs without sensors |
| `preload.js` | Context bridge: `mavlink`, `msp`, `missionStore`, `sitl`, `rtk`, `fpv`, `telForward`, `adsb`, `tlogLogger`, `logReplay`, `corvSerial`, `topography` (load/loadOne/save), `models`, `windowControls`, `devtools` | Secure IPC bridge between main and renderer processes (15 namespaced APIs). Topography API supports load, loadOne, and save for offline SRTM management. `corvSerial` and `msp` expose the non-MAVLink link bridges; `missionStore` exposes list/load/save/delete/rename plus the data-root path; `logReplay` exposes open-file / play / pause / seek / unload + tick & state events |
| `sitl-manager.js` | `initSITLHandlers()`, `cleanup()` | Download ArduPilot SITL binaries, spawn process (native Linux or WSL on Windows), TCP 5760 |
| `rtk-manager.js` | `initRTKHandlers()`, `cleanup()` | RTCM3 frame parsing from serial GPS base station, GPS_RTCM_DATA (ID 233) injection to drone via raw MAVLink v2 packets |
| `fpv-manager.js` | `initFPVHandlers()`, `cleanupFPV()` | Spawn ffmpeg for RTSP-to-MJPEG conversion, extract JPEG frames (SOI/EOI markers), send base64 frames via IPC |
| `telforward-manager.js` | `initTelForwardHandlers()`, `cleanup()` | Forward telemetry as LTM protocol (G/A/S/O frames) or MAVLink passthrough to an external serial port, or mirror it over UDP |
| `msp-manager.js` | `initMSPHandlers()`, `cleanup()`, internal: `encodeRequest()`, `parseBuffer()`, `tick()`, `decode()`, `resolveModeName()`, `emit*()` | MSP/MSP2 adapter for INAV and Betaflight over serial or TCP. Frames both v1 (`$M`, XOR checksum) and v2 (`$X`, CRC8 DVB-S2). MSP is request/response, so a 20 ms scheduler polls a rate table with **one request in flight at a time** (MSP has no sequence numbers). Replies are decoded and re-emitted as synthetic MAVLink (30/74/24/33/27/1/147/65/0) on the same `mavlink-message` channel. Flight mode is resolved from the active mode boxes read once via `MSP_BOXNAMES`. A command unanswered 3× is dropped from the schedule, so an absent sensor cannot starve the poll budget |
| `mission-store.js` | `initMissionStoreHandlers()`, `getRoot()`, `getLogsDir()`, `getMissionsDir()`, `rebuildIndex()` | On-disk data root: `<install>/data/` with `index.json`, `missions/` and `logs/`, falling back to the per-user data folder when the installation directory is not writable (the default `Program Files` case). Missions are saved via temp-file + rename so an interrupted overwrite cannot destroy the previous version; `index.json` is rebuilt from the folder contents on every read and write, so it is a cache and never the source of truth |

### 3.2 Core (`js/core/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `state.js` | `STATE`, `dataBuffer`, `pushGHistory()`, `demoAttitude`, `demoSurveyState`, `activeTraces`, `viewMode`, `resetDataBuffer()`, `resetReplayState()` | Global mutable state (~100+ properties): attitude, position, velocity (incl. NED `vn/ve/vd`), MAVLink state, battery, GPS, RTK, vibration, mission, traffic. RingBuffer-backed time-series via `dataBuffer` proxy. `resetReplayState()` clears live-telemetry state without touching connection or preferences — called by Log Replay before loading a file and on backward seek |
| `constants.js` | `ORIGIN`, `CAMERA_FOV`, `VISIBILITY_RADIUS`, `BUFFER_SIZE`, `SAMPLE_INTERVAL`, `TRACE_CONFIG`, demo constants | All configuration constants (reference origin, camera, terrain chunks, demo mode) |
| `utils.js` | `latLonToMeters()`, `calculateDistance()`, `lerpColor()`, `getHeightColor()`, `calculateCRC16()`, `latLonToTile()`, `tileToBounds()` | Coordinate conversion (WGS84 → local meters), Haversine distance, color interpolation, terrain palette, CRC-16, tile math |
| `RingBuffer.js` | `RingBuffer`, `MultiChannelRingBuffer` | O(1) circular buffer (Float64Array) with binary search (`lowerBound`), array export, clear. Used for telemetry time-series |
| `LRUCache.js` | `LRUCache` | Least-recently-used eviction cache for terrain satellite textures. Prevents GPU memory exhaustion |
| `ExpressionParser.js` | `compileExpression()`, `validateExpression()`, `getAvailableFields()`, `ExpressionError` | Safe math expression evaluator (whitelist-based, no eval) for user-supplied formulas |

### 3.3 MAVLink (`js/mavlink/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `MAVLinkManager.js` | `initMAVLink()`, `onMessage(msgId, handler)`, `offMessage()`, `connectMAVLinkSerial/UDP/TCP()`, `disconnectMAVLink()`, `listSerialPorts()` | Renderer-side message router. Registers IPC listeners, dispatches messages to handlers, calls `mapMessageToState()`, fires `serialUpdate` CustomEvent |
| `MAVLinkStateMapper.js` | `mapMessageToState(msgId, data)`, `getFlightModeName()`, `getFlightModeNumber()`, `getAvailableFlightModes()`, `getGPSFixName()`, `getVehicleTypeName()`, `computeAeroAngles()` | Decodes 17+ MAVLink message types into STATE fields. Maintains ArduPilot mode tables for Copter/Plane/Rover/Sub. Computes AoA/SSA from NED velocity. **Position/velocity priority chain (v1.3.2)**: GLOBAL_POSITION_INT (msg 33) is the primary source — lat/lon/alt + EKF NED velocity, with `STATE.gs = √(vn²+ve²)`. VFR_HUD (msg 74) always provides airspeed; it only fills `gs`/`vs` when GLOBAL_POSITION_INT is stale (>2 s window). GPS_RAW_INT (msg 24) is the final fallback for lat/lon/alt/gs/track (sentinel 65535 filtered). This prevents VFR_HUD's GPS-only groundspeed on some firmwares from poisoning the HUD |
| `CommandSender.js` | `armVehicle()`, `disarmVehicle()`, `setFlightMode()`, `takeoff()`, `land()`, `returnToLaunch()`, `setGuidedTarget()`, `setParameter()`, `requestAllParameters()`, `requestAllDataStreams()`, `uploadMission()`, `sendRCChannelsOverride()`, `changeAltitude()`, `calibrateAccel/Compass/Gyro()`, `rebootAutopilot()` | High-level autopilot command abstraction with retry/ACK logic. Covers arming, modes, navigation, parameters, mission upload protocol, RC override, calibration |
| `ConnectionManager.js` | `connect(type, options)`, `disconnect()`, `getAvailablePorts()`, `isHeartbeatAlive()`, `getConnectionInfo()` | Connection lifecycle orchestrator. Auto-requests data streams and home position on MAVLink connect. Polls HOME_POSITION every 5s until received, restarts on re-arm. Supports serial, UDP, TCP, legacy corv-binary |

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
| `maps/MapEngine.js` | `initMap()`, `updateMap()`, `invalidateSize()`, `updateMissionOverlay()`, `setTargetMarker()`, `clearTargetMarker()` | Leaflet 2D mini-map with Google Satellite imagery (cache-first). Aircraft SVG marker, red trail polyline (3000 points max, downsampled), mission waypoint circles, home marker, ADS-B traffic dots, click-to-go guided target |
| `maps/TileCache.js` | `TileCache` class: `get()`, `put()`, `getStats()`, `clear()`, `enumerateTiles()`, `bulkDownload()` | IndexedDB-based persistent tile cache. Cache-first strategy for satellite/OSM tiles. Bulk download engine with configurable concurrency (6 parallel). Tile enumeration for bounding-box download |
| `maps/CachedTileLayer.js` | `cachedTileLayer()` | Drop-in Leaflet TileLayer replacement with IndexedDB caching. Uses native `<img>` loading (CORS-safe for Electron), opportunistic cache-write via canvas→blob conversion |
| `maps/OfflineDownloader.js` | `estimateOfflineDownload()`, `startOfflineDownload()`, `initOfflinePanel()` | Bulk download engine for satellite tiles + SRTM1 elevation data. SRTM1 from AWS Mapzen (free, no auth). Gzip decompression via DecompressionStream. Saves HGT files to disk via IPC, registers in terrain engine immediately |

### 3.7 UI Controllers (`js/ui/`)

| File | Key Exports | Purpose |
|------|-------------|---------|
| `UIController.js` | `updateUI()`, `initHudCells()`, `toggleConfig()`, `toggleTelemetry()`, `updateOffset()`, `updateAGLDisplay()`, `setStatusMessage()`, `updateFPSDisplay()`, `initMoreMenu()`, `initConfigAutoClose()` | Telemetry display updates (40+ fields), configurable 2×3 HUD cell grid, config/telemetry panel toggles, FPS counter. Persists cell config to localStorage |
| `TabController.js` | `initTabs()`, `getCurrentTab()`, `initSurveyGrid()` | Tab-based page navigation (Flight Data, Flight Plan, Setup, Sys Config), mission editor UI, survey grid planner |
| `CommandBarController.js` | `initCommandBar()`, `updateCommandBar()` | Bottom command bar: ARM/DISARM button, flight mode dropdown (color-coded: yellow=manual, cyan=assisted, blue=auto, orange=RTL), battery/GPS/link indicators, flight timer |
| `GCSSidebarController.js` | `initGCSSidebar()`, `updateGCSSidebar()`, `getTargetCoords()` | Right sidebar: connection panel (serial/UDP/TCP port selection), SITL launcher, RTK base station, telemetry forwarding config |
| `ParametersPageController.js` | `initParamsPage()`, `toggleParamsPage()`, `formatParamValue()` | Full ArduPilot parameter editor with search, inline edit, save. Side catalog reads single parameters via `PARAM_REQUEST_READ` (serialized queue + retries) so a slow link never needs the full list |
| `ParamCatalog.js` | `getCatalog()`, `getGroups()`, `groupOf()`, `learnNames()`, `toggleFavorite()` | Parameter-name catalog per vehicle class: built-in seed + names learned from vehicles/.param files, persisted in localStorage |
| `FPVController.js` | `initFPV()`, `onFPVButtonClick()`, `setFPVActive()`, `stopFPVStream()`, `resizeFPV()`, `openFPVSettings()` | FPV camera overlay on 3D view. ffmpeg stream controls, SIYI HM30 / generic RTSP settings dialog |
| `LoadingOverlay.js` | `showLoadingOverlay()`, `hideLoadingOverlay()`, `checkInitialLoadComplete()`, `scheduleHideLoadingOverlaySoon()` | Animated splash screen with cloud parallax and plane animation, terrain loading progress bar |

### 3.8 Other Modules

| File | Key Exports | Purpose |
|------|-------------|---------|
| `adsb/ADSBManager.js` | `fetchADSBData()`, `getNearestTraffic(n)`, `downloadTrafficCSV()` | OpenSky Network ADS-B traffic polling (50km radius, via main process for CORS bypass). Rate limited (10s), stale entry removal (60s), CSV export |
| `joystick/JoystickManager.js` | `JoystickManager` class | Gamepad API polling at 25 Hz. Axis mapping (roll/pitch/yaw/throttle), deadzone, expo, inversion config. Sends RC_CHANNELS_OVERRIDE (1000–2000 PWM). Config persisted to localStorage |
| `joystick/JoystickUI.js` | `initJoystick()` | Joystick configuration UI: gamepad selection, axis live display, channel mapping |
| `logging/TlogLogger.js` | `TlogLogger` class | `.tlog` flight recording (raw MAVLink v2 packet capture). Auto-starts on MAVLink connect, auto-stops on disconnect — the actual file write happens in `main-mavlink.js`; this class is a renderer-side controller over IPC |
| `logging/LogReplayController.js` | `initLogReplay()` | Log Replay UI controller. Wires the GCS sidebar (OPEN FILE / UNLOAD / file info) and the bottom-right timeline (play/pause, scrubber, current/total). Gates visibility on connection state — replay is only allowed while disconnected, and going live auto-unloads. On file load, swaps to a dedicated replay model, draws the full flight trail in red (slightly under the camera plane), and on backward seek invokes `resetReplayState()` + clears the trail |
| `mission/MissionHistory.js` | `commitMission()`, `undoMission()`, `redoMission()`, `resetMissionHistory()` | Snapshot undo/redo for STATE.missionItems. Every editor mutation calls commitMission() afterwards; commits that change nothing are ignored |
| `mission/MissionLibrary.js` | `initMissionLibrary()`, `openMissionLibrary()`, `saveCurrentMission()`, `getCurrentMissionName()` | Local mission library UI: list, load, overwrite, rename, delete. Talks to mission-store.js over IPC |
| `mission/MissionCommands.js` | `MISSION_COMMANDS`, `getCmdDef()`, `getCmdName()`, `getCmdParams()`, `isNavCmd()`, `getGroupedCommands()` | MAVLink mission command catalog (100+ commands). Categories: Navigation, Condition, DO, Camera/Gimbal. Used by mission planner UI and CommandSender.uploadMission() |
| `serial/SerialHandler.js` | `connectSerial()` | CORV binary protocol v7/v8 via WebSerial API (460800 baud). Custom packets: `[0xA5, 0x5A, TYPE, LEN, PAYLOAD, CRC16]`. Decodes Navigation / Debug (with particle filter ESS/spread/N) / Raw Sensor frames. Re-emits as synthetic MAVLink (msgs 30/33/74/26/24/0) so the renderer telemetry pipeline is shared |

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

### 4.6 Log Replay Pipeline (.tlog / .bin)

```
User opens .tlog or .bin file
    │
    ▼
log-replay-manager.js  (main process)
    │ indexFile() ── timestamp + offset table
    │ format-detect: tlog (MAVLink raw stream) | bin (ArduPilot DataFlash)
    │
    ├── .tlog path
    │   │ replay-scoped MavLinkPacketSplitter / MavLinkPacketParser
    │   │ handlePacket() ── full decode (same as live)
    │
    └── .bin path
        │ log-replay-bin-parser.js
        │ AHR2 → GLOBAL_POSITION_INT (33), with GPS velocity carry-over
        │ GPS  → GPS_RAW_INT (24) + VFR_HUD synthesis (gs from GPS,
        │        climb from BARO if present)
        │ ATT  → ATTITUDE (30); BAT/MODE/MSG/RCIN/RCOU/VIBE/ORGN → 1/0/253/35/36/241/242
        ▼
    emitFakeMavlinkMessage()  via 20 Hz wall-clock tick (50 ms slices,
                              max 200 packets per tick)
        │
        ▼
IPC: 'mavlink-message'        (same channel as live connections)
        │
        ▼
MAVLinkManager → mapMessageToState → STATE  →  60 FPS render
        │
        │ Plus 10 Hz UI tick:
        │   'logReplay-tick' { tMs, durationMs, playing }  →  scrubber
        │
        │ Sticky re-emit on backward seek:
        │   re-emits the last HEARTBEAT/HOME_POSITION/MODE/BATTERY so
        │   the UI stays coherent after time travel
```

Replay is gated on telemetry connection state: it only runs while disconnected, and going live auto-unloads.

### 4.7 Offline Data Download Pipeline

```
User (Sys Config → Offline Data Download panel)
    │ lat/lon rectangle + zoom level + type selection
    ▼
OfflineDownloader.js
    │ startOfflineDownload()
    │
    ├── Phase 1: Satellite Tiles
    │   │ enumerateTiles(bbox, zoom 1→maxZoom)
    │   ▼
    │   TileCache.js → bulkDownload()
    │   │ 6 concurrent fetches → IndexedDB 'datad-tile-cache'
    │   ▼
    │   Available to CachedTileLayer (cache-first)
    │
    └── Phase 2: SRTM1 Elevation
        │ enumerate HGT files (N44E022.hgt, ...)
        ▼
        AWS Mapzen: elevation-tiles-prod.s3.amazonaws.com/skadi/
        │ fetch .hgt.gz → DecompressionStream → ArrayBuffer
        │
        ├── IPC: topography-save → disk (topo/N44E022.hgt)
        └── TerrainManager.addHGTFile() → immediate use
```

### 4.8 MSP Polling Pipeline (INAV / Betaflight)

MAVLink pushes; MSP does not. Nothing arrives unless the GCS asks, so the adapter owns a
scheduler instead of a parser loop:

```
Connect (serial 115200 or TCP)
    │
    ├── one-shot queue: MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BOXNAMES, MSP_BOXIDS
    │     └── box names are mandatory: without them the flight mode cannot be resolved
    │
    └── 20 ms tick ──▶ pick the most overdue command in the rate table
                          │  (skipped entirely while a request is in flight —
                          │   MSP has no sequence numbers, so two outstanding
                          │   requests cannot be matched to their replies)
                          ▼
                     encodeRequest() ──▶ $M< (v1) or $X< (v2) ──▶ link
                          │
                          ▼
                     reply ──▶ parseBuffer() ──▶ checksum/CRC8 ──▶ decode()
                          │                             │
                          │                             └── '!' error frame → clear pending
                          ▼
                     accumulate into the vehicle-state struct
                          │  (one MAVLink message needs fields from several MSP replies:
                          │   VFR_HUD wants speed + altitude + heading from three commands)
                          ▼
                     emit*() ──▶ 'mavlink-message' IPC ──▶ renderer (as in 4.1)

    Timeout (500 ms normal / 2 s slow link) ──▶ miss counter++
                          └── 3 consecutive misses ──▶ command dropped from the schedule
```

Poll rates, per profile:

| Command | Normal | Slow link | Becomes |
|---------|--------|-----------|---------|
| `MSP_ATTITUDE` | 25 Hz | 5 Hz | `ATTITUDE` (30) |
| `MSP_ALTITUDE` | 10 Hz | 2 Hz | `VFR_HUD` (74) |
| `MSP_RAW_GPS` | 5 Hz | 2 Hz | `GPS_RAW_INT` (24) + `GLOBAL_POSITION_INT` (33) |
| `MSP2_INAV_STATUS` | 5 Hz | 1 Hz | `HEARTBEAT` (0) — armed state + mode boxes |
| `MSP_RAW_IMU` | 5 Hz | — | `SCALED_IMU` (27) — feeds the G-load indicator |
| `MSP_RC` | 5 Hz | — | `RC_CHANNELS` (65) |
| `MSP_COMP_GPS` | 2 Hz | 0.5 Hz | home distance / bearing |
| `MSP2_INAV_ANALOG` | 2 Hz | 0.5 Hz | `SYS_STATUS` (1) + `BATTERY_STATUS` (147) |
| `MSP2_INAV_AIR_SPEED` | 2 Hz | — | airspeed field of `VFR_HUD` |

---

## 5. Key Integration Patterns

### 5.1 Single Source of Truth (STATE)
All telemetry flows through the global `STATE` object in `core/state.js`. The 60 FPS render loop reads STATE — no UI component queries the autopilot directly. `MAVLinkStateMapper` writes to STATE; all rendering and UI modules read from it.

### 5.2 IPC Bridge Architecture
`preload.js` exposes 13 namespaced APIs via `contextBridge.exposeInMainWorld()`: `mavlink`, `sitl`, `rtk`, `fpv`, `telForward`, `adsb`, `tlogLogger`, `logReplay`, `corvSerial`, `topography`, `models`, `windowControls`, `devtools`. All IPC uses `invoke`/`handle` (request-response) or `send`/`on` (events). Security: `contextIsolation: true`, no `nodeIntegration`.

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

### 5.7 Vehicle Abstraction
`MAVLinkStateMapper` maintains mode tables for ArduPilot Copter, Plane, Rover and Sub, plus one for INAV. Vehicle type is auto-detected from the HEARTBEAT `type` field; mode names and the available-mode list adapt to it.

The **link protocol wins over `MAV_TYPE`** when resolving mode names: an INAV plane also reports type 1, but its mode numbering has nothing to do with ArduPlane's, so `getModesForType()` checks `STATE.connectionType` first. The synthetic mode numbers the MSP adapter emits and the `INAV_MODES` table in the mapper are two halves of one contract — changing one without the other silently mislabels every flight mode.

> **Known gap.** There is no capability model yet: with an MSP link connected the parameter page, mission upload and the ARM/mode buttons still try to send MAVLink, which fails harmlessly but is not gated in the UI. A `STATE.caps` flag set per protocol is the natural fix.

### 5.8 Unified Telemetry Pipeline (live / CORV / MSP / replay)
Four completely different data sources converge on a single render path:

| Source | Decoder | Emits |
|--------|---------|-------|
| Live MAVLink (serial/UDP/TCP) | `main-mavlink.js` | parsed MAVLink |
| CORV binary v7/v8 | `main-mavlink.js` (`corvEmit*`) | synthetic MAVLink |
| MSP / MSP2 (INAV, Betaflight) | `msp-manager.js` | synthetic MAVLink |
| Log replay (.tlog / .bin) | `log-replay-manager.js` + `log-replay-bin-parser.js` | replayed / synthesized MAVLink |

All four emit MAVLink-shaped messages onto the same `mavlink-message` IPC channel → `MAVLinkManager.handleMessage()` → `mapMessageToState()` → STATE. UI modules read STATE without knowing the source. This is what makes the velocity priority chain, the HUD, the 3D view and the charts work uniformly across every link type — **and it is the contract any new protocol adapter must satisfy**: decode in the main process, emit MAVLink, touch nothing in the renderer.

### 5.9 Snapshot Undo/Redo
`mission/MissionHistory.js` keeps whole-mission snapshots rather than an inverse-command log. A mission is at most a few hundred small plain objects, so cloning costs less than the bookkeeping — and, more importantly, it cannot drift out of sync with `STATE.missionItems`, which a dozen call sites in `TabController` mutate directly. Editors mutate first and call `commitMission('label')` afterwards; a commit that changes nothing is ignored, so handlers that may be no-ops are safe to instrument. Snapshots are applied **in place** because other modules hold a reference to the array.

---

## 6. On-Disk Layout

Everything the application writes at runtime lives under one root, resolved once at startup
by `mission-store.js`:

```
<data root>/
├── index.json      catalogue of missions + logs (rebuilt on every read/write)
├── missions/       saved missions, one .json each
└── logs/           .tlog and .crv flight recordings
```

| Situation | Root |
|-----------|------|
| Packaged, installation folder writable | `<folder containing the .exe>/data` — the install is portable |
| Packaged under `Program Files` (not writable without elevation) | `%APPDATA%\CORV GCS\data` |
| Development (`npm start`) | `<project folder>/data` (git-ignored) |

Read-only inputs the operator supplies stay outside `data/`: `topo/` or `topography/` for SRTM
`.hgt` tiles and `models/` for GLB/GLTF airframes, both inside the installation folder.

**Mission file format** (`missions/<id>.json`):

```json
{
  "format": "corv-gcs-mission", "version": 1,
  "name": "Patrol circuit", "notes": "",
  "created": "2026-08-06T10:00:00.000Z", "modified": "2026-08-06T12:30:00.000Z",
  "vehicleType": 1, "meta": {},
  "items": [ { "seq": 0, "command": 16, "lat": 45.0, "lng": 11.0, "alt": 100, "frame": 0,
               "param1": 0, "param2": 0, "param3": 0, "param4": 0 } ]
}
```

The `id` is the filename stem, derived from the name on first save and stable across renames —
so overwriting keeps writing the same file instead of accumulating copies.

Browser-side state (parameter catalogue learned names, starred parameters, joystick mapping,
UI preferences) stays in `localStorage`, not in `data/`.

---

## 7. Dependencies

| Package | Version | Role |
|---------|---------|------|
| `electron` | ^39.2.7 | Desktop app runtime (Chromium + Node.js) |
| `electron-builder` | ^26.8.1 | Build & packaging (NSIS, AppImage, deb) |
| `node-mavlink` | ^2.3.0 | MAVLink v2 protocol parse/serialize (ardupilotmega dialect) |
| `serialport` | ^13.0.0 | Native serial port access |
| Three.js | r128 | 3D rendering (loaded via CDN in HTML) |
| Leaflet | 1.9.4 | 2D map tiles (loaded via CDN in HTML) |

---

## 8. Build & Run

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
