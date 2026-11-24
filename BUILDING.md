# Building CORV GCS

CORV GCS from version 2.0 on is a native rewrite (Qt 6.12 LTS, C++, QML, QRhi) of the Electron
CORV GCS (1.8.8). Qt modules under the LGPLv3 only; the program is Apache-2.0. The implementation
guide it follows, with the state of every phase and where the program departs from it, is
[docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md) (in Italian).

This page is for who builds or changes the program; what it does is in the [README](README.md), how
to use it in the [wiki](https://xarin94.github.io/Corv-GCS/).

## Status

| Phase | Content | State |
| --- | --- | --- |
| 1 Foundations | CMake, presets, frameless QML window, CI on three systems (Qt for Windows installed by `tools/ci/install-qt-windows.py`: aqtinstall 3.3 does not know the 6.12 repository layout), Linux AppImage (`tools/build-appimage.sh`), Windows NSIS installer (`tools/package-windows.ps1`: per user, choice of folder, VC++ runtime app-local, 35 MB with the bundled LTE cells and SDL3) and portable zip, macOS DMG (bundle, macdeployqt, universal, ad-hoc signed), all three built by CI and attached to the GitHub release of a `v*` tag (the tag is the version the program reports); update check at start against the public repository's releases, one-click install (Windows setup `/UPDATE`, AppImage replaced in place, DMG opened) | Done; signed / notarized packages not yet |
| 2 Link and state | MAVLink (ardupilotmega + development + CORV_CELL_INFO), serial / UDP / TCP, CORV-2 binary, demux, tlog, trace, StateMapper, PoseSmoother, commands, connection manager, `corv-link-cli`; MAVLink FTP and @PARAM/param.pck (READ ALL in 0.08 s on SITL); cellular link to an LTEtelem module through the relay (TLS, CRV3 Ed25519 access, AES-256-GCM end to end, replay window, reconnection; OpenSSL libcrypto); RTK / GPS inject (NTRIP caster with sourcetable and GGA, serial base, GPS_RTCM_DATA); telemetry forward (MAVLink mirror over UDP client / server / serial with write access, LTM); MSP v1 / v2 over serial and TCP (INAV / Betaflight state, INAV mission read and write) | Done, tried on SITL and against independent fakes of caster, relay, module and an MSP board; MSP on a real board to try |
| 3 Terrain 3D | HGT store with automatic SRTM1 download of the missing tiles (AWS, saved to `<data>/terrain`), 1/30 degree chunks, QRhi renderer with texture-array heights, screen-space LOD, schematic shader, fog, satellite imagery on the chunks (zoom bands by distance, BC1 texture arrays with mips; RGBA8 on the OpenGL backend, where BC1 arrays sample black on Mesa); overlays in screen-space thick lines with a ghost pass (trail, predicted trajectory corridor, home, mission route with symbols and loiter rings, ADS-B contacts, ownship ring); vehicle models (GLB of the vehicle type, model scale); first person with arrow-key look-around and horizon lock, third person orbit (drag, wheel); terrain clipping (off / vehicle / all); water surface grid and underwater specks; sun for the time and place; text labels (HOME, mission items, traffic) from a glyph atlas; SatelliteDetail (four clipmap textures of zoom 17-20 around the vehicle, SYS CONFIG → SATELLITE DETAIL); outlines of the schematic view (ridges and skyline found in the depth, three passes); UAS geozones painted on the terrain; LTE cell poles and the dashed link to the serving mast; debug server (`--debug-port`) | Image parity measurements; the vehicle model is not outlined (it was in Electron) |
| 4 Flight screen | HUD as separate instruments (HUDRenderer.js port, GPU tint, repaint on a pixel of change: 0.1-0.3 ms mean, under 1.5 ms worst per frame), command bar, connection sidebar, telemetry cells, mini-map, annunciator column (warnings / cautions, STATUSTEXT holds, silence on click, demo sequencer), EKF STATUS bar (ArduPilot EKF3 charts, flags, 5 Hz rates while open), FFT bar (strongest lines, live spectrum with peak hold, 60 s waterfall, logarithmic frequency axis), ROTOR LOAD schematic (per-motor PWM gauges on the airframe layout, built from the SERVOn_FUNCTION assignments: MotorN on the ring, a quadplane's pusher or a twin's engines in the middle; SAIL · RUDDER dials for a sailboat; FLAPS indicator over it for flap / flaperon outputs, Airbus EIS style, −10° to +30°; servo parameters read only while it is enabled), mode change banner, ADS-B traffic table (OpenSky every 30 s and ADSB_VEHICLE), LTE cell rows of the position box (LTE OVERLAY) | A full SITL flight |
| 5 Mission | RouteModel, RouteCompiler (INAV too), MissionTransfer, MissionStore v2, Platforms, RadioLink, FLIGHT PLAN page with undo/redo, layered map, profile with radio band, mission library, upload / read (MAVLink, and INAV over MSP); mission in 3D; UAS geozones (EU ED-269 / ED-318 files, FAA, DFS DIPUL, IGN, ENAIRE, BAZL) on the map with legend, tooltips and annunciators | Parity tests with missions saved by Electron, transfer on SITL |
| 6 Setup | SETUP → CONNECTION, SETUP → PARAMETERS (name catalog with favourites, READ ALL over FTP with PARAM_REQUEST_LIST fallback, queued single reads, in-place edit and WRITE CHANGED, .param files; ArduPilot documentation of every parameter from apm.pdef.json bundled per firmware by `tools/gen-param-meta.py` and updatable from autotest.ardupilot.org: description, units, range, value lists as selects, bitmask editor), SETUP → LOG DOWNLOAD (DataFlash logs on the SD card over LOG_REQUEST_LIST / LOG_REQUEST_DATA with gap filling, download queue, erase, open in LOG REPLAY), SETUP → ACCELEROMETER / COMPASS / GYRO / BARO (six positions following ACCELCAL_VEHICLE_POS, level, simple accel, onboard mag cal with the 80-section 3D sphere, large-vehicle magcal, stored calibration and compass tables), SYS CONFIG (every section, Electron keys in `settings.json`); SETUP → CELLULAR LINK, TELEMETRY FORWARD, RTK / GPS INJECT, FLIGHT MODES (the vehicle's own mode table), FAILSAFE, PID TUNING (panels of the vehicle class), EXT TUNING, RADIO CAL, VIBRATIONS, SERVO / RELAY, CORV SETUP (SystemConfig v9 over 0x10 / 0x11), SIMULATION (SITL launcher, WSL on Windows) | Calibrations tried on SITL up to what a still vehicle allows (a compass run to SUCCESS needs the `calibration` SITL model); JOYSTICK / RC (SDL3: gamepads in the standard layout, other joysticks raw; 8 or more inputs to RC channels with deadzone and expo, RC_CHANNELS_OVERRIDE at 10 / 25 / 50 Hz with the ArduPilot parameters saved and restored, 500 ms failsafe, servo and look buttons) | LIDAR and ROS pages done; the joystick tried without a device only |
| 7 Sensors and video | ADS-B: OpenSky polling, the traffic table, contacts on the mini-map and in 3D; LTE cell of the module on the flight screen and in CELLULAR LINK; LTE cell database (user CSV files, the cells of Italy, San Marino and Vatican City in `celldb/`, the 2024 OpenCelliD PMTiles copy read by blocks with HTTP ranges, online lookup on beaconDB / the OpenCelliD API) and the mast map: sites within 10 km and the serving one on the mini-map and in 3D, mast and distance in CELLULAR LINK and on the flight screen; Livox Mid-360 point cloud, ROS surfaces | FPV |
| 8 Logs and services | Rolling debug log, offline SRTM and tile download, SQLite tile store, LOG REPLAY of .tlog (memory-mapped, offsets only) and .bin (DataFlash → real MAVLink frames) on its own thread with timeline, seek and sticky messages, vibration spectra from the log's IMU samples (GYR / ACC, batch sampler, IMU; Welch, SYS CONFIG → FFT FROM LOGS) | Done for now; SITL launcher in SETUP → SIMULATION |
| 9 Release | | Not started |

## Layout

```
libs/core     geodesy, ring buffers, LRU, data root, settings, rolling debug log
libs/link     transports, MAVLink parser, CORV-2, demux, tlog, trace, LinkManager (link thread)
libs/vehicle  VehicleState, StateMapper, modes, SceneOrigin, RelativeNav, PoseSmoother,
              VehicleHub (snapshots), CommandSender, ConnectionManager
libs/input    gamepads and joysticks (SDL3, optional) and the RC mapping of the joystick
libs/terrain  HGT tiles, chunk manager, tile store (SQLite), satellite chunk imagery, BC1
libs/services log replay (.tlog / .bin index, replay engine on the log thread), spectra from logs,
              geozones, LTE cell database (CellDb)
libs/scene    renderer-neutral data (TerrainData, models, geozones, cell overlay)
libs/gpu      QRhi renderer and the SceneItem (the only code that includes QRhi)
libs/ui       QML module Corv.Ui: HUD instruments, view models, windows
app/          main.cpp, TestDriver (test commands), DebugServer (--debug-port)
shaders/      Vulkan-style GLSL, baked by qsb at build time
tools/        corv-link-cli, MAVLink generator (corv.xml), build and package scripts, debug-client.py
resources/    fonts, icons, vehicle models, SITL defaults, celldb/ (the bundled LTE cells)
tests/        Qt Test ports of the scripts/test-*.js suites
```

## Build (Windows)

Visual Studio 2022 Build Tools and Qt 6.12 for MSVC 2022 64-bit
(default `%USERPROFILE%\Qt\6.12.0\msvc2022_64`, or set `QT_DIR`):

```powershell
powershell -ExecutionPolicy Bypass -File tools\build.ps1 -Test
build\windows-release\corv-gcs.exe --connect tcp:<host>:5760
```

Executables land in `build\windows-release`; put Qt's `bin` on `PATH` to run
them from there (packaging copies the libraries next to the executable).
OpenSSL (libcrypto, for the cellular link) comes from `OPENSSL_DIR` or
`build\tools\openssl-*`; without it the build has no LTE link. SDL3 (joystick
input) comes from `SDL3_DIR` or `build\tools\sdl3\SDL3-*`; without it the build
has no joystick. `tools\package-windows.ps1` fetches both when missing.

Installer: `powershell -ExecutionPolicy Bypass -File tools\package-windows.ps1 [-Zip]`
builds, stages (windeployqt, VC++ runtime app-local, libcrypto, notices) and makes
`dist\CORV-GCS-<version>-setup-x64.exe` with NSIS: per-user install in
`%LOCALAPPDATA%\Programs\CORV Systems\CORV GCS` (folder selectable), Start menu and desktop
shortcuts, uninstaller that keeps the `data` folder. It sits next to, not over,
CORV GCS 1.x (`%LOCALAPPDATA%\Programs\CORV GCS`, which the 1.x uninstaller deletes
whole). Missing NSIS / OpenSSL are fetched into `build\tools` (NSIS 3.11
portable, OpenSSL 3.5 for MSVC) unless `-NoDownload`.

Link checks without the interface: `corv-link-cli serial COM5 57600`,
`corv-link-cli corv COM7 460800`, `corv-link-cli lte HOST 5765 --module ID --key HEX [--cert SHA256]`
print the vehicle state once a second and warn when bytes arrive but no MAVLink
or CORV-2 frame checks out (wrong baud rate or protocol).

Debug server: `--debug-port PORT` starts a JSON-RPC 2.0 server on 127.0.0.1 only
(one request per line): `set` / `get` {object, property, value} on a view model or a
named QML item, `call` {target: "Gcs.setMode", args: [...]} with the return value,
`stats` (what the 3D views report), `screenshot` {file} (no file: base64 PNG) and
`script` {line} for any of the commands below. `python tools/debug-client.py PORT METHOD [JSON]`
sends one request.

Test hooks: `--replay FILE` (a .tlog or .bin into LOG REPLAY), `--grab FILE --grab-after MS`,
`--quit-after MS`, and `--grab-script FILE` with lines `MS tab N`, `MS grab FILE`, `MS key Left` / `MS keyup Left` (arrow keys),
`MS emit toggleSatellite` (any header button signal), `MS call Params.readAll`
(a view-model method of `Params`, `Plan`, `Gcs`, `Calibration`, `Ekf`, `Fft`, `Replay`, `Annunciators`, `Rtk`, `TelForward`, `Lte`, `SetupCfg`, `Sitl`, `GeoZones`, `Lidar` or `Ros`), `MS set setupPage current PARAMETERS` (a property of a
named QML item or of a view model, e.g. `set flightView swapped true`, `set scene orbitPitch -0.7`), `MS size WxH`.
Grabs a few tenths of a second apart keep the GUI thread busy and stop the 3D view from
syncing while they last: grab seldom. Direct3D 11 renders no frames while the window is
covered; visual checks on a covered window use `QSG_RHI_BACKEND=opengl`.
`CORV_DEMO_ORIGIN=lat,lon` starts the demo flight somewhere else than Innsbruck (water, sea).

SRTM tiles: `topo/` next to the executable, `<data root>/terrain`, the
project's `topo/` in development, or folders listed in `CORV_TOPO_DIRS`.

## Build (Linux)

Qt 6.12 for GCC 64-bit with the CI modules, e.g. with aqtinstall:
`aqt install-qt linux desktop 6.12.0 linux_gcc_64 -m qtserialport qtshadertools qtwebsockets qtimageformats -O ~/Qt`
and `aqt install-tool linux desktop tools_ninja -O ~/Qt`. Qt's GuiPrivate needs the
libxkbcommon headers (`libxkbcommon-dev`); without root they can be unpacked from
the .deb and passed as `XKB_INCLUDE_DIR` / `XKB_LIBRARY`.

```sh
export PATH=~/Qt/Tools/Ninja:$PATH
cmake --preset linux-release -DCMAKE_PREFIX_PATH=~/Qt/6.12.0/gcc_64
cmake --build --preset linux-release
QT_QPA_PLATFORM=offscreen ctest --preset linux-release
LD_LIBRARY_PATH=~/Qt/6.12.0/gcc_64/lib build/linux-release/app/corv-gcs --connect tcp:127.0.0.1:5760
```

SITL on Linux listens on TCP 5760 directly (`arduplane --model plane -I0 ...`).

AppImage: `RUNTIME=<AppImage type-2 runtime> tools/build-appimage.sh` builds
`dist/CORV-GCS-<version>-x86_64.AppImage` from the local Qt (only the QML modules
the interface imports, the executable stripped) and downloads nothing. The runtime
comes from https://github.com/AppImage/type2-runtime/releases, or is cut out of any
type-2 AppImage given as RUNTIME. It runs on systems with a glibc at least as new
as the build machine's. Data go to `~/.config/CORV GCS/data`, shared with Electron.
`CORV_VERSION_LABEL=2.0.0-alpha.8` stamps the version (and the file name); without
it the build reports `2.0.0-dev` and is never offered an update.

Updates: a few seconds after start (SYS CONFIG → UPDATES can switch it off) the
GCS lists the releases of `CORV_UPDATE_REPO` (CMake, default `Xarin94/Corv-GCS`)
on the GitHub API, anonymously, and offers the newest version above its own that
has a package for the system: UPDATE at the top right, the notes, INSTALL. Only
then is the package downloaded and checked against GitHub's SHA-256.
`CORV_UPDATE_URL` points the check at another list (tests).

## Data

Same data root and formats as the Electron version (`data/` next to the
executable when writable, else `%APPDATA%\CORV GCS\data`): missions, logs
(`.tlog` with little-endian µs stamps), debug log, terrain. New: `settings.json`
(was localStorage) and `tiles/` (was IndexedDB). The LTE cell database keeps the
Electron layout in `celldb/` (blocks, leaves, sources, index, lookups), so both versions share it.

## SITL from Windows

SITL runs in WSL; connect to the WSL address (`wsl hostname -I`), not
127.0.0.1: the localhost forwarding drops the connection and SITL exits.
