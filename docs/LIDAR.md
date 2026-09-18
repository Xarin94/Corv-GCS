# Livox Mid-360 point cloud in Corv-GCS

Live 3D map of what the LiDAR sees, built on the ground station from the raw
Livox stream and the MAVLink telemetry, drawn in the 3D view and saved as `.ply`.

```
                 MAVLink (radio)                 ┌───────────────┐
  ArduPilot ────────────────────────────────────▶│               │
                                                 │   Corv-GCS    │──▶ 3D view (point cloud)
  Livox Mid-360 ── Ethernet ── LAN bridge ──────▶│               │──▶ data/lidar/*.ply
                (UDP, Livox SDK2 protocol)       └───────────────┘
```

Two independent links: the autopilot never sees the LiDAR, the LiDAR never
sees the autopilot. The GCS is where they meet.

## 1. Setup

### Network
- The Mid-360 has a fixed IP `192.168.1.1XX` (XX = last two digits of the S/N)
  and talks UDP only. The host must be reachable from it on ports **56101 /
  56201 / 56301 / 56401** (command ack, state push, point cloud, IMU). With a
  transparent L2 bridge (Microhard, Doodle Labs, Ubiquiti, Wi-Fi bridge) the
  GCS PC simply gets an address on the LiDAR's `/24` and *HOST IP = AUTO*
  works. Through a router, set *HOST IP* to the address the LiDAR can route
  back to.
- Bandwidth: 32-bit Cartesian ≈ **24 Mbit/s**, 16-bit (centimetre) Cartesian
  ≈ 14 Mbit/s, both at the sensor's full 200 kpts/s. Pick the 16-bit format on
  narrow links; the map is voxelised at 25 cm anyway.
- Nothing needs to run on the aircraft: the GCS sends the SDK2 configuration
  itself (host IP/ports, point format, work mode NORMAL) and repeats it every
  second until the sensor answers and packets flow.

### Mount attitude (SETUP → TOOLS → LIDAR)
The autopilot reports attitude in its body frame (FRD: X forward, Y right,
Z down). The Mid-360 measures in its own frame (FLU: X forward toward the
connector side, Y left, Z up, origin at the optical centre). Enter how the
sensor is installed relative to the autopilot IMU:

| Field | Meaning |
|-------|---------|
| ROLL / PITCH / YAW | ZYX Euler angles of the LiDAR body relative to the aircraft body, degrees. Belly mount with the sensor upside down = roll 180. Presets cover the usual cases. |
| LEVER ARM X / Y / Z | Position of the LiDAR optical centre from the IMU (the point ArduPilot reports its position for), metres, FRD. |

A wrong sign here is immediately visible: with the aircraft rolling or
yawing, walls double and the ground splits into two sheets. Use the emulator
(section 5) to check a mounting before flying.

The Mid-360 field of view is 360° × (-7° … +52°): mounted flat under the
belly it sees everything **except a cone straight down** (half-angle 38°, so a
blind disc of radius 0.78 × height). Tilting the sensor 30–45° nose-down (a
preset is provided) or mounting it on the side closes the gap — this is why
most Mid-360 drone brackets are angled.

### Telemetry lag
Telemetry over a radio arrives later than the LiDAR packets over the bridge.
The GCS aligns the two on arrival time, so that offset must be told:
*TELEMETRY LAG* in ms shifts the pose lookup. Tune it by hovering next to a
wall and yawing: the lag is right when the wall stays one sheet. Typical
values: 0–50 ms for a direct IP link (SITL, Wi-Fi telemetry), 100–300 ms for
SiK-class radios.

Raise `SR_POSITION` and `SR_EXTRA1` to 10 Hz (MAVLINK STREAM RATES panel): the
pose is interpolated between samples, and 3 Hz position updates at 20 m/s mean
7 m between samples.

### Minimum range
Anything closer than *MIN RANGE* (default 2.5 m) is discarded before every
other filter: on an aircraft the rudder, the landing gear, struts and
antennas sit inside the Mid-360's field of view and return strong, stable
echoes that would otherwise be painted along the whole track as a ghost
airframe. Set it just beyond the farthest part of the airframe the sensor
can see.

### Accumulation gate
Points are added only while navigation is trustworthy:

- GPS fix ≥ the selected level (3D / DGPS / RTK float / RTK fixed), at least
  N satellites, HDOP under the limit;
- EKF healthy (ArduPilot `EKF_STATUS_REPORT`: attitude, horizontal velocity,
  absolute horizontal and vertical position flags set; position and velocity
  variances ≤ 0.5, the level Mission Planner colours amber);
- position, attitude and GPS messages fresh.

The strip on the flight screen shows **ACCUMULATING** or **LIVE ONLY ·
reason**, and the HUD announces each transition. For a survey-grade map
require RTK fixed, as DJI does for the L1/L2.

### Live points when the gate is closed
Points that cannot be georeferenced are not discarded: a subsample (16 of
every 96) is drawn **relative to the aircraft** — levelled into a north-up
frame when the attitude is fresh, in the body frame otherwise — and fades out
after *LIVE POINTS FADE AFTER* seconds (default 3). The operator keeps seeing
what the sensor sees on the ground, before take-off, or with a degraded fix;
only georeferenced points enter the map, the `.ply` export and the raw
recording.

## 2. What the GCS does with a packet

Every UDP packet carries 96 points in the LiDAR frame (mm) and is processed on
a worker thread:

1. **Hold** the packet for `lag + 60 ms` so the pose buffer has samples on
   both sides of its measurement time (interpolation instead of extrapolation).
2. **Pose at t** — attitude interpolated on the shortest arc, position
   interpolated, or dead-reckoned on the EKF velocity for at most 400 ms past
   the last sample.
3. Per point: `FLU → FRD`, `R_mount · p + lever`, `R_att · p`, `ENU = pos + (e, n, -d)`.
4. **Filters**: range window (drops the airframe and the zero "no return"
   points), Livox noise tags (bits 0–3 of the tag byte), then a **voxel
   occupancy hash** — one point per 25 cm cell (configurable). This bounds the
   map to a few million points and makes the renderer and the `.ply` export
   independent of flight duration.
5. Accepted points go to the map (ENU metres from an origin fixed at the
   first accepted point), to the renderer in ≤ 20 Hz batches, and — with
   *RECORD RAW* — every filtered point, pre-voxel, to `raw-*.ply` with a
   timestamp.
6. Packets that fail the gate (or arrive with no usable pose) go to the
   **live** layer instead: vehicle-relative, faded by the renderer's shader
   after the TTL, never stored.

**CLEAR MAP** drops the map and the origin; the next packet re-anchors.
**SAVE** writes `data/lidar/map-*.ply` (binary little-endian, `x y z`
float + `intensity` uchar, origin lat/lon/alt MSL in the header comments —
CloudCompare / QGIS / Open3D read it directly; add the origin as a global
shift to get projected coordinates).

## 3. How other software does it

| Product | Approach | What we took from it |
|---------|----------|----------------------|
| **DJI Zenmuse L1 / L2 + Pilot 2** | Real-time preview on the controller during flight: onboard RTK+IMU (INS) direct georeferencing, decimated cloud streamed to the tablet, colour by height / reflectivity / distance; RTK FIX required to record; the survey-grade result comes later from DJI Terra with the post-processed trajectory. | The live-preview posture (decimated, gated on nav quality, not the final product), the colour modes, the RTK-fixed gate option. |
| **YellowScan LiveStation / Mapper** | Applanix INS onboard, georeferenced cloud sent to the ground over the radio for real-time QC; PPK afterwards. | Same split: the map is a QC tool for coverage and gaps, the raw file is what post-processing consumes. |
| **Livox Viewer 2** | Raw frames in the sensor frame, integration time accumulates N frames, `.lvx2` recording; no georeferencing. | The reflectivity ramp and the noise-tag semantics. Not enough for a moving platform. |
| **FAST-LIO2 / Point-LIO / Faster-LIO (with `livox_ros_driver2`)** | Tightly coupled LiDAR-inertial odometry on a companion computer: IMU propagation at 200 Hz, iterated Kalman with point-to-plane residuals, incremental kd-tree map, motion compensation ("deskew") inside each 100 ms scan, voxel-downsampled map (0.5 m outdoors). Works without GPS. | The voxel-downsampled map and the idea of deskewing by pose interpolation. Full LIO needs the companion computer and the Mid-360 IMU stream; it is the natural next step if a GPS-denied map is wanted. |
| **Emesent Hovermap, Rock Robotic, GeoSLAM** | Onboard SLAM, cloud streamed decimated to a tablet over Wi-Fi. | Streaming a *decimated* cloud, never the raw one, to the operator. |
| **Mission Planner / QGC** | No 3D point clouds. Mission Planner shows the 2D "proximity" radar from `OBSTACLE_DISTANCE`. | — |
| **ArduPilot** | Does not parse Livox; Mid-360 users run FAST-LIO on a companion computer and feed `VISION_POSITION_ESTIMATE` / `OBSTACLE_DISTANCE` back. | Nothing to add on the vehicle for this feature; the companion-computer route is compatible with it. |

The common thread is **direct georeferencing** — `world = T_world_body(t) ·
T_body_lidar · p_lidar` with the pose interpolated at each point's time —
which is exactly what runs here. The difference between a live preview and a
survey product is the trajectory quality (RTK + tactical IMU + PPS time sync,
or LIO) and post-processing, not the maths.

## 4. Limits of this implementation

- **Time base.** The Mid-360 has no PPS input (PTP/gPTP only), and the GCS
  has no hardware timestamp for the telemetry either, so both streams are
  aligned on arrival time. Radio jitter shows up as blur, proportional to
  angular rate: 5 ms × 30°/s × 40 m range ≈ 10 cm. Fine for coverage, gaps,
  obstacles and terrain; not for a centimetre survey.
- **Pose rate.** Between telemetry samples the pose is interpolated; fast
  attitude changes between 10 Hz samples are lost. The vehicle's own IMU is
  not used (it never leaves the autopilot at full rate), and the Mid-360 IMU
  stream is received but not fused.
- **One point per voxel, first wins.** A surface scanned first at long range
  is not refined by a later close pass. Clear the map, or use a finer voxel,
  when re-scanning.
- **Flat-earth scene.** The 3D scene uses one fixed origin for the whole
  world; the cloud group is scaled so it stays coherent with the aircraft
  model. Within a mission area the error is nil; the `.ply` files use proper
  local ENU at the cloud origin.
- **Terrain mesh vs cloud.** SRTM is 30 m / ±5 m; with the depth test on,
  strips of ground vanish under the mesh. *Draw through the terrain mesh* is on
  by default.

## 5. Testing without the sensor

### Demo flight (no link, no SITL)
With no vehicle connected the GCS flies its demo circuit, 50 m over the
airport. Enable the point cloud there (SETUP → TOOLS → LIDAR) and a
**synthetic scan** starts: rays leave the demo aircraft in a Mid-360 pattern
mounted with the spin axis along the fuselage — the 360° sweep fans around
the flight axis, the -7°…+52° band leans it forward — and are intersected
with the real SRTM terrain plus a fixed field of invisible obstacles (forest
patches, hangars, a line of pylons). Nothing is drawn but the returns: the
trees and buildings exist only in the cloud, the way a real scan reveals
what the imagery does not. The strip reads **DEMO · SYNTHETIC SCAN**; CLEAR
MAP works, SAVE does not (nothing to keep). A subset of rays returns from
the demo airframe itself (wing roots, gear, fin, 0.5–2.4 m): lower *MIN
RANGE* under 2 m and those false echoes appear as a trail along the track,
which is exactly what the setting is for. The scan stops by itself when a
link comes up.

### Emulated sensor against SITL
```
# terminal 1: launch the GCS, start SITL (SETUP → SIMULATION), connect
# terminal 2:
node scripts/livox-sim.js --mount 180,0,0 --lever 0,0,0.1 --pps 1000
```

The emulator speaks the real SDK2 protocol on `127.0.0.2` (search, config
ack, point packets) and synthesises returns from a ground plane and four boxes
using the *true* vehicle pose read from SITL's second MAVLink port (TCP 5762).
In the GCS set *LIDAR IP* `127.0.0.2`, *HOST IP* an address of this machine,
and the same mount / lever arm passed to the emulator, then enable the point
cloud and take off. Sharp boxes and a flat ground mean the frame chain is
right; anything wrong in signs, order or lag smears them.

`node scripts/test-lidar-math.js` runs the same pipeline offline: SDK2
framing and CRCs, identity / yaw / inverted / rolled / lever-arm cases,
filters, gate, live path, batching, lag.

Measured on the development machine with the emulator at the sensor's real
rate (2 040 packets/s, 196 kpts/s, 22 Mbit/s): worker thread ≈ 15 % of one
core, renderer at 60 FPS while accumulating 34 kpts/s in flight, 2.4 M points
drawn.
