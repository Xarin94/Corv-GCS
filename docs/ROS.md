---
title: ROS surface
nav_order: 13
description: LiDAR, echo sounder and sonar data from ROS through rosbridge, averaged into a light 3D mesh — setup, every setting, worked examples, troubleshooting
---

# ROS surface (rosbridge)

Terrain under an aircraft, a sea bed under a boat, the walls of an underwater cave: whatever a ROS
sensor measures, drawn live in the 3D view as an averaged mesh of grey lines over a transparent body
— not as a point cloud. The GCS connects to **rosbridge** on the vehicle's companion computer,
subscribes to one sensor topic, places every point with the MAVLink telemetry pose and averages them
into 30 cm cells.

```
  sensor ── ROS driver ── rosbridge_server ──── WebSocket ────▶ GCS worker ──▶ 3D view
  (LiDAR, sonar, echo sounder)     (CBOR, throttled)             (sample, place, average)   (changed tiles only)
  ArduPilot ── MAVLink ──────────────────────────────────────────▶ pose (absolute or relative)
```

Two independent links meet in the GCS: the autopilot does not need to know about ROS, and ROS does
not need to know about the autopilot. Nothing is installed on the vehicle besides rosbridge.

| | |
|---|---|
| [Quick start](#quick-start) | the five steps from a ROS sensor to a mesh on screen |
| [1. On the vehicle](#1-on-the-vehicle-rosbridge) | installing and starting rosbridge, the network, supported messages |
| [2. The ROS page](#2-setup--tools--ros-every-setting) | every setting on SETUP → TOOLS → ROS, with its default |
| [3. On the flight screen](#3-on-the-flight-screen-the-ros-strip) | the ROS strip, its three buttons and its colours |
| [4. Reading the status](#4-reading-the-status-and-troubleshooting) | the status lines, every state and what to do about it |
| [5. Worked examples](#5-worked-examples) | aerial LiDAR, SLAM map, echo sounder, survey sonar, ROV in a cave |
| [6. What happens to the points](#6-what-happens-to-the-points) | sampling, spikes, pose projection, placing, averaging, the mesh |
| [7. Caves](#7-caves-surfaces-3d) | the 3D signed-distance volume |
| [8. Absolute and relative](#8-absolute-and-relative-navigation) | with and without GPS |
| [9. Testing without a vehicle](#9-testing-without-a-vehicle) | the rosbridge emulator, the Lake Garda survey, measured accuracy |
| [10. Limits](#10-limits) | what the surface is not |

---

## Quick start

1. **On the companion computer**, start rosbridge (ROS 2: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`)
   and the sensor driver.
2. **In the GCS**, connect the vehicle as usual (MAVLink), so it has a pose.
3. Open **SETUP → TOOLS → ROS**, type the rosbridge address in **ROSBRIDGE URL** (`ws://<companion IP>:9090`)
   and tick **SURFACE FROM ROS**.
4. Pick the sensor topic in **TOPIC** (the list fills in once connected) and the **SENSOR** profile
   (*Aerial LiDAR* or *Sea bed*); check the **MOUNT** in the preview.
5. Go back to **FLIGHT DATA**: the **ROS** strip at the top shows **ACCUMULATING** and the surface
   grows under the vehicle as it moves.

The settings are remembered; the SURFACE FROM ROS tick is not — like the LiDAR, a ROS link is
started on purpose, each session.

---

## 1. On the vehicle: rosbridge

### Install and start

rosbridge is the standard ROS package that exposes topics over a WebSocket. The launch file also
starts **rosapi**, which the GCS uses to list the topics.

| | Install | Start |
|---|---|---|
| ROS 2 (Humble, Jazzy, …) | `sudo apt install ros-$ROS_DISTRO-rosbridge-suite` | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| ROS 1 (Noetic) | `sudo apt install ros-noetic-rosbridge-server` | `roslaunch rosbridge_server rosbridge_websocket.launch` |

It listens on **TCP port 9090** (`port:=…` changes it). Start it at boot together with the sensor
driver (a systemd unit or your launch file), so it is there whenever the vehicle is powered.

### Network

The GCS PC must reach the companion computer over IP: Wi-Fi, an Ethernet radio bridge (Microhard,
Doodle Labs, Silvus, Ubiquiti), a tether, or a VPN over a cellular link. Open TCP 9090 in the
companion's firewall. The address you type is the companion's IP as seen from the GCS:
`ws://192.168.1.20:9090`. `wss://` works for a rosbridge behind TLS.

A point cloud is heavy: rosbridge sends whole messages, and the GCS samples them only after they have
crossed the link. On a narrow link lower **MAX RATE**, or downsample on the vehicle (`pcl_ros`
VoxelGrid, the driver's own decimation) — see [Limits](#10-limits).

### Supported messages

| Message | Typical sensors | What is read |
|---------|-----------------|--------------|
| `sensor_msgs/PointCloud2` | 3D LiDAR (`livox_ros_driver2`, Velodyne, Ouster, Hesai), multibeam or profiling sonar, a SLAM map | the `x`, `y`, `z` fields (any numeric type), `header.frame_id`, `header.stamp` |
| `sensor_msgs/LaserScan` | 2D scanner, push-broom LiDAR, a multibeam swath published as a scan | `ranges`, `angle_min`, `angle_increment`, `range_min/max` |
| `sensor_msgs/Range` | single-beam echo sounder, altimeter, a downward rangefinder | `range`, `field_of_view`, `min/max_range` |

ROS 1 and ROS 2 type names (`sensor_msgs/msg/PointCloud2`) are both recognised. Points that are NaN,
infinite or exactly (0, 0, 0) — the "no return" value of many drivers — are dropped.

Sensors with their own message types — the Blue Robotics Ping360 and Ping1D
(`bluerobotics_ping_msgs`), a sonar SDK's own messages — need a small node on the companion that
republishes them as one of the three types above: a Ping1D distance as a `Range` (in metres, with the
beam width as `field_of_view`), a Ping360 or a multibeam ping as a `LaserScan` or a `PointCloud2` in
the sensor's frame.

---

## 2. SETUP → TOOLS → ROS: every setting

The page has two panels: **ROS · ROSBRIDGE** (the link and the topic) and **MOUNT & MESH** (where the
sensor sits and how the surface is built). Every change applies at once, also while connected. A
green dot beside **ROS** in the SETUP menu means rosbridge is connected.

### ROS · ROSBRIDGE

| Setting | Default | |
|---------|---------|---|
| **SURFACE FROM ROS** | off | Connects to rosbridge and starts building the surface; unticking disconnects, hides the strip and clears the surface. Not remembered between sessions. |
| **ROSBRIDGE URL** | `ws://127.0.0.1:9090` | Must start with `ws://` or `wss://`. Changing it while connected reconnects. If rosbridge is not there yet the GCS retries every 2 s. |
| **TRANSPORT** | CBOR | **CBOR** sends the binary point arrays as they are; **JSON** inflates them by a third (base64). Use JSON only for an old rosbridge without CBOR. |
| **TOPIC** | — | The topics rosapi lists, filtered to the three supported types, as `name · type`. **REFRESH** asks rosapi again (a driver started after the GCS connected). A saved topic nobody publishes right now stays selected, marked *(not published now)*: rosbridge subscribes ahead of the publisher. Changing topic clears the surface. |
| **SENSOR** | Aerial LiDAR | A profile that fills in the usual values (below). Choose it first, then adjust. |
| **SURFACES** | Floor only | *Floor only (terrain, sea bed)*: one height per cell. *3D: caves, shafts, overhangs*: a signed-distance volume ([§7](#7-caves-surfaces-3d)). |
| **POINTS FRAME** | Auto | *Auto* decides from `header.frame_id`: `map`, `odom`, `world`, `earth`, `local_origin` (optionally with `_enu`, or `_ned` for NED) are **world frames**, everything else is the **sensor's own frame**. *Sensor* and *World* force one or the other. |
| **MAX RATE (Hz)** | 10 (profile) | rosbridge sends at most this many messages a second, only the newest (`throttle_rate`, `queue_length` 1): a slow link never builds a backlog. |
| **POINTS PER MESSAGE** | 5 000 (aerial) / 1 000 (sea bed) | From each message at most this many points are sampled with a uniform stride from a random start; the rest are not used. 30 cm cells need about 10 000 points a second to fill a 100 m swath at 8 m/s. Up to 20 000. |
| **CLEAR SURFACE** | | Drops the surface; it starts again from the next message. The same as CLEAR on the strip. |
| **STATUS** | | The live status lines — see [§4](#4-reading-the-status-and-troubleshooting). |

What each **SENSOR** profile fills in:

| | Mount (roll, pitch, yaw) | Range window | Max rate | Points per message |
|---|---|---|---|---|
| **Aerial LiDAR** · terrain under the aircraft | 180, 0, 0 (Livox inverted under the belly) | 2.5 – 100 m | 10 Hz | 5 000 |
| **Sea bed** · echo sounder or sonar on a boat / ROV | 0, −90, 0 (looking down) | 0.5 – 100 m | 10 Hz | 1 000 |

### MOUNT & MESH

| Setting | Default | |
|---------|---------|---|
| **MOUNT ATTITUDE VS IMU (ROLL / PITCH / YAW °)** | from the profile | ZYX Euler angles of the sensor frame relative to the vehicle body. The sensor frame is the ROS one (REP-103: x forward, y left, z up; a Range and a LaserScan measure along x); the body is the autopilot's (FRD: x forward, y right, z down). **Preset…** fills in the common cases: *Looking down (x down)* 0, −90, 0 for an echo sounder, altimeter or push-broom scanner; *Inverted under the belly* 180, 0, 0 for a Livox Mid-360; *Upright, x forward* 0, 0, 0; *Upright, nose-down 45°* 0, −45, 0. |
| **LEVER ARM IMU → SENSOR (X FWD / Y RIGHT / Z DOWN, m)** | 0, 0, 0 | Where the sensor's origin is from the autopilot's IMU, in the body frame. On a boat, **Z** is the transducer's draft below the IMU (positive down). |
| **MOUNT PREVIEW** | | Draws the result as you type: the vehicle (a triangle, nose forward, fin on top), the sensor at its lever arm with its axes (X red, Y green, Z blue) and the zone it collects from, turned by the mount — a Range's cone, a LaserScan's fan, a cloud's directions over the last 10 s (also on the bench, with no vehicle connected). It uses the same rotation the points are placed with: if the zone points the wrong way here, the surface will be wrong. |
| **RANGE WINDOW (m)** | from the profile | Returns closer than the minimum (the airframe, the hull, the landing gear) or farther than the maximum are dropped, in the sensor frame, before anything else. |
| **SPIKE FILTER** | Spikes dropped | Along an ordered scan (a sonar's beams, a LaserScan's sweep, a lidar's rings) a return far off its neighbours is dropped: aeration under the hull, a fish, a particle. *Every return* turns it off. Never applied to a Range or to a cloud with no order. Details in [§6](#6-what-happens-to-the-points). |
| **TIME OF THE POINTS** | Arrival − lag | When the points were measured, for the pose lookup. *Arrival − lag*: the time they reached the GCS, minus LAG. *header.stamp + lag (clocks synced)*: the message's own stamp, when the companion's clock is synced with the GCS PC (chrony, NTP, GPS time); a stamp more than 2 s off this clock is not trusted, and the status line says so. |
| **LAG (ms)** | 0 | With *Arrival*: how much later than the telemetry the points reach the GCS (positive: points later). With *Stamp*: how late the telemetry reaches the GCS. Tune it as in the [aerial example](#a-aerial-lidar-terrain-under-a-drone). −2000 to 2000. |
| **MIN CELL (m)** | 0.3 | The finest cell: the mesh is this fine where the data allow. Changing it clears the surface. |
| **MIN SAMPLES** | 1 | Samples a cell needs to be drawn at its own size; with fewer, the mesh there is drawn coarser (60 cm, 1.2 m, 2.4 m, 4.8 m). Raise it for a cleaner, coarser mesh; lower it (down to 0.3) to see every cell touched once. |
| **MEMORY (samples)** | 20 | A cell averages at most this many samples (messages), then follows a moving average: lower follows a changing scene faster, higher averages more noise away. |
| **MAX TILES** | 2 048 | Tiles of 30 × 30 cells kept; past this, the farthest from the vehicle are dropped. 2 048 tiles ≈ 16 ha at 30 cm. |
| **TRIANGLE FILL OPACITY (%)** | 10 | How opaque the triangles between the lines are; 0 = grey lines only. Up to 60. |
| **DRAW THROUGH THE TERRAIN MESH** | on | The surface is drawn over the elevation model even where it is under it: SRTM is ±5 m, and a lake's flat surface would hide the sea bed. Untick to let the terrain hide it. |

---

## 3. On the flight screen: the ROS strip

While **SURFACE FROM ROS** is ticked a strip appears at the top of the FLIGHT DATA screen:

```
 ● ROS  ACCUMULATING   12 480 cells · 30 cm · 4–31 m · 62 % of the area   [ DIST ] [ VIEW ON ] [ CLEAR ]
```

| Part | |
|------|---|
| **LED** | green: accumulating; orange blinking: connecting or no rosbridge; orange: connected but not accumulating (no topic, no data, no pose…) |
| **State** | the same state as the status line — see [§4](#4-reading-the-status-and-troubleshooting) |
| **Counter** | cells filled and the cell size (or, in 3D mode, triangles); with DIST the distance range of the colours; with ROUGH the departure that reads full red; with a survey area planned, how much of it is covered |
| **GRAY / DIST / ROUGH** | the colour of the mesh, cycling on each click (remembered) |
| **VIEW ON / OFF** | hides or shows the surface; it **keeps accumulating** while hidden (remembered) |
| **CLEAR** | drops the surface and starts again |

The three colours:

- **GRAY**: grey lines over a transparent body — the default, the least distracting.
- **DIST**: red → blue by distance from the vehicle: red at the nearest surface, blue at
  NEAR + max(20 m, 2 × NEAR); the strip shows the range in metres. Reads as a depth or height map
  around the vehicle.
- **ROUGH**: grey where the surface is flat, level or sloping, amber then red the further it departs
  from its local plane: the RMS distance of the heights within 1.5 m from the plane that fits them
  best, full red at 0.5 m. Rocks, a wreck, a scarp, pockmarks stand out of a flat bed — and so does a
  bad average (a wrong mount or lag shows as red streaks along the track). In 3D mode, where a wall
  has no height, it is how much the faces around each vertex disagree in direction, full red from
  about 25°.

The HUD also announces each change: *ROS: connecting to …*, *ROS: averaging /topic*, *ROS: no
rosbridge at …*, and a warning when accumulation stops.

---

## 4. Reading the status and troubleshooting

The **STATUS** box on the ROS page has up to seven lines:

```
CONNECTED ws://192.168.1.20:9090
/livox/lidar · PointCloud2 · frame livox_frame → SENSOR · placed with the telemetry pose
10 msg/s · 200,000 pts/s in · 50,000 sampled · 48,900 averaged · 1,100 spikes dropped (2.2 %)
surface 214 / 2,048 tiles of 30×30 · cell 0.3 m · 96,120 cells (8,651 m²)
planned area 1: 0.12 km² · 7.2 % covered
drawn 9 / 9 blocks · 172,000 triangles · resolution 30 cm 81% · 60 cm 15% · 120 cm 4%
ACCUMULATING · pose absolute (GPS)
```

1. the link and the URL;
2. the topic, its type, its `frame_id` and how it is treated (sensor frame or world ENU / NED);
3. messages and points a second: received, sampled, averaged, and the spikes dropped (or *spike
   filter idle: the scan has no order*);
4. the surface: tiles used of MAX TILES, cell size, cells and area filled (3D mode: chunks and triangles);
5. one line per **area scan** in the flight plan: its size and how much of it the surface covers
   (green above 90 %);
6. what is drawn: blocks, triangles and how much of the mesh is at each resolution;
7. the state, absolute or relative pose, and the autopilot clock rate when it is not 1 (a SITL
   speed-up).

Warnings in orange appear in between: messages dropped for lack of a pose or a home, a
`header.stamp` that is not synced with this clock.

| State | Means | What to do |
|-------|-------|------------|
| **CONNECTING** | opening the WebSocket | wait; it retries every 2 s |
| **NO ROSBRIDGE** | the address does not answer | check the URL and port, that rosbridge runs (`ros2 node list` shows `/rosbridge_websocket`), the network (ping the companion) and the firewall (TCP 9090) |
| `rosapi: … is rosapi running` (error line) | connected but the topic list failed | start rosbridge with the launch file, which starts rosapi too, not the bare node |
| **NO TOPIC** | connected, no topic chosen (or of an unsupported type) | choose one in TOPIC; if the list is empty, check the driver publishes (`ros2 topic list -t`) and press REFRESH |
| **NO DATA** | no message for 3 s on the chosen topic | the driver stopped or publishes on another name; `ros2 topic hz /topic` on the companion |
| **NO VEHICLE** | no vehicle is connected (the demo flight is running) | connect the vehicle over MAVLink: points are placed with its pose |
| **NO POSITION** | the vehicle has no position yet | wait for a GPS fix, or switch to the relative navigation mode for a vehicle without GPS ([§8](#8-absolute-and-relative-navigation)) |
| **NO HOME (world frame)** | world-frame points (`map`, `odom`) but no home known | wait for the home position (HOME_POSITION after arming / the first fix), or use a sensor-frame topic |
| **EMPTY MESSAGES** | messages arrive with no points | the driver publishes empty clouds (sensor not spinning, filtered out) |
| **NO VALID POINTS** | every point was outside the range window, NaN or (0, 0, 0) | widen the RANGE WINDOW; check the cloud's units are metres |
| **FRAME CHANGED · CLEAR** | the navigation mode changed under the surface | press CLEAR (it clears by itself on a new connection or a mode switch) |
| **ACCUMULATING** | everything works | — |

When the surface looks wrong rather than absent:

| Symptom | Likely cause |
|---------|--------------|
| The surface is tilted, upside down, or appears above the vehicle | wrong MOUNT: compare the zone in MOUNT PREVIEW with where the sensor really looks |
| Two sheets, doubled walls, steps along every slope when the vehicle turns | wrong LAG (or a wrong yaw in the mount); low telemetry rates — raise `SR_EXTRA1` and `SR_POSITION` to 10 Hz or more in [MAVLink stream rates](sys-config.md#mavlink-stream-rates) |
| The surface is offset by a constant distance | wrong LEVER ARM, or (world frame) a map frame not anchored at the EKF origin |
| Shallow bumps under the track of a boat | aeration: leave the SPIKE FILTER on, raise the minimum range |
| Very sparse, coarse mesh | too few points: raise POINTS PER MESSAGE or MAX RATE, or lower MIN SAMPLES |
| The GCS slows down | lower POINTS PER MESSAGE, MAX TILES, or use 30 FPS (SYS CONFIG) |

---

## 5. Worked examples

### A. Aerial LiDAR: terrain under a drone

A Livox Mid-360 inverted under the belly, `livox_ros_driver2` publishing `/livox/lidar`
(PointCloud2, frame `livox_frame`).

1. SENSOR **Aerial LiDAR** (mount 180, 0, 0, range 2.5–100 m, 10 Hz, 5 000 points), SURFACES **Floor
   only**, POINTS FRAME **Auto** (the status shows `livox_frame → SENSOR`).
2. LEVER ARM: measure the sensor's optical centre from the autopilot, e.g. 0.05 m forward, 0, 0.12 m
   down.
3. RANGE WINDOW: raise the minimum until the landing gear and the propeller guards no longer show as a
   blob under the vehicle.
4. In SYS CONFIG → MAVLINK STREAM RATES set POSITION and EXTRA1 to 10 Hz and write them.
5. Fly: hover 10–20 m over a slope or next to a building and yaw slowly. If the slope doubles or
   steps as the vehicle turns, change LAG in steps of 20–50 ms until it stays one sheet — 0–50 ms on a
   direct IP link, 100–300 ms through a slower radio for the telemetry.

The [LiDAR chapter](LIDAR.md) covers the same sensor read directly (no ROS) as a point cloud; this
page averages it into a surface instead.

### B. A SLAM map in a world frame

FAST-LIO, Point-LIO or a MAVROS pipeline publish an already georeferenced cloud (`/cloud_registered`)
in a world frame. If its `frame_id` is `map`, `odom` or `world`, *Auto* treats it as a world frame:
points are placed from **home** (absolute mode) or from the local zero (relative mode), with no mount
or lag. A frame with another name (FAST-LIO's `camera_init`) needs POINTS FRAME **World**.

This only lines up when the world frame is ENU (x east, y north, z up) with its origin at the EKF
origin, as MAVROS publishes it. A SLAM that started elsewhere or with another heading does not —
subscribe to its sensor-frame topic instead and let the GCS place it.

### C. Single-beam echo sounder on a boat

A Ping1D (or any echo sounder) republished as `sensor_msgs/Range` on `/ping1d/range`.

1. SENSOR **Sea bed** (mount 0, −90, 0 — x looking down; range 0.5–100 m), SURFACES **Floor only**.
2. LEVER ARM **Z** = the transducer's depth below the autopilot (e.g. 0.3 m), X/Y its position.
3. A single beam fills one line of cells along the track: drive lanes close together (a few metres
   apart) and raise **MIN CELL** to 1–2 m, so neighbouring lanes join into a surface.
4. The status shows the beam as *Range · 25° cone along X* in the preview (from `field_of_view`).

### D. Multibeam or imaging sonar on a survey boat

1. Plan the survey in FLIGHT PLAN as an **Area scan** with *Lane spacing: From mapping sensor* —
   aperture, range, design depth, side overlap ([mission planning](mission-planning.md#sonar-and-lidar-surveys)).
2. ROS page: the sonar topic (PointCloud2 or LaserScan), SENSOR **Sea bed**, mount **0, −90, 0** if
   the fan is published with x along the beam axis looking down; check the fan in MOUNT PREVIEW lies
   **across** the hull.
3. Leave the **SPIKE FILTER** on: under a hull in a chop the aeration returns at 0.5–4.5 m outnumber
   the bed's in the cells under the track, and without the filter the bed there comes out metres too
   shallow.
4. Use **header.stamp** if the companion runs chrony against the GCS or GPS time — sonar pings are
   stamped at transmission, and a stamp beats arrival time on a variable link.
5. During the survey the strip shows *… % of the area*, and the status one line per planned area:
   the lanes still to fill are where the coverage stays below 100 %. **ROUGH** colour shows rocks,
   wrecks and scarps on the bed.

### E. ROV in a cave or a shaft

1. SURFACES **3D: caves, shafts, overhangs**, a profiling sonar (e.g. a 360° fan across the vehicle) in
   the sensor frame.
2. Without GPS under water, switch SYS CONFIG → NAVIGATION to **Relative**: the points are placed with
   the vehicle's `LOCAL_POSITION_NED` (a DVL or USBL fused by the EKF) — see
   [navigation without GPS](3D-NAVIGATION.md#5-navigation-without-gps).
3. Move or sweep: a surface needs the sensor to move; two fans from one place have no area.

---

## 6. What happens to the points

1. **Sampling.** The worker thread decodes each message (CBOR byte strings and typed arrays, or JSON
   base64) and keeps at most POINTS PER MESSAGE of them. Nothing of this reaches the render thread.
2. **Spikes.** Points in the order the sensor sent them — a sonar's beams across its swath, a
   LaserScan's sweep, a lidar's rings — form a profile: a real surface is continuous from one beam to
   the next but at a few edges, while aeration under the hull, a fish or a particle is a beam or a few
   far off their neighbours. A point more than max(0.5 m, 8 % of the range) from the median of the 9
   points around it is dropped; the median keeps edges (a wall, a wreck's side). A cloud without
   order (or sampled too sparsely to keep it) is left alone. The status line shows how many are
   dropped. Under a survey boat in a chop, aeration returns at 0.5–4.5 m land in the cells under the
   track, where they outnumber the bed's: without the filter the bed there came out 10 m too shallow.
3. **Pose at the points' time.** The pose is the one the 3D view draws the vehicle at — absolute
   (GPS) or relative (SYS CONFIG → NAVIGATION). Every ATTITUDE and GLOBAL_POSITION_INT reaches the
   worker with its arrival time and the autopilot's `time_boot_ms`; the sample just before the points
   is **projected** to their time: attitude with the gyro rates (Euler kinematics), position with the
   velocity. Without this, a copter yawing at 90°/s with an attitude 50 ms old puts a return 80 m away
   6 m off, and every slope the scan crosses grows spikes. The projection runs on the autopilot's
   clock, measured from `time_boot_ms` against arrival: 1 on a vehicle, the speed-up in a SITL (shown
   on the status line), where 50 ms here are a second of rolling in the waves.
4. **Placing.** Sensor frame → mount + lever arm → body → NED → ENU from an anchor (the first pose);
   world frames from home (absolute) or from the local zero (relative).
5. **Averaging.** Each message gives each 30 cm cell it touches one sample — the mean of its points
   there, weighted by how many (1 point = 1/3 of a sample) — and a cell keeps a running mean of at
   most MEMORY samples. Cells live in tiles of 30 × 30, made only where data arrive; past MAX TILES
   the farthest from the vehicle are dropped.
6. **Adaptive mesh.** 30 cm is the finest the mesh gets: each cell is drawn at the finest of 30 cm,
   60 cm, 1.2, 2.4 or 4.8 m that has MIN SAMPLES, and the lines at that spacing. Holes up to 1.2 m are
   interpolated, edges are not extended, and nothing is averaged or joined across a height jump (a
   cliff, a wall).

The renderer keeps the heights in GPU textures (blocks of 8 × 8 tiles), builds the triangles in the
vertex shader, draws the lines in the fragment shader (they fade where cells get smaller than a few
pixels; tile edges every 9 m stay) and coarsens far blocks. Only the tiles that changed travel from
the worker, at most four times a second. Points are not kept: the surface cannot be exported as a
point cloud (the [LiDAR](LIDAR.md) page saves `.ply` for the Livox read directly).

## 7. Caves (SURFACES: 3D)

A height per cell holds one surface: right for terrain, wrong in a shaft, a cave or under an overhang,
where floor, walls and ceiling share columns. In 3D mode the returns are fused into a volume of 30 cm
voxels holding a truncated signed distance to the rock along each ray (positive in the water,
negative behind the rock, averaged over MEMORY samples), and meshed where it changes sign (surface
nets). Every wall comes out with its own orientation. The volume is kept in chunks of 16³ voxels;
only the chunks that changed are meshed again and sent. A surface needs the sensor to move or sweep:
two fans from one place have no area. 3D mode has no levels of detail: it is meant for the scale of a
cave, not of a survey.

## 8. Absolute and relative navigation

| | Absolute (GPS) | Relative (no GPS) |
|---|---|---|
| Sensor-frame points | placed with GLOBAL_POSITION_INT + ATTITUDE | placed with LOCAL_POSITION_NED (DVL, USBL, visual odometry in the EKF) or the GCS dead reckoning |
| World-frame points (`map`, `odom`) | from home (a sub: its surface) | from the local zero |
| Switching mode, a vehicle connecting | the surface starts again: its frame is gone | |

In relative mode the mesh is only as good as the local position: with a DVL or USBL it holds;
dead-reckoning on a sub's EKF velocity drifts with it. Coverage of planned areas is measured in
absolute mode only (a relative frame has no place on the map).

## 9. Testing without a vehicle

These scripts run from a source checkout (`git clone`, `npm install` — see
[Getting started](getting-started.md#install)):

```
node scripts/test-ros-surface.js     # decoding, frames, mounts, projection, cells; rosbridge end to end
node scripts/test-ros-cave.js        # a ROV down the cave: mounting check and 3D mesh accuracy
node scripts/rosbridge-sim.js --scene terrain|seabed|cave|garda [--ros1] [--pose e,n,up,yaw]
```

`rosbridge-sim.js` speaks the rosbridge protocol (JSON and CBOR, rosapi, throttling) and publishes
`/livox/lidar`, `/cloud_registered` (`map`), `/scan`, `/sonar/multibeam`, `/ping1d/range`,
`/sonar/profiler` (dual 360° fan) and `/sonar/imaging` (an imaging sonar looking down: 256 beams over
90° across the track, 90 m range, a 10° vertical aperture, with aeration that grows with the hull's
motion, fish, dropouts and range noise), ray-cast from SITL's true pose (SIMSTATE / SIM_STATE on TCP
5762, also without GPS) against a known scene: hills with rocks, a 0.6 m slab and a ditch; a lake bed
with a wreck and rocks; a cave of 3 m shafts and passages down to 30 m; **garda**, 5 km² of the
southern basin of Lake Garda (2.5 × 2 km from 45.4960 N, 10.6490 E), 6 to 74 m deep: the slope from
the western shore, a moraine ridge with boulders, an old channel, a shoal, ripples, pockmarks and a
28 m wreck. Other options: `--port` (9090), `--mav` (`tcp:127.0.0.1:5762`), `--lidar-mount`,
`--down-mount`, `--profiler-mount`, `--lever`, `--lidar-points`, `--sonar-aperture`, `--sonar-beams`,
`--sonar-range`, `--sonar-rate`, `--sonar-clean` (no disturbances).

Step by step, with the GCS's own simulator:

1. SETUP → TOOLS → SIMULATION: launch a vehicle ([simulator](simulator.md)) — a Copter for
   `terrain`, the **Boat** for `seabed` or `garda`, a ROV for `cave`.
2. In a terminal: `node scripts/rosbridge-sim.js --scene garda` **before the vehicle moves** (the
   scene is anchored at home).
3. ROS page: URL `ws://127.0.0.1:9090`, tick SURFACE FROM ROS, pick a topic — mounts as the emulator
   uses them: `/livox/lidar` 180, 0, 0; `/ping1d/range`, `/scan`, `/sonar/multibeam` and
   `/sonar/imaging` 0, −90, 0; `/sonar/profiler` 0, 0, 0.
4. Arm and fly or drive: the surface should match the scene.

A survey of the garda scene: the **Boat** SITL, an area scan over the scene planned with *sensor*
spacing (aperture 90°, range 90 m, design depth 30 m, side overlap 50 % → 30 m between lanes,
2 m/s), and `/sonar/imaging` with mount (0, −90, 0). The status line shows how much of the planned
area the surface covers.

Measured here:

| | |
|---|---|
| Georeferencing chain (emulator → rosbridge → GCS) | 0.0 mm from the scene, every sensor and frame |
| Aerial LiDAR, 30 cm cells, cell mean vs scene | 1.6 cm RMS |
| A rock 1.2 m across, 0.8 m high | kept at 0.74 m with 30 cm cells (2 m cells: 0.12 m) |
| Copter hovering, Livox, SITL | 0.13 m RMS, 99 % within 0.24 m |
| Copter surveying at 8 m/s, without projection | 1.8 m RMS, spikes to 15 m — the reason for §6.3 |
| Cave, points with the true mount | 0.5 cm p99 from the rock; a wrong mount puts 28–56 % of them over 30 cm off |
| Cave, 3D mesh | median 2 cm from the rock, 98.6 % of triangles within 35 cm, the walls covered |
| Boat survey, imaging sonar, waves, SITL at ×21: first lanes, 1 m cells | under the track (0–2 m): 10.6 m too shallow, 17 m RMS without the spike filter and with wall-clock projection; 0.14 m bias, 0.7 m RMS with both. Across the swath 0.5–0.6 m RMS; by depth 0.2 m RMS under 15 m, 0.8 m past 50 m, where the 10° vertical aperture spreads an echo over ±4 m along the track |
| Rendering | 60 FPS with the surface on, in the SITL runs |

## 10. Limits

- The pose is the telemetry's, matched by arrival time (or a synced stamp): radio jitter blurs fast
  turns even with the projection.
- A cell is a mean: vegetation and its ground, a fish and the bottom, average together. A minimum or
  a median per cell would be the next step.
- 3D mode has no levels of detail: it is meant for the scale of a cave, not of a survey.
- rosbridge sends whole messages; POINTS PER MESSAGE spares the GCS, not the link. For a narrow link,
  downsample on the vehicle (`pcl_ros` VoxelGrid) or lower MAX RATE.
- The surface lives in memory only: CLEAR, unticking SURFACE FROM ROS, a new connection or a
  navigation-mode switch drop it; it is not saved.
