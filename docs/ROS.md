---
title: ROS surface
nav_order: 11
description: LiDAR, echo sounder and sonar data from ROS through rosbridge, averaged into a light 3D mesh
---

# ROS surface (rosbridge + roslib.js)

Terrain under an aircraft, a sea bed under a boat, the walls of an underwater
cave: whatever a ROS sensor measures, drawn in the 3D view as an averaged mesh
of grey lines over a transparent body — not as a point cloud.

```
  sensor ── ROS driver ── rosbridge_server ──── WebSocket ────▶ GCS worker ──▶ 3D view
  (LiDAR, sonar, echo sounder)     (CBOR, throttled)             (sample, place, average)   (changed tiles only)
  ArduPilot ── MAVLink ──────────────────────────────────────────▶ pose (absolute or relative)
```

## 1. On the vehicle

Run rosbridge with rosapi on the companion computer:

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml     # ROS 2
roslaunch rosbridge_server rosbridge_websocket.launch            # ROS 1
```

It listens on port 9090. The GCS reads `sensor_msgs/PointCloud2` (3D LiDAR —
`livox_ros_driver2`, Velodyne, Ouster —, a multibeam or profiling sonar, a SLAM
map), `sensor_msgs/LaserScan` (2D scanner, push-broom LiDAR, multibeam swath)
and `sensor_msgs/Range` (single-beam echo sounder, altimeter). Sensors with
their own message types (Ping360, Ping1D's `bluerobotics_ping_msgs`) need a
small node converting them to one of these.

## 2. SETUP → TOOLS → ROS

| Setting | |
|---------|---|
| **ROSBRIDGE URL / TRANSPORT** | `ws://companion:9090`. **CBOR** sends binary arrays as they are; JSON inflates them by a third (base64). |
| **TOPIC** | The topics rosapi lists, filtered to the three types above. |
| **SENSOR** | *Aerial LiDAR* or *Sea bed*: fills in the usual mount, range and sampling. |
| **SURFACES** | *Floor only* — a height per cell: terrain, sea bed. *3D* — caves, shafts, overhangs (§4). |
| **POINTS FRAME** | *Auto* from `header.frame_id`: `map`, `odom`, `world`, `earth` (`_ned` suffix: NED) are world-fixed frames from the EKF origin; anything else is the sensor's own frame, placed with the telemetry pose. |
| **MAX RATE / POINTS PER MESSAGE** | rosbridge sends at most this many messages a second (`throttle_rate`, `queue_length` 1); from each, this many points are sampled with a uniform stride. The rest are not used. |
| **MOUNT / LEVER ARM** | ZYX Euler angles of the sensor frame (REP-103: x forward, y left, z up; Range and LaserScan measure along x) relative to the vehicle body (FRD), and its offset from the IMU. *Looking down* (0, −90, 0) for an echo sounder or a push-broom scanner, *inverted* (180, 0, 0) for a Livox under the belly. |
| **MOUNT PREVIEW** | The vehicle (a triangle, nose forward, fin on top), the sensor at its lever arm with its axes (X red, Y green, Z blue) and the zone it collects from, turned by the mount: a Range's cone, a LaserScan's fan, a cloud's directions over the last 10 s (also with no vehicle connected). Drawn with the same rotation the points are placed with. |
| **RANGE WINDOW** | Returns closer (the airframe, the hull) or farther are dropped. |
| **TIME OF THE POINTS / LAG** | *Arrival − lag*, or *header.stamp + lag* when the companion computer's clock is synced with the GCS (chrony, NTP, GPS; a stamp more than 2 s off is not trusted). |
| **MIN CELL / MIN SAMPLES / MEMORY / MAX TILES** | 30 cm cells by default; see §3. |

The strip on the flight screen shows the state, the cells or triangles drawn,
and three buttons: **GRAY / DIST** (grey, or red → blue by distance from the
vehicle: red at the nearest surface, blue at NEAR + max(20 m, 2 × NEAR), the
range shown on the strip), **VIEW ON / OFF** (hide the surface; it keeps
accumulating) and **CLEAR**.

## 3. What happens to the points

1. **Sampling.** The worker thread decodes each message (CBOR byte strings and
   typed arrays, or JSON base64) and keeps at most POINTS PER MESSAGE of them.
   Nothing of this reaches the render thread.
2. **Pose at the points' time.** The pose is the one the 3D view draws the
   vehicle at — absolute (GPS) or relative (SYS CONFIG → NAVIGATION) — taken
   from STATE with the arrival time of its ATTITUDE and position, and
   **projected** to the points' time: attitude with the gyro rates (Euler
   kinematics), position with the velocity. Without this, a copter yawing at
   90°/s with an attitude 50 ms old puts a return 80 m away 6 m off, and every
   slope the scan crosses grows spikes.
3. **Placing.** Sensor frame → mount + lever arm → body → NED → ENU from an
   anchor (the first pose); world frames from home (absolute) or from the
   local zero (relative).
4. **Averaging.** Each message gives each 30 cm cell it touches one sample —
   the mean of its points there, weighted by how many (1 point = 1/3 of a
   sample) — and a cell keeps a running mean of at most MEMORY samples. Cells
   live in tiles of 30 × 30, made only where data arrive; past MAX TILES the
   farthest from the vehicle are dropped.
5. **Adaptive mesh.** 30 cm is the finest the mesh gets: each cell is drawn
   at the finest of 30 cm, 60 cm, 1.2, 2.4 or 4.8 m that has MIN SAMPLES, and
   the lines at that spacing. Holes up to 1.2 m are interpolated, edges are
   not extended, and nothing is averaged or joined across a height jump (a
   cliff, a wall).

The renderer keeps the heights in GPU textures (blocks of 8 × 8 tiles), builds
the triangles in the vertex shader, draws the lines in the fragment shader
(they fade where cells get smaller than a few pixels; tile edges every 9 m
stay) and coarsens far blocks. Only the tiles that changed travel from the
worker, at most four times a second.

## 4. Caves (SURFACES: 3D)

A height per cell holds one surface: right for terrain, wrong in a shaft, a
cave or under an overhang, where floor, walls and ceiling share columns. In 3D
mode the returns are fused into a volume of 30 cm voxels holding a truncated
signed distance to the rock along each ray (positive in the water, negative
behind the rock, averaged over MEMORY samples), and meshed where it changes
sign (surface nets). Every wall comes out with its own orientation. The volume
is kept in chunks of 16³ voxels; only the chunks that changed are meshed again
and sent. A surface needs the sensor to move or sweep: two fans from one place
have no area.

## 5. Absolute and relative

| | Absolute (GPS) | Relative (no GPS) |
|---|---|---|
| Sensor-frame points | placed with GLOBAL_POSITION_INT + ATTITUDE | placed with LOCAL_POSITION_NED (DVL, USBL, visual odometry in the EKF) or the GCS dead reckoning |
| World-frame points (`map`, `odom`) | from home (a sub: its surface) | from the local zero |
| Switching mode, a vehicle connecting | the surface starts again: its frame is gone | |

In relative mode the mesh is only as good as the local position: with a DVL
or USBL it holds; dead-reckoning on a sub's EKF velocity drifts with it.

## 6. Testing without a vehicle

```
node scripts/test-ros-surface.js     # decoding, frames, mounts, projection, cells; rosbridge end to end
node scripts/test-ros-cave.js        # a ROV down the cave: mounting check and 3D mesh accuracy
node scripts/rosbridge-sim.js --scene terrain|seabed|cave [--ros1] [--pose e,n,up,yaw]
```

`rosbridge-sim.js` speaks the rosbridge protocol (JSON and CBOR, rosapi,
throttling) and publishes `/livox/lidar`, `/cloud_registered` (`map`),
`/scan`, `/sonar/multibeam`, `/ping1d/range` and `/sonar/profiler` (dual 360°
fan), ray-cast from SITL's true pose (SIMSTATE / SIM_STATE on TCP 5762, also
without GPS) against a known scene: hills with rocks, a 0.6 m slab and a ditch;
a lake bed with a wreck and rocks; a cave of 3 m shafts and passages down to
30 m. Start it before the vehicle moves, then point the GCS at
`ws://127.0.0.1:9090`.

Measured here:

| | |
|---|---|
| Georeferencing chain (emulator → rosbridge → GCS) | 0.0 mm from the scene, every sensor and frame |
| Aerial LiDAR, 30 cm cells, cell mean vs scene | 1.6 cm RMS |
| A rock 1.2 m across, 0.8 m high | kept at 0.74 m with 30 cm cells (2 m cells: 0.12 m) |
| Copter hovering, Livox, SITL | 0.13 m RMS, 99 % within 0.24 m |
| Copter surveying at 8 m/s, without projection | 1.8 m RMS, spikes to 15 m — the reason for §3.2 |
| Cave, points with the true mount | 0.5 cm p99 from the rock; a wrong mount puts 28–56 % of them over 30 cm off |
| Cave, 3D mesh | median 2 cm from the rock, 98.6 % of triangles within 35 cm, the walls covered |
| Rendering | 60 FPS with the surface on, in the SITL runs |

## 7. Limits

- The pose is the telemetry's, matched by arrival time (or a synced stamp):
  radio jitter blurs fast turns even with the projection.
- A cell is a mean: vegetation and its ground, a fish and the bottom, average
  together. A minimum or a median per cell would be the next step.
- 3D mode has no levels of detail: it is meant for the scale of a cave, not of
  a survey.
- rosbridge sends whole messages; POINTS PER MESSAGE spares the GCS, not the
  link. For a narrow link, downsample on the vehicle (`pcl_ros` VoxelGrid) or
  lower MAX RATE.
