---
title: Mission planning
nav_order: 5
description: Drawing a route as segments, what each becomes on the autopilot, altitudes and terrain, surveys, upload and read-back
---

# Mission planning

The **FLIGHT PLAN** tab. You draw the mission as shapes — a waypoint, a circle to loiter in, a
perimeter, an area to photograph, a corridor, a point for the camera, a landing — and the route
between them is calculated a moment after every edit, the way UgCS works. The calculated route is a list of ArduPilot mission commands; this chapter says which ones,
with the ArduPilot page for each, so you know exactly what the autopilot will fly.

![Mission planning](images/mission-planning.jpg)

## The page

- **Tool palette** (left edge of the map), with keys: **V** select, **W** waypoint, **C** circle,
  **P** perimeter, **A** area scan, **K** corridor, **I** point of interest, **L** landing, **O**
  operator (ground antenna, for the [radio link](radio-link.md)). A polygon or line is finished with
  **Enter** or by clicking its first point; **Esc** cancels.
- **Segments** list (left panel): one card per segment, in flight order, with its settings
  inline and its actions. **Delete** removes the selected segment; **Ctrl+Z / Ctrl+Y** undo and redo.
- **Route card** (top left): length, flight time, waypoints, highest point above ground, the route
  settings, and every warning or error. **Errors block the upload.**
- **Map layers** (top right): **SAT** imagery, **WP** calculated waypoints, **CAM** photo footprints
  and POI view cones, **LINK** radio coverage, **FIT** zoom to the route.
- **Elevation profile** (bottom): the ground under the whole route and the height flown; hovering it
  shows the point on the map.

Drag points to move them, the mid-point handles to insert a vertex, the centre handle to move a whole
shape, the rim handle to change a circle's radius. Right-click for the context menu.

## Segments and the commands they become

| Segment | Mission commands | ArduPilot |
|---------|------------------|-----------|
| **Waypoint** | `NAV_WAYPOINT` (16), or `NAV_SPLINE_WAYPOINT` (82) with the spline turn type; hold time and acceptance radius | [Mission commands](https://ardupilot.org/copter/docs/mission-command-list.html) |
| **Circle** | `NAV_LOITER_TURNS` (18): turns, radius; a negative radius turns counter-clockwise. ArduPilot flies to the edge of the circle, not its centre | [Loiter turns](https://ardupilot.org/copter/docs/mission-command-list.html) |
| **Perimeter** | `NAV_WAYPOINT` at every vertex, as drawn or reversed, optionally back to the first | |
| **Area scan** | Parallel lanes of `NAV_WAYPOINT` across the polygon (direction, overshoot, optional cross grid), with `DO_SET_CAM_TRIGG_DIST` (206) started on the first lane and stopped after the last | [Cameras and gimbals](https://ardupilot.org/copter/docs/common-cameras-and-gimbals.html) |
| **Corridor** | Parallel passes of `NAV_WAYPOINT` along the line, covering the width; camera trigger as above | |
| **Point of interest** | `DO_SET_ROI_LOCATION` (201) — the camera / nose points at it — or `DO_SET_ROI_NONE` (197) to stop | [Mission commands](https://ardupilot.org/copter/docs/mission-command-list.html) |
| **Landing** | `NAV_LAND` (21), or `NAV_VTOL_LAND` (85) for a VTOL, with an abort altitude | [Plane automatic landing](https://ardupilot.org/plane/docs/automatic-landing.html) |

Around them the route adds `NAV_TAKEOFF` (22) when automatic take-off is on (15° climb pitch on a
plane), `DO_CHANGE_SPEED` (178) wherever the speed changes, and at the end `NAV_RETURN_TO_LAUNCH`
(20) or a landing in place.

**Actions** hang off a segment and become commands placed after its point (ArduPilot runs a `DO_`
command once the navigation command before it is reached), at the segment start or at every point:

| Action | Command |
|--------|---------|
| Camera trigger by distance | `DO_SET_CAM_TRIGG_DIST` (206) |
| Take a photo | `DO_DIGICAM_CONTROL` (203) |
| Camera attitude | `DO_MOUNT_CONTROL` (205) — pitch and yaw |
| Set heading | `CONDITION_YAW` (115) |
| Wait | `NAV_DELAY` (93) |
| Change speed | `DO_CHANGE_SPEED` (178) |
| Set servo | `DO_SET_SERVO` (183) — see [servo](https://ardupilot.org/copter/docs/common-servo.html) |
| Set relay | `DO_SET_RELAY` (181) — see [relay](https://ardupilot.org/copter/docs/common-relay.html) |
| MAVLink command | any command by number — what a mission read from a vehicle keeps when it has no editor |

The full reference of every command and parameter is ArduPilot's
[MAVLink mission commands](https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html)
page ([Plane version](https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html)).

## Altitudes and terrain

**Route settings → Altitude mode**:

| Mode | The height you type is | Sent to the autopilot as |
|------|------------------------|--------------------------|
| **AGL — follow terrain** | above the ground under each waypoint | resolved against the elevation model |
| **AMSL — constant** | above mean sea level | absolute |
| **Relative to take-off** | above the take-off point | relative |

In AGL mode each waypoint sits at its height above the ground under it and the vehicle flies straight
from one to the next. With **Terrain-following waypoints** on, intermediate waypoints are added so the
straight legs stay within the **AGL tolerance** of the terrain. Every leg — not just every waypoint —
is checked against the terrain: a leg that dips under **Min clearance** is a warning, one that goes
through the ground an error; **Max altitude AGL** warns above a ceiling. The autopilot's own terrain
following (with its terrain database) is described in ArduPilot's
[terrain following](https://ardupilot.org/copter/docs/terrain-following.html) page; the GCS does the
work in advance, so the mission flies the same with or without it.

When the autopilot's terrain database is on (`TERRAIN_ENABLE`), the vehicle asks the GCS for the
terrain around it and the GCS answers from the same SRTM tiles, as Mission Planner does — the area
around the vehicle (9 grids, at the default 100 m spacing) loads in about 40 s. Where a tile is missing or has holes, the GCS
does not answer rather than send a made-up height; the vehicle keeps asking and shows the terrain as
pending. If blocks stay pending and the vehicle asks for nothing, it is stuck reading its own SD card
(missing, full or failing): the HUD shows *TERRAIN: vehicle SD card not readable*, and the GCS cannot
help until the card is fixed and the autopilot rebooted.

Other route settings: default altitude and speed, turn type (straight or spline), automatic take-off
and its height, what happens after the last segment (return to launch, land in place, nothing), and
whether actions run at the segment start or at every point.

## Camera surveys

<p align="center"><img src="images/camera-footprint.png" width="300" alt="Camera footprint"/></p>

The camera profile (camera icon on the route card) gives the field of view and the resolution: pick a
preset (DJI Mavic 3, Zenmuse H20 / P1, Sony α6000, RX1R II, MicaSense RedEdge…) or enter sensor size,
focal length and image size. An area scan then works out the lane spacing from the side overlap and
the photo interval from the forward overlap — a Mavic 3 at 100 m gives 49 m between lanes, a photo
every 26 m and 2.7 cm per pixel on the ground. Every planned photo is a red dot on the route; hover it
to see the rectangle it covers. The default gimbal pitch and yaw are set next to the camera.

## Sonar and LiDAR surveys

For a sensor that sweeps a fan under the vehicle — an imaging or multibeam sonar, a LiDAR — set the
area scan's **SPACING** to *Sensor*: give the fan's **APERTURE** (across the track), its **RANGE** and
the **DESIGN DISTANCE** to the surface (the depth under a boat; 0: the segment's altitude). The swath
is the narrower of the aperture's width at that distance and the width the range reaches
(2·√(range² − distance²)); the lanes are spaced by the swath less the **SIDE OVERLAP**, and the card
says which of the two limited it. A 90° sonar with 90 m of range at 30 m sweeps 60 m: 30 m between
lanes with 50 % overlap. Choose the design distance from the shallowest water that must be covered: the
swath narrows with depth, so lanes spaced for 30 m leave gaps where the bed is shallower than 15 m. With
50 % overlap the bed under each lane is also seen by the next lane's outer beams, where the water under
a hull is noisy. A boat's mission has no take-off; no camera is triggered. During the survey the ROS
strip shows how much of each planned area is covered ([ROS](ROS.md)).

## Upload, read, save

- **UPLOAD** sends the calculated mission to the autopilot (MAVLink mission protocol, or MSP on INAV);
  **READ** brings back the mission stored on it, turned into segments — one waypoint per navigation
  command, loiters as circles, the rest as actions.
- **SAVE** and **LIBRARY** keep routes in `data/missions`, with the segments, so they can be edited
  again.
- The **…** menu exports a Mission Planner `.waypoints` file and imports `.waypoints`, `.txt` or `.json`.

The mission flown is shown on the flight screen in 3D the way this page draws it — see
[the mission in 3D](3D-NAVIGATION.md#1-the-mission-in-3d).

## INAV

On INAV the same plan is uploaded over MSP as an INAV waypoint list: navigation and hold points, POI,
heading, speed, landing and return to home, with the altitudes the terrain model worked out. What the
INAV format cannot carry is greyed out while you plan, with the reason on the control: no loiter
circles, no spline legs, no take-off waypoint, no camera, gimbal, servo or relay actions, and at most
60 waypoints (the board's own figure once connected). Two INAV-only settings: absolute (MSL) waypoint
altitudes for INAV 5 or later, and storing the mission in the board's memory after the upload.
