---
title: Side panel
nav_order: 4
description: The GCS controls panel on the right of the flight screen — vehicle, set home, reboot, mission, mute, ADS-B, rotor load, trail, RTL options, target, message log, log replay
---

# Side panel (GCS controls)

The **arrow on the right edge** of the screen (▶ / ◀) slides out a panel of controls that are needed
less often than the command bar. It starts closed; click the arrow again to close it. Each section
folds on a click on its title (▼), so you can keep open only what you use.

| Section | What it is for |
|---------|----------------|
| [VEHICLE](#vehicle) | what is connected |
| [ACTIONS](#actions) | set home here, reboot the autopilot |
| [MISSION](#mission) | mission progress |
| [GCS OPTIONS](#gcs-options) | mute the GCS, ADS-B traffic, rotor load, clear the trail |
| [RTL OPTIONS](#rtl-options) | return-to-launch and Smart RTL parameters |
| [TARGET](#target) | a reference point to fly to, with an arrow and a distance |
| [MESSAGE LOG](#message-log) | every autopilot message of the session |
| [LOG REPLAY](#log-replay) | replay a `.tlog` or `.bin` flight log |

## VEHICLE

| | |
|---|---|
| **TYPE** | The vehicle family from its heartbeat: Copter (multirotors and helicopters), Plane (fixed wing and VTOL), Rover (rovers and boats) or Sub |
| **SYS ID** | Its MAVLink system ID (1 unless changed with `SYSID_THISMAV`) |
| **FW** | The firmware version it reports |

`--` everywhere means no vehicle is connected.

## ACTIONS

| Button | What it does |
|--------|--------------|
| **SET HOME** | After a confirmation, makes the vehicle's **current position** its new home (`MAV_CMD_DO_SET_HOME`): RTL and the relative altitudes then refer to it. The autopilot needs a 3D GPS fix and a healthy EKF; if it refuses, the reason is shown. Useful when the vehicle was armed somewhere else than where it should return, e.g. a boat launched from a pier. |
| **REBOOT FC** | After a confirmation, reboots the autopilot (`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`) — needed after some parameter changes (a new frame, a serial port protocol, a calibration that asks for it). **Greyed out while armed.** The link drops and comes back when the autopilot has restarted. |

## MISSION

| | |
|---|---|
| **READ MISSION** | Asks the autopilot for its mission list (`MISSION_REQUEST_LIST`). To download the mission into the editor and see it on the map and in 3D, use **READ** on the [FLIGHT PLAN](mission-planning.md#upload-read-save) tab. |
| **WPT** | Active mission item / number of items, as the autopilot reports them (`MISSION_CURRENT`) — the same as **WP** in the command bar. |

## GCS OPTIONS

| Option | Default | |
|--------|---------|---|
| **MUTE GCS OUTPUT** | off | Stops **everything the GCS sends** to the vehicle: heartbeats, commands, mode changes, parameter writes, joystick RC override, RTK corrections, mission uploads. The GCS keeps receiving and displaying. For listening in on a vehicle flown by another ground station without interfering, or for replaying a link. The HUD says *GCS output MUTED — no messages will be sent*. **It is remembered between sessions**: if buttons seem to do nothing, check here first. Note that with the output muted the vehicle sees no GCS heartbeat, which can trigger its GCS failsafe if one is configured. |
| **SHOW ADS-B TRAFFIC** | on | Manned-aircraft traffic in the 3D view, on the mini-map and in the traffic table. The same switch as SYS CONFIG → GCS OPTIONS → ADS-B TRAFFIC. See [ADS-B traffic](flight-screen.md#ads-b-traffic). |
| **SHOW ROTOR LOAD** | on | The rotor load schematic at the bottom right. The same switch as SYS CONFIG → ROTOR LOAD → SCHEMATIC. See [rotor load](sys-config.md#rotor-load). |
| **CLEAR TRAIL** | | Erases the red trail of the path flown, in 3D and on the mini-map; it starts again from the current position. |

## RTL OPTIONS

The parameters that decide how the vehicle comes home, in user units (the GCS converts to the
centimetres ArduPilot stores):

| Field | Parameter | |
|-------|-----------|---|
| **RTL ALT (m)** | `RTL_ALT` | Height above home the vehicle climbs to (if lower) before flying back. 0 = return at the current height. |
| **RTL ALT FINAL (m)** | `RTL_ALT_FINAL` | Height it descends to over home at the end: 0 = land; above 0 = hover there. |
| **RTL SPEED (m/s)** | `RTL_SPEED` | Horizontal speed on the way back; 0 = the waypoint speed (`WPNAV_SPEED`). |
| **SRTL POINTS** | `SRTL_POINTS` | Smart RTL: how many points of the path flown the vehicle remembers to retrace (0 disables Smart RTL; needs a reboot). |
| **SRTL ACCURACY (m)** | `SRTL_ACCURACY` | Smart RTL: how closely the remembered path follows the real one. |

**READ FROM VEHICLE** fills the fields from the parameters the GCS already holds — load them first
with **READ ALL** (or **READ LISTED**) on [SETUP → PARAMETERS](setup.md#parameters). **WRITE TO
VEHICLE** sends only the fields you changed. These are Copter parameter names; ArduPilot:
[RTL mode](https://ardupilot.org/copter/docs/rtl-mode.html),
[Smart RTL](https://ardupilot.org/copter/docs/smartrtl-mode.html).

## TARGET

A point of reference for the pilot — a person to reach, a landing spot, the boat to return to.

1. Type its **LAT** and **LON** in decimal degrees (e.g. `45.60319`, `10.67127`).
2. Press **SET TARGET**.

The target then appears as a **red pole with a sphere and a ring** in the 3D view and as a red dot on
the mini-map, and the command bar shows a **red arrow** pointing to it relative to the vehicle's nose
(straight up = dead ahead) with the **distance** (m, or km beyond 1 km). **CLEAR** removes it.

The target is a display aid only: **nothing is sent to the vehicle**. To make the vehicle fly there,
plan a waypoint on it in FLIGHT PLAN, or fly there yourself following the arrow.

## MESSAGE LOG

Every text message the autopilot sent in the session (`STATUSTEXT`), oldest first, scrolled to the
newest: errors prefixed `[ERR]`, warnings `[WRN]`. The same messages appear briefly over the HUD; here
they stay, so a pre-arm failure or a calibration result can be read again. Select and copy the text
to paste it elsewhere. The full log, including the GCS's own events, is also in the
[debug log](sys-config.md#system-config).

## LOG REPLAY

Shown while no vehicle is connected.

1. **OPEN LOG FILE…** and choose an ArduPilot telemetry log (`.tlog`, written by any GCS — the GCS's
   own are in `data/logs`) or a DataFlash log (`.bin`, from the autopilot's SD card).
2. The panel shows **FILE**, **FORMAT**, **DUR** (duration) and **MSGS** (messages); the flight screen
   behaves as if the vehicle were connected: HUD, 3D, mini-map, annunciators, the mission.
3. The **REPLAY** timeline at the bottom right: **▶** play / **❚❚** pause, the time elapsed, the
   scrubber to jump anywhere, the total time, **×** to unload.
4. **UNLOAD** (or ×) ends the replay.

Replay works in the relative navigation mode too: a ROV dive without GPS replays with its
dead-reckoned position ([navigation without GPS](3D-NAVIGATION.md#5-navigation-without-gps)).
ArduPilot's [logs](https://ardupilot.org/copter/docs/common-logs.html) page explains the two kinds of log.
