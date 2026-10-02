---
title: Home
nav_order: 1
description: CORV GCS user guide — a 3D ground control station for ArduPilot, INAV and Betaflight
permalink: /
---

# CORV GCS user guide

CORV GCS is a ground control station: the software on the laptop that talks to the drone, shows what
it is doing, plans its missions and sends them to the autopilot. It works with ArduPilot vehicles
(Plane, Copter, Rover, Sub, Heli, VTOL), with INAV and Betaflight controllers over MSP, and draws
everything on a **real 3D model of the terrain**. It runs on Windows and Linux.

[Download the latest release](https://github.com/Xarin94/Corv-GCS/releases/latest){: .btn .btn-primary .mr-2 }
[Source on GitHub](https://github.com/Xarin94/Corv-GCS){: .btn }

![Flight screen](images/flight-hud.jpg)

## Chapters

| Chapter | What it covers |
|---------|----------------|
| [Getting started](getting-started.md) | Installing, the first launch, terrain data, connecting a vehicle, choosing the flight stack |
| [Flight screen](flight-screen.md) | The HUD, the 3D view and its controls, the schematic chart, annunciators, the command bar, FPV video, logs and replay |
| [Flight modes](flight-modes.md) | Every mode the GCS offers for Copter, Plane, Rover, Sub and INAV, which ones need a position, and the ArduPilot page for each |
| [Mission planning](mission-planning.md) | Drawing a route as segments, what each becomes on the autopilot, altitudes and terrain, camera surveys, upload and read-back |
| [Radio link planning](radio-link.md) | Coverage over the terrain, Fresnel zone, the link profile along the route |
| [3D view, water and ROVs](3D-NAVIGATION.md) | The mission in 3D, the low-altitude grid, lakes and seas, ROVs, navigation without GPS |
| [Vehicle setup](setup.md) | Every SETUP and SYS CONFIG page: links, calibration, failsafes, tuning, parameters, RTK, telemetry forwarding |
| [Simulator](simulator.md) | ArduPilot SITL with one click, the ROV on Lake Garda, connecting to your own simulator |
| [LiDAR point cloud](LIDAR.md) | Livox Mid-360 mapping: network, mounting, telemetry lag, saving `.ply` |
| [ROS surface](ROS.md) | LiDAR, echo sounder and sonar from ROS (rosbridge): an averaged 30 cm mesh, terrain, sea bed and caves in 3D |
| [Keyboard shortcuts](shortcuts.md) | All keys, on the flight screen and in the planner |

## Where the ArduPilot documentation fits

CORV GCS shows, commands and plans; the vehicle's behaviour is ArduPilot's. Each chapter links the
[ArduPilot documentation](https://ardupilot.org/ardupilot/) page for what the GCS is driving — the
flight mode, the mission command, the failsafe, the calibration — so you can check what the
autopilot will actually do. When the two disagree, the autopilot documentation for your firmware
version wins.

## Community

Questions, bug reports and feature requests: the
[issues](https://github.com/Xarin94/Corv-GCS/issues) on GitHub, or the
[CORV GCS thread](https://discuss.ardupilot.org/t/corv-gcs-open-source-3d-ground-control-station-for-ardupilot/142922)
on the ArduPilot forum. Developers: [ARCHITECTURE.md](https://github.com/Xarin94/Corv-GCS/blob/main/ARCHITECTURE.md)
describes every module and data flow.
