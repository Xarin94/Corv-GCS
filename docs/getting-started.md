---
title: Getting started
nav_order: 2
description: Install CORV GCS, load terrain, connect a vehicle and choose the flight stack
---

# Getting started

## Install

Download from the [Releases](https://github.com/Xarin94/Corv-GCS/releases/latest) page:

| System | File | |
|--------|------|---|
| Windows 10 / 11 (x64) | `CORV.GCS.Setup.<version>.exe` | Installer; you can choose the folder |
| Linux (x64) | `CORV.GCS-<version>.AppImage` | Portable: `chmod +x` and run |
| Debian / Ubuntu | `corv-gcs_<version>_amd64.deb` | `sudo apt install ./corv-gcs_<version>_amd64.deb` |

To run from source you need [Node.js](https://nodejs.org/) 18 or newer:

```bash
git clone https://github.com/Xarin94/Corv-GCS.git
cd Corv-GCS
npm install
npx electron-rebuild     # native serial-port module for Electron
npm start
```

A serial telemetry radio on Linux needs your user in the `dialout` group
(`sudo usermod -aG dialout $USER`, then log in again).

## First launch

The app opens on the **FLIGHT DATA** tab in demo mode: a simulated flight over Innsbruck, so the
3D view has something to show before any vehicle is connected. The loading screen waits for the
terrain around the aircraft; it goes away by itself after a short timeout if the network is slow.

The tabs along the top:

| Tab | |
|-----|---|
| **FLIGHT DATA** | The 3D view, the HUD and the command bar — see [Flight screen](flight-screen.md) |
| **FLIGHT PLAN** | The mission editor — see [Mission planning](mission-planning.md) |
| **SETUP** | Connection, calibration, failsafes, tuning, parameters, simulator, LiDAR — see [Vehicle setup](setup.md) |
| **SYS CONFIG** | Flight stack, navigation mode, language, 3D model, HUD fields, offline data — see [Vehicle setup](setup.md#sys-config) |

## Terrain and imagery

The 3D terrain is built from **SRTM** elevation tiles, one file per 1° × 1° square (about 25 MB at
30 m resolution, named like `N47E011.hgt`). The tiles under the vehicle are **downloaded
automatically** when a network is available and kept on disk. Without a network you can:

- put your own `.hgt` files in `topography/` inside the installation folder, or pick a folder in
  **SYS CONFIG → SYSTEM CONFIG → TERRAIN (HGT FOLDER)** — from
  [OpenTopography](https://portal.opentopography.org/raster?opentopoID=OTSRTM.082015.4326.1) or
  [USGS EarthExplorer](https://earthexplorer.usgs.gov/);
- download an area ahead of time in **SYS CONFIG → OFFLINE DATA DOWNLOAD**: a latitude/longitude
  box, the highest satellite zoom to keep, and the SRTM tiles under it.

Satellite tiles are cached as they are shown, so an area flown once works offline afterwards.

Missions, flight logs and LiDAR maps are written to a `data/` folder next to the installation
(`data/missions`, `data/logs`, `data/lidar`); under `Program Files`, where that is not allowed, they
go to the per-user data folder. Custom aircraft models (`.glb` / `.gltf`) go in `models/`.

## Choose the flight stack

**SYS CONFIG → FLIGHT STACK → ENVIRONMENT** tells the GCS which firmware it talks to:

| Choice | Link it suggests | What changes |
|--------|------------------|--------------|
| **ArduPilot** (Copter / Plane / Rover / Sub) | MAVLink serial, 57 600 baud | Everything available |
| **INAV** | MSP serial, 115 200 baud | Missions upload as INAV waypoint lists; what INAV cannot fly is greyed out while planning; MAVLink-only setup pages are disabled |
| **Betaflight** | MSP serial | No navigation: the FLIGHT PLAN tab is disabled; telemetry only |

## Connect a vehicle

**SETUP → COMMS → CONNECTION**:

| CONNECTION TYPE | Typical use | Default |
|-----------------|-------------|---------|
| MAVLink Serial | USB telemetry radio (SiK, RFD900, …) or the autopilot's USB port | 57 600 baud (115 200 on USB) |
| MAVLink UDP | MAVProxy, mavlink-router, a companion computer, Herelink | `127.0.0.1:14550` |
| MAVLink TCP | SITL, also through WSL on Windows | `127.0.0.1:5760` |
| MSP Serial (INAV / Betaflight) | The flight controller's USB or a telemetry UART | 115 200 baud; *Slow link* polling for long-range radios |
| MSP TCP (INAV SITL) | INAV's simulator | |
| CORV Binary (Legacy) | The onboard CORV autopilot over USB | 460 800 baud |

**SCAN** lists the serial ports; **CONNECT** opens the link and requests the data streams. The
**FIRMWARE INFO** panel shows what answered. A cellular module (LTEtelem) is connected from
**SETUP → COMMS → CELLULAR LINK** instead — see [Vehicle setup](setup.md#cellular-link).

Once connected, the command bar at the bottom shows the link, the battery, the GPS and the flight
mode, and the annunciators beside the speed tape flag anything wrong — see
[Flight screen](flight-screen.md). Before a first flight, work through the ArduPilot
[pre-arm checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html) and
[arming](https://ardupilot.org/copter/docs/arming_the_motors.html) pages for your vehicle.

## No aircraft at hand?

**SETUP → TOOLS → SIMULATION** downloads and starts an ArduPilot simulator with one click and
connects to it — see [Simulator](simulator.md).
