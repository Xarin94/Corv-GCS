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
| Windows 10 / 11 (x64) | `CORV-GCS-<version>-setup-x64.exe` | Installer, per user (no administrator rights); you can choose the folder |
| Windows 10 / 11 (x64) | `CORV-GCS-<version>-windows-x64.zip` | Portable: unzip anywhere and run `corv-gcs.exe` |
| Linux (x64) | `CORV-GCS-<version>-x86_64.AppImage` | Portable: `chmod +x` and run; Ubuntu 22.04 or a newer distribution |
| macOS (Apple silicon and Intel) | `CORV-GCS-<version>-macos-universal.dmg` | Drag *CORV GCS* to Applications |

Version 2.x installs next to 1.x, so the two can stay side by side: on Windows in its own
folder (`%LOCALAPPDATA%\Programs\CORV Systems\CORV GCS`) with its own uninstaller, and the
*CORV GCS* shortcuts start 2.x. Its settings start from the defaults; **SYS CONFIG → APPLICATION →
SETTINGS OF 1.x** copies those of 1.x found on the computer. Over a copy installed as
*CORV GCS 2* (the first 2.0.x installers), the installer keeps that folder and its data and
renames the shortcuts and the entry in the installed apps.

The macOS app is not notarized by Apple, so macOS refuses to open it the first time: open it from
**System Settings → Privacy & Security → Open Anyway**, or clear the download flag once after
copying it to Applications: `xattr -dr com.apple.quarantine "/Applications/CORV GCS.app"`.

### Updates

A few seconds after start the GCS looks at the releases of this repository on GitHub (no account,
nothing sent but the request). When a newer version is out, **UPDATE** appears at the top right;
it opens the release notes and **INSTALL** downloads only the package for your system, checks it
against the checksum GitHub publishes, and installs it: on Windows the installer runs by itself
into the same folder and starts the new version, on Linux the AppImage is replaced and restarted,
on macOS the disk image opens for the drag to Applications. Never while the vehicle is armed.
**SKIP THIS VERSION** stops the reminders for that version; [SYS CONFIG →
UPDATES](sys-config.md#updates) switches the check off or runs it now.

To build from source (C++, Qt 6.12, CMake) see
[BUILDING.md](https://github.com/Xarin94/Corv-GCS/blob/main/BUILDING.md).

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
| **SETUP** | Connection, calibration, failsafes, tuning, parameters, simulator, LiDAR, ROS — see [Vehicle setup](setup.md) |
| **SYS CONFIG** | Flight stack, navigation mode, language, 3D model, HUD fields, offline data, stream rates — see [GCS settings](sys-config.md) |

On the flight screen, the **GCS OPTIONS** tab on the right edge opens the [side panel](side-panel.md): set
home, reboot, mute the GCS, ADS-B traffic, RTL options, a target, the message log and log replay.
The window has no frame: move the mouse to the top edge for the minimise / maximise / close buttons.

## Terrain and imagery

The 3D terrain is built from **SRTM** elevation tiles, one file per 1° × 1° square (about 25 MB at
30 m resolution, named like `N47E011.hgt`). The tiles under the vehicle are **downloaded
automatically** when a network is available and kept on disk, in `data/terrain` next to the
installation (in the per-user data folder when the installation folder is read-only). Without a
network you can:

- put your own `.hgt` files in `topography/` inside the installation folder, or pick a folder in
  **SYS CONFIG → 3D VIEW → TERRAIN (HGT FOLDER)** — from
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
