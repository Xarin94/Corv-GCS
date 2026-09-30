---
title: Simulator
nav_order: 9
description: ArduPilot SITL with one click, the ROV on Lake Garda, and connecting to your own simulator
---

# Simulator

**SETUP → TOOLS → SIMULATION** runs ArduPilot's software-in-the-loop simulator (SITL) — the real
autopilot firmware, flying a physics model on the PC — and connects to it. Everything in the GCS
works against it: missions, modes, parameters, failsafes, logs. ArduPilot:
[SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html),
[using SITL for testing](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html).

<p align="center"><img src="images/sitl-rov.png" width="350" alt="SITL launcher"/></p>

## Launch

| Field | |
|-------|---|
| **VEHICLE TYPE** | See the list below |
| **VERSION** | Stable, Beta or Latest (development) firmware |
| **HOME POSITION** | Latitude and longitude of the start; the altitude is taken from the terrain under it (0 m for a ROV, see below) |
| **SPEED MULTIPLIER** | Run the simulation up to 10× faster than real time |

**LAUNCH & CONNECT** downloads the firmware the first time (from `firmware.ardupilot.org`, kept in the
app's data folder), starts it and connects over TCP. **STOP** disconnects and ends it. On Windows the
Linux binary runs inside **WSL**, which must be installed.

| Vehicle | Model |
|---------|-------|
| Copter (Quad) | ArduCopter, standard quad |
| Copter 12S High-Speed | ArduCopter on a fast 12S quad frame |
| Tricopter 12S | ArduCopter, symmetric 12S tricopter (3 × Hobbywing X8 G2) |
| Helicopter | ArduCopter, traditional helicopter |
| Plane | ArduPlane on a 22 kg jet model: 300 km/h cruise, about 660 km/h top speed (needs ArduPlane 4.7+) |
| QuadPlane | ArduPlane VTOL |
| Rover | ArduRover |
| ROV · Sub (BlueROV2, vectored) | ArduSub, with GPS |
| ROV · no GPS (dead reckoning) | ArduSub with no GPS at all |

## The ROV

Choosing a ROV moves the home to the **centre of Lake Garda** (45.60319 N, 10.67127 E), unless you
typed a home of your own. It is always launched at **0 m**: ArduSub's simulator keeps the water
surface at 0 m MSL whatever the home altitude, and a sub launched at the lake's own height (62 m in the
elevation data) floats in the air and cannot dive. The GCS puts that 0 on the lake surface, as it does
for a real ROV — the depth then reads correctly and the 3D view shows the vehicle under the lake. See
[Subs and ROVs](3D-NAVIGATION.md#4-subs-and-rovs).

**ROV · no GPS** starts ArduSub with the GPS driver and the simulated GPS off
(`default_params_subnogps.parm`): the autopilot has no position at all, as a ROV without DVL or USBL.
Launching it switches the [relative navigation mode](3D-NAVIGATION.md#5-navigation-without-gps) and the
dead-reckoning track on; **STOP** puts both back as they were.

Driving a ROV: a gamepad in [SETUP → Joystick](setup.md#joystick), or `MANUAL_CONTROL` from another
program on the simulator's second port (TCP 5762). ArduSub only takes pilot input from its own GCS
system ID (`SYSID_MYGCS`, 255), and reacts when that input stops for a few seconds
([pilot control failsafe](https://ardupilot.org/sub/docs/pilot-control-failsafe.html)) — in the
simulator it disarmed the vehicle.

## Your own simulator

**SITL MANUAL CONNECTION** connects over UDP to a simulator started elsewhere — MAVProxy's output,
`sim_vehicle.py`, a Gazebo setup — on `127.0.0.1:14550` by default. A TCP simulator
(`tcp:127.0.0.1:5760`) connects from [SETUP → COMMS → CONNECTION](getting-started.md#connect-a-vehicle).
