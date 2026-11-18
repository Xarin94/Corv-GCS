---
title: Flight modes
nav_order: 5
description: Every flight mode CORV GCS offers per vehicle, which ones need a position, with the ArduPilot page for each
---

# Flight modes

The flight mode decides who flies and how: the pilot through the sticks, the autopilot holding what
the pilot asks, or the autopilot on its own. CORV GCS shows the current mode in the command bar,
coloured by kind, and switches it from the list next to it; the behaviour of each mode is the
autopilot's, described in the ArduPilot documentation linked below for every one.

| Colour in the command bar | Kind |
|---------------------------|------|
| <span style="color:#ffcc00">■</span> yellow | **Manual** — the pilot flies, the autopilot at most stabilises |
| <span style="color:#00d2ff">■</span> cyan | **Assisted** — the autopilot holds height, position or attitude while the pilot steers (also every mode not listed otherwise) |
| <span style="color:#44ff44">■</span> green | **Automatic** — mission, autotune, brake |
| <span style="color:#ff9900">■</span> orange | **Return / land** — RTL, Smart RTL, Auto RTL, land, QRTL, QLAND, a sub's SURFACE |

A mode change is a request: the autopilot refuses a mode it cannot fly — most often one that needs a
position when it has none — and the list goes back to the current mode with the error. Which switch
position selects which mode on the radio is set in **SETUP → VEHICLE SETUP → FLIGHT MODES** (mode
channel and six slots, read from and written to the vehicle) — ArduPilot:
[flight mode configuration](https://ardupilot.org/copter/docs/common-rc-transmitter-flight-mode-configuration.html).

## Position and the modes that need it

ArduPilot divides its modes into those that **need a valid position** — a GPS fix, or another
position source feeding the EKF — and those that do not. Without a position, only the second group
can be flown. The tables below give the requirement as the ArduPilot documentation states it
([Copter flight modes](https://ardupilot.org/copter/docs/flight-modes.html),
[Plane flight modes](https://ardupilot.org/plane/docs/flight-modes.html),
[Sub modes](https://ardupilot.org/sub/docs/modes.html)); "see page" where that list does not.

This is why the GCS has a [relative navigation mode](3D-NAVIGATION.md#5-navigation-without-gps): with
no GPS at all the autopilot reports no position, and what can still be shown is a position the GCS
estimates. Flying the position modes without GPS needs a position source on the vehicle — optical
flow, visual odometry, a DVL or USBL on a sub —
[non-GPS navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html),
[EKF sources](https://ardupilot.org/copter/docs/common-ekf-sources.html).

## Copter

| # | Mode | Colour | Needs position | ArduPilot |
|---|------|--------|----------------|-----------|
| 0 | STABILIZE | yellow | no | [Stabilize](https://ardupilot.org/copter/docs/stabilize-mode.html) |
| 1 | ACRO | yellow | no | [Acro](https://ardupilot.org/copter/docs/acro-mode.html) |
| 2 | ALT_HOLD | cyan | no | [Alt Hold](https://ardupilot.org/copter/docs/altholdmode.html) |
| 3 | AUTO | green | **yes** | [Auto](https://ardupilot.org/copter/docs/auto-mode.html) |
| 4 | GUIDED | cyan | **yes** | [Guided](https://ardupilot.org/copter/docs/ac2_guidedmode.html) |
| 5 | LOITER | cyan | **yes** | [Loiter](https://ardupilot.org/copter/docs/loiter-mode.html) |
| 6 | RTL | orange | **yes** | [RTL](https://ardupilot.org/copter/docs/rtl-mode.html) |
| 7 | CIRCLE | cyan | **yes** | [Circle](https://ardupilot.org/copter/docs/circle-mode.html) |
| 9 | LAND | orange | no | [Land](https://ardupilot.org/copter/docs/land-mode.html) |
| 11 | DRIFT | cyan | **yes** | [Drift](https://ardupilot.org/copter/docs/drift-mode.html) |
| 13 | SPORT | cyan | no | [Sport](https://ardupilot.org/copter/docs/sport-mode.html) |
| 14 | FLIP | cyan | see page | [Flip](https://ardupilot.org/copter/docs/flip-mode.html) |
| 15 | AUTOTUNE | green | see page | [AutoTune](https://ardupilot.org/copter/docs/autotune.html) |
| 16 | POSHOLD | cyan | **yes** | [PosHold](https://ardupilot.org/copter/docs/poshold-mode.html) |
| 17 | BRAKE | green | see page | [Brake](https://ardupilot.org/copter/docs/brake-mode.html) |
| 18 | THROW | cyan | **yes** | [Throw](https://ardupilot.org/copter/docs/throw-mode.html) |
| 19 | AVOID_ADSB | cyan | see page | [ADS-B avoidance](https://ardupilot.org/copter/docs/common-ads-b-receiver.html) |
| 20 | GUIDED_NOGPS | cyan | no — attitude targets | [Guided](https://ardupilot.org/copter/docs/ac2_guidedmode.html) |
| 21 | SMART_RTL | orange | **yes** | [Smart RTL](https://ardupilot.org/copter/docs/smartrtl-mode.html) |
| 22 | FLOWHOLD | cyan | optical flow instead | [FlowHold](https://ardupilot.org/copter/docs/flowhold-mode.html) |
| 23 | FOLLOW | cyan | **yes** | [Follow](https://ardupilot.org/copter/docs/follow-mode.html) |
| 24 | ZIGZAG | cyan | **yes** | [ZigZag](https://ardupilot.org/copter/docs/zigzag-mode.html) |
| 25 | SYSTEMID | cyan | no | [System ID](https://ardupilot.org/copter/docs/common-systemid-mode.html) |
| 26 | AUTOROTATE | cyan | **yes** (heli) | [Autorotation](https://ardupilot.org/copter/docs/traditional-helicopter-autorotation-mode.html) |
| 27 | AUTO_RTL | orange | **yes** | [Auto](https://ardupilot.org/copter/docs/auto-mode.html) — the mission's landing sequence |
| 28 | TURTLE | cyan | no | [Turtle](https://ardupilot.org/copter/docs/turtle-mode.html) — flips a crashed multirotor back over |

The command bar has shortcut buttons for **RTL**, **AUTO** and **LAND**, and **TAKEOFF** (GUIDED
take-off) — see [Flight screen](flight-screen.md#command-bar).

## Plane and QuadPlane

| # | Mode | Colour | Needs GPS | ArduPilot |
|---|------|--------|-----------|-----------|
| 0 | MANUAL | yellow | no | [Manual](https://ardupilot.org/plane/docs/manual-mode.html) |
| 1 | CIRCLE | cyan | **yes** | [Circle](https://ardupilot.org/plane/docs/circle-mode.html) |
| 2 | STABILIZE | yellow | no | [Stabilize](https://ardupilot.org/plane/docs/stabilize-mode.html) |
| 3 | TRAINING | yellow | no | [Training](https://ardupilot.org/plane/docs/training-mode.html) |
| 4 | ACRO | yellow | no | [Acro](https://ardupilot.org/plane/docs/acro-mode.html) |
| 5 | FBWA | cyan | no | [FBWA](https://ardupilot.org/plane/docs/fbwa-mode.html) |
| 6 | FBWB | cyan | no | [FBWB](https://ardupilot.org/plane/docs/fbwb-mode.html) |
| 7 | CRUISE | cyan | see page | [Cruise](https://ardupilot.org/plane/docs/cruise-mode.html) |
| 8 | AUTOTUNE | green | no | [AutoTune](https://ardupilot.org/plane/docs/autotune-mode.html) |
| 10 | AUTO | green | **yes** | [Auto](https://ardupilot.org/plane/docs/auto-mode.html) |
| 11 | RTL | orange | **yes** | [RTL](https://ardupilot.org/plane/docs/rtl-mode.html) |
| 12 | LOITER | cyan | **yes** | [Loiter](https://ardupilot.org/plane/docs/loiter-mode.html) |
| 13 | TAKEOFF | cyan | **yes** | [Takeoff](https://ardupilot.org/plane/docs/takeoff-mode.html) |
| 14 | AVOID_ADSB | cyan | see page | [ADS-B avoidance](https://ardupilot.org/plane/docs/common-ads-b-receiver.html) |
| 15 | GUIDED | cyan | **yes** | [Guided](https://ardupilot.org/plane/docs/guided-mode.html) |
| 17 | QSTABILIZE | cyan | see page | [QStabilize](https://ardupilot.org/plane/docs/qstabilize-mode.html) |
| 18 | QHOVER | cyan | see page | [QHover](https://ardupilot.org/plane/docs/qhover-mode.html) |
| 19 | QLOITER | cyan | see page | [QLoiter](https://ardupilot.org/plane/docs/qloiter-mode.html) |
| 20 | QLAND | orange | see page | [QLand](https://ardupilot.org/plane/docs/qland-mode.html) |
| 21 | QRTL | orange | see page | [QRTL](https://ardupilot.org/plane/docs/qrtl-mode.html) |
| 22 | QAUTOTUNE | cyan | see page | [QAutoTune](https://ardupilot.org/plane/docs/qautotune-mode.html) |
| 23 | QACRO | cyan | see page | [QAcro](https://ardupilot.org/plane/docs/qacro-mode.html) |
| 24 | THERMAL | cyan | **yes** | [Thermal](https://ardupilot.org/plane/docs/thermal-mode.html) |
| 25 | LOITER_ALT_QLAND | cyan | see page | [QuadPlane modes](https://ardupilot.org/plane/docs/quadplane-flight-modes.html) |
| 26 | AUTOLAND | cyan | **yes** | [Autoland](https://ardupilot.org/plane/docs/mode_autoland.html) |

Without GPS a plane can still be flown in the manual and stabilised modes; the GCS can then only
estimate its position from airspeed and heading — see
[dead reckoning](3D-NAVIGATION.md#dead-reckoning). Measured in SITL, an ArduPlane that
never had a GPS does not start its EKF at all (DCM only), as also discussed on the
[ArduPilot forum](https://discuss.ardupilot.org/t/ekf3-and-ekf2-do-not-work-without-gps-in-arduplane-20769/85511).

## Rover and boat

| # | Mode | Colour | ArduPilot |
|---|------|--------|-----------|
| 0 | MANUAL | yellow | [Manual](https://ardupilot.org/rover/docs/manual-mode.html) |
| 1 | ACRO | yellow | [Acro](https://ardupilot.org/rover/docs/acro-mode.html) |
| 3 | STEERING | cyan | [Steering](https://ardupilot.org/rover/docs/steering-mode.html) |
| 4 | HOLD | cyan | [Hold](https://ardupilot.org/rover/docs/hold-mode.html) |
| 5 | LOITER | cyan | [Loiter](https://ardupilot.org/rover/docs/loiter-mode.html) (boats) |
| 6 | FOLLOW | cyan | [Follow](https://ardupilot.org/rover/docs/follow-mode.html) |
| 7 | SIMPLE | cyan | [Simple](https://ardupilot.org/rover/docs/simple-mode.html) |
| 8 | DOCK | cyan | [Dock](https://ardupilot.org/rover/docs/dock-mode.html) |
| 9 | CIRCLE | cyan | [Circle](https://ardupilot.org/rover/docs/circle-mode.html) |
| 10 | AUTO | green | [Auto](https://ardupilot.org/rover/docs/auto-mode.html) |
| 11 | RTL | orange | [RTL](https://ardupilot.org/rover/docs/rtl-mode.html) |
| 12 | SMART_RTL | orange | [Smart RTL](https://ardupilot.org/rover/docs/smartrtl-mode.html) |
| 15 | GUIDED | cyan | [Guided](https://ardupilot.org/rover/docs/guided-mode.html) |

The [Rover control modes](https://ardupilot.org/rover/docs/rover-control-modes.html) page gives the
requirements of each.

## Sub (ROV)

| # | Mode | Colour | Needs | ArduPilot |
|---|------|--------|-------|-----------|
| 0 | STABILIZE | yellow | — | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 1 | ACRO | yellow | — | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 2 | ALT_HOLD | cyan | depth sensor | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 3 | AUTO | green | **position** and depth | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 4 | GUIDED | cyan | **position** and depth | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 7 | CIRCLE | cyan | **position** and depth | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 9 | SURFACE | orange | — | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 16 | POSHOLD | cyan | **position** and depth | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 19 | MANUAL | yellow | — | [Sub modes](https://ardupilot.org/sub/docs/modes.html) |
| 20 | MOTOR_DETECT | cyan | — | [Sub modes](https://ardupilot.org/sub/docs/modes.html) — finds each thruster's direction |
| 21 | SURFTRAK | cyan | rangefinder | [Sub modes](https://ardupilot.org/sub/docs/modes.html) — holds the distance above the bottom |

ALT_HOLD is depth hold. A sub gets a position from a DVL or an underwater positioning system (USBL),
not from GPS; without one only MANUAL, STABILIZE, ACRO, ALT_HOLD, SURFACE and (with a downward
rangefinder) SURFTRAK are available. A real
BlueROV2 with a USBL, replayed in the GCS, flew POSHOLD at 20 m — see
[a real dive](3D-NAVIGATION.md#a-real-dive). ArduSub acts when the pilot input from its GCS stops for a few
seconds ([pilot control failsafe](https://ardupilot.org/sub/docs/pilot-control-failsafe.html)) — in the
simulator runs for this guide it disarmed the vehicle;
the other sub failsafes are on the [failsafes](https://ardupilot.org/sub/docs/failsafe-landing-page.html) page.

## INAV and Betaflight

Over MSP the GCS resolves INAV's active modes to one name: MANUAL, ACRO, ANGLE, HORIZON, ALTHOLD,
POSHOLD, COURSE HOLD, CRUISE, WAYPOINT, RTH, FAILSAFE. They are selected on the radio, as INAV
expects; the INAV [navigation modes](https://github.com/iNavFlight/inav/blob/master/docs/Navigation.md)
page describes them. Betaflight has no navigation modes.
