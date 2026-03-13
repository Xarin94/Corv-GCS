# CRV File Format Specification v1.0

Binary telemetry log format for CORV GCS. Records MAVLink telemetry at 10 Hz.

## Overview

| Property | Value |
|----------|-------|
| Extension | `.crv` |
| Byte order | Little-endian |
| Sample rate | 10 Hz (navigation), 1 Hz (system status) |
| Sync bytes | `0xA5 0x5A` |
| CRC | CRC-16-CCITT (poly 0x1021, init 0xFFFF) |
| Bandwidth | ~930 bytes/sec (~3.3 MB/hour) |

## Packet Structure

Every record in the file is a packet with this structure:

```
┌──────┬──────┬──────┬────────┬──────┬───────────┬─────────┐
│ 0xA5 │ 0x5A │ Type │ Length │ Seq  │ Payload   │ CRC-16  │
│ 1B   │ 1B   │ 1B   │ 2B     │ 1B   │ N bytes   │ 2B      │
└──────┴──────┴──────┴────────┴──────┴───────────┴─────────┘
```

| Field | Bytes | Description |
|-------|-------|-------------|
| Sync | 2 | `0xA5 0x5A` magic sync bytes |
| Type | 1 | Packet type ID (0x10–0x13) |
| Length | 2 | Payload length (little-endian uint16) |
| Sequence | 1 | Rolling counter 0–255 |
| Payload | N | Type-specific data |
| CRC-16 | 2 | CRC-16-CCITT over bytes from Type to end of Payload (little-endian) |

**Header size: 6 bytes. Total packet size: 6 + N + 2 bytes.**

CRC is computed on `[Type, LengthLo, LengthHi, Sequence, Payload...]` — i.e., everything between sync bytes and CRC.

---

## Packet Type 0x10 — File Header

Written once at the beginning of the file. Identifies the recording session.

**Payload: 32 bytes. Total packet: 40 bytes.**

| Offset | Field | Type | Bytes | Description |
|--------|-------|------|-------|-------------|
| 0 | magic | char[4] | 4 | ASCII `"CRV\0"` (0x43 0x52 0x56 0x00) |
| 4 | formatVersion | uint8 | 1 | Format version = `1` |
| 5 | reserved | uint8 | 1 | Reserved (0x00) |
| 6 | startTimestamp | uint64 | 8 | Recording start time, Unix epoch milliseconds |
| 14 | sampleRateHz | uint8 | 1 | Navigation sample rate = `10` |
| 15 | connectionType | uint8 | 1 | 0=none, 1=corv-binary, 2=mavlink-serial, 3=mavlink-udp, 4=mavlink-tcp |
| 16 | vehicleType | uint8 | 1 | MAV_TYPE from HEARTBEAT |
| 17 | autopilotType | uint8 | 1 | MAV_AUTOPILOT from HEARTBEAT |
| 18 | systemId | uint8 | 1 | MAVLink system ID |
| 19 | componentId | uint8 | 1 | MAVLink component ID |
| 20 | reserved2 | uint8[12] | 12 | Reserved for future use (zero-filled) |

---

## Packet Type 0x11 — Navigation Data

Primary telemetry record, written at 10 Hz.

**Payload: 80 bytes. Total packet: 88 bytes.**

| Offset | Field | Type | Bytes | Scale | Unit | Source |
|--------|-------|------|-------|-------|------|--------|
| 0 | timestampMs | uint32 | 4 | — | ms | Elapsed since recording start |
| 4 | roll | int16 | 2 | ×1000 | rad | STATE.roll |
| 6 | pitch | int16 | 2 | ×1000 | rad | STATE.pitch |
| 8 | yaw | int16 | 2 | ×1000 | rad | STATE.yaw |
| 10 | lat | float64 | 8 | — | deg | STATE.lat |
| 18 | lon | float64 | 8 | — | deg | STATE.lon |
| 26 | rawAlt | float32 | 4 | — | m | STATE.rawAlt (MSL) |
| 30 | offsetAlt | float32 | 4 | — | m | STATE.offsetAlt |
| 34 | airspeed | float32 | 4 | — | m/s | STATE.as |
| 38 | groundspeed | float32 | 4 | — | m/s | STATE.gs |
| 42 | verticalSpeed | float32 | 4 | — | m/s | STATE.vs |
| 46 | vn | float32 | 4 | — | m/s | STATE.vn (NED north) |
| 50 | ve | float32 | 4 | — | m/s | STATE.ve (NED east) |
| 54 | vd | float32 | 4 | — | m/s | STATE.vd (NED down) |
| 58 | ax | int16 | 2 | ×100 | m/s² | STATE.ax |
| 60 | ay | int16 | 2 | ×100 | m/s² | STATE.ay |
| 62 | az | int16 | 2 | ×100 | m/s² | STATE.az |
| 64 | aoa | int16 | 2 | ×1000 | rad | STATE.aoa |
| 66 | ssa | int16 | 2 | ×1000 | rad | STATE.ssa |
| 68 | gamma | int16 | 2 | ×1000 | rad | STATE.gamma (flight path angle) |
| 70 | track | int16 | 2 | ×1000 | rad | STATE.track |
| 72 | terrainHeight | float32 | 4 | — | m | STATE.terrainHeight (NaN if null) |
| 76 | rangefinderDist | float32 | 4 | — | m | STATE.rangefinderDist (NaN if null) |

### Scaling Notes
- **Attitude** (roll, pitch, yaw, aoa, ssa, gamma, track): stored as milliradians (int16). Range ±32.767 rad — sufficient for all angles.
- **Acceleration** (ax, ay, az): stored as cm/s² (int16). Range ±327.67 m/s² — sufficient for flight data.
- **Position** (lat, lon): stored as float64 for full precision.
- **Null values**: `terrainHeight` and `rangefinderDist` use IEEE 754 NaN when data is unavailable.

---

## Packet Type 0x12 — System Status

System and sensor status data, written at 1 Hz.

**Payload: 52 bytes. Total packet: 60 bytes.**

| Offset | Field | Type | Bytes | Scale | Description |
|--------|-------|------|-------|-------|-------------|
| 0 | timestampMs | uint32 | 4 | — | ms since recording start |
| 4 | batteryVoltage | uint16 | 2 | ×100 | Volts (e.g., 1250 = 12.50 V) |
| 6 | batteryCurrent | int16 | 2 | ×100 | Amps |
| 8 | batteryRemaining | int8 | 1 | — | 0–100%, or -1 = unknown |
| 9 | gpsFix | uint8 | 1 | — | 0=No GPS, 1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed |
| 10 | gpsNumSat | uint8 | 1 | — | Visible satellites |
| 11 | gpsHdop | uint16 | 2 | ×100 | Horizontal DOP |
| 13 | armed | uint8 | 1 | — | 0=disarmed, 1=armed |
| 14 | flightModeNum | uint8 | 1 | — | ArduPilot custom mode number |
| 15 | baseMode | uint8 | 1 | — | MAVLink base mode bitmask |
| 16 | linkQuality | uint8 | 1 | — | 0–100% |
| 17 | rtkIar | uint16 | 2 | — | Integer ambiguity resolution count |
| 19 | rtkBaseline | uint32 | 4 | — | Baseline to base station (mm) |
| 23 | rtkAccuracy | uint16 | 2 | — | Position accuracy (mm) |
| 25 | vibX | float32 | 4 | — | IMU vibration X |
| 29 | vibY | float32 | 4 | — | IMU vibration Y |
| 33 | vibZ | float32 | 4 | — | IMU vibration Z |
| 37 | homeLat | float32 | 4 | — | Home latitude (degrees) |
| 41 | homeLon | float32 | 4 | — | Home longitude (degrees) |
| 45 | homeAlt | float32 | 4 | — | Home altitude (meters) |
| 49 | reserved | uint8[3] | 3 | — | Reserved (zero) |

---

## Packet Type 0x13 — Event

Discrete events (arm/disarm, mode change, status messages). Written on-demand.

**Payload: 8 + textLength bytes (variable). Max total packet: 66 bytes.**

| Offset | Field | Type | Bytes | Description |
|--------|-------|------|-------|-------------|
| 0 | timestampMs | uint32 | 4 | ms since recording start |
| 4 | eventType | uint8 | 1 | 0=arm_change, 1=mode_change, 2=status_text |
| 5 | eventData | uint8 | 1 | arm: 0/1; mode: flightModeNum |
| 6 | severity | uint8 | 1 | MAVLink severity (for status_text), 0 otherwise |
| 7 | textLength | uint8 | 1 | 0–50 |
| 8 | text | char[] | 0–50 | UTF-8 text (flight mode name or status message) |

### Event Types
| Value | Name | eventData | text |
|-------|------|-----------|------|
| 0 | ARM_CHANGE | 0=disarmed, 1=armed | — |
| 1 | MODE_CHANGE | flightModeNum | Flight mode name |
| 2 | STATUS_TEXT | 0 | Status message from vehicle |

---

## File Layout

```
┌─────────────────────────────────────┐
│ File Header (0x10)     — 40 bytes   │  ← exactly once
├─────────────────────────────────────┤
│ Navigation (0x11)      — 88 bytes   │  ← 10 Hz
│ Navigation (0x11)      — 88 bytes   │
│ ...                                 │
│ System Status (0x12)   — 60 bytes   │  ← 1 Hz (every 10th nav)
│ Event (0x13)           — variable   │  ← on arm/mode/status change
│ Navigation (0x11)      — 88 bytes   │
│ ...                                 │
└─────────────────────────────────────┘
```

## Storage Estimates

| Duration | File Size |
|----------|-----------|
| 1 minute | ~56 KB |
| 10 minutes | ~560 KB |
| 1 hour | ~3.3 MB |
| 4 hours | ~13.2 MB |

## Connection Type Encoding

| Value | Type |
|-------|------|
| 0 | none |
| 1 | corv-binary |
| 2 | mavlink-serial |
| 3 | mavlink-udp |
| 4 | mavlink-tcp |
