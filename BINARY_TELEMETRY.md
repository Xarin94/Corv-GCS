# Complete Guide to Binary Telemetry - Current Implementation

## Table of Contents
1. [Overview](#overview)
2. [Packet Structure](#packet-structure)
3. [CRC-16-CCITT: Detailed Explanation](#crc-16-ccitt-detailed-explanation)
4. [Packet Type 0x01: Navigation](#packet-type-0x01-navigation)
5. [Packet Type 0x02: Debug](#packet-type-0x02-debug)
6. [Encoding and Decoding](#encoding-and-decoding)
7. [Configuration](#configuration)
8. [Practical Examples](#practical-examples)

---

## Overview

Binary telemetry is a compact and efficient protocol for transmitting navigation data from the RI-EKF system. Compared to text-based formats like JSON:

- **Reduces bandwidth** by 77.5% (from ~200 kbps to ~45 kbps)
- **Uses 9.7%** of link capacity at 460,800 baud
- **Maintains exact precision** for all critical data
- **Implements validation** via CRC-16 checksum

### Bandwidth Performance

| Packet Type | Frequency | Size | Bandwidth |
|---|---|---|---|
| Navigation (0x01) | 50 Hz | 111 bytes | 44.4 kbps |
| Debug (0x02) | 1 Hz | 71 bytes | 0.568 kbps |
| **Total** | - | - | **~45 kbps** |

---

## Packet Structure

All packets follow an identical structure:

```
┌─────────────┬──────────┬────────┬────────┬──────────┬─────────┐
│ Sync Byte 1 │ Sync Byte 2 │ Type ID │ Length   │ Sequence │ Payload │ CRC   │
│ 0xA5        │ 0x5A       │ 1 byte │ 1 byte   │ 1 byte  │ N bytes │ 2 bytes│
├─────────────┼──────────┼────────┼────────┼──────────┼─────────┤
│ Byte 0      │ Byte 1   │ Byte 2 │ Byte 3 │ Byte 4  │ Bytes 5..N+4 │ Byte N+5..N+6 │
└─────────────┴──────────┴────────┴────────┴──────────┴─────────┘

Header (5 bytes) + Payload (N bytes) + CRC (2 bytes)
```

### Header (5 bytes)

| Offset | Name | Type | Value | Description |
|--------|------|------|-------|-------------|
| **0** | Sync 1 | uint8 | `0xA5` | First sync byte (fixed) |
| **1** | Sync 2 | uint8 | `0x5A` | Second sync byte (fixed) |
| **2** | Type ID | uint8 | `0x01` or `0x02` | Packet type |
| **3** | Length | uint8 | `104` or `64` | Payload length **in bytes** |
| **4** | Sequence | uint8 | `0-255` | Sequential number (wraps at 256) |

### Global Parameters

- **Byte order**: LITTLE-ENDIAN (LSB first) for all multi-byte fields
- **Float**: IEEE 754 (32-bit single precision)
- **Double**: IEEE 754 (64-bit double precision)
- **Scaled integers**: int16_t with specific scale factors

---

## CRC-16-CCITT: Detailed Explanation

CRC (Cyclic Redundancy Check) is an algorithm that produces a 2-byte value to detect transmission errors.

### Algorithm Used: CRC-16-CCITT

**Fundamental parameters:**
- **Polynomial**: `0x1021` (representing x^16 + x^12 + x^5 + 1)
- **Initial value**: `0xFFFF` (all bits set to 1)
- **Bit order**: MSB (Most Significant Bit) first processing
- **CRC byte order**: Little-Endian (LSB written first)

### What the CRC Covers

```
┌─────────────┬──────────┬────────┬────────┬──────────┬─────────┐
│ 0xA5        │ 0x5A     │ Type   │ Length │ Sequence │ Payload │ CRC │
│ NOT included│ NOT incl │ ✓      │ ✓      │ ✓        │ ✓       │ ✓   │
└─────────────┴──────────┴────────┴────────┴──────────┴─────────┘
              ↑ Not included                            ↑ Computed over these bytes
```

**CRC covers**: Type ID (byte 2) + Length (byte 3) + Sequence (byte 4) + Payload (bytes 5 to 5+Length-1)

### C Implementation

```c
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;  // Initialization: all bits set to 1

    for (size_t i = 0; i < length; i++) {
        // XOR with current byte (shifted left by 8 bits)
        crc ^= (uint16_t)data[i] << 8;

        // Process 8 bits for the current byte
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {  // If the most significant bit is 1
                crc = (crc << 1) ^ 0x1021;  // Shift left and XOR with polynomial
            } else {
                crc = crc << 1;  // Just shift left
            }
        }
    }

    return crc;  // Final 16-bit CRC value
}
```

### Step-by-Step Example

Given a small payload: `Type=0x01, Length=0x04, Seq=0x00, Payload=[0x12,0x34,0x56,0x78]`

1. **Initialization**: `crc = 0xFFFF`
2. **Byte 0 (Type=0x01)**:
   - `crc ^= 0x01 << 8` -> `crc = 0xFEFE`
   - Process 8 bits...
3. **Byte 1 (Length=0x04)**:
   - Continue computation...
4. **Byte 2 (Seq=0x00)**:
   - Continue computation...
5. **Bytes 3-6 (Payload)**:
   - Continue computation...
6. **Final result**: `crc = 0xXXXX` (value depends on actual data)

### CRC Verification

**During reception:**

```c
// After receiving a complete packet:
uint8_t packet[...];  // includes header + payload + CRC
uint16_t crc_received = packet[total_length-2] | (packet[total_length-1] << 8);

// Compute CRC on received data (excluding CRC bytes)
uint16_t crc_calculated = calculateCRC16(packet + 2, total_length - 4);

if (crc_received == crc_calculated) {
    // Valid packet
} else {
    // Transmission error detected
}
```

---

## Packet Type 0x01: Navigation

High-frequency packet (50 Hz) containing essential navigation state.

### Sizes

| Component | Bytes |
|---|---|
| Header | 5 |
| Payload | 104 |
| CRC | 2 |
| **Total** | **111** |

### Payload Structure (104 bytes)

The payload is organized into logical blocks:

#### Timestamp (4 bytes)

| Offset | Field | Type | Length | Unit | Notes |
|--------|-------|------|--------|------|-------|
| **0** | `timestamp_ms` | uint32 | 4 | milliseconds | System uptime, little-endian |

**Example**: Value 0x000007D0 (little-endian) = 0xD0, 0x07, 0x00, 0x00 = 2000 ms

#### Attitude (6 bytes) - **SCALED INTEGERS**

| Offset | Field | Type | Length | Scale Factor | Precision | Range | Notes |
|--------|-------|------|--------|---|---|---|---|
| **4** | `roll` | int16 | 2 | /1000 | 0.001 rad | +/-32.767 rad | Milliradian, little-endian |
| **6** | `pitch` | int16 | 2 | /1000 | 0.001 rad | +/-32.767 rad | Milliradian, little-endian |
| **8** | `yaw` | int16 | 2 | /1000 | 0.001 rad | +/-32.767 rad | Milliradian, little-endian |

**Encoding**: `int16_value = (float_radians * 1000)`
**Decoding**: `float_radians = int16_value / 1000.0`

**Roll example**:
- Actual value: 1.234 rad
- Encoded as: 1234 (0x4D2 little-endian: 0xD2, 0x04)
- Decoded: 1234 / 1000.0 = 1.234 rad

#### Position (20 bytes)

| Offset | Field | Type | Length | Unit | Notes |
|--------|-------|------|--------|------|-------|
| **10** | `latitude` | double | 8 | degrees | [-90, +90], IEEE 754 double |
| **18** | `longitude` | double | 8 | degrees | [-180, +180], IEEE 754 double |
| **26** | `altitude_msl` | float | 4 | meters | IEEE 754 single precision |

**Note**: Latitude and Longitude use double (8 bytes) for GPS precision (~10 cm).

#### Velocity NED (12 bytes)

| Offset | Field | Type | Length | Unit |
|--------|-------|------|--------|------|
| **30** | `velocity_north` | float | 4 | m/s |
| **34** | `velocity_east` | float | 4 | m/s |
| **38** | `velocity_down` | float | 4 | m/s |

NED = North-East-Down (vehicle local coordinates)

#### Wind Estimation (12 bytes)

| Offset | Field | Type | Length | Unit |
|--------|-------|------|--------|------|
| **42** | `wind_north` | float | 4 | m/s |
| **46** | `wind_east` | float | 4 | m/s |
| **50** | `wind_magnitude` | float | 4 | m/s |

#### Aerodynamic Data (12 bytes)

| Offset | Field | Type | Length | Unit | Notes |
|--------|-------|------|--------|------|-------|
| **54** | `airspeed` | float | 4 | m/s | True airspeed (TAS) |
| **58** | `groundspeed` | float | 4 | m/s | Ground speed |
| **62** | `angle_of_attack` | float | 4 | radians | AOA |
| **66** | `sideslip_angle` | float | 4 | radians | Sideslip angle |

#### Fused IMU Data (18 bytes) - **SCALED ACCELERATION**

| Offset | Field | Type | Length | Scale Factor | Precision | Range | Notes |
|--------|-------|------|--------|---|---|---|---|
| **70** | `accel_x` | int16 | 2 | /100 | 0.01 m/s^2 | +/-327.67 m/s^2 | Centimeters per s^2, little-endian |
| **72** | `accel_y` | int16 | 2 | /100 | 0.01 m/s^2 | +/-327.67 m/s^2 | Centimeters per s^2, little-endian |
| **74** | `accel_z` | int16 | 2 | /100 | 0.01 m/s^2 | +/-327.67 m/s^2 | Centimeters per s^2, little-endian |
| **76** | `gyro_x` | float | 4 | (none) | IEEE 754 | +/-inf | rad/s, body frame |
| **80** | `gyro_y` | float | 4 | (none) | IEEE 754 | +/-inf | rad/s, body frame |
| **84** | `gyro_z` | float | 4 | (none) | IEEE 754 | +/-inf | rad/s, body frame |

**Acceleration encoding**: `int16_value = (float_ms2 * 100)`
**Acceleration decoding**: `float_ms2 = int16_value / 100.0`

**Acceleration Z example**:
- Actual value: 9.81 m/s^2
- Encoded as: 981 (0x03D5 little-endian: 0xD5, 0x03)
- Decoded: 981 / 100.0 = 9.81 m/s^2

#### Filter State (8 bytes)

| Offset | Field | Type | Length | Unit | Notes |
|--------|-------|------|--------|------|-------|
| **88** | `confidence` | float | 4 | 0.0-1.0 | Filter confidence metric |
| **92** | `covariance_trace` | float | 4 | - | Covariance matrix trace |

#### GPS Quality (6 bytes)

| Offset | Field | Type | Length | Values | Notes |
|--------|-------|------|--------|--------|-------|
| **96** | `gps_fix_type` | uint8 | 1 | 0-6 | GPS fix type (see table below) |
| **97** | `gps_num_satellites` | uint8 | 1 | 0-50 | Number of satellites used |
| **98** | `gps_hdop` | float | 4 | - | Horizontal Dilution of Precision |

**GPS Fix Type Values**:
```
0: No fix
1: Dead reckoning only
2: 2D fix
3: 3D fix
4: GNSS + Dead reckoning
5: RTK fixed
6: RTK float
```

#### Status Flags (2 bytes) - **BITFIELD**

| Offset | Field | Type | Length | Description |
|--------|-------|------|--------|-------------|
| **102** | `status_flags` | uint16 | 2 | Bitfield with 9 flags + 1 two-bit field |

**Bit Structure**:

```
Bit 15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
        │───────────────│ │───────│ │───────────────────────────│
        Reserved (5 bit) Aiding  GPS Mag Baro IMU2 IMU1 GPS ZUPT Init Conv
                Mode(2)  Bypass  (8 individual status bits)
```

| Bit | Mask | Name | Description |
|-----|------|------|-------------|
| **0** | `0x0001` | CONVERGED | 1 = Filter converged, 0 = Converging |
| **1** | `0x0002` | INITIALIZED | 1 = Initialized, 0 = Not initialized |
| **2** | `0x0004` | ZUPT_ACTIVE | 1 = Zero velocity detected (stationary), 0 = Moving |
| **3** | `0x0008` | GPS_AVAILABLE | 1 = GPS available, 0 = GPS denied |
| **4** | `0x0010` | IMU1_HEALTHY | 1 = Healthy, 0 = Faulty |
| **5** | `0x0020` | IMU2_HEALTHY | 1 = Healthy, 0 = Faulty |
| **6** | `0x0040` | BARO_HEALTHY | 1 = Healthy, 0 = Faulty |
| **7** | `0x0080` | MAG_HEALTHY | 1 = Healthy, 0 = Faulty |
| **8** | `0x0100` | GPS_BYPASS | 1 = GPS bypass mode active, 0 = Normal |
| **9-10** | `0x0600` | AIDING_MODE | Aiding mode (2 bits, see table below) |
| **11-15** | `0xF800` | RESERVED | Reserved for future use |

**Aiding Mode Values (Bit 9-10)**:
```
00 (0): GPS + Magnetometer
01 (1): GPS only
10 (2): Magnetometer only
11 (3): Dead Reckoning only
```

**Status Flags Creation Example**:
```c
uint16_t status_flags = 0;

// Individually:
status_flags |= (1 << 0);              // CONVERGED = 1
status_flags |= (1 << 1);              // INITIALIZED = 1
status_flags |= (0 << 2);              // ZUPT_ACTIVE = 0
status_flags |= (1 << 3);              // GPS_AVAILABLE = 1
status_flags |= (1 << 4);              // IMU1_HEALTHY = 1
status_flags |= (1 << 5);              // IMU2_HEALTHY = 1
status_flags |= (1 << 6);              // BARO_HEALTHY = 1
status_flags |= (0 << 7);              // MAG_HEALTHY = 0
status_flags |= (0 << 8);              // GPS_BYPASS = 0
status_flags |= (0 << 9) | (0 << 10);  // AIDING_MODE = 00 (GPS+Mag)

// Result: 0x0077 (119 in decimal)
```

---

## Packet Type 0x02: Debug

Low-frequency packet (1 Hz) containing diagnostics, biases, and performance metrics.

### Sizes

| Component | Bytes |
|---|---|
| Header | 5 |
| Payload | 64 |
| CRC | 2 |
| **Total** | **71** |

### Payload Structure (64 bytes)

#### Timestamp (4 bytes)

| Offset | Field | Type | Length | Unit |
|--------|-------|------|--------|------|
| **0** | `timestamp_ms` | uint32 | 4 | milliseconds |

#### Gyroscope Biases (12 bytes)

| Offset | Field | Type | Length | Unit | Range |
|--------|-------|------|--------|------|-------|
| **4** | `gyro_bias_x` | float | 4 | rad/s | +/-inf |
| **8** | `gyro_bias_y` | float | 4 | rad/s | +/-inf |
| **12** | `gyro_bias_z` | float | 4 | rad/s | +/-inf |

Current estimates of gyroscope offset biases, converge during filtering.

#### Accelerometer Biases (12 bytes)

| Offset | Field | Type | Length | Unit | Range |
|--------|-------|------|--------|------|-------|
| **16** | `accel_bias_x` | float | 4 | m/s^2 | +/-inf |
| **20** | `accel_bias_y` | float | 4 | m/s^2 | +/-inf |
| **24** | `accel_bias_z` | float | 4 | m/s^2 | +/-inf |

Current estimates of accelerometer biases.

#### Barometer Bias (4 bytes)

| Offset | Field | Type | Length | Unit |
|--------|-------|------|--------|------|
| **28** | `baro_bias` | float | 4 | meters |

Barometric altimeter bias estimate.

#### Magnetometer Quality (4 bytes)

| Offset | Field | Type | Length | Unit | Range |
|--------|-------|------|--------|------|-------|
| **32** | `mag_quality` | float | 4 | 0.0-1.0 | [0, 1] |

Magnetometer reading quality metric (0 = poor, 1 = excellent).

#### Performance Metrics (8 bytes)

| Offset | Field | Type | Length | Unit | Notes |
|--------|-------|------|--------|------|-------|
| **36** | `loop_time_us` | uint16 | 2 | microseconds | Average loop time |
| **38** | `filter_time_us` | uint16 | 2 | microseconds | EKF computation time |
| **40** | `sensor_time_us` | uint16 | 2 | microseconds | Sensor read time |
| **42** | `max_loop_time_us` | uint16 | 2 | microseconds | Max time since previous debug packet |

All times are measured in **microseconds (us)**.

#### Extended GPS Information (8 bytes)

| Offset | Field | Type | Length | Unit |
|--------|-------|------|--------|------|
| **44** | `gps_horizontal_accuracy` | float | 4 | meters |
| **48** | `gps_vertical_accuracy` | float | 4 | meters |

Accuracy estimates provided by the GPS receiver.

#### Sensor Counts (4 bytes)

| Offset | Field | Type | Length | Values | Description |
|--------|-------|------|--------|--------|-------------|
| **52** | `baro_sensor_count` | uint8 | 1 | 0-2 | Number of active barometers |
| **53** | `gps_quality_indicator` | uint8 | 1 | 0-255 | GPS quality indicator (receiver specific) |
| **54** | `imu_fused_status` | uint8 | 1 | 0 or 1 | 0=single IMU, 1=dual IMU |
| **55** | `reserved` | uint8 | 1 | - | Reserved (alignment padding) |

#### GPS Time (8 bytes) - UTC from GPS Receiver

| Offset | Field | Type | Length | Values | Unit |
|--------|-------|------|--------|--------|------|
| **56** | `gps_year` | uint16 | 2 | 1900-2100 | Years |
| **58** | `gps_month` | uint8 | 1 | 1-12 | Months |
| **59** | `gps_day` | uint8 | 1 | 1-31 | Days |
| **60** | `gps_hour` | uint8 | 1 | 0-23 | Hours (UTC) |
| **61** | `gps_minute` | uint8 | 1 | 0-59 | Minutes |
| **62** | `gps_second` | uint8 | 1 | 0-59 | Seconds |
| **63** | `gps_time_valid` | uint8 | 1 | 0 or 1 | 1=valid, 0=invalid |

---

## Encoding and Decoding

### Transmitter Side (Teensy 4.0)

#### 1. Creating a Navigation Packet

```c
#include "telemetry_binary.h"

// Encoder instance
TelemetryBinaryEncoder encoder;

// Data to transmit (from EKF, sensors, etc.)
NavigationPacket nav_packet;
nav_packet.timestamp_ms = 2000;
nav_packet.roll = (int16_t)(1.234 * 1000);      // 1234 (scaled)
nav_packet.pitch = (int16_t)(-0.080 * 1000);    // -80 (scaled)
nav_packet.yaw = (int16_t)(3.129 * 1000);       // 3129 (scaled)
nav_packet.latitude = 45.123456;
nav_packet.longitude = -122.654321;
nav_packet.altitude_msl = 100.5;
// ... other fields ...
nav_packet.accel_z = (int16_t)(9.81 * 100);     // 981 (scaled)
nav_packet.status_flags = 0x0077;
// ... etc ...

// Encode into buffer
uint8_t buffer[256];
size_t packet_size = encoder.encodeNavigationPacket(&nav_packet, buffer, sizeof(buffer));

// Transmit
Serial1.write(buffer, packet_size);  // 460,800 baud UART
```

#### 2. Creating a Debug Packet

```c
DebugPacket debug_packet;
debug_packet.timestamp_ms = 2000;
debug_packet.gyro_bias_x = 0.001;   // rad/s
debug_packet.gyro_bias_y = -0.0005;
debug_packet.gyro_bias_z = 0.0008;
debug_packet.accel_bias_x = 0.05;   // m/s^2
debug_packet.accel_bias_y = -0.03;
debug_packet.accel_bias_z = 0.02;
debug_packet.baro_bias = 2.5;       // meters
debug_packet.mag_quality = 0.95;
debug_packet.loop_time_us = 1250;   // 1250 us = 1.25 ms per cycle
debug_packet.filter_time_us = 850;  // EKF time
debug_packet.sensor_time_us = 200;  // Sensor read time
debug_packet.max_loop_time_us = 1500;
// ... other fields ...

// Encode
uint8_t buffer[256];
size_t packet_size = encoder.encodeDebugPacket(&debug_packet, buffer, sizeof(buffer));

// Transmit
Serial1.write(buffer, packet_size);
```

### Receiver Side (PC/Ground Station)

#### 1. Synchronization and Parsing

```python
import struct

class BinaryTelemetryDecoder:
    def __init__(self):
        self.buffer = []
        self.sync_state = 0

    def feed_bytes(self, data_bytes):
        """Feed raw bytes from serial"""
        for byte in data_bytes:
            self.parse_byte(byte)

    def parse_byte(self, byte):
        """Process a single byte"""
        if self.sync_state == 0:
            # Search for first sync byte
            if byte == 0xA5:
                self.sync_state = 1
                self.buffer = [byte]
        elif self.sync_state == 1:
            # Search for second sync byte
            if byte == 0x5A:
                self.sync_state = 2
                self.buffer.append(byte)
            else:
                # False alarm, search for new 0xA5
                self.sync_state = 1 if byte == 0xA5 else 0
                self.buffer = [byte] if byte == 0xA5 else []
        elif self.sync_state == 2:
            # Sync acquired, read header
            if len(self.buffer) < 5:
                self.buffer.append(byte)
                if len(self.buffer) == 5:
                    self.sync_state = 3  # Ready for payload
            else:
                # Read payload
                self.buffer.append(byte)
                # Calculate expected total length
                payload_length = self.buffer[3]
                crc_offset = 5 + payload_length
                if len(self.buffer) == crc_offset + 2:
                    # Complete packet!
                    self.process_packet()
                    self.sync_state = 0
                    self.buffer = []

    def process_packet(self):
        """Process the complete packet"""
        # Verify CRC
        packet_type = self.buffer[2]
        payload_length = self.buffer[3]

        # CRC computed over bytes 2..5+payload_length-1
        crc_data = self.buffer[2:5+payload_length]
        calculated_crc = self.calculate_crc16(crc_data)

        # Received CRC (little-endian)
        crc_received = self.buffer[5+payload_length] | (self.buffer[5+payload_length+1] << 8)

        if calculated_crc != crc_received:
            print(f"CRC Error! Got {crc_received:04X}, expected {calculated_crc:04X}")
            return

        # Parse based on type
        if packet_type == 0x01:
            self.parse_navigation_packet()
        elif packet_type == 0x02:
            self.parse_debug_packet()

    def calculate_crc16(self, data):
        """CRC-16-CCITT implementation"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def parse_navigation_packet(self):
        """Decode Navigation Packet (0x01)"""
        payload = self.buffer[5:5+104]

        # Unpack using struct (little-endian: '<')
        timestamp_ms = struct.unpack('<I', payload[0:4])[0]

        # Attitude (scaled int16 / 1000)
        roll = struct.unpack('<h', payload[4:6])[0] / 1000.0
        pitch = struct.unpack('<h', payload[6:8])[0] / 1000.0
        yaw = struct.unpack('<h', payload[8:10])[0] / 1000.0

        # Position
        latitude = struct.unpack('<d', payload[10:18])[0]
        longitude = struct.unpack('<d', payload[18:26])[0]
        altitude_msl = struct.unpack('<f', payload[26:30])[0]

        # Velocity NED
        velocity_north = struct.unpack('<f', payload[30:34])[0]
        velocity_east = struct.unpack('<f', payload[34:38])[0]
        velocity_down = struct.unpack('<f', payload[38:42])[0]

        # Wind
        wind_north = struct.unpack('<f', payload[42:46])[0]
        wind_east = struct.unpack('<f', payload[46:50])[0]
        wind_magnitude = struct.unpack('<f', payload[50:54])[0]

        # Air Data
        airspeed = struct.unpack('<f', payload[54:58])[0]
        groundspeed = struct.unpack('<f', payload[58:62])[0]
        angle_of_attack = struct.unpack('<f', payload[62:66])[0]
        sideslip_angle = struct.unpack('<f', payload[66:70])[0]

        # IMU (accel scaled int16 / 100)
        accel_x = struct.unpack('<h', payload[70:72])[0] / 100.0
        accel_y = struct.unpack('<h', payload[72:74])[0] / 100.0
        accel_z = struct.unpack('<h', payload[74:76])[0] / 100.0
        gyro_x = struct.unpack('<f', payload[76:80])[0]
        gyro_y = struct.unpack('<f', payload[80:84])[0]
        gyro_z = struct.unpack('<f', payload[84:88])[0]

        # Filter Status
        confidence = struct.unpack('<f', payload[88:92])[0]
        covariance_trace = struct.unpack('<f', payload[92:96])[0]

        # GPS Quality
        gps_fix_type = payload[96]
        gps_num_satellites = payload[97]
        gps_hdop = struct.unpack('<f', payload[98:102])[0]

        # Status Flags
        status_flags = struct.unpack('<H', payload[102:104])[0]

        # Print results
        print(f"Time: {timestamp_ms}ms")
        print(f"Attitude: R={roll:.3f}, P={pitch:.3f}, Y={yaw:.3f} rad")
        print(f"Position: Lat={latitude:.8f}, Lon={longitude:.8f}, Alt={altitude_msl:.1f}m")
        print(f"Velocity: V_N={velocity_north:.2f}, V_E={velocity_east:.2f}, V_D={velocity_down:.2f} m/s")
        print(f"Accel: {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f} m/s^2")
        print(f"Status: 0x{status_flags:04X}")
        return {
            'timestamp_ms': timestamp_ms,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'latitude': latitude,
            'longitude': longitude,
            'altitude_msl': altitude_msl,
            'velocity_north': velocity_north,
            'velocity_east': velocity_east,
            'velocity_down': velocity_down,
            'accel_x': accel_x,
            'accel_y': accel_y,
            'accel_z': accel_z,
            'gyro_x': gyro_x,
            'gyro_y': gyro_y,
            'gyro_z': gyro_z,
            'status_flags': status_flags,
        }

    def parse_debug_packet(self):
        """Decode Debug Packet (0x02)"""
        payload = self.buffer[5:5+64]

        timestamp_ms = struct.unpack('<I', payload[0:4])[0]

        # Biases
        gyro_bias_x = struct.unpack('<f', payload[4:8])[0]
        gyro_bias_y = struct.unpack('<f', payload[8:12])[0]
        gyro_bias_z = struct.unpack('<f', payload[12:16])[0]

        accel_bias_x = struct.unpack('<f', payload[16:20])[0]
        accel_bias_y = struct.unpack('<f', payload[20:24])[0]
        accel_bias_z = struct.unpack('<f', payload[24:28])[0]

        baro_bias = struct.unpack('<f', payload[28:32])[0]
        mag_quality = struct.unpack('<f', payload[32:36])[0]

        # Timing
        loop_time_us = struct.unpack('<H', payload[36:38])[0]
        filter_time_us = struct.unpack('<H', payload[38:40])[0]
        sensor_time_us = struct.unpack('<H', payload[40:42])[0]
        max_loop_time_us = struct.unpack('<H', payload[42:44])[0]

        # GPS Accuracy
        gps_hacc = struct.unpack('<f', payload[44:48])[0]
        gps_vacc = struct.unpack('<f', payload[48:52])[0]

        # Sensor Counts
        baro_count = payload[52]
        gps_quality = payload[53]
        imu_fused = payload[54]

        # GPS Time
        gps_year = struct.unpack('<H', payload[56:58])[0]
        gps_month = payload[58]
        gps_day = payload[59]
        gps_hour = payload[60]
        gps_minute = payload[61]
        gps_second = payload[62]
        gps_time_valid = payload[63]

        print(f"Debug - Time: {timestamp_ms}ms")
        print(f"Gyro Bias: X={gyro_bias_x:.6f}, Y={gyro_bias_y:.6f}, Z={gyro_bias_z:.6f} rad/s")
        print(f"Accel Bias: X={accel_bias_x:.4f}, Y={accel_bias_y:.4f}, Z={accel_bias_z:.4f} m/s^2")
        print(f"Performance: Loop={loop_time_us}us, Filter={filter_time_us}us, Sensor={sensor_time_us}us")
```

---

## Configuration

### Enablement in config.h

```c
// Telemetry format
#define TELEMETRY_FORMAT_BINARY true        // true = binary, false = JSON
#define BINARY_NAV_RATE_HZ 50               // Navigation packet frequency (Hz)
#define BINARY_DEBUG_RATE_HZ 1              // Debug packet frequency (Hz)

// Outputs
#define TELEMETRY_ENABLE_SERIAL1 true       // Output on Serial1 (460,800 baud)
#define TELEMETRY_ENABLE_USB true          // Output on USB (115,200 baud)
```

### Effective Bandwidth Calculation

**Default (50 Hz Navigation, 1 Hz Debug)**:
```
Navigation: 111 bytes x 50 Hz = 5,550 bytes/s = 44.4 kbps
Debug:      71 bytes x 1 Hz = 71 bytes/s = 0.568 kbps
Total:      ~44.968 kbps ~ 45 kbps

Link utilization @ 460,800 baud (57,600 bytes/s):
Percentage: 45 kbps / 460.8 kbps = 9.7%
Margin: 90.3% available
```

**High-Rate Configuration (100 Hz Navigation, 5 Hz Debug)**:
```
Navigation: 111 bytes x 100 Hz = 11,100 bytes/s = 88.8 kbps
Debug:      71 bytes x 5 Hz = 355 bytes/s = 2.84 kbps
Total:      ~91.6 kbps

Utilization: 91.6 / 460.8 = 19.9% (still plenty of margin)
```

---

## Practical Examples

### Example 1: Manually Decoding a Navigation Packet

**Received hex dump**:
```
A5 5A 01 68 2A
D0 07 00 00          # timestamp_ms = 0x000007D0 = 2000 ms
D2 04 B0 FF 39 0C    # roll, pitch, yaw (scaled)
40 F0 1F 40 45 8C 80 3F  # latitude = 40.1234 deg (double)
60 66 D6 BF D6 9B 47 C2  # longitude = -122.6789 deg (double)
00 00 48 42          # altitude_msl = 50.0 m (float)
...rest of payload...
XX XX                # CRC-16
```

**Decoding**:
```
Byte 0-3:  D0 07 00 00 (little-endian) = 0x000007D0 = 2000 ms
Byte 4-5:  D2 04 (little-endian) = 0x04D2 = 1234 -> 1234/1000 = 1.234 rad
Byte 6-7:  B0 FF (little-endian) = 0xFFB0 = -80 (signed) -> -80/1000 = -0.080 rad
Byte 8-9:  39 0C (little-endian) = 0x0C39 = 3129 -> 3129/1000 = 3.129 rad
```

### Example 2: Creating Status Flags

**Scenario**: Filter converged, initialized, GPS available, dual IMU healthy, barometer healthy, GPS+Mag mode

```c
uint16_t flags = 0;
flags |= (1 << 0);   // CONVERGED
flags |= (1 << 1);   // INITIALIZED
flags |= (1 << 3);   // GPS_AVAILABLE
flags |= (1 << 4);   // IMU1_HEALTHY
flags |= (1 << 5);   // IMU2_HEALTHY
flags |= (1 << 6);   // BARO_HEALTHY
flags |= (0 << 9) | (0 << 10);  // AIDING_MODE = 00 (GPS+Mag)

// Result: 0x007B (00000000 01111011)
//
// Bit 15-11: 00000 (reserved)
// Bit 10-9:  00 (AIDING_MODE = GPS+Mag)
// Bit 8:     0 (GPS_BYPASS = no)
// Bit 7:     0 (MAG_HEALTHY = ... not specified, assumed 0)
// Bit 6:     1 (BARO_HEALTHY = yes)
// Bit 5:     1 (IMU2_HEALTHY = yes)
// Bit 4:     1 (IMU1_HEALTHY = yes)
// Bit 3:     1 (GPS_AVAILABLE = yes)
// Bit 2:     0 (ZUPT_ACTIVE = no)
// Bit 1:     1 (INITIALIZED = yes)
// Bit 0:     1 (CONVERGED = yes)
```

### Example 3: CRC Calculation for a Small Payload

**Data**: `Type=0x01, Length=0x04, Seq=0x00, Payload=[0x12, 0x34, 0x56, 0x78]`

**Step-by-step calculation** (C function):
```c
uint8_t crc_data[] = {0x01, 0x04, 0x00, 0x12, 0x34, 0x56, 0x78};
uint16_t crc = 0xFFFF;

// Byte 0: 0x01
crc ^= 0x01 << 8;  // crc = 0xFEFF
// 8 bits of processing...
crc = 0x... (after 8 bits)

// Byte 1: 0x04
// ... continue ...

// Result: crc = 0x... (value depends on implementation)
```

To quickly verify, use online tools such as [CRC Calculator](https://crccalc.com/) with:
- Polynomial: 0x1021
- Initial: 0xFFFF
- Input Reflected: No
- Output Reflected: No

---

## Size Summary

| Element | Bytes |
|---------|-------|
| **Header** | 5 |
| &nbsp;&nbsp;Sync 1 | 1 |
| &nbsp;&nbsp;Sync 2 | 1 |
| &nbsp;&nbsp;Type | 1 |
| &nbsp;&nbsp;Length | 1 |
| &nbsp;&nbsp;Sequence | 1 |
| **Navigation Payload** | 104 |
| &nbsp;&nbsp;Timestamp | 4 |
| &nbsp;&nbsp;Attitude (scaled) | 6 |
| &nbsp;&nbsp;Position | 20 |
| &nbsp;&nbsp;Velocity | 12 |
| &nbsp;&nbsp;Wind | 12 |
| &nbsp;&nbsp;Air Data | 12 |
| &nbsp;&nbsp;IMU (accel scaled) | 18 |
| &nbsp;&nbsp;Filter Status | 8 |
| &nbsp;&nbsp;GPS Quality | 6 |
| &nbsp;&nbsp;Status Flags | 2 |
| **Debug Payload** | 64 |
| **CRC** | 2 |
| **TOTAL Navigation Packet** | 111 |
| **TOTAL Debug Packet** | 71 |

---

## Conclusion

This binary telemetry protocol provides:

- **Efficiency**: 77.5% less bandwidth than JSON
- **Reliability**: CRC-16 for error detection
- **Precision**: Integer scaling for attitude and acceleration
- **Clarity**: Well-defined and documented structures
- **Compatibility**: Easy to implement on any platform

For questions or issues, refer to the implementation in:
- [telemetry_protocol.h](../src/telemetry_protocol.h)
- [telemetry_binary.h/cpp](../src/telemetry_binary.h)
