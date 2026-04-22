# Protocollo Telemetria Binaria CORV-2 — Specifica Completa

Versione protocollo: **7** (stesso valore di `SYS_CONFIG_VERSION`).
Sorgente di verità: [src/telemetry_protocol.h](../src/telemetry_protocol.h), [src/system_config.h](../src/system_config.h), [src/system_config.cpp](../src/system_config.cpp).

## Indice
1. [Panoramica](#panoramica)
2. [Struttura generica del pacchetto](#struttura-generica-del-pacchetto)
3. [CRC-16-CCITT](#crc-16-ccitt)
4. [0x01 — Navigation](#0x01--navigation-device--gs)
5. [0x02 — Debug](#0x02--debug-device--gs)
6. [0x03 — Raw Sensor](#0x03--raw-sensor-device--gs)
7. [0x10 — Config Write (GS → Device, solo USB)](#0x10--config-write-gs--device-solo-usb)
8. [0x11 — Config Response (Device → GS)](#0x11--config-response-device--gs)
9. [SystemConfig — layout e default](#systemconfig--layout-e-default)
10. [EEPROM layout e validazione](#eeprom-layout-e-validazione)
11. [Sequenze tipiche GS → Device](#sequenze-tipiche-gs--device)

---

## Panoramica

| Tipo | ID | Direzione | Payload | Totale | Rate default | Note |
|------|----|-----------|---------|--------|--------------|------|
| Navigation | `0x01` | Device → GS | 104 B | 111 B | 50 Hz (`nav_rate_hz`) | USB + Serial1 |
| Debug | `0x02` | Device → GS | 92 B | 99 B | 1 Hz (`debug_rate_hz`) | USB + Serial1 |
| Raw Sensor | `0x03` | Device → GS | 74 B | 81 B | 5 Hz (`raw_rate_hz`) | USB + Serial1 |
| Config Write | `0x10` | **GS → Device** | 1 + N B | 8 + N B | on demand | **solo USB** |
| Config Response | `0x11` | Device → GS | 2 + N B | 9 + N B | per ogni 0x10 | **solo USB** |

I pacchetti di configurazione (0x10/0x11) sono accettati **solo** sulla porta USB Serial per sicurezza operativa; vengono ignorati su Serial1 (telemetria RF).

---

## Struttura generica del pacchetto

```
┌────────┬────────┬──────┬────────┬─────┬──────────────┬─────────┐
│ 0xA5   │ 0x5A   │ Type │ Length │ Seq │ Payload      │ CRC16   │
│ 1 B    │ 1 B    │ 1 B  │ 1 B    │ 1 B │ Length bytes │ 2 B LE  │
└────────┴────────┴──────┴────────┴─────┴──────────────┴─────────┘
```

- `Length` = dimensione del **solo** payload in byte (escluso header e CRC).
- `Seq` = contatore 0–255 che wrappa (uno per ciascun tipo: nav, debug, raw, config-response).
- Tutti i campi multi-byte sono **little-endian**.
- Float = IEEE 754 single (32-bit); double = IEEE 754 double (64-bit).

---

## CRC-16-CCITT

Parametri: polinomio `0x1021`, init `0xFFFF`, **senza** reflection in ingresso/uscita, **senza** XOR finale. Identico per telemetria e per il CRC della struct in EEPROM.

```c
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
```

**Copertura**: il CRC è calcolato su `Type | Length | Seq | Payload` (i due byte di sync NON sono inclusi).
**Posizione**: il CRC viene scritto come 2 byte little-endian subito dopo il payload (`buf[5 + Length] = crc & 0xFF; buf[5 + Length + 1] = crc >> 8`).

---

## 0x01 — Navigation (Device → GS)

Payload 104 byte (`struct NavigationPacket` packed). Offset assoluti nel payload:

| Offset | Campo | Tipo | Scala | Unità | Note |
|-------:|-------|------|------:|-------|------|
| 0 | `timestamp_ms` | uint32 | — | ms | Uptime |
| 4 | `roll` | int16 | ÷1000 | rad | 0.001 rad, ±32.767 rad |
| 6 | `pitch` | int16 | ÷1000 | rad | |
| 8 | `yaw` | int16 | ÷1000 | rad | |
| 10 | `latitude` | double | — | deg | |
| 18 | `longitude` | double | — | deg | |
| 26 | `altitude_msl` | float | — | m | |
| 30 | `velocity_north` | float | — | m/s | NED |
| 34 | `velocity_east` | float | — | m/s | |
| 38 | `velocity_down` | float | — | m/s | |
| 42 | `wind_north` | float | — | m/s | |
| 46 | `wind_east` | float | — | m/s | |
| 50 | `wind_magnitude` | float | — | m/s | |
| 54 | `airspeed` | float | — | m/s | TAS |
| 58 | `groundspeed` | float | — | m/s | |
| 62 | `angle_of_attack` | float | — | rad | |
| 66 | `sideslip_angle` | float | — | rad | |
| 70 | `accel_x` | int16 | ÷100 | m/s² | 0.01 m/s², ±327.67 |
| 72 | `accel_y` | int16 | ÷100 | m/s² | |
| 74 | `accel_z` | int16 | ÷100 | m/s² | |
| 76 | `gyro_x` | float | — | rad/s | body |
| 80 | `gyro_y` | float | — | rad/s | |
| 84 | `gyro_z` | float | — | rad/s | |
| 88 | `confidence` | float | — | 0–1 | |
| 92 | `covariance_trace` | float | — | — | tr(P) |
| 96 | `gps_fix_type` | uint8 | — | — | 0–6 |
| 97 | `gps_num_satellites` | uint8 | — | — | |
| 98 | `gps_hdop` | float | — | — | |
| 102 | `status_flags` | uint16 | — | bitfield | vedi sotto |

### GPS fix type

`0` nessun fix · `1` DR · `2` 2D · `3` 3D · `4` GNSS+DR · `5` RTK fixed · `6` RTK float.

### Status flags (uint16, bit 0 = LSB)

| Bit | Costante | Significato |
|-----|----------|-------------|
| 0 | `STATUS_FLAG_CONVERGED` | Filtro convergito |
| 1 | `STATUS_FLAG_INITIALIZED` | Filtro inizializzato |
| 2 | `STATUS_FLAG_ZUPT_ACTIVE` | ZUPT attivo |
| 3 | `STATUS_FLAG_GPS_AVAILABLE` | GPS aiding disponibile |
| 4 | `STATUS_FLAG_IMU1_HEALTHY` | IMU1 ok |
| 5 | `STATUS_FLAG_IMU2_HEALTHY` | IMU2 ok |
| 6 | `STATUS_FLAG_BARO_HEALTHY` | Baro ok |
| 7 | `STATUS_FLAG_MAG_HEALTHY` | Mag ok |
| 8 | `STATUS_FLAG_GPS_BYPASS` | Bypass GPS attivo |
| 9–10 | `STATUS_FLAG_AIDING_MODE_MASK` | Modalità aiding (00=GPS+Mag, 01=GPS, 10=Mag, 11=DR) |
| 11 | `STATUS_FLAG_GPS_JAMMING` | Jamming rilevato |
| 12 | `STATUS_FLAG_GPS_SPOOFING` | Spoofing sospetto |
| 13–15 | — | Riservati |

---

## 0x02 — Debug (Device → GS)

Payload 92 byte (`struct DebugPacket` packed). Include diagnostica PF e IAS.

| Offset | Campo | Tipo | Unità / Note |
|-------:|-------|------|--------------|
| 0 | `timestamp_ms` | uint32 | ms |
| 4 | `gyro_bias_x` | float | deg/s |
| 8 | `gyro_bias_y` | float | deg/s |
| 12 | `gyro_bias_z` | float | deg/s |
| 16 | `accel_bias_x` | float | m/s² |
| 20 | `accel_bias_y` | float | m/s² |
| 24 | `accel_bias_z` | float | m/s² |
| 28 | `baro_bias` | float | m |
| 32 | `mag_quality` | float | 0–1 |
| 36 | `hard_iron_x` | float | nT |
| 40 | `hard_iron_y` | float | nT |
| 44 | `hard_iron_z` | float | nT |
| 48 | `loop_time_us` | uint16 | µs (loop medio) |
| 50 | `filter_time_us` | uint16 | µs (EKF) |
| 52 | `sensor_time_us` | uint16 | µs (lettura sensori) |
| 54 | `max_loop_time_us` | uint16 | µs (max dal precedente debug) |
| 56 | `gps_horizontal_accuracy` | float | m |
| 60 | `gps_vertical_accuracy` | float | m |
| 64 | `baro_sensor_count` | uint8 | 0–2 |
| 65 | `gps_quality_indicator` | uint8 | ricevitore-specifico |
| 66 | `imu_fused_status` | uint8 | 0=singolo, 1=dual |
| 67 | `reserved` | uint8 | padding |
| 68 | `gps_year` | uint16 | |
| 70 | `gps_month` | uint8 | 1–12 |
| 71 | `gps_day` | uint8 | 1–31 |
| 72 | `gps_hour` | uint8 | 0–23 UTC |
| 73 | `gps_minute` | uint8 | 0–59 |
| 74 | `gps_second` | uint8 | 0–59 |
| 75 | `gps_time_valid` | uint8 | 0/1 |
| 76 | `pf_ess` | float | effective sample size |
| 80 | `pf_position_spread` | float | m (dispersione posizione particle cloud) |
| 84 | `pf_resample_count` | uint16 | conteggio resample (wrap 65535) |
| 86 | `pf_active_particles` | uint16 | N particle attive |
| 88 | `ias` | float | m/s (indicated airspeed, solo MINI con sensore SPI) |

---

## 0x03 — Raw Sensor (Device → GS)

Payload 74 byte (`struct RawSensorPacket` packed). Dati sensori grezzi pre-filtro.

| Offset | Campo | Tipo | Unità |
|-------:|-------|------|-------|
| 0 | `timestamp_ms` | uint32 | ms |
| 4 | `gps_latitude` | double | deg |
| 12 | `gps_longitude` | double | deg |
| 20 | `gps_altitude_msl` | float | m |
| 24 | `gps_vel_north` | float | m/s |
| 28 | `gps_vel_east` | float | m/s |
| 32 | `gps_vel_down` | float | m/s |
| 36 | `gps_fix_type` | uint8 | — |
| 37 | `gps_num_sats` | uint8 | — |
| 38 | `gps_h_accuracy` | float | m |
| 42 | `mag_x` | float | nT (body, post-remap) |
| 46 | `mag_y` | float | nT |
| 50 | `mag_z` | float | nT |
| 54 | `baro_altitude` | float | m |
| 58 | `baro_pressure` | float | Pa |
| 62 | `imu_accel_z` | float | m/s² |
| 66 | `imu_gyro_x` | float | rad/s |
| 70 | `imu_gyro_y` | float | rad/s |

Con `gps_sim_noise = 1` i campi GPS includono il rumore simulato.

---

## 0x10 — Config Write (GS → Device, solo USB)

Payload: `command_id (1 B) + command_data (N B)`.

| `command_id` | Nome | Data | Descrizione |
|--------------|------|------|-------------|
| `0x01` | `CONFIG_CMD_SET_CONFIG` | SystemConfig (106 B) | Scrive la config **runtime** (non-persistente fino a SAVE) |
| `0x02` | `CONFIG_CMD_GET_CONFIG` | vuoto | Richiede la config runtime corrente |
| `0x03` | `CONFIG_CMD_SAVE_CONFIG` | vuoto | Salva la config runtime in EEPROM |
| `0x04` | `CONFIG_CMD_RESET_DEFAULT` | vuoto | Ripristina default (preserva `board_type`) e salva |
| `0x05` | `CONFIG_CMD_REBOOT` | vuoto | Riavvio software (`SCB_AIRCR = 0x05FA0004`) |

### Esempi frame completi

GET_CONFIG (length = 1):
```
A5 5A 10 01 <seq> 02 <CRC_L CRC_H>
```

SET_CONFIG (length = 1 + 106 = 107 → 0x6B):
```
A5 5A 10 6B <seq> 01 <106 byte SystemConfig little-endian> <CRC_L CRC_H>
```

SAVE_CONFIG / RESET_DEFAULT / REBOOT (length = 1):
```
A5 5A 10 01 <seq> 03|04|05 <CRC_L CRC_H>
```

Note:
- `SET_CONFIG` applica i valori solo dopo validazione range; in caso di rifiuto, la config runtime **non** viene modificata e la risposta include il nome del primo campo rifiutato.
- `SET_CONFIG` **non salva** automaticamente: per persistere serve un successivo `SAVE_CONFIG`.
- `RESET_DEFAULT` preserva `board_type` (e forza airspeed SPI su MINI) prima di salvare.
- `REBOOT` invia la response OK, fa `Serial.flush()` + `delay(100 ms)`, poi resetta.

---

## 0x11 — Config Response (Device → GS)

Payload: `response_code (1 B) + command_id (1 B, echo) + data (N B)`.

| `response_code` | Nome | Significato |
|-----------------|------|-------------|
| `0x00` | `CONFIG_RESP_OK` | Eseguito |
| `0x01` | `CONFIG_RESP_ERROR` | Errore generico (I/O EEPROM, cmd sconosciuto, payload troppo corto) |
| `0x02` | `CONFIG_RESP_CRC_FAIL` | CRC del frame 0x10 non valido — `command_id` echo = `0x00` |
| `0x03` | `CONFIG_RESP_INVALID` | Validazione fallita; `data` contiene il nome ASCII del campo rifiutato (NON null-terminated, usa Length per calcolare la lunghezza stringa) |

### Data per comando

| Comando | Risposta tipica | Data |
|---------|-----------------|------|
| SET_CONFIG | OK / INVALID | vuoto / nome campo (es. `"pf_ess_threshold"`) |
| GET_CONFIG | OK | SystemConfig (106 B) |
| SAVE_CONFIG | OK / ERROR | vuoto |
| RESET_DEFAULT | OK | vuoto |
| REBOOT | OK | vuoto (inviato prima del reset) |

### Esempi

Risposta a GET_CONFIG (length = 2 + 106 = 108 → 0x6C):
```
A5 5A 11 6C <seq> 00 02 <106 byte SystemConfig> <CRC_L CRC_H>
```

Risposta a SET_CONFIG con errore di validazione su `pf_gps_sigma_h`:
```
A5 5A 11 <2+15=0x11> <seq> 03 01 'p' 'f' '_' 'g' 'p' 's' '_' 's' 'i' 'g' 'm' 'a' '_' 'h' <CRC_L CRC_H>
```

---

## SystemConfig — layout e default

Struct **packed**, 106 byte totali. Offset esatti e default come caricati da `loadDefaultConfig()`.

### GPS (5 B)

| Off | Campo | Tipo | Default | Range / valori |
|----:|-------|------|---------|----------------|
| 0 | `gps_type` | uint8 | `0` (UBlox) | `0`=UBlox, `1`=Mosaic |
| 1 | `gps_baud_rate` | uint32 | `460800` | `115200 / 230400 / 460800 / 921600` |

### Telemetry output (9 B)

| Off | Campo | Tipo | Default | Range |
|----:|-------|------|---------|-------|
| 5 | `serial1_baud` | uint32 | `921600` | `115200 / 230400 / 460800 / 921600` |
| 9 | `output_protocol` | uint8 | `1` (VectorNav) | `0`=Binary Custom, `1`=VectorNav |
| 10 | `telemetry_usb` | uint8 | `1` | `0/1` |
| 11 | `telemetry_serial1` | uint8 | `1` | `0/1` |
| 12 | `nav_rate_hz` | uint8 | `50` | `10–100` |
| 13 | `debug_rate_hz` | uint8 | `1` | `1–10` |
| 14 | `raw_rate_hz` | uint8 | `5` | `1–20` |

### Feature flags (8 B)

| Off | Campo | Tipo | Default | Range |
|----:|-------|------|---------|-------|
| 15 | `mag_enabled` | uint8 | `1` | `0/1` |
| 16 | `_reserved_gps_hdg` | uint8 | `0` | riservato (ex GPS heading init da COG) |
| 17 | `earth_rotation_comp` | uint8 | `1` | `0/1` |
| 18 | `zupt_enabled` | uint8 | `1` | `0/1` |
| 19 | `accel_leveling_enabled` | uint8 | `1` | `0/1` |
| 20 | `wind_estimation_enabled` | uint8 | `1` | `0/1` |
| 21 | `airspeed_enabled` | uint8 | `0` | `0/1` |
| 22 | `gps_sim_noise` | uint8 | `0` | `0/1` |

### Hardware (3 B)

| Off | Campo | Tipo | Default | Range |
|----:|-------|------|---------|-------|
| 23 | `baro_sensor_type` | uint8 | `1` (BMP581) | `0`=BMP390, `1`=BMP581 |
| 24 | `airspeed_bus` | uint8 | `0` (SPI) | `0`=SPI ELVH-M250D, `1`=CAN DroneCAN |
| 25 | `airspeed_mount_axis` | uint8 | `1` (Z) | `0`=X (fixed-wing), `1`=Z (quad) |

### Particle Filter (26 B)

| Off | Campo | Tipo | Default | Range |
|----:|-------|------|---------|-------|
| 26 | `pf_n_particles` | uint16 | `600` | `100 – PF_N_PARTICLES_MAX(600)` |
| 28 | `pf_ess_threshold` | float | `0.500` | `0.1 – 0.7` |
| 32 | `pf_roughening_att` | float | `0.002` rad | `> 0` (roll/pitch roughening σ) |
| 36 | `pf_roughening_pos` | float | `0.0` | `finite` (attualmente inutilizzato) |
| 40 | `pf_roughening_vel` | float | `0.0` | `finite` (attualmente inutilizzato) |
| 44 | `pf_gps_sigma_h` | float | `5.0` m | `> 0` (floor sigma orizzontale) |
| 48 | `pf_gps_sigma_v` | float | `8.0` m | `> 0` (floor sigma verticale) |

### EKF — rumore di processo dei bias (20 B)

| Off | Campo | Tipo | Default | Unità |
|----:|-------|------|---------|-------|
| 52 | `bias_gyro_noise` | float | `1e-9` | (rad/s)²/s |
| 56 | `bias_accel_noise` | float | `4e-7` | (m/s²)²/s |
| 60 | `bias_hard_iron_noise` | float | `1.0e-3` | (nT)²/s |
| 64 | `bias_baro_noise` | float | `1e-5` | (m)²/s |
| 68 | `bias_wind_noise` | float | `0.15` | (m/s)²/s |

Tutti richiesti `> 0` e finiti.

### EKF — covarianza iniziale dei bias (20 B)

| Off | Campo | Tipo | Default | Unità |
|----:|-------|------|---------|-------|
| 72 | `bias_init_gyro_cov` | float | `1.2e-5` | (rad/s)² — ≈ (0.2 deg/s)² |
| 76 | `bias_init_accel_cov` | float | `4e-4` | (m/s²)² — ≈ (2 mg)² |
| 80 | `bias_init_hard_iron_cov` | float | `1.0e6` | (nT)² |
| 84 | `bias_init_baro_cov` | float | `2500` | (m)² — σ≈50 m |
| 88 | `bias_init_wind_cov` | float | `100` | (m/s)² |

Tutti richiesti `> 0` e finiti.

### RBPF — rumore di processo per-particle (8 B)

| Off | Campo | Tipo | Default | Unità |
|----:|-------|------|---------|-------|
| 92 | `rbpf_ekf_vel_q` | float | `0.0009` | (m/s)²/s — σ_a² floor SCH16T-K10 |
| 96 | `rbpf_ekf_pos_q` | float | `0.00001` | (m)²/s — integrazione posizione |

### Board + airspeed ratio (6 B)

| Off | Campo | Tipo | Default | Range |
|----:|-------|------|---------|-------|
| 100 | `board_type` | uint8 | `0` (FULL) | `0`=FULL, `1`=MINI |
| 101 | `mag_bus` | uint8 | `0` (SPI) | `0`=SPI RM3100, `1`=CAN DroneCAN |
| 102 | `airspeed_ratio` | float | `1.0` | `0.5 – 10.0`, finito — `IAS = sqrt(DP_TO_IAS_FACTOR · ratio · |dp − offset|)` stile ArduPlane `ARSPD_RATIO`. L'offset è catturato allo startup, solo in RAM. |

**Totale: 106 B**.

### Vincoli incrociati

- Se `board_type = MINI` e `mag_bus ≠ CAN`, `applyBoardTypeConstraints()` forza `mag_enabled = 0` (MINI non ha RM3100 SPI).
- Su `RESET_DEFAULT` per `MINI`: `airspeed_enabled = 1` e `airspeed_bus = SPI` vengono forzati prima del save.

---

## EEPROM layout e validazione

Indirizzo base: `SYS_CONFIG_EEPROM_START = 100`.

```
Offset da base  Dato
  0..1          Signature  = 0xCF47       (uint16 LE)
  2..3          Version    = 7            (uint16 LE) — SYS_CONFIG_VERSION
  4..109        SystemConfig (106 byte, layout sopra)
  110..111      CRC-16-CCITT su SystemConfig (uint16 LE)
```

Occupazione totale: 112 B (indirizzi 100–211).

### Sequenza di boot (`loadSystemConfig()`)

1. Legge signature: se ≠ `0xCF47` → `loadDefaultConfig()` + `saveSystemConfig()`, ritorna `false`.
2. Legge version: se ≠ `SYS_CONFIG_VERSION` → stesso trattamento (default + save).
3. Legge struct + CRC: se mismatch → default + save.
4. `validateSystemConfig()`: se qualche campo fuori range → default + save.
5. Altrimenti copia in `sysConfig`, applica `applyBoardTypeConstraints()`, stampa sommario su USB.

Note operative:
- La versione `7` è incompatibile con versioni precedenti → all'upgrade firmware la EEPROM viene riscritta con i default.
- L'EEPROM contiene anche le calibrazioni mounting (0–19), accel (20–77), mag (78–99): **non toccarle** manualmente sotto l'indirizzo 100.

---

## Sequenze tipiche GS → Device

**Leggere la configurazione corrente**
```
→ 0x10 GET_CONFIG
← 0x11 OK + SystemConfig (106 B)
```

**Cambiare un parametro (es. `pf_ess_threshold` da 0.5 a 0.6) e persistere**
```
1. → 0x10 GET_CONFIG                      (leggi struct corrente)
   ← 0x11 OK + SystemConfig
2. modifica il campo in locale
3. → 0x10 SET_CONFIG + nuova SystemConfig (applica runtime)
   ← 0x11 OK                              (o INVALID + nome campo)
4. → 0x10 SAVE_CONFIG                     (persist in EEPROM)
   ← 0x11 OK
5. opzionale: → 0x10 REBOOT               (se servono cambi hardware: baud, baro type, board)
   ← 0x11 OK  ... reset dopo 100 ms
```

**Factory reset**
```
→ 0x10 RESET_DEFAULT
← 0x11 OK   (defaults caricati, board_type preservato, già salvati in EEPROM)
```

**Parametri che richiedono REBOOT per essere pienamente attivi**
- `gps_baud_rate`, `serial1_baud` (riapertura porte seriali)
- `baro_sensor_type`, `board_type`, `mag_bus`, `airspeed_bus` (init hardware)
- `gps_type` (parser GPS)

Tutti gli altri parametri (rate telemetria, flag feature, parametri PF/EKF, sigma GPS, airspeed_ratio) vengono presi in considerazione dal runtime senza riavvio.
