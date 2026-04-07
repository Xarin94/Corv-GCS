# Guida Completa alla Telemetria Binaria - Implementazione Attuale

## Indice
1. [Panoramica](#panoramica)
2. [Struttura dei Pacchetti](#struttura-dei-pacchetti)
3. [CRC-16-CCITT: Spiegazione Dettagliata](#crc-16-ccitt-spiegazione-dettagliata)
4. [Packet Type 0x01: Navigation](#packet-type-0x01-navigation)
5. [Packet Type 0x02: Debug](#packet-type-0x02-debug)
6. [Packet Type 0x03: Raw Sensor](#packet-type-0x03-raw-sensor)
7. [Packet Type 0x10: Config Write (USB)](#packet-type-0x10-config-write)
8. [Packet Type 0x11: Config Response (USB)](#packet-type-0x11-config-response)
9. [SystemConfig Structure](#systemconfig-structure)
10. [Codifica e Decodifica](#codifica-e-decodifica)
11. [Configurazione](#configurazione)
12. [Esempi Pratici](#esempi-pratici)

---

## Panoramica

La telemetria binaria è un protocollo compatto e efficiente per trasmettere i dati di navigazione del sistema RI-EKF. A differenza di formati testuali come JSON:

- **Riduce la banda** del 77.5% (da ~200 kbps a ~45 kbps)
- **Utilizza 9.7%** della capacità del link a 460,800 baud
- **Mantiene precisione esatta** per tutti i dati critici
- **Implementa validazione** tramite checksum CRC-16

### Prestazioni di Banda

| Tipo Pacchetto | Frequenza | Dimensione | Banda | Direzione |
|---|---|---|---|---|
| Navigation (0x01) | 50 Hz | 111 byte | 44.4 kbps | Device → GS |
| Debug (0x02) | 1 Hz | 75 byte | 0.600 kbps | Device → GS |
| Raw Sensor (0x03) | 5 Hz | 81 byte | 3.24 kbps | Device → GS |
| Config Write (0x10) | On demand | variabile | — | **GS → Device (USB only)** |
| Config Response (0x11) | On demand | variabile | — | Device → GS (USB only) |
| **Totale telemetria** | - | - | **~48 kbps** | |

---

## Struttura dei Pacchetti

Tutti i pacchetti seguono una struttura identica:

```
┌─────────────┬──────────┬────────┬────────┬──────────┬─────────┐
│ Sync Byte 1 │ Sync Byte 2 │ Type ID │ Lunghezza │ Sequenza │ Payload │ CRC │
│ 0xA5        │ 0x5A       │ 1 byte │ 1 byte   │ 1 byte  │ N byte │ 2 byte│
├─────────────┼──────────┼────────┼────────┼──────────┼─────────┤
│ Byte 0      │ Byte 1   │ Byte 2 │ Byte 3 │ Byte 4  │ Bytes 5..N+4 │ Byte N+5..N+6 │
└─────────────┴──────────┴────────┴────────┴──────────┴─────────┘

Header (5 byte) + Payload (N byte) + CRC (2 byte)
```

### Header (5 byte)

| Offset | Nome | Tipo | Valore | Descrizione |
|--------|------|------|--------|-------------|
| **0** | Sync 1 | uint8 | `0xA5` | Primo byte di sincronizzazione (fisso) |
| **1** | Sync 2 | uint8 | `0x5A` | Secondo byte di sincronizzazione (fisso) |
| **2** | Type ID | uint8 | `0x01`/`0x02`/`0x03`/`0x10`/`0x11` | Tipo di pacchetto |
| **3** | Lunghezza | uint8 | `104`/`68`/`74` | Lunghezza del payload **in byte** |
| **4** | Sequenza | uint8 | `0-255` | Numero sequenziale (avvolge a 256) |

### Parametri Globali

- **Byte order**: LITTLE-ENDIAN (LSB first) per tutti i campi multi-byte
- **Float**: IEEE 754 (32-bit single precision)
- **Double**: IEEE 754 (64-bit double precision)
- **Interi scalati**: int16_t con fattori di scala specifici

---

## CRC-16-CCITT: Spiegazione Dettagliata

Il CRC (Cyclic Redundancy Check) è un algoritmo che crea un valore di 2 byte per rilevare errori di trasmissione.

### Algoritmo Utilizzato: CRC-16-CCITT

**Parametri fondamentali:**
- **Polinomio**: `0x1021` (che rappresenta x¹⁶ + x¹² + x⁵ + 1)
- **Valore iniziale**: `0xFFFF` (tutti i bit a 1)
- **Bit order**: Processamento del bit MSB (Most Significant Bit) per primo
- **Byte order CRC finale**: Little-Endian (LSB scritto per primo)

### Cosa Viene Coperto dal CRC

```
┌─────────────┬──────────┬────────┬────────┬──────────┬─────────┐
│ 0xA5        │ 0x5A     │ Type   │ Length │ Sequence │ Payload │ CRC │
│ NON incluso │ NON incl │ ✓      │ ✓      │ ✓        │ ✓       │ ✓   │
└─────────────┴──────────┴────────┴────────┴──────────┴─────────┘
              ↑ Non inclusi                            ↑ Calcolato su questi byte
```

**Il CRC copre**: Type ID (byte 2) + Length (byte 3) + Sequence (byte 4) + Payload (bytes 5 a 5+Length-1)

### Implementazione in C

```c
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;  // Inizializzazione: tutti i bit a 1

    for (size_t i = 0; i < length; i++) {
        // XOR con il byte corrente (spostato a sinistra di 8 bit)
        crc ^= (uint16_t)data[i] << 8;

        // Processa 8 bit per il byte corrente
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {  // Se il bit più significativo è 1
                crc = (crc << 1) ^ 0x1021;  // Shift left e XOR con polinomio
            } else {
                crc = crc << 1;  // Solo shift left
            }
        }
    }

    return crc;  // Valore CRC finale a 16 bit
}
```

### Esempio Passo-Passo

Supponendo di avere un piccolo payload: `Type=0x01, Length=0x04, Seq=0x00, Payload=[0x12,0x34,0x56,0x78]`

1. **Inizializzazione**: `crc = 0xFFFF`
2. **Byte 0 (Type=0x01)**:
   - `crc ^= 0x01 << 8` → `crc = 0xFEFE`
   - Processa 8 bit...
3. **Byte 1 (Length=0x04)**:
   - Continua il calcolo...
4. **Byte 2 (Seq=0x00)**:
   - Continua il calcolo...
5. **Bytes 3-6 (Payload)**:
   - Continua il calcolo...
6. **Risultato finale**: `crc = 0xXXXX` (valore dipende dai dati reali)

### Verifica del CRC

**Durante la ricezione:**

```c
// Dopo aver ricevuto un pacchetto completo:
uint8_t packet[...];  // include header + payload + CRC
uint16_t crc_received = packet[total_length-2] | (packet[total_length-1] << 8);

// Calcola il CRC sul ricevuto (escludendo i byte CRC)
uint16_t crc_calculated = calculateCRC16(packet + 2, total_length - 4);

if (crc_received == crc_calculated) {
    // Pacchetto valido
} else {
    // Errore di trasmissione rilevato
}
```

---

## Packet Type 0x01: Navigation

Pacchetto ad alta frequenza (50 Hz) contenente lo stato di navigazione essenziale.

### Dimensioni

| Componente | Byte |
|---|---|
| Header | 5 |
| Payload | 104 |
| CRC | 2 |
| **Totale** | **111** |

### Struttura del Payload (104 byte)

Il payload è organizzato in blocchi logici:

#### Timestamp (4 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Note |
|--------|-------|------|-----------|-------|------|
| **0** | `timestamp_ms` | uint32 | 4 | milliseconds | Uptime del sistema, little-endian |

**Esempio**: Valore 0x000007D0 (little-endian) = 0xD0, 0x07, 0x00, 0x00 = 2000 ms

#### Atteggiamento (6 byte) - **SCALED INTEGERS**

| Offset | Campo | Tipo | Lunghezza | Fattore Scala | Precisione | Range | Note |
|--------|-------|------|-----------|---|---|---|---|
| **4** | `roll` | int16 | 2 | ÷1000 | 0.001 rad | ±32.767 rad | Milliradian, little-endian |
| **6** | `pitch` | int16 | 2 | ÷1000 | 0.001 rad | ±32.767 rad | Milliradian, little-endian |
| **8** | `yaw` | int16 | 2 | ÷1000 | 0.001 rad | ±32.767 rad | Milliradian, little-endian |

**Encoding**: `int16_value = (float_radians × 1000)`
**Decoding**: `float_radians = int16_value / 1000.0`

**Esempio Roll**:
- Valore reale: 1.234 rad
- Codificato come: 1234 (0x4D2 little-endian: 0xD2, 0x04)
- Decodificato: 1234 / 1000.0 = 1.234 rad

#### Posizione (20 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Note |
|--------|-------|------|-----------|-------|------|
| **10** | `latitude` | double | 8 | degrees | [-90, +90], IEEE 754 double |
| **18** | `longitude` | double | 8 | degrees | [-180, +180], IEEE 754 double |
| **26** | `altitude_msl` | float | 4 | meters | IEEE 754 single precision |

**Nota**: Latitude e Longitude usano double (8 byte) per precisione GPS (~10 cm).

#### Velocità NED (12 byte)

| Offset | Campo | Tipo | Lunghezza | Unità |
|--------|-------|------|-----------|-------|
| **30** | `velocity_north` | float | 4 | m/s |
| **34** | `velocity_east` | float | 4 | m/s |
| **38** | `velocity_down` | float | 4 | m/s |

NED = North-East-Down (coordinate locali del veicolo)

#### Stima del Vento (12 byte)

| Offset | Campo | Tipo | Lunghezza | Unità |
|--------|-------|------|-----------|-------|
| **42** | `wind_north` | float | 4 | m/s |
| **46** | `wind_east` | float | 4 | m/s |
| **50** | `wind_magnitude` | float | 4 | m/s |

#### Dati Aerodinamici (12 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Note |
|--------|-------|------|-----------|-------|------|
| **54** | `airspeed` | float | 4 | m/s | True airspeed (TAS) |
| **58** | `groundspeed` | float | 4 | m/s | Velocità al suolo |
| **62** | `angle_of_attack` | float | 4 | radians | AOA |
| **66** | `sideslip_angle` | float | 4 | radians | Angolo di scarroccio |

#### Dati IMU Fusi (18 byte) - **ACCELERAZIONE SCALED**

| Offset | Campo | Tipo | Lunghezza | Fattore Scala | Precisione | Range | Note |
|--------|-------|------|-----------|---|---|---|---|
| **70** | `accel_x` | int16 | 2 | ÷100 | 0.01 m/s² | ±327.67 m/s² | Centimeter per s², little-endian |
| **72** | `accel_y` | int16 | 2 | ÷100 | 0.01 m/s² | ±327.67 m/s² | Centimeter per s², little-endian |
| **74** | `accel_z` | int16 | 2 | ÷100 | 0.01 m/s² | ±327.67 m/s² | Centimeter per s², little-endian |
| **76** | `gyro_x` | float | 4 | (nessuno) | IEEE 754 | ±∞ | rad/s, body frame |
| **80** | `gyro_y` | float | 4 | (nessuno) | IEEE 754 | ±∞ | rad/s, body frame |
| **84** | `gyro_z` | float | 4 | (nessuno) | IEEE 754 | ±∞ | rad/s, body frame |

**Encoding Accelerazione**: `int16_value = (float_ms2 × 100)`
**Decoding Accelerazione**: `float_ms2 = int16_value / 100.0`

**Esempio Accelerazione Z**:
- Valore reale: 9.81 m/s²
- Codificato come: 981 (0x03D5 little-endian: 0xD5, 0x03)
- Decodificato: 981 / 100.0 = 9.81 m/s²

#### Stato del Filtro (8 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Note |
|--------|-------|------|-----------|-------|------|
| **88** | `confidence` | float | 4 | 0.0-1.0 | Metrica di confidenza del filtro |
| **92** | `covariance_trace` | float | 4 | - | Traccia della matrice di covarianza |

#### Qualità GPS (6 byte)

| Offset | Campo | Tipo | Lunghezza | Valori | Note |
|--------|-------|------|-----------|--------|------|
| **96** | `gps_fix_type` | uint8 | 1 | 0-6 | Tipo di fix GPS (vedi tabella sotto) |
| **97** | `gps_num_satellites` | uint8 | 1 | 0-50 | Numero di satelliti utilizzati |
| **98** | `gps_hdop` | float | 4 | - | Horizontal Dilution of Precision |

**GPS Fix Type Values**:
```
0: Nessun fix
1: Dead reckoning only
2: 2D fix
3: 3D fix
4: GNSS + Dead reckoning
5: RTK fixed
6: RTK float
```

#### Flag di Stato (2 byte) - **BITFIELD**

| Offset | Campo | Tipo | Lunghezza | Descrizione |
|--------|-------|------|-----------|-------------|
| **102** | `status_flags` | uint16 | 2 | Bitfield con 9 flag + 1 campo a 2 bit |

**Struttura dei Bit**:

```
Bit 15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
    │────────│ │───│ │───────│ │───────────────────────────────│
    Reserved  Spoof Jam Aiding GPS Mag Baro IMU2 IMU1 GPS ZUPT Init Conv
    (3 bit)        Mode(2) Bypass (8 bits di stato singoli)
```

| Bit | Mask | Nome | Descrizione |
|-----|------|------|-------------|
| **0** | `0x0001` | CONVERGED | 1 = Filtro convergito, 0 = In convergenza |
| **1** | `0x0002` | INITIALIZED | 1 = Inizializzato, 0 = Non inizializzato |
| **2** | `0x0004` | ZUPT_ACTIVE | 1 = Velocità zero rilevata (fermo), 0 = In movimento |
| **3** | `0x0008` | GPS_AVAILABLE | 1 = GPS disponibile, 0 = GPS negato |
| **4** | `0x0010` | IMU1_HEALTHY | 1 = Sano, 0 = Guasto |
| **5** | `0x0020` | IMU2_HEALTHY | 1 = Sano, 0 = Guasto |
| **6** | `0x0040` | BARO_HEALTHY | 1 = Sano, 0 = Guasto |
| **7** | `0x0080` | MAG_HEALTHY | 1 = Sano, 0 = Guasto |
| **8** | `0x0100` | GPS_BYPASS | 1 = Modalità bypass GPS attiva, 0 = Normale |
| **9-10** | `0x0600` | AIDING_MODE | Modalità di aiding (2 bit, vedi tabella sotto) |
| **11** | `0x0800` | GPS_JAMMING | 1 = Jamming GPS rilevato, 0 = Nessun jamming |
| **12** | `0x1000` | GPS_SPOOFING | 1 = Spoofing GPS sospetto, 0 = Nessun spoofing |
| **13-15** | `0xE000` | RESERVED | Riservato per uso futuro |

**Aiding Mode Values (Bit 9-10)**:
```
00 (0): GPS + Magnetometro
01 (1): Solo GPS
10 (2): Solo Magnetometro
11 (3): Dead Reckoning only
```

**Esempio Creazione Status Flags**:
```c
uint16_t status_flags = 0;

// Individualmente:
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
status_flags |= (0 << 11);             // GPS_JAMMING = 0
status_flags |= (0 << 12);             // GPS_SPOOFING = 0

// Risultato: 0x007B (123 in decimale)
```

**Esempio con Jamming Rilevato**:
```c
uint16_t status_flags = 0;
status_flags |= STATUS_FLAG_CONVERGED;
status_flags |= STATUS_FLAG_INITIALIZED;
status_flags |= STATUS_FLAG_GPS_AVAILABLE;
status_flags |= STATUS_FLAG_IMU1_HEALTHY;
status_flags |= STATUS_FLAG_IMU2_HEALTHY;
status_flags |= STATUS_FLAG_BARO_HEALTHY;
status_flags |= STATUS_FLAG_GPS_JAMMING;   // Jamming rilevato!

// Risultato: 0x087B - indica che il GPS è disponibile MA c'è jamming
```

---

## Packet Type 0x02: Debug

Pacchetto a bassa frequenza (1 Hz) contenente diagnostica, bias, stati magnetometro e metriche di performance.

**Nota (Protocol v5):** Rimossi `accel_bias_x/y` (non osservabili, solo Z stimato nel IEKF a 19 stati) e `earth_field_n/e/d` (campo magnetico fisso da WMM, non stimato dal filtro).

### Dimensioni

| Componente | Byte |
|---|---|
| Header | 5 |
| Payload | 68 |
| CRC | 2 |
| **Totale** | **75** |

### Struttura del Payload (68 byte)

#### Timestamp (4 byte)

| Offset | Campo | Tipo | Lunghezza | Unità |
|--------|-------|------|-----------|-------|
| **0** | `timestamp_ms` | uint32 | 4 | milliseconds |

#### Bias Giroscopio (12 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Range |
|--------|-------|------|-----------|-------|-------|
| **4** | `gyro_bias_x` | float | 4 | rad/s | ±∞ |
| **8** | `gyro_bias_y` | float | 4 | rad/s | ±∞ |
| **12** | `gyro_bias_z` | float | 4 | rad/s | ±∞ |

Stime attuali dei bias di offset del giroscopio, convergono durante il filtro.

#### Bias Accelerometro Z (4 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Range |
|--------|-------|------|-----------|-------|-------|
| **16** | `accel_bias_z` | float | 4 | m/s² | ±∞ |

Solo l'asse Z è stimato dal filtro. I bias X/Y non sono osservabili e sono stati rimossi.

#### Bias Barometro (4 byte)

| Offset | Campo | Tipo | Lunghezza | Unità |
|--------|-------|------|-----------|-------|
| **20** | `baro_bias` | float | 4 | meters |

Stima del bias dell'altimetro barometrico.

#### Qualità Magnetometro (4 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Range |
|--------|-------|------|-----------|-------|-------|
| **24** | `mag_quality` | float | 4 | 0.0-1.0 | [0, 1] |

Metrica di qualità della lettura magnetometrica (0 = pessimo, 1 = eccellente).

#### Bias Hard Iron (12 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Note |
|--------|-------|------|-----------|-------|------|
| **28** | `hard_iron_x` | float | 4 | nT | Bias hard iron asse X (body frame) |
| **32** | `hard_iron_y` | float | 4 | nT | Bias hard iron asse Y (body frame) |
| **36** | `hard_iron_z` | float | 4 | nT | Bias hard iron asse Z (body frame) |

Stime online del bias hard iron del magnetometro. Convergono tramite la fusione 3D direction-only.
Valori tipici: ±5000 nT. Limite: ±HARD_IRON_MAX_BIAS.

#### Metriche di Performance (8 byte)

| Offset | Campo | Tipo | Lunghezza | Unità | Note |
|--------|-------|------|-----------|-------|------|
| **40** | `loop_time_us` | uint16 | 2 | microseconds | Tempo medio del loop |
| **42** | `filter_time_us` | uint16 | 2 | microseconds | Tempo di calcolo EKF |
| **44** | `sensor_time_us` | uint16 | 2 | microseconds | Tempo di lettura sensori |
| **46** | `max_loop_time_us` | uint16 | 2 | microseconds | Tempo massimo dal precedente debug packet |

Tutti i tempi sono misurati in **microsecondi (µs)**.

#### Informazioni GPS Estese (8 byte)

| Offset | Campo | Tipo | Lunghezza | Unità |
|--------|-------|------|-----------|-------|
| **48** | `gps_horizontal_accuracy` | float | 4 | meters |
| **52** | `gps_vertical_accuracy` | float | 4 | meters |

Stime di accuratezza fornite dal ricevitore GPS.

#### Conteggi Sensori (4 byte)

| Offset | Campo | Tipo | Lunghezza | Valori | Descrizione |
|--------|-------|------|-----------|--------|-------------|
| **56** | `baro_sensor_count` | uint8 | 1 | 0-2 | Numero di barometri attivi |
| **57** | `gps_quality_indicator` | uint8 | 1 | 0-255 | Indicatore di qualità GPS (specifico ricevitore) |
| **58** | `imu_fused_status` | uint8 | 1 | 0 o 1 | 0=singolo IMU, 1=doppio IMU |
| **59** | `reserved` | uint8 | 1 | - | Riservato (padding di allineamento) |

#### Orario GPS (8 byte) - UTC dal Ricevitore GPS

| Offset | Campo | Tipo | Lunghezza | Valori | Unità |
|--------|-------|------|-----------|--------|-------|
| **60** | `gps_year` | uint16 | 2 | 1900-2100 | Anni |
| **62** | `gps_month` | uint8 | 1 | 1-12 | Mesi |
| **63** | `gps_day` | uint8 | 1 | 1-31 | Giorni |
| **64** | `gps_hour` | uint8 | 1 | 0-23 | Ore (UTC) |
| **65** | `gps_minute` | uint8 | 1 | 0-59 | Minuti |
| **66** | `gps_second` | uint8 | 1 | 0-59 | Secondi |
| **67** | `gps_time_valid` | uint8 | 1 | 0 o 1 | 1=valido, 0=non valido |

---

## Packet Type 0x03: Raw Sensor

Pacchetto con dati sensori grezzi (pre-filtro) per diagnostica.
Trasmesso a 5 Hz (`BINARY_RAW_RATE_HZ`).

**Dimensione payload:** 74 byte | **Dimensione totale:** 81 byte (header 5 + payload 74 + CRC 2)

### Layout dei Campi

| Offset | Campo | Tipo | Byte | Unita | Descrizione |
|--------|-------|------|------|-------|-------------|
| **0** | `timestamp_ms` | uint32 | 4 | ms | Timestamp di sistema |
| **4** | `gps_latitude` | double | 8 | deg | Latitudine GPS grezza |
| **12** | `gps_longitude` | double | 8 | deg | Longitudine GPS grezza |
| **20** | `gps_altitude_msl` | float | 4 | m | Altitudine GPS grezza MSL |
| **24** | `gps_vel_north` | float | 4 | m/s | Velocita GPS Nord grezza |
| **28** | `gps_vel_east` | float | 4 | m/s | Velocita GPS Est grezza |
| **32** | `gps_vel_down` | float | 4 | m/s | Velocita GPS Giu grezza |
| **36** | `gps_fix_type` | uint8 | 1 | - | Tipo fix GPS (0-5) |
| **37** | `gps_num_sats` | uint8 | 1 | - | Numero satelliti |
| **38** | `gps_h_accuracy` | float | 4 | m | Accuratezza orizzontale riportata |
| **42** | `mag_x` | float | 4 | nT | Magnetometro body X (avanti) |
| **46** | `mag_y` | float | 4 | nT | Magnetometro body Y (destra) |
| **50** | `mag_z` | float | 4 | nT | Magnetometro body Z (giu) |
| **54** | `baro_altitude` | float | 4 | m | Altitudine barometrica |
| **58** | `baro_pressure` | float | 4 | Pa | Pressione barometrica grezza |
| **62** | `imu_accel_z` | float | 4 | m/s2 | Accelerazione Z (test gravita) |
| **66** | `imu_gyro_x` | float | 4 | rad/s | Velocita angolare X |
| **70** | `imu_gyro_y` | float | 4 | rad/s | Velocita angolare Y |

**Nota:** Con `GPS_SIM_NOISE` attivo, i dati GPS in questo pacchetto includono il rumore simulato aggiunto.

---

## Packet Type 0x10: Config Write

**Direzione**: Ground Station → Device (**solo USB Serial**)
**Frequenza**: On demand (invio manuale dall'operatore)

> **SICUREZZA**: I comandi di configurazione sono accettati **esclusivamente** sulla porta USB Serial. Non vengono mai processati su Serial1 (porta RF/telemetria). Questo garantisce che durante il funzionamento operativo nessun comando esterno possa alterare la configurazione del sistema.

### Struttura del Pacchetto

```
┌──────┬──────┬──────┬────────┬─────┬───────────────────┬───────┐
│ 0xA5 │ 0x5A │ 0x10 │ Length │ Seq │ Payload           │ CRC16 │
│ 1B   │ 1B   │ 1B   │ 1B     │ 1B  │ 1 + N byte        │ 2B    │
└──────┴──────┴──────┴────────┴─────┴───────────────────┴───────┘
```

### Payload

| Offset | Nome | Tipo | Descrizione |
|--------|------|------|-------------|
| **0** | `command_id` | uint8 | Identificativo del comando |
| **1..N** | `command_data` | byte[] | Dati del comando (opzionale) |

### Command IDs

| ID | Comando | Payload Data | Descrizione |
|----|---------|-------------|-------------|
| `0x01` | SET_CONFIG | SystemConfig (41 byte) | Imposta la configurazione runtime corrente |
| `0x02` | GET_CONFIG | (vuoto) | Richiede la configurazione corrente |
| `0x03` | SAVE_CONFIG | (vuoto) | Salva la config corrente in EEPROM |
| `0x04` | RESET_DEFAULT | (vuoto) | Ripristina i valori di fabbrica e salva |
| `0x05` | REBOOT | (vuoto) | Riavvio software del sistema |

### Dettaglio Comandi

**SET_CONFIG (0x01)**: La ground station invia l'intera struct `SystemConfig` (41 byte). Il dispositivo valida i parametri (range check) e, se validi, li applica immediatamente alla configurazione runtime. **Non salva automaticamente in EEPROM** — per rendere permanenti le modifiche, inviare successivamente SAVE_CONFIG.

**GET_CONFIG (0x02)**: Il dispositivo risponde con la struct `SystemConfig` corrente nel payload della risposta.

**SAVE_CONFIG (0x03)**: Scrive in EEPROM: signature (0xCF47) + version (1) + struct SystemConfig + CRC-16. Al prossimo avvio il sistema caricherà questa configurazione.

**RESET_DEFAULT (0x04)**: Carica i valori di default (identici ai `#define` originali in `config.h`), li salva in EEPROM e li applica. Richiede un REBOOT per attivare completamente.

**REBOOT (0x05)**: Esegue un reset software del Teensy 4.x tramite `SCB_AIRCR = 0x05FA0004`. Il dispositivo si riavvia e carica la configurazione dall'EEPROM.

### Esempio: SET_CONFIG

```
TX: A5 5A 10 2A 00 01 [41 byte SystemConfig] [CRC_L CRC_H]
                 │  │  │  └─ SystemConfig struct packed
                 │  │  └──── command_id = SET_CONFIG
                 │  └─────── seq = 0
                 └────────── length = 42 (1 cmd + 41 struct)
```

### Esempio: GET_CONFIG

```
TX: A5 5A 10 01 00 02 [CRC_L CRC_H]
                 │  │  └─ command_id = GET_CONFIG
                 │  └──── seq = 0
                 └─────── length = 1
```

---

## Packet Type 0x11: Config Response

**Direzione**: Device → Ground Station (USB Serial)
**Frequenza**: In risposta a ogni comando 0x10

### Struttura del Pacchetto

```
┌──────┬──────┬──────┬────────┬─────┬───────────────────┬───────┐
│ 0xA5 │ 0x5A │ 0x11 │ Length │ Seq │ Payload           │ CRC16 │
│ 1B   │ 1B   │ 1B   │ 1B     │ 1B  │ 2 + N byte        │ 2B    │
└──────┴──────┴──────┴────────┴─────┴───────────────────┴───────┘
```

### Payload

| Offset | Nome | Tipo | Descrizione |
|--------|------|------|-------------|
| **0** | `response_code` | uint8 | Codice di risposta |
| **1** | `command_id` | uint8 | Echo del comando ricevuto |
| **2..N** | `data` | byte[] | Dati di risposta (opzionale) |

### Response Codes

| Codice | Nome | Descrizione |
|--------|------|-------------|
| `0x00` | OK | Comando eseguito con successo |
| `0x01` | ERROR | Errore generico nell'esecuzione |
| `0x02` | CRC_FAIL | CRC del pacchetto ricevuto non valido |
| `0x03` | INVALID | Parametro fuori range o valore non valido |

### Risposte per Comando

| Comando Ricevuto | Response Code | Data nel Payload |
|-----------------|---------------|------------------|
| SET_CONFIG | OK / INVALID | (vuoto) |
| GET_CONFIG | OK | SystemConfig (41 byte) |
| SAVE_CONFIG | OK / ERROR | (vuoto) |
| RESET_DEFAULT | OK | (vuoto) |
| REBOOT | OK | (vuoto, inviato prima del riavvio) |

### Esempio: Risposta a GET_CONFIG

```
RX: A5 5A 11 2B 00 00 02 [41 byte SystemConfig] [CRC_L CRC_H]
                 │  │  │  │  └─ SystemConfig struct
                 │  │  │  └──── command_id echo = GET_CONFIG
                 │  │  └─────── response_code = OK
                 │  └────────── seq = 0
                 └───────────── length = 43 (1 resp + 1 cmd + 41 struct)
```

### Esempio: Risposta a SET_CONFIG con errore

```
RX: A5 5A 11 02 00 03 01 [CRC_L CRC_H]
                 │  │  │  └─ command_id echo = SET_CONFIG
                 │  │  └──── response_code = INVALID
                 │  └─────── seq = 0
                 └────────── length = 2
```

---

## SystemConfig Structure

Struttura dati packed (41 byte) che contiene tutti i parametri di configurazione runtime del sistema. Memorizzata in EEPROM a partire dall'indirizzo 100.

### Layout EEPROM

```
Indirizzo 100: [Signature 2B][Version 2B][SystemConfig 41B][CRC-16 2B]
               │              │            │                  │
               │              │            │                  └─ CRC su SystemConfig
               │              │            └──────────────────── Dati configurazione (110 byte)
               │              └───────────────────────────────── Versione struct (2)
               └──────────────────────────────────────────────── Magic 0xCF47
```

Totale occupazione EEPROM: 116 byte (indirizzi 100-215).

### Tabella Campi

| Offset | Nome | Tipo | Byte | Range | Default | Descrizione |
|--------|------|------|------|-------|---------|-------------|
| **0** | `gps_type` | uint8 | 1 | 0-1 | 0 (UBlox) | 0=UBlox, 1=Mosaic |
| **1** | `gps_baud_rate` | uint32 | 4 | 9600-921600 | 460800 | Baud rate porta GPS |
| **5** | `serial1_baud` | uint32 | 4 | 9600-921600 | 921600 | Baud rate Serial1 (telemetria/output) |
| **9** | `output_protocol` | uint8 | 1 | 0-1 | 1 (VN) | 0=Binary Custom, 1=VectorNav |
| **10** | `telemetry_usb` | uint8 | 1 | 0-1 | 1 | Telemetria su USB (0=off, 1=on) |
| **11** | `telemetry_serial1` | uint8 | 1 | 0-1 | 1 | Telemetria su Serial1 (0=off, 1=on) |
| **12** | `nav_rate_hz` | uint8 | 1 | 10-100 | 50 | Frequenza pacchetti Navigation |
| **13** | `debug_rate_hz` | uint8 | 1 | 1-10 | 1 | Frequenza pacchetti Debug |
| **14** | `raw_rate_hz` | uint8 | 1 | 1-20 | 5 | Frequenza pacchetti Raw Sensor |
| **15** | `mag_enabled` | uint8 | 1 | 0-1 | 1 | Magnetometro abilitato |
| **16** | `gps_heading_init` | uint8 | 1 | 0-1 | 0 | Inizializzazione heading da GPS |
| **17** | `earth_rotation_comp` | uint8 | 1 | 0-1 | 1 | Compensazione rotazione terrestre |
| **18** | `zupt_enabled` | uint8 | 1 | 0-1 | 1 | Zero velocity update |
| **19** | `gyrocompass_enabled` | uint8 | 1 | 0-1 | 1 | Gyrocompassing |
| **20** | `accel_leveling_enabled` | uint8 | 1 | 0-1 | 1 | Livellamento accelerometrico |
| **21** | `wind_estimation_enabled` | uint8 | 1 | 0-1 | 1 | Stima del vento |
| **22** | `airspeed_enabled` | uint8 | 1 | 0-1 | 0 | Sensore airspeed (compile-time) |
| **23** | `gps_sim_noise` | uint8 | 1 | 0-1 | 0 | Simulazione rumore GPS (test) |
| **24** | `delay_compensation` | uint8 | 1 | 0-1 | 1 | Compensazione ritardi sensori (compile-time) |
| **25** | `baro_sensor_type` | uint8 | 1 | 0-1 | 0 (BMP390) | 0=BMP390, 1=BMP581 (compile-time) |
| | **Initial Covariance P₀** | | | | | |
| **26** | `init_att_cov` | float | 4 | >0 | 1e-2 | Attitude legacy reset (rad²) |
| **30** | `init_roll_pitch_cov` | float | 4 | >0 | 1e-3 | Roll/Pitch (rad²) |
| **34** | `init_yaw_cov` | float | 4 | >0 | 1.0 | Yaw (rad²) |
| **38** | `init_pos_cov` | float | 4 | >0 | 2.5 | Position (m²) |
| **42** | `init_vel_cov` | float | 4 | >0 | 0.1 | Velocity (m²/s²) |
| **46** | `init_gyro_bias_cov` | float | 4 | >0 | 1e-6 | Gyro bias (rad²/s²) |
| **50** | `init_accel_bias_cov` | float | 4 | >0 | 1e-5 | Accel bias (m²/s⁴) |
| **54** | `init_baro_bias_cov` | float | 4 | >0 | 2500 | Baro bias (m²) |
| **58** | `init_hard_iron_cov` | float | 4 | >0 | 1e6 | Hard iron (nT²) |
| **62** | `init_wind_cov` | float | 4 | >0 | 100 | Wind (m²/s²) |
| | **Process Noise Q (σ²/Hz)** | | | | | |
| **66** | `proc_att_noise` | float | 4 | >0 | 7.5e-9 | Gyroscope attitude (rad²/Hz) |
| **70** | `proc_vel_noise` | float | 4 | >0 | 1.5e-6 | Accelerometer velocity (m²/s⁴/Hz) |
| **74** | `proc_pos_noise` | float | 4 | >0 | 1e-10 | Position placeholder |
| **78** | `proc_gyro_bias_noise` | float | 4 | >0 | 1e-13 | Gyro bias stability |
| **82** | `proc_accel_bias_noise` | float | 4 | >0 | 2e-10 | Accel bias Z drift |
| **86** | `proc_accel_bias_xy_noise` | float | 4 | >0 | 2e-11 | Accel bias X/Y drift |
| **90** | `proc_baro_bias_noise` | float | 4 | >0 | 1e-5 | Atmospheric drift |
| **94** | `proc_hard_iron_noise` | float | 4 | >0 | 1e-3 | Hard iron drift |
| **98** | `proc_wind_noise` | float | 4 | >0 | 0.15 | Wind variation (m/s)²/Hz |
| **102** | `proc_att_linearization` | float | 4 | >0 | 1e-4 | Attitude linearization coeff |
| **106-109** | `reserved` | uint8[4] | 4 | — | 0 | Riservato per espansioni future |

**Nota**: I campi `airspeed_enabled`, `delay_compensation` e `baro_sensor_type` sono inclusi nella struct per completezza e configurazione futura, ma attualmente il loro funzionamento effettivo è controllato a **compile-time** tramite `#define` in `config.h`, poiché richiedono allocazione condizionale di memoria o definizione di macro di registro hardware.

**Nota**: Tutti i campi float di covarianza e rumore devono essere strettamente positivi e finiti. Un valore ≤ 0 o NaN/Inf causa il rifiuto della configurazione (response code INVALID).

### Validazione all'Avvio

Sequenza di verifica in `setup()`:

1. Leggi signature da EEPROM → se ≠ `0xCF47` → **primo avvio**: carica default, salva in EEPROM, prosegui
2. Leggi version → se ≠ `SYS_CONFIG_VERSION` → **primo avvio**: carica default, salva in EEPROM, prosegui
3. Leggi struct + CRC → se CRC mismatch → **ABORT**: stampa errore su USB, lampeggia LED, `while(1)`
4. Valida range parametri → se fuori range → **ABORT**: stampa errore su USB, lampeggia LED, `while(1)`
5. Applica configurazione alle variabili runtime → prosegui con inizializzazione normale

### CRC della Configurazione

Il CRC-16-CCITT viene calcolato sull'intera struct `SystemConfig` (110 byte) con gli stessi parametri del protocollo telemetria:
- Polinomio: `0x1021`
- Valore iniziale: `0xFFFF`
- Input/Output non riflessi

---

## Codifica e Decodifica

### Lato Trasmettitore (Teensy 4.0)

#### 1. Creazione del Pacchetto Navigation

```c
#include "telemetry_binary.h"

// Istanza dell'encoder
TelemetryBinaryEncoder encoder;

// Dati da trasmettere (da EKF, sensori, ecc.)
NavigationPacket nav_packet;
nav_packet.timestamp_ms = 2000;
nav_packet.roll = (int16_t)(1.234 * 1000);      // 1234 (scaled)
nav_packet.pitch = (int16_t)(-0.080 * 1000);    // -80 (scaled)
nav_packet.yaw = (int16_t)(3.129 * 1000);       // 3129 (scaled)
nav_packet.latitude = 45.123456;
nav_packet.longitude = -122.654321;
nav_packet.altitude_msl = 100.5;
// ... altri campi ...
nav_packet.accel_z = (int16_t)(9.81 * 100);     // 981 (scaled)
nav_packet.status_flags = 0x0077;
// ... ecc ...

// Codifica in buffer
uint8_t buffer[256];
size_t packet_size = encoder.encodeNavigationPacket(&nav_packet, buffer, sizeof(buffer));

// Trasmissione
Serial1.write(buffer, packet_size);  // 460,800 baud UART
```

#### 2. Creazione del Pacchetto Debug

```c
DebugPacket debug_packet;
debug_packet.timestamp_ms = 2000;
debug_packet.gyro_bias_x = 0.001;   // rad/s
debug_packet.gyro_bias_y = -0.0005;
debug_packet.gyro_bias_z = 0.0008;
debug_packet.accel_bias_z = 0.02;    // m/s² (solo asse Z)
debug_packet.baro_bias = 2.5;       // meters
debug_packet.mag_quality = 0.95;
debug_packet.hard_iron_x = 1250.0;  // nT
debug_packet.hard_iron_y = -800.0;
debug_packet.hard_iron_z = 350.0;
debug_packet.loop_time_us = 1250;   // 1250 µs = 1.25 ms per ciclo
debug_packet.filter_time_us = 850;  // Tempo EKF
debug_packet.sensor_time_us = 200;  // Tempo lettura sensori
debug_packet.max_loop_time_us = 1500;
// ... altri campi ...

// Codifica
uint8_t buffer[256];
size_t packet_size = encoder.encodeDebugPacket(&debug_packet, buffer, sizeof(buffer));

// Trasmissione
Serial1.write(buffer, packet_size);
```

### Lato Ricevitore (PC/Ground Station)

#### 1. Sincronizzazione e Parsing

```python
import struct

class BinaryTelemetryDecoder:
    def __init__(self):
        self.buffer = []
        self.sync_state = 0

    def feed_bytes(self, data_bytes):
        """Alimenta dati grezzi dal seriale"""
        for byte in data_bytes:
            self.parse_byte(byte)

    def parse_byte(self, byte):
        """Processa un singolo byte"""
        if self.sync_state == 0:
            # Ricerca primo sync byte
            if byte == 0xA5:
                self.sync_state = 1
                self.buffer = [byte]
        elif self.sync_state == 1:
            # Ricerca secondo sync byte
            if byte == 0x5A:
                self.sync_state = 2
                self.buffer.append(byte)
            else:
                # False alarm, ricerca dal nuovo 0xA5
                self.sync_state = 1 if byte == 0xA5 else 0
                self.buffer = [byte] if byte == 0xA5 else []
        elif self.sync_state == 2:
            # Abbiamo sync, leggi header
            if len(self.buffer) < 5:
                self.buffer.append(byte)
                if len(self.buffer) == 5:
                    self.sync_state = 3  # Pronto per payload
            else:
                # Leggi payload
                self.buffer.append(byte)
                # Calcola la lunghezza totale attesa
                payload_length = self.buffer[3]
                crc_offset = 5 + payload_length
                if len(self.buffer) == crc_offset + 2:
                    # Pacchetto completo!
                    self.process_packet()
                    self.sync_state = 0
                    self.buffer = []

    def process_packet(self):
        """Elabora il pacchetto completo"""
        # Verifica CRC
        packet_type = self.buffer[2]
        payload_length = self.buffer[3]

        # CRC calcolato su byte 2..5+payload_length-1
        crc_data = self.buffer[2:5+payload_length]
        calculated_crc = self.calculate_crc16(crc_data)

        # CRC ricevuto (little-endian)
        crc_received = self.buffer[5+payload_length] | (self.buffer[5+payload_length+1] << 8)

        if calculated_crc != crc_received:
            print(f"CRC Error! Got {crc_received:04X}, expected {calculated_crc:04X}")
            return

        # Parse basato su tipo
        if packet_type == 0x01:
            self.parse_navigation_packet()
        elif packet_type == 0x02:
            self.parse_debug_packet()
        elif packet_type == 0x03:
            self.parse_raw_sensor_packet()

    def calculate_crc16(self, data):
        """Implementazione CRC-16-CCITT"""
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
        """Decodifica Navigation Packet (0x01)"""
        payload = self.buffer[5:5+104]

        # Unpack usando struct (little-endian: '<')
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

        # Decode GPS integrity flags
        gps_jamming = bool(status_flags & 0x0800)    # Bit 11
        gps_spoofing = bool(status_flags & 0x1000)   # Bit 12

        # Stampa risultati
        print(f"Time: {timestamp_ms}ms")
        print(f"Attitude: R={roll:.3f}, P={pitch:.3f}, Y={yaw:.3f} rad")
        print(f"Position: Lat={latitude:.8f}°, Lon={longitude:.8f}°, Alt={altitude_msl:.1f}m")
        print(f"Velocity: V_N={velocity_north:.2f}, V_E={velocity_east:.2f}, V_D={velocity_down:.2f} m/s")
        print(f"Accel: {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f} m/s²")
        print(f"Status: 0x{status_flags:04X}")
        if gps_jamming:
            print("⚠️  GPS JAMMING DETECTED!")
        if gps_spoofing:
            print("⚠️  GPS SPOOFING SUSPECTED!")
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
            'gps_jamming': gps_jamming,
            'gps_spoofing': gps_spoofing,
        }

    def parse_debug_packet(self):
        """Decodifica Debug Packet (0x02) - Protocol v5"""
        payload = self.buffer[5:5+68]

        timestamp_ms = struct.unpack('<I', payload[0:4])[0]

        # Biases
        gyro_bias_x = struct.unpack('<f', payload[4:8])[0]
        gyro_bias_y = struct.unpack('<f', payload[8:12])[0]
        gyro_bias_z = struct.unpack('<f', payload[12:16])[0]

        accel_bias_z = struct.unpack('<f', payload[16:20])[0]

        baro_bias = struct.unpack('<f', payload[20:24])[0]
        mag_quality = struct.unpack('<f', payload[24:28])[0]

        # Hard Iron Bias (nT)
        hard_iron_x = struct.unpack('<f', payload[28:32])[0]
        hard_iron_y = struct.unpack('<f', payload[32:36])[0]
        hard_iron_z = struct.unpack('<f', payload[36:40])[0]

        # Timing
        loop_time_us = struct.unpack('<H', payload[40:42])[0]
        filter_time_us = struct.unpack('<H', payload[42:44])[0]
        sensor_time_us = struct.unpack('<H', payload[44:46])[0]
        max_loop_time_us = struct.unpack('<H', payload[46:48])[0]

        # GPS Accuracy
        gps_hacc = struct.unpack('<f', payload[48:52])[0]
        gps_vacc = struct.unpack('<f', payload[52:56])[0]

        # Sensor Counts
        baro_count = payload[56]
        gps_quality = payload[57]
        imu_fused = payload[58]

        # GPS Time
        gps_year = struct.unpack('<H', payload[60:62])[0]
        gps_month = payload[62]
        gps_day = payload[63]
        gps_hour = payload[64]
        gps_minute = payload[65]
        gps_second = payload[66]
        gps_time_valid = payload[67]

        print(f"Debug - Time: {timestamp_ms}ms")
        print(f"Gyro Bias: X={gyro_bias_x:.6f}, Y={gyro_bias_y:.6f}, Z={gyro_bias_z:.6f} rad/s")
        print(f"Accel Bias Z: {accel_bias_z:.4f} m/s²")
        print(f"Hard Iron: X={hard_iron_x:.0f}, Y={hard_iron_y:.0f}, Z={hard_iron_z:.0f} nT")
        print(f"Performance: Loop={loop_time_us}us, Filter={filter_time_us}us, Sensor={sensor_time_us}us")
```

---

## Configurazione

### Enablement in config.h

```c
// Formato telemetria
#define TELEMETRY_FORMAT_BINARY true        // true = binario, false = JSON
#define BINARY_NAV_RATE_HZ 50               // Frequenza Navigation packet (Hz)
#define BINARY_DEBUG_RATE_HZ 1              // Frequenza Debug packet (Hz)
#define BINARY_RAW_RATE_HZ 5                // Frequenza Raw Sensor packet (Hz)

// Destinazioni
#define TELEMETRY_ENABLE_SERIAL1 true       // Uscita su Serial1 (460,800 baud)
#define TELEMETRY_ENABLE_USB true          // Uscita su USB (115,200 baud)
```

### Calcolo della Banda Effettiva

**Default (50 Hz Navigation, 1 Hz Debug, 5 Hz Raw Sensor)**:
```
Navigation:  111 byte × 50 Hz = 5,550 byte/s = 44.4 kbps
Debug:        75 byte × 1 Hz  =    75 byte/s =  0.600 kbps
Raw Sensor:   81 byte × 5 Hz  =   405 byte/s =  3.240 kbps
Total:        ~48.2 kbps

Utilizzo del link @ 460,800 baud (57,600 byte/s):
Percentuale: 48.2 kbps / 460.8 kbps = 10.5%
Margine: 89.5% disponibile
```

**Configurazione High-Rate (100 Hz Navigation, 5 Hz Debug)**:
```
Navigation: 111 byte × 100 Hz = 11,100 byte/s = 88.8 kbps
Debug:      75 byte × 5 Hz = 375 byte/s = 3.00 kbps
Total:      ~91.8 kbps

Utilizzo: 91.8 / 460.8 = 19.9% (ancora molto margine)
```

---

## Esempi Pratici

### Esempio 1: Decodifica Manuale un Pacchetto Navigation

**Hex dump ricevuto**:
```
A5 5A 01 68 2A
D0 07 00 00          # timestamp_ms = 0x000007D0 = 2000 ms
D2 04 B0 FF 39 0C    # roll, pitch, yaw (scaled)
40 F0 1F 40 45 8C 80 3F  # latitude = 40.1234° (double)
60 66 D6 BF D6 9B 47 C2  # longitude = -122.6789° (double)
00 00 48 42          # altitude_msl = 50.0 m (float)
...resto payload...
XX XX                # CRC-16
```

**Decodifica**:
```
Byte 0-3:  D0 07 00 00 (little-endian) = 0x000007D0 = 2000 ms
Byte 4-5:  D2 04 (little-endian) = 0x04D2 = 1234 → 1234/1000 = 1.234 rad
Byte 6-7:  B0 FF (little-endian) = 0xFFB0 = -80 (signed) → -80/1000 = -0.080 rad
Byte 8-9:  39 0C (little-endian) = 0x0C39 = 3129 → 3129/1000 = 3.129 rad
```

### Esempio 2: Creazione Status Flags

**Scenario**: Filtro convergito, inizializzato, GPS disponibile, doppio IMU sano, barometro sano, modalità GPS+Mag, nessun jamming/spoofing

```c
uint16_t flags = 0;
flags |= (1 << 0);   // CONVERGED
flags |= (1 << 1);   // INITIALIZED
flags |= (1 << 3);   // GPS_AVAILABLE
flags |= (1 << 4);   // IMU1_HEALTHY
flags |= (1 << 5);   // IMU2_HEALTHY
flags |= (1 << 6);   // BARO_HEALTHY
flags |= (0 << 9) | (0 << 10);  // AIDING_MODE = 00 (GPS+Mag)
flags |= (0 << 11);  // GPS_JAMMING = no
flags |= (0 << 12);  // GPS_SPOOFING = no

// Risultato: 0x007B (0000 0000 0111 1011)
//
// Bit 15-13: 000 (reserved)
// Bit 12:    0 (GPS_SPOOFING = no)
// Bit 11:    0 (GPS_JAMMING = no)
// Bit 10-9:  00 (AIDING_MODE = GPS+Mag)
// Bit 8:     0 (GPS_BYPASS = no)
// Bit 7:     0 (MAG_HEALTHY = ... non specificato, assume 0)
// Bit 6:     1 (BARO_HEALTHY = yes)
// Bit 5:     1 (IMU2_HEALTHY = yes)
// Bit 4:     1 (IMU1_HEALTHY = yes)
// Bit 3:     1 (GPS_AVAILABLE = yes)
// Bit 2:     0 (ZUPT_ACTIVE = no)
// Bit 1:     1 (INITIALIZED = yes)
// Bit 0:     1 (CONVERGED = yes)
```

**Scenario con Jamming**: Se il GPS rileva jamming, il bit 11 sarà a 1:
```c
flags |= (1 << 11);  // GPS_JAMMING = yes
// Risultato: 0x087B (0000 1000 0111 1011)
```

### Esempio 3: Calcolo CRC per Piccolo Payload

**Dati**: `Type=0x01, Length=0x04, Seq=0x00, Payload=[0x12, 0x34, 0x56, 0x78]`

**Calcolo passo-passo** (funzione C):
```c
uint8_t crc_data[] = {0x01, 0x04, 0x00, 0x12, 0x34, 0x56, 0x78};
uint16_t crc = 0xFFFF;

// Byte 0: 0x01
crc ^= 0x01 << 8;  // crc = 0xFEFF
// 8 bit di processing...
crc = 0x... (dopo 8 bit)

// Byte 1: 0x04
// ... continua ...

// Risultato: crc = 0x... (valore dipende da implementazione)
```

Per verificare rapidamente, usare tool online come [CRC Calculator](https://crccalc.com/) con:
- Polynomial: 0x1021
- Initial: 0xFFFF
- Input Reflected: No
- Output Reflected: No

---

## Riepilogo Lunghezze

| Elemento | Byte |
|----------|------|
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
| **TOTALE Navigation Packet** | 111 |
| **TOTALE Debug Packet** | 71 |

---

## Conclusione

Questo protocollo telemetrico binario garantisce:

✓ **Efficienza**: 77.5% meno banda di JSON
✓ **Affidabilità**: CRC-16 per rilevamento errori
✓ **Precisione**: Scalamento intero per atteggiamento e accelerazione
✓ **Chiarezza**: Strutture ben definite e documentate
✓ **Compatibilità**: Facile implementazione su qualsiasi piattaforma

Per domande o problemi, consultare l'implementazione in:
- [telemetry_protocol.h](../src/telemetry_protocol.h)
- [telemetry_binary.h/cpp](../src/telemetry_binary.h)
