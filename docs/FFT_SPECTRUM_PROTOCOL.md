# IMU vibration spectrum over MAVLink

A worked example of every step, with figures and a real packet, is in [FFT_SPECTRUM_PROTOCOL.pdf](FFT_SPECTRUM_PROTOCOL.pdf).

How the Hybrid-PF firmware (ArduPilot `AP_HybridPF`, `HPF_FFT_ENABLE`) builds
the IMU vibration spectrum and packs it into MAVLink, so that any ground
station or tool can decode it, and any autopilot can produce a compatible
stream.

## Message

The spectrum travels in the standard MAVLink message
[`DEBUG_FLOAT_ARRAY`](https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY)
(id 350, `common.xml`). No dialect change is needed.

| Field | Type | Content |
|---|---|---|
| `time_usec` | `uint64_t` | Time of the newest IMU sample in the spectrum, µs since boot. The same for every message of one spectrum. |
| `name` | `char[10]` | `FFT_GYRO` or `FFT_ACC` (not null-terminated if 10 characters long; these are shorter). |
| `array_id` | `uint16_t` | `2 * sensor + part`: sensor 0 = gyroscope, 1 = accelerometer; part 0 or 1. |
| `data` | `float[58]` | 58 levels in dB (see below). MAVLink 2 extension field: always present in this stream. |

One spectrum of one sensor = **two messages** (part 0 and part 1) with the same
`time_usec`. A full update (gyro and accelerometer) = four messages:

| `array_id` | `name` | bins |
|---|---|---|
| 0 | `FFT_GYRO` | 0 – 57 |
| 1 | `FFT_GYRO` | 58 – 115 |
| 2 | `FFT_ACC` | 0 – 57 |
| 3 | `FFT_ACC` | 58 – 115 |

## Bins and frequencies

- Sample rate of the analysed signal: `fs = 800 Hz`
- FFT length: `N = 256`
- Bin spacing: `df = fs / N = 3.125 Hz`
- Bins sent: FFT bins 1 to 116 (DC is not sent)

Bin `i` of the assembled 116-value array (`i = 58 * part + k`, `k` the index in
`data`) is at

```
f(i) = (i + 1) * 3.125 Hz        i = 0 … 115   →   3.125 … 362.5 Hz
```

## Values

Each value is the one-sided power spectral density summed over the three axes
of the sensor, in decibels:

```
value = 10 * log10( PSD_x + PSD_y + PSD_z )
```

- Gyroscope: dB re 1 (rad/s)²/Hz
- Accelerometer: dB re 1 (m/s²)²/Hz
- Floor: values are clipped at −200 dB (power 1e−20)

To get the amplitude spectral density of the three axes together:
`ASD = 10^(value / 20)` in rad/s/√Hz or m/s²/√Hz.

## How the sender computes it

The IMU is read at the driver rate (about 1 kHz on the ESP32-S31 board, with
jitter: 880–950 Hz effective). Every 250 ms:

1. **Resample.** Take the last `(N − 1) / fs = 318.75 ms` of samples and
   interpolate them linearly, by their timestamps, on an even 800 Hz grid of
   256 points ending at the newest sample. If two consecutive samples are more
   than 20 ms apart inside the window, skip this spectrum.
2. **Detrend.** Remove the mean of each axis (gravity on the accelerometer,
   bias on the gyroscope).
3. **Window.** Multiply by a Hann window `w[n] = 0.5 (1 − cos(2πn / (N − 1)))`.
4. **FFT.** 256-point complex FFT of each axis (imaginary part zero).
5. **PSD.** For bins `k = 1 … 116`:
   `PSD_axis[k] = 2 |X[k]|² / (fs · Σ w[n]²)`
6. **Sum and convert.** Add the three axes, take `10 log10`.

The samples are those the navigation filter receives: calibrated, rotated to
the body frame, after the IMU's internal low-pass filter (68 Hz on the CORV
board). Above that cut-off the levels fall with the filter (4th order
Butterworth, about −24 dB per octave), but strong vibrations stay visible in dB.

## Timing and bandwidth

- One full update (4 messages) every ~250 ms.
- The sender emits **one message per main-loop cycle**, and only when every
  active link has room for it (it waits at most 100 ms for a full link), so
  the four messages of an update are spread over a few tens of ms. Do not
  expect them back to back.
- `DEBUG_FLOAT_ARRAY` is 252 bytes of payload, 264 with the MAVLink 2 header:
  about 4.3 kB/s on each link.

## Receiver: assembling a spectrum

```
on DEBUG_FLOAT_ARRAY msg:
    name = msg.name up to the first NUL
    sensor = msg.array_id >> 1          # 0 gyro, 1 accel
    part   = msg.array_id & 1
    if sensor > 1 or name != ["FFT_GYRO", "FFT_ACC"][sensor]: ignore
    pending = pending_for[sensor]
    if pending.time_usec != msg.time_usec: pending = new(time_usec), discard the old one
    pending.parts[part] = msg.data[0:58]
    if both parts present:
        spectrum = parts[0] + parts[1]   # 116 values, bin i at (i + 1) * 3.125 Hz
        publish(sensor, msg.time_usec, spectrum)
```

Rules:

- Join parts by `(sensor, time_usec)`. A part with a new `time_usec` starts a
  new spectrum: the previous one, incomplete, is dropped (one of its messages
  was lost).
- Parts can arrive in either order.
- A gap of more than about 1 s means the stream stopped (disabled, link lost,
  or the sender skipped spectra because of a gap in the IMU samples).
- `time_usec` is the autopilot's clock, not the GCS clock: use it to order and
  join messages, use the arrival time for display windows.

Python (pymavlink):

```python
pending = {}
def on_debug_float_array(m):
    name = m.name.split("\0")[0]
    sensor, part = m.array_id >> 1, m.array_id & 1
    if sensor > 1 or name != ("FFT_GYRO", "FFT_ACC")[sensor]:
        return None
    p = pending.get(sensor)
    if p is None or p["t"] != m.time_usec:
        p = pending[sensor] = {"t": m.time_usec, "parts": [None, None]}
    p["parts"][part] = list(m.data[:58])
    if None in p["parts"]:
        return None
    del pending[sensor]
    db = p["parts"][0] + p["parts"][1]
    freqs = [(i + 1) * 800 / 256 for i in range(116)]
    return sensor, m.time_usec, freqs, db
```

## Enabling it

On the vehicle: parameter `HPF_FFT_ENABLE = 1` (0 by default). The computing
thread starts at the first enable and runs until reboot; setting 0 stops the
messages only. Firmware built with `HAL_HYBRIDPF_SPECTRUM_ENABLED` (on by
default with the Hybrid-PF).

## Checking a receiver

With the board still on a bench, both spectra are flat up to the IMU filter
cut-off, at the sensor noise. With the SCH16T-K10 (gyro 6 m°/s/√Hz, accel
0.8 mm/s²/√Hz per axis, three axes summed):

- gyroscope about −75 dB re 1 (rad/s)²/Hz (measured on the CORV board: −77 dB
  from 5 to 60 Hz),
- accelerometer about −57 dB re 1 (m/s²)²/Hz (measured: −56 dB from 5 to
  40 Hz),

then falling above 68 Hz. A motor or propeller shows as a line at its rotation
frequency and harmonics.

## The same spectrum from a DataFlash log

Corv GCS computes spectra with this definition (mean removed, Hann window,
one-sided PSD `2 |X[k]|² / (fs · Σ w[n]²)`, three axes summed, dB) from the IMU
samples of a `.bin` log in replay, so a line reads the same live and from the
log. There N (256 – 8192), the time step and Welch averaging (K windows
overlapping by half, PSDs averaged) are set in SYS CONFIG → FFT FROM LOGS, fs is
the log's own rate, and bins 1 to N/2 − 1 are kept. GYR/ACC samples are taken
as evenly spaced (their `SampleUS` come in bursts) and a window never spans a
hole; IMU records at the loop rate are interpolated on an even grid like step 1
above. Implementation: `log-spectrum.js`.

## Reference implementation

- Sender: `libraries/AP_HybridPF/AP_HybridPF_Spectrum.{h,cpp}` in
  ardupilot-corv.
- Receiver: `js/core/SpectrumMonitor.js` (assembly) and `js/ui/SpectrumViews.js`
  (live chart and waterfall) in Corv GCS.
