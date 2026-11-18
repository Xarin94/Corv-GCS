// SpectrumMonitor.h - IMU vibration spectra, from the vehicle or from a log
// (port of js/core/SpectrumMonitor.js).
//
// Live: the CORV firmware (HPF_FFT_ENABLE = 1) sends, four times a
// second, the power spectral density of the IMU samples as DEBUG_FLOAT_ARRAY
// (350): name "FFT_GYRO" or "FFT_ACC", array_id = 2 * sensor + part, 58 bins a
// part, 116 bins from 3.125 to 362.5 Hz (bin i at (i + 1) * 800 / 256 Hz), in dB
// of (rad/s)²/Hz for the gyro and (m/s²)²/Hz for the accelerometer, three axes
// summed. The two parts of a spectrum share time_usec; a spectrum is kept once
// both have arrived.
//
// LOG REPLAY: a .tlog replays the same messages. A .bin may hold the IMU samples
// themselves: LogSpectrum (libs/services) computes the spectra from them with
// the resolution of SYS CONFIG → FFT FROM LOGS and sends them here, same
// definition and units, bin i at (i + 1) * df with the log's df.
//
// Time: live, the arrival time (monoMs()); in a replay, the replay cursor in log
// ms, so the views stop when it is paused and follow a seek. The last 60 s of
// each sensor feed the waterfall, the newest the live chart, a 10 s peak hold
// rides over it; the strongest lines of the last 2 s head the FFT bar.
// GUI thread.
#pragma once

#include <QMetaType>
#include <QString>
#include <QStringList>
#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

namespace corv {

constexpr double kFftRateHz = 800;
constexpr int kFftN = 256;
constexpr int kFftPartBins = 58;
constexpr int kFftBins = 2 * kFftPartBins;
constexpr double kFftDf = kFftRateHz / kFftN;
constexpr double kSpectrumHistoryMs = 60000;

/// A spectrum's bins: n of them, bin i at (i + 1) * df Hz.
struct SpectrumGrid {
    double df = kFftDf;
    int n = kFftBins;
    double binFreq(int i) const { return (i + 1) * df; }
    double maxHz() const { return n * df; }
    /// First bin a line can be in: above 5 Hz and past the window's leakage of the mean.
    int firstLineBin() const;
    bool operator==(const SpectrumGrid &o) const { return df == o.df && n == o.n; }
};

/// What LogSpectrum sends: a reset with the log's description, or rows.
struct LogSpectrumInfo {
    struct Stream {
        QString key, label;
    };
    struct Sensor {
        std::vector<Stream> streams;
        QString stream, choice, label;   // stream empty: no samples for this sensor
        double fs = 0, df = 0, windowMs = 0;
        int N = 0, avg = 1, n = 0, longest = 0;
        bool limited = false;
    };
    int cfgN = 1024, cfgAvg = 1, cfgStepMs = 250;
    std::array<Sensor, 2> sensors;   // gyro, accel
    QStringList dropped;
};
struct LogSpectrumRow {
    int sensor = 0;                  // 0 gyro, 1 accel
    double t = 0;                    // log ms
    std::shared_ptr<const std::vector<float>> db;
};
struct LogSpectrumMsg {
    int gen = 0;
    bool reset = false;
    std::shared_ptr<const LogSpectrumInfo> log;   // with a reset: null = no log
    std::vector<LogSpectrumRow> rows;
};

class SensorSpectra
{
public:
    struct Row {
        double t;
        std::shared_ptr<const std::vector<float>> db;
    };
    struct Line {
        double f, db, prom = 0;
    };
    struct Dominant {
        double floor;
        Line max;
        std::vector<Line> peaks;
    };

    SensorSpectra() { reset(); }
    void reset(const SpectrumGrid &grid);
    void reset() { reset(m_grid); }
    /// One part of a vehicle spectrum.
    void add(std::uint64_t usec, int part, const float *data, double now);
    /// A whole spectrum on this sensor's grid, at time t.
    void push(double t, std::shared_ptr<const std::vector<float>> db);

    const SpectrumGrid &grid() const { return m_grid; }
    const std::deque<Row> &rows() const { return m_rows; }
    const Row *latest() const { return m_rows.empty() ? nullptr : &m_rows.back(); }
    int version() const { return m_version; }
    /// How long a spectrum stands for: 1 s for the vehicle's 4 Hz, more for a log's batches.
    double holdMs() const;
    /// Strongest lines over the last windowMs: local maxima minProm dB over the
    /// local floor (median within ±25 Hz), up to n, strongest first.
    std::optional<Dominant> dominant(double now, double windowMs = 2000, int n = 3, double minProm = 6) const;
    /// Highest level per bin over the last 10 s.
    std::optional<std::vector<float>> peakHold(double now) const;

private:
    SpectrumGrid m_grid;
    std::deque<Row> m_rows;
    std::optional<std::uint64_t> m_pendingUsec;
    std::array<std::vector<float>, 2> m_parts;
    int m_version = 0;
    mutable std::optional<std::pair<int, double>> m_hold;   // version, ms
};

class SpectrumMonitor
{
public:
    enum Sensor { Gyro = 0, Accel = 1 };
    struct Source {
        QString kind;    // vehicle, tlog, log
        QString text, hint;
        std::vector<LogSpectrumInfo::Stream> streams;
        QString stream;
    };

    SensorSpectra &get(int sensor) { return m_sensors[std::size_t(sensor)]; }
    const SensorSpectra &get(int sensor) const { return m_sensors[std::size_t(sensor)]; }

    /// Now on the spectra's time base: arrival time live, the cursor in a replay. perfNow = monoMs().
    double clock(double perfNow) const;
    /// DEBUG_FLOAT_ARRAY from the vehicle (ignored while a .bin's spectra are in use).
    void ingest(const QString &name, int arrayId, std::uint64_t timeUsec, const float *data, int len, double now);
    void onLogSpectrum(const LogSpectrumMsg &m);

    // Replay
    void replayLoaded(const QString &format, double perfNow);
    void replayUnloaded();
    void replayTick(double currentMs, bool playing, double perfNow);
    void replaySeek();
    bool replaying() const { return m_replay.has_value(); }
    const LogSpectrumInfo *log() const { return m_log.get(); }

    /// A new link: the spectra start empty.
    void reset();
    bool live(int sensor, double now) const;
    Source source(int sensor) const;
    int version() const { return m_version; }

private:
    SpectrumGrid gridFor(int sensor) const;
    void resetRows();

    std::array<SensorSpectra, 2> m_sensors;
    struct Replay {
        QString format;
        double ms = 0, perf = 0;
        bool playing = false;
    };
    std::optional<Replay> m_replay;
    std::shared_ptr<const LogSpectrumInfo> m_log;
    int m_gen = -1;
    int m_version = 0;
};

} // namespace corv

Q_DECLARE_METATYPE(corv::LogSpectrumMsg)
