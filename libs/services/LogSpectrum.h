// LogSpectrum.h - IMU vibration spectra computed from a DataFlash log (port of
// log-spectrum.js).
//
// Live, the vehicle sends its own spectrum (HPF_FFT_ENABLE, 256 points at 800 Hz,
// SpectrumMonitor) and nothing here is involved. A .bin in LOG REPLAY may hold
// the IMU samples themselves; this turns them into spectra with the resolution
// of SYS CONFIG → FFT FROM LOGS and streams them in step with the replay cursor.
//
// Sources in the log, per sensor and IMU instance:
//   GYR / ACC    every sample at the sensor rate (INS_RAW_LOG_OPT); GYR instances
//                past the gyro count are the post-filter copies
//   ISBH / ISBD  batch sampler (INS_LOG_BAT_MASK): INS_LOG_BAT_CNT contiguous
//                samples per batch, int16 scaled by ISBH.mul
//   IMU          the filtered sample at the loop rate
//
// The spectrum has the definition of the live one: mean removed, Hann window,
// one-sided PSD 2|X|² / (fs Σw²), three axes summed, in dB, clipped at −200 dB;
// the log adds Welch averaging (K windows of N samples overlapping by half). Bin
// i is at (i + 1) fs / N, i = 0 … N/2 − 2.
//
// GYR / ACC samples are taken as evenly spaced at the rate of their run (FIFO
// sensors stamp them in bursts); a run ends at a hole and a window never
// straddles one. IMU records are interpolated onto an even grid at their typical
// step, holes up to 4 steps bridged.
//
// No Qt event loop here: the scheduler sends through a callback and the owner
// calls pump() every 50 ms (LogReplay, on the log thread).
#pragma once

#include "services/DataFlash.h"
#include "vehicle/SpectrumMonitor.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <array>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace corv {

struct LogFftConfig {
    int n = 1024, avg = 1, stepMs = 250;
    bool operator==(const LogFftConfig &o) const { return n == o.n && avg == o.avg && stepMs == o.stepMs; }
};
/// Values off the lists (256 … 8192 points; 1, 2, 4, 8 averages; 100, 250, 500, 1000 ms) fall back to the defaults.
LogFftConfig sanitizeLogFftConfig(int n, int avg, int stepMs);

/// One IMU stream of the log on the replay's time base (ms), cut into runs of evenly spaced samples.
struct ImuStream {
    struct Run {
        int start = 0, count = 0;
        double fs = 0;
    };
    double fs = 0;
    std::vector<double> t;
    std::vector<float> x, y, z;
    std::vector<Run> runs;
    int longest = 0;
    QString key, label, kind;   // kind: raw, batch, loop
    int sensor = 0;             // 0 gyro, 1 accel
    int imu = 0;
    bool filtered = false;
};
using ImuStreamPtr = std::shared_ptr<const ImuStream>;

struct ImuStreams {
    std::array<std::vector<ImuStreamPtr>, 2> sensors;   // gyro, accel; best first
    QStringList dropped;
};

/// Collects the IMU records of a .bin as the parser meets them.
class ImuCollector
{
public:
    static bool wants(const QByteArray &name);
    /// One record: fmt is its FMT entry, its payload starts at p (whole record in the buffer).
    void take(const df::Format &fmt, const std::uint8_t *p);
    /// The streams found; minUs is time 0 of the replay, params the log's first PARM values.
    ImuStreams finish(double minUs, const QHash<QString, double> &params);

private:
    struct Samples {
        std::vector<double> t;
        std::vector<float> x, y, z;
        void push(double tt, float a, float b, float c)
        {
            t.push_back(tt);
            x.push_back(a);
            y.push_back(b);
            z.push_back(c);
        }
    };
    struct Cont {
        QString msg;
        int sensor = 0, inst = 0;
        Samples s;
    };
    struct BatchRun {
        double t0 = 0, fs = 0;
        std::vector<float> x, y, z;
    };
    struct Batch {
        int sensor = 0, inst = 0;
        std::vector<BatchRun> runs;
    };
    struct Open {
        QString key;
        int sensor = 0, inst = 0, n = 0, count = 0, mul = 1;
        double fs = 0, t0 = 0;
        std::map<int, std::array<std::array<float, 32>, 3>> chunks;
    };
    struct TypeCache {
        bool resolved = false;
        std::vector<int> off;
    };

    Cont &cont(const QString &msg, int sensor, int inst);
    void closeBatch();

    std::map<QString, std::unique_ptr<Cont>> m_cont;        // "GYR:0:0" (message, sensor, instance)
    std::vector<Cont *> m_contOrder;                        // as first met: the order ties keep
    std::map<QString, Batch> m_batches;                     // "ISB:1:0"
    std::vector<QString> m_batchOrder;
    std::optional<Open> m_isb;
    std::array<TypeCache, 256> m_cache;
    std::array<std::array<Cont *, 256>, 3> m_fast{};        // GYR, ACC, IMU-gyro per instance (IMU-accel below)
    std::array<Cont *, 256> m_fastImuAcc{};
};

/// Welch PSD of the three axes summed, in dB: K windows of N samples, half
/// overlapping, the last ending at sample `end` (inclusive). N/2 − 1 values.
std::vector<float> welchPsdDb(const ImuStream &st, int end, int N, int K, double fs);

struct FftResolution {
    int N = 0, K = 1;
    bool limited = false;
};
/// FFT size and averages that fit the stream's longest run: averages go first.
FftResolution fitResolution(int longest, const LogFftConfig &cfg);

class LogSpectrum
{
public:
    using Send = std::function<void(const LogSpectrumMsg &)>;
    using Now = std::function<double()>;

    explicit LogSpectrum(Send send) : m_send(std::move(send)) {}

    LogFftConfig setConfig(const LogFftConfig &c);
    LogFftConfig config() const { return m_cfg; }
    void selectStream(int sensor, const QString &key);
    /// A .bin log is loaded; nowMs() is the replay cursor in log ms.
    void load(const ImuStreams &streams, Now nowMs);
    /// The log is gone (unloaded, or a .tlog that carries the vehicle's own spectrum).
    void unload();
    bool loaded() const { return m_loaded; }
    /// Start over from 60 s before the cursor: load, seek, a new setting.
    void restart();
    /// Compute what is due, at most 25 ms of work.
    void pump();

private:
    struct Active {
        ImuStreamPtr st;
        FftResolution res;
        int lastEnd = -1;
    };

    Send m_send;
    LogFftConfig m_cfg;
    bool m_loaded = false;
    ImuStreams m_streams;
    std::array<QString, 2> m_choice{QStringLiteral("auto"), QStringLiteral("auto")};
    Now m_now = [] { return 0.0; };
    int m_gen = 0;
    std::array<std::optional<Active>, 2> m_active;
    double m_nextT = 0, m_lastNow = 0;
};

} // namespace corv
