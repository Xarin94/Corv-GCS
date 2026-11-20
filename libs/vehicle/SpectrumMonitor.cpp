#include "vehicle/SpectrumMonitor.h"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr double kPeakHoldMs = 10000;
constexpr double kStaleMs = 2000;
constexpr double kMinLineHz = 5;   // lines are looked for above this (and above the first bins)
} // namespace

QString fmtDf(double v) { return (v >= 10 ? QString::number(v, 'f', 1) : v >= 1 ? QString::number(v, 'f', 2) : QString::number(v, 'f', 3)) + u" Hz"_s; }

QString fmtDur(double ms)
{
    return ms >= 10000 ? QString::number(ms / 1000, 'f', 0) + u" s"_s
         : ms >= 1000 ? QString::number(ms / 1000, 'f', 1) + u" s"_s
                      : QString::number(std::lround(ms)) + u" ms"_s;
}

int SpectrumGrid::firstLineBin() const
{
    return std::min(n - 1, std::max(1, int(std::ceil(kMinLineHz / df)) - 1));
}

// ── One sensor ───────────────────────────────────────────────────────────

void SensorSpectra::reset(const SpectrumGrid &grid)
{
    m_grid = grid;
    m_rows.clear();
    m_pendingUsec.reset();
    m_parts = {};
    ++m_version;
    m_hold.reset();
}

void SensorSpectra::add(std::uint64_t usec, int part, const float *data, double now)
{
    if (part < 0 || part > 1) return;
    if (!m_pendingUsec || *m_pendingUsec != usec) {
        m_pendingUsec = usec;
        m_parts = {};
    }
    m_parts[std::size_t(part)].assign(data, data + kFftPartBins);
    if (m_parts[0].empty() || m_parts[1].empty()) return;
    auto db = std::make_shared<std::vector<float>>(m_parts[0]);
    db->insert(db->end(), m_parts[1].begin(), m_parts[1].end());
    m_pendingUsec.reset();
    m_parts = {};
    push(now, std::move(db));
}

void SensorSpectra::push(double t, std::shared_ptr<const std::vector<float>> db)
{
    if (!db || int(db->size()) != m_grid.n) return;
    if (!m_rows.empty()) {
        const double prev = m_rows.back().t;
        if (t < prev) {
            // the replay clock went back: those spectra are of another time
            if (t < prev - 1000) m_rows.clear();
            else t = prev;                                // jitter of the interpolated cursor
        }
    }
    m_rows.push_back({t, std::move(db)});
    const double cut = t - kSpectrumHistoryMs;
    while (!m_rows.empty() && m_rows.front().t < cut) m_rows.pop_front();
    ++m_version;
}

double SensorSpectra::holdMs() const
{
    if (m_hold && m_hold->first == m_version) return m_hold->second;
    std::vector<double> gaps;
    gaps.reserve(m_rows.size());
    for (std::size_t i = 1; i < m_rows.size(); ++i) gaps.push_back(m_rows[i].t - m_rows[i - 1].t);
    std::sort(gaps.begin(), gaps.end());
    const double typical = gaps.empty() ? 250 : gaps[gaps.size() >> 1];
    const double ms = std::min(10000.0, std::max(1000.0, 1.5 * typical));
    m_hold = std::make_pair(m_version, ms);
    return ms;
}

std::optional<SensorSpectra::Dominant> SensorSpectra::dominant(double now, double windowMs, int n, double minProm) const
{
    const SpectrumGrid &g = m_grid;
    const int N = g.n;
    const int floorBins = std::min(64, std::max(4, int(std::lround(25 / g.df))));
    const double span = std::max(windowMs, 2 * holdMs());
    std::vector<const Row *> recent;
    for (const Row &r : m_rows)
        if (r.t >= now - span && r.t <= now) recent.push_back(&r);
    if (recent.empty()) return std::nullopt;
    auto db = std::vector<double>(std::size_t(N));
    for (int i = 0; i < N; ++i) {
        double p = 0;
        for (const Row *r : recent) p += std::pow(10.0, (*r->db)[std::size_t(i)] / 10);
        db[std::size_t(i)] = 10 * std::log10(p / double(recent.size()));
    }
    const int first = g.firstLineBin();
    const auto localFloor = [&](int i) {
        std::vector<double> v;
        for (int k = std::max(first, i - floorBins); k <= std::min(N - 1, i + floorBins); ++k)
            if (std::abs(k - i) > 1) v.push_back(db[std::size_t(k)]);
        std::sort(v.begin(), v.end());
        return v.empty() ? db[std::size_t(i)] : v[v.size() / 2];
    };
    std::vector<Line> peaks;
    int strongest = first;
    for (int i = first; i < N; ++i) {
        if (db[std::size_t(i)] > db[std::size_t(strongest)]) strongest = i;
        const double left = i > 0 ? db[std::size_t(i - 1)] : -std::numeric_limits<double>::infinity();
        const double right = i + 1 < N ? db[std::size_t(i + 1)] : -std::numeric_limits<double>::infinity();
        if (db[std::size_t(i)] >= left && db[std::size_t(i)] > right) {
            const double prom = db[std::size_t(i)] - localFloor(i);
            if (prom >= minProm) peaks.push_back({g.binFreq(i), db[std::size_t(i)], prom});
        }
    }
    std::sort(peaks.begin(), peaks.end(), [](const Line &a, const Line &b) { return a.db > b.db; });
    if (int(peaks.size()) > n) peaks.resize(std::size_t(n));
    return Dominant{localFloor(strongest), {g.binFreq(strongest), db[std::size_t(strongest)]}, peaks};
}

std::optional<std::vector<float>> SensorSpectra::peakHold(double now) const
{
    std::vector<float> out(std::size_t(m_grid.n), -std::numeric_limits<float>::infinity());
    bool any = false;
    for (auto it = m_rows.rbegin(); it != m_rows.rend() && it->t >= now - kPeakHoldMs; ++it) {
        if (it->t > now) continue;
        const auto &db = *it->db;
        for (std::size_t i = 0; i < db.size() && i < out.size(); ++i) out[i] = std::max(out[i], db[i]);
        any = true;
    }
    return any ? std::optional<std::vector<float>>(std::move(out)) : std::nullopt;
}

// ── Monitor ──────────────────────────────────────────────────────────────

double SpectrumMonitor::clock(double perfNow) const
{
    if (!m_replay) return perfNow;
    return m_replay->playing ? m_replay->ms + (perfNow - m_replay->perf) : m_replay->ms;
}

void SpectrumMonitor::ingest(const QString &name, int arrayId, std::uint64_t timeUsec, const float *data, int len, double now)
{
    if (m_log) return;   // a .bin's spectra come from its samples
    const int sensor = arrayId >> 1;
    if (sensor < 0 || sensor > 1) return;
    static const QString names[2] = {u"FFT_GYRO"_s, u"FFT_ACC"_s};
    if (name.trimmed() != names[sensor] || len < kFftPartBins) return;
    m_sensors[std::size_t(sensor)].add(timeUsec, arrayId & 1, data, now);
}

void SpectrumMonitor::onLogSpectrum(const LogSpectrumMsg &m)
{
    if (m.reset) {
        m_gen = m.gen;
        m_log = m.log;
        reset();
    }
    if (m.gen != m_gen) return;
    for (const LogSpectrumRow &r : m.rows)
        if (r.sensor == 0 || r.sensor == 1) m_sensors[std::size_t(r.sensor)].push(r.t, r.db);
}

void SpectrumMonitor::replayLoaded(const QString &format, double perfNow)
{
    if (m_replay) return;
    m_replay = Replay{format, 0, perfNow, false};
    reset();
}

void SpectrumMonitor::replayUnloaded()
{
    if (!m_replay) return;
    m_replay.reset();
    m_log.reset();
    reset();
}

void SpectrumMonitor::replayTick(double currentMs, bool playing, double perfNow)
{
    if (!m_replay) return;
    m_replay->ms = currentMs;
    m_replay->perf = perfNow;
    m_replay->playing = playing;
}

void SpectrumMonitor::replaySeek()
{
    if (m_replay) resetRows();   // the old spectra are of another time
}

SpectrumGrid SpectrumMonitor::gridFor(int sensor) const
{
    if (m_log) {
        const auto &l = m_log->sensors[std::size_t(sensor)];
        if (l.n > 0) return {l.df, l.n};
    }
    return SpectrumGrid{};
}

void SpectrumMonitor::reset()
{
    for (int s = 0; s < 2; ++s) m_sensors[std::size_t(s)].reset(gridFor(s));
    ++m_version;
}

void SpectrumMonitor::resetRows()
{
    for (auto &s : m_sensors) s.reset();
}

bool SpectrumMonitor::live(int sensor, double now) const
{
    const SensorSpectra &s = m_sensors[std::size_t(sensor)];
    const auto *l = s.latest();
    return l && now - l->t < std::max(kStaleMs, 2 * s.holdMs());
}

SpectrumMonitor::Source SpectrumMonitor::source(int sensor) const
{
    if (m_log) {
        const auto &l = m_log->sensors[std::size_t(sensor)];
        if (l.stream.isEmpty())
            return {u"log"_s, u"LOG · NO IMU SAMPLES"_s, u"no IMU samples in the log"_s, {}, {}};
        const QString avg = l.avg > 1 ? u" × %1 avg"_s.arg(l.avg) : QString();
        const QString lim = l.limited ? u" (cut to the %1-sample runs)"_s.arg(l.longest) : QString();
        return {u"log"_s,
                u"LOG · %1 pt%2 · %3 · %4 window%5"_s.arg(l.N).arg(avg, fmtDf(l.df), fmtDur(l.windowMs), lim),
                u"%1 of samples needed for a spectrum"_s.arg(fmtDur(l.windowMs)), l.streams, l.stream};
    }
    if (m_replay)
        return {u"tlog"_s, u"LOG · VEHICLE SPECTRUM · %1 pt at %2 Hz · %3"_s.arg(kFftN).arg(kFftRateHz).arg(fmtDf(kFftDf)),
                u"no FFT_GYRO / FFT_ACC in this log"_s, {}, {}};
    return {u"vehicle"_s, u"VEHICLE · %1 pt at %2 Hz · %3"_s.arg(kFftN).arg(kFftRateHz).arg(fmtDf(kFftDf)),
            u"HPF_FFT_ENABLE = 1 on the vehicle"_s, {}, {}};
}

} // namespace corv
