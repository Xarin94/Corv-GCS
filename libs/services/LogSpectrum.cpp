#include "services/LogSpectrum.h"

#include <QElapsedTimer>
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace corv {

// ── Settings ─────────────────────────────────────────────────────────────

LogFftConfig sanitizeLogFftConfig(int n, int avg, int stepMs)
{
    const auto pick = [](int v, std::initializer_list<int> list, int def) { return std::find(list.begin(), list.end(), v) != list.end() ? v : def; };
    return {pick(n, {256, 512, 1024, 2048, 4096, 8192}, 1024), pick(avg, {1, 2, 4, 8}, 1), pick(stepMs, {100, 250, 500, 1000}, 250)};
}

// ── Sample collection (fed by the .bin parser) ───────────────────────────

namespace {

constexpr int kIsbAccel = 0, kIsbGyro = 1;     // ISBH.type (AP_InertialSensor::IMU_SENSOR_TYPE)
constexpr double kGapFactor = 3;               // a step over 3 typical steps ends a run
constexpr int kTypicalSpan = 8;                // typical step from spans of 8 steps (bursts average out)
constexpr double kTypicalQuantile = 0.2;       // … their 20th percentile, clear of the holes
constexpr double kMinRunRate = 0.75;           // a run slower than this × the typical rate hides holes
constexpr double kResampleGap = 4;             // IMU at the loop rate: steps up to 4 typical ones are interpolated
constexpr int kMinRun = 64;                    // shorter runs are dropped

const char *const kGyrFields[] = {"TimeUS", "SampleUS", "I", "GyrX", "GyrY", "GyrZ"};
const char *const kAccFields[] = {"TimeUS", "SampleUS", "I", "AccX", "AccY", "AccZ"};
const char *const kImuFields[] = {"TimeUS", "I", "GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"};
const char *const kIsbhFields[] = {"N", "type", "instance", "mul", "smp_cnt", "SampleUS", "smp_rate"};
const char *const kIsbdFields[] = {"N", "seqno", "x", "y", "z"};

float rf(const std::uint8_t *p) { return qFromLittleEndian<float>(p); }

double quantile(std::vector<double> a, double q)
{
    if (a.empty()) return 0;
    std::sort(a.begin(), a.end());
    return a[std::min(a.size() - 1, std::size_t(std::floor(q * double(a.size()))))];
}

using StreamPtr = std::shared_ptr<ImuStream>;

/// A stream sampled continuously, cut into runs of evenly spaced samples. The
/// typical step comes from spans of 8 steps (FIFO bursts) and their 20th
/// percentile (a logger that cannot keep up leaves holes in most spans).
StreamPtr continuousStream(const std::vector<double> &T, const std::vector<float> &X, const std::vector<float> &Y,
                           const std::vector<float> &Z, double minUs)
{
    const int n = int(T.size());
    if (n < kMinRun) return nullptr;
    std::vector<double> sample;
    const int stride = std::max(1, n / 8000);
    for (int i = kTypicalSpan; i < n; i += stride) {
        const double d = (T[std::size_t(i)] - T[std::size_t(i - kTypicalSpan)]) / kTypicalSpan;
        if (d > 0) sample.push_back(d);
    }
    const double step = quantile(sample, kTypicalQuantile);
    if (!(step > 0)) return nullptr;
    const double gap = kGapFactor * step;
    std::vector<ImuStream::Run> runs;
    std::vector<double> spans;
    int start = 0;
    for (int i = 1; i <= n; ++i) {
        const double d = i < n ? T[std::size_t(i)] - T[std::size_t(i - 1)] : INFINITY;
        if (d < 0 || d > gap) {
            const int count = i - start;
            const double span = T[std::size_t(i - 1)] - T[std::size_t(start)];
            if (count >= kMinRun && span > 0) {
                const double fs = (count - 1) * 1e6 / span;
                if (fs >= kMinRunRate * 1e6 / step) {
                    runs.push_back({start, count, fs});
                    spans.push_back(span);
                }
            }
            start = i;
        }
    }
    if (runs.empty()) return nullptr;
    double num = 0, den = 0;
    for (std::size_t k = 0; k < runs.size(); ++k) {
        num += runs[k].count - 1;
        den += spans[k];
    }
    auto st = std::make_shared<ImuStream>();
    st->fs = num * 1e6 / den;
    st->t.resize(std::size_t(n));
    for (int i = 0; i < n; ++i) st->t[std::size_t(i)] = (T[std::size_t(i)] - minUs) / 1000;
    st->x = X;
    st->y = Y;
    st->z = Z;
    st->runs = std::move(runs);
    for (const auto &r : st->runs) st->longest = std::max(st->longest, r.count);
    return st;
}

/// IMU records at the loop rate, interpolated onto an even grid at their typical
/// (median) step. A step over 4 typical ones, or a negative one, ends a run.
StreamPtr resampledStream(const std::vector<double> &T, const std::vector<float> &X, const std::vector<float> &Y,
                          const std::vector<float> &Z, double minUs)
{
    const int n = int(T.size());
    if (n < kMinRun) return nullptr;
    std::vector<double> sample;
    const int stride = std::max(1, n / 8000);
    for (int i = 1; i < n; i += stride) {
        const double d = T[std::size_t(i)] - T[std::size_t(i - 1)];
        if (d > 0) sample.push_back(d);
    }
    const double step = quantile(sample, 0.5);
    if (!(step > 0)) return nullptr;
    struct Seg {
        int a, b, count;
    };
    std::vector<Seg> segs;
    int start = 0;
    for (int i = 1; i <= n; ++i) {
        const double d = i < n ? T[std::size_t(i)] - T[std::size_t(i - 1)] : INFINITY;
        if (d < 0 || d > kResampleGap * step) {
            const int count = int(std::floor((T[std::size_t(i - 1)] - T[std::size_t(start)]) / step)) + 1;
            if (count >= kMinRun) segs.push_back({start, i - 1, count});
            start = i;
        }
    }
    if (segs.empty()) return nullptr;
    int total = 0;
    for (const Seg &g : segs) total += g.count;
    auto st = std::make_shared<ImuStream>();
    st->t.resize(std::size_t(total));
    st->x.resize(std::size_t(total));
    st->y.resize(std::size_t(total));
    st->z.resize(std::size_t(total));
    int o = 0;
    for (const Seg &g : segs) {
        const double t0 = T[std::size_t(g.a)];
        int j = g.a;
        for (int k = 0; k < g.count; ++k) {
            const double tk = t0 + k * step;
            while (j < g.b && T[std::size_t(j + 1)] <= tk) ++j;
            const int j1 = std::min(j + 1, g.b);
            const double span = T[std::size_t(j1)] - T[std::size_t(j)];
            const double f = span > 0 ? std::min(1.0, std::max(0.0, (tk - T[std::size_t(j)]) / span)) : 0;
            const auto at = std::size_t(o + k);
            st->t[at] = (tk - minUs) / 1000;
            st->x[at] = float(X[std::size_t(j)] + f * (X[std::size_t(j1)] - X[std::size_t(j)]));
            st->y[at] = float(Y[std::size_t(j)] + f * (Y[std::size_t(j1)] - Y[std::size_t(j)]));
            st->z[at] = float(Z[std::size_t(j)] + f * (Z[std::size_t(j1)] - Z[std::size_t(j)]));
        }
        st->runs.push_back({o, g.count, 1e6 / step});
        o += g.count;
    }
    st->fs = 1e6 / step;
    for (const auto &r : st->runs) st->longest = std::max(st->longest, r.count);
    return st;
}

void describe(ImuStream &st, const QString &key, int sensor, const QString &kind, int imu, bool filtered)
{
    const QString what = kind == QLatin1String("loop") ? QStringLiteral("loop rate")
                         : (kind == QLatin1String("batch") ? QStringLiteral("batch") : QStringLiteral("raw")) + (filtered ? QStringLiteral(" filtered") : QString());
    const QString rate = st.fs >= 100 ? QString::number(std::lround(st.fs)) : QString::number(st.fs, 'f', 1);
    st.label = QStringLiteral("IMU%1 · %2 · %3 Hz").arg(imu + 1).arg(kind == QLatin1String("raw") && filtered ? QStringLiteral("filtered") : what, rate);
    st.key = key;
    st.sensor = sensor;
    st.kind = kind;
    st.imu = imu;
    st.filtered = filtered;
}

} // namespace

bool ImuCollector::wants(const QByteArray &name)
{
    return name == "GYR" || name == "ACC" || name == "IMU" || name == "ISBH" || name == "ISBD";
}

ImuCollector::Cont &ImuCollector::cont(const QString &msg, int sensor, int inst)
{
    const QString key = QStringLiteral("%1:%2:%3").arg(msg).arg(sensor).arg(inst);
    auto &slot = m_cont[key];
    if (!slot) {
        slot = std::make_unique<Cont>();
        slot->msg = msg;
        slot->sensor = sensor;
        slot->inst = inst;
        m_contOrder.push_back(slot.get());
    }
    return *slot;
}

void ImuCollector::take(const df::Format &fmt, const std::uint8_t *p)
{
    TypeCache &c = m_cache[fmt.type];
    if (!c.resolved) {
        c.resolved = true;
        const auto resolve = [&](const char *const *names, int count) {
            for (int i = 0; i < count; ++i) c.off.push_back(fmt.offsetOf(names[i]));
        };
        if (fmt.name == "GYR") resolve(kGyrFields, 6);
        else if (fmt.name == "ACC") resolve(kAccFields, 6);
        else if (fmt.name == "IMU") resolve(kImuFields, 8);
        else if (fmt.name == "ISBH") resolve(kIsbhFields, 7);
        else if (fmt.name == "ISBD") resolve(kIsbdFields, 5);
    }
    const auto &o = c.off;
    if (fmt.name == "GYR" || fmt.name == "ACC") {
        const int ts = o[0], su = o[1], ins = o[2], fx = o[3], fy = o[4], fz = o[5];
        if (fz < 0 || fx < 0 || fy < 0 || (su < 0 && ts < 0)) return;
        const int inst = ins >= 0 ? p[ins] : 0;
        const double t = df::u64(p + (su >= 0 ? su : ts));
        const bool gyr = fmt.name == "GYR";
        Cont *&fast = m_fast[gyr ? 0 : 1][std::size_t(inst)];
        if (!fast) fast = &cont(fmt.name == "GYR" ? QStringLiteral("GYR") : QStringLiteral("ACC"), gyr ? 0 : 1, inst);
        fast->s.push(t, rf(p + fx), rf(p + fy), rf(p + fz));
    } else if (fmt.name == "IMU") {
        const int ts = o[0], ins = o[1], gx = o[2], gy = o[3], gz = o[4], ax = o[5], ay = o[6], az = o[7];
        if (ts < 0 || gz < 0 || az < 0 || gx < 0 || gy < 0 || ax < 0 || ay < 0) return;
        const int inst = ins >= 0 ? p[ins] : 0;
        const double t = df::u64(p + ts);
        Cont *&g = m_fast[2][std::size_t(inst)];
        if (!g) g = &cont(QStringLiteral("IMU"), 0, inst);
        Cont *&a = m_fastImuAcc[std::size_t(inst)];
        if (!a) a = &cont(QStringLiteral("IMU"), 1, inst);
        g->s.push(t, rf(p + gx), rf(p + gy), rf(p + gz));
        a->s.push(t, rf(p + ax), rf(p + ay), rf(p + az));
    } else if (fmt.name == "ISBH") {
        for (int v : o)
            if (v < 0) return;
        const int n = o[0], type = o[1], ins = o[2], mul = o[3], cnt = o[4], su = o[5], rate = o[6];
        closeBatch();
        const int sensorType = p[type];
        const double fs = rf(p + rate);
        const int m = qFromLittleEndian<std::uint16_t>(p + mul);
        if ((sensorType != kIsbAccel && sensorType != kIsbGyro) || !(fs > 0) || !m) return;
        const int inst = ins >= 0 ? p[ins] : 0;
        Open b;
        b.key = QStringLiteral("ISB:%1:%2").arg(sensorType).arg(inst);
        b.sensor = sensorType == kIsbGyro ? 0 : 1;
        b.inst = inst;
        b.n = qFromLittleEndian<std::uint16_t>(p + n);
        b.count = qFromLittleEndian<std::uint16_t>(p + cnt);
        b.fs = fs;
        b.mul = m;
        b.t0 = df::u64(p + su);
        m_isb = std::move(b);
    } else if (fmt.name == "ISBD") {
        if (!m_isb) return;
        for (int v : o)
            if (v < 0) return;
        const int n = o[0], seq = o[1];
        if (qFromLittleEndian<std::uint16_t>(p + n) != m_isb->n) return;
        std::array<std::array<float, 32>, 3> d{};
        for (int axis = 0; axis < 3; ++axis)
            for (int k = 0; k < 32; ++k)
                d[std::size_t(axis)][std::size_t(k)] = float(qFromLittleEndian<std::int16_t>(p + o[2 + axis] + 2 * k)) / float(m_isb->mul);
        m_isb->chunks[qFromLittleEndian<std::uint16_t>(p + seq)] = d;
    }
}

/// The batch in assembly goes in as its runs of consecutive chunks.
void ImuCollector::closeBatch()
{
    if (!m_isb) return;
    Open b = std::move(*m_isb);
    m_isb.reset();
    if (b.chunks.empty()) return;
    auto it = m_batches.find(b.key);
    if (it == m_batches.end()) {
        it = m_batches.emplace(b.key, Batch{b.sensor, b.inst, {}}).first;
        m_batchOrder.push_back(b.key);
    }
    const int chunks = b.count / 32;
    int k = 0;
    while (k < chunks) {
        if (!b.chunks.count(k)) {
            ++k;
            continue;
        }
        const int start = k;
        while (k < chunks && b.chunks.count(k)) ++k;
        const int len = (k - start) * 32;
        if (len < kMinRun) continue;
        BatchRun run;
        run.t0 = b.t0 + start * 32 * 1e6 / b.fs;
        run.fs = b.fs;
        for (int ch = start; ch < k; ++ch) {
            const auto &d = b.chunks[ch];
            run.x.insert(run.x.end(), d[0].begin(), d[0].end());
            run.y.insert(run.y.end(), d[1].begin(), d[1].end());
            run.z.insert(run.z.end(), d[2].begin(), d[2].end());
        }
        it->second.runs.push_back(std::move(run));
    }
}

ImuStreams ImuCollector::finish(double minUs, const QHash<QString, double> &params)
{
    closeBatch();
    const auto count = [&](const QString &prefix) {
        int found = 0, nonzero = 0;
        for (const QString &k : {prefix + QStringLiteral("_ID"), prefix + QStringLiteral("2_ID"), prefix + QStringLiteral("3_ID")}) {
            if (!params.contains(k)) continue;
            ++found;
            if (params.value(k) != 0) ++nonzero;
        }
        return found ? nonzero : 0;
    };
    const int nGyro = count(QStringLiteral("INS_GYR")), nAcc = count(QStringLiteral("INS_ACC"));
    const int rawOpt = int(params.value(QStringLiteral("INS_RAW_LOG_OPT"), 0)), batOpt = int(params.value(QStringLiteral("INS_LOG_BAT_OPT"), 0));
    ImuStreams out;
    std::array<std::vector<StreamPtr>, 2> lists;

    for (Cont *c : m_contOrder) {
        StreamPtr st = c->msg == QLatin1String("IMU") ? resampledStream(c->s.t, c->s.x, c->s.y, c->s.z, minUs)
                                                      : continuousStream(c->s.t, c->s.x, c->s.y, c->s.z, minUs);
        if (!st) {
            // e.g. an 8 kHz gyro the logger could only keep in bursts
            const QString name = QStringLiteral("%1 %2").arg(c->msg).arg(c->inst);
            if (int(c->s.t.size()) >= kMinRun && !out.dropped.contains(name)) out.dropped << name;
            continue;
        }
        int imu = c->inst;
        bool filtered;
        QString kind;
        if (c->msg == QLatin1String("IMU")) {
            kind = QStringLiteral("loop");
            filtered = true;
        } else {
            kind = QStringLiteral("raw");
            const int nSensors = c->sensor == 0 ? nGyro : nAcc;
            if (nSensors && c->inst >= nSensors) {
                imu = c->inst - nSensors;
                filtered = true;
            } else {
                filtered = c->sensor == 0 && (rawOpt & 4) && !(rawOpt & 8);
            }
        }
        describe(*st, QStringLiteral("%1:%2").arg(c->msg).arg(c->inst), c->sensor, kind, imu, filtered);
        lists[std::size_t(c->sensor)].push_back(st);
    }
    for (const QString &key : m_batchOrder) {
        Batch &b = m_batches[key];
        if (b.runs.empty()) continue;
        std::stable_sort(b.runs.begin(), b.runs.end(), [](const BatchRun &a, const BatchRun &c) { return a.t0 < c.t0; });
        auto st = std::make_shared<ImuStream>();
        double num = 0, den = 0;
        for (const BatchRun &r : b.runs) {
            const int len = int(r.x.size());
            const int o = int(st->t.size());
            for (int i = 0; i < len; ++i) st->t.push_back((r.t0 + i * 1e6 / r.fs - minUs) / 1000);
            st->x.insert(st->x.end(), r.x.begin(), r.x.end());
            st->y.insert(st->y.end(), r.y.begin(), r.y.end());
            st->z.insert(st->z.end(), r.z.begin(), r.z.end());
            st->runs.push_back({o, len, r.fs});
            st->longest = std::max(st->longest, len);
            num += len * r.fs;
            den += len;
        }
        st->fs = num / den;
        const int nSensors = b.sensor == 0 ? nGyro : nAcc;
        int imu = b.inst;
        bool filtered = (batOpt & 2) && !(batOpt & 4);
        if (nSensors && b.inst >= nSensors) {
            imu = b.inst - nSensors;
            filtered = true;
        }
        describe(*st, key, b.sensor, QStringLiteral("batch"), imu, filtered);
        lists[std::size_t(b.sensor)].push_back(st);
    }
    const auto rank = [](const QString &k) { return k == QLatin1String("raw") ? 0 : k == QLatin1String("batch") ? 1 : 2; };
    for (int s = 0; s < 2; ++s) {
        auto &l = lists[std::size_t(s)];
        std::stable_sort(l.begin(), l.end(), [&](const StreamPtr &a, const StreamPtr &b) {
            if (rank(a->kind) != rank(b->kind)) return rank(a->kind) < rank(b->kind);
            if (a->filtered != b->filtered) return !a->filtered;
            if (a->imu != b->imu) return a->imu < b->imu;
            return a->fs > b->fs;
        });
        for (auto &p : l) out.sensors[std::size_t(s)].push_back(p);
    }
    m_cont.clear();
    m_contOrder.clear();
    m_batches.clear();
    m_batchOrder.clear();
    m_fast = {};
    m_fastImuAcc = {};
    return out;
}

// ── FFT and PSD ──────────────────────────────────────────────────────────

namespace {

struct Plan {
    int N = 0;
    std::vector<std::uint32_t> rev;
    std::vector<double> cos, sin, win, re, im;
    double w2 = 0;
};

/// Bit reversal, twiddles and the firmware's Hann window 0.5 (1 − cos(2πn / (N − 1))), cached per thread.
Plan &plan(int N)
{
    thread_local std::unordered_map<int, Plan> plans;
    Plan &p = plans[N];
    if (p.N == N) return p;
    p.N = N;
    int bits = 0;
    while ((1 << bits) < N) ++bits;
    p.rev.resize(std::size_t(N));
    for (int i = 0; i < N; ++i) {
        std::uint32_t r = 0;
        for (int b = 0; b < bits; ++b) r |= std::uint32_t((i >> b) & 1) << (bits - 1 - b);
        p.rev[std::size_t(i)] = r;
    }
    p.cos.resize(std::size_t(N / 2));
    p.sin.resize(std::size_t(N / 2));
    for (int k = 0; k < N / 2; ++k) {
        p.cos[std::size_t(k)] = std::cos(2 * M_PI * k / N);
        p.sin[std::size_t(k)] = -std::sin(2 * M_PI * k / N);
    }
    p.win.resize(std::size_t(N));
    p.w2 = 0;
    for (int i = 0; i < N; ++i) {
        p.win[std::size_t(i)] = 0.5 * (1 - std::cos(2 * M_PI * i / (N - 1)));
        p.w2 += p.win[std::size_t(i)] * p.win[std::size_t(i)];
    }
    p.re.assign(std::size_t(N), 0);
    p.im.assign(std::size_t(N), 0);
    return p;
}

/// In-place FFT of re + i im (inputs already in bit-reversed order).
void fft(Plan &p)
{
    const int N = p.N;
    double *re = p.re.data(), *im = p.im.data();
    for (int size = 2; size <= N; size <<= 1) {
        const int half = size >> 1, step = N / size;
        for (int i = 0; i < N; i += size) {
            for (int j = 0, k = 0; j < half; ++j, k += step) {
                const int a = i + j, b = a + half;
                const double tr = re[b] * p.cos[std::size_t(k)] - im[b] * p.sin[std::size_t(k)];
                const double ti = re[b] * p.sin[std::size_t(k)] + im[b] * p.cos[std::size_t(k)];
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
    }
}

/// Two real signals at once (a real part, b imaginary, b may be null), mean
/// removed and windowed: |A_k|² + |B_k|² = (|Z_k|² + |Z_{N−k}|²) / 2 added to acc[k − 1].
void addPower(Plan &p, const float *a, const float *b, std::vector<double> &acc)
{
    const int N = p.N;
    double ma = 0, mb = 0;
    for (int i = 0; i < N; ++i) {
        ma += a[i];
        if (b) mb += b[i];
    }
    ma /= N;
    mb /= N;
    for (int i = 0; i < N; ++i) {
        const auto r = p.rev[std::size_t(i)];
        p.re[r] = (a[i] - ma) * p.win[std::size_t(i)];
        p.im[r] = b ? (b[i] - mb) * p.win[std::size_t(i)] : 0;
    }
    fft(p);
    for (int k = 1; k < N / 2; ++k) {
        const int j = N - k;
        acc[std::size_t(k - 1)] += (p.re[std::size_t(k)] * p.re[std::size_t(k)] + p.im[std::size_t(k)] * p.im[std::size_t(k)]
                                    + p.re[std::size_t(j)] * p.re[std::size_t(j)] + p.im[std::size_t(j)] * p.im[std::size_t(j)]) / 2;
    }
}

int lastAtOrBefore(const std::vector<double> &t, double tMs)
{
    return int(std::upper_bound(t.begin(), t.end(), tMs) - t.begin()) - 1;
}

const ImuStream::Run *runOf(const ImuStream &st, int i)
{
    int lo = 0, hi = int(st.runs.size()) - 1;
    if (hi < 0) return nullptr;
    while (lo < hi) {
        const int m = (lo + hi + 1) >> 1;
        if (st.runs[std::size_t(m)].start <= i) lo = m;
        else hi = m - 1;
    }
    const auto &r = st.runs[std::size_t(lo)];
    return i >= r.start && i < r.start + r.count ? &r : nullptr;
}

struct At {
    int end;
    double t;
    std::vector<float> db;
};

/// The spectrum of the newest full window at or before tMs; none when the run there is shorter than the window.
std::optional<At> spectrumAt(const ImuStream &st, const FftResolution &res, double tMs)
{
    int end = lastAtOrBefore(st.t, tMs);
    if (end < 0) return std::nullopt;
    const ImuStream::Run *run = runOf(st, end);
    if (!run) {
        // sample in a dropped short run: the previous run ends before it
        for (int r = int(st.runs.size()) - 1; r >= 0; --r)
            if (st.runs[std::size_t(r)].start + st.runs[std::size_t(r)].count - 1 < end) {
                run = &st.runs[std::size_t(r)];
                break;
            }
        if (!run) return std::nullopt;
        end = run->start + run->count - 1;
    }
    const int L = res.N + (res.K - 1) * res.N / 2;
    if (end - run->start + 1 < L) return std::nullopt;
    return At{end, st.t[std::size_t(end)], welchPsdDb(st, end, res.N, res.K, st.fs)};
}

} // namespace

std::vector<float> welchPsdDb(const ImuStream &st, int end, int N, int K, double fs)
{
    Plan &p = plan(N);
    const int L = N + (K - 1) * N / 2;
    const int first = end - L + 1;
    std::vector<double> acc(std::size_t(N / 2 - 1), 0.0);
    for (int s = 0; s < K; ++s) {
        const int start = first + s * N / 2;
        addPower(p, st.x.data() + start, st.y.data() + start, acc);
        addPower(p, st.z.data() + start, nullptr, acc);
    }
    const double scale = 2 / (fs * p.w2 * K);
    std::vector<float> db(acc.size());
    for (std::size_t i = 0; i < db.size(); ++i) db[i] = float(10 * std::log10(std::max(acc[i] * scale, 1e-20)));
    return db;
}

FftResolution fitResolution(int longest, const LogFftConfig &cfg)
{
    int N = cfg.n, K = cfg.avg;
    const auto span = [](int n, int k) { return n + (k - 1) * n / 2; };
    while (K > 1 && span(N, K) > longest) K >>= 1;
    while (N > 64 && N > longest) N >>= 1;
    return {N, K, N != cfg.n || K != cfg.avg};
}

// ── Scheduler: spectra in step with the replay cursor ────────────────────

namespace {
constexpr double kHistoryMs = 60000;   // the waterfall's span: filled again after a seek
constexpr int kBudgetMs = 25;          // compute per pump, so a heavy setting fills over a few pumps
} // namespace

LogFftConfig LogSpectrum::setConfig(const LogFftConfig &c)
{
    m_cfg = sanitizeLogFftConfig(c.n, c.avg, c.stepMs);
    if (m_loaded) restart();
    return m_cfg;
}

void LogSpectrum::selectStream(int sensor, const QString &key)
{
    if (sensor != 0 && sensor != 1) return;
    m_choice[std::size_t(sensor)] = key.isEmpty() ? QStringLiteral("auto") : key;
    if (m_loaded) restart();
}

void LogSpectrum::load(const ImuStreams &streams, Now nowMs)
{
    m_streams = streams;
    m_loaded = true;
    m_now = std::move(nowMs);
    m_choice = {QStringLiteral("auto"), QStringLiteral("auto")};
    restart();
}

void LogSpectrum::unload()
{
    if (!m_loaded) return;
    m_loaded = false;
    m_streams = {};
    m_active = {};
    LogSpectrumMsg m;
    m.gen = ++m_gen;
    m.reset = true;
    if (m_send) m_send(m);
}

void LogSpectrum::restart()
{
    ++m_gen;
    m_active = {};
    auto info = std::make_shared<LogSpectrumInfo>();
    info->cfgN = m_cfg.n;
    info->cfgAvg = m_cfg.avg;
    info->cfgStepMs = m_cfg.stepMs;
    info->dropped = m_streams.dropped;
    for (int sensor = 0; sensor < 2; ++sensor) {
        const auto &list = m_streams.sensors[std::size_t(sensor)];
        ImuStreamPtr st;
        for (const auto &s : list)
            if (s->key == m_choice[std::size_t(sensor)]) st = s;
        if (!st && !list.empty()) st = list.front();
        auto &si = info->sensors[std::size_t(sensor)];
        if (!st) continue;
        const FftResolution res = fitResolution(st->longest, m_cfg);
        m_active[std::size_t(sensor)] = Active{st, res, -1};
        for (const auto &s : list) si.streams.push_back({s->key, s->label});
        si.stream = st->key;
        si.choice = m_choice[std::size_t(sensor)];
        si.label = st->label;
        si.fs = st->fs;
        si.N = res.N;
        si.avg = res.K;
        si.limited = res.limited;
        si.longest = st->longest;
        si.df = st->fs / res.N;
        si.n = res.N / 2 - 1;
        si.windowMs = (res.N + (res.K - 1) * res.N / 2) / st->fs * 1000;
    }
    const double now = m_now();
    const int step = m_cfg.stepMs;
    m_nextT = std::max(0.0, std::ceil((now - kHistoryMs) / step) * step);
    m_lastNow = now;
    LogSpectrumMsg m;
    m.gen = m_gen;
    m.reset = true;
    m.log = info;
    if (m_send) m_send(m);
    pump();
}

void LogSpectrum::pump()
{
    if (!m_loaded) return;
    const double now = m_now();
    // The cursor went back without a seek (play from the end rewinds): start over
    if (now + 1000 < m_lastNow) {
        m_lastNow = now;
        restart();
        return;
    }
    m_lastNow = now;
    const int step = m_cfg.stepMs;
    if (m_nextT < now - kHistoryMs) m_nextT = std::ceil((now - kHistoryMs) / step) * step;
    QElapsedTimer clock;
    clock.start();
    LogSpectrumMsg m;
    m.gen = m_gen;
    while (m_nextT <= now && clock.elapsed() < kBudgetMs) {
        for (int sensor = 0; sensor < 2; ++sensor) {
            auto &a = m_active[std::size_t(sensor)];
            if (!a) continue;
            const int end = lastAtOrBefore(a->st->t, m_nextT);
            if (end < 0 || end == a->lastEnd) continue;
            auto s = spectrumAt(*a->st, a->res, m_nextT);
            if (!s || s->end == a->lastEnd) continue;
            a->lastEnd = s->end;
            m.rows.push_back({sensor, s->t, std::make_shared<const std::vector<float>>(std::move(s->db))});
        }
        m_nextT += step;
    }
    if (!m.rows.empty() && m_send) m_send(m);
}

} // namespace corv
