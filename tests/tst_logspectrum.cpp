// Port of scripts/test-log-spectrum.js: the vibration spectra the GCS computes
// from the IMU samples of a .bin log.
//   - the PSD has the definition of the live one: same levels as a direct DFT of
//     the firmware's formula, Parseval on white noise, a sine's power under its
//     line, Welch averaging that flattens the floor
//   - a DataFlash file written here with GYR, ACC, IMU, ISBH/ISBD and PARM is read
//     back into streams: rates, post-filter instances, batch scaling, holes
//   - the scheduler follows the replay cursor: the 60 s before it after a seek,
//     one spectrum per batch, start over on rewind, no window across a hole
#include "link/MavlinkParser.h"
#include "services/LogIndex.h"

#include <QTest>
#include <QtEndian>
#include <cmath>
#include <map>
#include <set>

using namespace corv;

namespace {

std::int64_t g_seed = 12345;
double rnd()
{
    g_seed = (g_seed * 1103515245 + 12345) & 0x7fffffff;
    return double(g_seed) / 0x7fffffff;
}
double gauss() { return std::sqrt(-2 * std::log(rnd() + 1e-12)) * std::cos(2 * M_PI * rnd()); }

template <typename Gen>
ImuStream stream(int n, double fs, Gen gen)
{
    ImuStream st;
    st.fs = fs;
    for (int i = 0; i < n; ++i) {
        const auto v = gen(i / fs);
        st.x.push_back(float(v[0]));
        st.y.push_back(float(v[1]));
        st.z.push_back(float(v[2]));
        st.t.push_back(i / fs * 1000);
    }
    st.runs = {{0, n, fs}};
    st.longest = n;
    return st;
}

double meanDb(const std::vector<float> &db)
{
    double s = 0;
    for (float v : db) s += std::pow(10, v / 10.0);
    return 10 * std::log10(s / double(db.size()));
}
double spreadDb(const std::vector<float> &db)
{
    double m = 0;
    for (float v : db) m += v;
    m /= double(db.size());
    double s = 0;
    for (float v : db) s += (v - m) * (v - m);
    return std::sqrt(s / double(db.size()));
}

/// A DataFlash file, record by record.
class Writer
{
public:
    void fmt(int type, const char *name, const char *format, const char *labels)
    {
        int len = 3;
        for (const char *c = format; *c; ++c) len += df::typeSize(*c);
        QByteArray b(89, 0);
        b[0] = char(0xA3);
        b[1] = char(0x95);
        b[2] = char(128);
        b[3] = char(type);
        b[4] = char(len);
        std::memcpy(b.data() + 5, name, std::strlen(name));
        std::memcpy(b.data() + 9, format, std::strlen(format));
        std::memcpy(b.data() + 25, labels, std::strlen(labels));
        m_out.append(b);
        m_fmts[name] = {type, len, format};
    }
    struct V {
        double n = 0;
        QByteArray s;
        std::vector<int> a;
        V(double v) : n(v) {}
        V(int v) : n(v) {}
        V(const char *t) : s(t) {}
        V(std::vector<int> v) : a(std::move(v)) {}
    };
    void rec(const char *name, const std::vector<V> &values)
    {
        const auto &f = m_fmts[name];
        QByteArray b(f.len, 0);
        auto *p = reinterpret_cast<uchar *>(b.data());
        p[0] = 0xA3;
        p[1] = 0x95;
        p[2] = uchar(f.type);
        int o = 3;
        for (std::size_t i = 0; i < f.format.size(); ++i) {
            const char c = f.format[i];
            const V &v = values[i];
            switch (c) {
            case 'Q': qToLittleEndian<std::uint64_t>(std::uint64_t(std::llround(v.n)), p + o); break;
            case 'B': p[o] = uchar(v.n); break;
            case 'H': qToLittleEndian<std::uint16_t>(std::uint16_t(v.n), p + o); break;
            case 'f': qToLittleEndian<float>(float(v.n), p + o); break;
            case 'I': qToLittleEndian<std::uint32_t>(std::uint32_t(v.n), p + o); break;
            case 'Z': case 'N': case 'n': std::memcpy(p + o, v.s.constData(), std::size_t(v.s.size())); break;
            case 'a':
                for (int k = 0; k < 32; ++k) qToLittleEndian<std::int16_t>(std::int16_t(v.a[std::size_t(k)]), p + o + 2 * k);
                break;
            }
            o += df::typeSize(c);
        }
        m_out.append(b);
    }
    const QByteArray &bytes() const { return m_out; }

private:
    struct F {
        int type = 0, len = 0;
        std::string format;
    };
    std::map<std::string, F> m_fmts;
    QByteArray m_out;
};

ImuStreamPtr byKey(const std::vector<ImuStreamPtr> &list, const QString &key)
{
    for (const auto &s : list)
        if (s->key == key) return s;
    return nullptr;
}

} // namespace

class TstLogSpectrum : public QObject
{
    Q_OBJECT
    ParsedLog m_parsed;

private slots:
    void psdAgainstDft()
    {
        const int N = 256;
        const double fs = 800;
        const ImuStream st = stream(N, fs, [](double) { return std::array<double, 3>{gauss(), gauss() * 0.5, gauss() * 2}; });
        const std::vector<float> db = welchPsdDb(st, N - 1, N, 1, fs);
        double w2 = 0;
        std::vector<double> w(N);
        for (int i = 0; i < N; ++i) {
            w[std::size_t(i)] = 0.5 * (1 - std::cos(2 * M_PI * i / (N - 1)));
            w2 += w[std::size_t(i)] * w[std::size_t(i)];
        }
        double worst = 0;
        for (int k = 1; k < N / 2; ++k) {
            double p = 0;
            for (const auto *a : {&st.x, &st.y, &st.z}) {
                double m = 0;
                for (int i = 0; i < N; ++i) m += (*a)[std::size_t(i)];
                m /= N;
                double re = 0, im = 0;
                for (int i = 0; i < N; ++i) {
                    const double v = ((*a)[std::size_t(i)] - m) * w[std::size_t(i)];
                    re += v * std::cos(2 * M_PI * k * i / N);
                    im -= v * std::sin(2 * M_PI * k * i / N);
                }
                p += 2 * (re * re + im * im) / (fs * w2);
            }
            worst = std::max(worst, std::abs(10 * std::log10(p) - db[std::size_t(k - 1)]));
        }
        QCOMPARE(int(db.size()), N / 2 - 1);
        QVERIFY2(worst < 1e-3, qPrintable(QStringLiteral("max diff %1 dB").arg(worst)));
    }

    void parsevalAndWelch()
    {
        const int N = 1024, K = 8;
        const double fs = 2000, sigma = 0.1;
        const int n = N + (K - 1) * N / 2;
        const ImuStream st = stream(n, fs, [&](double) { return std::array<double, 3>{sigma * gauss(), sigma * gauss(), sigma * gauss()}; });
        const double expect = 10 * std::log10(3 * 2 * sigma * sigma / fs);
        const auto one = welchPsdDb(st, N - 1, N, 1, fs);
        const auto avg = welchPsdDb(st, n - 1, N, K, fs);
        QVERIFY2(std::abs(meanDb(avg) - expect) < 0.3, "white noise: floor at 3 axes × 2σ²/fs");
        QVERIFY2(spreadDb(avg) < 0.5 * spreadDb(one), "Welch ×8 flattens the floor");
    }

    void sine()
    {
        const int N = 2048;
        const double fs = 1931.4, f0 = 187.3, A = 0.4;
        const ImuStream st = stream(N, fs, [&](double t) { return std::array<double, 3>{A * std::sin(2 * M_PI * f0 * t), 0, 0}; });
        const auto db = welchPsdDb(st, N - 1, N, 1, fs);
        const double df = fs / N;
        int best = 0;
        for (int i = 1; i < int(db.size()); ++i)
            if (db[std::size_t(i)] > db[std::size_t(best)]) best = i;
        double p = 0;
        for (int i = best - 6; i <= best + 6; ++i) p += std::pow(10, db[std::size_t(i)] / 10.0) * df;
        QVERIFY2(std::abs((best + 1) * df - f0) <= df / 2 + 1e-9, "sine: line in its bin");
        QVERIFY2(std::abs(p / (A * A / 2) - 1) < 0.03, "sine power A²/2 under the line");
    }

    void resolution()
    {
        const auto a = fitResolution(1024, {2048, 1, 250});
        const auto b = fitResolution(1024, {512, 4, 250});
        const auto c = fitResolution(1024, {1024, 1, 250});
        QVERIFY(a.N == 1024 && a.K == 1 && a.limited);
        QVERIFY(b.N == 512 && b.K == 2 && b.limited);
        QVERIFY(c.N == 1024 && c.K == 1 && !c.limited);
        const auto s = sanitizeLogFftConfig(1000, 3, 7);
        QVERIFY(s.n == 1024 && s.avg == 1 && s.stepMs == 250);
    }

    void dataflash()
    {
        Writer w;
        w.fmt(64, "PARM", "QNf", "TimeUS,Name,Value");
        w.fmt(65, "ATT", "Qfff", "TimeUS,Roll,Pitch,Yaw");
        w.fmt(66, "GYR", "QBQfff", "TimeUS,I,SampleUS,GyrX,GyrY,GyrZ");
        w.fmt(67, "ACC", "QBQfff", "TimeUS,I,SampleUS,AccX,AccY,AccZ");
        w.fmt(68, "IMU", "QBffffffIIfBBHH", "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz");
        w.fmt(69, "ISBH", "QHBBHHQf", "TimeUS,N,type,instance,mul,smp_cnt,SampleUS,smp_rate");
        w.fmt(70, "ISBD", "QHHaaa", "TimeUS,N,seqno,x,y,z");
        const double T0 = 10e6;   // log starts 10 s after boot
        const std::pair<const char *, double> parms[] = {{"INS_GYR_ID", 1}, {"INS_GYR2_ID", 0}, {"INS_ACC_ID", 1}, {"INS_ACC2_ID", 0},
                                                         {"INS_RAW_LOG_OPT", 9}, {"INS_LOG_BAT_OPT", 5}};
        for (const auto &[k, v] : parms) w.rec("PARM", {T0, k, v});
        // Raw gyro at 2000 Hz, 30 s, a 120 Hz line, SampleUS in bursts of two and a
        // 0.2 s hole at 12 s; its post-filter copy as instance 1
        const int fsG = 2000, durS = 30;
        for (int i = 0; i < fsG * durS; ++i) {
            const double t = double(i) / fsG;
            if (t >= 12 && t < 12.2) continue;
            const double su = T0 + std::floor(i / 2) * 1000 + (i % 2) * 20;
            const double g = 0.05 * std::sin(2 * M_PI * 120 * t) + 0.001 * gauss();
            w.rec("GYR", {su + 100, 0, su, g, 0.001 * gauss(), 0.001 * gauss()});
            w.rec("GYR", {su + 100, 1, su, 0.2 * g, 0, 0});
            if (i % 2 == 0) w.rec("ACC", {su + 100, 0, su, 0.5 * std::sin(2 * M_PI * 80 * t), 0, -9.8});
            if (i % 40 == 0) {
                w.rec("ATT", {T0 + i * 500, 0, 0, 0});
                // IMU1 at 50 Hz; IMU2 with a 7 Hz line and two records in every 7 dropped
                w.rec("IMU", {T0 + i * 500, 0, g, 0, 0, 0, 0, -9.8, 0, 0, 25, 1, 1, 2000, 1000});
                const int k = i / 40;
                if (k % 7 < 5) w.rec("IMU", {T0 + i * 500, 1, 0.1 * std::sin(2 * M_PI * 7 * t), 0, 0, 0, 0, -9.8, 0, 0, 25, 1, 1, 2000, 1000});
            }
        }
        // Batch gyro: 1024 samples at 3276.8 Hz every 2 s, a 300 Hz line; one chunk of the third batch lost
        const double fsB = 3276.8;
        const int mul = 938;
        for (int bN = 0; bN < 10; ++bN) {
            const double su = T0 + 1e6 + bN * 2e6;
            w.rec("ISBH", {su + 400e3, bN, 1, 0, mul, 1024, su, fsB});
            for (int c = 0; c < 32; ++c) {
                if (bN == 2 && c == 20) continue;
                std::vector<int> x(32);
                for (int k = 0; k < 32; ++k) x[std::size_t(k)] = int(std::lround(mul * 0.3 * std::sin(2 * M_PI * 300 * (c * 32 + k) / fsB)));
                w.rec("ISBD", {su + 400e3 + c * 20e3, bN, c, x, std::vector<int>(32, 0), std::vector<int>(32, 0)});
            }
        }
        const QByteArray bytes = w.bytes();
        m_parsed = parseBinLog(reinterpret_cast<const std::uint8_t *>(bytes.constData()), bytes.size());
        const auto &G = m_parsed.imu.sensors[0], &A = m_parsed.imu.sensors[1];
        const auto raw = byKey(G, QStringLiteral("GYR:0")), post = byKey(G, QStringLiteral("GYR:1"));
        const auto bat = byKey(G, QStringLiteral("ISB:1:0")), loop = byKey(G, QStringLiteral("IMU:0"));
        QVERIFY2(raw && post && bat && loop, "gyro streams: raw, post-filter, batch, loop rate");
        QVERIFY2(G.front() == raw, "best gyro stream first: IMU1 raw");
        QVERIFY2(std::abs(raw->fs - 2000) < 1, qPrintable(QString::number(raw->fs)));
        QVERIFY2(post->imu == 0 && post->filtered && post->label.contains(QStringLiteral("IMU1 · filtered")), qPrintable(post->label));
        QCOMPARE(int(raw->runs.size()), 2);                                   // the 0.2 s hole
        QVERIFY2(std::abs(raw->t[0]) < 1, "raw gyro times on the replay base");
        QVERIFY(std::abs(bat->fs - 3276.8) < 0.01 && std::abs(bat->t[0] - 1000) < 1e-6);
        std::vector<int> partial;
        for (const auto &r : bat->runs)
            if (r.count < 1024) partial.push_back(r.count);
        QVERIFY2(partial == std::vector<int>({640, 352}), "batch with a lost chunk: its two runs");
        float peak = 0;
        for (int i = 0; i < 1024; ++i) peak = std::max(peak, bat->x[std::size_t(i)]);
        QVERIFY2(std::abs(peak - 0.3) < 0.002, "batch samples scaled by ISBH.mul");
        const auto acc = byKey(A, QStringLiteral("ACC:0"));
        QVERIFY(acc && std::abs(acc->fs - 1000) < 1);
        QVERIFY(byKey(A, QStringLiteral("IMU:0")) && std::abs(loop->fs - 50) < 0.1);
        const auto lossy = byKey(G, QStringLiteral("IMU:1"));
        QVERIFY2(lossy && lossy->runs.size() == 2 && std::abs(lossy->fs - 50) < 0.01, "IMU with 2 in 7 dropped: resampled, two runs");
        {
            const int N = 256;
            const auto &run = lossy->runs[1];
            const auto db = welchPsdDb(*lossy, run.start + N - 1, N, 1, lossy->fs);
            int best = 0;
            for (int i = 1; i < int(db.size()); ++i)
                if (db[std::size_t(i)] > db[std::size_t(best)]) best = i;
            QVERIFY2(std::abs((best + 1) * 50.0 / N - 7) < 50.0 / N, "…and its 7 Hz line");
        }
        // IMU samples are not replayed as messages: only ATT and HEARTBEAT
        std::set<int> ids;
        for (const LogEntry &e : m_parsed.index) ids.insert(frameMsgId(reinterpret_cast<const std::uint8_t *>(m_parsed.frames.constData()) + e.offset, e.len));
        QVERIFY(ids == std::set<int>({0, 30}));
    }

    void scheduler()
    {
        std::vector<LogSpectrumMsg> sent;
        double now = 0;
        LogSpectrum ls([&](const LogSpectrumMsg &m) { sent.push_back(m); });
        ls.setConfig({1024, 1, 250});
        ls.load(m_parsed.imu, [&] { return now; });
        const auto reset = std::find_if(sent.begin(), sent.end(), [](const LogSpectrumMsg &m) { return m.reset; });
        QVERIFY(reset != sent.end() && reset->log && reset->log->sensors[0].stream == QLatin1String("GYR:0")
                && reset->log->sensors[1].stream == QLatin1String("ACC:0"));
        QVERIFY(std::abs(reset->log->sensors[0].df - 1.953) < 0.01 && reset->log->sensors[0].n == 511);

        const auto rowsOf = [&](int sensor) {
            std::vector<LogSpectrumRow> out;
            for (const auto &m : sent)
                for (const auto &r : m.rows)
                    if (r.sensor == sensor) out.push_back(r);
            return out;
        };
        sent.clear();
        now = 20000;
        ls.restart();
        for (int k = 0; k < 40; ++k) ls.pump();
        const auto rows = rowsOf(0);
        QVERIFY2(rows.size() > 60 && rows.front().t < 1000 && rows.back().t <= 20000, "after a seek to 20 s: the spectra of 0 … 20 s");
        bool increasing = true;
        for (std::size_t i = 1; i < rows.size(); ++i) increasing &= rows[i].t > rows[i - 1].t;
        QVERIFY(increasing && std::abs((rows.back().t - rows.front().t) / double(rows.size() - 1) - 250) < 20);
        for (const auto &r : rows) QVERIFY2(!(r.t > 12000 && r.t < 12200 + 511), "no window across the 12 s hole");
        const auto r0 = std::find_if(rows.begin(), rows.end(), [](const LogSpectrumRow &r) { return r.t > 5000; });
        int best = 0;
        for (int i = 1; i < int(r0->db->size()); ++i)
            if ((*r0->db)[std::size_t(i)] > (*r0->db)[std::size_t(best)]) best = i;
        QVERIFY2(std::abs((best + 1) * 2000.0 / 1024 - 120) < 2, "the 120 Hz line in the log spectrum");

        // The batch stream: one spectrum per batch, at its last sample
        sent.clear();
        ls.selectStream(0, QStringLiteral("ISB:1:0"));
        for (int k = 0; k < 40; ++k) ls.pump();
        const auto brows = rowsOf(0);
        const auto sel = std::find_if(sent.begin(), sent.end(), [](const LogSpectrumMsg &m) { return m.reset; })->log->sensors[0];
        QVERIFY(sel.stream == QLatin1String("ISB:1:0") && sel.N == 1024 && !sel.limited);
        QCOMPARE(int(brows.size()), 9);
        for (const auto &r : brows) QVERIFY(std::abs(std::fmod(r.t - 1000, 2000) - 1023 / 3.2768) < 1);

        // Rewind without a seek: start over
        sent.clear();
        now = 3000;
        ls.pump();
        QVERIFY(std::any_of(sent.begin(), sent.end(), [](const LogSpectrumMsg &m) { return m.reset; }));
        ls.unload();
        QVERIFY(sent.back().reset && !sent.back().log);
    }

    void tlog()
    {
        // 25 frames 10 ms apart (the zero is the median of the first 20 stamps),
        // a 2-minute hole, two more: the hole shrinks to 100 ms, the rest keeps its steps
        QByteArray f;
        const double t0 = 1.7e15;   // µs, 2023
        std::vector<double> stamps;
        for (int i = 0; i < 25; ++i) stamps.push_back(t0 + i * 10e3);
        stamps.push_back(stamps.back() + 120e6);
        stamps.push_back(stamps.back() + 20e3);
        for (double ts : stamps) {
            mavlink_message_t m;
            mavlink_msg_global_position_int_pack(1, 1, &m, 0, 450500000, 97000000, 100000, 0, 0, 0, 0, 0);
            uchar b[8];
            qToLittleEndian<std::uint64_t>(std::uint64_t(ts), b);
            f.append(reinterpret_cast<const char *>(b), 8);
            f.append(MavlinkParser::serialize(m));
        }
        const ParsedLog p = indexTlog(reinterpret_cast<const std::uint8_t *>(f.constData()), f.size());
        QCOMPARE(int(p.index.size()), 27);
        // the median of the first 20 is the 11th stamp: the log starts 100 ms "before" 0 and clamps there
        QCOMPARE(int(p.index[0].tsMs), 0);
        QCOMPARE(int(p.index[24].tsMs), 140);
        QCOMPARE(int(p.index[25].tsMs), 240);     // 2 minutes → 100 ms
        QCOMPARE(int(p.index[26].tsMs), 260);
        QCOMPARE(p.gapsCompressed, 1);
        QCOMPARE(int(p.gpsTrack.size()), 27);
        QVERIFY(std::abs(p.gpsTrack[0].lat - 45.05) < 1e-9 && std::abs(p.gpsTrack[0].alt - 100) < 1e-9);
        QCOMPARE(frameMsgId(reinterpret_cast<const std::uint8_t *>(f.constData()) + p.index[0].offset, p.index[0].len), int(MAVLINK_MSG_ID_GLOBAL_POSITION_INT));
    }
};

QTEST_GUILESS_MAIN(TstLogSpectrum)
#include "tst_logspectrum.moc"
