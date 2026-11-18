// Port of scripts/test-pose-smoother.js: telemetry-rate samples (25 Hz,
// jittered, in bursts, at 5 Hz, with and without the vehicle clock and body
// rates) rendered at 60 fps; the drawn pose must move every frame at the
// vehicle's speed, stay on the true track through a turn, snap on a seek and
// stop projecting when the link goes quiet. Same thresholds as the JS test.
#include "vehicle/PoseSmoother.h"

#include <QTest>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

using namespace corv;

namespace {
constexpr double M = 111320, LAT0 = 45, LON0 = 10;
const double MLON = M * std::cos(LAT0 * M_PI / 180);
constexpr double TAU = 0.075, FRAME = 1000.0 / 60, G = 9.81, V = 70;
const double TURN = 20 * M_PI / 180;

PoseSmoother::Sample truth(double t, double turn = 0)
{
    const double s = t / 1000, h0 = M_PI / 4, hdg = h0 + turn * s;
    double n, e;
    if (turn != 0) {
        const double r = V / turn;
        n = r * (std::sin(hdg) - std::sin(h0));
        e = r * (std::cos(h0) - std::cos(hdg));
    } else {
        n = std::cos(h0) * V * s;
        e = std::sin(h0) * V * s;
    }
    const double roll = std::atan(V * turn / G), pitch = 0.05;
    PoseSmoother::Sample o;
    o.lat = LAT0 + n / M;
    o.lon = LON0 + e / MLON;
    o.rawAlt = 300 + 2 * s;
    o.vn = std::cos(hdg) * V;
    o.ve = std::sin(hdg) * V;
    o.vs = 2;
    o.gs = V;
    o.track = hdg;
    o.roll = roll;
    o.pitch = pitch;
    o.yaw = std::fmod(std::fmod(hdg, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    o.rollRate = -std::sin(pitch) * turn;
    o.pitchRate = turn * std::sin(roll) * std::cos(pitch);
    o.yawRate = turn * std::cos(roll) * std::cos(pitch);
    return o;
}

template <typename P>
double metres(const P &a, double lat, double lon) { return std::hypot((a.lat - lat) * M, (a.lon - lon) * MLON); }

struct Arrival { double take, at; };

std::vector<Arrival> every(double ms, double seconds, const std::function<double(double)> &at = [](double t) { return t + 5; })
{
    std::vector<Arrival> out;
    for (double take = 0; take <= seconds * 1000; take += ms) out.push_back({take, at(take)});
    std::stable_sort(out.begin(), out.end(), [](auto &a, auto &b) { return a.at < b.at; });
    return out;
}

struct Flight { std::vector<double> speeds, errors, yawRates, yawErrors; };

struct FlyOpts { double seconds = 4, tau = TAU, turn = 0; bool rates = true, boot = true; };

Flight fly(const std::vector<Arrival> &arrivals, FlyOpts o = {})
{
    PoseSmoother smoother;
    PoseSmoother::Sample state = truth(0, o.turn);
    Flight out;
    std::size_t next = 0;
    bool havePrev = false;
    Pose prev;
    for (double t = 0; t <= o.seconds * 1000; t += FRAME) {
        while (next < arrivals.size() && arrivals[next].at <= t) {
            const Arrival a = arrivals[next++];
            PoseSmoother::Sample s = truth(a.take, o.turn);
            if (!o.rates) s.rollRate = s.pitchRate = s.yawRate = 0;
            s.posTime = s.attTime = a.at;
            if (o.boot) { s.posBootMs = 10000 + a.take; s.attBootMs = 10000 + a.take; }
            state = s;
        }
        const Pose pose = smoother.update(state, t, o.tau);
        if (havePrev && t > 600) {
            const auto real = truth(t, o.turn);
            out.speeds.push_back(metres(pose, prev.lat, prev.lon) / (FRAME / 1000));
            out.errors.push_back(metres(pose, real.lat, real.lon));
            double dy = pose.yaw - prev.yaw;
            dy = std::atan2(std::sin(dy), std::cos(dy));
            out.yawRates.push_back(dy / (FRAME / 1000));
            out.yawErrors.push_back(std::abs(std::atan2(std::sin(pose.yaw - real.yaw), std::cos(pose.yaw - real.yaw))));
        }
        prev = pose;
        havePrev = true;
    }
    return out;
}

double mean(const std::vector<double> &a) { return std::accumulate(a.begin(), a.end(), 0.0) / a.size(); }
double cv(const std::vector<double> &a)
{
    const double m = mean(a);
    double acc = 0;
    for (double x : a) acc += (x - m) * (x - m);
    return std::sqrt(acc / a.size()) / std::abs(m);
}
double still(const std::vector<double> &a) { return double(std::count_if(a.begin(), a.end(), [](double x) { return x < V * 0.25; })) / a.size(); }
double maxOf(const std::vector<double> &a) { return *std::max_element(a.begin(), a.end()); }

std::uint64_t g_seed = 7;
double rnd() { g_seed = (g_seed * 16807) % 2147483647; return double(g_seed) / 2147483647; }
} // namespace

class TestPoseSmoother : public QObject
{
    Q_OBJECT
private slots:
    void steady25Hz()
    {
        const auto steady = every(40, 5);
        const auto raw = fly(steady, {.tau = 0});
        QVERIFY2(still(raw.speeds) > 0.5, "raw telemetry at 25 Hz stands still most frames (the bug)");
        const auto s25 = fly(steady);
        QCOMPARE(still(s25.speeds), 0.0);
        QVERIFY2(cv(s25.speeds) < 0.02, qPrintable(QString::number(cv(s25.speeds))));
        QVERIFY2(maxOf(s25.errors) < 0.5, qPrintable(QString::number(maxOf(s25.errors))));
    }

    void jitter()
    {
        g_seed = 7;
        const auto jit = every(40, 5, [](double take) { return take + 20 + (rnd() - 0.5) * 30; });
        const auto jb = fly(jit);
        const auto jn = fly(jit, {.boot = false});
        QVERIFY2(cv(jb.speeds) < 0.02, qPrintable(QString::number(cv(jb.speeds))));
        QVERIFY2(maxOf(jb.errors) < 1.6, qPrintable(QString::number(maxOf(jb.errors))));
        QVERIFY2(still(jn.speeds) == 0 && cv(jn.speeds) < 0.15, qPrintable(QString::number(cv(jn.speeds))));
    }

    void bursts()
    {
        const auto b = every(40, 5, [](double take) { return std::ceil((take + 1) / 160) * 160 + 30; });
        for (bool boot : {true, false}) {
            const auto sb = fly(b, {.boot = boot});
            QVERIFY2(still(sb.speeds) == 0 && cv(sb.speeds) < 0.1, qPrintable(QStringLiteral("boot %1 cv %2").arg(boot).arg(cv(sb.speeds))));
        }
    }

    void turn()
    {
        const auto steady = every(40, 5);
        const auto t25 = fly(steady, {.turn = TURN});
        QVERIFY2(maxOf(t25.errors) < 0.5, qPrintable(QString::number(maxOf(t25.errors))));
        QVERIFY(cv(t25.yawRates) < 0.05 && std::abs(mean(t25.yawRates) - TURN) < 0.01);
        QVERIFY(maxOf(t25.yawErrors) < 0.01);
        const auto slow = every(200, 5);
        const auto t5 = fly(slow, {.turn = TURN});
        QVERIFY2(maxOf(t5.errors) < 0.5, qPrintable(QString::number(maxOf(t5.errors))));
        QVERIFY(still(t5.speeds) == 0 && cv(t5.speeds) < 0.05);
    }

    void noRates()
    {
        const auto steady = every(40, 5);
        const auto nr = fly(steady, {.turn = TURN, .rates = false});
        const auto nrRaw = fly(steady, {.tau = 0, .turn = TURN, .rates = false});
        QVERIFY2(cv(nr.yawRates) < 0.05 && cv(nrRaw.yawRates) > 1,
                 qPrintable(QStringLiteral("%1 vs raw %2").arg(cv(nr.yawRates)).arg(cv(nrRaw.yawRates))));
        QVERIFY(maxOf(nr.yawErrors) < 0.01);
    }

    void yawThroughNorth()
    {
        PoseSmoother smoother;
        auto state = truth(0);
        state.yaw = -0.02;
        state.posTime = state.attTime = 0;
        state.rollRate = state.pitchRate = 0;
        state.yawRate = 0.5;
        state.roll = state.pitch = 0;
        double worst = 0, prev = smoother.update(state, 0, TAU).yaw;
        for (double t = FRAME; t < 1000; t += FRAME) {
            if (std::floor(t / 40) != std::floor((t - FRAME) / 40)) {
                const double y = -0.02 + 0.5 * t / 1000;
                state.yaw = y > M_PI ? y - 2 * M_PI : y;
                state.attTime = t;
            }
            const double y = smoother.update(state, t, TAU).yaw;
            double d = y - prev;
            d = std::atan2(std::sin(d), std::cos(d));
            worst = std::max(worst, std::abs(d));
            prev = y;
        }
        QVERIFY2(worst < 0.05, qPrintable(QString::number(worst)));
    }

    void seekSnaps()
    {
        PoseSmoother smoother;
        auto state = truth(0);
        smoother.update(state, 0, TAU);
        state.lat = LAT0 + 0.05;
        state.posTime = 40;
        const Pose pose = smoother.update(state, 41, TAU);
        QVERIFY(std::abs(pose.lat - (LAT0 + 0.05)) * M < 1);
    }

    void linkLost()
    {
        PoseSmoother smoother;
        auto state = truth(0, TURN);
        double t = 0;
        for (; t <= 1000; t += 40) {
            state = truth(t, TURN);
            state.posTime = state.attTime = t;
            state.posBootMs = state.attBootMs = t;
            smoother.update(state, t, TAU);
        }
        t -= 40;
        const auto last = state;
        auto at = [&](double ms) { return smoother.update(state, t + ms, TAU); };
        const Pose p1 = at(250), p2 = at(500), p3 = at(750), p4 = at(6000);
        const double ahead = metres(p2, last.lat, last.lon);
        QVERIFY(metres(p1, last.lat, last.lon) > V * 0.22 && ahead > V * 0.45);
        QVERIFY(metres(p2, p3.lat, p3.lon) < 0.01 && metres(p2, p4.lat, p4.lon) < 0.01);
        const auto real = truth(t + 500, TURN);
        QVERIFY2(metres(p2, real.lat, real.lon) < 0.5, qPrintable(QString::number(metres(p2, real.lat, real.lon))));
    }

    void perFrameState()
    {
        PoseSmoother smoother;
        double worst = 0;
        for (double t = 0; t < 2000; t += FRAME) {
            auto state = truth(t, TURN);
            worst = std::max(worst, metres(smoother.update(state, t, TAU), state.lat, state.lon));
        }
        QVERIFY2(worst < 0.05, qPrintable(QString::number(worst)));
    }

    void staleStamps()
    {
        PoseSmoother smoother;
        auto state = truth(0);
        for (double t = 0; t <= 1000; t += 40) {
            state = truth(t);
            state.posTime = state.attTime = t;
            state.posBootMs = state.attBootMs = 5000 + t;
            smoother.update(state, t, TAU);
        }
        double worst = 0;
        for (double t = 1000; t < 6000; t += FRAME) {
            const auto keep = state;
            state = truth(t);
            state.posTime = keep.posTime;
            state.attTime = keep.attTime;
            state.posBootMs = keep.posBootMs;
            state.attBootMs = keep.attBootMs;
            worst = std::max(worst, metres(smoother.update(state, t, TAU), state.lat, state.lon));
        }
        QVERIFY2(worst < 0.5, qPrintable(QString::number(worst)));
    }

    void smoothingOff()
    {
        PoseSmoother smoother;
        auto state = truth(1234);
        const Pose p = smoother.update(state, 5000, 0);
        QVERIFY(p.lat == state.lat && p.lon == state.lon && p.alt == state.rawAlt && p.yaw == state.yaw);
    }
};

QTEST_GUILESS_MAIN(TestPoseSmoother)
#include "tst_posesmoother.moc"
