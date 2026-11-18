// tst_framepacer - the 3D view's frame cap (FramePacer): whole vblanks per
// frame, the phase-locked grid, snapping to known vblanks, stalls, and a
// simulated render loop with timer jitter that must land every frame on the
// vblank it was meant for.
#include "scene/FramePacer.h"

#include <QTest>
#include <cmath>
#include <vector>

using namespace corv;

namespace {
// The vblank a frame requested at `at` is presented at: the first one after it is drawn
double presented(double at, double renderMs, const FramePacer::Vsync &vs)
{
    const double done = at + renderMs;
    return vs.vblankMs + std::ceil((done - vs.vblankMs) / vs.periodMs - 1e-9) * vs.periodMs;
}

// Deterministic jitter in [-1, 1] ms (a QTimer on Windows fires within a millisecond)
struct Jitter {
    unsigned s = 12345;
    double operator()()
    {
        s = s * 1103515245u + 12345u;
        return ((s >> 8) % 2001) / 1000.0 - 1.0;
    }
};

// Frames presented by a loop that asks the pacer after each frame, wakes with jitter and draws for renderMs
std::vector<double> run(FramePacer &p, double hz, const FramePacer::Vsync &vs, double renderMs, int frames)
{
    Jitter j;
    std::vector<double> out;
    double now = 1000.0;
    for (int k = 0; k < frames; ++k) {
        const double at = p.next(now, hz, vs);
        const double woke = std::max(now, at + j());   // a timer never fires before it is started
        const double shown = presented(woke, renderMs, vs);
        out.push_back(shown);
        now = woke + renderMs;   // the GUI thread hears of the frame once it is drawn
    }
    return out;
}
} // namespace

class TstFramePacer : public QObject
{
    Q_OBJECT
private slots:
    void wholeVblanks_data()
    {
        QTest::addColumn<double>("hz");
        QTest::addColumn<int>("cap");
        QTest::addColumn<double>("intervalMs");
        QTest::newRow("60 fps at 120 Hz") << 120.0 << 60 << 1000.0 / 60;
        QTest::newRow("60 fps at 60 Hz") << 60.0 << 60 << 1000.0 / 60;
        QTest::newRow("30 fps at 60 Hz") << 60.0 << 30 << 1000.0 / 30;
        QTest::newRow("30 fps at 120 Hz") << 120.0 << 30 << 1000.0 / 30;
        QTest::newRow("60 fps at 144 Hz: 72") << 144.0 << 60 << 2000.0 / 144;
        QTest::newRow("60 fps at 50 Hz: every vblank") << 50.0 << 60 << 20.0;
    }
    void wholeVblanks()
    {
        QFETCH(double, hz);
        QFETCH(int, cap);
        QFETCH(double, intervalMs);
        FramePacer p;
        p.setCap(cap);
        p.next(0, hz);
        const double first = p.dueMs();
        p.next(first, hz);   // drawn right on time
        QVERIFY2(std::abs(p.dueMs() - first - intervalMs) < 1e-6, qPrintable(QString::number(p.dueMs() - first)));
    }

    // Requested half a vblank before it is due; never in the past
    void leadAndLate()
    {
        FramePacer p;
        p.setCap(60);
        p.next(0, 120);
        const double due0 = p.dueMs();
        const double at = p.next(due0 + 2, 120);   // drawn 2 ms after its vblank
        QVERIFY(std::abs(at - (p.dueMs() - 1000.0 / 240)) < 1e-6);
        // A frame drawn after the next one should already have been requested: at once, grid kept
        const double due1 = p.dueMs();
        const double late = p.next(due1 + 14, 120);
        QCOMPARE(late, due1 + 14);
        QVERIFY(std::abs(p.dueMs() - due1 - 1000.0 / 60) < 1e-6);
    }

    // Longer than an interval without a frame (hidden view): a new grid, the next frame at once
    void stall()
    {
        FramePacer p;
        p.setCap(60);
        p.next(0, 120);
        const double at = p.next(5000, 120);
        QVERIFY(at >= 5000 && at <= 5000 + 1000.0 / 240 + 1e-9);
        QVERIFY(p.dueMs() > 5000 && p.dueMs() <= 5000 + 1000.0 / 120);
    }

    void uncapped()
    {
        FramePacer p;
        p.setCap(0);
        QCOMPARE(p.next(123.5, 120), 123.5);
        QCOMPARE(p.next(130.0, 120), 130.0);
    }

    // Known vblanks: every due time is one of them
    void snapsToVblanks()
    {
        FramePacer p;
        p.setCap(60);
        const FramePacer::Vsync vs{1000.0 / 120, 3.21};
        double now = 50;
        for (int k = 0; k < 20; ++k) {
            const double at = p.next(now, 120, vs);
            const double phase = std::fmod(p.dueMs() - vs.vblankMs, vs.periodMs);
            QVERIFY2(std::min(phase, vs.periodMs - phase) < 1e-6, qPrintable(QString::number(phase)));
            now = at + 2.5;
        }
    }

    // The loop with timer jitter: every frame presented exactly N vblanks after the previous
    void evenPacing_data()
    {
        QTest::addColumn<double>("hz");
        QTest::addColumn<int>("cap");
        QTest::addColumn<double>("renderMs");
        QTest::addColumn<int>("vblanks");
        QTest::newRow("120 Hz, 60, light frames") << 120.0 << 60 << 2.0 << 2;
        QTest::newRow("120 Hz, 60, heavy frames") << 120.0 << 60 << 7.0 << 2;
        QTest::newRow("60 Hz, 60") << 60.0 << 60 << 4.0 << 1;
        QTest::newRow("60 Hz, 30") << 60.0 << 30 << 4.0 << 2;
        QTest::newRow("120 Hz, 30") << 120.0 << 30 << 3.0 << 4;
    }
    void evenPacing()
    {
        QFETCH(double, hz);
        QFETCH(int, cap);
        QFETCH(double, renderMs);
        QFETCH(int, vblanks);
        FramePacer p;
        p.setCap(cap);
        const FramePacer::Vsync vs{1000.0 / hz, 0.37};
        const auto shown = run(p, hz, vs, renderMs, 600);
        for (std::size_t k = 2; k < shown.size(); ++k) {
            const double gap = (shown[k] - shown[k - 1]) / vs.periodMs;
            QVERIFY2(std::abs(gap - vblanks) < 1e-6, qPrintable(QStringLiteral("frame %1: %2 vblanks").arg(k).arg(gap)));
        }
    }
};

QTEST_GUILESS_MAIN(TstFramePacer)
#include "tst_framepacer.moc"
