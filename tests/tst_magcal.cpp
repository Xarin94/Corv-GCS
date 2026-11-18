// The compass calibration geometry: the 80 sections must carry the firmware's
// numbering (AP_GeodesicGrid), since MAG_CAL_PROGRESS.completion_mask is read
// against them, and the sphere fit of the 3D view must find the centre and
// radius of a sphere of samples and refuse a patch.
#include "vehicle/GeodesicGrid.h"
#include "vehicle/MagSamples.h"

#include <QRandomGenerator>
#include <QTest>
#include <cmath>

using namespace corv;

namespace {

// AP_GeodesicGrid::_inverses and _mid_inverses, as in the firmware
const double kInv[10][3][3] = {
    {{-0.309017, 0.500000, 0.190983}, {0.000000, 0.000000, -0.618034}, {-0.309017, -0.500000, 0.190983}},
    {{-0.190983, 0.309017, -0.500000}, {-0.500000, -0.190983, 0.309017}, {0.309017, -0.500000, -0.190983}},
    {{-0.618034, 0.000000, 0.000000}, {0.190983, -0.309017, -0.500000}, {0.190983, -0.309017, 0.500000}},
    {{-0.500000, 0.190983, -0.309017}, {0.000000, -0.618034, 0.000000}, {0.500000, 0.190983, -0.309017}},
    {{-0.190983, -0.309017, -0.500000}, {-0.190983, -0.309017, 0.500000}, {0.618034, 0.000000, 0.000000}},
    {{-0.309017, -0.500000, -0.190983}, {0.190983, 0.309017, -0.500000}, {0.500000, -0.190983, 0.309017}},
    {{0.309017, -0.500000, 0.190983}, {0.000000, 0.000000, -0.618034}, {0.309017, 0.500000, 0.190983}},
    {{0.190983, -0.309017, -0.500000}, {0.500000, 0.190983, 0.309017}, {-0.309017, 0.500000, -0.190983}},
    {{0.500000, -0.190983, -0.309017}, {0.000000, 0.618034, 0.000000}, {-0.500000, -0.190983, -0.309017}},
    {{0.309017, 0.500000, -0.190983}, {-0.500000, 0.190983, 0.309017}, {-0.190983, -0.309017, -0.500000}},
};
const double kMid[10][3][3] = {
    {{0.000000, 1.000000, -0.618034}, {0.000000, -1.000000, -0.618034}, {-0.618034, 0.000000, 1.000000}},
    {{-1.000000, 0.618034, 0.000000}, {0.000000, -1.000000, 0.618034}, {0.618034, 0.000000, -1.000000}},
    {{-0.618034, 0.000000, -1.000000}, {1.000000, -0.618034, 0.000000}, {-0.618034, 0.000000, 1.000000}},
    {{-1.000000, -0.618034, 0.000000}, {1.000000, -0.618034, 0.000000}, {0.000000, 1.000000, -0.618034}},
    {{-1.000000, -0.618034, 0.000000}, {0.618034, 0.000000, 1.000000}, {0.618034, 0.000000, -1.000000}},
    {{-0.618034, 0.000000, -1.000000}, {1.000000, 0.618034, 0.000000}, {0.000000, -1.000000, 0.618034}},
    {{0.000000, -1.000000, -0.618034}, {0.000000, 1.000000, -0.618034}, {0.618034, 0.000000, 1.000000}},
    {{1.000000, -0.618034, 0.000000}, {0.000000, 1.000000, 0.618034}, {-0.618034, 0.000000, -1.000000}},
    {{1.000000, 0.618034, 0.000000}, {-1.000000, 0.618034, 0.000000}, {0.000000, -1.000000, -0.618034}},
    {{0.000000, 1.000000, 0.618034}, {-1.000000, -0.618034, 0.000000}, {0.618034, 0.000000, -1.000000}},
};

void mul(const double m[3][3], const double v[3], double sign, double out[3])
{
    for (int i = 0; i < 3; ++i) out[i] = sign * (m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2]);
}

/// AP_GeodesicGrid::section by definition: the triangle whose vertex basis gives
/// non-negative coordinates, then the sub-triangle from the midpoint basis.
int firmwareSection(const double v[3])
{
    for (int t = 0; t < 20; ++t) {
        double w[3];
        mul(kInv[t % 10], v, t > 9 ? -1 : 1, w);
        if (w[0] < 0 || w[1] < 0 || w[2] < 0) continue;
        double s[3];
        mul(kMid[t % 10], v, t > 9 ? -1 : 1, s);
        const int sub = s[0] < 0 ? 3 : s[1] < 0 ? 1 : s[2] < 0 ? 2 : 0;
        return 4 * t + sub;
    }
    return -1;
}

QVector3D randomUnit(QRandomGenerator &rng)
{
    for (;;) {
        const QVector3D v(float(rng.generateDouble() * 2 - 1), float(rng.generateDouble() * 2 - 1), float(rng.generateDouble() * 2 - 1));
        const float l = v.length();
        if (l > 0.1f && l <= 1) return v / l;
    }
}

} // namespace

class TstMagCal : public QObject
{
    Q_OBJECT
private slots:
    void sectionsMatchFirmware()
    {
        QRandomGenerator rng(1234);
        for (int i = 0; i < 20000; ++i) {
            const QVector3D u = randomUnit(rng);
            const double v[3] = {u.x(), u.y(), u.z()};
            const int fw = firmwareSection(v);
            QVERIFY(fw >= 0);
            QCOMPARE(geogrid::sectionOf(u), fw);
        }
    }

    void sectionCentres()
    {
        // Every section's centre falls in that section; the triangles wind outwards
        const auto &secs = geogrid::sections();
        for (int s = 0; s < geogrid::kSections; ++s) {
            const auto &t = secs[std::size_t(s)];
            QCOMPARE(geogrid::sectionOf(t.center), s);
            const QVector3D n = QVector3D::crossProduct(t.b - t.a, t.c - t.a);
            QVERIFY(QVector3D::dotProduct(n, t.a) > 0);
        }
        QCOMPARE(geogrid::sectionOf(QVector3D()), -1);
    }

    void mask()
    {
        geogrid::Mask m{};
        QCOMPARE(geogrid::maskCount(m), 0);
        m[0] = 0x01;       // section 0
        m[9] = 0x80;       // section 79
        QVERIFY(geogrid::maskHas(m, 0));
        QVERIFY(geogrid::maskHas(m, 79));
        QVERIFY(!geogrid::maskHas(m, 1));
        QCOMPARE(geogrid::maskCount(m), 2);
    }

    void fitSphere()
    {
        // An offset, slightly noisy sphere of 400 mG: the fit finds it
        QRandomGenerator rng(7);
        const QVector3D c(120, -60, 35);
        std::vector<QVector3D> pts;
        for (int i = 0; i < 600; ++i) pts.push_back(c + randomUnit(rng) * float(400 + rng.generateDouble() * 6 - 3));
        const auto f = MagSamples::fitSphere(pts);
        QVERIFY(f.has_value());
        QVERIFY((f->center - c).length() < 2);
        QVERIFY(std::abs(f->r - 400) < 2);

        // One patch of the sphere: no fit
        std::vector<QVector3D> patch;
        for (int i = 0; i < 300; ++i) {
            QVector3D u = randomUnit(rng);
            u.setX(std::abs(u.x()) + 3);
            patch.push_back(c + u.normalized() * 400);
        }
        QVERIFY(!MagSamples::fitSphere(patch).has_value());
        // Too few points
        pts.resize(MagSamples::kFitMinPoints - 1);
        QVERIFY(!MagSamples::fitSphere(pts).has_value());
    }

    void samplesAndTarget()
    {
        MagSamples s;
        QVERIFY(!s.add(0, 0, 0, 0));                 // absent compass
        QVERIFY(s.add(0, 300, 0, 0));
        QVERIFY(!s.add(0, 301, 0, 0));               // within 2 % of the last kept
        QVERIFY(s.add(0, 300, 30, 0));
        QCOMPARE(s.count(0), 2);
        // Provisional frame: around the origin, radius of the latest sample
        const auto m = s.marker();
        QVERIFY(m.has_value());
        QVERIFY(std::abs(m->length() - 1) < 1e-4);
        QCOMPARE(s.target(), -1);                    // no mask, nothing to aim for

        // Everything covered but the section opposite the marker
        geogrid::Mask mask;
        mask.fill(0xFF);
        const int far = geogrid::sectionOf(-*m);
        mask[std::size_t(far >> 3)] &= uint8_t(~(1u << (far & 7)));
        s.setCoverage(mask);
        QCOMPARE(s.target(), far);
        QCOMPARE(s.covered(), 79);
        QCOMPARE(s.current(), geogrid::sectionOf(*m));

        // The ring keeps the newest kMaxPoints
        QRandomGenerator rng(3);
        for (int i = 0; i < MagSamples::kMaxPoints + 500; ++i) {
            const QVector3D u = randomUnit(rng) * 400;
            s.add(1, u.x(), u.y(), u.z());
        }
        QCOMPARE(s.count(1), MagSamples::kMaxPoints);
        s.refit();
        QVERIFY(s.cloud(1).fit.has_value());
        s.clear();
        QCOMPARE(s.count(1), 0);
        QVERIFY(!s.coverage().has_value());
    }
};

QTEST_GUILESS_MAIN(TstMagCal)
#include "tst_magcal.moc"
