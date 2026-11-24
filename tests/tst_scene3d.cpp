// The 3D view's pure parts: the GLB model loader (orientation and pivot as
// loadModel() of js/main.js left them), the vehicle type -> model table and the
// trajectory predictor (TrajectoryPredictor.js), the resolution profiles
// (scripts/test-render-backend.js), plus MAVLink frames read through line noise
// on a serial link.
#include "link/Mavlink.h"
#include "link/MavlinkParser.h"
#include "scene/ModelData.h"
#include "scene/RenderQuality.h"
#include "vehicle/TrajectoryPredictor.h"
#include "vehicle/VehicleState.h"

#include <QFile>
#include <QTest>
#include <cmath>
#include <cstring>

using namespace corv;

class TstScene3d : public QObject
{
    Q_OBJECT
private slots:
    void glbPlane()
    {
        QFile f(QStringLiteral(CORV_MODELS_DIR "/plane.glb"));
        QVERIFY(f.open(QIODevice::ReadOnly));
        QString err;
        const auto m = ModelData::fromGlb(f.readAll(), 1.f, QStringLiteral("plane.glb"), &err);
        QVERIFY2(m, qPrintable(err));
        QCOMPARE(m->vertexCount() % 3, 0);
        QVERIFY(m->vertexCount() > 100);
        // Nose forward (-Z) after the quarter turn: the file has the wings across Z (span 10 m)
        // and the fuselage along X (6.5 m); turned, the span lies along X and the length along Z
        float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
        for (std::size_t i = 0; i < m->vertices.size(); i += 9)
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], m->vertices[i + std::size_t(k)]);
                hi[k] = std::max(hi[k], m->vertices[i + std::size_t(k)]);
            }
        QVERIFY(hi[0] - lo[0] > 9.5f && hi[0] - lo[0] < 10.5f);   // wingspan across
        QVERIFY(hi[2] - lo[2] > 6.0f && hi[2] - lo[2] < 8.0f);    // length (propeller included) along the view axis
        // Pivot on the bounding box centre
        for (int k = 0; k < 3; ++k) QVERIFY(std::abs(lo[k] + hi[k]) < 1e-3f);
        QVERIFY(m->radius > 5 && m->radius < 7.5f);
    }

    void glbScaleAndBad()
    {
        QFile f(QStringLiteral(CORV_MODELS_DIR "/quadcopter.glb"));
        QVERIFY(f.open(QIODevice::ReadOnly));
        const QByteArray bytes = f.readAll();
        const auto a = ModelData::fromGlb(bytes, 1.f, QString());
        const auto b = ModelData::fromGlb(bytes, 2.f, QString());
        QVERIFY(a && b);
        QVERIFY(std::abs(b->radius - 2 * a->radius) < 1e-3f);
        QString err;
        QVERIFY(!ModelData::fromGlb(QByteArray("not a model at all"), 1.f, QString(), &err));
        QVERIFY(!err.isEmpty());
        QVERIFY(ModelData::placeholder()->vertexCount() > 0);
    }

    void modelTable()
    {
        QCOMPARE(modelForVehicleType(1), QStringLiteral("plane.glb"));
        QCOMPARE(modelForVehicleType(2), QStringLiteral("quadcopter.glb"));
        QCOMPARE(modelForVehicleType(12), QStringLiteral("rov.glb"));
        QCOMPARE(modelForVehicleType(21), QStringLiteral("quadplane-vtol.glb"));
        QCOMPARE(modelForVehicleType(999), QStringLiteral("plane.glb"));
    }

    void predictionTime()
    {
        QCOMPARE(TrajectoryPredictor::predictionTime(0), 5.0);
        QCOMPARE(TrajectoryPredictor::predictionTime(60), 20.0);
        QVERIFY(std::abs(TrajectoryPredictor::predictionTime(32.5) - 12.5) < 1e-9);
    }

    void straightAndTurn()
    {
        VehicleState s;
        s.gs = 20;
        s.track = 0;   // north... track 0 falls back to the yaw
        s.yaw = 0;
        TrajectoryPredictor p;
        auto path = p.predict(s, 45.0, 10.0, 100, 40, 10, 1000);
        QCOMPARE(int(path.size()), 40);
        // Straight north: ~200 m in 10 s, no drift east, same height
        const double north = (path.back().lat - 45.0) * 111320;
        QVERIFY(std::abs(north - 200) < 1);
        QVERIFY(std::abs(path.back().lon - 10.0) < 1e-9);
        QVERIFY(std::abs(path.back().alt - 100) < 1e-6);

        // 30 degrees right bank: the path curves east
        VehicleState t = s;
        t.roll = 30 * M_PI / 180;
        TrajectoryPredictor q;
        path = q.predict(t, 45.0, 10.0, 100, 40, 10, 1000);
        QVERIFY(path.back().lon > 10.0005);
        // Too slow: no prediction
        VehicleState slow;
        slow.gs = 1;
        TrajectoryPredictor r;
        QVERIFY(r.predict(slow, 45, 10, 100, 40, 10, 1000).empty());
    }

    void resolutionProfiles()
    {
        using namespace renderQuality;
        const auto n = QStringLiteral("native"), b = QStringLiteral("balanced"), e = QStringLiteral("eco");
        QCOMPARE(pixelRatio(n, 2), 2.0);
        QCOMPARE(pixelRatio(b, 2), 1.5);
        QCOMPARE(pixelRatio(e, 2), 1.0);
        QCOMPARE(pixelRatio(e, 0.75), 0.75);   // never more than the display has
        QCOMPARE(pixelRatio(b, 1.25), 1.25);
        QCOMPARE(pixelRatio(b, 0), 1.0);
        QCOMPARE(normalize(QString()), b);
        QCOMPARE(normalize(QStringLiteral("ultra")), b);
    }

    void mavlinkThroughNoise()
    {
        // Every link feeds the parser directly: frames between line noise, in odd chunks
        mavlink_message_t hb;
        mavlink_msg_heartbeat_pack(1, 1, &hb, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0);
        mavlink_message_t gp;
        mavlink_msg_global_position_int_pack(1, 1, &gp, 1000, 455000000, 102500000, 100000, 5000, 0, 0, 0, 0);
        const QByteArray hbBytes = MavlinkParser::serialize(hb), gpBytes = MavlinkParser::serialize(gp);
        int mav = 0, positions = 0;
        double gotLat = 0;
        MavlinkParser parser([&](const mavlink_message_t &m, const QByteArray &) {
            ++mav;
            if (m.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                ++positions;
                gotLat = mavlink_msg_global_position_int_get_lat(&m) / 1e7;
            }
        });
        // Fed in odd chunks, as a USB port delivers them
        const QByteArray stream = hbBytes + QByteArray("\x00\x13\x37\xA5\x5A", 5) + gpBytes + hbBytes;
        for (int i = 0; i < stream.size(); i += 7) parser.push(stream.mid(i, 7));
        QCOMPARE(mav, 3);
        QCOMPARE(positions, 1);
        QVERIFY(std::abs(gotLat - 45.5) < 1e-6);
    }
};

QTEST_GUILESS_MAIN(TstScene3d)
#include "tst_scene3d.moc"
