// MSP session (msp-manager.js): framing v1 / v2, a fake INAV board answering
// the poll, the flight mode from the active boxes, and a waypoint mission
// written and read back.
#include "link/MspSession.h"

#include <QSignalSpy>
#include <QTest>
#include <QtEndian>

using namespace corv;

namespace {
QByteArray replyFrame(int cmd, const QByteArray &payload)
{
    QByteArray b;
    if (cmd > 254) {
        b.append("$X>", 3);
        b.append(char(0));
        b.append(char(cmd & 0xff)).append(char(cmd >> 8));
        b.append(char(payload.size() & 0xff)).append(char(payload.size() >> 8));
        b.append(payload);
        uint8_t crc = 0;
        for (int i = 3; i < b.size(); ++i) crc = MspSession::crc8DvbS2(crc, uint8_t(b[i]));
        b.append(char(crc));
        return b;
    }
    b.append("$M>", 3);
    b.append(char(payload.size())).append(char(cmd)).append(payload);
    uint8_t x = uint8_t(payload.size()) ^ uint8_t(cmd);
    for (char c : payload) x ^= uint8_t(c);
    b.append(char(x));
    return b;
}

/// The command and payload of a request the session wrote.
bool parseRequest(const QByteArray &b, int *cmd, QByteArray *payload)
{
    if (b.size() < 6 || b[0] != '$') return false;
    if (b[1] == 'M') {
        *cmd = uint8_t(b[4]);
        *payload = b.mid(5, uint8_t(b[3]));
        return true;
    }
    *cmd = uint8_t(b[4]) | (uint8_t(b[5]) << 8);
    *payload = b.mid(8, uint8_t(b[6]) | (uint8_t(b[7]) << 8));
    return true;
}

template <typename T>
QByteArray le(T v)
{
    QByteArray b(sizeof(T), '\0');
    qToLittleEndian<T>(v, b.data());
    return b;
}
} // namespace

class TstMsp : public QObject
{
    Q_OBJECT
private slots:
    void framing()
    {
        // v1: $M< size cmd payload xor
        QCOMPARE(MspSession::encodeRequest(108), QByteArray("$M<\x00\x6c\x6c", 6));
        // v2 for commands over 254, CRC8 DVB-S2 over flag..payload
        const QByteArray v2 = MspSession::encodeRequest(0x2000);
        QCOMPARE(v2.left(3), QByteArray("$X<"));
        QCOMPARE(v2.size(), 9);
        uint8_t crc = 0;
        for (int i = 3; i < 8; ++i) crc = MspSession::crc8DvbS2(crc, uint8_t(v2[i]));
        QCOMPARE(uint8_t(v2[8]), crc);
        MspWaypoint w{3, 1, 457000000, 91000000, 12000, 500, -2, 7, 0xA5};
        MspWaypoint back;
        QVERIFY(MspSession::decodeWaypoint(MspSession::encodeWaypoint(w), &back));
        QCOMPARE(back.lat, w.lat);
        QCOMPARE(back.p2, -2);
        QCOMPARE(back.flag, 0xA5);
    }

    void fakeBoard()
    {
        QList<mavlink_message_t> got;
        QMap<int, QByteArray> stored;   // the board's waypoints
        MspSession *session = nullptr;
        auto answer = [&](const QByteArray &req) {
            int cmd = 0;
            QByteArray p;
            if (!parseRequest(req, &cmd, &p)) return;
            QByteArray r;
            switch (cmd) {
            case 2: r = "INAV"; break;
            case 3: r = QByteArray("\x07\x01\x00", 3); break;
            case 116: r = "ARM;ANGLE;NAV RTH;"; break;
            case 119: r = QByteArray("\x00\x01\x0a", 3); break;
            case 108: r = le<qint16>(123) + le<qint16>(-50) + le<qint16>(270); break;
            case 0x2000: {
                // cycle, i2c, sensors, cpu (u16 each), profile u8, armingFlags u32 (bit 2 armed), 4 x u32 boxes (ANGLE = bit 1)
                r = le<quint16>(1000) + le<quint16>(0) + le<quint16>(0) + le<quint16>(10) + QByteArray(1, '\0') + le<quint32>(0x04)
                    + le<quint32>(0x3) + le<quint32>(0) + le<quint32>(0) + le<quint32>(0);
                break;
            }
            case 20: r = QByteArray(1, '\0') + QByteArray(1, char(60)) + QByteArray(1, char(1)) + QByteArray(1, char(stored.size())); break;
            case 209: stored[uint8_t(p[0])] = p; r = QByteArray(); break;
            case 118: r = stored.value(uint8_t(p[0])); break;
            default: return;   // no pitot, no GPS...: silence, as a real board
            }
            QMetaObject::invokeMethod(session, [&, cmd, r] { session->push(replyFrame(cmd, r)); }, Qt::QueuedConnection);
        };
        MspSession s(answer, [&](const mavlink_message_t &m) { got << m; }, QStringLiteral("normal"));
        session = &s;
        QTRY_VERIFY_WITH_TIMEOUT(!s.fcVersion().isEmpty(), 3000);
        QCOMPARE(s.fcVariant(), QStringLiteral("INAV"));
        QCOMPARE(s.fcVersion(), QStringLiteral("7.1.0"));
        // Attitude and the heartbeat with the mode of the active box
        auto find = [&](uint32_t id) -> const mavlink_message_t * {
            for (int i = int(got.size()) - 1; i >= 0; --i)
                if (got[i].msgid == id) return &got[i];
            return nullptr;
        };
        QTRY_VERIFY_WITH_TIMEOUT(find(MAVLINK_MSG_ID_ATTITUDE) && find(MAVLINK_MSG_ID_HEARTBEAT), 3000);
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(find(MAVLINK_MSG_ID_ATTITUDE), &att);
        QVERIFY(std::abs(att.roll - 12.3 * M_PI / 180) < 1e-4);
        QVERIFY(std::abs(att.yaw - 270 * M_PI / 180) < 1e-4);
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(find(MAVLINK_MSG_ID_HEARTBEAT), &hb);
        QCOMPARE(int(hb.base_mode), 209);   // armed
        QCOMPARE(int(hb.custom_mode), 2);   // ANGLE

        // A mission written and read back
        QSignalSpy done(&s, &MspSession::missionDone);
        QList<MspWaypoint> wps;
        for (int i = 0; i < 4; ++i) wps << MspWaypoint{0, 1, 457000000 + i * 1000, 91000000, 5000, 0, 0, 0, i == 3 ? 0xA5 : 0};
        s.missionUpload(wps, false);
        QTRY_COMPARE_WITH_TIMEOUT(done.count(), 1, 5000);
        QCOMPARE(done[0][0].toString(), QStringLiteral("upload"));
        QVERIFY(done[0][1].toBool());
        QCOMPARE(stored.size(), 4);
        s.missionDownload();
        QTRY_COMPARE_WITH_TIMEOUT(done.count(), 2, 5000);
        QVERIFY(done[1][1].toBool());
        const QVariantList back = done[1][3].toMap().value(QStringLiteral("wps")).toList();
        QCOMPARE(back.size(), 4);
        QCOMPARE(back[2].toMap().value(QStringLiteral("lat")).toInt(), 457002000);
        QCOMPARE(back[3].toMap().value(QStringLiteral("flag")).toInt(), 0xA5);
    }
};

QTEST_GUILESS_MAIN(TstMsp)
#include "tst_msp.moc"
