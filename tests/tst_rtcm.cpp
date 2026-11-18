// The byte-level half of rtk-manager.js (services/Rtcm):
//   - RTCM3 frames out of a stream cut anywhere, junk skipped, message number read
//   - GPS_RTCM_DATA fragments: one with flags 0 up to 180 bytes, else 180-byte
//     pieces with the fragmented bit, the index and the 5-bit sequence id
//   - the GGA sentence (format and checksum) for both hemispheres
//   - the caster sourcetable and the chunked transfer of NTRIP v2
#include "services/Rtcm.h"

#include <QTest>

using namespace corv::rtcm;

namespace {

QByteArray frame(int type, int payloadLen)
{
    QByteArray f;
    f.append(char(0xD3));
    f.append(char((payloadLen >> 8) & 0x03));
    f.append(char(payloadLen & 0xFF));
    QByteArray p(payloadLen, '\x55');
    if (payloadLen >= 2) {
        p[0] = char(type >> 4);
        p[1] = char(((type & 0x0F) << 4) | 0x05);
    }
    f.append(p);
    f.append("\x01\x02\x03", 3);   // CRC-24 (not checked)
    return f;
}

} // namespace

class TstRtcm : public QObject
{
    Q_OBJECT
private slots:
    void framing()
    {
        const QByteArray a = frame(1005, 19), b = frame(1077, 300), c = frame(4072, 1);
        const QByteArray stream = QByteArray("junk") + a + b + c;
        Parser p;
        QList<Frame> got;
        for (int i = 0; i < stream.size(); i += 7) got += p.parse(stream.mid(i, 7));
        QCOMPARE(got.size(), 3);
        QCOMPARE(got[0].type, 1005);
        QCOMPARE(got[0].raw, a);
        QCOMPARE(got[1].type, 1077);
        QCOMPARE(got[1].length, 300);
        QCOMPARE(got[1].raw, b);
        QCOMPARE(got[2].type, 0);   // one payload byte: no message number
        QCOMPARE(typeName(1077), QStringLiteral("GPS MSM7"));
        QCOMPARE(typeName(1999), QStringLiteral("Type 1999"));
    }

    void fragmentsOf()
    {
        const QByteArray small = frame(1005, 19);
        auto f = fragments(small, 7);
        QCOMPARE(f.size(), 1);
        QCOMPARE(f[0].first, uint8_t(0));
        QCOMPARE(f[0].second, small);

        const QByteArray big = frame(1077, 400);   // 406 bytes: 180 + 180 + 46
        f = fragments(big, 37);                    // sequence id 37 & 31 = 5
        QCOMPARE(f.size(), 3);
        for (int i = 0; i < 3; ++i) QCOMPARE(f[i].first, uint8_t(1 | (i << 1) | (5 << 3)));
        QCOMPARE(f[0].second.size(), 180);
        QCOMPARE(f[2].second.size(), 46);
        QCOMPARE(f[0].second + f[1].second + f[2].second, big);
    }

    void gga()
    {
        const QDateTime t(QDate(2026, 11, 13), QTime(9, 5, 7), QTimeZone::UTC);
        const QByteArray g = buildGga(45.5, -9.25, 123.45, t);
        QVERIFY(g.startsWith("$GPGGA,090507.00,4530.0000000,N,00915.0000000,W,1,10,1.0,123.5,M,0.0,M,,*"));
        QVERIFY(g.endsWith("\r\n"));
        // checksum: XOR of the bytes between $ and *
        const QByteArray body = g.mid(1, g.indexOf('*') - 1);
        uint8_t cs = 0;
        for (char c : body) cs ^= uint8_t(c);
        QCOMPARE(g.mid(g.indexOf('*') + 1, 2), QByteArray::number(cs, 16).rightJustified(2, '0').toUpper());
        QVERIFY(buildGga(-33.9, 151.2, 0, t).contains(",S,15112.0000000,E,"));
    }

    void sourcetable()
    {
        const QByteArray reply = "SOURCETABLE 200 OK\r\nServer: x\r\n\r\n"
                                 "STR;MP1;Milano;RTCM 3.2;1004(1);2;GPS+GLO;NET;ITA;45.46;9.19;1;0;sNTRIP;none;B;N;9600;\r\n"
                                 "CAS;caster;2101;x;y;0;ITA;45;9;\r\n"
                                 "STR;MP2;;RTCM 3.3;;2;GPS;;CHE;46.0;8.9;0;0;;none;N;N;0;\r\n"
                                 "ENDSOURCETABLE\r\n";
        QList<Mount> m;
        QString err;
        QVERIFY(parseSourcetable(reply, &m, &err));
        QCOMPARE(m.size(), 2);
        QCOMPARE(m[0].mountpoint, QStringLiteral("MP1"));
        QCOMPARE(m[0].format, QStringLiteral("RTCM 3.2"));
        QCOMPARE(m[0].navSystem, QStringLiteral("GPS+GLO"));
        QCOMPARE(m[0].country, QStringLiteral("ITA"));
        QCOMPARE(m[0].lat, 45.46);
        QCOMPARE(m[1].lon, 8.9);
        QVERIFY(!parseSourcetable("HTTP/1.1 401 Unauthorized\r\n\r\n", &m, &err));
        QCOMPARE(err, QStringLiteral("Sourcetable request failed: HTTP/1.1 401 Unauthorized"));
    }

    void dechunk()
    {
        const QByteArray payload = frame(1005, 19) + frame(1074, 100);
        const QByteArray chunked = QByteArray::number(20, 16) + "\r\n" + payload.left(20) + "\r\n" +
                                   QByteArray::number(payload.size() - 20, 16) + "\r\n" + payload.mid(20) + "\r\n";
        Dechunker d;
        QByteArray out;
        for (int i = 0; i < chunked.size(); i += 5) out += d.feed(chunked.mid(i, 5));
        QCOMPARE(out, payload);
    }
};

QTEST_GUILESS_MAIN(TstRtcm)
#include "tst_rtcm.moc"
