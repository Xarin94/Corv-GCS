// tst_mavftp - MAVLink FTP client and the packed parameter file (port of
// scripts/test-mavlink-ftp.js).
//
// A fake vehicle answers like GCS_FTP.cpp does (open with a size estimate,
// bursts of 239-byte packets, short last packet or NAK EOF, single reads,
// busy sessions) with configurable packet loss, and the client has to come
// back with the exact file.
#include "link/MavFtp.h"

#include <QElapsedTimer>
#include <QEventLoop>
#include <QTimer>
#include <QtTest>
#include <cmath>
#include <cstring>

using namespace corv;
using namespace corv::ftp;

namespace {

QList<ParamToPack> sampleParams(int n = 1300)
{
    const QStringList groups = {"ATC_RAT_RLL_", "ATC_RAT_PIT_", "BATT_", "COMPASS_OFS_", "EK3_", "INS_", "SERVO", "RC", "GPS_", "WPNAV_"};
    const QStringList suff = {"P", "I", "D", "FF", "FLTD", "MAX", "MIN", "TYPE", "ENABLE", "X", "Y", "Z", "OPTIONS"};
    QList<ParamToPack> out;
    for (int i = 0; i < n; ++i) {
        QString name = groups[i % groups.size()] + (i % 7 ? QString() : QString::number(i % 16) + "_") + suff[(i * 7) % suff.size()]
                       + (i % 3 ? QString() : QString::number(i % 9));
        name = name.left(16);
        const int ptype = 1 + (i % 4);
        double value;
        if (ptype == 1) value = (i % 200) - 100;
        else if (ptype == 2) value = (i * 37) % 60000 - 30000;
        else if (ptype == 3) value = double((qint64(i) * 100003) % 2000000000 - 1000000000);
        else value = double(float(i * 0.137 - 50.5));
        out.append({name, value, ptype});
    }
    return out;
}

struct FakeVehicle {
    QByteArray file;
    MavFtpClient *client = nullptr;
    int dropEvery = 0;
    bool dropReadOnce = false;
    int dropTail = 0;           // packets lost at the end of the file, once, before the NAK that ends it
    bool busyFirst = false;
    bool silent = false;
    int burstLimit = 2000;
    int delayMs = 0;
    int sent = 0, burstSent = 0, reads = 0, opens = 0, resets = 0, terminates = 0;
    bool fd = false;

    void reply(Request r)
    {
        ++sent;
        const QByteArray p = buildPayload(r);
        MavFtpClient *c = client;
        QTimer::singleShot(delayMs, c, [c, p] { c->onPacket(p); });
    }
    void nak(const Packet &req, int code)
    {
        Request r;
        r.seq = uint16_t(req.seq + 1);
        r.session = req.session;
        r.opcode = Nak;
        r.reqOpcode = req.opcode;
        r.data = QByteArray(1, char(code));
        reply(r);
    }
    void onRequest(const QByteArray &payload)
    {
        if (silent) return;
        const Packet req = *parsePayload(payload);
        Request base;
        base.seq = uint16_t(req.seq + 1);
        base.session = req.session;
        base.opcode = Ack;
        base.reqOpcode = req.opcode;
        switch (req.opcode) {
        case ResetSessions:
            ++resets;
            busyFirst = false;
            fd = false;
            reply(base);
            return;
        case TerminateSession:
            ++terminates;
            fd = false;
            reply(base);
            return;
        case OpenFileRO: {
            ++opens;
            if (busyFirst) return nak(req, NoSessionsAvailable);
            if (QString::fromUtf8(req.data) != "@PARAM/param.pck") return nak(req, FileNotFound);
            fd = true;
            QByteArray d(4, '\0');
            qToLittleEndian<quint32>(quint32(file.size() + 400), d.data());   // an estimate, as the firmware gives
            base.data = d;
            reply(base);
            return;
        }
        case ReadFile: {
            ++reads;
            if (!fd) return nak(req, FileNotFound);
            if (req.offset >= uint32_t(file.size())) return nak(req, EndOfFile);
            const QByteArray d = file.mid(req.offset, std::min<qsizetype>(req.size, file.size() - req.offset));
            if (dropReadOnce) {
                dropReadOnce = false;
                return;
            }
            base.offset = req.offset;
            base.data = d;
            reply(base);
            return;
        }
        case BurstReadFile: {
            if (!fd) return nak(req, FileNotFound);
            uint32_t ofs = req.offset;
            uint16_t seq = uint16_t(req.seq + 1);
            const int max = req.size ? req.size : kDataLen;
            for (int i = 0; i < burstLimit; ++i) {
                if (ofs >= uint32_t(file.size())) {
                    Request r;
                    r.seq = seq;
                    r.session = req.session;
                    r.opcode = Nak;
                    r.reqOpcode = req.opcode;
                    r.burstComplete = true;
                    r.offset = ofs;
                    r.data = QByteArray(1, char(EndOfFile));
                    reply(r);
                    return;
                }
                const QByteArray d = file.mid(ofs, std::min<qsizetype>(max, file.size() - ofs));
                const bool last = i == burstLimit - 1 || d.size() < max;
                ++burstSent;
                bool drop = dropEvery && (burstSent % dropEvery == 0) && !last;
                const uint32_t left = uint32_t(file.size()) - ofs;
                if (dropTail && left <= uint32_t(dropTail) * max) {
                    // GCS_FTP.cpp sends the end as an ACK and then a NAK at the end's offset
                    drop = true;
                    if (d.size() < max) {
                        dropTail = 0;
                        Request r;
                        r.seq = uint16_t(seq + 1);
                        r.session = req.session;
                        r.opcode = Nak;
                        r.reqOpcode = req.opcode;
                        r.burstComplete = true;
                        r.offset = uint32_t(file.size());
                        r.data = QByteArray(1, char(EndOfFile));
                        reply(r);
                        return;
                    }
                }
                if (!drop) {
                    Request r;
                    r.seq = seq;
                    r.session = req.session;
                    r.opcode = Ack;
                    r.reqOpcode = req.opcode;
                    r.burstComplete = last;
                    r.offset = ofs;
                    r.data = d;
                    reply(r);
                }
                seq = uint16_t(seq + 1);
                ofs += uint32_t(d.size());
                if (last) return;
            }
            return;
        }
        default:
            nak(req, UnknownCommand);
        }
    }
};

struct Result {
    bool done = false, ok = false;
    QByteArray data;
    QString code, text;
    int progress = 0;
};

/// Run the client's readFile against the fake vehicle until it answers (or 10 s).
Result read(MavFtpClient &client, const QString &path, MavFtpClient::Options o)
{
    Result r;
    QEventLoop loop;
    client.readFile(
        path, o, [&r](const MavFtpClient::Progress &) { ++r.progress; },
        [&](bool ok, const QByteArray &data, const QString &code, const QString &text) {
            r.done = true;
            r.ok = ok;
            r.data = data;
            r.code = code;
            r.text = text;
            loop.quit();
        });
    QTimer::singleShot(10000, &loop, &QEventLoop::quit);
    if (!r.done) loop.exec();
    return r;
}

MavFtpClient::Options opts(int timeoutMs, int retries = 4)
{
    MavFtpClient::Options o;
    o.timeoutMs = timeoutMs;
    o.retries = retries;
    return o;
}

} // namespace

class TestMavFtp : public QObject
{
    Q_OBJECT
    QList<ParamToPack> params = sampleParams(1300);
    QByteArray file = encodeParamPack(params);

    struct Pair {
        FakeVehicle veh;
        std::unique_ptr<MavFtpClient> client;
    };
    std::unique_ptr<Pair> makePair(const QByteArray &f)
    {
        auto p = std::make_unique<Pair>();
        p->veh.file = f;
        FakeVehicle *v = &p->veh;
        p->client = std::make_unique<MavFtpClient>([v](const QByteArray &payload, int, int) { v->onRequest(payload); });
        p->veh.client = p->client.get();
        return p;
    }

private slots:
    void roundTrip()
    {
        const ParamPack d = decodeParamPack(file);
        QVERIFY2(d.ok, qPrintable(d.error));
        QVERIFY(d.complete);
        QCOMPARE(d.total, params.size());
        QCOMPARE(d.params.size(), params.size());
        static const int mav[] = {0, 2, 4, 6, 9};
        for (int i = 0; i < params.size(); ++i) {
            QCOMPARE(d.params[i].name, params[i].name);
            QCOMPARE(d.params[i].value, params[i].value);
            QCOMPARE(d.params[i].type, mav[params[i].ptype]);
        }
        QVERIFY(file.size() > 239 * 20);
    }

    void partialPrefix()
    {
        const ParamPack d = decodeParamPack(file.left(239 * 5));
        QVERIFY2(d.ok, qPrintable(d.error));
        QVERIFY(!d.complete);
        QVERIFY(d.count > 50 && d.count < params.size());
        QCOMPARE(d.total, params.size());
        QCOMPARE(d.params.first().name, params.first().name);
        QCOMPARE(d.params.last().name, params[d.count - 1].name);
    }

    void badMagic() { QVERIFY(!decodeParamPack(QByteArray::fromHex("01020304050607")).ok); }

    void losslessBurst()
    {
        auto p = makePair(file);
        const Result r = read(*p->client, "@PARAM/param.pck", opts(200));
        QVERIFY2(r.ok, qPrintable(r.text));
        QCOMPARE(r.data, file);
        QVERIFY(r.progress > 10);
        QCOMPARE(p->veh.reads, 0);
        QCOMPARE(p->veh.opens, 1);
        QCOMPARE(p->veh.terminates, 1);
    }

    void lostPacketsRefetched()
    {
        auto p = makePair(file);
        p->veh.dropEvery = 5;
        p->veh.dropReadOnce = true;
        const Result r = read(*p->client, "@PARAM/param.pck", opts(100));
        QVERIFY2(r.ok, qPrintable(r.text));
        QCOMPARE(r.data, file);
        QVERIFY(p->veh.reads >= p->veh.burstSent / 5);
    }

    void lostTailRefetched_data()
    {
        QTest::addColumn<int>("packets");
        QTest::newRow("the last one") << 1;
        QTest::newRow("the last three") << 3;
    }

    void lostTailRefetched()
    {
        QFETCH(int, packets);
        auto p = makePair(file);
        p->veh.dropTail = packets;
        const Result r = read(*p->client, "@PARAM/param.pck", opts(100));
        QVERIFY2(r.ok, qPrintable(r.text));
        QCOMPARE(r.data, file);
        QCOMPARE(p->veh.reads, packets);
    }

    void burstLimit()
    {
        auto p = makePair(file);
        p->veh.burstLimit = 10;
        const Result r = read(*p->client, "@PARAM/param.pck", opts(200));
        QVERIFY2(r.ok, qPrintable(r.text));
        QCOMPARE(r.data, file);
        QVERIFY(p->veh.burstSent >= int(std::ceil(file.size() / 239.0)));
    }

    void exactMultipleEndsWithNakEof()
    {
        const QByteArray exact(239 * 7, char(0xA5));
        auto p = makePair(exact);
        const Result r = read(*p->client, "@PARAM/param.pck", opts(200));
        QVERIFY2(r.ok, qPrintable(r.text));
        QCOMPARE(r.data, exact);
    }

    void tinyFile()
    {
        auto p = makePair("hello");
        const Result r = read(*p->client, "@PARAM/param.pck", opts(200));
        QVERIFY2(r.ok, qPrintable(r.text));
        QCOMPARE(r.data, QByteArray("hello"));
    }

    void busySessions()
    {
        auto p = makePair(file);
        p->veh.busyFirst = true;
        const Result r = read(*p->client, "@PARAM/param.pck", opts(200));
        QVERIFY2(r.ok, qPrintable(r.text));
        QCOMPARE(r.data, file);
        QCOMPARE(p->veh.resets, 1);
        QCOMPARE(p->veh.opens, 2);
    }

    void noFtpNoReply()
    {
        auto p = makePair(file);
        p->veh.silent = true;
        QElapsedTimer t;
        t.start();
        const Result r = read(*p->client, "@PARAM/param.pck", opts(50, 2));
        QVERIFY(!r.ok);
        QCOMPARE(r.code, QString("NOREPLY"));
        QVERIFY2(t.elapsed() >= 140 && t.elapsed() < 600, qPrintable(QString::number(t.elapsed())));
    }

    void unknownPath()
    {
        auto p = makePair(file);
        const Result r = read(*p->client, "@PARAM/nothing", opts(100));
        QVERIFY(!r.ok);
        QCOMPARE(r.code, QString("NAK"));
        QVERIFY(r.text.contains("FileNotFound"));
    }

    void cancel()
    {
        auto p = makePair(file);
        p->veh.delayMs = 20;
        MavFtpClient *c = p->client.get();
        QTimer::singleShot(15, c, [c] { c->cancel("link closed"); });
        const Result r = read(*c, "@PARAM/param.pck", opts(500));
        QVERIFY(!r.ok);
        QCOMPARE(r.code, QString("CANCELLED"));
        QVERIFY(!c->busy());
    }

    void secondReadRefused()
    {
        auto p = makePair(file);
        p->veh.delayMs = 10;
        bool firstOk = false;
        QEventLoop loop;
        p->client->readFile("@PARAM/param.pck", opts(500), {}, [&](bool ok, const QByteArray &, const QString &, const QString &) {
            firstOk = ok;
            loop.quit();
        });
        QString code;
        p->client->readFile("@PARAM/param.pck", opts(500), {}, [&](bool, const QByteArray &, const QString &c, const QString &) { code = c; });
        QCOMPARE(code, QString("BUSY"));
        QTimer::singleShot(10000, &loop, &QEventLoop::quit);
        loop.exec();
        QVERIFY(firstOk);
    }
};

QTEST_GUILESS_MAIN(TestMavFtp)
#include "tst_mavftp.moc"
