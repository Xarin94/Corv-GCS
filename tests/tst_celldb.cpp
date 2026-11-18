// The LTE cell database (CellDb, port of cell-db.js): PMTiles tile ids and
// blocks, directories and vector tiles decoded from hand-made bytes, CSV layers
// grouped into sites. No network: blocks of the PMTiles copy are not fetched.
#include "core/Geo.h"
#include "services/CellDb.h"

#include <QElapsedTimer>
#include <QFile>
#include <QTemporaryDir>
#include <QtTest>
#include <cmath>

using namespace corv;
using namespace corv::celldb;

namespace {
quint32 crc32(const QByteArray &d)
{
    quint32 c = 0xffffffffu;
    for (const char ch : d) {
        c ^= quint8(ch);
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1)));
    }
    return ~c;
}

/// gzip with one stored (uncompressed) deflate block: enough for the decoder.
QByteArray gzipStored(const QByteArray &data)
{
    QByteArray out("\x1f\x8b\x08\x00\x00\x00\x00\x00\x00\xff", 10);
    out.append(char(0x01));   // BFINAL, BTYPE 00
    const quint16 len = quint16(data.size()), nlen = quint16(~len);
    out.append(char(len & 0xff)).append(char(len >> 8)).append(char(nlen & 0xff)).append(char(nlen >> 8));
    out.append(data);
    const quint32 crc = crc32(data), size = quint32(data.size());
    for (int i = 0; i < 4; ++i) out.append(char((crc >> (8 * i)) & 0xff));
    for (int i = 0; i < 4; ++i) out.append(char((size >> (8 * i)) & 0xff));
    return out;
}

void varint(QByteArray &b, quint64 v)
{
    while (v >= 0x80) {
        b.append(char((v & 0x7f) | 0x80));
        v >>= 7;
    }
    b.append(char(v));
}
void key(QByteArray &b, int field, int wire) { varint(b, quint64(field) << 3 | quint64(wire)); }
void bytesField(QByteArray &b, int field, const QByteArray &v)
{
    key(b, field, 2);
    varint(b, quint64(v.size()));
    b.append(v);
}
QByteArray stringValue(const QByteArray &s)
{
    QByteArray v;
    bytesField(v, 1, s);
    return v;
}
QByteArray uintValue(quint64 n)
{
    QByteArray v;
    key(v, 5, 0);
    varint(v, n);
    return v;
}
quint64 zz(qint64 v) { return quint64((v << 1) ^ (v >> 63)); }
} // namespace

class TstCellDb : public QObject
{
    Q_OBJECT
    QTemporaryDir m_dir;

private slots:
    void tileIds()
    {
        // PMTiles specification vectors
        QCOMPARE(zxyToTileId(0, 0, 0), quint64(0));
        QCOMPARE(zxyToTileId(1, 0, 0), quint64(1));
        QCOMPARE(zxyToTileId(1, 0, 1), quint64(2));
        QCOMPARE(zxyToTileId(1, 1, 1), quint64(3));
        QCOMPARE(zxyToTileId(1, 1, 0), quint64(4));
        QCOMPARE(zxyToTileId(2, 0, 0), quint64(5));
        // Round trips up to the archive's zoom
        for (int z : {3, 8, 14}) {
            const quint32 n = 1u << z;
            for (quint32 x : {0u, 1u, n / 3, n - 1})
                for (quint32 y : {0u, 2u, n / 2, n - 1}) {
                    int zz2;
                    quint32 x2, y2;
                    QVERIFY(tileIdToZxy(zxyToTileId(z, x, y), &zz2, &x2, &y2));
                    QCOMPARE(zz2, z);
                    QCOMPARE(x2, x);
                    QCOMPARE(y2, y);
                }
        }
    }

    void blockIsOneRun()
    {
        // Innsbruck: the zoom-14 tiles of its block are one run of ids, all inside the block
        const int bx = lonToX(11.39, kBlockZ), by = latToY(47.26, kBlockZ);
        const auto [lo, hi] = blockTileRange(bx, by);
        QCOMPARE(hi - lo + 1, quint64(64 * 64));
        for (quint64 id : {lo, lo + 1, (lo + hi) / 2, hi}) {
            int z;
            quint32 x, y;
            QVERIFY(tileIdToZxy(id, &z, &x, &y));
            QCOMPARE(z, kTileZ);
            QCOMPARE(int(x >> 6), bx);
            QCOMPARE(int(y >> 6), by);
        }
        // and the tile of the city is in it
        const quint64 city = zxyToTileId(kTileZ, quint32(lonToX(11.39, kTileZ)), quint32(latToY(47.26, kTileZ)));
        QVERIFY(city >= lo && city <= hi);
    }

    void directory()
    {
        // Three entries: ids 10, 11 (a run of 3), 20; offsets 0, 100 (implicit), 500
        QByteArray d;
        varint(d, 3);
        for (quint64 v : {10, 1, 9}) varint(d, v);          // tile id deltas
        for (quint64 v : {1, 3, 0}) varint(d, v);           // run lengths (0: a leaf directory)
        for (quint64 v : {100, 50, 70}) varint(d, v);       // lengths
        for (quint64 v : {1, 0, 501}) varint(d, v);         // offsets + 1, 0: right after the previous
        bool ok = false;
        const auto e = parseDirectory(gzipStored(d), &ok);
        QVERIFY(ok);
        QCOMPARE(e.size(), std::size_t(3));
        QCOMPARE(e[0].tileId, quint64(10));
        QCOMPARE(e[1].tileId, quint64(11));
        QCOMPARE(e[2].tileId, quint64(20));
        QCOMPARE(e[1].runLength, quint64(3));
        QCOMPARE(e[2].runLength, quint64(0));
        QCOMPARE(e[0].offset, quint64(0));
        QCOMPARE(e[1].offset, quint64(100));
        QCOMPARE(e[2].offset, quint64(500));
        parseDirectory(QByteArray("not gzip"), &ok);
        QVERIFY(!ok);
    }

    void vectorTile()
    {
        // One layer, an LTE cell at the centre of tile (x, y) and a GSM one that is skipped
        QByteArray layer;
        bytesField(layer, 1, "cells");
        for (const char *k : {"radio", "mcc", "net", "area", "cell", "range", "samples"}) bytesField(layer, 3, k);
        bytesField(layer, 4, stringValue("LTE"));   // 0
        bytesField(layer, 4, uintValue(222));       // 1
        bytesField(layer, 4, uintValue(1));         // 2
        bytesField(layer, 4, uintValue(12345));     // 3
        bytesField(layer, 4, uintValue(256000 + 7)); // 4
        bytesField(layer, 4, uintValue(1500));      // 5
        bytesField(layer, 4, uintValue(42));        // 6
        bytesField(layer, 4, stringValue("GSM"));   // 7
        auto feature = [](std::initializer_list<quint64> tags) {
            QByteArray f, t, g;
            for (quint64 v : tags) varint(t, v);
            bytesField(f, 2, t);
            for (quint64 v : {quint64(9), zz(2048), zz(2048)}) varint(g, v);   // MoveTo(1) at the centre
            bytesField(f, 4, g);
            return f;
        };
        bytesField(layer, 2, feature({0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6}));
        bytesField(layer, 2, feature({0, 7, 1, 1, 2, 2, 4, 4}));
        key(layer, 5, 0);
        varint(layer, 4096);
        QByteArray tile;
        bytesField(tile, 3, layer);

        const quint32 x = 8710, y = 5745;   // zoom 14, Innsbruck
        std::vector<Row> rows;
        decodeTile(tile, x, y, rows);
        QCOMPARE(rows.size(), std::size_t(1));
        const Row &r = rows[0];
        QCOMPARE(r.mcc, 222);
        QCOMPARE(r.mnc, 1);
        QCOMPARE(r.tac, 12345u);
        QCOMPARE(r.eci, 256007u);
        QCOMPARE(r.range, 1500);
        QCOMPARE(r.samples, 42);
        const double n = 1 << kTileZ;
        const double lon = (x + 0.5) / n * 360 - 180;
        const double lat = std::atan(std::sinh(kPi * (1 - 2 * (y + 0.5) / n))) * kRad;
        QVERIFY(std::abs(r.lon - lon) < 2e-6);
        QVERIFY(std::abs(r.lat - lat) < 2e-6);
    }

    void csvLayersAndSites()
    {
        const QString dir = m_dir.filePath(QStringLiteral("celldb"));
        QDir().mkpath(dir + QStringLiteral("/sources"));
        // A layer with a header (bundle layout) ...
        {
            QFile f(dir + QStringLiteral("/sources/a.csv"));
            QVERIFY(f.open(QIODevice::WriteOnly));
            f.write("mcc,mnc,tac,eci,lat,lon,range,samples,updated,src\n"
                    "222,1,100,256001,45.00000,9.00000,500,1,1700000000,ocid\n"
                    "222,1,100,256002,45.00200,9.00000,500,3,1720000000,mls\n"     // same eNB 1000
                    "222,10,200,512001,45.01000,9.01000,800,5,1700000000,ocid\n"   // another network
                    "222,88,300,768001,45.02000,9.02000,800,5,1700000000,ocid\n"); // shared cell, under WindTre
        }
        // ... and an OpenCelliD file without one (its own column order)
        {
            QFile f(dir + QStringLiteral("/sources/b.csv"));
            QVERIFY(f.open(QIODevice::WriteOnly));
            f.write("GSM,222,1,5,999,0,9.1,45.1,100,1,1,1600000000,1600000000,0\n"
                    "LTE,222,1,400,1024001,0,9.05000,45.05000,900,2,1,1600000000,1690000000,0\n");
        }
        CellDb db(dir, QString());
        std::optional<CellDb::Result> res;
        CellDb::Query q;
        q.lat = 45;
        q.lon = 9;
        q.radiusKm = 50;
        q.mcc = 222;
        q.mnc = 1;
        q.eci = 768001;   // the serving cell: kept although listed under another MNC
        q.download = false;
        db.query(q, [&](CellDb::Result r) { res = std::move(r); });
        QTRY_VERIFY_WITH_TIMEOUT(res.has_value(), 10000);
        QVERIFY2(res->error.isEmpty(), qPrintable(res->error));
        QCOMPARE(res->sites.size(), std::size_t(3));   // eNB 1000, eNB 4000 (headerless file), the shared eNB 3000
        const CellSite *enb1000 = nullptr, *shared = nullptr, *headerless = nullptr;
        for (const CellSite &s : res->sites) {
            if (s.enb == 1000) enb1000 = &s;
            if (s.enb == 3000) shared = &s;
            if (s.enb == 4000) headerless = &s;
        }
        QVERIFY(enb1000 && shared && headerless);
        QCOMPARE(enb1000->cells.size(), std::size_t(2));
        // The mast: the mean of its cells weighted by their samples
        QVERIFY(std::abs(enb1000->lat - (45.0 * 1 + 45.002 * 3) / 4) < 1e-6);
        QCOMPARE(shared->mnc, 88);
        QVERIFY(std::abs(headerless->lat - 45.05) < 1e-5);
        QVERIFY(std::abs(headerless->lon - 9.05) < 1e-5);

        std::optional<QVariantMap> st;
        db.status([&](QVariantMap m) { st = m; });
        QTRY_VERIFY_WITH_TIMEOUT(st.has_value(), 10000);
        QCOMPARE(st->value(QStringLiteral("files")).toList().size(), 2);
        // a.csv: newest measurement of 2024-07-03 (1720000000)
        bool found = false;
        for (const QVariant &v : st->value(QStringLiteral("files")).toList())
            if (v.toMap().value(QStringLiteral("name")) == QLatin1String("a.csv")) {
                QCOMPARE(v.toMap().value(QStringLiteral("cells")).toInt(), 4);
                QCOMPARE(v.toMap().value(QStringLiteral("newest")).toString(), QStringLiteral("2024-07-03"));
                found = true;
            }
        QVERIFY(found);
    }

    void bundle()
    {
        // The cells shipped with the application: Italy, San Marino, Vatican City
        const QString dir = m_dir.filePath(QStringLiteral("celldb-bundle"));
        CellDb db(dir, QStringLiteral(CORV_BUNDLE_DIR));
        std::optional<CellDb::Result> res;
        CellDb::Query q;
        q.lat = 45.4642;   // Milan
        q.lon = 9.19;
        q.radiusKm = 20;
        q.mcc = 222;
        q.mnc = 1;
        q.download = false;
        QElapsedTimer t;
        t.start();
        db.query(q, [&](CellDb::Result r) { res = std::move(r); });
        QTRY_VERIFY_WITH_TIMEOUT(res.has_value(), 60000);
        qInfo() << "bundle loaded and queried in" << t.elapsed() << "ms:" << res->sites.size() << "TIM sites within 20 km of Milan";
        QVERIFY2(res->error.isEmpty(), qPrintable(res->error));
        QVERIFY(res->sites.size() > 50);
        QCOMPARE(res->missing, 0);
        QCOMPARE(res->date, QStringLiteral("2026-10-05"));
        for (const CellSite &s : res->sites) {
            QCOMPARE(s.mcc, 222);
            QCOMPARE(s.mnc, 1);
            QVERIFY(distanceKm(q.lat, q.lon, s.lat, s.lon) < 21);
        }
    }

    void liveArchive()
    {
        // The 2024 PMTiles copy over the network, a block outside the bundle (Innsbruck, A1)
        if (!qEnvironmentVariableIsSet("CORV_NET_TESTS")) QSKIP("set CORV_NET_TESTS=1 to read the PMTiles archive over the network");
        const QString dir = m_dir.filePath(QStringLiteral("celldb-live"));
        CellDb db(dir, QString());
        std::optional<CellDb::Result> res;
        CellDb::Query q;
        q.lat = 47.26;
        q.lon = 11.39;
        q.radiusKm = 15;
        q.mcc = 232;
        q.mnc = 1;
        QElapsedTimer t;
        t.start();
        db.query(q, [&](CellDb::Result r) { res = std::move(r); });
        QTRY_VERIFY_WITH_TIMEOUT(res.has_value(), 120000);
        qInfo() << "archive block read in" << t.elapsed() << "ms:" << res->sites.size() << "A1 sites within 15 km of Innsbruck";
        QVERIFY2(res->error.isEmpty(), qPrintable(res->error));
        QVERIFY(res->sites.size() > 5);
        // Kept on disk, in the Electron layout: found again offline
        QVERIFY(QFile::exists(dir + QStringLiteral("/index.json")));
        CellDb offline(dir, QString());
        std::optional<CellDb::Result> again;
        q.download = false;
        offline.query(q, [&](CellDb::Result r) { again = std::move(r); });
        QTRY_VERIFY_WITH_TIMEOUT(again.has_value(), 30000);
        QCOMPARE(again->sites.size(), res->sites.size());
    }
};

QTEST_GUILESS_MAIN(TstCellDb)
#include "tst_celldb.moc"
