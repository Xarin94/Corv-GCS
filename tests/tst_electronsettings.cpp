// ElectronSettings: the localStorage of CORV GCS 1.x out of Chromium's LevelDB. The
// files are made here as LevelDB writes them: a log, a plain table and a Snappy one.
#include "core/ElectronSettings.h"
#include "core/Settings.h"

#include <QJsonObject>
#include <QTemporaryDir>
#include <QTest>
#include <QtEndian>

using namespace corv;
using namespace Qt::StringLiterals;

namespace {
QByteArray varint(quint64 v)
{
    QByteArray o;
    for (; v >= 0x80; v >>= 7) o.append(char(v | 0x80));
    o.append(char(v));
    return o;
}
template <typename T>
QByteArray le(T v)
{
    QByteArray o(sizeof(T), 0);
    qToLittleEndian(v, o.data());
    return o;
}
QByteArray pageKey(const char *k) { return QByteArray("_file://") + '\0' + '\1' + k; }
QByteArray latin(const char *v) { return QByteArray("\1") + v; }
QByteArray utf16(const QString &v)
{
    QByteArray o(1, 0);
    for (const QChar c : v) o += le<quint16>(c.unicode());
    return o;
}

struct Put {
    QByteArray key, value;
    bool deleted = false;
};
QByteArray batch(quint64 seq, const QList<Put> &puts)
{
    QByteArray r = le<quint64>(seq) + le<quint32>(quint32(puts.size()));
    for (const Put &p : puts) {
        r.append(p.deleted ? '\0' : '\1');
        r += varint(quint64(p.key.size())) + p.key;
        if (!p.deleted) r += varint(quint64(p.value.size())) + p.value;
    }
    return r;
}
/// Records into 32 KB blocks, split where they cross one.
QByteArray logFile(const QList<QByteArray> &records)
{
    QByteArray out;
    for (const QByteArray &r : records) {
        qsizetype pos = 0;
        bool first = true;
        do {
            const qsizetype left = 32768 - out.size() % 32768;
            if (left < 7) {
                out.append(QByteArray(left, 0));
                continue;
            }
            const qsizetype n = std::min(r.size() - pos, left - 7);
            const bool last = pos + n == r.size();
            out += le<quint32>(0) + le<quint16>(quint16(n));
            out.append(char(first && last ? 1 : first ? 2 : last ? 4 : 3));
            out += r.mid(pos, n);
            pos += n;
            first = false;
        } while (pos < r.size());
    }
    return out;
}
/// Entries with prefix compression and one restart point.
QByteArray block(const QList<std::pair<QByteArray, QByteArray>> &entries)
{
    QByteArray out, prev;
    for (const auto &[k, v] : entries) {
        qsizetype shared = 0;
        while (shared < std::min(k.size(), prev.size()) && k[shared] == prev[shared]) ++shared;
        out += varint(quint64(shared)) + varint(quint64(k.size() - shared)) + varint(quint64(v.size())) + k.mid(shared) + v;
        prev = k;
    }
    return out + le<quint32>(0) + le<quint32>(1);
}
QByteArray internalKey(const QByteArray &k, quint64 seq, bool deleted) { return k + le<quint64>(seq << 8 | (deleted ? 0 : 1)); }
/// One literal of everything, as a Snappy encoder that finds no matches.
QByteArray snappyLiteral(const QByteArray &d)
{
    QByteArray o = varint(quint64(d.size()));
    const quint32 n = quint32(d.size() - 1);
    if (n < 60) o.append(char(n << 2));
    else if (n < 256) o.append(char(60 << 2)).append(char(n));
    else o.append(char(61 << 2)).append(char(n & 0xff)).append(char(n >> 8));
    return o + d;
}
QByteArray table(const QList<std::pair<QByteArray, QByteArray>> &entries, bool snappy)
{
    const QByteArray data = snappy ? snappyLiteral(block(entries)) : block(entries);
    QByteArray f = data;
    f.append(char(snappy ? 1 : 0));
    f += le<quint32>(0);
    const quint64 indexAt = quint64(f.size());
    const QByteArray index = block({{entries.last().first, varint(0) + varint(quint64(data.size()))}});
    f += index;
    f.append('\0');
    f += le<quint32>(0);
    QByteArray footer = varint(0) + varint(0) + varint(indexAt) + varint(quint64(index.size()));
    footer.append(QByteArray(40 - footer.size(), 0));
    return f + footer + le<quint64>(0xdb4775248b80fb57ull);
}
void write(const QString &path, const QByteArray &d)
{
    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write(d);
}
} // namespace

class TestElectronSettings : public QObject
{
    Q_OBJECT
private slots:
    void snappy()
    {
        // "abcdabcdabcd…": a literal, then a copy that overlaps what it writes
        QByteArray in = varint(16);
        in.append(char(3 << 2)).append("abcd");
        in.append(char((8 - 4) << 2 | 1)).append(char(4));    // 1-byte offset, length 8
        in.append(char((4 - 1) << 2 | 2)).append(char(4)).append('\0');   // 2-byte offset, length 4
        QCOMPARE(electronSettings::snappyDecompress(in), QByteArray("abcdabcdabcdabcd"));
        // Malformed: nothing, not a crash
        QVERIFY(electronSettings::snappyDecompress(in.left(7)).isEmpty());
        QByteArray far = varint(8);
        far.append(char(3 << 2)).append("abcd").append(char((4 - 4) << 2 | 1)).append(char(9));
        QVERIFY(electronSettings::snappyDecompress(far).isEmpty());
    }

    void newestValueWins()
    {
        QTemporaryDir dir;
        const QString d = dir.path() + '/'_L1;
        // An old table, a newer one, and the log over them
        write(d + u"000005.ldb"_s, table({{internalKey(pageKey("bat-vmax"), 10, false), latin("12.6")},
                                          {internalKey(pageKey("bat-vmin"), 11, false), latin("9.0")},
                                          {internalKey(pageKey("gcs-lang"), 12, false), latin("en")}}, false));
        write(d + u"000008.ldb"_s, table({{internalKey(pageKey("bat-vmax"), 30, false), latin("16.8")},
                                          {internalKey(pageKey("bat-vmin"), 31, true), {}}}, true));
        write(d + u"000009.log"_s, logFile({batch(40, {{"META:file://", "x"}, {pageKey("gcs-lang"), latin("zh")}}),
                                            batch(42, {{pageKey("corv.params.favorites"), utf16(u"[\"Ångström\"]"_s)},
                                                       {QByteArray("_https://elsewhere") + '\0' + '\1' + "gcs-lang", latin("fr")}})}));
        const auto got = electronSettings::read(dir.path());
        QCOMPARE(got.value(u"bat-vmax"_s), u"16.8"_s);
        QVERIFY(!got.contains(u"bat-vmin"_s));   // deleted later
        QCOMPARE(got.value(u"gcs-lang"_s), u"zh"_s);
        QCOMPARE(got.value(u"corv.params.favorites"_s), u"[\"Ångström\"]"_s);
        QCOMPARE(got.size(), 3);
    }

    void recordsAcrossBlocks()
    {
        QTemporaryDir dir;
        const QByteArray big = latin(QByteArray(70000, 'q').constData());
        write(dir.path() + u"/000001.log"_s, logFile({batch(1, {{pageKey("ros-config"), big}}), batch(2, {{pageKey("modelScale"), latin("1.5")}})}));
        const auto got = electronSettings::read(dir.path());
        QCOMPARE(got.value(u"ros-config"_s).size(), 70000);
        QCOMPARE(got.value(u"modelScale"_s), u"1.5"_s);
    }

    void damagedFilesGiveNothing()
    {
        QTemporaryDir dir;
        write(dir.path() + u"/000001.log"_s, QByteArray(500, '\x7f'));
        write(dir.path() + u"/000002.ldb"_s, QByteArray(500, '\x7f'));
        QString error;
        QVERIFY(electronSettings::read(dir.path(), &error).isEmpty());
        QVERIFY(!error.isEmpty());
    }

    void importKeepsWhatTheProgramReads()
    {
        QTemporaryDir dir;
        Settings::instance().load(dir.filePath(u"settings.json"_s));
        const int n = electronSettings::importInto({{u"rotor-load-cfg"_s, u"{\"enabled\":true,\"pwmMin\":1000}"_s},
                                                    {u"setup-nav-collapsed"_s, u"{\"tuning\":true}"_s},
                                                    {u"terrainRadiusM"_s, u"50000"_s},
                                                    {u"gcs-muted"_s, u"1"_s},
                                                    {u"some-extension"_s, u"x"_s}});
        QCOMPARE(n, 3);
        QCOMPARE(Settings::instance().json(u"rotor-load-cfg"_s).toObject().value(u"pwmMin"_s).toInt(), 1000);
        QCOMPARE(Settings::instance().value(u"setup-nav-collapsed"_s).toString(), u"{\"tuning\":true}"_s);
        QCOMPARE(Settings::instance().value(u"terrainRadiusM"_s).toDouble(), 50000.0);
        // A program that starts mute has to be told to
        QVERIFY(!Settings::instance().contains(u"gcs-muted"_s));
        QVERIFY(!Settings::instance().contains(u"some-extension"_s));
    }
};

QTEST_GUILESS_MAIN(TestElectronSettings)
#include "tst_electronsettings.moc"
