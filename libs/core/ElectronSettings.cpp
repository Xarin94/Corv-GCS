#include "core/ElectronSettings.h"

#include "core/DebugLog.h"
#include "core/Settings.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>
#include <QStringDecoder>
#include <QtEndian>

using namespace Qt::StringLiterals;

namespace corv::electronSettings {

namespace {

constexpr quint64 kTableMagic = 0xdb4775248b80fb57ull;
constexpr qsizetype kLogBlock = 32768;
constexpr qsizetype kFooter = 48;
const QByteArray kOrigin = QByteArrayLiteral("_file://\0").append('\1');   // then the key, Latin-1
constexpr qint64 kMaxFile = 64 * 1024 * 1024;

/// A cursor over bytes; ok goes false for good past the end.
struct Bytes {
    const QByteArray &d;
    qsizetype pos = 0;
    bool ok = true;

    quint64 varint()
    {
        quint64 v = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            if (pos >= d.size()) break;
            const quint8 b = quint8(d[pos++]);
            v |= quint64(b & 0x7f) << shift;
            if (!(b & 0x80)) return v;
        }
        ok = false;
        return 0;
    }
    QByteArray take(quint64 n)
    {
        if (!ok || n > quint64(d.size() - pos)) {
            ok = false;
            return {};
        }
        const QByteArray out = d.mid(pos, qsizetype(n));
        pos += qsizetype(n);
        return out;
    }
};

/// The newest value of every key seen.
class Newest
{
public:
    void put(const QByteArray &key, quint64 seq, bool deleted, const QByteArray &value)
    {
        Entry &e = m_entries[key];
        if (e.seen && e.seq > seq) return;
        e = {seq, true, deleted, value};
    }
    QHash<QString, QString> page() const
    {
        QHash<QString, QString> out;
        for (auto it = m_entries.cbegin(); it != m_entries.cend(); ++it) {
            if (it->deleted || !it.key().startsWith(kOrigin)) continue;
            out.insert(QString::fromLatin1(it.key().mid(kOrigin.size())), text(it->value));
        }
        return out;
    }

private:
    struct Entry {
        quint64 seq = 0;
        bool seen = false, deleted = false;
        QByteArray value;
    };
    /// The first byte says how a string is stored: 0 UTF-16, 1 Latin-1.
    static QString text(const QByteArray &v)
    {
        if (v.isEmpty()) return {};
        const QByteArray body = v.mid(1);
        if (v[0] == 0) return QStringDecoder(QStringDecoder::Utf16LE)(body);
        return QString::fromLatin1(body);
    }
    QHash<QByteArray, Entry> m_entries;
};

/// The write-ahead log: records in blocks of 32 KB, each a batch of writes.
void readLog(const QByteArray &d, Newest &out)
{
    QByteArray record;
    auto batch = [&out](const QByteArray &r) {
        if (r.size() < 12) return;
        quint64 seq = qFromLittleEndian<quint64>(r.constData());
        Bytes b{r, 12};
        while (b.ok && b.pos < r.size()) {
            const int tag = quint8(r[b.pos++]);
            const QByteArray key = b.take(b.varint());
            const QByteArray value = tag == 1 ? b.take(b.varint()) : QByteArray();
            if (!b.ok || tag > 1) return;
            out.put(key, seq++, tag == 0, value);
        }
    };
    for (qsizetype block = 0; block < d.size(); block += kLogBlock) {
        const qsizetype end = std::min(block + kLogBlock, d.size());
        for (qsizetype p = block; p + 7 <= end;) {
            const int len = qFromLittleEndian<quint16>(d.constData() + p + 4), type = quint8(d[p + 6]);
            if (type < 1 || type > 4 || p + 7 + len > end) break;   // the padding at the end, or a torn write
            const QByteArray part = d.mid(p + 7, len);
            p += 7 + len;
            if (type == 1 || type == 2) record = part;   // full, first
            else record += part;                         // middle, last
            if (type == 1 || type == 4) batch(record);
        }
    }
}

/// A block of a table: prefix-compressed entries, then the restart points.
template <typename F>
void eachEntry(const QByteArray &block, F &&fn)
{
    if (block.size() < 4) return;
    const quint32 restarts = qFromLittleEndian<quint32>(block.constData() + block.size() - 4);
    const qsizetype end = block.size() - 4 - qsizetype(restarts) * 4;
    if (end < 0) return;
    const QByteArray entries = block.left(end);
    Bytes b{entries};
    QByteArray key;
    while (b.ok && b.pos < end) {
        const quint64 shared = b.varint(), rest = b.varint(), valueLen = b.varint();
        if (!b.ok || shared > quint64(key.size())) return;
        key = key.left(qsizetype(shared)) + b.take(rest);
        const QByteArray value = b.take(valueLen);
        if (b.ok) fn(key, value);
    }
}

QByteArray readBlock(const QByteArray &file, quint64 offset, quint64 size)
{
    if (offset + size + 1 > quint64(file.size())) return {};
    const QByteArray raw = file.mid(qsizetype(offset), qsizetype(size));
    const int type = quint8(file[qsizetype(offset + size)]);
    return type == 1 ? snappyDecompress(raw) : type == 0 ? raw : QByteArray();
}

void readTable(const QByteArray &d, Newest &out)
{
    if (d.size() < kFooter || qFromLittleEndian<quint64>(d.constData() + d.size() - 8) != kTableMagic) return;
    const QByteArray footer = d.right(kFooter);
    Bytes b{footer};
    b.varint();   // the meta index: offset and size
    b.varint();
    const quint64 offset = b.varint(), size = b.varint();
    if (!b.ok) return;
    eachEntry(readBlock(d, offset, size), [&](const QByteArray &, const QByteArray &handle) {
        Bytes h{handle};
        const quint64 o = h.varint(), n = h.varint();
        if (!h.ok) return;
        eachEntry(readBlock(d, o, n), [&](const QByteArray &key, const QByteArray &value) {
            // The key, then 8 bytes: sequence and type (0 deleted)
            if (key.size() < 8) return;
            const quint64 tail = qFromLittleEndian<quint64>(key.constData() + key.size() - 8);
            out.put(key.left(key.size() - 8), tail >> 8, (tail & 0xff) == 0, value);
        });
    });
}

} // namespace

QByteArray snappyDecompress(const QByteArray &in)
{
    Bytes b{in};
    const quint64 want = b.varint();
    if (!b.ok || want > quint64(kMaxFile)) return {};
    QByteArray out;
    out.reserve(qsizetype(want));
    while (b.ok && b.pos < in.size()) {
        const quint8 tag = quint8(in[b.pos++]);
        if ((tag & 3) == 0) {
            quint64 len = tag >> 2;
            if (len >= 60) {
                const QByteArray n = b.take(len - 59);
                len = 0;
                for (qsizetype i = n.size(); i-- > 0;) len = len << 8 | quint8(n[i]);
            }
            out += b.take(len + 1);
            continue;
        }
        quint64 len = 0, off = 0;
        const int extra = (tag & 3) == 1 ? 1 : (tag & 3) == 2 ? 2 : 4;
        const QByteArray n = b.take(quint64(extra));
        for (qsizetype i = n.size(); i-- > 0;) off = off << 8 | quint8(n[i]);
        if ((tag & 3) == 1) {
            len = 4 + ((tag >> 2) & 7);
            off |= quint64(tag >> 5) << 8;
        } else {
            len = 1 + (tag >> 2);
        }
        if (!b.ok || off == 0 || off > quint64(out.size()) || out.size() + qsizetype(len) > qsizetype(want)) return {};
        // May overlap what it writes: byte by byte
        for (quint64 i = 0; i < len; ++i) out.append(out[out.size() - qsizetype(off)]);
    }
    return b.ok && quint64(out.size()) == want ? out : QByteArray();
}

QString findStore()
{
    // Electron's userData folder, named after the package or the product
    const QString base = QStandardPaths::writableLocation(QStandardPaths::GenericConfigLocation);
#ifdef Q_OS_WIN
    const QString roaming = QDir::fromNativeSeparators(qEnvironmentVariable("APPDATA"));
#elif defined(Q_OS_MACOS)
    const QString roaming = QDir::homePath() + u"/Library/Application Support"_s;
#else
    const QString roaming = base;
#endif
    QString best;
    QDateTime when;
    for (const QString &root : {roaming, base})
        for (const QString &name : {u"corv-gcs"_s, u"CORV GCS"_s, u"Corv-GCS"_s}) {
            const QString dir = root + '/'_L1 + name + u"/Local Storage/leveldb"_s;
            if (!QFileInfo::exists(dir + u"/CURRENT"_s)) continue;
            if (const QDateTime t = lastUsed(dir); best.isEmpty() || t > when) {
                best = dir;
                when = t;
            }
        }
    return best;
}

QDateTime lastUsed(const QString &dir)
{
    QDateTime out;
    for (const QFileInfo &f : QDir(dir).entryInfoList({u"*.log"_s, u"*.ldb"_s}, QDir::Files))
        if (!out.isValid() || f.lastModified() > out) out = f.lastModified();
    return out;
}

QHash<QString, QString> read(const QString &dir, QString *error)
{
    Newest newest;
    int files = 0;
    for (const QFileInfo &info : QDir(dir).entryInfoList({u"*.ldb"_s, u"*.sst"_s, u"*.log"_s}, QDir::Files)) {
        QFile f(info.filePath());
        if (info.size() > kMaxFile || !f.open(QIODevice::ReadOnly)) continue;
        ++files;
        if (info.suffix() == "log"_L1) readLog(f.readAll(), newest);
        else readTable(f.readAll(), newest);
    }
    const QHash<QString, QString> out = newest.page();
    if (error && out.isEmpty()) *error = files ? u"No settings found in "_s + dir : u"Nothing to read in "_s + dir;
    return out;
}

const QStringList &knownKeys()
{
    // Everything 1.8.8 kept, but MUTE GCS OUTPUT: a program that starts mute has to be told to
    static const QStringList keys = {
        u"adsb-enabled"_s, u"bat-vmax"_s, u"bat-vmin"_s, u"corv-relnav"_s, u"corv.fp.link"_s, u"corv.fp.radio"_s,
        u"corv.lastHome"_s, u"corv.params.favorites"_s, u"corv.params.learnedNames"_s, u"corv.platform"_s,
        u"datad-hud-cells"_s, u"datad-joystick-config"_s, u"ekf-status-enabled"_s, u"fft-panel-enabled"_s,
        u"fft-panel-sensor"_s, u"fpv-settings"_s, u"gcs-lang"_s, u"geozones-enabled"_s, u"ground-clamp-enabled"_s,
        u"lidar-config"_s, u"log-fft-cfg"_s, u"lte-cell-map"_s, u"lte-link-config"_s, u"modelScale"_s,
        u"ntrip-settings"_s, u"renderFpsCap"_s, u"renderQuality"_s, u"ros-config"_s, u"rotor-load-cfg"_s,
        u"satDetailMaxZoom"_s, u"satelliteRadiusM"_s, u"selectedModel"_s, u"setup-nav-collapsed"_s, u"telfwd-settings"_s,
        u"terrainRadiusM"_s};
    return keys;
}

int importInto(const QHash<QString, QString> &store)
{
    int n = 0;
    for (const QString &key : knownKeys()) {
        const auto it = store.constFind(key);
        if (it == store.cend()) continue;
        // Objects and lists were JSON text; the page's folded menu still is
        const QJsonDocument doc = QJsonDocument::fromJson(it->toUtf8());
        if (key == "setup-nav-collapsed"_L1 || doc.isNull()) Settings::instance().setValue(key, *it);
        else Settings::instance().setJson(key, doc.isArray() ? QJsonValue(doc.array()) : QJsonValue(doc.object()));
        ++n;
    }
    LOG_I("settings") << n << "settings of CORV GCS 1.x imported";
    return n;
}

} // namespace corv::electronSettings
