#include "services/CellDb.h"

#include "core/DebugLog.h"
#include "core/Geo.h"
#include "core/Inflate.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QByteArrayView>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMutex>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QTimeZone>
#include <QTimer>
#include <QtEndian>
#include <QUrlQuery>
#include <QWaitCondition>
#include <QtConcurrent>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <list>
#include <stdexcept>

namespace corv {

// ── Tiles, PMTiles directories, vector tiles ─────────────────────────────────

namespace celldb {

namespace {
constexpr int kSub = 1 << (kTileZ - kBlockZ);   // zoom-14 tiles per block side (64)

quint64 firstId(int z) { return ((quint64(1) << (2 * z)) - 1) / 3; }   // tiles of all lower zooms

void rotate(quint32 n, quint32 &x, quint32 &y, quint32 rx, quint32 ry)
{
    if (ry == 0) {
        if (rx == 1) {
            x = n - 1 - x;
            y = n - 1 - y;
        }
        std::swap(x, y);
    }
}

bool readVarint(const uchar *b, qsizetype end, qsizetype &pos, quint64 &out)
{
    out = 0;
    int shift = 0;
    while (pos < end && shift < 64) {
        const uchar c = b[pos++];
        out |= quint64(c & 0x7f) << shift;
        if (!(c & 0x80)) return true;
        shift += 7;
    }
    return false;
}

qint64 zigzag(quint64 v) { return qint64(v >> 1) ^ -qint64(v & 1); }

/// Protobuf fields of [start, end): cb(field, wire, value, a, b) - value for varints,
/// [a, b) for length-delimited, a for fixed32 / fixed64. False on a malformed message.
template <class F>
bool fields(const uchar *buf, qsizetype start, qsizetype end, F cb)
{
    qsizetype pos = start;
    while (pos < end) {
        quint64 key;
        if (!readVarint(buf, end, pos, key)) return false;
        const int field = int(key >> 3), wire = int(key & 7);
        if (wire == 0) {
            quint64 v;
            if (!readVarint(buf, end, pos, v)) return false;
            cb(field, wire, v, 0, 0);
        } else if (wire == 2) {
            quint64 len;
            if (!readVarint(buf, end, pos, len) || pos + qsizetype(len) > end) return false;
            cb(field, wire, 0, pos, pos + qsizetype(len));
            pos += qsizetype(len);
        } else if (wire == 1) {
            if (pos + 8 > end) return false;
            cb(field, wire, 0, pos, pos + 8);
            pos += 8;
        } else if (wire == 5) {
            if (pos + 4 > end) return false;
            cb(field, wire, 0, pos, pos + 4);
            pos += 4;
        } else {
            return false;
        }
    }
    return true;
}

std::vector<quint64> packed(const uchar *b, qsizetype a, qsizetype e)
{
    std::vector<quint64> out;
    quint64 v;
    while (a < e && readVarint(b, e, a, v)) out.push_back(v);
    return out;
}

struct Value {
    bool isString = false;
    QString s;
    double d = 0;
};

Value decodeValue(const uchar *b, qsizetype start, qsizetype end)
{
    Value v;
    fields(b, start, end, [&](int f, int, quint64 val, qsizetype a, qsizetype e) {
        if (f == 1) {
            v.isString = true;
            v.s = QString::fromUtf8(reinterpret_cast<const char *>(b + a), e - a);
        } else if (f == 2) {
            float x;
            std::memcpy(&x, b + a, 4);
            v.d = x;
        } else if (f == 3) {
            std::memcpy(&v.d, b + a, 8);
        } else if (f == 4 || f == 5) {
            v.d = double(val);
        } else if (f == 6) {
            v.d = double(zigzag(val));
        } else if (f == 7) {
            v.d = val ? 1 : 0;
        }
    });
    return v;
}
} // namespace

int lonToX(double lon, int z) { return int(std::floor((lon + 180) / 360 * (1 << z))); }

int latToY(double lat, int z)
{
    const double r = lat * kDeg;
    return int(std::floor((1 - std::asinh(std::tan(r)) / kPi) / 2 * (1 << z)));
}

quint64 zxyToTileId(int z, quint32 x, quint32 y)
{
    const quint32 n = quint32(1) << z;
    quint64 d = 0;
    for (quint32 s = n >> 1; s > 0; s >>= 1) {
        const quint32 rx = (x & s) ? 1 : 0, ry = (y & s) ? 1 : 0;
        d += quint64(s) * s * ((3 * rx) ^ ry);
        rotate(n, x, y, rx, ry);
    }
    return firstId(z) + d;
}

bool tileIdToZxy(quint64 id, int *zOut, quint32 *xOut, quint32 *yOut)
{
    quint64 acc = 0;
    for (int z = 0; z < 27; ++z) {
        const quint64 count = quint64(1) << (2 * z);
        if (id < acc + count) {
            quint64 t = id - acc;
            quint32 x = 0, y = 0;
            for (quint32 s = 1; s < (quint32(1) << z); s *= 2) {
                const quint32 rx = quint32(1 & (t >> 1));
                const quint32 ry = quint32(1 & (t ^ rx));
                rotate(s, x, y, rx, ry);
                x += s * rx;
                y += s * ry;
                t >>= 2;
            }
            *zOut = z;
            *xOut = x;
            *yOut = y;
            return true;
        }
        acc += count;
    }
    return false;
}

std::pair<quint64, quint64> blockTileRange(int bx, int by)
{
    const quint64 d = zxyToTileId(kBlockZ, quint32(bx), quint32(by)) - firstId(kBlockZ);
    const quint64 lo = firstId(kTileZ) + d * kSub * kSub;
    return {lo, lo + kSub * kSub - 1};
}

std::vector<DirEntry> parseDirectory(const QByteArray &gz, bool *ok)
{
    if (ok) *ok = false;
    const QByteArray buf = gunzip(gz);
    if (buf.isEmpty()) return {};
    const auto *b = reinterpret_cast<const uchar *>(buf.constData());
    const qsizetype end = buf.size();
    qsizetype pos = 0;
    quint64 n;
    if (!readVarint(b, end, pos, n) || n > 10000000) return {};
    std::vector<DirEntry> e(n);
    quint64 last = 0, v;
    for (quint64 k = 0; k < n; ++k) {
        if (!readVarint(b, end, pos, v)) return {};
        last += v;
        e[k].tileId = last;
    }
    for (quint64 k = 0; k < n; ++k) {
        if (!readVarint(b, end, pos, e[k].runLength)) return {};
    }
    for (quint64 k = 0; k < n; ++k) {
        if (!readVarint(b, end, pos, e[k].length)) return {};
    }
    for (quint64 k = 0; k < n; ++k) {
        if (!readVarint(b, end, pos, v)) return {};
        e[k].offset = (v == 0 && k > 0) ? e[k - 1].offset + e[k - 1].length : v - 1;
    }
    if (ok) *ok = true;
    return e;
}

void decodeTile(const QByteArray &mvt, quint32 x, quint32 y, std::vector<Row> &out)
{
    const auto *b = reinterpret_cast<const uchar *>(mvt.constData());
    fields(b, 0, mvt.size(), [&](int field, int wire, quint64, qsizetype ls, qsizetype le) {
        if (field != 3 || wire != 2) return;   // layers
        QStringList keys;
        std::vector<Value> values;
        std::vector<std::pair<qsizetype, qsizetype>> features;
        double extent = 4096;
        fields(b, ls, le, [&](int f, int, quint64 val, qsizetype a, qsizetype e) {
            if (f == 3) keys << QString::fromUtf8(reinterpret_cast<const char *>(b + a), e - a);
            else if (f == 4) values.push_back(decodeValue(b, a, e));
            else if (f == 2) features.emplace_back(a, e);
            else if (f == 5) extent = double(val);
        });
        const double n = double(1 << kTileZ);
        for (const auto &[fs, fe] : features) {
            std::vector<quint64> tags, geom;
            fields(b, fs, fe, [&](int f, int, quint64, qsizetype a, qsizetype e) {
                if (f == 2) tags = packed(b, a, e);
                else if (f == 4) geom = packed(b, a, e);
            });
            if (geom.size() < 3) continue;
            QString radio;
            double mcc = 0, net = 0, area = 0, cell = 0, range = 0, samples = 0;
            for (std::size_t k = 0; k + 1 < tags.size(); k += 2) {
                if (tags[k] >= quint64(keys.size()) || tags[k + 1] >= values.size()) continue;
                const QString &key = keys[int(tags[k])];
                const Value &val = values[tags[k + 1]];
                if (key == QLatin1String("radio")) radio = val.s;
                else if (key == QLatin1String("mcc")) mcc = val.d;
                else if (key == QLatin1String("net")) net = val.d;
                else if (key == QLatin1String("area")) area = val.d;
                else if (key == QLatin1String("cell")) cell = val.d;
                else if (key == QLatin1String("range")) range = val.d;
                else if (key == QLatin1String("samples")) samples = val.d;
            }
            if (radio != QLatin1String("LTE") || !(cell > 0)) continue;
            const double px = x + double(zigzag(geom[1])) / extent, py = y + double(zigzag(geom[2])) / extent;
            const double lon = px / n * 360 - 180;
            const double lat = std::atan(std::sinh(kPi * (1 - 2 * py / n))) * kRad;
            Row r;
            r.lat = std::round(lat * 1e6) / 1e6;
            r.lon = std::round(lon * 1e6) / 1e6;
            r.mcc = int(mcc);
            r.mnc = int(net);
            r.tac = unsigned(area);
            r.eci = unsigned(cell);
            r.range = int(range);
            r.samples = int(samples);
            out.push_back(r);
        }
    });
}

double distanceKm(double lat1, double lon1, double lat2, double lon2)
{
    const double a = std::pow(std::sin((lat2 - lat1) * kDeg / 2), 2) +
                     std::cos(lat1 * kDeg) * std::cos(lat2 * kDeg) * std::pow(std::sin((lon2 - lon1) * kDeg / 2), 2);
    return 12742 * std::asin(std::sqrt(a));
}

} // namespace celldb

// ── The database ─────────────────────────────────────────────────────────────

using namespace celldb;

namespace {
const QByteArray kUserAgent = QByteArrayLiteral("Corv-GCS cell-db");   // the CDN refuses requests without one; beaconDB asks for one
const QString kBeaconDbUrl = QStringLiteral("https://api.beacondb.net/v1/geolocate");
const QString kOcidCellUrl = QStringLiteral("https://opencellid.org/cell/get");
const QString kOcidDownloadUrl = QStringLiteral("https://opencellid.org/ocid/downloads");
constexpr qint64 kLookupMissMs = 6 * 3600 * 1000;   // a cell not found online is asked again after this
constexpr int kLookupTimeoutMs = 10000;
constexpr int kRangeTimeoutMs = 60000;
constexpr qint64 kMergeGap = 256 * 1024;            // read across gaps smaller than this...
constexpr qint64 kMaxRange = 16 * 1024 * 1024;      // ...up to this many bytes per request
constexpr std::size_t kMemBlocks = 48;              // decoded blocks kept in memory

struct Error : std::runtime_error {
    explicit Error(const QString &m) : std::runtime_error(m.toStdString()) {}
    QString text() const { return QString::fromStdString(what()); }
};

using Rows = std::vector<Row>;
using RowsPtr = std::shared_ptr<const Rows>;

/// A CSV layer: its LTE cells in columns, ordered by block so a block is one slice.
struct Layer {
    QString name, file;
    bool bundle = false;
    int cells = 0;
    QString newest;                  // date of the newest measurement
    std::vector<float> lat, lon;
    std::vector<qint32> num;         // mcc, mnc, tac, eci, range, samples
    std::vector<quint8> src;
    QStringList srcNames;
    std::vector<qint32> start;       // per block key: its first row; size blocks + 1

    bool has(int key) const { return !start.empty() && start[std::size_t(key) + 1] > start[std::size_t(key)]; }
};
using LayerPtr = std::shared_ptr<const Layer>;

int blockKey(int bx, int by) { return bx * (1 << kBlockZ) + by; }

double toNumber(const char *a, const char *b)
{
    while (a < b && (*a == ' ' || *a == '"')) ++a;
    while (b > a && (b[-1] == ' ' || b[-1] == '"')) --b;
    if (a == b) return 0;   // as JavaScript's +""
    bool ok = false;
    const double v = QByteArrayView(a, b - a).toDouble(&ok);   // C locale whatever the system's
    return ok ? v : std::nan("");
}
int toInt(double v) { return std::isfinite(v) ? int(qint64(v)) : 0; }   // as JavaScript's | 0

LayerPtr loadCsvLayer(const QString &file, const QString &name)
{
    const QString base = QFileInfo(file).fileName();
    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) throw Error(base + QStringLiteral(": cannot read the file"));
    QByteArray data = f.readAll();
    f.close();
    if (data.size() >= 2 && uchar(data[0]) == 0x1f && uchar(data[1]) == 0x8b) {
        data = gunzip(data, data.size() * 5);
        if (data.isEmpty()) throw Error(base + QStringLiteral(": not a valid gzip file"));
    }
    auto layer = std::make_shared<Layer>();
    layer->name = name;
    layer->file = file;
    layer->srcNames << name;
    QHash<QByteArray, int> srcIndex{{name.toUtf8(), 0}};

    struct Cols {
        int mcc = -1, mnc = -1, tac = -1, eci = -1, lat = -1, lon = -1, range = -1, samples = -1, updated = -1, radio = -1, src = -1;
    } col;
    bool header = false;
    std::vector<float> lat, lon;
    std::vector<qint32> num, key;
    std::vector<quint8> src;
    qint64 newest = 0;
    std::vector<std::pair<const char *, const char *>> fld;
    const char *p = data.constData(), *end = p + data.size();
    while (p < end) {
        const char *nl = static_cast<const char *>(std::memchr(p, '\n', std::size_t(end - p)));
        const char *ls = p, *le = nl ? nl : end;
        p = nl ? nl + 1 : end;
        if (le > ls && le[-1] == '\r') --le;
        if (le == ls) continue;
        fld.clear();
        for (const char *q = ls;;) {
            const char *c = static_cast<const char *>(std::memchr(q, ',', std::size_t(le - q)));
            if (!c) {
                fld.emplace_back(q, le);
                break;
            }
            fld.emplace_back(q, c);
            q = c + 1;
        }
        if (!header) {
            header = true;
            QStringList names;
            for (const auto &[a, b] : fld) names << QString::fromUtf8(a, b - a).trimmed().toLower().remove(QLatin1Char('"'));
            static const QStringList radios{QStringLiteral("gsm"), QStringLiteral("umts"), QStringLiteral("lte"), QStringLiteral("nr"),
                                            QStringLiteral("cdma")};
            if (radios.contains(names.value(0))) {
                // No header: the column order of OpenCelliD's own files
                col = {1, 2, 3, 4, 7, 6, 8, 9, 12, 0, -1};
            } else {
                auto find = [&](std::initializer_list<const char *> alts) {
                    for (int i = 0; i < names.size(); ++i)
                        for (const char *a : alts)
                            if (names[i] == QLatin1String(a)) return i;
                    return -1;
                };
                col.mcc = find({"mcc"});
                col.mnc = find({"mnc", "net"});
                col.tac = find({"tac", "area", "lac"});
                col.eci = find({"eci", "cell", "cellid"});
                col.lat = find({"lat"});
                col.lon = find({"lon"});
                col.range = find({"range"});
                col.samples = find({"samples"});
                col.updated = find({"updated"});
                col.radio = find({"radio"});
                col.src = find({"src"});
                const std::pair<int, const char *> required[] = {{col.mcc, "mcc"}, {col.mnc, "mnc"}, {col.eci, "eci"}, {col.lat, "lat"}, {col.lon, "lon"}};
                for (const auto &[c, n] : required)
                    if (c < 0) throw Error(QStringLiteral("%1: no \"%2\" column").arg(base, QLatin1String(n)));
                continue;
            }
        }
        auto field = [&](int i) { return i >= 0 && i < int(fld.size()) ? fld[std::size_t(i)] : std::pair<const char *, const char *>{nullptr, nullptr}; };
        auto number = [&](int i) {
            const auto [a, b] = field(i);
            return a ? toNumber(a, b) : std::nan("");
        };
        if (col.radio >= 0) {
            const auto [a, b] = field(col.radio);
            if (!a || b - a != 3 || std::memcmp(a, "LTE", 3) != 0) continue;
        }
        const double la = number(col.lat), lo = number(col.lon), eci = number(col.eci);
        if (!std::isfinite(la) || !std::isfinite(lo) || (la == 0 && lo == 0) || !(eci > 0) || std::abs(la) > 85) continue;
        if (col.updated >= 0) newest = std::max(newest, qint64(std::isfinite(number(col.updated)) ? number(col.updated) : 0));
        lat.push_back(float(la));
        lon.push_back(float(lo));
        num.insert(num.end(), {toInt(number(col.mcc)), toInt(number(col.mnc)), col.tac >= 0 ? toInt(number(col.tac)) : 0, qint32(qint64(eci)),
                               col.range >= 0 ? toInt(number(col.range)) : 0, col.samples >= 0 ? toInt(number(col.samples)) : 0});
        key.push_back(blockKey(lonToX(lo, kBlockZ), latToY(la, kBlockZ)));
        quint8 s = 0;
        if (col.src >= 0) {
            const auto [a, b] = field(col.src);
            if (a && b > a) {
                const QByteArray name8(a, b - a);
                auto it = srcIndex.find(name8);
                if (it == srcIndex.end()) {
                    if (layer->srcNames.size() < 255) {
                        it = srcIndex.insert(name8, int(layer->srcNames.size()));
                        layer->srcNames << QString::fromUtf8(name8);
                    }
                }
                if (it != srcIndex.end()) s = quint8(it.value());
            }
        }
        src.push_back(s);
    }
    // Counting sort by block
    const int n = int(lat.size());
    const int blocks = 1 << (2 * kBlockZ);
    layer->start.assign(std::size_t(blocks) + 1, 0);
    for (int i = 0; i < n; ++i) {
        const int k = std::clamp(key[std::size_t(i)], 0, blocks - 1);
        key[std::size_t(i)] = k;
        ++layer->start[std::size_t(k) + 1];
    }
    for (int b = 0; b < blocks; ++b) layer->start[std::size_t(b) + 1] += layer->start[std::size_t(b)];
    std::vector<qint32> fill(layer->start.begin(), layer->start.end() - 1);
    layer->lat.resize(std::size_t(n));
    layer->lon.resize(std::size_t(n));
    layer->src.resize(std::size_t(n));
    layer->num.resize(std::size_t(n) * 6);
    for (int i = 0; i < n; ++i) {
        const std::size_t j = std::size_t(fill[std::size_t(key[std::size_t(i)])]++);
        layer->lat[j] = lat[std::size_t(i)];
        layer->lon[j] = lon[std::size_t(i)];
        layer->src[j] = src[std::size_t(i)];
        std::copy_n(num.begin() + std::ptrdiff_t(i) * 6, 6, layer->num.begin() + std::ptrdiff_t(j) * 6);
    }
    layer->cells = n;
    if (newest) layer->newest = QDateTime::fromSecsSinceEpoch(newest, QTimeZone::UTC).toString(QStringLiteral("yyyy-MM-dd"));
    return layer;
}

/// The cells of one block of a CSV layer.
Rows layerRows(const Layer &l, int bx, int by)
{
    const int k = blockKey(bx, by);
    if (!l.has(k)) return {};
    Rows out;
    const qint32 a = l.start[std::size_t(k)], b = l.start[std::size_t(k) + 1];
    out.reserve(std::size_t(b - a));
    for (qint32 j = a; j < b; ++j) {
        const qint32 *v = &l.num[std::size_t(j) * 6];
        Row r;
        // float keeps ~0.3 m: round back to the 5 decimals of the files
        r.lat = std::round(double(l.lat[std::size_t(j)]) * 1e5) / 1e5;
        r.lon = std::round(double(l.lon[std::size_t(j)]) * 1e5) / 1e5;
        r.mcc = v[0];
        r.mnc = v[1];
        r.tac = unsigned(v[2]);
        r.eci = unsigned(v[3]);
        r.range = v[4];
        r.samples = v[5];
        r.src = l.srcNames.value(l.src[std::size_t(j)]);
        out.push_back(std::move(r));
    }
    return out;
}

struct Http {
    int status = 0;
    QByteArray body, etag, lastModified;
    QString error;
};

/// A blocking request on this worker thread; aborted at the timeout or when `cancel` is set.
Http http(const QUrl &url, const QByteArray &verb, const QByteArray &body, const QList<std::pair<QByteArray, QByteArray>> &headers, int timeoutMs,
          const std::atomic<bool> *cancel)
{
    thread_local std::unique_ptr<QNetworkAccessManager> nam;
    if (!nam) nam = std::make_unique<QNetworkAccessManager>();
    QNetworkRequest req(url);
    req.setHeader(QNetworkRequest::UserAgentHeader, kUserAgent);
    req.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    for (const auto &[k, v] : headers) req.setRawHeader(k, v);
    QNetworkReply *r = verb == "POST" ? nam->post(req, body) : nam->get(req);
    QEventLoop loop;
    QTimer timeout, poll;
    timeout.setSingleShot(true);
    QObject::connect(&timeout, &QTimer::timeout, r, &QNetworkReply::abort);
    QObject::connect(&poll, &QTimer::timeout, r, [r, cancel] {
        if (cancel && cancel->load()) r->abort();
    });
    QObject::connect(r, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    timeout.start(timeoutMs);
    poll.start(200);
    if (!r->isFinished()) loop.exec();
    Http out;
    out.status = r->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    out.body = r->readAll();
    out.etag = r->rawHeader("ETag");
    out.lastModified = r->rawHeader("Last-Modified");
    if (r->error() != QNetworkReply::NoError && out.status == 0) out.error = r->errorString();
    delete r;
    return out;
}

QJsonObject readJson(const QString &file)
{
    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) return {};
    return QJsonDocument::fromJson(f.readAll()).object();
}

void writeFile(const QString &file, const QByteArray &data)
{
    QDir().mkpath(QFileInfo(file).absolutePath());
    QSaveFile f(file);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(data);
        f.commit();
    }
}

void writeJson(const QString &file, const QJsonObject &o) { writeFile(file, QJsonDocument(o).toJson(QJsonDocument::Compact)); }

std::vector<std::pair<int, int>> blocksForBox(double south, double west, double north, double east)
{
    std::vector<std::pair<int, int>> out;
    const int x0 = lonToX(west, kBlockZ), x1 = lonToX(east, kBlockZ);
    const int y0 = latToY(north, kBlockZ), y1 = latToY(south, kBlockZ);
    for (int x = x0; x <= x1; ++x)
        for (int y = y0; y <= y1; ++y) out.emplace_back(x, y);
    return out;
}

std::vector<std::pair<int, int>> blocksForRadius(double lat, double lon, double radiusKm)
{
    const double dLat = radiusKm / 111.2;
    const double dLon = radiusKm / (111.2 * std::max(0.05, std::cos(lat * kDeg)));
    return blocksForBox(lat - dLat, lon - dLon, lat + dLat, lon + dLon);
}

quint64 cellKey(int mcc, int mnc, unsigned eci)
{
    return (quint64(quint32(mcc) & 0x3ff) << 38) | (quint64(quint32(mnc) & 0x3ff) << 28) | (eci & 0xfffffff);
}
} // namespace

struct CellDb::Core {
    QString dir, bundleDir, url = QString::fromLatin1(kSourceUrl);
    std::atomic<bool> cancelled{false};

    QString blocksDir() const { return dir + QStringLiteral("/blocks"); }
    QString leafDir() const { return dir + QStringLiteral("/leaves"); }
    QString sourcesDir() const { return dir + QStringLiteral("/sources"); }
    QString indexFile() const { return dir + QStringLiteral("/index.json"); }
    QString lookupFile() const { return dir + QStringLiteral("/lookups.json"); }
    QString settingsFile() const { return dir + QStringLiteral("/settings.json"); }

    struct Header {
        quint64 rootOffset = 0, rootLength = 0, leafOffset = 0, leafLength = 0, dataOffset = 0, dataLength = 0;
        bool clustered = false;
        int internalCompression = 0, tileCompression = 0, tileType = 0, minZoom = 0, maxZoom = 0;
    };

    QMutex lock;   // everything below
    bool indexLoaded = false;
    QJsonObject index;   // etag, date, blocks: { "x_y": cells }
    bool lookupsLoaded = false;
    QJsonObject lookups;   // "mcc-mnc-eci": { lat, lon, range, samples, src, tac, at } | { miss, at }
    bool headerLoaded = false;
    Header header;
    std::vector<DirEntry> root;
    QHash<quint64, std::vector<DirEntry>> leaves;
    std::list<std::pair<QString, RowsPtr>> memBlocks;   // most recent first
    QSet<QString> inFlight;
    QWaitCondition inFlightDone;
    std::optional<std::vector<LayerPtr>> layers;
    bool manifestLoaded = false;
    QJsonObject manifest;
    qint64 requests = 0, bytes = 0;

    QMutex openLock;     // one archive opening at a time
    QMutex layersLoad;   // one layer load at a time

    // ── Settings, index, lookups (call with the lock held) ──
    QJsonObject settingsLocked() const
    {
        QJsonObject s = readJson(settingsFile());
        if (!s.contains(QStringLiteral("online"))) s.insert(QStringLiteral("online"), true);
        if (!s.contains(QStringLiteral("token"))) s.insert(QStringLiteral("token"), QString());
        return s;
    }
    QJsonObject &indexLocked()
    {
        if (!indexLoaded) {
            index = readJson(indexFile());
            if (!index.value(QStringLiteral("blocks")).isObject()) index = QJsonObject{{QStringLiteral("blocks"), QJsonObject()}};
            indexLoaded = true;
        }
        return index;
    }
    QJsonObject &lookupsLocked()
    {
        if (!lookupsLoaded) {
            lookups = readJson(lookupFile());
            lookupsLoaded = true;
        }
        return lookups;
    }
    QJsonObject manifestLocked()
    {
        if (!manifestLoaded) {
            manifest = bundleDir.isEmpty() ? QJsonObject() : readJson(bundleDir + QStringLiteral("/manifest.json"));
            manifestLoaded = true;
        }
        return manifest;
    }

    // ── CSV layers ──
    QStringList sourceFiles() const
    {
        QDir d(sourcesDir());
        QFileInfoList files = d.entryInfoList({QStringLiteral("*.csv"), QStringLiteral("*.csv.gz"), QStringLiteral("*.CSV"), QStringLiteral("*.CSV.GZ")},
                                              QDir::Files, QDir::Time);   // newest first
        QStringList out;
        for (const QFileInfo &f : files) out << f.fileName();
        return out;
    }

    std::vector<LayerPtr> loadLayers()
    {
        QMutexLocker load(&layersLoad);
        {
            QMutexLocker l(&lock);
            if (layers) return *layers;
        }
        std::vector<LayerPtr> out;
        QElapsedTimer t;
        t.start();
        for (const QString &f : sourceFiles()) {
            try {
                out.push_back(loadCsvLayer(sourcesDir() + QLatin1Char('/') + f, f));
            } catch (const Error &e) {
                LOG_W("celldb") << f << ":" << e.text();
            }
        }
        QJsonObject m;
        {
            QMutexLocker l(&lock);
            m = manifestLocked();
        }
        if (!m.isEmpty()) {
            try {
                auto layer = std::const_pointer_cast<Layer>(loadCsvLayer(bundleDir + QLatin1Char('/') + m.value(QStringLiteral("file")).toString(),
                                                                         QStringLiteral("bundle")));
                layer->bundle = true;
                out.push_back(layer);
            } catch (const Error &e) {
                LOG_W("celldb") << "bundled cells:" << e.text();
            }
        }
        if (!out.empty()) {
            QStringList parts;
            for (const LayerPtr &l : out) parts << QStringLiteral("%1 %2").arg(l->name).arg(l->cells);
            LOG_I("celldb") << parts.join(QStringLiteral(", ")) << "LTE cells loaded in" << t.elapsed() << "ms";
        }
        QMutexLocker l(&lock);
        layers = out;
        return out;
    }

    void resetLayers()
    {
        QMutexLocker l(&lock);
        layers.reset();
    }

    // ── PMTiles ──
    Http range(quint64 offset, quint64 length)
    {
        const QByteArray r = QByteArrayLiteral("bytes=") + QByteArray::number(offset) + '-' + QByteArray::number(offset + length - 1);
        Http h = http(QUrl(url), "GET", {}, {{QByteArrayLiteral("Range"), r}}, kRangeTimeoutMs, &cancelled);
        // 200 = the server ignored Range and sent the whole archive
        if (h.status != 206) throw Error(h.error.isEmpty() ? QStringLiteral("cell database: HTTP %1").arg(h.status) : h.error);
        QMutexLocker l(&lock);
        ++requests;
        bytes += h.body.size();
        return h;
    }

    void open()
    {
        QMutexLocker ol(&openLock);
        {
            QMutexLocker l(&lock);
            if (headerLoaded) return;
        }
        const Http r = range(0, 16384);
        const QByteArray &b = r.body;
        if (b.size() < 127 || !b.startsWith("PMTiles") || uchar(b[7]) != 3) throw Error(QStringLiteral("not a PMTiles v3 archive"));
        auto u64 = [&](int o) {
            quint64 v;
            std::memcpy(&v, b.constData() + o, 8);
            return qFromLittleEndian(v);
        };
        Header h;
        h.rootOffset = u64(8);
        h.rootLength = u64(16);
        h.leafOffset = u64(40);
        h.leafLength = u64(48);
        h.dataOffset = u64(56);
        h.dataLength = u64(64);
        h.clustered = uchar(b[96]) == 1;
        h.internalCompression = uchar(b[97]);
        h.tileCompression = uchar(b[98]);
        h.tileType = uchar(b[99]);
        h.minZoom = uchar(b[100]);
        h.maxZoom = uchar(b[101]);
        if (h.tileType != 1 || h.tileCompression != 2 || h.internalCompression != 2 || !h.clustered)
            throw Error(QStringLiteral("cell database: unexpected archive layout"));
        if (h.maxZoom != kTileZ) throw Error(QStringLiteral("cell database: max zoom %1, expected %2").arg(h.maxZoom).arg(kTileZ));
        const QByteArray rootBytes = h.rootOffset + h.rootLength <= quint64(b.size()) ? b.mid(qsizetype(h.rootOffset), qsizetype(h.rootLength))
                                                                                      : range(h.rootOffset, h.rootLength).body;
        bool ok = false;
        std::vector<DirEntry> rootDir = parseDirectory(rootBytes, &ok);
        if (!ok) throw Error(QStringLiteral("cell database: unreadable root directory"));
        QMutexLocker l(&lock);
        QJsonObject &idx = indexLocked();
        const QString etag = QString::fromLatin1(r.etag);
        if (!etag.isEmpty() && !idx.value(QStringLiteral("etag")).toString().isEmpty() && etag != idx.value(QStringLiteral("etag")).toString()) {
            QDir(leafDir()).removeRecursively();
            leaves.clear();
            LOG_I("celldb") << "the archive changed: blocks downloaded from now on are from the new copy";
        }
        if (!etag.isEmpty()) idx.insert(QStringLiteral("etag"), etag);
        const QDateTime modified = QDateTime::fromString(QString::fromLatin1(r.lastModified), Qt::RFC2822Date);
        if (modified.isValid()) idx.insert(QStringLiteral("date"), modified.toUTC().toString(QStringLiteral("yyyy-MM-dd")));
        writeJson(indexFile(), idx);
        header = h;
        root = std::move(rootDir);
        headerLoaded = true;
    }

    std::vector<DirEntry> leaf(quint64 offset, quint64 length)
    {
        {
            QMutexLocker l(&lock);
            auto it = leaves.constFind(offset);
            if (it != leaves.cend()) return it.value();
        }
        const QString file = leafDir() + QStringLiteral("/%1.gz").arg(offset);
        QByteArray gz;
        QFile f(file);
        if (f.open(QIODevice::ReadOnly)) gz = f.readAll();
        f.close();
        if (quint64(gz.size()) != length) {
            quint64 base;
            {
                QMutexLocker l(&lock);
                base = header.leafOffset;
            }
            gz = range(base + offset, length).body;
            writeFile(file, gz);
        }
        bool ok = false;
        std::vector<DirEntry> entries = parseDirectory(gz, &ok);
        if (!ok) throw Error(QStringLiteral("cell database: unreadable leaf directory"));
        QMutexLocker l(&lock);
        leaves.insert(offset, entries);
        return entries;
    }

    /// Directory entries with tile data that overlap tile ids lo..hi.
    void entriesIn(const std::vector<DirEntry> &entries, quint64 lo, quint64 hi, std::vector<DirEntry> &out, int depth)
    {
        if (depth > 4) throw Error(QStringLiteral("cell database: directory too deep"));
        for (std::size_t k = 0; k < entries.size(); ++k) {
            const DirEntry &e = entries[k];
            const quint64 end = e.runLength > 0 ? e.tileId + e.runLength : (k + 1 < entries.size() ? entries[k + 1].tileId : ~quint64(0));
            if (end <= lo || e.tileId > hi) continue;
            if (e.runLength == 0) entriesIn(leaf(e.offset, e.length), lo, hi, out, depth + 1);
            else out.push_back(e);
        }
    }

    RowsPtr downloadBlock(int bx, int by)
    {
        open();
        const auto [lo, hi] = blockTileRange(bx, by);
        std::vector<DirEntry> rootDir;
        quint64 dataOffset;
        {
            QMutexLocker l(&lock);
            rootDir = root;
            dataOffset = header.dataOffset;
        }
        std::vector<DirEntry> entries;
        entriesIn(rootDir, lo, hi, entries, 0);
        // Tile data -> the tile ids that use it (a run repeats one tile)
        struct Piece {
            quint64 offset, length;
            std::vector<quint64> ids;
        };
        std::vector<Piece> pieces;
        for (const DirEntry &e : entries) {
            Piece p{e.offset, e.length, {}};
            for (quint64 t = std::max(e.tileId, lo); t < std::min(e.tileId + e.runLength, hi + 1); ++t) p.ids.push_back(t);
            if (!p.ids.empty()) pieces.push_back(std::move(p));
        }
        std::sort(pieces.begin(), pieces.end(), [](const Piece &a, const Piece &b) { return a.offset < b.offset; });
        auto rows = std::make_shared<Rows>();
        for (std::size_t i = 0; i < pieces.size();) {
            if (cancelled) throw Error(QStringLiteral("cancelled"));
            const quint64 start = pieces[i].offset;
            quint64 end = start + pieces[i].length;
            std::size_t j = i + 1;
            while (j < pieces.size() && qint64(pieces[j].offset) - qint64(end) < kMergeGap &&
                   pieces[j].offset + pieces[j].length - start <= quint64(kMaxRange)) {
                end = std::max(end, pieces[j].offset + pieces[j].length);
                ++j;
            }
            const QByteArray buf = range(dataOffset + start, end - start).body;
            for (std::size_t k = i; k < j; ++k) {
                const Piece &p = pieces[k];
                const QByteArray tile = gunzip(buf.mid(qsizetype(p.offset - start), qsizetype(p.length)));
                for (quint64 id : p.ids) {
                    int z;
                    quint32 x, y;
                    if (tileIdToZxy(id, &z, &x, &y)) decodeTile(tile, x, y, *rows);
                }
            }
            i = j;
        }
        QJsonArray arr;
        for (const Row &r : *rows) arr.append(QJsonArray{r.lat, r.lon, r.mcc, r.mnc, double(r.tac), double(r.eci), r.range, r.samples});
        const QString key = QStringLiteral("%1_%2").arg(bx).arg(by);
        writeFile(blocksDir() + QLatin1Char('/') + key + QStringLiteral(".json"), QJsonDocument(arr).toJson(QJsonDocument::Compact));
        QMutexLocker l(&lock);
        QJsonObject &idx = indexLocked();
        QJsonObject blocks = idx.value(QStringLiteral("blocks")).toObject();
        blocks.insert(key, int(rows->size()));
        idx.insert(QStringLiteral("blocks"), blocks);
        writeJson(indexFile(), idx);
        return rows;
    }

    RowsPtr rememberLocked(const QString &key, RowsPtr rows)
    {
        memBlocks.remove_if([&](const auto &e) { return e.first == key; });
        memBlocks.emplace_front(key, rows);
        while (memBlocks.size() > kMemBlocks) memBlocks.pop_back();
        return rows;
    }

    /// Rows of a block: memory, then disk, then (download) the network; null when missing.
    RowsPtr block(int bx, int by, bool download)
    {
        const QString key = QStringLiteral("%1_%2").arg(bx).arg(by);
        bool listed;
        {
            QMutexLocker l(&lock);
            for (const auto &e : memBlocks)
                if (e.first == key) return rememberLocked(key, e.second);
            listed = indexLocked().value(QStringLiteral("blocks")).toObject().contains(key);
        }
        if (listed) {
            QFile f(blocksDir() + QLatin1Char('/') + key + QStringLiteral(".json"));
            if (f.open(QIODevice::ReadOnly)) {
                const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
                if (doc.isArray()) {
                    auto rows = std::make_shared<Rows>();
                    for (const QJsonValue &v : doc.array()) {
                        const QJsonArray a = v.toArray();
                        Row r;
                        r.lat = a.at(0).toDouble();
                        r.lon = a.at(1).toDouble();
                        r.mcc = a.at(2).toInt();
                        r.mnc = a.at(3).toInt();
                        r.tac = unsigned(a.at(4).toDouble());
                        r.eci = unsigned(a.at(5).toDouble());
                        r.range = a.at(6).toInt();
                        r.samples = a.at(7).toInt();
                        rows->push_back(r);
                    }
                    QMutexLocker l(&lock);
                    return rememberLocked(key, rows);
                }
            }
            // listed but unreadable: fetch again
        }
        if (!download) return nullptr;
        {
            QMutexLocker l(&lock);
            while (inFlight.contains(key)) inFlightDone.wait(&lock);
            for (const auto &e : memBlocks)
                if (e.first == key) return e.second;
            inFlight.insert(key);
        }
        RowsPtr rows;
        try {
            rows = downloadBlock(bx, by);
        } catch (...) {
            QMutexLocker l(&lock);
            inFlight.remove(key);
            inFlightDone.wakeAll();
            throw;
        }
        QMutexLocker l(&lock);
        inFlight.remove(key);
        inFlightDone.wakeAll();
        return rememberLocked(key, rows);
    }

    // ── Query ──
    Result query(const Query &q)
    {
        const std::vector<LayerPtr> csv = loadLayers();
        Rows looked;
        QList<int> bundleMcc;
        {
            QMutexLocker l(&lock);
            const QJsonObject &lk = lookupsLocked();
            for (auto it = lk.begin(); it != lk.end(); ++it) {
                const QJsonObject v = it.value().toObject();
                if (v.value(QStringLiteral("miss")).toBool()) continue;
                const QStringList k = it.key().split(QLatin1Char('-'));
                if (k.size() != 3) continue;
                Row r;
                r.lat = v.value(QStringLiteral("lat")).toDouble();
                r.lon = v.value(QStringLiteral("lon")).toDouble();
                r.mcc = k[0].toInt();
                r.mnc = k[1].toInt();
                r.eci = k[2].toUInt();
                r.tac = unsigned(v.value(QStringLiteral("tac")).toDouble());
                r.range = v.value(QStringLiteral("range")).toInt();
                r.samples = v.value(QStringLiteral("samples")).toInt();
                r.src = v.value(QStringLiteral("src")).toString();
                looked.push_back(r);
            }
            for (const QJsonValue &m : manifestLocked().value(QStringLiteral("mcc")).toArray()) bundleMcc << m.toInt();
        }
        struct Acc {
            CellSite site;
            double w = 0;
        };
        std::vector<Acc> sites;
        QHash<quint64, int> siteIndex;
        QSet<quint64> seen;
        Result res;
        for (const auto &blk : blocksForRadius(q.lat, q.lon, q.radiusKm)) {
            const int bx = blk.first, by = blk.second;
            std::vector<Rows> sets;
            for (const LayerPtr &l : csv) {
                Rows r = layerRows(*l, bx, by);
                if (!r.empty()) sets.push_back(std::move(r));
            }
            const bool bundled = q.mcc && bundleMcc.contains(*q.mcc) &&
                                 std::any_of(csv.begin(), csv.end(), [&](const LayerPtr &l) { return l->bundle && l->has(blockKey(bx, by)); });
            if (!bundled) {
                RowsPtr rows;
                try {
                    rows = block(bx, by, q.download);
                } catch (const Error &e) {
                    LOG_W("celldb") << "block" << bx << "/" << by << ":" << e.text();
                }
                if (rows) {
                    Rows copy = *rows;
                    for (Row &r : copy) r.src = QStringLiteral("ocid-2024");
                    sets.push_back(std::move(copy));
                } else if (sets.empty()) {
                    ++res.missing;
                }
            }
            Rows lk;
            for (const Row &r : looked)
                if (lonToX(r.lon, kBlockZ) == bx && latToY(r.lat, kBlockZ) == by) lk.push_back(r);
            sets.push_back(std::move(lk));
            for (const Rows &rows : sets) {
                for (const Row &r : rows) {
                    if ((q.mcc && r.mcc != *q.mcc) || (q.mnc && r.mnc != *q.mnc && !(q.eci && r.eci == *q.eci))) continue;
                    const quint64 ck = cellKey(r.mcc, r.mnc, r.eci);
                    if (seen.contains(ck)) continue;
                    seen.insert(ck);
                    if (distanceKm(q.lat, q.lon, r.lat, r.lon) > q.radiusKm) continue;
                    const unsigned enb = r.eci / 256;
                    const quint64 sk = (quint64(quint32(r.mcc) & 0x3ff) << 30) | (quint64(quint32(r.mnc) & 0x3ff) << 20) | (enb & 0xfffff);
                    auto it = siteIndex.find(sk);
                    if (it == siteIndex.end()) {
                        it = siteIndex.insert(sk, int(sites.size()));
                        Acc a;
                        a.site.mcc = r.mcc;
                        a.site.mnc = r.mnc;
                        a.site.enb = enb;
                        sites.push_back(std::move(a));
                    }
                    Acc &a = sites[std::size_t(it.value())];
                    const double w = std::max(1, r.samples);
                    a.site.lat += r.lat * w;
                    a.site.lon += r.lon * w;
                    a.w += w;
                    a.site.cells.push_back({r.eci, r.tac, r.lat, r.lon, r.range, r.samples, r.src});
                }
            }
        }
        for (Acc &a : sites) {
            a.site.lat /= a.w;
            a.site.lon /= a.w;
            res.sites.push_back(std::move(a.site));
        }
        QMutexLocker l(&lock);
        for (const LayerPtr &layer : csv)
            if (!layer->bundle && !layer->newest.isEmpty()) {
                res.date = layer->newest;
                break;
            }
        if (res.date.isEmpty())
            res.date = manifestLocked().value(QStringLiteral("sources")).toArray().at(0).toObject().value(QStringLiteral("date")).toString();
        if (res.date.isEmpty()) res.date = indexLocked().value(QStringLiteral("date")).toString();
        return res;
    }

    // ── Online lookup ──
    std::optional<CellRecord> lookupOnline(int mcc, int mnc, unsigned tac, unsigned eci, const QString &token)
    {
        if (!token.isEmpty()) {
            QUrlQuery qy;
            qy.addQueryItem(QStringLiteral("key"), token);
            qy.addQueryItem(QStringLiteral("mcc"), QString::number(mcc));
            qy.addQueryItem(QStringLiteral("mnc"), QString::number(mnc));
            qy.addQueryItem(QStringLiteral("lac"), QString::number(tac));
            qy.addQueryItem(QStringLiteral("cellid"), QString::number(eci));
            qy.addQueryItem(QStringLiteral("radio"), QStringLiteral("LTE"));
            qy.addQueryItem(QStringLiteral("format"), QStringLiteral("json"));
            QUrl u(kOcidCellUrl);
            u.setQuery(qy);
            const Http r = http(u, "GET", {}, {}, kLookupTimeoutMs, &cancelled);
            const QJsonObject b = QJsonDocument::fromJson(r.body).object();
            const double lat = b.value(QStringLiteral("lat")).toDouble(), lon = b.value(QStringLiteral("lon")).toDouble();
            if (b.contains(QStringLiteral("lat")) && (lat != 0 || lon != 0))
                return CellRecord{eci, tac, lat, lon, b.value(QStringLiteral("range")).toInt(), b.value(QStringLiteral("samples")).toInt(),
                                  QStringLiteral("ocid-api")};
            // anything but "not found" (bad token, daily credits used up): say so, then try beaconDB
            const QString err = b.value(QStringLiteral("error")).toString();
            if (r.status != 200 || (!err.isEmpty() && !err.contains(QRegularExpression(QStringLiteral("not\\s*found"), QRegularExpression::CaseInsensitiveOption))))
                LOG_W("celldb") << "OpenCelliD API: HTTP" << r.status << (err.isEmpty() ? QString::fromUtf8(r.body.left(80)) : err);
        }
        // No IP or area fallback: only the cell itself, else beaconDB answers with the GCS position
        const QJsonObject req{{QStringLiteral("considerIp"), false},
                              {QStringLiteral("fallbacks"), QJsonObject{{QStringLiteral("lacf"), false}, {QStringLiteral("ipf"), false}}},
                              {QStringLiteral("cellTowers"), QJsonArray{QJsonObject{{QStringLiteral("radioType"), QStringLiteral("lte")},
                                                                                    {QStringLiteral("mobileCountryCode"), mcc},
                                                                                    {QStringLiteral("mobileNetworkCode"), mnc},
                                                                                    {QStringLiteral("locationAreaCode"), double(tac)},
                                                                                    {QStringLiteral("cellId"), double(eci)}}}}};
        const Http r = http(QUrl(kBeaconDbUrl), "POST", QJsonDocument(req).toJson(QJsonDocument::Compact),
                            {{QByteArrayLiteral("Content-Type"), QByteArrayLiteral("application/json")}}, kLookupTimeoutMs, &cancelled);
        const QJsonObject b = QJsonDocument::fromJson(r.body).object();
        const QJsonObject loc = b.value(QStringLiteral("location")).toObject();
        if (r.status == 200 && loc.contains(QStringLiteral("lat")) && loc.contains(QStringLiteral("lng")))
            return CellRecord{eci, tac, loc.value(QStringLiteral("lat")).toDouble(), loc.value(QStringLiteral("lng")).toDouble(),
                              int(std::lround(b.value(QStringLiteral("accuracy")).toDouble())), 0, QStringLiteral("beacondb")};
        if (r.status == 404) return std::nullopt;
        throw Error(r.error.isEmpty() ? QStringLiteral("beaconDB: HTTP %1").arg(r.status) : r.error);
    }

    std::optional<CellRecord> lookup(int mcc, int mnc, unsigned tac, unsigned eci)
    {
        const QString key = QStringLiteral("%1-%2-%3").arg(mcc).arg(mnc).arg(eci);
        QString token;
        {
            QMutexLocker l(&lock);
            const QJsonObject hit = lookupsLocked().value(key).toObject();
            if (!hit.isEmpty() && !hit.value(QStringLiteral("miss")).toBool())
                return CellRecord{eci, unsigned(hit.value(QStringLiteral("tac")).toDouble()), hit.value(QStringLiteral("lat")).toDouble(),
                                  hit.value(QStringLiteral("lon")).toDouble(), hit.value(QStringLiteral("range")).toInt(),
                                  hit.value(QStringLiteral("samples")).toInt(), hit.value(QStringLiteral("src")).toString()};
            if (!hit.isEmpty() && QDateTime::currentMSecsSinceEpoch() - qint64(hit.value(QStringLiteral("at")).toDouble()) < kLookupMissMs)
                return std::nullopt;
            const QJsonObject s = settingsLocked();
            if (!s.value(QStringLiteral("online")).toBool(true)) return std::nullopt;
            token = s.value(QStringLiteral("token")).toString();
        }
        const std::optional<CellRecord> found = lookupOnline(mcc, mnc, tac, eci, token);
        const double now = double(QDateTime::currentMSecsSinceEpoch());
        QJsonObject entry = found ? QJsonObject{{QStringLiteral("lat"), found->lat}, {QStringLiteral("lon"), found->lon},
                                                {QStringLiteral("range"), found->range}, {QStringLiteral("samples"), found->samples},
                                                {QStringLiteral("src"), found->src}, {QStringLiteral("tac"), double(tac)}, {QStringLiteral("at"), now}}
                                  : QJsonObject{{QStringLiteral("miss"), true}, {QStringLiteral("at"), now}};
        {
            QMutexLocker l(&lock);
            lookupsLocked().insert(key, entry);
            writeJson(lookupFile(), lookups);
        }
        if (found) LOG_I("celldb") << "lookup" << key << ":" << found->src << QString::number(found->lat, 'f', 5) << QString::number(found->lon, 'f', 5)
                                   << "+-" << found->range << "m";
        else LOG_I("celldb") << "lookup" << key << ": not found";
        return found;
    }

    // ── Files ──
    int downloadOpenCellid(int mcc)
    {
        QString token;
        {
            QMutexLocker l(&lock);
            token = settingsLocked().value(QStringLiteral("token")).toString();
        }
        if (token.isEmpty()) throw Error(QStringLiteral("no OpenCelliD token"));
        if (mcc < 100 || mcc > 999) throw Error(QStringLiteral("bad MCC %1").arg(mcc));
        QUrlQuery qy;
        qy.addQueryItem(QStringLiteral("token"), token);
        qy.addQueryItem(QStringLiteral("type"), QStringLiteral("mcc"));
        qy.addQueryItem(QStringLiteral("file"), QStringLiteral("%1.csv.gz").arg(mcc));
        QUrl u(kOcidDownloadUrl);
        u.setQuery(qy);
        const Http r = http(u, "GET", {}, {}, 600000, &cancelled);
        if (r.status != 200 || r.body.size() < 2 || uchar(r.body[0]) != 0x1f || uchar(r.body[1]) != 0x8b) {
            // OpenCelliD answers errors (bad token, limit reached) as text
            QString text = QString::fromUtf8(r.body.left(200));
            text.remove(QRegularExpression(QStringLiteral("<[^>]+>")));
            text = text.simplified();
            throw Error(QStringLiteral("OpenCelliD: ") + (text.isEmpty() ? (r.error.isEmpty() ? QStringLiteral("HTTP %1").arg(r.status) : r.error) : text));
        }
        const QString file = sourcesDir() + QStringLiteral("/opencellid-%1.csv.gz").arg(mcc);
        writeFile(file, r.body);
        resetLayers();
        for (const LayerPtr &l : loadLayers())
            if (QFileInfo(l->file) == QFileInfo(file)) return l->cells;
        throw Error(QStringLiteral("the downloaded file could not be read"));
    }

    int importCsv(const QString &srcFile)
    {
        const LayerPtr probe = loadCsvLayer(srcFile, QFileInfo(srcFile).fileName());
        if (!probe->cells) throw Error(QStringLiteral("no LTE cells in the file"));
        QDir().mkpath(sourcesDir());
        const QString dst = sourcesDir() + QLatin1Char('/') + QFileInfo(srcFile).fileName();
        if (QFileInfo(dst) != QFileInfo(srcFile)) {
            QFile::remove(dst);
            if (!QFile::copy(srcFile, dst)) throw Error(QStringLiteral("cannot copy the file into the cell database"));
        }
        resetLayers();
        return probe->cells;
    }

    QVariantMap status()
    {
        const std::vector<LayerPtr> csv = loadLayers();
        QVariantMap out;
        QMutexLocker l(&lock);
        const QJsonObject blocks = indexLocked().value(QStringLiteral("blocks")).toObject();
        qint64 cells = 0;
        for (const QJsonValue &v : blocks) cells += v.toInt();
        qint64 size = 0;
        for (const QFileInfo &f : QDir(blocksDir()).entryInfoList(QDir::Files)) size += f.size();
        out[QStringLiteral("blocks")] = int(blocks.size());
        out[QStringLiteral("cells")] = cells;
        out[QStringLiteral("bytes")] = size;
        out[QStringLiteral("date")] = indexLocked().value(QStringLiteral("date")).toString();
        out[QStringLiteral("source")] = url;
        const QJsonObject m = manifestLocked();
        out[QStringLiteral("bundle")] = m.isEmpty() ? QVariant() : QVariant(m.toVariantMap());
        QVariantList files;
        for (const LayerPtr &layer : csv)
            if (!layer->bundle)
                files << QVariantMap{{QStringLiteral("name"), layer->name}, {QStringLiteral("cells"), layer->cells}, {QStringLiteral("newest"), layer->newest}};
        out[QStringLiteral("files")] = files;
        int found = 0, missed = 0;
        for (const QJsonValue &v : lookupsLocked()) (v.toObject().value(QStringLiteral("miss")).toBool() ? missed : found)++;
        out[QStringLiteral("lookups")] = QVariantMap{{QStringLiteral("found"), found}, {QStringLiteral("missed"), missed}};
        const QJsonObject s = settingsLocked();
        out[QStringLiteral("settings")] = QVariantMap{{QStringLiteral("online"), s.value(QStringLiteral("online")).toBool(true)},
                                                      {QStringLiteral("hasToken"), !s.value(QStringLiteral("token")).toString().isEmpty()}};
        return out;
    }

    void clear()
    {
        QMutexLocker l(&lock);
        QDir(blocksDir()).removeRecursively();
        QDir(leafDir()).removeRecursively();
        QDir(sourcesDir()).removeRecursively();
        QFile::remove(indexFile());
        QFile::remove(lookupFile());
        indexLoaded = lookupsLoaded = headerLoaded = false;
        index = lookups = QJsonObject();
        root.clear();
        leaves.clear();
        memBlocks.clear();
        layers.reset();
    }
};

// ── Facade: work on the pools, answers on the GUI thread ─────────────────────

namespace {
/// Deliver on the GUI thread unless the CellDb went away meanwhile.
template <class F>
void deliver(const std::weak_ptr<CellDb::Core> &alive, F f)
{
    QMetaObject::invokeMethod(QCoreApplication::instance(), [alive, f = std::move(f)]() mutable {
        if (auto c = alive.lock(); c && !c->cancelled) f();
    }, Qt::QueuedConnection);
}
} // namespace

CellDb::CellDb(const QString &dir, const QString &bundleDir, QObject *parent) : QObject(parent), m_core(std::make_shared<Core>())
{
    m_core->dir = dir;
    m_core->bundleDir = bundleDir;
    m_pool.setMaxThreadCount(2);
    m_bulk.setMaxThreadCount(1);
}

CellDb::~CellDb()
{
    // Requests in flight are aborted within 0.2 s; their answers are dropped
    m_core->cancelled = true;
    m_pool.waitForDone();
    m_bulk.waitForDone();
}

void CellDb::query(const Query &q, std::function<void(Result)> done)
{
    std::weak_ptr<Core> alive = m_core;
    (void)QtConcurrent::run(&m_pool, [core = m_core, alive, q, done] {
        Result r;
        try {
            r = core->query(q);
        } catch (const Error &e) {
            r.error = e.text();
        } catch (const std::exception &e) {
            r.error = QString::fromUtf8(e.what());
        }
        deliver(alive, [done, r] { done(r); });
    });
}

void CellDb::lookup(int mcc, int mnc, unsigned tac, unsigned eci, std::function<void(std::optional<CellRecord>, QString)> done)
{
    std::weak_ptr<Core> alive = m_core;
    (void)QtConcurrent::run(&m_pool, [core = m_core, alive, mcc, mnc, tac, eci, done] {
        std::optional<CellRecord> hit;
        QString error;
        try {
            hit = core->lookup(mcc, mnc, tac, eci);
        } catch (const Error &e) {
            error = e.text();
        }
        deliver(alive, [done, hit, error] { done(hit, error); });
    });
}

void CellDb::downloadOpenCellid(int mcc, std::function<void(QString, int)> done)
{
    std::weak_ptr<Core> alive = m_core;
    (void)QtConcurrent::run(&m_bulk, [core = m_core, alive, mcc, done] {
        QString error;
        int cells = 0;
        try {
            cells = core->downloadOpenCellid(mcc);
        } catch (const Error &e) {
            error = e.text();
        }
        deliver(alive, [done, error, cells] { done(error, cells); });
    });
}

void CellDb::importCsv(const QString &file, std::function<void(QString, int)> done)
{
    std::weak_ptr<Core> alive = m_core;
    (void)QtConcurrent::run(&m_bulk, [core = m_core, alive, file, done] {
        QString error;
        int cells = 0;
        try {
            cells = core->importCsv(file);
        } catch (const Error &e) {
            error = e.text();
        }
        deliver(alive, [done, error, cells] { done(error, cells); });
    });
}

void CellDb::downloadBox(double south, double west, double north, double east, std::function<void(int, int, int, int)> progress,
                         std::function<void(int, int, int)> done)
{
    std::weak_ptr<Core> alive = m_core;
    (void)QtConcurrent::run(&m_bulk, [core = m_core, alive, south, west, north, east, progress, done] {
        const auto todo = blocksForBox(south, west, north, east);
        int n = 0, cells = 0, failed = 0;
        QElapsedTimer t;
        t.start();
        for (const auto &[bx, by] : todo) {
            if (core->cancelled) break;
            try {
                const RowsPtr rows = core->block(bx, by, true);
                cells += rows ? int(rows->size()) : 0;
            } catch (const Error &e) {
                ++failed;
                LOG_W("celldb") << "block" << bx << "/" << by << ":" << e.text();
            }
            ++n;
            const int total = int(todo.size());
            deliver(alive, [progress, n, total, cells, failed] { progress(n, total, cells, failed); });
        }
        LOG_I("celldb") << "box downloaded:" << todo.size() << "blocks," << cells << "LTE cells," << failed << "failed," << t.elapsed() / 1000.0 << "s";
        const int blocks = int(todo.size());
        deliver(alive, [done, blocks, cells, failed] { done(blocks, cells, failed); });
    });
}

void CellDb::status(std::function<void(QVariantMap)> done)
{
    std::weak_ptr<Core> alive = m_core;
    (void)QtConcurrent::run(&m_pool, [core = m_core, alive, done] {
        const QVariantMap st = core->status();
        deliver(alive, [done, st] { done(st); });
    });
}

void CellDb::clear(std::function<void()> done)
{
    std::weak_ptr<Core> alive = m_core;
    (void)QtConcurrent::run(&m_pool, [core = m_core, alive, done] {
        core->clear();
        deliver(alive, [done] { done(); });
    });
}

QVariantMap CellDb::settings() const
{
    QMutexLocker l(&m_core->lock);
    const QJsonObject s = m_core->settingsLocked();
    return {{QStringLiteral("online"), s.value(QStringLiteral("online")).toBool(true)},
            {QStringLiteral("hasToken"), !s.value(QStringLiteral("token")).toString().isEmpty()}};
}

void CellDb::setOnline(bool on)
{
    QMutexLocker l(&m_core->lock);
    QJsonObject s = m_core->settingsLocked();
    s.insert(QStringLiteral("online"), on);
    writeJson(m_core->settingsFile(), s);
}

void CellDb::setToken(const QString &token)
{
    QMutexLocker l(&m_core->lock);
    QJsonObject s = m_core->settingsLocked();
    s.insert(QStringLiteral("token"), token.trimmed());
    writeJson(m_core->settingsFile(), s);
    // A new token may find cells that were not found before
    QJsonObject &lk = m_core->lookupsLocked();
    for (const QString &k : lk.keys())
        if (lk.value(k).toObject().value(QStringLiteral("miss")).toBool()) lk.remove(k);
    writeJson(m_core->lookupFile(), lk);
}

} // namespace corv
