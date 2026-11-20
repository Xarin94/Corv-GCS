#include "services/GeoZoneStore.h"

#include "core/DebugLog.h"

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QNetworkReply>
#include <QRegularExpression>
#include <QSaveFile>
#include <QUrlQuery>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr qint64 kTtlMs = 6LL * 3600 * 1000;           // a tile is fetched again after this...
constexpr qint64 kRetryMs = 10LL * 60 * 1000;          // ...a failed one after this
constexpr qint64 kFileTtlMs = 7LL * 24 * 3600 * 1000;  // a whole-country file
constexpr int kTimeoutMs = 30000;
constexpr int kFileTimeoutMs = 180000;
constexpr int kConcurrency = 4;
constexpr int kBusyRetries = 4;                         // "too many requests": again after 2, 4, 8, 16 s
constexpr int kMaxTiles = 160;                          // per source and query
const char *kUserAgent = "Corv-GCS geozones";

using Box = std::array<double, 4>;

bool boxesIntersect(const Box &a, const Box &b) { return a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1]; }

Box circleBox(const GeoZoneStore::Circle &c)
{
    const double dLat = c.radiusKm / 111.32;
    const double dLon = dLat / std::max(0.05, std::cos(c.lat * M_PI / 180));
    return {c.lon - dLon, c.lat - dLat, c.lon + dLon, c.lat + dLat};
}

double distanceToBoxKm(double lat, double lon, const double box[4])
{
    const double cLat = std::clamp(lat, box[1], box[3]);
    const double cLon = std::clamp(lon, box[0], box[2]);
    const double k = std::cos(lat * M_PI / 180);
    return std::hypot((cLat - lat) * 111.32, (cLon - lon) * 111.32 * k);
}

QJsonObject readJson(const QString &file)
{
    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) return {};
    return QJsonDocument::fromJson(f.readAll()).object();
}

void writeJson(const QString &file, const QJsonObject &o)
{
    QDir().mkpath(QFileInfo(file).absolutePath());
    QSaveFile f(file);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(QJsonDocument(o).toJson(QJsonDocument::Compact));
        f.commit();
    }
}

QJsonObject zonesToJson(qint64 at, const GeoZoneList &zones)
{
    QJsonArray a;
    for (const GeoZone &z : zones) a.append(z.toJson());
    return QJsonObject{{u"at"_s, double(at)}, {u"zones"_s, a}};
}

std::shared_ptr<GeoZoneList> zonesFromJson(const QJsonObject &o)
{
    if (!o.value(u"zones"_s).isArray()) return nullptr;
    auto list = std::make_shared<GeoZoneList>();
    for (const QJsonValue &v : o.value(u"zones"_s).toArray())
        if (auto z = GeoZone::fromJson(v.toObject())) list->push_back(std::move(*z));
    return list;
}

QString s(const QJsonObject &o, const char *k) { return o.value(QLatin1String(k)).toVariant().toString(); }

/// "Surface", "400' AGL", "1200 MSL" -> metres and AGL (FAA national security restrictions).
std::pair<double, bool> faaLimitText(const QString &t, double fallbackFt)
{
    static const QRegularExpression sfc(u"surface|sfc|gnd"_s, QRegularExpression::CaseInsensitiveOption);
    static const QRegularExpression num(u"([\\d,.]+)"_s);
    if (sfc.match(t).hasMatch()) return {0, true};
    const auto m = num.match(t);
    const double ft = m.hasMatch() ? QString(m.captured(1)).remove(','_L1).toDouble() : fallbackFt;
    return {ft * geozone::kFt, !t.contains("msl"_L1, Qt::CaseInsensitive)};
}

/// FAA airspace limits: VAL, UOM, CODE (SFC, AGL, MSL, STD = flight level).
std::pair<std::optional<double>, bool> faaLimit(const QJsonValue &val, const QString &uom, const QString &code)
{
    const QString c = code.toUpper();
    if (c == "SFC"_L1) return {0.0, true};
    bool ok = false;
    double v = val.isDouble() ? val.toDouble() : val.toVariant().toString().toDouble(&ok);
    if (!val.isDouble() && !ok) return {std::nullopt, false};
    if (c == "STD"_L1 || uom.compare("FL"_L1, Qt::CaseInsensitive) == 0) v *= 100;
    return {geozone::toMetres(v, uom.isEmpty() ? u"FT"_s : uom), c == "AGL"_L1};
}

const QString kFaaBase = u"https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/ArcGIS/rest/services"_s;
const QString kEnaireUrl = u"https://servais.enaire.es/insignia/rest/services/NSF_SRV/SRV_UAS_ZG_V1/MapServer"_s;
const QString kDipulUrl = u"https://uas-betrieb.de/geoservices/dipul/wfs"_s;
const QString kIgnUrl = u"https://data.geopf.fr/wfs/ows"_s;
const QString kBazlUrl = u"https://data.geo.admin.ch/ch.bazl.einschraenkungen-drohnen/einschraenkungen-drohnen/einschraenkungen-drohnen_4326.json"_s;

QString boxText(const Box &b) { return u"%1,%2,%3,%4"_s.arg(b[0]).arg(b[1]).arg(b[2]).arg(b[3]); }

/// Run async steps one after the other; the first error stops the chain.
void chain(std::shared_ptr<QList<std::function<void(std::function<void(QString)>)>>> steps, std::function<void(QString)> done, int i = 0)
{
    if (i >= steps->size()) {
        done({});
        return;
    }
    (*steps)[i]([steps, done, i](const QString &err) {
        if (!err.isEmpty()) done(err);
        else chain(steps, done, i + 1);
    });
}
} // namespace

const QList<GeoZoneStore::Provider> &GeoZoneStore::providers()
{
    static const QList<Provider> list = {
        {u"faa"_s, u"FAA"_s, u"US"_s, 0.5, {}, 2,
         {{-125, 24, -66.5, 49.5}, {-170, 51, -129, 72}, {-161, 18.5, -154, 22.5}, {-68, 17.5, -65, 18.7}}},
        {u"dipul"_s, u"DFS DIPUL"_s, u"DE"_s, 1, {}, 4, {{5.8, 47.2, 15.1, 55.1}}},
        {u"ign"_s, u"IGN / DGAC"_s, u"FR"_s, 0.5, {}, 4, {{-5.3, 41.3, 9.7, 51.2}}},
        {u"enaire"_s, u"ENAIRE"_s, u"ES"_s, 0.5, {}, 4, {{-9.5, 35.9, 4.4, 43.9}, {-18.2, 27.6, -13.3, 29.5}}},
        {u"bazl"_s, u"FOCA / BAZL"_s, u"CH"_s, 0, kBazlUrl, 4, {{5.9, 45.8, 10.5, 47.9}}},
    };
    return list;
}

GeoZoneStore::GeoZoneStore(QString dir, QObject *parent) : QObject(parent), m_dir(std::move(dir))
{
    m_net.setTransferTimeout(kTimeoutMs);
    m_changeTimer.setSingleShot(true);
    m_changeTimer.setInterval(400);
    connect(&m_changeTimer, &QTimer::timeout, this, &GeoZoneStore::changed);
}

void GeoZoneStore::markChanged() { m_changeTimer.start(); }

QString GeoZoneStore::tileFile(const Provider &p, const QString &key) const
{
    return m_dir + '/'_L1 + p.id + '/'_L1 + key + u".json"_s;
}

GeoZoneStore::Tile &GeoZoneStore::tile(const Provider &p, const QString &key)
{
    const QString id = p.id + '/'_L1 + key;
    auto it = m_tiles.find(id);
    if (it != m_tiles.end()) return *it;
    Tile t;
    const QJsonObject o = readJson(tileFile(p, key));
    t.at = qint64(o.value(u"at"_s).toDouble());
    t.zones = zonesFromJson(o);
    return m_tiles.insert(id, t).value();
}

void GeoZoneStore::want(const Provider &p, int tx, int ty, qint64 now)
{
    const QString key = u"%1_%2_%3"_s.arg(p.tileDeg).arg(tx).arg(ty);
    const QString id = p.id + '/'_L1 + key;
    Tile &t = tile(p, key);
    const bool stale = !t.at || now - t.at > kTtlMs;
    const bool backoff = t.failedAt && now - t.failedAt < kRetryMs;
    if (stale && !backoff && !m_inFlight.contains(id)) {
        m_inFlight.insert(id);
        m_queue.append({&p, key, id, {tx * p.tileDeg, ty * p.tileDeg, (tx + 1) * p.tileDeg, (ty + 1) * p.tileDeg}});
    }
}

GeoZoneStore::Tile &GeoZoneStore::countryFile(const Provider &p, qint64 now)
{
    auto it = m_files.find(p.id);
    if (it == m_files.end()) {
        Tile t;
        const QJsonObject o = readJson(m_dir + '/'_L1 + p.id + u".json"_s);
        t.at = qint64(o.value(u"at"_s).toDouble());
        t.zones = zonesFromJson(o);
        it = m_files.insert(p.id, t);
    }
    Tile &f = *it;
    const bool stale = !f.at || now - f.at > kFileTtlMs;
    const bool backoff = f.failedAt && now - f.failedAt < kRetryMs;
    if (stale && !backoff && !m_inFlight.contains(p.id)) {
        m_inFlight.insert(p.id);
        const QString pid = p.id;
        QElapsedTimer t0;
        t0.start();
        fetchJson(QUrl(p.file), kFileTimeoutMs, [this, pid, t0](const QJsonDocument &doc, const QString &err) {
            Tile &f = m_files[pid];
            if (err.isEmpty()) {
                auto zones = std::make_shared<GeoZoneList>(geozone::parseEdZones(doc.isArray() ? QJsonValue(doc.array()) : QJsonValue(doc.object()), pid));
                f.at = QDateTime::currentMSecsSinceEpoch();
                f.zones = zones;
                writeJson(m_dir + '/'_L1 + pid + u".json"_s, zonesToJson(f.at, *zones));
                m_lastError.remove(pid);
                LOG_I("geozones") << pid << ":" << zones->size() << "zones," << t0.elapsed() / 1000.0 << "s";
            } else {
                f.failedAt = QDateTime::currentMSecsSinceEpoch();
                m_lastError[pid] = err;
                LOG_W("geozones") << pid << "failed:" << err;
            }
            m_inFlight.remove(pid);
            markChanged();
        });
    }
    return f;
}

void GeoZoneStore::loadImports()
{
    if (m_importsLoaded) return;
    m_importsLoaded = true;
    const QDir dir(m_dir + u"/import"_s);
    for (const QString &n : dir.entryList({u"*.json"_s}, QDir::Files)) {
        const QJsonObject o = readJson(dir.filePath(n));
        auto zones = zonesFromJson(o);
        if (!zones) continue;
        m_imports.insert(n, {o.value(u"name"_s).toString(n), qint64(o.value(u"at"_s).toDouble()), zones});
    }
}

QList<GeoZoneStore::Import> GeoZoneStore::imports()
{
    loadImports();
    QList<Import> out;
    for (auto it = m_imports.cbegin(); it != m_imports.cend(); ++it)
        out.append({it.key(), it->name, it->at, int(it->zones ? it->zones->size() : 0)});
    return out;
}

GeoZoneStore::Result GeoZoneStore::query(const QList<Circle> &circlesIn)
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    QList<Circle> circles;
    for (const Circle &c : circlesIn)
        if (std::isfinite(c.lat) && std::isfinite(c.lon) && c.radiusKm > 0) circles.append(c);
    auto out = std::make_shared<GeoZoneList>();
    QSet<QString> seenIds;
    auto add = [&](const std::shared_ptr<GeoZoneList> &zones) {
        if (!zones) return;
        for (const GeoZone &z : *zones) {
            if (seenIds.contains(z.id)) continue;
            bool near = false;
            for (const Circle &c : circles)
                if (distanceToBoxKm(c.lat, c.lon, z.bbox) <= c.radiusKm) { near = true; break; }
            if (!near) continue;
            seenIds.insert(z.id);
            out->push_back(z);
        }
    };
    Result r;
    if (!circles.isEmpty()) {
        QList<Box> boxes;
        for (const Circle &c : circles) boxes.append(circleBox(c));
        for (const Provider &p : providers()) {
            QList<Box> cover;
            for (const auto &c : p.coverage)
                for (const Box &b : boxes)
                    if (boxesIntersect(b, c)) { cover.append(c); break; }
            if (cover.isEmpty()) continue;
            Source src{p.id, p.name, p.country, m_lastError.value(p.id)};
            if (p.tileDeg == 0) {
                Tile &f = countryFile(p, now);
                src.tiles = 1;
                if (f.zones) { src.ready = 1; add(f.zones); }
                if (m_inFlight.contains(p.id)) src.pending = 1;
                r.sources.append(src);
                continue;
            }
            QSet<QString> seen;
            for (const Box &b : boxes) {
                for (const Box &c : cover) {
                    if (!boxesIntersect(b, c)) continue;
                    const double w = std::max(b[0], c[0]), s0 = std::max(b[1], c[1]), e = std::min(b[2], c[2]), n = std::min(b[3], c[3]);
                    for (int ty = int(std::floor(s0 / p.tileDeg)); ty <= int(std::floor(n / p.tileDeg)); ++ty) {
                        for (int tx = int(std::floor(w / p.tileDeg)); tx <= int(std::floor(e / p.tileDeg)); ++tx) {
                            const QString key = u"%1_%2"_s.arg(tx).arg(ty);
                            if (seen.contains(key) || seen.size() >= kMaxTiles) continue;
                            seen.insert(key);
                            want(p, tx, ty, now);
                            const QString tkey = u"%1_%2_%3"_s.arg(p.tileDeg).arg(tx).arg(ty);
                            const Tile &t = tile(p, tkey);
                            ++src.tiles;
                            if (t.zones) { ++src.ready; add(t.zones); }
                            if (m_inFlight.contains(p.id + '/'_L1 + tkey)) ++src.pending;
                        }
                    }
                }
            }
            r.sources.append(src);
        }
        loadImports();
        for (const ImportFile &f : std::as_const(m_imports)) add(f.zones);
        pump();
    }
    r.zones = out;
    r.pending = int(m_inFlight.size());
    return r;
}

void GeoZoneStore::pump()
{
    while (m_active < kConcurrency) {
        int i = -1;
        for (int k = 0; k < m_queue.size(); ++k)
            if (m_activeBy.value(m_queue[k].p->id) < m_queue[k].p->parallel) { i = k; break; }
        if (i < 0) break;
        const Job job = m_queue.takeAt(i);
        ++m_active;
        ++m_activeBy[job.p->id];
        QElapsedTimer t0;
        t0.start();
        fetchTile(job, [this, job, t0](GeoZoneList zones, const QString &err) {
            Tile &t = m_tiles[job.id];
            if (err.isEmpty()) {
                t.at = QDateTime::currentMSecsSinceEpoch();
                t.failedAt = 0;
                t.zones = std::make_shared<GeoZoneList>(std::move(zones));
                writeJson(tileFile(*job.p, job.key), zonesToJson(t.at, *t.zones));
                m_lastError.remove(job.p->id);
                LOG_I("geozones") << job.id << ":" << t.zones->size() << "zones," << t0.elapsed() / 1000.0 << "s";
            } else {
                t.failedAt = QDateTime::currentMSecsSinceEpoch();
                t.error = err;
                m_lastError[job.p->id] = err;
                LOG_W("geozones") << job.id << "failed:" << err;
            }
            --m_active;
            --m_activeBy[job.p->id];
            m_inFlight.remove(job.id);
            markChanged();
            pump();
        });
    }
}

int GeoZoneStore::importFile(const QString &path, QString *error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error) *error = f.errorString();
        return -1;
    }
    QByteArray bytes = f.readAll();
    if (bytes.startsWith("\xEF\xBB\xBF")) bytes.remove(0, 3);
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(bytes, &pe);
    if (doc.isNull()) {
        if (error) *error = u"Not JSON: %1"_s.arg(pe.errorString());
        return -1;
    }
    QString base = QFileInfo(path).completeBaseName();
    base.replace(QRegularExpression(u"[^\\w.-]+"_s), u"_"_s);
    auto zones = std::make_shared<GeoZoneList>(geozone::parseEdZones(doc.isArray() ? QJsonValue(doc.array()) : QJsonValue(doc.object()),
                                                                      u"import-"_s + base));
    if (zones->empty()) {
        if (error) *error = u"No ED-269 / ED-318 zones in this file"_s;
        return -1;
    }
    loadImports();
    const QString file = base + u".json"_s;
    const qint64 at = QDateTime::currentMSecsSinceEpoch();
    QJsonObject o = zonesToJson(at, *zones);
    o.insert(u"name"_s, QFileInfo(path).fileName());
    writeJson(m_dir + u"/import/"_s + file, o);
    m_imports.insert(file, {QFileInfo(path).fileName(), at, zones});
    LOG_I("geozones") << "imported" << QFileInfo(path).fileName() << ":" << zones->size() << "zones";
    markChanged();
    return int(zones->size());
}

void GeoZoneStore::removeImport(const QString &file)
{
    const QString name = QFileInfo(file).fileName();
    QFile::remove(m_dir + u"/import/"_s + name);
    m_imports.remove(name);
    markChanged();
}

void GeoZoneStore::clear()
{
    for (const Provider &p : providers()) {
        QDir(m_dir + '/'_L1 + p.id).removeRecursively();
        QFile::remove(m_dir + '/'_L1 + p.id + u".json"_s);
        m_files.remove(p.id);
    }
    m_tiles.clear();
    m_lastError.clear();
    markChanged();
}

// ── Network ──────────────────────────────────────────────────────────────────

void GeoZoneStore::fetchJson(const QUrl &url, int timeoutMs, JsonDone done, int attempt)
{
    QNetworkRequest req(url);
    req.setHeader(QNetworkRequest::UserAgentHeader, QString::fromLatin1(kUserAgent));
    req.setRawHeader("Accept", "application/json");
    req.setTransferTimeout(timeoutMs);
    QNetworkReply *reply = m_net.get(req);
    connect(reply, &QNetworkReply::finished, this, [this, reply, url, timeoutMs, done, attempt] {
        reply->deleteLater();
        const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        const QByteArray body = reply->readAll();
        QJsonParseError pe;
        const QJsonDocument doc = QJsonDocument::fromJson(body, &pe);
        const QJsonObject err = doc.object().value(u"error"_s).toObject();
        // A server that says it is busy (the FAA's ArcGIS limits parallel requests) is asked again
        const bool busy = status == 429 || status == 503 || err.value(u"code"_s).toInt() == 429
                          || err.value(u"message"_s).toString().contains("too many requests"_L1, Qt::CaseInsensitive);
        if (busy) {
            if (attempt >= kBusyRetries) {
                done({}, err.value(u"message"_s).toString(u"HTTP %1"_s.arg(status)));
                return;
            }
            QTimer::singleShot(2000 << attempt, this, [this, url, timeoutMs, done, attempt] { fetchJson(url, timeoutMs, done, attempt + 1); });
            return;
        }
        if (reply->error() != QNetworkReply::NoError && status < 400) {
            done({}, reply->errorString());
            return;
        }
        if (status >= 400) {
            done({}, u"HTTP %1"_s.arg(status));
            return;
        }
        if (doc.isNull()) {
            done({}, u"not JSON: %1"_s.arg(QString::fromUtf8(body.left(120)).simplified()));
            return;
        }
        done(doc, {});
    });
}

void GeoZoneStore::arcgisFeatures(const QString &layerUrl, const Box &box, const QString &where, const QString &outFields, int pageSize,
                                  std::function<void(QJsonArray, QString)> done, std::shared_ptr<QJsonArray> acc, int offset)
{
    if (!acc) acc = std::make_shared<QJsonArray>();
    QUrlQuery q;
    q.addQueryItem(u"where"_s, where.isEmpty() ? u"1=1"_s : where);
    q.addQueryItem(u"outFields"_s, outFields.isEmpty() ? u"*"_s : outFields);
    q.addQueryItem(u"f"_s, u"geojson"_s);
    q.addQueryItem(u"returnGeometry"_s, u"true"_s);
    q.addQueryItem(u"geometry"_s, boxText(box));
    q.addQueryItem(u"geometryType"_s, u"esriGeometryEnvelope"_s);
    q.addQueryItem(u"inSR"_s, u"4326"_s);
    q.addQueryItem(u"outSR"_s, u"4326"_s);
    q.addQueryItem(u"spatialRel"_s, u"esriSpatialRelIntersects"_s);
    q.addQueryItem(u"resultOffset"_s, QString::number(offset));
    q.addQueryItem(u"resultRecordCount"_s, QString::number(pageSize));
    q.addQueryItem(u"maxAllowableOffset"_s, QString::number(geozone::kSimplifyDeg));
    q.addQueryItem(u"geometryPrecision"_s, u"6"_s);
    QUrl url(layerUrl + u"/query"_s);
    url.setQuery(q);
    fetchJson(url, kTimeoutMs, [this, layerUrl, box, where, outFields, pageSize, done, acc, offset](const QJsonDocument &doc, const QString &err) {
        if (!err.isEmpty()) { done({}, err); return; }
        const QJsonObject o = doc.object();
        if (o.contains(u"error"_s)) {
            done({}, o.value(u"error"_s).toObject().value(u"message"_s).toString(u"ArcGIS error"_s));
            return;
        }
        const QJsonArray feats = o.value(u"features"_s).toArray();
        for (const QJsonValue &f : feats) acc->append(f);
        const bool more = o.value(u"exceededTransferLimit"_s).toBool()
                          || o.value(u"properties"_s).toObject().value(u"exceededTransferLimit"_s).toBool();
        if (!more || feats.isEmpty() || offset + feats.size() >= 100000) { done(*acc, {}); return; }
        arcgisFeatures(layerUrl, box, where, outFields, pageSize, done, acc, offset + int(feats.size()));
    });
}

void GeoZoneStore::fetchTile(const Job &job, Done done)
{
    const QString id = job.p->id;
    if (id == "faa"_L1) fetchFaa(job.box, done);
    else if (id == "enaire"_L1) fetchEnaire(job.box, done);
    else if (id == "dipul"_L1) fetchDipul(job.box, done);
    else if (id == "ign"_L1) fetchIgn(job.box, done);
    else done({}, u"unknown source"_s);
}

void GeoZoneStore::fetchFaa(const Box &box, Done done)
{
    struct Layer {
        QString id, url, where, outFields;
        std::function<std::optional<GeoZone>(const QJsonObject &p, const QJsonObject &geometry, const QString &fid)> zone;
    };
    using geozone::makeZone;
    using geozone::geojsonPolys;
    auto key = [](const QString &layer, const QJsonObject &p, const QString &fid) {
        const QString oid = p.value(u"OBJECTID"_s).toVariant().toString();
        return layer + '-'_L1 + (oid.isEmpty() ? fid : oid);
    };
    QList<Layer> layers = {
        {u"uasfm"_s, kFaaBase + u"/FAA_UAS_FacilityMap_Data/FeatureServer/0"_s, {},
         u"OBJECTID,CEILING,UNIT,APT1_ICAO,APT1_FAAID,APT1_NAME,AIRSPACE_1"_s,
         [key](const QJsonObject &p, const QJsonObject &g, const QString &fid) {
             const double ft = p.value(u"CEILING"_s).toDouble();
             QString apt = s(p, "APT1_ICAO");
             if (apt.isEmpty()) apt = s(p, "APT1_FAAID");
             if (apt.isEmpty()) apt = s(p, "APT1_NAME");
             const QString air = s(p, "AIRSPACE_1");
             const QString unit = s(p, "UNIT");
             return makeZone(u"faa"_s, key(u"uasfm"_s, p, fid),
                             u"UAS facility map · %1 class %2"_s.arg(apt, air.isEmpty() ? u"?"_s : air), u"prohibited"_s,
                             geozone::toMetres(ft, unit.isEmpty() ? u"Feet"_s : unit).value_or(0), true, std::nullopt, true, geojsonPolys(g),
                             ft > 0 ? u"Controlled airspace: LAANC authorisation up to %1 ft AGL"_s.arg(ft)
                                    : u"Controlled airspace: no LAANC authorisation, ask the FAA (DroneZone)"_s);
         }},
        {u"nsufr"_s, kFaaBase + u"/DoD_Mar_13/FeatureServer/0"_s, {}, u"OBJECTID,Facility,Base,Reason,Floor,Ceiling"_s,
         [key](const QJsonObject &p, const QJsonObject &g, const QString &fid) {
             const auto lo = faaLimitText(s(p, "Floor"), 0), hi = faaLimitText(s(p, "Ceiling"), 400);
             QString name = s(p, "Facility");
             if (name.isEmpty()) name = s(p, "Base");
             const QString reason = s(p, "Reason");
             return makeZone(u"faa"_s, key(u"nsufr"_s, p, fid), name, u"prohibited"_s, lo.first, lo.second, hi.first,
                             hi.second, geojsonPolys(g),
                             u"%1 UAS flight restriction (14 CFR 99.7)"_s.arg(reason.isEmpty() ? u"National security"_s : reason));
         }},
        {u"nsufr-pt"_s, kFaaBase + u"/Part_Time_National_Security_UAS_Flight_Restrictions/FeatureServer/0"_s, {},
         u"OBJECTID,Facility,Base,Reason,Floor,Ceiling,ALERTYPE"_s,
         [key](const QJsonObject &p, const QJsonObject &g, const QString &fid) {
             const auto lo = faaLimitText(s(p, "Floor"), 0), hi = faaLimitText(s(p, "Ceiling"), 400);
             QString name = s(p, "Facility");
             if (name.isEmpty()) name = s(p, "Base");
             const QString type = s(p, "ALERTYPE");
             return makeZone(u"faa"_s, key(u"nsufr-pt"_s, p, fid), name, u"conditional"_s, lo.first, lo.second, hi.first,
                             hi.second, geojsonPolys(g),
                             u"Part-time national security restriction (%1): active when published by NOTAM"_s.arg(type.isEmpty() ? u"intermittent"_s : type));
         }},
        {u"p"_s, kFaaBase + u"/Prohibited_Areas/FeatureServer/0"_s, {},
         u"OBJECTID,NAME,CITY,UPPER_VAL,UPPER_UOM,UPPER_CODE,LOWER_VAL,LOWER_UOM,LOWER_CODE,TIMESOFUSE"_s,
         [key](const QJsonObject &p, const QJsonObject &g, const QString &fid) {
             const auto lo = faaLimit(p.value(u"LOWER_VAL"_s), s(p, "LOWER_UOM"), s(p, "LOWER_CODE"));
             const auto hi = faaLimit(p.value(u"UPPER_VAL"_s), s(p, "UPPER_UOM"), s(p, "UPPER_CODE"));
             return makeZone(u"faa"_s, key(u"p"_s, p, fid), (s(p, "NAME") + ' '_L1 + s(p, "CITY")).trimmed(),
                             u"prohibited"_s, lo.first.value_or(0), lo.second, hi.first, hi.second, geojsonPolys(g),
                             u"Prohibited area · "_s + s(p, "TIMESOFUSE"));
         }},
        {u"r"_s, kFaaBase + u"/Special_Use_Airspace/FeatureServer/0"_s, u"TYPE_CODE='R'"_s,
         u"OBJECTID,NAME,UPPER_VAL,UPPER_UOM,UPPER_CODE,LOWER_VAL,LOWER_UOM,LOWER_CODE,TIMESOFUSE"_s,
         [key](const QJsonObject &p, const QJsonObject &g, const QString &fid) {
             const auto lo = faaLimit(p.value(u"LOWER_VAL"_s), s(p, "LOWER_UOM"), s(p, "LOWER_CODE"));
             const auto hi = faaLimit(p.value(u"UPPER_VAL"_s), s(p, "UPPER_UOM"), s(p, "UPPER_CODE"));
             return makeZone(u"faa"_s, key(u"r"_s, p, fid), s(p, "NAME"), u"conditional"_s, lo.first.value_or(0),
                             lo.second, hi.first, hi.second, geojsonPolys(g), u"Restricted area, when active · "_s + s(p, "TIMESOFUSE"));
         }},
        {u"nda"_s, kFaaBase + u"/National_Defense_Airspace_TFR_Areas/FeatureServer/0"_s, {}, u"OBJECTID,NAME,WKHR_RMK"_s,
         [key](const QJsonObject &p, const QJsonObject &g, const QString &fid) {
             const QString rmk = s(p, "WKHR_RMK");
             return makeZone(u"faa"_s, key(u"nda"_s, p, fid), s(p, "NAME"), u"conditional"_s, 0, true, std::nullopt, true,
                             geojsonPolys(g), u"National defense airspace TFR · "_s + (rmk.isEmpty() ? u"by NOTAM"_s : rmk));
         }},
    };
    auto zones = std::make_shared<GeoZoneList>();
    auto steps = std::make_shared<QList<std::function<void(std::function<void(QString)>)>>>();
    for (const Layer &l : layers) {
        steps->append([this, l, box, zones](std::function<void(QString)> next) {
            arcgisFeatures(l.url, box, l.where, l.outFields, 2000, [l, zones, next](const QJsonArray &feats, const QString &err) {
                if (!err.isEmpty()) { next(err); return; }
                for (const QJsonValue &fv : feats) {
                    const QJsonObject f = fv.toObject();
                    if (auto z = l.zone(f.value(u"properties"_s).toObject(), f.value(u"geometry"_s).toObject(),
                                        f.value(u"id"_s).toVariant().toString()))
                        zones->push_back(std::move(*z));
                }
                next({});
            });
        });
    }
    chain(steps, [zones, done](const QString &err) { done(err.isEmpty() ? std::move(*zones) : GeoZoneList{}, err); });
}

void GeoZoneStore::fetchEnaire(const Box &box, Done done)
{
    auto zones = std::make_shared<GeoZoneList>();
    auto steps = std::make_shared<QList<std::function<void(std::function<void(QString)>)>>>();
    for (const int layer : {0, 2, 3}) {   // infrastructure, aeronautical, urban
        steps->append([this, layer, box, zones](std::function<void(QString)> next) {
            arcgisFeatures(kEnaireUrl + '/'_L1 + QString::number(layer), box, {}, {}, 1000,
                           [layer, zones, next](const QJsonArray &feats, const QString &err) {
                if (!err.isEmpty()) { next(err); return; }
                for (const QJsonValue &fv : feats) {
                    const QJsonObject f = fv.toObject();
                    const QJsonObject p = f.value(u"properties"_s).toObject();
                    QString id = s(p, "GFID");
                    if (id.isEmpty()) id = s(p, "OBJECTID");
                    if (id.isEmpty()) id = f.value(u"id"_s).toVariant().toString();
                    QString name = s(p, "name");
                    if (name.isEmpty()) name = s(p, "identifier");
                    if (name.isEmpty()) name = s(p, "otherReasonInfo");
                    const QString other = s(p, "otherReasonInfo");
                    const QString uom = s(p, "uom");
                    auto z = geozone::makeZone(u"enaire"_s, u"%1-%2"_s.arg(layer).arg(id), name, geozone::restrictionKind(s(p, "type")),
                                               geozone::toMetres(p.value(u"lower"_s), uom).value_or(0), geozone::isAgl(s(p, "lowerReference")),
                                               geozone::toMetres(p.value(u"upper"_s), uom), geozone::isAgl(s(p, "upperReference")),
                                               geozone::geojsonPolys(f.value(u"geometry"_s).toObject()),
                                               other.isEmpty() ? s(p, "message") : other + u". "_s + s(p, "message"));
                    if (z) zones->push_back(std::move(*z));
                }
                next({});
            });
        });
    }
    chain(steps, [zones, done](const QString &err) { done(err.isEmpty() ? std::move(*zones) : GeoZoneList{}, err); });
}

void GeoZoneStore::fetchDipul(const Box &box, Done done)
{
    // Layer -> kind. Par. 21h LuftVO: prohibited unless permitted (airports, control
    // zones, sensitive sites) or under conditions set by the state (nature)
    static const QList<QPair<QString, QString>> layers = {
        {u"flughaefen"_s, u"prohibited"_s}, {u"flugplaetze"_s, u"prohibited"_s},
        {u"kontrollzonen"_s, u"prohibited"_s}, {u"flugbeschraenkungsgebiete"_s, u"prohibited"_s},
        {u"temporaere_betriebseinschraenkungen"_s, u"prohibited"_s}, {u"militaerische_anlagen"_s, u"prohibited"_s},
        {u"justizvollzugsanstalten"_s, u"prohibited"_s}, {u"kraftwerke"_s, u"prohibited"_s},
        {u"umspannwerke"_s, u"prohibited"_s}, {u"industrieanlagen"_s, u"prohibited"_s},
        {u"sicherheitsbehoerden"_s, u"prohibited"_s}, {u"polizei"_s, u"prohibited"_s},
        {u"behoerden"_s, u"prohibited"_s}, {u"diplomatische_vertretungen"_s, u"prohibited"_s},
        {u"internationale_organisationen"_s, u"prohibited"_s}, {u"labore"_s, u"prohibited"_s},
        {u"krankenhaeuser"_s, u"prohibited"_s}, {u"naturschutzgebiete"_s, u"conditional"_s},
        {u"nationalparks"_s, u"conditional"_s}, {u"ffh-gebiete"_s, u"conditional"_s},
        {u"vogelschutzgebiete"_s, u"conditional"_s}, {u"freibaeder"_s, u"conditional"_s},
    };
    auto zones = std::make_shared<GeoZoneList>();
    auto steps = std::make_shared<QList<std::function<void(std::function<void(QString)>)>>>();
    for (const auto &[layer, kind] : layers) {
        steps->append([this, layer, kind, box, zones](std::function<void(QString)> next) {
            QUrlQuery q;
            q.addQueryItem(u"SERVICE"_s, u"WFS"_s);
            q.addQueryItem(u"VERSION"_s, u"2.0.0"_s);
            q.addQueryItem(u"REQUEST"_s, u"GetFeature"_s);
            q.addQueryItem(u"TYPENAMES"_s, u"dipul:"_s + layer);
            q.addQueryItem(u"outputFormat"_s, u"application/json"_s);
            q.addQueryItem(u"srsName"_s, u"EPSG:4326"_s);
            q.addQueryItem(u"BBOX"_s, boxText(box) + u",EPSG:4326"_s);
            q.addQueryItem(u"count"_s, u"20000"_s);
            QUrl url(kDipulUrl);
            url.setQuery(q);
            fetchJson(url, kTimeoutMs, [layer, kind, zones, next](const QJsonDocument &doc, const QString &err) {
                if (!err.isEmpty()) { next(err); return; }
                for (const QJsonValue &fv : doc.object().value(u"features"_s).toArray()) {
                    const QJsonObject f = fv.toObject();
                    const QJsonObject p = f.value(u"properties"_s).toObject();
                    QString key = f.value(u"id"_s).toVariant().toString();
                    if (key.isEmpty()) key = layer + '-'_L1 + s(p, "external_reference");
                    QString name = s(p, "name");
                    if (name.isEmpty()) name = s(p, "generated_name_EN");
                    QStringList info;
                    const QString detail = s(p, "type_code_detail").isEmpty() ? s(p, "type_code") : s(p, "type_code_detail");
                    if (!detail.isEmpty()) info << detail;
                    if (!s(p, "legal_ref").isEmpty()) info << s(p, "legal_ref");
                    const QJsonValue up = p.value(u"upper_limit_altitude"_s);
                    auto z = geozone::makeZone(u"dipul"_s, key, name, kind,
                                               geozone::toMetres(p.value(u"lower_limit_altitude"_s), s(p, "lower_limit_unit")).value_or(0),
                                               geozone::isAgl(s(p, "lower_limit_alt_ref")),
                                               up.isNull() || up.isUndefined() ? std::nullopt : geozone::toMetres(up, s(p, "upper_limit_unit")),
                                               geozone::isAgl(s(p, "upper_limit_alt_ref")),
                                               geozone::geojsonPolys(f.value(u"geometry"_s).toObject()), info.join(u" · "_s));
                    if (z) zones->push_back(std::move(*z));
                }
                next({});
            });
        });
    }
    chain(steps, [zones, done](const QString &err) { done(err.isEmpty() ? std::move(*zones) : GeoZoneList{}, err); });
}

void GeoZoneStore::fetchIgn(const Box &box, Done done)
{
    QUrlQuery q;
    q.addQueryItem(u"SERVICE"_s, u"WFS"_s);
    q.addQueryItem(u"VERSION"_s, u"2.0.0"_s);
    q.addQueryItem(u"REQUEST"_s, u"GetFeature"_s);
    q.addQueryItem(u"TYPENAMES"_s, u"TRANSPORTS.DRONES.RESTRICTIONS:carte_restriction_drones_lf"_s);
    q.addQueryItem(u"outputFormat"_s, u"application/json"_s);
    q.addQueryItem(u"srsName"_s, u"EPSG:4326"_s);
    q.addQueryItem(u"BBOX"_s, boxText(box) + u",EPSG:4326"_s);
    q.addQueryItem(u"count"_s, u"20000"_s);
    QUrl url(kIgnUrl);
    url.setQuery(q);
    fetchJson(url, kTimeoutMs, [done](const QJsonDocument &doc, const QString &err) {
        if (!err.isEmpty()) { done({}, err); return; }
        static const QRegularExpression metres(u"(\\d+)\\s*m"_s), interdit(u"interdit"_s, QRegularExpression::CaseInsensitiveOption);
        GeoZoneList zones;
        for (const QJsonValue &fv : doc.object().value(u"features"_s).toArray()) {
            const QJsonObject f = fv.toObject();
            const QJsonObject p = f.value(u"properties"_s).toObject();
            const QString limit = s(p, "limite");
            const auto m = metres.match(limit);
            if (!m.hasMatch() && !interdit.match(limit).hasMatch()) continue;
            QString name = limit;
            name.remove(QRegularExpression(u"\\s*\\*$"_s));
            auto z = geozone::makeZone(u"ign"_s, f.value(u"id"_s).toVariant().toString(), name, u"prohibited"_s,
                                       m.hasMatch() ? m.captured(1).toDouble() : 0, true, std::nullopt, true,
                                       geozone::geojsonPolys(f.value(u"geometry"_s).toObject()), s(p, "remarque"));
            if (z) zones.push_back(std::move(*z));
        }
        done(std::move(zones), {});
    });
}

} // namespace corv
