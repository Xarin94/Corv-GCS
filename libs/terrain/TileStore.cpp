#include "terrain/TileStore.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"

#include <QCoreApplication>
#include <QNetworkReply>
#include <QSqlError>
#include <QSqlQuery>
#include <algorithm>
#include <cmath>

namespace corv {

namespace {
constexpr int kMaxInFlight = 6;
} // namespace

TileStore &TileStore::instance()
{
    static TileStore s;
    return s;
}

TileStore::TileStore()
{
    m_net.setParent(this);
    m_net.setTransferTimeout(15000);
    // The first caller may be the render thread (a map painting): disk and network belong to the GUI thread
    if (QCoreApplication::instance() && thread() != QCoreApplication::instance()->thread())
        moveToThread(QCoreApplication::instance()->thread());
}

QString TileStore::urlTemplate(const QString &provider) const
{
    const QString custom = Settings::instance().value(QStringLiteral("tile-url-") + provider).toString();
    if (!custom.isEmpty()) return custom;
    if (provider == QLatin1String("osm")) return QStringLiteral("https://tile.openstreetmap.org/{z}/{x}/{y}.png");
    // The satellite source of the Electron mini-map (MapEngine.js)
    return QStringLiteral("https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}");
}

QSqlDatabase TileStore::db(const QString &provider)
{
    auto it = m_connections.find(provider);
    if (it != m_connections.end()) return QSqlDatabase::database(*it);
    const QString name = QStringLiteral("tiles-") + provider;
    QSqlDatabase d = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), name);
    d.setDatabaseName(DataRoot::tilesDir() + QLatin1Char('/') + provider + QStringLiteral(".mbtiles"));
    if (!d.open()) {
        LOG_E("tiles") << "cannot open the tile store:" << d.lastError().text();
    } else {
        QSqlQuery q(d);
        q.exec(QStringLiteral("CREATE TABLE IF NOT EXISTS tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB, "
                              "PRIMARY KEY (zoom_level, tile_column, tile_row))"));
        q.exec(QStringLiteral("CREATE TABLE IF NOT EXISTS metadata (name TEXT PRIMARY KEY, value TEXT)"));
        q.exec(QStringLiteral("INSERT OR IGNORE INTO metadata VALUES ('name', '%1'), ('format', 'png'), ('type', 'baselayer')").arg(provider));
        q.exec(QStringLiteral("PRAGMA journal_mode=WAL"));
    }
    m_connections.insert(provider, name);
    return d;
}

QImage TileStore::cached(const QString &provider, int z, int x, int y)
{
    QMutexLocker l(&m_memLock);
    if (QImage *img = m_mem.get(key(provider, z, x, y))) return *img;
    return {};
}

QImage TileStore::tile(const QString &provider, int z, int x, int y)
{
    const QString k = key(provider, z, x, y);
    {
        QMutexLocker l(&m_memLock);
        if (QImage *img = m_mem.get(k)) return *img;
        if (m_scheduled.contains(k)) return {};
        m_scheduled.insert(k);
    }
    // Disk and network belong to the store's thread
    QMetaObject::invokeMethod(this, [this, provider, z, x, y] { load(provider, z, x, y); }, Qt::QueuedConnection);
    return {};
}

void TileStore::load(const QString &provider, int z, int x, int y)
{
    const QString k = key(provider, z, x, y);
    if (m_pending.contains(k)) return;
    if (m_failed.contains(k)) {
        emit tileFailed(provider, z, x, y);
        return;
    }

    // Disk first: MBTiles rows count from the south (TMS)
    QSqlDatabase d = db(provider);
    if (d.isOpen()) {
        QSqlQuery q(d);
        q.prepare(QStringLiteral("SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?"));
        q.addBindValue(z);
        q.addBindValue(x);
        q.addBindValue((1 << z) - 1 - y);
        if (q.exec() && q.next()) {
            QImage img;
            if (img.loadFromData(q.value(0).toByteArray())) {
                {
                    QMutexLocker l(&m_memLock);
                    m_mem.put(k, img);
                    m_scheduled.remove(k);
                }
                // Outside the lock: a listener may read the store again (ChunkImagery does)
                emit tileReady(provider, z, x, y);
                return;
            }
        }
    }
    if (!m_network) {
        // Asked again once the network is back
        QMutexLocker l(&m_memLock);
        m_scheduled.remove(k);
        l.unlock();
        emit tileFailed(provider, z, x, y);
        return;
    }
    m_pending.insert(k);
    m_queue.enqueue({provider, z, x, y});
    pump();
}

void TileStore::pump()
{
    while (m_inFlight < kMaxInFlight && !m_queue.isEmpty()) {
        const Req r = m_queue.dequeue();
        QString url = urlTemplate(r.provider);
        url.replace(QStringLiteral("{z}"), QString::number(r.z))
            .replace(QStringLiteral("{x}"), QString::number(r.x))
            .replace(QStringLiteral("{y}"), QString::number(r.y))
            .replace(QStringLiteral("{s}"), QStringLiteral("a"));
        QNetworkRequest req{QUrl(url)};
        // A generic client name: nothing about the user goes to the tile server
        req.setHeader(QNetworkRequest::UserAgentHeader, QStringLiteral("CORV-GCS/" CORV_VERSION));
        ++m_inFlight;
        QNetworkReply *reply = m_net.get(req);
        connect(reply, &QNetworkReply::finished, this, [this, reply, r] {
            reply->deleteLater();
            --m_inFlight;
            const QString k = key(r.provider, r.z, r.x, r.y);
            m_pending.remove(k);
            const QByteArray data = reply->readAll();
            QImage img;
            if (reply->error() != QNetworkReply::NoError || !img.loadFromData(data)) {
                m_failed.insert(k);
                if (DebugLog::throttle(QStringLiteral("tile-fail"), 30000) >= 0)
                    LOG_W("tiles") << "tile" << k << "failed:" << reply->errorString();
                emit tileFailed(r.provider, r.z, r.x, r.y);
            } else {
                {
                    QMutexLocker l(&m_memLock);
                    m_mem.put(k, img);
                    m_scheduled.remove(k);
                }
                QSqlDatabase d = db(r.provider);
                if (d.isOpen()) {
                    QSqlQuery q(d);
                    q.prepare(QStringLiteral("INSERT OR REPLACE INTO tiles VALUES (?, ?, ?, ?)"));
                    q.addBindValue(r.z);
                    q.addBindValue(r.x);
                    q.addBindValue((1 << r.z) - 1 - r.y);
                    q.addBindValue(data);
                    q.exec();
                }
                emit tileReady(r.provider, r.z, r.x, r.y);
            }
            pump();
        });
    }
}

// ── Bulk download ────────────────────────────────────────────────────────────

namespace {
int lonToX(double lon, int z) { return int(std::floor((lon + 180.0) / 360.0 * (1 << z))); }
int latToY(double lat, int z)
{
    const double r = std::clamp(lat, -85.0511, 85.0511) * M_PI / 180;
    return int(std::floor((1 - std::log(std::tan(r) + 1 / std::cos(r)) / M_PI) / 2 * (1 << z)));
}
} // namespace

qint64 TileStore::tileCount(double north, double south, double west, double east, int zmin, int zmax)
{
    qint64 n = 0;
    for (int z = zmin; z <= zmax; ++z) {
        const qint64 xs = std::max(0, lonToX(east, z) - lonToX(west, z) + 1);
        const qint64 ys = std::max(0, latToY(south, z) - latToY(north, z) + 1);
        n += xs * ys;
    }
    return n;
}

bool TileStore::onDisk(const QString &provider, int z, int x, int y)
{
    QSqlDatabase d = db(provider);
    if (!d.isOpen()) return false;
    QSqlQuery q(d);
    q.prepare(QStringLiteral("SELECT 1 FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?"));
    q.addBindValue(z);
    q.addBindValue(x);
    q.addBindValue((1 << z) - 1 - y);
    return q.exec() && q.next();
}

void TileStore::prefetchArea(const QString &provider, double north, double south, double west, double east, int zmin, int zmax)
{
    m_bulk = Bulk{};
    ++m_bulkGen;
    m_bulk.provider = provider;
    for (int z = zmin; z <= zmax; ++z)
        for (int y = latToY(north, z); y <= latToY(south, z); ++y)
            for (int x = lonToX(west, z); x <= lonToX(east, z); ++x) m_bulk.tiles.append({z, x, y});
    m_bulk.total = int(m_bulk.tiles.size());
    m_bulk.running = true;
    LOG_I("tiles") << "offline download of" << m_bulk.total << provider << "tiles, zoom" << zmin << "-" << zmax;
    emit prefetchProgress(0, 0, m_bulk.total);
    pumpPrefetch();
}

void TileStore::cancelPrefetch()
{
    if (!m_bulk.running) return;
    m_bulk.running = false;
    emit prefetchFinished(m_bulk.downloaded, m_bulk.failed, m_bulk.total, true);
}

void TileStore::pumpPrefetch()
{
    constexpr int kBulkInFlight = 4;   // the map's own requests keep their 6
    while (m_bulk.running && m_bulk.inFlight < kBulkInFlight && m_bulk.next < m_bulk.tiles.size()) {
        const auto t = m_bulk.tiles[m_bulk.next++];
        if (onDisk(m_bulk.provider, t[0], t[1], t[2])) {
            ++m_bulk.downloaded;
            if (m_bulk.downloaded % 50 == 0) emit prefetchProgress(m_bulk.downloaded, m_bulk.failed, m_bulk.total);
            continue;
        }
        QString url = urlTemplate(m_bulk.provider);
        url.replace(QStringLiteral("{z}"), QString::number(t[0])).replace(QStringLiteral("{x}"), QString::number(t[1]))
            .replace(QStringLiteral("{y}"), QString::number(t[2])).replace(QStringLiteral("{s}"), QStringLiteral("a"));
        QNetworkRequest req{QUrl(url)};
        req.setHeader(QNetworkRequest::UserAgentHeader, QStringLiteral("CORV-GCS/" CORV_VERSION));
        ++m_bulk.inFlight;
        QNetworkReply *reply = m_net.get(req);
        const QString provider = m_bulk.provider;
        connect(reply, &QNetworkReply::finished, this, [this, reply, t, provider, gen = m_bulkGen] {
            reply->deleteLater();
            if (gen != m_bulkGen) return;
            --m_bulk.inFlight;
            const QByteArray data = reply->readAll();
            if (reply->error() != QNetworkReply::NoError || data.isEmpty()) {
                ++m_bulk.failed;
            } else {
                QSqlDatabase d = db(provider);
                if (d.isOpen()) {
                    QSqlQuery q(d);
                    q.prepare(QStringLiteral("INSERT OR REPLACE INTO tiles VALUES (?, ?, ?, ?)"));
                    q.addBindValue(t[0]);
                    q.addBindValue(t[1]);
                    q.addBindValue((1 << t[0]) - 1 - t[2]);
                    q.addBindValue(data);
                    q.exec();
                }
                ++m_bulk.downloaded;
            }
            if (!m_bulk.running) return;
            emit prefetchProgress(m_bulk.downloaded, m_bulk.failed, m_bulk.total);
            if (m_bulk.next >= m_bulk.tiles.size() && m_bulk.inFlight == 0) {
                m_bulk.running = false;
                emit prefetchFinished(m_bulk.downloaded, m_bulk.failed, m_bulk.total, false);
                return;
            }
            pumpPrefetch();
        });
    }
    if (m_bulk.running && m_bulk.next >= m_bulk.tiles.size() && m_bulk.inFlight == 0) {
        m_bulk.running = false;
        emit prefetchFinished(m_bulk.downloaded, m_bulk.failed, m_bulk.total, false);
    }
}

} // namespace corv
