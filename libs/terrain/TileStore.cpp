#include "terrain/TileStore.h"

#include "core/Clock.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"

#include <QCoreApplication>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QSqlError>
#include <QSqlQuery>
#include <QUrl>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kMaxInFlight = 6;
constexpr int kBulkInFlight = 4;          // the map's own requests keep their 6
constexpr int kBulkChecksPerTurn = 200;   // tiles looked up on disk before the thread serves the map again
constexpr int kWriteDelayMs = 500;
constexpr std::size_t kMaxWrites = 64;
constexpr int kRetryMs = 60000;
constexpr int kOfflineRetryMs = 10000;
constexpr int kLostAfter = 8;             // downloads in a row without an answer
constexpr int kProbeMs = 10000;
const QLatin1StringView kUrlKey{"tile-url-"};
} // namespace

TileStore &TileStore::instance()
{
    static TileStore s;
    return s;
}

TileStore::TileStore()
{
    // Settings belong to the GUI thread: the URL templates are copied here and kept up to date
    readUrls();
    connect(&Settings::instance(), &Settings::changed, this, [this](const QString &k) {
        if (k.startsWith(kUrlKey)) readUrls();
    }, Qt::DirectConnection);

    m_decoders.setMaxThreadCount(std::clamp(QThread::idealThreadCount() / 4, 1, 4));
    m_decoders.setThreadPriority(QThread::LowPriority);
    m_thread.setObjectName(u"tiles"_s);
    moveToThread(&m_thread);
    m_thread.start(QThread::LowPriority);
    QMetaObject::invokeMethod(this, [this] {
        m_net = new QNetworkAccessManager(this);
        m_net->setTransferTimeout(15000);
        m_writeTimer = new QTimer(this);
        m_writeTimer->setSingleShot(true);
        m_writeTimer->setInterval(kWriteDelayMs);
        connect(m_writeTimer, &QTimer::timeout, this, &TileStore::flushWrites);
        m_probeTimer = new QTimer(this);
        m_probeTimer->setInterval(kProbeMs);
        connect(m_probeTimer, &QTimer::timeout, this, &TileStore::probe);
    }, Qt::QueuedConnection);
    // The end of the application: the last tiles are written and the connections closed
    // on the thread that opened them. Not in the destructor: static objects go after the
    // application, when waiting for a thread with an event loop never ends.
    qAddPostRoutine([] { TileStore::instance().shutdown(); });
}

void TileStore::shutdown()
{
    if (!m_thread.isRunning()) return;
    m_network = false;
    QMetaObject::invokeMethod(this, [this] {
        m_bulk.running = false;
        m_queue.clear();
        m_probeTimer->stop();
        flushWrites();
        delete m_net;
        m_net = nullptr;
        for (const QString &name : std::as_const(m_connections)) {
            QSqlDatabase::database(name, false).close();
            QSqlDatabase::removeDatabase(name);
        }
        m_connections.clear();
    }, Qt::BlockingQueuedConnection);
    m_thread.quit();
    m_thread.wait();
    m_decoders.clear();
    m_decoders.waitForDone();
}

// ── URLs ─────────────────────────────────────────────────────────────────────

void TileStore::readUrls()
{
    QHash<QString, QString> urls{
        {u"osm"_s, u"https://tile.openstreetmap.org/{z}/{x}/{y}.png"_s},
        // The satellite source of the Electron mini-map (MapEngine.js)
        {u"satellite"_s, u"https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"_s},
    };
    for (auto it = urls.begin(); it != urls.end(); ++it) {
        const QString custom = Settings::instance().value(kUrlKey + it.key()).toString();
        if (!custom.isEmpty()) *it = custom;
    }
    QMutexLocker l(&m_memLock);
    m_urls = urls;
}

QString TileStore::urlTemplate(const QString &provider) const
{
    QMutexLocker l(&m_memLock);
    return m_urls.value(provider, m_urls.value(u"satellite"_s));
}

QUrl TileStore::urlOf(const Req &r) const
{
    QString url = urlTemplate(r.provider);
    url.replace(u"{z}"_s, QString::number(r.z))
        .replace(u"{x}"_s, QString::number(r.x))
        .replace(u"{y}"_s, QString::number(r.y))
        .replace(u"{s}"_s, u"a"_s);
    return QUrl(url);
}

// ── Memory (any thread) ──────────────────────────────────────────────────────

QImage TileStore::cached(const QString &provider, int z, int x, int y)
{
    QMutexLocker l(&m_memLock);
    if (QImage *img = m_mem.get(key(provider, z, x, y))) return *img;
    return {};
}

bool TileStore::hasFailed(const QString &provider, int z, int x, int y) const
{
    const QString k = key(provider, z, x, y);
    QMutexLocker l(&m_memLock);
    return m_failed.contains(k) || m_retryAt.value(k) > wallMs();
}

bool TileStore::failedForNow(const QString &provider, int z, int x, int y) const
{
    QMutexLocker l(&m_memLock);
    return m_retryAt.contains(key(provider, z, x, y));
}

QImage TileStore::tile(const QString &provider, int z, int x, int y)
{
    const Req r{provider, z, x, y};
    const QString k = key(r);
    {
        QMutexLocker l(&m_memLock);
        if (QImage *img = m_mem.get(k)) return *img;
        if (m_scheduled.contains(k)) return {};
        if (const auto retry = m_retryAt.constFind(k); retry != m_retryAt.cend()) {
            if (*retry > wallMs()) return {};
            m_retryAt.erase(retry);
        }
        m_scheduled.insert(k);
    }
    QMetaObject::invokeMethod(this, [this, r] { load(r); }, Qt::QueuedConnection);
    return {};
}

void TileStore::deliver(const Req &r, const QImage &img)
{
    const QString k = key(r);
    {
        QMutexLocker l(&m_memLock);
        m_mem.put(k, img);
        m_scheduled.remove(k);
    }
    // Outside the lock: a listener may read the store again (ChunkImagery does)
    emit tileReady(r.provider, r.z, r.x, r.y);
}

void TileStore::fail(const Req &r, bool forGood)
{
    {
        // For good: it stays scheduled, so it is not asked again. Otherwise it is after a while
        QMutexLocker l(&m_memLock);
        if (forGood) {
            m_failed.insert(key(r));
        } else {
            m_retryAt.insert(key(r), wallMs() + (m_online ? kRetryMs : kOfflineRetryMs));
            m_scheduled.remove(key(r));
        }
    }
    emit tileFailed(r.provider, r.z, r.x, r.y);
}

// ── Disk (store thread) ──────────────────────────────────────────────────────

QSqlDatabase TileStore::db(const QString &provider)
{
    auto it = m_connections.find(provider);
    if (it != m_connections.end()) return QSqlDatabase::database(*it, false);
    const QString name = u"tiles-"_s + provider;
    QSqlDatabase d = QSqlDatabase::addDatabase(u"QSQLITE"_s, name);
    d.setDatabaseName(DataRoot::tilesDir() + '/'_L1 + provider + u".mbtiles"_s);
    if (!d.open()) {
        LOG_E("tiles") << "cannot open the tile store:" << d.lastError().text();
    } else {
        QSqlQuery q(d);
        q.exec(QStringLiteral("CREATE TABLE IF NOT EXISTS tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB, "
                              "PRIMARY KEY (zoom_level, tile_column, tile_row))"));
        q.exec(u"CREATE TABLE IF NOT EXISTS metadata (name TEXT PRIMARY KEY, value TEXT)"_s);
        q.exec(u"INSERT OR IGNORE INTO metadata VALUES ('name', '%1'), ('format', 'png'), ('type', 'baselayer')"_s.arg(provider));
        q.exec(u"PRAGMA journal_mode=WAL"_s);
        // A power cut may lose the last tiles, never the file: they are only a cache
        q.exec(u"PRAGMA synchronous=NORMAL"_s);
    }
    m_connections.insert(provider, name);
    return d;
}

namespace {
void bindTile(QSqlQuery &q, int z, int x, int y)
{
    // MBTiles rows count from the south (TMS)
    q.addBindValue(z);
    q.addBindValue(x);
    q.addBindValue((1 << z) - 1 - y);
}
} // namespace

QByteArray TileStore::readTile(const Req &r)
{
    QSqlDatabase d = db(r.provider);
    if (!d.isOpen()) return {};
    QSqlQuery q(d);
    q.setForwardOnly(true);
    q.prepare(u"SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?"_s);
    bindTile(q, r.z, r.x, r.y);
    return q.exec() && q.next() ? q.value(0).toByteArray() : QByteArray();
}

bool TileStore::onDisk(const Req &r)
{
    QSqlDatabase d = db(r.provider);
    if (!d.isOpen()) return false;
    QSqlQuery q(d);
    q.setForwardOnly(true);
    q.prepare(u"SELECT 1 FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?"_s);
    bindTile(q, r.z, r.x, r.y);
    return q.exec() && q.next();
}

void TileStore::queueWrite(const Req &r, const QByteArray &data)
{
    m_writes.push_back({r, data});
    if (m_writes.size() >= kMaxWrites) flushWrites();
    else if (!m_writeTimer->isActive()) m_writeTimer->start();
}

void TileStore::flushWrites()
{
    if (m_writeTimer) m_writeTimer->stop();
    const std::vector<Write> writes = std::exchange(m_writes, {});
    QStringList providers;
    for (const Write &w : writes)
        if (!providers.contains(w.tile.provider)) providers << w.tile.provider;
    for (const QString &p : std::as_const(providers)) {
        QSqlDatabase d = db(p);
        if (!d.isOpen()) continue;
        d.transaction();
        QSqlQuery q(d);
        q.prepare(u"INSERT OR REPLACE INTO tiles VALUES (?, ?, ?, ?)"_s);
        for (const Write &w : writes) {
            if (w.tile.provider != p) continue;
            bindTile(q, w.tile.z, w.tile.x, w.tile.y);
            q.addBindValue(w.data);
            q.exec();
        }
        if (!d.commit() && DebugLog::throttle(u"tile-write"_s, 30000) >= 0)
            LOG_W("tiles") << "tiles not written:" << d.lastError().text();
    }
}

// ── Loading ──────────────────────────────────────────────────────────────────

void TileStore::load(const Req &r)
{
    // Disk first
    if (const QByteArray data = readTile(r); !data.isEmpty()) decode(r, data, true);
    else fetch(r);
}

void TileStore::decode(const Req &r, const QByteArray &data, bool fromDisk)
{
    m_decoders.start([this, r, data, fromDisk] {
        QImage img;
        if (img.loadFromData(data)) {
            // The formats QPainter and the textures take without converting again
            deliver(r, std::move(img).convertToFormat(img.hasAlphaChannel() ? QImage::Format_ARGB32_Premultiplied : QImage::Format_RGB32));
            if (!fromDisk) QMetaObject::invokeMethod(this, [this, r, data] { queueWrite(r, data); }, Qt::QueuedConnection);
        } else if (fromDisk) {
            QMetaObject::invokeMethod(this, [this, r] { fetch(r); }, Qt::QueuedConnection);   // a damaged row
        } else {
            // (the login page of a captive portal, most often)
            if (DebugLog::throttle(u"tile-fail"_s, 30000) >= 0) LOG_W("tiles") << "tile" << key(r) << "is not an image";
            fail(r, false);
        }
    });
}

void TileStore::fetch(const Req &r)
{
    if (!m_network || !m_online) {
        fail(r, false);
        return;
    }
    m_queue.enqueue(r);
    pump();
}

void TileStore::pump()
{
    while (m_net && m_inFlight < kMaxInFlight && !m_queue.isEmpty()) {
        const Req r = m_queue.dequeue();
        QNetworkRequest req(urlOf(r));
        // A generic client name: nothing about the user goes to the tile server
        req.setHeader(QNetworkRequest::UserAgentHeader, QStringLiteral("CORV-GCS/" CORV_VERSION));
        ++m_inFlight;
        QNetworkReply *reply = m_net->get(req);
        connect(reply, &QNetworkReply::finished, this, [this, reply, r] {
            reply->deleteLater();
            --m_inFlight;
            const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
            if (status) m_unanswered = 0;
            if (reply->error() == QNetworkReply::NoError) {
                decode(r, reply->readAll(), false);
            } else {
                if (DebugLog::throttle(u"tile-fail"_s, 30000) >= 0)
                    LOG_W("tiles") << "tile" << key(r) << "failed:" << reply->errorString();
                // The server has no such tile; anything else may pass
                fail(r, status == 400 || status == 404 || status == 410);
                if (!status) unanswered();
            }
            pump();
        });
    }
}

void TileStore::unanswered()
{
    if (++m_unanswered < kLostAfter || !m_online) return;
    LOG_W("tiles") << "no answer from the tile servers: cached tiles only";
    m_online = false;
    while (!m_queue.isEmpty()) fail(m_queue.dequeue(), false);
    m_probeTimer->start();
    emit networkLost();
}

void TileStore::probe()
{
    if (!m_net || m_online) return;
    QNetworkRequest req(urlOf({u"satellite"_s, 0, 0, 0}));
    req.setHeader(QNetworkRequest::UserAgentHeader, QStringLiteral("CORV-GCS/" CORV_VERSION));
    req.setTransferTimeout(5000);
    QNetworkReply *reply = m_net->get(req);
    connect(reply, &QNetworkReply::finished, this, [this, reply] {
        reply->deleteLater();
        // Any answer will do: the network is there
        if (m_online || !reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt()) return;
        LOG_I("tiles") << "the tile servers answer again";
        m_unanswered = 0;
        m_online = true;
        m_probeTimer->stop();
        {
            QMutexLocker l(&m_memLock);
            m_retryAt.clear();
        }
        emit networkRestored();
    });
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

void TileStore::prefetchArea(const QString &provider, double north, double south, double west, double east, int zmin, int zmax)
{
    QMetaObject::invokeMethod(this, [=, this] {
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
    }, Qt::QueuedConnection);
}

void TileStore::cancelPrefetch()
{
    QMetaObject::invokeMethod(this, [this] {
        if (m_bulk.running) finishPrefetch(true);
    }, Qt::QueuedConnection);
}

void TileStore::finishPrefetch(bool cancelled)
{
    m_bulk.running = false;
    emit prefetchFinished(m_bulk.downloaded, m_bulk.failed, m_bulk.total, cancelled);
}

void TileStore::pumpPrefetch()
{
    int checked = 0;
    while (m_net && m_bulk.running && m_bulk.inFlight < kBulkInFlight && m_bulk.next < m_bulk.tiles.size()) {
        if (++checked > kBulkChecksPerTurn) {
            QMetaObject::invokeMethod(this, [this] { pumpPrefetch(); }, Qt::QueuedConnection);
            return;
        }
        const auto t = m_bulk.tiles[m_bulk.next++];
        const Req r{m_bulk.provider, t[0], t[1], t[2]};
        if (onDisk(r)) {
            ++m_bulk.downloaded;
            if (m_bulk.downloaded % 50 == 0) emit prefetchProgress(m_bulk.downloaded, m_bulk.failed, m_bulk.total);
            continue;
        }
        QNetworkRequest req(urlOf(r));
        req.setHeader(QNetworkRequest::UserAgentHeader, QStringLiteral("CORV-GCS/" CORV_VERSION));
        ++m_bulk.inFlight;
        QNetworkReply *reply = m_net->get(req);
        connect(reply, &QNetworkReply::finished, this, [this, reply, r, gen = m_bulkGen] {
            reply->deleteLater();
            if (gen != m_bulkGen) return;
            --m_bulk.inFlight;
            const QByteArray data = reply->readAll();
            if (reply->error() != QNetworkReply::NoError || data.isEmpty()) {
                ++m_bulk.failed;
            } else {
                queueWrite(r, data);
                ++m_bulk.downloaded;
            }
            if (!m_bulk.running) return;
            emit prefetchProgress(m_bulk.downloaded, m_bulk.failed, m_bulk.total);
            pumpPrefetch();
        });
    }
    if (m_bulk.running && m_bulk.next >= m_bulk.tiles.size() && m_bulk.inFlight == 0) finishPrefetch(false);
}

} // namespace corv
