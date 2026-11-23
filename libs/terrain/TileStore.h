// TileStore.h - one cache of map tiles for every consumer (TileCache.js, cache-first).
//
// <data root>/tiles/<provider>.mbtiles, SQLite with the MBTiles schema
// (zoom_level, tile_column, tile_row in TMS order, tile_data), readable by
// other tools. A request answers from memory, then from the file, then from
// the network (6 downloads at a time), and every downloaded tile is stored.
// The provider URL stays configurable as in the Electron version (settings
// key "tile-url-<provider>"); its terms of use apply unchanged.
//
// Consumers: the mini-map and the FLIGHT PLAN map (render thread, memory only),
// the 3D chunk imagery (ChunkImagery) and the offline download.
//
// Nothing here runs on the GUI thread: the store, its SQLite connections and
// its network manager live on a thread of their own, the images are decoded on
// a small low-priority pool, and the downloaded tiles are written in batches
// (one transaction every half second). On the GUI thread a tile cost a disk
// read, a JPEG decode and a commit, 3 to 20 ms of frozen frames for each.
// The public functions are safe from any thread; the signals are emitted off
// the GUI thread and reach its objects queued.
#pragma once

#include "core/LruCache.h"

#include <QHash>
#include <QImage>
#include <QMutex>
#include <QObject>
#include <QQueue>
#include <QSet>
#include <QSqlDatabase>
#include <QThread>
#include <QThreadPool>
#include <QTimer>
#include <array>
#include <atomic>
#include <vector>

QT_FORWARD_DECLARE_CLASS(QNetworkAccessManager)

namespace corv {

class TileStore : public QObject
{
    Q_OBJECT
public:
    static TileStore &instance();

    /// The tile if it is in memory; otherwise queued (disk, then network) and null for now.
    QImage tile(const QString &provider, int z, int x, int y);
    /// The tile if it is in memory, without queueing anything.
    QImage cached(const QString &provider, int z, int x, int y);
    QString urlTemplate(const QString &provider) const;
    /// The tile failed to download in this session (it is not asked again).
    bool hasFailed(const QString &provider, int z, int x, int y) const;

    void setNetworkEnabled(bool on) { m_network = on; }

    // Bulk download for a flight without network (SYS CONFIG → OFFLINE DATA DOWNLOAD)
    static qint64 tileCount(double north, double south, double west, double east, int zmin, int zmax);
    void prefetchArea(const QString &provider, double north, double south, double west, double east, int zmin, int zmax);
    void cancelPrefetch();

signals:
    void tileReady(const QString &provider, int z, int x, int y);
    /// Not on disk and the download failed (or the network is off): it will not come.
    void tileFailed(const QString &provider, int z, int x, int y);
    void prefetchProgress(int downloaded, int failed, int total);
    void prefetchFinished(int downloaded, int failed, int total, bool cancelled);

private:
    struct Req { QString provider; int z = 0, x = 0, y = 0; };
    struct Write { Req tile; QByteArray data; };

    TileStore();
    void shutdown();
    void readUrls();

    // Store thread
    QSqlDatabase db(const QString &provider);
    QByteArray readTile(const Req &r);
    bool onDisk(const Req &r);
    void load(const Req &r);
    void fetch(const Req &r);
    void pump();
    void queueWrite(const Req &r, const QByteArray &data);
    void flushWrites();
    void pumpPrefetch();
    void finishPrefetch(bool cancelled);
    QUrl urlOf(const Req &r) const;

    // Decoding pool
    void decode(const Req &r, const QByteArray &data, bool fromDisk);
    void deliver(const Req &r, const QImage &img);
    void fail(const Req &r, bool forGood);

    static QString key(const QString &p, int z, int x, int y) { return QStringLiteral("%1/%2/%3/%4").arg(p).arg(z).arg(x).arg(y); }
    static QString key(const Req &r) { return key(r.provider, r.z, r.x, r.y); }

    // Any thread, under m_memLock
    mutable QMutex m_memLock;
    LruCache<QString, QImage> m_mem{1500};
    QSet<QString> m_scheduled;
    QSet<QString> m_failed;
    QHash<QString, QString> m_urls;
    std::atomic<bool> m_network{true};

    // Store thread
    QThread m_thread;
    QThreadPool m_decoders;
    QNetworkAccessManager *m_net = nullptr;
    QQueue<Req> m_queue;
    QHash<QString, QString> m_connections;
    std::vector<Write> m_writes;
    QTimer *m_writeTimer = nullptr;
    int m_inFlight = 0;
    struct Bulk {
        QString provider;
        QList<std::array<int, 3>> tiles;
        qsizetype next = 0;
        int inFlight = 0, downloaded = 0, failed = 0, total = 0;
        bool running = false;
    } m_bulk;
    int m_bulkGen = 0;   // replies of an older run are ignored
};

} // namespace corv
