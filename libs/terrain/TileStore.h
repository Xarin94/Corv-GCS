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
// the 3D chunk imagery (ChunkImagery) and the offline download. The store
// itself, its SQLite connections and its network manager live on the GUI
// thread whichever thread asks first.
#pragma once

#include "core/LruCache.h"

#include <QHash>
#include <QMutex>
#include <QImage>
#include <QNetworkAccessManager>
#include <QObject>
#include <QQueue>
#include <QSet>
#include <QSqlDatabase>
#include <array>

namespace corv {

class TileStore : public QObject
{
    Q_OBJECT
public:
    static TileStore &instance();

    /// The tile if it is in memory; otherwise queued (disk, then network) and null for now.
    /// Any thread (the map paints on the render thread): only the memory cache is read here.
    QImage tile(const QString &provider, int z, int x, int y);
    /// The tile if it is in memory, without queueing anything.
    QImage cached(const QString &provider, int z, int x, int y);
    QString urlTemplate(const QString &provider) const;
    /// GUI thread: the tile failed to download in this session (it is not asked again).
    bool hasFailed(const QString &provider, int z, int x, int y) const { return m_failed.contains(key(provider, z, x, y)); }

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
    TileStore();
    QSqlDatabase db(const QString &provider);
    void load(const QString &provider, int z, int x, int y);
    void pump();
    void pumpPrefetch();
    bool onDisk(const QString &provider, int z, int x, int y);
    struct Bulk {
        QString provider;
        QList<std::array<int, 3>> tiles;
        qsizetype next = 0;
        int inFlight = 0, downloaded = 0, failed = 0, total = 0;
        bool running = false;
    } m_bulk;
    int m_bulkGen = 0;   // replies of an older run are ignored
    static QString key(const QString &p, int z, int x, int y) { return QStringLiteral("%1/%2/%3/%4").arg(p).arg(z).arg(x).arg(y); }

    struct Req { QString provider; int z, x, y; };
    QMutex m_memLock;
    LruCache<QString, QImage> m_mem{1500};
    QSet<QString> m_scheduled;
    QNetworkAccessManager m_net;
    QQueue<Req> m_queue;
    QSet<QString> m_pending;
    QSet<QString> m_failed;
    QHash<QString, QString> m_connections;
    int m_inFlight = 0;
    bool m_network = true;
};

} // namespace corv
