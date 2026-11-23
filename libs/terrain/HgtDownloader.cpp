#include "terrain/HgtDownloader.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Inflate.h"
#include "terrain/Hgt.h"

#include <QFutureWatcher>
#include <QNetworkReply>
#include <QSaveFile>
#include <QtConcurrent>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kMaxConcurrent = 4;
constexpr qint64 kSrtm1Bytes = 3601LL * 3601 * 2;
} // namespace

HgtDownloader &HgtDownloader::instance()
{
    static HgtDownloader d;
    return d;
}

HgtDownloader::HgtDownloader()
{
    m_net.setTransferTimeout(60000);
}

int HgtDownloader::tileCount(double latSouth, double latNorth, double lonWest, double lonEast)
{
    const int lat0 = int(std::floor(latSouth)), lat1 = int(std::floor(latNorth));
    const int lon0 = int(std::floor(lonWest)), lon1 = int(std::floor(lonEast));
    return std::max(0, lat1 - lat0 + 1) * std::max(0, lon1 - lon0 + 1);
}

void HgtDownloader::ensure(int latBase, int lonBase)
{
    const int k = key(latBase, lonBase);
    if (m_failed.contains(k) || m_queued.contains(k) || m_active.contains(k)) return;
    // The store already looked for a missing tile, on the pool: no listing of the folders here again
    if (!HgtStore::instance().missing(latBase, lonBase) && !HgtStore::instance().find(latBase, lonBase).isEmpty()) return;
    m_queued.insert(k);
    m_queue.enqueue({latBase, lonBase});
    pump();
}

void HgtDownloader::downloadArea(double latSouth, double latNorth, double lonWest, double lonEast)
{
    m_area.clear();
    m_dl = m_skip = m_nf = m_err = 0;
    m_areaRunning = true;
    const int lat0 = int(std::floor(latSouth)), lat1 = int(std::floor(latNorth));
    const int lon0 = int(std::floor(lonWest)), lon1 = int(std::floor(lonEast));
    m_areaTotal = tileCount(latSouth, latNorth, lonWest, lonEast);
    for (int la = lat0; la <= lat1; ++la) {
        for (int lo = lon0; lo <= lon1; ++lo) {
            const int k = key(la, lo);
            if (!HgtStore::instance().find(la, lo).isEmpty()) { ++m_skip; continue; }
            m_area.insert(k);
            m_failed.remove(k);
            if (!m_queued.contains(k) && !m_active.contains(k)) {
                m_queued.insert(k);
                m_queue.enqueue({la, lo});
            }
        }
    }
    emit areaProgress(m_dl, m_skip, m_nf, m_err, m_areaTotal);
    if (m_area.isEmpty()) {
        m_areaRunning = false;
        emit areaFinished(m_dl, m_skip, m_nf, m_err, m_areaTotal, false);
        return;
    }
    pump();
}

void HgtDownloader::cancelArea()
{
    if (!m_areaRunning) return;
    // Drop what has not started; the running downloads finish on their own
    QQueue<QPair<int, int>> keep;
    while (!m_queue.isEmpty()) {
        const auto t = m_queue.dequeue();
        if (m_area.contains(key(t.first, t.second))) m_queued.remove(key(t.first, t.second));
        else keep.enqueue(t);
    }
    m_queue = keep;
    m_areaRunning = false;
    m_area.clear();
    emit areaFinished(m_dl, m_skip, m_nf, m_err, m_areaTotal, true);
}

void HgtDownloader::pump()
{
    while (m_active.size() < kMaxConcurrent && !m_queue.isEmpty()) {
        const auto t = m_queue.dequeue();
        m_queued.remove(key(t.first, t.second));
        start(t.first, t.second);
    }
}

void HgtDownloader::start(int latBase, int lonBase)
{
    const int k = key(latBase, lonBase);
    m_active.insert(k);
    const QString name = HgtTile::name(latBase, lonBase);
    const QUrl url(u"https://elevation-tiles-prod.s3.amazonaws.com/skadi/%1/%2.hgt.gz"_s.arg(name.left(3), name));
    QNetworkRequest req(url);
    // A generic client name: nothing about the user goes to the server
    req.setHeader(QNetworkRequest::UserAgentHeader, QStringLiteral("CORV-GCS/" CORV_VERSION));
    LOG_I("terrain") << "downloading" << name << "from AWS";
    QNetworkReply *reply = m_net.get(req);
    connect(reply, &QNetworkReply::finished, this, [this, reply, latBase, lonBase, name] {
        reply->deleteLater();
        const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        if (status == 404 || status == 403) { finishOne(latBase, lonBase, NotFound); return; }
        if (reply->error() != QNetworkReply::NoError) {
            LOG_W("terrain") << "download of" << name << "failed:" << reply->errorString();
            finishOne(latBase, lonBase, Error);
            return;
        }
        const QByteArray gz = reply->readAll();
        // 25 MB to inflate: off the GUI thread
        auto *w = new QFutureWatcher<int>(this);
        connect(w, &QFutureWatcher<int>::finished, this, [this, w, latBase, lonBase] {
            w->deleteLater();
            finishOne(latBase, lonBase, Result(w->result()));
        });
        w->setFuture(QtConcurrent::run([gz, name]() -> int {
            const QByteArray hgt = gunzip(gz, kSrtm1Bytes);
            if (hgt.size() != kSrtm1Bytes) {
                LOG_W("terrain") << name << ".hgt unexpected size:" << hgt.size();
                return Error;
            }
            QSaveFile f(DataRoot::terrainDir() + '/'_L1 + name + u".hgt"_s);
            if (!f.open(QIODevice::WriteOnly) || f.write(hgt) != hgt.size() || !f.commit()) {
                LOG_W("terrain") << name << ".hgt downloaded but could not be saved:" << f.errorString();
                return Error;
            }
            return Downloaded;
        }));
    });
}

void HgtDownloader::finishOne(int latBase, int lonBase, Result r)
{
    const int k = key(latBase, lonBase);
    m_active.remove(k);
    if (r == Downloaded) {
        LOG_I("terrain") << "tile" << HgtTile::name(latBase, lonBase) << "downloaded";
        HgtStore::instance().forget(latBase, lonBase);
    } else {
        m_failed.insert(k);
    }
    emit tileDone(latBase, lonBase, r);
    if (m_areaRunning && m_area.remove(k)) {
        switch (r) {
        case Downloaded: ++m_dl; break;
        case Skipped: ++m_skip; break;
        case NotFound: ++m_nf; break;
        case Error: ++m_err; break;
        }
        emit areaProgress(m_dl, m_skip, m_nf, m_err, m_areaTotal);
        if (m_area.isEmpty()) {
            m_areaRunning = false;
            emit areaFinished(m_dl, m_skip, m_nf, m_err, m_areaTotal, false);
        }
    }
    pump();
}

} // namespace corv
