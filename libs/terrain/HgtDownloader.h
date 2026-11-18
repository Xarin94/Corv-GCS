// HgtDownloader.h - SRTM1 tiles from the AWS terrain tiles bucket (the
// autoDownloadSRTM of TerrainManager.js and the SRTM half of OfflineDownloader.js).
//
//   https://elevation-tiles-prod.s3.amazonaws.com/skadi/N47/N47E011.hgt.gz
//
// A downloaded tile is gunzipped, checked for the SRTM1 size and saved to
// <data root>/terrain, where HgtStore finds it. A tile the bucket does not
// have (open sea) is remembered as failed until resetFailures().
#pragma once

#include <QHash>
#include <QNetworkAccessManager>
#include <QObject>
#include <QQueue>
#include <QSet>

namespace corv {

class HgtDownloader : public QObject
{
    Q_OBJECT
public:
    enum Result { Downloaded, Skipped, NotFound, Error };

    static HgtDownloader &instance();

    /// Fetch the tile if no folder has it and it did not fail before. GUI thread.
    void ensure(int latBase, int lonBase);
    void resetFailures() { m_failed.clear(); }
    bool busy(int latBase, int lonBase) const { return m_active.contains(key(latBase, lonBase)) || m_queued.contains(key(latBase, lonBase)); }

    // Bulk download (OFFLINE DATA DOWNLOAD): tiles of a lat/lon rectangle, 4 at a time
    void downloadArea(double latSouth, double latNorth, double lonWest, double lonEast);
    void cancelArea();
    static int tileCount(double latSouth, double latNorth, double lonWest, double lonEast);

signals:
    void tileDone(int latBase, int lonBase, int result);
    /// Progress of downloadArea(): counts so far and the total.
    void areaProgress(int downloaded, int skipped, int notFound, int errors, int total);
    void areaFinished(int downloaded, int skipped, int notFound, int errors, int total, bool cancelled);

private:
    HgtDownloader();
    static int key(int latBase, int lonBase) { return (latBase + 90) * 1000 + (lonBase + 180); }
    void pump();
    void start(int latBase, int lonBase);
    void finishOne(int latBase, int lonBase, Result r);

    QNetworkAccessManager m_net;
    QQueue<QPair<int, int>> m_queue;
    QSet<int> m_queued, m_active, m_failed;
    // Area run
    QSet<int> m_area;
    int m_areaTotal = 0, m_dl = 0, m_skip = 0, m_nf = 0, m_err = 0;
    bool m_areaRunning = false;
};

} // namespace corv
