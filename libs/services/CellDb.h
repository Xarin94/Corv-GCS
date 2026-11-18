// CellDb.h - LTE cell sites around the vehicle (port of cell-db.js).
//
// Layers, first wins when a cell (mcc, mnc, ECI) is in more than one:
//  1. CSV files the user added (<data root>/celldb/sources): an OpenCelliD
//     country file downloaded with the user's token (DOWNLOAD FROM OPENCELLID),
//     or any OpenCelliD / MLS layout CSV (IMPORT CSV);
//  2. the CSV shipped with the installer (<install>/celldb: OpenCelliD of the
//     build date, the June 2024 OpenCelliD copy and the final Mozilla Location
//     Service export, for Italy, San Marino and the Vatican);
//  3. the OpenCelliD copy published as one PMTiles file by UN Smart Maps on
//     Source Cooperative (June 2024, public), read around the vehicle with HTTP
//     range requests, never the whole 560 MB file;
//  4. single cells found online (beaconDB, or the OpenCelliD API with a token)
//     when the serving cell is in none of the above (lookup).
//
// PMTiles storage unit: one zoom-8 tile (a "block", ~110 km at 45 N). Its
// zoom-14 tiles, the only zoom that holds every cell, are one contiguous run of
// tile ids in the Hilbert order of PMTiles and contiguous on disk (clustered
// archive): a block costs a directory lookup and one or two range requests.
// LTE cells of a block are kept as JSON under <data root>/celldb/blocks, in
// the Electron format, so the two versions share them; once there they work
// offline. CSV layers are indexed by the same blocks.
//
// Positions are estimates (averaged crowd measurements), not surveyed masts:
// off by hundreds of metres, sometimes more. OpenCelliD is CC BY-SA 4.0,
// MLS and beaconDB public domain.
//
// The work (CSV parsing, downloads, decoding) runs on worker threads with
// blocking requests; every call returns on the GUI thread through its callback.
#pragma once

#include "scene/CellOverlay.h"

#include <QObject>
#include <QThreadPool>
#include <QVariantMap>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace corv {

namespace celldb {
constexpr int kTileZ = 14;      // maximum zoom of the archive: no feature dropped
constexpr int kBlockZ = 8;

/// A cell as the layers hold it.
struct Row {
    double lat = 0, lon = 0;
    int mcc = 0, mnc = 0;
    unsigned tac = 0, eci = 0;
    int range = 0, samples = 0;
    QString src;
};

/// PMTiles directory entry.
struct DirEntry {
    quint64 tileId = 0;
    quint64 runLength = 0;
    quint64 length = 0;
    quint64 offset = 0;
};

int lonToX(double lon, int z);
int latToY(double lat, int z);
/// PMTiles v3 tile id: the tiles of all lower zooms first, then the Hilbert index.
quint64 zxyToTileId(int z, quint32 x, quint32 y);
bool tileIdToZxy(quint64 id, int *z, quint32 *x, quint32 *y);
/// Zoom-14 tile ids of a block, one run [lo, hi].
std::pair<quint64, quint64> blockTileRange(int bx, int by);
/// A gzip-compressed directory; empty with ok = false when malformed.
std::vector<DirEntry> parseDirectory(const QByteArray &gz, bool *ok = nullptr);
/// The LTE cells of one zoom-14 vector tile (MVT, already decompressed).
void decodeTile(const QByteArray &mvt, quint32 x, quint32 y, std::vector<Row> &out);
double distanceKm(double lat1, double lon1, double lat2, double lon2);
} // namespace celldb

class CellDb : public QObject
{
    Q_OBJECT
public:
    static constexpr const char *kSourceUrl = "https://data.source.coop/smartmaps/opencellid/cellid.pmtiles";

    struct Query {
        double lat = 0, lon = 0, radiusKm = 100;
        std::optional<int> mcc, mnc;        // only this network (a cell of `eci` is kept under any MNC)
        std::optional<unsigned> eci;
        bool download = true;
    };
    struct Result {
        std::vector<CellSite> sites;
        int missing = 0;                    // blocks with no cells from any layer
        QString date;                       // of the data
        QString error;
    };

    /// dir: <data root>/celldb; bundleDir: the folder of the CSV shipped with the installer.
    CellDb(const QString &dir, const QString &bundleDir, QObject *parent = nullptr);
    ~CellDb() override;

    void query(const Query &q, std::function<void(Result)> done);
    /// One cell looked up online and remembered: found for good, not found for 6 hours.
    /// error is set when offline (try again later).
    void lookup(int mcc, int mnc, unsigned tac, unsigned eci, std::function<void(std::optional<CellRecord>, QString error)> done);
    /// The OpenCelliD file of one country (MCC) with the user's token, as the newest layer.
    void downloadOpenCellid(int mcc, std::function<void(QString error, int cells)> done);
    /// A CSV (OpenCelliD / MLS layout, or the bundle's) added as the newest layer.
    void importCsv(const QString &file, std::function<void(QString error, int cells)> done);
    /// Every block of a box from the 2024 copy, ahead of a flight without network.
    void downloadBox(double south, double west, double north, double east, std::function<void(int done, int total, int cells, int failed)> progress,
                     std::function<void(int blocks, int cells, int failed)> done);
    /// blocks, cells, bytes, date, bundle, files, lookups, settings (as the CELLULAR LINK page shows them).
    void status(std::function<void(QVariantMap)> done);
    /// Deletes what was downloaded, imported or looked up; the bundle and the settings stay.
    void clear(std::function<void()> done);

    /// online (look missing cells up), hasToken.
    QVariantMap settings() const;
    void setOnline(bool on);
    void setToken(const QString &token);

    struct Core;

private:
    std::shared_ptr<Core> m_core;
    QThreadPool m_pool;      // queries and lookups
    QThreadPool m_bulk;      // country downloads and imports, which can take minutes
};

} // namespace corv
