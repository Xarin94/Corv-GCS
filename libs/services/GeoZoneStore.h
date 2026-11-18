// GeoZoneStore.h - UAS geographical zones around the mission and the vehicle
// (port of the store and the sources of geozones.js).
//
// No EASA service covers Europe: every state publishes its own zones (ED-269 /
// ED-318), the FAA publishes the US data on ArcGIS. Sources, each asked only
// where it can have data (its country's box):
//   faa     FAA (US): UAS facility map, national security restrictions (full
//           and part time), prohibited, restricted (SUA R) and national
//           defense TFR areas - ArcGIS FeatureServers, 0.5 degree tiles
//   dipul   DFS DIPUL (DE): WFS, one request per layer, 1 degree tiles
//   ign     IGN / DGAC (FR): Geoplateforme WFS "restrictions UAS"
//   enaire  ENAIRE (ES): ArcGIS MapServer, ED-318 attributes
//   bazl    FOCA / BAZL (CH): the whole country as one ED-269 file
//   imports ED-269 / ED-318 files the user loads (D-Flight for Italy...)
// Tiles are kept under <data root>/geozones/<source>/ (the same files as
// Electron) and fetched again after 6 h (a country file after 7 days); a tile
// that cannot be fetched keeps serving its last copy and is retried after
// 10 min. A query (circles of 50 km around the vehicle and the mission) answers
// at once with what is on disk and fetches the rest in the background;
// changed() follows when tiles arrive. GUI thread.
#pragma once

#include "scene/GeoZone.h"

#include <QHash>
#include <QJsonDocument>
#include <QNetworkAccessManager>
#include <QObject>
#include <QSet>
#include <QTimer>
#include <QVariantList>
#include <array>
#include <functional>

namespace corv {

class GeoZoneStore : public QObject
{
    Q_OBJECT
public:
    struct Circle {
        double lat = 0, lon = 0, radiusKm = 50;
    };
    struct Source {
        QString id, name, country, error;
        int tiles = 0, ready = 0, pending = 0;
    };
    struct Result {
        GeoZonesPtr zones;
        QList<Source> sources;
        int pending = 0;
    };
    struct Import {
        QString file, name;
        qint64 at = 0;
        int count = 0;
    };

    explicit GeoZoneStore(QString dir, QObject *parent = nullptr);

    Result query(const QList<Circle> &circles);
    /// Read an ED-269 / ED-318 file and keep its zones; the count, or -1 with *error.
    int importFile(const QString &path, QString *error);
    void removeImport(const QString &file);
    /// Forget every downloaded tile and country file (not the imports).
    void clear();
    QList<Import> imports();
    int pending() const { return int(m_inFlight.size()); }

signals:
    void changed();

private:
    struct Provider {
        QString id, name, country;
        double tileDeg = 0;          // 0: a country file
        QString file;
        int parallel = 4;
        QList<std::array<double, 4>> coverage;
    };
    struct Tile {
        qint64 at = 0, failedAt = 0;
        QString error;
        std::shared_ptr<GeoZoneList> zones;
    };
    struct Job {
        const Provider *p = nullptr;
        QString key, id;
        std::array<double, 4> box{};
    };
    using Done = std::function<void(GeoZoneList zones, QString error)>;
    using JsonDone = std::function<void(QJsonDocument doc, QString error)>;

    static const QList<Provider> &providers();
    Tile &tile(const Provider &p, const QString &key);
    Tile &countryFile(const Provider &p, qint64 now);
    void want(const Provider &p, int tx, int ty, qint64 now);
    void pump();
    void loadImports();
    void markChanged();
    QString tileFile(const Provider &p, const QString &key) const;

    void fetchJson(const QUrl &url, int timeoutMs, JsonDone done, int attempt = 0);
    void arcgisFeatures(const QString &layerUrl, const std::array<double, 4> &box, const QString &where, const QString &outFields,
                        int pageSize, std::function<void(QJsonArray, QString)> done, std::shared_ptr<QJsonArray> acc = {}, int offset = 0);
    void fetchTile(const Job &job, Done done);
    void fetchFaa(const std::array<double, 4> &box, Done done);
    void fetchEnaire(const std::array<double, 4> &box, Done done);
    void fetchDipul(const std::array<double, 4> &box, Done done);
    void fetchIgn(const std::array<double, 4> &box, Done done);

    QString m_dir;
    QNetworkAccessManager m_net;
    QHash<QString, Tile> m_tiles;          // "provider/key"
    QHash<QString, Tile> m_files;          // country files by provider id
    struct ImportFile {
        QString name;
        qint64 at = 0;
        std::shared_ptr<GeoZoneList> zones;
    };
    QHash<QString, ImportFile> m_imports;  // file name -> zones
    bool m_importsLoaded = false;
    QList<Job> m_queue;
    QSet<QString> m_inFlight;
    QHash<QString, int> m_activeBy;
    int m_active = 0;
    QHash<QString, QString> m_lastError;
    QTimer m_changeTimer;
};

} // namespace corv
