// MissionStore.h - saved missions on disk (port of mission-store.js).
//
//   <data root>/index.json       catalogue of every mission and log file
//   <data root>/missions/*.json  saved missions, format "corv-gcs-mission" v2
//   <data root>/logs/            flight logs (written by the link layer)
//
// index.json is rebuilt from the folders on every read and write: a cache for
// other tools, never the source of truth. Deleting it or dropping a mission
// file in by hand both work. The files are the Electron version's, unchanged.
#pragma once

#include <QJsonArray>
#include <QJsonObject>
#include <QList>
#include <QString>

namespace corv::mission {

struct MissionEntry {
    QString id, file, name, notes, created, modified;
    int waypoints = 0;
    double distanceM = 0;
};

struct LogEntry {
    QString file, name, type, modified;
    qint64 size = 0;
};

struct StoreIndex {
    QString root;
    bool portable = true;
    QList<MissionEntry> missions;   // newest first
    QList<LogEntry> logs;
};

class MissionStore
{
public:
    /// Scan missions/ and logs/ and rewrite index.json.
    static StoreIndex rebuildIndex();

    struct Saved { QString id, file, name; };
    /// With an id the file is overwritten (keeping its creation date); without one a
    /// new id is derived from the name. Throws nothing: an empty id in the result means failure.
    static Saved save(const QString &id, const QString &name, const QString &notes, const QJsonArray &items,
                      const QJsonObject &route, int vehicleType, QString *error = nullptr);
    /// The file's object (with "id"), or empty with `error` set.
    static QJsonObject load(const QString &id, QString *error = nullptr);
    static bool remove(const QString &id);
    /// Rename keeps the file (and its id); only the display name changes.
    static bool rename(const QString &id, const QString &newName, QString *error = nullptr);

    /// Filesystem-safe id from a mission name.
    static QString slugify(const QString &name);
};

} // namespace corv::mission
