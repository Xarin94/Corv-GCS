// DataRoot.h - the one folder everything the GCS writes lives under (port of the
// root logic of mission-store.js).
//
//   <root>/index.json  missions/  logs/  debug/  terrain/  celldb/  geozones/
//   <root>/lidar/      tiles/ (new: SQLite tile store)  settings.json (new)
//
// The root is <folder of the executable>/data when that is writable, which keeps
// an installation portable; otherwise the per-user folder Electron used, so the
// Electron and the Qt versions share one data root on the same machine:
//   Windows  %APPDATA%\CORV GCS\data
//   macOS    ~/Library/Application Support/CORV GCS/data
//   Linux    $XDG_CONFIG_HOME/CORV GCS/data (~/.config/CORV GCS/data)
// CORV_DATA_ROOT in the environment overrides both (tests, development).
#pragma once

#include <QString>

namespace corv {

class DataRoot
{
public:
    /// The root, resolved once and created if missing.
    static QString root();
    /// True when the install folder was not writable and the per-user folder is used.
    static bool isFallback();
    /// <root>/<name>, created if missing.
    static QString dir(const QString &name);

    static QString missionsDir() { return dir(QStringLiteral("missions")); }
    static QString logsDir() { return dir(QStringLiteral("logs")); }
    static QString debugDir() { return dir(QStringLiteral("debug")); }
    static QString terrainDir() { return dir(QStringLiteral("terrain")); }
    static QString tilesDir() { return dir(QStringLiteral("tiles")); }
    static QString geozonesDir() { return dir(QStringLiteral("geozones")); }
    static QString celldbDir() { return dir(QStringLiteral("celldb")); }
    static QString lidarDir() { return dir(QStringLiteral("lidar")); }

    /// Folder of the installation (read-only inputs: topo/, topography/, models/).
    static QString installDir();

    /// The per-user folder Electron's app.getPath('userData') pointed to.
    static QString electronUserDataDir();

    /// Forget the resolved root (tests only).
    static void resetForTests();
};

} // namespace corv
