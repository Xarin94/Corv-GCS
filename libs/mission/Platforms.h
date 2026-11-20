// Platforms.h - which flight stack the GCS is talking to (port of js/mission/Platforms.js).
//
// The route editor was written against ArduPilot: every segment and every
// action compiles to a MAV_CMD. INAV is a flat list of a few dozen waypoints
// over MSP; Betaflight has no navigation stack at all. The differences live
// here as one table: what a platform supports, which link it wants and — for
// everything it cannot do — why. The UI puts that "why" in the tooltip of
// every control it greys out.
//
// The active platform is a GCS-wide setting (SYS CONFIG → FLIGHT STACK), kept
// under the old localStorage key "corv.platform".
//
// Keys of unsupportedReason():
//   segment    <type>                                    a tool in the palette
//   action     <type>                                    an entry of the ACTIONS menu
//   routeParam <key> | <key>.<option>                    a route settings field
//   segField   <segType>.<key> | <segType>.<key>.<option> an inspector field
//   setup      <section>                                 a SETUP page entry
#pragma once

#include <QHash>
#include <QString>
#include <QStringList>

namespace corv::mission {

struct Platform {
    QString id, label, shortName, blurb, transport;   // transport: mavlink | msp
    bool missions = true;
    int itemLimit = 0;
    QString connType;                                 // CONNECTION page pre-set
    int connBaud = 57600;
    QString hint;
    QString noMissionReason;
    QString configurator;
    QHash<QString, QHash<QString, QString>> unsupported;   // kind → key → reason
};

const Platform &platform(const QString &id);
QString activePlatformId();
/// Switch stack; false when the id is unknown or already active.
bool setActivePlatform(const QString &id);
bool missionsSupported(const QString &id = activePlatformId());
bool usesMspMissions(const QString &id = activePlatformId());
/// Why `key` cannot be used on the platform, or empty when it can.
QString unsupportedReason(const QString &kind, const QString &key, const QString &id = activePlatformId());

} // namespace corv::mission
