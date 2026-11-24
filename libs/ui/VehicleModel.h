// VehicleModel.h - the vehicle as QML sees it ("Vehicle" singleton).
//
// A 30 Hz timer takes the latest VehicleState snapshot and notifies once
// (updated) when something changed; QML binds to the grouped properties.
// No signal per field per message: the link may bring hundreds a second.
// Paced (RENDER FPS CAP), the timer fires on the 3D view's frames.
#pragma once

#include "gpu/FrameTicker.h"
#include "vehicle/VehicleState.h"

#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QVariantList>
#include <cmath>
#include <limits>
#include <memory>

namespace corv {

class VehicleHub;

class VehicleModel : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Vehicle)
    QML_SINGLETON

    // Link
    Q_PROPERTY(bool connected READ connected NOTIFY updated)
    Q_PROPERTY(QString connectionType READ connectionType NOTIFY updated)
    Q_PROPERTY(QString linkType READ linkType NOTIFY updated)
    Q_PROPERTY(double linkKbps READ linkKbps NOTIFY updated)
    Q_PROPERTY(bool heartbeatAlive READ heartbeatAlive NOTIFY updated)
    Q_PROPERTY(bool demo READ demo NOTIFY updated)
    Q_PROPERTY(QString mode READ mode NOTIFY updated)
    // Identity
    Q_PROPERTY(bool armed READ armed NOTIFY updated)
    Q_PROPERTY(QString flightMode READ flightMode NOTIFY updated)
    Q_PROPERTY(QString modeClass READ modeClass NOTIFY updated)
    Q_PROPERTY(QStringList availableModes READ availableModes NOTIFY updated)
    Q_PROPERTY(QString vehicleTypeName READ vehicleTypeName NOTIFY updated)
    Q_PROPERTY(int systemStatus READ systemStatus NOTIFY updated)
    // Attitude and position
    Q_PROPERTY(double rollDeg READ rollDeg NOTIFY updated)
    Q_PROPERTY(double pitchDeg READ pitchDeg NOTIFY updated)
    Q_PROPERTY(double headingDeg READ headingDeg NOTIFY updated)
    Q_PROPERTY(double lat READ lat NOTIFY updated)
    Q_PROPERTY(double lon READ lon NOTIFY updated)
    Q_PROPERTY(double altMsl READ altMsl NOTIFY updated)
    Q_PROPERTY(double altRel READ altRel NOTIFY updated)
    Q_PROPERTY(double airspeed READ airspeed NOTIFY updated)
    Q_PROPERTY(double groundspeed READ groundspeed NOTIFY updated)
    Q_PROPERTY(double climb READ climb NOTIFY updated)
    Q_PROPERTY(double homeDistance READ homeDistance NOTIFY updated)
    Q_PROPERTY(bool homeSet READ homeSet NOTIFY updated)
    // Systems
    Q_PROPERTY(double batteryVoltage READ batteryVoltage NOTIFY updated)
    Q_PROPERTY(double batteryCurrent READ batteryCurrent NOTIFY updated)
    Q_PROPERTY(int batteryRemaining READ batteryRemaining NOTIFY updated)
    Q_PROPERTY(int gpsFix READ gpsFix NOTIFY updated)
    Q_PROPERTY(QString gpsFixName READ gpsFixName NOTIFY updated)
    Q_PROPERTY(int gpsNumSat READ gpsNumSat NOTIFY updated)
    Q_PROPERTY(double gpsHdop READ gpsHdop NOTIFY updated)
    Q_PROPERTY(bool gpsStale READ gpsStale NOTIFY updated)
    Q_PROPERTY(double linkQuality READ linkQuality NOTIFY updated)
    Q_PROPERTY(QString flightTime READ flightTime NOTIFY updated)
    Q_PROPERTY(int rssi READ rssi NOTIFY updated)
    Q_PROPERTY(int remRssi READ remRssi NOTIFY updated)
    /// The RSSI in dBm (the SiK scale, which the cellular module uses too); NaN without a reading.
    Q_PROPERTY(double rssiDbm READ rssiDbm NOTIFY updated)
    /// Signal bars lit, 0 to 5; -1 without a reading.
    Q_PROPERTY(int signalBars READ signalBars NOTIFY updated)
    Q_PROPERTY(bool sensorsOk READ sensorsOk NOTIFY updated)
    /// Pre-arm checks: 0 unknown, 1 pass (or armed), 2 fail; what the vehicle last said about them.
    Q_PROPERTY(int preArm READ preArm NOTIFY updated)
    Q_PROPERTY(QString preArmText READ preArmText NOTIFY updated)
    Q_PROPERTY(QString firmwareVersion READ firmwareVersion NOTIFY updated)
    Q_PROPERTY(double gLoad READ gLoad NOTIFY updated)
    Q_PROPERTY(double terrainAgl READ terrainAgl NOTIFY updated)
    Q_PROPERTY(double rangefinder READ rangefinder NOTIFY updated)
    Q_PROPERTY(int missionSeq READ missionSeq NOTIFY updated)
    Q_PROPERTY(int missionCount READ missionCount NOTIFY updated)
    Q_PROPERTY(int systemId READ systemId NOTIFY updated)
    Q_PROPERTY(QString autopilotName READ autopilotName NOTIFY updated)
    // The other values a HUD data cell can show (SYS CONFIG → HUD DATA FIELDS)
    Q_PROPERTY(double vibX READ vibX NOTIFY updated)
    Q_PROPERTY(double vibY READ vibY NOTIFY updated)
    Q_PROPERTY(double vibZ READ vibZ NOTIFY updated)
    Q_PROPERTY(double aoaDeg READ aoaDeg NOTIFY updated)
    Q_PROPERTY(double ssaDeg READ ssaDeg NOTIFY updated)
    Q_PROPERTY(double rtkBaseline READ rtkBaseline NOTIFY updated)
    Q_PROPERTY(double rtkAccuracy READ rtkAccuracy NOTIFY updated)
    // ADS-B traffic table: the 4 nearest, {callsign, alt, dist} (getNearestTraffic)
    Q_PROPERTY(QVariantList nearestTraffic READ nearestTraffic NOTIFY updated)
    Q_PROPERTY(double altOffset READ altOffset NOTIFY updated)
    /// Relative navigation: the velocity (NED, m/s) is shown instead of a synthetic latitude and longitude
    Q_PROPERTY(bool relativeMode READ relativeMode NOTIFY updated)
    Q_PROPERTY(double vn READ vn NOTIFY updated)
    Q_PROPERTY(double ve READ ve NOTIFY updated)
    Q_PROPERTY(double vd READ vd NOTIFY updated)

public:
    explicit VehicleModel(VehicleHub *hub, QObject *parent = nullptr);

    static VehicleModel *s_instance;
    static VehicleModel *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    bool connected() const { return m_s->connected; }
    QString connectionType() const { return m_s->connectionType; }
    QString linkType() const { return m_s->linkType; }
    double linkKbps() const { return m_s->linkKbps; }
    bool heartbeatAlive() const;
    bool demo() const { return m_s->isDemo(); }
    QString mode() const { return m_s->mode; }
    bool armed() const { return m_s->armed; }
    QString flightMode() const { return m_s->flightMode; }
    QString modeClass() const;
    QStringList availableModes() const;
    QString vehicleTypeName() const;
    int systemStatus() const { return m_s->systemStatus; }
    double rollDeg() const;
    double pitchDeg() const;
    double headingDeg() const;
    double lat() const { return m_s->lat; }
    double lon() const { return m_s->lon; }
    double altMsl() const { return m_s->rawAlt; }
    double altRel() const { return m_s->homeAlt ? m_s->rawAlt - *m_s->homeAlt : m_s->rawAlt; }
    double airspeed() const { return m_s->as; }
    double groundspeed() const { return m_s->gs; }
    double climb() const { return m_s->vs; }
    double homeDistance() const;
    bool homeSet() const { return m_s->homeLat.has_value(); }
    double batteryVoltage() const { return m_s->batteryVoltage; }
    double batteryCurrent() const { return m_s->batteryCurrent; }
    int batteryRemaining() const { return m_s->batteryRemaining; }
    int gpsFix() const { return m_s->gpsFix; }
    QString gpsFixName() const;
    int gpsNumSat() const { return m_s->gpsNumSat; }
    double gpsHdop() const { return m_s->gpsHdop; }
    bool gpsStale() const;
    double linkQuality() const { return m_s->linkQuality; }
    QString flightTime() const;

    int rssi() const { return m_s->rssi ? *m_s->rssi : -1; }
    int remRssi() const { return m_s->remRssi ? *m_s->remRssi : -1; }
    double rssiDbm() const { return m_s->rssi ? *m_s->rssi / 1.9 - 127 : std::numeric_limits<double>::quiet_NaN(); }
    int signalBars() const;
    int preArm() const { return heartbeatAlive() || m_s->armed ? int(m_s->preArm()) : 0; }
    QString preArmText() const { return m_s->preArmText; }
    QString firmwareVersion() const { return m_s->firmwareVersion; }
    /// Present and enabled sensors all healthy (the pre-arm dot)
    bool sensorsOk() const { return (m_s->sensorsPresent & m_s->sensorsEnabled & ~m_s->sensorsHealth) == 0; }
    double gLoad() const { return m_s->gLoad(); }
    double terrainAgl() const;
    double rangefinder() const { return m_s->rangefinderDist ? *m_s->rangefinderDist : std::numeric_limits<double>::quiet_NaN(); }
    int missionSeq() const { return m_s->missionCurrentSeq; }
    int missionCount() const { return m_s->missionCount; }
    int systemId() const { return m_s->systemId; }
    QString autopilotName() const;
    QVariantList nearestTraffic() const;
    double vibX() const { return m_s->vibX; }
    double vibY() const { return m_s->vibY; }
    double vibZ() const { return m_s->vibZ; }
    double aoaDeg() const { return m_s->aoa * 180.0 / M_PI; }
    double ssaDeg() const { return m_s->ssa * 180.0 / M_PI; }
    double rtkBaseline() const { return m_s->rtkBaseline; }
    double rtkAccuracy() const { return m_s->rtkAccuracy; }
    double altOffset() const { return m_s->offsetAlt; }
    bool relativeMode() const { return m_s->relativeMode; }
    double vn() const { return m_s->vn; }
    double ve() const { return m_s->ve; }
    double vd() const { return m_s->vd; }

    std::shared_ptr<const VehicleState> state() const { return m_s; }

signals:
    void updated();

private:
    void poll();

    VehicleHub *m_hub;
    std::shared_ptr<const VehicleState> m_s;
    PacedTimer m_timer;
    qint64 m_armedSince = 0;
    qint64 m_flightMs = 0;
};

} // namespace corv
