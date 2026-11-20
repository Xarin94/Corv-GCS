// GeoZoneController.h - UAS geographical zones on the flight screen, the maps
// and the terrain (js/safety/GeoZones.js and js/ui/GeoZonePanel.js), QML
// singleton "GeoZones".
//
// GCS OPTIONS / SYS CONFIG -> RESTRICTED ZONES (off by default, geozones-enabled).
// On, the zones within 50 km of the connected vehicle and of every point of
// the mission are asked of GeoZoneStore; asked again when the vehicle has moved
// 5 km, when the mission changes, when the store fetched new tiles, and every
// 10 min so tiles past their age are fetched again. No position is taken from
// the demo flight.
//
// For a flying vehicle (copter, plane) with a position, four times a second the
// zones and the region's general limit (120 m in Europe, 400 ft in the US) are
// set against its height above the ground: alerts for the strip beside the
// altitude tape (red NO-FLY, MAX n m over a ceiling or the general limit;
// amber ZONE, MAX n m under a ceiling; a click silences one until the next
// connection), overHeight (TERR ALT flashes), and on arming inside a zone the
// arming alarm, a banner for 10 s and a HUD message.
#pragma once

#include "scene/GeoZone.h"

#include <QObject>
#include <QQmlEngine>
#include <QSet>
#include <QTimer>
#include <QUrl>
#include <QVariantList>

namespace corv {

class GeoZoneStore;
class PlanController;
class VehicleHub;

class GeoZoneController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(GeoZones)
    QML_SINGLETON
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(int inReach READ inReach NOTIFY dataChanged)
    Q_PROPERTY(QString sourcesText READ sourcesText NOTIFY dataChanged)
    Q_PROPERTY(QVariantList imports READ imports NOTIFY dataChanged)
    Q_PROPERTY(int pending READ pending NOTIFY dataChanged)
    Q_PROPERTY(int version READ version NOTIFY dataChanged)
    Q_PROPERTY(QVariantList alerts READ alerts NOTIFY statusChanged)
    Q_PROPERTY(bool overHeight READ overHeight NOTIFY statusChanged)
    Q_PROPERTY(QVariantMap armAlert READ armAlert NOTIFY armAlertChanged)

public:
    GeoZoneController(VehicleHub *hub, PlanController *plan, QObject *parent);
    ~GeoZoneController() override;
    static GeoZoneController *s_instance;
    static GeoZoneController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// The zones the maps draw (null while off) and a version that changes with them. GUI thread.
    static GeoZonesPtr shownZones();

    bool enabled() const { return m_enabled; }
    void setEnabled(bool on);
    int inReach() const { return m_zones ? int(m_zones->size()) : 0; }
    QString sourcesText() const { return m_sourcesText; }
    QVariantList imports() const;
    int pending() const { return m_pending; }
    int version() const { return m_version; }
    QVariantList alerts() const { return m_alerts; }
    bool overHeight() const { return m_overHeight; }
    QVariantMap armAlert() const { return m_armAlert; }

    Q_INVOKABLE void importFile(const QUrl &file);
    Q_INVOKABLE void removeImport(const QString &file);
    /// Forget the downloaded zones and fetch them again (imports are kept).
    Q_INVOKABLE void refresh();
    Q_INVOKABLE void silence(const QString &id);
    Q_INVOKABLE void closeArmAlert();

signals:
    void enabledChanged();
    void dataChanged();
    void statusChanged();
    void armAlertChanged();
    void hudMessage(const QString &text, const QString &level);
    void toast(const QString &text);

private:
    void query();
    void evaluate();
    void notify();
    bool vehiclePosition(double *lat, double *lon) const;

    VehicleHub *m_hub;
    PlanController *m_plan;
    GeoZoneStore *m_store;
    bool m_enabled = false;
    GeoZonesPtr m_zones;
    QString m_sourcesText;
    int m_pending = 0;
    int m_version = 0;
    QTimer m_evalTimer, m_missionTimer, m_armTimer;
    qint64 m_lastQueryAt = 0;
    bool m_haveCentre = false;
    double m_centreLat = 0, m_centreLon = 0;
    QVariantList m_alerts;
    bool m_overHeight = false;
    QVariantMap m_armAlert;
    int m_wasArmed = -1;          // -1 until the first evaluation of a link
    QSet<QString> m_silenced;
};

} // namespace corv
