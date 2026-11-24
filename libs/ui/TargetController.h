// TargetController.h - GCS OPTIONS -> TARGET (QML singleton "Target"): a point of
// reference for the pilot, on the maps and in the 3D view, with the distance and
// the direction to it in the command bar. Nothing is sent to the vehicle.
// GUI thread.
#pragma once

#include "core/Geo.h"

#include <QObject>
#include <QQmlEngine>
#include <optional>

namespace corv {

class VehicleModel;

class TargetController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Target)
    QML_SINGLETON
    Q_PROPERTY(bool active READ active NOTIFY changed)
    Q_PROPERTY(double lat READ lat NOTIFY changed)
    Q_PROPERTY(double lon READ lon NOTIFY changed)
    /// m, and the direction from the nose, degrees clockwise.
    Q_PROPERTY(double distance READ distance NOTIFY moved)
    Q_PROPERTY(double relativeBearing READ relativeBearing NOTIFY moved)

public:
    TargetController(VehicleModel *vehicle, QObject *parent);
    static TargetController *s_instance;
    static TargetController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// Decimal degrees as typed; false (and a toast) when they are no position.
    Q_INVOKABLE bool set(const QString &lat, const QString &lon);
    Q_INVOKABLE void clear();

    bool active() const { return m_at.has_value(); }
    double lat() const { return m_at ? m_at->lat : 0; }
    double lon() const { return m_at ? m_at->lon : 0; }
    double distance() const;
    double relativeBearing() const;

signals:
    void changed();
    void moved();

private:
    void apply();

    VehicleModel *m_vehicle;
    std::optional<LatLon> m_at;
};

} // namespace corv
