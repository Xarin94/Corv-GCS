// MountPreviewItem.h - schematic 3D view of a sensor's mounting (port of
// js/ui/MountPreview.js), drawn in the LIDAR and ROS mount settings.
//
// The vehicle as one thin triangle with a small fin, fixed, seen from behind
// and above in a near-isometric view; the sensor at its lever arm from the
// IMU, with its own axes (REP-103: x red forward, y green left, z blue up);
// and the zone it collects from, translucent, turned by the mount - a band, a
// cone, a fan, or the directions the points actually came from.
//
// The orientation is the one the points are georeferenced with: the sensor
// frame (FLU) is flipped to FRD and turned by the mount's ZYX Euler matrix,
// exactly as LidarCore and RosPoints do, so what this shows is what the map
// will get. The zone's radius is schematic, not to scale. Orthographic: the
// view (x forward, y left, z up, metres) is projected along a fixed direction
// from behind-right-above. Painted on demand (an input changed, a resize).
#pragma once

#include <QColor>
#include <QQuickPaintedItem>
#include <QVariantList>
#include <QVariantMap>

namespace corv {

class MountPreviewItem : public QQuickPaintedItem
{
    Q_OBJECT
    QML_NAMED_ELEMENT(MountPreview)
    /// roll, pitch, yaw, degrees (sensor relative to the body)
    Q_PROPERTY(QVariantList mount READ mount WRITE setMount NOTIFY changed)
    /// IMU -> sensor, m, FRD
    Q_PROPERTY(QVariantList lever READ lever WRITE setLever NOTIFY changed)
    /// {type: band, elMin, elMax} | {type: cone, half} | {type: fan, min, max} | {type: dirs, az, el, bins} (degrees), or empty
    Q_PROPERTY(QVariantMap zone READ zone WRITE setZone NOTIFY changed)
    Q_PROPERTY(QString caption READ caption WRITE setCaption NOTIFY changed)
    Q_PROPERTY(QColor ink READ ink WRITE setInk NOTIFY changed)

public:
    explicit MountPreviewItem(QQuickItem *parent = nullptr);
    void paint(QPainter *p) override;

    QVariantList mount() const { return m_mount; }
    void setMount(const QVariantList &v) { m_mount = v; emit changed(); update(); }
    QVariantList lever() const { return m_lever; }
    void setLever(const QVariantList &v) { m_lever = v; emit changed(); update(); }
    QVariantMap zone() const { return m_zone; }
    void setZone(const QVariantMap &v) { m_zone = v; emit changed(); update(); }
    QString caption() const { return m_caption; }
    void setCaption(const QString &v) { m_caption = v; emit changed(); update(); }
    QColor ink() const { return m_ink; }
    void setInk(const QColor &c) { m_ink = c; emit changed(); update(); }

signals:
    void changed();

private:
    QVariantList m_mount{0, 0, 0}, m_lever{0, 0, 0};
    QVariantMap m_zone;
    QString m_caption;
    QColor m_ink{0xcf, 0xd8, 0xdd};
};

} // namespace corv
