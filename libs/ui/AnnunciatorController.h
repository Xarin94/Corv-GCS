// AnnunciatorController.h - vehicle health annunciators, the CAS strip
// ("Annunciators" QML singleton): the port of js/ui/AnnunciatorPanel.js.
//
// Grades the conditions Mission Planner flags on its HUD and shows them as
// flashing glyphs in the column inboard of the airspeed tape:
//
//   WARNING (red, from the top)   faults and interference that threaten the
//                                 flight: failsafe, EKF, vibration, IMU /
//                                 compass / baro / GPS health, GPS jamming /
//                                 spoofing / glitch, RC loss, motor, battery,
//                                 fence, link loss.
//   CAUTION (amber, from the bottom, above the mini-map) degraded but not yet
//                                 dangerous: calibrations, pre-arm, HDOP, 2D
//                                 fix, terrain, logging, LiDAR, optical flow,
//                                 proximity, weak link, low battery.
//
// Sources: SYS_STATUS sensor bits, HEARTBEAT system_status, EKF_STATUS_REPORT,
// VIBRATION, GPS_RAW_INT, GNSS_INTEGRITY, MAG_CAL_*, RADIO_STATUS (all read from
// the vehicle snapshot) plus STATUSTEXT matches, held 10 s because they are
// events. In the demo flight a random annunciator lights for a few seconds at a
// time. A click silences one until the next connection; silencing a warning
// also silences its caution twin, not the other way round. 10 Hz, GUI thread.
#pragma once

#include "gpu/FrameTicker.h"

#include <QHash>
#include <QObject>
#include <QQmlEngine>
#include <QSet>
#include <QTimer>
#include <QVariantList>

namespace corv {

class VehicleHub;

class AnnunciatorController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Annunciators)
    QML_SINGLETON
    Q_PROPERTY(QVariantList warnings READ warnings NOTIFY listChanged)
    Q_PROPERTY(QVariantList cautions READ cautions NOTIFY listChanged)

public:
    AnnunciatorController(VehicleHub *hub, QObject *parent);
    static AnnunciatorController *s_instance;
    static AnnunciatorController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    QVariantList warnings() const { return m_warnings; }
    QVariantList cautions() const { return m_cautions; }
    /// The ids lit now, warnings then cautions (tests).
    QStringList activeIds() const { return m_activeIds; }

    /// Click: off until the next connection.
    Q_INVOKABLE void silence(const QString &id);
    /// The parts of a glyph for SvgIcon (24 × 24).
    Q_INVOKABLE QVariantList glyph(const QString &icon) const;

    void update();

signals:
    void listChanged();

private:
    QSet<QString> evaluate(std::int64_t now);
    void stepDemo(std::int64_t now);
    void onStatusText(const QString &text);

    VehicleHub *m_hub;
    PacedTimer m_timer;   // on the 3D view's frames while it paces them (RENDER FPS CAP)
    QHash<QString, std::int64_t> m_timed;   // STATUSTEXT holds: id -> expiry
    QSet<QString> m_silenced;
    std::int64_t m_lastClipSum = -1, m_lastClipChange = 0;
    QString m_lastKey;
    QSet<QString> m_loggedOn;
    QString m_demoId;
    std::int64_t m_demoUntil = 0, m_demoNextAt = 0;
    QVariantList m_warnings, m_cautions;
    QStringList m_activeIds;
};

} // namespace corv
