// SetupController.h - the parameter-driven SETUP pages (TabController.js;
// QML singleton "SetupCfg"): FLIGHT MODES, FAILSAFE, PID TUNING, EXT TUNING,
// RADIO CAL, VIBRATIONS, SERVO / RELAY.
//
// The pages are layout only. They read their parameters with targeted
// PARAM_REQUEST_READs through ParamStore (a few dozen values, not the
// thousand-long list), show what is in ParamStore (paramRev changes with it)
// and write a set of values in one go, the result in a toast as Electron's
// alert did. RADIO CAL keeps the min / max / trim seen while calibrating and
// writes RCn_MIN / _MAX / _TRIM; VIBRATIONS and SERVO / RELAY show the live
// VIBRATION and SERVO_OUTPUT_RAW at 4 Hz while their page is open.
// GUI thread.
#pragma once

#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <array>

namespace corv {

class CommandSender;
class ParamStore;
class VehicleHub;

class SetupController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(SetupCfg)
    QML_SINGLETON

    Q_PROPERTY(int paramRev READ paramRev NOTIFY paramsChanged)
    Q_PROPERTY(QString page READ page WRITE setPage NOTIFY pageChanged)      // the SETUP entry on screen
    Q_PROPERTY(QVariantMap status READ status NOTIFY statusChanged)          // per page: {text, ok}
    Q_PROPERTY(QVariantList flightModeOptions READ flightModeOptions NOTIFY vehicleChanged)
    Q_PROPERTY(QString vehicleClass READ vehicleClass NOTIFY vehicleChanged) // Copter | Plane | Rover | Sub | "" (unknown)
    // RADIO CAL
    Q_PROPERTY(QVariantList rc READ rc NOTIFY liveChanged)                   // [{value, min, max}] x 16
    Q_PROPERTY(bool rcCalibrating READ rcCalibrating NOTIFY rcCalChanged)
    Q_PROPERTY(QString rcStatus READ rcStatus NOTIFY rcCalChanged)
    // VIBRATIONS
    Q_PROPERTY(QVariantMap vib READ vib NOTIFY liveChanged)                  // x, y, z, clip0..2
    Q_PROPERTY(QVariantList vibHistory READ vibHistory NOTIFY liveChanged)   // [[x, y, z]...]
    // SERVO / RELAY
    Q_PROPERTY(QVariantList servos READ servos NOTIFY liveChanged)
    Q_PROPERTY(QVariantList relays READ relays NOTIFY relaysChanged)

public:
    SetupController(ParamStore *params, CommandSender *cmd, VehicleHub *hub, QObject *parent = nullptr);

    static SetupController *s_instance;
    static SetupController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// The value in ParamStore, or undefined.
    Q_INVOKABLE QVariant param(const QString &name) const;
    /// Targeted reads; `page` gets "Reading N/M..." then the outcome.
    Q_INVOKABLE void readParams(const QStringList &names, const QString &page);
    /// Write {name: value}; `doneText` has %1 for the count written ("Written %1 failsafe parameters").
    Q_INVOKABLE void writeParams(const QVariantMap &values, const QString &page, const QString &doneText);

    // RADIO CAL
    Q_INVOKABLE void rcCalToggle();
    Q_INVOKABLE void rcCalSave();
    Q_INVOKABLE void rcCalReset();
    // SERVO / RELAY
    Q_INVOKABLE void toggleRelay(int relay);
    Q_INVOKABLE void servoTest(int servo, int pwm);

    int paramRev() const { return m_paramRev; }
    QString page() const { return m_page; }
    void setPage(const QString &p);
    QVariantMap status() const { return m_status; }
    QVariantList flightModeOptions() const;
    QString vehicleClass() const;
    QVariantList rc() const;
    bool rcCalibrating() const { return m_rcCalibrating; }
    QString rcStatus() const { return m_rcStatus; }
    QVariantMap vib() const { return m_vib; }
    QVariantList vibHistory() const { return m_vibHistory; }
    QVariantList servos() const { return m_servos; }
    QVariantList relays() const;

signals:
    void paramsChanged();
    void pageChanged();
    void statusChanged();
    void vehicleChanged();
    void liveChanged();
    void rcCalChanged();
    void relaysChanged();

private:
    void setStatus(const QString &page, const QString &text, bool ok);
    void tick();

    ParamStore *m_params;
    CommandSender *m_cmd;
    VehicleHub *m_hub;
    int m_paramRev = 0;
    bool m_revPending = false;
    QString m_page;
    QVariantMap m_status;
    int m_vehicleType = -1;
    QTimer m_live;

    std::array<int, 16> m_rcValue{}, m_rcMin{}, m_rcMax{}, m_rcTrim{};
    bool m_rcCalibrating = false;
    QString m_rcStatus = QStringLiteral("Move all sticks to extremes during calibration");
    QVariantMap m_vib;
    QVariantList m_vibHistory;
    QVariantList m_servos;
    std::array<bool, 4> m_relay{};
};

} // namespace corv
