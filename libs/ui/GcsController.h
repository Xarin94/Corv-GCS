// GcsController.h - the application core and its QML face ("Gcs" singleton).
//
// Owns the threads of the guide's architecture that exist so far: the link
// thread (LinkManager + VehicleHub), the log thread (LogReplay) and the
// GUI-side CommandSender, ConnectionManager and view models. QML calls the commands here; the view models read the
// vehicle through VehicleModel.
#pragma once

#include <QObject>
#include <QQmlEngine>
#include <QThread>
#include <QVariantList>
#include <memory>

namespace corv {

class AnnunciatorController;
class GeoZoneController;
class AdsbClient;
class RtkController;
class TelForwardController;
class UpdateController;
class LteController;
class JoystickController;
class SetupController;
class LidarController;
class RosController;
class SitlController;
class CalibrationController;
class EkfController;
class FftController;
class LogReplay;
class ReplayController;
class CommandSender;
class DialogHub;
class ParamStore;
class LogsController;
class ParamsController;
class ConnectionManager;
class LinkManager;
class MessageBus;
class PlanController;
class SysConfig;
class VehicleHub;
class VehicleModel;

class GcsController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Gcs)
    QML_SINGLETON
    Q_PROPERTY(QString version READ version CONSTANT)
    Q_PROPERTY(QString connectionState READ connectionState NOTIFY connectionStateChanged)
    Q_PROPERTY(QString lastError READ lastError NOTIFY connectionStateChanged)
    Q_PROPERTY(bool gcsMuted READ gcsMuted WRITE setGcsMuted NOTIFY gcsMutedChanged)
    Q_PROPERTY(QString messageLog READ messageLog NOTIFY messageLogChanged)

public:
    // No default constructor: QML must take the instance from create(), not build a second one
    explicit GcsController(QObject *parent);
    ~GcsController() override;

    static GcsController *s_instance;
    static GcsController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    LinkManager *link() const { return m_link; }
    CommandSender *commands() const { return m_cmd; }

    QString version() const;
    QString connectionState() const { return m_state; }
    QString lastError() const { return m_error; }
    bool gcsMuted() const;
    void setGcsMuted(bool m);

    /// [{path, label}] of the serial ports.
    Q_INVOKABLE QVariantList serialPorts() const;
    /// type: "TCP", "UDP", "SERIAL".
    Q_INVOKABLE void connectLink(const QString &type, const QString &address, int portOrBaud);
    Q_INVOKABLE void disconnectLink();
    /// SETUP -> CONNECTION: type as the old select values (mavlink-serial, mavlink-udp, mavlink-tcp, msp-serial, msp-tcp).
    Q_INVOKABLE void connectType(const QString &type, const QString &address, int portOrBaud);
    /// Persistent settings (settings.json, the old localStorage)
    Q_INVOKABLE QString setting(const QString &key, const QString &fallback) const;
    Q_INVOKABLE void setSetting(const QString &key, const QString &value);
    Q_INVOKABLE void setHomeHere();
    Q_INVOKABLE void rebootAutopilot();
    Q_INVOKABLE void setMissionSpeed(double speed);
    QString messageLog() const { return m_log.join(QLatin1Char('\n')); }

    Q_INVOKABLE void arm(bool force = false);
    Q_INVOKABLE void disarm(bool force = false);
    Q_INVOKABLE void setMode(const QString &mode);
    Q_INVOKABLE void takeoff(double altitude);
    Q_INVOKABLE void land();
    Q_INVOKABLE void returnToLaunch();

signals:
    void connectionStateChanged();
    void gcsMutedChanged();
    void messageLogChanged();
    /// A line for the HUD message list: level info / warning / error.
    void hudMessage(const QString &text, const QString &level);

private:
    void startLink();
    void createViewModels();
    void wireViewModels();
    void wireLink();

    std::unique_ptr<MessageBus> m_bus;
    QThread m_linkThread;
    LinkManager *m_link = nullptr;
    VehicleHub *m_hub = nullptr;
    CommandSender *m_cmd = nullptr;
    ConnectionManager *m_conn = nullptr;
    VehicleModel *m_vehicle = nullptr;
    SysConfig *m_sysConfig = nullptr;
    PlanController *m_plan = nullptr;
    DialogHub *m_dialogs = nullptr;
    ParamStore *m_params = nullptr;
    ParamsController *m_paramsUi = nullptr;
    LogsController *m_logs = nullptr;
    CalibrationController *m_calibration = nullptr;
    EkfController *m_ekf = nullptr;
    FftController *m_fft = nullptr;
    AnnunciatorController *m_annun = nullptr;
    GeoZoneController *m_geo = nullptr;
    AdsbClient *m_adsb = nullptr;
    RtkController *m_rtk = nullptr;
    TelForwardController *m_telForward = nullptr;
    UpdateController *m_updates = nullptr;
    LteController *m_lte = nullptr;
    JoystickController *m_joystick = nullptr;
    SetupController *m_setup = nullptr;
    LidarController *m_lidar = nullptr;
    RosController *m_ros = nullptr;
    SitlController *m_sitl = nullptr;
    QThread m_logThread;
    LogReplay *m_replayEngine = nullptr;
    ReplayController *m_replay = nullptr;
    QString m_state = QStringLiteral("DISCONNECTED");
    QString m_error;
    QStringList m_log;
};

} // namespace corv
