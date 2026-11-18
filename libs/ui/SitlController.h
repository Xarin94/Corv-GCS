// SitlController.h - SETUP → TOOLS → SIMULATION (sitl-manager.js and
// initSimulationTab of TabController.js; QML singleton "Sitl").
//
// Downloads the pre-built ArduPilot SITL of a vehicle and version from
// firmware.ardupilot.org (Linux x86-64 only: on Windows it runs inside WSL),
// launches it with the vehicle's model, home and speed-up, the default
// parameters and frame models of resources/sitl-defaults in its working
// directory, and connects to its serial0 on TCP 5760. A ROV starts on the
// water at 0 m; the one with no GPS also switches the relative navigation on
// until STOP. A boat starts on Lake Garda, anything else on land, unless the
// operator typed a home of their own. GUI thread.
#pragma once

#include <QObject>
#include <QPointer>
#include <QProcess>
#include <QQmlEngine>
#include <QVariantList>
#include <functional>

class QNetworkAccessManager;
class QNetworkReply;

namespace corv {

class ConnectionManager;
class SysConfig;

class SitlController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Sitl)
    QML_SINGLETON
    Q_PROPERTY(QVariantList vehicles READ vehicles CONSTANT)   // [{value, label}]
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(bool running READ running NOTIFY statusChanged)
    Q_PROPERTY(bool busy READ busy NOTIFY statusChanged)       // downloading or starting
    Q_PROPERTY(QString downloadText READ downloadText NOTIFY statusChanged)

public:
    SitlController(ConnectionManager *conn, SysConfig *sys, QObject *parent = nullptr);
    ~SitlController() override;
    static SitlController *s_instance;
    static SitlController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// The home a vehicle starts from: {lat, lon} (suggestSitlHome)
    Q_INVOKABLE QVariantMap suggestHome(const QString &vehicle, double lat, double lon) const;
    Q_INVOKABLE void download(const QString &vehicle, const QString &version);
    Q_INVOKABLE void launch(const QString &vehicle, const QString &version, double homeLat, double homeLon, int speedup);
    Q_INVOKABLE void stop();
    /// SITL MANUAL CONNECTION
    Q_INVOKABLE void connectUdp(const QString &host, int port);

    QVariantList vehicles() const;
    QString status() const { return m_status; }
    bool running() const { return m_proc && m_proc->state() != QProcess::NotRunning; }
    bool busy() const { return m_busy; }
    QString downloadText() const { return m_downloadText; }

signals:
    void statusChanged();

private:
    QString sitlDir() const;
    QString binaryPath(const QString &vehicle, const QString &version) const;
    bool binaryUsable(const QString &vehicle, const QString &version) const;
    void syncAssets() const;
    void startDownload(const QString &vehicle, const QString &version, std::function<void(bool ok, const QString &error)> done);
    void start(const QString &vehicle, const QString &version, double homeLat, double homeLon, double homeAlt, int speedup);
    void killSitl();
    void setStatus(const QString &text);
    void beginRovSession(const QString &vehicle);
    void endRovSession();

    ConnectionManager *m_conn;
    SysConfig *m_sys;
    QNetworkAccessManager *m_net = nullptr;
    QPointer<QNetworkReply> m_reply;
    QPointer<QProcess> m_proc;
    QString m_status = QStringLiteral("Not running");
    QString m_downloadText = QStringLiteral("DOWNLOAD");
    bool m_busy = false;
    bool m_rovSession = false;
    QString m_rovSavedNav;
};

} // namespace corv
