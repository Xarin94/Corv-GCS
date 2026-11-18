// RtkController.h - SETUP → COMMS → RTK / GPS INJECT (rtk-manager.js and
// initRTKTab of TabController.js; QML singleton "Rtk").
//
// RTCM3 corrections go to the vehicle as MAVLink GPS_RTCM_DATA (233), Mission
// Planner style, from one of two sources at a time:
//   - a serial base station (u-blox F9P and the like)
//   - an NTRIP caster over TCP or TLS: sourcetable browsing, GGA upload for VRS
//     networks (vehicle or manual position), reconnection 5 s after a drop
//     unless the caster refused the request (auth, unknown mountpoint).
// Statistics (rate over 5 s, totals, the last 12 message types) refresh the
// page at most 4 times a second; the vehicle's own RTK state 4 times a second.
// The NTRIP settings persist under 'ntrip-settings' (no password), as in Electron.
// GUI thread.
#pragma once

#include "services/Rtcm.h"

#include <QElapsedTimer>
#include <QObject>
#include <QPointer>
#include <QQmlEngine>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <QtNumeric>
#include <deque>
#include <optional>

class QSerialPort;
class QSslSocket;

namespace corv {

class LinkManager;
class VehicleHub;

class RtkController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Rtk)
    QML_SINGLETON

    Q_PROPERTY(bool connected READ connected NOTIFY statusChanged)
    Q_PROPERTY(QString source READ source NOTIFY statusChanged)                  // "serial" | "ntrip" | ""
    Q_PROPERTY(QString connStatus READ connStatus NOTIFY statusChanged)
    Q_PROPERTY(bool connStatusActive READ connStatusActive NOTIFY statusChanged)
    Q_PROPERTY(QString streamStatus READ streamStatus NOTIFY statusChanged)
    Q_PROPERTY(bool streaming READ streaming NOTIFY statusChanged)
    Q_PROPERTY(QString activeSource READ activeSource NOTIFY statusChanged)
    Q_PROPERTY(int msgPerSec READ msgPerSec NOTIFY statusChanged)
    Q_PROPERTY(qint64 msgCount READ msgCount NOTIFY statusChanged)
    Q_PROPERTY(QString bytesText READ bytesText NOTIFY statusChanged)
    Q_PROPERTY(QString ggaSent READ ggaSent NOTIFY statusChanged)
    Q_PROPERTY(QString reconnects READ reconnects NOTIFY statusChanged)
    Q_PROPERTY(QVariantList msgTypes READ msgTypes NOTIFY statusChanged)          // [{id, name}]
    Q_PROPERTY(bool connecting READ connecting NOTIFY statusChanged)
    // The form as last saved (restored on the page)
    Q_PROPERTY(QVariantMap saved READ saved CONSTANT)
    // Sourcetable
    Q_PROPERTY(QVariantList mounts READ mounts NOTIFY mountsChanged)             // [{value, label}]
    Q_PROPERTY(bool listing READ listing NOTIFY mountsChanged)
    // The vehicle's RTK state (4 Hz)
    Q_PROPERTY(QString droneFix READ droneFix NOTIFY droneChanged)
    Q_PROPERTY(QString droneFixColor READ droneFixColor NOTIFY droneChanged)
    Q_PROPERTY(QString droneSat READ droneSat NOTIFY droneChanged)
    Q_PROPERTY(QString droneHdop READ droneHdop NOTIFY droneChanged)
    Q_PROPERTY(QString droneBaseline READ droneBaseline NOTIFY droneChanged)
    Q_PROPERTY(QString droneAccuracy READ droneAccuracy NOTIFY droneChanged)
    Q_PROPERTY(QString droneIar READ droneIar NOTIFY droneChanged)

public:
    RtkController(LinkManager *link, VehicleHub *hub, QObject *parent = nullptr);
    ~RtkController() override;

    static RtkController *s_instance;
    static RtkController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// Serial base station
    Q_INVOKABLE void connectSerial(const QString &port, int baud);
    Q_INVOKABLE void disconnectSerial();
    /// NTRIP: {host, port, mountpoint, username, password, tls, ggaMode, lat, lon, alt, ggaInterval}
    Q_INVOKABLE void ntripConnect(const QVariantMap &cfg, const QString &source = QString());
    Q_INVOKABLE void ntripDisconnect();
    Q_INVOKABLE void fetchSourcetable(const QVariantMap &cfg);
    /// The page remembers the chosen source with the NTRIP settings
    Q_INVOKABLE void setSourceChoice(const QString &source);

    bool connected() const { return m_connected; }
    QString source() const { return m_source; }
    QString connStatus() const { return m_connStatus; }
    bool connStatusActive() const { return m_connStatusActive; }
    QString streamStatus() const;
    bool streaming() const { return m_connected && m_msgPerSec > 0; }
    QString activeSource() const;
    int msgPerSec() const { return m_msgPerSec; }
    qint64 msgCount() const { return qint64(m_msgCount); }
    QString bytesText() const;
    QString ggaSent() const;
    QString reconnects() const;
    QVariantList msgTypes() const { return m_types; }
    bool connecting() const { return m_connecting; }
    QVariantMap saved() const;
    QVariantList mounts() const { return m_mounts; }
    bool listing() const { return m_listing; }
    QString droneFix() const { return m_droneFix; }
    QString droneFixColor() const { return m_droneFixColor; }
    QString droneSat() const { return m_droneSat; }
    QString droneHdop() const { return m_droneHdop; }
    QString droneBaseline() const { return m_droneBaseline; }
    QString droneAccuracy() const { return m_droneAccuracy; }
    QString droneIar() const { return m_droneIar; }

signals:
    void statusChanged();
    void mountsChanged();
    void droneChanged();

private:
    struct NtripConfig {
        QString host;
        int port = 2101;
        QString mountpoint, username, password;
        bool tls = false;
        QString ggaMode = QStringLiteral("off");   // off | vehicle | manual
        double lat = qQNaN(), lon = qQNaN(), alt = 0;
        int ggaInterval = 10;
    };

    void resetStats(const QString &source);
    void handleData(const QByteArray &data);
    void inject(const QByteArray &frame);
    void notify(bool force = false);
    void setConnStatus(const QString &text, bool active);
    void updateDrone();

    void ntripOpen();
    void ntripOnData();
    void ntripOnClosed();
    void ntripFail(const QString &message, bool fatal);
    void stopNtrip(bool userRequested);
    void sendGga();
    void closeSerial();
    void saveNtrip(const NtripConfig &c, const QString &source) const;

    LinkManager *m_link;
    VehicleHub *m_hub;

    // Statistics (rtkStats)
    bool m_connected = false;
    QString m_source;
    QString m_portPath;
    int m_baud = 0;
    qint64 m_bytes = 0;
    std::uint64_t m_msgCount = 0;
    int m_msgPerSec = 0;
    std::deque<qint64> m_window;
    QVariantList m_types;
    qint64 m_lastNotify = 0;
    rtcm::Parser m_parser;
    QString m_connStatus = QStringLiteral("Not connected");
    bool m_connStatusActive = false;
    bool m_connecting = false;

    // Serial base station
    QSerialPort *m_serial = nullptr;

    // NTRIP
    QSslSocket *m_ntrip = nullptr;
    std::optional<NtripConfig> m_ntripCfg;
    bool m_userDisconnect = false;
    bool m_firstAttempt = false;    // the connect the user asked for: its failure is reported
    bool m_headerParsed = false;
    QByteArray m_headerBuf;
    bool m_chunked = false;
    rtcm::Dechunker m_dechunk;
    int m_reconnects = 0;
    int m_ggaSentCount = 0;
    QTimer m_reconnectTimer;
    QTimer m_ggaTimer;
    QTimer m_idleTimer;             // no data for 20 s: a dead link

    // Sourcetable
    QPointer<QSslSocket> m_table;
    QByteArray m_tableBuf;
    bool m_tableDone = false;
    QVariantList m_mounts;
    bool m_listing = false;

    // Vehicle RTK state
    QTimer m_droneTimer;
    QString m_droneFix = QStringLiteral("---"), m_droneFixColor, m_droneSat = QStringLiteral("---"),
            m_droneHdop = QStringLiteral("---"), m_droneBaseline = QStringLiteral("---"),
            m_droneAccuracy = QStringLiteral("---"), m_droneIar = QStringLiteral("---");
};

} // namespace corv
