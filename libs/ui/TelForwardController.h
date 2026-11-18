// TelForwardController.h - SETUP → COMMS → TELEMETRY FORWARD
// (telforward-manager.js and initTelForwardTab of TabController.js; QML
// singleton "TelForward").
//
// Live telemetry goes to any number of outputs at once, Mission Planner
// "MAVLink Mirror" style:
//   - serial port        (MAVLink passthrough or LTM)
//   - UDP client         (send to host:port, a second GCS)
//   - UDP server         (listen; forward to every peer that talks to us,
//                         forgotten after 60 s of silence)
// MAVLink outputs can have write access: what the mirror endpoint sends is
// injected into the vehicle link. LTM frames are built from the vehicle state
// (G and A at 5 Hz, S at 2 Hz, O at 0.5 Hz). The list refreshes once a second.
// The form persists under 'telfwd-settings'. GUI thread.
#pragma once

#include <QHash>
#include <QHostAddress>
#include <QObject>
#include <QPointer>
#include <QQmlEngine>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <deque>
#include <map>
#include <memory>

class QSerialPort;
class QUdpSocket;

namespace corv {

class LinkManager;
class VehicleHub;

class TelForwardController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(TelForward)
    QML_SINGLETON

    Q_PROPERTY(QVariantList outputs READ outputs NOTIFY outputsChanged)   // [{id, label, rw, connected, stats}]
    Q_PROPERTY(QVariantMap saved READ saved CONSTANT)
    Q_PROPERTY(QString addStatus READ addStatus NOTIFY addStatusChanged)
    Q_PROPERTY(bool addError READ addError NOTIFY addStatusChanged)
    Q_PROPERTY(QVariantMap current READ current NOTIFY currentChanged)    // CURRENT DATA, 4 Hz

public:
    TelForwardController(LinkManager *link, VehicleHub *hub, QObject *parent = nullptr);
    ~TelForwardController() override;

    static TelForwardController *s_instance;
    static TelForwardController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// {type: serial | udp-client | udp-server, protocol: mavlink | ltm, writeAccess, portPath, baudRate, host, port, listenPort}
    Q_INVOKABLE void addOutput(const QVariantMap &cfg);
    Q_INVOKABLE void removeOutput(int id);

    QVariantList outputs() const;
    QVariantMap saved() const;
    QString addStatus() const { return m_addStatus; }
    bool addError() const { return m_addError; }
    QVariantMap current() const { return m_current; }

signals:
    void outputsChanged();
    void addStatusChanged();
    void currentChanged();

private:
    struct Peer {
        QHostAddress address;
        quint16 port = 0;
        qint64 lastSeen = 0;
    };
    struct Output {
        int id = 0;
        QString type, protocol, label, error, portPath, host;
        bool writeAccess = false, connected = false;
        QHostAddress address;          // udp-client
        quint16 udpPort = 0, listenPort = 0;
        qint64 bytesSent = 0, msgCount = 0, bytesRx = 0;
        int msgPerSec = 0;
        std::deque<qint64> window;
        QSerialPort *serial = nullptr;
        QUdpSocket *udp = nullptr;
        QHash<QString, Peer> peers;   // udp-server
    };

    void onRawFrame(const QByteArray &frame);
    void writeOut(Output &o, const QByteArray &buf);
    void track(Output &o, qsizetype bytes);
    void received(Output &o, const QByteArray &data);
    void closeOutput(Output &o);
    void ensureTimers();
    void ltmTick(int which);
    void setAddStatus(const QString &text, bool error);
    void updateCurrent();

    LinkManager *m_link;
    VehicleHub *m_hub;
    std::map<int, std::unique_ptr<Output>> m_outputs;
    int m_nextId = 1;
    QTimer m_status, m_ltmFast, m_ltmSlow, m_ltmOrigin, m_currentTimer;
    QString m_addStatus;
    bool m_addError = false;
    QVariantMap m_current;
};

} // namespace corv
