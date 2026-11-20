// LinkManager.h - the one owner of the vehicle link (port of main-mavlink.js).
//
// Lives on its own thread ("Thread link" in the implementation guide): only
// it touches sockets and serial ports. It opens one transport at a time,
// parses MAVLink (or MSP), sends the 1 Hz GCS heartbeat (sysid 255,
// compid 190, MAV_TYPE_GCS), records the .tlog (auto start on connect, stop on
// disconnect, never during a replay), counts the link rate once a second and
// publishes every message on the MessageBus and through message().
//
// Every public method is safe to call from any thread: the call is posted to
// the link thread.
#pragma once

#include "link/LinkTrace.h"
#include "link/MspSession.h"
#include "link/MavMessage.h"
#include "link/TlogWriter.h"

#include <QObject>
#include <QPointer>
#include <QTimer>
#include <QVariantMap>
#include <atomic>
#include <memory>

namespace corv {

class MavlinkParser;
class MessageBus;
class Transport;

class LinkManager : public QObject
{
    Q_OBJECT
public:
    explicit LinkManager(MessageBus *bus, QObject *parent = nullptr);
    ~LinkManager() override;

    /// Connection types, as ConnectionManager.js named them.
    static constexpr const char *kMavlinkSerial = "mavlink-serial";
    static constexpr const char *kMavlinkUdp = "mavlink-udp";
    static constexpr const char *kMavlinkTcp = "mavlink-tcp";
    static constexpr const char *kMavlinkLte = "mavlink-lte";
    static constexpr const char *kMspSerial = "msp-serial";
    static constexpr const char *kMspTcp = "msp-tcp";

    void connectSerial(const QString &port, int baud);
    void connectUdp(const QString &host, int port);
    void connectTcp(const QString &host, int port);
    /// Cellular link through a relay: {moduleId, keyHex, host, port, certSha256}
    void connectLte(const QVariantMap &options);
    /// MSP (INAV, Betaflight): profile "normal" or "slow" (long-range radio)
    void connectMspSerial(const QString &port, int baud, const QString &profile);
    void connectMspTcp(const QString &host, int port, const QString &profile);
    /// INAV waypoint missions over the MSP link (results in mspMissionDone)
    void mspMissionUpload(const QList<MspWaypoint> &wps, bool save);
    void mspMissionDownload();
    void disconnectLink();

    /// Send a message packed by the caller: sysid/compid/sequence and CRC are set here.
    void send(const mavlink_message_t &msg);
    /// Raw bytes to the vehicle (RTCM injection); dropped while muted.
    void sendRaw(const QByteArray &bytes);

    void setGcsMuted(bool muted);
    bool gcsMuted() const { return m_muted.load(); }
    /// While a log is replayed no tlog is started and the trace is quiet.
    void setReplayActive(bool active);

    /// Feed a frame from a log replay as if it had arrived (link thread or any).
    void injectReplay(const MavMessage &m);

    QString connectionType() const;

signals:
    void connectionState(const QString &state);           // "CONNECTED" / "DISCONNECTED"
    void connected(const QString &type);
    void connectFailed(const QString &type, const QString &error);
    void linkStats(const QString &kind, double kbps, qint64 bytesRx);
    /// Every message, on the link thread (VehicleHub listens with a direct connection).
    void message(const corv::MavMessage &m);
    void rawFrame(const QByteArray &frame);                // received MAVLink bytes (telemetry forward)
    /// Cellular link status (LteTransport::status), once a second and on state changes
    void lteStatus(const QVariantMap &status);
    /// Something wrong with what arrives (no bytes, bytes but no frame): for the HUD.
    void linkWarning(const QString &text);
    void mspMissionProgress(const QString &phase, int done, int total);
    void mspMissionDone(const QString &op, bool ok, const QString &error, const QVariantMap &result);

private:
    void openTransport(Transport *t, const QString &type);
    void closeTransport(const QString &why, bool notify);
    void onBytes(const QByteArray &b);
    void deliver(const MavMessage &m);
    void sendNow(mavlink_message_t msg);
    void writeOut(const QByteArray &bytes, bool record);
    void heartbeat();
    void statsTick();
    void diagnose();
    void setConnectionState(const QString &state);
    template <typename F>
    void post(F &&f);

    MessageBus *m_bus;
    Transport *m_transport = nullptr;
    QString m_type;
    std::unique_ptr<MavlinkParser> m_parser;
    std::unique_ptr<MspSession> m_msp;
    QString m_mspProfile;
    TlogWriter m_tlog;
    bool m_tlogAuto = false;
    LinkTrace m_trace;
    QTimer *m_heartbeat = nullptr;
    QTimer *m_stats = nullptr;
    uint8_t m_seq = 0;
    qint64 m_bytesRx = 0, m_bytesAtTick = 0;
    qint64 m_connectedAt = 0, m_lastDiag = 0;
    quint64 m_decodedPackets = 0;   // MSP replies decoded
    bool m_warnedNoData = false, m_warnedNoFrames = false;
    std::atomic<bool> m_muted{false};
    std::atomic<bool> m_replay{false};
    mutable QString m_typeCopy;
};

} // namespace corv
