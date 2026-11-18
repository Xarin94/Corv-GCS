// MspSession.h - MSP / MSP2 telemetry adapter for INAV and Betaflight (port of
// msp-manager.js). Link thread.
//
// The flight controller's protocol is decoded here and handed on as synthetic
// MAVLink (ATTITUDE, VFR_HUD, GPS_RAW_INT, GLOBAL_POSITION_INT, RAW_IMU,
// SYS_STATUS, BATTERY_STATUS, RC_CHANNELS, HEARTBEAT): nothing downstream knows
// MSP exists. MSP is request / response, not a stream: a scheduler ticks every
// 20 ms and keeps exactly one request in flight (MSP has no sequence numbers),
// polling a fixed set at per-message rates ("normal", or "slow" for long-range
// radios); a command that stays silent three times is dropped from the
// schedule. The identification (FC variant, version, box names and ids) goes
// first: the flight mode is the set of active boxes, named by MSP_BOXNAMES.
//
// INAV waypoint missions: 21-byte records addressed by index (MSP_WP /
// MSP_SET_WP, MSP_WP_GETINFO for the capacity, MSP_WP_MISSION_SAVE to keep it
// in EEPROM). A transfer owns the request slot, each record retried three times.
//
// Framing:
//   v1  '$' 'M' <dir> <size:u8> <cmd:u8> <payload> <XOR of size..payload>
//   v2  '$' 'X' <dir> <flag:u8> <cmd:u16LE> <size:u16LE> <payload> <CRC8 DVB-S2>
#pragma once

#include "link/Mavlink.h"

#include <QByteArray>
#include <QList>
#include <QObject>
#include <QStringList>
#include <QTimer>
#include <QVariantMap>
#include <deque>
#include <functional>

namespace corv {

struct MspWaypoint {
    int index = 0, action = 1;
    qint32 lat = 0, lon = 0, alt = 0;   // 1e7 deg, cm
    int p1 = 0, p2 = 0, p3 = 0;
    int flag = 0;                       // 0xA5 on the last waypoint
};

class MspSession : public QObject
{
    Q_OBJECT
public:
    using WriteFn = std::function<void(const QByteArray &)>;
    using MessageFn = std::function<void(const mavlink_message_t &)>;

    MspSession(WriteFn write, MessageFn message, const QString &profile, QObject *parent = nullptr);
    ~MspSession() override;

    /// Received bytes.
    void push(const QByteArray &bytes);

    /// Write the waypoints (indexes 1..n), optionally to EEPROM. Result in missionDone("upload", ...).
    void missionUpload(const QList<MspWaypoint> &wps, bool save);
    /// Read the board's waypoint list. Result in missionDone("download", ..., {wps, maxWaypoints, valid}).
    void missionDownload();

    QString fcVariant() const { return m_fcVariant; }
    QString fcVersion() const { return m_fcVersion; }

    // Pure helpers (tests)
    static QByteArray encodeRequest(int cmd, const QByteArray &payload = {});
    static uint8_t crc8DvbS2(uint8_t crc, uint8_t byte);
    static QByteArray encodeWaypoint(const MspWaypoint &wp);
    static bool decodeWaypoint(const QByteArray &b, MspWaypoint *wp);

signals:
    void missionProgress(const QString &phase, int done, int total);
    /// op: upload | download; result: count, saved, maxWaypoints, wps (QVariantList of maps), valid
    void missionDone(const QString &op, bool ok, const QString &error, const QVariantMap &result);
    void identified(const QString &variant, const QString &version);

private:
    struct Job {
        int cmd = 0;
        QByteArray payload;
        std::function<void(const QByteArray &)> ok;
        std::function<void(const QString &)> fail;
        int tries = 1;
        int timeout = 0;
    };
    struct Entry {
        int cmd;
        double hz;
    };

    void tick();
    void send(const Job &job);
    void onTimeout();
    void reply(int cmd, const QByteArray &payload);
    void rejected(int cmd);
    void decode(int cmd, const QByteArray &p);
    void request(int cmd, const QByteArray &payload, int timeoutMs, std::function<void(const QByteArray &)> ok,
                 std::function<void(const QString &)> fail);
    void uploadNext(std::shared_ptr<QList<MspWaypoint>> wps, int i, bool save, int maxWaypoints);
    void downloadNext(std::shared_ptr<QVariantList> out, int i, int count, int maxWaypoints, bool valid);

    // Synthetic MAVLink
    void emitAttitude();
    void emitVfrHud();
    void emitGps();
    void emitGlobalPosition();
    void emitImu();
    void emitSysStatus();
    void emitRc();
    void emitHeartbeat();
    QString modeName(const std::array<quint32, 4> &boxes) const;

    WriteFn m_write;
    MessageFn m_message;
    QByteArray m_rx;
    QTimer m_tick, m_timeout;
    QList<Entry> m_schedule;
    QHash<int, qint64> m_nextDue;
    QHash<int, int> m_misses;
    int m_replyTimeoutMs = 500;
    bool m_pending = false;
    Job m_pendingJob;
    std::deque<Job> m_queue;
    bool m_missionBusy = false;

    // Decoded state between polls (one MAVLink message needs several MSP replies)
    double m_roll = 0, m_pitch = 0, m_yaw = 0;
    qint32 m_lat = 0, m_lon = 0;
    double m_altGps = 0, m_groundspeed = 0, m_gpsCourse = 0;
    int m_fixType = 0, m_numSat = 0, m_hdop = 65535;
    double m_altBaro = 0, m_climb = 0, m_airspeed = 0;
    double m_voltage = 0, m_current = 0;
    quint32 m_mahDrawn = 0;
    int m_rssi = 0, m_batteryPercent = 0;
    bool m_armed = false;
    QString m_mode = QStringLiteral("MANUAL");
    QStringList m_boxNames;
    double m_accX = 0, m_accY = 0, m_accZ = 0;
    int m_gyroX = 0, m_gyroY = 0, m_gyroZ = 0;
    QList<int> m_rc;
    QString m_fcVariant, m_fcVersion;
};

} // namespace corv
