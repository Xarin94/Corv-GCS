// LinkTrace.h - what the MAVLink link did, for the debug log (port of mavlink-trace.js).
//
// Logs the events worth reading after the fact: components appearing,
// heartbeats lost and back, arm / mode / system-status changes, STATUSTEXT,
// command and mission acknowledgements, GPS fix and EKF flag changes; while
// connected, every 10 s one link line (kB/s, msg/s, loss from sequence gaps)
// and every 30 s the top received messages. High-rate traffic is only counted.
#pragma once

#include "link/MavMessage.h"

#include <QElapsedTimer>
#include <QHash>
#include <QString>

namespace corv {

class LinkTrace
{
public:
    void onConnect(const QString &what);
    void onDisconnect(const QString &why);
    void onMessage(const MavMessage &m);
    void onSend(const mavlink_message_t &m);
    void onBytes(qint64 n) { m_bytes += n; }
    /// Called every second by the link manager: writes the periodic lines when due.
    void tick();

private:
    struct Comp {
        int lastSeq = -1;
        qint64 received = 0, lost = 0;
        qint64 lastHb = 0;
        bool hbLost = false;
    };
    bool m_connected = false;
    QString m_what;
    qint64 m_bytes = 0, m_bytesAtSnap = 0, m_msgs = 0, m_msgsAtSnap = 0;
    qint64 m_lastSnap = 0;
    int m_snaps = 0;
    QHash<quint16, Comp> m_comps;            // sysid << 8 | compid
    QHash<uint32_t, qint64> m_counts;        // per msgid since the last top line
    QHash<uint32_t, qint64> m_txCounts;
    // Vehicle (component 1) state for change lines
    int m_baseMode = -1;
    long long m_customMode = -1;
    int m_sysStatus = -1;
    int m_fix = -1;
    int m_ekfFlags = -1;
};

} // namespace corv
