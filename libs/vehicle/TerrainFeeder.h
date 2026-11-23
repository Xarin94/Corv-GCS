// TerrainFeeder.h - terrain heights for the vehicle's own database (port of
// js/mavlink/TerrainFeeder.js).
//
// ArduPilot with TERRAIN_ENABLE asks the GCS for the terrain it lacks: a
// TERRAIN_REQUEST names a grid (south-west corner, spacing) and a 56-bit mask
// of the 4x4 blocks it still needs, and each block goes back as one
// TERRAIN_DATA. The vehicle repeats the request every 2 s for whatever is still
// missing, so:
//   - a block is queued once however many requests name it, and sent again only
//     if the vehicle still asks for it a while after it went out (the packet
//     was lost);
//   - one queue drains at a fixed pace, so a burst of requests cannot flood a
//     telemetry radio;
//   - a block with a point of unknown height is not sent at all. The vehicle
//     keeps what it receives and stops asking, so a stand-in 0 m would be flown
//     as the ground. Unanswered, it keeps asking; meanwhile the tile is read
//     from disk or downloaded.
//
// TERRAIN_REPORT says how many blocks the vehicle still lacks. When that does
// not move for a while the feeder says why, once per connection: the vehicle
// asks for nothing (ArduPilot does not ask the GCS while a grid waits to be
// read from its SD card, and an unreadable card keeps it waiting for ever), the
// GCS has no heights for what it asks, or the vehicle drops what is sent.
//
// Requests of a log being replayed are not answered. GUI thread.
// Protocol: https://mavlink.io/en/services/terrain.html
#pragma once

#include "link/MavMessage.h"

#include <QElapsedTimer>
#include <QHash>
#include <QObject>
#include <QQueue>
#include <QSet>
#include <QTimer>
#include <array>
#include <functional>
#include <optional>

namespace corv {

class MessageBus;

class TerrainFeeder : public QObject
{
    Q_OBJECT
public:
    static constexpr int kBlocks = 56;          // 7 x 8 blocks (north x east) of 4 x 4 points
    static constexpr int kBlockPoints = 4;
    static constexpr int kSendIntervalMs = 50;  // ~55-byte packets: about a fifth of a 57600-baud radio
    static constexpr int kResendAfterMs = 3000; // requests repeat every 2 s; still asked after this: the packet was lost
    static constexpr int kStallMs = 20000;      // TERRAIN_REPORT unchanged this long with blocks pending: stalled

    /// Height at a point, metres AMSL. Pending: the tile is being read or downloaded, ask again.
    enum class Lookup { Found, Pending, Missing };
    using HeightFn = std::function<Lookup(double lat, double lon, double &metres)>;
    using SendFn = std::function<void(const mavlink_message_t &)>;

    struct Stats {
        int sent = 0;       // TERRAIN_DATA messages
        int missing = 0;    // blocks left unanswered: no elevation data
        int queued = 0;
        int pending = -1, loaded = -1;   // TERRAIN_REPORT, -1 before any
    };

    TerrainFeeder(MessageBus *bus, HeightFn height, SendFn send, QObject *parent = nullptr);

    /// A new link: start from what the vehicle asks then.
    void reset();
    /// GCS output muted: nothing is sent, nothing counted.
    void setMuted(bool on);
    const Stats &stats() const { return m_stats; }

    void onRequest(const mavlink_terrain_request_t &r);
    void onReport(const mavlink_terrain_report_t &r);

    /// Point `north` / `east` metres from a degE7 corner, as the vehicle computes it (Location::offset).
    static void offset(int32_t latE7, int32_t lonE7, double north, double east, double &lat, double &lon);

signals:
    void hudMessage(const QString &text, const QString &level);
    void statsChanged();

private:
    struct Block {
        int32_t lat = 0, lon = 0;
        quint16 spacing = 0;
        quint8 bit = 0;
        bool operator==(const Block &) const = default;
    };
    friend size_t qHash(const Block &b, size_t seed) { return qHashMulti(seed, b.lat, b.lon, b.spacing, b.bit); }

    void sendNext();
    std::optional<std::array<int16_t, 16>> heights(const Block &b, bool *pending) const;

    HeightFn m_height;
    SendFn m_send;
    QTimer m_timer;
    QElapsedTimer m_clock;
    QQueue<Block> m_queue;
    QSet<Block> m_queued;
    QHash<Block, qint64> m_sentAt;
    QSet<quint64> m_reportedMissing;   // grids
    Stats m_stats;
    bool m_muted = false;

    // What the vehicle reports and what it asks for, to explain a stall (ms on m_clock, -1 never)
    qint64 m_lastRequest = -1, m_lastSent = -1, m_lastMissing = -1, m_reportChanged = -1, m_reportLogged = -1;
    bool m_warned = false;
};

} // namespace corv
