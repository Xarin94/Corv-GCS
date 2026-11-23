#include "vehicle/TerrainFeeder.h"

#include "core/DebugLog.h"
#include "core/Geo.h"
#include "link/MessageBus.h"

#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kBlocksEast = 8;
constexpr qint64 kReportLogMs = 10000;
constexpr qint64 kForgetSentMs = 60000;
constexpr qsizetype kMaxSentKept = 4000;
// ArduPilot's LOCATION_SCALING_FACTOR_INV: 1e-7 degrees per metre
constexpr double kE7PerMetre = 89.83204953368922;

QString gridText(int32_t lat, int32_t lon)
{
    return u"%1, %2"_s.arg(lat * 1e-7, 0, 'f', 4).arg(lon * 1e-7, 0, 'f', 4);
}
} // namespace

TerrainFeeder::TerrainFeeder(MessageBus *bus, HeightFn height, SendFn send, QObject *parent)
    : QObject(parent), m_height(std::move(height)), m_send(std::move(send))
{
    m_clock.start();
    m_timer.setInterval(kSendIntervalMs);
    m_timer.setTimerType(Qt::PreciseTimer);
    connect(&m_timer, &QTimer::timeout, this, &TerrainFeeder::sendNext);
    if (!bus) return;
    bus->subscribe(MAVLINK_MSG_ID_TERRAIN_REQUEST, this, [this](const MavMessage &m) {
        if (m.replay || m.synthetic) return;
        mavlink_terrain_request_t r;
        mavlink_msg_terrain_request_decode(&m.msg, &r);
        onRequest(r);
    });
    bus->subscribe(MAVLINK_MSG_ID_TERRAIN_REPORT, this, [this](const MavMessage &m) {
        if (m.replay || m.synthetic) return;
        mavlink_terrain_report_t r;
        mavlink_msg_terrain_report_decode(&m.msg, &r);
        onReport(r);
    });
}

void TerrainFeeder::reset()
{
    m_timer.stop();
    m_queue.clear();
    m_queued.clear();
    m_sentAt.clear();
    m_reportedMissing.clear();
    m_stats = {};
    m_lastRequest = m_lastSent = m_lastMissing = m_reportChanged = m_reportLogged = -1;
    m_warned = false;
    emit statsChanged();
}

void TerrainFeeder::setMuted(bool on)
{
    m_muted = on;
    if (!on) return;
    // The vehicle asks again for what it still lacks
    m_timer.stop();
    m_queue.clear();
    m_queued.clear();
    m_stats.queued = 0;
}

void TerrainFeeder::offset(int32_t latE7, int32_t lonE7, double north, double east, double &lat, double &lon)
{
    const double dlat = north * kE7PerMetre;
    // Longitude scaled at the mid latitude
    const double scale = std::max(0.01, std::cos((latE7 + dlat / 2) * 1e-7 * kDeg));
    lat = (latE7 + dlat) * 1e-7;
    lon = (lonE7 + east * kE7PerMetre / scale) * 1e-7;
    if (lon >= 180) lon -= 360;
    else if (lon < -180) lon += 360;
}

void TerrainFeeder::onRequest(const mavlink_terrain_request_t &r)
{
    if (m_muted || r.grid_spacing == 0 || std::abs(r.lat) > 890000000) return;   // no grid at the poles
    const qint64 now = m_clock.elapsed();
    if (m_lastRequest < 0)
        LOG_I("terrain-feed") << "the vehicle asks for terrain: grid" << gridText(r.lat, r.lon) << "spacing" << r.grid_spacing << "m";
    m_lastRequest = now;
    if (m_sentAt.size() > kMaxSentKept) m_sentAt.removeIf([now](const auto &it) { return now - it.value() > kForgetSentMs; });
    for (int bit = 0; bit < kBlocks; ++bit) {
        if (!(r.mask & (quint64(1) << bit))) continue;
        const Block b{r.lat, r.lon, r.grid_spacing, quint8(bit)};
        if (m_queued.contains(b)) continue;
        const auto sent = m_sentAt.constFind(b);
        if (sent != m_sentAt.cend() && now - *sent < kResendAfterMs) continue;   // the answer is still on its way
        m_queue.enqueue(b);
        m_queued.insert(b);
    }
    m_stats.queued = int(m_queue.size());
    if (!m_queue.isEmpty() && !m_timer.isActive()) m_timer.start();
}

std::optional<std::array<int16_t, 16>> TerrainFeeder::heights(const Block &b, bool *pending) const
{
    // Block `bit` starts (bit / 8) * 4 points north and (bit % 8) * 4 east of the
    // corner; point (x north, y east) of a block is data[x * 4 + y]
    const int north0 = b.bit / kBlocksEast * kBlockPoints, east0 = b.bit % kBlocksEast * kBlockPoints;
    std::array<int16_t, 16> out{};
    for (int x = 0; x < kBlockPoints; ++x) {
        for (int y = 0; y < kBlockPoints; ++y) {
            double lat = 0, lon = 0, h = 0;
            offset(b.lat, b.lon, double(north0 + x) * b.spacing, double(east0 + y) * b.spacing, lat, lon);
            const Lookup l = m_height(lat, lon, h);
            if (l != Lookup::Found) {
                *pending = l == Lookup::Pending;
                return std::nullopt;
            }
            out[std::size_t(x * kBlockPoints + y)] = int16_t(std::clamp(std::lround(h), -32768L, 32767L));
        }
    }
    return out;
}

void TerrainFeeder::sendNext()
{
    const qint64 now = m_clock.elapsed();
    // One message per turn; the blocks that cannot be answered cost nothing on the link
    while (!m_queue.isEmpty()) {
        const Block b = m_queue.dequeue();
        m_queued.remove(b);
        bool pending = false;
        const auto h = heights(b, &pending);
        if (!h) {
            if (pending) continue;   // the tile is coming: the vehicle asks again in 2 s
            ++m_stats.missing;
            m_lastMissing = now;
            const quint64 grid = (quint64(quint32(b.lat)) << 32) | quint32(b.lon);
            if (!m_reportedMissing.contains(grid)) {
                m_reportedMissing.insert(grid);
                LOG_W("terrain-feed") << "no elevation data near" << gridText(b.lat, b.lon)
                                      << "- blocks left unanswered (SRTM tile missing or void)";
            }
            continue;
        }
        mavlink_message_t m;
        mavlink_msg_terrain_data_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, b.lat, b.lon, b.spacing, b.bit, h->data());
        m_send(m);
        m_sentAt.insert(b, now);
        m_lastSent = now;
        ++m_stats.sent;
        break;
    }
    m_stats.queued = int(m_queue.size());
    if (m_queue.isEmpty()) m_timer.stop();
    emit statsChanged();
}

void TerrainFeeder::onReport(const mavlink_terrain_report_t &r)
{
    const qint64 now = m_clock.elapsed();
    if (r.pending != m_stats.pending || r.loaded != m_stats.loaded) {
        // Every change while a grid loads is one line a second: the log keeps one per
        // kReportLogMs, plus the moment everything is loaded
        const bool done = r.pending == 0 && m_stats.pending > 0;
        if (done || (r.pending > 0 && (m_reportLogged < 0 || now - m_reportLogged >= kReportLogMs))) {
            LOG_I("terrain-feed") << "vehicle terrain: pending" << r.pending << "loaded" << r.loaded << "-" << m_stats.sent << "blocks sent";
            m_reportLogged = now;
        }
        m_stats.pending = r.pending;
        m_stats.loaded = r.loaded;
        m_reportChanged = now;
        emit statsChanged();
    }
    if (r.pending == 0 || m_warned || now - m_reportChanged < kStallMs) return;

    const auto recent = [now](qint64 t) { return t >= 0 && now - t < kStallMs; };
    QString why, hud;
    if (!recent(m_lastRequest)) {
        if (m_muted) return;   // requests are not looked at
        why = u"the vehicle has %1 blocks pending but asks the GCS for none. ArduPilot does not ask while a grid waits to be read "
              "from its own storage, and it waits for ever if the storage cannot be read: the SD card is missing, full, "
              "write-protected or failing. Check the card and its APM/TERRAIN folder, then reboot the autopilot. "
              "(Firmware newer than 4.6 can also run without it: TERRAIN_OPTIONS bit 1, Disable Disk.)"_s.arg(r.pending);
        hud = u"TERRAIN: vehicle SD card not readable"_s;
    } else if (recent(m_lastMissing) && !recent(m_lastSent)) {
        why = u"the vehicle asks for terrain the GCS has no elevation data for (SRTM tile missing or void): "
              "add the tile to topo/ or allow the download"_s;
        hud = u"TERRAIN: no elevation data here"_s;
    } else if (recent(m_lastSent)) {
        why = u"blocks are being sent but the vehicle's count does not move (pending %1, loaded %2)"_s.arg(r.pending).arg(r.loaded);
        hud = u"TERRAIN: vehicle not taking data"_s;
    } else {
        return;   // nothing to say yet
    }
    m_warned = true;
    LOG_W("terrain-feed") << "stalled:" << why;
    emit hudMessage(hud, u"warning"_s);
}

} // namespace corv
