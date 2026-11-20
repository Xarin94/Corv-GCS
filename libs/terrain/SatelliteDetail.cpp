#include "terrain/SatelliteDetail.h"

#include "core/DebugLog.h"
#include "terrain/TileStore.h"

#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr double kRecenter = 0.75;               // the window moves once the aircraft is this many tiles off its centre
constexpr double kDeactivate = 1.25;             // hysteresis on the distance a level is used to
constexpr std::size_t kMaxCopiesPerFrame = 8;    // tile uploads per frame
constexpr double kMPerPxZ0 = 156543.03392;       // Web Mercator metres per pixel at zoom 0 on the equator

int mod(int a, int n) { return ((a % n) + n) % n; }
} // namespace

SatelliteDetail::SatelliteDetail(QObject *parent) : QObject(parent)
{
    for (int i = 0; i < kLevels; ++i) {
        m_levels[std::size_t(i)].index = i;
        m_levels[std::size_t(i)].zoom = zoomOf(i);
    }
    TileStore &store = TileStore::instance();
    connect(&store, &TileStore::tileReady, this, [this](const QString &p, int z, int x, int y) { onTile(p, z, x, y, true); });
    connect(&store, &TileStore::tileFailed, this, [this](const QString &p, int z, int x, int y) { onTile(p, z, x, y, false); });
}

void SatelliteDetail::setMaxZoom(int zoom)
{
    m_maxZoom = std::clamp(zoom, 16, zoomOf(kLevels - 1));
}

void SatelliteDetail::release(Level &l, std::vector<DetailOp> &ops)
{
    l.active = false;
    l.placed = false;
    l.cells.fill({});
    if (l.allocated) {
        ops.push_back({DetailOp::Release, l.index, 0, {}});
        l.allocated = false;
    }
    std::erase_if(m_copies, [&](const Copy &c) { return c.level == l.index; });
}

void SatelliteDetail::request(Level &l, int slot)
{
    const Slot &s = l.cells[std::size_t(slot)];
    const QImage img = TileStore::instance().tile(m_provider, l.zoom, s.tx, s.ty);
    if (!img.isNull()) m_copies.push_back({l.index, slot, s.tx, s.ty, img});
    // otherwise tileReady / tileFailed answer
}

void SatelliteDetail::onTile(const QString &provider, int z, int x, int y, bool ok)
{
    if (provider != m_provider || z < zoomOf(0) || z > zoomOf(kLevels - 1)) return;
    Level &l = m_levels[std::size_t(z - zoomOf(0))];
    if (!l.placed) return;
    const int slot = mod(y, kTiles) * kTiles + mod(x, kTiles);
    Slot &s = l.cells[std::size_t(slot)];
    if (s.tx != x || s.ty != y || s.state != SlotState::Loading) return;   // the window moved on
    const QImage img = ok ? TileStore::instance().cached(provider, z, x, y) : QImage();
    if (img.isNull()) {
        s.state = SlotState::Missing;   // the chunk texture stays
        return;
    }
    m_copies.push_back({l.index, slot, x, y, img});
}

void SatelliteDetail::placeWindow(Level &l, double lat, double lon, std::vector<DetailOp> &ops)
{
    const double n = std::ldexp(1.0, l.zoom);
    const double fx = (lon + 180) / 360 * n;
    const double fy = (1 - std::asinh(std::tan(lat * kDeg)) / kPi) / 2 * n;
    const int half = kTiles / 2;
    if (l.placed && std::abs(fx - (l.tx0 + half)) < kRecenter && std::abs(fy - (l.ty0 + half)) < kRecenter) return;

    const int tx0 = int(std::lround(fx)) - half, ty0 = int(std::lround(fy)) - half;
    const bool jump = !l.placed || std::abs(tx0 - l.tx0) >= kTiles || std::abs(ty0 - l.ty0) >= kTiles;
    l.tx0 = tx0;
    l.ty0 = ty0;
    l.placed = true;
    if (jump) {
        // Nothing to keep: one clear instead of 64 slot clears
        l.cells.fill({});
        std::erase_if(m_copies, [&](const Copy &c) { return c.level == l.index; });
        ops.push_back({DetailOp::Clear, l.index, 0, {}});
    }

    struct Incoming {
        int slot;
        double d;
    };
    std::vector<Incoming> incoming;
    for (int ty = ty0; ty < ty0 + kTiles; ++ty) {
        for (int tx = tx0; tx < tx0 + kTiles; ++tx) {
            const int slot = mod(ty, kTiles) * kTiles + mod(tx, kTiles);
            Slot &held = l.cells[std::size_t(slot)];
            if (held.state != SlotState::Empty && held.tx == tx && held.ty == ty) continue;
            held = {tx, ty, SlotState::Loading};
            incoming.push_back({slot, std::hypot(tx + 0.5 - fx, ty + 0.5 - fy)});
        }
    }
    if (incoming.empty()) return;
    // The old tiles must not show at the new place, even for a frame
    if (!jump)
        for (const Incoming &t : incoming) ops.push_back({DetailOp::ClearSlot, l.index, t.slot, {}});
    // Nearest first
    std::sort(incoming.begin(), incoming.end(), [](const Incoming &a, const Incoming &b) { return a.d < b.d; });
    for (const Incoming &t : incoming) request(l, t.slot);
}

void SatelliteDetail::writeUniforms(const Level &l, LatLon origin, DetailFrame &frame) const
{
    const double n = std::ldexp(1.0, l.zoom);
    const double cx = l.tx0 + kTiles / 2, cy = l.ty0 + kTiles / 2;   // window centre, a tile corner
    const double lon = cx / n * 360 - 180;
    const double lat = std::atan(std::sinh(kPi * (1 - 2 * cy / n))) * kRad;
    const GroundMeters c = latLonToMeters(lat, lon, origin);
    // World metres per degree, as latLonToMeters() has them
    const double mPerLon = latLonToMeters(lat, lon + 1, origin).x - c.x;
    const double mPerLat = c.z - latLonToMeters(lat + 1, lon, origin).z;
    const double pxPerDeg = kTile * n / 360;
    frame.map[std::size_t(l.index)] = QVector4D(float(c.x), float(c.z), float(pxPerDeg / mPerLon),
                                                float(pxPerDeg / std::cos(lat * kDeg) / mPerLat));
    frame.win[std::size_t(l.index)] = QVector4D(kSize / 2.f, kSize / 2.f, float(mod(l.tx0, kTiles) * kTile), float(mod(l.ty0, kTiles) * kTile));
}

void SatelliteDetail::flushCopies(std::vector<DetailOp> &ops)
{
    std::size_t taken = 0;
    std::vector<Copy> keep;
    for (Copy &c : m_copies) {
        if (taken >= kMaxCopiesPerFrame) {
            keep.push_back(std::move(c));
            continue;
        }
        Level &l = m_levels[std::size_t(c.level)];
        Slot &s = l.cells[std::size_t(c.slot)];
        if (!l.allocated || s.tx != c.tx || s.ty != c.ty || s.state != SlotState::Loading) continue;   // the window moved on
        QImage img = c.image;
        if (img.size() != QSize(kTile, kTile)) img = img.scaled(kTile, kTile, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        ops.push_back({DetailOp::Upload, c.level, c.slot, img.convertToFormat(QImage::Format_RGBA8888_Premultiplied)});
        s.state = SlotState::Ready;
        ++taken;
    }
    m_copies = std::move(keep);
}

void SatelliteDetail::update(double lat, double lon, LatLon origin, const QVector3D &cam, const QVector3D &air,
                             std::optional<double> groundY, double pixelAngle, bool enabled, std::vector<DetailOp> &ops,
                             DetailFrame &frame)
{
    frame.on = QVector4D();
    // Levels above the setting give their texture back
    for (Level &l : m_levels)
        if (l.zoom > m_maxZoom && (l.allocated || l.placed)) release(l, ops);
    if (!enabled || !std::isfinite(lat) || !std::isfinite(lon) || !(pixelAngle > 0)) return;

    const double dy = groundY ? cam.y() - *groundY : 0.0;
    const double camDist = std::sqrt(double(cam.x() - air.x()) * (cam.x() - air.x()) + dy * dy
                                     + double(cam.z() - air.z()) * (cam.z() - air.z()));
    for (Level &l : m_levels) {
        if (l.zoom > m_maxZoom) continue;
        const double res = kMPerPxZ0 * std::cos(lat * kDeg) / std::ldexp(1.0, l.zoom);   // metres per texel
        // Farthest a pixel can be and still cover less than two texels, and how far
        // from the camera the window's nearest edge is (at most)
        const double useDist = 2 * res / pixelAngle;
        const double reach = camDist - res * kSize / 2;
        const bool was = l.active;
        if (!l.active && reach < useDist) l.active = true;
        else if (l.active && reach > useDist * kDeactivate) l.active = false;
        if (l.active != was) LOG_D("terrain") << "satellite detail zoom" << l.zoom << (l.active ? "on" : "off") << "camera" << int(camDist) << "m";
        if (l.active) {
            if (!l.allocated) {
                ops.push_back({DetailOp::Allocate, l.index, 0, {}});
                l.allocated = true;
                l.placed = false;   // the window starts with a clear
            }
            placeWindow(l, lat, lon, ops);
            writeUniforms(l, origin, frame);
        }
        frame.on[l.index] = l.active ? 1.f : 0.f;
    }
    flushCopies(ops);
    // What the levels hold, when it changed (at most every 10 s)
    if (m_statsClock.isValid() && m_statsClock.elapsed() < 10000) return;
    QStringList parts;
    for (const LevelStats &st : stats())
        if (st.active) parts << u"z%1 %2/%3/%4"_s.arg(st.zoom).arg(st.ready).arg(st.loading).arg(st.missing);
    const QString line = parts.join(u", "_s);
    if (line != m_lastStats) {
        m_lastStats = line;
        m_statsClock.restart();
        if (!line.isEmpty()) LOG_D("terrain") << "satellite detail ready/loading/missing:" << line;
    }
}

std::array<SatelliteDetail::LevelStats, SatelliteDetail::kLevels> SatelliteDetail::stats() const
{
    std::array<LevelStats, kLevels> out{};
    for (const Level &l : m_levels) {
        LevelStats &s = out[std::size_t(l.index)];
        s.zoom = l.zoom;
        s.active = l.active;
        s.allocated = l.allocated;
        for (const Slot &sl : l.cells) {
            if (sl.state == SlotState::Ready) ++s.ready;
            else if (sl.state == SlotState::Loading) ++s.loading;
            else if (sl.state == SlotState::Missing) ++s.missing;
        }
    }
    return out;
}

} // namespace corv
