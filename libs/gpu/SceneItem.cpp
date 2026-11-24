#include "gpu/SceneItem.h"

#include "core/Clock.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "gpu/FrameStats.h"
#include "gpu/OutlineRenderer.h"
#include "gpu/OverlayRenderer.h"
#include "gpu/PointCloudRenderer.h"
#include "gpu/RosSurfaceRenderer.h"
#include "gpu/TerrainRenderer.h"
#include "gpu/VideoRenderer.h"
#include "scene/CellOverlay.h"
#include "scene/Mission3D.h"
#include "scene/PointCloudData.h"
#include "scene/RenderQuality.h"
#include "scene/RosSurfaceData.h"
#include "scene/ModelData.h"
#include "scene/TrafficTrails.h"
#include "terrain/Hgt.h"
#include "terrain/HgtDownloader.h"
#include "vehicle/VehicleHub.h"

#include <QDateTime>
#include <QFutureWatcher>
#include <QtConcurrent>
#include <QElapsedTimer>
#include <QFile>
#include <QFontMetricsF>
#include <QHash>
#include <QKeyEvent>
#include <QMutex>
#include <QNativeGestureEvent>
#include <QSet>
#include <QPainter>
#include <QPainterPath>
#include <QQuickWindow>
#include <QScreen>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

VehicleHub *SceneItem::s_hub = nullptr;
int SceneItem::s_frameCap = 60;
double SceneItem::s_frameWallMs = 0;
double SceneItem::s_lastTickMs = -1e9;

bool SceneItem::pacing() { return s_frameCap > 0 && monoMs() - s_lastTickMs < 250; }

double SceneItem::frameTimeMs() { return pacing() ? s_frameWallMs : double(wallMs()); }

namespace {
constexpr double kTriGridFullAgl = 150;   // TERRAIN_GRID.fullAgl
constexpr double kTriGridMaxAgl = 200;    // TERRAIN_GRID.maxAgl
constexpr double kNear = 1.0, kFar = 300000.0;

// Trail (Scene3D.js)
constexpr std::size_t kMaxTrailPoints = 50000;
constexpr double kTrailMinStepM = 2;
constexpr double kTrailMaxJumpM = 2000;

// Orbit (main.js)
constexpr double kOrbitMinPitch = -1.2, kOrbitMaxPitch = 1.2;
constexpr double kOrbitMinDistance = 4, kOrbitMaxDistance = 2500;
constexpr double kOrbitRotateSpeed = 0.005, kOrbitZoomSpeed = 0.12;
constexpr double kChaseCamClearanceM = 30;

// Look-around (main.js)
const double kLookPitchMax = 90 * kDeg;
const double kLookYawMax = 110 * kDeg;
constexpr double kLookFullSweepS = 0.3;
constexpr double kLookMinSweepS = 0.12;

// Home marker
constexpr double kHomePoleM = 100;
constexpr double kHomeRingM = 25;

// Corridor (TrajectoryCorridor3D.js)
constexpr int kCorridorPoints = 40;
constexpr double kHalfWidthNear = 1.0, kHalfWidthFar = 6.0;
constexpr double kAltDrop = 1.2;
constexpr double kStartOffset = -8;
constexpr double kTickIntervalS = 5;
constexpr float kBorderAlpha = 0.95f, kFillAlpha = 0.28f, kEndAlpha = 0.35f;

// LTE cell sites
constexpr double kCellPoleM = 10;

// Labels: plain waypoints labelled at most, px from the point
constexpr std::size_t kMaxMissionLabels = 60;
constexpr float kMissionLabelPx = 17, kTrafficLabelPx = 22, kHomeLabelPx = 26;

// Target (Scene3D.setTargetMarker3D)
constexpr double kTargetPoleM = 100, kTargetSphereM = 10, kTargetRingM = 15;

// Traffic (Traffic3D.js): the nearest aircraft, circles sized by their distance
constexpr std::size_t kMaxTraffic = 8;
constexpr double kTrafficMaxM = 60000;
constexpr double kTrafficNearM = 1000, kTrafficFarM = 25000;
constexpr double kTrafficNearPx = 19, kTrafficFarPx = 10;   // radius
constexpr qint64 kTrailRefreshMs = 100;                     // the circle moves every frame, the trail at 10 Hz

// Water (main.js / Water3D.js)
constexpr double kWaterPlaneRadius = 500, kWaterPlaneCell = 25;
// Relative navigation: the plane at zero is the view's only reference
constexpr double kZeroPlaneRadius = 3000, kZeroPlaneCell = 50;
constexpr float kZeroPlaneOpacity = 0.45f;
const QColor kZeroPlaneColour(0x8f, 0xb4, 0xc4);

struct Palette {
    QColor trail, route, activeLeg, home, ownship, traffic, circle, area, corridor, perimeter, poi;
    QColor label;        // every label in this colour (invalid: the colour of its symbol)
    QColor labelHalo;
    float labelHaloPx;
};
const Palette kDark{QColor(0xff3b30), QColor(0x44ff44), QColor(0xd8ffd0), QColor(0xff8800), QColor(0xffffff), QColor(0xff2a2a),
                    QColor(0x4488ff), QColor(0xff6600), QColor(0xcc44ff), QColor(0xffaa00), QColor(0xff66aa), QColor(), QColor(0, 0, 0, 217), 3.5f};
const Palette kLight{QColor(0xe3261c), QColor(0x0a8f2a), QColor(0x05561b), QColor(0xd96a00), QColor(0x10324a), QColor(0xe3261c),
                     QColor(0x1c5fd8), QColor(0xd65400), QColor(0x9a2ccc), QColor(0xb87800), QColor(0xd02c78), QColor(0x0a37a6), QColor(0, 0, 0, 153), 1.6f};

QColor segmentColour(const Palette &pal, const QString &segType)
{
    if (segType == "area"_L1) return pal.area;
    if (segType == "corridor"_L1) return pal.corridor;
    if (segType == "perimeter"_L1) return pal.perimeter;
    if (segType == "circle"_L1) return pal.circle;
    if (segType == "poi"_L1) return pal.poi;
    return pal.route;
}

QMutex g_missionLock;
Mission3DInput g_mission;
int g_missionEpoch = 0;
std::optional<LatLon> g_target;

GeoZonesPtr g_zones;
int g_zonesVersion = 0;
bool g_zonesOn = false;

double lowAltGridStrength(double h)
{
    if (!std::isfinite(h)) return 0;
    const double t = std::clamp((h - kTriGridFullAgl) / (kTriGridMaxAgl - kTriGridFullAgl), 0.0, 1.0);
    return 1 - t * t * (3 - 2 * t);
}

/// Sun azimuth (from north, clockwise) and elevation, radians (SunPosition.js; low-precision almanac).
void sunPosition(const QDateTime &utc, double lat, double lon, double *az, double *el)
{
    const double d = utc.toMSecsSinceEpoch() / 86400000.0 + 2440587.5 - 2451545.0;
    const double g = (357.529 + 0.98560028 * d) * kDeg;
    const double q = 280.459 + 0.98564736 * d;
    const double L = (q + 1.915 * std::sin(g) + 0.020 * std::sin(2 * g)) * kDeg;
    const double e = (23.439 - 0.00000036 * d) * kDeg;
    const double ra = std::atan2(std::cos(e) * std::sin(L), std::cos(L));
    const double dec = std::asin(std::sin(e) * std::sin(L));
    const double gmst = std::fmod(18.697374558 + 24.06570982441908 * d, 24.0);
    const double ha = (gmst * 15 + lon) * kDeg - ra;
    const double phi = lat * kDeg;
    *el = std::asin(std::sin(phi) * std::sin(dec) + std::cos(phi) * std::cos(dec) * std::cos(ha));
    *az = std::atan2(-std::sin(ha), std::tan(dec) * std::cos(phi) - std::sin(phi) * std::cos(ha));
}

QColor mix(const QColor &a, const QColor &b, double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return QColor::fromRgbF(float(a.redF() + (b.redF() - a.redF()) * t), float(a.greenF() + (b.greenF() - a.greenF()) * t),
                            float(a.blueF() + (b.blueF() - a.blueF()) * t));
}

QVector3D world(double lat, double lon, double y, LatLon origin)
{
    const GroundMeters g = latLonToMeters(lat, lon, origin);
    return {float(g.x), float(y), float(g.z)};
}

/// A closed polygon of pixel offsets around a world point: a symbol that keeps its size on screen.
void symbol(std::vector<LineSeg> &out, const QVector3D &c, const std::vector<QPointF> &px, const QColor &col, float width,
            float ghostOpacity = 0.45f)
{
    for (std::size_t i = 0; i < px.size(); ++i) {
        LineSeg s = LineSeg::make(c, c, col, width, ghostOpacity, width * 0.7f);
        const QPointF &p = px[i], &q = px[(i + 1) % px.size()];
        s.off[0] = float(p.x()); s.off[1] = float(p.y()); s.off[2] = float(q.x()); s.off[3] = float(q.y());
        out.push_back(s);
    }
}

std::vector<QPointF> circlePx(double r, int n = 24)
{
    std::vector<QPointF> v;
    for (int i = 0; i < n; ++i) v.emplace_back(r * std::cos(2 * kPi * i / n), r * std::sin(2 * kPi * i / n));
    return v;
}

/// Text labels (makeLabel / fitLabel of SymbolLayer.js): each one is painted once,
/// main text and a smaller second text with a dark halo, into an atlas the 3D
/// view samples. Full: everything is painted again from an empty atlas.
class LabelAtlas
{
public:
    static constexpr int kW = 1024, kH = 512;
    struct Slot {
        float u0, v0, u1, v1;
        float w, h;   // px
    };

    LabelAtlas() : m_img(kW, kH, QImage::Format_RGBA8888_Premultiplied) { m_img.fill(Qt::transparent); }

    std::optional<Slot> get(const QString &main, const QString &sub, const QColor &col, const QColor &halo, float haloPx, qreal dpr)
    {
        const QString key = main + QChar(1) + sub + QChar(1) + col.name(QColor::HexArgb) + halo.name(QColor::HexArgb) + QChar(1)
                          + QString::number(haloPx) + QChar(1) + QString::number(dpr, 'f', 2);
        if (const auto it = m_slots.constFind(key); it != m_slots.cend()) return it.value();

        QFont fm(u"Rajdhani"_s);
        fm.setPixelSize(std::max(1, qRound(15 * dpr)));
        fm.setWeight(QFont::DemiBold);
        QFont fs = fm;
        fs.setPixelSize(std::max(1, qRound(12 * dpr)));
        fs.setWeight(QFont::Medium);
        const QFontMetricsF mm(fm), ms(fs);
        const qreal pad = 4 * dpr, gap = sub.isEmpty() ? 0 : 5 * dpr;
        const qreal wMain = mm.horizontalAdvance(main), wSub = sub.isEmpty() ? 0 : ms.horizontalAdvance(sub);
        const int w = int(std::ceil(2 * pad + wMain + gap + wSub));
        const int h = int(std::ceil(22.5 * dpr));
        if (w > kW) return std::nullopt;
        if (m_x + w > kW) {
            m_x = 0;
            m_y += m_rowH + 1;
            m_rowH = 0;
        }
        if (m_y + h > kH) return std::nullopt;

        QPainter p(&m_img);
        p.setRenderHint(QPainter::Antialiasing);
        p.setCompositionMode(QPainter::CompositionMode_Source);
        p.fillRect(QRect(m_x, m_y, w, h), Qt::transparent);
        p.setCompositionMode(QPainter::CompositionMode_SourceOver);
        // textBaseline 'middle': the em box centred on the label
        const qreal mid = m_y + h / 2.0;
        QPainterPath mainPath, subPath;
        mainPath.addText(m_x + pad, mid + (mm.ascent() - mm.descent()) / 2.0, fm, main);
        if (!sub.isEmpty()) subPath.addText(m_x + pad + wMain + gap, mid + dpr + (ms.ascent() - ms.descent()) / 2.0, fs, sub);
        const QPen haloPen(halo, haloPx * dpr, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        p.strokePath(mainPath, haloPen);
        p.fillPath(mainPath, col);
        if (!sub.isEmpty()) {
            p.setOpacity(0.85);
            p.strokePath(subPath, haloPen);
            p.fillPath(subPath, col);
        }
        p.end();

        const Slot slot{float(m_x) / kW, float(m_y) / kH, float(m_x + w) / kW, float(m_y + h) / kH, float(w), float(h)};
        m_x += w + 1;
        m_rowH = std::max(m_rowH, h);
        m_slots.insert(key, slot);
        m_dirty = true;
        return slot;
    }

    void reset()
    {
        m_slots.clear();
        m_img.fill(Qt::transparent);
        m_x = m_y = m_rowH = 0;
        m_dirty = true;
    }
    bool takeDirty() { return std::exchange(m_dirty, false); }
    const QImage &image() const { return m_img; }

private:
    QImage m_img;
    QHash<QString, Slot> m_slots;
    int m_x = 0, m_y = 0, m_rowH = 0;
    bool m_dirty = false;
};

/// A label to the right of its point, centred on it (fitLabel).
struct LabelSpec {
    QVector3D anchor;
    QString main, sub;
    QColor color;
    float offset = 18;     // px from the point to the label
};

std::vector<QPointF> shapePx(mission3d::Shape shape, double r)
{
    switch (shape) {
    case mission3d::Shape::Triangle: return {{0, r}, {-0.87 * r, -0.5 * r}, {0.87 * r, -0.5 * r}};
    case mission3d::Shape::Diamond: return {{0, r}, {r, 0}, {0, -r}, {-r, 0}};
    case mission3d::Shape::Circle:
    case mission3d::Shape::Target: return circlePx(r, 20);
    default: break;
    }
    const double h = r * 0.8;
    return {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
}

/// An open stroke of pixel offsets around a world point.
void stroke(std::vector<LineSeg> &out, const QVector3D &c, QPointF a, QPointF b, const QColor &col, float width, float ghostOpacity = 0.45f)
{
    LineSeg s = LineSeg::make(c, c, col, width, ghostOpacity, width * 0.7f);
    s.off[0] = float(a.x()); s.off[1] = float(a.y()); s.off[2] = float(b.x()); s.off[3] = float(b.y());
    out.push_back(s);
}

/// The symbol of a mission mark; the one being flown larger and bolder.
void markSymbol(std::vector<LineSeg> &out, const mission3d::Mark &m, const QColor &col, bool active)
{
    if (m.shape == mission3d::Shape::None) return;
    const double r = active ? 9.5 : 7;
    const float w = active ? 4.5f : 2.5f;
    symbol(out, m.at, shapePx(m.shape, r), col, w);
    if (m.shape == mission3d::Shape::Target) {
        stroke(out, m.at, {-1.5 * r, 0}, {1.5 * r, 0}, col, w);
        stroke(out, m.at, {0, -1.5 * r}, {0, 1.5 * r}, col, w);
    }
}
} // namespace

void SceneItem::setMission(Mission3DInput mission)
{
    QMutexLocker l(&g_missionLock);
    g_mission = std::move(mission);
    ++g_missionEpoch;
}

void SceneItem::setTarget(std::optional<LatLon> target)
{
    QMutexLocker l(&g_missionLock);
    g_target = target;
}

namespace {
// Gamepad look buttons and the views they turn (GUI thread)
bool g_padUp = false, g_padDown = false, g_padLeft = false, g_padRight = false;
QSet<SceneItem *> g_items;
} // namespace

void SceneItem::setGamepadLook(bool up, bool down, bool left, bool right)
{
    if (up == g_padUp && down == g_padDown && left == g_padLeft && right == g_padRight) return;
    g_padUp = up;
    g_padDown = down;
    g_padLeft = left;
    g_padRight = right;
    for (SceneItem *it : std::as_const(g_items)) it->m_lookTimer.start();
}

bool SceneItem::lookUp() const { return m_keyUp || g_padUp; }
bool SceneItem::lookDown() const { return m_keyDown || g_padDown; }
bool SceneItem::lookLeft() const { return m_keyLeft || g_padLeft; }
bool SceneItem::lookRight() const { return m_keyRight || g_padRight; }

void SceneItem::setGeoZones(GeoZonesPtr zones, int version, bool on)
{
    // GUI thread, as tick() that reads them
    g_zones = std::move(zones);
    g_zonesVersion = version;
    g_zonesOn = on;
}

void SceneItem::refreshZoneRaster(double lat, double lon)
{
    if (!g_zonesOn || !g_zones || g_zones->empty()) {
        if (m_zoneRaster) {
            m_zoneRaster.reset();
            ++m_zoneRasterSerial;
        }
        m_zoneBuiltVersion = -1;
        return;
    }
    if (m_zoneBuilding) return;
    const bool moved = m_zoneRaster && flatDistance(m_zoneRaster->centerLat, m_zoneRaster->centerLon, lat, lon) > GeoZoneRaster::kRecenterKm * 1000;
    if (m_zoneRaster && m_zoneBuiltVersion == g_zonesVersion && !moved) return;
    m_zoneBuilding = true;
    const GeoZonesPtr zones = g_zones;
    const int version = g_zonesVersion;
    auto *w = new QFutureWatcher<std::shared_ptr<GeoZoneRaster>>(this);
    connect(w, &QFutureWatcher<std::shared_ptr<GeoZoneRaster>>::finished, this, [this, w, version] {
        w->deleteLater();
        m_zoneBuilding = false;
        m_zoneRaster = w->result();
        m_zoneBuiltVersion = version;
        ++m_zoneRasterSerial;
        update();
    });
    w->setFuture(QtConcurrent::run([zones, lat, lon, version] { return GeoZoneRaster::build(*zones, lat, lon, version); }));
}

class SceneRenderer : public QQuickRhiItemRenderer
{
public:
    void initialize(QRhiCommandBuffer *) override
    {
        if (!m_terrain) {
            m_terrain = std::make_unique<TerrainRenderer>(rhi());
            m_overlay = std::make_unique<OverlayRenderer>(rhi());
            m_outline = std::make_unique<OutlineRenderer>(rhi());
            m_points = std::make_unique<PointCloudRenderer>(rhi());
            m_ros = std::make_unique<RosSurfaceRenderer>(rhi());
            m_video = std::make_unique<VideoRenderer>(rhi());
            m_api = QString::fromLatin1(rhi()->backendName());
            LOG_I("gpu") << "3D view on" << m_api << rhi()->driverInfo().deviceName;
        }
    }

    void synchronize(QQuickRhiItem *item) override
    {
        auto *it = static_cast<SceneItem *>(item);
        m_item = it;
        const double syncStart = monoMs();
        m_syncMs = 0;
        if (!SceneItem::s_hub) return;
        const auto s = SceneItem::s_hub->snapshot();
        const std::int64_t now = wallMs();
        // The pose of the vblank the frame is shown at, the instant the HUD took it for (paced)
        const Pose pose = it->m_smoother.update(PoseSmoother::fromState(*s), SceneItem::frameTimeMs(), it->m_tau);
        const LatLon origin = s->sceneOrigin;
        const int epoch = s->sceneOriginEpoch;
        // AR draws the schematic view in its dark colours, whatever is chosen (setARMode)
        m_videoMode = it->m_videoMode;
        // Relative navigation (SYS CONFIG -> NAVIGATION): positions are metres from a synthetic
        // origin, so no terrain, imagery, traffic or zones, but a plane at zero (applyRelativeFrame)
        const bool relative = s->relativeMode;
        m_relative = relative;
        const bool ar = m_videoMode == 2, schematic = it->m_schematic || ar || relative;
        const bool light = it->m_lightTheme && schematic && !ar;
        const Palette &pal = light ? kLight : kDark;
        const int clamp = relative ? 0 : it->m_clamp;
        HgtStore &hgt = HgtStore::instance();
        auto ground = [&hgt, relative](double lat, double lon) { return relative ? std::optional<double>() : hgt.elevation(lat, lon); };
        auto waterAt = [&hgt, relative](double lat, double lon) { return relative ? std::optional<HgtStore::Water>() : hgt.waterAt(lat, lon); };

        // Pixels of the target per point: the display's, or fewer (RESOLUTION)
        m_dpr = float(it->m_pixelRatio);
        m_frame.brightness = float(it->m_brightness);
        m_frame.fovDeg = float(it->m_fov);
        m_frame.schematic = schematic;
        m_frame.sunlight = it->m_sunlight;
        m_frame.lightTheme = it->m_lightTheme && !ar;
        m_frame.fogDensity = ar ? 0.f : float(1.75 / it->m_terrain.radius());
        m_changes = &it->m_terrain.changes();
        if (m_terrain && !m_formatReported) {
            it->m_terrain.imagery().setCompressed(m_terrain->bc1Supported());
            m_formatReported = true;
        }

        // ── The vehicle: where it is drawn ──
        const GroundMeters gm = latLonToMeters(pose.lat, pose.lon, origin);
        double alt = pose.alt + s->offsetAlt;
        const auto terr = ground(pose.lat, pose.lon);
        auto water = waterAt(pose.lat, pose.lon);
        bool synthetic = false;
        // A sub on water the elevation data does not show: the surface it started on
        if (!relative && !water && s->vehicleType == 12 && s->subSurfaceAlt) {
            water = HgtStore::Water{*s->subSurfaceAlt, std::nullopt};
            synthetic = true;
        }
        // Water is no floor (a boat sits on it, a ROV dives under it); the sea bed is
        const std::optional<double> floor = water ? water->bed : terr;
        if (clamp >= 1 && floor && alt < *floor + 0.5) alt = *floor + 0.5;
        // Above land the camera and the model never go below 1 m MSL
        if (!water && !relative) alt = std::max(alt, 1.0);
        const QVector3D P(float(gm.x), float(alt), float(gm.z));
        const double agl = terr ? alt - *terr : 1e9;
        m_frame.triGrid = float(agl <= kTriGridFullAgl ? 1.0 : agl >= kTriGridMaxAgl ? 0.0 : (kTriGridMaxAgl - agl) / (kTriGridMaxAgl - kTriGridFullAgl));
        m_frame.triCenter = P;

        QMatrix4x4 attitude;
        attitude.rotate(float(-pose.yaw * kRad), 0, 1, 0);
        attitude.rotate(float(pose.pitch * kRad), 1, 0, 0);
        attitude.rotate(float(-pose.roll * kRad), 0, 0, 1);
        m_overlayFrame.model.setToIdentity();
        m_overlayFrame.model.translate(P);
        m_overlayFrame.model *= attitude;
        m_overlayFrame.vehicle = it->m_thirdPerson;

        // ── Camera ──
        QVector3D cam = P;
        m_view.setToIdentity();
        if (it->m_thirdPerson) {
            const auto &o = it->m_orbit;
            const double lift = o.height * std::min(1.0, o.distance / 50);
            cam = P + QVector3D(float(o.distance * std::cos(o.pitch) * std::sin(o.yaw)), float(o.distance * std::sin(o.pitch) + lift),
                                float(o.distance * std::cos(o.pitch) * std::cos(o.yaw)));
            // Orbiting far out of a valley put the camera inside the mountains: keep it over the ground
            const LatLon cll = metersToLatLon(cam.x(), cam.z(), origin);
            const auto camWater = waterAt(cll.lat, cll.lon);
            const auto camGround = camWater ? camWater->bed : ground(cll.lat, cll.lon);
            if (camGround && cam.y() < *camGround + kChaseCamClearanceM) cam.setY(float(*camGround + kChaseCamClearanceM));
            m_view.lookAt(cam, P + QVector3D(0, float(lift * 0.2), 0), QVector3D(0, 1, 0));
        } else {
            QMatrix4x4 w;
            w.translate(P);
            w.rotate(float(-pose.yaw * kRad), 0, 1, 0);
            w.rotate(float((it->m_horizonLock ? 0.0 : pose.pitch) * kRad), 1, 0, 0);
            w.rotate(float(-pose.roll * kRad), 0, 0, 1);
            // Look-around, in the body frame: "up" is over the canopy, "left" over the wing
            w.rotate(float(it->m_look.yaw * kRad), 0, 1, 0);
            w.rotate(float(it->m_look.pitch * kRad), 1, 0, 0);
            m_view = w.inverted();
        }
        m_frame.camPos = cam;
        m_overlayFrame.camPos = cam;
        m_sunDir = it->m_sunlight ? it->m_sunDirWorld : QVector3D(-0.35f, 0.75f, 0.45f).normalized();
        m_overlayFrame.sunDir = m_sunDir;
        m_frame.sunDir = m_sunDir;

        // ── LiDAR point cloud (LidarCloud.js): the map from its own origin, the
        // live scan riding on the vehicle (level, or turned with it in the body frame) ──
        if (m_points) {
            const PointCloudData &pc = PointCloudData::instance();
            QMatrix4x4 mapModel, liveModel;
            if (pc.hasOrigin) {
                const GroundMeters b = latLonToMeters(pc.origin.lat, pc.origin.lon, origin);
                mapModel.translate(float(b.x), float(pc.origin.alt + s->offsetAlt), float(b.z));
                mapModel.scale(float(kMetersPerDegLat * std::cos(origin.lat * kDeg) / pc.origin.mPerLon), 1,
                               float(kMetersPerDegLat / pc.origin.mPerLat));
            }
            liveModel.translate(float(gm.x), float(std::max(pose.alt + s->offsetAlt, 1.0)), float(gm.z));
            if (pc.liveBody) liveModel *= attitude;
            m_points->sync(pc, mapModel, liveModel, m_dpr);
        }
        // ── ROS surface (RosMesh3D.js, RosVolume3D.js): levels of detail for this camera ──
        if (m_ros) {
            RosSurfaceRenderer::Camera rc;
            const double h = std::max(1.0, it->height() * m_dpr);
            QMatrix4x4 proj;
            proj.perspective(m_frame.fovDeg, float(std::max(1.0, it->width()) / std::max(1.0, it->height())), float(kNear), float(kFar));
            rc.viewProj = proj * m_view;
            rc.pos = cam;
            rc.focalPx = float(h / (2 * std::tan(m_frame.fovDeg * kDeg / 2)));
            m_ros->sync(RosSurfaceData::instance(), origin, s->offsetAlt, QVector3D(float(gm.x), float(pose.alt + s->offsetAlt), float(gm.z)), rc);
        }

        // ── Satellite imagery at full resolution around the aircraft ──
        {
            const double viewportPx = std::max(1.0, it->height() * m_dpr);
            const double pixelAngle = it->m_fov * kDeg / viewportPx;
            it->m_detail.update(pose.lat, pose.lon, origin, cam, P, terr, pixelAngle, !schematic, m_detailOps, m_frame.detail);
        }

        // ── Trail ──
        syncTrail(it, pose, alt, origin, epoch, pal);

        // ── Overlays rebuilt every frame ──
        std::vector<LineSeg> &L = m_overlayFrame.lines;
        L.clear();
        m_overlayFrame.fill.clear();

        // Predicted trajectory corridor, every third frame (computePredictedPath is the cost)
        if (it->m_trajectory) {
            if (++m_corridorFrame % 3 == 0 || m_corridorLines.empty())
                buildCorridor(it, *s, pose, origin, clamp, pal, now);
            L.insert(L.end(), m_corridorLines.begin(), m_corridorLines.end());
            m_overlayFrame.fill = m_corridorFill;
        } else {
            m_corridorLines.clear();
            m_corridorFill.clear();
        }

        // Mission route
        syncMission(*s, origin, epoch, clamp, pal);

        // Home: a pole from the ground, a ring on it and the symbol on top. Before the vehicle
        // has reported it (while planning), the mission's take-off point
        m_homeBase.reset();
        if (s->homeLat && s->homeLon) m_homeBase = homeBase(*s->homeLat, *s->homeLon, s->homeAlt.value_or(0) + s->offsetAlt, relative, origin);
        else if (m_missionHome && !relative) m_homeBase = homeBase(m_missionHome->lat, m_missionHome->lon, m_missionHome->altMsl + s->offsetAlt, relative, origin);
        if (m_homeBase) {
            const QVector3D base = *m_homeBase;
            const QVector3D top = base + QVector3D(0, float(kHomePoleM), 0);
            L.push_back(LineSeg::make(base, top, pal.home, 3, 0.4f, 2));
            const int n = 48;
            for (int i = 0; i < n; ++i) {
                const double a0 = 2 * kPi * i / n, a1 = 2 * kPi * (i + 1) / n;
                L.push_back(LineSeg::make(base + QVector3D(float(kHomeRingM * std::cos(a0)), 0.5f, float(kHomeRingM * std::sin(a0))),
                                          base + QVector3D(float(kHomeRingM * std::cos(a1)), 0.5f, float(kHomeRingM * std::sin(a1))),
                                          pal.home, 3, 0.4f, 2));
            }
            symbol(L, top, circlePx(11), pal.home, 3);
            symbol(L, top, {{-5, -4}, {0, 4}, {5, -4}}, pal.home, 2.5f);
        }

        L.insert(L.end(), m_missionLines.begin(), m_missionLines.end());
        if (!m_missionDraw.lines.empty()) {
            // The item being flown (MISSION_CURRENT): the path that leads to it, and its symbol
            for (const mission3d::Line &l : m_missionDraw.lines)
                if (l.seq == s->missionCurrentSeq) L.push_back(LineSeg::make(l.a, l.b, pal.activeLeg, 4.5f));
            for (const mission3d::Mark &m : m_missionDraw.marks)
                if (m.seq == s->missionCurrentSeq) markSymbol(L, m, segmentColour(pal, m.segType), true);
        }

        // Target (GCS OPTIONS -> TARGET): a pole, a sphere on it and a ring
        std::optional<LatLon> target;
        {
            QMutexLocker lock(&g_missionLock);
            target = g_target;
        }
        if (target && !relative) {
            const double y = ground(target->lat, target->lon).value_or(0);
            const QVector3D base = world(target->lat, target->lon, y, origin);
            const QVector3D top = base + QVector3D(0, float(kTargetPoleM), 0), ball = top + QVector3D(0, float(kTargetSphereM / 2), 0);
            const QColor red(0xff0000), ringRed(0xff4444);
            L.push_back(LineSeg::make(base, top, red, 4, 0.5f, 2.5f));
            const int n = 32;
            auto circle = [&](double r, const QVector3D &u, const QVector3D &v, const QColor &c, float w) {
                for (int i = 0; i < n; ++i) {
                    const double a0 = 2 * kPi * i / n, a1 = 2 * kPi * (i + 1) / n;
                    L.push_back(LineSeg::make(ball + (u * float(std::cos(a0)) + v * float(std::sin(a0))) * float(r),
                                              ball + (u * float(std::cos(a1)) + v * float(std::sin(a1))) * float(r), c, w, 0.4f, w * 0.6f));
                }
            };
            const QVector3D ex(1, 0, 0), ey(0, 1, 0), ez(0, 0, 1);
            circle(kTargetSphereM, ex, ey, red, 2.5f);
            circle(kTargetSphereM, ez, ey, red, 2.5f);
            circle(kTargetSphereM, ex, ez, red, 2.5f);
            circle(kTargetRingM, ex, ez, ringRed, 4);
            symbol(L, ball, circlePx(6), red, 6);   // what is left of it far away
        }

        // ADS-B traffic: a circle on each aircraft, larger the closer it is, dead-reckoned
        // between its reports and trailing the path it flew
        syncTraffic(*s, cam, origin, relative, pal, now);
        L.insert(L.end(), m_trafficLines.begin(), m_trafficLines.end());
        for (const TrafficMark &t : m_trafficMarks) {
            const double d = std::clamp(double((t.at - cam).length()), kTrafficNearM, kTrafficFarM);
            const double k = std::log(d / kTrafficNearM) / std::log(kTrafficFarM / kTrafficNearM);
            const double r = kTrafficNearPx + (kTrafficFarPx - kTrafficNearPx) * k;
            symbol(L, t.at, circlePx(r), pal.traffic, 3, 0.5f);
            symbol(L, t.at, circlePx(2, 8), pal.traffic, 4, 0.5f);
        }

        // LTE cell sites (Cell3D.js): a thin red pole 10 m tall on the terrain at each site
        // the mini-map shows, and in the chase view a dashed line from the vehicle to the
        // top of the serving one, in the colour of the link quality
        if (const CellOverlayPtr cells = cellOverlay::get(); cells && cells->live && !s->relativeMode) {
            for (std::size_t i = 0; i < cells->sites.size(); ++i) {
                const CellSite &site = cells->sites[i];
                const auto ground = hgt.elevation(site.lat, site.lon);
                if (!ground) continue;   // the tile is not in yet: next frame
                const QVector3D base = world(site.lat, site.lon, *ground + s->offsetAlt, origin);
                const QVector3D top = base + QVector3D(0, float(kCellPoleM), 0);
                L.push_back(LineSeg::make(base, top, QColor(0xff2020), 2));
                if (int(i) == cells->serving && it->m_thirdPerson) {
                    LineSeg link = LineSeg::make(P, top, cells->linkColor, 2, 0.35f, 1.5f);
                    link.style[3] = 10;   // dashes of 10 px, gaps of 7
                    L.push_back(link);
                }
            }
        }

        // Chase view: a ring marks the aircraft, the true-scale model shrinks to a few pixels
        if (it->m_thirdPerson && it->m_orbit.distance > 120) symbol(L, P, circlePx(15), pal.ownship, 2.5f, 0.5f);

        // ── Labels: HOME, the mission items, the traffic ──
        buildLabels(it, *s, origin, cam, s->rawAlt, pal);

        // ── Water ──
        m_overlayFrame.water = false;
        if (water && schematic) {
            const double strength = lowAltGridStrength(alt - water->level);
            const LatLon cll = metersToLatLon(cam.x(), cam.z(), origin);
            const auto camW = waterAt(cll.lat, cll.lon);
            const bool camUnder = camW && cam.y() < camW->level;
            if (strength > 0 && (camUnder || water->bed || synthetic)) {
                m_overlayFrame.water = true;
                m_overlayFrame.waterX = P.x();
                m_overlayFrame.waterZ = P.z();
                m_overlayFrame.waterLevel = float(water->level);
                m_overlayFrame.waterRadius = float(kWaterPlaneRadius);
                m_overlayFrame.waterCell = float(kWaterPlaneCell);
                m_overlayFrame.waterTriangles = true;
                m_overlayFrame.waterColor = QColor(0x2a7fff);
                m_overlayFrame.waterOpacity = float(0.6 * strength);
            }
        }
        if (relative) {
            // Squares of 50 m on zero, 3 km around the camera; specks below it
            m_overlayFrame.water = true;
            m_overlayFrame.waterX = cam.x();
            m_overlayFrame.waterZ = cam.z();
            m_overlayFrame.waterLevel = float(s->offsetAlt);
            m_overlayFrame.waterRadius = float(kZeroPlaneRadius);
            m_overlayFrame.waterCell = float(kZeroPlaneCell);
            m_overlayFrame.waterTriangles = false;
            m_overlayFrame.waterColor = kZeroPlaneColour;
            m_overlayFrame.waterOpacity = kZeroPlaneOpacity;
            m_overlayFrame.underwater = cam.y() < s->offsetAlt;
            m_overlayFrame.underwaterLevel = float(s->offsetAlt);
        } else {
            const LatLon cll = metersToLatLon(cam.x(), cam.z(), origin);
            auto camW = waterAt(cll.lat, cll.lon);
            if (!camW && synthetic) camW = water;
            m_overlayFrame.underwater = camW && cam.y() < camW->level;
            m_overlayFrame.underwaterLevel = camW ? float(camW->level) : 0.f;
        }

        // Model of the vehicle type
        if (it->m_modelVersion != m_modelVersion) {
            m_modelVersion = it->m_modelVersion;
            m_pendingModel = it->m_model;
            m_modelPending = true;
        }

        m_sky = ar ? QColor(0, 0, 0) : skyColour(it, schematic);

        // FPV camera image
        if (m_video) {
            quint64 serial = 0;
            VideoFramePtr f = m_videoMode ? VideoMailbox::instance().latest(&serial) : nullptr;
            m_video->sync(std::move(f), serial);
        }

        // UAS zones on the terrain: the raster's window in today's world metres
        if (it->m_zoneRasterSerial != m_zoneSerial) {
            m_zoneSerial = it->m_zoneRasterSerial;
            m_pendingZones = it->m_zoneRaster;
            m_zonesPending = true;
        }
        const auto &zr = it->m_zoneRaster;
        m_frame.zonesOn = zr && !zr->empty && !relative;
        if (zr) {
            const GroundMeters nw = latLonToMeters(zr->lat1, zr->lon0, origin);
            const double mPerLon = kMetersPerDegLat * std::cos(origin.lat * kDeg);
            m_frame.zoneMap = QVector4D(float(nw.x), float(nw.z), float(1.0 / ((zr->lon1 - zr->lon0) * mPerLon)),
                                        float(1.0 / ((zr->lat1 - zr->lat0) * kMetersPerDegLat)));
        }

        // Stats back to the item once a second
        if (m_statClock.isValid() && m_statClock.elapsed() >= 1000) {
            SceneItem::Stats st{m_terrain ? m_terrain->chunkCount() : 0, m_terrain ? m_terrain->drawCalls() : 0,
                                m_terrain ? m_terrain->visibleChunks() : 0, m_frames * 1000.0 / m_statClock.elapsed(), m_api,
                                m_terrain ? m_terrain->imageCount() : 0, m_terrain ? m_terrain->imageBytes() / 1048576.0 : 0};
            m_frames = 0;
            m_statClock.restart();
            QMetaObject::invokeMethod(it, [it, st] { it->reportStats(st); }, Qt::QueuedConnection);
        }
        if (!m_statClock.isValid()) m_statClock.start();
        m_syncMs = monoMs() - syncStart;
    }

    void render(QRhiCommandBuffer *cb) override
    {
        const double renderStart = monoMs();
        ++m_frames;
        QRhiResourceUpdateBatch *u = rhi()->nextResourceUpdateBatch();
        if (m_changes) m_terrain->sync(*m_changes, u);
        m_terrain->applyDetail(m_detailOps, u);
        m_detailOps.clear();
        if (m_zonesPending) {
            if (m_pendingZones) m_terrain->setGeoZones(m_pendingZones, u);
            m_pendingZones.reset();
            m_zonesPending = false;
        }
        if (m_modelPending) {
            m_overlay->setModel(m_pendingModel, u);
            m_pendingModel.reset();
            m_modelPending = false;
        }
        if (m_atlas.takeDirty()) m_overlay->setLabelAtlas(m_atlas.image(), u);
        if (m_trailResetPending) {
            m_overlay->trailReset(m_trailSegs, u);
            m_trailResetPending = false;
            m_trailSegs.clear();
        } else if (!m_trailSegs.empty()) {
            m_overlay->trailAppend(m_trailSegs, u);
            m_trailSegs.clear();
        }

        const QSize out = renderTarget()->pixelSize();
        QMatrix4x4 proj = rhi()->clipSpaceCorrMatrix();
        proj.perspective(m_frame.fovDeg, float(out.width()) / std::max(1, out.height()), float(kNear), float(kFar));
        m_frame.viewProj = proj * m_view;
        m_frame.viewportHeight = float(out.height());
        // The schematic view outlines its ridges and skyline (OutlineRenderer): the
        // terrain goes into a target of its own first
        m_video->prepare(out, m_videoMode == 2, renderTarget()->renderPassDescriptor(), renderTarget()->sampleCount(), u);
        // The camera alone: drawing the 3D under it would be thrown away
        if (m_videoMode == 1) {
            cb->beginPass(renderTarget(), QColor(0, 0, 0), {1.0f, 0}, u);
            m_video->draw(cb, out);
            cb->endPass();
            endFrame(renderStart);
            return;
        }
        const bool outlines = m_frame.schematic && m_outline->supported();
        QRhiTextureRenderTarget *terrainTarget = outlines ? m_outline->target(out) : nullptr;
        if (outlines) {
            m_terrain->prepare(m_frame, terrainTarget->renderPassDescriptor(), 1, u);
            OutlineRenderer::Params op;
            op.viewProj = m_frame.viewProj;
            op.camPos = m_frame.camPos;
            op.fogDensity = m_frame.fogDensity;
            op.nearPlane = float(kNear);
            op.farPlane = float(kFar);
            op.brightness = m_frame.brightness;
            op.light = m_frame.lightTheme;
            op.dpr = m_dpr;
            m_outline->prepare(op, renderTarget()->renderPassDescriptor(), renderTarget()->sampleCount(), u);
        } else {
            m_terrain->prepare(m_frame, renderTarget()->renderPassDescriptor(), renderTarget()->sampleCount(), u);
        }
        m_overlay->prepare(m_overlayFrame, m_frame.viewProj, out, renderTarget()->renderPassDescriptor(), renderTarget()->sampleCount(), u);
        m_ros->prepare(m_frame.viewProj, renderTarget()->renderPassDescriptor(), renderTarget()->sampleCount(), u);
        m_points->prepare(m_frame.viewProj, out, renderTarget()->renderPassDescriptor(), renderTarget()->sampleCount(), u);

        if (outlines) {
            cb->beginPass(terrainTarget, QColor(0, 0, 0), {1.0f, 0}, u);
            if (!m_relative) m_terrain->draw(cb, out);
            cb->endPass();
            cb->beginPass(renderTarget(), m_sky, {1.0f, 0});
            m_outline->draw(cb, out);
        } else {
            cb->beginPass(renderTarget(), m_sky, {1.0f, 0}, u);
            if (!m_relative) m_terrain->draw(cb, out);
        }
        // The solid ROS surface before the vehicle, so the vehicle stays in front of the bed it floats over
        m_ros->drawOpaque(cb, out);
        m_overlay->drawOpaque(cb, out);
        m_ros->drawTranslucent(cb, out);
        m_points->draw(cb, out);
        m_overlay->drawTranslucent(cb, out);
        if (m_videoMode == 2) m_video->draw(cb, out);   // AR: the camera shows through the black
        cb->endPass();
        endFrame(renderStart);
    }

private:
    void endFrame(double renderStart)
    {
        const double drawn = monoMs();
        FrameStats::instance().frame(renderStart, m_syncMs, drawn - renderStart);
        // Keep rendering, the pose moves every frame: the next frame at once or on the cap's schedule
        if (m_item)
            QMetaObject::invokeMethod(m_item, [i = m_item, drawn] {
                FrameStats::instance().gui(monoMs() - drawn);
                i->frameDrawn();
            }, Qt::QueuedConnection);
    }

    QColor skyColour(const SceneItem *it, bool schematic) const
    {
        if (schematic) return it->m_lightTheme ? QColor::fromRgbF(0.78f, 0.86f, 0.93f) : QColor(0, 0, 0);
        const QColor day = QColor::fromRgbF(0.62f, 0.74f, 0.86f);
        if (!it->m_sunlight) return day;
        // Realistic sunlight: the sky follows the sun down through dusk to night
        const double el = it->m_sunEl * kRad;
        const QColor dusk(0xd8, 0x8a, 0x5c), night(0x0b, 0x12, 0x20);
        if (el >= 8) return day;
        if (el >= 0) return mix(dusk, day, el / 8);
        return mix(dusk, night, -el / 8);
    }

    void buildLabels(SceneItem *it, const VehicleState &s, LatLon origin, const QVector3D &cam, double vehicleAlt, const Palette &pal)
    {
        std::vector<LabelSpec> specs;
        HgtStore &hgt = HgtStore::instance();
        if (m_homeBase)
            specs.push_back({*m_homeBase + QVector3D(0, float(kHomePoleM), 0), u"HOME"_s, {}, pal.label.isValid() ? pal.label : pal.home, kHomeLabelPx});
        // Mission items: built with the route
        specs.insert(specs.end(), m_missionLabels.begin(), m_missionLabels.end());
        // Traffic: callsign, height relative to us in steps of 10 m and a trend arrow (Traffic3D.updateLabel)
        for (const TrafficMark &t : m_trafficMarks) {
            const int rel = int(std::lround((t.alt - vehicleAlt) / 10.0)) * 10;
            QString sub = (rel >= 0 ? u"+"_s : u"\u2212"_s) + QString::number(std::abs(rel)) + u" m"_s;
            if (t.vertRate > 2) sub += u" \u2191"_s;
            else if (t.vertRate < -2) sub += u" \u2193"_s;
            specs.push_back({t.at, t.name, sub, pal.label.isValid() ? pal.label : pal.traffic, kTrafficLabelPx});
        }

        const qreal dpr = m_dpr;
        auto resolve = [&] {
            m_overlayFrame.labels.clear();
            for (const LabelSpec &sp : specs) {
                const auto slot = m_atlas.get(sp.main, sp.sub, sp.color, pal.labelHalo, pal.labelHaloPx, dpr);
                if (!slot) return false;
                LabelInst li;
                li.anchor[0] = sp.anchor.x(); li.anchor[1] = sp.anchor.y(); li.anchor[2] = sp.anchor.z(); li.anchor[3] = 1;
                li.rect[0] = std::round(sp.offset * float(dpr));
                li.rect[1] = std::floor(-slot->h / 2);
                li.rect[2] = slot->w;
                li.rect[3] = slot->h;
                li.uv[0] = slot->u0; li.uv[1] = slot->v0; li.uv[2] = slot->u1; li.uv[3] = slot->v1;
                m_overlayFrame.labels.push_back(li);
            }
            return true;
        };
        if (!resolve()) {
            // The atlas is full: start it again with what is on screen now
            m_atlas.reset();
            if (!resolve()) m_overlayFrame.labels.clear();
        }
    }

    void syncTrail(SceneItem *it, const Pose &pose, double alt, LatLon origin, int epoch, const Palette &pal)
    {
        auto &tr = it->m_trail;
        const QVector3D here = world(pose.lat, pose.lon, alt, origin);
        bool add = true;
        if (!tr.empty()) {
            const QVector3D last = world(tr.back().lat, tr.back().lon, tr.back().alt, origin);
            const float d2 = (here - last).lengthSquared();
            if (d2 > kTrailMaxJumpM * kTrailMaxJumpM) {
                // Not flight: a new connection after the demo, another vehicle, a log loaded
                tr.clear();
                ++it->m_trailVersion;
            } else if (d2 < kTrailMinStepM * kTrailMinStepM) {
                add = false;   // hovering or parked: no duplicates
            }
        }
        if (add) {
            if (tr.size() >= kMaxTrailPoints) {
                std::size_t w = 0;
                for (std::size_t r = 0; r < tr.size(); r += 2) tr[w++] = tr[r];
                tr.resize(w);
                ++it->m_trailVersion;
            }
            tr.push_back({pose.lat, pose.lon, float(alt)});
        }
        const bool paletteChanged = pal.trail != m_trailColour;
        auto seg = [&](std::size_t i) {
            return LineSeg::make(world(tr[i - 1].lat, tr[i - 1].lon, tr[i - 1].alt, origin), world(tr[i].lat, tr[i].lon, tr[i].alt, origin),
                                 pal.trail, 3.5f, 0.35f, 2.f);
        };
        if (it->m_trailVersion != m_trailVersion || epoch != m_trailEpoch || paletteChanged) {
            m_trailVersion = it->m_trailVersion;
            m_trailEpoch = epoch;
            m_trailColour = pal.trail;
            m_trailSegs.clear();
            for (std::size_t i = 1; i < tr.size(); ++i) m_trailSegs.push_back(seg(i));
            m_trailSent = tr.size();
            m_trailResetPending = true;
        } else if (tr.size() > m_trailSent) {
            for (std::size_t i = std::max<std::size_t>(1, m_trailSent); i < tr.size(); ++i) m_trailSegs.push_back(seg(i));
            m_trailSent = tr.size();
        }
    }

    void buildCorridor(SceneItem *it, const VehicleState &s, const Pose &pose, LatLon origin, int clamp, const Palette &pal, std::int64_t now)
    {
        m_corridorLines.clear();
        m_corridorFill.clear();
        const double predT = TrajectoryPredictor::predictionTime(s.gs);
        const auto path = it->m_predictor.predict(s, pose.lat, pose.lon, pose.alt + s.offsetAlt, kCorridorPoints, predT, now);
        if (path.size() < 2) return;
        std::vector<QVector3D> pts;
        pts.reserve(path.size());
        for (const auto &p : path) {
            double y = p.alt;
            if (clamp >= 2) {
                if (const auto g = HgtStore::instance().elevation(p.lat, p.lon); g && y < *g + 1) y = *g + 1;
            }
            pts.push_back(world(p.lat, p.lon, y, origin));
        }
        const int n = int(pts.size());
        const QVector3D d0 = pts[1] - pts[0];
        const float l0 = std::max(1e-3f, std::hypot(d0.x(), d0.z()));
        const float backX = -(d0.x() / l0) * float(kStartOffset), backZ = -(d0.z() / l0) * float(kStartOffset);
        QColor col = pal.route;
        std::vector<QVector3D> left(n), right(n);
        std::vector<float> fade(n);
        double nextTick = kTickIntervalS;
        for (int i = 0; i < n; ++i) {
            const QVector3D d = i < n - 1 ? pts[i + 1] - pts[i] : pts[i] - pts[i - 1];
            const float len = std::max(1e-3f, std::hypot(d.x(), d.z()));
            const float px = -d.z() / len, pz = d.x() / len;   // starboard
            const float y = pts[i].y() - float(kAltDrop);
            const float cx = pts[i].x() + backX, cz = pts[i].z() + backZ;
            const double along = double(i) / (n - 1);
            const double hw = kHalfWidthNear + (kHalfWidthFar - kHalfWidthNear) * along;
            const double bank = path[std::size_t(i)].bank;
            const float wH = float(hw * std::cos(bank)), wV = float(hw * std::sin(bank));
            left[i] = QVector3D(cx + px * wH, y - wV, cz + pz * wH);
            right[i] = QVector3D(cx - px * wH, y + wV, cz - pz * wH);
            const double fadeOut = 1 - (1 - kEndAlpha) * along;
            const double fadeIn = std::min(i / 4.0, 1.0);
            fade[i] = float(fadeOut * fadeIn);
            if (path[std::size_t(i)].t >= nextTick - 1e-6) {
                m_corridorLines.push_back(LineSeg::make(left[i], right[i], col, 2.5f, 0, 0, kBorderAlpha * fade[i]));
                nextTick += kTickIntervalS;
            }
        }
        for (int i = 1; i < n; ++i) {
            const float a = kBorderAlpha * 0.5f * (fade[i - 1] + fade[i]);
            m_corridorLines.push_back(LineSeg::make(left[i - 1], left[i], col, 3, 0, 0, a));
            m_corridorLines.push_back(LineSeg::make(right[i - 1], right[i], col, 3, 0, 0, a));
            auto v = [&](const QVector3D &p, float f) {
                return FillVertex{{p.x(), p.y(), p.z()}, {float(col.redF()), float(col.greenF()), float(col.blueF()), kFillAlpha * f}};
            };
            const FillVertex l0v = v(left[i - 1], fade[i - 1]), l1v = v(left[i], fade[i]), r0v = v(right[i - 1], fade[i - 1]), r1v = v(right[i], fade[i]);
            m_corridorFill.insert(m_corridorFill.end(), {l0v, l1v, r1v, l0v, r1v, r0v});
        }
    }

    /// The foot of the home marker: on the terrain where it is known.
    static QVector3D homeBase(double lat, double lon, double fallback, bool relative, LatLon origin)
    {
        const auto hg = relative ? std::optional<double>() : HgtStore::instance().elevation(lat, lon);
        return world(lat, lon, hg.value_or(fallback) + 0.5, origin);
    }

    void syncMission(const VehicleState &s, LatLon origin, int epoch, int clamp, const Palette &pal)
    {
        // A fixed wing, and the VTOL types, fly NAV_LAND down a slope
        const bool glide = s.vehicleType == 1 || (s.vehicleType >= 19 && s.vehicleType <= 25);
        {
            QMutexLocker l(&g_missionLock);
            if (g_missionEpoch == m_missionEpoch && epoch == m_missionOriginEpoch && clamp == m_missionClamp && pal.route == m_missionColour
                && s.offsetAlt == m_missionOffset && s.relativeMode == m_missionRelative && glide == m_missionGlide && !m_missionWaitingGround)
                return;
            m_mission = g_mission;
            m_missionEpoch = g_missionEpoch;
        }
        m_missionOriginEpoch = epoch;
        m_missionClamp = clamp;
        m_missionOffset = s.offsetAlt;
        m_missionRelative = s.relativeMode;
        m_missionGlide = glide;
        m_missionColour = pal.route;
        m_missionWaitingGround = false;
        m_missionHome.reset();
        HgtStore &hgt = HgtStore::instance();
        auto groundOf = [&](const std::optional<double> &stated, double lat, double lon) {
            const std::optional<double> g = s.relativeMode ? std::optional<double>() : stated ? stated : hgt.elevation(lat, lon);
            if (!g && !s.relativeMode) m_missionWaitingGround = true;   // try again when the tile is in
            return g ? std::optional<float>(float(*g)) : std::nullopt;
        };
        std::vector<mission3d::Point> pts;
        pts.reserve(m_mission.points.size());
        for (const MissionPoint3D &p : m_mission.points) {
            if (p.kind == "home"_L1 && !m_missionHome) m_missionHome = p;
            mission3d::Point q;
            q.ground = groundOf(p.ground, p.lat, p.lon);
            double y = p.altMsl + s.offsetAlt;   // ALTITUDE OFFSET: the route stays with the vehicle flying it
            if (clamp >= 2 && q.ground && y < *q.ground + 1) y = *q.ground + 1;
            q.pos = world(p.lat, p.lon, y, origin);
            q.seq = p.seq;
            q.kind = p.kind;
            q.derived = p.derived;
            q.segId = p.segId;
            q.segType = p.segType;
            q.loiterRadius = p.loiterRadius;
            q.turns = p.turns;
            q.ccw = p.ccw;
            q.glide = p.navLand && glide;
            q.alt = p.alt;
            q.hold = p.hold;
            pts.push_back(q);
        }
        std::vector<mission3d::Poi> pois;
        for (const MissionPoi3D &p : m_mission.pois) {
            mission3d::Poi q;
            q.ground = groundOf(p.ground, p.lat, p.lon);
            q.pos = world(p.lat, p.lon, p.altMsl + s.offsetAlt, origin);
            q.seq = p.seq;
            pois.push_back(q);
        }
        m_missionDraw = mission3d::build(pts, pois, m_mission.rtlSeq);

        m_missionLines.clear();
        for (const mission3d::Line &l : m_missionDraw.lines) {
            LineSeg seg;
            switch (l.style) {
            case mission3d::Style::Route: seg = LineSeg::make(l.a, l.b, pal.route, 3, 0.35f, 2); break;
            case mission3d::Style::Pattern: seg = LineSeg::make(l.a, l.b, segmentColour(pal, l.segType), 2, 0.3f, 1.5f); break;
            case mission3d::Style::Ring: seg = LineSeg::make(l.a, l.b, pal.circle, 2.5f, 0.35f, 1.5f); break;
            case mission3d::Style::Return: seg = LineSeg::make(l.a, l.b, pal.route, 2, 0.3f, 1.5f); break;
            }
            if (l.style == mission3d::Style::Pattern || l.style == mission3d::Style::Return) seg.style[3] = 9;   // dashes of 9 px
            m_missionLines.push_back(seg);
        }
        for (const mission3d::Mark &m : m_missionDraw.marks) {
            const QColor col = segmentColour(pal, m.segType);
            // A drop line to a dot on the ground, the shadow of the point
            if (m.ground && m.at.y() - *m.ground >= 1 && !s.relativeMode) {
                const QVector3D foot(m.at.x(), *m.ground, m.at.z());
                m_missionLines.push_back(LineSeg::make(foot, m.at, col, 1.5f, 0, 0, 0.5f));
                symbol(m_missionLines, foot, circlePx(1.6, 8), col, 3);
            }
            markSymbol(m_missionLines, m, col, false);
        }
        // Labels: a long list of plain waypoints gets every n-th one (and the last) so the screen stays legible
        m_missionLabels.clear();
        const int plainCount = int(std::count_if(m_missionDraw.marks.begin(), m_missionDraw.marks.end(), [](const mission3d::Mark &m) { return m.plain; }));
        const int every = std::max(1, (plainCount + int(kMaxMissionLabels) - 1) / int(kMaxMissionLabels));
        int plainIdx = 0;
        for (const mission3d::Mark &m : m_missionDraw.marks) {
            if (m.plain) {
                const int idx = plainIdx++;
                if (idx % every != 0 && idx != plainCount - 1) continue;
            }
            m_missionLabels.push_back({m.at, m.main, m.sub, pal.label.isValid() ? pal.label : segmentColour(pal, m.segType), kMissionLabelPx});
        }
    }

    void syncTraffic(const VehicleState &s, const QVector3D &cam, LatLon origin, bool relative, const Palette &pal, std::int64_t now)
    {
        m_trafficMarks.clear();
        if (!s.adsbEnabled || relative || s.traffic.empty()) {
            m_trails.clear();
            m_trafficLines.clear();
            return;
        }
        // The nearest ones
        std::vector<const TrafficEntry *> near;
        for (const TrafficEntry &t : s.traffic) near.push_back(&t);
        auto dist = [&](const TrafficEntry *t) { return flatDistance(s.lat, s.lon, t->lat, t->lon); };
        std::sort(near.begin(), near.end(), [&](auto *a, auto *b) { return dist(a) < dist(b); });
        if (near.size() > kMaxTraffic) near.resize(kMaxTraffic);
        std::vector<TrafficTrails::Report> reports;
        for (const TrafficEntry *t : near)
            reports.push_back({t->icao24, t->lat, t->lon, t->alt, t->velocity, t->heading, t->vertRate, t->posTs ? t->posTs : t->ts});
        m_trails.update(reports);

        const bool trailDue = now - m_trafficTrailAt >= kTrailRefreshMs || pal.traffic != m_trafficColour || origin.lat != m_trafficOrigin.lat
                           || origin.lon != m_trafficOrigin.lon;
        if (trailDue) {
            m_trafficTrailAt = now;
            m_trafficColour = pal.traffic;
            m_trafficOrigin = origin;
            m_trafficLines.clear();
        }
        for (const TrafficEntry *t : near) {
            double lat = 0, lon = 0, alt = 0;
            if (!m_trails.head(t->icao24, double(now), &lat, &lon, &alt)) continue;
            const QVector3D p = world(lat, lon, alt + s.offsetAlt, origin);
            if ((p - cam).length() > kTrafficMaxM) continue;
            const QString name = t->callsign.trimmed();
            m_trafficMarks.push_back({p, alt, t->vertRate, name.isEmpty() ? t->icao24.toUpper() : name});
            if (!trailDue) continue;
            const auto trail = m_trails.trail(t->icao24, double(now));
            for (std::size_t i = 1; i < trail.size(); ++i) {
                const float a = float((trail[i - 1].alpha + trail[i].alpha) / 2);
                if (a < 0.01f) continue;
                m_trafficLines.push_back(LineSeg::make(world(trail[i - 1].lat, trail[i - 1].lon, trail[i - 1].alt + s.offsetAlt, origin),
                                                       world(trail[i].lat, trail[i].lon, trail[i].alt + s.offsetAlt, origin), pal.traffic, 2.5f, 0.3f, 1.5f, a));
            }
        }
    }

    std::unique_ptr<TerrainRenderer> m_terrain;
    std::unique_ptr<OverlayRenderer> m_overlay;
    std::unique_ptr<OutlineRenderer> m_outline;
    std::unique_ptr<PointCloudRenderer> m_points;
    std::unique_ptr<RosSurfaceRenderer> m_ros;
    std::unique_ptr<VideoRenderer> m_video;
    int m_videoMode = 0;
    bool m_relative = false;
    float m_dpr = 1;
    TerrainChanges *m_changes = nullptr;
    TerrainFrame m_frame;
    OverlayFrame m_overlayFrame;
    QMatrix4x4 m_view;
    QVector3D m_sunDir{0.35f, 0.75f, 0.45f};
    QColor m_sky;
    SceneItem *m_item = nullptr;
    QElapsedTimer m_statClock;
    int m_frames = 0;
    double m_syncMs = 0;
    QString m_api;
    bool m_formatReported = false;
    // UAS zones
    int m_zoneSerial = 0;
    bool m_zonesPending = false;
    std::shared_ptr<const GeoZoneRaster> m_pendingZones;
    // Model
    int m_modelVersion = -1;
    bool m_modelPending = false;
    std::shared_ptr<const ModelData> m_pendingModel;
    // Trail
    int m_trailVersion = -1, m_trailEpoch = -1;
    std::size_t m_trailSent = 0;
    std::vector<LineSeg> m_trailSegs;
    bool m_trailResetPending = false;
    QColor m_trailColour;
    // Corridor
    int m_corridorFrame = 0;
    std::vector<LineSeg> m_corridorLines;
    std::vector<FillVertex> m_corridorFill;
    // Mission
    int m_missionEpoch = -1, m_missionOriginEpoch = -1, m_missionClamp = -1;
    double m_missionOffset = 0;
    bool m_missionRelative = false;
    bool m_missionWaitingGround = false;
    bool m_missionGlide = false;
    QColor m_missionColour;
    Mission3DInput m_mission;
    mission3d::Drawing m_missionDraw;
    std::optional<MissionPoint3D> m_missionHome;   // its take-off point
    std::optional<QVector3D> m_homeBase;           // foot of the home marker, this frame
    std::vector<LineSeg> m_missionLines;
    std::vector<LabelSpec> m_missionLabels;
    // Traffic
    struct TrafficMark {
        QVector3D at;
        double alt = 0, vertRate = 0;
        QString name;
    };
    TrafficTrails m_trails;
    std::vector<TrafficMark> m_trafficMarks;
    std::vector<LineSeg> m_trafficLines;
    std::int64_t m_trafficTrailAt = 0;
    QColor m_trafficColour;
    LatLon m_trafficOrigin;
    // Labels
    LabelAtlas m_atlas;
    // Satellite detail: operations from synchronize to render
    std::vector<DetailOp> m_detailOps;

    friend class SceneItem;
};

SceneItem::SceneItem(QQuickItem *parent) : QQuickRhiItem(parent)
{
    setSampleCount(4);
    setAcceptedMouseButtons(Qt::LeftButton);
    setAcceptTouchEvents(true);
    m_terrain.setSatellite(!m_schematic);
    m_terrainTimer.setInterval(250);
    connect(&m_terrainTimer, &QTimer::timeout, this, &SceneItem::tick);
    m_terrainTimer.start();
    m_clock.start();
    // The look-around sweep runs on its own 60 Hz while a key is held or the view is still turning
    g_items.insert(this);
    m_lookTimer.setInterval(16);
    m_lookTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_lookTimer, &QTimer::timeout, this, &SceneItem::advanceLook);
    // RENDER FPS CAP: the next frame, requested half a vblank before it is due
    m_frameTimer.setSingleShot(true);
    m_frameTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_frameTimer, &QTimer::timeout, this, &SceneItem::requestFrame);
}

void SceneItem::frameDrawn()
{
    if (m_pacer.cap() != s_frameCap) {
        m_pacer.setCap(s_frameCap);
        m_pacer.reset();
    }
    if (s_frameCap <= 0) {
        update();   // every vblank; the HUD moves on afterAnimating
        return;
    }
    const double now = monoMs();
    const QScreen *screen = window() ? window()->screen() : nullptr;
    const double hz = screen ? screen->refreshRate() : 60.0;
    FramePacer::Vsync vs = FramePacer::systemVsync();
    // DWM times the primary display: use it only while this window is on a display of that rate
    if (vs.periodMs > 0 && std::abs(1000.0 / vs.periodMs - hz) > 0.02 * hz) vs = {};
    const double at = m_pacer.next(now, hz, vs);
    const int waitMs = int(at - now);   // rounded down: early rather than past the vblank
    m_frameAtMs = now + std::max(0, waitMs);
    if (waitMs <= 0) requestFrame();
    else m_frameTimer.start(waitMs);
}

void SceneItem::requestFrame()
{
    // The vblank the frame is meant for, on the wall clock: the HUD and the 3D view take the pose of that instant
    const double mono = monoMs();
    FrameStats::instance().gui(std::max(0.0, mono - m_frameAtMs));
    s_frameWallMs = double(wallMs()) + (m_pacer.dueMs() - mono);
    s_lastTickMs = mono;
    emit FrameTicker::instance()->tick(s_frameWallMs);
    update();
}

SceneItem::~SceneItem()
{
    g_items.remove(this);
    if (m_window) m_window->removeEventFilter(this);
}

QQuickRhiItemRenderer *SceneItem::createRenderer() { return new SceneRenderer; }

void SceneItem::setThirdPerson(bool v)
{
    if (v == m_thirdPerson) return;
    m_thirdPerson = v;
    // Entering the chase view: start behind the vehicle (north is -z: yaw 0 is south of it)
    if (v && s_hub) {
        m_orbit.yaw = -s_hub->snapshot()->yaw;
        m_orbit.pitch = 0.25;
    }
    releaseKeys();
    emit settingsChanged();
    update();
}

void SceneItem::setLookKeys(bool v)
{
    if (v == m_lookKeys) return;
    m_lookKeys = v;
    if (!v) releaseKeys();
    emit settingsChanged();
}

void SceneItem::clearTrail()
{
    m_trail.clear();
    ++m_trailVersion;
    update();
}

void SceneItem::itemChange(ItemChange change, const ItemChangeData &value)
{
    QQuickRhiItem::itemChange(change, value);
    if (change == ItemDevicePixelRatioHasChanged) updateResolution();
    if (change != ItemSceneChange) return;
    updateResolution();
    if (m_window) {
        m_window->removeEventFilter(this);
        disconnect(m_window, nullptr, this, nullptr);
    }
    m_window = value.window;
    if (m_window) {
        m_window->installEventFilter(this);
        connect(m_window, &QQuickWindow::frameSwapped, this, [] { FrameStats::instance().swapped(monoMs()); }, Qt::DirectConnection);
        connect(m_window, &QWindow::activeChanged, this, [this] { if (m_window && !m_window->isActive()) releaseKeys(); });
    }
}

void SceneItem::geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry)
{
    QQuickRhiItem::geometryChange(newGeometry, oldGeometry);
    updateResolution();
}

void SceneItem::setQuality(const QString &q)
{
    const QString n = renderQuality::normalize(q);
    if (n == m_quality) return;
    m_quality = n;
    updateResolution();
    emit settingsChanged();
}

// The target of its own size while the scene has fewer pixels than its rectangle on the display
void SceneItem::updateResolution()
{
    const double dpr = window() ? window()->effectiveDevicePixelRatio() : 1.0;
    // The camera image has no reason to lose detail: at the display's resolution when it is alone
    m_pixelRatio = m_videoMode == 1 ? dpr : renderQuality::pixelRatio(m_quality, dpr);
    const bool fixed = m_pixelRatio < dpr - 1e-6;
    setFixedColorBufferWidth(fixed ? std::max(1, qRound(width() * m_pixelRatio)) : 0);
    setFixedColorBufferHeight(fixed ? std::max(1, qRound(height() * m_pixelRatio)) : 0);
    update();
}

void SceneItem::setVideoMode(int mode)
{
    mode = std::clamp(mode, 0, 2);
    if (mode == m_videoMode) return;
    m_videoMode = mode;
    updateResolution();
    emit settingsChanged();
    update();
}

void SceneItem::setTimeOfDay(int minutes)
{
    minutes = minutes < 0 ? -1 : std::min(minutes, 24 * 60);
    if (minutes == m_timeOfDay) return;
    m_timeOfDay = minutes;
    updateSun();   // at once: tick() only moves it once a second
    emit settingsChanged();
    update();
}

bool SceneItem::eventFilter(QObject *o, QEvent *e)
{
    if (o != m_window) return false;
    if (e->type() != QEvent::KeyPress && e->type() != QEvent::KeyRelease) return false;
    auto *k = static_cast<QKeyEvent *>(e);
    bool *flag = nullptr;
    switch (k->key()) {
    case Qt::Key_Up: flag = &m_keyUp; break;
    case Qt::Key_Down: flag = &m_keyDown; break;
    case Qt::Key_Left: flag = &m_keyLeft; break;
    case Qt::Key_Right: flag = &m_keyRight; break;
    default: return false;
    }
    if (e->type() == QEvent::KeyRelease) {
        if (!k->isAutoRepeat()) {
            *flag = false;
            m_lookTimer.start();
        }
        return false;
    }
    // Same guards as the single-key toggles: the flight screen, not while typing
    if (k->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier)) return false;
    if (!m_lookKeys || !isVisible() || m_thirdPerson) return false;
    if (QQuickItem *f = m_window->activeFocusItem()) {
        if (f->inherits("QQuickTextInput") || f->inherits("QQuickTextEdit") || f->inherits("QQuickComboBox")) return false;
    }
    *flag = true;
    if (!m_lookTimer.isActive()) {
        m_lookTimer.start();
        advanceLook();
    }
    return true;   // swallowed: no list scrolls under the view
}

void SceneItem::releaseKeys()
{
    m_keyUp = m_keyDown = m_keyLeft = m_keyRight = false;
    m_lookTimer.start();   // back to the nose
}

void SceneItem::advanceLook()
{
    if (m_dragLook) return;   // held by the finger
    const bool first = !m_thirdPerson && m_lookKeys && isVisible();
    const double tp = first ? ((lookUp() ? 1 : 0) - (lookDown() ? 1 : 0)) * kLookPitchMax : 0;
    const double ty = first ? ((lookLeft() ? 1 : 0) - (lookRight() ? 1 : 0)) * kLookYawMax : 0;
    const qint64 now = m_clock.elapsed();
    Look &l = m_look;
    if (ty != l.toYaw || tp != l.toPitch) {
        // A new command restarts the sweep from wherever the view is now
        l.fromYaw = l.yaw;
        l.fromPitch = l.pitch;
        l.toYaw = ty;
        l.toPitch = tp;
        const double dist = std::hypot(ty - l.yaw, tp - l.pitch);
        l.dur = std::max(kLookMinSweepS, kLookFullSweepS * std::min(1.0, dist / kLookYawMax));
        l.t0 = now;
    }
    if (l.yaw != l.toYaw || l.pitch != l.toPitch) {
        const double t = std::min(1.0, (now - l.t0) / (l.dur * 1000));
        // ease-in-out cubic: out of rest, then braking into a clean stop
        const double e = t < 0.5 ? 4 * t * t * t : 1 - std::pow(-2 * t + 2, 3) / 2;
        l.yaw = t >= 1 ? l.toYaw : l.fromYaw + (l.toYaw - l.fromYaw) * e;
        l.pitch = t >= 1 ? l.toPitch : l.fromPitch + (l.toPitch - l.fromPitch) * e;
        emit lookChanged();
        update();
    } else if (!lookUp() && !lookDown() && !lookLeft() && !lookRight()) {
        m_lookTimer.stop();   // settled on the nose
    }
}

void SceneItem::mousePressEvent(QMouseEvent *e)
{
    m_pressed = true;
    m_dragging = false;
    m_press = m_last = e->scenePosition();   // in the window: the corner panel grows under the cursor, which is no drag
    e->accept();
}

void SceneItem::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_pressed) return;
    const QPointF d = e->scenePosition() - m_press;
    if (!m_dragging && std::hypot(d.x(), d.y()) < 4) return;
    m_dragging = true;
    dragBy(e->scenePosition());
}

void SceneItem::dragBy(QPointF pos)
{
    if (m_thirdPerson || m_videoMode == 1 || !m_lookKeys) {
        orbitBy(pos);
        return;
    }
    // First person: the picture follows the finger, to the limits of the arrow keys; back to the nose when it leaves
    const QPointF step = pos - m_last;
    m_last = pos;
    const double perPx = m_fov * kDeg / std::max(1.0, height());
    Look &l = m_look;
    l.yaw = std::clamp(l.yaw + step.x() * perPx, -kLookYawMax, kLookYawMax);
    l.pitch = std::clamp(l.pitch + step.y() * perPx, -kLookPitchMax, kLookPitchMax);
    l.fromYaw = l.toYaw = l.yaw;
    l.fromPitch = l.toPitch = l.pitch;
    m_dragLook = true;
    emit lookChanged();
    update();
}

void SceneItem::endDrag()
{
    if (!m_dragLook) return;
    m_dragLook = false;
    m_lookTimer.start();
}

void SceneItem::orbitBy(QPointF pos)
{
    const QPointF step = pos - m_last;
    m_last = pos;
    if (!m_thirdPerson || m_videoMode == 1) return;
    m_orbit.yaw -= step.x() * kOrbitRotateSpeed;
    m_orbit.pitch = std::clamp(m_orbit.pitch - step.y() * kOrbitRotateSpeed, kOrbitMinPitch, kOrbitMaxPitch);
    update();
}

void SceneItem::orbitZoom(double factor)
{
    if (!m_thirdPerson || m_videoMode == 1) return;
    m_orbit.distance = std::clamp(m_orbit.distance * factor, kOrbitMinDistance, kOrbitMaxDistance);
    update();
}

void SceneItem::pinch(QPointF, QPointF, double scale)
{
    // Fingers apart: closer
    orbitZoom(1 / std::clamp(scale, 0.2, 5.0));
}

bool SceneItem::event(QEvent *e)
{
    if (e->type() == QEvent::NativeGesture && m_thirdPerson) {
        if (double levels = 0; TouchGesture::nativeZoom(static_cast<QNativeGestureEvent *>(e), levels)) {
            orbitZoom(std::exp2(-levels));
            return true;
        }
    }
    return QQuickRhiItem::event(e);
}

void SceneItem::mouseReleaseEvent(QMouseEvent *)
{
    if (m_pressed && !m_dragging) emit clicked();
    m_pressed = m_dragging = false;
    endDrag();
}

void SceneItem::wheelEvent(QWheelEvent *e)
{
    if (!m_thirdPerson) {
        e->ignore();
        return;
    }
    // Towards the user: farther; a notch is one step, a touchpad's small ones add up to it
    orbitZoom(std::pow(1 + kOrbitZoomSpeed, -e->angleDelta().y() / 120.0));
    e->accept();
}

void SceneItem::loadModel(const QString &name, double scale)
{
    m_modelName = name;
    m_modelScale = scale;
    QByteArray bytes;
    QString from;
    for (const QString &dir : {DataRoot::installDir() + u"/models"_s, DataRoot::installDir() + u"/resources/models"_s,
                               u":/models"_s}) {
        QFile f(dir + '/'_L1 + name);
        if (f.open(QIODevice::ReadOnly)) {
            bytes = f.readAll();
            from = f.fileName();
            break;
        }
    }
    QString error;
    std::shared_ptr<ModelData> m = bytes.isEmpty() ? nullptr : ModelData::fromGlb(bytes, float(scale), name, &error);
    if (!m) {
        LOG_W("gpu") << "model" << name << "not loaded:" << (bytes.isEmpty() ? u"no such file"_s : error) << "- placeholder";
        m = ModelData::placeholder();
    } else {
        LOG_I("gpu") << "model" << name << "loaded from" << from << ":" << m->vertexCount() / 3 << "triangles, scale" << scale;
    }
    m_model = m;
    ++m_modelVersion;
    update();
}

void SceneItem::tick()
{
    if (!s_hub) return;
    const auto s = s_hub->snapshot();
    // The trail of one frame means nothing in the other
    if (s->relativeMode != m_wasRelative) {
        m_wasRelative = s->relativeMode;
        clearTrail();
    }
    if (s->relativeMode) {
        m_agl = s->rawAlt;   // the ground is the plane at zero: no terrain is loaded for a synthetic position
    } else {
        m_terrain.update(s->lat, s->lon, s->gs, s->sceneOrigin, s->sceneOriginEpoch);
        if (const auto h = HgtStore::instance().elevation(s->lat, s->lon)) m_agl = s->rawAlt - *h;
        refreshZoneRaster(s->lat, s->lon);
    }
    updateLoad(s->lat, s->lon);

    // SYS CONFIG -> 3D VIEW: terrain clipping, model and scale
    Settings &st = Settings::instance();
    const QString mode = st.value(u"ground-clamp-mode"_s).toString();
    m_clamp = mode == "off"_L1 ? 0 : mode == "vehicle"_L1 ? 1 : mode == "all"_L1 ? 2
            : st.value(u"ground-clamp-enabled"_s, u"1"_s).toString() == "0"_L1 ? 0 : 2;
    QString model = st.value(u"selectedModel"_s, u"plane.glb"_s).toString();
    const double scale = std::clamp(st.value(u"modelScale"_s, 1.0).toDouble(), 0.1, 5.0);
    // The airframe on screen follows the one announced by HEARTBEAT (syncModelToVehicleType);
    // a pick in SYS CONFIG sticks until the vehicle type itself changes
    if (s->vehicleType && s->vehicleType != m_lastVehicleType) {
        m_lastVehicleType = s->vehicleType;
        const QString wanted = modelForVehicleType(s->vehicleType);
        if (wanted != model) {
            model = wanted;
            st.setValue(u"selectedModel"_s, model);
        }
    }
    if (model != m_modelName || std::abs(scale - m_modelScale) > 1e-6 || !m_model) loadModel(model, scale);
    // SYS CONFIG -> SATELLITE DETAIL: the sharpest imagery around the aircraft
    m_detail.setMaxZoom(st.value(u"satDetailMaxZoom"_s, SatelliteDetail::kDefaultMaxZoom).toInt());
    // render() asks for the next frame itself; this restarts the chain should a frame be
    // skipped (not while a capped frame is waiting for its vblank: that would draw off the grid)
    if (!m_frameTimer.isActive()) update();

    // The sun for the time of day, once a second is plenty
    if (m_sunClock.isValid() && m_sunClock.elapsed() < 1000) return;
    m_sunClock.restart();
    updateSun();
}

void SceneItem::updateLoad(double lat, double lon)
{
    // What the loading screen and RELOADING MAP wait for
    HgtStore &hgt = HgtStore::instance();
    const int la = int(std::floor(lat)), lo = int(std::floor(lon));
    const bool coming = hgt.loading(la, lo) || HgtDownloader::instance().busy(la, lo);
    const ChunkImagery &img = m_terrain.imagery();
    const QVariantMap load{{u"hgtLoading"_s, hgt.loadingCount()},
                           {u"hgtLoaded"_s, hgt.loadedCount()},
                           {u"hgtMissing"_s, hgt.missing(la, lo) && !coming},
                           {u"chunks"_s, m_terrain.activeChunks()},
                           {u"building"_s, m_terrain.buildingChunks()},
                           {u"images"_s, img.images()},
                           {u"imagesPending"_s, img.pending()},
                           {u"imagesDone"_s, m_schematic || img.firstPassDone()},
                           {u"covered"_s, m_wasRelative || m_terrain.covers(lat, lon)}};
    if (load == m_load) return;
    m_load = load;
    emit loadChanged();
}

void SceneItem::updateSun()
{
    if (!s_hub) return;
    const auto s = s_hub->snapshot();
    // TIME OF DAY holds today's sun at a chosen local time
    const QDateTime when = m_timeOfDay >= 0 ? QDate::currentDate().startOfDay().addSecs(m_timeOfDay * 60) : QDateTime::currentDateTime();
    double az = 0, el = 0;
    sunPosition(when.toUTC(), s->lat, s->lon, &az, &el);
    m_sunEl = el;
    // X east, Y up, Z south; a sun under the horizon still lights a little from above
    const double e = std::max(el, 0.05);
    m_sunDirWorld = QVector3D(float(std::sin(az) * std::cos(e)), float(std::sin(e)), float(-std::cos(az) * std::cos(e))).normalized();
}

void SceneItem::reportStats(const Stats &s)
{
    m_stats = s;
    m_stats.chunks = m_terrain.activeChunks();
    emit statsChanged();
}

} // namespace corv
