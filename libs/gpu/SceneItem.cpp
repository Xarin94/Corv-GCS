#include "gpu/SceneItem.h"

#include "core/Clock.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "gpu/OutlineRenderer.h"
#include "gpu/OverlayRenderer.h"
#include "gpu/PointCloudRenderer.h"
#include "gpu/RosSurfaceRenderer.h"
#include "gpu/TerrainRenderer.h"
#include "scene/CellOverlay.h"
#include "scene/PointCloudData.h"
#include "scene/RosSurfaceData.h"
#include "scene/ModelData.h"
#include "terrain/Hgt.h"
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
#include <QSet>
#include <QPainter>
#include <QPainterPath>
#include <QQuickWindow>
#include <QScreen>
#include <cmath>

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

// Water (main.js / Water3D.js)
constexpr double kWaterPlaneRadius = 500, kWaterPlaneCell = 25;

struct Palette {
    QColor trail, route, activeLeg, home, ownship, traffic, circle, area, corridor, perimeter;
    QColor label;        // every label in this colour (invalid: the colour of its symbol)
    QColor labelHalo;
    float labelHaloPx;
};
const Palette kDark{QColor(0xff3b30), QColor(0x44ff44), QColor(0xd8ffd0), QColor(0xff8800), QColor(0xffffff), QColor(0xff2a2a),
                    QColor(0x4488ff), QColor(0xff6600), QColor(0xcc44ff), QColor(0xffaa00), QColor(), QColor(0, 0, 0, 217), 3.5f};
const Palette kLight{QColor(0xe3261c), QColor(0x0a8f2a), QColor(0x05561b), QColor(0xd96a00), QColor(0x10324a), QColor(0xe3261c),
                     QColor(0x1c5fd8), QColor(0xd65400), QColor(0x9a2ccc), QColor(0xb87800), QColor(0x0a37a6), QColor(0, 0, 0, 153), 1.6f};

QColor segmentColour(const Palette &pal, const QString &segType)
{
    if (segType == QLatin1String("area")) return pal.area;
    if (segType == QLatin1String("corridor")) return pal.corridor;
    if (segType == QLatin1String("perimeter")) return pal.perimeter;
    if (segType == QLatin1String("circle")) return pal.circle;
    return pal.route;
}

QMutex g_missionLock;
std::vector<MissionPoint3D> g_mission;
int g_missionEpoch = 0;

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

        QFont fm(QStringLiteral("Rajdhani"));
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

std::vector<QPointF> shapePx(const QString &kind, double r)
{
    if (kind == QLatin1String("takeoff")) return {{0, r}, {-0.87 * r, -0.5 * r}, {0.87 * r, -0.5 * r}};
    if (kind == QLatin1String("land")) return {{0, r}, {r, 0}, {0, -r}, {-r, 0}};
    if (kind == QLatin1String("loiter")) return circlePx(r, 20);
    const double h = r * 0.8;
    return {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
}
} // namespace

void SceneItem::setMission(std::vector<MissionPoint3D> points)
{
    QMutexLocker l(&g_missionLock);
    g_mission = std::move(points);
    ++g_missionEpoch;
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
            m_api = QString::fromLatin1(rhi()->backendName());
            LOG_I("gpu") << "3D view on" << m_api << rhi()->driverInfo().deviceName;
        }
    }

    void synchronize(QQuickRhiItem *item) override
    {
        auto *it = static_cast<SceneItem *>(item);
        m_item = it;
        if (!SceneItem::s_hub) return;
        const auto s = SceneItem::s_hub->snapshot();
        const std::int64_t now = wallMs();
        // The pose of the vblank the frame is shown at, the instant the HUD took it for (paced)
        const Pose pose = it->m_smoother.update(PoseSmoother::fromState(*s), SceneItem::frameTimeMs(), 0.075);
        const LatLon origin = s->sceneOrigin;
        const int epoch = s->sceneOriginEpoch;
        const bool light = it->m_lightTheme && it->m_schematic;
        const Palette &pal = light ? kLight : kDark;
        const int clamp = it->m_clamp;
        HgtStore &hgt = HgtStore::instance();

        m_dpr = it->window() ? float(it->window()->effectiveDevicePixelRatio()) : 1.f;
        m_frame.fovDeg = float(it->m_fov);
        m_frame.schematic = it->m_schematic;
        m_frame.sunlight = it->m_sunlight;
        m_frame.lightTheme = it->m_lightTheme;
        m_frame.fogDensity = float(1.75 / it->m_terrain.radius());
        m_changes = &it->m_terrain.changes();
        if (m_terrain && !m_formatReported) {
            it->m_terrain.imagery().setCompressed(m_terrain->bc1Supported());
            m_formatReported = true;
        }

        // ── The vehicle: where it is drawn ──
        const GroundMeters gm = latLonToMeters(pose.lat, pose.lon, origin);
        double alt = pose.alt + s->offsetAlt;
        const auto terr = hgt.elevation(pose.lat, pose.lon);
        auto water = hgt.waterAt(pose.lat, pose.lon);
        bool synthetic = false;
        // A sub on water the elevation data does not show: the surface it started on
        if (!water && s->vehicleType == 12 && s->subSurfaceAlt) {
            water = HgtStore::Water{*s->subSurfaceAlt, std::nullopt};
            synthetic = true;
        }
        // Water is no floor (a boat sits on it, a ROV dives under it); the sea bed is
        const std::optional<double> floor = water ? water->bed : terr;
        if (clamp >= 1 && floor && alt < *floor + 0.5) alt = *floor + 0.5;
        // Above land the camera and the model never go below 1 m MSL
        if (!water) alt = std::max(alt, 1.0);
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
            const auto camWater = hgt.waterAt(cll.lat, cll.lon);
            const auto camGround = camWater ? camWater->bed : hgt.elevation(cll.lat, cll.lon);
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
            const qreal dpr = it->window() ? it->window()->effectiveDevicePixelRatio() : 1.0;
            const double viewportPx = std::max(1.0, it->height() * dpr);
            const double pixelAngle = it->m_fov * kDeg / viewportPx;
            it->m_detail.update(pose.lat, pose.lon, origin, cam, P, terr, pixelAngle, !it->m_schematic, m_detailOps, m_frame.detail);
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

        // Home: a pole from the ground, a ring on it and the symbol on top
        if (s->homeLat && s->homeLon) {
            const auto hg = hgt.elevation(*s->homeLat, *s->homeLon);
            const double ground = hg ? *hg : s->homeAlt.value_or(0);
            const QVector3D base = world(*s->homeLat, *s->homeLon, ground + 0.5, origin);
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

        // Mission route
        syncMission(it, *s, origin, epoch, clamp, pal);
        L.insert(L.end(), m_missionLines.begin(), m_missionLines.end());
        if (!m_missionPts.empty()) {
            // The leg being flown, towards MISSION_CURRENT
            for (std::size_t i = 1; i < m_missionPts.size(); ++i) {
                if (m_missionPts[i].seq != s->missionCurrentSeq || m_missionPts[i].derived) continue;
                std::size_t j = i;
                while (j > 0) {
                    L.push_back(LineSeg::make(m_missionWorld[j - 1], m_missionWorld[j], pal.activeLeg, 4.5f));
                    --j;
                    if (!m_missionPts[j].derived) break;
                }
                break;
            }
        }

        // ADS-B traffic: a diamond on each contact and a line along its track
        if (s->adsbEnabled) {
            for (const TrafficEntry &t : s->traffic) {
                const QVector3D p = world(t.lat, t.lon, t.alt, origin);
                if ((p - cam).length() > 60000) continue;
                symbol(L, p, {{0, 8}, {8, 0}, {0, -8}, {-8, 0}}, pal.traffic, 2.5f, 0.5f);
                const double h = t.heading * kDeg;
                const QVector3D ahead = p + QVector3D(float(std::sin(h) * 400), 0, float(-std::cos(h) * 400));
                L.push_back(LineSeg::make(p, ahead, pal.traffic, 2, 0.3f, 1.5f));
            }
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
        if (water && it->m_schematic) {
            const double strength = lowAltGridStrength(alt - water->level);
            const LatLon cll = metersToLatLon(cam.x(), cam.z(), origin);
            const auto camW = hgt.waterAt(cll.lat, cll.lon);
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
        {
            const LatLon cll = metersToLatLon(cam.x(), cam.z(), origin);
            auto camW = hgt.waterAt(cll.lat, cll.lon);
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

        m_sky = skyColour(it);

        // UAS zones on the terrain: the raster's window in today's world metres
        if (it->m_zoneRasterSerial != m_zoneSerial) {
            m_zoneSerial = it->m_zoneRasterSerial;
            m_pendingZones = it->m_zoneRaster;
            m_zonesPending = true;
        }
        const auto &zr = it->m_zoneRaster;
        m_frame.zonesOn = zr && !zr->empty;
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
    }

    void render(QRhiCommandBuffer *cb) override
    {
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
            m_terrain->draw(cb, out);
            cb->endPass();
            cb->beginPass(renderTarget(), m_sky, {1.0f, 0});
            m_outline->draw(cb, out);
        } else {
            cb->beginPass(renderTarget(), m_sky, {1.0f, 0}, u);
            m_terrain->draw(cb, out);
        }
        // The solid ROS surface before the vehicle, so the vehicle stays in front of the bed it floats over
        m_ros->drawOpaque(cb, out);
        m_overlay->drawOpaque(cb, out);
        m_ros->drawTranslucent(cb, out);
        m_points->draw(cb, out);
        m_overlay->drawTranslucent(cb, out);
        cb->endPass();
        // Keep rendering, the pose moves every frame: the next frame at once or on the cap's schedule
        if (m_item) QMetaObject::invokeMethod(m_item, [i = m_item] { i->frameDrawn(); }, Qt::QueuedConnection);
    }

private:
    QColor skyColour(const SceneItem *it) const
    {
        if (it->m_schematic) return it->m_lightTheme ? QColor::fromRgbF(0.78f, 0.86f, 0.93f) : QColor(0, 0, 0);
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
        if (s.homeLat && s.homeLon) {
            const auto hg = hgt.elevation(*s.homeLat, *s.homeLon);
            const double ground = hg ? *hg : s.homeAlt.value_or(0);
            specs.push_back({world(*s.homeLat, *s.homeLon, ground + 0.5 + kHomePoleM, origin), QStringLiteral("HOME"), {},
                             pal.label.isValid() ? pal.label : pal.home, kHomeLabelPx});
        }
        // Mission items: built with the route
        specs.insert(specs.end(), m_missionLabels.begin(), m_missionLabels.end());
        // Traffic: callsign, height relative to us in steps of 10 m and a trend arrow (Traffic3D.updateLabel)
        if (s.adsbEnabled) {
            for (const TrafficEntry &t : s.traffic) {
                const QVector3D p = world(t.lat, t.lon, t.alt, origin);
                if ((p - cam).length() > 60000) continue;
                QString main = t.callsign.trimmed();
                if (main.isEmpty()) main = t.icao24.toUpper();
                const int rel = int(std::lround((t.alt - vehicleAlt) / 10.0)) * 10;
                QString sub = (rel >= 0 ? QStringLiteral("+") : QStringLiteral("\u2212")) + QString::number(std::abs(rel)) + QStringLiteral(" m");
                if (t.vertRate > 2) sub += QStringLiteral(" \u2191");
                else if (t.vertRate < -2) sub += QStringLiteral(" \u2193");
                specs.push_back({p, main, sub, pal.traffic, kTrafficLabelPx});
            }
        }

        const qreal dpr = it->window() ? it->window()->effectiveDevicePixelRatio() : 1.0;
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

    void syncMission(SceneItem *, const VehicleState &, LatLon origin, int epoch, int clamp, const Palette &pal)
    {
        int missionEpoch;
        {
            QMutexLocker l(&g_missionLock);
            missionEpoch = g_missionEpoch;
            if (missionEpoch == m_missionEpoch && epoch == m_missionOriginEpoch && clamp == m_missionClamp && pal.route == m_missionColour
                && !m_missionWaitingGround)
                return;
            m_missionPts = g_mission;
        }
        m_missionEpoch = missionEpoch;
        m_missionOriginEpoch = epoch;
        m_missionClamp = clamp;
        m_missionColour = pal.route;
        m_missionWaitingGround = false;
        m_missionLines.clear();
        m_missionWorld.clear();
        HgtStore &hgt = HgtStore::instance();
        for (const MissionPoint3D &p : m_missionPts) {
            std::optional<double> ground = p.ground ? p.ground : hgt.elevation(p.lat, p.lon);
            if (!ground) m_missionWaitingGround = true;   // try again when the tile is in
            double y = p.altMsl;
            if (clamp >= 2 && ground && y < *ground + 1) y = *ground + 1;
            m_missionWorld.push_back(world(p.lat, p.lon, y, origin));
        }
        m_missionLabels.clear();
        struct Mark {
            std::size_t i;
            QString main, sub;
            QColor color;
            bool plain;
        };
        std::vector<Mark> marks;
        for (std::size_t i = 0; i < m_missionPts.size(); ++i) {
            const MissionPoint3D &p = m_missionPts[i];
            const QVector3D &w = m_missionWorld[i];
            if (i > 0) m_missionLines.push_back(LineSeg::make(m_missionWorld[i - 1], w, pal.route, 3, 0.35f, 2));
            if (p.derived || p.kind == QLatin1String("home")) continue;
            if (p.segPos == 2) continue;   // inside a pattern only its ends are marked
            // The label (Mission3D.js marks)
            {
                const QString seq = QString::number(p.seq);
                const QString alt = QStringLiteral("%1 m").arg(qRound(p.alt));
                Mark m{i, {}, {}, pal.route, false};
                if (p.kind == QLatin1String("loiter") && p.loiterRadius > 0) {
                    m.color = pal.circle;
                    m.main = seq + QStringLiteral(" CIRC");
                    m.sub = alt + (p.turns > 1 ? QStringLiteral(" \u00b7 %1\u00d7").arg(p.turns) : QString());
                } else if (!p.segType.isEmpty()) {
                    m.color = segmentColour(pal, p.segType);
                    static const QHash<QString, QString> tags{{QStringLiteral("area"), QStringLiteral("AREA")},
                                                              {QStringLiteral("corridor"), QStringLiteral("CORR")},
                                                              {QStringLiteral("perimeter"), QStringLiteral("PERIM")},
                                                              {QStringLiteral("circle"), QStringLiteral("CIRC")}};
                    m.main = p.segPos == 1 ? (seq + QLatin1Char(' ') + tags.value(p.segType)).trimmed() : seq;
                    if (p.segPos == 1) m.sub = alt;
                } else {
                    const QString tag = p.kind == QLatin1String("takeoff") ? QStringLiteral("T/O")
                                      : p.kind == QLatin1String("land")    ? QStringLiteral("LAND")
                                      : p.kind == QLatin1String("loiter")  ? QStringLiteral("LOIT")
                                                                           : QString();
                    m.main = tag.isEmpty() ? seq : seq + QLatin1Char(' ') + tag;
                    if (p.kind != QLatin1String("land"))
                        m.sub = alt + (p.hold > 0 ? QStringLiteral(" \u00b7 %1 s").arg(qRound(p.hold)) : QString());
                    m.plain = p.kind == QLatin1String("wp");
                }
                marks.push_back(m);
            }
            const QColor col = marks.back().color;
            // Drop line to the ground and the symbol of the item
            const auto ground = p.ground ? p.ground : hgt.elevation(p.lat, p.lon);
            if (ground) m_missionLines.push_back(LineSeg::make(world(p.lat, p.lon, *ground, origin), w, col, 1.5f, 0, 0, 0.5f));
            symbol(m_missionLines, w, shapePx(p.segType.isEmpty() ? p.kind : QStringLiteral("wp"), 7), col, 2.5f);
            if (p.kind == QLatin1String("loiter") && p.loiterRadius > 0) {
                const int n = 72;
                for (int k = 0; k < n; ++k) {
                    const double a0 = 2 * kPi * k / n, a1 = 2 * kPi * (k + 1) / n;
                    const QVector3D q0 = w + QVector3D(float(p.loiterRadius * std::cos(a0)), 0, float(p.loiterRadius * std::sin(a0)));
                    const QVector3D q1 = w + QVector3D(float(p.loiterRadius * std::cos(a1)), 0, float(p.loiterRadius * std::sin(a1)));
                    m_missionLines.push_back(LineSeg::make(q0, q1, pal.circle, 2.5f, 0.35f, 1.5f));
                }
            }
        }
        // A long list of plain waypoints gets every n-th label (and the last) so the screen stays legible
        const int plainCount = int(std::count_if(marks.begin(), marks.end(), [](const Mark &m) { return m.plain; }));
        const int every = std::max(1, (plainCount + int(kMaxMissionLabels) - 1) / int(kMaxMissionLabels));
        int plainIdx = 0;
        for (const Mark &m : marks) {
            if (m.plain) {
                const int idx = plainIdx++;
                if (idx % every != 0 && idx != plainCount - 1) continue;
            }
            m_missionLabels.push_back({m_missionWorld[m.i], m.main, m.sub, pal.label.isValid() ? pal.label : m.color, kMissionLabelPx});
        }
    }

    std::unique_ptr<TerrainRenderer> m_terrain;
    std::unique_ptr<OverlayRenderer> m_overlay;
    std::unique_ptr<OutlineRenderer> m_outline;
    std::unique_ptr<PointCloudRenderer> m_points;
    std::unique_ptr<RosSurfaceRenderer> m_ros;
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
    bool m_missionWaitingGround = false;
    QColor m_missionColour;
    std::vector<MissionPoint3D> m_missionPts;
    std::vector<QVector3D> m_missionWorld;
    std::vector<LineSeg> m_missionLines;
    std::vector<LabelSpec> m_missionLabels;
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
    if (waitMs <= 0) requestFrame();
    else m_frameTimer.start(waitMs);
}

void SceneItem::requestFrame()
{
    // The vblank the frame is meant for, on the wall clock: the HUD and the 3D view take the pose of that instant
    const double mono = monoMs();
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
    // Entering the chase view: start behind the vehicle
    if (v && s_hub) {
        m_orbit.yaw = -s_hub->snapshot()->yaw + kPi;
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
    if (change != ItemSceneChange) return;
    if (m_window) {
        m_window->removeEventFilter(this);
        disconnect(m_window, nullptr, this, nullptr);
    }
    m_window = value.window;
    if (m_window) {
        m_window->installEventFilter(this);
        connect(m_window, &QWindow::activeChanged, this, [this] { if (m_window && !m_window->isActive()) releaseKeys(); });
    }
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
    m_press = m_last = e->position();
    e->accept();
}

void SceneItem::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_pressed) return;
    const QPointF d = e->position() - m_press;
    if (!m_dragging && std::hypot(d.x(), d.y()) < 4) return;
    m_dragging = true;
    const QPointF step = e->position() - m_last;
    m_last = e->position();
    if (!m_thirdPerson) return;
    m_orbit.yaw -= step.x() * kOrbitRotateSpeed;
    m_orbit.pitch = std::clamp(m_orbit.pitch - step.y() * kOrbitRotateSpeed, kOrbitMinPitch, kOrbitMaxPitch);
    update();
}

void SceneItem::mouseReleaseEvent(QMouseEvent *)
{
    if (m_pressed && !m_dragging) emit clicked();
    m_pressed = m_dragging = false;
}

void SceneItem::wheelEvent(QWheelEvent *e)
{
    if (!m_thirdPerson) {
        e->ignore();
        return;
    }
    const int dir = e->angleDelta().y() > 0 ? -1 : e->angleDelta().y() < 0 ? 1 : 0;   // towards the user: farther
    m_orbit.distance = std::clamp(m_orbit.distance * (1 + dir * kOrbitZoomSpeed), kOrbitMinDistance, kOrbitMaxDistance);
    e->accept();
    update();
}

void SceneItem::loadModel(const QString &name, double scale)
{
    m_modelName = name;
    m_modelScale = scale;
    QByteArray bytes;
    QString from;
    for (const QString &dir : {DataRoot::installDir() + QStringLiteral("/models"), DataRoot::installDir() + QStringLiteral("/resources/models"),
                               QStringLiteral(":/models")}) {
        QFile f(dir + QLatin1Char('/') + name);
        if (f.open(QIODevice::ReadOnly)) {
            bytes = f.readAll();
            from = f.fileName();
            break;
        }
    }
    QString error;
    std::shared_ptr<ModelData> m = bytes.isEmpty() ? nullptr : ModelData::fromGlb(bytes, float(scale), name, &error);
    if (!m) {
        LOG_W("gpu") << "model" << name << "not loaded:" << (bytes.isEmpty() ? QStringLiteral("no such file") : error) << "- placeholder";
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
    m_terrain.update(s->lat, s->lon, s->gs, s->sceneOrigin, s->sceneOriginEpoch);
    if (const auto h = HgtStore::instance().elevation(s->lat, s->lon)) m_agl = s->rawAlt - *h;
    refreshZoneRaster(s->lat, s->lon);

    // SYS CONFIG -> 3D VIEW: terrain clipping, model and scale
    Settings &st = Settings::instance();
    const QString mode = st.value(QStringLiteral("ground-clamp-mode")).toString();
    m_clamp = mode == QLatin1String("off") ? 0 : mode == QLatin1String("vehicle") ? 1 : mode == QLatin1String("all") ? 2
            : st.value(QStringLiteral("ground-clamp-enabled"), QStringLiteral("1")).toString() == QLatin1String("0") ? 0 : 2;
    QString model = st.value(QStringLiteral("selectedModel"), QStringLiteral("plane.glb")).toString();
    const double scale = std::clamp(st.value(QStringLiteral("modelScale"), 1.0).toDouble(), 0.1, 5.0);
    // The airframe on screen follows the one announced by HEARTBEAT (syncModelToVehicleType);
    // a pick in SYS CONFIG sticks until the vehicle type itself changes
    if (s->vehicleType && s->vehicleType != m_lastVehicleType) {
        m_lastVehicleType = s->vehicleType;
        const QString wanted = modelForVehicleType(s->vehicleType);
        if (wanted != model) {
            model = wanted;
            st.setValue(QStringLiteral("selectedModel"), model);
        }
    }
    if (model != m_modelName || std::abs(scale - m_modelScale) > 1e-6 || !m_model) loadModel(model, scale);
    // SYS CONFIG -> SATELLITE DETAIL: the sharpest imagery around the aircraft
    m_detail.setMaxZoom(st.value(QStringLiteral("satDetailMaxZoom"), SatelliteDetail::kDefaultMaxZoom).toInt());
    // render() asks for the next frame itself; this restarts the chain should a frame be
    // skipped (not while a capped frame is waiting for its vblank: that would draw off the grid)
    if (!m_frameTimer.isActive()) update();

    // The sun for the time of day, once a second is plenty
    if (m_sunClock.isValid() && m_sunClock.elapsed() < 1000) return;
    m_sunClock.restart();
    double az = 0, el = 0;
    sunPosition(QDateTime::currentDateTimeUtc(), s->lat, s->lon, &az, &el);
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
