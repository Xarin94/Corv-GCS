// SceneItem.h - the 3D view (Scene3D.js and the 3D part of js/main.js): a
// QQuickRhiItem drawing the terrain and, over it, the vehicle, its trail, the
// predicted trajectory corridor, home, the target, the mission route (Mission3D),
// ADS-B traffic with its trails, the water surface and the specks under it.
//
// Cameras (setCameraMode of main.js):
//   first person  at the vehicle, attitude of the vehicle (pitch held level
//                 with the horizon lock); the arrow keys turn the view while
//                 held, 90 deg up / down and 110 deg left / right, an
//                 ease-in-out sweep there and back (initLookAround);
//   third person  orbit around the model: drag to turn, wheel or pinch to zoom,
//                 never closer than 30 m to the ground under the camera.
// Relative navigation (no GPS): no terrain, imagery or traffic; a grid on zero.
// FPV video (VideoRenderer): the camera image alone instead of the 3D, or the
// schematic 3D, on black, blended over it (AR).
// Terrain clipping (SYS CONFIG -> 3D VIEW -> TERRAIN CLIPPING):
//   off      everything at its true height, also under the ground;
//   vehicle  the model and the first-person camera stay 0.5 m over the ground
//            (over water: the sea bed, when the data has one) - GROUND CLAMP;
//   all      and so do the trail, the corridor and the mission route.
//
// GUI thread: a 4 Hz timer brings the terrain chunks (and the satellite
// imagery) up to date around the vehicle, loads the model of the vehicle type
// and reads the settings; the look-around sweep advances every frame
// (afterAnimating). Render thread (GUI blocked in synchronize()): the smoothed
// pose, the camera and every overlay are built; render() draws. Nothing else
// in the program touches QRhi.
#pragma once

#include "gpu/FrameTicker.h"
#include "gpu/TouchGesture.h"
#include "scene/FramePacer.h"
#include "scene/GeoZoneRaster.h"
#include "terrain/SatelliteDetail.h"
#include "terrain/TerrainManager.h"
#include "vehicle/PoseSmoother.h"
#include "vehicle/TrajectoryPredictor.h"

#include <QElapsedTimer>
#include <QPointer>
#include <QQuickRhiItem>
#include <QTimer>
#include <QVariantMap>
#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

QT_FORWARD_DECLARE_CLASS(QQuickWindow)

namespace corv {

class VehicleHub;
struct ModelData;

/// A located mission item as the 3D view draws it (buildMissionTrajectory3D of main.js).
struct MissionPoint3D {
    double lat = 0, lon = 0;
    double altMsl = 0;
    std::optional<double> ground;   // terrain under the point, MSL
    int seq = 0;
    QString kind;                   // home | takeoff | land | loiter | wp
    bool derived = false;           // a terrain-following vertex the compiler inserted
    double loiterRadius = 0;
    int turns = 0;                  // loiter turns
    bool ccw = false;
    bool navLand = false;           // NAV_LAND: a fixed wing flies it down a glide slope
    double alt = 0;                 // as the item states it, in its own frame
    double hold = 0;                // s
    QString segId, segType;         // the planner's segment: area | corridor | perimeter | circle...
};

/// A point the camera looks at (DO_SET_ROI), not flown.
struct MissionPoi3D {
    double lat = 0, lon = 0;
    double altMsl = 0;
    std::optional<double> ground;
    int seq = 0;
};

struct Mission3DInput {
    std::vector<MissionPoint3D> points;
    std::vector<MissionPoi3D> pois;
    std::optional<int> rtlSeq;      // a RETURN_TO_LAUNCH closes the route
};

class SceneItem : public QQuickRhiItem, private TouchGesture::Client
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(double fovDeg READ fovDeg WRITE setFovDeg NOTIFY settingsChanged)
    Q_PROPERTY(bool schematic READ schematic WRITE setSchematic NOTIFY settingsChanged)
    Q_PROPERTY(bool sunlight READ sunlight WRITE setSunlight NOTIFY settingsChanged)
    Q_PROPERTY(bool lightTheme READ lightTheme WRITE setLightTheme NOTIFY settingsChanged)
    Q_PROPERTY(double terrainRadius READ terrainRadius WRITE setTerrainRadius NOTIFY settingsChanged)
    Q_PROPERTY(double satelliteRadius READ satelliteRadius WRITE setSatelliteRadius NOTIFY settingsChanged)
    Q_PROPERTY(bool thirdPerson READ thirdPerson WRITE setThirdPerson NOTIFY settingsChanged)
    Q_PROPERTY(bool horizonLock READ horizonLock WRITE setHorizonLock NOTIFY settingsChanged)
    Q_PROPERTY(bool trajectory READ trajectory WRITE setTrajectory NOTIFY settingsChanged)
    Q_PROPERTY(bool lookKeys READ lookKeys WRITE setLookKeys NOTIFY settingsChanged)
    Q_PROPERTY(double orbitPitch READ orbitPitch WRITE setOrbitPitch NOTIFY settingsChanged)
    Q_PROPERTY(double orbitDistance READ orbitDistance WRITE setOrbitDistance NOTIFY settingsChanged)
    /// Blend time constant of the drawn pose, s (SYS CONFIG -> MOTION SMOOTHING; 0 draws the telemetry as it arrives).
    Q_PROPERTY(double smoothing READ smoothing WRITE setSmoothing NOTIFY settingsChanged)
    /// MAP BRIGHTNESS: the terrain the sun does not light (0.3 .. 1.6).
    Q_PROPERTY(double brightness READ brightness WRITE setBrightness NOTIFY settingsChanged)
    /// TIME OF DAY: minutes from midnight, local, the sun is shown at; -1 follows the clock.
    Q_PROPERTY(int timeOfDay READ timeOfDay WRITE setTimeOfDay NOTIFY settingsChanged)
    /// RESOLUTION of the scene: native | balanced | eco.
    Q_PROPERTY(QString quality READ quality WRITE setQuality NOTIFY settingsChanged)
    /// FPV camera: 0 none, 1 the camera image alone, 2 the 3D view blended over it (AR).
    Q_PROPERTY(int videoMode READ videoMode WRITE setVideoMode NOTIFY settingsChanged)
    Q_PROPERTY(double pixelRatio READ pixelRatio NOTIFY statsChanged)
    Q_PROPERTY(double sunElevation READ sunElevation NOTIFY statsChanged)
    Q_PROPERTY(double lookYaw READ lookYaw NOTIFY lookChanged)
    Q_PROPERTY(double lookPitch READ lookPitch NOTIFY lookChanged)
    Q_PROPERTY(int chunks READ chunks NOTIFY statsChanged)
    Q_PROPERTY(int drawCalls READ drawCalls NOTIFY statsChanged)
    Q_PROPERTY(int visibleChunks READ visibleChunks NOTIFY statsChanged)
    Q_PROPERTY(double fps READ fps NOTIFY statsChanged)
    Q_PROPERTY(double agl READ agl NOTIFY statsChanged)
    Q_PROPERTY(QString graphicsApi READ graphicsApi NOTIFY statsChanged)
    Q_PROPERTY(int satelliteImages READ satelliteImages NOTIFY statsChanged)
    Q_PROPERTY(double satelliteMB READ satelliteMB NOTIFY statsChanged)
    /// How far the terrain and its imagery are (the loading screen): hgtLoading, hgtLoaded, hgtMissing,
    /// chunks, building, images, imagesPending, imagesDone, covered.
    Q_PROPERTY(QVariantMap load READ load NOTIFY loadChanged)

public:
    explicit SceneItem(QQuickItem *parent = nullptr);
    ~SceneItem() override;
    static void setHub(VehicleHub *hub) { s_hub = hub; }
    /// The mission route drawn in 3D (FLIGHT PLAN compiles it). Any number of views.
    static void setMission(Mission3DInput mission);
    /// GCS OPTIONS -> TARGET: a red pole with a sphere and a ring; none clears it. Any number of views.
    static void setTarget(std::optional<LatLon> target);
    /// UAS geographical zones painted on the terrain (GeoZones); version changes with them.
    static void setGeoZones(GeoZonesPtr zones, int version, bool on);
    /// Look-around buttons of the gamepad (JOYSTICK / RC → FPV LOOK BUTTONS), as the arrow keys. GUI thread.
    static void setGamepadLook(bool up, bool down, bool left, bool right);
    /// SYS CONFIG -> RENDER FPS CAP: at most this many frames a second in every 3D view
    /// (60 or 30; 0 draws at every vblank). GUI thread.
    static void setFrameCap(int fps) { s_frameCap = fps; }
    /// The 3D view is pacing the window's frames (capped, a tick in the last 250 ms).
    static bool pacing();
    /// When the pose of the frame being prepared is taken: its vblank while pacing, now otherwise.
    static double frameTimeMs();

    double fovDeg() const { return m_fov; }
    void setFovDeg(double v) { m_fov = v; emit settingsChanged(); }
    bool schematic() const { return m_schematic; }
    void setSchematic(bool v)
    {
        m_schematic = v;
        m_terrain.setSatellite(!v);
        emit settingsChanged();
        update();
    }
    bool sunlight() const { return m_sunlight; }
    void setSunlight(bool v) { m_sunlight = v; emit settingsChanged(); update(); }
    bool lightTheme() const { return m_lightTheme; }
    void setLightTheme(bool v) { m_lightTheme = v; emit settingsChanged(); update(); }
    double terrainRadius() const { return m_terrain.radius(); }
    void setTerrainRadius(double v) { m_terrain.setRadius(v); emit settingsChanged(); }
    double satelliteRadius() const { return m_terrain.satelliteRadius(); }
    void setSatelliteRadius(double v) { m_terrain.setSatelliteRadius(v); emit settingsChanged(); }
    bool thirdPerson() const { return m_thirdPerson; }
    void setThirdPerson(bool v);
    bool horizonLock() const { return m_horizonLock; }
    void setHorizonLock(bool v) { m_horizonLock = v; emit settingsChanged(); update(); }
    bool trajectory() const { return m_trajectory; }
    void setTrajectory(bool v) { m_trajectory = v; m_predictor.reset(); emit settingsChanged(); update(); }
    /// Arrow keys turn the first-person view (off while the view is not the flight screen's main one).
    bool lookKeys() const { return m_lookKeys; }
    void setLookKeys(bool v);
    double orbitPitch() const { return m_orbit.pitch; }
    void setOrbitPitch(double v) { m_orbit.pitch = std::clamp(v, -1.2, 1.2); emit settingsChanged(); update(); }
    double orbitDistance() const { return m_orbit.distance; }
    void setOrbitDistance(double v) { m_orbit.distance = std::clamp(v, 4.0, 2500.0); emit settingsChanged(); update(); }
    double smoothing() const { return m_tau; }
    void setSmoothing(double v) { m_tau = std::max(0.0, v); emit settingsChanged(); update(); }
    double brightness() const { return m_brightness; }
    void setBrightness(double v) { m_brightness = std::clamp(v, 0.3, 1.6); emit settingsChanged(); update(); }
    int timeOfDay() const { return m_timeOfDay; }
    void setTimeOfDay(int minutes);
    QString quality() const { return m_quality; }
    void setQuality(const QString &q);
    int videoMode() const { return m_videoMode; }
    void setVideoMode(int mode);
    double pixelRatio() const { return m_pixelRatio; }
    double sunElevation() const { return m_sunEl * 57.29577951308232; }
    double lookYaw() const { return m_look.yaw; }
    double lookPitch() const { return m_look.pitch; }

    int chunks() const { return m_stats.chunks; }
    int drawCalls() const { return m_stats.drawCalls; }
    int visibleChunks() const { return m_stats.visible; }
    double fps() const { return m_stats.fps; }
    double agl() const { return m_agl; }
    QString graphicsApi() const { return m_stats.api; }
    int satelliteImages() const { return m_stats.images; }
    double satelliteMB() const { return m_stats.imageMB; }
    QVariantMap load() const { return m_load; }

    /// GCS OPTIONS -> CLEAR TRAIL.
    Q_INVOKABLE void clearTrail();

    struct Stats { int chunks = 0, drawCalls = 0, visible = 0; double fps = 0; QString api; int images = 0; double imageMB = 0; };
    void reportStats(const Stats &s);

signals:
    void settingsChanged();
    void statsChanged();
    void lookChanged();
    void loadChanged();
    /// A click without a drag (the mini-map swap in the flight screen).
    void clicked();

protected:
    QQuickRhiItemRenderer *createRenderer() override;
    void itemChange(ItemChange change, const ItemChangeData &value) override;
    void geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry) override;
    bool eventFilter(QObject *o, QEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void touchEvent(QTouchEvent *e) override { m_touch.event(e); }
    void touchUngrabEvent() override { m_touch.cancel(); }
    bool event(QEvent *e) override;

private:
    friend class SceneRenderer;
    void touchDragBegin(QPointF press) override { m_last = press; }
    void touchDrag(QPointF p) override { dragBy(p); }
    void touchDragEnd(QPointF) override { endDrag(); }
    void touchCancel() override { endDrag(); }
    void touchTap(QPointF) override { emit clicked(); }
    void pinch(QPointF centre, QPointF moved, double scale) override;
    /// The chase camera turned by the movement to `pos`.
    void orbitBy(QPointF pos);
    /// A drag on the view: the orbit in third person, the look in first.
    void dragBy(QPointF pos);
    void endDrag();
    void orbitZoom(double factor);
    void tick();
    void advanceLook();
    void releaseKeys();
    void loadModel(const QString &name, double scale);
    void updateSun();
    void updateResolution();
    /// The render thread drew a frame: ask for the next one now or on the cap's schedule.
    void frameDrawn();
    /// A paced frame: the tick for the HUD, then the 3D view.
    void requestFrame();

    static VehicleHub *s_hub;
    static int s_frameCap;
    static double s_frameWallMs, s_lastTickMs;
    FramePacer m_pacer;
    QTimer m_frameTimer;
    double m_frameAtMs = 0;   // when the frame timer was to fire
    TerrainManager m_terrain;
    SatelliteDetail m_detail;
    QTimer m_terrainTimer;
    PoseSmoother m_smoother;
    TrajectoryPredictor m_predictor;
    double m_fov = 60;
    bool m_schematic = true;
    bool m_sunlight = false;   // btn-sun starts off
    bool m_lightTheme = false;
    bool m_thirdPerson = false;
    bool m_horizonLock = false;
    bool m_trajectory = false;
    bool m_lookKeys = true;
    double m_tau = PoseSmoother::tauFor(PoseSmoother::kDefaultSmoothing);
    double m_brightness = 0.85;
    int m_timeOfDay = -1;
    QString m_quality = QStringLiteral("balanced");
    double m_pixelRatio = 1;   // of the render target
    int m_videoMode = 0;
    double m_agl = 0;
    QVariantMap m_load;
    void updateLoad(double lat, double lon);
    bool m_wasRelative = false;   // relative navigation: a plane at zero instead of the terrain
    Stats m_stats;

    // Terrain clipping: 0 off, 1 vehicle and camera, 2 also trail, corridor, mission
    int m_clamp = 2;

    // Third-person orbit (orbit of main.js)
    struct Orbit {
        double yaw = 0, pitch = 0.35, distance = 350, height = 40;
    } m_orbit;
    QPointF m_press, m_last;
    bool m_dragging = false, m_pressed = false;
    bool m_dragLook = false;   // the first-person view held off the nose by a drag
    TouchGesture m_touch{this};

    // Look-around (look of main.js)
    struct Look {
        double yaw = 0, pitch = 0, fromYaw = 0, fromPitch = 0, toYaw = 0, toPitch = 0, dur = 0;
        qint64 t0 = 0;
    } m_look;
    bool m_keyUp = false, m_keyDown = false, m_keyLeft = false, m_keyRight = false;
    bool lookUp() const;
    bool lookDown() const;
    bool lookLeft() const;
    bool lookRight() const;
    QElapsedTimer m_clock;
    QTimer m_lookTimer;
    QPointer<QQuickWindow> m_window;

    // Trail (Scene3D.updateTrail): geographic, so a new scene origin moves it
    struct TrailPt {
        double lat, lon;
        float alt;
    };
    std::vector<TrailPt> m_trail;
    int m_trailVersion = 0;

    // Vehicle model
    std::shared_ptr<const ModelData> m_model;
    int m_modelVersion = 0;
    QString m_modelName;
    double m_modelScale = 1;
    int m_lastVehicleType = 0;

    // UAS zones on the terrain: rasterised off the GUI thread around the vehicle
    void refreshZoneRaster(double lat, double lon);
    std::shared_ptr<const GeoZoneRaster> m_zoneRaster;
    int m_zoneRasterSerial = 0;
    int m_zoneBuiltVersion = -1;
    bool m_zoneBuilding = false;

    // The sun for the time and place (realistic sunlight)
    QElapsedTimer m_sunClock;
    double m_sunEl = 0.8;
    QVector3D m_sunDirWorld{-0.35f, 0.75f, 0.45f};
};

} // namespace corv
