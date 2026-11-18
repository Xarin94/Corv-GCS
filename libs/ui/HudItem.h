// HudItem.h - flight instruments drawn over the 3D view (port of js/hud/HUDRenderer.js).
//
// Conformal symbology for a 60 degree vertical field of view: pitch ladder
// and horizon, flight path marker (air-relative, AoA/SSA), ground-track
// diamond, boresight, heading tape with track diamond and nav bug, bank scale
// with sky pointer and slip/skid, VSI, flight mode
// annunciator with waypoint line, wind arrow and the message list. Green when
// armed (or demo), pulsing red/orange when disarmed, dark blue over the
// light-theme schematic view, as in the Electron HUD.
//
// The guide's rule for the HUD cost ("HUD, strumenti e grafici"): a
// QQuickPaintedItem paints in software and uploads a texture, so the HUD is
// not one full-window canvas but a set of instruments, each an item of its
// own size (HudInstruments.h), repainted only when what it shows moves by at
// least a pixel. What only moves is moved by the scene graph: the ladder and
// the sky pointer are painted once and rotated with the roll, the markers are
// translated. HudItem computes the frame once (pose from PoseSmoother at frame
// rate, the 1.8.5 fix; the rest from the latest VehicleState snapshot) and
// hands it to every instrument.
//
// cpuMs / cpuMsMax: the instruments' paint time per frame, mean and worst over
// the last second (the 2 ms ceiling of phase 4); also in the debug log.
#pragma once

#include "vehicle/PoseSmoother.h"
#include "vehicle/VehicleState.h"

#include <QColor>
#include <QElapsedTimer>
#include <QQuickItem>
#include <atomic>
#include <memory>
#include <vector>

namespace corv {

class VehicleHub;
class HudInstrument;
class HudMessages;

/// What every instrument draws from, computed once per frame on the GUI thread.
struct HudFrame {
    std::shared_ptr<const VehicleState> s;
    Pose pose;
    QColor color;
    double W = 0, H = 0;           // HUD size
    double pxPerDeg = 12, pxPerRad = 12 * 57.29577951308232;
    bool pitchLocked = false;
    double pitchShift = 0;         // ladder shift reference (0 in horizon lock)
    bool navFresh = false;         // NAV_CONTROLLER_OUTPUT younger than 3 s
    double cageX = 0, cageY = 0;
    double projectAngle(double rad) const;
};

class HudItem : public QQuickItem
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(double fovDeg READ fovDeg WRITE setFovDeg NOTIFY fovDegChanged)
    Q_PROPERTY(bool pitchLocked READ pitchLocked WRITE setPitchLocked NOTIFY pitchLockedChanged)
    Q_PROPERTY(double smoothing READ smoothing WRITE setSmoothing NOTIFY smoothingChanged)
    /// Height kept free at the bottom-left for the mini-map: messages stack above it
    Q_PROPERTY(double messagesBottom READ messagesBottom WRITE setMessagesBottom NOTIFY messagesBottomChanged)
    /// Light-theme schematic view underneath: the symbology turns dark blue
    Q_PROPERTY(bool lightScene READ lightScene WRITE setLightScene NOTIFY lightSceneChanged)
    Q_PROPERTY(double cpuMs READ cpuMs NOTIFY statsChanged)
    Q_PROPERTY(double cpuMsMax READ cpuMsMax NOTIFY statsChanged)
    Q_PROPERTY(int repaintsPerSecond READ repaintsPerSecond NOTIFY statsChanged)
public:
    explicit HudItem(QQuickItem *parent = nullptr);
    ~HudItem() override;

    static void setHub(VehicleHub *hub);

    double fovDeg() const { return m_fovDeg; }
    void setFovDeg(double v);
    bool pitchLocked() const { return m_pitchLocked; }
    void setPitchLocked(bool v);
    double smoothing() const { return m_tau; }
    void setSmoothing(double v);
    double messagesBottom() const { return m_messagesBottom; }
    void setMessagesBottom(double v);
    bool lightScene() const { return m_lightScene; }
    void setLightScene(bool v);
    double cpuMs() const { return m_cpuMs; }
    double cpuMsMax() const { return m_cpuMsMax; }
    int repaintsPerSecond() const { return m_repaintsPerSecond; }

    /// A line in the message list (bottom-left), as pushHudMessage().
    Q_INVOKABLE void pushMessage(const QString &text, const QString &level = QStringLiteral("info"));

    /// Instruments report their paint time here (render thread).
    void addPaintTime(qint64 ns)
    {
        m_paintNs.fetch_add(ns, std::memory_order_relaxed);
        m_paints.fetch_add(1, std::memory_order_relaxed);
    }

signals:
    void fovDegChanged();
    void pitchLockedChanged();
    void smoothingChanged();
    void messagesBottomChanged();
    void lightSceneChanged();
    void statsChanged();

protected:
    void geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry) override;
    void itemChange(ItemChange change, const ItemChangeData &value) override;

private:
    /// The HUD for a frame shown at nowMs (wall clock).
    void frame(double nowMs);
    void takeStats();

    static VehicleHub *s_hub;
    PoseSmoother m_smoother;
    double m_fovDeg = 60;
    bool m_pitchLocked = false;
    double m_tau = 0.075;
    double m_messagesBottom = 60;
    bool m_lightScene = false;
    double m_flash = 0;
    bool m_prevArmed = false;
    QElapsedTimer m_clock;

    // The roll frame: an empty item at the HUD centre rotated with -roll; the
    // ladder and the sky pointer live in it
    QQuickItem *m_rollFrame = nullptr;
    QQuickItem *m_headingClip = nullptr;
    std::vector<HudInstrument *> m_instruments;
    HudMessages *m_messages = nullptr;
    QMetaObject::Connection m_frameConn;

    // Paint time accounting: per frame from the render thread, rolled up here once a second
    std::atomic<qint64> m_paintNs{0};
    std::atomic<int> m_paints{0};
    QElapsedTimer m_statClock;
    double m_frameSumMs = 0, m_frameMaxMs = 0;
    int m_frames = 0, m_repaints = 0;
    double m_cpuMs = 0, m_cpuMsMax = 0;
    int m_repaintsPerSecond = 0;
    int m_logCountdown = 5;   // first line 5 s after start, then every 30 s
};

} // namespace corv
