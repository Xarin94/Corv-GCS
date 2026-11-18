#include "ui/HudItem.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "core/Geo.h"
#include "gpu/SceneItem.h"
#include "ui/HudInstruments.h"
#include "vehicle/VehicleHub.h"

#include <QQuickWindow>
#include <cmath>

namespace corv {

VehicleHub *HudItem::s_hub = nullptr;

namespace {
const QColor kHudGreen(0, 255, 127);
const QColor kHudBlue(10, 55, 166);   // over the light-theme schematic view
constexpr int kLogEverySeconds = 30;
} // namespace

HudItem::HudItem(QQuickItem *parent) : QQuickItem(parent)
{
    m_clock.start();
    m_statClock.start();

    // Roll frame first: the ladder is the bottom layer, as it was drawn last-but-one
    // in HUDRenderer.js under the fixed symbols
    m_rollFrame = new QQuickItem(this);
    m_rollFrame->setTransformOrigin(QQuickItem::TopLeft);
    m_instruments.push_back(new HudLadder(this, m_rollFrame));
    m_instruments.push_back(new HudSkyPointer(this, m_rollFrame));
    m_instruments.push_back(new HudBankScale(this, this));
    m_instruments.push_back(new HudFpm(this, this));
    m_instruments.push_back(new HudTrackMarker(this, this));
    m_instruments.push_back(new HudBoresight(this, this));
    m_instruments.push_back(new HudHeadingTape(this, this));
    // The heading scale slides inside a clip as wide as the 61 degrees it shows
    m_headingClip = new QQuickItem(this);
    m_headingClip->setClip(true);
    m_instruments.push_back(new HudHeadingStrip(this, m_headingClip));
    m_instruments.push_back(new HudTapeDiamond(this, m_headingClip));
    m_instruments.push_back(new HudNavBug(this, this));
    m_instruments.push_back(new HudVario(this, this));
    m_instruments.push_back(new HudFma(this, this));
    m_messages = new HudMessages(this, this);
    m_instruments.push_back(m_messages);

    // RENDER FPS CAP: with the 3D view pacing the frames, move on its ticks
    connect(FrameTicker::instance(), &FrameTicker::tick, this, [this](double frameWallMs) { frame(frameWallMs); });
}

HudItem::~HudItem() = default;

void HudItem::setHub(VehicleHub *hub) { s_hub = hub; }

void HudItem::setFovDeg(double v)
{
    if (v == m_fovDeg) return;
    m_fovDeg = v;
    emit fovDegChanged();
}

void HudItem::setPitchLocked(bool v)
{
    if (v == m_pitchLocked) return;
    m_pitchLocked = v;
    emit pitchLockedChanged();
}

void HudItem::setSmoothing(double v)
{
    if (v == m_tau) return;
    m_tau = v;
    emit smoothingChanged();
}

void HudItem::setMessagesBottom(double v)
{
    if (v == m_messagesBottom) return;
    m_messagesBottom = v;
    emit messagesBottomChanged();
}

void HudItem::setLightScene(bool v)
{
    if (v == m_lightScene) return;
    m_lightScene = v;
    emit lightSceneChanged();
}

void HudItem::pushMessage(const QString &text, const QString &level)
{
    LOG_I("hud") << level << text;
    m_messages->push(text, level);
}

void HudItem::itemChange(ItemChange change, const ItemChangeData &value)
{
    if (change == ItemSceneChange) {
        disconnect(m_frameConn);
        // One frame computation per frame of the window, before it syncs to the render thread;
        // not while the 3D view paces the frames (RENDER FPS CAP): the ticks move the HUD then
        if (value.window)
            m_frameConn = connect(value.window, &QQuickWindow::afterAnimating, this, [this] {
                if (!SceneItem::pacing()) frame(double(wallMs()));
            });
    }
    QQuickItem::itemChange(change, value);
}

void HudItem::geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry)
{
    QQuickItem::geometryChange(newGeometry, oldGeometry);
    if (window()) window()->update();
}

void HudItem::takeStats()
{
    // Paint time of the frame rendered since the last call
    const qint64 ns = m_paintNs.exchange(0, std::memory_order_relaxed);
    const int paints = m_paints.exchange(0, std::memory_order_relaxed);
    const double ms = ns / 1e6;
    m_frameSumMs += ms;
    m_frameMaxMs = std::max(m_frameMaxMs, ms);
    ++m_frames;
    m_repaints += paints;
    if (m_statClock.elapsed() < 1000) return;
    m_cpuMs = m_frames ? m_frameSumMs / m_frames : 0;
    m_cpuMsMax = m_frameMaxMs;
    m_repaintsPerSecond = int(std::lround(m_repaints * 1000.0 / m_statClock.elapsed()));
    if (--m_logCountdown <= 0) {
        LOG_I("hud") << "paint per frame" << QString::number(m_cpuMs, 'f', 3) << "ms mean," << QString::number(m_cpuMsMax, 'f', 3)
                     << "ms worst," << m_repaintsPerSecond << "instrument repaints/s over" << m_frames << "frames";
        m_logCountdown = kLogEverySeconds;
    }
    m_frameSumMs = m_frameMaxMs = 0;
    m_frames = m_repaints = 0;
    m_statClock.restart();
    emit statsChanged();
}

void HudItem::frame(double nowMs)
{
    if (!s_hub || !isVisible() || width() <= 0 || height() <= 0) return;
    takeStats();

    HudFrame f;
    f.s = s_hub->snapshot();
    const VehicleState &s = *f.s;
    f.pose = m_smoother.update(PoseSmoother::fromState(s), nowMs, m_tau);
    f.W = width();
    f.H = height();
    f.pxPerDeg = 12;   // HUDRenderer.js settings.pixelPerDeg, whatever the window size
    f.pxPerRad = f.pxPerDeg * kRad;
    f.pitchLocked = m_pitchLocked;
    // Locked, the camera stays level: the ladder holds still and the boresight moves
    f.pitchShift = m_pitchLocked ? 0 : f.pose.pitch;
    f.cageX = f.W * 0.35;
    f.cageY = f.H * 0.35;
    f.navFresh = s.navDataTime > 0 && wallMs() - s.navDataTime < 3000;

    if (s.armed != m_prevArmed) {
        if (s.armed) pushMessage(QStringLiteral("ARMED"), QStringLiteral("warning"));
        m_prevArmed = s.armed;
    }

    // Green when armed or in the demo (a simulated airframe in flight), pulsing
    // red/orange when disarmed; the pulse moves in 20 steps so the instruments
    // repaint a few times a second rather than every frame
    if (s.armed || s.isDemo()) {
        f.color = m_lightScene ? kHudBlue : kHudGreen;
    } else {
        m_flash += 0.03;
        const double t = std::round((std::sin(m_flash * 2.5) + 1) / 2 * 20) / 20;
        f.color = QColor(255, int(std::lround(40 + t * 100)), int(std::lround(t * 20)));
    }

    // Bank right = the horizon turns left
    m_rollFrame->setPosition(QPointF(std::round(f.W / 2), std::round(f.H / 2)));
    m_rollFrame->setRotation(-f.pose.roll * kRad);
    const QSizeF scale = HudHeadingTape::scaleSize();
    m_headingClip->setPosition(QPointF(std::round(f.W / 2 - scale.width() / 2), std::round(HudHeadingTape::scaleTop())));
    m_headingClip->setSize(scale);
    for (HudInstrument *i : m_instruments) i->sync(f);
}

} // namespace corv
