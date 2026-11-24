// VehicleHub.h - owner of the VehicleState (rule 2 of the guide: "lo stato si
// legge, non si interroga").
//
// Lives on the link thread next to LinkManager: every message goes through
// StateMapper and a new immutable snapshot is published. The render thread
// takes the latest snapshot every frame, the GUI at 30 Hz (Vehicle facade).
// Setters are thread-safe: they are posted to the link thread.
#pragma once

#include "vehicle/DemoFlight.h"
#include "vehicle/StateMapper.h"
#include "vehicle/VehicleState.h"

#include <QMutex>
#include <QObject>
#include <functional>
#include <memory>

QT_FORWARD_DECLARE_CLASS(QTimer)

namespace corv {

class LinkManager;

class VehicleHub : public QObject
{
    Q_OBJECT
public:
    explicit VehicleHub(LinkManager *link, QObject *parent = nullptr);

    /// The latest state. Any thread; never null.
    std::shared_ptr<const VehicleState> snapshot() const;

    /// Change the state on the link thread (demo flight, FREEZE tests, settings). Publishes after.
    void mutate(std::function<void(VehicleState &)> fn);

    void setMode(const QString &mode);
    void setAdsbEnabled(bool on);
    void setBatteryRange(double vmin, double vmax);
    /// SYS CONFIG -> ALTITUDE OFFSET: added to every altitude drawn (m). A log loaded or closed clears it.
    void setAltitudeOffset(double metres);
    /// What a sub's depth is measured from: the water (else the ground) at a point. Before the first message.
    void setSurface(std::function<std::optional<double>(double lat, double lon)> fn);
    void setRelativeNav(const RelativeNav::Settings &s);
    /// SYS CONFIG → RESET ESTIMATE: the relative position and the dead-reckoned track start again from zero.
    void resetRelativeNav();
    void resetTelemetry();
    /// A stream opened: its first valid position becomes the scene origin.
    void awaitSceneOriginFix();
    void setSceneOrigin(double lat, double lon);
    /// Start the demo flight (60 Hz on the link thread while LIVE and not connected).
    void startDemo(DemoFlight::TerrainFn terrain);

signals:
    void commandAck(int command, int result);
    void statusText(int severity, const QString &text);
    void lastHomeChanged(double lat, double lon);
    void connectionChanged(bool connected, const QString &type);
    /// After every publish (link thread). Cheap listeners only.
    void published();

private:
    void onMessage(const MavMessage &m);
    void publish();
    void newVehicle();
    template <typename F>
    void post(F &&f);

    LinkManager *m_link;
    StateMapper m_mapper;
    StateMapper::Hooks m_hooks;
    VehicleState m_state;
    mutable QMutex m_snapLock;
    std::shared_ptr<const VehicleState> m_snapshot;
    std::unique_ptr<DemoFlight> m_demo;
    QTimer *m_demoTimer = nullptr;
    qint64 m_demoLast = 0;
};

} // namespace corv
