// RosController.h - ROS surface: settings, rosbridge client, strip (port of
// js/ui/RosController.js; QML singleton "Ros").
//
// Wires SETUP -> TOOLS -> ROS to RosClient (the rosbridge client, sampling and
// averaging, on its own thread) and RosSurfaceData (what the 3D view draws).
// The client takes the vehicle pose itself (the MessageBus and the state);
// the surface is cleared whenever the frame it was placed in changes
// (relative mode switched, a vehicle connecting, which restarts the relative
// frame).
//
// Origin of world-frame points (map, odom): home in absolute mode - a sub's
// surface for ArduSub, whose EKF origin is at 0 m - and zero of the local
// frame in relative mode.
//
// The strip on the flight screen switches the colour (grey; red -> blue by
// distance from the vehicle; roughness) and shows / hides the surface;
// accumulation goes on while it is hidden.
//
// Settings persist under 'ros-config' (the strip's two switches too); the
// ENABLE state does not - like the LiDAR, a link is started deliberately, per
// flight.
#pragma once

#include <QObject>
#include <QQmlEngine>
#include <QThread>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <functional>

namespace corv {

class MessageBus;
class PlanController;
class RosClient;
class VehicleHub;
struct RosTilesMsg;
struct RosVolumeMsg;

class RosController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Ros)
    QML_SINGLETON
    Q_PROPERTY(QVariantMap cfg READ cfg NOTIFY cfgChanged)
    Q_PROPERTY(bool enabled READ enabled NOTIFY enabledChanged)
    Q_PROPERTY(bool active READ active NOTIFY statusChanged)   // the SETUP entry's green dot
    Q_PROPERTY(QVariantList topicOptions READ topicOptions NOTIFY topicsChanged)
    Q_PROPERTY(QString topicTip READ topicTip NOTIFY topicsChanged)
    Q_PROPERTY(QString statusText READ statusText NOTIFY statusChanged)
    Q_PROPERTY(QVariantMap previewZone READ previewZone NOTIFY statusChanged)
    Q_PROPERTY(QString previewCaption READ previewCaption NOTIFY statusChanged)
    Q_PROPERTY(QString stripState READ stripState NOTIFY stripChanged)   // search | hold | accum
    Q_PROPERTY(QString stripLabel READ stripLabel NOTIFY stripChanged)
    Q_PROPERTY(QString stripText READ stripText NOTIFY stripChanged)
    Q_PROPERTY(QString colorLabel READ colorLabel NOTIFY cfgChanged)
    Q_PROPERTY(QString colorMode READ colorMode NOTIFY cfgChanged)
    Q_PROPERTY(bool shown READ shown NOTIFY cfgChanged)

public:
    RosController(VehicleHub *hub, MessageBus *bus, PlanController *plan, QObject *parent = nullptr);
    ~RosController() override;

    static RosController *s_instance;
    static RosController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    QVariantMap cfg() const { return m_cfg; }
    bool enabled() const { return m_enabled; }
    bool active() const { return m_active; }
    QVariantList topicOptions() const { return m_topicOptions; }
    QString topicTip() const { return m_topicTip; }
    QString statusText() const { return m_statusText; }
    QVariantMap previewZone() const { return m_previewZone; }
    QString previewCaption() const { return m_previewCaption; }
    QString stripState() const { return m_stripState; }
    QString stripLabel() const { return m_stripLabel; }
    QString stripText() const { return m_stripText; }
    QString colorMode() const;
    QString colorLabel() const;
    bool shown() const;

    Q_INVOKABLE void setEnabled(bool on);
    /// One setting changed (a SENSOR profile fills in its usual mount, range, sampling and cell).
    Q_INVOKABLE void setField(const QString &key, const QVariant &value);
    /// A mount preset "roll,pitch,yaw".
    Q_INVOKABLE void applyPreset(const QString &rpy);
    Q_INVOKABLE void selectTopic(const QString &name);
    Q_INVOKABLE void refreshTopics();
    Q_INVOKABLE void clearSurface();
    /// The strip's colour button: grey -> distance -> roughness -> grey.
    Q_INVOKABLE void cycleColor();
    Q_INVOKABLE void toggleView();

signals:
    void cfgChanged();
    void enabledChanged();
    void topicsChanged();
    void statusChanged();
    void stripChanged();
    void hudMessage(const QString &text, const QString &level);

private:
    void post(std::function<void(RosClient *)> fn);
    QVariantMap workerConfig() const;
    void saveConfig();
    void applyView();
    void pushConfig();
    void applyProfile(const QString &name);
    void clear(bool announce);
    void onStatus(const QVariantMap &st);
    void onTopics(const QVariantList &list);
    void onTiles(const RosTilesMsg &m);
    void onVolume(const RosVolumeMsg &m);
    void renderStatus(const QVariantMap &st);
    void updateStrip(const QVariantMap &st);
    void updatePreview();
    void sendAreas();
    void tick();

    VehicleHub *m_hub;
    MessageBus *m_bus;
    PlanController *m_plan;
    QThread m_thread;
    RosClient *m_client = nullptr;
    QVariantMap m_cfg;
    bool m_enabled = false, m_active = false;
    QVariantMap m_lastStatus;
    QString m_lastState;
    QVariantList m_topics, m_topicOptions;
    QString m_topicTip;
    QString m_statusText = QStringLiteral("OFF");
    QVariantMap m_previewZone;
    QString m_previewCaption;
    QString m_stripState = QStringLiteral("search"), m_stripLabel = QStringLiteral("OFF"), m_stripText = QStringLiteral("—");
    QString m_sentAreas;
    QTimer m_tick;   // the drawn heights, the colour scales
    bool m_wasRelative = false;
};

} // namespace corv
