// LidarController.h - Livox Mid-360 point cloud: settings and flight strip
// (port of js/ui/LidarController.js, js/lidar/LidarCloud.js and
// js/lidar/LidarDemo.js; QML singleton "Lidar").
//
// Wires the LIDAR page (SETUP -> TOOLS) - connection, mount attitude relative
// to the autopilot IMU, filters, navigation-quality gate, display, recording -
// to LidarCore on its thread, and the strip on the flight screen (state LED,
// point count, CLEAR MAP / SAVE) that is only shown while the cloud is
// enabled. Points and origin events go straight to PointCloudData.
//
// Settings persist under 'lidar-config'; the ENABLE state does not - a LiDAR
// link is started deliberately, per flight.
//
// In the demo flight (no link) enabling the cloud starts a synthetic scan
// instead of the network client: every frame a few hundred rays leave the demo
// aircraft and are intersected with the real SRTM heightfield, so the cloud
// that builds up is the actual relief under the circuit - fictitious points,
// real terrain - through the same cloud, strip and CLEAR MAP. Sensor model: a
// Mid-360 with its spin axis along the fuselage; the 360 deg sweep fans
// around the flight axis and the -7...+52 deg band leans that fan forward: a
// cross-track push-broom. Range stretched to 150 m so the swath reaches beyond
// the runway edges from 50 m AGL.
#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QQmlEngine>
#include <QSet>
#include <QThread>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <array>
#include <functional>

namespace corv {

class LidarCore;
class MessageBus;
class VehicleHub;

class LidarController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Lidar)
    QML_SINGLETON
    Q_PROPERTY(QVariantMap cfg READ cfg NOTIFY cfgChanged)
    Q_PROPERTY(bool enabled READ enabled NOTIFY enabledChanged)
    Q_PROPERTY(bool demo READ demo NOTIFY enabledChanged)
    Q_PROPERTY(bool active READ active NOTIFY statusChanged)   // the SETUP entry's green dot
    Q_PROPERTY(QString statusText READ statusText NOTIFY statusChanged)
    Q_PROPERTY(QString stripState READ stripState NOTIFY stripChanged)   // search | hold | accum | full | error
    Q_PROPERTY(QString stripLabel READ stripLabel NOTIFY stripChanged)
    Q_PROPERTY(QString stripCount READ stripCount NOTIFY stripChanged)
    Q_PROPERTY(QVariantList hostOptions READ hostOptions NOTIFY hostOptionsChanged)

public:
    LidarController(MessageBus *bus, VehicleHub *hub, QObject *parent = nullptr);
    ~LidarController() override;

    static LidarController *s_instance;
    static LidarController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    QVariantMap cfg() const { return m_cfg; }
    bool enabled() const { return m_enabled; }
    bool demo() const { return m_demoRunning; }
    bool active() const { return m_active; }
    QString statusText() const { return m_statusText; }
    QString stripState() const { return m_stripState; }
    QString stripLabel() const { return m_stripLabel; }
    QString stripCount() const { return m_stripCount; }
    QVariantList hostOptions() const { return m_hostOptions; }

    /// The POINT CLOUD checkbox.
    Q_INVOKABLE void setEnabled(bool on);
    /// One setting changed: saved, and pushed to the client live (mount, filters,
    /// gate thresholds take effect on the next packet; IP / format on reconnect).
    Q_INVOKABLE void setField(const QString &key, const QVariant &value);
    /// A mount preset "roll,pitch,yaw".
    Q_INVOKABLE void applyPreset(const QString &rpy);
    Q_INVOKABLE void clearMap();
    Q_INVOKABLE void saveMap();
    Q_INVOKABLE void refreshInterfaces();

signals:
    void cfgChanged();
    void enabledChanged();
    void statusChanged();
    void stripChanged();
    void hostOptionsChanged();
    void hudMessage(const QString &text, const QString &level);

private:
    void call(std::function<QVariantMap(LidarCore *)> fn, std::function<void(const QVariantMap &)> done = {});
    QVariantMap coreConfig() const;
    void saveConfig();
    void applyDisplayConfig();
    void onStatus(const QVariantMap &st);
    void renderStatus(const QVariantMap &st);
    void updateStrip(const QVariantMap &st);
    void setStrip(const QString &state, const QString &label);
    void updateCount();
    void startDemo();
    void stopDemo();
    void demoTick();
    void setActive(bool a);

    MessageBus *m_bus;
    VehicleHub *m_hub;
    QThread m_thread;
    LidarCore *m_core = nullptr;
    QVariantMap m_cfg;
    bool m_enabled = false;
    bool m_active = false;
    QVariantMap m_lastStatus;
    int m_lastGateOk = -1;   // -1 unknown
    QString m_statusText = QStringLiteral("OFF");
    QString m_stripState = QStringLiteral("search"), m_stripLabel = QStringLiteral("OFF"), m_stripCount = QStringLiteral("0 pts");
    QVariantList m_hostOptions;
    QTimer m_rangeTimer;   // the height ramp while the cloud shows

    // Demo scan
    bool m_demoRunning = false;
    QTimer m_demoTimer, m_demoCountTimer;
    QElapsedTimer m_demoClock;
    double m_demoLastT = 0, m_demoCarry = 0;
    bool m_demoOrigin = false;
    double m_demoOLat = 0, m_demoOLon = 0, m_demoOAlt = 0, m_demoMPerLat = 0, m_demoMPerLon = 0;
    int m_demoEpoch = 100;   // separate range from the client's epochs
    QSet<qint64> m_demoVoxels;
    qint64 m_demoTotal = 0;
};

} // namespace corv
