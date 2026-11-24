// SysConfig.h - the SYS CONFIG tab ("SysConfig" QML singleton).
//
// Every setting keeps the localStorage key of the Electron version in
// settings.json, so a migrated installation opens with the operator's values:
//   corv.platform (Plan), corv-relnav, adsb-enabled, bat-vmin / bat-vmax,
//   datad-hud-cells, rotor-load-cfg, fpv-settings, log-fft-cfg, gcs-lang,
//   selectedModel, modelScale, ground-clamp-enabled, renderFpsCap,
//   renderQuality, satDetailMaxZoom, satelliteRadiusM, terrainRadiusM, hgt-folder.
// Every one is applied live. New in this version, under keys of their own:
//   motionSmoothing, mapBrightness (remembered; Electron kept them for the session).
// Not remembered: ALTITUDE OFFSET and TIME OF DAY.
#pragma once

#include <QJsonObject>
#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QUrl>
#include <QVariantList>
#include <functional>

namespace corv {

class CommandSender;
class VehicleHub;

class SysConfig : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(SysConfig)
    QML_SINGLETON

    // NAVIGATION
    Q_PROPERTY(QString navPosition READ navPosition WRITE setNavPosition NOTIFY navChanged)
    Q_PROPERTY(QString navVel READ navVel WRITE setNavVel NOTIFY navChanged)
    Q_PROPERTY(QString navWater READ navWater WRITE setNavWater NOTIFY navChanged)
    Q_PROPERTY(QString navStatus READ navStatus NOTIFY navStatusChanged)
    // AIRSPACE / BATTERY
    Q_PROPERTY(bool adsbEnabled READ adsbEnabled WRITE setAdsbEnabled NOTIFY adsbChanged)
    Q_PROPERTY(QString batVMin READ batVMin NOTIFY batteryChanged)
    Q_PROPERTY(QString batVMax READ batVMax NOTIFY batteryChanged)
    Q_PROPERTY(QString batCells READ batCells NOTIFY batteryChanged)
    // 3D VIEW
    Q_PROPERTY(QStringList models READ models CONSTANT)
    /// The model drawn, as picked here or as the vehicle type set it
    Q_PROPERTY(QString selectedModel READ selectedModel WRITE setSelectedModel NOTIFY modelChanged)
    Q_PROPERTY(QString hgtStatus READ hgtStatus NOTIFY hgtChanged)
    Q_PROPERTY(double terrainRadius READ terrainRadius NOTIFY viewChanged)
    Q_PROPERTY(double satelliteRadius READ satelliteRadius NOTIFY viewChanged)
    Q_PROPERTY(double motionSmoothing READ motionSmoothing WRITE setMotionSmoothing NOTIFY displayChanged)
    /// The blend time constant MOTION SMOOTHING stands for, s: the 3D view and the HUD draw their pose with it
    Q_PROPERTY(double smoothingTau READ smoothingTau NOTIFY displayChanged)
    Q_PROPERTY(double mapBrightness READ mapBrightness WRITE setMapBrightness NOTIFY displayChanged)
    Q_PROPERTY(QString renderQuality READ renderQuality WRITE setRenderQuality NOTIFY displayChanged)
    /// Minutes from midnight the sun is held at, -1 follows the clock
    Q_PROPERTY(int timeOfDay READ timeOfDay WRITE setTimeOfDay NOTIFY timeOfDayChanged)
    Q_PROPERTY(int clockMinutes READ clockMinutes NOTIFY clockChanged)
    // APPLICATION
    Q_PROPERTY(QString language READ language WRITE setLanguage NOTIFY languageChanged)
    /// The settings of CORV GCS 1.x on this computer: when they were last used, empty when there are none.
    Q_PROPERTY(QString legacySettings READ legacySettings CONSTANT)
    // HUD DATA FIELDS
    Q_PROPERTY(QVariantList hudFields READ hudFields CONSTANT)
    Q_PROPERTY(QVariantList hudCells READ hudCells NOTIFY hudCellsChanged)
    // MAVLINK STREAM RATES
    Q_PROPERTY(QVariantList streamRates READ streamRates NOTIFY streamRatesChanged)
    Q_PROPERTY(QVariantList streamRateState READ streamRateState NOTIFY streamRatesChanged)
    // OFFLINE DATA DOWNLOAD
    Q_PROPERTY(bool offlineBusy READ offlineBusy NOTIFY offlineChanged)
    Q_PROPERTY(QString offlineStatus READ offlineStatus NOTIFY offlineChanged)
    Q_PROPERTY(double offlineProgress READ offlineProgress NOTIFY offlineChanged)

public:
    SysConfig(VehicleHub *hub, CommandSender *cmd, QObject *parent);
    static SysConfig *s_instance;
    static SysConfig *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    // Plain stored values (string), and fields of stored JSON objects
    Q_INVOKABLE QString value(const QString &key, const QString &fallback) const;
    Q_INVOKABLE void setValue(const QString &key, const QString &value);
    Q_INVOKABLE QVariant jsonField(const QString &key, const QString &field, const QVariant &fallback) const;
    Q_INVOKABLE void setJsonField(const QString &key, const QString &field, const QVariant &value);

    QString navPosition() const;
    void setNavPosition(const QString &v);
    QString navVel() const;
    void setNavVel(const QString &v);
    QString navWater() const;
    void setNavWater(const QString &v);
    QString navStatus() const { return m_navStatus; }
    Q_INVOKABLE void resetEstimate();

    bool adsbEnabled() const;
    void setAdsbEnabled(bool on);
    QString batVMin() const;
    QString batVMax() const;
    QString batCells() const;
    /// The two fields as typed; invalid values fall back to 9.6 / 12.6 as in the Electron version.
    Q_INVOKABLE void setBatteryRange(const QString &vmin, const QString &vmax);

    QStringList models() const;
    QString selectedModel() const;
    void setSelectedModel(const QString &name);
    QString hgtStatus() const;
    Q_INVOKABLE void selectHgtFolder(const QUrl &folder);
    double terrainRadius() const;
    double satelliteRadius() const;
    Q_INVOKABLE void setTerrainRadius(double meters);
    double motionSmoothing() const;
    void setMotionSmoothing(double v);
    double smoothingTau() const;
    double mapBrightness() const;
    void setMapBrightness(double v);
    QString renderQuality() const;
    void setRenderQuality(const QString &q);
    int timeOfDay() const { return m_timeOfDay; }
    void setTimeOfDay(int minutes);
    int clockMinutes() const;
    /// ALTITUDE OFFSET as typed (parseFloat(...) || 0); Vehicle.altOffset is the one in use.
    Q_INVOKABLE void setAltitudeOffset(const QString &text);

    QString legacySettings() const;
    /// Asked first; the ones read only at start need a restart.
    Q_INVOKABLE void importLegacySettings();
    /// The interface's language, "en" or "zh" (gcs-lang): changed at once.
    QString language() const;
    void setLanguage(const QString &code);

    QVariantList hudFields() const;
    QVariantList hudCells() const;
    Q_INVOKABLE void setHudCellField(int index, const QString &field);
    Q_INVOKABLE void setHudCellMultiplier(int index, const QString &text);
    Q_INVOKABLE void setHudCellUnit(int index, const QString &unit);
    Q_INVOKABLE void resetHudCells();

    QVariantList streamRates() const { return m_rates; }
    QVariantList streamRateState() const { return m_rateState; }
    Q_INVOKABLE void setStreamRate(int index, const QString &text);
    Q_INVOKABLE void readStreamRates();
    Q_INVOKABLE void writeStreamRates();

    Q_INVOKABLE QString offlineEstimate(const QString &n, const QString &s, const QString &w, const QString &e, const QString &zoom,
                                        bool sat, bool srtm) const;
    Q_INVOKABLE void startOffline(const QString &n, const QString &s, const QString &w, const QString &e, const QString &zoom, bool sat, bool srtm);
    Q_INVOKABLE void cancelOffline();
    bool offlineBusy() const { return m_offlineBusy; }
    QString offlineStatus() const { return m_offlineStatus; }
    double offlineProgress() const { return m_offlineProgress; }

    Q_INVOKABLE void saveDebugLogCopy(const QUrl &file);
    Q_INVOKABLE void revealDebugLog();
    Q_INVOKABLE QString debugLogName() const;

signals:
    /// A JSON setting changed through setJsonField (log-fft-cfg reaches LOG REPLAY this way).
    void jsonFieldChanged(const QString &key);
    void navChanged();
    void navStatusChanged();
    void adsbChanged();
    void batteryChanged();
    void hgtChanged();
    void viewChanged();
    void displayChanged();
    void timeOfDayChanged();
    void modelChanged();
    void languageChanged();
    void clockChanged();
    void hudCellsChanged();
    void streamRatesChanged();
    void offlineChanged();
    void toast(const QString &text);

private:
    QJsonObject relNav() const;
    void applyRelNav();
    void applyFrameCap();
    QJsonArray hudCellsJson() const;
    void saveHudCells(const QJsonArray &a);
    void startSatellite();
    void withRatePrefix(std::function<void(bool known)> then);
    void flashRate(int i, const QString &state);

    VehicleHub *m_hub;
    CommandSender *m_cmd;
    QTimer m_statusTimer;
    QString m_navStatus = QStringLiteral("--");
    int m_timeOfDay = -1;
    int m_clockMinute = -1;
    QVariantList m_rates, m_rateState;
    QString m_ratePrefix;   // SR0_ or MAV1_, once the vehicle told
    // Offline run
    bool m_offlineBusy = false;
    QString m_offlineStatus;
    double m_offlineProgress = 0;
    struct Pending { double n, s, w, e; int z; bool sat; } m_pendingSat{};
    bool m_satAfterSrtm = false;
};

} // namespace corv
