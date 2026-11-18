// SysConfig.h - the SYS CONFIG tab ("SysConfig" QML singleton).
//
// Every setting keeps the localStorage key of the Electron version in
// settings.json, so a migrated installation opens with the operator's values:
//   corv.platform (Plan), corv-relnav, adsb-enabled, bat-vmin / bat-vmax,
//   datad-hud-cells, rotor-load-cfg, fpv-settings, log-fft-cfg, gcs-lang,
//   selectedModel, modelScale, ground-clamp-enabled, renderFpsCap,
//   renderQuality, satDetailMaxZoom, satelliteRadiusM, terrainRadiusM, hgt-folder.
// What has a consumer in this version is applied live (navigation, battery,
// ADS-B, HUD cells, terrain and satellite radius, stream rates, HGT folder, offline data);
// the rest is stored for the components still to be ported.
#pragma once

#include <QJsonObject>
#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QUrl>
#include <QVariantList>

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
    Q_PROPERTY(QString hgtStatus READ hgtStatus NOTIFY hgtChanged)
    Q_PROPERTY(double terrainRadius READ terrainRadius NOTIFY viewChanged)
    Q_PROPERTY(double satelliteRadius READ satelliteRadius NOTIFY viewChanged)
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
    QString hgtStatus() const;
    Q_INVOKABLE void selectHgtFolder(const QUrl &folder);
    double terrainRadius() const;
    double satelliteRadius() const;
    Q_INVOKABLE void setTerrainRadius(double meters);

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

    VehicleHub *m_hub;
    CommandSender *m_cmd;
    QTimer m_statusTimer;
    QString m_navStatus = QStringLiteral("--");
    QVariantList m_rates, m_rateState;
    // Offline run
    bool m_offlineBusy = false;
    QString m_offlineStatus;
    double m_offlineProgress = 0;
    struct Pending { double n, s, w, e; int z; bool sat; } m_pendingSat{};
    bool m_satAfterSrtm = false;
};

} // namespace corv
