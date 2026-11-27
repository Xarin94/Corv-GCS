#include "ui/SysConfig.h"

#include "ui/DialogHub.h"
#include "ui/Language.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/ElectronSettings.h"
#include "core/Settings.h"
#include "gpu/SceneItem.h"
#include "scene/RenderQuality.h"
#include "terrain/Hgt.h"
#include "terrain/HgtDownloader.h"
#include "terrain/TileStore.h"
#include "vehicle/CommandSender.h"
#include "vehicle/PoseSmoother.h"
#include "vehicle/VehicleHub.h"

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QJsonDocument>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QLocale>
#include <QPointer>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

SysConfig *SysConfig::s_instance = nullptr;

namespace {

const char *kRelNav = "corv-relnav";
const char *kHudCells = "datad-hud-cells";
constexpr int kSatMinZoom = 11;           // OfflineDownloader.js
constexpr qint64 kMaxSatTiles = 300000;   // ~4-6 GB of imagery
constexpr int kMaxSrtmTiles = 200;        // ~5 GB of elevation data

// The fields a HUD cell can show (HUD_FIELD_DEFS of UIController.js)
struct HudField { const char *key, *label, *unit; };
const HudField kHudFields[] = {
    {"as", "Airspeed", "m/s"}, {"gs", "Ground Spd", "m/s"}, {"vs", "Vert Speed", "m/s"}, {"gload", "G-Load", "G"},
    {"alt", "Altitude", "m MSL"}, {"agl", "Terr Alt", "m AGL"}, {"lidar", "LiDAR", "m AGL"}, {"roll", "Roll", "°"},
    {"pitch", "Pitch", "°"}, {"yaw", "Heading", "°"}, {"battV", "Battery V", "V"}, {"battA", "Battery A", "A"},
    {"battPct", "Battery %", "%"}, {"gpsSat", "GPS Sats", ""}, {"gpsHdop", "GPS HDOP", ""}, {"linkQ", "Link Qual", "%"},
    {"vibX", "Vib X", ""}, {"vibY", "Vib Y", ""}, {"vibZ", "Vib Z", ""}, {"aoa", "AoA", "°"}, {"ssa", "SSA", "°"},
    {"rtkBaseline", "RTK Base", "mm"}, {"rtkAccuracy", "RTK Acc", "mm"},
};
const char *kDefaultCells[6] = {"as", "gs", "gload", "alt", "agl", "lidar"};

// SR0_* of the autopilot's first MAVLink port (normally its USB), named MAV1_* from firmware
// 4.7 on, and the streams / messages WRITE also sets on the link in use
const char *kRatePrefixes[] = {"SR0_", "MAV1_"};
struct Rate { const char *param; int def; int stream; int msgs[2]; };
const Rate kRates[] = {
    {"RAW_SENS", 2, 1, {27, -1}}, {"EXT_STAT", 2, 2, {-1, -1}}, {"RC_CHAN", 5, 3, {-1, -1}}, {"RAW_CTRL", 1, 4, {-1, -1}},
    {"POSITION", 3, 6, {-1, -1}}, {"EXTRA1", 10, 10, {30, 31}}, {"EXTRA2", 10, 11, {74, -1}}, {"EXTRA3", 1, 12, {241, -1}},
    {"PARAMS", 10, -1, {-1, -1}},
};

bool isHudField(const QString &k)
{
    for (const HudField &f : kHudFields) if (k == QLatin1String(f.key)) return true;
    return false;
}

QJsonArray defaultCells()
{
    QJsonArray a;
    for (const char *k : kDefaultCells)
        a.append(QJsonObject{{u"field"_s, QString::fromLatin1(k)}, {u"multiplier"_s, 1}, {u"unitLabel"_s, QString()}});
    return a;
}

} // namespace

SysConfig::SysConfig(VehicleHub *hub, CommandSender *cmd, QObject *parent) : QObject(parent), m_hub(hub), m_cmd(cmd)
{
    s_instance = this;
    for (const Rate &r : kRates) { m_rates << r.def; m_rateState << QString(); }

    // Apply what the vehicle side and the 3D views need at start
    applyRelNav();
    applyFrameCap();
    if (m_hub) m_hub->setAdsbEnabled(adsbEnabled());
    const QString folder = Settings::instance().value(u"hgt-folder"_s).toString();
    if (!folder.isEmpty()) HgtStore::instance().addSearchDir(folder);

    // Another vehicle may have the other names
    if (m_hub) connect(m_hub, &VehicleHub::connectionChanged, this, [this] { m_ratePrefix.clear(); }, Qt::QueuedConnection);

    // Twice a second: where the position comes from (TabController.initNavigationPanel)
    m_statusTimer.setInterval(500);
    connect(&m_statusTimer, &QTimer::timeout, this, [this] {
        QString text = u"absolute · GPS"_s;
        if (m_hub) {
            const auto s = m_hub->snapshot();
            static const QHash<QString, QString> vel{{u"air"_s, u"airspeed + heading"_s},
                                                     {u"ground"_s, u"ground speed + heading"_s},
                                                     {u"ekf"_s, u"EKF velocity"_s}};
            if (s->relativeSource == "local"_L1) text = u"relative · vehicle EKF position"_s;
            else if (!s->relativeSource.isEmpty())
                text = u"relative · dead reckoning (%1)"_s.arg(vel.value(s->relativeVelSource, u"no velocity yet"_s));
        }
        if (text != m_navStatus) { m_navStatus = text; emit navStatusChanged(); }
        if (const int minute = clockMinutes(); minute != m_clockMinute) { m_clockMinute = minute; emit clockChanged(); }
    });
    m_statusTimer.start();

    // The 3D view follows the vehicle type: the select with it
    connect(&Settings::instance(), &Settings::changed, this, [this](const QString &key) {
        if (key == "selectedModel"_L1) emit modelChanged();
    });
    connect(&HgtStore::instance(), &HgtStore::tileLoaded, this, [this] { emit hgtChanged(); }, Qt::QueuedConnection);
    connect(&HgtDownloader::instance(), &HgtDownloader::tileDone, this, [this] { emit hgtChanged(); });

    // Offline download progress: SRTM first, then the satellite tiles
    connect(&HgtDownloader::instance(), &HgtDownloader::areaProgress, this, [this](int dl, int, int nf, int err, int total) {
        if (!m_offlineBusy) return;
        m_offlineStatus = u"SRTM: %1 downloaded, %2 no data, %3 errors / %4"_s.arg(dl).arg(nf).arg(err).arg(total);
        m_offlineProgress = total ? double(dl + nf + err) / total : 1;
        emit offlineChanged();
    });
    connect(&HgtDownloader::instance(), &HgtDownloader::areaFinished, this, [this](int dl, int, int, int, int, bool cancelled) {
        if (!m_offlineBusy) return;
        if (cancelled) { m_offlineBusy = false; m_offlineStatus = u"Cancelled"_s; emit offlineChanged(); return; }
        m_offlineStatus = u"SRTM: %1 downloaded"_s.arg(dl);
        emit offlineChanged();
        emit hgtChanged();
        if (m_satAfterSrtm) startSatellite();
        else { m_offlineBusy = false; m_offlineStatus = u"Done! SRTM: %1 downloaded"_s.arg(dl); emit offlineChanged(); }
    });
    connect(&TileStore::instance(), &TileStore::prefetchProgress, this, [this](int dl, int failed, int total) {
        if (!m_offlineBusy) return;
        m_offlineStatus = u"Satellite: %1 / %2%3"_s.arg(dl).arg(total).arg(failed ? u" (%1 failed)"_s.arg(failed) : QString());
        m_offlineProgress = total ? double(dl + failed) / total : 1;
        emit offlineChanged();
    });
    connect(&TileStore::instance(), &TileStore::prefetchFinished, this, [this](int dl, int, int, bool cancelled) {
        if (!m_offlineBusy) return;
        m_offlineBusy = false;
        m_offlineStatus = cancelled ? u"Cancelled"_s : u"Done! Sat: %1 tiles"_s.arg(dl);
        emit offlineChanged();
    });
}

// ── Generic values ───────────────────────────────────────────────────────────

QString SysConfig::value(const QString &key, const QString &fallback) const
{
    const QVariant v = Settings::instance().value(key);
    return v.isValid() && !v.toString().isEmpty() ? v.toString() : fallback;
}

void SysConfig::setValue(const QString &key, const QString &value)
{
    Settings::instance().setValue(key, value);
    if (key == "terrainRadiusM"_L1 || key == "satelliteRadiusM"_L1) emit viewChanged();
    if (key == "renderFpsCap"_L1) applyFrameCap();
}

// RENDER FPS CAP: 60 by default, 30 in eco mode (setRenderFpsCap of main.js)
void SysConfig::applyFrameCap()
{
    SceneItem::setFrameCap(value(u"renderFpsCap"_s, u"60"_s) == "30"_L1 ? 30 : 60);
}

QVariant SysConfig::jsonField(const QString &key, const QString &field, const QVariant &fallback) const
{
    QJsonObject o = Settings::instance().json(key).toObject();
    if (o.isEmpty()) {
        // Some keys were stored as JSON text by the Electron version
        const QString raw = Settings::instance().value(key).toString();
        if (!raw.isEmpty()) o = QJsonDocument::fromJson(raw.toUtf8()).object();
    }
    return o.contains(field) ? o.value(field).toVariant() : fallback;
}

void SysConfig::setJsonField(const QString &key, const QString &field, const QVariant &value)
{
    QJsonObject o = Settings::instance().json(key).toObject();
    if (o.isEmpty()) {
        const QString raw = Settings::instance().value(key).toString();
        if (!raw.isEmpty()) o = QJsonDocument::fromJson(raw.toUtf8()).object();
    }
    o.insert(field, QJsonValue::fromVariant(value));
    Settings::instance().setJson(key, o);
    emit jsonFieldChanged(key);
}

// ── Navigation ───────────────────────────────────────────────────────────────

QJsonObject SysConfig::relNav() const
{
    QJsonObject o = Settings::instance().json(QLatin1String(kRelNav)).toObject();
    if (o.isEmpty()) {
        const QString raw = Settings::instance().value(QLatin1String(kRelNav)).toString();
        if (!raw.isEmpty()) o = QJsonDocument::fromJson(raw.toUtf8()).object();
    }
    return o;
}

void SysConfig::applyRelNav()
{
    const QJsonObject o = relNav();
    RelativeNav::Settings s;
    s.relative = o.value(u"relative"_s).toBool(false);
    s.velSource = o.value(u"velSource"_s).toString(u"auto"_s);
    s.water = o.value(u"water"_s).toString(u"fresh"_s);
    if (m_hub) m_hub->setRelativeNav(s);
}

QString SysConfig::navPosition() const { return relNav().value(u"relative"_s).toBool() ? u"relative"_s : u"absolute"_s; }
QString SysConfig::navVel() const { return relNav().value(u"velSource"_s).toString(u"auto"_s); }
QString SysConfig::navWater() const { return relNav().value(u"water"_s).toString(u"fresh"_s); }

void SysConfig::setNavPosition(const QString &v)
{
    QJsonObject o = relNav();
    o.insert(u"relative"_s, v == "relative"_L1);
    Settings::instance().setJson(QLatin1String(kRelNav), o);
    applyRelNav();
    emit navChanged();
}

void SysConfig::setNavVel(const QString &v)
{
    QJsonObject o = relNav();
    o.insert(u"velSource"_s, v);
    Settings::instance().setJson(QLatin1String(kRelNav), o);
    applyRelNav();
    emit navChanged();
}

void SysConfig::setNavWater(const QString &v)
{
    QJsonObject o = relNav();
    o.insert(u"water"_s, v);
    Settings::instance().setJson(QLatin1String(kRelNav), o);
    applyRelNav();
    emit navChanged();
}

void SysConfig::resetEstimate()
{
    if (m_hub) m_hub->resetRelativeNav();
    LOG_I("nav") << "relative estimate reset";
}

// ── Airspace / battery ───────────────────────────────────────────────────────

bool SysConfig::adsbEnabled() const { return Settings::instance().value(u"adsb-enabled"_s).toString() != "0"_L1; }

void SysConfig::setAdsbEnabled(bool on)
{
    Settings::instance().setValue(u"adsb-enabled"_s, on ? u"1"_s : u"0"_s);
    if (m_hub) m_hub->setAdsbEnabled(on);
    emit adsbChanged();
}

namespace {
constexpr int kMaxCells = 14;
double cellVMin() { return Settings::instance().value(u"bat-cell-vmin"_s, StateMapper::kCellEmptyV).toDouble(); }
double cellVMax() { return Settings::instance().value(u"bat-cell-vmax"_s, StateMapper::kCellFullV).toDouble(); }
int fixedCells() { return std::clamp(Settings::instance().value(u"bat-cells"_s, 0).toInt(), 0, kMaxCells); }
} // namespace

QString SysConfig::batCellVMin() const { return QString::number(cellVMin()); }
QString SysConfig::batCellVMax() const { return QString::number(cellVMax()); }
QString SysConfig::batCells() const
{
    const int n = fixedCells();
    return n > 0 ? QString::number(n) : u"auto"_s;
}

void SysConfig::setBatCells(const QString &v)
{
    const int n = v == "auto"_L1 ? 0 : std::clamp(v.toInt(), 0, kMaxCells);
    if (n == fixedCells()) return;
    Settings::instance().setValue(u"bat-cells"_s, n);
    if (m_hub) m_hub->setBatteryGauge(cellVMin(), cellVMax(), n);
    emit batteryChanged();
}

void SysConfig::setCellRange(const QString &vminText, const QString &vmaxText)
{
    bool ok1 = false, ok2 = false;
    double vmin = QLocale::c().toDouble(vminText.trimmed().replace(','_L1, '.'_L1), &ok1);
    double vmax = QLocale::c().toDouble(vmaxText.trimmed().replace(','_L1, '.'_L1), &ok2);
    if (!ok1 || vmin <= 0 || vmin > 5) vmin = StateMapper::kCellEmptyV;
    if (!ok2 || vmax <= 0 || vmax > 5) vmax = StateMapper::kCellFullV;
    // vmax <= vmin would divide by zero / invert the estimate downstream
    if (vmax <= vmin) { vmin = StateMapper::kCellEmptyV; vmax = StateMapper::kCellFullV; }
    Settings::instance().setValue(u"bat-cell-vmin"_s, vmin);
    Settings::instance().setValue(u"bat-cell-vmax"_s, vmax);
    if (m_hub) m_hub->setBatteryGauge(vmin, vmax, fixedCells());
    emit batteryChanged();
}

void SysConfig::loadBatteryGauge(VehicleHub *hub)
{
    Settings &st = Settings::instance();
    // The Electron version kept a pack range (bat-vmin / bat-vmax) and only showed vmax / 4.2
    // as the count. A range set by hand becomes that many cells, fixed, and volts per cell;
    // the untouched default (9.6 / 12.6) becomes the automatic count.
    if (!st.contains(u"bat-cell-vmin"_s) && st.contains(u"bat-vmax"_s)) {
        const double vmin = st.value(u"bat-vmin"_s, 9.6).toDouble(), vmax = st.value(u"bat-vmax"_s, 12.6).toDouble();
        const bool byHand = std::abs(vmin - 9.6) > 1e-6 || std::abs(vmax - 12.6) > 1e-6;
        const int n = std::clamp(int(std::lround(vmax / 4.2)), 1, kMaxCells);
        if (byHand && vmax > vmin && vmin > 0) {
            st.setValue(u"bat-cells"_s, n);
            st.setValue(u"bat-cell-vmin"_s, vmin / n);
            st.setValue(u"bat-cell-vmax"_s, vmax / n);
        } else {
            st.setValue(u"bat-cell-vmin"_s, StateMapper::kCellEmptyV);
            st.setValue(u"bat-cell-vmax"_s, StateMapper::kCellFullV);
        }
        LOG_I("battery") << "gauge from the pack range" << vmin << "-" << vmax << "V:"
                         << (byHand ? u"%1 cells fixed"_s.arg(n) : u"automatic count"_s);
    }
    if (hub) hub->setBatteryGauge(cellVMin(), cellVMax(), fixedCells());
}

// ── 3D view ──────────────────────────────────────────────────────────────────

QStringList SysConfig::models() const
{
    // The .glb models shipped with the installation (resources/models in development)
    QStringList names;
    for (const QString &dir : {DataRoot::installDir() + u"/models"_s, DataRoot::installDir() + u"/resources/models"_s,
#ifdef CORV_SOURCE_DIR
                               QStringLiteral(CORV_SOURCE_DIR "/resources/models"),
#endif
                               u":/models"_s}) {
        for (const QFileInfo &fi : QDir(dir).entryInfoList({u"*.glb"_s}, QDir::Files))
            if (!names.contains(fi.completeBaseName())) names << fi.completeBaseName();
        if (!names.isEmpty()) break;
    }
    names.sort();
    return names;
}

QString SysConfig::selectedModel() const { return value(u"selectedModel"_s, u"plane.glb"_s); }

void SysConfig::setSelectedModel(const QString &name) { Settings::instance().setValue(u"selectedModel"_s, name); }

QString SysConfig::hgtStatus() const
{
    const int n = HgtStore::instance().availableCount();
    return n ? u"%1 HGT AVAILABLE (lazy)"_s.arg(n) : u"NO DATA"_s;
}

void SysConfig::selectHgtFolder(const QUrl &folder)
{
    const QString dir = folder.toLocalFile();
    if (dir.isEmpty()) return;
    Settings::instance().setValue(u"hgt-folder"_s, dir);
    HgtStore::instance().addSearchDir(dir);
    emit hgtChanged();
}

double SysConfig::terrainRadius() const
{
    const double v = Settings::instance().value(u"terrainRadiusM"_s, 35000).toDouble();
    return v == 50000 || v == 70000 ? v : 35000;
}

double SysConfig::satelliteRadius() const
{
    // SATELLITE_RADIUS_OPTIONS of TerrainManager.js; anything else falls back to the default
    const double v = Settings::instance().value(u"satelliteRadiusM"_s, 10000).toDouble();
    for (double o : {5000.0, 10000.0, 15000.0, 20000.0, 30000.0})
        if (v == o) return v;
    return 10000;
}

void SysConfig::setTerrainRadius(double meters)
{
    Settings::instance().setValue(u"terrainRadiusM"_s, QString::number(qRound64(meters)));
    emit viewChanged();
}

double SysConfig::motionSmoothing() const
{
    const double v = Settings::instance().value(u"motionSmoothing"_s, PoseSmoother::kDefaultSmoothing).toDouble();
    return std::isfinite(v) ? std::clamp(v, 0.0, PoseSmoother::kMaxSmoothing) : PoseSmoother::kDefaultSmoothing;
}

void SysConfig::setMotionSmoothing(double v)
{
    v = std::clamp(std::round(v * 100) / 100, 0.0, PoseSmoother::kMaxSmoothing);
    if (v == motionSmoothing()) return;
    Settings::instance().setValue(u"motionSmoothing"_s, v);
    emit displayChanged();
}

double SysConfig::smoothingTau() const { return PoseSmoother::tauFor(motionSmoothing()); }

double SysConfig::mapBrightness() const
{
    const double v = Settings::instance().value(u"mapBrightness"_s, 0.85).toDouble();
    return std::isfinite(v) && v > 0 ? std::clamp(v, 0.3, 1.6) : 0.85;
}

void SysConfig::setMapBrightness(double v)
{
    v = std::clamp(std::round(v * 100) / 100, 0.3, 1.6);
    if (v == mapBrightness()) return;
    Settings::instance().setValue(u"mapBrightness"_s, v);
    emit displayChanged();
}

QString SysConfig::renderQuality() const { return renderQuality::normalize(Settings::instance().value(u"renderQuality"_s).toString()); }

void SysConfig::setRenderQuality(const QString &q)
{
    if (renderQuality::normalize(q) == renderQuality()) return;
    Settings::instance().setValue(u"renderQuality"_s, renderQuality::normalize(q));
    emit displayChanged();
}

int SysConfig::clockMinutes() const
{
    const QTime t = QTime::currentTime();
    return t.hour() * 60 + t.minute();
}

void SysConfig::setTimeOfDay(int minutes)
{
    minutes = minutes < 0 ? -1 : std::min(minutes, 24 * 60);
    if (minutes == m_timeOfDay) return;
    m_timeOfDay = minutes;
    emit timeOfDayChanged();
}

void SysConfig::setAltitudeOffset(const QString &text)
{
    bool ok = false;
    QString t = text.trimmed();
    t.replace(','_L1, '.'_L1);
    const double v = QLocale::c().toDouble(t, &ok);
    if (m_hub) m_hub->setAltitudeOffset(ok && std::isfinite(v) ? v : 0);
}

// ── Language ─────────────────────────────────────────────────────────────────

QString SysConfig::legacySettings() const
{
    const QString dir = electronSettings::findStore();
    return dir.isEmpty() ? QString() : u"Last used %1"_s.arg(QLocale().toString(electronSettings::lastUsed(dir), QLocale::ShortFormat));
}

void SysConfig::importLegacySettings()
{
    QString error;
    const auto store = electronSettings::read(electronSettings::findStore(), &error);
    int n = 0;
    for (const QString &k : electronSettings::knownKeys()) n += store.contains(k);
    if (!n) {
        DialogHub::toast(error.isEmpty() ? u"No settings of 1.x to import"_s : error);
        return;
    }
    DialogHub::confirm(u"Import %1 settings of CORV GCS 1.x? They replace the same settings here; restart the program to apply them all."_s.arg(n),
                       [store](bool ok) {
                           if (!ok) return;
                           const int done = electronSettings::importInto(store);
                           DialogHub::toast(u"%1 settings imported: restart to apply them all"_s.arg(done));
                       });
}

QString SysConfig::language() const { return Language::instance().code(); }

void SysConfig::setLanguage(const QString &code)
{
    if (code == language()) return;
    Language::instance().set(code);
    Settings::instance().setValue(u"gcs-lang"_s, language());
    emit languageChanged();
}

// ── HUD data fields ──────────────────────────────────────────────────────────

QVariantList SysConfig::hudFields() const
{
    QVariantList out;
    for (const HudField &f : kHudFields)
        out << QVariantMap{{u"key"_s, QString::fromLatin1(f.key)}, {u"label"_s, QString::fromUtf8(f.label)},
                           {u"unit"_s, QString::fromUtf8(f.unit)}};
    return out;
}

QJsonArray SysConfig::hudCellsJson() const
{
    QJsonArray a = Settings::instance().json(QLatin1String(kHudCells)).toArray();
    if (a.isEmpty()) {
        const QString raw = Settings::instance().value(QLatin1String(kHudCells)).toString();
        if (!raw.isEmpty()) a = QJsonDocument::fromJson(raw.toUtf8()).array();
    }
    bool valid = a.size() == 6;
    for (const QJsonValue &v : a) valid = valid && isHudField(v.toObject().value(u"field"_s).toString());
    return valid ? a : defaultCells();
}

void SysConfig::saveHudCells(const QJsonArray &a)
{
    Settings::instance().setJson(QLatin1String(kHudCells), a);
    emit hudCellsChanged();
}

QVariantList SysConfig::hudCells() const
{
    QVariantList out;
    for (const QJsonValue &v : hudCellsJson()) {
        const QJsonObject o = v.toObject();
        const QString field = o.value(u"field"_s).toString();
        QString label, unit;
        for (const HudField &f : kHudFields)
            if (field == QLatin1String(f.key)) { label = QString::fromUtf8(f.label); unit = QString::fromUtf8(f.unit); }
        const double mult = o.value(u"multiplier"_s).toDouble(1);
        const QString unitLabel = o.value(u"unitLabel"_s).toString();
        out << QVariantMap{{u"field"_s, field}, {u"label"_s, label},
                           {u"multiplier"_s, mult ? mult : 1.0},
                           {u"multText"_s, mult && mult != 1 ? QString::number(mult) : QString()},
                           {u"unitLabel"_s, unitLabel},
                           {u"unit"_s, unitLabel.isEmpty() ? unit : unitLabel}};
    }
    return out;
}

void SysConfig::setHudCellField(int index, const QString &field)
{
    QJsonArray a = hudCellsJson();
    if (index < 0 || index >= a.size() || !isHudField(field)) return;
    QJsonObject o = a[index].toObject();
    o.insert(u"field"_s, field);
    a[index] = o;
    saveHudCells(a);
}

void SysConfig::setHudCellMultiplier(int index, const QString &text)
{
    QJsonArray a = hudCellsJson();
    if (index < 0 || index >= a.size()) return;
    bool ok = false;
    const double m = QLocale::c().toDouble(text.trimmed(), &ok);
    QJsonObject o = a[index].toObject();
    o.insert(u"multiplier"_s, ok && std::isfinite(m) && m != 0 ? m : 1.0);   // parseFloat(...) || 1
    a[index] = o;
    saveHudCells(a);
}

void SysConfig::setHudCellUnit(int index, const QString &unit)
{
    QJsonArray a = hudCellsJson();
    if (index < 0 || index >= a.size()) return;
    QJsonObject o = a[index].toObject();
    o.insert(u"unitLabel"_s, unit);
    a[index] = o;
    saveHudCells(a);
}

void SysConfig::resetHudCells() { saveHudCells(defaultCells()); }

// ── Stream rates ─────────────────────────────────────────────────────────────

void SysConfig::setStreamRate(int index, const QString &text)
{
    if (index < 0 || index >= m_rates.size()) return;
    bool ok = false;
    const int v = text.trimmed().toInt(&ok);
    if (ok) m_rates[index] = std::clamp(v, 0, 50);
}

// Which of the two names this firmware has: asked once per connection, with the first rate
void SysConfig::withRatePrefix(std::function<void(bool known)> then)
{
    if (!m_ratePrefix.isEmpty()) return then(true);
    QPointer<SysConfig> self(this);
    auto left = std::make_shared<int>(int(std::size(kRatePrefixes)));
    auto done = std::make_shared<bool>(false);
    for (const char *prefix : kRatePrefixes) {
        const QString p = QLatin1String(prefix);
        m_cmd->fetchParameter(p + QLatin1String(kRates[0].param), 1500, 2, [self, p, left, done, then](bool ok, float value, int) {
            --*left;
            if (!self || *done) return;
            if (ok) {
                *done = true;
                self->m_ratePrefix = p;
                self->m_rates[0] = int(std::lround(value));
                emit self->streamRatesChanged();
                then(true);
            } else if (*left == 0) {
                then(false);
            }
        });
    }
}

void SysConfig::flashRate(int i, const QString &state)
{
    m_rateState[i] = state;
    emit streamRatesChanged();
    QTimer::singleShot(2000, this, [this, i] {
        m_rateState[i] = QString();
        emit streamRatesChanged();
    });
}

void SysConfig::readStreamRates()
{
    if (!m_cmd) return;
    QPointer<SysConfig> self(this);
    withRatePrefix([self](bool known) {
        if (!self) return;
        for (int i = 0; i < int(std::size(kRates)); ++i) {
            if (!known) {
                self->flashRate(i, u"error"_s);   // nobody answered
                continue;
            }
            self->m_cmd->fetchParameter(self->m_ratePrefix + QLatin1String(kRates[i].param), 2000, 2, [self, i](bool ok, float value, int) {
                if (!self) return;
                if (!ok) return self->flashRate(i, u"error"_s);
                self->m_rates[i] = int(std::lround(value));
                emit self->streamRatesChanged();
            });
        }
    });
}

void SysConfig::writeStreamRates()
{
    if (!m_cmd) return;
    QPointer<SysConfig> self(this);
    // As typed: the probe for the parameter names writes the first one over
    const QVariantList wanted = m_rates;
    withRatePrefix([self, wanted](bool known) {
        if (!self) return;
        self->m_rates = wanted;
        for (int i = 0; i < int(std::size(kRates)); ++i) {
            if (!known) {
                self->flashRate(i, u"error"_s);
                continue;
            }
            const Rate &r = kRates[i];
            const int rate = wanted[i].toInt();
            // SRn_* and MAVn_* are AP_Int16
            self->m_cmd->setParameter(self->m_ratePrefix + QLatin1String(r.param), float(rate), MAV_PARAM_TYPE_INT16, [self, i, r, rate](bool ok, float) {
                if (!self) return;
                if (ok && r.stream >= 0) {
                    // The rates of the link in use: the stream group, and the messages pinned by requestAllDataStreams
                    self->m_cmd->requestDataStream(r.stream, rate, rate > 0 ? 1 : 0);
                    for (int id : r.msgs) if (id >= 0) self->m_cmd->setMessageInterval(uint32_t(id), rate);
                }
                self->flashRate(i, ok ? u"ok"_s : u"error"_s);
            });
        }
        emit self->streamRatesChanged();
    });
}

// ── Offline data download ────────────────────────────────────────────────────

QString SysConfig::offlineEstimate(const QString &nT, const QString &sT, const QString &wT, const QString &eT, const QString &zT, bool sat, bool srtm) const
{
    bool o1, o2, o3, o4, o5;
    const double n = nT.toDouble(&o1), s = sT.toDouble(&o2), w = wT.toDouble(&o3), e = eT.toDouble(&o4);
    int z = zT.toInt(&o5);
    if (!(o1 && o2 && o3 && o4)) return u"Invalid bounds (numbers required)"_s;
    if (n <= s || e <= w) return u"Invalid bounds (North > South, East > West)"_s;
    if (n > 85 || s < -85 || w < -180 || e > 180) return u"Bounds out of range (lat ±85, lon ±180)"_s;
    if (!o5) z = 16;
    z = std::clamp(z, kSatMinZoom, 17);
    const qint64 satTiles = sat ? TileStore::tileCount(n, s, w, e, kSatMinZoom, z) : 0;
    const int srtmTiles = srtm ? HgtDownloader::tileCount(s, n, w, e) : 0;
    QStringList parts;
    const QLocale loc(QLocale::English);
    if (satTiles > 0) parts << u"~%1 sat tiles"_s.arg(loc.toString(satTiles));
    if (srtmTiles > 0) parts << u"%1 SRTM tiles (~%2 MB)"_s.arg(srtmTiles).arg(loc.toString(srtmTiles * 25));
    if (satTiles > kMaxSatTiles || srtmTiles > kMaxSrtmTiles) parts << u"— AREA TOO LARGE, reduce bounds or zoom"_s;
    return parts.isEmpty() ? u"Nothing selected"_s : parts.join(u" + "_s);
}

void SysConfig::startOffline(const QString &nT, const QString &sT, const QString &wT, const QString &eT, const QString &zT, bool sat, bool srtm)
{
    const QString est = offlineEstimate(nT, sT, wT, eT, zT, sat, srtm);
    if (est.startsWith(u"Invalid"_s) || est.startsWith(u"Bounds"_s)) { m_offlineStatus = est; emit offlineChanged(); return; }
    if (est.contains(u"TOO LARGE"_s)) { m_offlineStatus = u"Area too large — reduce bounds or zoom level"_s; emit offlineChanged(); return; }
    if (!sat && !srtm) return;
    bool ok = false;
    int z = zT.toInt(&ok);
    if (!ok) z = 16;
    m_pendingSat = {nT.toDouble(), sT.toDouble(), wT.toDouble(), eT.toDouble(), std::clamp(z, kSatMinZoom, 17), sat};
    m_satAfterSrtm = sat;
    m_offlineBusy = true;
    m_offlineProgress = 0;
    m_offlineStatus = u"Starting..."_s;
    emit offlineChanged();
    LOG_I("offline") << "download N" << m_pendingSat.n << "S" << m_pendingSat.s << "W" << m_pendingSat.w << "E" << m_pendingSat.e
                     << "zoom" << m_pendingSat.z << "sat" << sat << "srtm" << srtm;
    // Phase 1: SRTM, phase 2: satellite
    if (srtm) {
        HgtDownloader::instance().resetFailures();
        HgtDownloader::instance().downloadArea(m_pendingSat.s, m_pendingSat.n, m_pendingSat.w, m_pendingSat.e);
    } else {
        startSatellite();
    }
}

void SysConfig::startSatellite()
{
    m_satAfterSrtm = false;
    TileStore::instance().prefetchArea(u"satellite"_s, m_pendingSat.n, m_pendingSat.s, m_pendingSat.w, m_pendingSat.e, kSatMinZoom, m_pendingSat.z);
}

void SysConfig::cancelOffline()
{
    if (!m_offlineBusy) return;
    m_satAfterSrtm = false;
    HgtDownloader::instance().cancelArea();
    TileStore::instance().cancelPrefetch();
    m_offlineBusy = false;
    m_offlineStatus = u"Cancelled"_s;
    emit offlineChanged();
}

// ── Debug log ────────────────────────────────────────────────────────────────

QString SysConfig::debugLogName() const
{
    return u"corv-gcs-debug-%1.log"_s.arg(QDateTime::currentDateTime().toString(u"yyyyMMdd-HHmmss"_s));
}

void SysConfig::saveDebugLogCopy(const QUrl &file)
{
    DebugLog::flushSync();
    const QString target = file.toLocalFile();
    QFile::remove(target);
    if (QFile::copy(DebugLog::filePath(), target)) emit toast(u"Debug log saved: %1"_s.arg(target));
    else emit toast(u"Could not save the debug log to %1"_s.arg(target));
}

void SysConfig::revealDebugLog()
{
    DebugLog::flushSync();
    QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(DebugLog::filePath()).absolutePath()));
}

} // namespace corv
