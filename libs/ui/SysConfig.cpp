#include "ui/SysConfig.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "gpu/SceneItem.h"
#include "terrain/Hgt.h"
#include "terrain/HgtDownloader.h"
#include "terrain/TileStore.h"
#include "vehicle/CommandSender.h"
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
#include <cmath>

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

// SR0_* of the autopilot's USB port and the streams / messages WRITE also sets on the link in use
struct Rate { const char *param; int def; int stream; int msgs[2]; };
const Rate kRates[] = {
    {"SR0_RAW_SENS", 2, 1, {27, -1}}, {"SR0_EXT_STAT", 2, 2, {-1, -1}}, {"SR0_RC_CHAN", 5, 3, {-1, -1}}, {"SR0_RAW_CTRL", 1, 4, {-1, -1}},
    {"SR0_POSITION", 3, 6, {-1, -1}}, {"SR0_EXTRA1", 10, 10, {30, 31}}, {"SR0_EXTRA2", 10, 11, {74, -1}}, {"SR0_EXTRA3", 1, 12, {241, -1}},
    {"SR0_PARAMS", 10, -1, {-1, -1}},
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
        a.append(QJsonObject{{QStringLiteral("field"), QString::fromLatin1(k)}, {QStringLiteral("multiplier"), 1}, {QStringLiteral("unitLabel"), QString()}});
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
    const QString folder = Settings::instance().value(QStringLiteral("hgt-folder")).toString();
    if (!folder.isEmpty()) HgtStore::instance().addSearchDir(folder);

    // Twice a second: where the position comes from (TabController.initNavigationPanel)
    m_statusTimer.setInterval(500);
    connect(&m_statusTimer, &QTimer::timeout, this, [this] {
        QString text = QStringLiteral("absolute · GPS");
        if (m_hub) {
            const auto s = m_hub->snapshot();
            static const QHash<QString, QString> vel{{QStringLiteral("air"), QStringLiteral("airspeed + heading")},
                                                     {QStringLiteral("ground"), QStringLiteral("ground speed + heading")},
                                                     {QStringLiteral("ekf"), QStringLiteral("EKF velocity")}};
            if (s->relativeSource == QLatin1String("local")) text = QStringLiteral("relative · vehicle EKF position");
            else if (!s->relativeSource.isEmpty())
                text = QStringLiteral("relative · dead reckoning (%1)").arg(vel.value(s->relativeVelSource, QStringLiteral("no velocity yet")));
        }
        if (text != m_navStatus) { m_navStatus = text; emit navStatusChanged(); }
    });
    m_statusTimer.start();

    connect(&HgtStore::instance(), &HgtStore::tileLoaded, this, [this] { emit hgtChanged(); }, Qt::QueuedConnection);
    connect(&HgtDownloader::instance(), &HgtDownloader::tileDone, this, [this] { emit hgtChanged(); });

    // Offline download progress: SRTM first, then the satellite tiles
    connect(&HgtDownloader::instance(), &HgtDownloader::areaProgress, this, [this](int dl, int, int nf, int err, int total) {
        if (!m_offlineBusy) return;
        m_offlineStatus = QStringLiteral("SRTM: %1 downloaded, %2 no data, %3 errors / %4").arg(dl).arg(nf).arg(err).arg(total);
        m_offlineProgress = total ? double(dl + nf + err) / total : 1;
        emit offlineChanged();
    });
    connect(&HgtDownloader::instance(), &HgtDownloader::areaFinished, this, [this](int dl, int, int, int, int, bool cancelled) {
        if (!m_offlineBusy) return;
        if (cancelled) { m_offlineBusy = false; m_offlineStatus = QStringLiteral("Cancelled"); emit offlineChanged(); return; }
        m_offlineStatus = QStringLiteral("SRTM: %1 downloaded").arg(dl);
        emit offlineChanged();
        emit hgtChanged();
        if (m_satAfterSrtm) startSatellite();
        else { m_offlineBusy = false; m_offlineStatus = QStringLiteral("Done! SRTM: %1 downloaded").arg(dl); emit offlineChanged(); }
    });
    connect(&TileStore::instance(), &TileStore::prefetchProgress, this, [this](int dl, int failed, int total) {
        if (!m_offlineBusy) return;
        m_offlineStatus = QStringLiteral("Satellite: %1 / %2%3").arg(dl).arg(total).arg(failed ? QStringLiteral(" (%1 failed)").arg(failed) : QString());
        m_offlineProgress = total ? double(dl + failed) / total : 1;
        emit offlineChanged();
    });
    connect(&TileStore::instance(), &TileStore::prefetchFinished, this, [this](int dl, int, int, bool cancelled) {
        if (!m_offlineBusy) return;
        m_offlineBusy = false;
        m_offlineStatus = cancelled ? QStringLiteral("Cancelled") : QStringLiteral("Done! Sat: %1 tiles").arg(dl);
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
    if (key == QLatin1String("terrainRadiusM") || key == QLatin1String("satelliteRadiusM")) emit viewChanged();
    if (key == QLatin1String("renderFpsCap")) applyFrameCap();
}

// RENDER FPS CAP: 60 by default, 30 in eco mode (setRenderFpsCap of main.js)
void SysConfig::applyFrameCap()
{
    SceneItem::setFrameCap(value(QStringLiteral("renderFpsCap"), QStringLiteral("60")) == QLatin1String("30") ? 30 : 60);
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
    s.relative = o.value(QStringLiteral("relative")).toBool(false);
    s.velSource = o.value(QStringLiteral("velSource")).toString(QStringLiteral("auto"));
    s.water = o.value(QStringLiteral("water")).toString(QStringLiteral("fresh"));
    if (m_hub) m_hub->setRelativeNav(s);
}

QString SysConfig::navPosition() const { return relNav().value(QStringLiteral("relative")).toBool() ? QStringLiteral("relative") : QStringLiteral("absolute"); }
QString SysConfig::navVel() const { return relNav().value(QStringLiteral("velSource")).toString(QStringLiteral("auto")); }
QString SysConfig::navWater() const { return relNav().value(QStringLiteral("water")).toString(QStringLiteral("fresh")); }

void SysConfig::setNavPosition(const QString &v)
{
    QJsonObject o = relNav();
    o.insert(QStringLiteral("relative"), v == QLatin1String("relative"));
    Settings::instance().setJson(QLatin1String(kRelNav), o);
    applyRelNav();
    emit navChanged();
}

void SysConfig::setNavVel(const QString &v)
{
    QJsonObject o = relNav();
    o.insert(QStringLiteral("velSource"), v);
    Settings::instance().setJson(QLatin1String(kRelNav), o);
    applyRelNav();
    emit navChanged();
}

void SysConfig::setNavWater(const QString &v)
{
    QJsonObject o = relNav();
    o.insert(QStringLiteral("water"), v);
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

bool SysConfig::adsbEnabled() const { return Settings::instance().value(QStringLiteral("adsb-enabled")).toString() != QLatin1String("0"); }

void SysConfig::setAdsbEnabled(bool on)
{
    Settings::instance().setValue(QStringLiteral("adsb-enabled"), on ? QStringLiteral("1") : QStringLiteral("0"));
    if (m_hub) m_hub->setAdsbEnabled(on);
    emit adsbChanged();
}

QString SysConfig::batVMin() const { return QString::number(Settings::instance().value(QStringLiteral("bat-vmin"), 9.6).toDouble()); }
QString SysConfig::batVMax() const { return QString::number(Settings::instance().value(QStringLiteral("bat-vmax"), 12.6).toDouble()); }
QString SysConfig::batCells() const
{
    return QStringLiteral("%1S").arg(qRound(Settings::instance().value(QStringLiteral("bat-vmax"), 12.6).toDouble() / 4.2));
}

void SysConfig::setBatteryRange(const QString &vminText, const QString &vmaxText)
{
    bool ok1 = false, ok2 = false;
    double vmin = QLocale::c().toDouble(vminText.trimmed(), &ok1);
    double vmax = QLocale::c().toDouble(vmaxText.trimmed(), &ok2);
    if (!ok1 || vmin <= 0) vmin = 9.6;
    if (!ok2 || vmax <= 0) vmax = 12.6;
    // vmax <= vmin would divide by zero / invert the estimate downstream
    if (vmax <= vmin) { vmin = 9.6; vmax = 12.6; }
    Settings::instance().setValue(QStringLiteral("bat-vmin"), vmin);
    Settings::instance().setValue(QStringLiteral("bat-vmax"), vmax);
    if (m_hub) m_hub->setBatteryRange(vmin, vmax);
    emit batteryChanged();
}

// ── 3D view ──────────────────────────────────────────────────────────────────

QStringList SysConfig::models() const
{
    // The .glb models shipped with the installation (resources/models in development)
    QStringList names;
    for (const QString &dir : {DataRoot::installDir() + QStringLiteral("/models"), DataRoot::installDir() + QStringLiteral("/resources/models"),
#ifdef CORV_SOURCE_DIR
                               QStringLiteral(CORV_SOURCE_DIR "/resources/models"),
#endif
                               QStringLiteral(":/models")}) {
        for (const QFileInfo &fi : QDir(dir).entryInfoList({QStringLiteral("*.glb")}, QDir::Files))
            if (!names.contains(fi.completeBaseName())) names << fi.completeBaseName();
        if (!names.isEmpty()) break;
    }
    names.sort();
    return names;
}

QString SysConfig::hgtStatus() const
{
    const int n = HgtStore::instance().availableCount();
    return n ? QStringLiteral("%1 HGT AVAILABLE (lazy)").arg(n) : QStringLiteral("NO DATA");
}

void SysConfig::selectHgtFolder(const QUrl &folder)
{
    const QString dir = folder.toLocalFile();
    if (dir.isEmpty()) return;
    Settings::instance().setValue(QStringLiteral("hgt-folder"), dir);
    HgtStore::instance().addSearchDir(dir);
    emit hgtChanged();
}

double SysConfig::terrainRadius() const
{
    const double v = Settings::instance().value(QStringLiteral("terrainRadiusM"), 35000).toDouble();
    return v == 50000 || v == 70000 ? v : 35000;
}

double SysConfig::satelliteRadius() const
{
    // SATELLITE_RADIUS_OPTIONS of TerrainManager.js; anything else falls back to the default
    const double v = Settings::instance().value(QStringLiteral("satelliteRadiusM"), 10000).toDouble();
    for (double o : {5000.0, 10000.0, 15000.0, 20000.0, 30000.0})
        if (v == o) return v;
    return 10000;
}

void SysConfig::setTerrainRadius(double meters)
{
    Settings::instance().setValue(QStringLiteral("terrainRadiusM"), QString::number(qRound64(meters)));
    emit viewChanged();
}

// ── HUD data fields ──────────────────────────────────────────────────────────

QVariantList SysConfig::hudFields() const
{
    QVariantList out;
    for (const HudField &f : kHudFields)
        out << QVariantMap{{QStringLiteral("key"), QString::fromLatin1(f.key)}, {QStringLiteral("label"), QString::fromUtf8(f.label)},
                           {QStringLiteral("unit"), QString::fromUtf8(f.unit)}};
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
    for (const QJsonValue &v : a) valid = valid && isHudField(v.toObject().value(QStringLiteral("field")).toString());
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
        const QString field = o.value(QStringLiteral("field")).toString();
        QString label, unit;
        for (const HudField &f : kHudFields)
            if (field == QLatin1String(f.key)) { label = QString::fromUtf8(f.label); unit = QString::fromUtf8(f.unit); }
        const double mult = o.value(QStringLiteral("multiplier")).toDouble(1);
        const QString unitLabel = o.value(QStringLiteral("unitLabel")).toString();
        out << QVariantMap{{QStringLiteral("field"), field}, {QStringLiteral("label"), label},
                           {QStringLiteral("multiplier"), mult ? mult : 1.0},
                           {QStringLiteral("multText"), mult && mult != 1 ? QString::number(mult) : QString()},
                           {QStringLiteral("unitLabel"), unitLabel},
                           {QStringLiteral("unit"), unitLabel.isEmpty() ? unit : unitLabel}};
    }
    return out;
}

void SysConfig::setHudCellField(int index, const QString &field)
{
    QJsonArray a = hudCellsJson();
    if (index < 0 || index >= a.size() || !isHudField(field)) return;
    QJsonObject o = a[index].toObject();
    o.insert(QStringLiteral("field"), field);
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
    o.insert(QStringLiteral("multiplier"), ok && std::isfinite(m) && m != 0 ? m : 1.0);   // parseFloat(...) || 1
    a[index] = o;
    saveHudCells(a);
}

void SysConfig::setHudCellUnit(int index, const QString &unit)
{
    QJsonArray a = hudCellsJson();
    if (index < 0 || index >= a.size()) return;
    QJsonObject o = a[index].toObject();
    o.insert(QStringLiteral("unitLabel"), unit);
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

void SysConfig::readStreamRates()
{
    if (!m_cmd) return;
    QPointer<SysConfig> self(this);
    for (int i = 0; i < int(std::size(kRates)); ++i) {
        m_cmd->fetchParameter(QString::fromLatin1(kRates[i].param), 2000, 2, [self, i](bool ok, float value, int) {
            if (!self || !ok) return;
            self->m_rates[i] = int(std::lround(value));
            emit self->streamRatesChanged();
        });
    }
}

void SysConfig::writeStreamRates()
{
    if (!m_cmd) return;
    QPointer<SysConfig> self(this);
    for (int i = 0; i < int(std::size(kRates)); ++i) {
        const Rate &r = kRates[i];
        const int rate = m_rates[i].toInt();
        // SRn_* are AP_Int16
        m_cmd->setParameter(QString::fromLatin1(r.param), float(rate), MAV_PARAM_TYPE_INT16, [self, i, r, rate](bool ok, float) {
            if (!self) return;
            self->m_rateState[i] = ok ? QStringLiteral("ok") : QStringLiteral("error");
            emit self->streamRatesChanged();
            if (ok && r.stream >= 0) {
                // The rates of the link in use: the stream group, and the messages pinned by requestAllDataStreams
                self->m_cmd->requestDataStream(r.stream, rate, rate > 0 ? 1 : 0);
                for (int id : r.msgs) if (id >= 0) self->m_cmd->setMessageInterval(uint32_t(id), rate);
            }
            QTimer::singleShot(2000, self, [self, i] {
                if (!self) return;
                self->m_rateState[i] = QString();
                emit self->streamRatesChanged();
            });
        });
    }
}

// ── Offline data download ────────────────────────────────────────────────────

QString SysConfig::offlineEstimate(const QString &nT, const QString &sT, const QString &wT, const QString &eT, const QString &zT, bool sat, bool srtm) const
{
    bool o1, o2, o3, o4, o5;
    const double n = nT.toDouble(&o1), s = sT.toDouble(&o2), w = wT.toDouble(&o3), e = eT.toDouble(&o4);
    int z = zT.toInt(&o5);
    if (!(o1 && o2 && o3 && o4)) return QStringLiteral("Invalid bounds (numbers required)");
    if (n <= s || e <= w) return QStringLiteral("Invalid bounds (North > South, East > West)");
    if (n > 85 || s < -85 || w < -180 || e > 180) return QStringLiteral("Bounds out of range (lat ±85, lon ±180)");
    if (!o5) z = 16;
    z = std::clamp(z, kSatMinZoom, 17);
    const qint64 satTiles = sat ? TileStore::tileCount(n, s, w, e, kSatMinZoom, z) : 0;
    const int srtmTiles = srtm ? HgtDownloader::tileCount(s, n, w, e) : 0;
    QStringList parts;
    const QLocale loc(QLocale::English);
    if (satTiles > 0) parts << QStringLiteral("~%1 sat tiles").arg(loc.toString(satTiles));
    if (srtmTiles > 0) parts << QStringLiteral("%1 SRTM tiles (~%2 MB)").arg(srtmTiles).arg(loc.toString(srtmTiles * 25));
    if (satTiles > kMaxSatTiles || srtmTiles > kMaxSrtmTiles) parts << QStringLiteral("— AREA TOO LARGE, reduce bounds or zoom");
    return parts.isEmpty() ? QStringLiteral("Nothing selected") : parts.join(QStringLiteral(" + "));
}

void SysConfig::startOffline(const QString &nT, const QString &sT, const QString &wT, const QString &eT, const QString &zT, bool sat, bool srtm)
{
    const QString est = offlineEstimate(nT, sT, wT, eT, zT, sat, srtm);
    if (est.startsWith(QStringLiteral("Invalid")) || est.startsWith(QStringLiteral("Bounds"))) { m_offlineStatus = est; emit offlineChanged(); return; }
    if (est.contains(QStringLiteral("TOO LARGE"))) { m_offlineStatus = QStringLiteral("Area too large — reduce bounds or zoom level"); emit offlineChanged(); return; }
    if (!sat && !srtm) return;
    bool ok = false;
    int z = zT.toInt(&ok);
    if (!ok) z = 16;
    m_pendingSat = {nT.toDouble(), sT.toDouble(), wT.toDouble(), eT.toDouble(), std::clamp(z, kSatMinZoom, 17), sat};
    m_satAfterSrtm = sat;
    m_offlineBusy = true;
    m_offlineProgress = 0;
    m_offlineStatus = QStringLiteral("Starting...");
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
    TileStore::instance().prefetchArea(QStringLiteral("satellite"), m_pendingSat.n, m_pendingSat.s, m_pendingSat.w, m_pendingSat.e, kSatMinZoom, m_pendingSat.z);
}

void SysConfig::cancelOffline()
{
    if (!m_offlineBusy) return;
    m_satAfterSrtm = false;
    HgtDownloader::instance().cancelArea();
    TileStore::instance().cancelPrefetch();
    m_offlineBusy = false;
    m_offlineStatus = QStringLiteral("Cancelled");
    emit offlineChanged();
}

// ── Debug log ────────────────────────────────────────────────────────────────

QString SysConfig::debugLogName() const
{
    return QStringLiteral("corv-gcs-debug-%1.log").arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss")));
}

void SysConfig::saveDebugLogCopy(const QUrl &file)
{
    DebugLog::flushSync();
    const QString target = file.toLocalFile();
    QFile::remove(target);
    if (QFile::copy(DebugLog::filePath(), target)) emit toast(QStringLiteral("Debug log saved: %1").arg(target));
    else emit toast(QStringLiteral("Could not save the debug log to %1").arg(target));
}

void SysConfig::revealDebugLog()
{
    DebugLog::flushSync();
    QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(DebugLog::filePath()).absolutePath()));
}

} // namespace corv
