#include "ui/LteController.h"

#include "core/Clock.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "link/LteLink.h"
#include "services/CellDb.h"
#include "ui/DialogHub.h"
#include "vehicle/CommandSender.h"
#include "vehicle/ConnectionManager.h"
#include "vehicle/VehicleHub.h"
#include "vehicle/VehicleState.h"

#include <QDir>
#include <QFile>
#include <QFutureWatcher>
#include <QLocale>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QtConcurrent>
#include <cmath>

#if defined(Q_OS_WIN)
#include <windows.h>
#include <dpapi.h>
#elif defined(Q_OS_LINUX)
#include <dlfcn.h>
#endif

using namespace Qt::StringLiterals;

namespace corv {

namespace {

const QString kConfigKey = u"lte-link-config"_s;
const QString kCellMapKey = u"lte-cell-map"_s;
const QString kDefaultRelayHost = u"94.177.202.247"_s;   // CORV relay
constexpr std::int64_t kCellStaleMs = 10000;   // no CORV_CELL_INFO for this long: overlay off
constexpr std::int64_t kStaleLinkMs = 5000;    // no CORV_CELL_INFO for this long: link poor
constexpr double kRadiusKm = 100;              // sites fetched around the vehicle
constexpr double kRequeryKm = 10;
constexpr double kShowKm = 10;                 // sites drawn besides the serving one
constexpr double kRefilterKm = 0.25;

// DOWNLOAD 2024 COPY boxes (south, west, north, east) and the MCC for DOWNLOAD FROM OPENCELLID
struct Country {
    const char *name;
    double box[4];
    int mcc;
};
const Country kCountries[] = {
    {"Italy", {35.4, 6.6, 47.1, 18.6}, 222},     {"Switzerland", {45.8, 5.9, 47.9, 10.5}, 228}, {"Austria", {46.4, 9.5, 49.0, 17.2}, 232},
    {"Slovenia", {45.4, 13.4, 46.9, 16.6}, 293}, {"France", {42.3, -4.8, 51.1, 8.2}, 208},     {"Germany", {47.3, 5.9, 55.1, 15.0}, 262},
    {"Spain", {36.0, -9.3, 43.8, 3.3}, 214},
};
const Country *country(const QString &name)
{
    for (const Country &c : kCountries)
        if (name == QLatin1String(c.name)) return &c;
    return nullptr;
}

// Where a cell position comes from (CellRecord::src)
QString sourceName(const QString &src)
{
    static const QHash<QString, QString> names{{u"ocid"_s, u"OpenCelliD"_s},
                                              {u"ocid-2024"_s, u"OpenCelliD 2024"_s},
                                              {u"mls"_s, u"Mozilla MLS 2024"_s},
                                              {u"beacondb"_s, u"beaconDB (online)"_s},
                                              {u"ocid-api"_s, u"OpenCelliD API (online)"_s},
                                              {u"bundle"_s, u"built-in"_s}};
    return names.value(src, src);
}

double kmBetween(const LatLon &a, double lat, double lon) { return celldb::distanceKm(a.lat, a.lon, lat, lon); }

QJsonObject loadJson(const QString &key)
{
    QJsonObject o = Settings::instance().json(key).toObject();
    if (o.isEmpty()) {
        const QString raw = Settings::instance().value(key).toString();
        if (!raw.isEmpty()) o = QJsonDocument::fromJson(raw.toUtf8()).object();
    }
    return o;
}

QString formatBytes(qint64 n)
{
    if (n >= 1000000) return QString::number(n / 1e6, 'f', 1) + u" MB"_s;
    if (n >= 1000) return QString::number(n / 1e3, 'f', 1) + u" kB"_s;
    return QString::number(n) + u" B"_s;
}

// ── OS keychain for REMEMBER KEY (Electron safeStorage) ─────────────────────

namespace keychain {

#if defined(Q_OS_WIN)

QString file() { return QDir(DataRoot::root()).filePath(u"lte-link-key-qt.bin"_s); }
bool available() { return true; }
bool save(const QString &key)
{
    const QByteArray plain = key.toUtf8();
    DATA_BLOB in{DWORD(plain.size()), reinterpret_cast<BYTE *>(const_cast<char *>(plain.constData()))}, out{};
    if (!CryptProtectData(&in, L"CORV GCS LTE key", nullptr, nullptr, nullptr, CRYPTPROTECT_UI_FORBIDDEN, &out)) return false;
    QFile f(file());
    const bool ok = f.open(QIODevice::WriteOnly) && f.write(reinterpret_cast<const char *>(out.pbData), out.cbData) == qint64(out.cbData);
    LocalFree(out.pbData);
    return ok;
}
QString load()
{
    QFile f(file());
    if (!f.open(QIODevice::ReadOnly)) return {};
    QByteArray blob = f.readAll();
    DATA_BLOB in{DWORD(blob.size()), reinterpret_cast<BYTE *>(blob.data())}, out{};
    if (!CryptUnprotectData(&in, nullptr, nullptr, nullptr, nullptr, CRYPTPROTECT_UI_FORBIDDEN, &out)) return {};
    const QString key = QString::fromUtf8(reinterpret_cast<const char *>(out.pbData), int(out.cbData));
    SecureZeroMemory(out.pbData, out.cbData);
    LocalFree(out.pbData);
    return key;
}
void forget() { QFile::remove(file()); }

#elif defined(Q_OS_LINUX)

// libsecret, loaded at run time (no build dependency): the Secret Service of
// GNOME Keyring or KWallet. The schema mirrors libsecret's SecretSchema ABI.
struct SchemaAttribute {
    const char *name;
    int type;   // SECRET_SCHEMA_ATTRIBUTE_STRING = 0
};
struct Schema {
    const char *name;
    int flags;
    SchemaAttribute attributes[32];
    int reserved;
    void *reserved1, *reserved2, *reserved3, *reserved4, *reserved5, *reserved6, *reserved7;
};
using StoreFn = int (*)(const Schema *, const char *, const char *, const char *, void *, void **, ...);
using LookupFn = char *(*)(const Schema *, void *, void **, ...);
using ClearFn = int (*)(const Schema *, void *, void **, ...);
using FreeFn = void (*)(char *);

struct Lib {
    StoreFn store = nullptr;
    LookupFn lookup = nullptr;
    ClearFn clear = nullptr;
    FreeFn free = nullptr;
    bool ok = false;
};

const Lib &lib()
{
    static const Lib l = [] {
        Lib r;
        if (qEnvironmentVariableIsEmpty("DBUS_SESSION_BUS_ADDRESS")) return r;
        void *h = dlopen("libsecret-1.so.0", RTLD_NOW | RTLD_LOCAL);
        if (!h) return r;
        r.store = reinterpret_cast<StoreFn>(dlsym(h, "secret_password_store_sync"));
        r.lookup = reinterpret_cast<LookupFn>(dlsym(h, "secret_password_lookup_sync"));
        r.clear = reinterpret_cast<ClearFn>(dlsym(h, "secret_password_clear_sync"));
        r.free = reinterpret_cast<FreeFn>(dlsym(h, "secret_password_free"));
        r.ok = r.store && r.lookup && r.clear && r.free;
        return r;
    }();
    return l;
}

const Schema &schema()
{
    static const Schema s{"org.corv.gcs.LteKey", 0, {{"purpose", 0}, {nullptr, 0}}, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    return s;
}

bool available() { return lib().ok; }
bool save(const QString &key)
{
    if (!lib().ok) return false;
    const QByteArray k = key.toUtf8();
    return lib().store(&schema(), "default", "CORV GCS cellular link key", k.constData(), nullptr, nullptr, "purpose", "lte-link", nullptr) != 0;
}
QString load()
{
    if (!lib().ok) return {};
    char *p = lib().lookup(&schema(), nullptr, nullptr, "purpose", "lte-link", nullptr);
    if (!p) return {};
    const QString key = QString::fromUtf8(p);
    lib().free(p);
    return key;
}
void forget()
{
    if (lib().ok) lib().clear(&schema(), nullptr, nullptr, "purpose", "lte-link", nullptr);
}

#else

bool available() { return false; }
bool save(const QString &) { return false; }
QString load() { return {}; }
void forget() {}

#endif

} // namespace keychain

const QHash<int, QString> &operators222()
{
    static const QHash<int, QString> o{{1, u"TIM"_s}, {10, u"Vodafone"_s}, {50, u"Iliad"_s},
                                       {88, u"WindTre"_s}, {99, u"WindTre (3)"_s}, {43, u"TIM"_s},
                                       {48, u"TIM"_s}};
    return o;
}

QString v(const std::optional<int> &x, const QString &unit = QString())
{
    if (!x) return u"-"_s;
    return unit.isEmpty() ? QString::number(*x) : u"%1 %2"_s.arg(*x).arg(unit);
}

} // namespace

LteController *LteController::s_instance = nullptr;

LteController::LteController(LinkManager *link, ConnectionManager *conn, CommandSender *cmd, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_link(link), m_conn(conn), m_cmd(cmd), m_hub(hub)
{
    s_instance = this;
    m_status = QVariantMap{{u"state"_s, u"Not connected"_s}};
    connect(m_link, &LinkManager::lteStatus, this, &LteController::onLteStatus, Qt::QueuedConnection);
    connect(m_conn, &ConnectionManager::connectFailed, this, [this](const QString &type, const QString &error) {
        if (type != QLatin1String(LinkManager::kMavlinkLte)) return;
        m_connecting = false;
        m_status[u"state"_s] = u"ERROR"_s;
        m_status[u"stateColor"_s] = u"#e44"_s;
        m_status[u"error"_s] = error;
        emit statusChanged();
        DialogHub::toast(u"Cellular link failed: "_s + error);
    });
    connect(m_link, &LinkManager::connectionState, this, [this](const QString &state) {
        if (state == "DISCONNECTED"_L1 && m_online) {
            m_online = false;
            emit statusChanged();
        }
    }, Qt::QueuedConnection);
    m_cellTimer.setInterval(1000);
    connect(&m_cellTimer, &QTimer::timeout, this, &LteController::renderCell);
    m_cellTimer.start();
    m_rowsTimer.setInterval(250);
    connect(&m_rowsTimer, &QTimer::timeout, this, [this] {
        updateCellMap();
        updateCellRows();
    });
    m_rowsTimer.start();
    // The cell database: <data root>/celldb, and the cells shipped next to the executable
    m_db = std::make_unique<CellDb>(DataRoot::celldbDir(), DataRoot::installDir() + u"/celldb"_s);
    m_dbView = QVariantMap{{u"status"_s, u"--"_s}};
    // The remembered key, read off the GUI thread (the keychain may be slow)
    if (keychain() && saved().value(u"rememberKey"_s).toBool()) {
        auto *w = new QFutureWatcher<QString>(this);
        connect(w, &QFutureWatcher<QString>::finished, this, [this, w] {
            m_savedKey = w->result();
            emit savedKeyChanged();
            w->deleteLater();
        });
        w->setFuture(QtConcurrent::run([] { return keychain::load(); }));
    }
}

LteController::~LteController() = default;

bool LteController::keychain() const { return keychain::available(); }
bool LteController::available() const { return lte::available(); }

QVariantMap LteController::saved() const
{
    QVariantMap m = loadJson(kConfigKey).toVariantMap();
    if (m.value(u"host"_s).toString().isEmpty()) m[u"host"_s] = kDefaultRelayHost;
    if (m.value(u"port"_s).toInt() <= 0) m[u"port"_s] = 5765;
    if (!keychain()) m[u"rememberKey"_s] = false;
    return m;
}

void LteController::saveConfig(const QVariantMap &cfg)
{
    Settings::instance().setJson(kConfigKey, QJsonObject::fromVariantMap(cfg));
    emit savedChanged();
}

void LteController::connectModule(const QVariantMap &f)
{
    const QString moduleId = f.value(u"moduleId"_s).toString().trimmed();
    QString keyHex = f.value(u"keyHex"_s).toString();
    keyHex.remove(QRegularExpression(u"[\\s:]"_s));
    const QString host = f.value(u"host"_s).toString().trimmed();
    const int port = f.value(u"port"_s).toInt() > 0 ? f.value(u"port"_s).toInt() : 5765;
    const QString cert = f.value(u"certSha256"_s).toString().trimmed();
    const bool remember = f.value(u"rememberKey"_s).toBool() && keychain();

    QString problem;
    if (moduleId.isEmpty() || moduleId.contains(QRegularExpression(u"\\s"_s))) problem = u"Enter the module ID"_s;
    else if (!QRegularExpression(u"^[0-9a-fA-F]{64}$"_s).match(keyHex).hasMatch())
        problem = u"The AES-256 key must be 64 hexadecimal characters"_s;
    else if (host.isEmpty()) problem = u"Enter the relay host (RELAY panel)"_s;
    if (!problem.isEmpty()) {
        DialogHub::toast(problem);
        return;
    }
    m_lastModuleSession.clear();   // a new link: its first session is not a restart
    saveConfig(QVariantMap{{u"moduleId"_s, moduleId}, {u"host"_s, host}, {u"port"_s, port},
                           {u"certSha256"_s, cert}, {u"rememberKey"_s, remember}});
    if (keychain()) {
        (void)QtConcurrent::run([remember, keyHex] {
            if (remember) keychain::save(keyHex);
            else keychain::forget();
        });
        m_savedKey = remember ? keyHex : QString();
    }
    m_connecting = true;
    m_status[u"state"_s] = u"Connecting to relay…"_s;
    m_status[u"stateColor"_s] = QString();
    emit statusChanged();
    m_conn->connectLink(QLatin1String(LinkManager::kMavlinkLte),
                        QVariantMap{{u"moduleId"_s, moduleId}, {u"keyHex"_s, keyHex}, {u"host"_s, host},
                                    {u"port"_s, port}, {u"certSha256"_s, cert}});
}

void LteController::disconnectModule()
{
    if (m_hub->snapshot()->connectionType == QLatin1String(LinkManager::kMavlinkLte)) m_conn->disconnectLink();
}

QString LteController::pinCertificate()
{
    if (m_lastFingerprint.isEmpty()) {
        DialogHub::toast(u"Connect once to read the relay certificate, then pin it."_s);
        return {};
    }
    QVariantMap cfg = loadJson(kConfigKey).toVariantMap();
    cfg[u"certSha256"_s] = m_lastFingerprint;
    saveConfig(cfg);
    return m_lastFingerprint;
}

void LteController::onLteStatus(const QVariantMap &st)
{
    static const QHash<QString, QString> labels{
        {u"idle"_s, u"Not connected"_s},       {u"connecting"_s, u"Connecting to relay…"_s},
        {u"handshake"_s, u"Authenticating…"_s}, {u"online"_s, u"ONLINE"_s},
        {u"reconnecting"_s, u"Link lost — reconnecting…"_s}, {u"error"_s, u"ERROR"_s},
        {u"stopped"_s, u"Not connected"_s}};
    const QString state = st.value(u"state"_s).toString();
    const bool online = state == "online"_L1;
    if (online || state == "error"_L1 || state == "stopped"_L1) m_connecting = false;
    m_online = online;
    const QString moduleId = st.value(u"moduleId"_s).toString();
    const qint64 recordsIn = st.value(u"recordsIn"_s).toLongLong();
    const QVariant age = st.value(u"lastRxAge"_s);
    const bool receiving = online && recordsIn > 0 && age.isValid() && !age.isNull() && age.toLongLong() < 3000;
    QVariantMap s;
    s[u"state"_s] = labels.value(state, state);
    s[u"stateColor"_s] = online ? u"#33cc33"_s
                                    : (state == "error"_L1 || state == "reconnecting"_L1) ? u"#e44"_s : QString();
    s[u"air"_s] = !online ? u"--"_s
                             : receiving ? moduleId + u" — receiving"_s
                             : st.value(u"airOnline"_s).toBool()
                                 ? moduleId + u" — online, waiting for data"_s
                                 : u"%1 — offline on the relay (key fingerprint %2: same as e2e_key_fp on the module?)"_s
                                       .arg(moduleId, st.value(u"keyFingerprint"_s).toString());
    const QString session = st.value(u"vehicleSession"_s).toString();
    s[u"session"_s] = session.isEmpty() ? u"--"_s : session;
    // A new module session means the module restarted, usually with the flight
    // controller: the autopilot is back on its default stream rates, ask again
    if (!session.isEmpty() && session != m_lastModuleSession) {
        if (!m_lastModuleSession.isEmpty()) QTimer::singleShot(1000, this, [this] { m_cmd->requestAllDataStreams(); });
        m_lastModuleSession = session;
    }
    const QVariantMap tls = st.value(u"tls"_s).toMap();
    if (!tls.isEmpty()) {
        m_lastFingerprint = tls.value(u"fingerprint"_s).toString();
        s[u"tls"_s] = u"%1 %2 — %3"_s
                                       .arg(tls.value(u"version"_s).toString(), tls.value(u"cipher"_s).toString(),
                                            tls.value(u"verified"_s).toBool() ? u"certificate pinned"_s
                                                                                          : u"certificate NOT verified"_s);
    } else {
        s[u"tls"_s] = m_status.value(u"tls"_s, u"--"_s);
    }
    s[u"records"_s] = u"%1 / %2"_s.arg(recordsIn).arg(st.value(u"recordsOut"_s).toLongLong());
    s[u"rejected"_s] = u"%1 not authentic, %2 replayed, %3 unsent (no module session yet)"_s
                                        .arg(st.value(u"authFail"_s).toLongLong())
                                        .arg(st.value(u"replay"_s).toLongLong())
                                        .arg(st.value(u"droppedNoSession"_s).toLongLong());
    s[u"traffic"_s] = u"↓ %1  ↑ %2"_s
                                       .arg(formatBytes(st.value(u"bytesRx"_s).toLongLong()),
                                            formatBytes(st.value(u"bytesTx"_s).toLongLong()));
    s[u"age"_s] = age.isValid() && !age.isNull() ? u"%1 s ago"_s.arg(age.toLongLong() / 1000.0, 0, 'f', 1)
                                                              : u"--"_s;
    s[u"reconnects"_s] = QString::number(st.value(u"reconnects"_s).toLongLong());
    const QString err = st.value(u"lastError"_s).toString();
    s[u"error"_s] = err.isEmpty() ? u"--"_s : err;
    m_status = s;
    emit statusChanged();
}

// ── Cells ───────────────────────────────────────────────────────────────────

QString LteController::frequencyText(unsigned earfcn, int band, const QString &fallback)
{
    // 3GPP TS 36.101 table 5.7.3-1, downlink: F_DL = F_DL_low + 0.1 (N_DL - N_Offs-DL)
    struct B {
        int band;
        double low;
        unsigned offs, max;
    };
    static const B bands[] = {
        {1, 2110, 0, 599}, {2, 1930, 600, 1199}, {3, 1805, 1200, 1949}, {4, 2110, 1950, 2399}, {5, 869, 2400, 2649},
        {6, 875, 2650, 2749}, {7, 2620, 2750, 3449}, {8, 925, 3450, 3799}, {9, 1844.9, 3800, 4149}, {10, 2110, 4150, 4749},
        {11, 1475.9, 4750, 4949}, {12, 729, 5010, 5179}, {13, 746, 5180, 5279}, {14, 758, 5280, 5379}, {17, 734, 5730, 5849},
        {18, 860, 5850, 5999}, {19, 875, 6000, 6149}, {20, 791, 6150, 6449}, {21, 1495.9, 6450, 6599}, {22, 3510, 6600, 7399},
        {23, 2180, 7500, 7699}, {24, 1525, 7700, 8039}, {25, 1930, 8040, 8689}, {26, 859, 8690, 9039}, {27, 852, 9040, 9209},
        {28, 758, 9210, 9659}, {29, 717, 9660, 9769}, {30, 2350, 9770, 9869}, {31, 462.5, 9870, 9919}, {32, 1452, 9920, 10359},
        {33, 1900, 36000, 36199}, {34, 2010, 36200, 36349}, {35, 1850, 36350, 36949}, {36, 1930, 36950, 37549},
        {37, 1910, 37550, 37749}, {38, 2570, 37750, 38249}, {39, 1880, 38250, 38649}, {40, 2300, 38650, 39649},
        {41, 2496, 39650, 41589}, {42, 3400, 41590, 43589}, {43, 3600, 43590, 45589}, {44, 703, 45590, 46589},
        {45, 1447, 46590, 46789}, {46, 5150, 46790, 54539}, {47, 5855, 54540, 55239}, {48, 3550, 55240, 56739},
        {49, 3550, 56740, 58239}, {50, 1432, 58240, 59089}, {51, 1427, 59090, 59139}, {52, 3300, 59140, 60139},
        {53, 2483.5, 60140, 60254}, {65, 2110, 65536, 66435}, {66, 2110, 66436, 67335}, {67, 738, 67336, 67535},
        {68, 753, 67536, 67835}, {69, 2570, 67836, 68335}, {70, 1995, 68336, 68585}, {71, 617, 68586, 68935},
        {72, 461, 68936, 68985}, {73, 460, 68986, 69035}, {74, 1475, 69036, 69465}, {75, 1432, 69466, 70315},
        {76, 1427, 70316, 70365}, {85, 728, 70366, 70545}, {87, 420, 70546, 70595}, {88, 422, 70596, 70645}};
    // EARFCN 0 is a real band 1 channel, but a modem that knows nothing sends 0 and band 0
    if (earfcn || band == 1)
        for (const B &b : bands)
            if (earfcn >= b.offs && earfcn <= b.max)
                return QString::number(std::round((b.low + 0.1 * (earfcn - b.offs)) * 10) / 10, 'f', 1) + u" MHz"_s;
    return band ? u"B%1"_s.arg(band) : fallback;
}

QString LteController::operatorName(int mcc, int mnc)
{
    const QString plmn = u"%1-%2"_s.arg(mcc).arg(mnc, 2, 10, '0'_L1);
    return mcc == 222 && operators222().contains(mnc) ? u"%1 (%2)"_s.arg(operators222().value(mnc), plmn) : plmn;
}

QString LteController::operatorShort(int mcc, int mnc)
{
    if (mcc == 222 && operators222().contains(mnc)) return operators222().value(mnc);
    return u"%1-%2"_s.arg(mcc).arg(mnc, 2, 10, '0'_L1);
}

QVariantMap LteController::cellMap() const
{
    const QJsonObject o = loadJson(kCellMapKey);
    return QVariantMap{{u"show"_s, o.value(u"show"_s).toBool(true)},
                       {u"allOperators"_s, o.value(u"allOperators"_s).toBool(false)},
                       {u"table"_s, o.value(u"table"_s).toBool(true)}};
}

void LteController::setCellMap(const QString &name, bool on)
{
    QJsonObject o = QJsonObject::fromVariantMap(cellMap());
    o.insert(name, on);
    Settings::instance().setJson(kCellMapKey, o);
    emit cellMapChanged();
    m_queryKey.clear();   // query again with the new filter
    updateCellMap();
    updateCellRows();
}

// ── Cell sites (CellTowerLayer.js) ───────────────────────────────────────────

QColor LteController::linkColor() const
{
    // linkQuality(): good with RSRP from -100 dBm and an average SINR from 5 dB, poor
    // under -110 dBm or 0 dB (or no report for 5 s), fair in between
    const auto s = m_hub->snapshot();
    const CellInfo &c = s->cell;
    if (!c.valid || !c.eci || wallMs() - c.receivedAt > kStaleLinkMs || (c.rsrp && *c.rsrp < -110) || (c.sinrAvg && *c.sinrAvg < 0))
        return QColor(0xff3030);
    if (c.rsrp && *c.rsrp >= -100 && c.sinrAvg && *c.sinrAvg >= 5) return QColor(0x3dff6e);
    return QColor(0xffaa00);
}

void LteController::hideOverlay()
{
    if (!m_shown) return;
    m_shown = false;
    m_serving = -1;
    m_servingEci = 0;
    m_filterVersion = -1;   // drawn again when the overlay comes back
    m_servingKm.reset();
    cellOverlay::set(nullptr);
}

void LteController::publishOverlay()
{
    auto o = std::make_shared<CellOverlay>();
    o->live = m_shown;
    o->sites = m_drawn;
    o->serving = m_drawnServing;
    const auto s = m_hub->snapshot();
    o->ownMcc = s->cell.mcc;
    o->ownMnc = s->cell.mnc;
    o->linkColor = m_linkColor;
    o->version = ++m_overlayVersion;
    cellOverlay::set(std::move(o));
}

void LteController::updateCellMap()
{
    const auto s = m_hub->snapshot();
    const CellInfo &c = s->cell;
    const qint64 now = wallMs();
    const bool live = c.valid && now - c.receivedAt < kCellStaleMs && cellMap().value(u"show"_s).toBool() && !s->relativeMode;
    if (!live) {
        hideOverlay();
        return;
    }
    const bool appeared = !m_shown;
    m_shown = true;
    const bool valid = s->lat != 0 || s->lon != 0;
    const LatLon vehicle{s->lat, s->lon};
    const bool all = cellMap().value(u"allOperators"_s).toBool();
    const QString key = u"%1-%2-%3"_s.arg(c.mcc).arg(c.mnc).arg(all);
    if (valid && c.mcc && !m_querying && now >= m_queryRetryAt &&
        (key != m_queryKey || !m_queryCentre || kmBetween(*m_queryCentre, vehicle.lat, vehicle.lon) > kRequeryKm))
        queryCells(vehicle);
    if (c.eci != m_servingEci) findServing();
    bool changed = appeared;
    if (m_sitesVersion != m_filterVersion || m_serving != m_filterServing || (valid && !m_filterCentre) ||
        (valid && m_filterCentre && kmBetween(*m_filterCentre, vehicle.lat, vehicle.lon) > kRefilterKm)) {
        pickSites(valid ? std::optional<LatLon>(vehicle) : std::nullopt);
        changed = true;
    }
    const QColor colour = linkColor();
    if (colour != m_linkColor) {
        m_linkColor = colour;
        changed = true;
    }
    m_servingKm.reset();
    if (m_serving >= 0 && valid) m_servingKm = kmBetween(vehicle, m_sites[std::size_t(m_serving)].lat, m_sites[std::size_t(m_serving)].lon);
    if (changed) publishOverlay();
}

void LteController::queryCells(const LatLon &centre)
{
    const auto s = m_hub->snapshot();
    const CellInfo &c = s->cell;
    const bool all = cellMap().value(u"allOperators"_s).toBool();
    m_querying = true;
    m_queryCentre = centre;
    m_queryKey = u"%1-%2-%3"_s.arg(c.mcc).arg(c.mnc).arg(all);
    CellDb::Query q;
    q.lat = centre.lat;
    q.lon = centre.lon;
    q.radiusKm = kRadiusKm;
    if (!all) {
        q.mcc = c.mcc;
        q.mnc = c.mnc;
    }
    if (c.eci) q.eci = c.eci;
    m_db->query(q, [this](CellDb::Result r) {
        m_querying = false;
        if (!r.error.isEmpty()) {
            m_queryError = r.error;
            m_queryCentre.reset();               // try again...
            m_queryRetryAt = wallMs() + 10000;   // ...in 10 s
            return;
        }
        m_queryError.clear();
        m_sites = std::move(r.sites);
        ++m_sitesVersion;
        m_dbDate = r.date;
        m_dbMissing = r.missing;
        findServing();
    });
}

void LteController::findServing()
{
    const auto s = m_hub->snapshot();
    const CellInfo &c = s->cell;
    m_servingEci = c.valid ? c.eci : 0;
    m_serving = -1;
    if (!c.valid || !c.eci) return;
    // The eNB is found even when this sector is missing from the database; a shared
    // cell is in the database under the partner's network code
    for (std::size_t i = 0; i < m_sites.size() && m_serving < 0; ++i)
        if (m_sites[i].mcc == c.mcc && m_sites[i].mnc == c.mnc && m_sites[i].enb == c.enb) m_serving = int(i);
    for (std::size_t i = 0; i < m_sites.size() && m_serving < 0; ++i) {
        if (m_sites[i].mcc != c.mcc) continue;
        for (const CellRecord &x : m_sites[i].cells)
            if (x.eci == c.eci) {
                m_serving = int(i);
                break;
            }
    }
    if (m_serving >= 0 || m_querying || !m_queryCentre) return;
    if (m_requeriedEci != c.eci) {
        m_requeriedEci = c.eci;
        m_queryCentre.reset();   // the next update asks again with this ECI
    } else {
        lookupServing();
    }
}

void LteController::lookupServing()
{
    const auto s = m_hub->snapshot();
    const CellInfo c = s->cell;
    if (m_lookupEci == c.eci) return;
    m_lookupEci = c.eci;
    m_lookingUp = true;
    m_db->lookup(c.mcc, c.mnc, c.tac, c.eci, [this, c](std::optional<CellRecord> hit, const QString &error) {
        m_lookingUp = false;
        if (!error.isEmpty()) {
            m_lookupEci = 0;   // offline: try again at the next cell change
            LOG_W("cell") << "lookup:" << error;
            return;
        }
        const auto now = m_hub->snapshot();
        if (!hit || !now->cell.valid || now->cell.eci != c.eci) return;
        CellSite site;
        site.mcc = c.mcc;
        site.mnc = c.mnc;
        site.enb = c.enb;
        site.lat = hit->lat;
        site.lon = hit->lon;
        CellRecord cell = *hit;
        cell.eci = c.eci;
        cell.tac = c.tac;
        site.cells.push_back(cell);
        m_sites.push_back(std::move(site));
        ++m_sitesVersion;
        findServing();
    });
}

void LteController::pickSites(const std::optional<LatLon> &vehicle)
{
    m_filterCentre = vehicle;
    m_filterServing = m_serving;
    m_filterVersion = m_sitesVersion;
    m_drawn.clear();
    m_drawnServing = -1;
    for (std::size_t i = 0; i < m_sites.size(); ++i) {
        const bool serving = int(i) == m_serving;
        if (!serving && (!vehicle || kmBetween(*vehicle, m_sites[i].lat, m_sites[i].lon) > kShowKm)) continue;
        if (serving) m_drawnServing = int(m_drawn.size());
        m_drawn.push_back(m_sites[i]);
    }
}

// ── Cell database panel ──────────────────────────────────────────────────────

QStringList LteController::countries() const
{
    QStringList out;
    for (const Country &c : kCountries) out << QLatin1String(c.name);
    return out;
}

void LteController::renderDbStatus(const QVariantMap &st)
{
    QVariantMap v = m_dbView;
    const QVariantMap b = st.value(u"bundle"_s).toMap();
    if (!b.isEmpty()) {
        QStringList mcc, srcs;
        for (const QVariant &m : b.value(u"mcc"_s).toList()) mcc << m.toString();
        for (const QVariant &x : b.value(u"sources"_s).toList()) {
            const QVariantMap sm = x.toMap();
            srcs << u"%1 %2"_s.arg(sourceName(sm.value(u"src"_s).toString()), sm.value(u"date"_s).toString());
        }
        v[u"bundle"_s] = u"%1 LTE cells (MCC %2): %3"_s
                                          .arg(QLocale().toString(b.value(u"cells"_s).toLongLong()), mcc.join(u", "_s),
                                               srcs.join(u", "_s));
    } else {
        v[u"bundle"_s] = u"none"_s;
    }
    QStringList files;
    for (const QVariant &x : st.value(u"files"_s).toList()) {
        const QVariantMap f = x.toMap();
        const QString newest = f.value(u"newest"_s).toString();
        files << u"%1: %2 cells%3"_s
                     .arg(f.value(u"name"_s).toString(), QLocale().toString(f.value(u"cells"_s).toLongLong()),
                          newest.isEmpty() ? QString() : u" (to %1)"_s.arg(newest));
    }
    v[u"files"_s] = files.isEmpty() ? u"none"_s : files.join(u", "_s);
    const int blocks = st.value(u"blocks"_s).toInt();
    const QString date = st.value(u"date"_s).toString();
    v[u"status"_s] = blocks ? u"%1 LTE cells in %2 areas, %3%4"_s
                                               .arg(QLocale().toString(st.value(u"cells"_s).toLongLong()))
                                               .arg(blocks)
                                               .arg(formatBytes(st.value(u"bytes"_s).toLongLong()),
                                                    date.isEmpty() ? QString() : u", data of %1"_s.arg(date))
                                         : u"none"_s;
    const QVariantMap lk = st.value(u"lookups"_s).toMap();
    v[u"lookups"_s] =
        u"%1 found, %2 not found"_s.arg(lk.value(u"found"_s).toInt()).arg(lk.value(u"missed"_s).toInt());
    const QVariantMap set = st.value(u"settings"_s).toMap();
    v[u"online"_s] = set.value(u"online"_s, true).toBool();
    v[u"hasToken"_s] = set.value(u"hasToken"_s).toBool();
    v[u"busy"_s] = m_dbBusy;
    m_dbView = v;
    emit dbChanged();
}

void LteController::dbRefresh()
{
    m_db->status([this](const QVariantMap &st) { renderDbStatus(st); });
}

void LteController::dbSetOnline(bool on)
{
    m_db->setOnline(on);
    dbRefresh();
}

void LteController::dbSaveToken(const QString &token)
{
    m_db->setToken(token);
    m_lookupEci = 0;   // a new token may find the serving cell
    dbRefresh();
}

void LteController::dbDownloadOpenCellid(const QString &name)
{
    const Country *c = country(name);
    if (!c || m_dbBusy) return;
    m_dbBusy = true;
    m_dbView[u"busy"_s] = true;
    m_dbView[u"files"_s] = u"downloading the OpenCelliD file of MCC %1..."_s.arg(c->mcc);
    emit dbChanged();
    m_db->downloadOpenCellid(c->mcc, [this](const QString &error, int cells) {
        m_dbBusy = false;
        if (!error.isEmpty()) DialogHub::toast(u"OpenCelliD download failed: "_s + error);
        else LOG_I("celldb") << "OpenCelliD file:" << cells << "LTE cells";
        m_queryKey.clear();   // the map queries again
        dbRefresh();
    });
}

void LteController::dbDownloadCopy(const QString &name)
{
    const Country *c = country(name);
    if (!c || m_dbBusy) return;
    m_dbBusy = true;
    m_dbView[u"busy"_s] = true;
    emit dbChanged();
    m_db->downloadBox(
        c->box[0], c->box[1], c->box[2], c->box[3],
        [this](int done, int total, int cells, int failed) {
            m_dbView[u"status"_s] = u"downloading: area %1 of %2, %3 cells%4"_s
                                                     .arg(done)
                                                     .arg(total)
                                                     .arg(QLocale().toString(cells))
                                                     .arg(failed ? u", %1 failed"_s.arg(failed) : QString());
            emit dbChanged();
        },
        [this](int blocks, int, int failed) {
            m_dbBusy = false;
            if (failed) DialogHub::toast(u"%1 of %2 areas could not be downloaded: try again later."_s.arg(failed).arg(blocks));
            m_queryKey.clear();
            dbRefresh();
        });
}

void LteController::dbImport(const QUrl &file)
{
    const QString path = file.isLocalFile() ? file.toLocalFile() : file.toString();
    if (path.isEmpty() || m_dbBusy) return;
    m_dbBusy = true;
    m_dbView[u"busy"_s] = true;
    emit dbChanged();
    m_db->importCsv(path, [this](const QString &error, int cells) {
        m_dbBusy = false;
        if (!error.isEmpty()) DialogHub::toast(u"Import failed: "_s + error);
        else LOG_I("celldb") << "imported:" << cells << "LTE cells";
        m_queryKey.clear();
        dbRefresh();
    });
}

void LteController::dbClear()
{
    DialogHub::confirm(u"Delete the downloaded, imported and looked-up cells? The built-in ones stay."_s, [this](bool ok) {
        if (!ok) return;
        m_db->clear([this] {
            m_queryKey.clear();
            m_lookupEci = 0;
            dbRefresh();
        });
    });
}


void LteController::renderCell()
{
    const auto s = m_hub->snapshot();
    const CellInfo &c = s->cell;
    const bool live = c.valid && wallMs() - c.receivedAt < kCellStaleMs;
    QVariantMap m;
    if (!live) {
        for (const char *k : {"serving", "signal", "neighbours", "changes", "mast"}) m[QLatin1String(k)] = u"--"_s;
        if (c.valid) m[u"serving"_s] = u"no CORV_CELL_INFO for 10 s"_s;
    } else {
        m[u"serving"_s] = c.eci ? u"%1 eNB %2 sector %3 (ECI %4), TAC %5, PCI %6, EARFCN %7 (band %8, %9 downlink), %10"_s
                                                   .arg(operatorName(c.mcc, c.mnc))
                                                   .arg(c.enb)
                                                   .arg(c.sector)
                                                   .arg(c.eci)
                                                   .arg(c.tac)
                                                   .arg(c.pci)
                                                   .arg(c.earfcn)
                                                   .arg(c.band)
                                                   .arg(frequencyText(c.earfcn, c.band, u"?"_s), c.state)
                                             : u"no cell (%1)"_s.arg(c.state);
        m[u"signal"_s] = u"RSRP %1, RSRQ %2, SINR %3, RSSI %4"_s
                                          .arg(v(c.rsrp, u"dBm"_s), v(c.rsrq, u"dB"_s), v(c.sinr, u"dB"_s),
                                               v(c.rssi, u"dBm"_s));
        if (c.neighbours) {
            QStringList nb;
            for (const CellNeighbour &n : c.nb)
                nb << u"PCI %1%2 %3"_s.arg(n.pci).arg(n.earfcn != c.earfcn ? u"/%1"_s.arg(n.earfcn) : QString(),
                                                                   v(n.rsrp, u"dBm"_s));
            m[u"neighbours"_s] = u"%1: %2"_s.arg(c.neighbours).arg(nb.join(u", "_s));
        } else {
            m[u"neighbours"_s] = u"0"_s;
        }
        m[u"changes"_s] = QString::number(c.handovers) +
                                       (c.changedAt ? u", last %1 s ago"_s.arg(std::llround((wallMs() - c.changedAt) / 1000.0)) : QString());
        QString mast = cellMap().value(u"show"_s).toBool() ? u"looking for the site..."_s
                                                                         : u"shown with SHOW CELL SITES ON THE MAP"_s;
        if (!m_queryError.isEmpty()) {
            mast = u"database: "_s + m_queryError;
        } else if (m_serving >= 0) {
            const CellSite &site = m_sites[std::size_t(m_serving)];
            QStringList srcs;
            for (const CellRecord &x : site.cells)
                if (!srcs.contains(sourceName(x.src))) srcs << sourceName(x.src);
            mast = u"%1, %2 (estimate from %3 cell%4 of the eNB, %5)"_s
                       .arg(QString::number(site.lat, 'f', 5), QString::number(site.lon, 'f', 5))
                       .arg(site.cells.size())
                       .arg(site.cells.size() > 1 ? u"s"_s : QString(), srcs.join(u", "_s));
            if (site.mnc != c.mnc) mast += u"; shared cell, listed under "_s + operatorName(site.mcc, site.mnc);
        } else if (m_lookingUp) {
            mast = u"not in the database: looking it up online"_s;
        } else if (!m_querying && m_queryCentre && c.eci) {
            mast = u"not in the database nor online (site never measured, or newer than the data)"_s;
        }
        m[u"mast"_s] = mast;
    }
    if (m == m_cell) return;
    m_cell = m;
    emit cellChanged();
}

/// LTE cell rows of the position panel: on the cellular link while the module
/// reports and LTE OVERLAY is on; text rewritten at most 4 times a second.
void LteController::updateCellRows()
{
    const auto s = m_hub->snapshot();
    const CellInfo &c = s->cell;
    const qint64 now = wallMs();
    const bool live = s->connectionType == QLatin1String(LinkManager::kMavlinkLte) && c.valid && now - c.receivedAt < kCellStaleMs &&
                      cellMap().value(u"table"_s).toBool();
    QVariantMap r{{u"live"_s, live}};
    if (live) {
        r[u"changed"_s] = c.changedAt && now - c.changedAt < 5000;
        r[u"cell"_s] = c.eci ? u"%1 %2/%3"_s.arg(operatorShort(c.mcc, c.mnc)).arg(c.enb).arg(c.sector)
                                          : u"NO CELL (%1)"_s.arg(c.state);
        r[u"band"_s] = c.eci ? u"%1 · %2"_s.arg(frequencyText(c.earfcn, c.band, u"-"_s)).arg(c.pci)
                                          : u"---"_s;
        r[u"rssi"_s] = v(c.rssi);
        r[u"sig"_s] = u"%1 · %2"_s.arg(v(c.rsrp), v(c.sinr));
        r[u"nb"_s] = QString::number(c.neighbours);
        r[u"ho"_s] = QString::number(c.handovers);
        const bool notInDb = c.eci && m_serving < 0 && !m_querying && !m_lookingUp && m_queryCentre;
        r[u"dist"_s] = m_servingKm ? QString::number(*m_servingKm, 'f', 1) : notInDb ? u"NOT IN DB"_s : u"---"_s;
        r[u"distUnit"_s] = m_servingKm ? u"km"_s : QString();
    }
    if (r == m_cellRows) return;
    m_cellRows = r;
    emit cellRowsChanged();
}

} // namespace corv
