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

namespace corv {

namespace {

const QString kConfigKey = QStringLiteral("lte-link-config");
const QString kCellMapKey = QStringLiteral("lte-cell-map");
const QString kDefaultRelayHost = QStringLiteral("94.177.202.247");   // CORV relay
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
    static const QHash<QString, QString> names{{QStringLiteral("ocid"), QStringLiteral("OpenCelliD")},
                                              {QStringLiteral("ocid-2024"), QStringLiteral("OpenCelliD 2024")},
                                              {QStringLiteral("mls"), QStringLiteral("Mozilla MLS 2024")},
                                              {QStringLiteral("beacondb"), QStringLiteral("beaconDB (online)")},
                                              {QStringLiteral("ocid-api"), QStringLiteral("OpenCelliD API (online)")},
                                              {QStringLiteral("bundle"), QStringLiteral("built-in")}};
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
    if (n >= 1000000) return QString::number(n / 1e6, 'f', 1) + QStringLiteral(" MB");
    if (n >= 1000) return QString::number(n / 1e3, 'f', 1) + QStringLiteral(" kB");
    return QString::number(n) + QStringLiteral(" B");
}

// ── OS keychain for REMEMBER KEY (Electron safeStorage) ─────────────────────

namespace keychain {

#if defined(Q_OS_WIN)

QString file() { return QDir(DataRoot::root()).filePath(QStringLiteral("lte-link-key-qt.bin")); }
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
    static const QHash<int, QString> o{{1, QStringLiteral("TIM")}, {10, QStringLiteral("Vodafone")}, {50, QStringLiteral("Iliad")},
                                       {88, QStringLiteral("WindTre")}, {99, QStringLiteral("WindTre (3)")}, {43, QStringLiteral("TIM")},
                                       {48, QStringLiteral("TIM")}};
    return o;
}

QString v(const std::optional<int> &x, const QString &unit = QString())
{
    if (!x) return QStringLiteral("-");
    return unit.isEmpty() ? QString::number(*x) : QStringLiteral("%1 %2").arg(*x).arg(unit);
}

} // namespace

LteController *LteController::s_instance = nullptr;

LteController::LteController(LinkManager *link, ConnectionManager *conn, CommandSender *cmd, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_link(link), m_conn(conn), m_cmd(cmd), m_hub(hub)
{
    s_instance = this;
    m_status = QVariantMap{{QStringLiteral("state"), QStringLiteral("Not connected")}};
    connect(m_link, &LinkManager::lteStatus, this, &LteController::onLteStatus, Qt::QueuedConnection);
    connect(m_conn, &ConnectionManager::connectFailed, this, [this](const QString &type, const QString &error) {
        if (type != QLatin1String(LinkManager::kMavlinkLte)) return;
        m_connecting = false;
        m_status[QStringLiteral("state")] = QStringLiteral("ERROR");
        m_status[QStringLiteral("stateColor")] = QStringLiteral("#e44");
        m_status[QStringLiteral("error")] = error;
        emit statusChanged();
        DialogHub::toast(QStringLiteral("Cellular link failed: ") + error);
    });
    connect(m_link, &LinkManager::connectionState, this, [this](const QString &state) {
        if (state == QLatin1String("DISCONNECTED") && m_online) {
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
    m_db = std::make_unique<CellDb>(DataRoot::celldbDir(), DataRoot::installDir() + QStringLiteral("/celldb"));
    m_dbView = QVariantMap{{QStringLiteral("status"), QStringLiteral("--")}};
    // The remembered key, read off the GUI thread (the keychain may be slow)
    if (keychain() && saved().value(QStringLiteral("rememberKey")).toBool()) {
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
    if (m.value(QStringLiteral("host")).toString().isEmpty()) m[QStringLiteral("host")] = kDefaultRelayHost;
    if (m.value(QStringLiteral("port")).toInt() <= 0) m[QStringLiteral("port")] = 5765;
    if (!keychain()) m[QStringLiteral("rememberKey")] = false;
    return m;
}

void LteController::saveConfig(const QVariantMap &cfg)
{
    Settings::instance().setJson(kConfigKey, QJsonObject::fromVariantMap(cfg));
    emit savedChanged();
}

void LteController::connectModule(const QVariantMap &f)
{
    const QString moduleId = f.value(QStringLiteral("moduleId")).toString().trimmed();
    QString keyHex = f.value(QStringLiteral("keyHex")).toString();
    keyHex.remove(QRegularExpression(QStringLiteral("[\\s:]")));
    const QString host = f.value(QStringLiteral("host")).toString().trimmed();
    const int port = f.value(QStringLiteral("port")).toInt() > 0 ? f.value(QStringLiteral("port")).toInt() : 5765;
    const QString cert = f.value(QStringLiteral("certSha256")).toString().trimmed();
    const bool remember = f.value(QStringLiteral("rememberKey")).toBool() && keychain();

    QString problem;
    if (moduleId.isEmpty() || moduleId.contains(QRegularExpression(QStringLiteral("\\s")))) problem = QStringLiteral("Enter the module ID");
    else if (!QRegularExpression(QStringLiteral("^[0-9a-fA-F]{64}$")).match(keyHex).hasMatch())
        problem = QStringLiteral("The AES-256 key must be 64 hexadecimal characters");
    else if (host.isEmpty()) problem = QStringLiteral("Enter the relay host (RELAY panel)");
    if (!problem.isEmpty()) {
        DialogHub::toast(problem);
        return;
    }
    m_lastModuleSession.clear();   // a new link: its first session is not a restart
    saveConfig(QVariantMap{{QStringLiteral("moduleId"), moduleId}, {QStringLiteral("host"), host}, {QStringLiteral("port"), port},
                           {QStringLiteral("certSha256"), cert}, {QStringLiteral("rememberKey"), remember}});
    if (keychain()) {
        (void)QtConcurrent::run([remember, keyHex] {
            if (remember) keychain::save(keyHex);
            else keychain::forget();
        });
        m_savedKey = remember ? keyHex : QString();
    }
    m_connecting = true;
    m_status[QStringLiteral("state")] = QStringLiteral("Connecting to relay…");
    m_status[QStringLiteral("stateColor")] = QString();
    emit statusChanged();
    m_conn->connectLink(QLatin1String(LinkManager::kMavlinkLte),
                        QVariantMap{{QStringLiteral("moduleId"), moduleId}, {QStringLiteral("keyHex"), keyHex}, {QStringLiteral("host"), host},
                                    {QStringLiteral("port"), port}, {QStringLiteral("certSha256"), cert}});
}

void LteController::disconnectModule()
{
    if (m_hub->snapshot()->connectionType == QLatin1String(LinkManager::kMavlinkLte)) m_conn->disconnectLink();
}

QString LteController::pinCertificate()
{
    if (m_lastFingerprint.isEmpty()) {
        DialogHub::toast(QStringLiteral("Connect once to read the relay certificate, then pin it."));
        return {};
    }
    QVariantMap cfg = loadJson(kConfigKey).toVariantMap();
    cfg[QStringLiteral("certSha256")] = m_lastFingerprint;
    saveConfig(cfg);
    return m_lastFingerprint;
}

void LteController::onLteStatus(const QVariantMap &st)
{
    static const QHash<QString, QString> labels{
        {QStringLiteral("idle"), QStringLiteral("Not connected")},       {QStringLiteral("connecting"), QStringLiteral("Connecting to relay…")},
        {QStringLiteral("handshake"), QStringLiteral("Authenticating…")}, {QStringLiteral("online"), QStringLiteral("ONLINE")},
        {QStringLiteral("reconnecting"), QStringLiteral("Link lost — reconnecting…")}, {QStringLiteral("error"), QStringLiteral("ERROR")},
        {QStringLiteral("stopped"), QStringLiteral("Not connected")}};
    const QString state = st.value(QStringLiteral("state")).toString();
    const bool online = state == QLatin1String("online");
    if (online || state == QLatin1String("error") || state == QLatin1String("stopped")) m_connecting = false;
    m_online = online;
    const QString moduleId = st.value(QStringLiteral("moduleId")).toString();
    const qint64 recordsIn = st.value(QStringLiteral("recordsIn")).toLongLong();
    const QVariant age = st.value(QStringLiteral("lastRxAge"));
    const bool receiving = online && recordsIn > 0 && age.isValid() && !age.isNull() && age.toLongLong() < 3000;
    QVariantMap s;
    s[QStringLiteral("state")] = labels.value(state, state);
    s[QStringLiteral("stateColor")] = online ? QStringLiteral("#33cc33")
                                    : (state == QLatin1String("error") || state == QLatin1String("reconnecting")) ? QStringLiteral("#e44") : QString();
    s[QStringLiteral("air")] = !online ? QStringLiteral("--")
                             : receiving ? moduleId + QStringLiteral(" — receiving")
                             : st.value(QStringLiteral("airOnline")).toBool()
                                 ? moduleId + QStringLiteral(" — online, waiting for data")
                                 : QStringLiteral("%1 — offline on the relay (key fingerprint %2: same as e2e_key_fp on the module?)")
                                       .arg(moduleId, st.value(QStringLiteral("keyFingerprint")).toString());
    const QString session = st.value(QStringLiteral("vehicleSession")).toString();
    s[QStringLiteral("session")] = session.isEmpty() ? QStringLiteral("--") : session;
    // A new module session means the module restarted, usually with the flight
    // controller: the autopilot is back on its default stream rates, ask again
    if (!session.isEmpty() && session != m_lastModuleSession) {
        if (!m_lastModuleSession.isEmpty()) QTimer::singleShot(1000, this, [this] { m_cmd->requestAllDataStreams(); });
        m_lastModuleSession = session;
    }
    const QVariantMap tls = st.value(QStringLiteral("tls")).toMap();
    if (!tls.isEmpty()) {
        m_lastFingerprint = tls.value(QStringLiteral("fingerprint")).toString();
        s[QStringLiteral("tls")] = QStringLiteral("%1 %2 — %3")
                                       .arg(tls.value(QStringLiteral("version")).toString(), tls.value(QStringLiteral("cipher")).toString(),
                                            tls.value(QStringLiteral("verified")).toBool() ? QStringLiteral("certificate pinned")
                                                                                          : QStringLiteral("certificate NOT verified"));
    } else {
        s[QStringLiteral("tls")] = m_status.value(QStringLiteral("tls"), QStringLiteral("--"));
    }
    s[QStringLiteral("records")] = QStringLiteral("%1 / %2").arg(recordsIn).arg(st.value(QStringLiteral("recordsOut")).toLongLong());
    s[QStringLiteral("rejected")] = QStringLiteral("%1 not authentic, %2 replayed, %3 unsent (no module session yet)")
                                        .arg(st.value(QStringLiteral("authFail")).toLongLong())
                                        .arg(st.value(QStringLiteral("replay")).toLongLong())
                                        .arg(st.value(QStringLiteral("droppedNoSession")).toLongLong());
    s[QStringLiteral("traffic")] = QStringLiteral("↓ %1  ↑ %2")
                                       .arg(formatBytes(st.value(QStringLiteral("bytesRx")).toLongLong()),
                                            formatBytes(st.value(QStringLiteral("bytesTx")).toLongLong()));
    s[QStringLiteral("age")] = age.isValid() && !age.isNull() ? QStringLiteral("%1 s ago").arg(age.toLongLong() / 1000.0, 0, 'f', 1)
                                                              : QStringLiteral("--");
    s[QStringLiteral("reconnects")] = QString::number(st.value(QStringLiteral("reconnects")).toLongLong());
    const QString err = st.value(QStringLiteral("lastError")).toString();
    s[QStringLiteral("error")] = err.isEmpty() ? QStringLiteral("--") : err;
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
                return QString::number(std::round((b.low + 0.1 * (earfcn - b.offs)) * 10) / 10, 'f', 1) + QStringLiteral(" MHz");
    return band ? QStringLiteral("B%1").arg(band) : fallback;
}

QString LteController::operatorName(int mcc, int mnc)
{
    const QString plmn = QStringLiteral("%1-%2").arg(mcc).arg(mnc, 2, 10, QLatin1Char('0'));
    return mcc == 222 && operators222().contains(mnc) ? QStringLiteral("%1 (%2)").arg(operators222().value(mnc), plmn) : plmn;
}

QString LteController::operatorShort(int mcc, int mnc)
{
    if (mcc == 222 && operators222().contains(mnc)) return operators222().value(mnc);
    return QStringLiteral("%1-%2").arg(mcc).arg(mnc, 2, 10, QLatin1Char('0'));
}

QVariantMap LteController::cellMap() const
{
    const QJsonObject o = loadJson(kCellMapKey);
    return QVariantMap{{QStringLiteral("show"), o.value(QStringLiteral("show")).toBool(true)},
                       {QStringLiteral("allOperators"), o.value(QStringLiteral("allOperators")).toBool(false)},
                       {QStringLiteral("table"), o.value(QStringLiteral("table")).toBool(true)}};
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
    const bool live = c.valid && now - c.receivedAt < kCellStaleMs && cellMap().value(QStringLiteral("show")).toBool() && !s->relativeMode;
    if (!live) {
        hideOverlay();
        return;
    }
    const bool appeared = !m_shown;
    m_shown = true;
    const bool valid = s->lat != 0 || s->lon != 0;
    const LatLon vehicle{s->lat, s->lon};
    const bool all = cellMap().value(QStringLiteral("allOperators")).toBool();
    const QString key = QStringLiteral("%1-%2-%3").arg(c.mcc).arg(c.mnc).arg(all);
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
    const bool all = cellMap().value(QStringLiteral("allOperators")).toBool();
    m_querying = true;
    m_queryCentre = centre;
    m_queryKey = QStringLiteral("%1-%2-%3").arg(c.mcc).arg(c.mnc).arg(all);
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
    const QVariantMap b = st.value(QStringLiteral("bundle")).toMap();
    if (!b.isEmpty()) {
        QStringList mcc, srcs;
        for (const QVariant &m : b.value(QStringLiteral("mcc")).toList()) mcc << m.toString();
        for (const QVariant &x : b.value(QStringLiteral("sources")).toList()) {
            const QVariantMap sm = x.toMap();
            srcs << QStringLiteral("%1 %2").arg(sourceName(sm.value(QStringLiteral("src")).toString()), sm.value(QStringLiteral("date")).toString());
        }
        v[QStringLiteral("bundle")] = QStringLiteral("%1 LTE cells (MCC %2): %3")
                                          .arg(QLocale().toString(b.value(QStringLiteral("cells")).toLongLong()), mcc.join(QStringLiteral(", ")),
                                               srcs.join(QStringLiteral(", ")));
    } else {
        v[QStringLiteral("bundle")] = QStringLiteral("none");
    }
    QStringList files;
    for (const QVariant &x : st.value(QStringLiteral("files")).toList()) {
        const QVariantMap f = x.toMap();
        const QString newest = f.value(QStringLiteral("newest")).toString();
        files << QStringLiteral("%1: %2 cells%3")
                     .arg(f.value(QStringLiteral("name")).toString(), QLocale().toString(f.value(QStringLiteral("cells")).toLongLong()),
                          newest.isEmpty() ? QString() : QStringLiteral(" (to %1)").arg(newest));
    }
    v[QStringLiteral("files")] = files.isEmpty() ? QStringLiteral("none") : files.join(QStringLiteral(", "));
    const int blocks = st.value(QStringLiteral("blocks")).toInt();
    const QString date = st.value(QStringLiteral("date")).toString();
    v[QStringLiteral("status")] = blocks ? QStringLiteral("%1 LTE cells in %2 areas, %3%4")
                                               .arg(QLocale().toString(st.value(QStringLiteral("cells")).toLongLong()))
                                               .arg(blocks)
                                               .arg(formatBytes(st.value(QStringLiteral("bytes")).toLongLong()),
                                                    date.isEmpty() ? QString() : QStringLiteral(", data of %1").arg(date))
                                         : QStringLiteral("none");
    const QVariantMap lk = st.value(QStringLiteral("lookups")).toMap();
    v[QStringLiteral("lookups")] =
        QStringLiteral("%1 found, %2 not found").arg(lk.value(QStringLiteral("found")).toInt()).arg(lk.value(QStringLiteral("missed")).toInt());
    const QVariantMap set = st.value(QStringLiteral("settings")).toMap();
    v[QStringLiteral("online")] = set.value(QStringLiteral("online"), true).toBool();
    v[QStringLiteral("hasToken")] = set.value(QStringLiteral("hasToken")).toBool();
    v[QStringLiteral("busy")] = m_dbBusy;
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
    m_dbView[QStringLiteral("busy")] = true;
    m_dbView[QStringLiteral("files")] = QStringLiteral("downloading the OpenCelliD file of MCC %1...").arg(c->mcc);
    emit dbChanged();
    m_db->downloadOpenCellid(c->mcc, [this](const QString &error, int cells) {
        m_dbBusy = false;
        if (!error.isEmpty()) DialogHub::toast(QStringLiteral("OpenCelliD download failed: ") + error);
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
    m_dbView[QStringLiteral("busy")] = true;
    emit dbChanged();
    m_db->downloadBox(
        c->box[0], c->box[1], c->box[2], c->box[3],
        [this](int done, int total, int cells, int failed) {
            m_dbView[QStringLiteral("status")] = QStringLiteral("downloading: area %1 of %2, %3 cells%4")
                                                     .arg(done)
                                                     .arg(total)
                                                     .arg(QLocale().toString(cells))
                                                     .arg(failed ? QStringLiteral(", %1 failed").arg(failed) : QString());
            emit dbChanged();
        },
        [this](int blocks, int, int failed) {
            m_dbBusy = false;
            if (failed) DialogHub::toast(QStringLiteral("%1 of %2 areas could not be downloaded: try again later.").arg(failed).arg(blocks));
            m_queryKey.clear();
            dbRefresh();
        });
}

void LteController::dbImport(const QUrl &file)
{
    const QString path = file.isLocalFile() ? file.toLocalFile() : file.toString();
    if (path.isEmpty() || m_dbBusy) return;
    m_dbBusy = true;
    m_dbView[QStringLiteral("busy")] = true;
    emit dbChanged();
    m_db->importCsv(path, [this](const QString &error, int cells) {
        m_dbBusy = false;
        if (!error.isEmpty()) DialogHub::toast(QStringLiteral("Import failed: ") + error);
        else LOG_I("celldb") << "imported:" << cells << "LTE cells";
        m_queryKey.clear();
        dbRefresh();
    });
}

void LteController::dbClear()
{
    DialogHub::confirm(QStringLiteral("Delete the downloaded, imported and looked-up cells? The built-in ones stay."), [this](bool ok) {
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
        for (const char *k : {"serving", "signal", "neighbours", "changes", "mast"}) m[QLatin1String(k)] = QStringLiteral("--");
        if (c.valid) m[QStringLiteral("serving")] = QStringLiteral("no CORV_CELL_INFO for 10 s");
    } else {
        m[QStringLiteral("serving")] = c.eci ? QStringLiteral("%1 eNB %2 sector %3 (ECI %4), TAC %5, PCI %6, EARFCN %7 (band %8, %9 downlink), %10")
                                                   .arg(operatorName(c.mcc, c.mnc))
                                                   .arg(c.enb)
                                                   .arg(c.sector)
                                                   .arg(c.eci)
                                                   .arg(c.tac)
                                                   .arg(c.pci)
                                                   .arg(c.earfcn)
                                                   .arg(c.band)
                                                   .arg(frequencyText(c.earfcn, c.band, QStringLiteral("?")), c.state)
                                             : QStringLiteral("no cell (%1)").arg(c.state);
        m[QStringLiteral("signal")] = QStringLiteral("RSRP %1, RSRQ %2, SINR %3, RSSI %4")
                                          .arg(v(c.rsrp, QStringLiteral("dBm")), v(c.rsrq, QStringLiteral("dB")), v(c.sinr, QStringLiteral("dB")),
                                               v(c.rssi, QStringLiteral("dBm")));
        if (c.neighbours) {
            QStringList nb;
            for (const CellNeighbour &n : c.nb)
                nb << QStringLiteral("PCI %1%2 %3").arg(n.pci).arg(n.earfcn != c.earfcn ? QStringLiteral("/%1").arg(n.earfcn) : QString(),
                                                                   v(n.rsrp, QStringLiteral("dBm")));
            m[QStringLiteral("neighbours")] = QStringLiteral("%1: %2").arg(c.neighbours).arg(nb.join(QStringLiteral(", ")));
        } else {
            m[QStringLiteral("neighbours")] = QStringLiteral("0");
        }
        m[QStringLiteral("changes")] = QString::number(c.handovers) +
                                       (c.changedAt ? QStringLiteral(", last %1 s ago").arg(std::llround((wallMs() - c.changedAt) / 1000.0)) : QString());
        QString mast = cellMap().value(QStringLiteral("show")).toBool() ? QStringLiteral("looking for the site...")
                                                                         : QStringLiteral("shown with SHOW CELL SITES ON THE MAP");
        if (!m_queryError.isEmpty()) {
            mast = QStringLiteral("database: ") + m_queryError;
        } else if (m_serving >= 0) {
            const CellSite &site = m_sites[std::size_t(m_serving)];
            QStringList srcs;
            for (const CellRecord &x : site.cells)
                if (!srcs.contains(sourceName(x.src))) srcs << sourceName(x.src);
            mast = QStringLiteral("%1, %2 (estimate from %3 cell%4 of the eNB, %5)")
                       .arg(QString::number(site.lat, 'f', 5), QString::number(site.lon, 'f', 5))
                       .arg(site.cells.size())
                       .arg(site.cells.size() > 1 ? QStringLiteral("s") : QString(), srcs.join(QStringLiteral(", ")));
            if (site.mnc != c.mnc) mast += QStringLiteral("; shared cell, listed under ") + operatorName(site.mcc, site.mnc);
        } else if (m_lookingUp) {
            mast = QStringLiteral("not in the database: looking it up online");
        } else if (!m_querying && m_queryCentre && c.eci) {
            mast = QStringLiteral("not in the database nor online (site never measured, or newer than the data)");
        }
        m[QStringLiteral("mast")] = mast;
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
                      cellMap().value(QStringLiteral("table")).toBool();
    QVariantMap r{{QStringLiteral("live"), live}};
    if (live) {
        r[QStringLiteral("changed")] = c.changedAt && now - c.changedAt < 5000;
        r[QStringLiteral("cell")] = c.eci ? QStringLiteral("%1 %2/%3").arg(operatorShort(c.mcc, c.mnc)).arg(c.enb).arg(c.sector)
                                          : QStringLiteral("NO CELL (%1)").arg(c.state);
        r[QStringLiteral("band")] = c.eci ? QStringLiteral("%1 · %2").arg(frequencyText(c.earfcn, c.band, QStringLiteral("-"))).arg(c.pci)
                                          : QStringLiteral("---");
        r[QStringLiteral("rssi")] = v(c.rssi);
        r[QStringLiteral("sig")] = QStringLiteral("%1 · %2").arg(v(c.rsrp), v(c.sinr));
        r[QStringLiteral("nb")] = QString::number(c.neighbours);
        r[QStringLiteral("ho")] = QString::number(c.handovers);
        const bool notInDb = c.eci && m_serving < 0 && !m_querying && !m_lookingUp && m_queryCentre;
        r[QStringLiteral("dist")] = m_servingKm ? QString::number(*m_servingKm, 'f', 1) : notInDb ? QStringLiteral("NOT IN DB") : QStringLiteral("---");
        r[QStringLiteral("distUnit")] = m_servingKm ? QStringLiteral("km") : QString();
    }
    if (r == m_cellRows) return;
    m_cellRows = r;
    emit cellRowsChanged();
}

} // namespace corv
