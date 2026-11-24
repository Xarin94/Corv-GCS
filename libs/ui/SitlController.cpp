#include "ui/SitlController.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "link/LinkManager.h"
#include "terrain/Hgt.h"
#include "terrain/HgtDownloader.h"
#include "ui/SysConfig.h"
#include "vehicle/ConnectionManager.h"

#include <QDir>
#include <QFile>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QRegularExpression>
#include <QTimer>
#include <cmath>
#include <memory>

using namespace Qt::StringLiterals;

namespace corv {

namespace {

struct Vehicle {
    const char *key, *label, *binary, *model;
};
const Vehicle kVehicles[] = {
    {"copter", "Copter (Quad)", "arducopter", "quad"},
    {"copter12s", "Copter 12S High-Speed", "arducopter", "X:quad12s.json"},
    {"tri12s", "Tricopter 12S (3x X8 G2)", "arducopter", "tri:tri12s.json"},
    {"heli", "Helicopter", "arducopter-heli", "heli"},   // a firmware of its own: the copter one has no helicopter frames
    {"plane", "Plane", "arduplane", "plane-jet:jet600.json"},
    {"quadplane", "QuadPlane", "arduplane", "quadplane"},
    {"rover", "Rover", "ardurover", "rover"},
    {"boat", "Boat (ArduRover, waves on Lake Garda)", "ardurover", "motorboat"},
    {"sailboat", "Sailboat (ArduRover, wing sail and wind vane)", "ardurover", "sailboat"},
    {"sub", "ROV · Sub (BlueROV2, vectored)", "ardusub", "vectored"},
    {"subnogps", "ROV · no GPS (dead reckoning)", "ardusub", "vectored"},
};

const Vehicle *find(const QString &key)
{
    for (const Vehicle &v : kVehicles)
        if (key == QLatin1String(v.key)) return &v;
    return nullptr;
}

QString firmwareDir(const QString &vehicle)
{
    if (vehicle == "plane"_L1 || vehicle == "quadplane"_L1) return u"Plane"_s;
    if (vehicle == "rover"_L1 || vehicle == "boat"_L1 || vehicle == "sailboat"_L1) return u"Rover"_s;
    if (vehicle == "sub"_L1 || vehicle == "subnogps"_L1) return u"Sub"_s;
    return u"Copter"_s;
}

bool isRov(const QString &v) { return v == "sub"_L1 || v == "subnogps"_L1; }
bool isBoat(const QString &v) { return v == "boat"_L1 || v == "sailboat"_L1; }

constexpr double kLandLat = 47.2603, kLandLon = 11.3439;
constexpr double kRovLat = 45.60319, kRovLon = 10.67127;    // centre of Lake Garda
constexpr double kBoatLat = 45.4960, kBoatLon = 10.6490;    // southern basin

#ifdef Q_OS_WIN
QString wslPath(QString p)
{
    p.replace('\\'_L1, '/'_L1);
    static const QRegularExpression drive(u"^([A-Za-z]):"_s);
    const auto m = drive.match(p);
    if (m.hasMatch()) p = u"/mnt/"_s + m.captured(1).toLower() + p.mid(2);
    return p;
}
#endif

} // namespace

SitlController *SitlController::s_instance = nullptr;

SitlController::SitlController(ConnectionManager *conn, SysConfig *sys, QObject *parent) : QObject(parent), m_conn(conn), m_sys(sys)
{
    s_instance = this;
}

SitlController::~SitlController()
{
    if (m_proc && m_proc->state() != QProcess::NotRunning) {
        m_proc->kill();
        m_proc->waitForFinished(1000);
    }
}

QVariantList SitlController::vehicles() const
{
    QVariantList out;
    for (const Vehicle &v : kVehicles)
        out << QVariantMap{{u"value"_s, QString::fromLatin1(v.key)}, {u"label"_s, QString::fromUtf8(v.label)}};
    return out;
}

QVariantMap SitlController::suggestHome(const QString &vehicle, double lat, double lon) const
{
    const auto same = [&](double hl, double ho) { return std::abs(lat - hl) < 1e-6 && std::abs(lon - ho) < 1e-6; };
    double tl = kLandLat, to = kLandLon;
    if (isRov(vehicle)) tl = kRovLat, to = kRovLon;
    else if (isBoat(vehicle)) tl = kBoatLat, to = kBoatLon;
    if (std::isnan(lat) || same(kLandLat, kLandLon) || same(kRovLat, kRovLon) || same(kBoatLat, kBoatLon))
        return QVariantMap{{u"lat"_s, tl}, {u"lon"_s, to}};
    return QVariantMap{{u"lat"_s, lat}, {u"lon"_s, lon}};
}

void SitlController::setStatus(const QString &text)
{
    m_status = text;
    emit statusChanged();
}

QString SitlController::sitlDir() const { return DataRoot::dir(u"sitl"_s); }

QString SitlController::binaryPath(const QString &vehicle, const QString &version) const
{
    const Vehicle *v = find(vehicle);
    // Always the Linux binary, also on Windows (run through WSL)
    return v ? QDir(sitlDir()).filePath(u"%1_%2"_s.arg(QLatin1String(v->binary), version)) : QString();
}

/// A binary too old for the vehicle's model counts as missing: ArduPlane reads
/// plane-jet:<file>.json from 4.7 on, older builds silently ignore it.
bool SitlController::binaryUsable(const QString &vehicle, const QString &version) const
{
    QFile f(binaryPath(vehicle, version));
    if (!f.exists()) return false;
    if (vehicle != "plane"_L1) return true;
    if (!f.open(QIODevice::ReadOnly)) return false;
    return f.readAll().contains("Loaded plane aero coefficients");
}

/// The bundled frame models and default parameters, refreshed in the SITL dir
/// when they differ (hand edits survive until the app ships an update).
void SitlController::syncAssets() const
{
    QDir src(u":/sitl-defaults"_s);
    for (const QString &name : src.entryList(QDir::Files)) {
        QFile in(src.filePath(name));
        if (!in.open(QIODevice::ReadOnly)) continue;
        const QByteArray data = in.readAll();
        const QString dst = QDir(sitlDir()).filePath(name);
        QFile cur(dst);
        if (cur.open(QIODevice::ReadOnly) && cur.readAll() == data) continue;
        cur.close();
        QFile out(dst);
        if (out.open(QIODevice::WriteOnly | QIODevice::Truncate)) out.write(data);
        else LOG_I("sitl") << "could not sync asset" << name;
    }
}

void SitlController::download(const QString &vehicle, const QString &version)
{
    if (m_busy) return;
    m_busy = true;
    startDownload(vehicle, version, [this](bool ok, const QString &error) {
        m_busy = false;
        if (!ok) {
            m_downloadText = u"DOWNLOAD"_s;
            setStatus(u"Download failed: "_s + error);
            return;
        }
        m_downloadText = u"DOWNLOADED"_s;
        emit statusChanged();
        QTimer::singleShot(2000, this, [this] {
            m_downloadText = u"DOWNLOAD"_s;
            emit statusChanged();
        });
    });
}

void SitlController::startDownload(const QString &vehicle, const QString &version, std::function<void(bool, const QString &)> done)
{
    const Vehicle *v = find(vehicle);
    if (!v) return done(false, u"Unknown vehicle: "_s + vehicle);
    const QString binary = QLatin1String(v->binary);
    const QString board = binary.endsWith("-heli"_L1) ? u"SITL_x86_64_linux_gnu-heli"_s : u"SITL_x86_64_linux_gnu"_s;
    const QUrl url(u"https://firmware.ardupilot.org/%1/%2/%3/%4"_s.arg(firmwareDir(vehicle), version, board, binary));
    LOG_I("sitl") << "downloading" << vehicle << version << "from" << url.toString();
    m_downloadText = u"DOWNLOADING..."_s;
    setStatus(u"Downloading %1 %2..."_s.arg(vehicle, version));
    if (!m_net) m_net = new QNetworkAccessManager(this);
    QNetworkRequest req(url);
    req.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    m_reply = m_net->get(req);
    const QString dest = binaryPath(vehicle, version);
    connect(m_reply, &QNetworkReply::downloadProgress, this, [this](qint64 got, qint64 total) {
        if (total > 0) setStatus(u"Downloading... %1%"_s.arg(std::lround(100.0 * got / total)));
    });
    QNetworkReply *reply = m_reply;
    connect(reply, &QNetworkReply::finished, this, [this, reply, dest, vehicle, version, done] {
        reply->deleteLater();
        const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        if (reply->error() != QNetworkReply::NoError || status != 200)
            return done(false, status ? u"HTTP %1"_s.arg(status) : reply->errorString());
        QFile f(dest);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return done(false, f.errorString());
        f.write(reply->readAll());
        f.close();
        f.setPermissions(f.permissions() | QFileDevice::ExeOwner | QFileDevice::ExeGroup | QFileDevice::ExeOther);
        LOG_I("sitl") << "downloaded to" << dest;
        setStatus(u"%1 %2 ready"_s.arg(vehicle, version));
        done(true, QString());
    });
}

void SitlController::launch(const QString &vehicle, const QString &version, double homeLat, double homeLon, int speedup)
{
    if (m_busy) return;
    if (!std::isfinite(homeLat) || homeLat == 0) homeLat = kLandLat;
    if (!std::isfinite(homeLon) || homeLon == 0) homeLon = kLandLon;
    m_busy = true;
    // A ROV starts on the water at 0 m, anything else on the ground of its home
    if (isRov(vehicle)) {
        beginRovSession(vehicle);
        launchAt(vehicle, version, homeLat, homeLon, 0, speedup);
        return;
    }
    endRovSession();
    setStatus(u"Reading the elevation of the home..."_s);
    withHomeAltitude(homeLat, homeLon, [=, this](std::optional<double> terrain) {
        const double homeAlt = terrain && *terrain > 0 ? *terrain : 0;
        LOG_I("sitl") << "home" << homeLat << homeLon << "terrain" << (terrain ? QString::number(*terrain) : u"none"_s) << "homeAlt" << homeAlt;
        launchAt(vehicle, version, homeLat, homeLon, homeAlt, speedup);
    });
}

/// The ground at a point: its tile is read from disk, or downloaded, if it was not in memory yet
/// (a vehicle launched before the tile was in started at sea level, under the terrain).
void SitlController::withHomeAltitude(double lat, double lon, std::function<void(std::optional<double>)> done)
{
    const int la = int(std::floor(lat)), lo = int(std::floor(lon));
    auto *poll = new QTimer(this);
    poll->setInterval(100);
    auto tries = std::make_shared<int>(0);
    connect(poll, &QTimer::timeout, this, [=] {
        HgtStore &store = HgtStore::instance();
        HgtDownloader &dl = HgtDownloader::instance();
        const std::optional<double> h = store.elevation(lat, lon);
        const bool late = ++*tries > 200;   // 20 s: a slow network
        if (!h && !late && !store.get(la, lo)) {
            if (store.loading(la, lo) || dl.busy(la, lo)) return;
            if (store.missing(la, lo)) dl.ensure(la, lo);
            // Not missing: being read, or just downloaded
            if (!store.missing(la, lo) || dl.busy(la, lo)) return;
        }
        poll->deleteLater();
        done(h);
    });
    poll->start();
}

void SitlController::launchAt(const QString &vehicle, const QString &version, double homeLat, double homeLon, double homeAlt, int speedup)
{
    emit statusChanged();
    if (binaryUsable(vehicle, version)) {
        start(vehicle, version, homeLat, homeLon, homeAlt, speedup);
        return;
    }
    setStatus(u"Binary not found, downloading..."_s);
    startDownload(vehicle, version, [=, this](bool ok, const QString &error) {
        m_downloadText = u"DOWNLOAD"_s;
        if (!ok) {
            m_busy = false;
            endRovSession();
            setStatus(u"Launch failed: "_s + error);
            return;
        }
        if (!binaryUsable(vehicle, version)) {
            m_busy = false;
            setStatus(u"Launch failed: this %1 SITL build is too old for the %2 model — download it again."_s.arg(version, vehicle));
            return;
        }
        start(vehicle, version, homeLat, homeLon, homeAlt, speedup);
    });
}

void SitlController::start(const QString &vehicle, const QString &version, double homeLat, double homeLon, double homeAlt, int speedup)
{
    killSitl();
    syncAssets();
    const Vehicle *v = find(vehicle);
    QStringList args{u"--model"_s, QLatin1String(v->model),
                     u"--home"_s, u"%1,%2,%3,0"_s.arg(homeLat, 0, 'g', 10).arg(homeLon, 0, 'g', 10).arg(homeAlt, 0, 'g', 8),
                     u"--speedup"_s, QString::number(std::max(1, speedup)), u"-I0"_s};
    // Default parameters by bare file name: SITL resolves them from its cwd
    const QString defaults = u"default_params_%1.parm"_s.arg(vehicle);
    if (QFile::exists(QDir(sitlDir()).filePath(defaults))) args << u"--defaults"_s << defaults;

    auto *p = new QProcess(this);
    m_proc = p;
    p->setProcessChannelMode(QProcess::MergedChannels);
    connect(p, &QProcess::readyRead, p, [p] {
        for (const QByteArray &line : p->readAll().split('\n'))
            if (!line.trimmed().isEmpty()) LOG_I("sitl:out") << QString::fromLocal8Bit(line.trimmed());
    });
    connect(p, &QProcess::errorOccurred, this, [this, p](QProcess::ProcessError) {
        LOG_E("sitl") << "process error:" << p->errorString();
        setStatus(p->errorString());
    });
    connect(p, &QProcess::finished, this, [this, p](int code, QProcess::ExitStatus) {
        LOG_I("sitl") << "process exited: code" << code;
        if (m_proc == p) setStatus(u"SITL stopped (code %1)"_s.arg(code));
        p->deleteLater();
    });
#ifdef Q_OS_WIN
    // The Linux binary inside WSL, copied to a native WSL dir (SITL writes its
    // eeprom and logs in its cwd); --wipe clears a corrupted eeprom
    const QString runDir = u"/tmp/sitl_run"_s;
    args << u"--wipe"_s;
    const QString script = QStringList{u"mkdir -p %1"_s.arg(runDir),
                                       u"cp \"%1\" %2/sitl_bin"_s.arg(wslPath(binaryPath(vehicle, version)), runDir),
                                       u"(cp \"%1\"/*.parm \"%1\"/*.json %2/ 2>/dev/null || true)"_s.arg(wslPath(sitlDir()), runDir),
                                       u"chmod +x %1/sitl_bin"_s.arg(runDir), u"cd %1"_s.arg(runDir),
                                       u"./sitl_bin %1 2>&1"_s.arg(args.join(' '_L1))}
                               .join(u" && "_s);
    LOG_I("sitl") << "launching via WSL:" << args.join(' '_L1);
    p->start(u"wsl"_s, {u"bash"_s, u"-c"_s, script});
#else
    p->setWorkingDirectory(sitlDir());
    LOG_I("sitl") << "launching:" << binaryPath(vehicle, version) << args.join(' '_L1);
    p->start(binaryPath(vehicle, version), args);
#endif
    setStatus(u"Starting %1 SITL..."_s.arg(vehicle));
    // SITL needs time to bind its TCP listeners; then connect to serial0 (5760)
    QTimer::singleShot(5000, this, [this, p, vehicle] {
        m_busy = false;
        if (m_proc != p || p->state() == QProcess::NotRunning) {
            endRovSession();
            setStatus(u"Launch failed: SITL process failed to start"_s);
            return;
        }
        setStatus(u"%1 SITL running"_s.arg(vehicle));
        QTimer::singleShot(1000, this, [this, vehicle] {
            m_conn->connectLink(QLatin1String(LinkManager::kMavlinkTcp), {{u"host"_s, u"127.0.0.1"_s}, {u"port"_s, 5760}});
            setStatus(u"%1 SITL running — connected (mavlink-tcp)"_s.arg(vehicle));
        });
    });
}

void SitlController::killSitl()
{
    if (m_proc && m_proc->state() != QProcess::NotRunning) {
        LOG_I("sitl") << "killing SITL process";
        QProcess *p = m_proc;
        m_proc = nullptr;
        p->terminate();
        if (!p->waitForFinished(500)) p->kill();
    }
#ifdef Q_OS_WIN
    // Lingering SITL processes inside WSL
    QProcess::execute(u"wsl"_s, {u"bash"_s, u"-c"_s,
                                              u"pkill -9 -f sitl_bin 2>/dev/null; pkill -9 -f arducopter 2>/dev/null"_s});
#endif
}

void SitlController::stop()
{
    m_conn->disconnectLink();
    killSitl();
    endRovSession();
    setStatus(u"SITL stopped"_s);
}

void SitlController::connectUdp(const QString &host, int port)
{
    m_conn->connectLink(QLatin1String(LinkManager::kMavlinkUdp), {{u"host"_s, host.isEmpty() ? u"127.0.0.1"_s : host},
                                                                   {u"port"_s, port > 0 ? port : 14550}});
    setStatus(u"Connected to SITL"_s);
}

/// Before a ROV launch: relative navigation for the one without a GPS.
void SitlController::beginRovSession(const QString &vehicle)
{
    endRovSession();
    if (vehicle != "subnogps"_L1 || !m_sys) return;
    m_rovSession = true;
    m_rovSavedNav = m_sys->navPosition();
    m_sys->setNavPosition(u"relative"_s);
}

void SitlController::endRovSession()
{
    if (!m_rovSession || !m_sys) return;
    m_sys->setNavPosition(m_rovSavedNav);
    m_rovSession = false;
}

} // namespace corv
