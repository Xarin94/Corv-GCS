#include "ui/SitlController.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "link/LinkManager.h"
#include "terrain/Hgt.h"
#include "ui/SysConfig.h"
#include "vehicle/ConnectionManager.h"

#include <QDir>
#include <QFile>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QRegularExpression>
#include <QTimer>
#include <cmath>

namespace corv {

namespace {

struct Vehicle {
    const char *key, *label, *binary, *model;
};
const Vehicle kVehicles[] = {
    {"copter", "Copter (Quad)", "arducopter", "quad"},
    {"copter12s", "Copter 12S High-Speed", "arducopter", "X:quad12s.json"},
    {"tri12s", "Tricopter 12S (3x X8 G2)", "arducopter", "tri:tri12s.json"},
    {"heli", "Helicopter", "arducopter", "heli"},
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
    if (vehicle == QLatin1String("plane") || vehicle == QLatin1String("quadplane")) return QStringLiteral("Plane");
    if (vehicle == QLatin1String("rover") || vehicle == QLatin1String("boat") || vehicle == QLatin1String("sailboat")) return QStringLiteral("Rover");
    if (vehicle == QLatin1String("sub") || vehicle == QLatin1String("subnogps")) return QStringLiteral("Sub");
    return QStringLiteral("Copter");
}

bool isRov(const QString &v) { return v == QLatin1String("sub") || v == QLatin1String("subnogps"); }
bool isBoat(const QString &v) { return v == QLatin1String("boat") || v == QLatin1String("sailboat"); }

constexpr double kLandLat = 47.2603, kLandLon = 11.3439;
constexpr double kRovLat = 45.60319, kRovLon = 10.67127;    // centre of Lake Garda
constexpr double kBoatLat = 45.4960, kBoatLon = 10.6490;    // southern basin

#ifdef Q_OS_WIN
QString wslPath(QString p)
{
    p.replace(QLatin1Char('\\'), QLatin1Char('/'));
    static const QRegularExpression drive(QStringLiteral("^([A-Za-z]):"));
    const auto m = drive.match(p);
    if (m.hasMatch()) p = QStringLiteral("/mnt/") + m.captured(1).toLower() + p.mid(2);
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
        out << QVariantMap{{QStringLiteral("value"), QString::fromLatin1(v.key)}, {QStringLiteral("label"), QString::fromUtf8(v.label)}};
    return out;
}

QVariantMap SitlController::suggestHome(const QString &vehicle, double lat, double lon) const
{
    const auto same = [&](double hl, double ho) { return std::abs(lat - hl) < 1e-6 && std::abs(lon - ho) < 1e-6; };
    double tl = kLandLat, to = kLandLon;
    if (isRov(vehicle)) tl = kRovLat, to = kRovLon;
    else if (isBoat(vehicle)) tl = kBoatLat, to = kBoatLon;
    if (std::isnan(lat) || same(kLandLat, kLandLon) || same(kRovLat, kRovLon) || same(kBoatLat, kBoatLon))
        return QVariantMap{{QStringLiteral("lat"), tl}, {QStringLiteral("lon"), to}};
    return QVariantMap{{QStringLiteral("lat"), lat}, {QStringLiteral("lon"), lon}};
}

void SitlController::setStatus(const QString &text)
{
    m_status = text;
    emit statusChanged();
}

QString SitlController::sitlDir() const { return DataRoot::dir(QStringLiteral("sitl")); }

QString SitlController::binaryPath(const QString &vehicle, const QString &version) const
{
    const Vehicle *v = find(vehicle);
    // Always the Linux binary, also on Windows (run through WSL)
    return v ? QDir(sitlDir()).filePath(QStringLiteral("%1_%2").arg(QLatin1String(v->binary), version)) : QString();
}

/// A binary too old for the vehicle's model counts as missing: ArduPlane reads
/// plane-jet:<file>.json from 4.7 on, older builds silently ignore it.
bool SitlController::binaryUsable(const QString &vehicle, const QString &version) const
{
    QFile f(binaryPath(vehicle, version));
    if (!f.exists()) return false;
    if (vehicle != QLatin1String("plane")) return true;
    if (!f.open(QIODevice::ReadOnly)) return false;
    return f.readAll().contains("Loaded plane aero coefficients");
}

/// The bundled frame models and default parameters, refreshed in the SITL dir
/// when they differ (hand edits survive until the app ships an update).
void SitlController::syncAssets() const
{
    QDir src(QStringLiteral(":/sitl-defaults"));
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
            m_downloadText = QStringLiteral("DOWNLOAD");
            setStatus(QStringLiteral("Download failed: ") + error);
            return;
        }
        m_downloadText = QStringLiteral("DOWNLOADED");
        emit statusChanged();
        QTimer::singleShot(2000, this, [this] {
            m_downloadText = QStringLiteral("DOWNLOAD");
            emit statusChanged();
        });
    });
}

void SitlController::startDownload(const QString &vehicle, const QString &version, std::function<void(bool, const QString &)> done)
{
    const Vehicle *v = find(vehicle);
    if (!v) return done(false, QStringLiteral("Unknown vehicle: ") + vehicle);
    const QUrl url(QStringLiteral("https://firmware.ardupilot.org/%1/%2/SITL_x86_64_linux_gnu/%3").arg(firmwareDir(vehicle), version, QLatin1String(v->binary)));
    LOG_I("sitl") << "downloading" << vehicle << version << "from" << url.toString();
    m_downloadText = QStringLiteral("DOWNLOADING...");
    setStatus(QStringLiteral("Downloading %1 %2...").arg(vehicle, version));
    if (!m_net) m_net = new QNetworkAccessManager(this);
    QNetworkRequest req(url);
    req.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    m_reply = m_net->get(req);
    const QString dest = binaryPath(vehicle, version);
    connect(m_reply, &QNetworkReply::downloadProgress, this, [this](qint64 got, qint64 total) {
        if (total > 0) setStatus(QStringLiteral("Downloading... %1%").arg(std::lround(100.0 * got / total)));
    });
    QNetworkReply *reply = m_reply;
    connect(reply, &QNetworkReply::finished, this, [this, reply, dest, vehicle, version, done] {
        reply->deleteLater();
        const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        if (reply->error() != QNetworkReply::NoError || status != 200)
            return done(false, status ? QStringLiteral("HTTP %1").arg(status) : reply->errorString());
        QFile f(dest);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return done(false, f.errorString());
        f.write(reply->readAll());
        f.close();
        f.setPermissions(f.permissions() | QFileDevice::ExeOwner | QFileDevice::ExeGroup | QFileDevice::ExeOther);
        LOG_I("sitl") << "downloaded to" << dest;
        setStatus(QStringLiteral("%1 %2 ready").arg(vehicle, version));
        done(true, QString());
    });
}

void SitlController::launch(const QString &vehicle, const QString &version, double homeLat, double homeLon, int speedup)
{
    if (m_busy) return;
    if (!std::isfinite(homeLat) || homeLat == 0) homeLat = kLandLat;
    if (!std::isfinite(homeLon) || homeLon == 0) homeLon = kLandLon;
    // Terrain elevation at home: a ROV starts on the water at 0 m
    const std::optional<double> terrain = HgtStore::instance().elevation(homeLat, homeLon);
    const bool rov = isRov(vehicle);
    const double homeAlt = !rov && terrain && *terrain > 0 ? *terrain : 0;
    LOG_I("sitl") << "home" << homeLat << homeLon << "terrain" << (terrain ? QString::number(*terrain) : QStringLiteral("none")) << "homeAlt" << homeAlt;
    if (rov) beginRovSession(vehicle);
    else endRovSession();
    m_busy = true;
    emit statusChanged();
    if (binaryUsable(vehicle, version)) {
        start(vehicle, version, homeLat, homeLon, homeAlt, speedup);
        return;
    }
    setStatus(QStringLiteral("Binary not found, downloading..."));
    startDownload(vehicle, version, [=, this](bool ok, const QString &error) {
        m_downloadText = QStringLiteral("DOWNLOAD");
        if (!ok) {
            m_busy = false;
            endRovSession();
            setStatus(QStringLiteral("Launch failed: ") + error);
            return;
        }
        if (!binaryUsable(vehicle, version)) {
            m_busy = false;
            setStatus(QStringLiteral("Launch failed: this %1 SITL build is too old for the %2 model — download it again.").arg(version, vehicle));
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
    QStringList args{QStringLiteral("--model"), QLatin1String(v->model),
                     QStringLiteral("--home"), QStringLiteral("%1,%2,%3,0").arg(homeLat, 0, 'g', 10).arg(homeLon, 0, 'g', 10).arg(homeAlt, 0, 'g', 8),
                     QStringLiteral("--speedup"), QString::number(std::max(1, speedup)), QStringLiteral("-I0")};
    // Default parameters by bare file name: SITL resolves them from its cwd
    const QString defaults = QStringLiteral("default_params_%1.parm").arg(vehicle);
    if (QFile::exists(QDir(sitlDir()).filePath(defaults))) args << QStringLiteral("--defaults") << defaults;

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
        if (m_proc == p) setStatus(QStringLiteral("SITL stopped (code %1)").arg(code));
        p->deleteLater();
    });
#ifdef Q_OS_WIN
    // The Linux binary inside WSL, copied to a native WSL dir (SITL writes its
    // eeprom and logs in its cwd); --wipe clears a corrupted eeprom
    const QString runDir = QStringLiteral("/tmp/sitl_run");
    args << QStringLiteral("--wipe");
    const QString script = QStringList{QStringLiteral("mkdir -p %1").arg(runDir),
                                       QStringLiteral("cp \"%1\" %2/sitl_bin").arg(wslPath(binaryPath(vehicle, version)), runDir),
                                       QStringLiteral("(cp \"%1\"/*.parm \"%1\"/*.json %2/ 2>/dev/null || true)").arg(wslPath(sitlDir()), runDir),
                                       QStringLiteral("chmod +x %1/sitl_bin").arg(runDir), QStringLiteral("cd %1").arg(runDir),
                                       QStringLiteral("./sitl_bin %1 2>&1").arg(args.join(QLatin1Char(' ')))}
                               .join(QStringLiteral(" && "));
    LOG_I("sitl") << "launching via WSL:" << args.join(QLatin1Char(' '));
    p->start(QStringLiteral("wsl"), {QStringLiteral("bash"), QStringLiteral("-c"), script});
#else
    p->setWorkingDirectory(sitlDir());
    LOG_I("sitl") << "launching:" << binaryPath(vehicle, version) << args.join(QLatin1Char(' '));
    p->start(binaryPath(vehicle, version), args);
#endif
    setStatus(QStringLiteral("Starting %1 SITL...").arg(vehicle));
    // SITL needs time to bind its TCP listeners; then connect to serial0 (5760)
    QTimer::singleShot(5000, this, [this, p, vehicle] {
        m_busy = false;
        if (m_proc != p || p->state() == QProcess::NotRunning) {
            endRovSession();
            setStatus(QStringLiteral("Launch failed: SITL process failed to start"));
            return;
        }
        setStatus(QStringLiteral("%1 SITL running").arg(vehicle));
        QTimer::singleShot(1000, this, [this, vehicle] {
            m_conn->connectLink(QLatin1String(LinkManager::kMavlinkTcp), {{QStringLiteral("host"), QStringLiteral("127.0.0.1")}, {QStringLiteral("port"), 5760}});
            setStatus(QStringLiteral("%1 SITL running — connected (mavlink-tcp)").arg(vehicle));
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
    QProcess::execute(QStringLiteral("wsl"), {QStringLiteral("bash"), QStringLiteral("-c"),
                                              QStringLiteral("pkill -9 -f sitl_bin 2>/dev/null; pkill -9 -f arducopter 2>/dev/null")});
#endif
}

void SitlController::stop()
{
    m_conn->disconnectLink();
    killSitl();
    endRovSession();
    setStatus(QStringLiteral("SITL stopped"));
}

void SitlController::connectUdp(const QString &host, int port)
{
    m_conn->connectLink(QLatin1String(LinkManager::kMavlinkUdp), {{QStringLiteral("host"), host.isEmpty() ? QStringLiteral("127.0.0.1") : host},
                                                                   {QStringLiteral("port"), port > 0 ? port : 14550}});
    setStatus(QStringLiteral("Connected to SITL"));
}

/// Before a ROV launch: relative navigation for the one without a GPS.
void SitlController::beginRovSession(const QString &vehicle)
{
    endRovSession();
    if (vehicle != QLatin1String("subnogps") || !m_sys) return;
    m_rovSession = true;
    m_rovSavedNav = m_sys->navPosition();
    m_sys->setNavPosition(QStringLiteral("relative"));
}

void SitlController::endRovSession()
{
    if (!m_rovSession || !m_sys) return;
    m_sys->setNavPosition(m_rovSavedNav);
    m_rovSession = false;
}

} // namespace corv
