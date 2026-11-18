#include "ui/UpdateController.h"

#include "core/DebugLog.h"
#include "ui/DialogHub.h"
#include "ui/SysConfig.h"
#include "vehicle/VehicleHub.h"
#include "vehicle/VehicleState.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QNetworkReply>
#include <QProcess>
#include <QStandardPaths>
#include <QTimer>
#include <QUrl>
#include <cstdio>
#include <utility>

namespace corv {

UpdateController *UpdateController::s_instance = nullptr;

namespace {
const QString kCfg = QStringLiteral("update-cfg");
// Late enough not to compete with the map tiles and the link at start
constexpr int kStartDelayMs = 6000;
const QByteArray kUserAgent = QByteArrayLiteral("CORV-GCS/" CORV_VERSION);

update::Platform platform() { return update::currentPlatform(); }
} // namespace

UpdateController::UpdateController(VehicleHub *hub, QObject *parent) : QObject(parent), m_hub(hub)
{
    s_instance = this;
    // Aborts a request that stalls, not a slow download
    m_net.setTransferTimeout(30000);
    if (SysConfig::s_instance)
        connect(SysConfig::s_instance, &SysConfig::jsonFieldChanged, this, [this](const QString &key) {
            if (key == kCfg) emit changed();
        });
    if (checkAtStart())
        QTimer::singleShot(kStartDelayMs, this, [this] {
            m_atStart = true;
            check();
        });
}

UpdateController::~UpdateController()
{
    if (m_reply) m_reply->abort();
    if (s_instance == this) s_instance = nullptr;
}

QString UpdateController::version() const { return QStringLiteral(CORV_VERSION); }
QString UpdateController::repo() const { return QStringLiteral(CORV_UPDATE_REPO); }

bool UpdateController::notice() const
{
    if (!m_release) return false;
    if (m_state == QLatin1String("downloading") || m_state == QLatin1String("installing") || m_state == QLatin1String("ready"))
        return true;
    const QString skipped = SysConfig::s_instance ? SysConfig::s_instance->jsonField(kCfg, QStringLiteral("skip"), QString()).toString() : QString();
    return latest() != skipped;
}

bool UpdateController::checkAtStart() const
{
    return SysConfig::s_instance ? SysConfig::s_instance->jsonField(kCfg, QStringLiteral("checkAtStart"), true).toBool() : true;
}

void UpdateController::setCheckAtStart(bool on)
{
    if (SysConfig::s_instance) SysConfig::s_instance->setJsonField(kCfg, QStringLiteral("checkAtStart"), on);
}

QString UpdateController::installText() const
{
    switch (platform()) {
    case update::Platform::Windows:
        return QStringLiteral("Downloads the installer, updates CORV GCS in its folder and starts the new version.");
    case update::Platform::MacOS:
        return QStringLiteral("Downloads the disk image and opens it: drag CORV GCS to Applications, replacing the old one.");
    case update::Platform::Linux:
        break;
    }
    if (!qEnvironmentVariable("APPIMAGE").isEmpty())
        return QStringLiteral("Downloads the new AppImage, puts it in place of this one and restarts CORV GCS.");
    return QStringLiteral("Downloads the AppImage into the Downloads folder.");
}

void UpdateController::setState(const QString &state, const QString &message)
{
    m_state = state;
    m_message = message;
    emit changed();
}

// ── Check ───────────────────────────────────────────────────────────────

void UpdateController::check()
{
    if (m_state == QLatin1String("checking") || m_state == QLatin1String("downloading") || m_state == QLatin1String("installing"))
        return;
    const QString forced = qEnvironmentVariable("CORV_UPDATE_URL");
    const QUrl url(forced.isEmpty() ? QStringLiteral("https://api.github.com/repos/%1/releases?per_page=30").arg(repo()) : forced);
    QNetworkRequest req(url);
    req.setRawHeader("Accept", "application/vnd.github+json");
    req.setRawHeader("X-GitHub-Api-Version", "2022-11-28");
    req.setHeader(QNetworkRequest::UserAgentHeader, kUserAgent);
    setState(QStringLiteral("checking"), QStringLiteral("Checking for updates…"));
    QNetworkReply *r = m_net.get(req);
    connect(r, &QNetworkReply::finished, this, [this, r] { onListed(r); });
}

void UpdateController::onListed(QNetworkReply *r)
{
    r->deleteLater();
    const bool atStart = std::exchange(m_atStart, false);
    if (r->error() != QNetworkReply::NoError) {
        const int code = r->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        LOG_W("update") << "release list failed:" << code << r->errorString();
        // Anonymous calls: 60 an hour per address
        setState(QStringLiteral("error"), code == 403 || code == 429 ? QStringLiteral("GitHub refused the check for now (rate limit): try again later")
                                                                      : QStringLiteral("Update check failed: %1").arg(r->errorString()));
        return;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(r->readAll());
    const auto current = update::parseVersion(version());
    if (!doc.isArray() || !current) {
        setState(QStringLiteral("error"), QStringLiteral("Update check: unexpected answer"));
        return;
    }
    // A pre-release moves to newer pre-releases; a final release only to final ones
    m_release = update::newest(doc.array(), *current, current->isPrerelease(), platform(), update::currentArch());
    m_asset = m_release ? update::packageFor(*m_release, platform(), update::currentArch()) : std::nullopt;
    if (!m_release) {
        LOG_I("update") << "up to date:" << version() << "(" << doc.array().size() << "releases of" << repo() << ")";
        setState(QStringLiteral("uptodate"), QStringLiteral("CORV GCS %1 is up to date").arg(version()));
        return;
    }
    LOG_I("update") << "available:" << latest() << m_asset->name << m_asset->size << "bytes";
    setState(QStringLiteral("available"), QStringLiteral("CORV GCS %1 is available").arg(latest()));
    if (atStart && notice()) DialogHub::toast(QStringLiteral("CORV GCS %1 is available: UPDATE at the top right").arg(latest()));
}

// ── Download ────────────────────────────────────────────────────────────

/// Where the package goes: the running AppImage itself (replaced at the end),
/// a temporary folder for the Windows setup, Downloads otherwise.
QString UpdateController::downloadTarget() const
{
    if (platform() == update::Platform::Windows)
        return QDir(QStandardPaths::writableLocation(QStandardPaths::TempLocation)).filePath(QStringLiteral("corv-gcs-update/") + m_asset->name);
    if (platform() == update::Platform::Linux) {
        const QString appImage = qEnvironmentVariable("APPIMAGE");
        if (!appImage.isEmpty() && QFileInfo(QFileInfo(appImage).absolutePath()).isWritable()) return appImage;
    }
    QString dir = QStandardPaths::writableLocation(QStandardPaths::DownloadLocation);
    if (dir.isEmpty()) dir = QDir::homePath();
    return QDir(dir).filePath(m_asset->name);
}

void UpdateController::install()
{
    if (!m_release || !m_asset || m_reply) return;
    if (m_state != QLatin1String("available") && m_state != QLatin1String("error")) return;
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    if (s && s->connected && s->armed) {
        setState(m_state, QStringLiteral("Disarm the vehicle before updating"));
        DialogHub::toast(m_message);
        return;
    }
    m_target = downloadTarget();
    QDir().mkpath(QFileInfo(m_target).absolutePath());
    m_file = std::make_unique<QFile>(m_target + QStringLiteral(".part"));
    if (!m_file->open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        setState(QStringLiteral("error"), QStringLiteral("Cannot write %1").arg(m_file->fileName()));
        m_file.reset();
        return;
    }
    QNetworkRequest req{QUrl(m_asset->url)};
    req.setRawHeader("Accept", "application/octet-stream");
    req.setHeader(QNetworkRequest::UserAgentHeader, kUserAgent);
    req.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    m_reply = m_net.get(req);
    m_progress = 0;
    emit progressChanged();
    connect(m_reply, &QNetworkReply::readyRead, this, [this] {
        if (m_file && m_reply) m_file->write(m_reply->readAll());
    });
    connect(m_reply, &QNetworkReply::downloadProgress, this, [this](qint64 got, qint64 total) {
        const qint64 t = total > 0 ? total : (m_asset ? m_asset->size : 0);
        m_progress = t > 0 ? std::min(1.0, double(got) / double(t)) : 0;
        emit progressChanged();
    });
    connect(m_reply, &QNetworkReply::finished, this, &UpdateController::onDownloaded);
    LOG_I("update") << "downloading" << m_asset->url << "to" << m_file->fileName();
    setState(QStringLiteral("downloading"), QStringLiteral("Downloading %1…").arg(m_asset->name));
}

void UpdateController::cancel()
{
    if (m_reply) m_reply->abort();
}

void UpdateController::onDownloaded()
{
    QNetworkReply *r = m_reply.data();
    m_reply = nullptr;
    if (!r || !m_file) return;
    r->deleteLater();
    m_file->write(r->readAll());
    m_file->close();
    const QString part = m_file->fileName();
    m_file.reset();
    if (r->error() != QNetworkReply::NoError) {
        QFile::remove(part);
        if (r->error() == QNetworkReply::OperationCanceledError) {
            setState(QStringLiteral("available"), QStringLiteral("Download cancelled"));
            return;
        }
        LOG_W("update") << "download failed:" << r->errorString();
        setState(QStringLiteral("error"), QStringLiteral("Download failed: %1").arg(r->errorString()));
        return;
    }
    // The package GitHub has, byte for byte
    if (!m_asset->sha256.isEmpty()) {
        QFile f(part);
        QCryptographicHash h(QCryptographicHash::Sha256);
        if (!f.open(QIODevice::ReadOnly) || !h.addData(&f) || h.result().toHex() != m_asset->sha256.toLatin1()) {
            f.close();
            QFile::remove(part);
            LOG_W("update") << "SHA-256 mismatch on" << m_asset->name;
            setState(QStringLiteral("error"), QStringLiteral("The download is damaged (SHA-256 does not match): try again"));
            return;
        }
    } else {
        LOG_W("update") << "no SHA-256 published for" << m_asset->name << "- relying on HTTPS";
    }
    finishInstall(part);
}

// ── Install ─────────────────────────────────────────────────────────────

void UpdateController::finishInstall(const QString &part)
{
    const auto fail = [&](const QString &why) {
        LOG_W("update") << why;
        setState(QStringLiteral("error"), why);
    };
    // The finished file under its own name (an atomic replace on POSIX)
    if (platform() == update::Platform::Windows) QFile::remove(m_target);
    if (std::rename(QFile::encodeName(part).constData(), QFile::encodeName(m_target).constData()) != 0)
        return fail(QStringLiteral("Cannot put the download in place: %1").arg(m_target));
    LOG_I("update") << "installing" << latest() << "from" << m_target;

    switch (platform()) {
    case update::Platform::Windows:
        // Silent, into the folder of the installed copy; it closes this one and starts the new
        if (!QProcess::startDetached(m_target, {QStringLiteral("/UPDATE")}))
            return fail(QStringLiteral("Cannot start the installer %1").arg(m_target));
        setState(QStringLiteral("installing"), QStringLiteral("Installing CORV GCS %1…").arg(latest()));
        QTimer::singleShot(500, qApp, &QCoreApplication::quit);
        return;
    case update::Platform::MacOS:
        QDesktopServices::openUrl(QUrl::fromLocalFile(m_target));
        setState(QStringLiteral("ready"), QStringLiteral("Drag CORV GCS to Applications, replacing the old one, then start it again"));
        return;
    case update::Platform::Linux:
        break;
    }
    QFile::setPermissions(m_target, QFile::permissions(m_target) | QFileDevice::ExeOwner | QFileDevice::ExeGroup | QFileDevice::ExeOther);
    if (m_target == qEnvironmentVariable("APPIMAGE")) {
        // This AppImage now holds the new version (the running one stays mounted
        // until it exits): start it with the same arguments
        if (!QProcess::startDetached(m_target, QCoreApplication::arguments().mid(1)))
            return fail(QStringLiteral("Updated: start %1 again").arg(m_target));
        setState(QStringLiteral("installing"), QStringLiteral("Restarting with CORV GCS %1…").arg(latest()));
        QTimer::singleShot(500, qApp, &QCoreApplication::quit);
        return;
    }
    QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(m_target).absolutePath()));
    setState(QStringLiteral("ready"), QStringLiteral("Saved to %1").arg(m_target));
}

void UpdateController::skip()
{
    if (!m_release || !SysConfig::s_instance) return;
    SysConfig::s_instance->setJsonField(kCfg, QStringLiteral("skip"), latest());
    emit changed();
}

void UpdateController::openPage()
{
    const QString page = pageUrl().isEmpty() ? QStringLiteral("https://github.com/%1/releases").arg(repo()) : pageUrl();
    QDesktopServices::openUrl(QUrl(page));
}

} // namespace corv
