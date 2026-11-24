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

using namespace Qt::StringLiterals;

namespace corv {

UpdateController *UpdateController::s_instance = nullptr;

namespace {
const QString kCfg = u"update-cfg"_s;
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
    // A build without a release label ("2.0.3-dev") is nobody's older version: it only looks when asked
    if (checkAtStart() && !version().endsWith("-dev"_L1))
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
    if (m_state == "downloading"_L1 || m_state == "installing"_L1 || m_state == "ready"_L1)
        return true;
    const QString skipped = SysConfig::s_instance ? SysConfig::s_instance->jsonField(kCfg, u"skip"_s, QString()).toString() : QString();
    return latest() != skipped;
}

bool UpdateController::checkAtStart() const
{
    return SysConfig::s_instance ? SysConfig::s_instance->jsonField(kCfg, u"checkAtStart"_s, true).toBool() : true;
}

void UpdateController::setCheckAtStart(bool on)
{
    if (SysConfig::s_instance) SysConfig::s_instance->setJsonField(kCfg, u"checkAtStart"_s, on);
}

QString UpdateController::installText() const
{
    switch (platform()) {
    case update::Platform::Windows:
        return u"Downloads the installer, updates CORV GCS in its folder and starts the new version."_s;
    case update::Platform::MacOS:
        return u"Downloads the disk image and opens it: drag CORV GCS to Applications, replacing the old one."_s;
    case update::Platform::Linux:
        break;
    }
    if (!qEnvironmentVariable("APPIMAGE").isEmpty())
        return u"Downloads the new AppImage, puts it in place of this one and restarts CORV GCS."_s;
    return u"Downloads the AppImage into the Downloads folder."_s;
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
    if (m_state == "checking"_L1 || m_state == "downloading"_L1 || m_state == "installing"_L1)
        return;
    const QString forced = qEnvironmentVariable("CORV_UPDATE_URL");
    const QUrl url(forced.isEmpty() ? u"https://api.github.com/repos/%1/releases?per_page=30"_s.arg(repo()) : forced);
    QNetworkRequest req(url);
    req.setRawHeader("Accept", "application/vnd.github+json");
    req.setRawHeader("X-GitHub-Api-Version", "2022-11-28");
    req.setHeader(QNetworkRequest::UserAgentHeader, kUserAgent);
    setState(u"checking"_s, u"Checking for updates…"_s);
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
        setState(u"error"_s, code == 403 || code == 429 ? u"GitHub refused the check for now (rate limit): try again later"_s
                                                                      : u"Update check failed: %1"_s.arg(r->errorString()));
        return;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(r->readAll());
    const auto current = update::parseVersion(version());
    if (!doc.isArray() || !current) {
        setState(u"error"_s, u"Update check: unexpected answer"_s);
        return;
    }
    // A pre-release moves to newer pre-releases; a final release only to final ones
    m_release = update::newest(doc.array(), *current, current->isPrerelease(), platform(), update::currentArch());
    m_asset = m_release ? update::packageFor(*m_release, platform(), update::currentArch()) : std::nullopt;
    if (!m_release) {
        LOG_I("update") << "up to date:" << version() << "(" << doc.array().size() << "releases of" << repo() << ")";
        setState(u"uptodate"_s, u"CORV GCS %1 is up to date"_s.arg(version()));
        return;
    }
    LOG_I("update") << "available:" << latest() << m_asset->name << m_asset->size << "bytes";
    setState(u"available"_s, u"CORV GCS %1 is available"_s.arg(latest()));
    if (atStart && notice()) DialogHub::toast(u"CORV GCS %1 is available: UPDATE at the top right"_s.arg(latest()));
}

// ── Download ────────────────────────────────────────────────────────────

/// Where the package goes: the running AppImage itself (replaced at the end),
/// a temporary folder for the Windows setup, Downloads otherwise.
QString UpdateController::downloadTarget() const
{
    if (platform() == update::Platform::Windows)
        return QDir(QStandardPaths::writableLocation(QStandardPaths::TempLocation)).filePath(u"corv-gcs-update/"_s + m_asset->name);
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
    if (m_state != "available"_L1 && m_state != "error"_L1) return;
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    if (s && s->connected && s->armed) {
        setState(m_state, u"Disarm the vehicle before updating"_s);
        DialogHub::toast(m_message);
        return;
    }
    m_target = downloadTarget();
    QDir().mkpath(QFileInfo(m_target).absolutePath());
    m_file = std::make_unique<QFile>(m_target + u".part"_s);
    if (!m_file->open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        setState(u"error"_s, u"Cannot write %1"_s.arg(m_file->fileName()));
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
    setState(u"downloading"_s, u"Downloading %1…"_s.arg(m_asset->name));
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
            setState(u"available"_s, u"Download cancelled"_s);
            return;
        }
        LOG_W("update") << "download failed:" << r->errorString();
        setState(u"error"_s, u"Download failed: %1"_s.arg(r->errorString()));
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
            setState(u"error"_s, u"The download is damaged (SHA-256 does not match): try again"_s);
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
        setState(u"error"_s, why);
    };
    // The finished file under its own name (an atomic replace on POSIX)
    if (platform() == update::Platform::Windows) QFile::remove(m_target);
    if (std::rename(QFile::encodeName(part).constData(), QFile::encodeName(m_target).constData()) != 0)
        return fail(u"Cannot put the download in place: %1"_s.arg(m_target));
    LOG_I("update") << "installing" << latest() << "from" << m_target;

    switch (platform()) {
    case update::Platform::Windows:
        // Silent, into the folder of the installed copy; it closes this one and starts the new
        if (!QProcess::startDetached(m_target, {u"/UPDATE"_s}))
            return fail(u"Cannot start the installer %1"_s.arg(m_target));
        setState(u"installing"_s, u"Installing CORV GCS %1…"_s.arg(latest()));
        QTimer::singleShot(500, qApp, &QCoreApplication::quit);
        return;
    case update::Platform::MacOS:
        QDesktopServices::openUrl(QUrl::fromLocalFile(m_target));
        setState(u"ready"_s, u"Drag CORV GCS to Applications, replacing the old one, then start it again"_s);
        return;
    case update::Platform::Linux:
        break;
    }
    QFile::setPermissions(m_target, QFile::permissions(m_target) | QFileDevice::ExeOwner | QFileDevice::ExeGroup | QFileDevice::ExeOther);
    if (m_target == qEnvironmentVariable("APPIMAGE")) {
        // This AppImage now holds the new version (the running one stays mounted
        // until it exits): start it with the same arguments
        if (!QProcess::startDetached(m_target, QCoreApplication::arguments().mid(1)))
            return fail(u"Updated: start %1 again"_s.arg(m_target));
        setState(u"installing"_s, u"Restarting with CORV GCS %1…"_s.arg(latest()));
        QTimer::singleShot(500, qApp, &QCoreApplication::quit);
        return;
    }
    QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(m_target).absolutePath()));
    setState(u"ready"_s, u"Saved to %1"_s.arg(m_target));
}

void UpdateController::skip()
{
    if (!m_release || !SysConfig::s_instance) return;
    SysConfig::s_instance->setJsonField(kCfg, u"skip"_s, latest());
    emit changed();
}

void UpdateController::openPage()
{
    const QString page = pageUrl().isEmpty() ? u"https://github.com/%1/releases"_s.arg(repo()) : pageUrl();
    QDesktopServices::openUrl(QUrl(page));
}

} // namespace corv
