// UpdateController.h - the update check at start and the one-click install
// (QML singleton "Updates": the UPDATE button of the header, its dialog and
// SYS CONFIG → UPDATES).
//
// A few seconds after start, unless switched off, the GitHub releases of the
// public repository (CORV_UPDATE_REPO) are listed, anonymously; UpdateCheck
// picks the newest one above the running version that has a package for this
// system. Nothing is downloaded until INSTALL: then only that package, into a
// temporary file, checked against the SHA-256 GitHub publishes for it. Then:
//
//   Windows   the NSIS setup runs with /UPDATE (silent, same folder, starts the
//             new version when done) and the GCS closes
//   Linux     the running AppImage file is replaced in place and the GCS
//             restarts; outside an AppImage the package lands in Downloads
//   macOS     the DMG is opened for the drag to Applications
//
// Never while the vehicle is armed. A version can be skipped: it is not
// announced again, a newer one is. Settings in "update-cfg" (checkAtStart,
// skip). CORV_UPDATE_URL replaces the API address (tests). GUI thread.
#pragma once

#include "services/UpdateCheck.h"

#include <QFile>
#include <QNetworkAccessManager>
#include <QObject>
#include <QPointer>
#include <QQmlEngine>
#include <memory>
#include <optional>

class QNetworkReply;

namespace corv {

class VehicleHub;

class UpdateController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Updates)
    QML_SINGLETON

    Q_PROPERTY(QString version READ version CONSTANT)
    Q_PROPERTY(QString repo READ repo CONSTANT)
    /// idle, checking, uptodate, available, downloading, installing, ready, error
    Q_PROPERTY(QString state READ state NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
    Q_PROPERTY(QString latest READ latest NOTIFY changed)
    Q_PROPERTY(QString title READ title NOTIFY changed)
    Q_PROPERTY(QString notes READ notes NOTIFY changed)
    Q_PROPERTY(QString pageUrl READ pageUrl NOTIFY changed)
    Q_PROPERTY(QString packageName READ packageName NOTIFY changed)
    Q_PROPERTY(qint64 packageSize READ packageSize NOTIFY changed)
    /// The header shows UPDATE: a version not skipped, or a download under way
    Q_PROPERTY(bool notice READ notice NOTIFY changed)
    Q_PROPERTY(double progress READ progress NOTIFY progressChanged)
    Q_PROPERTY(bool checkAtStart READ checkAtStart WRITE setCheckAtStart NOTIFY changed)
    /// What INSTALL does on this system, for the dialog
    Q_PROPERTY(QString installText READ installText CONSTANT)

public:
    UpdateController(VehicleHub *hub, QObject *parent = nullptr);
    ~UpdateController() override;

    static UpdateController *s_instance;
    static UpdateController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    QString version() const;
    QString repo() const;
    QString state() const { return m_state; }
    QString message() const { return m_message; }
    QString latest() const { return m_release ? m_release->version.toString() : QString(); }
    QString title() const { return m_release ? m_release->name : QString(); }
    QString notes() const { return m_release ? m_release->notes : QString(); }
    QString pageUrl() const { return m_release ? m_release->page : QString(); }
    QString packageName() const { return m_asset ? m_asset->name : QString(); }
    qint64 packageSize() const { return m_asset ? m_asset->size : 0; }
    bool notice() const;
    double progress() const { return m_progress; }
    bool checkAtStart() const;
    void setCheckAtStart(bool on);
    QString installText() const;

    Q_INVOKABLE void check();
    Q_INVOKABLE void install();
    Q_INVOKABLE void cancel();
    /// Not announced again; a newer version is
    Q_INVOKABLE void skip();
    Q_INVOKABLE void openPage();

signals:
    void changed();
    void progressChanged();

private:
    void setState(const QString &state, const QString &message = {});
    void onListed(QNetworkReply *reply);
    void onDownloaded();
    void finishInstall(const QString &file);
    QString downloadTarget() const;

    VehicleHub *m_hub;
    QNetworkAccessManager m_net;
    QString m_state = QStringLiteral("idle");
    QString m_message;
    std::optional<update::Release> m_release;
    std::optional<update::Asset> m_asset;
    QPointer<QNetworkReply> m_reply;
    std::unique_ptr<QFile> m_file;
    QString m_target;   // where the finished download goes
    double m_progress = 0;
    bool m_atStart = false;   // the automatic check: quiet when nothing is found
};

} // namespace corv
