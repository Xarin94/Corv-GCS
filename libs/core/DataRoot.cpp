#include "core/DataRoot.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QStandardPaths>
#include <QtGlobal>

namespace corv {

namespace {
QMutex g_lock;
QString g_root;
bool g_fallback = false;

bool isWritable(const QString &dir)
{
    if (!QDir().mkpath(dir)) return false;
    QFile probe(dir + QStringLiteral("/.write-probe-%1").arg(QCoreApplication::applicationPid()));
    if (!probe.open(QIODevice::WriteOnly)) return false;
    const bool ok = probe.write("ok", 2) == 2;
    probe.close();
    probe.remove();
    return ok;
}
} // namespace

QString DataRoot::installDir()
{
    if (QCoreApplication::instance()) return QCoreApplication::applicationDirPath();
    return QDir::currentPath();
}

QString DataRoot::electronUserDataDir()
{
    const QString app = QStringLiteral(CORV_APP_NAME);
#if defined(Q_OS_WIN)
    const QString base = qEnvironmentVariable("APPDATA",
        QDir::homePath() + QStringLiteral("/AppData/Roaming"));
#elif defined(Q_OS_MACOS)
    const QString base = QDir::homePath() + QStringLiteral("/Library/Application Support");
#else
    const QString base = qEnvironmentVariable("XDG_CONFIG_HOME", QDir::homePath() + QStringLiteral("/.config"));
#endif
    return QDir::cleanPath(base + QLatin1Char('/') + app);
}

QString DataRoot::root()
{
    QMutexLocker lock(&g_lock);
    if (!g_root.isEmpty()) return g_root;

    const QString forced = qEnvironmentVariable("CORV_DATA_ROOT");
    if (!forced.isEmpty()) {
        QDir().mkpath(forced);
        g_root = QDir::cleanPath(forced);
        g_fallback = false;
        return g_root;
    }

    const QString candidate = QDir::cleanPath(installDir() + QStringLiteral("/data"));
#if defined(Q_OS_MACOS)
    // Inside an .app the folder next to the executable belongs to the signed
    // bundle, and an update replaces the bundle whole: the per-user folder
    const bool bundled = installDir().contains(QLatin1String(".app/Contents/MacOS"));
#else
    const bool bundled = false;
#endif
    if (!bundled && isWritable(candidate)) {
        g_root = candidate;
        g_fallback = false;
    } else {
        g_root = QDir::cleanPath(electronUserDataDir() + QStringLiteral("/data"));
        g_fallback = true;
        QDir().mkpath(g_root);
    }
    return g_root;
}

bool DataRoot::isFallback()
{
    root();
    return g_fallback;
}

QString DataRoot::dir(const QString &name)
{
    const QString d = root() + QLatin1Char('/') + name;
    QDir().mkpath(d);
    return d;
}

void DataRoot::resetForTests()
{
    QMutexLocker lock(&g_lock);
    g_root.clear();
    g_fallback = false;
}

} // namespace corv
