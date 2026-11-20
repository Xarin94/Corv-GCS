#include "core/DataRoot.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QStandardPaths>
#include <QtGlobal>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
QMutex g_lock;
QString g_root;
bool g_fallback = false;

bool isWritable(const QString &dir)
{
    if (!QDir().mkpath(dir)) return false;
    QFile probe(dir + u"/.write-probe-%1"_s.arg(QCoreApplication::applicationPid()));
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
        QDir::homePath() + u"/AppData/Roaming"_s);
#elif defined(Q_OS_MACOS)
    const QString base = QDir::homePath() + u"/Library/Application Support"_s;
#else
    const QString base = qEnvironmentVariable("XDG_CONFIG_HOME", QDir::homePath() + u"/.config"_s);
#endif
    return QDir::cleanPath(base + '/'_L1 + app);
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

    const QString candidate = QDir::cleanPath(installDir() + u"/data"_s);
#if defined(Q_OS_MACOS)
    // Inside an .app the folder next to the executable belongs to the signed
    // bundle, and an update replaces the bundle whole: the per-user folder
    const bool bundled = installDir().contains(".app/Contents/MacOS"_L1);
#else
    const bool bundled = false;
#endif
    if (!bundled && isWritable(candidate)) {
        g_root = candidate;
        g_fallback = false;
    } else {
        g_root = QDir::cleanPath(electronUserDataDir() + u"/data"_s);
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
    const QString d = root() + '/'_L1 + name;
    QDir().mkpath(d);
    return d;
}

} // namespace corv
