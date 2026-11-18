#include "core/DebugLog.h"

#include "core/DataRoot.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QHash>
#include <QMutex>
#include <QSaveFile>
#include <QSysInfo>
#include <QThread>
#include <QThreadPool>
#include <QTimer>

#include <cstdio>
#include <deque>
#include <map>

namespace corv {

namespace {

constexpr qint64 kWindowMs = 5 * 60 * 1000;
constexpr qint64 kMaxBytes = 3 * 1024 * 1024;
constexpr int kFlushMs = 3000;
constexpr int kFlushSoonMs = 400;
constexpr int kMaxLinesPerSec = 200;
constexpr int kMaxMsg = 1500;
constexpr int kMaxErrMsg = 6000;
constexpr int kOldIssues = 40;
const char *kFileName = "corv-gcs-debug.log";
const char *kPrevName = "corv-gcs-debug.prev.log";

struct Entry {
    qint64 t = 0, tLast = 0;
    char lvl = 'I';
    QString src, msg;
    int n = 1;
    qint64 bytes = 0;
};

struct State {
    QMutex lock;
    std::deque<Entry> entries;
    qint64 bytes = 0;
    bool dirty = false;
    bool started = false;
    bool writing = false;
    qint64 sessionStart = QDateTime::currentMSecsSinceEpoch();
    qint64 floodSecond = 0;
    int floodCount = 0, floodDropped = 0, droppedTotal = 0, trimmedBySize = 0;
    std::deque<QString> oldIssues;
    std::map<QString, QString> header;   // ordered like insertion is not needed; keys sort stably
    QList<QString> headerOrder;
    QHash<QString, QPair<qint64, int>> throttles;
    QString dir;
    QTimer *timer = nullptr;
    QtMessageHandler previous = nullptr;
};

State &S()
{
    static State s;
    return s;
}

QString pad(qint64 n, int w = 2) { return QString::number(n).rightJustified(w, QLatin1Char('0')); }

QString clockStr(qint64 t)
{
    const QDateTime d = QDateTime::fromMSecsSinceEpoch(t);
    return d.toString(QStringLiteral("HH:mm:ss.zzz"));
}

QString stampStr(qint64 t)
{
    return QDateTime::fromMSecsSinceEpoch(t).toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"));
}

QString mb(qint64 b) { return QString::number(b / 1048576.0, 'f', 1); }

QString fmtDuration(qint64 ms)
{
    const qint64 s = ms / 1000, h = s / 3600, m = (s % 3600) / 60;
    return h ? QStringLiteral("%1h%2m").arg(h).arg(pad(m)) : QStringLiteral("%1m%2s").arg(m).arg(pad(s % 60));
}

QString render(const Entry &e)
{
    QString msg = e.msg;
    msg.replace(QLatin1Char('\n'), QStringLiteral("\n                    "));
    QString s = QStringLiteral("%1 %2 %3 %4").arg(clockStr(e.t), QString(QLatin1Char(e.lvl)), e.src.leftJustified(4), msg);
    if (e.n > 1) s += QStringLiteral("  [x%1, last %2]").arg(e.n).arg(clockStr(e.tLast));
    return s;
}

// Called with the lock held
void trimLocked(qint64 now)
{
    State &s = S();
    const qint64 cutoff = now - kWindowMs;
    while (!s.entries.empty()) {
        const Entry &e = s.entries.front();
        const bool old = e.tLast < cutoff;
        const bool tooBig = s.bytes > kMaxBytes;
        if (!old && !tooBig) break;
        if (!old) ++s.trimmedBySize;
        if (e.lvl == 'E' || e.lvl == 'W') {
            s.oldIssues.push_back(render(e));
            if (s.oldIssues.size() > std::size_t(kOldIssues)) s.oldIssues.pop_front();
        }
        s.bytes -= e.bytes;
        s.entries.pop_front();
    }
}

QString buildTextLocked()
{
    State &s = S();
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    trimLocked(now);
    const qint64 oldest = s.entries.empty() ? now : s.entries.front().t;
    QStringList out;
    out << QStringLiteral("==== CORV GCS debug log ====");
    out << QStringLiteral("Written     %1  (session started %2, up %3)")
               .arg(stampStr(now), stampStr(s.sessionStart), fmtDuration(now - s.sessionStart));
    QString covers = QStringLiteral("Covers      last %1 min: %2 lines from %3, %4 MB")
                         .arg(kWindowMs / 60000).arg(s.entries.size()).arg(clockStr(oldest), mb(s.bytes));
    if (s.trimmedBySize) covers += QStringLiteral("; %1 lines cut early by the %2 MB cap").arg(s.trimmedBySize).arg(mb(kMaxBytes));
    if (s.droppedTotal) covers += QStringLiteral("; %1 dropped by the flood guard").arg(s.droppedTotal);
    out << covers;
    for (const QString &k : s.headerOrder) {
        QString v = s.header[k];
        v.replace(QLatin1Char('\n'), QStringLiteral("\n            "));
        out << k.leftJustified(11) + QLatin1Char(' ') + v;
    }
    if (!s.oldIssues.empty()) {
        out << QString();
        out << QStringLiteral("---- Errors and warnings older than the window (last %1) ----").arg(kOldIssues);
        for (const QString &l : s.oldIssues) out << l;
    }
    out << QString();
    out << QStringLiteral("---- Log (oldest first; E error, W warning, I info, D debug; main = app, qt = Qt) ----");
    for (const Entry &e : s.entries) out << render(e);
    out << QString();
    return out.join(QLatin1Char('\n'));
}

void writeFile(const QString &path, const QString &text)
{
    QSaveFile f(path);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(text.toUtf8());
        if (f.commit()) return;
    }
    // Windows refuses the rename while another program holds the file open
    QFile plain(path);
    if (plain.open(QIODevice::WriteOnly | QIODevice::Truncate)) plain.write(text.toUtf8());
}

void flushAsync()
{
    State &s = S();
    QString text, path;
    {
        QMutexLocker lock(&s.lock);
        if (!s.dirty || s.writing || !s.started) return;
        s.dirty = false;
        s.writing = true;
        text = buildTextLocked();
        path = s.dir + QLatin1Char('/') + QLatin1String(kFileName);
    }
    QThreadPool::globalInstance()->start([text, path] {
        writeFile(path, text);
        QMutexLocker lock(&S().lock);
        S().writing = false;
    });
}

void messageHandler(QtMsgType type, const QMessageLogContext &ctx, const QString &msg)
{
    DebugLog::Level lvl = DebugLog::Info;
    switch (type) {
    case QtDebugMsg: lvl = DebugLog::Debug; break;
    case QtInfoMsg: lvl = DebugLog::Info; break;
    case QtWarningMsg: lvl = DebugLog::Warning; break;
    case QtCriticalMsg:
    case QtFatalMsg: lvl = DebugLog::Error; break;
    }
    const bool fromQt = ctx.category && qstrncmp(ctx.category, "qt.", 3) == 0;
    DebugLog::write(lvl, fromQt ? QStringLiteral("qt") : QStringLiteral("main"), msg);
    if (type == QtFatalMsg) DebugLog::flushSync();
}

} // namespace

void DebugLog::init()
{
    State &s = S();
    {
        QMutexLocker lock(&s.lock);
        if (s.started) return;
        s.dir = DataRoot::debugDir();
        const QString file = s.dir + QLatin1Char('/') + QLatin1String(kFileName);
        const QString prev = s.dir + QLatin1Char('/') + QLatin1String(kPrevName);
        if (QFile::exists(file)) {
            QFile::remove(prev);
            QFile::rename(file, prev);
        }
        s.started = true;
        s.dirty = true;
    }
    s.previous = qInstallMessageHandler(messageHandler);
    setHeader(QStringLiteral("App"), QStringLiteral("%1 %2 (Qt %3, %4)")
                                         .arg(QStringLiteral(CORV_APP_NAME), QStringLiteral(CORV_VERSION),
                                              QString::fromLatin1(qVersion()), QSysInfo::buildAbi()));
    setHeader(QStringLiteral("System"), QStringLiteral("%1, %2 cores")
                                            .arg(QSysInfo::prettyProductName())
                                            .arg(QThread::idealThreadCount()));
    setHeader(QStringLiteral("Data root"), DataRoot::root() + (DataRoot::isFallback() ? QStringLiteral(" (per-user fallback)") : QString()));
    s.timer = new QTimer(QCoreApplication::instance());
    s.timer->setInterval(kFlushMs);
    QObject::connect(s.timer, &QTimer::timeout, [] { flushAsync(); });
    s.timer->start();
}

void DebugLog::shutdown()
{
    flushSync();
    qInstallMessageHandler(S().previous);
}

void DebugLog::write(Level lvl, const QString &src, const QString &msgIn)
{
    // Console copy, as the Electron main process printed its console
    {
        const QByteArray line = (QString(QLatin1Char(char(lvl))) + QLatin1Char(' ') + msgIn).toLocal8Bit();
        std::fprintf(stderr, "%s\n", line.constData());
    }
    State &s = S();
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    QString msg = msgIn;
    const int cap = lvl == Error ? kMaxErrMsg : kMaxMsg;
    if (msg.size() > cap) msg = msg.left(cap) + QStringLiteral(" …[+%1 chars]").arg(msgIn.size() - cap);

    bool soon = false;
    {
        QMutexLocker lock(&s.lock);
        if (!s.entries.empty()) {
            Entry &last = s.entries.back();
            if (last.msg == msg && last.lvl == lvl && last.src == src) {
                ++last.n;
                last.tLast = now;
                s.dirty = true;
                return;
            }
        }
        const qint64 sec = now / 1000;
        if (sec != s.floodSecond) {
            if (s.floodDropped) {
                Entry e;
                e.t = e.tLast = now;
                e.lvl = 'W';
                e.src = QStringLiteral("log");
                e.msg = QStringLiteral("[log] %1 lines dropped in one second (flood guard, %2/s)").arg(s.floodDropped).arg(kMaxLinesPerSec);
                e.bytes = e.msg.size() + 24;
                s.bytes += e.bytes;
                s.entries.push_back(e);
                s.floodDropped = 0;
            }
            s.floodSecond = sec;
            s.floodCount = 0;
        }
        if (++s.floodCount > kMaxLinesPerSec && lvl != Error) {
            ++s.floodDropped;
            ++s.droppedTotal;
            return;
        }
        Entry e;
        e.t = e.tLast = now;
        e.lvl = char(lvl);
        e.src = src;
        e.msg = msg;
        e.bytes = msg.size() + 24;
        s.bytes += e.bytes;
        s.entries.push_back(std::move(e));
        s.dirty = true;
        if (s.bytes > kMaxBytes * 5 / 4) trimLocked(now);
        soon = lvl == Error && s.started;
    }
    if (soon && QCoreApplication::instance())
        QMetaObject::invokeMethod(QCoreApplication::instance(), [] {
            QTimer::singleShot(kFlushSoonMs, [] { flushAsync(); });
        }, Qt::QueuedConnection);
}

void DebugLog::setHeader(const QString &keyIn, const QString &value)
{
    State &s = S();
    const QString key = keyIn.left(24);
    QMutexLocker lock(&s.lock);
    if (value.isEmpty()) {
        s.header.erase(key);
        s.headerOrder.removeAll(key);
    } else {
        if (!s.header.count(key)) s.headerOrder << key;
        s.header[key] = value.left(3000);
    }
    s.dirty = true;
}

int DebugLog::throttle(const QString &key, int ms)
{
    State &s = S();
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    QMutexLocker lock(&s.lock);
    auto it = s.throttles.find(key);
    if (it != s.throttles.end() && now - it->first < ms) {
        ++it->second;
        return -1;
    }
    const int skipped = it != s.throttles.end() ? it->second : 0;
    s.throttles.insert(key, {now, 0});
    return skipped;
}

QString DebugLog::filePath()
{
    return DataRoot::debugDir() + QLatin1Char('/') + QLatin1String(kFileName);
}

void DebugLog::flushSync()
{
    State &s = S();
    QString text, path;
    {
        QMutexLocker lock(&s.lock);
        if (!s.started) return;
        text = buildTextLocked();
        path = s.dir + QLatin1Char('/') + QLatin1String(kFileName);
        s.dirty = false;
    }
    writeFile(path, text);
}

QString DebugLog::buildText()
{
    QMutexLocker lock(&S().lock);
    return buildTextLocked();
}

} // namespace corv
