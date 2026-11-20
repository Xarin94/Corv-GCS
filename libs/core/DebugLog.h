// DebugLog.h - rolling debug log: the last five minutes of the app, on disk
// (port of app-log.js).
//
// Every Qt message (qDebug/qInfo/qWarning/qCritical) and every LOG_x line goes
// into an in-memory window. Every 3 s the whole window is written over
//     <data root>/debug/corv-gcs-debug.log
// through a temp file + rename. Lines older than 5 minutes fall off the top;
// past 3 MB the oldest go even inside the window. At startup the previous
// session's file becomes corv-gcs-debug.prev.log. The file opens with an
// always-current header (setHeader) and the session's last errors older than
// the window. Identical consecutive lines collapse, a flood guard caps the
// lines per second. Same text layout as the Electron version, so the two
// versions' logs read the same way.
//
//     LOG_I("mavlink") << "connected to" << port;     // "[mavlink] connected to COM5"
//
// Thread-safe: any thread may write.
#pragma once

#include <QDebug>
#include <QString>

namespace corv {

class DebugLog
{
public:
    enum Level : char { Debug = 'D', Info = 'I', Warning = 'W', Error = 'E' };

    /// Start: install the Qt message handler, rotate the previous file, start the flush timer.
    /// Call from the GUI thread after the QCoreApplication exists.
    static void init();
    static void shutdown();

    static void write(Level lvl, const QString &src, const QString &msg);
    /// A header line, always current; an empty value removes it.
    static void setHeader(const QString &key, const QString &value);
    /// -1 when this call must stay quiet, else how many were kept quiet since the last one.
    static int throttle(const QString &key, int ms);

    static QString filePath();
    /// Synchronous write (crash and quit paths).
    static void flushSync();
    /// Text of the file as it would be written now (tests).
};

/// One log line built with QDebug's << and written when it goes out of scope.
class LogLine
{
public:
    LogLine(DebugLog::Level lvl, const char *tag) : m_lvl(lvl), m_dbg(&m_buf)
    {
        m_dbg.noquote().nospace() << '[' << tag << "] ";
        m_dbg.space();
    }
    ~LogLine() { DebugLog::write(m_lvl, QStringLiteral("main"), m_buf.trimmed()); }
    template <typename T>
    LogLine &operator<<(const T &v)
    {
        m_dbg << v;
        return *this;
    }

private:
    DebugLog::Level m_lvl;
    QString m_buf;
    QDebug m_dbg;
};

} // namespace corv

#define LOG_D(tag) ::corv::LogLine(::corv::DebugLog::Debug, tag)
#define LOG_I(tag) ::corv::LogLine(::corv::DebugLog::Info, tag)
#define LOG_W(tag) ::corv::LogLine(::corv::DebugLog::Warning, tag)
#define LOG_E(tag) ::corv::LogLine(::corv::DebugLog::Error, tag)
