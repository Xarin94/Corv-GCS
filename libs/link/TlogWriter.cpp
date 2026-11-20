#include "link/TlogWriter.h"

#include "core/DebugLog.h"

#include <QDateTime>
#include <QDir>
#include <QtEndian>

using namespace Qt::StringLiterals;

namespace corv {

QString TlogWriter::start(const QString &dir)
{
    if (m_file) return m_path;
    QDir().mkpath(dir);
    const QString stamp = QDateTime::currentDateTime().toString(u"yyyy-MM-dd HH-mm-ss"_s);
    const QString path = dir + '/'_L1 + stamp + u".tlog"_s;
    auto f = std::make_unique<QFile>(path);
    if (!f->open(QIODevice::WriteOnly)) {
        LOG_E("tlog") << "cannot record to" << path << f->errorString();
        return {};
    }
    m_file = std::move(f);
    m_path = path;
    LOG_I("tlog") << "recording to" << path;
    return path;
}

QString TlogWriter::stop()
{
    if (!m_file) return {};
    m_file->close();
    m_file.reset();
    const QString p = m_path;
    m_path.clear();
    LOG_I("tlog") << "recording stopped:" << p;
    return p;
}

void TlogWriter::write(const QByteArray &frame, std::int64_t wallMs)
{
    if (!m_file) return;
    const quint64 us = quint64(wallMs) * 1000ULL;
    uchar ts[8];
    qToLittleEndian(us, ts);
    m_file->write(reinterpret_cast<const char *>(ts), 8);
    m_file->write(frame);
}

} // namespace corv
