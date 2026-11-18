// TlogWriter.h - .tlog flight recording, same format as the Electron version:
// every record is [uint64 little-endian microseconds since the Unix epoch][raw MAVLink frame].
// Files are named "yyyy-MM-dd HH-mm-ss.tlog" under <data root>/logs.
#pragma once

#include <QByteArray>
#include <QFile>
#include <QString>
#include <memory>

namespace corv {

class TlogWriter
{
public:
    /// Start a new file in dir (or keep the open one). Returns the path, empty on failure.
    QString start(const QString &dir);
    /// Close; returns the path that was being written.
    QString stop();
    bool isOpen() const { return m_file != nullptr; }
    QString path() const { return m_path; }

    void write(const QByteArray &frame, std::int64_t wallMs);

private:
    std::unique_ptr<QFile> m_file;
    QString m_path;
};

} // namespace corv
