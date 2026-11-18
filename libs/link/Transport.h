// Transport.h - a byte pipe to the vehicle. Lives on the link thread.
#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>

namespace corv {

class Transport : public QObject
{
    Q_OBJECT
public:
    using QObject::QObject;

    /// Open synchronously (the link thread may block for a TCP retry). False with *error set on failure.
    virtual bool open(QString *error) = 0;
    virtual void close() = 0;
    virtual qint64 write(const QByteArray &bytes) = 0;
    /// False while there is nowhere to send (UDP before the first packet arrived).
    virtual bool canSend() const { return true; }
    /// "MAVLink serial COM5 @ 57600" style, for the debug log.
    virtual QString describe() const = 0;
    /// "SERIAL", "UDP", "TCP", "LTE": the link type shown in the status bar.
    virtual QString kind() const = 0;

signals:
    void received(const QByteArray &bytes);
    /// The far side or the OS closed the pipe; reason for the log.
    void closed(const QString &reason);
};

} // namespace corv
