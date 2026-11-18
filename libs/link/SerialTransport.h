#pragma once

#include "link/Transport.h"

#include <QList>
#include <QSerialPort>

namespace corv {

struct SerialPortInfo {
    QString path, manufacturer, friendlyName, vendorId, productId, serialNumber;
};

/// Serial port. DTR is kept low, also on close: on the esp32s31corv USB
/// closing with it high silences the port (1.8.8 fix).
class SerialTransport : public Transport
{
    Q_OBJECT
public:
    SerialTransport(QString port, int baud, QString label, QObject *parent = nullptr);
    ~SerialTransport() override;

    bool open(QString *error) override;
    void close() override;
    qint64 write(const QByteArray &bytes) override;
    QString describe() const override;
    QString kind() const override { return m_label; }

    static QList<SerialPortInfo> listPorts();

private:
    QSerialPort *m_port = nullptr;
    QString m_name;
    int m_baud;
    QString m_label;
    bool m_closing = false;
};

} // namespace corv
