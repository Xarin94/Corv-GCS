#include "link/SerialTransport.h"

#include <QSerialPortInfo>

using namespace Qt::StringLiterals;

namespace corv {

SerialTransport::SerialTransport(QString port, int baud, QString label, QObject *parent)
    : Transport(parent), m_name(std::move(port)), m_baud(baud), m_label(std::move(label))
{
}

SerialTransport::~SerialTransport() { close(); }

bool SerialTransport::open(QString *error)
{
    m_port = new QSerialPort(this);
    m_port->setPortName(m_name);
    m_port->setBaudRate(m_baud);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    m_port->setFlowControl(QSerialPort::NoFlowControl);
    // QSerialPort::close() puts back the settings the port had before open(): on
    // Windows that is DTR_CONTROL_ENABLE, so DTR went high just before the handle
    // closed, the very thing that silences the esp32s31corv USB until it is
    // replugged (the next connection then saw no data). Electron (node-serialport,
    // hupcl false) opened with DTR and RTS disabled and restored nothing.
    m_port->setSettingsRestoredOnClose(false);
    if (!m_port->open(QIODevice::ReadWrite)) {
        if (error) *error = m_port->errorString();
        delete m_port;
        m_port = nullptr;
        return false;
    }
    m_port->setDataTerminalReady(false);
    m_port->setRequestToSend(false);
    // Bytes the driver held from before the connection are not this link's
    m_port->clear(QSerialPort::AllDirections);
    connect(m_port, &QSerialPort::readyRead, this, [this] { emit received(m_port->readAll()); });
    connect(m_port, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e) {
        if (e == QSerialPort::NoError || m_closing) return;
        if (e == QSerialPort::ResourceError) emit closed(u"device unplugged"_s);
        else if (e != QSerialPort::TimeoutError) emit closed(u"serial error: %1"_s.arg(m_port->errorString()));
    });
    return true;
}

void SerialTransport::close()
{
    if (!m_port) return;
    m_closing = true;
    if (m_port->isOpen()) {
        // DTR low to the end (settings are not restored on close, see open())
        m_port->setDataTerminalReady(false);
        m_port->setRequestToSend(false);
        m_port->close();
    }
    m_port->deleteLater();
    m_port = nullptr;
}

qint64 SerialTransport::write(const QByteArray &bytes)
{
    return m_port && m_port->isOpen() ? m_port->write(bytes) : -1;
}

QString SerialTransport::describe() const
{
    return u"%1 serial %2 @ %3"_s.arg(m_label == "MSP"_L1 ? u"MSP"_s : u"MAVLink"_s, m_name).arg(m_baud);
}

QList<SerialPortInfo> SerialTransport::listPorts()
{
    QList<SerialPortInfo> out;
    for (const QSerialPortInfo &p : QSerialPortInfo::availablePorts()) {
        SerialPortInfo i;
        i.path = p.portName();
        i.manufacturer = p.manufacturer();
        i.friendlyName = p.description();
        i.vendorId = p.hasVendorIdentifier() ? QString::number(p.vendorIdentifier(), 16).rightJustified(4, '0'_L1) : QString();
        i.productId = p.hasProductIdentifier() ? QString::number(p.productIdentifier(), 16).rightJustified(4, '0'_L1) : QString();
        i.serialNumber = p.serialNumber();
        out << i;
    }
    return out;
}

} // namespace corv
