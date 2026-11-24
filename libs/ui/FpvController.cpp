#include "ui/FpvController.h"

#include "core/DebugLog.h"
#include "core/Settings.h"
#include "video/FpvStream.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>

using namespace Qt::StringLiterals;

namespace corv {

FpvController *FpvController::s_instance = nullptr;

namespace {
QJsonObject settings()
{
    QJsonObject o = Settings::instance().json(u"fpv-settings"_s).toObject();
    if (o.isEmpty()) o = QJsonDocument::fromJson(Settings::instance().value(u"fpv-settings"_s).toString().toUtf8()).object();
    return o;
}
} // namespace

FpvController::FpvController(QObject *parent) : QObject(parent), m_stream(new FpvStream)
{
    s_instance = this;
    m_thread.setObjectName(u"video"_s);
    m_stream->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_stream, &QObject::deleteLater);
    connect(m_stream, &FpvStream::started, this, [this](const QString &description, int w, int h) {
        if (!m_connecting) return;   // stopped meanwhile
        m_connecting = false;
        m_connected = true;
        m_info = u"%1×%2 · %3"_s.arg(w).arg(h).arg(description);
        if (m_mode == 0) m_mode = 1;
        emit changed();
        showStatus(u"Stream connected"_s);
    });
    connect(m_stream, &FpvStream::ended, this, [this](bool wasStarted, const QString &error) {
        if (!m_connecting && !m_connected) return;
        m_connecting = m_connected = false;
        m_mode = 0;
        m_info.clear();
        emit changed();
        if (!wasStarted) showStatus(u"Connection failed: "_s + (error.isEmpty() ? u"closed by the camera"_s : error), true);
        else showStatus(error.isEmpty() ? u"Stream disconnected"_s : u"Stream disconnected: "_s + error, true);
    });
    m_thread.start();

    m_statusTimer.setSingleShot(true);
    m_statusTimer.setInterval(4000);
    connect(&m_statusTimer, &QTimer::timeout, this, [this] {
        m_status.clear();
        emit statusChanged();
    });
}

FpvController::~FpvController()
{
    QMetaObject::invokeMethod(m_stream, &FpvStream::stop, Qt::BlockingQueuedConnection);
    m_thread.quit();
    m_thread.wait();
    s_instance = nullptr;
}

bool FpvController::available() const { return VideoDecoder::available(); }

QString FpvController::url() const
{
    const QJsonObject o = settings();
    auto text = [&](const QString &key, const QString &def) {
        const QString v = o.value(key).toString().trimmed();
        return v.isEmpty() ? def : v;
    };
    const QString ip = text(u"ip"_s, u"192.168.144.25"_s);
    QString path = text(u"path"_s, u"/main.264"_s);
    if (!path.startsWith('/'_L1)) path.prepend('/'_L1);
    const int port = o.value(u"port"_s).toVariant().toInt();
    return u"rtsp://%1:%2%3"_s.arg(ip).arg(port > 0 && port < 65536 ? port : 8554).arg(path);
}

int FpvController::fps() const
{
    const int v = settings().value(u"fps"_s).toVariant().toInt();
    return v > 0 ? std::clamp(v, 1, 60) : 30;
}

QString FpvController::tip() const
{
    if (m_connecting) return u"FPV Camera — connecting, click to cancel"_s;
    if (m_mode == 0) return u"FPV Camera — click to start stream"_s;
    if (m_mode == 1) return u"FPV Camera active — click for AR overlay"_s;
    return u"AR Mode active — click to stop"_s;
}

void FpvController::click()
{
    if (m_connecting) {
        stop();
    } else if (m_mode == 0) {
        start();
    } else if (m_mode == 1) {
        m_mode = 2;
        emit changed();
        showStatus(u"AR overlay active"_s);
    } else {
        stop();
    }
}

void FpvController::start()
{
    if (!available()) {
        showStatus(u"FPV not available: this build has no video decoder"_s, true);
        return;
    }
    m_connecting = true;
    m_connected = false;
    emit changed();
    showStatus(u"Connecting to camera..."_s);
    QMetaObject::invokeMethod(m_stream, [s = m_stream, url = url(), fps = fps()] { s->start(url, fps); }, Qt::QueuedConnection);
}

void FpvController::stop()
{
    if (!m_connecting && !m_connected && m_mode == 0) return;
    QMetaObject::invokeMethod(m_stream, &FpvStream::stop, Qt::QueuedConnection);
    m_connecting = m_connected = false;
    m_mode = 0;
    m_info.clear();
    emit changed();
    showStatus(u"Stream stopped"_s);
}

void FpvController::showStatus(const QString &text, bool error)
{
    m_status = text;
    m_statusError = error;
    emit statusChanged();
    m_statusTimer.start();
}

} // namespace corv
