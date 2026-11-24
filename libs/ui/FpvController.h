// FpvController.h - the FPV camera button and its view ("Fpv" QML singleton; port
// of FPVController.js).
//
// The button cycles three states: the stream full screen (CAMERA), the 3D view
// blended over it (AR), off. The stream itself is an FpvStream on the video
// thread; the address, port, path and frame rate are those of SYS CONFIG -> SIYI
// CAMERA STREAM (fpv-settings).
#pragma once

#include <QObject>
#include <QQmlEngine>
#include <QThread>
#include <QTimer>

namespace corv {

class FpvStream;

class FpvController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Fpv)
    QML_SINGLETON
    /// 0 off, 1 camera, 2 AR
    Q_PROPERTY(int mode READ mode NOTIFY changed)
    Q_PROPERTY(bool active READ active NOTIFY changed)
    Q_PROPERTY(bool connecting READ connecting NOTIFY changed)
    Q_PROPERTY(bool connected READ connected NOTIFY changed)
    /// What the button does next, for its tooltip
    Q_PROPERTY(QString tip READ tip NOTIFY changed)
    /// "1920×1080 · H264 · VAAPI" while streaming
    Q_PROPERTY(QString streamInfo READ streamInfo NOTIFY changed)
    /// The line in the middle of the view; empty after 4 s
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(bool statusError READ statusError NOTIFY statusChanged)
    /// The stream address the settings give (SYS CONFIG)
    Q_PROPERTY(QString url READ url NOTIFY urlChanged)
    Q_PROPERTY(bool available READ available CONSTANT)

public:
    explicit FpvController(QObject *parent);
    ~FpvController() override;
    static FpvController *s_instance;
    static FpvController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    int mode() const { return m_mode; }
    bool active() const { return m_mode != 0; }
    bool connecting() const { return m_connecting; }
    bool connected() const { return m_connected; }
    QString tip() const;
    QString streamInfo() const { return m_info; }
    QString status() const { return m_status; }
    bool statusError() const { return m_statusError; }
    QString url() const;
    bool available() const;

    /// The header's FPV button.
    Q_INVOKABLE void click();
    Q_INVOKABLE void stop();
    /// fpv-settings changed in SYS CONFIG.
    void settingsChanged() { emit urlChanged(); }

signals:
    void changed();
    void statusChanged();
    void urlChanged();

private:
    void start();
    void showStatus(const QString &text, bool error = false);
    int fps() const;

    QThread m_thread;
    FpvStream *m_stream;
    int m_mode = 0;
    bool m_connecting = false, m_connected = false;
    QString m_info, m_status;
    bool m_statusError = false;
    QTimer m_statusTimer;
};

} // namespace corv
