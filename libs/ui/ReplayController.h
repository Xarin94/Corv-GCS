// ReplayController.h - LOG REPLAY ("Replay" QML singleton): the port of
// js/logging/LogReplayController.js.
//
//   - the sidebar LOG REPLAY section: OPEN LOG FILE, the file's name, format,
//     duration and messages, UNLOAD; shown only while no live link is up
//   - the timeline over the bottom-right corner: play / pause, scrubber,
//     current and total time, ×
//   - a live connection unloads the replay; the full-flight track is drawn on
//     the mini-map from the moment the log is loaded and stays through play,
//     pause and seek
//
// The engine (LogReplay) runs on the log thread: every call here is queued to
// it, every answer comes back as a queued signal. The spectra it computes from
// a .bin go to the FFT bar (FftController). GUI thread.
#pragma once

#include "services/LogReplay.h"

#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QUrl>

namespace corv {

class FftController;
class SysConfig;

class ReplayController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Replay)
    QML_SINGLETON
    Q_PROPERTY(bool loaded READ loaded NOTIFY stateChanged)
    Q_PROPERTY(bool loading READ loading NOTIFY stateChanged)
    Q_PROPERTY(bool playing READ playing NOTIFY tickChanged)
    Q_PROPERTY(QString fileName READ fileName NOTIFY stateChanged)
    Q_PROPERTY(QString format READ format NOTIFY stateChanged)
    Q_PROPERTY(QString durationText READ durationText NOTIFY stateChanged)
    Q_PROPERTY(int messages READ messages NOTIFY stateChanged)
    Q_PROPERTY(double totalMs READ totalMs NOTIFY tickChanged)
    Q_PROPERTY(double currentMs READ currentMs NOTIFY tickChanged)
    Q_PROPERTY(QString currentText READ currentText NOTIFY tickChanged)
    Q_PROPERTY(QString totalText READ totalText NOTIFY tickChanged)
    Q_PROPERTY(double fraction READ fraction WRITE scrub NOTIFY tickChanged)
    Q_PROPERTY(bool sectionVisible READ sectionVisible NOTIFY linkChanged)
    Q_PROPERTY(bool openEnabled READ openEnabled NOTIFY linkChanged)
    Q_PROPERTY(QUrl logsFolder READ logsFolder NOTIFY folderChanged)

public:
    ReplayController(LogReplay *engine, FftController *fft, QObject *parent);
    static ReplayController *s_instance;
    static ReplayController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    bool loaded() const { return m_loaded; }
    bool loading() const { return m_loading; }
    bool playing() const { return m_playing; }
    QString fileName() const { return m_fileName; }
    QString format() const { return m_format.toUpper(); }
    QString durationText() const { return formatMs(m_totalMs); }
    int messages() const { return m_messages; }
    double totalMs() const { return m_totalMs; }
    double currentMs() const { return m_currentMs; }
    QString currentText() const { return formatMs(m_currentMs); }
    QString totalText() const { return formatMs(m_totalMs); }
    double fraction() const { return m_totalMs > 0 ? std::clamp(m_currentMs / m_totalMs, 0.0, 1.0) : 0; }
    bool sectionVisible() const { return !m_live; }
    bool openEnabled() const { return !m_live && !m_connecting && !m_loading; }
    QUrl logsFolder() const;

    Q_INVOKABLE void open(const QUrl &file);
    /// Tests: a path.
    Q_INVOKABLE void openPath(const QString &path);
    Q_INVOKABLE void unload();
    Q_INVOKABLE void play();
    Q_INVOKABLE void pause();
    Q_INVOKABLE void playPause();
    /// The scrubber: a fraction of the log, debounced 80 ms.
    Q_INVOKABLE void scrub(double fraction);

    /// The link state of GcsController: CONNECTED unloads the replay.
    void setConnectionState(const QString &state);
    /// SYS CONFIG → FFT FROM LOGS (settings key log-fft-cfg).
    void applyFftConfig();

    static QString formatMs(double ms);

signals:
    void stateChanged();
    void tickChanged();
    void linkChanged();
    void folderChanged();

private:
    void seekNow();
    LogReplay *m_engine;
    FftController *m_fft;
    bool m_loaded = false, m_loading = false, m_playing = false;
    bool m_live = false, m_connecting = false;
    bool m_scrubbing = false;
    QString m_fileName, m_format;
    int m_messages = 0;
    double m_totalMs = 0, m_currentMs = 0;
    double m_scrubTarget = 0;
    QTimer m_seekDebounce;
};

} // namespace corv
