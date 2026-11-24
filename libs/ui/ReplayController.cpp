#include "ui/ReplayController.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "ui/DialogHub.h"
#include "ui/FftController.h"
#include "ui/MapItem.h"

#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

ReplayController *ReplayController::s_instance = nullptr;

namespace {
const char *const kFolderKey = "dialog-dir.log-replay";

/// log-fft-cfg as an object, whether stored as JSON or as the string localStorage kept
QJsonObject fftConfig()
{
    const QJsonValue v = Settings::instance().json(u"log-fft-cfg"_s);
    if (v.isObject()) return v.toObject();
    return QJsonDocument::fromJson(v.toString().toUtf8()).object();
}
}

ReplayController::ReplayController(LogReplay *engine, FftController *fft, QObject *parent) : QObject(parent), m_engine(engine), m_fft(fft)
{
    s_instance = this;
    connect(m_engine, &LogReplay::stateChanged, this, [this](const QString &state, const QString &fileName, const QString &format, double totalMs, int total) {
        const bool on = state == "LOADED"_L1 || state == "PLAYING"_L1 || state == "PAUSED"_L1;
        m_loading = false;
        if (on) {
            m_loaded = true;
            m_playing = state == "PLAYING"_L1;
            m_fileName = fileName;
            m_format = format;
            m_totalMs = totalMs;
            m_messages = total;
        } else {
            m_loaded = m_playing = false;
            m_fileName.clear();
            m_format.clear();
            m_totalMs = m_currentMs = 0;
            m_messages = 0;
            // The live trail resumes
            MapItem::setFrozenTrail({});
        }
        if (m_fft) m_fft->replayState(state, format);
        emit stateChanged();
        emit tickChanged();
        emit linkChanged();
    });
    connect(m_engine, &LogReplay::loaded, this, [](const TrackList &track) {
        // The whole recorded flight as a static trail on the mini-map
        std::vector<LatLon> pts;
        pts.reserve(std::size_t(track.size()));
        for (const TrackPoint &p : track) pts.push_back({p.lat, p.lon});
        MapItem::setFrozenTrail(pts);
    });
    connect(m_engine, &LogReplay::loadFailed, this, [this](const QString &e) {
        m_loading = false;
        emit stateChanged();
        emit linkChanged();
        DialogHub::toast(u"Failed to load log: %1"_s.arg(e));
    });
    connect(m_engine, &LogReplay::tick, this, [this](double currentMs, double totalMs, bool playing) {
        m_playing = playing;
        m_totalMs = totalMs;
        if (!m_scrubbing) m_currentMs = currentMs;
        if (m_fft) m_fft->replayTick(currentMs, playing);
        emit tickChanged();
    });
    connect(m_engine, &LogReplay::resetState, this, [this] {
        if (m_fft) m_fft->replaySeek();
    });
    connect(m_engine, &LogReplay::spectrum, this, [this](const LogSpectrumMsg &m) {
        if (m_fft) m_fft->onLogSpectrum(m);
    });
    connect(m_engine, &LogReplay::spectrumConfigApplied, this, [](int n, int avg, int stepMs) {
        // The engine keeps only values off its lists: show what it uses
        const QJsonObject cur = fftConfig();
        if (cur.value(u"n"_s).toInt(1024) != n || cur.value(u"avg"_s).toInt(1) != avg
            || cur.value(u"stepMs"_s).toInt(250) != stepMs)
            Settings::instance().setJson(u"log-fft-cfg"_s, QJsonObject{{u"n"_s, n}, {u"avg"_s, avg}, {u"stepMs"_s, stepMs}});
    });
    if (m_fft)
        connect(m_fft, &FftController::streamSelected, this, [this](int sensor, const QString &key) {
            QMetaObject::invokeMethod(m_engine, [e = m_engine, sensor, key] { e->selectSpectrumStream(sensor, key); });
        });

    m_seekDebounce.setSingleShot(true);
    m_seekDebounce.setInterval(80);
    connect(&m_seekDebounce, &QTimer::timeout, this, &ReplayController::seekNow);
    applyFftConfig();
}

QString ReplayController::formatMs(double ms)
{
    if (!std::isfinite(ms) || ms < 0) ms = 0;
    const qint64 s = qint64(ms / 1000);
    return u"%1:%2"_s.arg(s / 60, 2, 10, '0'_L1).arg(s % 60, 2, 10, '0'_L1);
}

QUrl ReplayController::logsFolder() const
{
    // The GCS's own tlog folder until another folder has been used
    const QString last = Settings::instance().value(QLatin1String(kFolderKey)).toString();
    return QUrl::fromLocalFile(!last.isEmpty() && QDir(last).exists() ? last : DataRoot::logsDir());
}

void ReplayController::open(const QUrl &file) { openPath(file.toLocalFile()); }

void ReplayController::openPath(const QString &path)
{
    if (m_live || path.isEmpty()) return;
    Settings::instance().setValue(QLatin1String(kFolderKey), QFileInfo(path).absolutePath());
    emit folderChanged();
    m_loading = true;
    emit stateChanged();
    emit linkChanged();
    LOG_I("replay") << "opening" << path;
    QMetaObject::invokeMethod(m_engine, [e = m_engine, path] { e->load(path); });
}

void ReplayController::unload() { QMetaObject::invokeMethod(m_engine, [e = m_engine] { e->unload(); }); }
void ReplayController::seekNow()
{
    m_seekDebounce.stop();
    const double target = m_scrubTarget;
    QMetaObject::invokeMethod(m_engine, [e = m_engine, target] { e->seek(target); });
    m_scrubbing = false;
}

void ReplayController::play()
{
    // From where the timeline was just dragged to, not from where the log was
    if (m_seekDebounce.isActive()) seekNow();
    QMetaObject::invokeMethod(m_engine, [e = m_engine] { e->play(); });
}

void ReplayController::pause() { QMetaObject::invokeMethod(m_engine, [e = m_engine] { e->pause(); }); }

void ReplayController::playPause()
{
    if (!m_loaded) return;
    if (m_playing) pause();
    else play();
}

void ReplayController::scrub(double fraction)
{
    if (!m_loaded) return;
    m_scrubbing = true;
    m_scrubTarget = std::round(m_totalMs * std::clamp(fraction, 0.0, 1.0));
    m_currentMs = m_scrubTarget;
    emit tickChanged();
    m_seekDebounce.start();
}

void ReplayController::setConnectionState(const QString &state)
{
    const bool live = state == "CONNECTED"_L1;
    const bool connecting = state == "CONNECTING"_L1;
    if (live && m_loaded) unload();   // a live link drops the replay
    if (live == m_live && connecting == m_connecting) return;
    m_live = live;
    m_connecting = connecting;
    emit linkChanged();
}

void ReplayController::applyFftConfig()
{
    const QJsonObject o = fftConfig();
    const int n = o.value(u"n"_s).toInt(1024), avg = o.value(u"avg"_s).toInt(1), step = o.value(u"stepMs"_s).toInt(250);
    QMetaObject::invokeMethod(m_engine, [e = m_engine, n, avg, step] { e->setSpectrumConfig(n, avg, step); });
}

} // namespace corv
