// LogReplay.h - LOG REPLAY engine (port of log-replay-manager.js), on its own
// "log" thread.
//
// Indexes a .tlog (memory-mapped, offsets only) or a .bin (synthetic MAVLink
// frames, LogIndex.h) and plays its frames into the link's normal pipeline
// (LinkManager::injectReplay), so StateMapper, the HUD, the map and every panel
// are animated without knowing the source is a file. VehicleHub is put in the
// REPLAY mode, where it takes only replay frames.
//
// Clock: every 50 ms the frames whose time has come go out (at most 200 a tick);
// every 100 ms a tick tells the timeline where the cursor is. A seek resets the
// vehicle state, re-sends the last of the "sticky" messages before the cursor
// (mode, home, battery, GPS...) and restarts the log spectra 60 s back. A .bin's
// IMU samples feed LogSpectrum, pumped every 50 ms on this thread.
//
// Everything is invoked through queued calls from the GUI (ReplayController).
#pragma once

#include "services/LogIndex.h"
#include "vehicle/SpectrumMonitor.h"

#include <QFile>
#include <QList>
#include <QMetaType>
#include <QObject>
#include <QTimer>
#include <memory>

namespace corv {

class LinkManager;
class MavlinkParser;
class VehicleHub;

using TrackList = QList<TrackPoint>;

class LogReplay : public QObject
{
    Q_OBJECT
public:
    LogReplay(LinkManager *link, VehicleHub *hub, QObject *parent = nullptr);
    ~LogReplay() override;

    /// Log ms at the cursor (or the play clock while playing).
    double currentLogMs() const;

public slots:
    void load(const QString &path);
    void unload();
    void play();
    void pause();
    void seek(double targetMs);
    void setSpectrumConfig(int n, int avg, int stepMs);
    void selectSpectrumStream(int sensor, const QString &key);

signals:
    /// LOADED, PLAYING, PAUSED, UNLOADED
    void stateChanged(const QString &state, const QString &fileName, const QString &format, double totalMs, int totalMessages);
    void loaded(const corv::TrackList &track);
    void loadFailed(const QString &error);
    void tick(double currentMs, double totalMs, bool playing);
    /// A seek or a load: the views drop what they drew of another time.
    void resetState();
    void spectrum(const corv::LogSpectrumMsg &m);
    void spectrumConfigApplied(int n, int avg, int stepMs);

private:
    void emitAt(std::size_t i);
    void onTick();
    void sendUiTick();
    void sendState(const QString &state);
    void internalPause();
    void internalPlay();
    void replayStickyUpTo(std::size_t cursor);
    void seedPosition();
    const std::uint8_t *data() const;

    LinkManager *m_link;
    VehicleHub *m_hub;
    std::unique_ptr<MavlinkParser> m_parser;
    std::unique_ptr<LogSpectrum> m_spectrum;
    QTimer m_tick, m_uiTick, m_spectrumPump;

    bool m_loaded = false;
    QString m_fileName, m_format;
    QFile m_file;                         // .tlog, mapped
    const std::uint8_t *m_map = nullptr;
    ParsedLog m_log;
    std::size_t m_cursor = 0;
    bool m_playing = false;
    qint64 m_baseWall = 0;
    double m_baseLogMs = 0;
    std::optional<TrackPoint> m_initialPos;
};

} // namespace corv

Q_DECLARE_METATYPE(corv::TrackList)
