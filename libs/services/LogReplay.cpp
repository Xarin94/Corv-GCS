#include "services/LogReplay.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "link/LinkManager.h"
#include "link/MavlinkParser.h"
#include "vehicle/VehicleHub.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <algorithm>
#include <map>

namespace corv {

namespace {
constexpr int kTickMs = 50;
constexpr int kUiTickMs = 100;
constexpr int kMaxPerTick = 200;   // safety cap per tick
// Re-sent after a seek so mode, home, battery and GPS are right at once
const int kSticky[] = {0, 1, 24, 33, 42, 74, 147, 242, 253};
} // namespace

LogReplay::LogReplay(LinkManager *link, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_link(link), m_hub(hub), m_tick(this), m_uiTick(this), m_spectrumPump(this)
{
    qRegisterMetaType<corv::TrackList>();
    qRegisterMetaType<corv::LogSpectrumMsg>();
    m_parser = std::make_unique<MavlinkParser>([this](const mavlink_message_t &msg, const QByteArray &) {
        MavMessage m;
        m.msg = msg;
        m.arrivalMs = wallMs();
        m.replay = true;
        m.synthetic = m_format == QLatin1String("bin");
        if (m_link) m_link->injectReplay(m);
    });
    m_spectrum = std::make_unique<LogSpectrum>([this](const LogSpectrumMsg &m) { emit spectrum(m); });
    m_tick.setInterval(kTickMs);
    m_uiTick.setInterval(kUiTickMs);
    m_spectrumPump.setInterval(50);
    connect(&m_tick, &QTimer::timeout, this, &LogReplay::onTick);
    connect(&m_uiTick, &QTimer::timeout, this, &LogReplay::sendUiTick);
    connect(&m_spectrumPump, &QTimer::timeout, this, [this] { m_spectrum->pump(); });
}

LogReplay::~LogReplay() = default;

const std::uint8_t *LogReplay::data() const
{
    return m_format == QLatin1String("bin") ? reinterpret_cast<const std::uint8_t *>(m_log.frames.constData()) : m_map;
}

void LogReplay::emitAt(std::size_t i)
{
    if (i >= m_log.index.size()) return;
    const LogEntry &e = m_log.index[i];
    m_parser->push(data() + e.offset, qsizetype(e.len));
}

double LogReplay::currentLogMs() const
{
    if (!m_loaded) return 0;
    if (!m_playing) return m_cursor < m_log.index.size() ? m_log.index[m_cursor].tsMs : 0;
    return m_baseLogMs + double(wallMs() - m_baseWall);   // speed 1x
}

void LogReplay::onTick()
{
    if (!m_loaded || !m_playing) return;
    const double target = currentLogMs();
    int emitted = 0;
    while (m_cursor < m_log.index.size() && m_log.index[m_cursor].tsMs <= target) {
        emitAt(m_cursor++);
        if (++emitted > kMaxPerTick) break;
    }
    if (m_cursor >= m_log.index.size()) {
        // End of the file: pause on the last frame
        internalPause();
        sendState(QStringLiteral("PAUSED"));
    }
}

void LogReplay::sendUiTick()
{
    if (!m_loaded) return;
    emit tick(currentLogMs(), m_log.totalMs, m_playing);
}

void LogReplay::sendState(const QString &state)
{
    if (state != QLatin1String("UNLOADED"))
        LOG_I("replay") << state << "at" << QString::number(currentLogMs() / 1000, 'f', 1) << "s";
    emit stateChanged(state, m_fileName, m_format, m_log.totalMs, int(m_log.index.size()));
}

void LogReplay::internalPause()
{
    m_playing = false;
    m_tick.stop();
    m_uiTick.stop();
    sendUiTick();
}

void LogReplay::internalPlay()
{
    if (m_cursor >= m_log.index.size()) m_cursor = 0;   // rewind at the end
    m_baseLogMs = m_cursor < m_log.index.size() ? m_log.index[m_cursor].tsMs : 0;
    m_baseWall = wallMs();
    m_playing = true;
    m_tick.start();
    m_uiTick.start();
    sendUiTick();
}

void LogReplay::replayStickyUpTo(std::size_t cursor)
{
    std::map<int, std::size_t> last;   // msgid -> index
    const std::uint8_t *d = data();
    for (std::size_t i = 0; i < cursor && i < m_log.index.size(); ++i) {
        const LogEntry &e = m_log.index[i];
        const int id = frameMsgId(d + e.offset, e.len);
        if (std::find(std::begin(kSticky), std::end(kSticky), id) != std::end(kSticky)) last[id] = i;
    }
    std::vector<std::size_t> order;
    for (const auto &kv : last) order.push_back(kv.second);
    std::sort(order.begin(), order.end());
    for (std::size_t i : order) emitAt(i);
}

/// The vehicle starts where the log does, before its first position streams in.
void LogReplay::seedPosition()
{
    if (!m_initialPos) return;
    const double lat = m_initialPos->lat, lon = m_initialPos->lon;
    m_hub->mutate([lat, lon](VehicleState &s) {
        s.lat = lat;
        s.lon = lon;
    });
}

void LogReplay::seek(double targetMs)
{
    if (!m_loaded) return;
    targetMs = std::clamp(targetMs, 0.0, double(m_log.totalMs));
    const bool wasPlaying = m_playing;
    if (wasPlaying) internalPause();
    const auto it = std::lower_bound(m_log.index.begin(), m_log.index.end(), targetMs,
                                     [](const LogEntry &e, double t) { return double(e.tsMs) < t; });
    m_cursor = std::min<std::size_t>(std::size_t(it - m_log.index.begin()), m_log.index.empty() ? 0 : m_log.index.size() - 1);

    // The vehicle state of another time goes, posted before the sticky frames
    m_hub->resetTelemetry();
    seedPosition();
    emit resetState();
    m_parser->reset();
    replayStickyUpTo(m_cursor);
    if (m_format == QLatin1String("bin")) m_spectrum->restart();
    if (wasPlaying) internalPlay();
    else sendUiTick();
}

void LogReplay::unload()
{
    if (!m_loaded) return;
    internalPause();
    m_spectrum->unload();
    m_spectrumPump.stop();
    m_loaded = false;
    m_log = ParsedLog{};
    if (m_map) m_file.unmap(const_cast<uchar *>(m_map));
    m_map = nullptr;
    m_file.close();
    m_initialPos.reset();
    m_link->setReplayActive(false);
    m_hub->setMode(QStringLiteral("LIVE"));
    sendState(QStringLiteral("UNLOADED"));
    m_fileName.clear();
    m_format.clear();
    emit tick(0, 0, false);
    LOG_I("replay") << "unloaded";
}

void LogReplay::load(const QString &path)
{
    if (m_loaded) unload();
    const QFileInfo fi(path);
    const QString ext = fi.suffix().toLower();
    if (ext != QLatin1String("tlog") && ext != QLatin1String("bin")) {
        emit loadFailed(QStringLiteral("Unsupported extension: .%1").arg(ext));
        return;
    }
    m_file.setFileName(path);
    if (!m_file.open(QIODevice::ReadOnly)) {
        emit loadFailed(m_file.errorString());
        return;
    }
    const qint64 size = m_file.size();
    uchar *map = size > 0 ? m_file.map(0, size) : nullptr;
    if (!map) {
        m_file.close();
        emit loadFailed(size > 0 ? QStringLiteral("cannot map the file") : QStringLiteral("empty file"));
        return;
    }
    QElapsedTimer clock;
    clock.start();
    if (ext == QLatin1String("tlog")) {
        m_log = indexTlog(map, size);
        m_map = map;
    } else {
        m_log = parseBinLog(map, size);
        // The frames are ours now: the file is not needed while playing
        m_file.unmap(map);
        m_file.close();
        m_map = nullptr;
    }
    m_format = m_log.format;
    m_fileName = fi.fileName();
    m_cursor = 0;
    m_playing = false;
    m_loaded = true;
    LOG_I("replay") << "loaded" << m_fileName << "(" << m_format << "," << m_log.index.size() << "msgs,"
                    << QString::number(m_log.totalMs / 1000.0, 'f', 0) << "s of flight) in" << clock.elapsed() << "ms"
                    << (m_log.gapsCompressed ? QStringLiteral(", %1 gaps compressed").arg(m_log.gapsCompressed) : QString());

    m_link->setReplayActive(true);
    // REPLAY: VehicleHub takes only replay frames, and the demo flight stops
    m_hub->setMode(QStringLiteral("REPLAY"));
    m_hub->resetTelemetry();
    m_initialPos = m_log.gpsTrack.empty() ? std::nullopt : std::optional<TrackPoint>(m_log.gpsTrack.front());
    // A log is a data stream too: the scene starts at its first fix
    if (m_initialPos) m_hub->setSceneOrigin(m_initialPos->lat, m_initialPos->lon);
    seedPosition();
    m_parser->reset();
    emit resetState();
    sendState(QStringLiteral("LOADED"));
    sendUiTick();
    emit loaded(TrackList(m_log.gpsTrack.begin(), m_log.gpsTrack.end()));

    // A .bin with IMU samples: the GCS computes the spectra. A .tlog carries the
    // vehicle's own (DEBUG_FLOAT_ARRAY), replayed like any message.
    if (m_format == QLatin1String("bin")) {
        m_spectrum->load(m_log.imu, [this] { return currentLogMs(); });
        m_spectrumPump.start();
        QStringList names;
        for (const auto &list : m_log.imu.sensors)
            for (const auto &s : list) names << QStringLiteral("%1 %2 Hz").arg(s->key).arg(std::lround(s->fs));
        LOG_I("replay") << "IMU samples for the spectrum:" << (names.isEmpty() ? QStringLiteral("none") : names.join(QLatin1String(", ")));
    } else {
        m_spectrum->unload();
    }
}

void LogReplay::play()
{
    if (!m_loaded) return;
    internalPlay();
    sendState(QStringLiteral("PLAYING"));
}

void LogReplay::pause()
{
    if (!m_loaded) return;
    internalPause();
    sendState(QStringLiteral("PAUSED"));
}

void LogReplay::setSpectrumConfig(int n, int avg, int stepMs)
{
    const LogFftConfig applied = m_spectrum->setConfig({n, avg, stepMs});
    emit spectrumConfigApplied(applied.n, applied.avg, applied.stepMs);
}

void LogReplay::selectSpectrumStream(int sensor, const QString &key) { m_spectrum->selectStream(sensor, key); }

} // namespace corv
