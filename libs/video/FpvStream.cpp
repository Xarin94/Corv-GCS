#include "video/FpvStream.h"

#include "core/DebugLog.h"

#include <QTimer>
#include <algorithm>

using namespace Qt::StringLiterals;

namespace corv {

FpvStream::FpvStream(QObject *parent) : QObject(parent)
{
    m_watchdog = new QTimer(this);
    m_watchdog->setSingleShot(true);
    connect(m_watchdog, &QTimer::timeout, this, [this] {
        // Playing, and nothing to show: a firewall on the UDP ports, or a stream that cannot be decoded
        end(m_client && m_client->received() > 0 ? u"no picture could be decoded"_s : u"no video arrived (firewall?)"_s);
    });
    m_decodeTimer = new QTimer(this);
    m_decodeTimer->setSingleShot(true);
    m_decodeTimer->setInterval(0);   // after the network: what has arrived is queued first
    connect(m_decodeTimer, &QTimer::timeout, this, &FpvStream::decodeNext);
    m_clock.start();
}

FpvStream::~FpvStream() { reset(); }

void FpvStream::reset()
{
    m_watchdog->stop();
    m_decodeTimer->stop();
    m_queue.clear();
    if (m_client) {
        m_client->disconnect(this);
        m_client->stop();
        m_client->deleteLater();
        m_client = nullptr;
    }
    m_decoder.close();
    VideoMailbox::instance().clear();
    m_started = false;
    m_awaitKey = true;
    m_errors.clear();
    m_frames = m_shown = m_skipped = 0;
    m_nextFrameMs = 0;
    m_lastPtsUs = -1;
    m_byArrival = false;
}

bool FpvStream::openDecoder(bool hardware)
{
    QString error;
    m_hardware = hardware;
    if (m_decoder.open(m_client->codec(), hardware, &error)) return true;
    end(error);
    return false;
}

void FpvStream::start(const QString &url, int targetFps)
{
    reset();
    m_framePeriodMs = 1000.0 / std::clamp(targetFps, 1, 60);
    LOG_I("fpv") << "starting" << url << "at up to" << std::clamp(targetFps, 1, 60) << "fps";
    m_client = new RtspClient(url, this);
    connect(m_client, &RtspClient::playing, this, [this] {
        LOG_I("fpv") << "stream" << (m_client->codec() == VideoCodec::H265 ? "H265" : "H264") << "over" << m_client->transport().toUpper();
        if (openDecoder(true)) m_watchdog->start(kFirstFrameTimeoutMs);
    });
    connect(m_client, &RtspClient::accessUnit, this, &FpvStream::onAccessUnit);
    connect(m_client, &RtspClient::finished, this, &FpvStream::end);
    m_client->start();
}

void FpvStream::stop()
{
    if (m_client) LOG_I("fpv") << "stopped:" << m_frames << "frames decoded," << m_shown << "shown," << m_skipped << "skipped";
    reset();
}

void FpvStream::end(const QString &error)
{
    const bool was = m_started;
    LOG_W("fpv") << "stream ended" << (error.isEmpty() ? u"(closed by the camera)"_s : error);
    reset();
    emit ended(was, error);
}

void FpvStream::onAccessUnit(const AccessUnit &au, qint64 timestampUs)
{
    if (m_awaitKey) {
        if (!au.key) return;
        m_awaitKey = false;
    }
    if (int(m_queue.size()) >= kMaxQueue) {
        m_skipped += qint64(m_queue.size());
        m_queue.clear();
        if (!au.key) {
            m_awaitKey = true;
            return;
        }
    }
    m_queue.push_back({au, timestampUs});
    if (!m_decodeTimer->isActive()) m_decodeTimer->start();
}

void FpvStream::decodeNext()
{
    if (m_queue.empty() || !m_client) return;
    const Item item = std::move(m_queue.front());
    m_queue.pop_front();
    QString error;
    const bool ok = m_decoder.decode(item.au.data, item.timestampUs, [this](VideoFramePtr f) { show(std::move(f)); }, &error);
    if (!m_client) return;
    if (!ok) {
        LOG_W("fpv") << "decoder:" << error;
        m_queue.clear();
        m_awaitKey = true;
        const qint64 now = m_clock.elapsed();
        while (!m_errors.empty() && now - m_errors.front() > 10000) m_errors.pop_front();
        m_errors.push_back(now);
        if (m_errors.size() >= 3 && m_hardware && m_decoder.hardware()) {
            LOG_W("fpv") << "the hardware decoder keeps failing: software from here on";
            m_errors.clear();
            if (!openDecoder(false)) return;
        }
    }
    if (!m_queue.empty()) m_decodeTimer->start();
}

void FpvStream::show(VideoFramePtr f)
{
    ++m_frames;
    // TARGET FPS: the frames in between are decoded (the next ones need them) and not
    // shown. By the camera's time stamps, which the network does not jitter
    if (f->ptsUs <= m_lastPtsUs && !m_byArrival) {
        m_byArrival = true;
        m_nextFrameMs = 0;
    }
    m_lastPtsUs = f->ptsUs;
    const double at = m_byArrival ? double(m_clock.nsecsElapsed()) / 1e6 : double(f->ptsUs) / 1e3;
    if (m_started && at < m_nextFrameMs - m_framePeriodMs / 4) return;
    m_nextFrameMs = at - m_nextFrameMs > m_framePeriodMs ? at + m_framePeriodMs : m_nextFrameMs + m_framePeriodMs;
    ++m_shown;
    const int w = f->width, h = f->height;
    VideoMailbox::instance().put(std::move(f));
    if (m_started) return;
    m_started = true;
    m_watchdog->stop();
    emit started(m_decoder.description(), w, h);
}

} // namespace corv
