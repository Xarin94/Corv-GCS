// FpvStream.h - the camera's video, from the RTSP address to frames in the
// VideoMailbox (the main-process half of fpv-manager.js and the decoder half of
// FPVController.js). Lives on the video thread: start() and stop() are queued.
//
// A picture is decoded as soon as it is complete. Should decoding fall more than
// kMaxQueue pictures behind, it takes up again at the next key frame rather than
// show a video that lags further and further. At most targetFps frames a second
// reach the 3D view (SYS CONFIG -> TARGET FPS).
#pragma once

#include "video/RtspClient.h"
#include "video/VideoDecoder.h"

#include <QElapsedTimer>
#include <QObject>
#include <deque>

QT_FORWARD_DECLARE_CLASS(QTimer)

namespace corv {

class FpvStream : public QObject
{
    Q_OBJECT
public:
    static constexpr int kFirstFrameTimeoutMs = 6000;
    static constexpr int kMaxQueue = 8;

    explicit FpvStream(QObject *parent = nullptr);
    ~FpvStream() override;

public slots:
    void start(const QString &url, int targetFps);
    void stop();

signals:
    /// The first picture of the stream is in the mailbox.
    void started(const QString &description, int width, int height);
    /// The stream could not start or ended on its own; error may be empty.
    void ended(bool wasStarted, const QString &error);

private:
    void onAccessUnit(const AccessUnit &au, qint64 timestampUs);
    void decodeNext();
    void show(VideoFramePtr f);
    void end(const QString &error);
    void reset();
    bool openDecoder(bool hardware);

    RtspClient *m_client = nullptr;
    VideoDecoder m_decoder;
    QTimer *m_watchdog = nullptr, *m_decodeTimer = nullptr;
    struct Item { AccessUnit au; qint64 timestampUs; };
    std::deque<Item> m_queue;
    bool m_awaitKey = true;
    bool m_started = false;
    bool m_hardware = true;
    QElapsedTimer m_clock;
    std::deque<qint64> m_errors;     // of the decoder, ms: three in 10 s and it is replaced
    double m_framePeriodMs = 1000.0 / 30, m_nextFrameMs = 0;
    qint64 m_lastPtsUs = -1;
    bool m_byArrival = false;        // the camera's time stamps do not advance: thinned out on the clock
    qint64 m_frames = 0, m_shown = 0, m_skipped = 0;
};

} // namespace corv
