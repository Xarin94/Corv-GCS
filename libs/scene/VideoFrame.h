// VideoFrame.h - a decoded camera frame on its way from the video thread to the
// 3D view (guide, "Video FPV": copied in memory, uploaded as planes, converted to
// RGB in the shader). No graphics API here.
#pragma once

#include <QByteArray>
#include <QMutex>
#include <QMutexLocker>
#include <QSize>
#include <memory>

namespace corv {

/// Planar 8-bit YUV: Y, then U and V of their own size (4:2:0, 4:2:2 or 4:4:4), or
/// one interleaved UV plane (NV12, what the hardware decoders hand out). Rows tight.
struct VideoFrame {
    int width = 0, height = 0;
    int chromaWidth = 0, chromaHeight = 0;
    bool interleaved = false;          // uv holds U V pairs, v is empty
    bool fullRange = false;            // JPEG levels instead of 16..235
    bool bt709 = false;                // else BT.601
    qint64 ptsUs = 0;                  // the camera's time stamp
    QByteArray y, u, v;

    QSize size() const { return {width, height}; }
};
using VideoFramePtr = std::shared_ptr<const VideoFrame>;

/// The latest frame: the video thread puts it, the render thread takes it.
class VideoMailbox
{
public:
    static VideoMailbox &instance()
    {
        static VideoMailbox m;
        return m;
    }
    void put(VideoFramePtr f)
    {
        QMutexLocker l(&m_lock);
        m_frame = std::move(f);
        ++m_serial;
    }
    /// The frame and its serial; null before the first and after clear().
    VideoFramePtr latest(quint64 *serial) const
    {
        QMutexLocker l(&m_lock);
        *serial = m_serial;
        return m_frame;
    }
    void clear() { put(nullptr); }

private:
    mutable QMutex m_lock;
    VideoFramePtr m_frame;
    quint64 m_serial = 0;
};

} // namespace corv
