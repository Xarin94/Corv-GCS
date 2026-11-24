// VideoDecoder.h - H.264 / H.265 access units to frames, with FFmpeg's libavcodec
// (guide, "Video FPV"): the video engine of the GPU first (VAAPI, D3D11VA,
// VideoToolbox), the software decoder when there is none or it keeps failing.
// One thread at a time; no reordering delay (low-delay streams).
#pragma once

#include "scene/VideoFrame.h"
#include "video/Rtp.h"

#include <QString>
#include <functional>
#include <memory>

namespace corv {

class VideoDecoder
{
public:
    VideoDecoder();
    ~VideoDecoder();
    VideoDecoder(const VideoDecoder &) = delete;
    VideoDecoder &operator=(const VideoDecoder &) = delete;

    /// Built with FFmpeg. Without it there is no FPV video.
    static bool available();

    /// hardware: try the GPU's decoder first (CORV_VIDEO_HW=0 keeps to the software one).
    bool open(VideoCodec codec, bool hardware, QString *error);
    void close();
    /// Pictures come out through the sink, in this call. False: the decoder refused the data.
    bool decode(const QByteArray &annexB, qint64 ptsUs, const std::function<void(VideoFramePtr)> &sink, QString *error);
    /// "H264 · VAAPI", "H265 · software": what decodes (after the first frame it is what really did).
    QString description() const;
    bool hardware() const;

private:
    struct Private;
    std::unique_ptr<Private> d;
};

} // namespace corv
