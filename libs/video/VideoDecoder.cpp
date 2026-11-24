#include "video/VideoDecoder.h"

#include "core/DebugLog.h"

#include <cstring>

#ifdef CORV_HAVE_FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/hwcontext.h>
#include <libavutil/pixdesc.h>
}
#endif

using namespace Qt::StringLiterals;

namespace corv {

#ifdef CORV_HAVE_FFMPEG

namespace {
QString avError(int code)
{
    char buf[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(code, buf, sizeof buf);
    return QString::fromUtf8(buf);
}

const AVHWDeviceType kHardware[] = {
#if defined(Q_OS_WIN)
    AV_HWDEVICE_TYPE_D3D11VA,
#elif defined(Q_OS_MACOS)
    AV_HWDEVICE_TYPE_VIDEOTOOLBOX,
#else
    AV_HWDEVICE_TYPE_VAAPI,
#endif
};

/// Rows without the decoder's padding; 10-bit samples (little endian, in 16 bits) cut to 8.
QByteArray plane(const std::uint8_t *data, int stride, int bytesPerRow, int rows, int depthShift)
{
    if (!data || bytesPerRow <= 0 || rows <= 0) return {};
    QByteArray out(qsizetype(bytesPerRow) * rows, Qt::Uninitialized);
    for (int r = 0; r < rows; ++r) {
        const std::uint8_t *src = data + qptrdiff(r) * stride;
        auto *dst = reinterpret_cast<std::uint8_t *>(out.data()) + qptrdiff(r) * bytesPerRow;
        if (depthShift < 0) {
            std::memcpy(dst, src, std::size_t(bytesPerRow));
        } else {
            for (int i = 0; i < bytesPerRow; ++i) dst[i] = std::uint8_t((src[2 * i] | (src[2 * i + 1] << 8)) >> depthShift);
        }
    }
    return out;
}

VideoFramePtr toFrame(const AVFrame *f, qint64 ptsUs)
{
    const auto fmt = AVPixelFormat(f->format);
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(fmt);
    if (!desc || (desc->flags & (AV_PIX_FMT_FLAG_RGB | AV_PIX_FMT_FLAG_HWACCEL | AV_PIX_FMT_FLAG_BE)) || desc->nb_components < 3) return nullptr;
    const int depth = desc->comp[0].depth;
    // P010 and the like keep their bits on top, yuv420p10 at the bottom
    const int shift = depth <= 8 ? -1 : desc->comp[0].shift + depth - 8;
    auto out = std::make_shared<VideoFrame>();
    out->width = f->width;
    out->height = f->height;
    out->ptsUs = ptsUs;
    out->chromaWidth = AV_CEIL_RSHIFT(f->width, desc->log2_chroma_w);
    out->chromaHeight = AV_CEIL_RSHIFT(f->height, desc->log2_chroma_h);
    out->fullRange = f->color_range == AVCOL_RANGE_JPEG || fmt == AV_PIX_FMT_YUVJ420P || fmt == AV_PIX_FMT_YUVJ422P || fmt == AV_PIX_FMT_YUVJ444P;
    // Cameras seldom say: HD is BT.709, below it BT.601
    out->bt709 = f->colorspace == AVCOL_SPC_BT709 || (f->colorspace == AVCOL_SPC_UNSPECIFIED && f->height >= 720);
    out->y = plane(f->data[0], f->linesize[0], out->width, out->height, shift);
    if (desc->comp[1].plane == desc->comp[2].plane) {   // NV12, P010
        if (desc->comp[1].offset > desc->comp[2].offset) return nullptr;   // NV21: no decoder gives it
        out->interleaved = true;
        out->u = plane(f->data[1], f->linesize[1], out->chromaWidth * 2, out->chromaHeight, shift);
    } else {
        out->u = plane(f->data[1], f->linesize[1], out->chromaWidth, out->chromaHeight, shift);
        out->v = plane(f->data[2], f->linesize[2], out->chromaWidth, out->chromaHeight, shift);
        if (out->v.isEmpty()) return nullptr;
    }
    if (out->y.isEmpty() || out->u.isEmpty()) return nullptr;
    return out;
}
} // namespace

struct VideoDecoder::Private {
    AVCodecContext *ctx = nullptr;
    AVBufferRef *device = nullptr;
    AVPixelFormat hwFormat = AV_PIX_FMT_NONE;
    AVFrame *frame = nullptr, *sw = nullptr;
    AVPacket *packet = nullptr;
    VideoCodec codec = VideoCodec::H264;
    QString hwName;
    bool usedHardware = false, firstFrame = true;

    static AVPixelFormat chooseFormat(AVCodecContext *c, const AVPixelFormat *list)
    {
        const auto *self = static_cast<const Private *>(c->opaque);
        for (const AVPixelFormat *p = list; *p != AV_PIX_FMT_NONE; ++p)
            if (*p == self->hwFormat) return *p;
        // Not in the list (a profile the engine does not do): the software format
        for (const AVPixelFormat *p = list; *p != AV_PIX_FMT_NONE; ++p) {
            const AVPixFmtDescriptor *d = av_pix_fmt_desc_get(*p);
            if (d && !(d->flags & AV_PIX_FMT_FLAG_HWACCEL)) return *p;
        }
        return AV_PIX_FMT_NONE;
    }
};

VideoDecoder::VideoDecoder() : d(std::make_unique<Private>()) {}
VideoDecoder::~VideoDecoder() { close(); }

bool VideoDecoder::available() { return true; }

void VideoDecoder::close()
{
    if (d->ctx) avcodec_free_context(&d->ctx);
    if (d->device) av_buffer_unref(&d->device);
    if (d->frame) av_frame_free(&d->frame);
    if (d->sw) av_frame_free(&d->sw);
    if (d->packet) av_packet_free(&d->packet);
    d->hwFormat = AV_PIX_FMT_NONE;
    d->hwName.clear();
    d->usedHardware = false;
    d->firstFrame = true;
}

bool VideoDecoder::open(VideoCodec codec, bool hardware, QString *error)
{
    close();
    av_log_set_level(AV_LOG_QUIET);   // what matters comes back as error codes
    d->codec = codec;
    const AVCodec *c = avcodec_find_decoder(codec == VideoCodec::H265 ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264);
    if (!c) {
        if (error) *error = u"no %1 decoder in this FFmpeg"_s.arg(codec == VideoCodec::H265 ? u"H.265"_s : u"H.264"_s);
        return false;
    }
    d->ctx = avcodec_alloc_context3(c);
    d->frame = av_frame_alloc();
    d->sw = av_frame_alloc();
    d->packet = av_packet_alloc();
    if (!d->ctx || !d->frame || !d->sw || !d->packet) {
        if (error) *error = u"out of memory"_s;
        close();
        return false;
    }
    d->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    d->ctx->flags2 |= AV_CODEC_FLAG2_FAST;
    // Frame threads would hold back as many pictures as there are threads
    d->ctx->thread_type = FF_THREAD_SLICE;
    d->ctx->thread_count = 0;
    d->ctx->opaque = d.get();

    if (hardware && qEnvironmentVariable("CORV_VIDEO_HW") != "0"_L1) {
        for (const AVHWDeviceType type : kHardware) {
            AVPixelFormat fmt = AV_PIX_FMT_NONE;
            for (int i = 0;; ++i) {
                const AVCodecHWConfig *cfg = avcodec_get_hw_config(c, i);
                if (!cfg) break;
                if ((cfg->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) && cfg->device_type == type) {
                    fmt = cfg->pix_fmt;
                    break;
                }
            }
            if (fmt == AV_PIX_FMT_NONE) continue;
            const int rc = av_hwdevice_ctx_create(&d->device, type, nullptr, nullptr, 0);
            const QString name = QString::fromLatin1(av_hwdevice_get_type_name(type)).toUpper();
            if (rc < 0) {
                LOG_I("fpv") << name << "not available:" << avError(rc);
                continue;
            }
            d->hwFormat = fmt;
            d->hwName = name;
            d->ctx->hw_device_ctx = av_buffer_ref(d->device);
            d->ctx->get_format = &Private::chooseFormat;
            break;
        }
    }
    const int rc = avcodec_open2(d->ctx, c, nullptr);
    if (rc < 0) {
        if (error) *error = avError(rc);
        close();
        return false;
    }
    return true;
}

bool VideoDecoder::decode(const QByteArray &annexB, qint64 ptsUs, const std::function<void(VideoFramePtr)> &sink, QString *error)
{
    if (!d->ctx) return false;
    // The bitstream readers of libavcodec look past the end
    QByteArray padded = annexB;
    padded.append(AV_INPUT_BUFFER_PADDING_SIZE, '\0');
    d->packet->data = reinterpret_cast<std::uint8_t *>(padded.data());
    d->packet->size = int(annexB.size());
    d->packet->pts = ptsUs;
    int rc = avcodec_send_packet(d->ctx, d->packet);
    d->packet->data = nullptr;
    d->packet->size = 0;
    if (rc < 0) {
        if (error) *error = avError(rc);
        return false;
    }
    for (;;) {
        rc = avcodec_receive_frame(d->ctx, d->frame);
        if (rc == AVERROR(EAGAIN) || rc == AVERROR_EOF) return true;
        if (rc < 0) {
            if (error) *error = avError(rc);
            return false;
        }
        const AVFrame *f = d->frame;
        d->usedHardware = d->hwFormat != AV_PIX_FMT_NONE && d->frame->format == d->hwFormat;
        if (d->usedHardware) {
            rc = av_hwframe_transfer_data(d->sw, d->frame, 0);
            if (rc < 0) {
                av_frame_unref(d->frame);
                if (error) *error = avError(rc);
                return false;
            }
            d->sw->color_range = d->frame->color_range;
            d->sw->colorspace = d->frame->colorspace;
            // The surface is padded to whole blocks: the picture is what the stream says
            d->sw->width = d->frame->width;
            d->sw->height = d->frame->height;
            f = d->sw;
        }
        const qint64 pts = d->frame->pts != AV_NOPTS_VALUE ? d->frame->pts : ptsUs;
        VideoFramePtr out = toFrame(f, pts);
        const QString format = QString::fromLatin1(av_get_pix_fmt_name(AVPixelFormat(f->format)));
        av_frame_unref(d->sw);
        av_frame_unref(d->frame);
        if (!out) {
            if (error) *error = u"pixel format %1 not supported"_s.arg(format);
            return false;
        }
        if (std::exchange(d->firstFrame, false))
            LOG_I("fpv") << "video" << out->width << "x" << out->height << format << description();
        sink(std::move(out));
    }
}

bool VideoDecoder::hardware() const { return d->ctx && (d->firstFrame ? d->hwFormat != AV_PIX_FMT_NONE : d->usedHardware); }

QString VideoDecoder::description() const
{
    return u"%1 · %2"_s.arg(d->codec == VideoCodec::H265 ? u"H265"_s : u"H264"_s, hardware() ? d->hwName : u"software"_s);
}

#else   // no FFmpeg in this build

struct VideoDecoder::Private {};
VideoDecoder::VideoDecoder() : d(std::make_unique<Private>()) {}
VideoDecoder::~VideoDecoder() = default;
bool VideoDecoder::available() { return false; }
bool VideoDecoder::open(VideoCodec, bool, QString *error)
{
    if (error) *error = u"this build has no video decoder (FFmpeg)"_s;
    return false;
}
void VideoDecoder::close() {}
bool VideoDecoder::decode(const QByteArray &, qint64, const std::function<void(VideoFramePtr)> &, QString *error)
{
    if (error) *error = u"this build has no video decoder (FFmpeg)"_s;
    return false;
}
QString VideoDecoder::description() const { return {}; }
bool VideoDecoder::hardware() const { return false; }

#endif

} // namespace corv
