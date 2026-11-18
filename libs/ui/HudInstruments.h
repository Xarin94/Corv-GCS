// HudInstruments.h - the HUD split into instruments (the drawing functions of
// js/hud/HUDRenderer.js, one item each).
//
// Every instrument is an item of its own size whose drawing is a texture.
// sync() runs on the GUI thread once per frame with the frame HudItem
// computed: it moves the item and builds a key of what the drawing depends
// on, rounded to the pixel; only a changed key repaints. The repaint happens
// in updatePaintNode() on the render thread (GUI blocked) with QPainter, into
// an image of the item's size in device pixels, and reads only what sync()
// stored.
//
// The drawing is white over a black shadow and the HUD colour is applied by
// the material (shaders/hudtint.*): green, blue and the disarmed red/orange
// pulse cost a uniform, not a repaint.
//
//   instrument      moves with                    repaints on
//   HudLadder       roll (rotation), pitch (y)    nothing after the first paint
//   HudSkyPointer   roll (rotation)               slip/skid pixel
//   HudBankScale    -                             size
//   HudFpm          AoA / SSA (x, y)              caged, speed-error pixel
//   HudTrackMarker  track / flight-path angle     caged
//   HudBoresight    pitch in horizon lock         caged
//   HudHeadingTape  -                             heading degree (the box and its number)
//   HudHeadingStrip heading (x, clipped)          nothing after the first paint (the scale)
//   HudTapeDiamond  track (x, clipped)            nothing (the ground-track diamond on the scale)
//   HudNavBug       nav bearing (x)               nothing (cyan, not tinted)
//   HudVario        -                             pointer pixel or readout of the VSI
//   HudFma          -                             mode, waypoint line, wind arrow degree
//   HudMessages     -                             a message added, expired or fading (own colours)
#pragma once

#include "ui/HudItem.h"

#include <QElapsedTimer>
#include <QQuickItem>
#include <deque>

class QPainter;

namespace corv {

/// FNV-1a over the values a drawing depends on.
struct HudKey {
    quint64 h = 1469598103934665603ull;
    HudKey &operator<<(qint64 v)
    {
        h ^= quint64(v);
        h *= 1099511628211ull;
        return *this;
    }
    HudKey &operator<<(const QString &s) { return *this << qint64(qHash(s)); }
};

class HudInstrument : public QQuickItem
{
    Q_OBJECT
public:
    HudInstrument(HudItem *hud, QQuickItem *parent);
    /// GUI thread, once per frame: place the item, set the tint, repaint if the key changed.
    virtual void sync(const HudFrame &f) = 0;

protected:
    QSGNode *updatePaintNode(QSGNode *old, UpdatePaintNodeData *) override;
    /// Render thread, GUI blocked: draw in item coordinates (white symbols, black shadow).
    virtual void draw(QPainter *p) = 0;

    void repaintIf(const HudKey &k);
    void setTint(const QColor &c);
    /// Position rounded to the device pixel: a texture drawn between pixels blurs its lines.
    void place(double x, double y);
    void resizeTo(double w, double h);

    HudItem *m_hud;

private:
    QColor m_tint = Qt::white;
    quint64 m_key = 0;
    bool m_keyed = false;
    bool m_dirty = true;
};

class HudLadder : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    double m_pxPerDeg = 12;
};

class HudSkyPointer : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    double m_skid = 0;
};

class HudBankScale : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    double m_radius = 180;
};

class HudFpm : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    bool m_caged = false;
    double m_worm = 0;   // speed error bar, px (0: hidden)
};

class HudTrackMarker : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    bool m_caged = false;
};

class HudBoresight : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    bool m_caged = false;
};

class HudHeadingTape : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;
    /// Where the tick scale starts, HUD y (the strip and the nav bug sit on it)
    static double scaleTop();
    /// Width and height of the visible scale (61 degrees)
    static QSizeF scaleSize();

protected:
    void draw(QPainter *p) override;
    int m_heading = 0;    // the number in the box
};

/// The tick scale of the heading tape, painted once from -40 to 400 degrees and
/// slid under a clip as wide as the 61 degrees shown.
class HudHeadingStrip : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
};

class HudTapeDiamond : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
};

class HudNavBug : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
};

class HudVario : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    double m_tapeH = 110;
    double m_vs = 0;    // readout, rounded to 0.1
    double m_vsY = 0;   // pointer position, px
};

class HudFma : public HudInstrument
{
    Q_OBJECT
public:
    using HudInstrument::HudInstrument;
    void sync(const HudFrame &f) override;

protected:
    void draw(QPainter *p) override;
    QString m_mode, m_wp, m_windText;
    bool m_wind = false;
    double m_windRot = 0;      // degrees, rounded
    double m_cx = 0, m_windX = 0;
};

class HudMessages : public HudInstrument
{
    Q_OBJECT
public:
    HudMessages(HudItem *hud, QQuickItem *parent);
    void sync(const HudFrame &f) override;
    void push(const QString &text, const QString &level);

protected:
    void draw(QPainter *p) override;

private:
    struct Msg {
        QString text, level;
        qint64 t;
        double alpha = 1;
    };
    std::deque<Msg> m_msgs;
    QElapsedTimer m_clock;
    double m_fontPx = 13;
};

} // namespace corv
