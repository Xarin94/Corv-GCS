#include "ui/HudInstruments.h"

#include "core/Clock.h"
#include "core/Geo.h"

#include <QElapsedTimer>
#include <QFontMetricsF>
#include <QHash>
#include <QPainter>
#include <QPainterPath>
#include <QQuickWindow>
#include <QSGGeometryNode>
#include <QSGMaterial>
#include <QSGMaterialShader>
#include <QSGTexture>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace corv {

namespace {
// style of HUDRenderer.js: 2 px lines; every group drawn first as a dark copy
// 1.8 px down-right with 2.5 px lines (drawWithShadow), then the symbol. The
// symbol is drawn white here and tinted by the material.
const QColor kInk(255, 255, 255);
const QColor kShadow(0, 0, 0, 153);
const QColor kCyan(0x00, 0xd2, 0xff);
constexpr double kShadowOffset = 1.8;
constexpr double kShadowWidth = 2.5;
constexpr double kLineWidth = 2;
constexpr double kStepWidth = 8;   // style.stepWidth: px per degree on the heading tape
constexpr int kTapeSteps = 61;     // degrees shown on the heading tape
constexpr int kMaxMsgs = 5;
constexpr qint64 kMsgDurationMs = 5000;

QFont hudFont(double px)
{
    QFont f;
    f.setFamilies({QStringLiteral("Arial")});
    f.setPixelSize(std::max(1, int(std::lround(px))));
    f.setBold(true);
    return f;
}

/// fillText with canvas textAlign / textBaseline semantics.
void textAt(QPainter *p, double x, double y, const QString &t, Qt::Alignment h, Qt::Alignment v)
{
    const QFontMetricsF fm(p->font());
    const double w = fm.horizontalAdvance(t);
    double tx = x, ty = y;
    if (h & Qt::AlignRight) tx -= w;
    else if (h & Qt::AlignHCenter) tx -= w / 2;
    if (v & Qt::AlignVCenter) ty += (fm.ascent() - fm.descent()) / 2;
    else if (v & Qt::AlignTop) ty += fm.ascent();
    else if (v & Qt::AlignBottom) ty -= fm.descent();
    p->drawText(QPointF(tx, ty), t);
}

/// Canvas-like pen: butt caps, miter joins, dash lengths in pixels (Qt counts them in line widths).
QPen hudPen(const QColor &c, double w, std::initializer_list<qreal> dashPx = {})
{
    QPen pen(c, w, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin);
    if (dashPx.size()) {
        QList<qreal> d;
        for (qreal v : dashPx) d << v / w;
        pen.setDashPattern(d);
    }
    return pen;
}

template <class F>
void withShadow(QPainter *p, F &&draw)
{
    p->save();
    p->translate(kShadowOffset, kShadowOffset);
    draw(kShadow, kShadowWidth);
    p->restore();
    draw(kInk, kLineWidth);
}

void fillPolygon(QPainter *p, const QColor &c, std::initializer_list<QPointF> pts)
{
    p->setPen(Qt::NoPen);
    p->setBrush(c);
    p->drawPolygon(pts.begin(), int(pts.size()));
    p->setBrush(Qt::NoBrush);
}

// ── Tinted texture material ────────────────────────────────────────────────

class HudTintMaterial : public QSGMaterial
{
public:
    HudTintMaterial() { setFlag(Blending); }
    ~HudTintMaterial() override { delete texture; }
    QSGMaterialType *type() const override
    {
        static QSGMaterialType t;
        return &t;
    }
    QSGMaterialShader *createShader(QSGRendererInterface::RenderMode) const override;
    int compare(const QSGMaterial *other) const override
    {
        auto *o = static_cast<const HudTintMaterial *>(other);
        if (texture != o->texture) return texture < o->texture ? -1 : 1;
        if (tint != o->tint) return tint.rgba() < o->tint.rgba() ? -1 : 1;
        return 0;
    }
    QSGTexture *texture = nullptr;
    QColor tint = Qt::white;
};

class HudTintShader : public QSGMaterialShader
{
public:
    HudTintShader()
    {
        setShaderFileName(VertexStage, QStringLiteral(":/hudshaders/hudtint.vert.qsb"));
        setShaderFileName(FragmentStage, QStringLiteral(":/hudshaders/hudtint.frag.qsb"));
    }
    bool updateUniformData(RenderState &state, QSGMaterial *newMat, QSGMaterial *oldMat) override
    {
        QByteArray *buf = state.uniformData();
        bool changed = false;
        if (state.isMatrixDirty()) {
            std::memcpy(buf->data(), state.combinedMatrix().constData(), 64);
            changed = true;
        }
        auto *m = static_cast<HudTintMaterial *>(newMat);
        auto *o = static_cast<HudTintMaterial *>(oldMat);
        if (!o || o->tint != m->tint) {
            const float t[4] = {float(m->tint.redF()), float(m->tint.greenF()), float(m->tint.blueF()), 1.f};
            std::memcpy(buf->data() + 64, t, 16);
            changed = true;
        }
        if (state.isOpacityDirty()) {
            const float op = state.opacity();
            std::memcpy(buf->data() + 80, &op, 4);
            changed = true;
        }
        return changed;
    }
    void updateSampledImage(RenderState &state, int binding, QSGTexture **texture, QSGMaterial *newMat, QSGMaterial *) override
    {
        if (binding != 1) return;
        auto *m = static_cast<HudTintMaterial *>(newMat);
        if (m->texture) m->texture->commitTextureOperations(state.rhi(), state.resourceUpdateBatch());
        *texture = m->texture;
    }
};

QSGMaterialShader *HudTintMaterial::createShader(QSGRendererInterface::RenderMode) const { return new HudTintShader; }

class HudNode : public QSGGeometryNode
{
public:
    HudNode() : m_geometry(QSGGeometry::defaultAttributes_TexturedPoint2D(), 4)
    {
        setGeometry(&m_geometry);
        setMaterial(&m_material);
    }
    HudTintMaterial m_material;
    QSGGeometry m_geometry;
};
} // namespace

double HudFrame::projectAngle(double rad) const { return std::tan(rad) * pxPerRad; }

// ── HudInstrument ──────────────────────────────────────────────────────────

HudInstrument::HudInstrument(HudItem *hud, QQuickItem *parent) : QQuickItem(parent), m_hud(hud)
{
    setFlag(ItemHasContents, true);
    setAntialiasing(true);
}

QSGNode *HudInstrument::updatePaintNode(QSGNode *old, UpdatePaintNodeData *)
{
    auto *node = static_cast<HudNode *>(old);
    const QSizeF size(width(), height());
    if (size.isEmpty() || !window()) {
        delete node;
        return nullptr;
    }
    if (!node) {
        node = new HudNode;
        m_dirty = true;
    }
    const qreal dpr = window()->effectiveDevicePixelRatio();
    if (m_dirty || !node->m_material.texture) {
        QElapsedTimer t;
        t.start();
        const QSize px(int(std::ceil(size.width() * dpr)), int(std::ceil(size.height() * dpr)));
        QImage img(px, QImage::Format_ARGB32_Premultiplied);
        img.setDevicePixelRatio(dpr);
        img.fill(Qt::transparent);
        {
            QPainter p(&img);
            p.setRenderHint(QPainter::Antialiasing, true);
            p.setRenderHint(QPainter::TextAntialiasing, true);
            draw(&p);
        }
        QSGTexture *tex = window()->createTextureFromImage(img, QQuickWindow::TextureHasAlphaChannel);
        tex->setFiltering(QSGTexture::Linear);   // the ladder and the sky pointer turn
        delete node->m_material.texture;
        node->m_material.texture = tex;
        node->markDirty(QSGNode::DirtyMaterial);
        m_dirty = false;
        m_hud->addPaintTime(t.nsecsElapsed());
    }
    if (node->m_material.tint != m_tint) {
        node->m_material.tint = m_tint;
        node->markDirty(QSGNode::DirtyMaterial);
    }
    QSGGeometry::updateTexturedRectGeometry(&node->m_geometry, QRectF(QPointF(0, 0), size), QRectF(0, 0, 1, 1));
    node->markDirty(QSGNode::DirtyGeometry);
    return node;
}

void HudInstrument::repaintIf(const HudKey &k)
{
    if (m_keyed && k.h == m_key) return;
    m_key = k.h;
    m_keyed = true;
    m_dirty = true;
    update();
}

void HudInstrument::setTint(const QColor &c)
{
    if (c == m_tint) return;
    m_tint = c;
    update();
}

void HudInstrument::place(double x, double y)
{
    const qreal dpr = window() ? window()->effectiveDevicePixelRatio() : 1.0;
    setPosition(QPointF(std::round(x * dpr) / dpr, std::round(y * dpr) / dpr));
}

void HudInstrument::resizeTo(double w, double h)
{
    w = std::ceil(w);
    h = std::ceil(h);
    if (w == width() && h == height()) return;
    setSize(QSizeF(w, h));
    m_keyed = false;   // a new texture: paint it whatever the key says
}

// ── Pitch ladder and horizon (drawHorizonLadder, drawPitchLadder) ───────────
// Painted once for -90..+90 degrees; the roll frame rotates it and the pitch
// moves it, both on the GPU.

void HudLadder::sync(const HudFrame &f)
{
    setTint(f.color);
    m_pxPerDeg = f.pxPerDeg;
    const double halfH = 90 * f.pxPerDeg + 40;
    resizeTo(480, 2 * halfH);
    // In the roll frame (origin on the HUD centre): the horizon sits pitchShift below it
    place(-240, -halfH + f.pitchShift * f.pxPerRad);
    repaintIf(HudKey() << qint64(std::lround(f.pxPerDeg * 100)));
}

void HudLadder::draw(QPainter *p)
{
    const double cx = width() / 2, cy = height() / 2;
    withShadow(p, [&](const QColor &c, double lw) {
        p->save();
        p->translate(cx, cy);
        {   // Horizon
            const double length = 460, space = 80, q = 12;
            QPainterPath path;
            path.moveTo(space / 2, 0); path.lineTo(length / 2 - q, 0); path.lineTo(length / 2, q);
            path.moveTo(-space / 2, 0); path.lineTo(-(length / 2 - q), 0); path.lineTo(-length / 2, q);
            p->setPen(hudPen(c, lw));
            p->drawPath(path);
        }
        p->setFont(hudFont(16));
        const double textW = QFontMetricsF(p->font()).horizontalAdvance(QStringLiteral("-90"));
        for (int deg = -90; deg <= 90; deg += 10) {
            if (deg == 0) continue;
            const double y = -(deg * m_pxPerDeg), length = 200, space = 80, q = 12;
            const double dq = deg > 0 ? q : -q;
            QPainterPath path;
            path.moveTo(space / 2, y); path.lineTo(length / 2 - q, y); path.lineTo(length / 2, y + dq);
            path.moveTo(-space / 2, y); path.lineTo(-(length / 2 - q), y); path.lineTo(-length / 2, y + dq);
            // Below-horizon rungs dashed (PFD / HUD convention)
            p->setPen(deg < 0 ? hudPen(c, lw, {10, 7}) : hudPen(c, lw));
            p->drawPath(path);
            p->setPen(c);
            textAt(p, length / 2 + 4 + textW, y + dq / 2, QString::number(deg), Qt::AlignRight, Qt::AlignVCenter);
            textAt(p, -(length / 2 + 4), y + dq / 2, QString::number(deg), Qt::AlignRight, Qt::AlignVCenter);
        }
        p->restore();
    });
}

// ── Sky pointer and slip/skid (the rotating half of drawBankArc) ───────────

void HudSkyPointer::sync(const HudFrame &f)
{
    setTint(f.color);
    const double radius = std::min(180.0, f.H * 0.28);
    m_skid = std::round(std::clamp(-f.s->ay / 9.81 * 2, -1.0, 1.0) * 16);
    resizeTo(64, 32);
    place(-32, -radius);   // local (32, 0) is (0, -radius) of the roll frame
    repaintIf(HudKey() << qint64(m_skid));
}

void HudSkyPointer::draw(QPainter *p)
{
    withShadow(p, [&](const QColor &c, double) {
        fillPolygon(p, c, {{32, 3}, {25, 16}, {39, 16}});
        const double x = 32 + m_skid;
        fillPolygon(p, c, {{x - 8, 20}, {x + 8, 20}, {x + 10, 27}, {x - 10, 27}});
    });
}

// ── Bank scale (the fixed half of drawBankArc) ─────────────────────────────

void HudBankScale::sync(const HudFrame &f)
{
    setTint(f.color);
    m_radius = std::round(std::min(180.0, f.H * 0.28));
    resizeTo(2 * (m_radius + 20), m_radius / 2 + 28);
    place(std::round(f.W / 2) - m_radius - 20, std::round(f.H / 2) - m_radius - 20);
    repaintIf(HudKey() << qint64(m_radius));
}

void HudBankScale::draw(QPainter *p)
{
    const double r = m_radius;
    withShadow(p, [&](const QColor &c, double lw) {
        p->save();
        p->translate(r + 20, r + 20);
        p->setPen(hudPen(c, lw));
        for (int t : {-60, -45, -30, -20, -10, 10, 20, 30, 45, 60}) {
            const double len = std::abs(t) % 30 == 0 ? 14 : std::abs(t) == 45 ? 10 : 7;
            p->save();
            p->rotate(t);
            p->drawLine(QPointF(0, -r), QPointF(0, -r - len));
            p->restore();
        }
        // Zero reference, apex down, outside the arc
        const QPointF zero[3] = {{0, -r - 2}, {-7, -r - 15}, {7, -r - 15}};
        p->drawPolygon(zero, 3);
        p->restore();
    });
}

// ── Flight path marker and speed error worm (drawFlightPath, drawSpeedError) ─

void HudFpm::sync(const HudFrame &f)
{
    const VehicleState &s = *f.s;
    setTint(f.color);
    // Air-relative in both modes: its offset from the horizon stays aoa - pitch
    double x = f.projectAngle(s.ssa), y = f.projectAngle(s.aoa - f.pose.pitch + f.pitchShift);
    m_caged = std::abs(x) > f.cageX || std::abs(y) > f.cageY;
    x = std::clamp(x, -f.cageX, f.cageX);
    y = std::clamp(y, -f.cageY, f.cageY);
    m_worm = 0;
    if (f.navFresh) {
        const double len = std::clamp(-s.aspdError * 6, -40.0, 40.0);   // grows up when fast
        if (std::abs(len) >= 4) m_worm = std::round(len);
    }
    resizeTo(64, 112);
    place(f.W / 2 + x - 32, f.H / 2 + y - 56);
    repaintIf(HudKey() << qint64(m_caged) << qint64(m_worm));
}

void HudFpm::draw(QPainter *p)
{
    const double cx = 32, cy = 56;
    withShadow(p, [&](const QColor &c, double lw) {
        p->setPen(m_caged ? hudPen(c, lw, {4, 4}) : hudPen(c, lw));
        const double r = 8, wing = 12, fin = 8;
        QPainterPath path;
        path.addEllipse(QPointF(cx, cy), r, r);
        path.moveTo(cx + r, cy); path.lineTo(cx + r + wing, cy);
        path.moveTo(cx - r, cy); path.lineTo(cx - r - wing, cy);
        path.moveTo(cx, cy - r); path.lineTo(cx, cy - r - fin);
        p->drawPath(path);
    });
    if (m_worm != 0)
        withShadow(p, [&](const QColor &c, double) {
            p->setPen(hudPen(c, kLineWidth + 2));
            p->drawLine(QPointF(cx - 20, cy), QPointF(cx - 20, cy - m_worm));
        });
}

// ── Ground-track diamond (drawGroundTrackMarker) ───────────────────────────

void HudTrackMarker::sync(const HudFrame &f)
{
    const VehicleState &s = *f.s;
    setTint(f.color);
    // GPS-only trajectory: track vs heading, flight-path angle vs pitch
    double x = f.projectAngle(wrapPi(s.track - f.pose.yaw)), y = f.projectAngle(f.pitchShift - s.gamma);
    m_caged = std::abs(x) > f.cageX || std::abs(y) > f.cageY;
    x = std::clamp(x, -f.cageX, f.cageX);
    y = std::clamp(y, -f.cageY, f.cageY);
    resizeTo(24, 24);
    place(f.W / 2 + x - 12, f.H / 2 + y - 12);
    repaintIf(HudKey() << qint64(m_caged));
}

void HudTrackMarker::draw(QPainter *p)
{
    withShadow(p, [&](const QColor &c, double lw) {
        p->setPen(m_caged ? hudPen(c, lw, {4, 4}) : hudPen(c, lw));
        const double d = 7;
        const QPointF q[4] = {{12, 12 - d}, {12 + d, 12}, {12, 12 + d}, {12 - d, 12}};
        p->drawPolygon(q, 4);
    });
}

// ── Boresight (drawBoresight): fixed, or moving with the pitch in horizon lock ─

void HudBoresight::sync(const HudFrame &f)
{
    setTint(f.color);
    double y = f.pitchLocked ? -f.projectAngle(f.pose.pitch) : 0;
    m_caged = f.pitchLocked && std::abs(y) > f.cageY;
    y = std::clamp(y, -f.cageY, f.cageY);
    resizeTo(84, 32);
    place(f.W / 2 - 42, f.H / 2 + y - 10);
    repaintIf(HudKey() << qint64(m_caged));
}

void HudBoresight::draw(QPainter *p)
{
    withShadow(p, [&](const QColor &c, double lw) {
        p->setPen(m_caged ? hudPen(c, lw, {4, 4}) : hudPen(c, lw));
        const double cx = 42, y = 10, wingLen = 25, gap = 8, drop = 12;
        QPainterPath path;
        path.addEllipse(QPointF(cx, y), 3, 3);
        path.moveTo(cx - gap, y); path.lineTo(cx - gap - wingLen, y); path.lineTo(cx - gap - wingLen, y + drop);
        path.moveTo(cx + gap, y); path.lineTo(cx + gap + wingLen, y); path.lineTo(cx + gap + wingLen, y + drop);
        p->drawPath(path);
    });
}

// ── Heading tape (drawHeading), top centre ─────────────────────────────────
// The box with the heading repaints when the number changes; the scale under
// it is a strip painted once and slid by the scene graph inside a clip of
// 61 degrees, the ground-track diamond moves over it.

namespace {
struct HeadingBox {
    double textWidth, length, height;
};
HeadingBox headingBox()
{
    const double fontSize = 20, textSide = 5, textTop = 4;
    const double textWidth = QFontMetricsF(hudFont(fontSize)).horizontalAdvance(QStringLiteral("360"));
    const double length = textSide * 2 + textWidth;
    return {textWidth, length, textTop * 1.5 + fontSize + length / 4};
}
constexpr double kScaleFont = 16, kScaleTextBorder = 2;
constexpr double kStepLen[3] = {16, 11, 7};
constexpr int kStripFrom = -40, kStripTo = 400;   // degrees painted on the strip
constexpr double kStripMargin = 24;               // px each side, for the labels and the shadow

/// Heading, degrees in [0, 360), rounded to one pixel of the tape.
double tapeHeading(const HudFrame &f)
{
    double v = std::round(f.pose.yaw * kRad * kStepWidth) / kStepWidth;
    v = std::fmod(v, 360.0);
    return v < 0 ? v + 360 : v;
}
} // namespace

double HudHeadingTape::scaleTop()
{
    static const double top = 16 + headingBox().height + 4;
    return top;
}

QSizeF HudHeadingTape::scaleSize() { return {kTapeSteps * kStepWidth, kStepLen[0] + 2 * kScaleTextBorder + kScaleFont}; }

void HudHeadingTape::sync(const HudFrame &f)
{
    setTint(f.color);
    int hdg = int(std::lround(tapeHeading(f)));
    m_heading = ((hdg % 360) + 360) % 360;
    const HeadingBox box = headingBox();
    resizeTo(box.length + 8, box.height + 8);
    place(f.W / 2 - box.length / 2 - 4, 16 - 4);   // local (length/2 + 4, 4) is (centre, 16)
    repaintIf(HudKey() << qint64(m_heading));
}

void HudHeadingTape::draw(QPainter *p)
{
    const HeadingBox box = headingBox();
    withShadow(p, [&](const QColor &c, double lw) {
        p->save();
        p->translate(box.length / 2 + 4, 4);
        const double fontSize = 20, textTop = 4;
        p->setPen(hudPen(c, lw));
        const QPointF outline[5] = {{-box.length / 2, 0}, {box.length / 2, 0}, {box.length / 2, textTop * 1.5 + fontSize}, {0, box.height},
                                    {-box.length / 2, textTop * 1.5 + fontSize}};
        p->drawPolygon(outline, 5);
        p->setPen(c);
        p->setFont(hudFont(fontSize));
        textAt(p, box.textWidth / 2, (2 * textTop + fontSize) / 2, QString::number(m_heading), Qt::AlignRight, Qt::AlignVCenter);
        p->restore();
    });
}

void HudHeadingStrip::sync(const HudFrame &f)
{
    setTint(f.color);
    resizeTo((kStripTo - kStripFrom) * kStepWidth + 2 * kStripMargin, HudHeadingTape::scaleSize().height() + 4);
    // In the clip (origin on the scale's left edge, the heading under its centre)
    const double centre = HudHeadingTape::scaleSize().width() / 2;
    place(centre - kStripMargin - (tapeHeading(f) - kStripFrom) * kStepWidth, 0);
    repaintIf(HudKey());
}

void HudHeadingStrip::draw(QPainter *p)
{
    static const QHash<int, QString> card = {{0, QStringLiteral("N")}, {45, QStringLiteral("NE")}, {90, QStringLiteral("E")},
                                            {135, QStringLiteral("SE")}, {180, QStringLiteral("S")}, {225, QStringLiteral("SW")},
                                            {270, QStringLiteral("W")}, {315, QStringLiteral("NW")}};
    withShadow(p, [&](const QColor &c, double lw) {
        p->setFont(hudFont(kScaleFont));
        p->setPen(c);
        QPainterPath ticks;
        for (int i = kStripFrom; i <= kStripTo; ++i) {
            const double x = kStripMargin + (i - kStripFrom) * kStepWidth;
            const int posI = std::abs(i);
            const double len = posI % 10 == 0 ? kStepLen[0] : posI % 5 == 0 ? kStepLen[1] : kStepLen[2];
            ticks.moveTo(x, 0);
            ticks.lineTo(x, len);
            if (posI % 10 == 0 || posI % 45 == 0) {
                const int mod = ((i % 360) + 360) % 360;
                textAt(p, x, kStepLen[0] + kScaleTextBorder + kScaleFont / 2, card.value(mod, QString::number(mod)), Qt::AlignHCenter,
                       Qt::AlignVCenter);
            }
        }
        p->setPen(hudPen(c, lw));
        p->drawPath(ticks);
    });
}

void HudTapeDiamond::sync(const HudFrame &f)
{
    setTint(f.color);
    // Drift / crab: the track against the heading, on the scale
    const double tx = wrapDeg180(f.s->track * kRad - tapeHeading(f)) * kStepWidth;
    resizeTo(16, 20);
    place(HudHeadingTape::scaleSize().width() / 2 + tx - 7, 0);
    repaintIf(HudKey());
}

void HudTapeDiamond::draw(QPainter *p)
{
    withShadow(p, [&](const QColor &c, double lw) {
        p->setPen(hudPen(c, lw));
        const QPointF d[4] = {{7, 2}, {12, 8}, {7, 14}, {2, 8}};
        p->drawPolygon(d, 4);
    });
}

// ── Nav bearing bug: the cyan caret on the tape while guidance is active ───

void HudNavBug::sync(const HudFrame &f)
{
    const VehicleState &s = *f.s;
    const double bx = wrapDeg180(s.navBearing - tapeHeading(f)) * kStepWidth;
    // Clipped with the tape it sits on
    setVisible(f.navFresh && s.wpDist > 0 && std::abs(bx) <= kTapeSteps * kStepWidth / 2);
    resizeTo(20, 20);
    place(f.W / 2 + bx - 8, HudHeadingTape::scaleTop());
    repaintIf(HudKey());
}

void HudNavBug::draw(QPainter *p)
{
    p->save();
    p->translate(kShadowOffset, kShadowOffset);
    fillPolygon(p, kShadow, {{8, 2}, {2, 13}, {14, 13}});
    p->restore();
    fillPolygon(p, kCyan, {{8, 2}, {2, 13}, {14, 13}});
}

// ── VSI (drawVSI, drawVertTape) ─────────────────────────────────────────────

namespace {
/// Non-linear tape: +-5 m/s over 72 % of the scale, 5-10 m/s compressed.
double varioY(double v, double tapeH)
{
    constexpr double maxV = 10;
    const double a = std::min(std::abs(v), maxV);
    const double frac = a <= 5 ? a / 5 * 0.72 : 0.72 + (a - 5) / 5 * 0.28;
    return -(v > 0 ? 1 : v < 0 ? -1 : 0) * frac * tapeH;
}
} // namespace

void HudVario::sync(const HudFrame &f)
{
    setTint(f.color);
    const double cx = f.W / 2, cy = f.H / 2;
    const double x = std::min(cx + 320, f.W - 150);
    m_tapeH = std::round(std::min(110.0, f.H * 0.16));
    const double vs = std::isfinite(f.s->vs) ? f.s->vs : 0;
    m_vs = std::round(vs * 10) / 10;
    m_vsY = std::round(varioY(vs, m_tapeH));
    resizeTo(40 + 72, 2 * m_tapeH + 48);
    place(x - 40, std::round(cy) - m_tapeH - 24);
    repaintIf(HudKey() << qint64(m_tapeH) << qint64(m_vsY) << std::llround(m_vs * 10));
}

void HudVario::draw(QPainter *p)
{
    const double H = m_tapeH, cy = H + 24;
    // One tape, named VS on top, its unit underneath (the total-energy tape
    // that stood beside it is gone)
    auto tape = [&](const QColor &c, double lw, double x, double value, double vy, const QString &top, bool unit) {
        p->save();
        p->translate(x, cy);
        p->setPen(hudPen(c, lw));
        QPainterPath path;
        path.moveTo(0, -H); path.lineTo(0, H);
        for (double v : {-10.0, -5.0, -2.0, -1.0, 1.0, 2.0, 5.0, 10.0}) {
            path.moveTo(0, varioY(v, H));
            path.lineTo(std::abs(v) >= 5 ? -10 : -6, varioY(v, H));
        }
        path.moveTo(0, 0); path.lineTo(-12, 0);
        p->drawPath(path);
        p->setFont(hudFont(12));
        p->setPen(c);
        textAt(p, -14, varioY(5, H), QStringLiteral("5"), Qt::AlignRight, Qt::AlignVCenter);
        textAt(p, -14, varioY(-5, H), QStringLiteral("5"), Qt::AlignRight, Qt::AlignVCenter);
        fillPolygon(p, c, {{0, vy}, {9, vy - 5}, {9, vy + 5}});
        p->setPen(c);
        p->setFont(hudFont(14));
        textAt(p, 13, vy, QString::number(value, 'f', 1), Qt::AlignLeft, Qt::AlignVCenter);
        p->setFont(hudFont(10));
        if (!top.isEmpty()) textAt(p, 0, -H - 6, top, Qt::AlignHCenter, Qt::AlignBottom);
        if (unit) textAt(p, 0, H + 6, QStringLiteral("M/S"), Qt::AlignHCenter, Qt::AlignTop);
        p->restore();
    };
    withShadow(p, [&](const QColor &c, double lw) {
        tape(c, lw, 40, m_vs, m_vsY, QStringLiteral("VS"), true);
    });
}

// ── Flight mode annunciator, waypoint line and wind (drawFMA, drawWind) ─────

void HudFma::sync(const HudFrame &f)
{
    const VehicleState &s = *f.s;
    setTint(f.color);
    const double cx = std::round(f.W / 2), windX = std::max(20.0, cx - 290);
    m_mode = s.flightMode.isEmpty() ? QStringLiteral("---") : s.flightMode;
    m_wp.clear();
    if (f.navFresh && s.wpDist > 0) {
        const double xtk = s.xtrackError;
        m_wp = QStringLiteral("WP %1m   XTK %2%3m").arg(std::lround(s.wpDist)).arg(xtk >= 0 ? QStringLiteral("+") : QString()).arg(xtk, 0, 'f', 0);
    }
    const qint64 now = wallMs();
    m_wind = s.windDataTime && now - s.windDataTime <= 5000 && std::isfinite(s.windSpeed) && s.windSpeed >= 0.5;
    if (m_wind) {
        // windDir is where the wind comes from; the arrow points where it goes
        m_windRot = std::fmod(std::round(s.windDir + 180 - f.pose.yaw * kRad), 360.0);
        m_windText = QStringLiteral("%1 M/S").arg(s.windSpeed, 0, 'f', 1);
    }
    const double x0 = std::min(windX - 24, cx - 240);
    resizeTo(cx + 240 - x0, 60);
    place(x0, 88);
    m_cx = cx - x0;
    m_windX = windX - x0;
    repaintIf(HudKey() << m_mode << m_wp << qint64(m_wind) << qint64(m_windRot) << m_windText << qint64(m_cx));
}

void HudFma::draw(QPainter *p)
{
    // Local y: the FMA at 96 of the HUD is 8 here, the waypoint line 28, the wind 16
    withShadow(p, [&](const QColor &c, double) {
        p->setPen(c);
        p->setFont(hudFont(15));
        textAt(p, m_cx, 8, m_mode, Qt::AlignHCenter, Qt::AlignTop);
        if (!m_wp.isEmpty()) {
            QColor dim = c;
            dim.setAlphaF(c.alphaF() * 0.85);
            p->setPen(dim);
            p->setFont(hudFont(12));
            textAt(p, m_cx, 28, m_wp, Qt::AlignHCenter, Qt::AlignTop);
        }
    });
    if (!m_wind) return;
    withShadow(p, [&](const QColor &c, double lw) {
        p->save();
        p->translate(m_windX, 16);
        p->rotate(m_windRot);
        p->setPen(hudPen(c, lw));
        QPainterPath a;
        a.moveTo(0, 10); a.lineTo(0, -10);
        a.moveTo(-4, -5); a.lineTo(0, -10); a.lineTo(4, -5);
        p->drawPath(a);
        p->restore();
        p->setPen(c);
        p->setFont(hudFont(12));
        textAt(p, m_windX + 12, 16, m_windText, Qt::AlignLeft, Qt::AlignVCenter);
    });
}

// ── Message list, bottom-left (drawHudMessages), in its own colours ────────

HudMessages::HudMessages(HudItem *hud, QQuickItem *parent) : HudInstrument(hud, parent) { m_clock.start(); }

void HudMessages::push(const QString &text, const QString &level)
{
    m_msgs.push_back({text, level, m_clock.elapsed()});
    if (m_msgs.size() > kMaxMsgs) m_msgs.pop_front();
}

void HudMessages::sync(const HudFrame &f)
{
    const qint64 now = m_clock.elapsed();
    while (!m_msgs.empty() && now - m_msgs.front().t > kMsgDurationMs) m_msgs.pop_front();
    m_fontPx = std::clamp(f.W * 0.009, 8.0, 10.0) * 1.3;
    HudKey k;
    k << std::llround(m_fontPx * 10);
    for (Msg &m : m_msgs) {
        const qint64 age = now - m.t;
        // Fade out over the last second, in 32 steps
        const double a = age > kMsgDurationMs - 1000 ? std::max(0.0, (kMsgDurationMs - age) / 1000.0) : 1.0;
        m.alpha = std::round(a * 32) / 32;
        k << m.text << m.level << qint64(m.alpha * 32);
    }
    const double h = kMaxMsgs * (m_fontPx + 6) + 12;
    resizeTo(std::max(240.0, f.W * 0.6), h);
    // The newest line's baseline sits messagesBottom above the HUD's bottom edge
    place(0, f.H - m_hud->messagesBottom() + 4 - h);
    repaintIf(k);
}

void HudMessages::draw(QPainter *p)
{
    if (m_msgs.empty()) return;
    const double fs = m_fontPx;
    p->setFont(hudFont(fs));
    const QFontMetricsF fm(p->font());
    double y = height() - 4;
    for (auto it = m_msgs.rbegin(); it != m_msgs.rend(); ++it) {
        const double a = it->alpha;
        QColor col = it->level == QLatin1String("error") ? QColor(255, 60, 60)
                     : it->level == QLatin1String("warning") ? QColor(255, 200, 50) : QColor(200, 255, 200);
        col.setAlphaF(a);
        p->fillRect(QRectF(12 - 4, y - fs - 2, fm.horizontalAdvance(it->text) + 8, fs + 4), QColor(0, 0, 0, int(102 * a)));
        p->setPen(col);
        textAt(p, 12, y, it->text, Qt::AlignLeft, Qt::AlignBottom);
        y -= fs + 6;
    }
}

} // namespace corv
