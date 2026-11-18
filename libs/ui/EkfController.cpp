#include "ui/EkfController.h"

#include "core/Clock.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "vehicle/CommandSender.h"
#include "vehicle/VehicleHub.h"

#include <QCursor>
#include <QFontMetricsF>
#include <QHoverEvent>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace corv {

EkfController *EkfController::s_instance = nullptr;

namespace {

const char *const kEnabledKey = "ekf-status-enabled";
constexpr int kDrawMs = 50;         // charts at 20 Hz
constexpr int kTextEvery = 4;       // legends, chips and flags at 5 Hz
constexpr double kGapMs = 2500;     // samples further apart are not joined
constexpr int kArduPilot = 3;       // MAV_AUTOPILOT_ARDUPILOTMEGA
const std::pair<std::uint32_t, double> kApRates[] = {{193, 5}, {178, 5}, {24, 5}};   // EKF_STATUS_REPORT, AHRS2, GPS_RAW_INT

// Series colours: a series keeps its colour; status colours only for thresholds and flags
const char *const kSeriesColors[] = {"#3987e5", "#d95926", "#199e70", "#c98500", "#d55181"};
QColor statusColor(const char *s)
{
    if (!s) return QColor(0xc3, 0xc2, 0xb7);
    if (!std::strcmp(s, "good")) return QColor(0x0c, 0xa3, 0x0c);
    if (!std::strcmp(s, "warning")) return QColor(0xfa, 0xb2, 0x19);
    return QColor(0xd0, 0x3b, 0x3b);
}
const QColor kInkSecondary(0xc3, 0xc2, 0xb7), kInkMuted(0x89, 0x87, 0x81);
const QColor kGrid(255, 255, 255, 18), kAxis(255, 255, 255, 56);

const char *const kFixNames[] = {"NO GPS", "NO FIX", "2D", "3D", "DGPS", "RTK FLT", "RTK FIX", "STATIC", "PPP"};

QString fmt(std::optional<double> v, int digits) { return v && std::isfinite(*v) ? QString::number(*v, 'f', digits) : QStringLiteral("—"); }
QString fmtInt(std::optional<int> v) { return v ? QString::number(*v) : QStringLiteral("—"); }
QVariantMap chip(const QString &k, const QString &v, const QString &status = {})
{
    return {{QStringLiteral("k"), k}, {QStringLiteral("v"), v}, {QStringLiteral("status"), status}};
}

double niceCeil(double v)
{
    const double p = std::pow(10, std::floor(std::log10(v)));
    for (double m : {1.0, 2.0, 2.5, 5.0, 10.0})
        if (m * p >= v - 1e-12) return m * p;
    return 10 * p;
}

constexpr double kPadL = 30, kPadT = 6, kPadB = 13;

QFont uiFont(double px)
{
    QFont f(QStringLiteral("Rajdhani"));
    f.setPixelSize(int(px));
    f.setWeight(QFont::DemiBold);
    return f;
}
QFont dataFont(double px)
{
    QFont f(QStringLiteral("Roboto Mono"));
    f.setPixelSize(int(px));
    return f;
}

} // namespace

EkfController::EkfController(MessageBus *bus, LinkManager *link, CommandSender *cmd, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_bus(bus), m_cmd(cmd), m_hub(hub)
{
    s_instance = this;
    m_enabled = Settings::instance().value(QLatin1String(kEnabledKey)).toString() == QLatin1String("1");
    for (std::uint32_t id : {193u, 24u, 30u, 178u, 241u})
        m_bus->subscribe(id, this, [this](const MavMessage &m) { m_monitor.ingestMavlink(m.msg, monoMs()); });
    m_draw.setInterval(kDrawMs);
    connect(&m_draw, &PacedTimer::timeout, this, &EkfController::tickDraw);
    m_rates.setInterval(1000);
    connect(&m_rates, &QTimer::timeout, this, [this] { manageRates(monoMs()); });
    m_rates.start();
    if (m_enabled) m_draw.start();
    updateText(monoMs());
}

void EkfController::setEnabled(bool on)
{
    if (on == m_enabled) return;
    m_enabled = on;
    Settings::instance().setValue(QLatin1String(kEnabledKey), on ? QStringLiteral("1") : QStringLiteral("0"));
    if (m_enabled && m_shown) m_draw.start();
    else m_draw.stop();
    manageRates(monoMs());
    emit enabledChanged();
}

void EkfController::setShown(bool on)
{
    if (on == m_shown) return;
    m_shown = on;
    if (m_enabled && m_shown) m_draw.start();
    else m_draw.stop();
    emit shownChanged();
}

void EkfController::reset()
{
    m_monitor.reset();
    m_ratesRequested = false;
    updateText(monoMs());
}

/// 5 Hz EKF messages from an ArduPilot while the bar is open, the defaults back after.
void EkfController::manageRates(double now)
{
    const auto s = m_hub->snapshot();
    const bool ardupilot = s->connected && s->autopilotType == kArduPilot && wallMs() - s->lastHeartbeatTime < 3000;
    if (!ardupilot) {
        if (!s->connected) m_ratesRequested = false;
        return;
    }
    if (m_enabled && !m_ratesRequested) {
        m_ratesRequested = true;
        for (const auto &[id, hz] : kApRates) m_cmd->setMessageInterval(id, hz);
    } else if (!m_enabled && m_ratesRequested) {
        m_ratesRequested = false;
        for (const auto &[id, hz] : kApRates) m_cmd->resetMessageInterval(id);
    }
}

void EkfController::tickDraw()
{
    const double now = monoMs();
    const QString src = m_monitor.source(now);
    if (!src.isEmpty() && src != m_cardSource) m_cardSource = src;
    emit redraw();
    if (++m_drawCount % kTextEvery == 0) updateText(now);
}

// ── Chart specs ──────────────────────────────────────────────────────────

const std::vector<EkfController::Spec> &EkfController::specs() const
{
    static const std::vector<Spec> ap = [] {
        std::vector<Spec> v(3);
        v[0].title = "EKF INNOVATIONS";
        v[0].unit = "test ratio · 1 = measurement rejected";
        v[0].min = 0;
        v[0].floor = 1;
        v[0].digits = 2;
        v[0].lines = {{"apVel", "VEL"}, {"apPosH", "POS H"}, {"apPosV", "POS V"}, {"apMag", "MAG"}, {"apTas", "TAS", true}};
        v[0].marks = {{0.5, "warning", "0.5"}, {0.8, "critical", "0.8"}};
        v[0].chips = [](const EkfMonitor &m) { return QVariantList{chip(QStringLiteral("TERR"), fmt(m.ap().terr, 2))}; };

        v[1].title = "GPS ACCURACY";
        v[1].unit = "H, V in m · SPEED in m/s";
        v[1].min = 0;
        v[1].floor = 1;
        v[1].digits = 2;
        v[1].lines = {{"apHAcc", "H"}, {"apVAcc", "V"}, {"apVelAcc", "SPEED", true}};
        // Receivers that report no accuracy: the dilution of precision instead
        v[1].fallback = std::make_pair("HDOP · the receiver reports no accuracy", std::make_pair(2.0, std::vector<Line>{{"apHdop", "HDOP"}}));
        v[1].chips = [](const EkfMonitor &m) {
            const auto fix = m.ap().fix;
            return QVariantList{chip(QStringLiteral("FIX"), fix && *fix >= 0 && *fix < 9 ? QLatin1String(kFixNames[*fix]) : QStringLiteral("—")),
                                chip(QStringLiteral("SATS"), fmtInt(m.ap().sats)), chip(QStringLiteral("HDOP"), fmt(m.ap().hdop, 1))};
        };

        // AHRS2 is the estimator not in use: the DCM while the EKF flies
        v[2].title = "ATTITUDE  EKF vs DCM";
        v[2].unit = "° · active estimator − backup (AHRS2)";
        v[2].symmetric = true;
        v[2].floor = 2;
        v[2].digits = 2;
        v[2].lines = {{"apDRoll", "ROLL"}, {"apDPitch", "PITCH"}, {"apDYaw", "YAW"}};
        v[2].chips = [](const EkfMonitor &m) {
            const auto &vibe = m.ap().vibe;
            const double worst = vibe ? std::max({(*vibe)[0], (*vibe)[1], (*vibe)[2]}) : 0;
            const QString vt = vibe ? QStringLiteral("%1/%2/%3").arg((*vibe)[0], 0, 'f', 0).arg((*vibe)[1], 0, 'f', 0).arg((*vibe)[2], 0, 'f', 0) : QStringLiteral("—");
            const auto clip = m.ap().clip;
            return QVariantList{chip(QStringLiteral("VIBE"), vt, worst > 60 ? QStringLiteral("critical") : worst > 30 ? QStringLiteral("warning") : QString()),
                                chip(QStringLiteral("CLIP"), fmtInt(clip), clip && *clip > 0 ? QStringLiteral("warning") : QString())};
        };
        return v;
    }();
    return ap;
}

/// Lines to draw: the main set, or the fallback when none of it has data.
EkfController::Active EkfController::activeLines(const Spec &s) const
{
    const auto has = [&](const Line &l) { return !m_monitor.series(QLatin1String(l.key)).empty(); };
    Active a;
    for (const Line &l : s.lines)
        if (!l.optional || has(l)) a.lines.push_back(l);
    a.unit = QString::fromUtf8(s.unit);
    a.floor = s.floor;
    if (s.fallback && std::none_of(a.lines.begin(), a.lines.end(), has)
        && std::any_of(s.fallback->second.second.begin(), s.fallback->second.second.end(), has)) {
        a.lines = s.fallback->second.second;
        a.unit = QString::fromUtf8(s.fallback->first);
        a.floor = s.fallback->second.first;
    }
    return a;
}

QString EkfController::seriesColor(const Spec &s, const Line &l, int i) const
{
    for (std::size_t k = 0; k < s.lines.size(); ++k)
        if (!std::strcmp(s.lines[k].key, l.key)) return QLatin1String(kSeriesColors[k % 5]);
    return QLatin1String(kSeriesColors[i % 5]);
}

// ── Texts ────────────────────────────────────────────────────────────────

void EkfController::updateText(double now)
{
    const QString source = m_monitor.source(now);
    static const QHash<QString, QString> labels = {{QStringLiteral("ardupilot"), QStringLiteral("ARDUPILOT EKF3")}};
    m_sourceText = source.isEmpty() ? QStringLiteral("WAITING FOR ESTIMATOR DATA")
                                    : labels.value(source) + (m_monitor.live(now) ? QString() : QStringLiteral(" · NO DATA"));

    QVariantList flags;
    const auto add = [&](const QString &label, const QString &state) { flags << QVariantMap{{QStringLiteral("label"), label}, {QStringLiteral("state"), state}}; };
    if (source == QLatin1String("ardupilot") && m_monitor.ap().flags) {
        const int f = *m_monitor.ap().flags;
        const auto ok = [&](int bit, const char *label) { add(QLatin1String(label), f & bit ? QStringLiteral("good") : QStringLiteral("critical")); };
        ok(ekfflags::Attitude, "ATT");
        ok(ekfflags::VelocityHoriz, "VEL H");
        ok(ekfflags::VelocityVert, "VEL V");
        add(QStringLiteral("POS H"), f & (ekfflags::PosHorizAbs | ekfflags::PosHorizRel) ? QStringLiteral("good") : QStringLiteral("critical"));
        ok(ekfflags::PosVertAbs, "POS V");
        if (f & ekfflags::ConstPosMode) add(QStringLiteral("CONST POS"), QStringLiteral("warning"));
        if (f & ekfflags::GpsGlitching) add(QStringLiteral("GPS GLITCH"), QStringLiteral("critical"));
        if (f & ekfflags::Uninitialized) add(QStringLiteral("UNINITIALISED"), QStringLiteral("critical"));
    }
    m_flags = flags;

    QVariantList cards;
    const auto &sp = specs();
    for (std::size_t c = 0; c < sp.size(); ++c) {
        const Spec &s = sp[c];
        const Active a = activeLines(s);
        // Legend: values at the pointer, else the latest
        const double padR = a.lines.size() < 2 ? 10 : 80;
        const double pw = m_hoverW[c] - kPadL - padR;
        const std::optional<double> t = m_hoverX[c] >= 0 && pw > 0 ? std::optional<double>(now - kEkfWindowMs + (m_hoverX[c] - kPadL) / pw * kEkfWindowMs)
                                                                     : std::nullopt;
        QVariantList legend;
        for (std::size_t i = 0; i < a.lines.size(); ++i) {
            const EkfSeries &ser = m_monitor.series(QLatin1String(a.lines[i].key));
            std::optional<double> v;
            if (t) v = ser.at(*t);
            else if (ser.lastTime() && *ser.lastTime() >= now - kEkfWindowMs) v = ser.last();
            const QString text = s.log && v ? QString::number(*v, 'e', 2) : fmt(v, s.digits);
            legend << QVariantMap{{QStringLiteral("name"), QString::fromUtf8(a.lines[i].name)},
                                  {QStringLiteral("color"), seriesColor(s, a.lines[i], int(i))},
                                  {QStringLiteral("value"), text}};
        }
        cards << QVariantMap{{QStringLiteral("title"), QString::fromUtf8(s.title)}, {QStringLiteral("unit"), a.unit},
                             {QStringLiteral("chips"), s.chips(m_monitor)}, {QStringLiteral("legend"), legend}};
    }
    m_cards = cards;
    emit textChanged();
}

void EkfController::setHover(int card, double x, double w)
{
    if (card < 0 || card > 2) return;
    m_hoverX[std::size_t(card)] = x;
    m_hoverW[std::size_t(card)] = w;
}

// ── Chart ────────────────────────────────────────────────────────────────

void EkfController::drawCard(QPainter *p, int card, double w, double h, double hoverX) const
{
    const auto &sp = specs();
    if (card < 0 || card >= int(sp.size()) || w <= 0 || h <= 0) return;
    const Spec &spec = sp[std::size_t(card)];
    const double now = monoMs();
    const Active a = activeLines(spec);
    const auto &lines = a.lines;
    p->setRenderHint(QPainter::Antialiasing, true);

    // Right margin: room for the series names at the line ends (none for a single series)
    double padR = 10;
    const QFont endFont = uiFont(10);
    if (lines.size() >= 2) {
        const QFontMetricsF fm(endFont);
        double widest = 0;
        for (const Line &l : lines) widest = std::max(widest, fm.horizontalAdvance(QString::fromUtf8(l.name)));
        padR = std::min(80.0, std::ceil(widest) + 12);
    }
    const double pw = w - kPadL - padR, ph = h - kPadT - kPadB;
    if (pw < 20 || ph < 20) return;
    const double t0 = now - kEkfWindowMs;
    const auto X = [&](double t) { return kPadL + (t - t0) / kEkfWindowMs * pw; };

    // Range over what is on screen
    double mx = -INFINITY, mn = INFINITY;
    for (const Line &l : lines) {
        const EkfSeries &s = m_monitor.series(QLatin1String(l.key));
        for (int i = 0; i < s.size(); ++i)
            if (s.t(i) >= t0) {
                const double v = spec.symmetric ? std::abs(s.v(i)) : s.v(i);
                mx = std::max(mx, v);
                mn = std::min(mn, v);
            }
    }
    double lo, hi;
    std::function<double(double)> scale = [](double v) { return v; };
    if (spec.log) {
        scale = [](double v) { return std::log10(std::max(v, 1e-12)); };
        const double ea = std::isfinite(mn) ? std::floor(scale(mn)) : -3, eb = std::isfinite(mx) ? std::ceil(scale(mx)) : 0;
        lo = std::pow(10, ea);
        hi = std::pow(10, eb > ea ? eb : ea + 1);
    } else if (spec.symmetric) {
        hi = niceCeil(std::max(a.floor, std::isfinite(mx) ? mx * 1.15 : 0));
        lo = -hi;
    } else {
        lo = spec.min.value_or(0);
        hi = spec.max ? *spec.max : niceCeil(std::max(a.floor, std::isfinite(mx) ? mx * 1.15 : 0));
    }
    const auto Y = [&](double v) { return kPadT + ph - (scale(std::clamp(v, lo, hi)) - scale(lo)) / (scale(hi) - scale(lo)) * ph; };

    // Grid and y labels: recessive
    p->setFont(dataFont(10));
    std::vector<double> ticks;
    if (spec.log) {
        for (int e = int(std::lround(std::log10(lo))); e <= int(std::lround(std::log10(hi))); ++e) ticks.push_back(std::pow(10, e));
        if (ticks.size() > 4) ticks = {ticks.front(), ticks[ticks.size() / 2], ticks.back()};
    } else if (spec.symmetric) {
        ticks = {lo, 0, hi};
    } else {
        ticks = {lo, (lo + hi) / 2, hi};
    }
    const auto tickLabel = [&](double v) {
        if (spec.log) {
            const int e = int(std::lround(std::log10(v)));
            return QStringLiteral("1e%1").arg(e);
        }
        if (std::abs(v) >= 10 || v == 0) return QString::number(v, 'f', 0);
        return QString::number(std::round(v * 100) / 100, 'g', 6);
    };
    for (double v : ticks) {
        bool marked = false;
        for (const Mark &m : spec.marks) marked |= m.v == v;
        if (marked) continue;   // the threshold line carries its own label
        const double y = std::round(Y(v)) + 0.5;
        p->setPen(QPen(v == 0 || v == lo ? kAxis : kGrid, 1));
        p->drawLine(QPointF(kPadL, y), QPointF(kPadL + pw, y));
        p->setPen(kInkMuted);
        p->drawText(QRectF(0, y - 7, kPadL - 4, 14), Qt::AlignRight | Qt::AlignVCenter, tickLabel(v));
    }
    // Time labels
    p->setPen(kInkMuted);
    p->drawText(QRectF(kPadL, h - 13, 60, 13), Qt::AlignLeft | Qt::AlignBottom, QStringLiteral("−20 s"));
    p->drawText(QRectF(kPadL + pw / 2 - 30, h - 13, 60, 13), Qt::AlignHCenter | Qt::AlignBottom, QStringLiteral("−10 s"));
    p->drawText(QRectF(kPadL + pw - 60, h - 13, 60, 13), Qt::AlignRight | Qt::AlignBottom, QStringLiteral("now"));

    // Thresholds: dashed, labelled; status colour only where the level is a state
    double lastLabelY = INFINITY;
    for (const Mark &m : spec.marks) {
        if (m.v < lo || m.v > hi) continue;
        const double y = std::round(Y(m.v)) + 0.5;
        QColor c = statusColor(m.status);
        c.setAlphaF(0.8f);
        QPen pen(c, 1);
        pen.setDashPattern({4, 3});
        p->setPen(pen);
        p->drawLine(QPointF(kPadL, y), QPointF(kPadL + pw, y));
        if (std::abs(lastLabelY - y) < 10) continue;   // squeezed together: the lower label alone
        lastLabelY = y;
        const QString label = QString::fromUtf8(m.label);
        const double tw = QFontMetricsF(p->font()).horizontalAdvance(label);
        p->fillRect(QRectF(kPadL + 1, y - 11, tw + 4, 10), QColor(5, 8, 12, 217));
        p->setPen(kInkSecondary);
        p->drawText(QRectF(kPadL + 3, y - 12, tw + 4, 12), Qt::AlignLeft | Qt::AlignBottom, label);
    }

    // Events (resamples): ticks on the baseline
    if (spec.events) {
        const EkfSeries &s = m_monitor.series(QLatin1String(spec.events));
        p->setPen(QPen(kInkSecondary, 1));
        for (int i = 0; i < s.size(); ++i)
            if (s.t(i) >= t0) {
                const double x = std::round(X(s.t(i))) + 0.5;
                p->drawLine(QPointF(x, kPadT + ph), QPointF(x, kPadT + ph - 4 - std::min(6.0, s.v(i))));
            }
    }

    // Series: 2 px lines, clipped to the plot; a hole in the data stays a hole
    struct End {
        QString name;
        QColor color;
        double x, y;
    };
    std::vector<End> ends;
    p->save();
    p->setClipRect(QRectF(kPadL, kPadT - 1, pw, ph + 2));
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const EkfSeries &s = m_monitor.series(QLatin1String(lines[i].key));
        const QColor color(seriesColor(spec, lines[i], int(i)));
        QPainterPath path;
        bool started = false;
        for (int k = 0; k < s.size(); ++k) {
            if (s.t(k) < t0 && !(k + 1 < s.size() && s.t(k + 1) >= t0)) continue;
            const QPointF pt(X(s.t(k)), Y(s.v(k)));
            if (started && s.t(k) - s.t(k - 1) <= kGapMs) path.lineTo(pt);
            else {
                path.moveTo(pt);
                started = true;
            }
        }
        QPen pen(color, 2);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        p->setPen(pen);
        p->setBrush(Qt::NoBrush);
        p->drawPath(path);
        if (s.size() && *s.lastTime() >= t0) ends.push_back({QString::fromUtf8(lines[i].name), color, X(*s.lastTime()), Y(*s.last())});
    }
    p->restore();

    // End points, and the names beside them (pushed apart so they never overlap)
    p->setPen(Qt::NoPen);
    for (const End &e : ends) {
        p->setBrush(QColor(12, 18, 24, 230));
        p->drawEllipse(QPointF(e.x, e.y), 4, 4);
        p->setBrush(e.color);
        p->drawEllipse(QPointF(e.x, e.y), 2.5, 2.5);
    }
    if (lines.size() > 1 && !ends.empty()) {
        std::sort(ends.begin(), ends.end(), [](const End &a, const End &b) { return a.y < b.y; });
        for (std::size_t i = 1; i < ends.size(); ++i) ends[i].y = std::max(ends[i].y, ends[i - 1].y + 10);
        const double over = ends.back().y - (kPadT + ph);
        if (over > 0)
            for (End &e : ends) e.y -= over;
        p->setFont(endFont);
        p->setPen(kInkSecondary);
        for (const End &e : ends) p->drawText(QRectF(kPadL + pw + 6, e.y - 7, padR, 14), Qt::AlignLeft | Qt::AlignVCenter, e.name);
    }

    // Hover: crosshair at the pointer's time
    if (hoverX >= kPadL && hoverX <= kPadL + pw) {
        p->setPen(QPen(kInkSecondary, 1));
        const double x = std::round(hoverX) + 0.5;
        p->drawLine(QPointF(x, kPadT), QPointF(x, kPadT + ph));
    }

    bool any = false;
    for (const Line &l : lines) any |= !m_monitor.series(QLatin1String(l.key)).empty();
    if (!any) {
        p->setPen(kInkMuted);
        p->setFont(uiFont(11));
        p->drawText(QRectF(kPadL, kPadT, pw, ph), Qt::AlignCenter, QStringLiteral("NO DATA"));
    }
}

// ── EkfChart ─────────────────────────────────────────────────────────────

EkfChart::EkfChart(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAcceptHoverEvents(true);
    setAntialiasing(true);
    setCursor(Qt::CrossCursor);
}

void EkfChart::componentComplete()
{
    QQuickPaintedItem::componentComplete();
    if (EkfController::s_instance) connect(EkfController::s_instance, &EkfController::redraw, this, [this] { update(); });
}

void EkfChart::setCard(int c)
{
    if (c == m_card) return;
    m_card = c;
    emit cardChanged();
    update();
}

void EkfChart::paint(QPainter *p)
{
    if (EkfController::s_instance) EkfController::s_instance->drawCard(p, m_card, width(), height(), m_hover);
}

void EkfChart::hoverMoveEvent(QHoverEvent *e)
{
    m_hover = e->position().x();
    if (EkfController::s_instance) EkfController::s_instance->setHover(m_card, m_hover, width());
    update();
}

void EkfChart::hoverLeaveEvent(QHoverEvent *)
{
    m_hover = -1;
    if (EkfController::s_instance) EkfController::s_instance->setHover(m_card, -1, width());
    update();
}

} // namespace corv
