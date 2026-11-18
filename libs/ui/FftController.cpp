#include "ui/FftController.h"

#include "core/Clock.h"
#include "core/Settings.h"
#include "link/MessageBus.h"

#include <QCursor>
#include <QFontMetricsF>
#include <QHoverEvent>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

namespace corv {

FftController *FftController::s_instance = nullptr;

namespace {

const char *const kEnabledKey = "fft-panel-enabled";
const char *const kSensorKey = "fft-panel-sensor";
constexpr int kDrawMs = 100;       // 60 s across the plot: about 1 px of scroll a step
constexpr int kTextEvery = 5;      // the texts every 500 ms

const QColor kInkSecondary(0xc3, 0xc2, 0xb7), kInkMuted(0x89, 0x87, 0x81);
const QColor kGrid(255, 255, 255, 18), kAxis(255, 255, 255, 56);
const QColor kLineColor(0x39, 0x87, 0xe5), kPeakColor(0xc9, 0x85, 0x00);
constexpr double kWfDbLo = -100, kWfDbHi = 0;   // fixed colour scale of the waterfall

QString fmtDb(double v) { return (v < 0 ? QStringLiteral("−") : QString()) + QString::number(std::abs(v), 'f', 0); }
QString fmtDf(double v) { return (v >= 10 ? QString::number(v, 'f', 1) : v >= 1 ? QString::number(v, 'f', 2) : QString::number(v, 'f', 3)) + QStringLiteral(" Hz"); }
QString fmtDur(double ms)
{
    return ms >= 10000 ? QString::number(ms / 1000, 'f', 0) + QStringLiteral(" s")
         : ms >= 1000 ? QString::number(ms / 1000, 'f', 1) + QStringLiteral(" s")
                      : QString::number(std::lround(ms)) + QStringLiteral(" ms");
}

// Perceptual dark-to-bright map (inferno-like): low levels sink into the black
const std::array<std::array<std::uint8_t, 3>, 256> &lut()
{
    static const auto table = [] {
        const std::pair<double, std::array<double, 3>> stops[] = {
            {0.00, {0, 0, 4}}, {0.14, {40, 11, 84}}, {0.29, {101, 21, 110}}, {0.43, {159, 42, 99}},
            {0.57, {212, 72, 66}}, {0.71, {245, 125, 21}}, {0.86, {250, 193, 39}}, {1.00, {252, 255, 164}}};
        std::array<std::array<std::uint8_t, 3>, 256> l{};
        for (int i = 0; i < 256; ++i) {
            const double t = i / 255.0;
            int k = 1;
            while (k < 7 && stops[k].first < t) ++k;
            const auto &[t0, c0] = stops[k - 1];
            const auto &[t1, c1] = stops[k];
            const double f = (t - t0) / (t1 - t0);
            for (int c = 0; c < 3; ++c) l[std::size_t(i)][std::size_t(c)] = std::uint8_t(std::clamp(std::lround(c0[std::size_t(c)] + f * (c1[std::size_t(c)] - c0[std::size_t(c)])), 0L, 255L));
        }
        return l;
    }();
    return table;
}

/// The frequency axis: linear from 0 Hz to the top bin of the source, as the
/// Electron charts (SpectrumViews.js).
struct FreqAxis {
    double fMax, p0, len;
    FreqAxis(const SpectrumGrid &g, double start, double length) : fMax(std::max(g.maxHz(), 2 * g.df)), p0(start), len(length) {}
    double pos(double f) const { return p0 + std::clamp(f, 0.0, fMax) / fMax * len; }
    double freq(double p) const { return std::clamp((p - p0) / len, 0.0, 1.0) * fMax; }
    /// The bin under a position on the axis.
    int bin(double p, const SpectrumGrid &g) const { return std::clamp(int(std::lround(freq(p) / g.df)) - 1, 0, g.n - 1); }
};

/// Frequency ticks from 0 to the top, a 1-2-5 step at least minPx apart (freqTicks of SpectrumViews.js).
std::vector<double> freqTicks(const FreqAxis &a, double minPx)
{
    const double raw = a.fMax * minPx / std::max(1.0, a.len);
    const double mag = std::pow(10, std::floor(std::log10(raw)));
    double step = 10 * mag;
    for (double m : {1.0, 2.0, 5.0, 10.0})
        if (m * mag >= raw) {
            step = m * mag;
            break;
        }
    std::vector<double> out;
    for (int k = 0; k * step <= a.fMax + 1e-9; ++k) out.push_back(k * step);
    return out;
}
QString fmtHz(double f, bool unit)
{
    const QString v = f >= 1000 ? QString::number(f / 1000, 'g', 3) + QStringLiteral("k") : QString::number(f, 'g', f >= 1 ? 4 : 2);
    return unit ? v + QStringLiteral(" Hz") : v;
}

/// Per pixel row (or column) along the axis: the bins it covers, or the
/// nearest one where the bins are wider than a pixel.
std::vector<std::pair<int, int>> binRanges(const FreqAxis &a, const SpectrumGrid &g, int pixels)
{
    auto r = std::vector<std::pair<int, int>>(std::size_t(pixels));
    for (int k = 0; k < pixels; ++k) {
        const double fa = a.freq(a.p0 + k * a.len / pixels), fb = a.freq(a.p0 + (k + 1) * a.len / pixels);
        int i0 = std::max(0, int(std::ceil(fa / g.df - 1e-9)) - 1), i1 = std::min(g.n - 1, int(std::ceil(fb / g.df - 1e-9)) - 2);
        if (i1 < i0) i0 = i1 = std::clamp(int(std::lround(0.5 * (fa + fb) / g.df)) - 1, 0, g.n - 1);
        r[std::size_t(k)] = {i0, i1};
    }
    return r;
}

QFont tickFont()
{
    QFont f(QStringLiteral("Roboto Mono"));
    f.setPixelSize(10);
    return f;
}
QFont labelFont()
{
    QFont f(QStringLiteral("Rajdhani"));
    f.setPixelSize(11);
    f.setWeight(QFont::DemiBold);
    return f;
}

void noData(QPainter *p, double x, double y, const QString &hint)
{
    p->setPen(kInkMuted);
    p->setFont(labelFont());
    p->drawText(QRectF(x - 200, y - 15, 400, 16), Qt::AlignCenter, QStringLiteral("NO DATA"));
    p->setFont(tickFont());
    p->drawText(QRectF(x - 200, y + 1, 400, 16), Qt::AlignCenter, hint);
}

/// Bin of the highest level, above the first bins (the residual of the mean).
std::pair<double, double> peakOf(const std::vector<float> &db, const SpectrumGrid &grid)
{
    int best = grid.firstLineBin();
    for (int i = best + 1; i < int(db.size()); ++i)
        if (db[std::size_t(i)] > db[std::size_t(best)]) best = i;
    return {grid.binFreq(best), db[std::size_t(best)]};
}

constexpr double kLiveL = 34, kLiveR = 8, kLiveT = 8, kLiveB = 14;
constexpr double kWfL = 40, kWfR = 8, kWfT = 6, kWfB = 14;

} // namespace

// ── Controller ───────────────────────────────────────────────────────────

FftController::FftController(MessageBus *bus, QObject *parent) : QObject(parent)
{
    s_instance = this;
    m_enabled = Settings::instance().value(QLatin1String(kEnabledKey)).toString() == QLatin1String("1");
    m_sensor = Settings::instance().value(QLatin1String(kSensorKey)).toString() == QLatin1String("accel") ? 1 : 0;
    bus->subscribe(MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, this, [this](const MavMessage &mm) {
        mavlink_debug_float_array_t d;
        mavlink_msg_debug_float_array_decode(&mm.msg, &d);
        m_monitor.ingest(mav::fieldString(d.name, 10), d.array_id, d.time_usec, d.data, MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_DATA_LEN,
                         m_monitor.clock(monoMs()));
    });
    m_draw.setInterval(kDrawMs);
    connect(&m_draw, &PacedTimer::timeout, this, [this] {
        emit redraw();
        if (++m_drawCount % kTextEvery == 0) updateText();
    });
    if (m_enabled) m_draw.start();
    updateText();
    updateLogStatus();
}

double FftController::now() const { return m_monitor.clock(monoMs()); }

void FftController::setEnabled(bool on)
{
    if (on == m_enabled) return;
    m_enabled = on;
    Settings::instance().setValue(QLatin1String(kEnabledKey), on ? QStringLiteral("1") : QStringLiteral("0"));
    if (m_enabled && m_shown) m_draw.start();
    else m_draw.stop();
    updateText();
    emit enabledChanged();
}

void FftController::setShown(bool on)
{
    if (on == m_shown) return;
    m_shown = on;
    if (m_enabled && m_shown) m_draw.start();
    else m_draw.stop();
    emit shownChanged();
}

void FftController::setSensor(const QString &s)
{
    const int i = s == QLatin1String("accel") ? 1 : 0;
    if (i == m_sensor) return;
    m_sensor = i;
    Settings::instance().setValue(QLatin1String(kSensorKey), sensor());
    emit sensorChanged();
    updateText();
    emit redraw();
}

QString FftController::unit() const { return m_sensor == 0 ? QStringLiteral("dB re 1 (rad/s)²/Hz") : QStringLiteral("dB re 1 (m/s²)²/Hz"); }

void FftController::selectStream(const QString &key) { emit streamSelected(m_sensor, key); }

void FftController::onLogSpectrum(const LogSpectrumMsg &m)
{
    m_monitor.onLogSpectrum(m);
    if (m.reset) {
        updateText();
        updateLogStatus();
    }
}

void FftController::replayState(const QString &state, const QString &format)
{
    const bool on = state == QLatin1String("LOADED") || state == QLatin1String("PLAYING") || state == QLatin1String("PAUSED");
    if (on && !m_monitor.replaying()) m_monitor.replayLoaded(format, monoMs());
    else if (!on && m_monitor.replaying()) m_monitor.replayUnloaded();
    updateText();
    updateLogStatus();
}

void FftController::replayTick(double currentMs, bool playing) { m_monitor.replayTick(currentMs, playing, monoMs()); }
void FftController::replaySeek() { m_monitor.replaySeek(); }

void FftController::reset()
{
    m_monitor.reset();
    updateText();
}

void FftController::updateText()
{
    const auto src = m_monitor.source(m_sensor);
    m_sourceText = src.text;
    QVariantList streams;
    if (src.kind == QLatin1String("log"))
        for (const auto &s : src.streams) streams << QVariantMap{{QStringLiteral("value"), s.key}, {QStringLiteral("label"), s.label}};
    m_streams = streams;
    m_stream = src.stream;

    const double now = this->now();
    QVariantList peaks;
    for (int s = 0; s < 2; ++s) {
        QVariantMap row{{QStringLiteral("label"), s == 0 ? QStringLiteral("GYRO") : QStringLiteral("ACC")}};
        const auto d = m_monitor.get(s).dominant(now);
        if (!d) {
            row[QStringLiteral("none")] = QStringLiteral("NO DATA · %1").arg(m_monitor.source(s).hint);
        } else if (d->peaks.empty()) {
            row[QStringLiteral("main")] = QStringLiteral("%1 Hz %2 dB").arg(d->max.f, 0, 'f', 1).arg(fmtDb(d->max.db));
            row[QStringLiteral("more")] = QStringLiteral("no line over the noise · floor %1 dB").arg(fmtDb(d->floor));
            row[QStringLiteral("moreDim")] = true;
        } else {
            const auto &p = d->peaks.front();
            row[QStringLiteral("main")] = QStringLiteral("<font color=\"#fab219\"><b>%1 Hz</b></font> %2 dB · +%3 dB over the floor")
                                              .arg(p.f, 0, 'f', 1).arg(fmtDb(p.db)).arg(p.prom, 0, 'f', 0);
            QStringList more;
            for (std::size_t i = 1; i < d->peaks.size(); ++i)
                more << QStringLiteral("%1 Hz %2 dB").arg(d->peaks[i].f, 0, 'f', 1).arg(fmtDb(d->peaks[i].db));
            more << QStringLiteral("floor %1 dB").arg(fmtDb(d->floor));
            row[QStringLiteral("more")] = more.join(QStringLiteral(" · "));
        }
        peaks << row;
    }
    m_peaks = peaks;
    emit textChanged();
}

/// SYS CONFIG → FFT FROM LOGS: what the loaded log holds.
void FftController::updateLogStatus()
{
    const LogSpectrumInfo *log = m_monitor.log();
    QString s;
    if (!log) {
        s = m_monitor.replaying() ? QStringLiteral("a .tlog: the vehicle's own spectrum") : QStringLiteral("no .bin loaded");
    } else {
        QStringList lines;
        for (int k = 0; k < 2; ++k) {
            const auto &l = log->sensors[std::size_t(k)];
            const QString name = k == 0 ? QStringLiteral("GYRO") : QStringLiteral("ACC");
            if (l.stream.isEmpty()) {
                lines << name + QStringLiteral(": no samples");
                continue;
            }
            const QString cut = l.limited ? QStringLiteral(" (cut to %1 pt%2 by the %3-sample runs)").arg(l.N).arg(l.avg > 1 ? QStringLiteral(" × %1").arg(l.avg) : QString()).arg(l.longest)
                                          : QString();
            lines << QStringLiteral("%1: %2 → %3, %4 per spectrum%5").arg(name, l.label.toHtmlEscaped(), fmtDf(l.df), fmtDur(l.windowMs), cut);
        }
        if (!log->dropped.isEmpty())
            lines << QStringLiteral("<font color=\"#ff7070\">left out, in fragments: %1</font>").arg(log->dropped.join(QStringLiteral(", ")).toHtmlEscaped());
        s = lines.join(QStringLiteral("<br>"));
    }
    if (s != m_logStatus) {
        m_logStatus = s;
        emit logStatusChanged();
    }
}

// ── SpectrumView ─────────────────────────────────────────────────────────

SpectrumView::SpectrumView(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAcceptHoverEvents(true);
    setAntialiasing(true);
    setCursor(Qt::CrossCursor);
}

void SpectrumView::componentComplete()
{
    QQuickPaintedItem::componentComplete();
    if (auto *c = FftController::s_instance) {
        connect(c, &FftController::redraw, this, &SpectrumView::refresh);
        connect(c, &FftController::sensorChanged, this, [this] {
            m_dirty = true;
            refresh();
        });
    }
    connect(this, &QQuickItem::widthChanged, this, [this] { m_dirty = true; });
    connect(this, &QQuickItem::heightChanged, this, [this] { m_dirty = true; });
}

void SpectrumView::setMode(const QString &m)
{
    if (m == m_mode) return;
    m_mode = m;
    m_dirty = true;
    emit modeChanged();
    update();
}

void SpectrumView::hoverMoveEvent(QHoverEvent *e)
{
    m_hover = e->position();
    m_dirty = true;
    refresh();
}

void SpectrumView::hoverLeaveEvent(QHoverEvent *)
{
    m_hover = {-1, -1};
    m_dirty = true;
    refresh();
}

/// GUI thread: the readouts for the legend, then a repaint when something changed.
void SpectrumView::refresh()
{
    auto *c = FftController::s_instance;
    if (!c || !isVisible()) return;
    const SensorSpectra &s = c->monitor().get(c->sensorIndex());
    const double perf = monoMs();
    if (m_mode == QLatin1String("live")) {
        // The newest spectrum: on new data, a resize or a pointer move, and once a second for the staleness note
        if (!(s.version() != m_lastVersion || m_dirty || perf - m_lastPaint >= 1000)) return;
        m_lastVersion = s.version();
        const QString r = liveReadout(s);
        if (r != m_readout) {
            m_readout = r;
            emit readoutChanged();
        }
    } else {
        const QString r = waterfallReadout(s, c->now());
        const QString range = s.rows().empty() ? QStringLiteral("—") : QStringLiteral("%1 … %2 dB").arg(kWfDbLo).arg(kWfDbHi);
        if (r != m_readout || range != m_range) {
            m_readout = r;
            m_range = range;
            emit readoutChanged();
        }
    }
    m_dirty = false;
    m_lastPaint = perf;
    update();
}

QString SpectrumView::liveReadout(const SensorSpectra &s) const
{
    const auto *latest = s.latest();
    const double pw = width() - kLiveL - kLiveR;
    if (!latest || pw < 30) return {};
    const SpectrumGrid &g = s.grid();
    const FreqAxis ax(g, kLiveL, pw);
    double f, db;
    QString label;
    if (m_hover.x() >= kLiveL && m_hover.x() <= kLiveL + pw) {
        const int i = ax.bin(m_hover.x(), g);
        f = g.binFreq(i);
        db = (*latest->db)[std::size_t(i)];
        label = QStringLiteral("AT");
    } else {
        std::tie(f, db) = peakOf(*latest->db, g);
        label = QStringLiteral("PEAK");
    }
    return QStringLiteral("%1 %2 Hz · %3 dB").arg(label, QString::number(f, 'f', f < 10 ? 1 : 0), QString::number(db, 'f', 1));
}

QString SpectrumView::waterfallReadout(const SensorSpectra &s, double now) const
{
    const double pw = width() - kWfL - kWfR, ph = height() - kWfT - kWfB;
    if (pw < 30 || ph < 30 || s.rows().empty()) return {};
    if (!(m_hover.x() >= kWfL && m_hover.x() <= kWfL + pw && m_hover.y() >= kWfT && m_hover.y() <= kWfT + ph)) return {};
    const SpectrumGrid &g = s.grid();
    const int i = FreqAxis(g, kWfT, ph).bin(m_hover.y(), g);
    const double age = (1 - (m_hover.x() - kWfL) / pw) * kSpectrumHistoryMs;
    const SensorSpectra::Row *row = nullptr;
    const auto &rows = s.rows();
    for (auto it = rows.rbegin(); it != rows.rend() && it->t >= now - age; ++it)
        if (it->t <= now) row = &*it;
    if (!row || row->t - (now - age) > s.holdMs()) return {};
    return QStringLiteral("%1 Hz · −%2 s · %3 dB").arg(g.binFreq(i), 0, 'f', 1).arg(age / 1000, 0, 'f', 0).arg((*row->db)[std::size_t(i)], 0, 'f', 1);
}

void SpectrumView::paint(QPainter *p)
{
    auto *c = FftController::s_instance;
    if (!c) return;
    p->setRenderHint(QPainter::Antialiasing, true);
    const SensorSpectra &s = c->monitor().get(c->sensorIndex());
    if (m_mode == QLatin1String("live")) paintLive(p, s, c->now());
    else paintWaterfall(p, s, c->now());
}

void SpectrumView::paintLive(QPainter *p, const SensorSpectra &s, double now)
{
    const double w = width(), h = height();
    const SpectrumGrid &grid = s.grid();
    const int n = grid.n;
    const double pw = w - kLiveL - kLiveR, ph = h - kLiveT - kLiveB;
    if (pw < 30 || ph < 30) return;
    const FreqAxis ax(grid, kLiveL, pw);
    const auto ticks = freqTicks(ax, 34);
    const auto freqAxis = [&] {
        p->setFont(tickFont());
        p->setPen(kInkMuted);
        for (std::size_t k = 0; k < ticks.size(); ++k) {
            const double x = ax.pos(ticks[k]);
            if (k == 0) p->drawText(QRectF(x, h - 14, 60, 13), Qt::AlignLeft | Qt::AlignBottom, fmtHz(ticks[k], true));
            else p->drawText(QRectF(x - 30, h - 14, 60, 13), Qt::AlignHCenter | Qt::AlignBottom, fmtHz(ticks[k], false));
        }
    };
    const auto *latest = s.latest();
    auto *c = FftController::s_instance;
    if (!latest) {
        freqAxis();
        noData(p, kLiveL + pw / 2, kLiveT + ph / 2, c->monitor().source(c->sensorIndex()).hint);
        return;
    }
    const auto &db = *latest->db;
    const auto peak = s.peakHold(now);
    double mx = -INFINITY, mn = INFINITY;
    for (int i = std::min(1, n - 1); i < n; ++i) {
        mx = std::max({mx, double(db[std::size_t(i)]), peak ? double((*peak)[std::size_t(i)]) : -INFINITY});
        mn = std::min(mn, double(db[std::size_t(i)]));
    }
    const double hi = std::ceil((mx + 3) / 10) * 10;
    double lo = std::max(std::floor((mn - 3) / 10) * 10, hi - 90);
    if (hi - lo < 30) lo = hi - 30;
    const auto Y = [&](double v) { return kLiveT + ph - (std::clamp(v, lo, hi) - lo) / (hi - lo) * ph; };

    // dB grid
    p->setFont(tickFont());
    const double stepDb = hi - lo > 60 ? 20 : 10;
    for (double v = lo; v <= hi; v += stepDb) {
        const double y = std::round(Y(v)) + 0.5;
        p->setPen(QPen(v == lo ? kAxis : kGrid, 1));
        p->drawLine(QPointF(kLiveL, y), QPointF(kLiveL + pw, y));
        p->setPen(kInkMuted);
        p->drawText(QRectF(0, y - 7, kLiveL - 4, 14), Qt::AlignRight | Qt::AlignVCenter, QString::number(v));
    }
    p->setPen(QPen(kGrid, 1));
    for (double f : ticks) {
        const double x = std::round(ax.pos(f)) + 0.5;
        p->drawLine(QPointF(x, kLiveT), QPointF(x, kLiveT + ph));
    }
    freqAxis();

    p->save();
    p->setClipRect(QRectF(kLiveL, kLiveT - 1, pw, ph + 2));
    // One point per pixel column where several bins share it (the highest of
    // them), every bin where they are wider than a column
    const int cols = std::max(2, int(std::lround(pw)));
    const auto ranges = binRanges(ax, grid, cols);
    const auto path = [&](const std::vector<float> &d, const QColor &color, double width) {
        QPainterPath pp;
        int lastBin = -1;
        bool started = false;
        for (int col = 0; col < cols; ++col) {
            const auto [i0, i1] = ranges[std::size_t(col)];
            QPointF pt;
            if (i0 == i1) {
                if (i0 == lastBin) continue;   // a wide bin: one point at its own frequency
                pt = QPointF(ax.pos(grid.binFreq(i0)), Y(d[std::size_t(i0)]));
            } else {
                double v = -INFINITY;
                for (int i = i0; i <= i1; ++i) v = std::max(v, double(d[std::size_t(i)]));
                pt = QPointF(kLiveL + (col + 0.5) * pw / cols, Y(v));
            }
            lastBin = i1;
            if (started) pp.lineTo(pt);
            else {
                pp.moveTo(pt);
                started = true;
            }
        }
        QPen pen(color, width);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        p->setPen(pen);
        p->setBrush(Qt::NoBrush);
        p->drawPath(pp);
    };
    if (peak) {
        p->setOpacity(0.85);
        path(*peak, kPeakColor, 1);
        p->setOpacity(1);
    }
    path(db, kLineColor, 1.5);
    p->restore();

    if (!c->monitor().live(c->sensorIndex(), now)) {
        p->setPen(kInkMuted);
        p->setFont(labelFont());
        p->drawText(QRectF(kLiveL, kLiveT + 2, pw - 2, 14), Qt::AlignRight | Qt::AlignTop, QStringLiteral("NO NEW DATA"));
    }
    // Pointer: the level of the bin under it
    if (m_hover.x() >= kLiveL && m_hover.x() <= kLiveL + pw) {
        const double x = std::round(ax.pos(grid.binFreq(ax.bin(m_hover.x(), grid)))) + 0.5;
        p->setPen(QPen(kInkSecondary, 1));
        p->drawLine(QPointF(x, kLiveT), QPointF(x, kLiveT + ph));
    }
}

void SpectrumView::paintWaterfall(QPainter *p, const SensorSpectra &s, double now)
{
    const double w = width(), h = height();
    const auto &rows = s.rows();
    const SpectrumGrid &grid = s.grid();
    const double pw = w - kWfL - kWfR, ph = h - kWfT - kWfB;
    if (pw < 30 || ph < 30) return;
    // Frequency down the plot: 0 Hz at the top
    const FreqAxis ax(grid, kWfT, ph);

    // Frequency labels down the left, time labels along the bottom
    p->setFont(tickFont());
    p->setPen(kInkMuted);
    const auto ticks = freqTicks(ax, 26);
    for (std::size_t k = 0; k < ticks.size(); ++k) {
        const double y = k == 0 ? std::max(ax.pos(ticks[k]), kWfT + 5) : ax.pos(ticks[k]);
        if (k && y > kWfT + ph - 4) continue;
        p->drawText(QRectF(0, y - 7, kWfL - 4, 14), Qt::AlignRight | Qt::AlignVCenter, fmtHz(ticks[k], k == 0));
    }
    p->drawText(QRectF(kWfL, h - 14, 60, 13), Qt::AlignLeft | Qt::AlignBottom, QStringLiteral("−60 s"));
    p->drawText(QRectF(kWfL + pw / 2 - 30, h - 14, 60, 13), Qt::AlignHCenter | Qt::AlignBottom, QStringLiteral("−30 s"));
    p->drawText(QRectF(kWfL + pw - 60, h - 14, 60, 13), Qt::AlignRight | Qt::AlignBottom, QStringLiteral("now"));
    p->setPen(QPen(kAxis, 1));
    p->setBrush(Qt::NoBrush);
    p->drawRect(QRectF(kWfL - 0.5, kWfT - 0.5, pw + 1, ph + 1));
    if (rows.empty()) {
        auto *c = FftController::s_instance;
        noData(p, kWfL + pw / 2, kWfT + ph / 2, c->monitor().source(c->sensorIndex()).hint);
        return;
    }

    const int W = std::max(1, int(std::lround(pw)));
    const int H = std::max(1, int(std::lround(ph)));
    const auto ranges = binRanges(ax, grid, H);
    QImage img(W, H, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::transparent);
    const auto &L = lut();
    // Each pixel column shows the spectrum that covers its time: the first one
    // at or after it, within its hold (a longer hole stays black)
    const double hold = s.holdMs();
    std::size_t r = 0;
    for (int x = 0; x < W; ++x) {
        const double t = now - (1 - (x + 0.5) / W) * kSpectrumHistoryMs;
        while (r < rows.size() - 1 && rows[r].t < t) ++r;
        const auto &row = rows[r];
        if (row.t < t || row.t - t > hold || row.t > now) continue;
        const auto &db = *row.db;
        for (int y = 0; y < H; ++y) {
            const auto [i0, i1] = ranges[std::size_t(y)];
            double v = -INFINITY;
            for (int i = i0; i <= i1; ++i) v = std::max(v, double(db[std::size_t(i)]));
            const int k = std::clamp(int(std::lround((v - kWfDbLo) / (kWfDbHi - kWfDbLo) * 255)), 0, 255);
            const auto &c = L[std::size_t(k)];
            img.setPixel(x, y, qRgb(c[0], c[1], c[2]));
        }
    }
    p->save();
    p->setClipRect(QRectF(kWfL, kWfT, pw, ph));
    p->setRenderHint(QPainter::SmoothPixmapTransform, false);
    p->drawImage(QRectF(kWfL, kWfT, pw, ph), img);
    p->restore();

    if (m_hover.x() >= kWfL && m_hover.x() <= kWfL + pw && m_hover.y() >= kWfT && m_hover.y() <= kWfT + ph) {
        const double y = std::round(ax.pos(grid.binFreq(ax.bin(m_hover.y(), grid)))) + 0.5, x = std::round(m_hover.x()) + 0.5;
        p->setPen(QPen(QColor(255, 255, 255, 128), 1));
        p->drawLine(QPointF(kWfL, y), QPointF(kWfL + pw, y));
        p->drawLine(QPointF(x, kWfT), QPointF(x, kWfT + ph));
    }
}

} // namespace corv
