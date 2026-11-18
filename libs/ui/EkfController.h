// EkfController.h - EKF STATUS bar ("Ekf" QML singleton): the port of
// js/ui/EkfStatusPanel.js.
//
// Three charts stacked in a black bar down the left side, the flight view moved
// over to make room; the last 20 s of ArduPilot's EKF3 (data in EkfMonitor):
//
//   EKF INNOVATIONS  · GPS ACCURACY  · ATTITUDE EKF vs DCM
//
// The EKF STATUS tab on the left edge opens and closes it; the FFT tab under it
// opens the vibration spectra in the same place: one bar at a time.
//
// A sound estimate reads as: innovations under 0.5, the receiver's accuracy
// steady, the two attitude estimators within a degree or two. The header lists
// the solutions the filter itself flags as good or bad.
//
// While the bar is open on an ArduPilot link, EKF_STATUS_REPORT, AHRS2 and
// GPS_RAW_INT are asked for at 5 Hz and put back on close. Charts repaint at
// 20 Hz (EkfChart items), texts at 5 Hz, only while shown. GUI thread.
#pragma once

#include "gpu/FrameTicker.h"
#include "vehicle/EkfMonitor.h"

#include <QObject>
#include <QQuickPaintedItem>
#include <array>
#include <QQmlEngine>
#include <QTimer>
#include <QVariantList>
#include <functional>
#include <optional>

class QPainter;

namespace corv {

class CommandSender;
class LinkManager;
class MessageBus;
class VehicleHub;

class EkfController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Ekf)
    QML_SINGLETON
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(bool shown READ shown WRITE setShown NOTIFY shownChanged)
    Q_PROPERTY(QString sourceText READ sourceText NOTIFY textChanged)
    Q_PROPERTY(QVariantList flags READ flags NOTIFY textChanged)
    Q_PROPERTY(QVariantList cards READ cards NOTIFY textChanged)

public:
    EkfController(MessageBus *bus, LinkManager *link, CommandSender *cmd, VehicleHub *hub, QObject *parent);
    static EkfController *s_instance;
    static EkfController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    bool enabled() const { return m_enabled; }
    void setEnabled(bool on);
    bool shown() const { return m_shown; }
    void setShown(bool on);
    QString sourceText() const { return m_sourceText; }
    QVariantList flags() const { return m_flags; }
    QVariantList cards() const { return m_cards; }

    /// A new link is a new vehicle: the charts start empty.
    void reset();
    const EkfMonitor &monitor() const { return m_monitor; }

    /// Chart `card` into a w × h box; hoverX < 0: no pointer. Called from EkfChart::paint.
    void drawCard(QPainter *p, int card, double w, double h, double hoverX) const;
    void setHover(int card, double x, double w);

signals:
    void enabledChanged();
    void shownChanged();
    void textChanged();
    /// 20 Hz while shown: the charts repaint.
    void redraw();

private:
    struct Line {
        const char *key, *name;
        bool optional = false;
    };
    struct Mark {
        double v;
        const char *status;   // nullptr: a plain reference line
        const char *label;
    };
    struct Spec {
        const char *title, *unit;
        std::optional<double> min, max;
        double floor = 1;
        int digits = 2;
        bool symmetric = false, log = false;
        std::vector<Line> lines;
        std::vector<Mark> marks;
        const char *events = nullptr;
        std::optional<std::pair<const char *, std::pair<double, std::vector<Line>>>> fallback;   // unit, (floor, lines)
        std::function<QVariantList(const EkfMonitor &)> chips;
    };
    struct Active {
        std::vector<Line> lines;
        QString unit;
        double floor;
    };

    const std::vector<Spec> &specs() const;
    Active activeLines(const Spec &s) const;
    QString seriesColor(const Spec &s, const Line &l, int i) const;
    void tickDraw();
    void updateText(double now);
    void manageRates(double now);

    MessageBus *m_bus;
    CommandSender *m_cmd;
    VehicleHub *m_hub;
    EkfMonitor m_monitor;
    bool m_enabled = false;
    bool m_shown = true;
    QString m_cardSource = QStringLiteral("ardupilot");   // the charts' estimator
    QString m_sourceText;
    QVariantList m_flags, m_cards;
    std::array<double, 3> m_hoverX{-1, -1, -1};
    std::array<double, 3> m_hoverW{0, 0, 0};
    PacedTimer m_draw;   // on the 3D view's frames while it paces them (RENDER FPS CAP)
    QTimer m_rates;
    int m_drawCount = 0;
    bool m_ratesRequested = false;
};

/// One chart of the EKF STATUS bar.
class EkfChart : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int card READ card WRITE setCard NOTIFY cardChanged)
public:
    explicit EkfChart(QQuickItem *parent = nullptr);
    int card() const { return m_card; }
    void setCard(int c);
    void paint(QPainter *p) override;

signals:
    void cardChanged();

protected:
    void componentComplete() override;
    void hoverMoveEvent(QHoverEvent *e) override;
    void hoverLeaveEvent(QHoverEvent *e) override;

private:
    int m_card = 0;
    double m_hover = -1;
};

} // namespace corv
