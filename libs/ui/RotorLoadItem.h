// RotorLoadItem.h - ROTOR LOAD schematic (js/ui/RotorLoadPanel.js)
//
// Top-down functional diagram of the propulsion: one round gauge per rotor,
// placed where that motor sits on the airframe and driven by the live
// SERVO_OUTPUT_RAW PWM of its output. Fixed wing and helicopter get a single
// gauge for the main rotor / propeller.
//
// With FRAME on AUTO the motors are the ones the outputs are assigned to
// (SERVOn_FUNCTION, see ServoMap): MotorN on the ring in ArduPilot's matrix
// positions, the fixed-wing throttles (a quadplane's pusher, a twin's left and
// right engines) in the middle or side by side; MAV_TYPE stands in until every
// SERVOn_FUNCTION is known. The same list says whether the plane has flaps or
// flaperons: the FLAPS indicator over the schematic (FlapIndicatorItem) shows
// their deployment, read back from the outputs (`flapShown`, `flapPercent`).
//
// The gauges copy the geometry of a Boeing/Airbus N1 indicator: a bare 270°
// scale circle starting lower-left, a fixed tick at zero, a moving tick and
// filled arc that follow the PWM, and the motor label in the middle.
//
// A sailboat (Rover with a rotating mast) gets two round dials instead: the
// wing and the rudder drawn as NACA foils at the commanded angle, and the
// apparent wind.
//
// The parameters are read one by one, and only while the schematic is enabled.
// The settings are SYS CONFIG → ROTOR LOAD ("rotor-load-cfg", same JSON as
// Electron's localStorage entry). The item repaints on every VehicleModel
// update (30 Hz); `shown` and `sail` drive the caption and the size in QML.
#pragma once

#include "vehicle/ServoMap.h"

#include <QQuickPaintedItem>
#include <QSet>
#include <QStringList>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace corv {

struct VehicleState;

class RotorLoadItem : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT
    /// cfg.enabled, and a vehicle with something to draw (applyVisibility)
    Q_PROPERTY(bool shown READ shown NOTIFY layoutChanged)
    /// Wing and rudder dials in place of the rotor gauges
    Q_PROPERTY(bool sail READ sail NOTIFY layoutChanged)
    /// Shown, and an output carries flaps or flaperons
    Q_PROPERTY(bool flapShown READ flapShown NOTIFY layoutChanged)
    /// Flap deployment in percent (negative: reflex), NaN with no PWM
    Q_PROPERTY(qreal flapPercent READ flapPercent NOTIFY flapChanged)
public:
    explicit RotorLoadItem(QQuickItem *parent = nullptr);
    void paint(QPainter *p) override;

    bool shown() const { return m_shown; }
    bool sail() const { return m_sailShown; }
    bool flapShown() const { return m_flapShown; }
    qreal flapPercent() const { return m_flapPct; }

signals:
    void layoutChanged();
    void flapChanged();

protected:
    void componentComplete() override;

private:
    struct Config {
        bool enabled = true;
        double pwmMin = 1000;     // motors stopped / scale floor
        double pwmMax = 2000;     // scale ceiling
        double thGreen = 1100;    // above this → green (spinning)
        double thOrange = 1500;   // above this → orange (high load)
        double thRed = 1700;      // above this → red (max load)
        QString frame = QStringLiteral("auto");   // 'auto' (servo outputs, MAV_TYPE), 'sail', or a rotor count
        int singleChannel = 3;    // output channel used for the single-rotor gauge (1-16)
        double rudderMax = 35;    // rudder angle at full steering: a property of the boat
    };
    struct SailLayout {
        int sail = 4;
        int rudder = 1;
        double sailMax = 90;
    };
    struct Probe {
        std::int64_t hb = 0;
        int sys = -1;
        QStringList queue;
        QSet<QString> asked;      // requested on this link
        std::int64_t next = 0;
        std::int64_t lastSent = 0;
        int rounds = 0;           // retries of what never arrived
    };
    /// One gauge: a 1-based output, its label, and its bearing on the ring
    /// (degrees clockwise from the nose; none = the middle, or a row)
    struct Gauge {
        int output;
        QString label;
        std::optional<double> angle;
    };
    struct Rotors {
        std::vector<Gauge> ring;     // multirotor arms
        std::vector<Gauge> centre;   // the single rotor, or the forward engines left to right
    };

    void loadConfig();
    void refresh();
    void applyVisibility(const std::optional<SailLayout> &lay);
    void updateFlaps();

    std::optional<double> param(const QString &name) const;
    std::optional<SailLayout> sailLayout() const;
    std::optional<SailLayout> sailMode() const;
    QStringList wantedParams() const;
    void probeParams(std::int64_t now);
    bool servosApply() const;
    Rotors rotors() const;
    int activeRotorCount() const;
    bool isHiddenVehicle() const;

    std::optional<double> commandedDeg(int output, double maxDeg) const;
    std::optional<double> apparentWindDeg(std::int64_t now) const;
    std::optional<double> pwmFor(int output, std::optional<double> angleDeg) const;
    double demoPwm(std::optional<double> angleDeg) const;
    QColor zoneColor(std::optional<double> pwm) const;

    // Drawing; `m_shadow` turns every colour into the drop shadow's
    QColor ink(const QColor &c) const;
    void drawRotors(QPainter *p, double w, double h);
    void drawGauge(QPainter *p, double cx, double cy, double r, std::optional<double> pwm, const QString &label);
    void tick(QPainter *p, double cx, double cy, double angle, double scaleR, double r, double len, const QColor &color);
    void drawSail(QPainter *p, double w, double h, const SailLayout &lay);
    void drawDial(QPainter *p, double cx, double cy, double r, std::optional<double> le,
                  std::optional<std::pair<double, double>> travel, std::optional<double> windDeg);
    void drawFoil(QPainter *p, double cx, double cy, double le, double chord, double lw);

    Config m_cfg;
    std::shared_ptr<const VehicleState> m_s;
    std::optional<SailLayout> m_lay;
    ServoMap m_servos;
    bool m_servosDirty = true;   // the parameter store changed since m_servos was read
    Rotors m_rotors;
    bool m_shown = false;
    bool m_sailShown = false;
    bool m_flapShown = false;
    qreal m_flapPct;
    bool m_applied = false;
    bool m_shadow = false;
    Probe m_probe;
};

} // namespace corv
