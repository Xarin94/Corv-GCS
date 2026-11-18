// FlapIndicatorItem.h - FLAPS indicator over the ROTOR LOAD schematic
//
// Drawn in the rotor gauges' Airbus EIS style (bare 4 px scale arc, fixed tick
// at zero, moving tick and coloured arc following the value, no digits, the
// value in the colour of its zone) on the layout of the FLAP display of an
// Airbus E/WD: the wing section seen from the side as a fixed reference, the
// flap hinged at its trailing edge moving down over detent dots. The scale
// runs from −10° (reflex, up) to +30° (down): full flap (100 %) is drawn at
// 30°, retracted (0 %) flush with the wing. It carries the deployment percent
// ArduPilot puts on the outputs, not the surface's real angle, which only the
// airframe knows; the percentage is written under the wing.
//
// Paint only: RotorLoadItem finds the flap outputs (SERVOn_FUNCTION Flap,
// FlapAuto or the flaperon pair) and reads the deployment back from their
// PWM; QML binds `percent` to its `flapPercent` and shows the item only while
// `flapShown`.
#pragma once

#include <QQuickPaintedItem>

namespace corv {

class FlapIndicatorItem : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT
    /// Deployment in percent (0 retracted, 100 full, negative reflex); NaN = no data
    Q_PROPERTY(qreal percent READ percent WRITE setPercent NOTIFY percentChanged)
public:
    explicit FlapIndicatorItem(QQuickItem *parent = nullptr);
    void paint(QPainter *p) override;

    qreal percent() const { return m_percent; }
    void setPercent(qreal v);

    /// Flap angle drawn for a deployment: 100 % → 30° down, clamped to the scale.
    static double angleFor(double percent);

signals:
    void percentChanged();

private:
    void draw(QPainter *p, double w, double h, bool shadow);

    qreal m_percent;
};

} // namespace corv
