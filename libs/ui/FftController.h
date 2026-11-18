// FftController.h - FFT bar ("Fft" QML singleton): the port of js/ui/FftPanel.js
// and of the status line of js/ui/LogFftConfig.js.
//
// The IMU vibration spectra (SpectrumMonitor) in a black bar down the left side,
// in the place of the EKF STATUS bar: the two tabs take turns.
//
//   top:     the strongest lines of the last 2 s, gyro and accelerometer:
//            frequency, level and height over the noise floor
//   middle:  the newest spectrum with a 10 s peak hold; the peak, or the level
//            under the pointer
//   bottom:  waterfall of the last 60 s, frequency down the plot (0 Hz at the
//            top), time to the right, colour = level in dB
//
// One GYRO | ACC switch drives the chart and the waterfall. The vehicle sends
// the spectra with HPF_FFT_ENABLE = 1; in LOG REPLAY of a .bin the GCS computes
// them from the IMU samples of the log (LogSpectrum), and the line over the
// chart says which and picks the IMU stream of the log.
// GUI thread; the charts are SpectrumView items.
#pragma once

#include "gpu/FrameTicker.h"
#include "vehicle/SpectrumMonitor.h"

#include <QImage>
#include <QObject>
#include <QQmlEngine>
#include <QQuickPaintedItem>
#include <QTimer>
#include <QVariantList>

namespace corv {

class MessageBus;

class FftController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Fft)
    QML_SINGLETON
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(bool shown READ shown WRITE setShown NOTIFY shownChanged)
    Q_PROPERTY(QString sensor READ sensor WRITE setSensor NOTIFY sensorChanged)
    Q_PROPERTY(QString unit READ unit NOTIFY sensorChanged)
    Q_PROPERTY(QString sourceText READ sourceText NOTIFY textChanged)
    Q_PROPERTY(QVariantList streams READ streams NOTIFY textChanged)
    Q_PROPERTY(QString stream READ stream NOTIFY textChanged)
    Q_PROPERTY(QVariantList peaks READ peaks NOTIFY textChanged)
    Q_PROPERTY(QString logStatus READ logStatus NOTIFY logStatusChanged)

public:
    FftController(MessageBus *bus, QObject *parent);
    static FftController *s_instance;
    static FftController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    bool enabled() const { return m_enabled; }
    void setEnabled(bool on);
    bool shown() const { return m_shown; }
    void setShown(bool on);
    QString sensor() const { return m_sensor == 0 ? QStringLiteral("gyro") : QStringLiteral("accel"); }
    void setSensor(const QString &s);
    int sensorIndex() const { return m_sensor; }
    QString unit() const;
    QString sourceText() const { return m_sourceText; }
    QVariantList streams() const { return m_streams; }
    QString stream() const { return m_stream; }
    QVariantList peaks() const { return m_peaks; }
    QString logStatus() const { return m_logStatus; }

    const SpectrumMonitor &monitor() const { return m_monitor; }
    /// Now on the spectra's time base (the replay cursor while a log plays).
    double now() const;

    /// The IMU stream of the loaded .bin for the current sensor (LogReplay picks it).
    Q_INVOKABLE void selectStream(const QString &key);

    // From LOG REPLAY (ReplayController)
    void onLogSpectrum(const LogSpectrumMsg &m);
    void replayState(const QString &state, const QString &format);
    void replayTick(double currentMs, bool playing);
    void replaySeek();
    /// A new link is a new vehicle: its spectra start empty.
    void reset();

signals:
    void enabledChanged();
    void shownChanged();
    void sensorChanged();
    void textChanged();
    void logStatusChanged();
    /// 10 Hz while shown: the charts repaint.
    void redraw();
    void streamSelected(int sensor, const QString &key);

private:
    void updateText();
    void updateLogStatus();

    SpectrumMonitor m_monitor;
    bool m_enabled = false, m_shown = true;
    int m_sensor = 0;
    QString m_sourceText, m_stream, m_logStatus;
    QVariantList m_streams, m_peaks;
    PacedTimer m_draw;   // on the 3D view's frames while it paces them (RENDER FPS CAP)
    int m_drawCount = 0;
};

/// The newest spectrum (mode "live") or the 60 s waterfall (mode "waterfall") of Fft.sensor.
class SpectrumView : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QString mode READ mode WRITE setMode NOTIFY modeChanged)
    Q_PROPERTY(QString readout READ readout NOTIFY readoutChanged)
    Q_PROPERTY(QString rangeText READ rangeText NOTIFY readoutChanged)
public:
    explicit SpectrumView(QQuickItem *parent = nullptr);
    QString mode() const { return m_mode; }
    void setMode(const QString &m);
    QString readout() const { return m_readout; }
    QString rangeText() const { return m_range; }
    void paint(QPainter *p) override;

signals:
    void modeChanged();
    void readoutChanged();

protected:
    void componentComplete() override;
    void hoverMoveEvent(QHoverEvent *e) override;
    void hoverLeaveEvent(QHoverEvent *e) override;

private:
    void refresh();
    void paintLive(QPainter *p, const SensorSpectra &s, double now);
    void paintWaterfall(QPainter *p, const SensorSpectra &s, double now);
    QString liveReadout(const SensorSpectra &s) const;
    QString waterfallReadout(const SensorSpectra &s, double now) const;

    QString m_mode = QStringLiteral("live");
    QString m_readout, m_range = QStringLiteral("—");
    QPointF m_hover{-1, -1};
    int m_lastVersion = -1;
    double m_lastPaint = 0;
    bool m_dirty = true;
};

} // namespace corv
