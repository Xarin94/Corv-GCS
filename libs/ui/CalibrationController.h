// CalibrationController.h - SETUP → ACCELEROMETER, COMPASS, GYRO / BARO
// ("Calibration" QML singleton): the port of js/ui/CalibrationController.js.
//
// The procedures Mission Planner runs, over the same MAVLink exchanges.
//
// Accelerometer, six positions: PREFLIGHT_CALIBRATION (param5 = 1). Once the
// gyros are sampled the autopilot asks for each position with a COMMAND_LONG
// ACCELCAL_VEHICLE_POS, repeated every second; the GCS answers with the same
// command and position when the vehicle is in place. ArduPilot stops sending its
// "Place vehicle ..." texts as soon as a GCS answers that way, so this page
// follows the commands, not the texts. The end is the same command carrying
// SUCCESS or FAILED as the position.
//
// Compass: DO_START_MAG_CAL, then a MAG_CAL_PROGRESS per compass (percentage and
// the 80-section coverage mask) and, once done, a MAG_CAL_REPORT per compass with
// the result. ACCEPT saves a result that was not auto-saved, CANCEL stops. The raw
// samples (RAW_IMU, SCALED_IMU2/3) feed the 3D view (MagCalView).
//
// Level, simple accel, gyros, baro, large-vehicle magcal: one command each, whose
// COMMAND_ACK is the result.
//
// Every command waits for its COMMAND_ACK and says what came back: a refused start
// (armed, unsupported) must not look like a calibration that silently never
// happens. GUI thread.
#pragma once

#include "link/MavMessage.h"
#include "vehicle/MagSamples.h"

#include <QHash>
#include <QObject>
#include <QQmlEngine>
#include <QSet>
#include <QTimer>
#include <QVariantList>
#include <array>
#include <functional>

namespace corv {

class CommandSender;
class MessageBus;
class ParamStore;
class VehicleHub;

class CalibrationController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Calibration)
    QML_SINGLETON

    // ── Accelerometer ──
    Q_PROPERTY(QStringList accelSteps READ accelSteps NOTIFY accelChanged)
    Q_PROPERTY(QString accelInstruction READ accelInstruction NOTIFY accelChanged)
    Q_PROPERTY(QString accelKind READ accelKind NOTIFY accelChanged)
    Q_PROPERTY(QString accelButton READ accelButton NOTIFY accelChanged)
    Q_PROPERTY(bool accelButtonEnabled READ accelButtonEnabled NOTIFY accelChanged)
    Q_PROPERTY(bool accelWaiting READ accelWaiting NOTIFY accelChanged)
    Q_PROPERTY(bool accelActive READ accelActive NOTIFY accelChanged)
    Q_PROPERTY(QString accelLog READ accelLog NOTIFY accelChanged)
    Q_PROPERTY(QVariantList accelTable READ accelTable NOTIFY paramsChanged)
    // ── One-command calibrations: id -> {text, kind, busy} ──
    Q_PROPERTY(QVariantMap results READ results NOTIFY resultsChanged)
    // ── Compass ──
    Q_PROPERTY(bool magActive READ magActive NOTIFY magChanged)
    Q_PROPERTY(bool magCanStart READ magCanStart NOTIFY magChanged)
    Q_PROPERTY(bool magCanAccept READ magCanAccept NOTIFY magChanged)
    Q_PROPERTY(bool magCanCancel READ magCanCancel NOTIFY magChanged)
    Q_PROPERTY(bool magRebootVisible READ magRebootVisible NOTIFY magChanged)
    Q_PROPERTY(QString magInstruction READ magInstruction NOTIFY magChanged)
    Q_PROPERTY(QString magKind READ magKind NOTIFY magChanged)
    Q_PROPERTY(QVariantList magBars READ magBars NOTIFY magChanged)
    Q_PROPERTY(QVariantList magResults READ magResults NOTIFY magChanged)
    Q_PROPERTY(QString magStats READ magStats NOTIFY statsChanged)
    Q_PROPERTY(int magFocus READ magFocus WRITE setMagFocus NOTIFY magFocusChanged)
    Q_PROPERTY(bool magAutosave READ magAutosave WRITE setMagAutosave NOTIFY magAutosaveChanged)
    Q_PROPERTY(QVariantList magFitOptions READ magFitOptions NOTIFY paramsChanged)
    Q_PROPERTY(QString magFit READ magFit NOTIFY paramsChanged)
    Q_PROPERTY(QVariantList compassRows READ compassRows NOTIFY paramsChanged)
    Q_PROPERTY(QString compassNote READ compassNote NOTIFY paramsChanged)
    Q_PROPERTY(QStringList magColors READ magColors CONSTANT)

public:
    CalibrationController(MessageBus *bus, CommandSender *cmd, ParamStore *params, VehicleHub *hub, QObject *parent);
    static CalibrationController *s_instance;
    static CalibrationController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// The samples the 3D view draws; magViewChanged() when it should repaint.
    const MagSamples &samples() const { return m_samples; }
    static const QStringList &magColorList();

    QStringList accelSteps() const;
    QString accelInstruction() const { return m_accelText; }
    QString accelKind() const { return m_accelKind; }
    QString accelButton() const;
    bool accelButtonEnabled() const;
    bool accelWaiting() const { return m_accel.phase == Position; }
    bool accelActive() const;
    QString accelLog() const { return m_accelLog.join(QLatin1Char('\n')); }
    QVariantList accelTable() const { return m_accelTable; }
    QVariantMap results() const { return m_results; }

    bool magActive() const { return m_mag.running; }
    bool magCanStart() const { return m_magButtons[0]; }
    bool magCanAccept() const { return m_magButtons[1]; }
    bool magCanCancel() const { return m_magButtons[2]; }
    bool magRebootVisible() const { return m_magReboot; }
    QString magInstruction() const { return m_magText; }
    QString magKind() const { return m_magKind; }
    QVariantList magBars() const;
    QVariantList magResults() const;
    QString magStats() const { return m_magStats; }
    int magFocus() const { return m_samples.focus(); }
    void setMagFocus(int i);
    bool magAutosave() const { return m_magAutosave; }
    void setMagAutosave(bool on);
    QVariantList magFitOptions() const { return m_fitOptions; }
    QString magFit() const { return m_fitValue; }
    QVariantList compassRows() const { return m_compassRows; }
    QString compassNote() const { return m_compassNote; }
    QStringList magColors() const { return magColorList(); }

    // Accelerometer
    /// CALIBRATE ACCEL, or CONTINUE once the autopilot asks for a position.
    Q_INVOKABLE void accelPrimary();
    Q_INVOKABLE void accelAbort();
    Q_INVOKABLE void calibrateLevel();
    Q_INVOKABLE void calibrateSimple();
    Q_INVOKABLE void readAccelParams();
    // Gyro / baro
    Q_INVOKABLE void calibrateGyro();
    Q_INVOKABLE void calibrateBaro();
    // Compass
    Q_INVOKABLE void magStart();
    Q_INVOKABLE void magAccept();
    Q_INVOKABLE void magCancel();
    Q_INVOKABLE void magReboot();
    Q_INVOKABLE void setMagFit(const QString &value);
    Q_INVOKABLE void setCompassUse(const QString &param, bool on);
    Q_INVOKABLE void readCompassParams();
    Q_INVOKABLE void magClear();
    Q_INVOKABLE void calibrateFromHeading(const QString &yawText);
    // The pages were opened / closed: parameters read, the 3D view fed
    Q_INVOKABLE void accelPageShown();
    Q_INVOKABLE void compassPageShown(bool shown);

signals:
    void accelChanged();
    void resultsChanged();
    void magChanged();
    void statsChanged();
    void magFocusChanged();
    void magAutosaveChanged();
    void paramsChanged();
    /// The samples, the focus or the coverage changed: the 3D view repaints.
    void magViewChanged();

private:
    enum AccelPhase { Idle, Starting, Waiting, Position, Sampling, Done, Failed };
    struct Accel {
        AccelPhase phase = Idle;
        int pos = 0;
        QSet<int> done;
        int confirmedPos = 0;
        qint64 confirmedAt = 0;
    };
    struct MagProgress {
        bool has = false;
        int status = 0;
        double pct = 0;
        int attempt = 0;
        geogrid::Mask mask{};
    };
    struct MagReport {
        bool has = false;
        int status = 0;
        bool autosaved = false;
        float fitness = 0, ofsX = 0, ofsY = 0, ofsZ = 0, scale = 0;
        int oldOrientation = -1, newOrientation = -1;
    };
    struct Mag {
        bool running = false;
        bool starting = false;
        int calMask = 0;
        std::array<MagProgress, 3> progress;
        std::array<MagReport, 3> reports;
        QString said;
    };

    bool linked() const;
    void onStatusText(const QString &text);
    void requestParams(const QStringList &names, bool force);
    void rebuildParamViews();
    double param(const QString &name, bool *has = nullptr) const;

    // Accelerometer
    void startAccel();
    void continueAccel();
    void onAccelPosition(int pos);
    void finishAccel(bool ok, const QString &why = {});
    void failAccel(const QString &text);
    void renderAccel(const QString *instruction = nullptr);
    void setAccelText(const QString &text);
    void rebuildAccelTable();

    // One-command calibrations
    using AckCommand = std::function<void(std::function<void(bool, int)>)>;
    void runOneShot(const QString &id, const AckCommand &command, const QString &running, const QString &done,
                    std::function<void()> after = {});
    void setResult(const QString &id, const QString &text, const QString &kind, bool busy);

    // Compass
    void onMagProgress(const MavMessage &m);
    void onMagReport(const MavMessage &m);
    void onMagSample(int compass, float x, float y, float z);
    QList<int> calibratingIds() const;
    void finishMag();
    void raiseRates();
    void restoreRates();
    void refreshCoverage();
    void updateMagInstruction();
    void updateStats();
    void setMagButtons(bool start, bool accept, bool cancel);
    void setMagText(const QString &text, const QString &kind);
    void rebuildCompassRows();
    void rebuildFitOptions();

    MessageBus *m_bus;
    CommandSender *m_cmd;
    ParamStore *m_params;
    VehicleHub *m_hub;

    Accel m_accel;
    int m_accelRun = 0;          // drops the answers of a run that was replaced
    qint64 m_accelEndedAt = 0;
    qint64 m_magEndedAt = 0;   // a late progress of a cancelled run is not a new one
    QTimer m_accelTimer;
    QString m_accelText = QStringLiteral("Press CALIBRATE ACCEL to start.");
    QString m_accelKind;
    QStringList m_accelLog;
    QVariantList m_accelTable;

    QVariantMap m_results;
    QHash<QString, QString> m_oneShotSaid;   // the one-shots running, and the last calibration text each heard
    QHash<QString, int> m_lastOneShotCommand;  // for the timeout text

    Mag m_mag;
    MagSamples m_samples;
    std::array<bool, 3> m_magButtons{true, false, false};
    bool m_magReboot = false;
    bool m_magAutosave = true;
    bool m_compassShown = false;
    QString m_magText = QStringLiteral("Not running.");
    QString m_magKind;
    QString m_magStats = QStringLiteral("No samples");
    QTimer m_magFinish;
    QTimer m_statsTimer;
    QTimer m_fitTimer;
    QVariantList m_fitOptions;
    QString m_fitValue = QStringLiteral("16");
    QVariantList m_compassRows;
    QString m_compassNote;
};

} // namespace corv
