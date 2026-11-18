// JoystickController.h - SETUP → JOYSTICK / RC (JoystickManager.js and
// JoystickUI.js; QML singleton "Joystick").
//
// The selected device (Gamepad, SDL3) is read at 100 Hz: each input row maps
// an axis or an analog button to an RC channel (inversion, deadzone, expo,
// 1000-2000 PWM). With the override on, RC_CHANNELS_OVERRIDE goes out at 10,
// 25 or 50 Hz; unmapped channels are 0, released to the receiver. Turning it
// on writes SYSID_MYGCS = 255 (ArduPilot takes overrides only from its GCS),
// RC_OVERRIDE_TIME = 3 s, clears the ignore-override bit of RC_OPTIONS and
// FS_THR_ENABLE = 0 (no receiver), each read first; turning it off writes the
// saved values back. No device data for 500 ms (unplugged): the channels are
// released and the override waits, suspended, for the same device to return.
//
// Buttons: two servo commands (a press sends DO_SET_SERVO once, armed per
// session, never persisted) and the four look directions of the first-person
// view (as the arrow keys). Buttons count only while the window is active.
// Configuration under 'datad-joystick-config', the Electron key. GUI thread.
#pragma once

#include "input/Gamepad.h"
#include "input/JoystickMath.h"

#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <array>
#include <optional>

namespace corv {

class CommandSender;
class VehicleHub;

class JoystickController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Joystick)
    QML_SINGLETON

    Q_PROPERTY(bool available READ available CONSTANT)                    // SDL3 in this build
    Q_PROPERTY(QVariantList devices READ devices NOTIFY devicesChanged)    // [{ id, label }]
    Q_PROPERTY(int selected READ selected NOTIFY stateChanged)            // device id, -1 none
    Q_PROPERTY(bool enabled READ enabled NOTIFY stateChanged)
    Q_PROPERTY(QString status READ status NOTIFY stateChanged)
    Q_PROPERTY(QString statusKind READ statusKind NOTIFY stateChanged)    // "", warning, active
    Q_PROPERTY(int sendRate READ sendRate NOTIFY stateChanged)
    Q_PROPERTY(QVariantList inputs READ inputs NOTIFY configChanged)      // [{ source, channel, inverted, deadzone }]
    Q_PROPERTY(QVariantList sourceOptions READ sourceOptions NOTIFY configChanged)
    Q_PROPERTY(QVariantList buttonOptions READ buttonOptions NOTIFY configChanged)
    Q_PROPERTY(QVariantList servos READ servos NOTIFY configChanged)      // [{ button, servo, percent }]
    Q_PROPERTY(QVariantMap view READ view NOTIFY configChanged)           // up, down, left, right → button
    Q_PROPERTY(bool servoButtons READ servoButtons NOTIFY stateChanged)
    Q_PROPERTY(QString servoStatus READ servoStatus NOTIFY stateChanged)
    Q_PROPERTY(QVariantList raw READ raw NOTIFY liveChanged)              // per input -1..1, null when unavailable
    Q_PROPERTY(QVariantList channels READ channels NOTIFY liveChanged)    // 18 PWM, 0 released
    Q_PROPERTY(QString pressed READ pressed NOTIFY liveChanged)

public:
    JoystickController(CommandSender *cmd, VehicleHub *hub, QObject *parent = nullptr);
    ~JoystickController() override;

    static JoystickController *s_instance;
    static JoystickController *create(QQmlEngine *, QJSEngine *)
    {
        QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership);
        return s_instance;
    }

    Q_INVOKABLE void scan();
    Q_INVOKABLE void select(int id);
    Q_INVOKABLE void setEnabled(bool on);
    Q_INVOKABLE void setSendRate(int hz);
    Q_INVOKABLE void setInput(int index, const QVariantMap &patch);   // source "axis:2", channel, inverted, deadzone, expo
    Q_INVOKABLE void setServo(int index, const QVariantMap &patch);   // button, servo, percent
    Q_INVOKABLE void setViewButton(const QString &direction, int button);
    Q_INVOKABLE void setServoButtons(bool on);

    bool available() const { return Gamepad::available(); }
    QVariantList devices() const { return m_deviceList; }
    int selected() const { return m_pad.isOpen() ? int(m_pad.device().id) : -1; }
    bool enabled() const { return m_enabled; }
    QString status() const;
    QString statusKind() const;
    int sendRate() const { return m_sendRate; }
    QVariantList inputs() const;
    QVariantList sourceOptions() const;
    QVariantList buttonOptions() const;
    QVariantList servos() const;
    QVariantMap view() const;
    bool servoButtons() const { return m_servoButtons; }
    QString servoStatus() const { return m_servoStatus; }
    QVariantList raw() const;
    QVariantList channels() const;
    QString pressed() const;

signals:
    void devicesChanged();
    void stateChanged();
    void configChanged();
    void liveChanged();

private:
    void poll();
    void send();
    void pollButtons(const std::vector<double> &buttons);
    void releaseAll();
    void resetButtons();
    void suspend(const QString &why);
    bool reselect();
    void load();
    void save();
    void applyParameters();
    void restoreParameters();

    CommandSender *m_cmd;
    VehicleHub *m_hub;
    Gamepad m_pad;
    QString m_padName;                  // the device we use, picked again by name after a replug
    QVariantList m_deviceList;
    QTimer m_pollTimer, m_sendTimer, m_scanTimer;
    bool m_enabled = false, m_suspended = false;
    bool m_rcActive = false;
    int m_sendRate = 25;
    qint64 m_lastData = 0, m_lastPreview = 0;
    std::vector<joystick::AxisConfig> m_axes;
    std::array<joystick::ServoConfig, 2> m_servoMap;
    std::array<int, 4> m_viewMap{{-1, -1, -1, -1}};   // up, down, left, right
    bool m_servoButtons = false;
    QString m_servoStatus = QStringLiteral("Disabled");
    std::array<uint16_t, joystick::kChannels> m_channels{};
    std::vector<std::optional<double>> m_raw;
    std::vector<int> m_pressed;
    std::optional<std::vector<bool>> m_prevButtons;
    QString m_buttonTarget;
    struct Saved {
        QString id;
        float value = 0;
        int type = 0;
    };
    std::vector<Saved> m_saved;          // parameters to write back
    int m_paramSession = 0;
};

} // namespace corv
