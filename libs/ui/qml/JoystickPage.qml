// SETUP → JOYSTICK / RC (#subtab-joystick, JoystickUI.js): the gamepad and the
// RC override, the mapping of 8 or more inputs to RC channels with a live bar,
// the PWM of the 18 channels, two servo buttons and the look buttons of the
// first-person view. The logic is in JoystickController (singleton Joystick).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentWidth: width
    contentHeight: flow.implicitHeight + 48
    clip: true
    ScrollBar.vertical: ScrollBar { }

    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21

        SetupPanel {
            title: "JOYSTICK / GAMEPAD"
            CfgCheck {
                text: "ENABLE RC OVERRIDE"
                checked: Joystick.enabled
                onToggled: (c) => Joystick.setEnabled(c)
            }
            CfgRow {
                label: "GAMEPAD"
                CfgSelect {
                    width: 210
                    options: [{ value: "-1", label: Joystick.devices.length ? "-- Select gamepad --" : "-- No gamepad detected --" }]
                             .concat(Joystick.devices.map(d => ({ value: String(d.id), label: d.label })))
                    value: String(Joystick.selected)
                    onPicked: (v) => Joystick.select(parseInt(v))
                }
                GcsButton { width: 54; height: 26; text: "SCAN"; onClicked: Joystick.scan() }
            }
            CfgRow {
                label: "SEND RATE"
                CfgSelect {
                    width: 120
                    options: [{ value: "50", label: "50 Hz" }, { value: "25", label: "25 Hz" }, { value: "10", label: "10 Hz" }]
                    value: String(Joystick.sendRate)
                    onPicked: (v) => Joystick.setSendRate(parseInt(v))
                }
            }
            CfgRow {
                label: "STATUS"
                Text {
                    text: Joystick.status
                    color: Joystick.statusKind === "active" ? "#3dff6e" : Joystick.statusKind === "warning" ? "#ffaa00" : Theme.textMain
                    font.family: Theme.fontData
                    font.pixelSize: 11
                    wrapMode: Text.Wrap
                    width: 260
                }
            }
            CfgNote {
                visible: !Joystick.available
                color: "#ff3333"
                text: "This build has no SDL3: gamepads and joysticks are not available."
            }
        }

        SetupPanel {
            title: "RC INPUT MAPPING (" + Joystick.inputs.length + " AXES)"
            width: Math.min(700, flow.width)
            CfgNote {
                text: "Choose a physical axis or an analog button/trigger for each input. Button: released = 1000, pressed = 2000 PWM. INV reverses the range. Unavailable inputs release their channel."
            }
            Repeater {
                model: Joystick.inputs
                delegate: Row {
                    id: row
                    required property var modelData
                    required property int index
                    readonly property var rawValue: Joystick.raw[index]
                    spacing: 6
                    height: 28
                    Text {
                        width: 34
                        anchors.verticalCenter: parent.verticalCenter
                        text: "IN " + (row.index + 1)
                        color: Theme.textDim
                        font.family: Theme.fontUi
                        font.pixelSize: 11
                    }
                    CfgSelect {
                        width: 118
                        options: Joystick.sourceOptions
                        value: row.modelData.source
                        onPicked: (v) => Joystick.setInput(row.index, { source: v })
                    }
                    CfgSelect {
                        width: 64
                        options: [{ value: "0", label: "--" }].concat(Array.from({ length: 18 }, (_, i) => ({ value: String(i + 1), label: "CH" + (i + 1) })))
                        value: String(row.modelData.channel)
                        onPicked: (v) => Joystick.setInput(row.index, { channel: parseInt(v) })
                    }
                    CfgCheck {
                        text: "INV"
                        anchors.verticalCenter: parent.verticalCenter
                        checked: row.modelData.inverted
                        onToggled: (c) => Joystick.setInput(row.index, { inverted: c })
                    }
                    CfgSlider {
                        width: 92
                        anchors.verticalCenter: parent.verticalCenter
                        from: 0; to: 0.9; stepSize: 0.01
                        value: row.modelData.deadzone
                        valueText: "DZ " + Math.round(value * 100) + "%"
                        onMoved: (v) => Joystick.setInput(row.index, { deadzone: v })
                    }
                    // -1..1 as a bar growing from the centre
                    Rectangle {
                        width: 90; height: 8
                        anchors.verticalCenter: parent.verticalCenter
                        color: Qt.rgba(1, 1, 1, 0.06)
                        border.color: Theme.borderInput
                        Rectangle { x: parent.width / 2; width: 1; height: parent.height; color: Theme.textDim }
                        Rectangle {
                            readonly property real v: row.rawValue === null || row.rawValue === undefined ? 0 : row.rawValue
                            x: v >= 0 ? parent.width / 2 : parent.width / 2 * (1 + v)
                            width: Math.abs(v) * parent.width / 2
                            height: parent.height
                            color: "#75cffa"
                        }
                    }
                    Text {
                        width: 40
                        anchors.verticalCenter: parent.verticalCenter
                        horizontalAlignment: Text.AlignRight
                        text: row.rawValue === null || row.rawValue === undefined ? "--" : row.rawValue.toFixed(2)
                        color: Theme.textMain
                        font.family: Theme.fontData
                        font.pixelSize: 11
                    }
                }
            }
        }

        SetupPanel {
            title: "CHANNEL OUTPUT (PWM)"
            width: 450
            Grid {
                columns: 6
                spacing: 6
                Repeater {
                    model: 18
                    delegate: Rectangle {
                        required property int index
                        readonly property int pwm: Joystick.channels[index] || 0
                        width: 62; height: 38
                        color: pwm ? Qt.rgba(0.46, 0.81, 0.98, 0.12) : Qt.rgba(1, 1, 1, 0.04)
                        border.color: pwm ? "#75cffa" : Theme.borderInput
                        Column {
                            anchors.centerIn: parent
                            Text { anchors.horizontalCenter: parent.horizontalCenter; text: "CH" + (index + 1); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10 }
                            Text { anchors.horizontalCenter: parent.horizontalCenter; text: pwm ? pwm : "--"; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 12 }
                        }
                    }
                }
            }
        }

        SetupPanel {
            title: "SERVO BUTTONS"
            CfgCheck {
                text: "ENABLE SERVO BUTTONS"
                checked: Joystick.servoButtons
                onToggled: (c) => Joystick.setServoButtons(c)
            }
            CfgNote {
                text: "Each press sends the chosen position once; release leaves it there. Works independently of RC override while connected. 0–100% = 1000–2000 µs. Use an output configured to accept servo commands."
            }
            Repeater {
                model: Joystick.servos
                delegate: Row {
                    id: srow
                    required property var modelData
                    required property int index
                    spacing: 6
                    height: 30
                    Text { width: 46; anchors.verticalCenter: parent.verticalCenter; text: "CMD " + (srow.index + 1); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 11 }
                    CfgSelect {
                        width: 110
                        options: Joystick.buttonOptions
                        value: String(srow.modelData.button)
                        onPicked: (v) => Joystick.setServo(srow.index, { button: parseInt(v) })
                    }
                    CfgSelect {
                        width: 90
                        options: Array.from({ length: 16 }, (_, n) => ({ value: String(n + 1), label: "Servo " + (n + 1) }))
                        value: String(srow.modelData.servo)
                        onPicked: (v) => Joystick.setServo(srow.index, { servo: parseInt(v) })
                    }
                    CfgInput {
                        width: 52
                        numeric: true
                        text: String(srow.modelData.percent)
                        onCommitted: (t) => Joystick.setServo(srow.index, { percent: parseFloat(t) })
                    }
                    Text { anchors.verticalCenter: parent.verticalCenter; text: "%"; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 11 }
                }
            }
            CfgRow {
                label: "STATUS"
                Text { text: Joystick.servoStatus; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 11; width: 240; wrapMode: Text.Wrap }
            }
        }

        SetupPanel {
            title: "FPV LOOK BUTTONS"
            CfgNote {
                text: "Hold to look around, release to return to center, like the arrow keys. Active in the Flight Data first-person view, also with RC override disabled."
            }
            Repeater {
                model: ["up", "down", "left", "right"]
                delegate: CfgRow {
                    required property string modelData
                    label: modelData.toUpperCase()
                    CfgSelect {
                        width: 140
                        options: Joystick.buttonOptions
                        value: String(Joystick.view[modelData])
                        onPicked: (v) => Joystick.setViewButton(modelData, parseInt(v))
                    }
                }
            }
            CfgRow {
                label: "PRESSED BUTTONS"
                Text { text: Joystick.pressed + "  (numbers match the selectors)"; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 11 }
            }
        }
    }
}
