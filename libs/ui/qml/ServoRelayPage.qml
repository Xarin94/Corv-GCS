// SETUP → SERVO / RELAY (#subtab-servo-relay, initServoRelay): the 16 servo
// outputs (SERVO_OUTPUT_RAW), four relay toggles (DO_SET_RELAY) and a manual
// servo test (DO_SET_SERVO).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: flow.implicitHeight + 48
    clip: true
    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21
        SetupPanel {
            title: "SERVO OUTPUT"
            width: 620
            Row {
                spacing: 8
                Repeater {
                    model: 16
                    delegate: Column {
                        required property int index
                        readonly property int v: SetupCfg.servos[index] || 0
                        spacing: 3
                        Text { anchors.horizontalCenter: parent.horizontalCenter; text: "S" + (parent.index + 1); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.DemiBold }
                        Rectangle {
                            anchors.horizontalCenter: parent.horizontalCenter
                            width: 22; height: 100; radius: 2
                            color: Theme.inputBg
                            border.color: Theme.borderInput; border.width: Theme.hairline
                            Rectangle {
                                anchors.bottom: parent.bottom
                                width: parent.width
                                height: Math.max(0, Math.min(100, (parent.parent.v - 1000) / 10)) / 100 * parent.height
                                color: Theme.accentCyan
                                opacity: 0.8
                            }
                        }
                        Text { anchors.horizontalCenter: parent.horizontalCenter; text: parent.v; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 9 }
                    }
                }
            }
        }
        SetupPanel {
            title: "RELAY CONTROL"
            Repeater {
                model: 4
                delegate: CfgRow {
                    id: relayRow
                    required property int index
                    readonly property bool on: !!SetupCfg.relays[index]
                    label: "RELAY " + (index + 1)
                    GcsButton {
                        width: 70; height: 26
                        text: relayRow.on ? "ON" : "OFF"
                        primary: relayRow.on
                        onClicked: SetupCfg.toggleRelay(relayRow.index)
                    }
                }
            }
        }
        SetupPanel {
            title: "MANUAL SERVO TEST"
            CfgRow {
                label: "SERVO #"
                CfgSelect { id: servoSel; width: 80; options: ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"]; value: "1"; onPicked: (v) => value = v }
            }
            CfgRow {
                label: "PWM"
                CfgSlider { id: pwm; width: 200; from: 1000; to: 2000; stepSize: 10; value: 1500; valueText: String(Math.round(value)); onMoved: (v) => value = v }
            }
            GcsButton { width: parent.width; height: 29; primary: true; text: "SEND"; onClicked: SetupCfg.servoTest(parseInt(servoSel.value), Math.round(pwm.value)) }
        }
    }
}
