// SETUP → RADIO CAL (#subtab-radio-cal, initRadioCalibration): 16 RC input
// bars centred on 1500, the min / max seen while calibrating; STOP captures
// the trims, SAVE writes RCn_MIN / _MAX / _TRIM.
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: col.implicitHeight + 48
    clip: true
    Column {
        id: col
        x: 24; y: 24
        spacing: 21
        SetupPanel {
            title: "RC INPUT CHANNELS"
            width: 620
            Repeater {
                model: SetupCfg.rc
                delegate: Row {
                    required property int index
                    required property var modelData
                    spacing: 10
                    height: 18
                    Text { width: 40; anchors.verticalCenter: parent.verticalCenter; text: "CH" + (parent.index + 1); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.DemiBold }
                    Rectangle {
                        id: outer
                        width: 380; height: 12
                        anchors.verticalCenter: parent.verticalCenter
                        radius: 2
                        color: Theme.inputBg
                        border.color: Theme.borderInput; border.width: Theme.hairline
                        readonly property real pct: parent.modelData.value ? Math.max(0, Math.min(100, (parent.modelData.value - 1000) / 10)) : 50
                        Rectangle { x: parent.width / 2; width: 1; height: parent.height; color: Theme.textDim }
                        Rectangle {
                            x: Math.min(outer.pct, 50) / 100 * outer.width
                            width: Math.abs(outer.pct - 50) / 100 * outer.width
                            height: parent.height
                            color: Theme.accentCyan
                            opacity: 0.8
                        }
                    }
                    Text { width: 40; anchors.verticalCenter: parent.verticalCenter; horizontalAlignment: Text.AlignRight; text: Theme.tr(parent.modelData.value || 0); color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 11 }
                    Text { anchors.verticalCenter: parent.verticalCenter; text: SetupCfg.rcCalibrating || parent.modelData.min < parent.modelData.max ? parent.modelData.min + "/" + parent.modelData.max : "--/--"; color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 10 }
                }
            }
            Row {
                spacing: 6
                GcsButton {
                    width: 190; height: 29
                    text: SetupCfg.rcCalibrating ? "STOP CALIBRATION" : "START CALIBRATION"
                    frame: SetupCfg.rcCalibrating ? Theme.accentOrange : Theme.borderInput
                    onClicked: SetupCfg.rcCalToggle()
                }
                GcsButton { width: 190; height: 29; primary: true; text: "SAVE CALIBRATION"; onClicked: SetupCfg.rcCalSave() }
                GcsButton { width: 100; height: 29; text: "RESET"; onClicked: SetupCfg.rcCalReset() }
            }
            CfgNote { text: Theme.tr(SetupCfg.rcStatus) }
        }
    }
}
