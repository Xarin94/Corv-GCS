// SETUP → FLIGHT MODES (#subtab-flight-modes, initFlightModes): the mode
// channel and the six FLTMODE slots with their PWM bands.
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: col.implicitHeight + 48
    clip: true
    readonly property var bands: ["PWM 1000-1230", "PWM 1231-1360", "PWM 1361-1490", "PWM 1491-1620", "PWM 1621-1749", "PWM 1750-2000"]
    readonly property var defaults: ["0", "2", "5", "3", "6", "9"]
    Column {
        id: col
        x: 24; y: 24
        spacing: 21
        SetupPanel {
            title: "FLIGHT MODES"
            width: 420
            ParamSelect {
                id: chSel
                label: "MODE CHANNEL"
                param: "FLTMODE_CH"
                fallback: "5"
                fieldWidth: 160
                options: [{ value: "5", label: "RC Channel 5" }, { value: "6", label: "RC Channel 6" },
                          { value: "7", label: "RC Channel 7" }, { value: "8", label: "RC Channel 8" }]
            }
            Repeater {
                id: modes
                model: 6
                delegate: Row {
                    required property int index
                    readonly property string value: sel.value
                    spacing: 10
                    ParamSelect {
                        id: sel
                        width: 270
                        label: "MODE " + (parent.index + 1)
                        param: "FLTMODE" + (parent.index + 1)
                        fallback: page.defaults[parent.index]
                        fieldWidth: 160
                        options: SetupCfg.flightModeOptions
                    }
                    Text {
                        anchors.verticalCenter: parent.verticalCenter
                        text: page.bands[parent.index]
                        color: Theme.textDim
                        font.family: Theme.fontData
                        font.pixelSize: 10
                    }
                }
            }
            Row {
                spacing: 6
                GcsButton {
                    width: 190; height: 29
                    text: "READ FROM VEHICLE"
                    onClicked: SetupCfg.readParams(["FLTMODE_CH", "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5", "FLTMODE6"], "FLIGHT MODES")
                }
                GcsButton {
                    width: 190; height: 29
                    primary: true
                    text: "WRITE TO VEHICLE"
                    onClicked: {
                        const v = { FLTMODE_CH: chSel.value }
                        for (let i = 0; i < 6; ++i) v["FLTMODE" + (i + 1)] = modes.itemAt(i).value
                        SetupCfg.writeParams(v, "FLIGHT MODES", "Written %1 flight mode parameters")
                    }
                }
            }
            CfgNote { text: SetupCfg.status["FLIGHT MODES"] ? SetupCfg.status["FLIGHT MODES"].text : "" }
        }
    }
}
