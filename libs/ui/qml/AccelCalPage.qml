// SETUP → ACCELEROMETER (#subtab-accel-cal): the six positions, LEVEL, SIMPLE
// ACCEL CAL and the stored calibration. Logic in CalibrationController
// ("Calibration").
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Flickable {
    id: page
    contentHeight: col.height + 48
    clip: true
    boundsBehavior: Flickable.StopAtBounds
    ScrollBar.vertical: ScrollBar { }
    onVisibleChanged: if (visible) Calibration.accelPageShown()

    // ACCELCAL_VEHICLE_POS 1..6: a side view (nose right) or a view from behind
    readonly property var positions: [
        { title: "LEVEL", caption: "flat, as it flies", view: "side", rot: 0, lift: 5 },
        { title: "LEFT SIDE", caption: "left side down", view: "rear", rot: -90, lift: 25 },
        { title: "RIGHT SIDE", caption: "right side down", view: "rear", rot: 90, lift: 25 },
        { title: "NOSE DOWN", caption: "nose pointing at the ground", view: "side", rot: 90, lift: 32 },
        { title: "NOSE UP", caption: "nose pointing at the sky", view: "side", rot: -90, lift: 25 },
        { title: "BACK", caption: "upside down", view: "side", flip: true, lift: 16.5 }
    ]
    readonly property real fullW: Math.min(860, width - 40)

    Column {
        id: col
        y: 24
        x: (page.width - page.fullW) / 2
        width: page.fullW
        spacing: 20

        SetupPanel {
            title: "ACCELEROMETER CALIBRATION"
            width: page.fullW
            CfgNote { text: qsTr("Six positions, disarmed. When a position lights up, hold the vehicle that way and still on a flat surface, then press CONTINUE. The gyros are measured first: keep it still when you start.") }
            Row {
                id: steps
                width: parent.width
                spacing: 8
                topPadding: 2
                readonly property real stepW: (width - 5 * spacing) / 6
                Repeater {
                    model: page.positions
                    delegate: Rectangle {
                        id: step
                        required property var modelData
                        required property int index
                        readonly property string state: Calibration.accelSteps[index] || ""
                        readonly property bool lit: state !== ""
                        width: steps.stepW
                        height: stepCol.implicitHeight + 14
                        radius: 4
                        color: state === "current" ? Qt.rgba(0, 210 / 255, 1, 0.07) : "transparent"
                        border.width: state === "current" ? 2 * Theme.hairline + 1 : Theme.hairline + 0.4
                        border.color: state === "current" ? Theme.accentCyan : state === "sampling" ? Theme.accentOrange
                                    : state === "done" ? Qt.rgba(68 / 255, 1, 68 / 255, 0.35) : Theme.borderSubtle
                        Behavior on border.color { ColorAnimation { duration: 200 } }
                        Column {
                            id: stepCol
                            x: 6; y: 6
                            width: parent.width - 12
                            spacing: 2
                            AccelGlyph {
                                width: parent.width
                                view: step.modelData.view
                                rot: step.modelData.rot || 0
                                flip: !!step.modelData.flip
                                lift: step.modelData.lift
                                ink: step.lit ? Theme.textMain : Theme.textDim
                            }
                            Row {
                                anchors.horizontalCenter: parent.horizontalCenter
                                spacing: 5
                                Rectangle {
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: 14; height: 14; radius: 7
                                    color: step.state === "current" ? Theme.accentCyan : step.state === "done" ? Theme.accentGreen : Theme.borderSubtle
                                    Text {
                                        anchors.centerIn: parent
                                        text: step.index + 1
                                        color: step.state === "current" || step.state === "done" ? "#000000" : (step.lit ? Theme.textMain : Theme.textDim)
                                        font.family: Theme.fontUi; font.pixelSize: 9; font.weight: Font.Bold
                                    }
                                }
                                Text {
                                    text: Theme.tr(step.modelData.title)
                                    color: step.lit ? Theme.textMain : Theme.textDim
                                    font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.Bold; font.letterSpacing: 1
                                }
                            }
                            Text {
                                width: parent.width
                                horizontalAlignment: Text.AlignHCenter
                                wrapMode: Text.Wrap
                                text: step.modelData.caption + (step.modelData.view === "rear" ? qsTr(" · seen from behind") : "")
                                color: step.lit ? Theme.textMain : Theme.textDim
                                opacity: 0.75
                                font.family: Theme.fontUi; font.pixelSize: 9
                            }
                        }
                    }
                }
            }
            CalInstruction { text: Calibration.accelInstruction; kind: Calibration.accelKind }
            Row {
                width: parent.width
                spacing: 8
                CalButton {
                    width: Calibration.accelActive ? (parent.width - 8) / 2 : parent.width
                    text: Calibration.accelButton
                    primary: true
                    armed: Calibration.accelWaiting
                    enabled: Calibration.accelButtonEnabled
                    onClicked: Calibration.accelPrimary()
                }
                CalButton {
                    visible: Calibration.accelActive
                    width: (parent.width - 8) / 2
                    text: "ABORT"
                    onClicked: Calibration.accelAbort()
                }
            }
            Text {
                width: parent.width
                visible: text.length > 0
                text: Theme.tr(Calibration.accelLog)
                wrapMode: Text.Wrap
                color: Theme.textDim
                font.family: Theme.fontData; font.pixelSize: 10
                lineHeight: 1.3
            }
        }

        Row {
            anchors.horizontalCenter: parent.horizontalCenter
            spacing: 20
            SetupPanel {
                title: "LEVEL"
                CfgNote { text: qsTr("Sets AHRS_TRIM so that the attitude reads level when the vehicle sits level. Place it level and still, as it flies.") }
                CalButton {
                    width: parent.width
                    text: "CALIBRATE LEVEL"
                    enabled: !(Calibration.results.level && Calibration.results.level.busy)
                    onClicked: Calibration.calibrateLevel()
                }
                CalResult { result: Calibration.results.level }
            }
            SetupPanel {
                title: "SIMPLE ACCEL CAL"
                CfgNote { text: qsTr("One position only, for vehicles too large to turn over: level and still. Less accurate than the six positions; Copter firmware.") }
                CalButton {
                    width: parent.width
                    text: "SIMPLE ACCEL CAL"
                    enabled: !(Calibration.results.simple && Calibration.results.simple.busy)
                    onClicked: Calibration.calibrateSimple()
                }
                CalResult { result: Calibration.results.simple }
            }
        }

        SetupPanel {
            title: "STORED CALIBRATION"
            width: page.fullW
            CalTable {
                columns: [["", 0.12], ["OFFSETS X / Y / Z (m/s²)", 0.36], ["SCALE X / Y / Z", 0.32], ["", 0.2]]
                rows: Calibration.accelTable
            }
            CalButton {
                width: parent.width
                text: "READ FROM VEHICLE"
                onClicked: Calibration.readAccelParams()
            }
        }
    }
}
