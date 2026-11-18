// SETUP → COMPASS (#subtab-compass-cal): the onboard calibration with its 3D
// view, the compasses the autopilot has, and the large-vehicle calibration
// from a known heading. Logic in CalibrationController ("Calibration"), the
// view in MagCalView.
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Flickable {
    id: page
    contentHeight: col.height + 48
    clip: true
    boundsBehavior: Flickable.StopAtBounds
    ScrollBar.vertical: ScrollBar { }
    onVisibleChanged: Calibration.compassPageShown(visible)
    Component.onCompleted: if (visible) Calibration.compassPageShown(true)

    readonly property real fullW: Math.min(860, width - 40)

    component Swatch: Rectangle {
        property bool square: false
        width: 8; height: 8
        radius: square ? 2 : 4
        anchors.verticalCenter: parent ? parent.verticalCenter : undefined
    }
    component Cell: Text {
        height: 24
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
        color: Theme.textMain
        font.family: Theme.fontData; font.pointSize: 7.875
    }

    Column {
        id: col
        y: 24
        x: (page.width - page.fullW) / 2
        width: page.fullW
        spacing: 20

        SetupPanel {
            id: onboard
            title: "ONBOARD MAG CALIBRATION"
            width: page.fullW
            Row {
                width: parent.width
                spacing: 16
                Column {
                    id: side
                    width: 280
                    spacing: 10
                    CfgNote { text: "Press START, then turn the vehicle slowly through every attitude — nose down, tail down, each side down, upside down — until every bar is full. Keep away from metal, cars and cables." }
                    Row {
                        width: parent.width
                        spacing: 8
                        readonly property real bw: (width - 16) / 3
                        CalButton { width: parent.bw; text: "START"; primary: true; enabled: Calibration.magCanStart; onClicked: Calibration.magStart() }
                        CalButton { width: parent.bw; text: "ACCEPT"; enabled: Calibration.magCanAccept; onClicked: Calibration.magAccept() }
                        CalButton { width: parent.bw; text: "CANCEL"; enabled: Calibration.magCanCancel; onClicked: Calibration.magCancel() }
                    }
                    Column {
                        width: parent.width
                        spacing: 4
                        CfgLabel { text: "FITNESS" }
                        CfgSelect {
                            width: parent.width
                            options: Calibration.magFitOptions
                            value: Calibration.magFit
                            onPicked: (v) => Calibration.setMagFit(v)
                            ToolTip.visible: hovered
                            ToolTip.text: "COMPASS_CAL_FIT: how close to a sphere the samples must be"
                            ToolTip.delay: 600
                        }
                    }
                    CfgCheck {
                        width: parent.width
                        text: "AUTO-ACCEPT A GOOD RESULT"
                        checked: Calibration.magAutosave
                        onToggled: (on) => Calibration.magAutosave = on
                    }
                    Repeater {
                        model: Calibration.magBars
                        delegate: Item {
                            id: bar
                            required property var modelData
                            width: side.width
                            height: 16
                            opacity: modelData.state === "idle" ? 0.4 : 1
                            Row {
                                width: 56
                                anchors.verticalCenter: parent.verticalCenter
                                spacing: 5
                                Swatch { color: bar.modelData.color }
                                Text { text: bar.modelData.name; color: Theme.textMain; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1 }
                            }
                            Rectangle {
                                x: 64
                                width: parent.width - 64 - barText.width - 8
                                height: 10
                                anchors.verticalCenter: parent.verticalCenter
                                radius: 2
                                color: Theme.inputBg
                                border.color: Theme.borderSubtle
                                border.width: Theme.hairline
                                clip: true
                                Rectangle {
                                    width: parent.width * bar.modelData.pct / 100
                                    height: parent.height
                                    color: bar.modelData.state === "ok" ? Theme.accentGreen : bar.modelData.state === "bad" ? Theme.accentRed : Theme.accentCyan
                                    Behavior on width { NumberAnimation { duration: 300 } }
                                }
                            }
                            Text {
                                id: barText
                                anchors.right: parent.right
                                anchors.verticalCenter: parent.verticalCenter
                                width: Math.max(108, implicitWidth)
                                horizontalAlignment: Text.AlignRight
                                text: bar.modelData.text
                                color: Theme.textDim
                                font.family: Theme.fontData; font.pointSize: 7.125
                            }
                        }
                    }
                    CalInstruction { text: Calibration.magInstruction; kind: Calibration.magKind }
                    CalButton {
                        visible: Calibration.magRebootVisible
                        width: parent.width
                        text: "REBOOT AUTOPILOT"
                        onClicked: Calibration.magReboot()
                    }
                }
                Column {
                    width: parent.width - side.width - 16
                    spacing: 6
                    MagCalView {
                        id: view
                        objectName: "magCalView"
                        width: parent.width
                        height: 400
                        HoverHandler { id: viewHover }
                        ToolTip.visible: viewHover.hovered
                        ToolTip.text: "Drag to turn · wheel to zoom · double-click to reset"
                        ToolTip.delay: 1200
                    }
                    Flow {
                        width: parent.width
                        spacing: 12
                        Repeater {
                            model: [[Calibration.magColors[0], "MAG 1", false], [Calibration.magColors[1], "MAG 2", false],
                                    [Calibration.magColors[2], "MAG 3", false], ["#1fbf62", "covered", true],
                                    ["#ffa020", "aim here", true], ["#ffffff", "now", false]]
                            delegate: Row {
                                required property var modelData
                                spacing: 5
                                height: 14
                                Swatch { color: parent.modelData[0]; square: parent.modelData[2] }
                                Text { anchors.verticalCenter: parent.verticalCenter; text: parent.modelData[1]; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10 }
                            }
                        }
                    }
                    Row {
                        width: parent.width
                        spacing: 8
                        CfgSelect {
                            id: focusSel
                            width: 80
                            options: [{ value: "0", label: "MAG 1" }, { value: "1", label: "MAG 2" }, { value: "2", label: "MAG 3" }]
                            value: String(Calibration.magFocus)
                            onPicked: (v) => Calibration.magFocus = parseInt(v)
                            ToolTip.visible: hovered
                            ToolTip.text: "The compass the sphere and its coverage refer to"
                            ToolTip.delay: 600
                        }
                        Text {
                            width: parent.width - focusSel.width - clearBtn.width - resetBtn.width - 24
                            anchors.verticalCenter: parent.verticalCenter
                            text: Calibration.magStats
                            elide: Text.ElideRight
                            color: Theme.textDim
                            font.family: Theme.fontData; font.pixelSize: 10
                        }
                        CalButton { id: clearBtn; height: 25; text: "CLEAR"; onClicked: Calibration.magClear() }
                        CalButton { id: resetBtn; height: 25; text: "RESET VIEW"; onClicked: view.resetView() }
                    }
                }
            }
            CalTable {
                id: resTable
                visible: Calibration.magResults.length > 0
                columns: [["", 0.11], ["RESULT", 0.15], ["FITNESS (mG)", 0.12], ["OFFSETS X / Y / Z", 0.2], ["SCALE", 0.09], ["ORIENTATION", 0.21], ["", 0.12]]
                rows: Calibration.magResults
                rowDelegate: Item {
                    id: rr
                    readonly property var r: parent ? parent.row : ({ cells: [] })
                    readonly property var t: resTable
                    width: t.width
                    height: 24
                    Row {
                        x: 6
                        height: 24
                        spacing: 5
                        Swatch { color: rr.r.color }
                        Cell { text: rr.r.name }
                    }
                    Repeater {
                        model: rr.r.cells
                        delegate: Cell {
                            required property var modelData
                            required property int index
                            x: rr.t.colX(index + 1) + 6
                            width: rr.t.colW(index + 1) - 12
                            text: modelData
                            color: index === 0 ? (rr.r.ok ? Theme.accentGreen : "#ff7070") : Theme.textMain
                        }
                    }
                    Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                }
            }
        }

        SetupPanel {
            title: "COMPASSES"
            width: page.fullW
            CalTable {
                id: compassTable
                columns: Calibration.compassRows.length ? [["PRIORITY", 0.1], ["DEVICE", 0.22], ["BUS", 0.15], ["USE", 0.07], ["MOUNT", 0.11], ["ORIENTATION", 0.15], ["OFFSETS X / Y / Z", 0.2]] : []
                rows: Calibration.compassRows
                rowDelegate: Item {
                    id: cr
                    readonly property var r: parent ? parent.row : ({})
                    readonly property var t: compassTable
                    width: t.width
                    height: 24
                    Cell { x: cr.t.colX(0) + 6; width: cr.t.colW(0) - 12; text: cr.r.prio }
                    Row {
                        x: cr.t.colX(1) + 6
                        width: cr.t.colW(1) - 12
                        spacing: 6
                        Cell { text: cr.r.device }
                        Cell { visible: cr.r.missing; text: "not detected"; color: Theme.accentOrange }
                    }
                    Cell { x: cr.t.colX(2) + 6; width: cr.t.colW(2) - 12; text: cr.r.bus }
                    CfgCheck {
                        x: cr.t.colX(3) + 6
                        anchors.verticalCenter: parent.verticalCenter
                        checked: cr.r.use
                        enabled: cr.r.useKnown
                        opacity: enabled ? 1 : 0.4
                        onToggled: (on) => Calibration.setCompassUse(cr.r.useParam, on)
                        ToolTip.visible: hov.hovered
                        ToolTip.text: cr.r.useParam + ": use this compass for heading"
                        ToolTip.delay: 600
                        HoverHandler { id: hov }
                    }
                    Cell { x: cr.t.colX(4) + 6; width: cr.t.colW(4) - 12; text: cr.r.mount }
                    Cell { x: cr.t.colX(5) + 6; width: cr.t.colW(5) - 12; text: cr.r.orientation }
                    Cell { x: cr.t.colX(6) + 6; width: cr.t.colW(6) - 12; text: cr.r.offsets }
                    Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                }
            }
            Text {
                visible: Calibration.compassNote.length > 0
                width: parent.width
                text: Calibration.compassNote
                color: Theme.textDim
                font.family: Theme.fontData; font.pointSize: 7.875
            }
            CalButton {
                width: parent.width
                text: "READ FROM VEHICLE"
                onClicked: Calibration.readCompassParams()
            }
        }

        SetupPanel {
            title: "LARGE VEHICLE MAGCAL"
            anchors.horizontalCenter: parent.horizontalCenter
            CfgNote { text: "For a vehicle too large to turn: point its nose at a known true heading, with a GPS fix, and enter that heading. The offsets come from the world magnetic model." }
            Column {
                width: parent.width
                spacing: 4
                CfgLabel { text: "NOSE HEADING (° TRUE)" }
                CfgInput {
                    id: yawInput
                    width: parent.width
                    numeric: true
                    text: "0"
                }
            }
            CalButton {
                width: parent.width
                text: "CALIBRATE FROM HEADING"
                enabled: !(Calibration.results.fixed && Calibration.results.fixed.busy)
                onClicked: Calibration.calibrateFromHeading(yawInput.text)
            }
            CalResult { result: Calibration.results.fixed }
        }
    }
}
