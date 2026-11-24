// FFT bar (#fft-panel, FftPanel.js): the strongest lines of the last 2 s, the
// newest spectrum with its 10 s peak hold, the last 60 s as a waterfall. One
// GYRO | ACC switch drives both charts. Logic in FftController ("Fft"), the
// charts are SpectrumView items.
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Corv.Ui

LeftBar {
    id: bar
    title: "VIBRATION FFT"
    window: "LAST 2 s"

    component Small: Text {
        color: "#c3c2b7"
        font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.DemiBold; font.letterSpacing: 0.06 * Theme.fsXs
    }
    component SectionTitle: Text {
        color: "#f0efec"
        font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.Bold; font.letterSpacing: 0.12 * Theme.fsXs
    }
    component Rule: Rectangle { Layout.fillWidth: true; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.10) }

    // ── The strongest lines ──
    Rule { }
    GridLayout {
        Layout.fillWidth: true
        columns: 2
        columnSpacing: 10
        rowSpacing: 4
        Repeater {
            model: Fft.peaks
            delegate: Item {
                id: peak
                required property var modelData
                required property int index
                Layout.columnSpan: 2
                Layout.fillWidth: true
                implicitHeight: peakCol.implicitHeight
                Text {
                    id: peakLbl
                    width: 34
                    text: Theme.tr(peak.modelData.label)
                    color: "#c3c2b7"
                    font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.Bold; font.letterSpacing: 0.12 * Theme.fsXs
                }
                Column {
                    id: peakCol
                    x: 44
                    width: parent.width - x
                    Text {
                        width: parent.width
                        visible: !!peak.modelData.none
                        text: Theme.tr(peak.modelData.none || "")
                        wrapMode: Text.Wrap
                        color: "#898781"
                        font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.letterSpacing: 0.06 * Theme.fsXs
                    }
                    Text {
                        width: parent.width
                        visible: !!peak.modelData.main
                        textFormat: Text.StyledText
                        text: Theme.tr(peak.modelData.main || "")
                        wrapMode: Text.Wrap
                        color: "#f0efec"
                        font.family: Theme.fontData; font.pixelSize: Theme.fsSm
                    }
                    Text {
                        width: parent.width
                        visible: !!peak.modelData.more
                        text: Theme.tr(peak.modelData.more || "")
                        wrapMode: Text.Wrap
                        color: peak.modelData.moreDim ? "#898781" : "#c3c2b7"
                        font.family: peak.modelData.moreDim ? Theme.fontUi : Theme.fontData
                        font.pixelSize: Theme.fsXs
                    }
                }
            }
        }
    }

    // ── The newest spectrum ──
    Rule { }
    RowLayout {
        Layout.fillWidth: true
        SectionTitle { text: qsTr("SPECTRUM · NOW") }
        Item { Layout.fillWidth: true }
        // .spec-seg: GYRO | ACC
        Rectangle {
            implicitWidth: seg.implicitWidth + 2
            implicitHeight: seg.implicitHeight + 2
            radius: 3
            color: "transparent"
            border.width: Theme.hairline + 0.3
            border.color: Qt.rgba(1, 1, 1, 0.22)
            clip: true
            Row {
                id: seg
                x: 1; y: 1
                Repeater {
                    model: [["gyro", "GYRO"], ["accel", "ACC"]]
                    delegate: Rectangle {
                        required property var modelData
                        readonly property bool active: Fft.sensor === modelData[0]
                        width: segText.implicitWidth + 16
                        height: segText.implicitHeight + 2
                        color: active ? Qt.rgba(57 / 255, 135 / 255, 229 / 255, 0.25) : "transparent"
                        Text {
                            id: segText
                            anchors.centerIn: parent
                            text: Theme.tr(parent.modelData[1])
                            color: parent.active || segMouse.containsMouse ? "#f0efec" : "#898781"
                            font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.Bold; font.letterSpacing: 0.1 * Theme.fsXs
                        }
                        MouseArea { id: segMouse; anchors.fill: parent; hoverEnabled: true; cursorShape: Qt.PointingHandCursor; onClicked: Fft.sensor = parent.modelData[0] }
                    }
                }
            }
        }
    }
    Flow {
        Layout.fillWidth: true
        spacing: 8
        Text {
            text: Theme.tr(Fft.sourceText)
            color: "#c3c2b7"
            font.family: Theme.fontData; font.pixelSize: Theme.fsXs
        }
        // The IMU stream of the log the spectrum is computed from
        CfgSelect {
            visible: Fft.streams.length > 0
            enabled: Fft.streams.length > 1
            width: Math.min(bar.width - 24, 220)
            implicitHeight: 20
            fontSize: 9
            tip: "IMU samples of the log the spectrum is computed from"
            options: Fft.streams
            value: Fft.stream
            onPicked: (v) => Fft.selectStream(v)
        }
    }
    Text {
        Layout.fillWidth: true
        text: Theme.tr(Fft.unit)
        color: "#898781"
        font.family: Theme.fontUi; font.pixelSize: Theme.clamp(9, 0.6 * Theme.vw, 12)
    }
    SpectrumView {
        id: live
        mode: "live"
        Layout.fillWidth: true
        Layout.preferredHeight: Theme.clamp(110, 21 * Theme.vh, 190)
    }
    RowLayout {
        Layout.fillWidth: true
        spacing: 12
        Row { spacing: 4; Rectangle { anchors.verticalCenter: parent.verticalCenter; width: 10; height: 2; color: "#3987e5" } Small { text: qsTr("NOW") } }
        Row { spacing: 4; Rectangle { anchors.verticalCenter: parent.verticalCenter; width: 10; height: 2; color: "#c98500" } Small { text: qsTr("PEAK 10 s") } }
        Item { Layout.fillWidth: true }
        Text { text: live.readout; color: "#f0efec"; font.family: Theme.fontData; font.pixelSize: Theme.fsXs }
    }

    // ── Waterfall ──
    Rule { }
    SectionTitle { text: qsTr("WATERFALL · LAST 60 s") }
    Text {
        text: qsTr("frequency down, time to the right, colour = level")
        color: "#898781"
        font.family: Theme.fontUi; font.pixelSize: Theme.clamp(9, 0.6 * Theme.vw, 12)
    }
    SpectrumView {
        id: wf
        mode: "waterfall"
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumHeight: 120
    }
    RowLayout {
        Layout.fillWidth: true
        Layout.bottomMargin: 4
        spacing: 6
        // the colour scale, then its range in dB
        Rectangle {
            width: 70; height: 7; radius: 1
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.00; color: Qt.rgba(0, 0, 4 / 255, 1) }
                GradientStop { position: 0.14; color: Qt.rgba(40 / 255, 11 / 255, 84 / 255, 1) }
                GradientStop { position: 0.29; color: Qt.rgba(101 / 255, 21 / 255, 110 / 255, 1) }
                GradientStop { position: 0.43; color: Qt.rgba(159 / 255, 42 / 255, 99 / 255, 1) }
                GradientStop { position: 0.57; color: Qt.rgba(212 / 255, 72 / 255, 66 / 255, 1) }
                GradientStop { position: 0.71; color: Qt.rgba(245 / 255, 125 / 255, 21 / 255, 1) }
                GradientStop { position: 0.86; color: Qt.rgba(250 / 255, 193 / 255, 39 / 255, 1) }
                GradientStop { position: 1.00; color: Qt.rgba(252 / 255, 1, 164 / 255, 1) }
            }
        }
        Text { text: wf.rangeText; color: "#f0efec"; font.family: Theme.fontData; font.pixelSize: Theme.fsXs }
        Item { Layout.fillWidth: true }
        Text { text: wf.readout; color: "#f0efec"; font.family: Theme.fontData; font.pixelSize: Theme.fsXs }
    }
}
