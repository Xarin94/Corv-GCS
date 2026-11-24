// .lidar-strip of css/lidar.css: the LIDAR and ROS strips at the bottom centre
// of the 3D view while the feature is on - a state LED (blinking while it
// searches), the title, the state, a count and the actions that matter in
// flight. Same over-scene language as the HUD: thin, monochrome.
import QtQuick
import QtQuick.Controls.Basic

Rectangle {
    id: strip
    property string title
    property string state: "search"    // search | hold | accum | full | error
    property string label
    property string count
    property var buttons: []          // [{ text, tip, on: bool, rough: bool, off: bool }]
    signal clicked(int index)

    readonly property color green: Qt.rgba(0, 1, 127 / 255, 0.95)
    readonly property color stateColor: state === "hold" || state === "search" ? Theme.accentOrange : state === "error" ? Theme.statusRed : green

    implicitWidth: row.implicitWidth + 24
    implicitHeight: row.implicitHeight + 10
    radius: 3
    color: Qt.rgba(0, 12 / 255, 6 / 255, 0.72)
    border.color: Qt.rgba(0, 1, 127 / 255, 0.35)
    border.width: Theme.hairline

    Row {
        id: row
        anchors.centerIn: parent
        spacing: 10
        Rectangle {
            id: led
            anchors.verticalCenter: parent.verticalCenter
            width: 7; height: 7; radius: 3.5
            color: strip.state === "accum" ? Theme.accentGreen : strip.state === "full" ? Theme.accentCyan
                 : strip.state === "error" ? Theme.statusRed : strip.state === "hold" || strip.state === "search" ? Theme.accentOrange : "#555566"
            border.color: Qt.rgba(0, 0, 0, 0.6)
            border.width: 1
            SequentialAnimation on opacity {
                running: strip.state === "search" && strip.visible
                loops: Animation.Infinite
                onRunningChanged: if (!running) led.opacity = 1
                PropertyAction { value: 1 }
                PauseAnimation { duration: 500 }
                PropertyAction { value: 0.25 }
                PauseAnimation { duration: 500 }
            }
        }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: Theme.tr(strip.title)
            color: Theme.hudText
            font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.DemiBold; font.letterSpacing: 3
        }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            width: Math.max(90, implicitWidth)
            text: Theme.tr(strip.label)
            color: strip.stateColor
            font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.DemiBold; font.letterSpacing: 1.5
        }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            width: Math.max(80, implicitWidth)
            horizontalAlignment: Text.AlignRight
            text: Theme.tr(strip.count)
            color: Theme.textMutedBright
            font.family: Theme.fontData; font.pixelSize: 11; font.weight: Font.Medium; font.letterSpacing: 0.5
        }
        Repeater {
            model: strip.buttons
            delegate: Rectangle {
                id: btn
                required property var modelData
                required property int index
                anchors.verticalCenter: parent.verticalCenter
                width: Math.max(btnText.implicitWidth + 18, modelData.minWidth || 0)
                height: btnText.implicitHeight + 6
                radius: 2
                opacity: modelData.off ? 0.55 : 1
                border.width: 1
                border.color: modelData.on ? Qt.rgba(120 / 255, 140 / 255, 1, 0.8) : modelData.rough ? Qt.rgba(1, 120 / 255, 60 / 255, 0.8) : Qt.rgba(0, 1, 127 / 255, 0.45)
                gradient: Gradient {
                    orientation: Gradient.Horizontal
                    GradientStop { position: 0; color: btn.modelData.on ? Qt.rgba(1, 56 / 255, 41 / 255, 0.35) : btn.modelData.rough ? Qt.rgba(166 / 255, 166 / 255, 166 / 255, 0.3) : btnHover.hovered ? Qt.rgba(0, 1, 127 / 255, 0.18) : "transparent" }
                    GradientStop { position: 0.5; color: btn.modelData.rough ? Qt.rgba(1, 174 / 255, 46 / 255, 0.35) : btn.modelData.on ? Qt.rgba(0.6, 0.3, 0.6, 0.35) : btnHover.hovered ? Qt.rgba(0, 1, 127 / 255, 0.18) : "transparent" }
                    GradientStop { position: 1; color: btn.modelData.on ? Qt.rgba(46 / 255, 115 / 255, 1, 0.35) : btn.modelData.rough ? Qt.rgba(1, 41 / 255, 31 / 255, 0.45) : btnHover.hovered ? Qt.rgba(0, 1, 127 / 255, 0.18) : "transparent" }
                }
                Text {
                    id: btnText
                    anchors.centerIn: parent
                    text: Theme.tr(btn.modelData.text)
                    color: btn.modelData.on || btn.modelData.rough || btnHover.hovered ? "white" : strip.green
                    font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5
                }
                HoverHandler { id: btnHover; cursorShape: Qt.PointingHandCursor }
                TapHandler { gesturePolicy: TapHandler.ReleaseWithinBounds; onTapped: strip.clicked(btn.index) }
                ToolTip.visible: !!btn.modelData.tip && btnHover.hovered
                ToolTip.text: Theme.tr(btn.modelData.tip || "")
                ToolTip.delay: 600
            }
        }
    }
}
