// #log-replay-timeline (log-replay.css): over the bottom-right corner, above
// the command bar, while a log is loaded: play / pause, the scrubber, current
// and total time, × to unload. Logic in ReplayController ("Replay").
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Rectangle {
    id: tl
    width: Math.min(500, Theme.winW - 260)
    height: 40
    color: Theme.glassToolbar
    topLeftRadius: 6
    border.width: 0
    Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderAccent }
    Rectangle { width: Theme.hairline; height: parent.height; color: Theme.borderAccent }

    component LrtButton: Rectangle {
        id: b
        property string label
        property real labelSize: 11
        signal clicked()
        width: 28; height: 24; radius: 3
        color: m.containsMouse ? Theme.cmdButtonHover : Theme.cmdButton
        border.width: Theme.hairline
        border.color: Theme.borderInput
        Text {
            anchors.centerIn: parent
            text: Theme.tr(b.label)
            color: m.containsMouse ? Theme.accentCyan : Theme.textMain
            font.pixelSize: b.labelSize
        }
        MouseArea { id: m; anchors.fill: parent; hoverEnabled: true; cursorShape: Qt.PointingHandCursor; onClicked: b.clicked() }
    }
    component Time: Text {
        width: 42
        horizontalAlignment: Text.AlignHCenter
        color: Theme.textMain
        font.family: Theme.fontData; font.pixelSize: 11; font.letterSpacing: 0.5
    }

    Row {
        x: 10
        height: parent.height
        spacing: 8
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: qsTr("REPLAY")
            color: Theme.accentCyan
            opacity: 0.75
            font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5
        }
        LrtButton {
            objectName: "replayPlay"
            anchors.verticalCenter: parent.verticalCenter
            label: Replay.playing ? "❚❚" : "▶"
            ToolTip.visible: false
            onClicked: Replay.playPause()
        }
        Time { anchors.verticalCenter: parent.verticalCenter; text: Replay.currentText }
        Slider {
            id: scrub
            anchors.verticalCenter: parent.verticalCenter
            width: tl.width - 10 - 8 * 6 - 50 - 28 - 42 - 42 - 24 - 10
            from: 0; to: 1
            value: Replay.fraction
            live: true
            onMoved: Replay.scrub(value)
            background: Rectangle {
                x: scrub.leftPadding
                y: scrub.topPadding + scrub.availableHeight / 2 - height / 2
                width: scrub.availableWidth
                height: 4
                radius: 2
                color: Qt.rgba(1, 1, 1, 0.12)
            }
            handle: Rectangle {
                x: scrub.leftPadding + scrub.visualPosition * (scrub.availableWidth - width)
                y: scrub.topPadding + scrub.availableHeight / 2 - height / 2
                width: 12; height: 12; radius: 6
                color: Theme.accentCyan
                scale: scrub.hovered || scrub.pressed ? 1.15 : 1
            }
        }
        Time { anchors.verticalCenter: parent.verticalCenter; text: Replay.totalText }
    }
    LrtButton {
        anchors.right: parent.right
        anchors.rightMargin: 10
        anchors.verticalCenter: parent.verticalCenter
        width: 24
        label: "×"
        labelSize: 16
        HoverTip { text: "Unload log" }
        onClicked: Replay.unload()
    }
}
