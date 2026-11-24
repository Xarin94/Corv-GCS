// .fp-popover: the panels that open over the segment list (route settings,
// camera & gimbal, radio link): 8 px from the panel edges, solid glass, cyan
// header with a close button, scrolling when taller than the panel.
import QtQuick
import QtQuick.Controls.Basic

Rectangle {
    id: pop
    property string title
    default property alias body: inner.data
    signal closeRequested()

    x: 8
    y: 8
    width: parent ? parent.width - 16 : 300
    height: Math.min(inner.implicitHeight + head.height + 2, (parent ? parent.height : 600) - 16)
    z: 30
    radius: 4
    color: Qt.rgba(4 / 255, 8 / 255, 14 / 255, 0.98)
    border.color: Theme.borderAccentMid
    border.width: Theme.hairline

    // box-shadow: 0 8px 30px rgba(0,0,0,0.6)
    Rectangle {
        z: -1
        anchors.fill: parent
        anchors.margins: -6
        anchors.topMargin: 2
        anchors.bottomMargin: -14
        radius: 10
        color: Qt.rgba(0, 0, 0, 0.35)
    }
    MouseArea { anchors.fill: parent; acceptedButtons: Qt.AllButtons; onWheel: (w) => w.accepted = false }

    Item {
        id: head
        x: 1
        width: parent.width - 2
        height: 31
        Text {
            x: 10
            anchors.verticalCenter: parent.verticalCenter
            anchors.verticalCenterOffset: 1
            text: Theme.tr(pop.title)
            color: Theme.accentCyan
            font.family: Theme.fontUi
            font.pixelSize: 11
            font.weight: Font.Bold
            font.letterSpacing: 2
        }
        FpIconButton {
            anchors.right: parent.right
            anchors.rightMargin: 10
            anchors.verticalCenter: parent.verticalCenter
            glyph: "×"
            tip: "Close"
            onClicked: pop.closeRequested()
        }
        Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderAccent }
    }
    Flickable {
        id: flick
        x: 1
        y: head.height
        width: parent.width - 2
        height: parent.height - head.height - 1
        contentHeight: inner.implicitHeight
        clip: true
        boundsBehavior: Flickable.StopAtBounds
        ScrollBar.vertical: ScrollBar { width: 6 }
        Column {
            id: inner
            width: flick.width
        }
    }
}
