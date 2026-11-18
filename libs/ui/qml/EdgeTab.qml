// The tabs on the window edges (EKF STATUS and FFT on the left, GCS OPTIONS
// on the right): 24 px wide, vertical label, arrow; rounded on the inner side.
import QtQuick

Rectangle {
    id: tab
    property string label
    property bool rightEdge: false
    property bool open: false
    signal clicked()

    width: 24
    height: col.implicitHeight + 18
    color: mouse.containsMouse ? Theme.toggleBgHover : Theme.toggleBg
    border.color: Theme.borderAccent
    border.width: Theme.hairline
    topLeftRadius: rightEdge ? 4 : 0
    bottomLeftRadius: rightEdge ? 4 : 0
    topRightRadius: rightEdge ? 0 : 4
    bottomRightRadius: rightEdge ? 0 : 4

    Column {
        id: col
        y: 10
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: 6
        // Vertical label: a left-edge tab reads bottom to top, a right-edge one top to bottom
        Item {
            width: 12
            height: txt.implicitWidth
            Text {
                id: txt
                anchors.centerIn: parent
                rotation: tab.rightEdge ? 90 : -90
                text: tab.label
                color: Theme.accentCyan
                font.family: Theme.fontUi
                font.pixelSize: Theme.fsXs
                font.weight: Font.Bold
                font.letterSpacing: 0.16 * Theme.fsXs
            }
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: tab.rightEdge ? (tab.open ? "▶" : "◀") : (tab.open ? "◀" : "▶")
            color: Theme.accentCyan
            font.family: Theme.fontUi
            font.pixelSize: 12
        }
    }
    MouseArea {
        id: mouse
        anchors.fill: parent
        hoverEnabled: true
        onClicked: tab.clicked()
    }
}
