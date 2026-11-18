// Custom title bar (title-bar.css): a 40 px zone over the top of the window;
// the 32 px bar with its three buttons fades in only while the mouse is in it.
import QtQuick

Item {
    id: zone
    required property var window
    height: 40

    HoverHandler { id: hover }

    Rectangle {
        id: bar
        anchors.left: parent.left
        anchors.right: parent.right
        height: 32
        opacity: hover.hovered ? 1 : 0
        Behavior on opacity { NumberAnimation { duration: 200 } }
        gradient: Gradient {
            GradientStop { position: 0; color: Qt.rgba(0, 0, 0, 0.95) }
            GradientStop { position: 1; color: Qt.rgba(0, 0, 0, 0) }
        }

        MouseArea {
            anchors.fill: parent
            anchors.rightMargin: 138
            enabled: hover.hovered
            onPressed: zone.window.startSystemMove()
            onDoubleClicked: zone.window.visibility === Window.Maximized ? zone.window.showNormal() : zone.window.showMaximized()
        }

        Row {
            anchors.right: parent.right
            height: parent.height
            Repeater {
                model: [{ k: "min", i: Icons.minimize }, { k: "max", i: Icons.maximize }, { k: "close", i: Icons.close }]
                delegate: Rectangle {
                    required property var modelData
                    width: 46
                    height: 32
                    color: ma.containsMouse ? (modelData.k === "close" ? "#e81123" : Qt.rgba(1, 1, 1, 0.1)) : "transparent"
                    SvgIcon {
                        anchors.centerIn: parent
                        width: 12
                        height: 12
                        viewBox: 12
                        parts: parent.modelData.i
                        color: "white"
                    }
                    MouseArea {
                        id: ma
                        anchors.fill: parent
                        hoverEnabled: true
                        enabled: hover.hovered
                        onClicked: {
                            if (parent.modelData.k === "min") zone.window.showMinimized()
                            else if (parent.modelData.k === "max") zone.window.visibility === Window.Maximized ? zone.window.showNormal() : zone.window.showMaximized()
                            else zone.window.close()
                        }
                    }
                }
            }
        }
    }
}
