// Global header (60 px: brand, FPS counter) and the tab bar under it (32 px:
// tabs on the left, the square view buttons on the right).
import QtQuick

Item {
    id: header
    property int currentTab: 0
    readonly property var tabs: ["FLIGHT DATA", "FLIGHT PLAN", "SETUP", "SYS CONFIG"]
    property real fps: 0
    // View toggles (bound by Main to the 3D scene and the HUD)
    property bool satellite: true
    property bool sunlight: true
    property bool firstPerson: true
    property bool horizonLock: false
    property bool trajectory: false
    property bool fpvOn: false
    signal toggleFpv()
    signal toggleSatellite()
    signal toggleSunlight()
    signal toggleCamera()
    signal toggleHorizonLock()
    signal toggleTrajectory()
    signal toggleTheme()
    signal openUpdate()

    height: 93

    // ── Global header ──
    Item {
        id: global
        width: parent.width
        height: 60
        Row {
            x: 24
            y: 15
            height: 30
            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: "CORV "
                color: Theme.textMain
                font.family: Theme.fontUi
                font.pixelSize: 24
                font.weight: Font.Bold
                font.letterSpacing: 4
            }
            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: "SYSTEMS"
                color: Theme.accentCyan
                font.family: Theme.fontUi
                font.pixelSize: 24
                font.weight: Font.Bold
                font.letterSpacing: 4
            }
        }
        // A newer version on GitHub (Updates): opens the update dialog
        Rectangle {
            id: updateBtn
            visible: Updates.notice
            anchors.right: fpsText.left
            anchors.rightMargin: 16
            y: 18
            width: updText.implicitWidth + 20
            height: 24
            radius: 4
            color: updMouse.containsMouse ? Qt.rgba(0, 210 / 255, 1, 0.18) : Qt.rgba(0, 210 / 255, 1, 0.08)
            border.color: Theme.accentCyan
            border.width: 1
            Text {
                id: updText
                anchors.centerIn: parent
                text: (Updates.state === "downloading" ? "DOWNLOADING " + (Updates.progress * 100).toFixed(0) + " %"
                       : "UPDATE " + Updates.latest)
                color: Theme.accentCyan
                font.family: Theme.fontUi
                font.pixelSize: 12
                font.weight: Font.DemiBold
                font.letterSpacing: 1
            }
            MouseArea {
                id: updMouse
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: header.openUpdate()
            }
        }
        Text {
            id: fpsText
            anchors.right: parent.right
            anchors.rightMargin: 24
            y: 15
            height: 30
            verticalAlignment: Text.AlignVCenter
            text: (header.fps > 0 ? header.fps.toFixed(0) : "--") + " FPS"
            color: Theme.textDim
            opacity: 0.7
            font.family: Theme.fontData
            font.pixelSize: 11
        }
    }

    // ── Tab bar ──
    Rectangle {
        y: 60
        width: parent.width
        height: 33
        color: Theme.glassToolbar
        Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderTab }

        Row {
            x: 12
            height: 32
            Repeater {
                model: header.tabs
                delegate: Rectangle {
                    required property string modelData
                    required property int index
                    readonly property bool active: header.currentTab === index
                    width: label.implicitWidth + 28
                    height: 32
                    color: active ? Qt.rgba(0, 210 / 255, 1, 0.06) : "transparent"
                    Text {
                        id: label
                        anchors.centerIn: parent
                        text: parent.modelData
                        color: parent.active ? Theme.accentCyan : (tabMouse.containsMouse ? Theme.textMutedBright : Theme.textDim)
                        font.family: Theme.fontUi
                        font.pixelSize: Theme.fsMd
                        font.weight: Font.DemiBold
                        font.letterSpacing: 1.5
                    }
                    Rectangle {
                        anchors.bottom: parent.bottom
                        width: parent.width
                        height: 2
                        color: parent.active ? Theme.accentCyan : "transparent"
                    }
                    MouseArea {
                        id: tabMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: header.currentTab = parent.index
                    }
                }
            }
        }

        Row {
            anchors.right: parent.right
            anchors.rightMargin: 8
            y: 1
            spacing: 2
            SquareButton { icon: Icons.fpv; active: header.fpvOn; tooltip: "FPV Camera (enable/disable stream)"; onClicked: header.toggleFpv() }
            SquareButton { icon: Icons.satellite; active: header.satellite; tooltip: "Satellite imagery (M) — off: schematic isoline view"; onClicked: header.toggleSatellite() }
            SquareButton { icon: Icons.sun; active: header.sunlight; tooltip: "Toggle realistic sunlight (L)"; onClicked: header.toggleSunlight() }
            SquareButton { label: header.firstPerson ? "1P" : "3P"; tooltip: "Toggle 1st/3rd person camera"; onClicked: header.toggleCamera() }
            SquareButton { icon: Icons.tilt; active: header.horizonLock; tooltip: "Horizon-lock view — pitch always level (T)"; onClicked: header.toggleHorizonLock() }
            SquareButton { icon: Icons.trajectory; active: header.trajectory; tooltip: "Predicted trajectory corridor (P)"; onClicked: header.toggleTrajectory() }
            SquareButton { icon: Icons.theme; tooltip: "Toggle light/dark theme"; onClicked: header.toggleTheme() }
        }
    }
}
