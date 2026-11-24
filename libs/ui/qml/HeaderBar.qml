// Global header (60 px: brand, FPS counter) and the tab bar under it (32 px:
// tabs on the left, the square view buttons on the right).
import QtQuick
import QtQuick.Controls.Basic

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
    signal toggleSatellite()
    signal toggleSunlight()
    signal toggleCamera()
    signal toggleHorizonLock()
    signal toggleTrajectory()
    signal toggleTheme()
    signal openUpdate()

    height: 93

    function openTab(i) { if (i !== 1 || Plan.missionsSupported) currentTab = i }
    Connections {
        target: Plan
        function onPlatformChanged() { if (!Plan.missionsSupported && header.currentTab === 1) header.currentTab = 0 }
    }
    Component.onCompleted: if (!Plan.missionsSupported && currentTab === 1) currentTab = 0
    // Ctrl+1 … Ctrl+4: the tabs
    Shortcut { sequence: "Ctrl+1"; onActivated: header.openTab(0) }
    Shortcut { sequence: "Ctrl+2"; onActivated: header.openTab(1) }
    Shortcut { sequence: "Ctrl+3"; onActivated: header.openTab(2) }
    Shortcut { sequence: "Ctrl+4"; onActivated: header.openTab(3) }

    // ── Global header ──
    Item {
        id: global
        width: parent.width
        height: 60
        Row {
            id: brand
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
        Item {
            anchors.fill: brand
            HoverHandler { cursorShape: Qt.PointingHandCursor }
            TapHandler { onTapped: Qt.openUrlExternally("https://www.corvsystems.com") }
            HoverTip { text: "www.corvsystems.com" }
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
                text: (Updates.state === "downloading" ? qsTr("DOWNLOADING ") + (Updates.progress * 100).toFixed(0) + " %"
                       : qsTr("UPDATE ") + Updates.latest)
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
                    // FLIGHT PLAN on a stack that cannot fly missions: greyed, the reason in the tooltip
                    readonly property bool off: index === 1 && !Plan.missionsSupported
                    opacity: off ? 0.4 : 1
                    ToolTip.visible: off && tabMouse.containsMouse
                    ToolTip.text: Theme.tr(Plan.noMissionReason)
                    ToolTip.delay: 400
                    width: label.implicitWidth + 28
                    height: 32
                    color: active ? Qt.rgba(0, 210 / 255, 1, 0.06) : "transparent"
                    Text {
                        id: label
                        anchors.centerIn: parent
                        text: Theme.tr(parent.modelData)
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
                        onClicked: header.openTab(parent.index)
                    }
                }
            }
        }

        Row {
            anchors.right: parent.right
            anchors.rightMargin: 8
            y: 1
            spacing: 2
            SquareButton {
                objectName: "btnFpv"
                icon: Icons.fpv
                active: Fpv.active || Fpv.connecting
                // Amber in AR, to tell it from the camera alone (#btn-fpv.ar-active)
                accent: Fpv.mode === 2 ? "#ffaa00" : Theme.accentCyan
                opacity: Fpv.connecting ? 0.6 : 1
                tooltip: Fpv.tip
                onClicked: Fpv.click()
            }
            SquareButton { icon: Icons.satellite; active: header.satellite; tooltip: "Satellite imagery (M) — off: schematic isoline view"; onClicked: header.toggleSatellite() }
            SquareButton { icon: Icons.sun; active: header.sunlight; tooltip: "Toggle realistic sunlight (L)"; onClicked: header.toggleSunlight() }
            SquareButton { label: header.firstPerson ? "1P" : "3P"; tooltip: "Toggle 1st/3rd person camera"; onClicked: header.toggleCamera() }
            SquareButton { icon: Icons.tilt; active: header.horizonLock; tooltip: "Horizon-lock view — pitch always level (T)"; onClicked: header.toggleHorizonLock() }
            SquareButton { icon: Icons.trajectory; active: header.trajectory; tooltip: "Predicted trajectory corridor (P)"; onClicked: header.toggleTrajectory() }
            SquareButton { icon: Icons.theme; tooltip: "Toggle light/dark theme"; onClicked: header.toggleTheme() }
        }
    }
}
