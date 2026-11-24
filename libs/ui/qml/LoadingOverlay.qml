// The loading screen (LoadingOverlay.js): up at start until the terrain around the
// vehicle and the first satellite pass are there, never longer than a few seconds
// past what the network gives; a click goes on without waiting.
import QtQuick
import Corv.Ui

Rectangle {
    id: ov
    property var load: ({})
    property bool relative: false
    property bool satellite: true
    readonly property bool done: p.done
    color: Theme.bg
    visible: opacity > 0
    opacity: p.done ? 0 : 1
    Behavior on opacity { NumberAnimation { duration: 600; easing.type: Easing.InOutQuad } }

    readonly property int minVisibleMs: 600
    readonly property int satelliteMaxMs: 20000
    readonly property int maxMs: 45000
    readonly property int settleMs: 400        // idle this long: between two stages nothing is pending for a moment
    readonly property int noTerrainMs: 3000    // no elevation data here: nothing to wait for

    QtObject {
        id: p
        property bool done: false
        property bool terrain: false
        property real t0: Date.now()
        property real idleSince: 0
        property string text: qsTr("LOADING...")
        property real bar: 0

        function finish() {
            bar = 1
            text = qsTr("SYSTEMS READY")
            closing.start()
        }
        function idle(ms) {
            if (!idleSince) idleSince = Date.now()
            return Date.now() - idleSince >= ms
        }
        function step() {
            const l = ov.load, age = Date.now() - t0
            if (closing.running || age < ov.minVisibleMs) return
            if (age > ov.maxMs || ov.relative) return finish()
            const hgt = l.hgtLoading || 0, building = l.building || 0, chunks = l.chunks || 0
            if (!terrain) {
                if (chunks === 0 && building === 0 && hgt === 0) {
                    text = qsTr("WAITING FOR TERRAIN DATA...")
                    bar = 0.1
                    if (l.hgtMissing && idle(ov.noTerrainMs)) finish()
                    return
                }
                if (building === 0 && hgt === 0) {
                    if (!idle(ov.settleMs)) return
                    terrain = true
                    idleSince = 0
                } else {
                    idleSince = 0
                    if (chunks === 0) {
                        text = qsTr("READING TERRAIN DATA...") + " " + (l.hgtLoaded || 0) + " HGT"
                        bar = 0.05 + 0.05 * (l.hgtLoaded || 0) / Math.max(1, (l.hgtLoaded || 0) + hgt)
                    } else {
                        text = qsTr("BUILDING TERRAIN...") + " " + chunks + " (" + building + ")"
                        bar = 0.1 + 0.4 * chunks / Math.max(1, chunks + building)
                    }
                    return
                }
            }
            if (!ov.satellite || l.imagesDone) return idle(ov.settleMs) ? finish() : undefined
            idleSince = 0
            if (age > ov.satelliteMaxMs) return finish()
            const total = (l.images || 0) + (l.imagesPending || 0)
            text = total > 0 ? qsTr("LOADING SATELLITE...") + " " + (l.images || 0) + "/" + total : qsTr("PREPARING SATELLITE...")
            bar = 0.5 + 0.45 * (total > 0 ? (l.images || 0) / total : 0)
        }
    }
    Timer { interval: 100; repeat: true; running: !p.done; onTriggered: p.step() }
    Timer { id: closing; interval: 250; onTriggered: p.done = true }
    MouseArea { anchors.fill: parent; enabled: !p.done; hoverEnabled: true; onClicked: p.done = true }

    Column {
        anchors.centerIn: parent
        anchors.verticalCenterOffset: -0.05 * parent.height
        spacing: 8
        Image {
            anchors.horizontalCenter: parent.horizontalCenter
            readonly property real side: Math.min(420, 0.42 * ov.height)
            width: side
            height: side
            source: "qrc:/icons/icon-512x512.png"
            sourceSize: Qt.size(512, 512)
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
        }
        Item { width: 1; height: 16 }
        Row {
            anchors.horizontalCenter: parent.horizontalCenter
            component Title: Text {
                font.family: Theme.fontUi
                font.pixelSize: 48
                font.weight: Font.Bold
                font.letterSpacing: 12
            }
            Title { text: "CORV "; color: Theme.textMain }
            Title { text: "SYSTEMS"; color: Theme.accentCyan }
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: qsTr("GROUND CONTROL STATION")
            color: Theme.textDim
            font.family: Theme.fontUi
            font.pixelSize: 14
            font.letterSpacing: 6
        }
        Item { width: 1; height: 24 }
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            width: 300
            height: 4
            radius: 2
            color: Qt.rgba(1, 1, 1, 0.03)
            border.color: Qt.rgba(0, 210 / 255, 1, 0.25)
            border.width: 1
            Rectangle {
                height: parent.height
                radius: 2
                width: parent.width * Math.max(0, Math.min(1, p.bar))
                color: Theme.accentCyan
                Behavior on width { NumberAnimation { duration: 300; easing.type: Easing.OutQuad } }
            }
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            topPadding: 6
            text: p.text
            color: Theme.textDim
            font.family: Theme.fontData
            font.pixelSize: 11
            font.letterSpacing: 1
        }
    }
}
