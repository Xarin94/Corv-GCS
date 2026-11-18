// #zone-annun (GeoZonePanel.js): what the UAS geographical zones say, inboard
// of the altitude tape, drawn like the health annunciators: red flashing
// NO-FLY in a zone restricted from the surface, MAX n m over a ceiling or the
// general limit (ALT LIMIT); amber ZONE where conditions apply and MAX n m
// under a ceiling. A click silences one until the next connection.
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Column {
    id: col
    spacing: Theme.clamp(4, 0.9 * Theme.vh, 12)

    Repeater {
        model: GeoZones.alerts
        delegate: Item {
            id: a
            required property var modelData
            readonly property bool warning: modelData.level === "warn"
            readonly property color tone: warning ? "#ff2a2a" : "#ffb300"
            readonly property var parts: Annunciators.glyph(modelData.icon)
            width: Theme.annunW
            height: inner.implicitHeight
            SequentialAnimation on opacity {
                loops: Animation.Infinite
                running: true
                PropertyAction { value: 1.0 }
                PauseAnimation { duration: a.warning ? 385 : 660 }
                PropertyAction { value: 0.22 }
                PauseAnimation { duration: a.warning ? 315 : 540 }
            }
            Column {
                id: inner
                width: parent.width
                spacing: 1
                Item {
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: Theme.annunW * 0.74
                    height: width
                    SvgIcon {
                        anchors.fill: parent
                        color: "black"
                        parts: a.parts.map(p => Object.assign({}, p, { stroke: (p.stroke || 0) + 1.3, fill: p.fill }))
                    }
                    SvgIcon { anchors.fill: parent; color: a.tone; parts: a.parts }
                }
                OutlinedText {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: a.modelData.label
                    color: a.tone
                    font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; font.weight: Font.Bold; font.letterSpacing: 0.08 * Theme.fsSm
                }
                OutlinedText {
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: Theme.annunW + 30
                    visible: text.length > 0
                    horizontalAlignment: Text.AlignHCenter
                    wrapMode: Text.Wrap
                    text: a.modelData.value
                    color: Theme.hudText
                    font.family: Theme.fontData; font.pixelSize: Theme.fsXs
                }
            }
            MouseArea {
                id: am
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: GeoZones.silence(a.modelData.id)
            }
            ToolTip.visible: am.containsMouse
            ToolTip.text: modelData.title
            ToolTip.delay: 500
        }
    }
}
