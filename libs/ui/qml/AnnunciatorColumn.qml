// #annunciator-col (AnnunciatorPanel.js): the CAS strip inboard of the airspeed
// tape. Warnings (red, fast flash) hang from the top, cautions (amber, slower)
// stack up from the bottom; no boxes, stroked glyphs outlined over the scene.
// A click silences one until the next connection. Logic in
// AnnunciatorController ("Annunciators").
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Item {
    id: col
    readonly property real gap: Theme.clamp(4, 0.9 * Theme.vh, 12)

    component Annun: Item {
        id: a
        required property var modelData
        property bool warning: true
        readonly property color tone: warning ? "#ff2a2a" : "#ffb300"
        readonly property var parts: Annunciators.glyph(modelData.icon)
        width: Theme.annunW
        height: inner.implicitHeight
        // Warning 0.7 s, caution 1.2 s: on for 55 %, then at 0.22 (steps, no fade)
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
                // A hard black edge under the glyph, as the drop-shadow of the CSS
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
                visible: text.length > 0
                horizontalAlignment: Text.AlignHCenter
                text: a.modelData.value
                color: Theme.hudText
                lineHeight: 1.15
                font.family: Theme.fontData; font.pixelSize: Theme.fsSm
            }
        }
        MouseArea {
            id: am
            anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            onClicked: Annunciators.silence(a.modelData.id)
        }
        ToolTip.visible: am.containsMouse
        ToolTip.text: modelData.title
        ToolTip.delay: 500
    }

    Column {
        width: parent.width
        spacing: col.gap
        Repeater {
            model: Annunciators.warnings
            delegate: Annun { warning: true }
        }
    }
    Column {
        anchors.bottom: parent.bottom
        width: parent.width
        spacing: col.gap
        Repeater {
            model: Annunciators.cautions
            delegate: Annun { warning: false }
        }
    }
}
