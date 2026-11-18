// .fp-icon-btn: 24 px square, transparent; muted icon that turns white on a
// faint cyan background under the mouse; 30 % when disabled.
import QtQuick
import QtQuick.Controls.Basic

Rectangle {
    id: b
    property var parts: []
    property real iconSize: 14
    property string glyph: ""          // a text glyph instead of an icon (×)
    property real glyphSize: 16
    property string tip: ""
    property color iconColor: hover.hovered && enabled ? Theme.textMain : Theme.textMuted
    signal clicked()

    width: 24
    height: 24
    radius: 3
    opacity: enabled ? 1 : 0.3
    color: hover.hovered && enabled ? Qt.rgba(0, 210 / 255, 1, 0.1) : "transparent"
    border.color: hover.hovered && enabled ? Theme.borderAccent : "transparent"
    border.width: Theme.hairline

    SvgIcon {
        visible: b.glyph.length === 0
        anchors.centerIn: parent
        width: b.iconSize
        height: b.iconSize
        parts: b.parts
        color: b.iconColor
    }
    Text {
        visible: b.glyph.length > 0
        anchors.centerIn: parent
        text: b.glyph
        color: b.iconColor
        font.pointSize: (b.glyphSize) * 0.75
    }
    HoverHandler { id: hover; cursorShape: b.enabled ? Qt.PointingHandCursor : Qt.ArrowCursor }
    TapHandler { enabled: b.enabled; onTapped: b.clicked() }
    ToolTip.visible: tip.length > 0 && hover.hovered
    ToolTip.text: tip
    ToolTip.delay: 600
}
