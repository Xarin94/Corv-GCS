// The title="" of the Electron page: what the item it is in does, while the mouse
// rests on it. A child of the control, which it covers without taking its clicks.
import QtQuick
import QtQuick.Controls.Basic

Item {
    id: tip
    property string text
    anchors.fill: parent
    HoverHandler { id: hover }
    ToolTip {
        visible: hover.hovered && tip.text.length > 0
        delay: 600
        y: -height - 4
        width: Math.min(implicitWidth, 340)
        contentItem: Text {
            text: Theme.tr(tip.text)
            wrapMode: Text.Wrap
            color: Theme.textMain
            font.family: Theme.fontUi
            font.pixelSize: 11
        }
    }
}
