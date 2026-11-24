// .setup-panel inside SYS CONFIG: 360 px, padding 14 16 8, the cyan header
// with its hairline (padding-bottom 10, margin-bottom 12), then the rows.
import QtQuick

Rectangle {
    id: panel
    property string title
    default property alias body: inner.data
    width: 360
    height: inner.implicitHeight + 14 + header.height + 12 + 8 + 2 * Theme.hairline
    radius: 6
    color: Theme.glassPanel
    border.color: Theme.borderPanel
    border.width: Theme.hairline

    Item {
        id: header
        x: 16.67
        y: 14.67
        width: parent.width - 33.33
        height: 26
        Text {
            text: Theme.tr(panel.title)
            color: Theme.accentCyan
            font.family: Theme.fontUi
            font.pixelSize: 12
            font.weight: Font.Bold
            font.letterSpacing: 2
        }
        Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderTab }
    }
    Column {
        id: inner
        x: header.x
        y: header.y + header.height + 12
        width: header.width
        spacing: 0
    }
}
