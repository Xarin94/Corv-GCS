// .setup-panel: dark card with the cyan header line (setup pages). 353 px by
// default (320 of content); wider for the .full-width panels.
import QtQuick

Rectangle {
    id: panel
    property string title
    default property alias body: inner.data
    width: 353
    height: inner.implicitHeight + 70
    radius: 6
    color: Theme.glassPanel
    border.color: Theme.borderPanel
    border.width: Theme.hairline

    Text {
        x: 17
        y: 17
        text: Theme.tr(panel.title)
        color: Theme.accentCyan
        font.family: Theme.fontUi
        font.pixelSize: 12
        font.weight: Font.Bold
        font.letterSpacing: 2
    }
    Rectangle { x: 17; y: 43; width: panel.width - 33; height: Theme.hairline; color: Theme.borderTab }
    Column {
        id: inner
        x: 17
        y: 55
        width: panel.width - 33
        spacing: 10
    }
}
