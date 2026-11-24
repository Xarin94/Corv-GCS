// #ekf-panel / #fft-panel: the black bar down the left side of the flight view
// (the flight view moves over to make room). Content stacked in a column.
import QtQuick
import QtQuick.Layouts

Rectangle {
    id: bar
    default property alias content: col.data
    property string title
    property string window
    width: Theme.ekfBarW
    color: "#05080c"
    Rectangle { anchors.right: parent.right; width: Theme.hairline; height: parent.height; color: Theme.borderAccent }

    ColumnLayout {
        id: col
        x: 12
        y: 8
        width: parent.width - 24
        height: parent.height - 16
        spacing: 6
        RowLayout {
            Layout.fillWidth: true
            Text {
                text: Theme.tr(bar.title)
                color: Theme.accentCyan
                font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; font.weight: Font.Bold; font.letterSpacing: 0.16 * Theme.fsSm
            }
            Item { Layout.fillWidth: true }
            Text {
                text: Theme.tr(bar.window)
                color: "#898781"
                font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.DemiBold; font.letterSpacing: 0.12 * Theme.fsXs
            }
        }
    }
}
