// .cal-table: a header row and rows of cells (Roboto Mono 10.5 px), hairline
// rules under every row. columns: [[title, width fraction], ...]; rows: what
// the delegate draws (default: rows of {cells: [...], warn, span}). A custom
// rowDelegate reads its row as parent.row (the parent is null while it is
// being destroyed) and the column geometry from the table by id.
import QtQuick

Column {
    id: t
    property var columns: []
    property var rows: []
    property Component rowDelegate: defaultRow
    width: parent ? parent.width : 320

    function colX(i) { let x = 0; for (let k = 0; k < i; ++k) x += columns[k][1] * width; return x }
    function colW(i) { return columns[i][1] * width }

    Item {
        width: t.width
        height: 24
        visible: t.columns.length > 0
        Repeater {
            model: t.columns
            delegate: Text {
                required property var modelData
                required property int index
                x: t.colX(index) + 6
                width: t.colW(index) - 12
                height: 24
                verticalAlignment: Text.AlignVCenter
                text: Theme.tr(modelData[0])
                elide: Text.ElideRight
                color: Theme.textDim
                font.family: Theme.fontUi; font.pixelSize: 9; font.weight: Font.Bold; font.letterSpacing: 1
            }
        }
        Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
    }
    Repeater {
        model: t.rows
        delegate: Loader {
            required property var modelData
            required property int index
            width: t.width
            sourceComponent: t.rowDelegate
            property var row: modelData
        }
    }

    Component {
        id: defaultRow
        Item {
            readonly property var r: parent ? parent.row : ({ cells: [] })
            width: t.width
            height: 24
            Repeater {
                model: r.cells
                delegate: Text {
                    required property var modelData
                    required property int index
                    readonly property bool last: index === r.cells.length - 1
                    x: t.colX(index) + 6
                    width: (r.span && last ? t.width - t.colX(index) : t.colW(index)) - 12
                    height: 24
                    verticalAlignment: Text.AlignVCenter
                    text: Theme.tr(modelData)
                    elide: Text.ElideRight
                    color: r.warn && last ? Theme.accentOrange : Theme.textMain
                    font.family: Theme.fontData; font.pointSize: 7.875
                }
            }
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
        }
    }
}
