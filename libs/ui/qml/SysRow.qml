// #tab-sys-config .cfg-row: the label in a 132 px column, the control in the
// rest (column gap 10), centred, at least 26 px tall, 8 px under it.
// stack: the label above the control (row gap 4). full: no label column.
// note / sub: a .cfg-note paragraph or a .cfg-sub caption instead of a row.
import QtQuick

Item {
    id: r
    property string label: ""
    property bool stack: false
    property bool full: false
    property string note: ""
    property bool noteTop: false
    property string sub: ""
    default property alias content: slot.data

    readonly property real slotWidth: stack || full ? width : width - 142
    width: parent ? parent.width : 327
    implicitHeight: note.length ? noteText.implicitHeight + (noteTop ? 10 : 8)
                  : sub.length ? 4 + Theme.hairline + 8 + subText.implicitHeight + 8
                  : stack ? lbl.implicitHeight + 4 + slot.childrenRect.height + 8
                  : Math.max(26, slot.childrenRect.height, full ? 0 : lbl.implicitHeight) + 8

    // .cfg-note
    Text {
        id: noteText
        visible: r.note.length > 0
        width: r.width
        text: r.note
        wrapMode: Text.Wrap
        color: Theme.textDim
        font.family: Theme.fontUi
        font.pixelSize: 10
        lineHeight: 15
        lineHeightMode: Text.FixedHeight
    }
    // .cfg-sub
    Item {
        visible: r.sub.length > 0
        width: r.width
        Rectangle { y: 4; width: parent.width; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.06) }
        Text {
            id: subText
            y: 4 + Theme.hairline + 8
            text: r.sub
            color: Theme.textDim
            opacity: 0.8
            font.family: Theme.fontUi
            font.pixelSize: 9
            font.weight: Font.DemiBold
            font.letterSpacing: 1.8
        }
    }

    Text {
        id: lbl
        visible: !r.full && r.note.length === 0 && r.sub.length === 0
        width: r.stack ? r.width : 132
        y: r.stack ? 0 : (r.height - 8 - height) / 2
        text: r.label
        wrapMode: Text.Wrap
        color: Theme.textDim
        font.family: Theme.fontUi
        font.pixelSize: 10
        font.weight: Font.DemiBold
        font.letterSpacing: 1
        lineHeight: 1.25
    }
    Item {
        id: slot
        visible: r.note.length === 0 && r.sub.length === 0
        x: r.stack || r.full ? 0 : 142
        y: r.stack ? lbl.implicitHeight + 4 : (r.height - 8 - height) / 2
        width: r.slotWidth
        height: childrenRect.height
    }
}
