// .cfg-row of the setup panels: the label on the left, the controls on the
// right (an empty label leaves the controls alone on the row).
import QtQuick

Item {
    id: row
    property string label
    property real controlsWidth: slot.implicitWidth
    default property alias content: slot.data
    width: parent ? parent.width : 320
    height: Math.max(26, slot.implicitHeight)
    CfgLabel {
        anchors.verticalCenter: parent.verticalCenter
        width: row.width - slot.width - 8
        elide: Text.ElideRight
        text: row.label
        visible: text.length > 0
    }
    Row {
        id: slot
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        spacing: 6
    }
}
