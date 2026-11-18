// .fp-field: one schema-driven field of the route editor (inspector, route
// settings, camera, radio): the label on the left in 10 px capitals, the
// control on the right — a select, a switch or a number with its unit.
// field: { key, label, unit, type, value, checked, options:[{value,label,disabled,title}],
//          placeholder, title, disabled } as PlanController::fieldsFor builds it.
import QtQuick
import QtQuick.Controls.Basic

Item {
    id: f
    property var field: ({})
    property bool compact: false
    property real selectWidth: 130
    signal edited(string key, var value)

    readonly property bool off: !!field.disabled
    width: parent ? parent.width : 280
    implicitHeight: compact ? 20 : 24
    opacity: off ? 0.42 : 1

    Text {
        id: lbl
        anchors.verticalCenter: parent.verticalCenter
        width: Math.min(implicitWidth, f.width - ctl.width - 8)
        elide: Text.ElideRight
        text: String(f.field.label || "").toUpperCase()
        color: Theme.textDim
        font.family: Theme.fontUi
        font.pixelSize: 10
        font.weight: Font.DemiBold
        font.letterSpacing: 0.4
    }

    Item {
        id: ctl
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        width: f.field.type === "select" ? f.selectWidth : f.field.type === "check" ? 30 : num.width + 4 + unit.width
        height: f.field.type === "check" ? 16 : 20

        CfgSelect {
            visible: f.field.type === "select"
            enabled: !f.off
            anchors.fill: parent
            fontSize: 10.5
            options: f.field.options || []
            value: (f.field.options && f.field.options[f.field.current]) ? f.field.options[f.field.current].value : ""
            onPicked: (v) => f.edited(f.field.key, v)
        }

        // .fp-switch
        Rectangle {
            visible: f.field.type === "check"
            width: 30
            height: 16
            radius: 8
            color: f.field.checked ? Qt.rgba(0, 210 / 255, 1, 0.25) : Qt.rgba(1, 1, 1, 0.1)
            border.color: f.field.checked ? Theme.borderAccentMid : Theme.borderInput
            border.width: Theme.hairline
            Rectangle {
                width: 10; height: 10; radius: 5
                x: f.field.checked ? 16 : 2.5
                y: 2.5
                color: f.field.checked ? Theme.accentCyan : Theme.textMuted
                Behavior on x { NumberAnimation { duration: 150 } }
            }
            MouseArea {
                anchors.fill: parent
                enabled: !f.off
                cursorShape: Qt.PointingHandCursor
                onClicked: f.edited(f.field.key, !f.field.checked)
            }
        }

        CfgInput {
            id: num
            visible: f.field.type === "number"
            enabled: !f.off
            width: f.compact ? 58 : 66
            height: 20
            fontSize: 11
            numeric: true
            leftPadding: f.compact ? 5 : 6
            rightPadding: f.compact ? 5 : 6
            horizontalAlignment: TextInput.AlignRight
            text: f.field.value !== undefined ? f.field.value : ""
            placeholderText: f.field.placeholder || ""
            onCommitted: (t) => f.edited(f.field.key, t)
        }
        Text {
            id: unit
            visible: f.field.type === "number"
            anchors.left: num.right
            anchors.leftMargin: 4
            anchors.verticalCenter: parent.verticalCenter
            width: f.field.unit ? Math.max(22, implicitWidth) : 0
            text: f.field.unit || ""
            color: Theme.textDim
            font.family: Theme.fontData
            font.pixelSize: 9
        }
    }

    HoverHandler { id: hov }
    ToolTip.visible: hov.hovered && !!f.field.title
    ToolTip.text: f.field.title || ""
    ToolTip.delay: 700
}
