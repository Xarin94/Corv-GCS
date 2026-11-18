// .cfg-select: the dark select of the configuration pages (11 px Rajdhani,
// padding 4 6, 25 px tall, hairline border, chevron on the right).
// model: [{ value, label, disabled?, title? }] or plain strings.
import QtQuick
import QtQuick.Controls.Basic

ComboBox {
    id: cb
    property real fontSize: 11
    property var options: []
    property string value: ""
    property real popupMinWidth: 120
    signal picked(string value)

    implicitHeight: 25
    implicitWidth: 185
    font.family: Theme.fontUi
    font.pointSize: (fontSize) * 0.75
    model: options.map(o => typeof o === "string" ? o : o.label)
    currentIndex: Math.max(0, options.findIndex(o => (typeof o === "string" ? o : String(o.value)) === String(value)))
    onActivated: (i) => {
        const o = options[i]
        if (o === undefined || (typeof o !== "string" && o.disabled)) { currentIndex = Math.max(0, options.findIndex(x => (typeof x === "string" ? x : String(x.value)) === String(value))); return }
        picked(typeof o === "string" ? o : String(o.value))
    }

    contentItem: Text {
        leftPadding: 6
        rightPadding: 18
        text: cb.displayText
        font: cb.font
        color: cb.enabled ? Theme.textMain : Theme.textDim
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }
    indicator: SvgIcon {
        x: cb.width - width - 6
        anchors.verticalCenter: parent.verticalCenter
        width: 12; height: 12
        parts: [{ d: "M5 9l7 7 7-7", stroke: 2.6, cap: "round" }]
        color: cb.enabled ? Theme.textMain : Theme.textDim
    }
    background: Rectangle {
        radius: 3
        color: Theme.inputBg
        border.color: cb.activeFocus ? Theme.borderAccentMid : Theme.borderInput
        border.width: Theme.hairline
    }
    delegate: ItemDelegate {
        id: opt
        required property int index
        required property var modelData
        readonly property var o: cb.options[index]
        readonly property bool off: o !== undefined && typeof o !== "string" && !!o.disabled
        width: ListView.view ? ListView.view.width : cb.width
        height: 22
        // The Basic style pads an ItemDelegate by 12 px: in a 22 px row that left the
        // text no height, and an eliding Text with no room draws nothing (the "black list")
        padding: 0
        topPadding: 0; bottomPadding: 0; leftPadding: 0; rightPadding: 0
        enabled: !off
        contentItem: Text {
            leftPadding: 6
            text: opt.modelData
            font.family: Theme.fontUi
            font.pointSize: (cb.fontSize + 1) * 0.75
            color: opt.off ? Theme.textDim : Theme.textMain
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
        }
        background: Rectangle {
            color: opt.highlighted ? Qt.rgba(0, 210 / 255, 1, 0.22) : (opt.hovered ? Qt.rgba(0, 210 / 255, 1, 0.12) : "transparent")
        }
        highlighted: cb.highlightedIndex === index
        ToolTip.visible: off && hovered && o.title !== undefined && o.title.length > 0
        ToolTip.text: off && o.title !== undefined ? o.title : ""
        ToolTip.delay: 400
    }
    popup: Popup {
        y: cb.height
        width: Math.max(cb.width, cb.popupMinWidth)
        implicitHeight: Math.min(contentItem.implicitHeight + 2, 360)
        padding: 1
        contentItem: ListView {
            clip: true
            implicitHeight: contentHeight
            model: cb.popup.visible ? cb.delegateModel : null
            currentIndex: cb.highlightedIndex
            ScrollIndicator.vertical: ScrollIndicator { }
        }
        background: Rectangle {
            color: Qt.rgba(4 / 255, 8 / 255, 14 / 255, 0.98)
            border.color: Theme.borderAccentMid
            border.width: Theme.hairline
            radius: 3
        }
    }
}
