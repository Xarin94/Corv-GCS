// A ComboBox whose list is drawn by the interface, not by the style: the Basic
// style takes the popup and item colours from the system palette, which on a
// dark desktop theme gave black text on a black list. Same look as CfgSelect
// (dark list, cyan hairline, cyan highlight); contentItem, indicator and
// background can still be replaced by the user of the component.
import QtQuick
import QtQuick.Controls.Basic

ComboBox {
    id: cb
    property color textColor: Theme.textMain
    property real itemHeight: 24
    property real itemPixelSize: font.pixelSize > 0 ? font.pixelSize : 12
    property string itemFamily: font.family
    property real popupMinWidth: 120
    property real popupMaxHeight: 360

    font.family: Theme.fontUi
    font.pixelSize: Theme.fsMd

    contentItem: Text {
        leftPadding: 10
        rightPadding: 20
        text: Theme.tr(cb.displayText)
        font: cb.font
        color: cb.enabled ? cb.textColor : Theme.textDim
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }
    indicator: SvgIcon {
        x: cb.width - width - 8
        anchors.verticalCenter: parent.verticalCenter
        width: 13; height: 13
        parts: [{ d: "M5 9l7 7 7-7", stroke: 2.4, cap: "round" }]
        color: cb.enabled ? cb.textColor : Theme.textDim
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
        width: ListView.view ? ListView.view.width : cb.width
        height: cb.itemHeight
        highlighted: cb.highlightedIndex === index
        // The Basic style pads an ItemDelegate by 12 px: in a 22 px row that left the
        // text no height, and an eliding Text with no room draws nothing (the "black list")
        padding: 0
        topPadding: 0; bottomPadding: 0; leftPadding: 0; rightPadding: 0
        contentItem: Text {
            leftPadding: 8
            text: Theme.tr(cb.textRole && opt.modelData !== null && typeof opt.modelData === "object" ? String(opt.modelData[cb.textRole]) : String(opt.modelData))
            font.family: cb.itemFamily
            font.pixelSize: cb.itemPixelSize
            font.weight: cb.currentIndex === opt.index ? Font.DemiBold : Font.Normal
            color: cb.currentIndex === opt.index ? Theme.accentCyan : Theme.textMain
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
        }
        background: Rectangle {
            color: opt.highlighted ? Qt.rgba(0, 210 / 255, 1, 0.22) : (opt.hovered ? Qt.rgba(0, 210 / 255, 1, 0.12) : "transparent")
        }
    }
    popup: Popup {
        y: cb.height
        width: Math.max(cb.width, cb.popupMinWidth)
        implicitHeight: Math.min(contentItem.implicitHeight + 2, cb.popupMaxHeight)
        padding: 1
        contentItem: ListView {
            clip: true
            implicitHeight: contentHeight
            model: cb.popup.visible ? cb.delegateModel : null
            currentIndex: cb.highlightedIndex
            boundsBehavior: Flickable.StopAtBounds
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
