// The bitmask editor of SETUP → PARAMETERS (Mission Planner's bitmask checklist):
// one check per documented bit, plus the bits set in the value that the
// documentation does not name, the value in decimal and hex while it changes,
// APPLY puts it in the table as a typed (amber) value for WRITE CHANGED.
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Popup {
    id: pop
    property string name: ""
    property var info: ({})
    property real value: 0
    readonly property var entries: {
        const bits = (info.bits || []).map(b => ({ bit: b.bit, label: b.label, known: true }))
        // Bits set but not documented, so they can be cleared
        for (let b = 0; b < 32; ++b)
            if (pop.isSet(pop.value, b) && !bits.some(x => x.bit === b)) bits.push({ bit: b, label: "(undocumented)", known: false })
        return bits.sort((a, b) => a.bit - b.bit)
    }

    function isSet(v, b) { return Math.floor(v / Math.pow(2, b)) % 2 === 1 }
    function setBit(b, on) {
        const was = isSet(value, b)
        if (on && !was) value += Math.pow(2, b)
        else if (!on && was) value -= Math.pow(2, b)
    }
    function openFor(paramName) {
        name = paramName
        info = Params.meta(paramName)
        value = Number(info.currentNumber) || 0
        open()
    }

    modal: true
    focus: true
    anchors.centerIn: Overlay.overlay
    width: Math.min(parent ? parent.width - 40 : 720, entries.length > 12 ? 720 : 420)
    height: Math.min(parent ? parent.height - 40 : 640, head.implicitHeight + grid.implicitHeight + foot.implicitHeight + 64)
    padding: 18
    closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
    Overlay.modal: Rectangle { color: Qt.rgba(0, 0, 0, 0.55) }
    background: Rectangle {
        radius: 6
        color: Qt.rgba(8 / 255, 12 / 255, 18 / 255, 0.98)
        border.color: Theme.borderAccentMid
        border.width: Theme.hairline
    }

    contentItem: Item {
        Column {
            id: head
            width: parent.width
            spacing: 4
            Row {
                spacing: 10
                Text { id: nameText; text: pop.name; color: Theme.accentCyan; font.family: Theme.fontData; font.pixelSize: 13; font.weight: Font.Bold }
                Text { text: qsTr("BITMASK"); anchors.baseline: nameText.baseline; color: Theme.groupAmber; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5 }
            }
            Text {
                width: parent.width
                text: pop.info.title || ""
                visible: text.length > 0
                color: Theme.textMain; font.family: Theme.fontUi; font.pixelSize: 13; font.weight: Font.DemiBold
                wrapMode: Text.Wrap
            }
            Text {
                width: parent.width
                text: pop.info.description || ""
                visible: text.length > 0
                color: Theme.textMuted; font.family: Theme.fontUi; font.pixelSize: 11
                wrapMode: Text.Wrap
                maximumLineCount: 4
                elide: Text.ElideRight
            }
            Item { width: 1; height: 6 }
            Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderTab }
        }

        Flickable {
            id: flick
            y: head.height + 8
            width: parent.width
            height: parent.height - y - foot.height - 10
            contentHeight: grid.implicitHeight
            clip: true
            boundsBehavior: Flickable.StopAtBounds
            ScrollBar.vertical: ScrollBar { }
            Grid {
                id: grid
                width: flick.width - 12
                columns: pop.entries.length > 12 ? 2 : 1
                columnSpacing: 18
                rowSpacing: 0
                Repeater {
                    model: pop.entries
                    delegate: Item {
                        required property var modelData
                        width: (grid.width - (grid.columns - 1) * grid.columnSpacing) / grid.columns
                        height: Math.max(26, chk.implicitHeight)
                        Text {
                            id: bitNo
                            width: 26
                            anchors.verticalCenter: parent.verticalCenter
                            text: parent.modelData.bit
                            horizontalAlignment: Text.AlignRight
                            color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 10
                        }
                        CfgCheck {
                            id: chk
                            x: 34
                            width: parent.width - 34
                            text: parent.modelData.label
                            checked: pop.isSet(pop.value, parent.modelData.bit)
                            opacity: parent.modelData.known ? 1 : 0.7
                            // CfgCheck flips `checked` itself: bind it to the value again (ALL / NONE)
                            onToggled: (on) => {
                                const bit = parent.modelData.bit
                                pop.setBit(bit, on)
                                checked = Qt.binding(() => pop.isSet(pop.value, bit))
                            }
                        }
                    }
                }
            }
        }

        Column {
            id: foot
            anchors.bottom: parent.bottom
            width: parent.width
            spacing: 10
            Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderTab }
            Item {
                width: parent.width
                height: 28
                Row {
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 8
                    Text { anchors.verticalCenter: parent.verticalCenter; text: qsTr("VALUE"); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5 }
                    Text {
                        anchors.verticalCenter: parent.verticalCenter
                        text: pop.value + "   0x" + pop.value.toString(16).toUpperCase()
                        color: Theme.accentCyan; font.family: Theme.fontData; font.pixelSize: 13
                    }
                    Text {
                        anchors.verticalCenter: parent.verticalCenter
                        visible: pop.info.value !== undefined && pop.info.value !== "" && Number(pop.info.value) !== pop.value
                        text: qsTr("vehicle ") + pop.info.value
                        color: "#ffaa00"; font.family: Theme.fontData; font.pixelSize: 10
                    }
                }
                Row {
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 8
                    GcsButton { height: 28; text: "ALL"; tip: "Set every documented bit"; onClicked: { for (const e of pop.entries) if (e.known) pop.setBit(e.bit, true) } }
                    GcsButton { height: 28; text: "NONE"; onClicked: pop.value = 0 }
                    GcsButton { height: 28; text: "CANCEL"; onClicked: pop.close() }
                    GcsButton {
                        height: 28
                        text: "APPLY"
                        primary: true
                        tip: "Put the value in the table; WRITE CHANGED sends it"
                        onClicked: { Params.setEdit(pop.name, String(pop.value)); pop.close() }
                    }
                }
            }
        }
    }
}
