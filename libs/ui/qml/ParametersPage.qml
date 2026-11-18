// SETUP → PARAMETERS (#subtab-parameters): the side catalog (PARAM LIST: read
// one parameter instead of the whole list) and the table of what is loaded,
// edited in place. Logic in ParamsController ("Params").
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs
import Corv.Ui

Rectangle {
    id: page
    objectName: "parametersPage"
    radius: 6
    color: Theme.glassPanel
    border.color: Theme.borderPanel
    border.width: Theme.hairline
    property string selected: ""
    onVisibleChanged: if (visible) Params.pageShown()

    BitmaskEditor { id: bitmask; parent: Overlay.overlay }

    Text {
        x: 17; y: 17
        text: "PARAMETERS"
        color: Theme.accentCyan
        font.family: Theme.fontUi; font.pixelSize: 12; font.weight: Font.Bold; font.letterSpacing: 2
    }
    Rectangle { x: 17; y: 43; width: parent.width - 34; height: Theme.hairline; color: Theme.borderTab }

    Item {
        id: layout
        x: 17; y: 55
        width: parent.width - 34
        height: parent.height - 72

        // ── Side catalog ──
        Rectangle {
            id: cat
            width: layout.width < 900 ? 220 : 290
            height: parent.height
            radius: 4
            color: "transparent"
            border.color: Qt.rgba(1, 1, 1, 0.06)
            border.width: Theme.hairline

            Column {
                id: catHead
                x: 8; y: 8
                width: cat.width - 16
                spacing: 8
                Item {
                    width: parent.width; height: 16
                    Text { text: "PARAM LIST"; color: Theme.accentCyan; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5 }
                    Text { anchors.right: parent.right; text: Params.catalogCount; color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 10 }
                }
                CfgInput {
                    id: catSearch
                    width: parent.width
                    numeric: false
                    placeholderText: "Type a parameter name..."
                    font.family: Theme.fontUi
                    onTextEdited: catDebounce.restart()
                    // Enter reads the typed name directly
                    Keys.onReturnPressed: { Params.catalogFilter = text; Params.fetchTyped() }
                    Keys.onEnterPressed: { Params.catalogFilter = text; Params.fetchTyped() }
                    Timer { id: catDebounce; interval: 120; onTriggered: Params.catalogFilter = catSearch.text }
                }
                Row {
                    spacing: 6
                    CfgSelect {
                        width: (catHead.width - 6) * 0.6
                        options: [{ value: "", label: "All groups" }].concat(Params.groups.map(g => ({ value: g, label: g })))
                        value: Params.catalogGroup
                        onPicked: (v) => Params.catalogGroup = v
                    }
                    CfgSelect {
                        width: (catHead.width - 6) * 0.4
                        options: [{ value: "2000", label: "2 s" }, { value: "4000", label: "4 s" }, { value: "8000", label: "8 s" }, { value: "15000", label: "15 s" }]
                        value: String(Params.fetchTimeout)
                        onPicked: (v) => Params.fetchTimeout = parseInt(v)
                        ToolTip.visible: hovered; ToolTip.text: "Per-request timeout — raise it on high-latency links"
                    }
                }
            }

            ListView {
                id: catList
                x: 1
                y: catHead.y + catHead.height + 8
                width: cat.width - 2
                height: cat.height - y - catFoot.height - 16
                clip: true
                model: Params.catalog
                ScrollBar.vertical: ScrollBar { }
                Rectangle { anchors.top: parent.top; width: parent.width; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.06) }
                delegate: Rectangle {
                    id: row
                    required property string name
                    required property string value
                    required property bool loaded
                    required property bool custom
                    required property string fetchState
                    required property bool favorite
                    width: catList.width
                    height: 24
                    color: custom ? Qt.rgba(0, 210 / 255, 1, 0.08) : (rowMouse.containsMouse ? Qt.rgba(0, 210 / 255, 1, 0.05) : "transparent")
                    Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.03) }
                    MouseArea { id: rowMouse; anchors.fill: parent; hoverEnabled: true; onClicked: Params.fetch(row.name) }
                    Text {
                        x: 6; width: 14
                        anchors.verticalCenter: parent.verticalCenter
                        horizontalAlignment: Text.AlignHCenter
                        text: row.favorite ? "★" : "☆"
                        color: row.favorite ? "#ffb300" : Qt.rgba(1, 1, 1, 0.18)
                        font.pixelSize: 11
                        MouseArea { anchors.fill: parent; anchors.margins: -4; onClicked: Params.toggleFavorite(row.name) }
                    }
                    Text {
                        x: 26
                        width: parent.width - 26 - 70
                        anchors.verticalCenter: parent.verticalCenter
                        text: row.name
                        elide: Text.ElideRight
                        color: row.loaded ? Theme.textMain : Theme.textMuted
                        font.family: Theme.fontData; font.pixelSize: 10
                    }
                    Text {
                        anchors.right: stateMark.left; anchors.rightMargin: 4
                        anchors.verticalCenter: parent.verticalCenter
                        width: 56
                        horizontalAlignment: Text.AlignRight
                        text: row.value
                        elide: Text.ElideLeft
                        color: Theme.accentCyan
                        font.family: Theme.fontData; font.pixelSize: 10
                    }
                    Text {
                        id: stateMark
                        anchors.right: parent.right; anchors.rightMargin: 6
                        anchors.verticalCenter: parent.verticalCenter
                        width: 10
                        text: row.fetchState === "pending" ? "◌" : row.fetchState === "ok" ? "●" : row.fetchState === "fail" ? "✕" : ""
                        color: row.fetchState === "pending" ? Theme.accentCyan : row.fetchState === "ok" ? "#44ff44" : "#ff4444"
                        font.pixelSize: 10
                    }
                }
                Text {
                    visible: catList.count === 0
                    anchors.centerIn: parent
                    horizontalAlignment: Text.AlignHCenter
                    text: "No matching parameter.\nType a full name to read it anyway."
                    color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 11
                }
            }

            Column {
                id: catFoot
                x: 8
                anchors.bottom: parent.bottom; anchors.bottomMargin: 8
                width: cat.width - 16
                spacing: 6
                Row {
                    spacing: 6
                    GcsButton { width: (catFoot.width - 6) / 2; text: "READ LISTED"; tip: "Read every parameter currently listed"; enabled: Params.linkReady; onClicked: Params.fetchListed() }
                    GcsButton { width: (catFoot.width - 6) / 2; text: "READ ★"; tip: "Read all starred parameters"; enabled: Params.linkReady; onClicked: Params.fetchFavorites() }
                }
                Text {
                    width: parent.width
                    text: Params.queueText
                    visible: text.length > 0
                    elide: Text.ElideRight
                    color: Params.queueBusy ? Theme.accentCyan : Params.queueError ? "#ff8844" : Theme.textDim
                    font.family: Theme.fontData; font.pixelSize: 10
                }
            }
        }

        // ── Table ──
        Item {
            id: main
            x: cat.width + 16
            width: layout.width - x
            height: parent.height

            Flow {
                id: toolbar
                width: parent.width
                spacing: 10
                GcsButton { text: Params.reading ? "STOP" : "READ ALL"; height: 30; enabled: Params.linkReady || Params.reading; onClicked: Params.reading ? Params.cancelRead() : Params.readAll() }
                GcsButton { text: Params.changedCount ? "WRITE CHANGED (" + Params.changedCount + ")" : "WRITE CHANGED"; height: 30; enabled: Params.linkReady; onClicked: Params.writeChanged() }
                Row {
                    visible: Params.readText.length > 0
                    height: 30
                    spacing: 8
                    Rectangle {
                        anchors.verticalCenter: parent.verticalCenter
                        width: 120; height: 6; radius: 3
                        color: Qt.rgba(1, 1, 1, 0.08)
                        Rectangle { width: parent.width * Params.readFraction; height: parent.height; radius: 3; color: Theme.accentCyan }
                    }
                    Text { anchors.verticalCenter: parent.verticalCenter; text: Params.readText; color: Theme.textMuted; font.family: Theme.fontData; font.pixelSize: 10 }
                }
                CfgInput {
                    id: tableSearch
                    width: Math.max(180, main.width - 600)
                    height: 30
                    font.family: Theme.fontUi
                    placeholderText: "Filter loaded parameters..."
                    onTextEdited: tableDebounce.restart()
                    Timer { id: tableDebounce; interval: 200; onTriggered: Params.tableFilter = tableSearch.text }
                }
                GcsButton { text: "SAVE .PARAM"; height: 30; onClicked: saveDialog.open() }
                GcsButton { text: "LOAD .PARAM"; height: 30; enabled: Params.linkReady; onClicked: loadDialog.open() }
            }

            Rectangle {
                id: tableFrame
                y: toolbar.height + 12
                width: parent.width
                height: parent.height - y - detail.height - 8
                color: "transparent"
                border.color: Qt.rgba(1, 1, 1, 0.06)
                border.width: Theme.hairline
                radius: 4
                clip: true

                readonly property real nameW: 170
                readonly property real valueW: 170
                readonly property real unitsW: 64
                readonly property real meaningW: Math.max(150, Math.min(300, width * 0.22))
                readonly property real descX: nameW + valueW + unitsW + meaningW

                Rectangle {
                    id: head
                    width: parent.width; height: 30
                    color: Theme.glassToolbar
                    Row {
                        anchors.fill: parent
                        Repeater {
                            model: [["NAME", tableFrame.nameW], ["VALUE", tableFrame.valueW], ["UNITS", tableFrame.unitsW],
                                    ["MEANING / RANGE", tableFrame.meaningW], ["DESCRIPTION", tableFrame.width - tableFrame.descX]]
                            delegate: Text {
                                required property var modelData
                                width: modelData[1]; height: 30
                                leftPadding: 12
                                verticalAlignment: Text.AlignVCenter
                                text: modelData[0]
                                elide: Text.ElideRight
                                color: Theme.accentCyan
                                font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5
                            }
                        }
                    }
                }

                ListView {
                    id: table
                    y: head.height
                    width: parent.width
                    height: parent.height - head.height
                    clip: true
                    model: Params.table
                    reuseItems: true
                    ScrollBar.vertical: ScrollBar { }
                    delegate: Rectangle {
                        id: prow
                        required property string name
                        required property string value
                        required property string desc
                        required property bool edited
                        required property string editText
                        required property string title
                        required property string units
                        required property string kind
                        required property string meaning
                        required property var options
                        required property bool outOfRange
                        readonly property bool selected: page.selected === name
                        width: table.width
                        height: 30
                        color: selected ? Qt.rgba(0, 210 / 255, 1, 0.09) : (hover.hovered ? Qt.rgba(0, 210 / 255, 1, 0.04) : "transparent")
                        HoverHandler { id: hover }
                        MouseArea { anchors.fill: parent; onClicked: page.selected = prow.name }
                        Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.04) }
                        Rectangle { visible: prow.selected; width: 2; height: parent.height; color: Theme.accentCyan }
                        Text {
                            width: tableFrame.nameW
                            height: parent.height
                            leftPadding: 12
                            verticalAlignment: Text.AlignVCenter
                            text: prow.name
                            elide: Text.ElideRight
                            color: Theme.textMain
                            font.family: Theme.fontData; font.pixelSize: 11
                        }
                        // A listed value: pick it; a bitmask: the number and the bit editor; else the number
                        CfgSelect {
                            visible: prow.kind === "values" && prow.options.length > 0
                            x: tableFrame.nameW + 6
                            anchors.verticalCenter: parent.verticalCenter
                            width: tableFrame.valueW - 12
                            height: 24
                            popupMinWidth: 280
                            options: visible ? prow.options : []
                            value: prow.editText
                            onPicked: (v) => { page.selected = prow.name; Params.setEdit(prow.name, v) }
                            Rectangle {
                                anchors.fill: parent
                                radius: 3
                                color: "transparent"
                                visible: prow.edited
                                border.color: "#ffaa00"
                                border.width: Theme.hairline
                            }
                        }
                        CfgInput {
                            visible: !(prow.kind === "values" && prow.options.length > 0)
                            x: tableFrame.nameW + 6
                            anchors.verticalCenter: parent.verticalCenter
                            width: tableFrame.valueW - 12 - (prow.kind === "bitmask" ? 30 : 0)
                            height: 24
                            text: prow.editText
                            color: Theme.accentCyan
                            frame: prow.outOfRange ? "#ff4444" : prow.edited ? "#ffaa00" : Theme.borderInput
                            onActiveFocusChanged: if (activeFocus) page.selected = prow.name
                            onCommitted: (t) => Params.setEdit(prow.name, t)
                        }
                        GcsButton {
                            visible: prow.kind === "bitmask"
                            x: tableFrame.nameW + tableFrame.valueW - 6 - width
                            anchors.verticalCenter: parent.verticalCenter
                            width: 26; height: 24
                            text: "▦"
                            fontSize: 12
                            tip: "Edit the bits"
                            onClicked: { page.selected = prow.name; bitmask.openFor(prow.name) }
                        }
                        Text {
                            x: tableFrame.nameW + tableFrame.valueW
                            width: tableFrame.unitsW
                            height: parent.height
                            leftPadding: 12
                            verticalAlignment: Text.AlignVCenter
                            text: prow.units
                            elide: Text.ElideRight
                            color: Theme.textMuted
                            font.family: Theme.fontData; font.pixelSize: 10
                        }
                        Text {
                            x: tableFrame.nameW + tableFrame.valueW + tableFrame.unitsW
                            width: tableFrame.meaningW
                            height: parent.height
                            leftPadding: 12
                            verticalAlignment: Text.AlignVCenter
                            text: prow.meaning
                            elide: Text.ElideRight
                            color: prow.outOfRange ? "#ff6666" : prow.kind === "number" ? Theme.textDim : Theme.textMutedBright
                            font.family: prow.kind === "number" ? Theme.fontData : Theme.fontUi
                            font.pixelSize: prow.kind === "number" ? 10 : 11
                        }
                        Row {
                            x: tableFrame.descX
                            width: parent.width - x - 8
                            height: parent.height
                            leftPadding: 12
                            spacing: 8
                            Text {
                                id: titleText
                                width: Math.min(implicitWidth, (parent.width - 12) * 0.5)
                                height: parent.height
                                verticalAlignment: Text.AlignVCenter
                                text: prow.title
                                visible: text.length > 0
                                elide: Text.ElideRight
                                color: Theme.textMain
                                font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.DemiBold
                            }
                            Text {
                                width: parent.width - 12 - (titleText.visible ? titleText.width + 8 : 0)
                                height: parent.height
                                verticalAlignment: Text.AlignVCenter
                                text: prow.desc
                                elide: Text.ElideRight
                                color: Theme.textDim
                                font.family: Theme.fontUi; font.pixelSize: 11
                            }
                        }
                    }
                    Text {
                        visible: table.count === 0
                        anchors.centerIn: parent
                        horizontalAlignment: Text.AlignHCenter
                        text: Params.loadedCount === 0 ? "No parameters loaded.\nREAD ALL, or pick names in the PARAM LIST." : "No parameter matches the filter."
                        color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 12
                    }
                }
            }

            // ── Documentation of the selected parameter ──
            Rectangle {
                id: detail
                readonly property var info: page.selected.length > 0 && Params.metaRev >= 0 ? Params.meta(page.selected) : null
                anchors.bottom: parent.bottom
                width: parent.width
                height: 132
                radius: 4
                color: Qt.rgba(1, 1, 1, 0.02)
                border.color: Qt.rgba(1, 1, 1, 0.06)
                border.width: Theme.hairline

                component Badge: Rectangle {
                    property alias text: badgeText.text
                    property color tint: Theme.groupAmber
                    anchors.verticalCenter: parent.verticalCenter
                    width: badgeText.implicitWidth + 10; height: 16; radius: 2
                    color: Qt.rgba(tint.r, tint.g, tint.b, 0.12)
                    border.color: Qt.rgba(tint.r, tint.g, tint.b, 0.5)
                    border.width: Theme.hairline
                    Text { id: badgeText; anchors.centerIn: parent; color: parent.tint; font.family: Theme.fontUi; font.pixelSize: 9; font.weight: Font.Bold; font.letterSpacing: 1 }
                }

                Text {
                    visible: !detail.info
                    anchors.centerIn: parent
                    text: "Click a parameter to see what it does, its range, its values or its bits."
                    color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 12
                }

                Column {
                    visible: !!detail.info
                    x: 12; y: 10
                    width: parent.width - 24
                    spacing: 6
                    Row {
                        spacing: 10
                        height: 18
                        Text { anchors.verticalCenter: parent.verticalCenter; text: detail.info ? detail.info.name : ""; color: Theme.accentCyan; font.family: Theme.fontData; font.pixelSize: 12; font.weight: Font.Bold }
                        Text {
                            anchors.verticalCenter: parent.verticalCenter
                            text: detail.info && detail.info.title ? detail.info.title : (detail.info && !detail.info.known ? "Not documented for this firmware" : "")
                            color: detail.info && detail.info.known ? Theme.textMain : Theme.textDim
                            font.family: Theme.fontUi; font.pixelSize: 13; font.weight: Font.DemiBold
                        }
                        Badge { visible: !!detail.info && !!detail.info.reboot; text: "REBOOT REQUIRED" }
                        Badge { visible: !!detail.info && !!detail.info.readOnly; text: "READ ONLY"; tint: Theme.statusRed }
                        Badge { visible: !!detail.info && detail.info.user === "Advanced"; text: "ADVANCED"; tint: Theme.textNav }
                        Badge { visible: !!detail.info && !!detail.info.outOfRange; text: "OUT OF RANGE"; tint: Theme.statusRed }
                    }
                    Text {
                        width: parent.width
                        text: detail.info ? (detail.info.description || "") : ""
                        wrapMode: Text.Wrap
                        maximumLineCount: 3
                        elide: Text.ElideRight
                        color: Theme.textMutedBright
                        font.family: Theme.fontUi; font.pixelSize: 12
                    }
                    Row {
                        spacing: 18
                        height: 26
                        Repeater {
                            model: detail.info ? [
                                ["VALUE", detail.info.current + (detail.info.edited && detail.info.value !== "" ? "  (vehicle " + detail.info.value + ")" : "")],
                                ["UNITS", detail.info.units || ""],
                                ["RANGE", detail.info.range || ""],
                                ["INCREMENT", detail.info.increment || ""],
                                [detail.info.kind === "bitmask" ? "BITS SET" : "MEANING", detail.info.kind === "number" ? "" : (detail.info.meaning || "")]
                            ].filter(f => f[1] !== "") : []
                            delegate: Row {
                                required property var modelData
                                anchors.verticalCenter: parent.verticalCenter
                                spacing: 6
                                Text { anchors.verticalCenter: parent.verticalCenter; text: parent.modelData[0]; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.2 }
                                Text {
                                    anchors.verticalCenter: parent.verticalCenter
                                    text: parent.modelData[1]
                                    width: Math.min(implicitWidth, 420)
                                    elide: Text.ElideRight
                                    color: Theme.accentCyan; font.family: Theme.fontData; font.pixelSize: 11
                                }
                            }
                        }
                        GcsButton {
                            visible: !!detail.info && detail.info.kind === "bitmask"
                            anchors.verticalCenter: parent.verticalCenter
                            height: 24
                            text: "EDIT BITS"
                            onClicked: bitmask.openFor(page.selected)
                        }
                    }
                }

                // Where the descriptions come from, and Mission Planner's metadata update
                Row {
                    anchors.right: parent.right; anchors.rightMargin: 10
                    anchors.bottom: parent.bottom; anchors.bottomMargin: 8
                    spacing: 8
                    Text {
                        anchors.verticalCenter: parent.verticalCenter
                        text: Params.metaSource
                        color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 9
                    }
                    GcsButton {
                        height: 20
                        fontSize: 9
                        text: Params.metaBusy ? "DOWNLOADING…" : "UPDATE DESCRIPTIONS"
                        enabled: !Params.metaBusy
                        tip: "Download the latest parameter documentation of this firmware from autotest.ardupilot.org"
                        onClicked: Params.updateMeta()
                    }
                    GcsButton {
                        height: 20
                        fontSize: 9
                        text: "BUNDLED"
                        visible: Params.metaSource.indexOf("autotest") >= 0
                        tip: "Go back to the documentation shipped with the GCS"
                        onClicked: Params.resetMeta()
                    }
                }
            }
        }
    }

    FileDialog {
        id: saveDialog
        title: "Save parameters"
        fileMode: FileDialog.SaveFile
        defaultSuffix: "param"
        nameFilters: ["Parameter files (*.param *.txt)"]
        onAccepted: Params.saveFile(selectedFile)
    }
    FileDialog {
        id: loadDialog
        title: "Load parameters"
        nameFilters: ["Parameter files (*.param *.txt)", "All files (*)"]
        onAccepted: Params.loadFile(selectedFile)
    }
}
