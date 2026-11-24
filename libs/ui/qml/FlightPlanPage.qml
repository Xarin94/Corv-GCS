// FLIGHT PLAN tab (#tab-flight-plan, flight-plan.css): segment panel on the
// left (route card, segment cards with their inspector, popovers), the map in
// the middle with the tool palette and the layer buttons, the elevation
// profile along the bottom. The logic is PlanController ("Plan").
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs
import Corv.Ui

Item {
    id: page
    property bool shown: false
    property string popover: ""          // route | camera | radio
    property bool menuOpen: false
    onShownChanged: Plan.pageShown(shown)

    // ── Icons of FlightPlanController.js (24 px viewBox) ──
    readonly property var icon: ({
        select: [{ d: "M5 3l14 8-6 2-3 6z", stroke: 2, join: "round" }],
        waypoint: [{ d: "M12 21s7-6.5 7-12a7 7 0 1 0-14 0c0 5.5 7 12 7 12z", stroke: 2, join: "round" }, { d: Icons.circle(12, 9, 2.5), stroke: 2 }],
        circle: [{ d: Icons.circle(12, 12, 8), stroke: 2 }, { d: Icons.circle(12, 12, 1.5), fill: true }, { d: "M12 12h8", stroke: 2, dash: [2, 2] }],
        perimeter: [{ d: "M5 6l9-2 6 7-4 9-11-3z", stroke: 2, join: "round" }, { d: Icons.circle(5, 6, 1.5), fill: true },
                    { d: Icons.circle(14, 4, 1.5), fill: true }, { d: Icons.circle(20, 11, 1.5), fill: true },
                    { d: Icons.circle(16, 20, 1.5), fill: true }, { d: Icons.circle(5, 17, 1.5), fill: true }],
        area: [{ d: "M4 5l16-1v15L4 20z", stroke: 2, join: "round" }, { d: "M7 8h10M7 12h10M7 16h10", stroke: 1.3 }],
        corridor: [{ d: "M3 18c4-2 5-9 9-9s5 6 9 4", stroke: 2, cap: "round" }, { d: "M3 13c4-2 5-9 9-9s5 6 9 4", stroke: 1.2, cap: "round", opacity: 0.6 },
                   { d: "M3 23c4-2 5-9 9-9s5 6 9 4", stroke: 1.2, cap: "round", opacity: 0.6 }],
        poi: [{ d: Icons.circle(12, 12, 3), stroke: 2 }, { d: "M12 2v4M12 18v4M2 12h4M18 12h4", stroke: 2 }],
        landing: [{ d: "M12 3v11", stroke: 2, cap: "round" }, { d: "M7 10l5 5 5-5", stroke: 2, cap: "round" }, { d: "M4 20h16", stroke: 2, cap: "round" }],
        operator: [{ d: "M12 22v-9", stroke: 2, cap: "round" }, { d: Icons.circle(12, 10.5, 2.2), stroke: 2 },
                   { d: "M8.3 6.8a5.2 5.2 0 0 0 0 7.4M15.7 6.8a5.2 5.2 0 0 1 0 7.4", stroke: 2, cap: "round" },
                   { d: "M5.5 4a9.2 9.2 0 0 0 0 13M18.5 4a9.2 9.2 0 0 1 0 13", stroke: 2, cap: "round" }],
        trash: [{ d: "M4 7h16M10 11v6M14 11v6M6 7l1 13h10l1-13M9 7V4h6v3", stroke: 2, cap: "round" }],
        warn: [{ d: "M12 3L2 21h20z", stroke: 2, join: "round" }, { d: "M12 10v5M12 18v.5", stroke: 2 }],
        camera: [{ d: "M3 8h4l2-3h6l2 3h4v11H3z", stroke: 2, join: "round" }, { d: Icons.circle(12, 13, 3.5), stroke: 2 }],
        gear: [{ d: Icons.circle(12, 12, 3), stroke: 2 },
               { d: "M19.4 15a1.7 1.7 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.8-.3 1.7 1.7 0 0 0-1 1.5V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1.1-1.5 1.7 1.7 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.8 1.7 1.7 0 0 0-1.5-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.5-1.1 1.7 1.7 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.8.3H9a1.7 1.7 0 0 0 1-1.5V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.5 1.7 1.7 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.8V9a1.7 1.7 0 0 0 1.5 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.5 1z", stroke: 2 }],
        menu: [{ d: Icons.circle(5, 12, 2), fill: true }, { d: Icons.circle(12, 12, 2), fill: true }, { d: Icons.circle(19, 12, 2), fill: true }],
        undo: [{ d: "M9 14 4 9l5-5", stroke: 2, cap: "round" }, { d: "M4 9h11a5 5 0 0 1 0 10h-3", stroke: 2, cap: "round" }],
        redo: [{ d: "m15 14 5-5-5-5", stroke: 2, cap: "round" }, { d: "M20 9H9a5 5 0 0 0 0 10h3", stroke: 2, cap: "round" }]
    })

    function togglePopover(which) { menuOpen = false; popover = popover === which ? "" : which }
    function boldify(s) { return String(s).replace(/<b>/g, "<font color=\"#cfd8dd\">").replace(/<\/b>/g, "</font>") }

    Connections {
        target: Plan
        function onOpenPopover(which) { page.menuOpen = false; page.popover = which }
        function onScrollToSegment(index) { segList.positionViewAtIndex(index, ListView.Contain) }
        function onSaveFileRequested(name) { exportDialog.selectedFile = "file:///" + name; exportDialog.open() }
    }

    // ── Keyboard (initKeyboard): text fields keep their own keys ──
    Shortcut { sequence: "Escape"; enabled: page.shown; onActivated: { ctxMenu.visible = false; page.popover = ""; page.menuOpen = false; Plan.escape() } }
    Shortcut { sequences: ["Return", "Enter"]; enabled: page.shown && Plan.drafting; onActivated: Plan.finishDraft() }
    Shortcut { sequences: ["Delete", "Backspace"]; enabled: page.shown && Plan.selectedId.length > 0; onActivated: Plan.deleteSelected() }
    Shortcut { sequence: "Ctrl+Z"; enabled: page.shown; onActivated: Plan.undo() }
    Shortcut { sequences: ["Ctrl+Y", "Ctrl+Shift+Z"]; enabled: page.shown; onActivated: Plan.redo() }
    Repeater {
        model: ["V", "W", "C", "P", "A", "K", "I", "L", "O"]
        delegate: Item {
            required property string modelData
            Shortcut { sequence: parent.modelData; enabled: page.shown; onActivated: Plan.keyTool(parent.modelData) }
        }
    }

    // ═══════════════════════ Layout ═══════════════════════
    Item {
        id: layout
        width: parent.width
        height: parent.height - 177

        // ── Side panel ──
        Rectangle {
            id: side
            width: 321
            height: parent.height
            color: Theme.glassPanel
            Rectangle { anchors.right: parent.right; width: Theme.hairline; height: parent.height; color: Theme.borderAccent }

            // Route card
            Rectangle {
                id: card
                width: 320
                height: cardCol.implicitHeight + 18
                color: Qt.rgba(0, 15 / 255, 25 / 255, 0.5)
                Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderAccent }

                Column {
                    id: cardCol
                    x: 12
                    y: 10
                    width: 296
                    spacing: 0

                    // Head: status, name, camera, radio, settings, menu
                    Item {
                        width: parent.width
                        height: 24
                        Rectangle {
                            id: statusDot
                            width: 10; height: 10; radius: 5
                            anchors.verticalCenter: parent.verticalCenter
                            color: Plan.status === "ok" ? Theme.accentGreen : Plan.status === "warn" ? Theme.accentOrange
                                 : Plan.status === "error" ? "#ff3b3b" : Plan.status === "busy" ? Theme.accentCyan : "#3a4550"
                            Rectangle {
                                z: -1
                                anchors.centerIn: parent
                                width: 14; height: 14; radius: 7
                                color: Plan.status === "ok" ? Qt.rgba(68 / 255, 1, 68 / 255, 0.3) : Plan.status === "warn" ? Qt.rgba(1, 170 / 255, 0, 0.3)
                                     : Plan.status === "error" ? Qt.rgba(1, 60 / 255, 60 / 255, 0.35) : Qt.rgba(1, 1, 1, 0.06)
                            }
                            SequentialAnimation on opacity {
                                running: Plan.status === "busy"
                                loops: Animation.Infinite
                                NumberAnimation { to: 0.25; duration: 400; easing.type: Easing.InOutSine }
                                NumberAnimation { to: 1; duration: 400; easing.type: Easing.InOutSine }
                                onStopped: statusDot.opacity = 1
                            }
                            HoverHandler { id: statusHover }
                            ToolTip.visible: statusHover.hovered
                            ToolTip.text: Theme.tr(Plan.statusTitle)
                        }
                        TextField {
                            id: nameField
                            x: 18
                            width: 150
                            height: 24
                            leftPadding: 0
                            rightPadding: 0
                            topPadding: 2
                            bottomPadding: 2
                            text: Plan.routeName
                            placeholderText: qsTr("Untitled route")
                            placeholderTextColor: Theme.textDim
                            color: Theme.textMain
                            font.family: Theme.fontUi
                            font.pixelSize: 15
                            font.weight: Font.Bold
                            font.letterSpacing: 1
                            selectByMouse: true
                            background: Rectangle {
                                color: "transparent"
                                Rectangle {
                                    anchors.bottom: parent.bottom
                                    width: parent.width
                                    height: Theme.hairline
                                    color: nameField.activeFocus || nameHover.hovered ? Theme.borderAccentMid : "transparent"
                                }
                            }
                            HoverHandler { id: nameHover }
                            onEditingFinished: if (text !== Plan.routeName) Plan.setRouteName(text)
                        }
                        Row {
                            anchors.right: parent.right
                            spacing: 8
                            FpIconButton { parts: page.icon.camera; iconSize: 14; tip: "Camera & gimbal"; onClicked: page.togglePopover("camera") }
                            FpIconButton { parts: page.icon.operator; iconSize: 14; tip: "Radio link"; onClicked: page.togglePopover("radio") }
                            FpIconButton { parts: page.icon.gear; iconSize: 14; tip: "Route settings"; onClicked: page.togglePopover("route") }
                            FpIconButton { parts: page.icon.menu; iconSize: 14; tip: "Route menu"; onClicked: { page.popover = ""; page.menuOpen = !page.menuOpen } }
                        }
                    }
                    Item { width: 1; height: 8 }

                    // Stats
                    Row {
                        spacing: 4
                        Repeater {
                            model: ["LENGTH", "TIME", "WAYPOINTS", "MAX AGL"]
                            delegate: Rectangle {
                                required property string modelData
                                required property int index
                                width: (296 - 12) / 4
                                height: 37
                                radius: 3
                                color: Qt.rgba(0, 0, 0, 0.25)
                                border.color: Theme.borderSubtle
                                border.width: Theme.hairline
                                Text {
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    y: 5
                                    text: Plan.stats[parent.index]
                                    color: Theme.textMain
                                    font.family: Theme.fontData
                                    font.pixelSize: 13
                                }
                                Text {
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    y: 22
                                    text: Theme.tr(parent.modelData)
                                    color: Theme.textDim
                                    font.family: Theme.fontUi
                                    font.pixelSize: 8
                                    font.weight: Font.Bold
                                    font.letterSpacing: 1
                                }
                            }
                        }
                    }
                    Item { width: 1; height: 8 }

                    // Chips: the route settings at a glance (a click opens them)
                    Flow {
                        id: chips
                        width: parent.width
                        spacing: 4
                        HoverHandler { id: chipsHover; cursorShape: Qt.PointingHandCursor }
                        TapHandler { onTapped: page.togglePopover("route") }
                        Repeater {
                            model: Plan.chips
                            delegate: Rectangle {
                                required property var modelData
                                readonly property color tone: modelData.cls === "link-good" ? "#60ff80" : modelData.cls === "link-degraded" ? "#ffaa00"
                                                            : modelData.cls === "link-marginal" ? "#ff5c5c" : modelData.cls === "link-none" ? "#9aa4b0" : Theme.textMutedBright
                                width: chipText.implicitWidth + 12
                                height: 15
                                radius: 2
                                color: Qt.rgba(0, 210 / 255, 1, 0.06)
                                border.color: modelData.cls ? Qt.rgba(tone.r, tone.g, tone.b, 0.45) : chipsHover.hovered ? Theme.borderAccentMid : Theme.borderAccent
                                border.width: Theme.hairline
                                Text {
                                    id: chipText
                                    anchors.centerIn: parent
                                    text: Theme.tr(parent.modelData.text)
                                    color: parent.tone
                                    font.family: Theme.fontData
                                    font.pixelSize: 9
                                    font.letterSpacing: 0.3
                                }
                            }
                        }
                    }
                    Item { width: 1; height: 6 }

                    // Issues
                    Column {
                        width: parent.width
                        spacing: 2
                        visible: Plan.issues.length > 0
                        bottomPadding: 6
                        Repeater {
                            model: Plan.issues
                            delegate: Rectangle {
                                required property var modelData
                                readonly property bool err: modelData.level === "error"
                                readonly property bool wrn: modelData.level === "warn"
                                width: parent.width
                                height: issueText.implicitHeight + 6
                                color: err ? Qt.rgba(1, 60 / 255, 60 / 255, 0.1) : wrn ? Qt.rgba(1, 170 / 255, 0, 0.08) : Qt.rgba(1, 1, 1, 0.03)
                                Rectangle { width: 2; height: parent.height; color: parent.err ? "#ff3b3b" : parent.wrn ? Theme.accentOrange : Theme.borderAccentMid }
                                Text {
                                    id: issueText
                                    x: 8
                                    y: 3
                                    width: parent.width - 14
                                    wrapMode: Text.Wrap
                                    textFormat: Text.StyledText
                                    text: (parent.modelData.where ? "<font face=\"Roboto Mono\" color=\"#ffffff\"><b>" + parent.modelData.where + "</b></font> " : "") + Theme.tr(parent.modelData.text)
                                    color: parent.err ? "#ff9a9a" : parent.wrn ? "#ffd28a" : Theme.textMuted
                                    font.family: Theme.fontUi
                                    font.pointSize: 7.875
                                    font.weight: Font.DemiBold
                                    font.letterSpacing: 0.3
                                }
                                MouseArea {
                                    anchors.fill: parent
                                    enabled: parent.modelData.segId.length > 0
                                    cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                                    onClicked: Plan.issueClicked(parent.modelData.segId)
                                }
                            }
                        }
                    }

                    Row {
                        spacing: 6
                        GcsButton {
                            width: 145; height: 27; fontSize: 10.5; spacing: 0.8
                            primary: true
                            blocked: Plan.uploadBlocked
                            text: Plan.uploadText
                            tip: "Write the calculated route to the autopilot"
                            onClicked: Plan.upload()
                        }
                        GcsButton {
                            width: 145; height: 27; fontSize: 10.5; spacing: 0.8
                            text: Plan.readText
                            tip: "Read the mission stored on the autopilot"
                            onClicked: Plan.read()
                        }
                    }
                    Item { width: 1; height: 6 }
                    Row {
                        spacing: 6
                        GcsButton { width: 145; height: 27; fontSize: 10.5; spacing: 0.8; text: Plan.saveText; tip: "Save to the local mission library"; onClicked: Plan.save() }
                        GcsButton { width: 145; height: 27; fontSize: 10.5; spacing: 0.8; text: "LIBRARY"; tip: "Open the local mission library"; onClicked: Plan.openLibrary() }
                    }
                }
            }

            // List head
            Item {
                id: listHead
                y: card.height
                width: 320
                height: 38
                Row {
                    x: 12
                    y: 13
                    spacing: 0
                    Text { text: qsTr("SEGMENTS "); color: Theme.accentCyan; font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.Bold; font.letterSpacing: 2 }
                    Text { text: Plan.segmentCount ? "(" + Plan.segmentCount + ")" : ""; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.Bold }
                }
                Row {
                    x: 258
                    y: 8
                    spacing: 2
                    FpIconButton { parts: page.icon.undo; enabled: Plan.canUndo; tip: Plan.undoTip; onClicked: Plan.undo() }
                    FpIconButton { parts: page.icon.redo; enabled: Plan.canRedo; tip: Plan.redoTip; onClicked: Plan.redo() }
                }
            }

            // Empty plan
            Column {
                visible: Plan.segmentCount === 0
                x: 18
                y: listHead.y + listHead.height + 18
                width: 320 - 36
                spacing: 6
                Text { text: qsTr("No segments yet"); color: Theme.textMain; font.family: Theme.fontUi; font.pixelSize: 13; font.weight: Font.Bold; font.letterSpacing: 1 }
                Text {
                    width: parent.width
                    wrapMode: Text.Wrap
                    textFormat: Text.StyledText
                    lineHeight: 1.5
                    text: qsTr("Pick a tool on the map: a <font color=\"#cfd8dd\"><b>waypoint</b></font>, a <font color=\"#cfd8dd\"><b>circle</b></font>, a <font color=\"#cfd8dd\"><b>perimeter</b></font> to fly around, an <font color=\"#cfd8dd\"><b>area</b></font> to scan, a <font color=\"#cfd8dd\"><b>corridor</b></font> to map, a <font color=\"#cfd8dd\"><b>POI</b></font> for the camera, a <font color=\"#cfd8dd\"><b>landing</b></font>.")
                    color: Theme.textMuted
                    font.family: Theme.fontUi
                    font.pointSize: 8.625
                }
                Text {
                    topPadding: 4
                    width: parent.width
                    wrapMode: Text.Wrap
                    textFormat: Text.StyledText
                    text: qsTr("The route is calculated as you draw. Open <font color=\"#cfd8dd\"><b>⚙ Route settings</b></font> for altitude mode, speed, take-off and what happens at the end.")
                    color: Theme.textDim
                    font.family: Theme.fontUi
                    font.pointSize: 7.875
                }
            }

            // Segment cards
            ListView {
                id: segList
                x: 8
                y: listHead.y + listHead.height
                width: 304
                height: side.height - y
                clip: true
                spacing: 4
                bottomMargin: 12
                boundsBehavior: Flickable.StopAtBounds
                model: Plan.segmentModel
                ScrollBar.vertical: ScrollBar { width: 6 }
                property int dragFrom: -1
                property int dragOver: -1

                delegate: Rectangle {
                    id: seg
                    required property string segId
                    required property int segIndex
                    required property string segType
                    required property string segLabel
                    required property string segColor
                    required property string summary
                    required property string worst
                    required property string issuesText
                    required property bool open

                    width: segList.width
                    height: segHead.height + (open ? inspector.height : 0) + 2 * Theme.hairline
                    radius: 3
                    color: open ? Qt.rgba(0, 210 / 255, 1, 0.05) : segHover.hovered ? Qt.rgba(0, 210 / 255, 1, 0.04) : Qt.rgba(0, 0, 0, 0.2)
                    border.color: worst === "error" ? Qt.rgba(1, 60 / 255, 60 / 255, 0.35) : open ? Theme.borderAccentMid : Theme.borderSubtle
                    border.width: Theme.hairline
                    opacity: segList.dragFrom === segIndex ? 0.4 : 1
                    HoverHandler { id: segHover }

                    // border-left: 3px solid var(--seg-color)
                    Rectangle { width: 2.67; height: parent.height; color: seg.segColor; radius: 1 }
                    Rectangle { visible: segList.dragOver === seg.segIndex && segList.dragFrom !== seg.segIndex; width: parent.width; height: 2; color: Theme.accentCyan }

                    Item {
                        id: segHead
                        width: parent.width
                        height: 37
                        Rectangle {
                            id: idxPill
                            x: 11
                            anchors.verticalCenter: parent.verticalCenter
                            width: Math.max(18, idxText.implicitWidth) + 8
                            height: 18
                            radius: 9
                            color: seg.segColor
                            Text { id: idxText; anchors.centerIn: parent; text: seg.segIndex + 1; color: "#000000"; font.family: Theme.fontData; font.pixelSize: 10; font.weight: Font.Bold }
                        }
                        SvgIcon {
                            id: segIcon
                            x: idxPill.x + idxPill.width + 7
                            anchors.verticalCenter: parent.verticalCenter
                            width: 15; height: 15
                            parts: page.icon[seg.segType] || []
                            color: seg.segColor
                        }
                        Column {
                            x: segIcon.x + 15 + 7
                            anchors.verticalCenter: parent.verticalCenter
                            width: delBtn.x - x - 7 - (badge.visible ? 20 : 0)
                            Text { width: parent.width; elide: Text.ElideRight; text: Theme.tr(seg.segLabel); color: Theme.textMain; font.family: Theme.fontUi; font.pixelSize: 12; font.weight: Font.Bold; font.letterSpacing: 0.8 }
                            Text { width: parent.width; elide: Text.ElideRight; text: Theme.tr(seg.summary); color: Theme.textMuted; font.family: Theme.fontData; font.pixelSize: 9 }
                        }
                        SvgIcon {
                            id: badge
                            visible: seg.worst.length > 0
                            x: delBtn.x - 20
                            anchors.verticalCenter: parent.verticalCenter
                            width: 13; height: 13
                            parts: page.icon.warn
                            color: seg.worst === "error" ? "#ff3b3b" : Theme.accentOrange
                            HoverHandler { id: badgeHover }
                            ToolTip.visible: badgeHover.hovered
                            ToolTip.text: Theme.tr(seg.issuesText)
                        }
                        // Head clicks: open / close (double click also pans), drag to reorder
                        MouseArea {
                            anchors.fill: parent
                            anchors.rightMargin: 30
                            property real pressY: 0
                            property bool dragging: false
                            cursorShape: dragging ? Qt.ClosedHandCursor : Qt.PointingHandCursor
                            onPressed: (m) => { pressY = m.y; dragging = false }
                            onPositionChanged: (m) => {
                                if (!dragging && Math.abs(m.y - pressY) > 6) { dragging = true; segList.dragFrom = seg.segIndex }
                                if (dragging) {
                                    const p = mapToItem(segList.contentItem, m.x, m.y)
                                    segList.dragOver = segList.indexAt(10, p.y)
                                }
                            }
                            onReleased: {
                                if (dragging && segList.dragOver >= 0 && segList.dragOver !== segList.dragFrom) Plan.moveSegment(segList.dragFrom, segList.dragOver)
                                else if (!dragging) Plan.toggleSegment(seg.segId, false)
                                dragging = false
                                segList.dragFrom = -1
                                segList.dragOver = -1
                            }
                            onDoubleClicked: Plan.selectSegment(seg.segId, true)
                        }
                        FpIconButton {
                            id: delBtn
                            x: parent.width - width - 6
                            anchors.verticalCenter: parent.verticalCenter
                            parts: page.icon.trash
                            opacity: segHover.hovered || seg.open ? 1 : 0
                            iconColor: hovered2.hovered ? "#ff5c5c" : Theme.textMuted
                            tip: "Delete segment"
                            HoverHandler { id: hovered2 }
                            onClicked: Plan.deleteSegment(seg.segId)
                        }
                    }

                    // Inspector
                    Item {
                        id: inspector
                        visible: seg.open
                        y: segHead.height
                        width: parent.width
                        height: seg.open ? inspCol.implicitHeight + 12 : 0
                        Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                        Column {
                            id: inspCol
                            x: 11
                            y: 4
                            width: parent.width - 19
                            spacing: 0

                            Column {
                                width: parent.width
                                spacing: 2
                                topPadding: 4
                                bottomPadding: 6
                                visible: seg.open && Plan.inspectorIssues.length > 0
                                Repeater {
                                    model: seg.open ? Plan.inspectorIssues : []
                                    delegate: Rectangle {
                                        required property var modelData
                                        readonly property bool err: modelData.level === "error"
                                        width: parent.width
                                        height: it.implicitHeight + 6
                                        color: err ? Qt.rgba(1, 60 / 255, 60 / 255, 0.1) : Qt.rgba(1, 170 / 255, 0, 0.08)
                                        Rectangle { width: 2; height: parent.height; color: parent.err ? "#ff3b3b" : Theme.accentOrange }
                                        Text {
                                            id: it
                                            x: 8; y: 3
                                            width: parent.width - 14
                                            wrapMode: Text.Wrap
                                            text: Theme.tr(parent.modelData.text)
                                            color: parent.err ? "#ff9a9a" : "#ffd28a"
                                            font.family: Theme.fontUi; font.pointSize: 7.875; font.weight: Font.DemiBold; font.letterSpacing: 0.3
                                        }
                                    }
                                }
                            }

                            Column {
                                width: parent.width
                                spacing: 3
                                Repeater {
                                    model: seg.open ? Plan.inspectorFields : []
                                    delegate: FpField {
                                        required property var modelData
                                        field: modelData
                                        width: parent.width
                                        onEdited: (key, value) => Plan.setField("seg", seg.segId, -1, key, value)
                                    }
                                }
                            }

                            // .fp-derived
                            Rectangle {
                                visible: seg.open && Plan.inspectorDerived.length > 0
                                width: parent.width
                                height: visible ? derivedFlow.implicitHeight + 10 + 8 : 0
                                color: "transparent"
                                Rectangle {
                                    y: 6
                                    width: parent.width
                                    height: derivedFlow.implicitHeight + 10
                                    radius: 2
                                    color: Qt.rgba(0, 0, 0, 0.25)
                                    Flow {
                                        id: derivedFlow
                                        x: 8; y: 5
                                        width: parent.width - 16
                                        spacing: 10
                                        Repeater {
                                            model: seg.open ? Plan.inspectorDerived : []
                                            delegate: Text {
                                                required property string modelData
                                                textFormat: Text.StyledText
                                                text: Theme.tr(page.boldify(modelData))
                                                color: Theme.textDim
                                                font.family: Theme.fontData
                                                font.pixelSize: 9
                                                bottomPadding: 3
                                            }
                                        }
                                    }
                                }
                            }

                            // Actions
                            Column {
                                visible: seg.open && Plan.inspectorCanAct
                                width: parent.width
                                topPadding: 8
                                spacing: 0
                                Item {
                                    width: parent.width
                                    height: 22
                                    Text {
                                        anchors.verticalCenter: parent.verticalCenter
                                        text: qsTr("ACTIONS")
                                        color: Theme.accentCyan
                                        font.family: Theme.fontUi; font.pointSize: 7.125; font.weight: Font.Bold; font.letterSpacing: 1.5
                                    }
                                    CfgSelect {
                                        anchors.right: parent.right
                                        anchors.verticalCenter: parent.verticalCenter
                                        width: 120
                                        height: 19
                                        fontSize: 10
                                        options: Plan.addActionOptions
                                        value: ""
                                        onPicked: (v) => { if (v.length) Plan.addAction(seg.segId, v); currentIndex = 0 }
                                    }
                                }
                                Item { width: 1; height: 4 }
                                Repeater {
                                    model: seg.open ? Plan.inspectorActions : []
                                    delegate: Rectangle {
                                        id: act
                                        required property var modelData
                                        width: parent.width
                                        height: actCol.implicitHeight + 8 + 3
                                        color: "transparent"
                                        Rectangle {
                                            width: parent.width
                                            height: actCol.implicitHeight + 8
                                            radius: 2
                                            color: Qt.rgba(0, 0, 0, 0.22)
                                            border.color: Theme.borderSubtle
                                            border.width: Theme.hairline
                                            opacity: act.modelData.unsupported ? 0.5 : 1
                                            Column {
                                                id: actCol
                                                x: 6; y: 4
                                                width: parent.width - 12
                                                Item {
                                                    width: parent.width
                                                    height: 18
                                                    Text {
                                                        anchors.verticalCenter: parent.verticalCenter
                                                        text: Theme.tr(act.modelData.label)
                                                        color: Theme.textMutedBright
                                                        font.family: Theme.fontUi; font.pointSize: 7.875; font.weight: Font.DemiBold
                                                    }
                                                    Text {
                                                        anchors.right: actDel.left
                                                        anchors.rightMargin: 6
                                                        anchors.verticalCenter: parent.verticalCenter
                                                        text: Theme.tr(act.modelData.summary)
                                                        color: Theme.textDim
                                                        font.family: Theme.fontData; font.pixelSize: 9
                                                        font.italic: act.modelData.unsupported
                                                    }
                                                    FpIconButton {
                                                        id: actDel
                                                        anchors.right: parent.right
                                                        anchors.verticalCenter: parent.verticalCenter
                                                        width: 18; height: 18
                                                        glyph: "×"; glyphSize: 13
                                                        tip: "Remove action"
                                                        onClicked: Plan.removeAction(seg.segId, act.modelData.idx)
                                                    }
                                                }
                                                Column {
                                                    width: parent.width
                                                    topPadding: act.modelData.fields.length ? 3 : 0
                                                    spacing: 3
                                                    Repeater {
                                                        model: act.modelData.fields
                                                        delegate: FpField {
                                                            required property var modelData
                                                            field: modelData
                                                            compact: true
                                                            width: parent.width
                                                            onEdited: (key, value) => Plan.setField("action", seg.segId, act.modelData.idx, key, value)
                                                        }
                                                    }
                                                }
                                            }
                                            HoverHandler { id: actHover }
                                            ToolTip.visible: actHover.hovered && act.modelData.title.length > 0
                                            ToolTip.text: Theme.tr(act.modelData.title)
                                        }
                                    }
                                }
                            }

                            Text {
                                topPadding: 6
                                width: parent.width
                                wrapMode: Text.Wrap
                                text: seg.open ? Plan.inspectorCoords : ""
                                color: Theme.textDim
                                font.family: Theme.fontData
                                font.pointSize: 6.375
                                font.letterSpacing: 0.2
                            }
                        }
                    }
                }
            }

            // ── Popovers ──
            FpPopover {
                visible: page.popover === "route"
                title: "ROUTE SETTINGS"
                onCloseRequested: page.popover = ""
                Column {
                    x: 10
                    width: parent.width - 20
                    topPadding: 8
                    bottomPadding: 8
                    spacing: 4
                    Repeater {
                        model: page.popover === "route" ? Plan.routeForm : []
                        delegate: FpField {
                            required property var modelData
                            field: modelData
                            selectWidth: 150
                            width: parent.width
                            onEdited: (key, value) => Plan.setField("route", "", -1, key, value)
                        }
                    }
                }
                Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                Column {
                    x: 10
                    width: parent.width - 20
                    topPadding: 8
                    bottomPadding: 10
                    Text { text: qsTr("TAKE-OFF POINT"); color: Theme.accentCyan; font.family: Theme.fontUi; font.pointSize: 7.125; font.weight: Font.Bold; font.letterSpacing: 1.5 }
                    Text { topPadding: 3; bottomPadding: 4; width: parent.width; wrapMode: Text.Wrap; text: Theme.tr(Plan.homeDesc); color: Theme.textMutedBright; font.family: Theme.fontData; font.pixelSize: 10 }
                    Row {
                        topPadding: 6
                        spacing: 6
                        GcsButton { width: (parent.parent.width - 6) / 2; height: 27; fontSize: 10.5; spacing: 0.8; text: "VEHICLE"; tip: "Use the vehicle's current position"; onClicked: Plan.setHomeVehicle() }
                        GcsButton { width: (parent.parent.width - 6) / 2; height: 27; fontSize: 10.5; spacing: 0.8; text: "AUTO"; tip: "Follow the vehicle's home position"; onClicked: Plan.resetHome() }
                    }
                    Text { topPadding: 6; width: parent.width; wrapMode: Text.Wrap; text: qsTr("Drag the H marker on the map to move it"); color: Theme.textDim; font.family: Theme.fontUi; font.pointSize: 7.125 }
                }
            }

            FpPopover {
                visible: page.popover === "camera"
                title: "CAMERA & GIMBAL"
                onCloseRequested: page.popover = ""
                Column {
                    x: 10
                    width: parent.width - 20
                    topPadding: 8
                    bottomPadding: 8
                    spacing: 4
                    Repeater {
                        model: page.popover === "camera" ? Plan.cameraForm : []
                        delegate: FpField {
                            required property var modelData
                            field: modelData
                            selectWidth: 170
                            width: parent.width
                            onEdited: (key, value) => Plan.setField("camera", "", -1, key, value)
                        }
                    }
                    Text { topPadding: 8; text: qsTr("GIMBAL"); color: Theme.accentCyan; font.family: Theme.fontUi; font.pointSize: 7.125; font.weight: Font.Bold; font.letterSpacing: 1.5 }
                    Repeater {
                        model: page.popover === "camera" ? Plan.gimbalForm : []
                        delegate: FpField {
                            required property var modelData
                            field: modelData
                            width: parent.width
                            onEdited: (key, value) => Plan.setField("gimbal", "", -1, key, value)
                        }
                    }
                }
                Rectangle {
                    x: 10
                    width: parent.width - 20
                    height: camFlow.implicitHeight + 10
                    radius: 2
                    color: Qt.rgba(0, 0, 0, 0.25)
                    Flow {
                        id: camFlow
                        x: 8; y: 5
                        width: parent.width - 16
                        spacing: 10
                        Repeater {
                            model: Plan.cameraDerived
                            delegate: Text { required property string modelData; textFormat: Text.StyledText; text: Theme.tr(page.boldify(modelData)); color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 9; bottomPadding: 3 }
                        }
                    }
                }
                Item { width: 1; height: 8 }
                Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                Text {
                    x: 10
                    width: parent.width - 20
                    topPadding: 14
                    bottomPadding: 10
                    wrapMode: Text.Wrap
                    text: qsTr("A red dot on the route marks every photo (trigger-by-distance along survey lanes, single shots). Hover a dot to see the ground footprint at the planned height, heading and camera pitch. While a POI is active, hover a waypoint to see the wedge the camera is looking through.")
                    color: Theme.textDim
                    font.family: Theme.fontUi
                    font.pointSize: 7.125
                }
            }

            FpPopover {
                visible: page.popover === "radio"
                title: "RADIO LINK"
                onCloseRequested: page.popover = ""
                Column {
                    x: 10
                    width: parent.width - 20
                    topPadding: 8
                    bottomPadding: 8
                    Text { text: qsTr("OPERATOR POSITION"); color: Theme.accentCyan; font.family: Theme.fontUi; font.pointSize: 7.125; font.weight: Font.Bold; font.letterSpacing: 1.5 }
                    Text { topPadding: 3; bottomPadding: 4; width: parent.width; wrapMode: Text.Wrap; text: Theme.tr(Plan.operatorDesc); color: Theme.textMutedBright; font.family: Theme.fontData; font.pixelSize: 10 }
                    Row {
                        topPadding: 6
                        spacing: 6
                        GcsButton { width: (parent.parent.width - 6) / 2; height: 27; fontSize: 10.5; spacing: 0.8; text: "PLACE ON MAP"; tip: "Click on the map to place the ground antenna (O)"; onClicked: { page.popover = ""; Plan.placeOperator() } }
                        GcsButton { width: (parent.parent.width - 6) / 2; height: 27; fontSize: 10.5; spacing: 0.8; text: "TAKE-OFF"; enabled: Plan.operatorPlaced; tip: "Put the operator back at the take-off point"; onClicked: Plan.resetOperator() }
                    }
                    Text { topPadding: 6; width: parent.width; wrapMode: Text.Wrap; text: qsTr("Drag the antenna marker on the map to move it"); color: Theme.textDim; font.family: Theme.fontUi; font.pointSize: 7.125 }
                    Item { width: 1; height: 8 }
                    FpField {
                        width: parent.width
                        field: ({ key: "link", label: "Coverage overlay on the map", type: "check", checked: Plan.showLink })
                        onEdited: (key, value) => Plan.showLink = value
                    }
                }
                Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                Column {
                    x: 10
                    width: parent.width - 20
                    topPadding: 8
                    bottomPadding: 8
                    spacing: 4
                    Repeater {
                        model: page.popover === "radio" ? Plan.radioForm : []
                        delegate: FpField {
                            required property var modelData
                            field: modelData
                            selectWidth: 190
                            width: parent.width
                            onEdited: (key, value) => Plan.setField("radio", "", -1, key, value)
                        }
                    }
                }
                Rectangle {
                    x: 10
                    width: parent.width - 20
                    height: radioFlow.implicitHeight + 10
                    radius: 2
                    color: Qt.rgba(0, 0, 0, 0.25)
                    Flow {
                        id: radioFlow
                        x: 8; y: 5
                        width: parent.width - 16
                        spacing: 10
                        Repeater {
                            model: Plan.radioDerived
                            delegate: Text { required property string modelData; textFormat: Text.StyledText; text: Theme.tr(page.boldify(modelData)); color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 9; bottomPadding: 3 }
                        }
                    }
                }
                Item { width: 1; height: 8 }
                Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                Column {
                    x: 10
                    width: parent.width - 20
                    topPadding: 8
                    bottomPadding: 10
                    Flow {
                        width: parent.width
                        spacing: 10
                        topPadding: 6
                        Repeater {
                            model: [["good link", Qt.rgba(96 / 255, 1, 128 / 255, 0.5)], ["degraded", Qt.rgba(1, 170 / 255, 0, 0.6)],
                                    ["below margin", Qt.rgba(1, 60 / 255, 60 / 255, 0.6)], ["no link", Qt.rgba(154 / 255, 164 / 255, 176 / 255, 0.6)]]
                            delegate: Row {
                                required property var modelData
                                spacing: 5
                                Rectangle { width: 14; height: 8; radius: 1; anchors.verticalCenter: parent.verticalCenter; color: parent.modelData[1]; border.color: Qt.rgba(1, 1, 1, 0.15); border.width: Theme.hairline }
                                Text { text: Theme.tr(parent.modelData[0]); color: Theme.textMuted; font.family: Theme.fontUi; font.pointSize: 7.125 }
                            }
                        }
                    }
                    Text {
                        topPadding: 6
                        width: parent.width
                        wrapMode: Text.Wrap
                        text: qsTr("Line of sight and first Fresnel zone are checked against the elevation model between the ground antenna and the aircraft. Green: clear zone and margin above the safety budget. Orange: the terrain intrudes into the Fresnel zone or blocks the line of sight (diffraction path). Red: link possible but below the safety budget. The map overlay is computed at the route default altitude; the route band and the profile use the planned altitude of every point. Preset figures are typical datasheet values — verify them against your radio and air rate.")
                        color: Theme.textDim
                        font.family: Theme.fontUi
                        font.pointSize: 7.125
                    }
                }
            }

            // Route menu
            Rectangle {
                id: menu
                visible: page.menuOpen
                z: 31
                x: side.width - 1 - 10 - width
                y: 40
                width: 180
                height: menuCol.implicitHeight + 8
                radius: 4
                color: Qt.rgba(4 / 255, 8 / 255, 14 / 255, 0.98)
                border.color: Theme.borderAccentMid
                border.width: Theme.hairline
                Column {
                    id: menuCol
                    y: 4
                    width: parent.width
                    Repeater {
                        model: [["new", "New route"], ["invert", "Invert direction"], ["flatten", "Convert to waypoints"], ["fit", "Zoom to route"], ["-"],
                                ["import", "Import .waypoints…"], ["export", "Export .waypoints"], ["-"], ["clear", "Clear route", true]]
                        delegate: Item {
                            required property var modelData
                            width: menuCol.width
                            height: modelData[0] === "-" ? 9 : 28
                            Rectangle { visible: parent.modelData[0] === "-"; y: 4; width: parent.width; height: Theme.hairline; color: Theme.borderSubtle }
                            Rectangle {
                                visible: parent.modelData[0] !== "-"
                                anchors.fill: parent
                                color: mh.hovered ? (parent.modelData[2] ? Qt.rgba(1, 60 / 255, 60 / 255, 0.12) : Qt.rgba(0, 210 / 255, 1, 0.12)) : "transparent"
                                Text {
                                    x: 14
                                    anchors.verticalCenter: parent.verticalCenter
                                    text: parent.parent.modelData[1] || ""
                                    color: parent.parent.modelData[2] ? (mh.hovered ? "#ff9090" : "#ff7070") : (mh.hovered ? Theme.accentCyan : Theme.textMain)
                                    font.family: Theme.fontUi; font.pointSize: 8.625; font.weight: Font.DemiBold; font.letterSpacing: 0.5
                                }
                                HoverHandler { id: mh; cursorShape: Qt.PointingHandCursor }
                                TapHandler {
                                    onTapped: {
                                        page.menuOpen = false
                                        const act = parent.parent.modelData[0]
                                        if (act === "import") importDialog.open()
                                        else Plan.menuAction(act)
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // ── Map ──
        Item {
            id: mapWrap
            x: side.width
            width: parent.width - side.width
            height: parent.height
            clip: true

            PlanMapItem {
                id: map
                objectName: "planMap"
                anchors.fill: parent
                onContextMenu: (x, y, items, lat, lng, segId, pointIdx) => ctxMenu.show(x, y, items, lat, lng, segId, pointIdx)
                onClickedAway: { ctxMenu.visible = false; page.menuOpen = false }
                HoverHandler { id: mapHoverZ }
                ToolTip.visible: map.zoneTip.length > 0 && mapHoverZ.hovered
                ToolTip.text: Theme.tr(map.zoneTip)
                ToolTip.delay: 300
            }
            // UAS zones key while they are on (GeoZoneLayer.js legend)
            GeoZoneLegend {
                visible: GeoZones.enabled && GeoZones.inReach > 0
                anchors.left: parent.left
                anchors.bottom: parent.bottom
                anchors.margins: 10
                z: 5
            }

            // Tool palette
            Rectangle {
                id: tools
                x: 10
                y: 10
                width: 35
                height: toolCol.implicitHeight + 2
                radius: 4
                color: Qt.rgba(5 / 255, 12 / 255, 20 / 255, 0.95)
                border.color: Theme.borderAccent
                border.width: Theme.hairline
                clip: true
                Column {
                    id: toolCol
                    x: 0.67
                    y: 0.67
                    Repeater {
                        model: Plan.tools
                        delegate: Rectangle {
                            id: tb
                            required property var modelData
                            required property int index
                            readonly property bool active: Plan.tool === modelData.id
                            width: 34
                            height: 34
                            color: active ? Qt.rgba(0, 210 / 255, 1, 0.22) : th.hovered && !modelData.disabled ? Qt.rgba(0, 210 / 255, 1, 0.12) : "transparent"
                            opacity: modelData.disabled ? 0.28 : 1
                            Rectangle { visible: tb.active; width: 3; height: parent.height; color: Theme.accentCyan }
                            Rectangle { visible: tb.modelData.aux; width: parent.width; height: Theme.hairline; color: Theme.borderAccent }
                            Rectangle {
                                visible: tb.index < Plan.tools.length - 1 && !Plan.tools[tb.index + 1].aux
                                anchors.bottom: parent.bottom
                                width: parent.width
                                height: Theme.hairline
                                color: tb.index === 0 ? Theme.borderAccent : Theme.borderSubtle
                            }
                            SvgIcon {
                                anchors.centerIn: parent
                                width: 18; height: 18
                                parts: page.icon[tb.modelData.id] || []
                                color: tb.active ? Theme.accentCyan : th.hovered && !tb.modelData.disabled ? "#ffffff" : "#aaaabb"
                            }
                            HoverHandler { id: th; cursorShape: tb.modelData.disabled ? Qt.ForbiddenCursor : Qt.PointingHandCursor }
                            TapHandler { gesturePolicy: TapHandler.ReleaseWithinBounds; onTapped: Plan.setTool(tb.modelData.id) }
                            ToolTip.visible: th.hovered
                            ToolTip.text: Theme.tr(tb.modelData.title)
                            ToolTip.delay: 500
                        }
                    }
                }
            }

            // Drawing hint
            Rectangle {
                visible: Plan.hint.length > 0
                x: 56
                y: 12
                width: hintText.implicitWidth + 20
                height: hintText.implicitHeight + 10
                radius: 3
                color: Qt.rgba(5 / 255, 12 / 255, 20 / 255, 0.95)
                border.color: Theme.borderAccentMid
                border.width: Theme.hairline
                Text {
                    id: hintText
                    anchors.centerIn: parent
                    text: Theme.tr(Plan.hint)
                    color: Theme.accentCyan
                    font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.DemiBold; font.letterSpacing: 0.5
                }
            }

            // Layers: SAT WP CAM LINK FIT
            Rectangle {
                x: mapWrap.width - 52 - width
                y: 10
                width: layerRow.implicitWidth + 2
                height: 30
                radius: 4
                color: Qt.rgba(5 / 255, 12 / 255, 20 / 255, 0.95)
                border.color: Theme.borderAccent
                border.width: Theme.hairline
                clip: true
                Row {
                    id: layerRow
                    x: 1
                    y: 1
                    Repeater {
                        model: [["SAT", "Satellite imagery"], ["WP", "Show calculated waypoints"], ["CAM", "Show camera footprints and POI view cones"],
                                ["LINK", "Radio link coverage from the operator position"], ["FIT", "Zoom to route"]]
                        delegate: Rectangle {
                            id: lb
                            required property var modelData
                            required property int index
                            readonly property bool active: index === 0 ? Plan.showSat : index === 1 ? Plan.showCalc : index === 2 ? Plan.showCam : index === 3 ? Plan.showLink : false
                            width: lbText.implicitWidth + 20
                            height: 28
                            color: active ? Qt.rgba(0, 210 / 255, 1, 0.16) : lh.hovered ? Qt.rgba(0, 210 / 255, 1, 0.1) : "transparent"
                            Rectangle { visible: lb.index < 4; anchors.right: parent.right; width: Theme.hairline; height: parent.height; color: Theme.borderSubtle }
                            Text {
                                id: lbText
                                anchors.centerIn: parent
                                text: Theme.tr(lb.modelData[0])
                                color: lb.active ? Theme.accentCyan : lh.hovered ? Theme.textMain : Theme.textMuted
                                font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1
                            }
                            SequentialAnimation on opacity {
                                running: lb.index === 3 && Plan.linkBusy
                                loops: Animation.Infinite
                                NumberAnimation { to: 0.25; duration: 400 }
                                NumberAnimation { to: 1; duration: 400 }
                                onStopped: lb.opacity = 1
                            }
                            HoverHandler { id: lh; cursorShape: Qt.PointingHandCursor }
                            TapHandler {
                                gesturePolicy: TapHandler.ReleaseWithinBounds
                                onTapped: {
                                    if (lb.index === 0) Plan.showSat = !Plan.showSat
                                    else if (lb.index === 1) Plan.showCalc = !Plan.showCalc
                                    else if (lb.index === 2) Plan.showCam = !Plan.showCam
                                    else if (lb.index === 3) Plan.showLink = !Plan.showLink
                                    else map.fitRoute()
                                }
                            }
                            ToolTip.visible: lh.hovered
                            ToolTip.text: Theme.tr(lb.modelData[1])
                            ToolTip.delay: 600
                        }
                    }
                }
            }

            // Zoom control (Leaflet's, restyled by vendors.css)
            Rectangle {
                x: mapWrap.width - 10 - width
                y: 10
                width: 32
                height: 63
                radius: 4
                color: "transparent"
                border.color: Qt.rgba(0, 210 / 255, 1, 0.6)
                border.width: Theme.hairline
                clip: true
                Column {
                    x: 1; y: 1
                    Repeater {
                        model: ["+", "−"]
                        delegate: Rectangle {
                            id: zb
                            required property string modelData
                            required property int index
                            width: 30
                            height: 30.5
                            color: zh.hovered ? Qt.rgba(0, 0, 0, 0.95) : Qt.rgba(0, 0, 0, 0.9)
                            Rectangle { visible: zb.index === 0; anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Qt.rgba(0, 210 / 255, 1, 0.35) }
                            Text {
                                anchors.centerIn: parent
                                text: zb.modelData
                                color: zh.hovered ? Theme.accentCyan : Theme.textMain
                                font.family: "Lucida Console"
                                font.pixelSize: 18
                                font.bold: true
                            }
                            HoverHandler { id: zh; cursorShape: Qt.PointingHandCursor }
                            TapHandler { gesturePolicy: TapHandler.ReleaseWithinBounds; onTapped: zb.index === 0 ? map.zoomIn() : map.zoomOut() }
                        }
                    }
                }
            }

            // Context menu
            Rectangle {
                id: ctxMenu
                visible: false
                z: 50
                property real lat: 0
                property real lng: 0
                property string segId: ""
                property int pointIdx: -1
                property var items: []
                function show(px, py, list, la, ln, sid, pi) {
                    items = list; lat = la; lng = ln; segId = sid; pointIdx = pi
                    x = Math.min(px, mapWrap.width - width - 4)
                    y = Math.min(py, mapWrap.height - height - 4)
                    visible = true
                }
                width: 170
                height: ctxCol.implicitHeight + 8
                radius: 4
                color: Qt.rgba(4 / 255, 8 / 255, 14 / 255, 0.98)
                border.color: Theme.borderAccentMid
                border.width: Theme.hairline
                Column {
                    id: ctxCol
                    y: 4
                    width: parent.width
                    Repeater {
                        model: ctxMenu.items
                        delegate: Rectangle {
                            required property var modelData
                            width: ctxCol.width
                            height: 28
                            color: ch.hovered ? (modelData[2] ? Qt.rgba(1, 60 / 255, 60 / 255, 0.12) : Qt.rgba(0, 210 / 255, 1, 0.12)) : "transparent"
                            Text {
                                x: 14
                                anchors.verticalCenter: parent.verticalCenter
                                text: Theme.tr(parent.modelData[1])
                                color: parent.modelData[2] ? (ch.hovered ? "#ff9090" : "#ff7070") : (ch.hovered ? Theme.accentCyan : Theme.textMain)
                                font.family: Theme.fontUi; font.pointSize: 8.625; font.weight: Font.DemiBold; font.letterSpacing: 0.5
                            }
                            HoverHandler { id: ch; cursorShape: Qt.PointingHandCursor }
                            TapHandler {
                                gesturePolicy: TapHandler.ReleaseWithinBounds
                                onTapped: {
                                    ctxMenu.visible = false
                                    Plan.contextAction(parent.modelData[0], ctxMenu.lat, ctxMenu.lng, ctxMenu.segId, ctxMenu.pointIdx)
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ── Elevation profile ──
    Rectangle {
        id: profileBox
        y: layout.height
        width: parent.width
        height: 177
        color: Qt.rgba(5 / 255, 12 / 255, 20 / 255, 0.95)
        Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderAccent }
        ProfileItem {
            id: profile
            y: 1
            width: parent.width
            height: parent.height - 1
        }
        Rectangle {
            visible: profile.tipVisible
            x: profile.tipX
            y: profile.tipY + 1
            width: tipText.implicitWidth + 16
            height: tipText.implicitHeight + 8
            radius: 3
            color: Qt.rgba(4 / 255, 8 / 255, 14 / 255, 0.98)
            border.color: Theme.borderAccentMid
            border.width: Theme.hairline
            Text {
                id: tipText
                x: 8; y: 4
                textFormat: Text.StyledText
                text: Theme.tr(profile.tipText.replace(/<b>/g, "<font color=\"#ffffff\">").replace(/<\/b>/g, "</font>"))
                color: Theme.textMutedBright
                font.family: Theme.fontData
                font.pointSize: 7.125
                lineHeight: 1.5
            }
        }
    }

    FileDialog {
        id: importDialog
        title: "Import route"
        nameFilters: ["Waypoint files (*.waypoints *.txt *.json)", "All files (*)"]
        onAccepted: Plan.importFile(selectedFile)
    }
    FileDialog {
        id: exportDialog
        title: "Export .waypoints"
        fileMode: FileDialog.SaveFile
        nameFilters: ["Waypoint files (*.waypoints)", "All files (*)"]
        onAccepted: Plan.exportFile(selectedFile)
    }
}
