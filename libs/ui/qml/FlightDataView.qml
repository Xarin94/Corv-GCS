// FLIGHT DATA tab (#hud-wrapper and #ui-layer): 3D scene, HUD canvas, the
// data tapes on both sides, the DEMO / DISARMED banner, the mini-map, the
// position box, the EKF STATUS / FFT tabs on the left edge and the GCS
// OPTIONS tab with its sidebar on the right edge.
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui
import Corv.Gpu

Item {
    id: view
    property alias scene: scene
    property alias hud: hud
    property bool sidebarOpen: false
    // The 2D map is the main view and the 3D the corner panel (body.minimap-swapped)
    property bool swapped: false
    // TERR ALT flashes while a UAS zone ceiling or the general height limit is broken
    property bool zoneBlink: false
    Timer {
        interval: 400
        repeat: true
        running: GeoZones.overHeight
        onTriggered: view.zoneBlink = !view.zoneBlink
        onRunningChanged: if (!running) view.zoneBlink = false
    }
    // Height of the window, for the edge tabs centred on it
    property real windowHeight: 900
    property real windowTop: 0
    // EKF STATUS or FFT open: the flight view moves over by the bar's width
    readonly property bool leftBarOpen: Ekf.enabled || Fft.enabled
    onVisibleChanged: { Ekf.shown = visible; Fft.shown = visible }
    Component.onCompleted: { Ekf.shown = visible; Fft.shown = visible }

    // The HUD as a plane at the camera's focal distance, rotated about the eye by
    // the inverse of the look offset: perspective(f) translateZ(f) rotateX(pitch)
    // rotateY(yaw) translateZ(-f) about the centre, as the CSS transform of
    // applyHudLookTransform; z flattened so nothing is clipped
    function hudLookMatrix(yaw, pitch, w, h, fovDeg) {
        if (yaw === 0 && pitch === 0) return Qt.matrix4x4()
        const f = (h / 2) / Math.tan((fovDeg || 60) * Math.PI / 360)
        const ay = -yaw, ax = pitch
        const cy = Math.cos(ay), sy = Math.sin(ay), cx = Math.cos(ax), sx = Math.sin(ax)
        const toCentre = Qt.matrix4x4(1, 0, 0, w / 2, 0, 1, 0, h / 2, 0, 0, 1, 0, 0, 0, 0, 1)
        const persp = Qt.matrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -1 / f, 1)
        const push = Qt.matrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, f, 0, 0, 0, 1)
        const rotX = Qt.matrix4x4(1, 0, 0, 0, 0, cx, -sx, 0, 0, sx, cx, 0, 0, 0, 0, 1)
        const rotY = Qt.matrix4x4(cy, 0, sy, 0, 0, 1, 0, 0, -sy, 0, cy, 0, 0, 0, 0, 1)
        const pull = Qt.matrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -f, 0, 0, 0, 1)
        const fromCentre = Qt.matrix4x4(1, 0, 0, -w / 2, 0, 1, 0, -h / 2, 0, 0, 1, 0, 0, 0, 0, 1)
        const m = toCentre.times(persp).times(push).times(rotX).times(rotY).times(pull).times(fromCentre)
        return Qt.matrix4x4(m.m11, m.m12, m.m13, m.m14, m.m21, m.m22, m.m23, m.m24, 0, 0, 0, 0, m.m41, m.m42, m.m43, m.m44)
    }

    // Single-key toggles of the 3D view, on the flight screen only (initViewShortcuts):
    // T horizon lock, P predicted trajectory, M satellite / schematic, L sunlight,
    // C first / third person
    Shortcut { sequence: "T"; enabled: view.visible; onActivated: hud.pitchLocked = !hud.pitchLocked }
    Shortcut { sequence: "P"; enabled: view.visible; onActivated: scene.trajectory = !scene.trajectory }
    Shortcut { sequence: "M"; enabled: view.visible; onActivated: scene.schematic = !scene.schematic }
    Shortcut { sequence: "L"; enabled: view.visible; onActivated: scene.sunlight = !scene.sunlight }
    Shortcut { sequence: "C"; enabled: view.visible; onActivated: scene.thirdPerson = !scene.thirdPerson }

    // A HUD data cell as SYS CONFIG → HUD DATA FIELDS sets it (HUD_FIELD_DEFS of UIController.js)
    function cell(i) {
        const c = SysConfig.hudCells[i]
        if (!c) return { label: "", value: "---", unit: "", color: "" }
        const m = c.multiplier || 1
        let v = "---", col = ""
        switch (c.field) {
        case "as": v = (Vehicle.airspeed * m).toFixed(1); break
        case "gs": v = (Vehicle.groundspeed * m).toFixed(1); break
        case "vs": v = (Vehicle.climb * m).toFixed(1); break
        case "gload": {
            const g = Vehicle.gLoad
            v = (g * m).toFixed(1)
            col = g >= -1 && g <= 4 ? "#00ff7f" : g >= -2 && g <= 6 ? "#ffcc00" : "#ff3333"
            break
        }
        case "alt": v = String(Math.round((Vehicle.altMsl + Vehicle.altOffset) * m)); break
        case "agl":
            v = isNaN(Vehicle.terrainAgl) ? "---" : String(Math.round(Vehicle.terrainAgl * m))
            if (GeoZones.overHeight) col = view.zoneBlink ? "#ff3333" : "#ffcc00"
            break
        case "lidar": {
            const r = Vehicle.rangefinder
            if (!isNaN(r) && r > 0) {
                v = (r * m).toFixed(2)
                col = r < 1 ? "#ff3333" : r <= 10 ? "#ff8800" : ""
            }
            break
        }
        case "roll": v = (Vehicle.rollDeg * m).toFixed(1); break
        case "pitch": v = (Vehicle.pitchDeg * m).toFixed(1); break
        case "yaw": v = ((((Vehicle.headingDeg + 360) % 360) * m)).toFixed(0).padStart(3, "0"); break
        case "battV": v = (Vehicle.batteryVoltage * m).toFixed(1); break
        case "battA": v = (Vehicle.batteryCurrent * m).toFixed(1); break
        case "battPct": v = Vehicle.batteryRemaining < 0 ? "---" : String(Math.round(Vehicle.batteryRemaining * m)); break
        case "gpsSat": v = String(Vehicle.gpsNumSat * m); break
        case "gpsHdop": v = (Vehicle.gpsHdop * m).toFixed(1); break
        case "linkQ": v = String(Math.round(Vehicle.linkQuality * m)); break
        case "vibX": v = (Vehicle.vibX * m).toFixed(1); break
        case "vibY": v = (Vehicle.vibY * m).toFixed(1); break
        case "vibZ": v = (Vehicle.vibZ * m).toFixed(1); break
        case "aoa": v = (Vehicle.aoaDeg * m).toFixed(1); break
        case "ssa": v = (Vehicle.ssaDeg * m).toFixed(1); break
        case "rtkBaseline": v = (Vehicle.rtkBaseline * m).toFixed(0); break
        case "rtkAccuracy": v = (Vehicle.rtkAccuracy * m).toFixed(0); break
        }
        return { label: c.label, value: v, unit: c.unit, color: col }
    }

    // A row of the ADS-B traffic table (#traffic-bar .stat-item): callsign on
    // half the width, altitude and distance on a quarter each, right-aligned
    component TfcRow: Item {
        id: tr
        property var cells: ["", "", ""]
        property bool header: false
        property bool first: false
        property bool empty: false
        property bool last: false
        property real rowPad: 3
        height: rowText.implicitHeight + (last ? 0 : rowPad)
        Row {
            id: rowText
            width: parent.width
            Repeater {
                model: 3
                delegate: Text {
                    required property int index
                    width: tr.width * (index === 0 ? 0.5 : 0.25)
                    horizontalAlignment: index === 0 ? Text.AlignLeft : Text.AlignRight
                    text: tr.cells[index]
                    elide: Text.ElideRight
                    font.family: tr.header ? Theme.fontUi : Theme.fontData
                    font.pixelSize: Theme.fsXs
                    font.letterSpacing: tr.header ? 0.09 * Theme.fsXs : 0
                    color: tr.header || tr.empty ? Theme.textDim : tr.first ? Theme.textMain : Theme.textMutedBright
                    opacity: tr.empty ? 0.5 : 1
                }
            }
        }
        Rectangle {
            visible: !tr.last
            anchors.bottom: parent.bottom
            width: parent.width
            height: 1
            color: tr.header ? Theme.borderAccentMid : Theme.borderSubtle
        }
    }

    Item {
        id: flight
        x: view.leftBarOpen ? Theme.ekfBarW : 0
        width: view.width - x - (view.sidebarOpen ? side.width : 0)
        height: view.height
        clip: true

        // The 3D view and the 2D map share the screen as in Electron's #scene-container
        // and #mini-map-container: one fills the view, the other sits in the corner
        // panel (bottom-left, larger under the mouse). A click on the mini-map makes
        // it the main view; a click on the small 3D view brings the 3D back.
        readonly property bool smallHovered: view.swapped ? sceneHover.hovered : mapHover.hovered
        // --minimap-hover-w/h: a fifth of the window's area, at the panel's 260:180 aspect
        readonly property real hoverH: Math.round(Math.sqrt(Theme.winW * Theme.winH / (5 * 260 / 180)))
        readonly property real hoverW: Math.round(hoverH * 260 / 180)

        Rectangle {
            id: miniFrame
            x: Theme.gutter
            width: flight.smallHovered ? flight.hoverW : Theme.minimapW
            height: flight.smallHovered ? flight.hoverH : Theme.minimapH
            y: flight.height - Theme.gutter - height
            z: 5
            radius: 4
            color: Qt.rgba(0, 0, 0, 0.5)
            Behavior on width { NumberAnimation { duration: 300; easing.type: Easing.InOutQuad } }
            Behavior on height { NumberAnimation { duration: 300; easing.type: Easing.InOutQuad } }
        }

        SceneItem {
            id: scene
            objectName: "scene"
            x: view.swapped ? miniFrame.x + 1 : 0
            y: view.swapped ? miniFrame.y + 1 : 0
            width: view.swapped ? miniFrame.width - 2 : flight.width
            height: view.swapped ? miniFrame.height - 2 : flight.height
            z: view.swapped ? 6 : 0
            fovDeg: hud.fovDeg
            terrainRadius: SysConfig.terrainRadius
            satelliteRadius: SysConfig.satelliteRadius
            horizonLock: hud.pitchLocked
            // The arrow keys turn the view where it is the main one
            lookKeys: !view.swapped
            onClicked: if (view.swapped) view.swapped = false
            HoverHandler { id: sceneHover; acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad | PointerDevice.Stylus }
            // The switches of the header as they were left (view-state): satellite or
            // schematic, sunlight, 1st / 3rd person, horizon lock, trajectory, theme
            property bool restored: false
            Component.onCompleted: {
                let v = {}
                try { v = JSON.parse(Gcs.setting("view-state", "{}")) || {} } catch (e) { }
                if (v.satellite !== undefined) schematic = !v.satellite
                if (v.sunlight !== undefined) sunlight = !!v.sunlight
                if (v.thirdPerson !== undefined) thirdPerson = !!v.thirdPerson
                if (v.trajectory !== undefined) trajectory = !!v.trajectory
                if (v.lightTheme !== undefined) lightTheme = !!v.lightTheme
                if (v.horizonLock !== undefined) hud.pitchLocked = !!v.horizonLock
                restored = true
            }
            function saveView() {
                if (!restored) return
                Gcs.setSetting("view-state", JSON.stringify({ satellite: !schematic, sunlight: sunlight, thirdPerson: thirdPerson,
                                                              horizonLock: hud.pitchLocked, trajectory: trajectory, lightTheme: lightTheme }))
            }
            onSettingsChanged: saveView()
            Connections { target: hud; function onPitchLockedChanged() { scene.saveView() } }
        }
        MapItem {
            id: miniMap
            x: view.swapped ? 0 : miniFrame.x + 1
            y: view.swapped ? 0 : miniFrame.y + 1
            width: view.swapped ? flight.width : miniFrame.width - 2
            height: view.swapped ? flight.height : miniFrame.height - 2
            z: view.swapped ? 0 : 6
            objectName: "miniMap"
            // In the corner the map stays on the vehicle; as the main view it can be moved
            panEnabled: view.swapped
            onClicked: if (!view.swapped) view.swapped = true
            // (a finger is no hover: the panel would grow under it)
            HoverHandler {
                id: mapHover
                acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad | PointerDevice.Stylus
                cursorShape: view.swapped ? Qt.ArrowCursor : Qt.PointingHandCursor
            }
        }
        // + / - / centre of the 2D map: beside the corner panel while the map is the main
        // view, small inside it under the mouse
        Column {
            id: mapZoom
            objectName: "mapZoom"
            readonly property real side: view.swapped ? 36 : 22
            visible: view.swapped || mapHover.hovered
            x: view.swapped ? miniFrame.x + miniFrame.width + Theme.gutter : miniFrame.x + miniFrame.width - side - 6
            y: view.swapped ? miniFrame.y + miniFrame.height - height : miniFrame.y + 6
            z: 8
            spacing: 2
            Repeater {
                model: ["+", "−", "◎"]
                delegate: Rectangle {
                    id: mz
                    required property string modelData
                    required property int index
                    readonly property bool usable: index < 2 || !miniMap.follow
                    visible: index < 2 || view.swapped
                    width: mapZoom.side
                    height: mapZoom.side
                    radius: 4
                    opacity: usable ? 1 : 0.4
                    color: mzHover.hovered && usable ? Qt.rgba(0, 0, 0, 0.95) : Qt.rgba(0, 0, 0, 0.75)
                    border.color: mzHover.hovered && usable ? Theme.accentCyan : Theme.borderAccentMid
                    border.width: Theme.hairline
                    Text {
                        anchors.centerIn: parent
                        text: mz.modelData
                        color: mzHover.hovered && mz.usable ? Theme.accentCyan : Theme.textMain
                        font.family: mz.index < 2 ? "Lucida Console" : Theme.fontUi
                        font.pixelSize: mapZoom.side * 0.55
                        font.bold: true
                    }
                    HoverHandler { id: mzHover; cursorShape: Qt.PointingHandCursor }
                    // The press stays here: under it the map would move or come to the front
                    TapHandler {
                        gesturePolicy: TapHandler.ReleaseWithinBounds
                        onTapped: {
                            if (mz.index === 0) miniMap.zoomIn()
                            else if (mz.index === 1) miniMap.zoomOut()
                            else miniMap.recentre()
                        }
                    }
                    ToolTip.visible: mzHover.hovered && mz.index === 2
                    ToolTip.text: "Centre on the vehicle"
                    ToolTip.delay: 500
                }
            }
        }
        Rectangle {
            // The corner panel's border, over whichever view is in it
            x: miniFrame.x
            y: miniFrame.y
            width: miniFrame.width
            height: miniFrame.height
            z: 7
            radius: 4
            color: "transparent"
            border.color: Theme.borderAccentMid
            border.width: Theme.hairline
        }

        HudItem {
            id: hud
            anchors.fill: parent
            z: 1
            // Third person shows the model, not the glass (setCameraMode)
            visible: !scene.thirdPerson
            messagesBottom: miniFrame.height + Theme.gutter + 28
            lightScene: scene.schematic && scene.lightTheme
            // The HUD is the glass in front of the nose: when the arrow keys turn the
            // view it slides out of frame with the scene (applyHudLookTransform)
            readonly property real lookAngle: Math.acos(Math.max(-1, Math.min(1, Math.cos(scene.lookYaw) * Math.cos(scene.lookPitch))))
            opacity: 1 - Math.max(0, Math.min(1, (lookAngle - 55 * Math.PI / 180) / (25 * Math.PI / 180)))
            transform: Matrix4x4 { matrix: view.hudLookMatrix(scene.lookYaw, scene.lookPitch, hud.width, hud.height, hud.fovDeg) }
        }

        // ── UI layer: the grid of index.html, same padding and columns ──
        Item {
            id: ui
            anchors.fill: parent
            z: 2
            readonly property real padL: Theme.gutter
            readonly property real padR: Theme.gutter + 26
            readonly property real tapeW: Math.max(120, 0.15 * (width - padL - padR))
            readonly property real tapeY: Theme.chromeTop + Theme.gutter - 2   // #ui-layer padding-top: 92 px + gutter
            readonly property real gap: Theme.clamp(6, 1.4 * Theme.vh, 20)

            Column {
                id: leftTape
                x: ui.padL
                y: ui.tapeY
                width: ui.tapeW
                spacing: ui.gap
                Repeater {
                    model: 3
                    delegate: DataCell {
                        required property int index
                        readonly property var c: view.cell(index)
                        width: leftTape.width
                        primary: index === 0
                        label: c.label
                        value: c.value
                        unit: c.unit
                        valueColor: c.color.length ? c.color : (primary ? Theme.hudText : Theme.hudTextSub)
                    }
                }
                Item {
                    width: parent.width
                    height: banner.height + Theme.clamp(6, 1.6 * Theme.vh, 22)
                    visible: !Vehicle.armed || Vehicle.demo
                    Rectangle {
                        id: banner
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.bottom: parent.bottom
                        width: bannerText.implicitWidth + 24
                        height: bannerText.implicitHeight + 8
                        radius: 3
                        color: Qt.rgba(0, 0, 0, 0.5)
                        Text {
                            id: bannerText
                            anchors.centerIn: parent
                            text: Vehicle.demo ? "DEMO" : "DISARMED"
                            color: Vehicle.demo ? Theme.hudGreen : "#ff2a2a"
                            font.family: Theme.fontData
                            font.pixelSize: Theme.fsLg
                            font.weight: Font.Bold
                            font.letterSpacing: 0.2 * Theme.fsLg
                            // demo-blink: 1.4 s, on for 60 %, at 0.15 for the rest
                            SequentialAnimation on opacity {
                                running: Vehicle.demo
                                loops: Animation.Infinite
                                PropertyAction { value: 1.0 }
                                PauseAnimation { duration: 840 }
                                PropertyAction { value: 0.15 }
                                PauseAnimation { duration: 560 }
                                onStopped: bannerText.opacity = 1
                            }
                        }
                    }
                }
            }

            Column {
                id: rightTape
                x: ui.width - ui.padR - ui.tapeW
                y: ui.tapeY
                width: ui.tapeW
                spacing: ui.gap
                Repeater {
                    model: 3
                    delegate: DataCell {
                        required property int index
                        readonly property var c: view.cell(index + 3)
                        width: rightTape.width
                        primary: index === 0
                        label: c.label
                        value: c.value
                        unit: c.unit
                        valueColor: c.color.length ? c.color : (primary ? Theme.hudText : Theme.hudTextSub)
                    }
                }
            }

            // UAS geographical zones, inboard of the altitude tape (#zone-annun)
            ZoneAnnunciators {
                x: rightTape.x - Theme.gutter - Theme.annunW
                y: ui.tapeY
                width: Theme.annunW
            }

            // Annunciators, inboard of the airspeed tape: warnings from the top,
            // cautions from the bottom just above the mini-map
            AnnunciatorColumn {
                x: ui.padL + ui.tapeW + Theme.gutter
                y: ui.tapeY
                width: Theme.annunW
                height: ui.height - y - Theme.gutter - Theme.minimapH - Theme.gutter
            }

            // .bottom-bar-group: the position box and the ADS-B traffic table side
            // by side at the bottom right, the ROTOR LOAD schematic above the
            // rightmost of the two (the traffic table when it is up). During a
            // replay the tables are hidden but keep their room, and the
            // schematic stays: it replays the logged motor outputs.
            readonly property bool trafficShown: SysConfig.adsbEnabled
            readonly property Item rightmostBar: trafficShown ? trafficBar : positionBar
            readonly property real barsTop: Math.min(positionBar.y, trafficShown ? trafficBar.y : positionBar.y)

            // ROTOR LOAD schematic and its caption (.bottom-bar-rotor), with the
            // FLAPS indicator over it while an output carries flaps or flaperons
            Column {
                id: rotorCol
                visible: rotor.shown
                spacing: Theme.clamp(2, 0.4 * Theme.vh, 6)
                x: Math.min(ui.rightmostBar.x + (ui.rightmostBar.width - width) / 2, ui.width - ui.padR - width)
                y: ui.barsTop - Theme.clamp(2, 0.5 * Theme.vh, 8) - height
                OutlinedText {
                    visible: rotor.flapShown
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "FLAPS"
                    color: Theme.hudTextLbl
                    font.family: Theme.fontUi
                    font.pixelSize: Theme.fsXs
                    font.weight: Font.DemiBold
                    font.letterSpacing: 0.18 * Theme.fsXs
                }
                FlapIndicatorItem {
                    visible: rotor.flapShown
                    anchors.horizontalCenter: parent.horizontalCenter
                    // as wide as the rotor gauges
                    width: Theme.clamp(110, 9.5 * Theme.vw, 190)
                    height: Math.round(width * 0.45)
                    percent: rotor.flapPercent
                }
                OutlinedText {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: rotor.sail ? "SAIL · RUDDER" : "ROTOR LOAD"
                    color: Theme.hudTextLbl
                    font.family: Theme.fontUi
                    font.pixelSize: Theme.fsXs
                    font.weight: Font.DemiBold
                    font.letterSpacing: 0.18 * Theme.fsXs
                }
                RotorLoadItem {
                    id: rotor
                    objectName: "rotorLoad"
                    anchors.horizontalCenter: parent.horizontalCenter
                    // #rotor-load-panel: clamp(110px, 9.5vw, 190px), wider for the sail dials
                    width: sail ? Theme.clamp(200, 17 * Theme.vw, 340) : Theme.clamp(110, 9.5 * Theme.vw, 190)
                    height: Theme.clamp(110, 9.5 * Theme.vw, 190)
                }
            }

            // Position box, bottom-right
            Rectangle {
                id: positionBar
                // hidden under the replay timeline while a log is loaded
                visible: !Replay.loaded
                x: (ui.trafficShown ? trafficBar.x - Theme.gutter : ui.width - ui.padR) - width
                anchors.bottom: parent.bottom
                anchors.bottomMargin: Theme.gutter
                width: posCol.implicitWidth + 21.6
                height: posCol.implicitHeight + 16.8
                radius: 4
                color: Theme.glassPanel
                border.color: Theme.borderAccentMid
                border.width: Theme.hairline
                Column {
                    id: posCol
                    x: 10.8
                    y: 8.4
                    spacing: 4
                    // LATITUDE / LONGITUDE, then the LTE cell rows of a cellular module
                    // (CORV_CELL_INFO) while it reports on the cellular link and LTE
                    // OVERLAY is on; a cell change turns them amber for 5 s
                    readonly property var cr: Lte.cellRows
                    readonly property bool cellLive: !!cr.live
                    readonly property color cellColor: cr.changed ? "#ffaa00" : Theme.textMain
                    Repeater {
                        model: [["LATITUDE", Vehicle.lat.toFixed(5), "", Theme.textMain], ["LONGITUDE", Vehicle.lon.toFixed(5), "", Theme.textMain]]
                               .concat(posCol.cellLive ? [["LTE CELL", posCol.cr.cell, "", posCol.cellColor],
                                                          ["FREQ · PCI", posCol.cr.band, "", posCol.cellColor],
                                                          ["RSSI", posCol.cr.rssi, "dBm", posCol.cellColor],
                                                          ["RSRP · SINR", posCol.cr.sig, "dBm · dB", posCol.cellColor],
                                                          ["NEIGHBOURS", posCol.cr.nb, "", posCol.cellColor],
                                                          ["CELL CHANGES", posCol.cr.ho, "", posCol.cellColor],
                                                          ["MAST DIST", posCol.cr.dist, posCol.cr.distUnit || "", posCol.cellColor]] : [])
                        delegate: Row {
                            required property var modelData
                            spacing: 0
                            Text {
                                width: 83
                                text: parent.modelData[0]
                                color: Theme.textDim
                                font.family: Theme.fontUi
                                font.pixelSize: Theme.fsXs
                                font.letterSpacing: 0.09 * Theme.fsXs
                            }
                            Text {
                                width: posCol.cellLive ? 130 : 51
                                horizontalAlignment: Text.AlignRight
                                textFormat: Text.StyledText
                                text: parent.modelData[1] + (parent.modelData[2] ? "<font color='" + Theme.textDim + "' size='1'> " + parent.modelData[2] + "</font>" : "")
                                color: parent.modelData[3]
                                font.family: Theme.fontData
                                font.pixelSize: Theme.fsSm
                            }
                        }
                    }
                }
            }

            // ADS-B traffic table (#traffic-bar): the 4 nearest contacts, while
            // SHOW ADS-B TRAFFIC is on; empty rows recede so a quiet sky does
            // not read as four contacts
            Rectangle {
                id: trafficBar
                visible: ui.trafficShown && !Replay.loaded
                readonly property real padV: Theme.clamp(5, 0.7 * Theme.vw, 11)
                readonly property real padH: Theme.clamp(7, 0.9 * Theme.vw, 14)
                readonly property real rowPad: Theme.clamp(2, 0.35 * Theme.vh, 5)
                x: ui.width - ui.padR - width
                anchors.bottom: parent.bottom
                anchors.bottomMargin: Theme.gutter
                width: Theme.clamp(195, 17 * Theme.vw, 330)
                height: tfcCol.implicitHeight + 2 * padV
                radius: 4
                color: Theme.glassPanel
                border.color: Theme.borderAccentMid
                border.width: Theme.hairline

                Column {
                    id: tfcCol
                    x: trafficBar.padH
                    y: trafficBar.padV
                    width: trafficBar.width - 2 * trafficBar.padH
                    spacing: Theme.clamp(2, 0.4 * Theme.vh, 5)
                    TfcRow { width: tfcCol.width; rowPad: trafficBar.rowPad; header: true; cells: ["CALLSIGN", "ALT (m)", "DIST (km)"] }
                    Repeater {
                        model: 4
                        delegate: TfcRow {
                            required property int index
                            readonly property var t: Vehicle.nearestTraffic[index]
                            width: tfcCol.width
                            rowPad: trafficBar.rowPad
                            first: index === 0
                            last: index === 3
                            empty: !t
                            cells: t ? [t.callsign, t.alt, t.dist] : ["---", "---", "---"]
                        }
                    }
                }
            }
        }

        // Armed inside a UAS geographical zone (#zone-arm-alert): under the mode
        // banner for 10 s, red in a no-fly zone, amber otherwise; a click closes it
        Rectangle {
            id: zoneArm
            z: 8
            readonly property var a: GeoZones.armAlert
            visible: !!a.title
            anchors.horizontalCenter: parent.horizontalCenter
            y: 140
            width: Math.max(zaTitle.implicitWidth, zaDetail.implicitWidth) + 36
            height: zaCol.implicitHeight + 16
            radius: 4
            color: a.level === "warn" ? Qt.rgba(0.55, 0.03, 0.03, 0.88) : Qt.rgba(0.45, 0.30, 0.0, 0.88)
            border.color: a.level === "warn" ? "#ff2a2a" : "#ffb300"
            border.width: 1.5
            Column {
                id: zaCol
                anchors.centerIn: parent
                spacing: 3
                Text {
                    id: zaTitle
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: zoneArm.a.title || ""
                    color: "white"
                    font.family: Theme.fontUi; font.pixelSize: Theme.fsLg; font.weight: Font.Bold; font.letterSpacing: 1.5
                }
                Text {
                    id: zaDetail
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: zoneArm.a.detail || ""
                    color: "#f0f0f0"
                    font.family: Theme.fontUi; font.pixelSize: Theme.fsSm
                }
            }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: GeoZones.closeArmAlert() }
        }

        // LiDAR map and ROS surface strips (#lidar-strip, #ros-strip): bottom centre of
        // the 3D view while the feature is on; the ROS strip above the LiDAR one when both show
        SensorStrip {
            id: lidarStrip
            z: 8
            visible: Lidar.enabled
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 14
            title: "LIDAR"
            state: Lidar.stripState
            label: Lidar.stripLabel
            count: Lidar.stripCount
            buttons: [{ text: "CLEAR MAP", tip: "Clear the accumulated map" }, { text: "SAVE", tip: "Save the map as .ply" }]
            onClicked: (i) => i === 0 ? Lidar.clearMap() : Lidar.saveMap()
        }
        SensorStrip {
            z: 8
            visible: Ros.enabled
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: lidarStrip.visible ? 50 : 14
            title: "ROS"
            state: Ros.stripState
            label: Ros.stripLabel
            count: Ros.stripText
            buttons: [{ text: Ros.colorLabel, minWidth: 46, on: Ros.colorMode === "distance", rough: Ros.colorMode === "rough",
                        tip: "Colour: grey lines; or a solid surface, red (near) to blue (far) by distance from the vehicle; or a solid surface by roughness: blue where it departs from its local plane no more than on average, red well above that" },
                      { text: Ros.shown ? "VIEW ON" : "VIEW OFF", off: !Ros.shown, tip: "Show or hide the surface (it keeps accumulating)" },
                      { text: "CLEAR", tip: "Clear the averaged surface" }]
            onClicked: (i) => i === 0 ? Ros.cycleColor() : i === 1 ? Ros.toggleView() : Ros.clearSurface()
        }

        // Mode change banner, centred 80 px from the top of #hud-wrapper
        ModeBanner {
            z: 8
            anchors.horizontalCenter: parent.horizontalCenter
            y: 80
        }
    }

    // EKF STATUS and FFT bars down the left side, one at a time; they slide in
    EkfPanel {
        x: Ekf.enabled ? 0 : -width
        visible: x > -width
        height: view.height
        Behavior on x { NumberAnimation { duration: 300; easing.type: Easing.OutQuad } }
    }
    FftPanel {
        x: Fft.enabled ? 0 : -width
        visible: x > -width
        height: view.height
        Behavior on x { NumberAnimation { duration: 300; easing.type: Easing.OutQuad } }
    }
    // Their tabs on the left edge around the window's middle, riding on the bar's edge when one is open
    EdgeTab {
        x: view.leftBarOpen ? Theme.ekfBarW : 0
        y: view.windowHeight / 2 - view.windowTop - height - 3
        label: "EKF STATUS"
        open: Ekf.enabled
        onClicked: Ekf.enabled = !Ekf.enabled
        Behavior on x { NumberAnimation { duration: 300; easing.type: Easing.OutQuad } }
    }
    EdgeTab {
        x: view.leftBarOpen ? Theme.ekfBarW : 0
        y: view.windowHeight / 2 - view.windowTop + 3
        label: "FFT"
        open: Fft.enabled
        onClicked: Fft.enabled = !Fft.enabled
        Behavior on x { NumberAnimation { duration: 300; easing.type: Easing.OutQuad } }
    }

    // GCS OPTIONS tab and sidebar on the right edge
    EdgeTab {
        x: flight.x + flight.width - width
        y: view.windowHeight / 2 - view.windowTop - height / 2
        label: "GCS OPTIONS"
        rightEdge: true
        open: view.sidebarOpen
        onClicked: view.sidebarOpen = !view.sidebarOpen
    }
    GcsSidebar {
        id: side
        x: view.width - (view.sidebarOpen ? width : 0)
        y: -33
        height: view.height + 33
        visible: view.sidebarOpen
        onClearTrail: { miniMap.clearTrail(); scene.clearTrail() }
    }

    Connections {
        target: Gcs
        function onHudMessage(text, level) { hud.pushMessage(text, level) }
    }
}
