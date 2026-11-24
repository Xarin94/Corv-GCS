// SYS CONFIG tab (#tab-sys-config): three 360 px columns (two under 1220 px,
// one under 820 px) centred in the page, gap 24, padding 24 24 48.
// VEHICLE & LINK · DISPLAY · DATA & TOOLS. Values persist under the keys of
// the Electron version (SysConfig.h).
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs
import Corv.Ui

Item {
    id: page
    // The 3D view, for the settings that act on it (FlightDataView.scene)
    property var scene: null

    readonly property int columns: Theme.winW >= 1220 ? 3 : Theme.winW >= 820 ? 2 : 1

    component ColumnTitle: Item {
        property alias text: t.text
        width: 360
        height: t.implicitHeight - 8
        Text {
            id: t
            x: 2
            color: Theme.textDim
            font.family: Theme.fontUi
            font.pixelSize: 10
            font.weight: Font.DemiBold
            font.letterSpacing: 2.5
        }
    }
    component Val: Text {
        width: parent ? parent.width : 185
        wrapMode: Text.Wrap
        color: Theme.textMain
        font.family: Theme.fontData
        font.pixelSize: 11
        lineHeight: 1.4
    }
    component Status: Text {
        width: parent ? parent.width : 185
        wrapMode: Text.Wrap
        color: Theme.accentCyan
        font.family: Theme.fontUi
        font.pixelSize: 10
        lineHeight: 1.4
    }
    component NumField: CfgInput {
        width: 96
        numeric: true
    }

    function jsonNum(key, field, def) { return String(SysConfig.jsonField(key, field, def)) }

    Flickable {
        id: flick
        objectName: "sysConfigFlick"
        anchors.fill: parent
        anchors.rightMargin: 15
        contentWidth: width
        contentHeight: grid.y + grid.height + 48
        clip: true
        boundsBehavior: Flickable.StopAtBounds
        ScrollBar.vertical: bar

        Grid {
            id: grid
            columns: page.columns
            columnSpacing: 24
            rowSpacing: 24
            x: Math.max(24, (flick.width - (page.columns * 360 + (page.columns - 1) * 24)) / 2)
            y: 24
            verticalItemAlignment: Grid.AlignTop

            // ───────────── VEHICLE & LINK ─────────────
            Column {
                width: 360
                spacing: 20
                ColumnTitle { text: qsTr("VEHICLE & LINK") }

                SysPanel {
                    title: "FLIGHT STACK"
                    SysRow {
                        label: "ENVIRONMENT"
                        tip: "Which firmware this GCS is set up for: it suggests the link protocol and decides which mission features can be flown"
                        stack: true
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "ardupilot", label: "ArduPilot · Copter / Plane / Rover" },
                                      { value: "inav", label: "INAV · waypoint missions over MSP" },
                                      { value: "betaflight", label: "Betaflight · acro, no navigation" }]
                            value: Plan.platformId
                            onPicked: (v) => Plan.platformId = v
                        }
                    }
                    SysRow { label: "SUGGESTED LINK"; Val { text: Theme.tr(Plan.platformLink) } }
                    SysRow { note: Plan.platformHint }
                }

                SysPanel {
                    title: "NAVIGATION"
                    SysRow {
                        label: "POSITION"
                        tip: "Relative: for a vehicle that has never had a GPS (a ROV, a flight with no GNSS). Position from the vehicle's local frame when it sends one, otherwise dead-reckoned here; no map or terrain, a plane at zero"
                        stack: true
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "absolute", label: "GPS · absolute, map and terrain" },
                                      { value: "relative", label: "Relative · no GPS, plane at zero" }]
                            value: SysConfig.navPosition
                            onPicked: (v) => SysConfig.navPosition = v
                        }
                    }
                    SysRow {
                        label: "VELOCITY SOURCE"
                        tip: "What the GCS dead-reckons on when the vehicle sends no local position of its own. Auto: a fixed wing's airspeed along the heading, else the EKF velocity, else ground speed along the heading"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "auto", label: "Auto" }, { value: "air", label: "Airspeed + heading" },
                                      { value: "ground", label: "Ground speed + heading" }, { value: "ekf", label: "EKF velocity" }]
                            value: SysConfig.navVel
                            onPicked: (v) => SysConfig.navVel = v
                        }
                    }
                    SysRow {
                        label: "WATER (DEPTH SENSOR)"
                        tip: "Density used to turn a sub's pressure reading into depth in the relative mode"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "fresh", label: "Fresh water" }, { value: "salt", label: "Sea water" }]
                            value: SysConfig.navWater
                            onPicked: (v) => SysConfig.navWater = v
                        }
                    }
                    SysRow { label: "ESTIMATE"; Val { text: Theme.tr(SysConfig.navStatus) } }
                    SysRow {
                        label: ""
                        GcsButton {
                            width: parent.width
                            text: "RESET ESTIMATE"
                            tip: "Start the relative position and the dead-reckoned track again from zero"
                            onClicked: SysConfig.resetEstimate()
                        }
                    }
                }

                SysPanel {
                    title: "ROTOR LOAD"
                    SysRow {
                        label: "SCHEMATIC"
                        CfgCheck {
                            id: rotorEnable
                            width: parent.width
                            text: "Show rotor load schematic"
                            checked: SysConfig.jsonField("rotor-load-cfg", "enabled", true)
                            onToggled: (c) => SysConfig.setJsonField("rotor-load-cfg", "enabled", c)
                            // The sidebar's SHOW ROTOR LOAD drives the same setting (syncConfigInputs)
                            Connections {
                                target: SysConfig
                                function onJsonFieldChanged(key) {
                                    if (key === "rotor-load-cfg") rotorEnable.checked = SysConfig.jsonField("rotor-load-cfg", "enabled", true)
                                }
                            }
                        }
                    }
                    SysRow {
                        label: "FRAME"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "auto", label: "AUTO (servo outputs, MAV_TYPE)" }, { value: "sail", label: "SAILBOAT (wing + rudder)" },
                                      { value: "1", label: "SINGLE ROTOR (plane / heli)" }, { value: "3", label: "TRICOPTER (3)" },
                                      { value: "4", label: "QUAD (4)" }, { value: "6", label: "HEXA (6)" }, { value: "8", label: "OCTO (8)" },
                                      { value: "10", label: "DECA (10)" }, { value: "12", label: "DODECA (12)" }]
                            value: String(SysConfig.jsonField("rotor-load-cfg", "frame", "auto"))
                            onPicked: (v) => SysConfig.setJsonField("rotor-load-cfg", "frame", v)
                        }
                    }
                    SysRow { sub: "PWM SCALE" }
                    Repeater {
                        model: [["MOTORS OFF (MIN)", "pwmMin", 1000], ["FULL (MAX)", "pwmMax", 2000], ["GREEN ABOVE", "thGreen", 1100],
                                ["ORANGE ABOVE", "thOrange", 1500], ["RED ABOVE", "thRed", 1700]]
                        delegate: SysRow {
                            required property var modelData
                            label: modelData[0]
                            NumField {
                                text: page.jsonNum("rotor-load-cfg", modelData[1], modelData[2])
                                onCommitted: (t) => { const v = parseInt(t); if (!isNaN(v)) SysConfig.setJsonField("rotor-load-cfg", modelData[1], Math.max(500, Math.min(2500, v))) }
                            }
                        }
                    }
                    SysRow { sub: "SINGLE ROTOR & SAIL" }
                    SysRow {
                        label: "MAIN ROTOR CHANNEL"
                        NumField {
                            text: page.jsonNum("rotor-load-cfg", "singleChannel", 3)
                            onCommitted: (t) => { const v = parseInt(t); if (!isNaN(v)) SysConfig.setJsonField("rotor-load-cfg", "singleChannel", Math.max(1, Math.min(16, v))) }
                        }
                    }
                    SysRow {
                        label: "RUDDER MAX (DEG)"
                        NumField {
                            text: page.jsonNum("rotor-load-cfg", "rudderMax", 35)
                            onCommitted: (t) => { const v = parseInt(t); if (!isNaN(v)) SysConfig.setJsonField("rotor-load-cfg", "rudderMax", Math.max(1, Math.min(180, v))) }
                        }
                    }
                    SysRow { note: "Below the green threshold the gauges read blue (motors stopped). On AUTO the motors are the ones the outputs are assigned to (SERVOn_FUNCTION: MotorN on the ring, the fixed-wing throttles in the middle), read from the vehicle only while the schematic is shown; the FLAPS indicator appears over it when an output carries flaps or flaperons. Until those parameters arrive, plane and helicopter show only the main rotor, on the channel above (ArduPlane throttle = 3, heli RSC = 8). A Rover with SAIL_ENABLE or a MastRotation output shows the wing and rudder dials instead; the rudder angle at full steering is a property of the boat, not a parameter (AutoSail: 35)." }
                }

                SysPanel {
                    title: "MAVLINK STREAM RATES"
                    SysRow { note: "Messages per second the vehicle sends on this link (SR0_* parameters, MAV1_* from firmware 4.7). Read them first; a write stores them on the vehicle."; noteTop: true }
                    Repeater {
                        model: ["RAW SENSORS", "EXT STATUS", "RC CHANNELS", "RAW CTRL", "POSITION", "EXTRA1 (ATTITUDE)", "EXTRA2 (VFR HUD)", "EXTRA3", "PARAMS"]
                        delegate: SysRow {
                            required property string modelData
                            required property int index
                            label: modelData
                            NumField {
                                text: String(SysConfig.streamRates[index])
                                frame: SysConfig.streamRateState[index] === "ok" ? Theme.accentGreen
                                     : SysConfig.streamRateState[index] === "error" ? Theme.statusRed : Theme.borderInput
                                onTextChanged: SysConfig.setStreamRate(index, text)
                            }
                        }
                    }
                    SysRow {
                        full: true
                        Row {
                            width: parent.width
                            spacing: 8
                            GcsButton { width: (parent.width - 8) / 2; text: "READ"; onClicked: SysConfig.readStreamRates() }
                            GcsButton { width: (parent.width - 8) / 2; text: "WRITE"; onClicked: SysConfig.writeStreamRates() }
                        }
                    }
                }
            }

            // ───────────── DISPLAY ─────────────
            Column {
                width: 360
                spacing: 20
                ColumnTitle { text: qsTr("DISPLAY") }

                SysPanel {
                    title: "3D VIEW"
                    SysRow { sub: "MODEL" }
                    SysRow {
                        label: "3D MODEL"
                        CfgSelect {
                            width: parent.width
                            options: SysConfig.models.map(m => ({ value: m + ".glb", label: m }))
                            value: SysConfig.selectedModel
                            onPicked: (v) => SysConfig.selectedModel = v
                        }
                    }
                    SysRow {
                        label: "MODEL SCALE"
                        CfgSlider {
                            width: parent.width
                            from: 0.1; to: 5; stepSize: 0.1
                            value: parseFloat(SysConfig.value("modelScale", "1"))
                            valueText: value.toFixed(1)
                            onMoved: (v) => SysConfig.setValue("modelScale", v.toFixed(1))
                        }
                    }
                    SysRow {
                        // What may not go under the ground in the 3D view (ground-clamp-enabled
                        // stays in step for the Electron version: off is "0")
                        label: "TERRAIN CLIPPING"
                        tip: "What may not go under the terrain in the 3D view"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "all", label: "Vehicle, camera, trail, mission" },
                                      { value: "vehicle", label: "Vehicle and camera only" },
                                      { value: "off", label: "Off (true altitudes)" }]
                            value: SysConfig.value("ground-clamp-mode", SysConfig.value("ground-clamp-enabled", "1") === "0" ? "off" : "all")
                            onPicked: (v) => {
                                SysConfig.setValue("ground-clamp-mode", v)
                                SysConfig.setValue("ground-clamp-enabled", v === "off" ? "0" : "1")
                            }
                        }
                    }
                    SysRow {
                        // Position, attitude and angular velocity drawn between the telemetry
                        // samples, in the 3D view and on the HUD (PoseSmoother)
                        label: "MOTION SMOOTHING"
                        tip: "How long a new telemetry sample takes to blend into the drawn movement (slider × 0.5 s): the position, the attitude and the angular velocity it turns with between samples. 0 draws the telemetry as received, without any propagation"
                        CfgSlider {
                            width: parent.width
                            from: 0; to: 0.5; stepSize: 0.01
                            bound: true
                            value: SysConfig.motionSmoothing
                            valueText: value > 0 ? value.toFixed(2) : "OFF"
                            onMoved: (v) => SysConfig.motionSmoothing = v
                        }
                    }
                    SysRow { sub: "TERRAIN & LIGHT" }
                    SysRow {
                        label: "TERRAIN (HGT FOLDER)"
                        Column {
                            width: parent.width
                            spacing: 4
                            GcsButton {
                                width: parent.width
                                dashed: true
                                text: "SELECT HGT FOLDER"
                                onClicked: hgtDialog.open()
                            }
                            Status { text: Theme.tr(SysConfig.hgtStatus) }
                        }
                    }
                    SysRow {
                        // TERRAIN_REQUEST / TERRAIN_DATA: the database of an autopilot with TERRAIN_ENABLE
                        label: "VEHICLE TERRAIN DATA"
                        Status { width: parent.width; text: Theme.tr(Gcs.terrainFeed) }
                    }
                    SysRow {
                        label: "ALTITUDE OFFSET (M)"
                        tip: "Added to every vehicle altitude in the 3D view, to line it up with the terrain model. Not remembered"
                        CfgInput {
                            id: altOffset
                            objectName: "altOffset"
                            width: parent.width
                            numeric: true
                            // The one in use: a log loaded or closed clears it
                            readonly property string inUse: String(Math.round(Vehicle.altOffset * 100) / 100)
                            text: inUse
                            onInUseChanged: if (!activeFocus) text = inUse
                            onCommitted: (t) => { SysConfig.setAltitudeOffset(t); text = Qt.binding(() => inUse) }
                        }
                    }
                    SysRow {
                        // Not remembered: the clock again at the next start
                        label: "TIME OF DAY (DEBUG)"
                        Row {
                            width: parent.width
                            spacing: 6
                            CfgSlider {
                                anchors.verticalCenter: parent.verticalCenter
                                width: parent.width - clockBtn.width - 6
                                from: 0; to: 1440; stepSize: 1
                                bound: true
                                value: SysConfig.timeOfDay >= 0 ? SysConfig.timeOfDay : SysConfig.clockMinutes
                                valueText: String(Math.floor(value / 60)).padStart(2, "0") + ":" + String(Math.round(value) % 60).padStart(2, "0")
                                onMoved: (v) => SysConfig.timeOfDay = Math.round(v)
                            }
                            GcsButton {
                                id: clockBtn
                                width: 52
                                text: "CLOCK"
                                enabled: SysConfig.timeOfDay >= 0
                                tip: "The sun follows the clock again"
                                onClicked: SysConfig.timeOfDay = -1
                            }
                        }
                    }
                    SysRow {
                        // The brightness applies whenever the sun does not light the terrain
                        visible: !(page.scene && page.scene.sunlight && !page.scene.schematic)
                        height: visible ? implicitHeight : 0
                        label: "MAP BRIGHTNESS"
                        CfgSlider {
                            width: parent.width
                            from: 0.3; to: 1.6; stepSize: 0.01
                            bound: true
                            value: SysConfig.mapBrightness
                            onMoved: (v) => SysConfig.mapBrightness = v
                        }
                    }
                    SysRow { sub: "QUALITY & RANGE" }
                    SysRow {
                        label: "FRAME RATE"
                        tip: "30 FPS halves the rendering and compositing work — use it on battery"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "60", label: "60 FPS" }, { value: "30", label: "30 FPS (ECO)" }]
                            value: SysConfig.value("renderFpsCap", "60") === "30" ? "30" : "60"
                            onPicked: (v) => SysConfig.setValue("renderFpsCap", v)
                        }
                    }
                    SysRow {
                        label: "RESOLUTION"
                        tip: "Reduce 3D rendering resolution to save GPU work. HUD and text keep their display resolution"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "native", label: "NATIVE" }, { value: "balanced", label: "BALANCED (DEFAULT)" }, { value: "eco", label: "ECO" }]
                            value: SysConfig.renderQuality
                            onPicked: (v) => SysConfig.renderQuality = v
                        }
                    }
                    SysRow {
                        label: "SATELLITE DETAIL"
                        tip: "Sharpest satellite imagery drawn around the aircraft in the 3D view (resolution at mid latitudes). Each step doubles the detail, and the tiles downloaded when flying low"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "16", label: "ZOOM 16 · 1.6 m (OFF)" }, { value: "17", label: "ZOOM 17 · 0.8 m" },
                                      { value: "18", label: "ZOOM 18 · 0.4 m (DEFAULT)" }, { value: "19", label: "ZOOM 19 · 0.2 m" },
                                      { value: "20", label: "ZOOM 20 · 0.1 m (MAX)" }]
                            value: SysConfig.value("satDetailMaxZoom", "18")
                            onPicked: (v) => SysConfig.setValue("satDetailMaxZoom", v)
                        }
                    }
                    SysRow {
                        label: "SATELLITE RADIUS"
                        tip: "How far around the aircraft the 3D terrain gets satellite imagery; beyond it the terrain is coloured by height. A wider radius downloads more tiles and draws more"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "5000", label: "5 KM" }, { value: "10000", label: "10 KM (DEFAULT)" }, { value: "15000", label: "15 KM" },
                                      { value: "20000", label: "20 KM" }, { value: "30000", label: "30 KM" }]
                            value: String(SysConfig.satelliteRadius)
                            onPicked: (v) => SysConfig.setValue("satelliteRadiusM", v)
                        }
                    }
                    SysRow {
                        label: "TERRAIN RADIUS"
                        tip: "How far around the aircraft the 3D terrain is built. The haze follows it: a farther edge, thinner haze. A wider radius loads more elevation data and draws more"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "35000", label: "35 KM (DEFAULT)" }, { value: "50000", label: "50 KM" }, { value: "70000", label: "70 KM" }]
                            value: String(SysConfig.terrainRadius)
                            onPicked: (v) => SysConfig.setTerrainRadius(parseFloat(v))
                        }
                    }
                }

                SysPanel {
                    title: "HUD DATA FIELDS"
                    SysRow { note: "The six data cells beside the HUD. For each: the value shown, a multiplier and the unit label printed (blank = the value's own)."; noteTop: true }
                    Repeater {
                        model: ["LEFT 1", "LEFT 2", "LEFT 3", "RIGHT 1", "RIGHT 2", "RIGHT 3"]
                        delegate: SysRow {
                            id: cellRow
                            required property string modelData
                            required property int index
                            readonly property var cell: SysConfig.hudCells[index]
                            label: modelData
                            stack: true
                            Row {
                                width: parent.width
                                spacing: 6
                                CfgSelect {
                                    width: parent.width - 52 - 56 - 12
                                    options: SysConfig.hudFields.map(f => ({ value: f.key, label: f.label }))
                                    value: cellRow.cell ? cellRow.cell.field : ""
                                    onPicked: (v) => SysConfig.setHudCellField(cellRow.index, v)
                                }
                                CfgInput {
                                    width: 52
                                    leftPadding: 4; rightPadding: 4
                                    horizontalAlignment: TextInput.AlignHCenter
                                    placeholderText: "×1"
                                    text: cellRow.cell ? cellRow.cell.multText : ""
                                    onCommitted: (t) => SysConfig.setHudCellMultiplier(cellRow.index, t)
                                }
                                CfgInput {
                                    width: 56
                                    leftPadding: 4; rightPadding: 4
                                    horizontalAlignment: TextInput.AlignHCenter
                                    placeholderText: qsTr("unit")
                                    text: cellRow.cell ? cellRow.cell.unitLabel : ""
                                    onCommitted: (t) => SysConfig.setHudCellUnit(cellRow.index, t)
                                }
                            }
                        }
                    }
                    SysRow {
                        full: true
                        GcsButton { width: parent.width; text: "RESET DEFAULTS"; onClicked: SysConfig.resetHudCells() }
                    }
                }

                SysPanel {
                    title: "SIYI CAMERA STREAM"
                    SysRow {
                        label: "IP ADDRESS"
                        CfgInput { width: parent.width; text: String(SysConfig.jsonField("fpv-settings", "ip", "192.168.144.25")); onCommitted: (t) => SysConfig.setJsonField("fpv-settings", "ip", t) }
                    }
                    SysRow {
                        label: "RTSP PORT"
                        NumField { text: page.jsonNum("fpv-settings", "port", 8554); onCommitted: (t) => { const v = parseInt(t); if (!isNaN(v)) SysConfig.setJsonField("fpv-settings", "port", v) } }
                    }
                    SysRow {
                        label: "STREAM PATH"
                        CfgInput { width: parent.width; text: String(SysConfig.jsonField("fpv-settings", "path", "/main.264")); onCommitted: (t) => SysConfig.setJsonField("fpv-settings", "path", t) }
                    }
                    SysRow {
                        label: "TARGET FPS"
                        NumField { text: page.jsonNum("fpv-settings", "fps", 30); onCommitted: (t) => { const v = parseInt(t); if (!isNaN(v)) SysConfig.setJsonField("fpv-settings", "fps", Math.max(1, Math.min(60, v))) } }
                    }
                    SysRow { label: "ADDRESS"; stack: true; Val { text: Fpv.url } }
                    SysRow {
                        label: "STREAM"
                        Val {
                            text: Theme.tr(!Fpv.available ? "this build has no video decoder"
                                : Fpv.connected ? Fpv.streamInfo : Fpv.connecting ? "connecting…" : "off")
                        }
                    }
                    SysRow { note: "Opened by the FPV button at the top right. Network adapter: static IP in the 192.168.144.x subnet. TARGET FPS is the most frames shown a second: a stream that sends more is decoded in full and thinned out." }
                }
            }

            // ───────────── DATA & TOOLS ─────────────
            Column {
                width: 360
                spacing: 20
                ColumnTitle { text: qsTr("DATA & TOOLS") }

                SysPanel {
                    title: "AIRSPACE"
                    SysRow {
                        label: "ADS-B TRAFFIC"
                        CfgCheck { width: parent.width; text: "Show manned traffic"; checked: SysConfig.adsbEnabled; onToggled: (c) => SysConfig.adsbEnabled = c }
                    }
                    SysRow { sub: "UAS GEO ZONES" }
                    SysRow {
                        label: "RESTRICTED ZONES"
                        CfgCheck {
                            id: gzCheck
                            width: parent.width
                            text: "On the maps and the terrain, with height warnings"
                            checked: GeoZones.enabled
                            onToggled: (c) => GeoZones.enabled = c
                        }
                    }
                    SysRow {
                        label: "IN REACH"
                        Val { text: !GeoZones.enabled ? qsTr("Off") : GeoZones.inReach + qsTr(" zones") + (GeoZones.pending ? " (" + GeoZones.pending + qsTr(" loading)") : "") }
                    }
                    SysRow { label: "SOURCES"; Val { text: Theme.tr(GeoZones.enabled ? (GeoZones.sourcesText || "--") : "--"); wrapMode: Text.Wrap } }
                    SysRow {
                        label: "IMPORTED FILES"
                        Column {
                            width: parent.width
                            spacing: 2
                            Val { visible: GeoZones.imports.length === 0; text: qsTr("none") }
                            Repeater {
                                model: GeoZones.imports
                                delegate: Row {
                                    required property var modelData
                                    spacing: 6
                                    Val { width: implicitWidth; text: modelData.name + " · " + modelData.count + qsTr(" zones · ") + modelData.date }
                                    Text {
                                        text: "✕"
                                        color: "#ff6b6b"
                                        font.pixelSize: 11
                                        MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: GeoZones.removeImport(parent.parent.modelData.file) }
                                    }
                                }
                            }
                        }
                    }
                    SysRow {
                        label: "ED-269 / ED-318"
                        Row {
                            width: parent.width
                            spacing: 8
                            GcsButton {
                                width: (parent.width - 8) / 2
                                text: "IMPORT FILE…"
                                tip: "Load the UAS geographical zones file published by a state, e.g. Italy's from D-Flight (profile page → Download UAS Zone Geo)"
                                onClicked: gzDialog.open()
                            }
                            GcsButton {
                                width: (parent.width - 8) / 2
                                text: "REFRESH"
                                tip: "Forget the downloaded zones and fetch them again (imported files are kept)"
                                onClicked: GeoZones.refresh()
                            }
                        }
                    }
                    SysRow { note: "Zones within 50 km of the connected vehicle and of the mission, only from the sources of the country in reach: FAA (US), DFS DIPUL (DE), IGN / DGAC (FR), ENAIRE (ES), FOCA / BAZL (CH); fetched again every 6 h. Italy and the other states: import the ED-269 file of the national portal. Advisory only: the state's official map is what applies." }
                }

                SysPanel {
                    title: "BATTERY GAUGE"
                    SysRow { note: "For the percentage when the vehicle does not report one."; noteTop: true }
                    SysRow {
                        label: "MIN VOLTAGE (EMPTY)"
                        NumField { id: vmin; text: SysConfig.batVMin; onCommitted: SysConfig.setBatteryRange(vmin.text, vmax.text) }
                    }
                    SysRow {
                        label: "MAX VOLTAGE (FULL)"
                        NumField { id: vmax; text: SysConfig.batVMax; onCommitted: SysConfig.setBatteryRange(vmin.text, vmax.text) }
                    }
                    SysRow { label: "CELLS (AUTO)"; Val { text: SysConfig.batCells } }
                }

                SysPanel {
                    id: offline
                    title: "OFFLINE DATA DOWNLOAD"
                    function estimate() { return SysConfig.offlineEstimate(oN.text, oS.text, oW.text, oE.text, oZ.text, oSat.checked, oSrtm.checked) }
                    SysRow { note: "Satellite imagery and elevation for an area, fetched now for a flight without network."; noteTop: true }
                    SysRow {
                        label: "LATITUDE N · S"
                        Row {
                            width: parent.width
                            spacing: 6
                            Text { width: 12; height: 26; text: "N"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.DemiBold }
                            CfgInput { id: oN; width: (parent.width - 24 - 18) / 2; numeric: true; text: "47" }
                            Text { width: 12; height: 26; text: "S"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.DemiBold }
                            CfgInput { id: oS; width: (parent.width - 24 - 18) / 2; numeric: true; text: "36" }
                        }
                    }
                    SysRow {
                        label: "LONGITUDE W · E"
                        Row {
                            width: parent.width
                            spacing: 6
                            Text { width: 12; height: 26; text: "W"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.DemiBold }
                            CfgInput { id: oW; width: (parent.width - 24 - 18) / 2; numeric: true; text: "6" }
                            Text { width: 12; height: 26; text: "E"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.DemiBold }
                            CfgInput { id: oE; width: (parent.width - 24 - 18) / 2; numeric: true; text: "19" }
                        }
                    }
                    SysRow { label: "SATELLITE ZOOM MAX"; NumField { id: oZ; text: "16" } }
                    SysRow {
                        label: "CONTENT"
                        Column {
                            width: parent.width
                            spacing: 2
                            CfgCheck { id: oSat; width: parent.width; text: "Satellite tiles"; checked: true }
                            CfgCheck { id: oSrtm; width: parent.width; text: "SRTM1 elevation"; checked: true }
                        }
                    }
                    SysRow {
                        label: "ESTIMATE"
                        stack: true
                        Text {
                            width: parent.width
                            wrapMode: Text.Wrap
                            text: Theme.tr(offline.estimate())
                            color: Theme.textMutedBright
                            font.family: Theme.fontData
                            font.pixelSize: 10
                            lineHeight: 1.4
                        }
                    }
                    SysRow {
                        full: true
                        GcsButton {
                            visible: !SysConfig.offlineBusy
                            width: parent.width
                            text: "DOWNLOAD"
                            onClicked: SysConfig.startOffline(oN.text, oS.text, oW.text, oE.text, oZ.text, oSat.checked, oSrtm.checked)
                        }
                        GcsButton {
                            visible: SysConfig.offlineBusy
                            width: parent.width
                            text: "CANCEL"
                            onClicked: SysConfig.cancelOffline()
                        }
                    }
                    SysRow {
                        full: true
                        visible: SysConfig.offlineBusy
                        height: visible ? implicitHeight : 0
                        ProgressBar {
                            width: parent.width
                            height: 10
                            value: SysConfig.offlineProgress
                            background: Rectangle { radius: 2; color: Qt.rgba(1, 1, 1, 0.1) }
                            contentItem: Item {
                                Rectangle { width: parent.width * SysConfig.offlineProgress; height: parent.height; radius: 2; color: Theme.accentCyan }
                            }
                        }
                    }
                    SysRow {
                        full: true
                        Status { text: Theme.tr(SysConfig.offlineStatus) }
                    }
                }

                SysPanel {
                    title: "FFT FROM LOGS"
                    SysRow {
                        label: "FFT SIZE"
                        tip: "Samples in each FFT. The frequency step is the log's sample rate divided by this: more points separate closer lines, but each spectrum covers a longer stretch of the log"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "256", label: "256 POINTS" }, { value: "512", label: "512 POINTS" }, { value: "1024", label: "1024 POINTS (DEFAULT)" },
                                      { value: "2048", label: "2048 POINTS" }, { value: "4096", label: "4096 POINTS" }, { value: "8192", label: "8192 POINTS" }]
                            value: String(SysConfig.jsonField("log-fft-cfg", "n", 1024))
                            onPicked: (v) => SysConfig.setJsonField("log-fft-cfg", "n", parseInt(v))
                        }
                    }
                    SysRow {
                        label: "AVERAGING"
                        tip: "Welch averaging: the spectra of several windows, each overlapping the next by half, averaged into one. A steadier noise floor and clearer weak lines, from a longer stretch of the log"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "1", label: "NONE (DEFAULT)" }, { value: "2", label: "2 WINDOWS" }, { value: "4", label: "4 WINDOWS" }, { value: "8", label: "8 WINDOWS" }]
                            value: String(SysConfig.jsonField("log-fft-cfg", "avg", 1))
                            onPicked: (v) => SysConfig.setJsonField("log-fft-cfg", "avg", parseInt(v))
                        }
                    }
                    SysRow {
                        label: "TIME STEP"
                        tip: "A new spectrum for every step of log time: the columns of the waterfall"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "100", label: "0.1 s" }, { value: "250", label: "0.25 s (DEFAULT)" }, { value: "500", label: "0.5 s" }, { value: "1000", label: "1 s" }]
                            value: String(SysConfig.jsonField("log-fft-cfg", "stepMs", 250))
                            onPicked: (v) => SysConfig.setJsonField("log-fft-cfg", "stepMs", parseInt(v))
                        }
                    }
                    SysRow { label: "LOG IN REPLAY"; Val { text: Theme.tr(Fft.logStatus); textFormat: Text.StyledText } }
                    SysRow { note: "For a DataFlash log (.bin) in LOG REPLAY the GCS computes the vibration spectra of the FFT bar from the IMU samples in the log: GYR / ACC at the sensor rate (LOG_BITMASK IMU_RAW, INS_RAW_LOG_OPT), the batch sampler (INS_LOG_BAT_MASK) or IMU at the loop rate, whichever is there; the FFT bar picks the stream. Frequency step = sample rate ÷ FFT size (a 2 kHz gyro: 1024 points → 1.95 Hz, 0.5 s of log). A batch is INS_LOG_BAT_CNT samples long, and the FFT never spans more. The spectrum the vehicle sends live (HPF_FFT_ENABLE = 1) is the firmware's and does not change: 256 points at 800 Hz, 3.125 Hz, every 0.25 s." }
                }

                SysPanel {
                    title: "APPLICATION"
                    SysRow {
                        label: "LANGUAGE"
                        CfgSelect {
                            width: parent.width
                            options: [{ value: "en", label: "English" }, { value: "zh", label: "中文 (Chinese)" }]
                            value: SysConfig.language
                            onPicked: (v) => SysConfig.language = v
                        }
                    }
                    SysRow {
                        label: "SETTINGS OF 1.x"
                        visible: SysConfig.legacySettings.length > 0
                        tip: "Copies the settings of CORV GCS 1.x found on this computer: HUD fields, models, views, joystick, LiDAR, ROS, RTK, radio, battery, parameter favourites. The cellular key is not among them: it was encrypted for the old program"
                        GcsButton {
                            width: parent.width
                            text: "IMPORT…"
                            tip: SysConfig.legacySettings
                            onClicked: SysConfig.importLegacySettings()
                        }
                    }
                    SysRow {
                        label: "DEVTOOLS"
                        GcsButton {
                            width: parent.width
                            text: "OPEN DEVTOOLS"
                            onClicked: SysConfig.toast("No DevTools in this version: the debug log below has the same record")
                        }
                    }
                    SysRow {
                        label: "DEBUG LOG (LAST 5 MIN)"
                        Row {
                            width: parent.width
                            spacing: 8
                            GcsButton {
                                width: (parent.width - 8) / 2
                                text: "SAVE COPY…"
                                tip: "One file with the last 5 minutes of this session and of the previous one: send it to support"
                                onClicked: { logDialog.selectedFile = "file:///" + SysConfig.debugLogName(); logDialog.open() }
                            }
                            GcsButton {
                                width: (parent.width - 8) / 2
                                text: "SHOW FILE"
                                tip: "Show the debug log file, rewritten every few seconds"
                                onClicked: SysConfig.revealDebugLog()
                            }
                        }
                    }
                }

                SysPanel {
                    title: "UPDATES"
                    SysRow {
                        label: "VERSION"
                        CfgValue { width: parent.width; text: Updates.version }
                    }
                    SysRow {
                        label: "AT START"
                        CfgCheck {
                            width: parent.width
                            text: "Check for a new version"
                            checked: Updates.checkAtStart
                            onToggled: (c) => Updates.checkAtStart = c
                        }
                    }
                    SysRow {
                        label: "NOW"
                        Row {
                            width: parent.width
                            spacing: 8
                            GcsButton {
                                width: (parent.width - 8) / 2
                                text: Updates.state === "checking" ? "CHECKING…" : "CHECK NOW"
                                onClicked: Updates.check()
                            }
                            GcsButton {
                                width: (parent.width - 8) / 2
                                text: "RELEASES"
                                tip: "The release page on GitHub"
                                onClicked: Updates.openPage()
                            }
                        }
                    }
                    SysRow {
                        note: (Updates.message.length > 0 ? Updates.message + ". " : "")
                              + qsTr("The GCS asks GitHub (%1) for the list of releases, without an account, a few seconds after start. Nothing is downloaded until INSTALL in the update dialog (UPDATE at the top right): then only the package for this system, checked against its SHA-256.").arg(Updates.repo)
                    }
                }
            }
        }
    }

    // The page scrollbar of the Electron version (Chromium's light classic bar)
    ScrollBar {
        id: bar
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 15
        policy: ScrollBar.AlwaysOn
        padding: 2
        topPadding: 15
        bottomPadding: 15
        background: Rectangle {
            color: "#f1f1f1"
            Text { anchors.horizontalCenter: parent.horizontalCenter; y: 2; text: "▲"; font.pixelSize: 7; color: "#505050" }
            Text { anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.bottom; anchors.bottomMargin: 2; text: "▼"; font.pixelSize: 7; color: "#505050" }
        }
        contentItem: Rectangle {
            implicitWidth: 9
            color: bar.pressed ? "#787878" : bar.hovered ? "#a8a8a8" : "#c1c1c1"
        }
    }

    FolderDialog {
        id: hgtDialog
        title: "Select HGT folder"
        onAccepted: SysConfig.selectHgtFolder(selectedFolder)
    }
    FileDialog {
        id: gzDialog
        title: "Import UAS geographical zones (ED-269 / ED-318)"
        nameFilters: ["ED-269 / ED-318 JSON (*.json *.geojson)", "All files (*)"]
        onAccepted: GeoZones.importFile(selectedFile)
    }
    Connections {
        target: GeoZones
        function onToast(text) { SysConfig.toast(text) }
    }
    FileDialog {
        id: logDialog
        title: "Save debug log"
        fileMode: FileDialog.SaveFile
        nameFilters: ["Log files (*.log)", "All files (*)"]
        onAccepted: SysConfig.saveDebugLogCopy(selectedFile)
    }
}
