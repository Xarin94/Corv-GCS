// SETUP tab: vertical navigation on the left (COMMS, VEHICLE SETUP, TUNING,
// TOOLS) and the page of the selected entry.
import QtQuick
import QtQuick.Controls.Basic

Item {
    id: page
    property string current: "CONNECTION"
    onCurrentChanged: SetupCfg.page = current
    Component.onCompleted: SetupCfg.page = current

    // .nav-status-dot / .nav-group-dot: 6 px, green with a glow
    component StatusDot: Rectangle {
        anchors.verticalCenter: parent.verticalCenter
        width: 6; height: 6; radius: 3
        color: "#00ff7f"
        Rectangle { anchors.centerIn: parent; width: 10; height: 10; radius: 5; color: Qt.rgba(0, 1, 127 / 255, 0.25); z: -1 }
    }
    // The pages a flight stack cannot use (Platforms.h): greyed, the reason in the tooltip
    readonly property var sections: ({ "CONNECTION": "connection", "CELLULAR LINK": "lte-link", "TELEMETRY FORWARD": "tel-forward", "RTK / GPS INJECT": "rtk-gps",
                                       "ACCELEROMETER": "accel-cal", "COMPASS": "compass-cal", "GYRO / BARO": "sensor-cal", "RADIO CAL": "radio-cal",
                                       "JOYSTICK / RC": "joystick", "FLIGHT MODES": "flight-modes", "FAILSAFE": "failsafe", "SERVO / RELAY": "servo-relay",
                                       "PID TUNING": "pid-tuning", "EXT TUNING": "ext-tuning", "VIBRATIONS": "vibration", "PARAMETERS": "parameters",
                                       "LOG DOWNLOAD": "log-download", "SIMULATION": "simulation", "LIDAR": "lidar", "ROS": "ros" })
    function reason(name) { return Plan.platformId.length > 0 ? Plan.setupReason(sections[name] || "") : "" }
    Connections {
        target: Plan
        // The operator was standing on a page that just went away under them
        function onPlatformChanged() { if (page.reason(page.current).length > 0) page.current = "CONNECTION" }
    }
    // A procedure running on a page: green dot on its entry and on its group
    function busy(name) {
        switch (name) {
        case "ACCELEROMETER": return Calibration.accelActive
        case "COMPASS": return Calibration.magActive
        case "RTK / GPS INJECT": return Rtk.connected      // setNavDot('rtk-gps')
        case "TELEMETRY FORWARD": return TelForward.outputs.length > 0
        case "CELLULAR LINK": return Lte.online                // setNavDot('lte-link')
        case "JOYSTICK / RC": return Joystick.enabled
        case "LOG DOWNLOAD": return Logs.downloading
        case "LIDAR": return Lidar.active                     // setNavDot('lidar')
        case "ROS": return Ros.active                         // setNavDot('ros')
        }
        return false
    }
    function groupBusy(items) { return items.some(it => busy(it[0])) }

    // Groups folded as they were left (setup-nav-collapsed, Electron's group names)
    property var collapsed: { try { return JSON.parse(Gcs.setting("setup-nav-collapsed", "{}")) || {} } catch (e) { return {} } }
    function setCollapsed(key, fold) {
        const c = Object.assign({}, collapsed)
        c[key] = fold
        collapsed = c
        Gcs.setSetting("setup-nav-collapsed", JSON.stringify(c))
    }

    readonly property var groups: [
        { name: "COMMS", items: [["CONNECTION", "Vehicle link · Serial / UDP / TCP"], ["CELLULAR LINK", "LTEtelem module · relay · E2E AES-256"],
                                 ["TELEMETRY FORWARD", "MAVLink mirror · UDP / Serial / LTM"], ["RTK / GPS INJECT", "NTRIP caster · Serial base"]] },
        { name: "VEHICLE SETUP", items: [["ACCELEROMETER", "6 positions · Level · Simple"], ["COMPASS", "Onboard mag cal · 3D sphere"],
                                         ["GYRO / BARO", "Gyro offsets · Ground pressure"], ["RADIO CAL", "RC input range"],
                                         ["JOYSTICK / RC", "Gamepad override"], ["FLIGHT MODES", "Mode switch mapping"],
                                         ["FAILSAFE", "Battery · RC · GCS"], ["SERVO / RELAY", "Output functions"]] },
        { name: "TUNING", items: [["PID TUNING", "Rate & attitude gains"], ["EXT TUNING", "Extended tuning"],
                                  ["VIBRATIONS", "IMU vibration & clipping"], ["PARAMETERS", "Full parameter list"]] },
        { name: "TOOLS", items: [["LOG DOWNLOAD", "SD card logs · DataFlash .bin"], ["SIMULATION", "SITL launcher"],
                                 ["LIDAR", "Livox Mid-360 · point cloud"], ["ROS", "rosbridge · averaged surface"]] }
    ]

    // ── Vertical navigation ──
    Rectangle {
        id: nav
        width: 181
        height: parent.height
        color: Theme.bg
        Flickable {
            anchors.fill: parent
            anchors.rightMargin: 16
            contentHeight: navCol.implicitHeight
            clip: true
            ScrollBar.vertical: ScrollBar { }
            Column {
                id: navCol
                width: 165
                topPadding: 8
                Repeater {
                    model: page.groups
                    delegate: Column {
                        id: grp
                        required property var modelData
                        readonly property string key: ({ "COMMS": "comms", "VEHICLE SETUP": "vehicle", "TUNING": "tuning", "TOOLS": "tools" })[modelData.name]
                        property bool open: !page.collapsed[key]
                        // All of its entries gone: dimmed with them
                        opacity: modelData.items.every(it => page.reason(it[0]).length > 0) ? 0.4 : 1
                        width: 165
                        Item {
                            width: parent.width
                            height: 28
                            Row {
                                x: 16
                                anchors.verticalCenter: parent.verticalCenter
                                spacing: 6
                                Text { text: "▾"; rotation: grp.open ? 0 : -90; color: Theme.groupAmber; font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.Bold }
                                Text { text: Theme.tr(grp.modelData.name); color: Theme.groupAmber; font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.Bold; font.letterSpacing: 2 }
                            }
                            StatusDot { anchors.right: parent.right; anchors.rightMargin: 4; visible: page.groupBusy(grp.modelData.items) }
                            MouseArea { anchors.fill: parent; onClicked: page.setCollapsed(grp.key, grp.open) }
                        }
                        Repeater {
                            model: grp.open ? grp.modelData.items : []
                            delegate: Rectangle {
                                required property var modelData
                                readonly property bool active: page.current === modelData[0]
                                readonly property string why: page.reason(modelData[0])
                                opacity: why.length > 0 ? 0.4 : 1
                                width: 165
                                height: navTitle.implicitHeight + navDesc.implicitHeight + 14
                                color: active ? Qt.rgba(0, 210 / 255, 1, 0.06) : (navMouse.containsMouse ? Qt.rgba(1, 1, 1, 0.03) : "transparent")
                                Rectangle { width: 2.67; height: parent.height; color: parent.active ? Theme.accentCyan : "transparent" }
                                Text {
                                    id: navTitle
                                    x: 19
                                    y: 7
                                    width: 130
                                    text: Theme.tr(parent.modelData[0])
                                    color: parent.active ? Theme.accentCyan : (navMouse.containsMouse ? Theme.textMutedBright : Theme.textDim)
                                    font.family: Theme.fontUi
                                    font.pixelSize: Theme.fsSm
                                    font.weight: Font.DemiBold
                                    font.letterSpacing: 1
                                }
                                Text {
                                    id: navDesc
                                    x: 19
                                    anchors.top: navTitle.bottom
                                    width: 130
                                    wrapMode: Text.Wrap
                                    text: Theme.tr(parent.modelData[1])
                                    color: Theme.textDim
                                    font.family: Theme.fontUi
                                    font.pixelSize: Theme.fs2xs
                                    font.letterSpacing: 0.5
                                }
                                StatusDot { anchors.right: parent.right; anchors.rightMargin: 10; visible: page.busy(parent.modelData[0]) }
                                MouseArea { id: navMouse; anchors.fill: parent; hoverEnabled: true; onClicked: if (parent.why.length === 0) page.current = parent.modelData[0] }
                                ToolTip.visible: why.length > 0 && navMouse.containsMouse
                                ToolTip.text: Theme.tr(why)
                                ToolTip.delay: 400
                            }
                        }
                    }
                }
            }
        }
    }

    // ── Content ──
    Item {
        id: content
        x: nav.width
        width: parent.width - nav.width
        height: parent.height

        Row {
            visible: page.current === "CONNECTION"
            x: 86
            y: 24
            spacing: 21
            ConnectionPanel { }
            SetupPanel {
                title: "FIRMWARE INFO"
                Repeater {
                    model: [["AUTOPILOT", Vehicle.connected ? Vehicle.autopilotName : "--"],
                            ["VEHICLE TYPE", Vehicle.connected ? Vehicle.vehicleTypeName : "--"],
                            ["SYSTEM ID", Vehicle.connected ? String(Vehicle.systemId) : "--"]]
                    delegate: Column {
                        required property var modelData
                        width: 320
                        spacing: 5
                        bottomPadding: 9
                        CfgLabel { text: Theme.tr(parent.modelData[0]) }
                        Text { text: Theme.tr(parent.modelData[1]); color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 12 }
                    }
                }
            }
        }

        AccelCalPage {
            visible: page.current === "ACCELEROMETER"
            anchors.fill: parent
        }
        CompassCalPage {
            visible: page.current === "COMPASS"
            anchors.fill: parent
        }
        SensorCalPage {
            visible: page.current === "GYRO / BARO"
            anchors.fill: parent
        }

        RtkPage {
            visible: page.current === "RTK / GPS INJECT"
            anchors.fill: parent
        }
        TelForwardPage {
            visible: page.current === "TELEMETRY FORWARD"
            anchors.fill: parent
        }
        CellularLinkPage {
            visible: page.current === "CELLULAR LINK"
            anchors.fill: parent
        }
        JoystickPage {
            visible: page.current === "JOYSTICK / RC"
            anchors.fill: parent
        }
        FlightModesPage { visible: page.current === "FLIGHT MODES"; anchors.fill: parent }
        FailsafePage { visible: page.current === "FAILSAFE"; anchors.fill: parent }
        PidTuningPage { visible: page.current === "PID TUNING"; anchors.fill: parent }
        ExtTuningPage { visible: page.current === "EXT TUNING"; anchors.fill: parent }
        RadioCalPage { visible: page.current === "RADIO CAL"; anchors.fill: parent }
        VibrationPage { visible: page.current === "VIBRATIONS"; anchors.fill: parent }
        ServoRelayPage { visible: page.current === "SERVO / RELAY"; anchors.fill: parent }
        LidarPage { visible: page.current === "LIDAR"; anchors.fill: parent }
        RosPage { visible: page.current === "ROS"; anchors.fill: parent }
        SimulationPage { visible: page.current === "SIMULATION"; anchors.fill: parent }
        LogDownloadPage {
            visible: page.current === "LOG DOWNLOAD"
            x: 24; y: 24
            width: parent.width - 48
            height: parent.height - 48
        }

        ParametersPage {
            visible: page.current === "PARAMETERS"
            x: 24; y: 24
            width: parent.width - 48
            height: parent.height - 48
        }
    }
}
