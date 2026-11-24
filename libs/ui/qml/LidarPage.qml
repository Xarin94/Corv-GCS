// SETUP → TOOLS → LIDAR (#subtab-lidar of index.html, LidarController.js):
// the Livox Mid-360 link, the navigation-quality gate, recording and the
// status; the mount relative to the autopilot IMU with its preview, the
// filters and the display. The logic is in LidarController (singleton Lidar).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentWidth: width
    contentHeight: flow.implicitHeight + 48
    clip: true
    ScrollBar.vertical: ScrollBar { }

    readonly property var c: Lidar.cfg

    // A .cfg-row of the Electron page: the label over its controls
    component Field: Column {
        property string label
        default property alias content: slot.data
        width: parent ? parent.width : 320
        spacing: 5
        CfgLabel { width: parent.width; text: Theme.tr(parent.label); wrapMode: Text.Wrap; visible: text.length > 0 }
        Row { id: slot; spacing: 6 }
    }
    component Num: CfgInput {
        property string key
        property string tip
        numeric: true
        width: 96
        text: String(page.c[key])
        onCommitted: (t) => Lidar.setField(key, t)
        HoverTip { text: parent.tip }
    }

    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21

        SetupPanel {
            title: "LIDAR · LIVOX MID-360"
            Field {
                label: "POINT CLOUD"
                CfgCheck {
                    width: 300
                    text: "Connect to the LiDAR and draw the 3D map"
                    checked: Lidar.enabled
                    onToggled: (on) => { Lidar.setEnabled(on); checked = Qt.binding(() => Lidar.enabled) }
                }
            }
            Field {
                label: "LIDAR IP"
                CfgInput { width: 160; text: page.c.lidarIp; placeholderText: "192.168.1.1XX (XX = S/N)"; onCommitted: (t) => Lidar.setField("lidarIp", t) }
            }
            Field {
                label: "HOST IP (POINT DESTINATION)"
                CfgSelect {
                    width: 300
                    options: Lidar.hostOptions
                    value: page.c.hostIp
                    onPicked: (v) => Lidar.setField("hostIp", v)
                }
            }
            Field {
                label: "POINT FORMAT"
                CfgSelect {
                    width: 300
                    options: [{ value: "1", label: "Cartesian 32-bit (mm) · ~24 Mbit/s" }, { value: "2", label: "Cartesian 16-bit (cm) · ~14 Mbit/s" }]
                    value: String(page.c.dataType)
                    onPicked: (v) => Lidar.setField("dataType", v)
                }
            }
            Field {
                label: "TELEMETRY LAG (ms)"
                Num { key: "lagMs"; tip: "How much later than the point packets the telemetry reaches the GCS" }
            }
            Field {
                label: "ACCUMULATE ONLY WITH"
                Column {
                    spacing: 6
                    CfgSelect {
                        width: 300
                        options: [{ value: "3", label: "GPS 3D fix or better" }, { value: "4", label: "DGPS or better" },
                                  { value: "5", label: "RTK float or better" }, { value: "6", label: "RTK fixed" }]
                        value: String(page.c.minFix)
                        onPicked: (v) => Lidar.setField("minFix", v)
                    }
                    Row {
                        spacing: 6
                        CfgLabel { anchors.verticalCenter: parent.verticalCenter; text: qsTr("SATS ≥") }
                        Num { key: "minSats"; width: 70; tip: "Minimum satellites" }
                        CfgLabel { anchors.verticalCenter: parent.verticalCenter; text: qsTr("HDOP ≤") }
                        Num { key: "maxHdop"; width: 70; tip: "Maximum HDOP" }
                    }
                    CfgCheck {
                        width: 300
                        text: "EKF healthy (position flags + variances)"
                        checked: !!page.c.requireEkf
                        onToggled: (on) => Lidar.setField("requireEkf", on)
                    }
                }
            }
            Field {
                label: "LIVE POINTS (NOT GEOREFERENCED) FADE AFTER (s)"
                Num { key: "liveSeconds"; ToolTip.text: qsTr("Without a usable position the scan is still shown around the aircraft, then fades"); ToolTip.visible: hovered; ToolTip.delay: 600 }
            }
            Field {
                label: "RECORD RAW POINTS (.PLY)"
                CfgCheck {
                    width: 300
                    text: "Stream every point to data/lidar/raw-*.ply"
                    checked: !!page.c.recordRaw
                    onToggled: (on) => Lidar.setField("recordRaw", on)
                }
            }
            Row {
                spacing: 6
                GcsButton { width: 157; height: 29; text: "CLEAR MAP"; onClicked: Lidar.clearMap() }
                GcsButton { width: 157; height: 29; text: "SAVE MAP .PLY"; onClicked: Lidar.saveMap() }
            }
            Field {
                label: "STATUS"
                Text {
                    width: 320
                    text: Theme.tr(Lidar.statusText)
                    textFormat: Text.StyledText
                    wrapMode: Text.Wrap
                    color: Theme.textMutedBright
                    font.family: Theme.fontData; font.pixelSize: 11
                    lineHeight: 1.5
                }
            }
            CfgNote {
                opacity: 0.6
                font.pixelSize: 11
                text: qsTr("Livox SDK2 over UDP: the LiDAR must reach this host on ports 56101 / 56201 / 56301 / 56401 through the LAN bridge. Raise SR_POSITION and SR_EXTRA1 to 10 Hz for a sharper map.")
            }
        }

        SetupPanel {
            title: "MOUNT & FILTERS"
            Field {
                label: "MOUNT ATTITUDE VS IMU (ROLL / PITCH / YAW °)"
                Column {
                    spacing: 6
                    Row {
                        spacing: 6
                        Num { id: mRoll; key: "mountRoll" }
                        Num { id: mPitch; key: "mountPitch" }
                        Num { id: mYaw; key: "mountYaw" }
                    }
                    CfgSelect {
                        width: 300
                        options: [{ value: "", label: "Preset…" }, { value: "0,0,0", label: "Upright, connector aft (Z up)" },
                                  { value: "180,0,0", label: "Inverted under the belly (Z down)" }, { value: "180,0,90", label: "Inverted, rotated 90°" },
                                  { value: "0,-45,0", label: "Upright, nose-down 45°" }, { value: "0,-90,0", label: "Forward-looking (Z fwd)" }]
                        value: ""
                        onPicked: (v) => { if (v) Lidar.applyPreset(v); currentIndex = 0 }
                    }
                }
            }
            Field {
                label: "LEVER ARM IMU → LIDAR (X FWD / Y RIGHT / Z DOWN, m)"
                Num { id: lX; key: "leverX"; tip: "X forward" }
                Num { id: lY; key: "leverY"; tip: "Y right" }
                Num { id: lZ; key: "leverZ"; tip: "Z down" }
            }
            Field {
                label: "MOUNT PREVIEW"
                Rectangle {
                    width: 320; height: 210
                    radius: 4
                    color: Qt.rgba(0, 0, 0, 0.28)
                    border.color: Qt.rgba(0, 210 / 255, 1, 0.12)
                    border.width: Theme.hairline
                    HoverTip { text: "The vehicle (nose forward, fin on top), the LiDAR at its lever arm with its axes (X red, Y green, Z blue) and the zone it scans" }
                    // From the inputs as they are typed, before they are applied
                    MountPreview {
                        anchors.fill: parent
                        ink: Theme.textMutedBright
                        mount: [parseFloat(mRoll.text) || 0, parseFloat(mPitch.text) || 0, parseFloat(mYaw.text) || 0]
                        lever: [parseFloat(lX.text) || 0, parseFloat(lY.text) || 0, parseFloat(lZ.text) || 0]
                        zone: ({ type: "band", elMin: -7, elMax: 52 })
                        caption: "Livox Mid-360 · 360° × −7…+52°"
                    }
                }
            }
            Field {
                label: "MIN RANGE — DISCARD CLOSER THAN (m)"
                Num { key: "minRange"; tip: "Returns closer than this are false echoes from the airframe" }
                CfgNote { width: 200; anchors.verticalCenter: parent.verticalCenter; text: qsTr("rudder, landing gear, antennas in the field of view") }
            }
            Field {
                label: "MAX RANGE (m)"
                Num { key: "maxRange"; tip: "Maximum range" }
            }
            Field {
                label: "VOXEL SIZE (m) / MAX POINTS"
                Num { key: "voxel"; tip: "One point kept per voxel" }
                Num { key: "maxPoints"; width: 120; tip: "Map capacity" }
            }
            Field {
                label: "NOISE FILTER"
                CfgCheck {
                    width: 300
                    text: "Drop points the LiDAR tags as noise"
                    checked: !!page.c.dropNoise
                    onToggled: (on) => Lidar.setField("dropNoise", on)
                }
            }
            Field {
                label: "COLOR BY / POINT SIZE (px)"
                Column {
                    spacing: 6
                    Row {
                        spacing: 6
                        CfgSelect {
                            width: 200
                            options: [{ value: "height", label: "Height" }, { value: "intensity", label: "Reflectivity" }]
                            value: page.c.colorMode
                            onPicked: (v) => Lidar.setField("colorMode", v)
                        }
                        Num { key: "pointSize"; width: 70; tip: "Point size in pixels" }
                    }
                    CfgCheck {
                        width: 300
                        text: "Draw through the terrain mesh (SRTM is coarser than the LiDAR)"
                        checked: !!page.c.overTerrain
                        onToggled: (on) => Lidar.setField("overTerrain", on)
                    }
                }
            }
            CfgNote {
                opacity: 0.6
                font.pixelSize: 11
                text: qsTr("Angles are ZYX Euler of the LiDAR body relative to the aircraft body (FRD). The Mid-360 sees 360° × (−7°…+52°): mounted flat under the belly it is blind straight down, tilt it to close the gap.")
            }
        }
    }
}
