// SETUP → TOOLS → ROS (#subtab-ros of index.html, RosController.js): the
// rosbridge link, the topic, the sensor profile and the surfaces; the mount
// with its preview, the range window and spike filter, the time of the
// points, the vertical reference and the mesh. The logic is in RosController
// (singleton Ros).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentWidth: width
    contentHeight: flow.implicitHeight + 48
    clip: true
    ScrollBar.vertical: ScrollBar { }

    readonly property var c: Ros.cfg

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
        onCommitted: (t) => Ros.setField(key, t)
        ToolTip.text: Theme.tr(tip)
        ToolTip.visible: tip.length > 0 && hovered
        ToolTip.delay: 600
    }

    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21

        SetupPanel {
            title: "ROS · ROSBRIDGE"
            Field {
                label: "SURFACE FROM ROS"
                CfgCheck {
                    width: 300
                    text: "Connect to rosbridge and build the averaged surface mesh"
                    checked: Ros.enabled
                    onToggled: (on) => { Ros.setEnabled(on); checked = Qt.binding(() => Ros.enabled) }
                }
            }
            Field {
                label: "ROSBRIDGE URL / TRANSPORT"
                CfgInput { width: 210; text: page.c.url; placeholderText: "ws://companion:9090"; onCommitted: (t) => Ros.setField("url", t) }
                CfgSelect {
                    width: 90
                    tip: "CBOR sends binary arrays as they are; JSON inflates them by a third (base64)"
                    options: [{ value: "cbor", label: "CBOR" }, { value: "json", label: "JSON" }]
                    value: page.c.transport
                    onPicked: (v) => Ros.setField("transport", v)
                }
            }
            Field {
                label: "TOPIC"
                CfgSelect {
                    width: 236
                    options: Ros.topicOptions
                    value: page.c.topic || ""
                    onPicked: (v) => Ros.selectTopic(v)
                    ToolTip.text: Theme.tr(Ros.topicTip)
                    ToolTip.visible: Ros.topicTip.length > 0 && hovered
                    ToolTip.delay: 600
                }
                GcsButton { width: 64; height: 25; text: "REFRESH"; tip: "Ask rosapi for the topics again"; onClicked: Ros.refreshTopics() }
            }
            Field {
                label: "SENSOR"
                CfgSelect {
                    width: 306
                    tip: "Choosing one fills in its usual mount, range, sampling and cell size"
                    options: [{ value: "aerial", label: "Aerial LiDAR · terrain under the aircraft" },
                              { value: "seabed", label: "Sea bed · echo sounder or sonar on a boat / ROV" },
                              { value: "sector", label: "Sector sonar · mechanical scanning head, one beam per ping" }]
                    value: page.c.profile
                    onPicked: (v) => Ros.setField("profile", v)
                }
            }
            Field {
                label: "SURFACES"
                CfgSelect {
                    width: 306
                    tip: "A height per cell holds one surface: right for terrain and sea bed. In a cave the returns are fused into a 3D volume and meshed where it meets the rock"
                    options: [{ value: "floor", label: "Floor only (terrain, sea bed)" },
                              { value: "cave", label: "3D: caves, shafts, overhangs (signed-distance volume)" }]
                    value: page.c.layers
                    onPicked: (v) => Ros.setField("layers", v)
                }
            }
            Field {
                label: "POINTS FRAME"
                CfgSelect {
                    width: 306
                    options: [{ value: "auto", label: "Auto (from header.frame_id)" },
                              { value: "sensor", label: "Sensor on the vehicle · placed with the telemetry pose" },
                              { value: "world", label: "World · ENU from the EKF origin (map, odom)" }]
                    value: page.c.frame
                    onPicked: (v) => Ros.setField("frame", v)
                }
            }
            Field {
                label: "MAX RATE (Hz) / POINTS PER MESSAGE"
                Num { key: "rateHz"; tip: "rosbridge sends at most this many messages a second (throttle_rate), only the newest" }
                Num { key: "maxPoints"; tip: "Points sampled from each message; the rest are not used" }
            }
            GcsButton { width: 157; height: 29; text: "CLEAR SURFACE"; onClicked: Ros.clearSurface() }
            Field {
                label: "STATUS"
                Text {
                    width: 320
                    text: Theme.tr(Ros.statusText)
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
                text: qsTr("sensor_msgs PointCloud2, LaserScan and Range, ROS 1 or ROS 2, through rosbridge_server with rosapi (ros2 launch rosbridge_server rosbridge_websocket_launch.xml). Points are not kept: each message moves the mean height of the cells it falls in.")
            }
        }

        SetupPanel {
            title: "MOUNT & MESH"
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
                        width: 306
                        options: [{ value: "", label: "Preset…" }, { value: "0,-90,0", label: "Looking down (x down): echo sounder, altimeter, push-broom scanner" },
                                  { value: "180,0,0", label: "Inverted under the belly (Livox Mid-360)" }, { value: "0,0,0", label: "Upright, x forward" },
                                  { value: "0,-45,0", label: "Upright, nose-down 45°" }]
                        value: ""
                        onPicked: (v) => { if (v) Ros.applyPreset(v); currentIndex = 0 }
                    }
                }
            }
            Field {
                label: "LEVER ARM IMU → SENSOR (X FWD / Y RIGHT / Z DOWN, m)"
                Num { id: lX; key: "leverX"; tip: "X forward" }
                Num { id: lY; key: "leverY"; tip: "Y right" }
                Num { id: lZ; key: "leverZ"; tip: "Z down (a transducer's draft)" }
            }
            Field {
                label: "MOUNT PREVIEW"
                Rectangle {
                    width: 320; height: 210
                    radius: 4
                    color: Qt.rgba(0, 0, 0, 0.28)
                    border.color: Qt.rgba(0, 210 / 255, 1, 0.12)
                    border.width: Theme.hairline
                    HoverTip { text: "The vehicle (nose forward, fin on top), the sensor at its lever arm with its axes (X red, Y green, Z blue) and the zone it collects from" }
                    MountPreview {
                        anchors.fill: parent
                        ink: Theme.textMutedBright
                        mount: [parseFloat(mRoll.text) || 0, parseFloat(mPitch.text) || 0, parseFloat(mYaw.text) || 0]
                        lever: [parseFloat(lX.text) || 0, parseFloat(lY.text) || 0, parseFloat(lZ.text) || 0]
                        zone: Ros.previewZone
                        caption: Ros.previewCaption
                    }
                }
            }
            Field {
                label: "RANGE WINDOW (m) / SPIKE FILTER"
                Num { key: "minRange"; width: 70; tip: "Minimum range: closer returns (the airframe, the hull) are dropped" }
                Num { key: "maxRange"; width: 70; tip: "Maximum range" }
                CfgSelect {
                    width: 154
                    tip: "Drops returns far off their neighbours along the scan (aeration under the hull, fish, particles); only on ordered scans (a sonar's beams, a LaserScan, a lidar's rings), never on clouds without order"
                    options: [{ value: "on", label: "Spikes dropped" }, { value: "off", label: "Every return" }]
                    value: page.c.despike
                    onPicked: (v) => Ros.setField("despike", v)
                }
            }
            Field {
                label: "TIME OF THE POINTS / LAG (ms)"
                CfgSelect {
                    width: 210
                    tip: "The pose is projected to this time with the gyro rates and the velocity"
                    options: [{ value: "arrival", label: "Arrival − lag" }, { value: "stamp", label: "header.stamp + lag (clocks synced)" }]
                    value: page.c.timeBase
                    onPicked: (v) => Ros.setField("timeBase", v)
                }
                Num { key: "lagMs"; tip: "Arrival: how much later than the telemetry the points reach the GCS. Stamp: how late the telemetry reaches the GCS" }
            }
            Field {
                label: "VERTICAL REFERENCE"
                CfgSelect {
                    width: 306
                    tip: "The height the sensor is placed at: the vehicle's altitude, or — for a boat — the water surface at home (the altitude it was armed at), so the EKF's barometric drift does not lift or sink the bed"
                    options: [{ value: "vehicle", label: "Vehicle altitude (EKF)" }, { value: "water", label: "Water surface at home · a boat" }]
                    value: page.c.vertical
                    onPicked: (v) => Ros.setField("vertical", v)
                }
            }
            Field {
                label: "MIN CELL (m) / MIN SAMPLES / MEMORY (samples) / MAX TILES"
                Num { key: "cell"; width: 72; tip: "Finest cell: the mesh is this fine where the data allow; changing it clears the surface" }
                Num { key: "minSamples"; width: 72; tip: "Samples a cell needs to be drawn at its own size; with fewer the mesh there is drawn coarser (60 cm, 1.2 m, 2.4 m, 4.8 m)" }
                Num { key: "memory"; width: 72; tip: "A cell averages over at most this many samples (messages), then follows a moving average" }
                Num { key: "maxTiles"; width: 72; tip: "Tiles of 30 × 30 cells kept; past this the farthest from the vehicle are dropped (2048 ≈ 16 ha at 30 cm)" }
            }
            Field {
                label: "FILL HOLES UP TO"
                CfgSelect {
                    width: 306
                    tip: "Empty cells between data are interpolated from the cells around them up to this size (at most 16 cells), never past the edge of the data or across a jump; the coverage of a planned area is counted the same way"
                    options: [{ value: "0", label: "4 cells (1.2 m at 30 cm)" }, { value: "2", label: "2 m" }, { value: "4", label: "4 m" },
                              { value: "8", label: "8 m · sensors that sample in lines (sector sonar, echo sounder)" }]
                    value: String(page.c.fillM)
                    onPicked: (v) => Ros.setField("fillM", v)
                }
            }
            Field {
                label: "TRIANGLE FILL OPACITY (%)"
                Column {
                    spacing: 6
                    Num { key: "fillPct"; tip: "0 = grey lines only" }
                    CfgCheck {
                        width: 300
                        text: "Draw through the terrain mesh (SRTM is ±5 m; a lake's surface covers the sea bed)"
                        checked: !!page.c.overTerrain
                        onToggled: (on) => Ros.setField("overTerrain", on)
                    }
                }
            }
            CfgNote {
                opacity: 0.6
                font.pixelSize: 11
                text: qsTr("Mount angles are ZYX Euler of the sensor frame (REP-103: x forward, y left, z up; Range and LaserScan measure along x) relative to the vehicle body (FRD). The surface is kept in tiles of 30×30 cells, made where data arrive; the mesh is as fine as MIN CELL where cells have MIN SAMPLES, coarser where they do not. The pose is projected to the time of the points with the gyro rates and the velocity.")
            }
        }
    }
}
