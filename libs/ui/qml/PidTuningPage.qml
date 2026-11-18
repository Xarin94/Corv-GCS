// SETUP → PID TUNING (#subtab-pid-tuning, initConfigTuningTab): rate and
// angle gains and the primary configuration, the panels of the connected
// vehicle class only (all of them until a HEARTBEAT names it). READ asks only
// the parameters on screen; WRITE ALL writes every value shown.
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: col.implicitHeight + 48
    clip: true
    ScrollBar.vertical: ScrollBar { }

    readonly property var panels: [
        { title: "RATE PID", vehicles: ["Copter", "Sub"], groups: [
            { name: "ROLL", fields: [["P", "ATC_RAT_RLL_P"], ["I", "ATC_RAT_RLL_I"], ["D", "ATC_RAT_RLL_D"]] },
            { name: "PITCH", fields: [["P", "ATC_RAT_PIT_P"], ["I", "ATC_RAT_PIT_I"], ["D", "ATC_RAT_PIT_D"]] },
            { name: "YAW", fields: [["P", "ATC_RAT_YAW_P"], ["I", "ATC_RAT_YAW_I"], ["D", "ATC_RAT_YAW_D"]] }] },
        { title: "ANGLE P", vehicles: ["Copter", "Sub"], groups: [
            { name: "", fields: [["ROLL", "ATC_ANG_RLL_P"], ["PITCH", "ATC_ANG_PIT_P"], ["YAW", "ATC_ANG_YAW_P"]] }] },
        { title: "RATE PID", vehicles: ["Plane"], groups: [
            { name: "ROLL", fields: [["P", "RLL_RATE_P"], ["I", "RLL_RATE_I"], ["D", "RLL_RATE_D"], ["FF", "RLL_RATE_FF"]] },
            { name: "PITCH", fields: [["P", "PTCH_RATE_P"], ["I", "PTCH_RATE_I"], ["D", "PTCH_RATE_D"], ["FF", "PTCH_RATE_FF"]] },
            { name: "YAW", fields: [["DAMP", "YAW2SRV_DAMP"], ["INT", "YAW2SRV_INT"], ["SLIP", "YAW2SRV_SLIP"], ["RLL", "YAW2SRV_RLL"]] }] },
        { title: "ATTITUDE LIMITS", vehicles: ["Plane"], groups: [
            { name: "", fields: [["RLL TCONST", "RLL2SRV_TCONST"], ["PTCH TCONST", "PTCH2SRV_TCONST"], ["RLL RMAX", "RLL2SRV_RMAX"],
                                 ["PTCH RMAX UP", "PTCH2SRV_RMAX_UP"], ["PTCH RMAX DN", "PTCH2SRV_RMAX_DN"], ["PTCH2SRV RLL", "PTCH2SRV_RLL"]] }] },
        { title: "TECS — SPEED / HEIGHT", vehicles: ["Plane"], groups: [
            { name: "", fields: [["CLMB MAX", "TECS_CLMB_MAX"], ["SINK MIN", "TECS_SINK_MIN"], ["SINK MAX", "TECS_SINK_MAX"],
                                 ["TIME CONST", "TECS_TIME_CONST"], ["PTCH DAMP", "TECS_PTCH_DAMP"], ["THR DAMP", "TECS_THR_DAMP"],
                                 ["INTEG GAIN", "TECS_INTEG_GAIN"], ["SPD WEIGHT", "TECS_SPDWEIGHT"], ["PITCH MAX", "TECS_PITCH_MAX"],
                                 ["PITCH MIN", "TECS_PITCH_MIN"], ["RLL2THR", "TECS_RLL2THR"], ["VERT ACC", "TECS_VERT_ACC"]] }] },
        { title: "PRIMARY CONFIG", vehicles: ["Copter", "Sub"], groups: [
            { name: "", fields: [["THR HOVER", "MOT_THST_HOVER"], ["SPIN ARM", "MOT_SPIN_ARM"], ["SPIN MIN", "MOT_SPIN_MIN"],
                                 ["CLIMB P", "PSC_ACCZ_P"], ["CLIMB I", "PSC_ACCZ_I"], ["RC FEEL", "ATC_INPUT_TC"],
                                 ["ANGLE MAX", "ANGLE_MAX"], ["WP SPEED", "WPNAV_SPEED"], ["RTL ALT", "RTL_ALT"]] }] },
        { title: "PRIMARY CONFIG", vehicles: ["Plane"], groups: [
            { name: "", fields: [["CRUISE ARSPD", "TRIM_ARSPD_CM"], ["ARSPD MIN", "ARSPD_FBW_MIN"], ["ARSPD MAX", "ARSPD_FBW_MAX"],
                                 ["ROLL LIMIT", "LIM_ROLL_CD"], ["PITCH MAX", "LIM_PITCH_MAX"], ["PITCH MIN", "LIM_PITCH_MIN"],
                                 ["TRIM THR", "TRIM_THROTTLE"], ["THR MIN", "THR_MIN"], ["THR MAX", "THR_MAX"]] }] }
    ]
    function applies(p) { return SetupCfg.vehicleClass === "" || p.vehicles.indexOf(SetupCfg.vehicleClass) >= 0 }
    property var fields: []   // every ParamField on the page, registered as it is created

    Column {
        id: col
        x: 24; y: 24
        width: page.width - 48
        spacing: 16
        Row {
            spacing: 6
            GcsButton {
                width: 180; height: 29
                text: "READ FROM VEHICLE"
                tip: "Read every value on this page from the vehicle"
                onClicked: SetupCfg.readParams(page.fields.filter(f => f.shown).map(f => f.param), "PID TUNING")
            }
            GcsButton {
                width: 140; height: 29
                primary: true
                text: "WRITE ALL"
                tip: "Write every value on this page to the vehicle"
                onClicked: {
                    const v = {}
                    for (const f of page.fields) if (f.shown && f.text !== "") v[f.param] = f.text
                    SetupCfg.writeParams(v, "PID TUNING", "%1 parameters written")
                }
            }
            Text {
                anchors.verticalCenter: parent.verticalCenter
                leftPadding: 10
                text: SetupCfg.status["PID TUNING"] ? SetupCfg.status["PID TUNING"].text : ""
                color: SetupCfg.status["PID TUNING"] && SetupCfg.status["PID TUNING"].ok ? Theme.accentGreen : Theme.textDim
                font.family: Theme.fontUi
                font.pixelSize: Theme.fsSm
            }
        }
        Flow {
            width: parent.width
            spacing: 21
            Repeater {
                model: page.panels
                delegate: SetupPanel {
                    id: pnl
                    required property var modelData
                    visible: page.applies(modelData)
                    title: modelData.title
                    Repeater {
                        model: pnl.modelData.groups
                        delegate: Column {
                            id: grp
                            required property var modelData
                            width: parent.width
                            spacing: 6
                            Text {
                                visible: grp.modelData.name !== ""
                                text: grp.modelData.name
                                color: Theme.groupAmber
                                font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.Bold; font.letterSpacing: 2
                            }
                            Repeater {
                                model: grp.modelData.fields
                                delegate: ParamField {
                                    required property var modelData
                                    readonly property bool shown: pnl.visible
                                    label: modelData[0]
                                    param: modelData[1]
                                    Component.onCompleted: page.fields.push(this)
                                    Component.onDestruction: { const i = page.fields.indexOf(this); if (i >= 0) page.fields.splice(i, 1) }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
