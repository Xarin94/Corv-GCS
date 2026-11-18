#include "mission/RouteModel.h"

#include <QDateTime>
#include <QJsonDocument>
#include <QMutex>
#include <atomic>
#include <cmath>

namespace corv::mission {

namespace {

// The catalogues of RouteModel.js, verbatim. `fields` drive the inspector;
// `points` says how many base points the tool collects on the map.
const char *kCatalog = R"JSON({
"segments": {
  "waypoint": {
    "label": "Waypoint", "short": "WP", "points": 1, "color": "#44ff44", "hotkey": "W",
    "hint": "Click on the map to add waypoints — Esc to stop",
    "defaults": { "alt": null, "speed": 0, "turn": "route", "delay": 0, "acceptRadius": 0 },
    "fields": [
      { "key": "alt", "label": "Altitude", "unit": "m", "type": "number", "min": -500, "max": 10000, "step": 1, "nullable": true, "placeholder": "route" },
      { "key": "speed", "label": "Speed", "unit": "m/s", "type": "number", "min": 0, "max": 100, "step": 0.5, "zeroLabel": "route" },
      { "key": "turn", "label": "Turn type", "type": "select", "options": [["route", "Route default"], ["straight", "Straight"], ["spline", "Spline"]] },
      { "key": "delay", "label": "Hold", "unit": "s", "type": "number", "min": 0, "max": 3600, "step": 1 },
      { "key": "acceptRadius", "label": "Accept radius", "unit": "m", "type": "number", "min": 0, "max": 500, "step": 1, "zeroLabel": "auto" }
    ]
  },
  "circle": {
    "label": "Circle", "short": "CIRC", "points": 1, "color": "#4488ff", "hotkey": "C",
    "hint": "Click to place the circle centre",
    "defaults": { "alt": null, "radius": 50, "turns": 1, "direction": "cw", "speed": 0 },
    "fields": [
      { "key": "alt", "label": "Altitude", "unit": "m", "type": "number", "min": -500, "max": 10000, "step": 1, "nullable": true, "placeholder": "route" },
      { "key": "radius", "label": "Radius", "unit": "m", "type": "number", "min": 5, "max": 5000, "step": 5 },
      { "key": "turns", "label": "Loops", "type": "number", "min": 1, "max": 100, "step": 1 },
      { "key": "direction", "label": "Direction", "type": "select", "options": [["cw", "Clockwise"], ["ccw", "Counter-clockwise"]] },
      { "key": "speed", "label": "Speed", "unit": "m/s", "type": "number", "min": 0, "max": 100, "step": 0.5, "zeroLabel": "route" }
    ]
  },
  "perimeter": {
    "label": "Perimeter", "short": "PERIM", "points": "poly", "color": "#ffaa00", "hotkey": "P",
    "hint": "Click the vertices — Enter or click the first vertex to close, Esc to cancel",
    "defaults": { "alt": null, "speed": 0, "direction": "cw", "closed": true },
    "fields": [
      { "key": "alt", "label": "Altitude", "unit": "m", "type": "number", "min": -500, "max": 10000, "step": 1, "nullable": true, "placeholder": "route" },
      { "key": "speed", "label": "Speed", "unit": "m/s", "type": "number", "min": 0, "max": 100, "step": 0.5, "zeroLabel": "route" },
      { "key": "direction", "label": "Direction", "type": "select", "options": [["cw", "As drawn"], ["ccw", "Reversed"]] },
      { "key": "closed", "label": "Return to start", "type": "check" }
    ]
  },
)JSON" R"JSON(  "area": {
    "label": "Area scan", "short": "AREA", "points": "poly", "color": "#ff6600", "hotkey": "A",
    "hint": "Click the vertices of the area — Enter or click the first vertex to close, Esc to cancel",
    "defaults": { "alt": null, "speed": 0, "angle": 0, "spacingMode": "camera", "sideDistance": 30,
                  "sideOverlap": 70, "forwardOverlap": 80, "sensorAperture": 90, "sensorRange": 90, "sensorDistance": 30,
                  "overshoot": 0, "doubleGrid": false, "trigger": true },
    "fields": [
      { "key": "alt", "label": "Altitude", "unit": "m", "type": "number", "min": -500, "max": 10000, "step": 1, "nullable": true, "placeholder": "route" },
      { "key": "speed", "label": "Speed", "unit": "m/s", "type": "number", "min": 0, "max": 100, "step": 0.5, "zeroLabel": "route" },
      { "key": "angle", "label": "Direction", "unit": "°", "type": "number", "min": -180, "max": 180, "step": 1 },
      { "key": "spacingMode", "label": "Lane spacing", "type": "select", "options": [["camera", "From camera"], ["sensor", "From mapping sensor"], ["manual", "Manual"]] },
      { "key": "sideDistance", "label": "Side distance", "unit": "m", "type": "number", "min": 1, "max": 5000, "step": 1, "when": { "spacingMode": "manual" } },
      { "key": "sensorAperture", "label": "Sensor swath angle", "unit": "°", "type": "number", "min": 1, "max": 180, "step": 1, "when": { "spacingMode": "sensor" }, "title": "Across-track aperture of the mapping sensor: an imaging sonar 90°, a multibeam 120–140°, a LiDAR scanning across the track 70°" },
      { "key": "sensorRange", "label": "Sensor range", "unit": "m", "type": "number", "min": 1, "max": 2000, "step": 1, "when": { "spacingMode": "sensor" } },
      { "key": "sensorDistance", "label": "Distance to the surface", "unit": "m", "type": "number", "min": 0.5, "max": 5000, "step": 1, "nullable": true, "placeholder": "altitude", "when": { "spacingMode": "sensor" }, "title": "Expected distance from the sensor to what it maps: the depth for a boat, the height above the bed for a ROV; empty = the segment altitude (an aircraft over the ground)" },
      { "key": "sideOverlap", "label": "Side overlap", "unit": "%", "type": "number", "min": 0, "max": 95, "step": 5, "when": { "spacingMode": ["camera", "sensor"] } },
      { "key": "forwardOverlap", "label": "Forward overlap", "unit": "%", "type": "number", "min": 0, "max": 95, "step": 5, "when": { "spacingMode": ["camera", "manual"] } },
      { "key": "overshoot", "label": "Overshoot", "unit": "m", "type": "number", "min": 0, "max": 500, "step": 5 },
      { "key": "doubleGrid", "label": "Double grid", "type": "check" },
      { "key": "trigger", "label": "Camera trigger by distance", "type": "check", "when": { "spacingMode": ["camera", "manual"] } }
    ]
  },
  "corridor": {
    "label": "Corridor", "short": "CORR", "points": "line", "color": "#cc44ff", "hotkey": "K",
    "hint": "Click along the corridor — Enter to finish, Esc to cancel",
    "defaults": { "alt": null, "speed": 0, "width": 60, "spacingMode": "camera", "sideDistance": 30,
                  "sideOverlap": 70, "forwardOverlap": 80, "trigger": true,
                  "sensorAperture": 90, "sensorRange": 90, "sensorDistance": 30 },
    "fields": [
      { "key": "alt", "label": "Altitude", "unit": "m", "type": "number", "min": -500, "max": 10000, "step": 1, "nullable": true, "placeholder": "route" },
      { "key": "speed", "label": "Speed", "unit": "m/s", "type": "number", "min": 0, "max": 100, "step": 0.5, "zeroLabel": "route" },
      { "key": "width", "label": "Corridor width", "unit": "m", "type": "number", "min": 1, "max": 5000, "step": 5 },
      { "key": "spacingMode", "label": "Lane spacing", "type": "select", "options": [["camera", "From camera"], ["sensor", "From mapping sensor"], ["manual", "Manual"]] },
      { "key": "sideDistance", "label": "Side distance", "unit": "m", "type": "number", "min": 1, "max": 5000, "step": 1, "when": { "spacingMode": "manual" } },
      { "key": "sensorAperture", "label": "Sensor swath angle", "unit": "°", "type": "number", "min": 1, "max": 180, "step": 1, "when": { "spacingMode": "sensor" }, "title": "Across-track aperture of the mapping sensor: an imaging sonar 90°, a multibeam 120–140°, a LiDAR scanning across the track 70°" },
      { "key": "sensorRange", "label": "Sensor range", "unit": "m", "type": "number", "min": 1, "max": 2000, "step": 1, "when": { "spacingMode": "sensor" } },
      { "key": "sensorDistance", "label": "Distance to the surface", "unit": "m", "type": "number", "min": 0.5, "max": 5000, "step": 1, "nullable": true, "placeholder": "altitude", "when": { "spacingMode": "sensor" }, "title": "Expected distance from the sensor to what it maps: the depth for a boat, the height above the bed for a ROV; empty = the segment altitude (an aircraft over the ground)" },
      { "key": "sideOverlap", "label": "Side overlap", "unit": "%", "type": "number", "min": 0, "max": 95, "step": 5, "when": { "spacingMode": ["camera", "sensor"] } },
      { "key": "forwardOverlap", "label": "Forward overlap", "unit": "%", "type": "number", "min": 0, "max": 95, "step": 5, "when": { "spacingMode": ["camera", "manual"] } },
      { "key": "trigger", "label": "Camera trigger by distance", "type": "check", "when": { "spacingMode": ["camera", "manual"] } }
    ]
  },
  "poi": {
    "label": "Point of interest", "short": "POI", "points": 1, "color": "#ff66aa", "hotkey": "I",
    "hint": "Click to place the point the camera should look at",
    "defaults": { "alt": 0, "mode": "set" },
    "fields": [
      { "key": "mode", "label": "Camera", "type": "select", "options": [["set", "Look at this point"], ["clear", "Stop looking (ROI off)"]] },
      { "key": "alt", "label": "Target height", "unit": "m", "type": "number", "min": -500, "max": 10000, "step": 1, "when": { "mode": "set" } }
    ]
  },
  "landing": {
    "label": "Landing", "short": "LAND", "points": 1, "color": "#ff3333", "hotkey": "L",
    "hint": "Click to place the landing point",
    "defaults": { "vtol": false, "abortAlt": 0 },
    "fields": [
      { "key": "vtol", "label": "VTOL landing", "type": "check" },
      { "key": "abortAlt", "label": "Abort altitude", "unit": "m", "type": "number", "min": 0, "max": 1000, "step": 5, "zeroLabel": "off" }
    ]
  }
},
)JSON" R"JSON("actions": {
  "trigger_dist": { "label": "Camera trigger by distance", "short": "TRIG", "cmd": 206, "defaults": { "distance": 20 },
    "fields": [{ "key": "distance", "label": "Every", "unit": "m", "type": "number", "min": 0, "max": 5000, "step": 1, "zeroLabel": "stop" }] },
  "shot": { "label": "Take a photo", "short": "SHOT", "cmd": 203, "defaults": {}, "fields": [] },
  "gimbal": { "label": "Camera attitude", "short": "GIMB", "cmd": 205, "defaults": { "pitch": -90, "yaw": 0 },
    "fields": [
      { "key": "pitch", "label": "Pitch", "unit": "°", "type": "number", "min": -90, "max": 30, "step": 5 },
      { "key": "yaw", "label": "Yaw", "unit": "°", "type": "number", "min": -180, "max": 180, "step": 5 }
    ] },
  "yaw": { "label": "Set heading", "short": "YAW", "cmd": 115, "defaults": { "heading": 0 },
    "fields": [{ "key": "heading", "label": "Heading", "unit": "°", "type": "number", "min": 0, "max": 359, "step": 5 }] },
  "wait": { "label": "Wait", "short": "WAIT", "cmd": 93, "defaults": { "seconds": 5 },
    "fields": [{ "key": "seconds", "label": "Duration", "unit": "s", "type": "number", "min": 0, "max": 3600, "step": 1 }] },
  "speed": { "label": "Change speed", "short": "SPD", "cmd": 178, "defaults": { "speed": 10 },
    "fields": [{ "key": "speed", "label": "Speed", "unit": "m/s", "type": "number", "min": 0.5, "max": 100, "step": 0.5 }] },
  "servo": { "label": "Set servo", "short": "SRV", "cmd": 183, "defaults": { "channel": 9, "pwm": 1500 },
    "fields": [
      { "key": "channel", "label": "Channel", "type": "number", "min": 1, "max": 16, "step": 1 },
      { "key": "pwm", "label": "PWM", "unit": "µs", "type": "number", "min": 800, "max": 2200, "step": 10 }
    ] },
  "relay": { "label": "Set relay", "short": "RLY", "cmd": 181, "defaults": { "relay": 0, "state": 1 },
    "fields": [
      { "key": "relay", "label": "Relay", "type": "number", "min": 0, "max": 5, "step": 1 },
      { "key": "state", "label": "State", "type": "select", "options": [["1", "On"], ["0", "Off"]] }
    ] },
  "raw": { "label": "MAVLink command", "short": "CMD", "cmd": null, "defaults": { "command": 0, "param1": 0, "param2": 0, "param3": 0, "param4": 0 },
    "fields": [
      { "key": "command", "label": "Command", "type": "number", "min": 0, "max": 65535, "step": 1 },
      { "key": "param1", "label": "Param 1", "type": "number", "step": 0.01 },
      { "key": "param2", "label": "Param 2", "type": "number", "step": 0.01 },
      { "key": "param3", "label": "Param 3", "type": "number", "step": 0.01 },
      { "key": "param4", "label": "Param 4", "type": "number", "step": 0.01 }
    ] }
},
)JSON" R"JSON("cameraPresets": {
  "generic":  { "name": "Generic 4:3 (73°)",      "sensorW": 6.17, "sensorH": 4.63, "focal": 4.2,  "imgW": 4000, "imgH": 3000 },
  "mavic3":   { "name": "DJI Mavic 3 (4/3\")",     "sensorW": 17.3, "sensorH": 13.0, "focal": 12.3, "imgW": 5280, "imgH": 3956 },
  "h20wide":  { "name": "DJI Zenmuse H20 wide",   "sensorW": 6.17, "sensorH": 4.55, "focal": 4.5,  "imgW": 4056, "imgH": 3040 },
  "p1_35":    { "name": "DJI Zenmuse P1 · 35 mm", "sensorW": 35.9, "sensorH": 24.0, "focal": 35,   "imgW": 8192, "imgH": 5460 },
  "a6000_20": { "name": "Sony α6000 · 20 mm",     "sensorW": 23.5, "sensorH": 15.6, "focal": 20,   "imgW": 6000, "imgH": 4000 },
  "rx1r2":    { "name": "Sony RX1R II · 35 mm",   "sensorW": 35.9, "sensorH": 24.0, "focal": 35,   "imgW": 7952, "imgH": 5304 },
  "rededge":  { "name": "MicaSense RedEdge",      "sensorW": 4.8,  "sensorH": 3.6,  "focal": 5.5,  "imgW": 1280, "imgH": 960 },
  "custom":   { "name": "Custom" }
},
"cameraOrder": ["generic", "mavic3", "h20wide", "p1_35", "a6000_20", "rx1r2", "rededge", "custom"],
"cameraFields": [
  { "key": "sensorW", "label": "Sensor width", "unit": "mm", "type": "number", "min": 1, "max": 100, "step": 0.1 },
  { "key": "sensorH", "label": "Sensor height", "unit": "mm", "type": "number", "min": 1, "max": 100, "step": 0.1 },
  { "key": "focal", "label": "Focal length", "unit": "mm", "type": "number", "min": 1, "max": 600, "step": 0.1 },
  { "key": "imgW", "label": "Image width", "unit": "px", "type": "number", "min": 100, "max": 20000, "step": 1 },
  { "key": "imgH", "label": "Image height", "unit": "px", "type": "number", "min": 100, "max": 20000, "step": 1 }
],
"radioPresets": {
  "rfd900x":   { "name": "RFD900x · 900 MHz · 1 W",          "freq": 915,  "txPower": 30, "minRssi": -105, "gainGround": 2, "gainAir": 2 },
  "rfd868x":   { "name": "RFD868x · 868 MHz · 500 mW",       "freq": 868,  "txPower": 27, "minRssi": -105, "gainGround": 2, "gainAir": 2 },
  "sik433":    { "name": "SiK telemetry · 433 MHz · 100 mW", "freq": 433,  "txPower": 20, "minRssi": -108, "gainGround": 2, "gainAir": 2 },
  "sik915":    { "name": "SiK telemetry · 915 MHz · 100 mW", "freq": 915,  "txPower": 20, "minRssi": -108, "gainGround": 2, "gainAir": 2 },
  "herelink":  { "name": "Herelink · 2.4 GHz",               "freq": 2400, "txPower": 20, "minRssi": -98,  "gainGround": 5, "gainAir": 2 },
  "pmddl2450": { "name": "Microhard pMDDL2450 · 1 W",        "freq": 2450, "txPower": 30, "minRssi": -96,  "gainGround": 5, "gainAir": 2 },
  "doodle24":  { "name": "Doodle Labs Mesh Rider · 2.4 GHz", "freq": 2450, "txPower": 30, "minRssi": -98,  "gainGround": 5, "gainAir": 2 },
  "elrs900":   { "name": "ExpressLRS · 900 MHz · 1 W",       "freq": 915,  "txPower": 30, "minRssi": -117, "gainGround": 2, "gainAir": 2 },
  "elrs24":    { "name": "ExpressLRS · 2.4 GHz · 250 mW",    "freq": 2440, "txPower": 24, "minRssi": -108, "gainGround": 2, "gainAir": 2 },
  "crossfire": { "name": "TBS Crossfire · 868/915 MHz · 2 W", "freq": 900, "txPower": 33, "minRssi": -120, "gainGround": 2, "gainAir": 2 },
  "custom":    { "name": "Custom" }
},
"radioOrder": ["rfd900x", "rfd868x", "sik433", "sik915", "herelink", "pmddl2450", "doodle24", "elrs900", "elrs24", "crossfire", "custom"],
"radioFields": [
  { "key": "freq", "label": "Frequency", "unit": "MHz", "type": "number", "min": 100, "max": 6000, "step": 1 },
  { "key": "txPower", "label": "TX power", "unit": "dBm", "type": "number", "min": -10, "max": 50, "step": 1 },
  { "key": "gainGround", "label": "Ground antenna gain", "unit": "dBi", "type": "number", "min": -10, "max": 30, "step": 0.5 },
  { "key": "gainAir", "label": "Aircraft antenna gain", "unit": "dBi", "type": "number", "min": -10, "max": 30, "step": 0.5 },
  { "key": "losses", "label": "Cable / system losses", "unit": "dB", "type": "number", "min": 0, "max": 30, "step": 0.5 },
  { "key": "minRssi", "label": "Min RSSI (sensitivity)", "unit": "dBm", "type": "number", "min": -140, "max": 0, "step": 1 },
  { "key": "minMargin", "label": "Min link budget", "unit": "dB", "type": "number", "min": 0, "max": 60, "step": 1, "title": "Safety margin above the sensitivity — less than this is shown in red" },
  { "key": "groundHeight", "label": "Ground antenna height", "unit": "m", "type": "number", "min": 0, "max": 200, "step": 0.5, "title": "Above the ground at the operator position" },
  { "key": "coverageRange", "label": "Coverage radius", "unit": "km", "type": "number", "min": 0.5, "max": 100, "step": 0.5, "title": "Extent of the coverage overlay around the operator" }
],
)JSON" R"JSON("routeParamFields": [
  { "key": "altMode", "label": "Altitude mode", "type": "select", "options": [["agl", "AGL — follow terrain"], ["amsl", "AMSL — constant"], ["rel", "Relative to take-off"]] },
  { "key": "defaultAlt", "label": "Default altitude", "unit": "m", "type": "number", "min": -500, "max": 10000, "step": 5 },
  { "key": "terrainWaypoints", "label": "Terrain-following waypoints", "type": "check", "when": { "altMode": "agl" }, "title": "Off: the vehicle flies straight from each waypoint to the next, every waypoint at its height above the ground under it. On: intermediate waypoints are added so the straight legs stay within the AGL tolerance of the terrain" },
  { "key": "aglTolerance", "label": "AGL tolerance", "unit": "m", "type": "number", "min": 1, "max": 200, "step": 1, "when": { "altMode": "agl", "terrainWaypoints": true } },
  { "key": "defaultSpeed", "label": "Default speed", "unit": "m/s", "type": "number", "min": 0, "max": 100, "step": 0.5, "zeroLabel": "vehicle" },
  { "key": "turnType", "label": "Turn type", "type": "select", "options": [["straight", "Straight"], ["spline", "Spline"]] },
  { "key": "takeoff", "label": "Automatic take-off", "type": "check" },
  { "key": "takeoffAlt", "label": "Take-off altitude", "unit": "m", "type": "number", "min": 1, "max": 1000, "step": 5, "when": { "takeoff": true } },
  { "key": "endAction", "label": "After last segment", "type": "select", "options": [["rtl", "Return to launch"], ["land", "Land in place"], ["none", "Nothing"]] },
  { "key": "maxAgl", "label": "Max altitude AGL", "unit": "m", "type": "number", "min": 0, "max": 10000, "step": 10, "zeroLabel": "no limit" },
  { "key": "minClearance", "label": "Min clearance", "unit": "m", "type": "number", "min": 0, "max": 1000, "step": 5 },
  { "key": "actionExec", "label": "Segment actions run", "type": "select", "options": [["start", "At segment start"], ["every", "At every point"]] },
  { "key": "inavAltAmsl", "label": "Waypoint altitudes AMSL", "type": "check", "platforms": ["inav"], "title": "Off: altitudes are relative to the arming point, which every INAV version understands. On: absolute MSL (needs INAV 5 or later)" },
  { "key": "inavSaveEeprom", "label": "Store mission on the board", "type": "check", "platforms": ["inav"], "title": "After the upload, also write the mission to the flight controller storage so it survives a reboot" }
],
"gimbalFields": [
  { "key": "gimbalPitch", "label": "Default camera pitch", "unit": "°", "type": "number", "min": -90, "max": 30, "step": 5, "title": "-90 = nadir (straight down), 0 = horizon" },
  { "key": "gimbalYaw", "label": "Default camera yaw", "unit": "°", "type": "number", "min": -180, "max": 180, "step": 5, "title": "Relative to the flight direction" }
]
})JSON";

struct Catalog {
    QJsonObject segments, actions, cameraPresets, radioPresets;
    QJsonArray cameraFields, radioFields, routeParamFields, gimbalFields;
    QStringList segmentOrder{QStringLiteral("waypoint"), QStringLiteral("circle"), QStringLiteral("perimeter"), QStringLiteral("area"),
                             QStringLiteral("corridor"), QStringLiteral("poi"), QStringLiteral("landing")};
    QStringList actionOrder{QStringLiteral("trigger_dist"), QStringLiteral("shot"), QStringLiteral("gimbal"), QStringLiteral("yaw"),
                            QStringLiteral("wait"), QStringLiteral("speed"), QStringLiteral("servo"), QStringLiteral("relay")};
};

QJsonArray presetOptions(const QJsonObject &presets, const QJsonArray &order)
{
    QJsonArray opts;
    for (const QJsonValue &k : order)
        opts.append(QJsonArray{k, presets.value(k.toString()).toObject().value(QStringLiteral("name"))});
    return opts;
}

const Catalog &catalog()
{
    static const Catalog c = [] {
        Catalog c;
        const QJsonObject root = QJsonDocument::fromJson(QByteArray(kCatalog)).object();
        Q_ASSERT(!root.isEmpty());
        c.segments = root.value(QStringLiteral("segments")).toObject();
        c.actions = root.value(QStringLiteral("actions")).toObject();
        c.cameraPresets = root.value(QStringLiteral("cameraPresets")).toObject();
        c.radioPresets = root.value(QStringLiteral("radioPresets")).toObject();
        // The preset select leads both field lists, with the preset names as options
        c.cameraFields.append(QJsonObject{{QStringLiteral("key"), QStringLiteral("preset")}, {QStringLiteral("label"), QStringLiteral("Camera")},
                                          {QStringLiteral("type"), QStringLiteral("select")},
                                          {QStringLiteral("options"), presetOptions(c.cameraPresets, root.value(QStringLiteral("cameraOrder")).toArray())}});
        for (const QJsonValue &f : root.value(QStringLiteral("cameraFields")).toArray()) c.cameraFields.append(f);
        c.radioFields.append(QJsonObject{{QStringLiteral("key"), QStringLiteral("preset")}, {QStringLiteral("label"), QStringLiteral("Radio")},
                                         {QStringLiteral("type"), QStringLiteral("select")},
                                         {QStringLiteral("options"), presetOptions(c.radioPresets, root.value(QStringLiteral("radioOrder")).toArray())}});
        for (const QJsonValue &f : root.value(QStringLiteral("radioFields")).toArray()) c.radioFields.append(f);
        c.routeParamFields = root.value(QStringLiteral("routeParamFields")).toArray();
        c.gimbalFields = root.value(QStringLiteral("gimbalFields")).toArray();
        return c;
    }();
    return c;
}

QJsonObject merged(QJsonObject base, const QJsonObject &over)
{
    for (auto it = over.begin(); it != over.end(); ++it) base.insert(it.key(), it.value());
    return base;
}

QMutex g_radioLock;
QJsonObject g_rememberedRadio;
std::atomic<int> g_nextId{1};

} // namespace

const QJsonObject &segmentTypes() { return catalog().segments; }
const QStringList &segmentOrder() { return catalog().segmentOrder; }
const QJsonObject &actionTypes() { return catalog().actions; }
const QStringList &actionOrder() { return catalog().actionOrder; }
const QJsonObject &cameraPresets() { return catalog().cameraPresets; }
const QJsonArray &cameraFields() { return catalog().cameraFields; }
const QJsonObject &radioPresets() { return catalog().radioPresets; }
const QJsonArray &radioFields() { return catalog().radioFields; }
const QJsonArray &routeParamFields() { return catalog().routeParamFields; }
const QJsonArray &gimbalFields() { return catalog().gimbalFields; }

QJsonObject segmentDef(const QString &type) { return catalog().segments.value(type).toObject(); }
QJsonObject actionDef(const QString &type) { return catalog().actions.value(type).toObject(); }

double num(const QJsonValue &v, double fallback)
{
    if (v.isDouble()) return v.toDouble();
    if (v.isBool()) return v.toBool() ? 1 : 0;
    if (v.isString()) {
        bool ok = false;
        const double d = v.toString().trimmed().toDouble(&ok);
        return ok ? d : fallback;
    }
    return fallback;
}

bool truthy(const QJsonValue &v)
{
    if (v.isBool()) return v.toBool();
    if (v.isDouble()) return v.toDouble() != 0 && !std::isnan(v.toDouble());
    if (v.isString()) return !v.toString().isEmpty();
    return v.isObject() || v.isArray();
}

namespace {
QString fmtNum(double v)
{
    // JS template literals print 20, 2.5, -90 — never 20.0
    return QString::number(v, 'g', 12);
}
} // namespace

QString actionSummary(const QJsonObject &a)
{
    const QString t = a.value(QStringLiteral("type")).toString();
    auto n = [&](const char *k) { return num(a.value(QLatin1String(k))); };
    if (t == QLatin1String("trigger_dist"))
        return n("distance") > 0 ? QStringLiteral("every %1 m").arg(fmtNum(n("distance"))) : QStringLiteral("stop");
    if (t == QLatin1String("shot")) return QStringLiteral("single shot");
    if (t == QLatin1String("gimbal")) return QStringLiteral("pitch %1° yaw %2°").arg(fmtNum(n("pitch")), fmtNum(n("yaw")));
    if (t == QLatin1String("yaw")) return QStringLiteral("%1°").arg(fmtNum(n("heading")));
    if (t == QLatin1String("wait")) return QStringLiteral("%1 s").arg(fmtNum(n("seconds")));
    if (t == QLatin1String("speed")) return QStringLiteral("%1 m/s").arg(fmtNum(n("speed")));
    if (t == QLatin1String("servo")) return QStringLiteral("ch%1 → %2").arg(fmtNum(n("channel")), fmtNum(n("pwm")));
    if (t == QLatin1String("relay"))
        return QStringLiteral("relay %1 %2").arg(fmtNum(n("relay")), n("state") != 0 ? QStringLiteral("on") : QStringLiteral("off"));
    return QStringLiteral("cmd %1").arg(fmtNum(n("command")));
}

QJsonObject defaultCamera()
{
    QJsonObject cam = cameraPresets().value(QStringLiteral("generic")).toObject();
    cam.insert(QStringLiteral("preset"), QStringLiteral("generic"));
    return cam;
}

QJsonObject defaultRadio()
{
    QJsonObject base;
    {
        QMutexLocker l(&g_radioLock);
        base = g_rememberedRadio;
    }
    if (base.isEmpty()) {
        base = radioPresets().value(QStringLiteral("rfd900x")).toObject();
        base.insert(QStringLiteral("preset"), QStringLiteral("rfd900x"));
    }
    return merged(QJsonObject{{QStringLiteral("losses"), 1}, {QStringLiteral("minMargin"), 10},
                              {QStringLiteral("groundHeight"), 2}, {QStringLiteral("coverageRange"), 15}},
                  base);
}

void rememberRadio(const QJsonObject &radio)
{
    QMutexLocker l(&g_radioLock);
    g_rememberedRadio = radio;
}

QJsonObject defaultRouteParams()
{
    return QJsonObject{
        {QStringLiteral("altMode"), QStringLiteral("agl")},
        {QStringLiteral("defaultAlt"), 100},
        {QStringLiteral("terrainWaypoints"), false},
        {QStringLiteral("aglTolerance"), 10},
        {QStringLiteral("defaultSpeed"), 10},
        {QStringLiteral("turnType"), QStringLiteral("straight")},
        {QStringLiteral("takeoff"), true},
        {QStringLiteral("takeoffAlt"), 30},
        {QStringLiteral("endAction"), QStringLiteral("rtl")},
        {QStringLiteral("maxAgl"), 120},
        {QStringLiteral("minClearance"), 20},
        {QStringLiteral("actionExec"), QStringLiteral("start")},
        {QStringLiteral("inavAltAmsl"), false},
        {QStringLiteral("inavSaveEeprom"), false},
        {QStringLiteral("home"), QJsonValue::Null},
        {QStringLiteral("camera"), defaultCamera()},
        {QStringLiteral("gimbalPitch"), -90},
        {QStringLiteral("gimbalYaw"), 0},
        {QStringLiteral("radio"), defaultRadio()},
        {QStringLiteral("operator"), QJsonValue::Null},
    };
}

QString newId()
{
    return QStringLiteral("s%1%2").arg(QString::number(QDateTime::currentMSecsSinceEpoch(), 36),
                                       QString::number(g_nextId.fetch_add(1), 36));
}

QJsonObject createRoute()
{
    return QJsonObject{{QStringLiteral("name"), QJsonValue::Null},
                       {QStringLiteral("params"), defaultRouteParams()},
                       {QStringLiteral("segments"), QJsonArray{}}};
}

QJsonArray pointsToJson(const QList<LL> &pts)
{
    QJsonArray a;
    for (const LL &p : pts) a.append(QJsonObject{{QStringLiteral("lat"), p.lat}, {QStringLiteral("lng"), p.lng}});
    return a;
}

QList<LL> pointsOf(const QJsonObject &seg)
{
    QList<LL> out;
    for (const QJsonValue &v : seg.value(QStringLiteral("points")).toArray()) {
        const QJsonObject o = v.toObject();
        out.append({num(o.value(QStringLiteral("lat"))), num(o.value(QStringLiteral("lng")))});
    }
    return out;
}

QJsonObject createSegment(const QString &type, const QList<LL> &points, const QJsonObject &overrides)
{
    const QJsonObject def = segmentDef(type);
    return QJsonObject{{QStringLiteral("id"), newId()},
                       {QStringLiteral("type"), type},
                       {QStringLiteral("points"), pointsToJson(points)},
                       {QStringLiteral("params"), merged(def.value(QStringLiteral("defaults")).toObject(), overrides)},
                       {QStringLiteral("actions"), QJsonArray{}}};
}

QJsonObject createAction(const QString &type, const QJsonObject &overrides)
{
    QJsonObject a = actionDef(type).value(QStringLiteral("defaults")).toObject();
    a.insert(QStringLiteral("type"), type);
    return merged(a, overrides);
}

namespace {
QJsonObject normalizeSegment(const QJsonObject &s)
{
    QString type = s.value(QStringLiteral("type")).toString();
    if (!segmentTypes().contains(type)) type = QStringLiteral("waypoint");
    const QJsonObject def = segmentDef(type);
    QJsonArray actions;
    for (const QJsonValue &v : s.value(QStringLiteral("actions")).toArray()) {
        const QJsonObject a = v.toObject();
        actions.append(merged(actionDef(a.value(QStringLiteral("type")).toString()).value(QStringLiteral("defaults")).toObject(), a));
    }
    QString id = s.value(QStringLiteral("id")).toString();
    if (id.isEmpty()) id = newId();
    return QJsonObject{{QStringLiteral("id"), id},
                       {QStringLiteral("type"), type},
                       {QStringLiteral("points"), pointsToJson(pointsOf(s))},
                       {QStringLiteral("params"), merged(def.value(QStringLiteral("defaults")).toObject(), s.value(QStringLiteral("params")).toObject())},
                       {QStringLiteral("actions"), actions}};
}
} // namespace

QJsonObject normalizeRoute(const QJsonObject &next)
{
    const QJsonObject p = next.value(QStringLiteral("params")).toObject();
    QJsonObject params = merged(defaultRouteParams(), p);
    params.insert(QStringLiteral("camera"), merged(defaultCamera(), p.value(QStringLiteral("camera")).toObject()));
    params.insert(QStringLiteral("radio"), merged(defaultRadio(), p.value(QStringLiteral("radio")).toObject()));
    QJsonArray segs;
    for (const QJsonValue &v : next.value(QStringLiteral("segments")).toArray()) segs.append(normalizeSegment(v.toObject()));
    const QJsonValue name = next.value(QStringLiteral("name"));
    return QJsonObject{{QStringLiteral("name"), name.isString() && !name.toString().isEmpty() ? name : QJsonValue(QJsonValue::Null)},
                       {QStringLiteral("params"), params},
                       {QStringLiteral("segments"), segs}};
}

double segmentAlt(const QJsonObject &seg, const QJsonObject &params)
{
    const QJsonValue a = seg.value(QStringLiteral("params")).toObject().value(QStringLiteral("alt"));
    if (a.isNull() || a.isUndefined() || (a.isString() && a.toString().isEmpty()))
        return num(params.value(QStringLiteral("defaultAlt")));
    return num(a);
}

bool fieldVisible(const QJsonObject &field, const QJsonObject &values)
{
    const QJsonObject when = field.value(QStringLiteral("when")).toObject();
    for (auto it = when.begin(); it != when.end(); ++it) {
        const QJsonValue have = values.value(it.key());
        if (it.value().isArray()) {
            if (!it.value().toArray().contains(have)) return false;
        } else if (it.value() != have) {
            return false;
        }
    }
    return true;
}

} // namespace corv::mission
