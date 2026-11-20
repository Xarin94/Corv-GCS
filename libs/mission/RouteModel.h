// RouteModel.h - the flight-plan document (port of js/mission/RouteModel.js).
//
// A route is what the operator edits: an ordered list of high-level SEGMENTS
// (waypoint, circle, perimeter, area scan, corridor, POI, landing) plus the
// route-wide parameters (altitude mode, default speed, take-off, end action…).
// It is not a list of MAVLink commands: RouteCompiler turns it into mission
// items a moment after the operator stops editing.
//
// The document stays JSON, as in the Electron version: the catalogues below
// drive the inspector fields generically, the mission files on disk carry the
// same object, and QML reads it without a conversion layer.
//
//   route   { name, params: {…}, segments: [ { id, type, points:[{lat,lng}], params:{…}, actions:[{type,…}] } ] }
//   field   { key, label, unit?, type: number|select|check, min?, max?, step?, options?:[[value,label]],
//             nullable?, placeholder?, zeroLabel?, when?:{key: value|[values]}, title?, platforms?:[ids] }
#pragma once

#include <QJsonArray>
#include <QJsonObject>
#include <QList>
#include <QString>
#include <QStringList>

namespace corv::mission {

struct LL {
    double lat = 0, lng = 0;
};

// ── Catalogues ───────────────────────────────────────────────────────────────
/// type → { label, short, points (1 | "poly" | "line"), color, hotkey, hint, defaults, fields }
const QJsonObject &segmentTypes();
const QStringList &segmentOrder();
/// type → { label, short, cmd, defaults, fields }
const QJsonObject &actionTypes();
const QStringList &actionOrder();
const QJsonObject &cameraPresets();
const QJsonArray &cameraFields();
const QJsonObject &radioPresets();
const QJsonArray &radioFields();
const QJsonArray &routeParamFields();
const QJsonArray &gimbalFields();

QJsonObject segmentDef(const QString &type);
QJsonObject actionDef(const QString &type);
/// The one-line summary of an action ("every 20 m", "pitch -90° yaw 0°"…).
QString actionSummary(const QJsonObject &action);

// ── Defaults ─────────────────────────────────────────────────────────────────
QJsonObject defaultCamera();
/// The radio a new route starts with: the last one used, else the RFD900x.
QJsonObject defaultRadio();
/// The ground radio rarely changes between missions: remembered for the next new route.
void rememberRadio(const QJsonObject &radio);
QJsonObject defaultRouteParams();

// ── The document ─────────────────────────────────────────────────────────────
QString newId();
QJsonObject createRoute();
/// replaceRoute(): defaults filled in, unknown segment types become waypoints, ids kept.
QJsonObject normalizeRoute(const QJsonObject &route);
QJsonObject createSegment(const QString &type, const QList<LL> &points, const QJsonObject &overrides = {});
QJsonObject createAction(const QString &type, const QJsonObject &overrides = {});

/// Effective altitude of a segment: its own, or the route default.
double segmentAlt(const QJsonObject &seg, const QJsonObject &params);
/// Whether a field is visible given the current values (`when` clauses).
bool fieldVisible(const QJsonObject &field, const QJsonObject &values);

QList<LL> pointsOf(const QJsonObject &seg);
QJsonArray pointsToJson(const QList<LL> &pts);

/// JS-style number of a JSON value: numbers, numeric strings and booleans; `fallback` otherwise.
double num(const QJsonValue &v, double fallback = 0);
/// JS truthiness.
bool truthy(const QJsonValue &v);

/// A number as the route editor prints it: 20, 2.5, -90, never 20.0.
QString fmtNum(double v);
/// "850 m", "1.2 km", "12 km".
QString fmtDist(double m);
/// "--" for none, "12 min", "1h 05m".
QString fmtTime(double s);

} // namespace corv::mission
