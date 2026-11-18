// MissionTransfer.h - a flat MAVLink item list back into route segments
// (itemsToRoute of js/mission/MissionTransfer.js), plus the QGC WPL text format
// of the route menu (import / export .waypoints) and the INAV records read back.
//
// The compiler is lossy in one direction only: a lawn-mower grid read back
// from the autopilot comes back as plain waypoints, never as the area it was
// generated from. That trade-off lets an old mission file, or a plan written
// by Mission Planner, open as an editable route.
#pragma once

#include "mission/RouteCompiler.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QList>
#include <QString>

namespace corv::mission {

/// A mission item in transit (vehicle, file): lat/lng in degrees.
struct RawItem {
    int seq = 0, command = 16, frame = 3;
    double lat = 0, lng = 0, alt = 0;
    double param1 = 0, param2 = 0, param3 = 0, param4 = 0;
    bool isHome = false;
};

/// fromVehicle keeps the raw frames instead of the app's "alt is AGL" convention of saved files.
QJsonObject itemsToRoute(const QList<RawItem> &items, const QString &name, bool fromVehicle);

/// Items in the exact frame / altitude ArduPilot expects (uploadItems of the controller).
QList<RawItem> uploadItems(const QList<CompiledItem> &items);

/// "QGC WPL 110" text, the lingua franca of ArduPilot plans.
QString toWpl(const QList<RawItem> &items);
/// Parse a QGC WPL file; false with `error` set when it is not one.
bool parseWpl(const QString &text, QList<RawItem> &out, QString *error);

/// The compiled items without runtime-only fields, as the mission files store them.
QJsonArray serializeItems(const QList<CompiledItem> &items);
QList<RawItem> itemsFromJson(const QJsonArray &a);

/// INAV records read back → items itemsToRoute() understands (synthetic home first).
QList<RawItem> inavToItems(const QList<InavWp> &wps);

} // namespace corv::mission
