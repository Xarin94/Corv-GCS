// FlightModes.h - mode tables (from MAVLinkStateMapper.js).
//
// The link protocol wins over MAV_TYPE: an INAV plane reports type 1 too, but
// its custom_mode numbering has nothing to do with ArduPlane's. The INAV
// numbers are not a firmware enum: the MSP adapter encodes the active mode box
// with them (keep the two tables in step).
#pragma once

#include <QList>
#include <QPair>
#include <QString>

namespace corv::modes {

enum class Table { Copter, Plane, Rover, Sub, Inav };

Table tableFor(int vehicleType, const QString &connectionType);
QString name(std::uint32_t customMode, int vehicleType, const QString &connectionType);
/// -1 when the name is not in the table.
int number(const QString &name, int vehicleType, const QString &connectionType);
QList<QPair<int, QString>> available(int vehicleType, const QString &connectionType);

QString vehicleTypeName(int vehicleType);   // Plane, Rover, Sub, Copter
QString gpsFixName(int fixType);

bool isPlaneType(int t);
bool isRoverType(int t);
bool isSubType(int t);

/// Colour class of a mode in the command bar: manual, assisted, auto, emergency, or empty.
QString modeClass(const QString &name);

} // namespace corv::modes
