#include "vehicle/FlightModes.h"

#include <QHash>
#include <map>

namespace corv::modes {

namespace {
using ModeMap = std::map<int, const char *>;

const ModeMap kCopter = {
    {0, "STABILIZE"}, {1, "ACRO"}, {2, "ALT_HOLD"}, {3, "AUTO"}, {4, "GUIDED"}, {5, "LOITER"}, {6, "RTL"},
    {7, "CIRCLE"}, {9, "LAND"}, {11, "DRIFT"}, {13, "SPORT"}, {14, "FLIP"}, {15, "AUTOTUNE"}, {16, "POSHOLD"},
    {17, "BRAKE"}, {18, "THROW"}, {19, "AVOID_ADSB"}, {20, "GUIDED_NOGPS"}, {21, "SMART_RTL"}, {22, "FLOWHOLD"},
    {23, "FOLLOW"}, {24, "ZIGZAG"}, {25, "SYSTEMID"}, {26, "AUTOROTATE"}, {27, "AUTO_RTL"}, {28, "TURTLE"}};
const ModeMap kPlane = {
    {0, "MANUAL"}, {1, "CIRCLE"}, {2, "STABILIZE"}, {3, "TRAINING"}, {4, "ACRO"}, {5, "FBWA"}, {6, "FBWB"},
    {7, "CRUISE"}, {8, "AUTOTUNE"}, {10, "AUTO"}, {11, "RTL"}, {12, "LOITER"}, {13, "TAKEOFF"}, {14, "AVOID_ADSB"},
    {15, "GUIDED"}, {17, "QSTABILIZE"}, {18, "QHOVER"}, {19, "QLOITER"}, {20, "QLAND"}, {21, "QRTL"},
    {22, "QAUTOTUNE"}, {23, "QACRO"}, {24, "THERMAL"}, {25, "LOITER_ALT_QLAND"}, {26, "AUTOLAND"}};
const ModeMap kRover = {{0, "MANUAL"}, {1, "ACRO"}, {3, "STEERING"}, {4, "HOLD"}, {5, "LOITER"}, {6, "FOLLOW"},
                        {7, "SIMPLE"}, {8, "DOCK"}, {9, "CIRCLE"}, {10, "AUTO"}, {11, "RTL"}, {12, "SMART_RTL"},
                        {15, "GUIDED"}};
const ModeMap kSub = {{0, "STABILIZE"}, {1, "ACRO"}, {2, "ALT_HOLD"}, {3, "AUTO"}, {4, "GUIDED"}, {7, "CIRCLE"},
                      {9, "SURFACE"}, {16, "POSHOLD"}, {19, "MANUAL"}, {20, "MOTOR_DETECT"}, {21, "SURFTRAK"}};
const ModeMap kInav = {{0, "MANUAL"}, {1, "ACRO"}, {2, "ANGLE"}, {3, "HORIZON"}, {4, "ALTHOLD"}, {5, "POSHOLD"},
                       {6, "COURSE HOLD"}, {7, "CRUISE"}, {8, "WAYPOINT"}, {9, "RTH"}, {10, "FAILSAFE"}};

const ModeMap &mapOf(Table t)
{
    switch (t) {
    case Table::Plane: return kPlane;
    case Table::Rover: return kRover;
    case Table::Sub: return kSub;
    case Table::Inav: return kInav;
    case Table::Copter: break;
    }
    return kCopter;
}
} // namespace

bool isPlaneType(int t) { return t == 1 || t == 20 || t == 21; }
bool isRoverType(int t) { return t == 10 || t == 11; }
bool isSubType(int t) { return t == 12; }

Table tableFor(int vehicleType, const QString &connectionType)
{
    if (connectionType.startsWith(QLatin1String("msp"))) return Table::Inav;
    if (isPlaneType(vehicleType)) return Table::Plane;
    if (isRoverType(vehicleType)) return Table::Rover;
    if (isSubType(vehicleType)) return Table::Sub;
    return Table::Copter;
}

QString name(std::uint32_t customMode, int vehicleType, const QString &connectionType)
{
    const ModeMap &m = mapOf(tableFor(vehicleType, connectionType));
    const auto it = m.find(int(customMode));
    return it != m.end() ? QString::fromLatin1(it->second) : QStringLiteral("MODE_%1").arg(customMode);
}

int number(const QString &n, int vehicleType, const QString &connectionType)
{
    for (const auto &[num, nm] : mapOf(tableFor(vehicleType, connectionType)))
        if (n == QLatin1String(nm)) return num;
    return -1;
}

QList<QPair<int, QString>> available(int vehicleType, const QString &connectionType)
{
    QList<QPair<int, QString>> out;
    for (const auto &[num, nm] : mapOf(tableFor(vehicleType, connectionType))) out << qMakePair(num, QString::fromLatin1(nm));
    return out;
}

QString vehicleTypeName(int t)
{
    if (isPlaneType(t)) return QStringLiteral("Plane");
    if (isRoverType(t)) return QStringLiteral("Rover");
    if (isSubType(t)) return QStringLiteral("Sub");
    return QStringLiteral("Copter");
}

QString gpsFixName(int f)
{
    static const char *names[] = {"No GPS", "No Fix", "2D Fix", "3D Fix", "DGPS", "RTK Float", "RTK Fixed"};
    return f >= 0 && f < 7 ? QString::fromLatin1(names[f]) : QStringLiteral("Fix %1").arg(f);
}

QString modeClass(const QString &n)
{
    // MODE_CATEGORIES of CommandBarController.js; anything else has no colour class
    static const QHash<QString, QString> cat = {
        {QStringLiteral("STABILIZE"), QStringLiteral("manual")}, {QStringLiteral("ACRO"), QStringLiteral("manual")},
        {QStringLiteral("MANUAL"), QStringLiteral("manual")}, {QStringLiteral("TRAINING"), QStringLiteral("manual")},
        {QStringLiteral("ALT_HOLD"), QStringLiteral("assisted")}, {QStringLiteral("POSHOLD"), QStringLiteral("assisted")},
        {QStringLiteral("LOITER"), QStringLiteral("assisted")}, {QStringLiteral("FBWA"), QStringLiteral("assisted")},
        {QStringLiteral("FBWB"), QStringLiteral("assisted")}, {QStringLiteral("CIRCLE"), QStringLiteral("assisted")},
        {QStringLiteral("GUIDED"), QStringLiteral("assisted")}, {QStringLiteral("SPORT"), QStringLiteral("assisted")},
        {QStringLiteral("FLOWHOLD"), QStringLiteral("assisted")}, {QStringLiteral("FOLLOW"), QStringLiteral("assisted")},
        {QStringLiteral("ZIGZAG"), QStringLiteral("assisted")},
        {QStringLiteral("AUTO"), QStringLiteral("auto")}, {QStringLiteral("AUTOTUNE"), QStringLiteral("auto")},
        {QStringLiteral("BRAKE"), QStringLiteral("auto")},
        {QStringLiteral("RTL"), QStringLiteral("emergency")}, {QStringLiteral("LAND"), QStringLiteral("emergency")},
        {QStringLiteral("QRTL"), QStringLiteral("emergency")}, {QStringLiteral("QLAND"), QStringLiteral("emergency")},
        {QStringLiteral("SMART_RTL"), QStringLiteral("emergency")}, {QStringLiteral("AUTO_RTL"), QStringLiteral("emergency")},
        {QStringLiteral("SURFACE"), QStringLiteral("emergency")}};
    return cat.value(n);
}

} // namespace corv::modes
