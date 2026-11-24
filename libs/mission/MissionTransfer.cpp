#include "mission/MissionTransfer.h"

#include <QJsonArray>
#include <QRegularExpression>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv::mission {

namespace {
// float32 on the wire → 671.7000122; one decimetre is all a plan needs
double r1(double v) { return std::isfinite(v) ? std::round(v * 10) / 10 : v; }
QJsonObject o(std::initializer_list<QPair<QString, QJsonValue>> kv) { QJsonObject j; for (const auto &p : kv) j.insert(p.first, p.second); return j; }
} // namespace

QJsonObject itemsToRoute(const QList<RawItem> &itemsIn, const QString &name, bool fromVehicle)
{
    QJsonObject route = createRoute();
    route.insert(u"name"_s, name.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(name));
    QJsonObject P = defaultRouteParams();
    if (itemsIn.isEmpty()) { route.insert(u"params"_s, P); return route; }

    QList<RawItem> list;
    for (RawItem it : itemsIn) {
        it.alt = r1(it.alt);
        it.param1 = r1(it.param1); it.param2 = r1(it.param2); it.param3 = r1(it.param3); it.param4 = r1(it.param4);
        list.append(it);
    }
    std::optional<double> homeZ;
    // seq 0 is the home position on ArduPilot; saved files flag it explicitly
    if (list[0].isHome || (fromVehicle && list[0].command == 16)) {
        const RawItem &h = list[0];
        if (std::isfinite(h.lat) && std::isfinite(h.lng) && (h.lat || h.lng))
            P.insert(u"home"_s, o({{u"lat"_s, h.lat}, {u"lng"_s, h.lng}}));
        if (std::isfinite(h.alt)) homeZ = h.alt;
        list.removeFirst();
    }

    if (fromVehicle) {
        // The frame of the flying waypoints decides the altitude mode
        int total = 0, rel = 0;
        bool terrainFrame = false;
        for (const RawItem &it : list) {
            if (it.command != 16 && it.command != 82) continue;
            ++total;
            if (it.frame == 3) ++rel;
            if (it.frame == 10) terrainFrame = true;
        }
        P.insert(u"altMode"_s, rel >= total / 2.0 ? u"rel"_s : terrainFrame ? u"agl"_s : u"amsl"_s);
    } else {
        P.insert(u"altMode"_s, u"agl"_s);   // saved files carry AGL in `alt`
    }
    P.insert(u"takeoff"_s, false);
    P.insert(u"endAction"_s, u"none"_s);
    P.insert(u"defaultSpeed"_s, 0);   // the vehicle's own, unless the mission says one

    QJsonArray segs;
    int last = -1;                      // last segment that can carry actions
    // A DO_CHANGE_SPEED runs once the point before it is reached: the speed of the legs that
    // follow, which is where the compiler takes it from. Read as the speed of the point before it
    // (the Electron version did), every speed of a mission read back moved one segment earlier.
    double routeSpeed = 0, speed = 0;        // of the route, of the legs so far
    std::optional<double> pendingSpeed;      // asked for since the last point
    bool defaultSpeedSet = false;
    auto append = [&](const QString &type, const LL &at, QJsonObject params) {
        if (pendingSpeed) speed = *pendingSpeed;
        pendingSpeed.reset();
        if (speed != routeSpeed) params.insert(u"speed"_s, speed);
        segs.append(createSegment(type, {at}, params));
        last = int(segs.size()) - 1;
    };
    auto attachTo = [&](const QJsonObject &action) {
        if (last < 0) {
            // Actions before the first navigation point: kept on a zero-length waypoint
            const QJsonObject home = P.value(u"home"_s).toObject();
            const LL at = home.isEmpty() ? LL{itemsIn[0].lat, itemsIn[0].lng} : LL{num(home.value(u"lat"_s)), num(home.value(u"lng"_s))};
            segs.append(createSegment(u"waypoint"_s, {at}));
            last = int(segs.size()) - 1;
        }
        QJsonObject s = segs[last].toObject();
        QJsonArray acts = s.value(u"actions"_s).toArray();
        acts.append(action);
        s.insert(u"actions"_s, acts);
        segs[last] = s;
    };
    // Something else is done between the speed change and the next point: the change is an
    // action too, or the two would swap
    auto flushSpeed = [&] {
        if (!pendingSpeed) return;
        attachTo(createAction(u"speed"_s, o({{u"speed"_s, *pendingSpeed}})));
        pendingSpeed.reset();
    };
    auto attach = [&](const QJsonObject &action) {
        flushSpeed();
        attachTo(action);
    };
    auto altOf = [](const RawItem &it) { return std::isfinite(it.alt) ? QJsonValue(it.alt) : QJsonValue(QJsonValue::Null); };

    for (qsizetype i = 0; i < list.size(); ++i) {
        const RawItem &it = list[i];
        const bool isLast = i == list.size() - 1;
        const LL at{it.lat, it.lng};
        switch (it.command) {
        case 16: case 82:
            append(u"waypoint"_s, at, o({{u"alt"_s, altOf(it)},
                                         {u"turn"_s, it.command == 82 ? u"spline"_s : u"route"_s},
                                         {u"delay"_s, it.param1}, {u"acceptRadius"_s, it.param2}}));
            break;
        case 17: case 18: case 19: case 31: {
            const double pr = it.param3 ? it.param3 : it.param2;
            const double rr = std::abs(pr ? pr : 50);
            append(u"circle"_s, at, o({{u"alt"_s, altOf(it)}, {u"radius"_s, rr},
                                       {u"turns"_s, it.command == 18 ? std::max(1.0, it.param1 ? it.param1 : 1) : 1.0},
                                       {u"direction"_s, pr < 0 ? u"ccw"_s : u"cw"_s}}));
            break;
        }
        case 22: case 84: {
            P.insert(u"takeoff"_s, true);
            // Take-off height is always above the take-off point; an absolute frame carries home MSL
            const double rel = (it.frame == 0 && homeZ) ? it.alt - *homeZ : it.alt;
            if (rel > 0) P.insert(u"takeoffAlt"_s, r1(rel));
            break;
        }
        case 20:
            if (isLast) P.insert(u"endAction"_s, u"rtl"_s);
            else attach(createAction(u"raw"_s, o({{u"command"_s, 20}})));
            break;
        case 21: case 85: {
            bool samePlace = false;
            if (last >= 0) {
                const QJsonObject ls = segs[last].toObject();
                const QList<LL> lp = pointsOf(ls);
                samePlace = !lp.isEmpty() && std::abs(lp[0].lat - it.lat) < 1e-6 && std::abs(lp[0].lng - it.lng) < 1e-6
                            && ls.value(u"type"_s).toString() != "landing"_L1;
            }
            if (isLast && samePlace) {
                P.insert(u"endAction"_s, u"land"_s);
            } else {
                append(u"landing"_s, at, o({{u"vtol"_s, it.command == 85}, {u"abortAlt"_s, it.param1}}));
            }
            break;
        }
        case 201: case 195:
            flushSpeed();
            segs.append(createSegment(u"poi"_s, {at}, o({{u"mode"_s, u"set"_s}, {u"alt"_s, std::isfinite(it.alt) ? it.alt : 0}})));
            break;
        case 197: {
            flushSpeed();
            LL where = at;
            if (!it.lat && !it.lng) {
                if (last >= 0 && !pointsOf(segs[last].toObject()).isEmpty()) where = pointsOf(segs[last].toObject())[0];
                else {
                    const QJsonObject home = P.value(u"home"_s).toObject();
                    where = home.isEmpty() ? LL{0, 0} : LL{num(home.value(u"lat"_s)), num(home.value(u"lng"_s))};
                }
            }
            segs.append(createSegment(u"poi"_s, {where}, o({{u"mode"_s, u"clear"_s}})));
            break;
        }
        case 178:
            if (it.param2 <= 0) break;   // no change (-1) or the vehicle's own
            if (!defaultSpeedSet && last < 0) {
                P.insert(u"defaultSpeed"_s, it.param2);
                routeSpeed = speed = it.param2;
                defaultSpeedSet = true;
            } else {
                flushSpeed();
                pendingSpeed = it.param2;
            }
            break;
        case 206: attach(createAction(u"trigger_dist"_s, o({{u"distance"_s, it.param1}}))); break;
        case 203: attach(createAction(u"shot"_s)); break;
        case 205: attach(createAction(u"gimbal"_s, o({{u"pitch"_s, it.param1}, {u"yaw"_s, it.param3}}))); break;
        case 115: attach(createAction(u"yaw"_s, o({{u"heading"_s, it.param1}}))); break;
        case 93: case 112: attach(createAction(u"wait"_s, o({{u"seconds"_s, it.param1}}))); break;
        case 183: attach(createAction(u"servo"_s, o({{u"channel"_s, it.param1}, {u"pwm"_s, it.param2}}))); break;
        case 181: attach(createAction(u"relay"_s, o({{u"relay"_s, it.param1}, {u"state"_s, it.param2 ? 1 : 0}}))); break;
        default:
            attach(createAction(u"raw"_s, o({{u"command"_s, it.command}, {u"param1"_s, it.param1},
                                                          {u"param2"_s, it.param2}, {u"param3"_s, it.param3},
                                                          {u"param4"_s, it.param4}})));
        }
    }

    flushSpeed();   // nothing is flown after it
    route.insert(u"params"_s, P);
    route.insert(u"segments"_s, segs);
    return route;
}

QList<RawItem> uploadItems(const QList<CompiledItem> &items)
{
    QList<RawItem> out;
    for (const CompiledItem &it : items) {
        RawItem r;
        r.seq = it.seq;
        r.command = it.command;
        r.lat = it.lat;
        r.lng = it.lng;
        r.param1 = it.param1; r.param2 = it.param2; r.param3 = it.param3; r.param4 = it.param4;
        r.isHome = it.isHome;
        if (it.roi) { r.frame = 3; r.alt = it.altRel; }
        else if (it.loc) { r.frame = 0; r.alt = std::round(it.altMsl * 10) / 10; }   // absolute MSL
        else { r.frame = 3; r.alt = it.alt; }
        out.append(r);
    }
    return out;
}

QString toWpl(const QList<RawItem> &items)
{
    QStringList lines{u"QGC WPL 110"_s};
    auto g = [](double v) { return QString::number(v, 'g', 10); };
    for (const RawItem &it : items) {
        lines << QStringList{QString::number(it.seq), it.seq == 0 ? u"1"_s : u"0"_s, QString::number(it.frame),
                             QString::number(it.command), g(it.param1), g(it.param2), g(it.param3), g(it.param4),
                             QString::number(it.lat, 'f', 8), QString::number(it.lng, 'f', 8), QString::number(it.alt, 'f', 2), u"1"_s}
                     .join('\t'_L1);
    }
    return lines.join('\n'_L1) + '\n'_L1;
}

bool parseWpl(const QString &text, QList<RawItem> &out, QString *error)
{
    const QStringList lines = text.split(QRegularExpression(u"\r?\n"_s));
    if (lines.isEmpty() || !lines[0].startsWith(u"QGC WPL"_s, Qt::CaseInsensitive)) {
        if (error) *error = u"Not a QGC WPL file"_s;
        return false;
    }
    static const QRegularExpression ws(u"\\s+"_s);
    for (qsizetype i = 1; i < lines.size(); ++i) {
        const QStringList f = lines[i].trimmed().split(ws, Qt::SkipEmptyParts);
        if (f.size() < 12) continue;
        RawItem it;
        it.seq = f[0].toInt();
        it.frame = f[2].toInt();
        it.command = f[3].toInt();
        it.param1 = f[4].toDouble(); it.param2 = f[5].toDouble(); it.param3 = f[6].toDouble(); it.param4 = f[7].toDouble();
        it.lat = f[8].toDouble(); it.lng = f[9].toDouble(); it.alt = f[10].toDouble();
        out.append(it);
    }
    return true;
}

QJsonArray serializeItems(const QList<CompiledItem> &items)
{
    QJsonArray a;
    for (const CompiledItem &it : items) {
        QJsonObject j{{u"seq"_s, it.seq}, {u"command"_s, it.command}, {u"lat"_s, it.lat},
                      {u"lng"_s, it.lng}, {u"alt"_s, it.alt}, {u"altMsl"_s, it.altMsl},
                      {u"frame"_s, it.frame}, {u"param1"_s, it.param1}, {u"param2"_s, it.param2},
                      {u"param3"_s, it.param3}, {u"param4"_s, it.param4}};
        if (it.isHome) j.insert(u"isHome"_s, true);
        a.append(j);
    }
    return a;
}

QList<RawItem> itemsFromJson(const QJsonArray &a)
{
    QList<RawItem> out;
    for (const QJsonValue &v : a) {
        const QJsonObject j = v.toObject();
        RawItem it;
        it.seq = j.value(u"seq"_s).toInt();
        it.command = int(num(j.value(u"command"_s), 16));
        it.frame = int(num(j.value(u"frame"_s), 3));
        it.lat = num(j.value(u"lat"_s));
        it.lng = num(j.value(j.contains(u"lng"_s) ? u"lng"_s : u"lon"_s));
        it.alt = num(j.value(u"alt"_s), NAN);
        it.param1 = num(j.value(u"param1"_s));
        it.param2 = num(j.value(u"param2"_s));
        it.param3 = num(j.value(u"param3"_s));
        it.param4 = num(j.value(u"param4"_s));
        it.isHome = j.value(u"isHome"_s).toBool();
        out.append(it);
    }
    return out;
}

QList<RawItem> inavToItems(const QList<InavWp> &wps)
{
    QList<RawItem> items;
    RawItem home;
    home.isHome = true;
    home.frame = 0;
    items.append(home);
    double lastSpeed = 0;
    auto speedItem = [&](int cms) {
        const double v = std::round(cms / 100.0 * 10) / 10;
        if (v <= 0 || v == lastSpeed) return;
        lastSpeed = v;
        RawItem s;
        s.command = 178; s.frame = 3; s.param1 = 1; s.param2 = v; s.param3 = -1;
        items.append(s);
    };
    for (const InavWp &w : wps) {
        RawItem loc;
        loc.frame = (w.p3 & 1) ? 0 : 3;
        loc.lat = w.lat / 1e7; loc.lng = w.lon / 1e7; loc.alt = w.alt / 100.0;
        switch (w.action) {
        case 1: speedItem(w.p1); loc.command = 16; items.append(loc); break;
        case 3: speedItem(w.p2); loc.command = 16; loc.param1 = w.p1; items.append(loc); break;
        case 4: { RawItem r; r.command = 20; items.append(r); break; }
        case 8: loc.command = 21; items.append(loc); break;
        case 5: loc.command = 201; items.append(loc); break;
        case 7: { RawItem r; r.command = 115; r.param1 = w.p1; items.append(r); break; }
        case 6: { RawItem r; r.command = 177; r.param1 = w.p1; r.param2 = w.p2; items.append(r); break; }
        default: break;
        }
    }
    return items;
}

} // namespace corv::mission
