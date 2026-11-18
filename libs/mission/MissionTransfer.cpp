#include "mission/MissionTransfer.h"

#include <QJsonArray>
#include <QRegularExpression>
#include <cmath>

namespace corv::mission {

namespace {
// float32 on the wire → 671.7000122; one decimetre is all a plan needs
double r1(double v) { return std::isfinite(v) ? std::round(v * 10) / 10 : v; }
QJsonObject o(std::initializer_list<QPair<QString, QJsonValue>> kv) { QJsonObject j; for (const auto &p : kv) j.insert(p.first, p.second); return j; }
} // namespace

QJsonObject itemsToRoute(const QList<RawItem> &itemsIn, const QString &name, bool fromVehicle)
{
    QJsonObject route = createRoute();
    route.insert(QStringLiteral("name"), name.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(name));
    QJsonObject P = defaultRouteParams();
    if (itemsIn.isEmpty()) { route.insert(QStringLiteral("params"), P); return route; }

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
            P.insert(QStringLiteral("home"), o({{QStringLiteral("lat"), h.lat}, {QStringLiteral("lng"), h.lng}}));
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
        P.insert(QStringLiteral("altMode"), rel >= total / 2.0 ? QStringLiteral("rel") : terrainFrame ? QStringLiteral("agl") : QStringLiteral("amsl"));
    } else {
        P.insert(QStringLiteral("altMode"), QStringLiteral("agl"));   // saved files carry AGL in `alt`
    }
    P.insert(QStringLiteral("takeoff"), false);
    P.insert(QStringLiteral("endAction"), QStringLiteral("none"));

    QJsonArray segs;
    int last = -1;                      // last segment that can carry actions
    bool defaultSpeedSet = false;
    auto attach = [&](const QJsonObject &action) {
        if (last < 0) {
            // Actions before the first navigation point: kept on a zero-length waypoint
            const QJsonObject home = P.value(QStringLiteral("home")).toObject();
            const LL at = home.isEmpty() ? LL{itemsIn[0].lat, itemsIn[0].lng} : LL{num(home.value(QStringLiteral("lat"))), num(home.value(QStringLiteral("lng")))};
            segs.append(createSegment(QStringLiteral("waypoint"), {at}));
            last = int(segs.size()) - 1;
        }
        QJsonObject s = segs[last].toObject();
        QJsonArray acts = s.value(QStringLiteral("actions")).toArray();
        acts.append(action);
        s.insert(QStringLiteral("actions"), acts);
        segs[last] = s;
    };
    auto altOf = [](const RawItem &it) { return std::isfinite(it.alt) ? QJsonValue(it.alt) : QJsonValue(QJsonValue::Null); };

    for (qsizetype i = 0; i < list.size(); ++i) {
        const RawItem &it = list[i];
        const bool isLast = i == list.size() - 1;
        const LL at{it.lat, it.lng};
        switch (it.command) {
        case 16: case 82:
            segs.append(createSegment(QStringLiteral("waypoint"), {at},
                                      o({{QStringLiteral("alt"), altOf(it)},
                                         {QStringLiteral("turn"), it.command == 82 ? QStringLiteral("spline") : QStringLiteral("route")},
                                         {QStringLiteral("delay"), it.param1}, {QStringLiteral("acceptRadius"), it.param2}})));
            last = int(segs.size()) - 1;
            break;
        case 17: case 18: case 19: case 31: {
            const double pr = it.param3 ? it.param3 : it.param2;
            const double rr = std::abs(pr ? pr : 50);
            segs.append(createSegment(QStringLiteral("circle"), {at},
                                      o({{QStringLiteral("alt"), altOf(it)}, {QStringLiteral("radius"), rr},
                                         {QStringLiteral("turns"), it.command == 18 ? std::max(1.0, it.param1 ? it.param1 : 1) : 1.0},
                                         {QStringLiteral("direction"), pr < 0 ? QStringLiteral("ccw") : QStringLiteral("cw")}})));
            last = int(segs.size()) - 1;
            break;
        }
        case 22: case 84: {
            P.insert(QStringLiteral("takeoff"), true);
            // Take-off height is always above the take-off point; an absolute frame carries home MSL
            const double rel = (it.frame == 0 && homeZ) ? it.alt - *homeZ : it.alt;
            if (rel > 0) P.insert(QStringLiteral("takeoffAlt"), r1(rel));
            break;
        }
        case 20:
            if (isLast) P.insert(QStringLiteral("endAction"), QStringLiteral("rtl"));
            else attach(createAction(QStringLiteral("raw"), o({{QStringLiteral("command"), 20}})));
            break;
        case 21: case 85: {
            bool samePlace = false;
            if (last >= 0) {
                const QJsonObject ls = segs[last].toObject();
                const QList<LL> lp = pointsOf(ls);
                samePlace = !lp.isEmpty() && std::abs(lp[0].lat - it.lat) < 1e-6 && std::abs(lp[0].lng - it.lng) < 1e-6
                            && ls.value(QStringLiteral("type")).toString() != QLatin1String("landing");
            }
            if (isLast && samePlace) {
                P.insert(QStringLiteral("endAction"), QStringLiteral("land"));
            } else {
                segs.append(createSegment(QStringLiteral("landing"), {at},
                                          o({{QStringLiteral("vtol"), it.command == 85}, {QStringLiteral("abortAlt"), it.param1}})));
                last = int(segs.size()) - 1;
            }
            break;
        }
        case 201: case 195:
            segs.append(createSegment(QStringLiteral("poi"), {at}, o({{QStringLiteral("mode"), QStringLiteral("set")}, {QStringLiteral("alt"), std::isfinite(it.alt) ? it.alt : 0}})));
            break;
        case 197: {
            LL where = at;
            if (!it.lat && !it.lng) {
                if (last >= 0 && !pointsOf(segs[last].toObject()).isEmpty()) where = pointsOf(segs[last].toObject())[0];
                else {
                    const QJsonObject home = P.value(QStringLiteral("home")).toObject();
                    where = home.isEmpty() ? LL{0, 0} : LL{num(home.value(QStringLiteral("lat"))), num(home.value(QStringLiteral("lng")))};
                }
            }
            segs.append(createSegment(QStringLiteral("poi"), {where}, o({{QStringLiteral("mode"), QStringLiteral("clear")}})));
            break;
        }
        case 178:
            if (!defaultSpeedSet && last < 0) {
                if (it.param2) P.insert(QStringLiteral("defaultSpeed"), it.param2);
                defaultSpeedSet = true;
            } else if (last >= 0 && !num(segs[last].toObject().value(QStringLiteral("params")).toObject().value(QStringLiteral("speed")))) {
                QJsonObject s = segs[last].toObject();
                QJsonObject sp = s.value(QStringLiteral("params")).toObject();
                sp.insert(QStringLiteral("speed"), it.param2);
                s.insert(QStringLiteral("params"), sp);
                segs[last] = s;
            } else {
                attach(createAction(QStringLiteral("speed"), o({{QStringLiteral("speed"), it.param2}})));
            }
            break;
        case 206: attach(createAction(QStringLiteral("trigger_dist"), o({{QStringLiteral("distance"), it.param1}}))); break;
        case 203: attach(createAction(QStringLiteral("shot"))); break;
        case 205: attach(createAction(QStringLiteral("gimbal"), o({{QStringLiteral("pitch"), it.param1}, {QStringLiteral("yaw"), it.param3}}))); break;
        case 115: attach(createAction(QStringLiteral("yaw"), o({{QStringLiteral("heading"), it.param1}}))); break;
        case 93: case 112: attach(createAction(QStringLiteral("wait"), o({{QStringLiteral("seconds"), it.param1}}))); break;
        case 183: attach(createAction(QStringLiteral("servo"), o({{QStringLiteral("channel"), it.param1}, {QStringLiteral("pwm"), it.param2}}))); break;
        case 181: attach(createAction(QStringLiteral("relay"), o({{QStringLiteral("relay"), it.param1}, {QStringLiteral("state"), it.param2 ? 1 : 0}}))); break;
        default:
            attach(createAction(QStringLiteral("raw"), o({{QStringLiteral("command"), it.command}, {QStringLiteral("param1"), it.param1},
                                                          {QStringLiteral("param2"), it.param2}, {QStringLiteral("param3"), it.param3},
                                                          {QStringLiteral("param4"), it.param4}})));
        }
    }

    // A speed change that only ever sits on the first segment is really the route speed
    if (!defaultSpeedSet && !segs.isEmpty()) {
        const double first = num(segs[0].toObject().value(QStringLiteral("params")).toObject().value(QStringLiteral("speed")));
        bool others = false;
        for (qsizetype i = 1; i < segs.size(); ++i)
            if (num(segs[i].toObject().value(QStringLiteral("params")).toObject().value(QStringLiteral("speed")))) others = true;
        if (first > 0 && !others) {
            P.insert(QStringLiteral("defaultSpeed"), first);
            QJsonObject s = segs[0].toObject();
            QJsonObject sp = s.value(QStringLiteral("params")).toObject();
            sp.insert(QStringLiteral("speed"), 0);
            s.insert(QStringLiteral("params"), sp);
            segs[0] = s;
        }
    }
    route.insert(QStringLiteral("params"), P);
    route.insert(QStringLiteral("segments"), segs);
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
    QStringList lines{QStringLiteral("QGC WPL 110")};
    auto g = [](double v) { return QString::number(v, 'g', 10); };
    for (const RawItem &it : items) {
        lines << QStringList{QString::number(it.seq), it.seq == 0 ? QStringLiteral("1") : QStringLiteral("0"), QString::number(it.frame),
                             QString::number(it.command), g(it.param1), g(it.param2), g(it.param3), g(it.param4),
                             QString::number(it.lat, 'f', 8), QString::number(it.lng, 'f', 8), QString::number(it.alt, 'f', 2), QStringLiteral("1")}
                     .join(QLatin1Char('\t'));
    }
    return lines.join(QLatin1Char('\n')) + QLatin1Char('\n');
}

bool parseWpl(const QString &text, QList<RawItem> &out, QString *error)
{
    const QStringList lines = text.split(QRegularExpression(QStringLiteral("\r?\n")));
    if (lines.isEmpty() || !lines[0].startsWith(QStringLiteral("QGC WPL"), Qt::CaseInsensitive)) {
        if (error) *error = QStringLiteral("Not a QGC WPL file");
        return false;
    }
    static const QRegularExpression ws(QStringLiteral("\\s+"));
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
        QJsonObject j{{QStringLiteral("seq"), it.seq}, {QStringLiteral("command"), it.command}, {QStringLiteral("lat"), it.lat},
                      {QStringLiteral("lng"), it.lng}, {QStringLiteral("alt"), it.alt}, {QStringLiteral("altMsl"), it.altMsl},
                      {QStringLiteral("frame"), it.frame}, {QStringLiteral("param1"), it.param1}, {QStringLiteral("param2"), it.param2},
                      {QStringLiteral("param3"), it.param3}, {QStringLiteral("param4"), it.param4}};
        if (it.isHome) j.insert(QStringLiteral("isHome"), true);
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
        it.seq = j.value(QStringLiteral("seq")).toInt();
        it.command = int(num(j.value(QStringLiteral("command")), 16));
        it.frame = int(num(j.value(QStringLiteral("frame")), 3));
        it.lat = num(j.value(QStringLiteral("lat")));
        it.lng = num(j.value(j.contains(QStringLiteral("lng")) ? QStringLiteral("lng") : QStringLiteral("lon")));
        it.alt = num(j.value(QStringLiteral("alt")), NAN);
        it.param1 = num(j.value(QStringLiteral("param1")));
        it.param2 = num(j.value(QStringLiteral("param2")));
        it.param3 = num(j.value(QStringLiteral("param3")));
        it.param4 = num(j.value(QStringLiteral("param4")));
        it.isHome = j.value(QStringLiteral("isHome")).toBool();
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
