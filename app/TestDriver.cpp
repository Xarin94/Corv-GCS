#include "TestDriver.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "gpu/FrameStats.h"
#include "ui/AnnunciatorController.h"
#include "ui/CalibrationController.h"
#include "ui/DialogHub.h"
#include "ui/RtlOptions.h"
#include "ui/TargetController.h"
#include "ui/EkfController.h"
#include "ui/FftController.h"
#include "ui/FpvController.h"
#include "ui/GcsController.h"
#include "ui/GeoZoneController.h"
#include "ui/JoystickController.h"
#include "ui/LogsController.h"
#include "ui/LteController.h"
#include "ui/ParamsController.h"
#include "ui/PlanController.h"
#include "ui/ReplayController.h"
#include "ui/RtkController.h"
#include "ui/SetupController.h"
#include "ui/SitlController.h"
#include "ui/LidarController.h"
#include "ui/SysConfig.h"
#include "ui/RosController.h"
#include "ui/TelForwardController.h"
#include "ui/UpdateController.h"
#include "ui/VehicleModel.h"

#include <QAbstractItemModel>
#include <QBuffer>
#include <QCoreApplication>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QMetaMethod>
#include <QMetaProperty>
#include <QPointingDevice>
#include <QQmlApplicationEngine>
#include <QQuickItem>
#include <QQuickWindow>
#include <QScreen>
#include <cmath>
#include <qpa/qwindowsysteminterface.h>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
QVariant toVariant(const QJsonValue &v)
{
    if (v.isObject()) return v.toObject().toVariantMap();
    if (v.isArray()) return v.toArray().toVariantList();
    return v.toVariant();
}

constexpr qsizetype kMaxEvents = 4000;

/// A property value as JSON: NaN and infinity as null, objects (models, items) by their class name.
QJsonValue toJson(const QVariant &v)
{
    if (const int t = v.typeId(); t == QMetaType::Double || t == QMetaType::Float) {
        const double d = v.toDouble();
        return std::isfinite(d) ? QJsonValue(d) : QJsonValue();
    }
    if (QObject *o = v.value<QObject *>()) return QString::fromLatin1(o->metaObject()->className());
    return QJsonValue::fromVariant(v);
}

QJsonValue opt(const std::optional<double> &v)
{
    return v && std::isfinite(*v) ? QJsonValue(*v) : QJsonValue();
}
} // namespace

TestDriver::TestDriver(QQmlApplicationEngine &engine) : m_engine(engine) {}

QObject *TestDriver::viewModel(const QString &name)
{
    if (name == "Params"_L1) return ParamsController::s_instance;
    if (name == "Logs"_L1) return LogsController::s_instance;
    if (name == "Plan"_L1) return PlanController::s_instance;
    if (name == "Gcs"_L1) return GcsController::s_instance;
    if (name == "Calibration"_L1) return CalibrationController::s_instance;
    if (name == "Ekf"_L1) return EkfController::s_instance;
    if (name == "Fft"_L1) return FftController::s_instance;
    if (name == "Replay"_L1) return ReplayController::s_instance;
    if (name == "Annunciators"_L1) return AnnunciatorController::s_instance;
    if (name == "Rtk"_L1) return RtkController::s_instance;
    if (name == "TelForward"_L1) return TelForwardController::s_instance;
    if (name == "Updates"_L1) return UpdateController::s_instance;
    if (name == "Lte"_L1) return LteController::s_instance;
    if (name == "SetupCfg"_L1) return SetupController::s_instance;
    if (name == "Sitl"_L1) return SitlController::s_instance;
    if (name == "GeoZones"_L1) return GeoZoneController::s_instance;
    if (name == "SysConfig"_L1) return SysConfig::s_instance;
    if (name == "Joystick"_L1) return JoystickController::s_instance;
    if (name == "Lidar"_L1) return LidarController::s_instance;
    if (name == "Ros"_L1) return RosController::s_instance;
    if (name == "Target"_L1) return TargetController::s_instance;
    if (name == "Rtl"_L1) return RtlOptions::s_instance;
    if (name == "Fpv"_L1) return FpvController::s_instance;
    if (name == "Vehicle"_L1) return VehicleModel::s_instance;
    if (name == "DialogHub"_L1) return DialogHub::s_instance;
    return nullptr;
}

QQuickWindow *TestDriver::window() const
{
    return qobject_cast<QQuickWindow *>(m_engine.rootObjects().value(0));
}

QObject *TestDriver::header() const
{
    QObject *root = m_engine.rootObjects().value(0);
    return root ? root->findChild<QObject *>(u"header"_s) : nullptr;
}

QObject *TestDriver::object(const QString &name) const
{
    if (QObject *vm = viewModel(name)) return vm;
    QObject *root = m_engine.rootObjects().value(0);
    if (name == "window"_L1) return root;
    return root ? root->findChild<QObject *>(name) : nullptr;
}

QJsonValue TestDriver::call(const QString &target, const QJsonValue &args, QString *error)
{
    const QString obj = target.section('.'_L1, 0, 0), method = target.section('.'_L1, 1);
    QObject *o = object(obj);   // a view model, or a named QML item (paramsPage.editBits)
    if (!o) {
        if (error) *error = u"no object "_s + obj;
        return {};
    }
    // The arguments: none, one value, or the list
    QVariantList list;
    if (args.isArray()) {
        for (const QJsonValue &v : args.toArray()) list << toVariant(v);
    } else if (!args.isNull() && !args.isUndefined()) {
        list << toVariant(args);
    }
    const QMetaObject *mo = o->metaObject();
    for (int i = 0; i < mo->methodCount(); ++i) {
        const QMetaMethod mm = mo->method(i);
        if (QString::fromLatin1(mm.name()) != method || mm.parameterCount() != list.size() || list.size() > 6) continue;
        QVariantList conv = list;
        bool fits = true;
        // A QVariant parameter takes the value as it is (Lidar.setField "key" value)
        for (int k = 0; k < conv.size() && fits; ++k)
            fits = mm.parameterType(k) == QMetaType::QVariant || conv[k].convert(QMetaType(mm.parameterType(k)));
        if (!fits) continue;
        QGenericArgument g[6];
        for (int k = 0; k < conv.size(); ++k)
            g[k] = mm.parameterType(k) == QMetaType::QVariant ? QGenericArgument("QVariant", &conv[k]) : QGenericArgument(conv[k].typeName(), conv[k].constData());
        QVariant ret;
        bool ok;
        if (mm.returnType() == QMetaType::QVariant) {
            // A JavaScript function of a QML item, or a method that gives a QVariant (SetupCfg.param)
            ok = mm.invoke(o, Qt::DirectConnection, QGenericReturnArgument("QVariant", &ret), g[0], g[1], g[2], g[3], g[4], g[5]);
        } else if (mm.returnType() != QMetaType::Void && mm.returnType() != QMetaType::UnknownType) {
            ret = QVariant(QMetaType(mm.returnType()));
            ok = mm.invoke(o, Qt::DirectConnection, QGenericReturnArgument(mm.typeName(), ret.data()), g[0], g[1], g[2], g[3], g[4], g[5]);
        } else {
            ok = mm.invoke(o, Qt::DirectConnection, g[0], g[1], g[2], g[3], g[4], g[5]);
        }
        if (!ok) {
            if (error) *error = u"call failed: "_s + target;
            return {};
        }
        return ret.isValid() ? QJsonValue::fromVariant(ret) : QJsonValue();
    }
    if (error) *error = u"no method %1 taking %2 argument(s)"_s.arg(target).arg(list.size());
    return {};
}

QJsonValue TestDriver::stats() const
{
    // Every 3D view (corv::SceneItem), read through its properties
    QJsonArray views;
    QObject *root = m_engine.rootObjects().value(0);
    if (root) {
        for (QQuickItem *it : root->findChildren<QQuickItem *>()) {
            if (!it->inherits("corv::SceneItem")) continue;   // QML derives a type of its own
            QJsonObject v;
            v.insert(u"objectName"_s, it->objectName());
            v.insert(u"visible"_s, it->isVisible());
            v.insert(u"width"_s, it->width());
            v.insert(u"height"_s, it->height());
            for (const char *p : {"graphicsApi", "fps", "chunks", "visibleChunks", "drawCalls", "satelliteImages", "satelliteMB", "agl",
                                  "schematic", "thirdPerson", "trajectory", "orbitDistance", "orbitPitch"})
                v.insert(QString::fromLatin1(p), QJsonValue::fromVariant(it->property(p)));
            views.append(v);
        }
    }
    QJsonObject out;
    out.insert(u"views"_s, views);
    // The frames drawn since the last "stats"
    const FrameStats::Summary f = FrameStats::instance().summary();
    FrameStats::instance().reset();
    auto r1 = [](double v) { return std::round(v * 10) / 10; };
    out.insert(u"frames"_s, QJsonObject{{u"count"_s, f.frames}, {u"late"_s, f.late}, {u"dropped"_s, f.dropped},
                                        {u"medianMs"_s, r1(f.median)}, {u"p95Ms"_s, r1(f.p95)}, {u"p99Ms"_s, r1(f.p99)},
                                        {u"worstMs"_s, r1(f.worst)}, {u"renderWorstMs"_s, r1(f.renderWorst)},
                                        {u"renderP99Ms"_s, r1(f.renderP99)}, {u"syncWorstMs"_s, r1(f.syncWorst)},
                                        {u"swapWorstMs"_s, r1(f.swapWorst)}, {u"swapP99Ms"_s, r1(f.swapP99)},
                                        {u"guiWorstMs"_s, r1(f.guiWorst)}, {u"guiP99Ms"_s, r1(f.guiP99)}});
    if (QQuickWindow *w = window()) {
        out.insert(u"window"_s, QJsonObject{{u"width"_s, w->width()}, {u"height"_s, w->height()},
                                                         {u"dpr"_s, w->effectiveDevicePixelRatio()},
                                                         {u"exposed"_s, w->isExposed()}});
    }
    return out;
}

QJsonValue TestDriver::state() const
{
    if (!VehicleModel::s_instance) return {};
    const VehicleState &s = *VehicleModel::s_instance->state();
    QJsonObject o{
        {u"mode"_s, s.mode}, {u"connected"_s, s.connected}, {u"connectionType"_s, s.connectionType},
        {u"vehicleType"_s, s.vehicleType}, {u"systemId"_s, s.systemId}, {u"componentId"_s, s.componentId},
        {u"armed"_s, s.armed}, {u"flightMode"_s, s.flightMode}, {u"heartbeatCount"_s, qint64(s.heartbeatCount)},
        {u"lastHeartbeatTime"_s, qint64(s.lastHeartbeatTime)}, {u"firmwareVersion"_s, s.firmwareVersion},
        {u"roll"_s, s.roll}, {u"pitch"_s, s.pitch}, {u"yaw"_s, s.yaw},
        {u"rollRate"_s, s.rollRate}, {u"pitchRate"_s, s.pitchRate}, {u"yawRate"_s, s.yawRate},
        {u"lat"_s, s.lat}, {u"lon"_s, s.lon}, {u"rawAlt"_s, s.rawAlt}, {u"offsetAlt"_s, s.offsetAlt},
        {u"subSurfaceAlt"_s, opt(s.subSurfaceAlt)}, {u"rangefinder"_s, opt(s.rangefinderDist)},
        {u"as"_s, s.as}, {u"gs"_s, s.gs}, {u"vs"_s, s.vs}, {u"vn"_s, s.vn}, {u"ve"_s, s.ve}, {u"vd"_s, s.vd},
        {u"ax"_s, s.ax}, {u"ay"_s, s.ay}, {u"az"_s, s.az},
        {u"homeLat"_s, opt(s.homeLat)}, {u"homeLon"_s, opt(s.homeLon)}, {u"homeAlt"_s, opt(s.homeAlt)},
        {u"batteryVoltage"_s, s.batteryVoltage}, {u"batteryCurrent"_s, s.batteryCurrent}, {u"batteryRemaining"_s, s.batteryRemaining},
        {u"gpsFix"_s, s.gpsFix}, {u"gpsNumSat"_s, s.gpsNumSat}, {u"gpsHdop"_s, s.gpsHdop},
        {u"preArm"_s, int(s.preArm())}, {u"preArmText"_s, s.preArmText},
        {u"ekfFlags"_s, s.ekfFlags}, {u"ekfVariance"_s, s.ekfVariance},
        {u"missionCount"_s, s.missionCount}, {u"missionSeq"_s, s.missionCurrentSeq},
        {u"wpDist"_s, s.wpDist}, {u"xtrackError"_s, s.xtrackError},
        {u"windDir"_s, s.windDir}, {u"windSpeed"_s, s.windSpeed},
        {u"appWindDir"_s, s.appWindDir}, {u"appWindSpeed"_s, s.appWindSpeed},
        {u"terrainPending"_s, s.terrainPending}, {u"terrainLoaded"_s, s.terrainLoaded},
        {u"statusText"_s, s.statusText}, {u"statusSeverity"_s, s.statusSeverity},
        {u"ackCommand"_s, s.lastAckCommand}, {u"ackResult"_s, s.lastAckResult},
        {u"originLat"_s, s.sceneOrigin.lat}, {u"originLon"_s, s.sceneOrigin.lon}, {u"originEpoch"_s, s.sceneOriginEpoch},
        {u"relativeMode"_s, s.relativeMode}, {u"relativeSource"_s, s.relativeSource}, {u"relativeVelSource"_s, s.relativeVelSource},
        {u"vibX"_s, s.vibX}, {u"vibY"_s, s.vibY}, {u"vibZ"_s, s.vibZ},
    };
    QJsonArray rc, servo, traffic;
    QJsonArray rpm, esc;
    const std::int64_t now = wallMs();
    const auto turn = [](std::optional<double> v) { return v ? QJsonValue(*v) : QJsonValue(); };
    for (int n = 1; n <= 2; ++n) rpm.append(turn(s.sensorRpm(n, now)));
    for (int n = 1; n <= int(s.esc.size()); ++n) esc.append(turn(s.escRpm(n, now)));
    o.insert(u"rpm"_s, rpm);
    o.insert(u"esc"_s, esc);
    for (const auto v : s.rcChannels) rc.append(int(v));
    for (const auto v : s.servoOutputs) servo.append(int(v));
    for (const TrafficEntry &t : s.traffic)
        traffic.append(QJsonObject{{u"icao24"_s, t.icao24}, {u"callsign"_s, t.callsign}, {u"lat"_s, t.lat}, {u"lon"_s, t.lon},
                                   {u"alt"_s, t.alt}, {u"heading"_s, t.heading}, {u"velocity"_s, t.velocity}});
    o.insert(u"rc"_s, rc);
    o.insert(u"servo"_s, servo);
    o.insert(u"traffic"_s, traffic);
    return o;
}

QJsonValue TestDriver::props(const QString &name, QString *error) const
{
    QObject *o = object(name);
    if (!o) {
        *error = u"no such object"_s;
        return {};
    }
    // A view model: what it declares; a QML item: what its own type declares (not the Item's)
    QJsonObject out;
    const QMetaObject *mo = o->metaObject();
    const int from = o->inherits("QQuickItem") ? QQuickItem::staticMetaObject.propertyCount() : QObject::staticMetaObject.propertyCount();
    for (int i = from; i < mo->propertyCount(); ++i) {
        const QMetaProperty p = mo->property(i);
        if (p.isReadable()) out.insert(QString::fromLatin1(p.name()), toJson(p.read(o)));
    }
    return out;
}

QJsonValue TestDriver::rows(const QString &name, const QString &property, int from, int max, QString *error) const
{
    QObject *o = object(name);
    const QVariant v = o ? o->property(property.toLatin1().constData()) : QVariant();
    const auto *model = qobject_cast<QAbstractItemModel *>(v.value<QObject *>());
    if (!model) {
        *error = u"no such model"_s;
        return {};
    }
    QJsonArray out;
    const auto roles = model->roleNames();
    const int count = model->rowCount();
    for (int r = std::max(0, from); r < count && out.size() < max; ++r) {
        QJsonObject row;
        for (auto it = roles.cbegin(); it != roles.cend(); ++it)
            row.insert(QString::fromLatin1(it.value()), toJson(model->data(model->index(r, 0), it.key())));
        out.append(row);
    }
    return QJsonObject{{u"count"_s, count}, {u"rows"_s, out}};
}

void TestDriver::addEvent(QJsonObject e)
{
    e.insert(u"n"_s, ++m_eventSeq);
    e.insert(u"time"_s, QDateTime::currentMSecsSinceEpoch());
    m_events.append(std::move(e));
    if (m_events.size() > kMaxEvents) m_events.removeFirst();
}

QJsonValue TestDriver::events(qint64 since) const
{
    QJsonArray out;
    for (const QJsonObject &e : m_events)
        if (e.value(u"n"_s).toInteger() > since) out.append(e);
    return QJsonObject{{u"last"_s, m_eventSeq}, {u"events"_s, out}};
}

void TestDriver::recordEvents()
{
    auto said = [this](const QString &kind, const QString &text, const QString &level) {
        addEvent(QJsonObject{{u"kind"_s, kind}, {u"text"_s, text}, {u"level"_s, level}});
    };
    auto toast = [said](const QString &text) { said(u"toast"_s, text, QString()); };
    auto confirm = [this](int token, const QString &text) {
        addEvent(QJsonObject{{u"kind"_s, u"confirm"_s}, {u"token"_s, token}, {u"text"_s, text}});
    };
    auto prompt = [this](int token, const QString &text, const QString &value) {
        addEvent(QJsonObject{{u"kind"_s, u"prompt"_s}, {u"token"_s, token}, {u"text"_s, text}, {u"value"_s, value}});
    };
    if (auto *hub = DialogHub::s_instance) {
        QObject::connect(hub, &DialogHub::toastRequested, &m_context, toast);
        QObject::connect(hub, &DialogHub::confirmRequested, &m_context, confirm);
        QObject::connect(hub, &DialogHub::promptRequested, &m_context, prompt);
    }
    if (auto *plan = PlanController::s_instance) {
        QObject::connect(plan, &PlanController::toast, &m_context, toast);
        QObject::connect(plan, &PlanController::confirmRequested, &m_context, confirm);
        QObject::connect(plan, &PlanController::promptRequested, &m_context, prompt);
    }
    if (auto *sys = SysConfig::s_instance) QObject::connect(sys, &SysConfig::toast, &m_context, toast);
    if (auto *gcs = GcsController::s_instance)
        QObject::connect(gcs, &GcsController::hudMessage, &m_context,
                         [said](const QString &text, const QString &level) { said(u"hud"_s, text, level); });
}

QJsonValue TestDriver::run(const QString &cmd, const QString &arg, QString *error)
{
    QQuickWindow *win = window();
    QObject *hdr = header();
    auto fail = [&](const QString &e) {
        if (error) *error = e;
        return QJsonValue();
    };
    if (cmd == "tab"_L1) {
        if (!hdr) return fail(u"no header"_s);
        hdr->setProperty("currentTab", arg.toInt());
        return {};
    }
    if (cmd == "grab"_L1) {
        if (!win) return fail(u"no window"_s);
        const bool ok = win->grabWindow().save(arg);
        LOG_I("app") << "grab" << arg << ok;
        return ok ? QJsonValue(arg) : fail(u"cannot save "_s + arg);
    }
    if (cmd == "select"_L1) {
        if (!PlanController::s_instance) return fail(u"no plan"_s);
        const QJsonArray segs = PlanController::s_instance->route().value(u"segments"_s).toArray();
        const int i = arg.toInt();
        PlanController::s_instance->selectSegment(i >= 0 && i < segs.size() ? segs[i].toObject().value(u"id"_s).toString() : QString());
        return {};
    }
    if (cmd == "popover"_L1) {
        if (!PlanController::s_instance) return fail(u"no plan"_s);
        emit PlanController::s_instance->openPopover(arg == "none"_L1 ? QString() : arg);
        return {};
    }
    if (cmd == "library"_L1) {
        if (!PlanController::s_instance) return fail(u"no plan"_s);
        if (arg == "open"_L1) PlanController::s_instance->openLibrary();
        else emit PlanController::s_instance->libraryOpenChanged(false);
        return {};
    }
    if (cmd == "emit"_L1) {
        // A view button of the header bar: toggleSatellite, toggleTheme, toggleHorizonLock...
        if (!hdr || !QMetaObject::invokeMethod(hdr, arg.toLatin1().constData())) return fail(u"no signal "_s + arg);
        return {};
    }
    if (cmd == "call"_L1) {
        // "call Params.readAll", "call Gcs.setMode FBWA", "call Rtk.ntripConnect {...}",
        // "call SetupCfg.readParams [["FLTMODE1"],"FLIGHT MODES"]"
        const QString target = arg.section(' '_L1, 0, 0), param = arg.section(' '_L1, 1);
        QJsonValue args;
        if (param.startsWith('{'_L1) || param.startsWith('['_L1)) {
            const QJsonDocument json = QJsonDocument::fromJson(param.toUtf8());
            args = json.isArray() ? QJsonValue(json.array()) : QJsonValue(QJsonArray{json.object()});
        } else if (!param.isEmpty()) {
            args = QJsonArray{param};
        }
        QString e;
        const QJsonValue r = call(target, args, &e);
        LOG_I("app") << "call" << arg << (e.isEmpty() ? QString() : u"(%1)"_s.arg(e));
        if (!e.isEmpty()) return fail(e);
        return r;
    }
    if (cmd == "set"_L1 || cmd == "get"_L1) {
        // A property of a view model or a named QML item: "set setupPage current PARAMETERS"
        const QStringList a = arg.split(' '_L1);
        QObject *o = a.size() >= 2 ? object(a[0]) : nullptr;
        if (!o) return fail(u"no such object "_s + a.value(0));
        if (cmd == "get"_L1) return QJsonValue::fromVariant(o->property(a[1].toLatin1().constData()));
        if (a.size() < 3) return fail(u"set needs a value"_s);
        o->setProperty(a[1].toLatin1().constData(), a.mid(2).join(' '_L1));
        LOG_I("app") << "set" << arg;
        return {};
    }
    if (cmd == "key"_L1 || cmd == "keyup"_L1) {
        // An arrow held or released: "key Left", "keyup Left"
        if (!win) return fail(u"no window"_s);
        static const QHash<QString, int> keys = {{u"Up"_s, Qt::Key_Up}, {u"Down"_s, Qt::Key_Down},
                                                 {u"Left"_s, Qt::Key_Left}, {u"Right"_s, Qt::Key_Right}};
        QKeyEvent ev(cmd == "key"_L1 ? QEvent::KeyPress : QEvent::KeyRelease, keys.value(arg, Qt::Key_unknown), Qt::NoModifier);
        QCoreApplication::sendEvent(win, &ev);
        return {};
    }
    if (cmd == "click"_L1) {
        // A left click at window coordinates: "click 120 340" (opens a select, presses a button)
        const QStringList xy = arg.split(' '_L1, Qt::SkipEmptyParts);
        if (!win || xy.size() != 2) return fail(u"click X Y"_s);
        const QPointF pos(xy[0].toDouble(), xy[1].toDouble());
        const QPointF global = win->mapToGlobal(pos);
        QMouseEvent press(QEvent::MouseButtonPress, pos, global, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QCoreApplication::sendEvent(win, &press);
        QMouseEvent release(QEvent::MouseButtonRelease, pos, global, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
        QCoreApplication::sendEvent(win, &release);
        return {};
    }
    if (cmd == "touch"_L1) {
        // Fingers on the window: "touch press|move|release X Y [X Y]", as the system reports them
        const QStringList a = arg.split(' '_L1, Qt::SkipEmptyParts);
        if (!win || a.size() < 3 || a.size() % 2 == 0) return fail(u"touch press|move|release X Y [X Y]"_s);
        static const QPointingDevice *device = [] {
            auto *d = new QPointingDevice(u"test touch"_s, 4711, QInputDevice::DeviceType::TouchScreen, QPointingDevice::PointerType::Finger,
                                          QInputDevice::Capability::Position | QInputDevice::Capability::Area, 10, 0);
            QWindowSystemInterface::registerInputDevice(d);
            return d;
        }();
        static QList<int> down;
        const auto state = a[0] == "press"_L1 ? QEventPoint::State::Pressed : a[0] == "release"_L1 ? QEventPoint::State::Released : QEventPoint::State::Updated;
        const qreal dpr = win->effectiveDevicePixelRatio();
        QList<QWindowSystemInterface::TouchPoint> points;
        for (int i = 0; 2 * i + 2 < a.size(); ++i) {
            QWindowSystemInterface::TouchPoint p;
            p.id = i + 1;
            // A finger that was already down is not pressed again
            p.state = state == QEventPoint::State::Pressed && down.contains(p.id) ? QEventPoint::State::Updated : state;
            const QPointF g = win->mapToGlobal(QPointF(a[1 + 2 * i].toDouble(), a[2 + 2 * i].toDouble())) * dpr;
            p.area = QRectF(g - QPointF(4, 4), QSizeF(8, 8));
            p.pressure = state == QEventPoint::State::Released ? 0 : 1;
            points << p;
            if (state == QEventPoint::State::Pressed && !down.contains(p.id)) down << p.id;
            if (state == QEventPoint::State::Released) down.removeAll(p.id);
        }
        QWindowSystemInterface::handleTouchEvent<QWindowSystemInterface::SynchronousDelivery>(win, device, points);
        return {};
    }
    if (cmd == "size"_L1) {
        const QStringList wh = arg.split('x'_L1);
        if (!win || wh.size() != 2) return fail(u"size WxH"_s);
        win->resize(wh[0].toInt(), wh[1].toInt());
        return {};
    }
    if (cmd == "stats"_L1) return stats();
    return fail(u"unknown command "_s + cmd);
}

QJsonValue TestDriver::runJson(const QString &method, const QJsonValue &params, QString *error)
{
    const QJsonObject p = params.toObject();
    if (method == "set"_L1) {
        // {object, property, value}: the value keeps its JSON type
        QObject *o = object(p.value(u"object"_s).toString());
        if (!o) {
            *error = u"no such object"_s;
            return {};
        }
        const QByteArray prop = p.value(u"property"_s).toString().toLatin1();
        if (!o->setProperty(prop.constData(), toVariant(p.value(u"value"_s)))) {
            *error = u"property not set: "_s + QString::fromLatin1(prop);
            return {};
        }
        return QJsonValue::fromVariant(o->property(prop.constData()));
    }
    if (method == "get"_L1) {
        QObject *o = object(p.value(u"object"_s).toString());
        if (!o) {
            *error = u"no such object"_s;
            return {};
        }
        return QJsonValue::fromVariant(o->property(p.value(u"property"_s).toString().toLatin1().constData()));
    }
    if (method == "call"_L1) return call(p.value(u"target"_s).toString(), p.value(u"args"_s), error);
    if (method == "stats"_L1) return stats();
    if (method == "state"_L1) return state();
    if (method == "props"_L1) return props(p.value(u"object"_s).toString(), error);
    if (method == "rows"_L1)
        return rows(p.value(u"object"_s).toString(), p.value(u"property"_s).toString(), p.value(u"from"_s).toInt(),
                    p.value(u"max"_s).toInt(10000), error);
    if (method == "events"_L1) return events(p.value(u"since"_s).toInteger());
    if (method == "quit"_L1) {
        // As the operator closes the window, once this answer is out
        if (QQuickWindow *win = window()) QMetaObject::invokeMethod(win, &QWindow::close, Qt::QueuedConnection);
        return true;
    }
    if (method == "screenshot"_L1) {
        QQuickWindow *win = window();
        if (!win) {
            *error = u"no window"_s;
            return {};
        }
        const QImage img = win->grabWindow();
        const QString file = p.value(u"file"_s).toString();
        if (!file.isEmpty()) {
            if (!img.save(file)) *error = u"cannot save "_s + file;
            return QJsonObject{{u"file"_s, file}, {u"width"_s, img.width()}, {u"height"_s, img.height()}};
        }
        // No file: the PNG itself, base64
        QByteArray png;
        QBuffer buf(&png);
        buf.open(QIODevice::WriteOnly);
        img.save(&buf, "PNG");
        return QJsonObject{{u"png"_s, QString::fromLatin1(png.toBase64())}, {u"width"_s, img.width()},
                           {u"height"_s, img.height()}};
    }
    if (method == "script"_L1) {
        // Any --grab-script line: {"line": "tab 1"}
        const QString line = p.value(u"line"_s).toString().trimmed();
        return run(line.section(' '_L1, 0, 0), line.section(' '_L1, 1), error);
    }
    *error = u"unknown method "_s + method;
    return {};
}

} // namespace corv
