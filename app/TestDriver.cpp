#include "TestDriver.h"

#include "core/DebugLog.h"
#include "ui/AnnunciatorController.h"
#include "ui/CalibrationController.h"
#include "ui/EkfController.h"
#include "ui/FftController.h"
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

#include <QBuffer>
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QMetaMethod>
#include <QQmlApplicationEngine>
#include <QQuickItem>
#include <QQuickWindow>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
QVariant toVariant(const QJsonValue &v)
{
    if (v.isObject()) return v.toObject().toVariantMap();
    if (v.isArray()) return v.toArray().toVariantList();
    return v.toVariant();
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
    return root ? root->findChild<QObject *>(name) : nullptr;
}

QJsonValue TestDriver::call(const QString &target, const QJsonValue &args, QString *error)
{
    const QString obj = target.section('.'_L1, 0, 0), method = target.section('.'_L1, 1);
    QObject *o = viewModel(obj);
    if (!o) {
        if (error) *error = u"no view model "_s + obj;
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
        if (mm.returnType() != QMetaType::Void && mm.returnType() != QMetaType::UnknownType) {
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
            if (qstrcmp(it->metaObject()->className(), "corv::SceneItem") != 0) continue;
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
    if (QQuickWindow *w = window()) {
        out.insert(u"window"_s, QJsonObject{{u"width"_s, w->width()}, {u"height"_s, w->height()},
                                                         {u"dpr"_s, w->effectiveDevicePixelRatio()},
                                                         {u"exposed"_s, w->isExposed()}});
    }
    return out;
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
