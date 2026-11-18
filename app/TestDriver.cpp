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
    if (name == QLatin1String("Params")) return ParamsController::s_instance;
    if (name == QLatin1String("Logs")) return LogsController::s_instance;
    if (name == QLatin1String("Plan")) return PlanController::s_instance;
    if (name == QLatin1String("Gcs")) return GcsController::s_instance;
    if (name == QLatin1String("Calibration")) return CalibrationController::s_instance;
    if (name == QLatin1String("Ekf")) return EkfController::s_instance;
    if (name == QLatin1String("Fft")) return FftController::s_instance;
    if (name == QLatin1String("Replay")) return ReplayController::s_instance;
    if (name == QLatin1String("Annunciators")) return AnnunciatorController::s_instance;
    if (name == QLatin1String("Rtk")) return RtkController::s_instance;
    if (name == QLatin1String("TelForward")) return TelForwardController::s_instance;
    if (name == QLatin1String("Updates")) return UpdateController::s_instance;
    if (name == QLatin1String("Lte")) return LteController::s_instance;
    if (name == QLatin1String("SetupCfg")) return SetupController::s_instance;
    if (name == QLatin1String("Sitl")) return SitlController::s_instance;
    if (name == QLatin1String("GeoZones")) return GeoZoneController::s_instance;
    if (name == QLatin1String("SysConfig")) return SysConfig::s_instance;
    if (name == QLatin1String("Joystick")) return JoystickController::s_instance;
    if (name == QLatin1String("Lidar")) return LidarController::s_instance;
    if (name == QLatin1String("Ros")) return RosController::s_instance;
    return nullptr;
}

QQuickWindow *TestDriver::window() const
{
    return qobject_cast<QQuickWindow *>(m_engine.rootObjects().value(0));
}

QObject *TestDriver::header() const
{
    QObject *root = m_engine.rootObjects().value(0);
    return root ? root->findChild<QObject *>(QStringLiteral("header")) : nullptr;
}

QObject *TestDriver::object(const QString &name) const
{
    if (QObject *vm = viewModel(name)) return vm;
    QObject *root = m_engine.rootObjects().value(0);
    return root ? root->findChild<QObject *>(name) : nullptr;
}

QJsonValue TestDriver::call(const QString &target, const QJsonValue &args, QString *error)
{
    const QString obj = target.section(QLatin1Char('.'), 0, 0), method = target.section(QLatin1Char('.'), 1);
    QObject *o = viewModel(obj);
    if (!o) {
        if (error) *error = QStringLiteral("no view model ") + obj;
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
            if (error) *error = QStringLiteral("call failed: ") + target;
            return {};
        }
        return ret.isValid() ? QJsonValue::fromVariant(ret) : QJsonValue();
    }
    if (error) *error = QStringLiteral("no method %1 taking %2 argument(s)").arg(target).arg(list.size());
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
            v.insert(QStringLiteral("objectName"), it->objectName());
            v.insert(QStringLiteral("visible"), it->isVisible());
            v.insert(QStringLiteral("width"), it->width());
            v.insert(QStringLiteral("height"), it->height());
            for (const char *p : {"graphicsApi", "fps", "chunks", "visibleChunks", "drawCalls", "satelliteImages", "satelliteMB", "agl",
                                  "schematic", "thirdPerson", "trajectory", "orbitDistance", "orbitPitch"})
                v.insert(QString::fromLatin1(p), QJsonValue::fromVariant(it->property(p)));
            views.append(v);
        }
    }
    QJsonObject out;
    out.insert(QStringLiteral("views"), views);
    if (QQuickWindow *w = window()) {
        out.insert(QStringLiteral("window"), QJsonObject{{QStringLiteral("width"), w->width()}, {QStringLiteral("height"), w->height()},
                                                         {QStringLiteral("dpr"), w->effectiveDevicePixelRatio()},
                                                         {QStringLiteral("exposed"), w->isExposed()}});
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
    if (cmd == QLatin1String("tab")) {
        if (!hdr) return fail(QStringLiteral("no header"));
        hdr->setProperty("currentTab", arg.toInt());
        return {};
    }
    if (cmd == QLatin1String("grab")) {
        if (!win) return fail(QStringLiteral("no window"));
        const bool ok = win->grabWindow().save(arg);
        LOG_I("app") << "grab" << arg << ok;
        return ok ? QJsonValue(arg) : fail(QStringLiteral("cannot save ") + arg);
    }
    if (cmd == QLatin1String("select")) {
        if (!PlanController::s_instance) return fail(QStringLiteral("no plan"));
        const QJsonArray segs = PlanController::s_instance->route().value(QStringLiteral("segments")).toArray();
        const int i = arg.toInt();
        PlanController::s_instance->selectSegment(i >= 0 && i < segs.size() ? segs[i].toObject().value(QStringLiteral("id")).toString() : QString());
        return {};
    }
    if (cmd == QLatin1String("popover")) {
        if (!PlanController::s_instance) return fail(QStringLiteral("no plan"));
        emit PlanController::s_instance->openPopover(arg == QLatin1String("none") ? QString() : arg);
        return {};
    }
    if (cmd == QLatin1String("library")) {
        if (!PlanController::s_instance) return fail(QStringLiteral("no plan"));
        if (arg == QLatin1String("open")) PlanController::s_instance->openLibrary();
        else emit PlanController::s_instance->libraryOpenChanged(false);
        return {};
    }
    if (cmd == QLatin1String("emit")) {
        // A view button of the header bar: toggleSatellite, toggleTheme, toggleHorizonLock...
        if (!hdr || !QMetaObject::invokeMethod(hdr, arg.toLatin1().constData())) return fail(QStringLiteral("no signal ") + arg);
        return {};
    }
    if (cmd == QLatin1String("call")) {
        // "call Params.readAll", "call Gcs.setMode FBWA", "call Rtk.ntripConnect {...}",
        // "call SetupCfg.readParams [["FLTMODE1"],"FLIGHT MODES"]"
        const QString target = arg.section(QLatin1Char(' '), 0, 0), param = arg.section(QLatin1Char(' '), 1);
        QJsonValue args;
        if (param.startsWith(QLatin1Char('{')) || param.startsWith(QLatin1Char('['))) {
            const QJsonDocument json = QJsonDocument::fromJson(param.toUtf8());
            args = json.isArray() ? QJsonValue(json.array()) : QJsonValue(QJsonArray{json.object()});
        } else if (!param.isEmpty()) {
            args = QJsonArray{param};
        }
        QString e;
        const QJsonValue r = call(target, args, &e);
        LOG_I("app") << "call" << arg << (e.isEmpty() ? QString() : QStringLiteral("(%1)").arg(e));
        if (!e.isEmpty()) return fail(e);
        return r;
    }
    if (cmd == QLatin1String("set") || cmd == QLatin1String("get")) {
        // A property of a view model or a named QML item: "set setupPage current PARAMETERS"
        const QStringList a = arg.split(QLatin1Char(' '));
        QObject *o = a.size() >= 2 ? object(a[0]) : nullptr;
        if (!o) return fail(QStringLiteral("no such object ") + a.value(0));
        if (cmd == QLatin1String("get")) return QJsonValue::fromVariant(o->property(a[1].toLatin1().constData()));
        if (a.size() < 3) return fail(QStringLiteral("set needs a value"));
        o->setProperty(a[1].toLatin1().constData(), a.mid(2).join(QLatin1Char(' ')));
        LOG_I("app") << "set" << arg;
        return {};
    }
    if (cmd == QLatin1String("key") || cmd == QLatin1String("keyup")) {
        // An arrow held or released: "key Left", "keyup Left"
        if (!win) return fail(QStringLiteral("no window"));
        static const QHash<QString, int> keys = {{QStringLiteral("Up"), Qt::Key_Up}, {QStringLiteral("Down"), Qt::Key_Down},
                                                 {QStringLiteral("Left"), Qt::Key_Left}, {QStringLiteral("Right"), Qt::Key_Right}};
        QKeyEvent ev(cmd == QLatin1String("key") ? QEvent::KeyPress : QEvent::KeyRelease, keys.value(arg, Qt::Key_unknown), Qt::NoModifier);
        QCoreApplication::sendEvent(win, &ev);
        return {};
    }
    if (cmd == QLatin1String("click")) {
        // A left click at window coordinates: "click 120 340" (opens a select, presses a button)
        const QStringList xy = arg.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        if (!win || xy.size() != 2) return fail(QStringLiteral("click X Y"));
        const QPointF pos(xy[0].toDouble(), xy[1].toDouble());
        const QPointF global = win->mapToGlobal(pos);
        QMouseEvent press(QEvent::MouseButtonPress, pos, global, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QCoreApplication::sendEvent(win, &press);
        QMouseEvent release(QEvent::MouseButtonRelease, pos, global, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
        QCoreApplication::sendEvent(win, &release);
        return {};
    }
    if (cmd == QLatin1String("size")) {
        const QStringList wh = arg.split(QLatin1Char('x'));
        if (!win || wh.size() != 2) return fail(QStringLiteral("size WxH"));
        win->resize(wh[0].toInt(), wh[1].toInt());
        return {};
    }
    if (cmd == QLatin1String("stats")) return stats();
    return fail(QStringLiteral("unknown command ") + cmd);
}

QJsonValue TestDriver::runJson(const QString &method, const QJsonValue &params, QString *error)
{
    const QJsonObject p = params.toObject();
    if (method == QLatin1String("set")) {
        // {object, property, value}: the value keeps its JSON type
        QObject *o = object(p.value(QStringLiteral("object")).toString());
        if (!o) {
            *error = QStringLiteral("no such object");
            return {};
        }
        const QByteArray prop = p.value(QStringLiteral("property")).toString().toLatin1();
        if (!o->setProperty(prop.constData(), toVariant(p.value(QStringLiteral("value"))))) {
            *error = QStringLiteral("property not set: ") + QString::fromLatin1(prop);
            return {};
        }
        return QJsonValue::fromVariant(o->property(prop.constData()));
    }
    if (method == QLatin1String("get")) {
        QObject *o = object(p.value(QStringLiteral("object")).toString());
        if (!o) {
            *error = QStringLiteral("no such object");
            return {};
        }
        return QJsonValue::fromVariant(o->property(p.value(QStringLiteral("property")).toString().toLatin1().constData()));
    }
    if (method == QLatin1String("call")) return call(p.value(QStringLiteral("target")).toString(), p.value(QStringLiteral("args")), error);
    if (method == QLatin1String("stats")) return stats();
    if (method == QLatin1String("screenshot")) {
        QQuickWindow *win = window();
        if (!win) {
            *error = QStringLiteral("no window");
            return {};
        }
        const QImage img = win->grabWindow();
        const QString file = p.value(QStringLiteral("file")).toString();
        if (!file.isEmpty()) {
            if (!img.save(file)) *error = QStringLiteral("cannot save ") + file;
            return QJsonObject{{QStringLiteral("file"), file}, {QStringLiteral("width"), img.width()}, {QStringLiteral("height"), img.height()}};
        }
        // No file: the PNG itself, base64
        QByteArray png;
        QBuffer buf(&png);
        buf.open(QIODevice::WriteOnly);
        img.save(&buf, "PNG");
        return QJsonObject{{QStringLiteral("png"), QString::fromLatin1(png.toBase64())}, {QStringLiteral("width"), img.width()},
                           {QStringLiteral("height"), img.height()}};
    }
    if (method == QLatin1String("script")) {
        // Any --grab-script line: {"line": "tab 1"}
        const QString line = p.value(QStringLiteral("line")).toString().trimmed();
        return run(line.section(QLatin1Char(' '), 0, 0), line.section(QLatin1Char(' '), 1), error);
    }
    *error = QStringLiteral("unknown method ") + method;
    return {};
}

} // namespace corv
