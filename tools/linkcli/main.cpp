// corv-link-cli - phase 2 of the port: link and state without any interface.
//
//   corv-link-cli tcp 127.0.0.1 5760        SITL
//   corv-link-cli udp 0.0.0.0 14550         radio / MAVProxy output
//   corv-link-cli serial COM5 57600
//   corv-link-cli msp COM5 115200 [--slow]  INAV / Betaflight over MSP (msptcp HOST PORT: INAV SITL)
//   corv-link-cli lte relay.example 5765 --module ID --key HEX [--cert SHA256]
//                                           LTEtelem module through the relay
//   corv-link-cli ports                     list serial ports
//
// Options: --seconds N (stop after N s), --arm, --mode NAME, --takeoff ALT.
// Prints one state line a second, the same fields the debug log's vehicle line has.
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "link/SerialTransport.h"
#include "vehicle/CommandSender.h"
#include "vehicle/ConnectionManager.h"
#include "vehicle/FlightModes.h"
#include "vehicle/VehicleHub.h"

#include <QCommandLineParser>
#include <QCoreApplication>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <cstdio>

using namespace corv;

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("corv-link-cli"));
    QCommandLineParser p;
    p.setApplicationDescription(QStringLiteral("CORV GCS link and vehicle state, no interface"));
    p.addHelpOption();
    p.addPositionalArgument(QStringLiteral("type"), QStringLiteral("tcp | udp | serial | msp | msptcp | lte | ports"));
    p.addPositionalArgument(QStringLiteral("address"), QStringLiteral("host or serial port"));
    p.addPositionalArgument(QStringLiteral("port"), QStringLiteral("TCP/UDP port or baud rate"));
    QCommandLineOption seconds(QStringLiteral("seconds"), QStringLiteral("Stop after N seconds"), QStringLiteral("N"), QStringLiteral("0"));
    QCommandLineOption arm(QStringLiteral("arm"), QStringLiteral("Arm once the vehicle is linked"));
    QCommandLineOption mode(QStringLiteral("mode"), QStringLiteral("Set this flight mode once linked"), QStringLiteral("NAME"));
    QCommandLineOption takeoff(QStringLiteral("takeoff"), QStringLiteral("GUIDED, arm and take off to ALT m"), QStringLiteral("ALT"));
    QCommandLineOption module(QStringLiteral("module"), QStringLiteral("lte: module (vehicle) ID"), QStringLiteral("ID"));
    QCommandLineOption key(QStringLiteral("key"), QStringLiteral("lte: 256-bit module key, 64 hex characters"), QStringLiteral("HEX"));
    QCommandLineOption cert(QStringLiteral("cert"), QStringLiteral("lte: relay certificate SHA-256 to pin"), QStringLiteral("SHA256"));
    QCommandLineOption slow(QStringLiteral("slow"), QStringLiteral("msp: the slow poll profile (long-range radio)"));
    p.addOptions({seconds, arm, mode, takeoff, module, key, cert, slow});
    p.process(app);

    const QStringList args = p.positionalArguments();
    QTextStream out(stdout);
    if (args.value(0) == QLatin1String("ports")) {
        for (const auto &i : SerialTransport::listPorts())
            out << i.path << "  " << i.friendlyName << "  " << i.vendorId << ':' << i.productId << Qt::endl;
        return 0;
    }
    if (args.size() < 3) p.showHelp(1);

    DebugLog::init();
    Settings::instance().load();

    MessageBus bus;
    QThread linkThread;
    linkThread.setObjectName(QStringLiteral("link"));
    auto *link = new LinkManager(&bus);
    auto *hub = new VehicleHub(link);
    link->moveToThread(&linkThread);
    hub->moveToThread(&linkThread);
    QObject::connect(&linkThread, &QThread::finished, link, &QObject::deleteLater);
    QObject::connect(&linkThread, &QThread::finished, hub, &QObject::deleteLater);
    linkThread.start();

    CommandSender cmd(link, &bus, hub);
    ConnectionManager conn(link, hub, &bus, &cmd);

    QObject::connect(&conn, &ConnectionManager::connectFailed, &app, [&](const QString &, const QString &e) {
        out << "connect failed: " << e << Qt::endl;
        app.exit(2);
    });
    QObject::connect(hub, &VehicleHub::statusText, &app, [&](int sev, const QString &t) { out << "STATUSTEXT " << sev << ": " << t << Qt::endl; });
    QObject::connect(hub, &VehicleHub::commandAck, &app, [&](int c, int r) {
        out << "ACK " << mav::commandName(c) << ' ' << mav::resultName(uint8_t(r)) << Qt::endl;
    });

    const QString type = args[0];
    QVariantMap o;
    if (type == QLatin1String("tcp") || type == QLatin1String("udp")) {
        o[QStringLiteral("host")] = args[1];
        o[QStringLiteral("port")] = args[2].toInt();
        conn.connectLink(type == QLatin1String("tcp") ? QLatin1String(LinkManager::kMavlinkTcp) : QLatin1String(LinkManager::kMavlinkUdp), o);
    } else if (type == QLatin1String("serial")) {
        o[QStringLiteral("port")] = args[1];
        o[QStringLiteral("baudRate")] = args[2].toInt();
        conn.connectLink(QLatin1String(LinkManager::kMavlinkSerial), o);
    } else if (type == QLatin1String("msp") || type == QLatin1String("msptcp")) {
        o[QStringLiteral("profile")] = p.isSet(slow) ? QStringLiteral("slow") : QStringLiteral("normal");
        if (type == QLatin1String("msp")) {
            o[QStringLiteral("port")] = args[1];
            o[QStringLiteral("baudRate")] = args[2].toInt();
            conn.connectLink(QLatin1String(LinkManager::kMspSerial), o);
        } else {
            o[QStringLiteral("host")] = args[1];
            o[QStringLiteral("port")] = args[2].toInt();
            conn.connectLink(QLatin1String(LinkManager::kMspTcp), o);
        }
    } else if (type == QLatin1String("lte")) {
        o[QStringLiteral("host")] = args[1];
        o[QStringLiteral("port")] = args[2].toInt();
        o[QStringLiteral("moduleId")] = p.value(module);
        o[QStringLiteral("keyHex")] = p.value(key);
        o[QStringLiteral("certSha256")] = p.value(cert);
        QObject::connect(link, &LinkManager::lteStatus, &app, [&out](const QVariantMap &st) {
            static QString last;
            const QString line = QStringLiteral("LTE %1 air=%2 records in %3 out %4 authFail %5 replay %6")
                                     .arg(st.value(QStringLiteral("state")).toString(), st.value(QStringLiteral("airOnline")).toBool() ? QStringLiteral("online") : QStringLiteral("offline"))
                                     .arg(st.value(QStringLiteral("recordsIn")).toLongLong()).arg(st.value(QStringLiteral("recordsOut")).toLongLong())
                                     .arg(st.value(QStringLiteral("authFail")).toLongLong()).arg(st.value(QStringLiteral("replay")).toLongLong());
            if (line != last) out << line << Qt::endl;
            last = line;
        });
        conn.connectLink(QLatin1String(LinkManager::kMavlinkLte), o);
    } else {
        p.showHelp(1);
    }
    QObject::connect(link, &LinkManager::linkWarning, &app, [&out](const QString &t) { out << "WARNING " << t << Qt::endl; });

    bool acted = false;
    QTimer tick;
    tick.setInterval(1000);
    QObject::connect(&tick, &QTimer::timeout, &app, [&] {
        const auto s = hub->snapshot();
        out << QStringLiteral("%1 %2 %3 hb=%4 | %5 %6 | %7, %8 alt %9 m | att %10/%11/%12 deg | gs %13 as %14 vs %15 | bat %16 V %17% | gps %18 sats %19 | link %20 kbps")
                   .arg(s->connected ? QStringLiteral("LINK") : QStringLiteral("----"), s->connectionType, s->linkType)
                   .arg(s->heartbeatCount)
                   .arg(modes::vehicleTypeName(s->vehicleType), s->armed ? QStringLiteral("ARMED ") + s->flightMode : s->flightMode)
                   .arg(s->lat, 0, 'f', 6).arg(s->lon, 0, 'f', 6).arg(s->rawAlt, 0, 'f', 1)
                   .arg(s->roll * kRad, 0, 'f', 1).arg(s->pitch * kRad, 0, 'f', 1).arg(s->yaw * kRad, 0, 'f', 1)
                   .arg(s->gs, 0, 'f', 1).arg(s->as, 0, 'f', 1).arg(s->vs, 0, 'f', 1)
                   .arg(s->batteryVoltage, 0, 'f', 2).arg(s->batteryRemaining)
                   .arg(s->gpsFix).arg(s->gpsNumSat).arg(s->linkKbps, 0, 'f', 1)
            << Qt::endl;
        // Commands once the vehicle is linked. Arming is retried every 3 s until
        // the EKF uses the GPS (ArduPilot refuses it before), then the takeoff goes.
        static int ticks = 0;
        if (s->heartbeatCount > 2 && !acted && ++ticks % 3 == 0) {
            if (p.isSet(mode)) cmd.setFlightMode(p.value(mode));
            const bool wantsArm = p.isSet(arm) || p.isSet(takeoff);
            if (wantsArm && !s->armed) {
                if (p.isSet(takeoff)) cmd.setFlightMode(QStringLiteral("GUIDED"));
                cmd.armVehicle();
            } else {
                acted = true;
                if (p.isSet(takeoff)) cmd.takeoff(p.value(takeoff).toDouble());
            }
        }
    });
    tick.start();

    if (const int n = p.value(seconds).toInt(); n > 0)
        QTimer::singleShot(n * 1000, &app, [&] {
            conn.disconnectLink();
            QTimer::singleShot(300, &app, &QCoreApplication::quit);
        });

    const int rc = app.exec();
    linkThread.quit();
    linkThread.wait();
    DebugLog::shutdown();
    return rc;
}
