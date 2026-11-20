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

using namespace Qt::StringLiterals;

using namespace corv;

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(u"corv-link-cli"_s);
    QCommandLineParser p;
    p.setApplicationDescription(u"CORV GCS link and vehicle state, no interface"_s);
    p.addHelpOption();
    p.addPositionalArgument(u"type"_s, u"tcp | udp | serial | msp | msptcp | lte | ports"_s);
    p.addPositionalArgument(u"address"_s, u"host or serial port"_s);
    p.addPositionalArgument(u"port"_s, u"TCP/UDP port or baud rate"_s);
    QCommandLineOption seconds(u"seconds"_s, u"Stop after N seconds"_s, u"N"_s, u"0"_s);
    QCommandLineOption arm(u"arm"_s, u"Arm once the vehicle is linked"_s);
    QCommandLineOption mode(u"mode"_s, u"Set this flight mode once linked"_s, u"NAME"_s);
    QCommandLineOption takeoff(u"takeoff"_s, u"GUIDED, arm and take off to ALT m"_s, u"ALT"_s);
    QCommandLineOption module(u"module"_s, u"lte: module (vehicle) ID"_s, u"ID"_s);
    QCommandLineOption key(u"key"_s, u"lte: 256-bit module key, 64 hex characters"_s, u"HEX"_s);
    QCommandLineOption cert(u"cert"_s, u"lte: relay certificate SHA-256 to pin"_s, u"SHA256"_s);
    QCommandLineOption slow(u"slow"_s, u"msp: the slow poll profile (long-range radio)"_s);
    p.addOptions({seconds, arm, mode, takeoff, module, key, cert, slow});
    p.process(app);

    const QStringList args = p.positionalArguments();
    QTextStream out(stdout);
    if (args.value(0) == "ports"_L1) {
        for (const auto &i : SerialTransport::listPorts())
            out << i.path << "  " << i.friendlyName << "  " << i.vendorId << ':' << i.productId << Qt::endl;
        return 0;
    }
    if (args.size() < 3) p.showHelp(1);

    DebugLog::init();
    Settings::instance().load();

    MessageBus bus;
    QThread linkThread;
    linkThread.setObjectName(u"link"_s);
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
    if (type == "tcp"_L1 || type == "udp"_L1) {
        o[u"host"_s] = args[1];
        o[u"port"_s] = args[2].toInt();
        conn.connectLink(type == "tcp"_L1 ? QLatin1String(LinkManager::kMavlinkTcp) : QLatin1String(LinkManager::kMavlinkUdp), o);
    } else if (type == "serial"_L1) {
        o[u"port"_s] = args[1];
        o[u"baudRate"_s] = args[2].toInt();
        conn.connectLink(QLatin1String(LinkManager::kMavlinkSerial), o);
    } else if (type == "msp"_L1 || type == "msptcp"_L1) {
        o[u"profile"_s] = p.isSet(slow) ? u"slow"_s : u"normal"_s;
        if (type == "msp"_L1) {
            o[u"port"_s] = args[1];
            o[u"baudRate"_s] = args[2].toInt();
            conn.connectLink(QLatin1String(LinkManager::kMspSerial), o);
        } else {
            o[u"host"_s] = args[1];
            o[u"port"_s] = args[2].toInt();
            conn.connectLink(QLatin1String(LinkManager::kMspTcp), o);
        }
    } else if (type == "lte"_L1) {
        o[u"host"_s] = args[1];
        o[u"port"_s] = args[2].toInt();
        o[u"moduleId"_s] = p.value(module);
        o[u"keyHex"_s] = p.value(key);
        o[u"certSha256"_s] = p.value(cert);
        QObject::connect(link, &LinkManager::lteStatus, &app, [&out](const QVariantMap &st) {
            static QString last;
            const QString line = u"LTE %1 air=%2 records in %3 out %4 authFail %5 replay %6"_s
                                     .arg(st.value(u"state"_s).toString(), st.value(u"airOnline"_s).toBool() ? u"online"_s : u"offline"_s)
                                     .arg(st.value(u"recordsIn"_s).toLongLong()).arg(st.value(u"recordsOut"_s).toLongLong())
                                     .arg(st.value(u"authFail"_s).toLongLong()).arg(st.value(u"replay"_s).toLongLong());
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
        out << u"%1 %2 %3 hb=%4 | %5 %6 | %7, %8 alt %9 m | att %10/%11/%12 deg | gs %13 as %14 vs %15 | bat %16 V %17% | gps %18 sats %19 | link %20 kbps"_s
                   .arg(s->connected ? u"LINK"_s : u"----"_s, s->connectionType, s->linkType)
                   .arg(s->heartbeatCount)
                   .arg(modes::vehicleTypeName(s->vehicleType), s->armed ? u"ARMED "_s + s->flightMode : s->flightMode)
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
                if (p.isSet(takeoff)) cmd.setFlightMode(u"GUIDED"_s);
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
