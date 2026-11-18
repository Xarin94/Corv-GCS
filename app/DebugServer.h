// DebugServer.h - the local JSON-RPC server of --debug-port PORT (the tool that
// replaces the Chromium DevTools protocol of the Electron tests).
//
// Listens on 127.0.0.1 only, never on a network interface. One JSON-RPC 2.0
// request per line, one response per line:
//
//   {"jsonrpc":"2.0","id":1,"method":"set","params":{"object":"scene","property":"orbitDistance","value":300}}
//   {"jsonrpc":"2.0","id":2,"method":"call","params":{"target":"Gcs.setMode","args":["FBWA"]}}
//   {"jsonrpc":"2.0","id":3,"method":"stats"}
//   {"jsonrpc":"2.0","id":4,"method":"screenshot","params":{"file":"C:/tmp/a.png"}}   (no file: base64 PNG)
//
// and "get" {object, property}, "script" {line} for any --grab-script command.
// tools/debug-client.py sends them from the command line. GUI thread.
#pragma once

#include <QObject>
#include <QTcpServer>

namespace corv {

class TestDriver;

class DebugServer : public QObject
{
    Q_OBJECT
public:
    DebugServer(TestDriver &driver, QObject *parent = nullptr);
    bool listen(quint16 port);

private:
    void onConnection();
    QByteArray handle(const QByteArray &line);

    TestDriver &m_driver;
    QTcpServer m_server;
};

} // namespace corv
