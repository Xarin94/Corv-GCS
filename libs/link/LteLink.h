// LteLink.h - cellular link to an LTEtelem module through a relay (port of
// lte-link.js).
//
// The module (ESP32-C6 + cellular modem on the vehicle) and this GCS both dial
// out to a relay server. Every module has its own key and the relay stores
// none (CRV3 access): it pairs module and GCS by an Ed25519 public key derived
// from the module key. Everything after the handshake is end-to-end encrypted
// between module and GCS, so the relay forwards records it can neither read
// nor modify.
//
// Handshake (TLS to the relay's GCS port):
//   relay -> "CRV2-GCS <challenge hex>\n"
//   GCS   -> "CRV3 <module id> <public key hex> <signature hex>\n"
//            signature = Ed25519('CRV3-GCS-AUTH ' || challenge || public key || id)
//   relay -> "OK <module id> air=online|offline\n"  or  "ERR auth\n"
// A wrong key is not refused (the relay knows no keys): the module just stays
// offline. Records (both directions): C7 | type | length (u16 LE) | body
//   DATA (0x01):      session id (8) | counter (4, BE) | ciphertext | tag (16)
//   KEEPALIVE (0x02): empty, relay -> module only
// AES-256-GCM, nonce = session id || counter. From the 256-bit module key:
//   up   = HMAC-SHA256(key, "CRV2 air->gcs")  telemetry, module -> GCS
//   down = HMAC-SHA256(key, "CRV2 gcs->air")  commands, GCS -> module
//   sign = Ed25519 seed HMAC-SHA256(key, "CRV3 relay-sign")
// AAD is the record header; GCS -> module records also bind the module's
// current session id. Replay: a 32-record sliding window per session id.
//
// AES-GCM and Ed25519 come from OpenSSL (libcrypto); a build without it has
// no cellular link (available() is false). Reference: LTEtelem tools/relay/e2e.py.
#pragma once

#include "link/Transport.h"

#include <QByteArray>
#include <QHash>
#include <QList>
#include <QPointer>
#include <QTimer>
#include <QVariantMap>
#include <cstdint>

class QSslSocket;

namespace corv::lte {

constexpr uint8_t kMagic = 0xC7;
constexpr uint8_t kData = 0x01;
constexpr uint8_t kKeepalive = 0x02;
constexpr int kHdr = 4, kSid = 8, kTag = 16;
constexpr int kMaxBody = 1400;
constexpr int kMaxPlain = kMaxBody - kSid - 4 - kTag;
constexpr int kReplayWindow = 32;

/// The crypto backend is in this build.
bool available();

struct Keys {
    QByteArray up, down, seed, pub;   // 32 bytes each
    QString fingerprint;              // first 4 bytes of SHA-256(key), hex: e2e_key_fp on the module
};
/// From the 32-byte module key; false without a crypto backend.
bool deriveKeys(const QByteArray &key, Keys *out);
QByteArray sign(const QByteArray &seed, const QByteArray &message);
/// AES-256-GCM. seal returns ciphertext || tag; open returns false on a bad tag.
QByteArray seal(const QByteArray &key, const QByteArray &nonce, const QByteArray &aad, const QByteArray &plain);
bool open(const QByteArray &key, const QByteArray &nonce, const QByteArray &aad, const QByteArray &ctAndTag, QByteArray *plain);

/// Splits the byte stream into whole records without looking inside them.
class RecordFramer
{
public:
    QList<QByteArray> feed(const QByteArray &data);
    qint64 garbage() const { return m_garbage; }

private:
    QByteArray m_buf;
    qint64 m_garbage = 0;
};

/// Anti-replay window: true the first time a counter is seen within the window.
class ReplayWindow
{
public:
    bool accept(const QByteArray &sid, uint32_t counter);

private:
    struct W {
        uint32_t top = 0, mask = 0;
    };
    QHash<QByteArray, W> m_seen;
};

QByteArray header(uint8_t type, int bodyLen);

} // namespace corv::lte

namespace corv {

/// The cellular link as a Transport of LinkManager (link thread). open() waits
/// for the first handshake and fails without retrying; after that drops are
/// reconnected (1, 2, 5, 10 s) and never close the transport, so the GCS stays
/// CONNECTED until the user disconnects.
class LteTransport : public Transport
{
    Q_OBJECT
public:
    struct Options {
        QString host;
        int port = 5765;
        QString moduleId;
        QString keyHex;
        QString certSha256;   // relay certificate to pin, empty = not verified
    };
    explicit LteTransport(const Options &o, QObject *parent = nullptr);
    ~LteTransport() override;

    bool open(QString *error) override;
    void close() override;
    qint64 write(const QByteArray &bytes) override;
    QString describe() const override;
    QString kind() const override { return QStringLiteral("LTE"); }

signals:
    /// state, moduleId, relay, keyFingerprint, relayIdentity, airOnline, vehicleSession,
    /// tls {version, cipher, fingerprint, verified}, lastError, counters, lastRxAge.
    void status(const QVariantMap &status);

private:
    void connectRelay();
    void onEncrypted();
    void onData();
    void onRecords(const QByteArray &data);
    QByteArray openRecord(const QByteArray &rec);
    void fail(const QString &message, bool fatal);
    void scheduleReconnect();
    void resetSession();
    void flush();
    QByteArray sealRecord(const QByteArray &plain);
    void rawWrite(const QByteArray &buf);
    void setState(const QString &state);
    void emitStatus();

    Options m_opt;
    QString m_pin;
    lte::Keys m_keys;
    QSslSocket *m_sock = nullptr;
    bool m_stopped = true;
    bool m_fatal = false;
    bool m_handshakeDone = false;
    bool m_firstPending = false;   // open() is waiting for the first result
    bool m_firstOk = false;
    QString m_firstError;
    QByteArray m_lineBuf;
    int m_stage = 0;               // 0 banner, 1 reply
    int m_backoff = 0;
    QTimer m_reconnectTimer, m_statsTimer, m_batchTimer, m_handshakeTimer;
    QByteArray m_batch;
    QList<QByteArray> m_pending;
    int m_pendingLen = 0;
    lte::RecordFramer m_framer;
    QByteArray m_mySid;
    uint32_t m_myCtr = 0;
    lte::ReplayWindow m_seen;
    QByteArray m_airSid;
    qint64 m_lastRx = 0;

    // status
    QString m_state = QStringLiteral("idle");
    bool m_airOnline = false;
    QString m_vehicleSession;
    QVariantMap m_tls;
    QString m_lastError;
    qint64 m_bytesRx = 0, m_bytesTx = 0, m_recordsIn = 0, m_recordsOut = 0;
    qint64 m_authFail = 0, m_replay = 0, m_droppedNoSession = 0, m_reconnects = 0;
};

} // namespace corv
