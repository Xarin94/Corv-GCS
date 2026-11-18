#include "link/LteLink.h"

#include "core/Clock.h"
#include "core/DebugLog.h"

#include <QCryptographicHash>
#include <QEventLoop>
#include <QHostAddress>
#include <QMessageAuthenticationCode>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QSslCipher>
#include <QSslSocket>
#include <QtEndian>

#ifdef CORV_HAVE_OPENSSL
#include <openssl/evp.h>
#endif

namespace corv::lte {

namespace {
QByteArray hmac(const QByteArray &key, const char *label)
{
    return QMessageAuthenticationCode::hash(QByteArray(label), key, QCryptographicHash::Sha256);
}
} // namespace

bool available()
{
#ifdef CORV_HAVE_OPENSSL
    return true;
#else
    return false;
#endif
}

#ifdef CORV_HAVE_OPENSSL

bool deriveKeys(const QByteArray &key, Keys *out)
{
    out->up = hmac(key, "CRV2 air->gcs");
    out->down = hmac(key, "CRV2 gcs->air");
    out->seed = hmac(key, "CRV3 relay-sign");
    out->fingerprint = QString::fromLatin1(QCryptographicHash::hash(key, QCryptographicHash::Sha256).left(4).toHex());
    EVP_PKEY *pk = EVP_PKEY_new_raw_private_key(EVP_PKEY_ED25519, nullptr, reinterpret_cast<const unsigned char *>(out->seed.constData()), 32);
    if (!pk) return false;
    QByteArray pub(32, '\0');
    size_t len = 32;
    const bool ok = EVP_PKEY_get_raw_public_key(pk, reinterpret_cast<unsigned char *>(pub.data()), &len) == 1 && len == 32;
    EVP_PKEY_free(pk);
    out->pub = pub;
    return ok;
}

QByteArray sign(const QByteArray &seed, const QByteArray &message)
{
    EVP_PKEY *pk = EVP_PKEY_new_raw_private_key(EVP_PKEY_ED25519, nullptr, reinterpret_cast<const unsigned char *>(seed.constData()), 32);
    if (!pk) return {};
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    QByteArray sig(64, '\0');
    size_t len = 64;
    const bool ok = ctx && EVP_DigestSignInit(ctx, nullptr, nullptr, nullptr, pk) == 1 &&
                    EVP_DigestSign(ctx, reinterpret_cast<unsigned char *>(sig.data()), &len,
                                   reinterpret_cast<const unsigned char *>(message.constData()), size_t(message.size())) == 1;
    EVP_MD_CTX_free(ctx);
    EVP_PKEY_free(pk);
    return ok ? sig.left(qsizetype(len)) : QByteArray();
}

QByteArray seal(const QByteArray &key, const QByteArray &nonce, const QByteArray &aad, const QByteArray &plain)
{
    EVP_CIPHER_CTX *c = EVP_CIPHER_CTX_new();
    QByteArray out(plain.size() + kTag, '\0');
    int len = 0, total = 0;
    const auto u = [](const QByteArray &b) { return reinterpret_cast<const unsigned char *>(b.constData()); };
    bool ok = EVP_EncryptInit_ex(c, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) == 1 &&
              EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_GCM_SET_IVLEN, int(nonce.size()), nullptr) == 1 &&
              EVP_EncryptInit_ex(c, nullptr, nullptr, u(key), u(nonce)) == 1 &&
              EVP_EncryptUpdate(c, nullptr, &len, u(aad), int(aad.size())) == 1 &&
              EVP_EncryptUpdate(c, reinterpret_cast<unsigned char *>(out.data()), &len, u(plain), int(plain.size())) == 1;
    total = len;
    ok = ok && EVP_EncryptFinal_ex(c, reinterpret_cast<unsigned char *>(out.data()) + total, &len) == 1;
    total += len;
    ok = ok && EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_GCM_GET_TAG, kTag, out.data() + total) == 1;
    EVP_CIPHER_CTX_free(c);
    return ok ? out.left(total + kTag) : QByteArray();
}

bool open(const QByteArray &key, const QByteArray &nonce, const QByteArray &aad, const QByteArray &ctAndTag, QByteArray *plain)
{
    if (ctAndTag.size() < kTag) return false;
    const QByteArray ct = ctAndTag.left(ctAndTag.size() - kTag);
    QByteArray tag = ctAndTag.right(kTag);
    EVP_CIPHER_CTX *c = EVP_CIPHER_CTX_new();
    QByteArray out(ct.size() + 16, '\0');
    int len = 0, total = 0;
    const auto u = [](const QByteArray &b) { return reinterpret_cast<const unsigned char *>(b.constData()); };
    bool ok = EVP_DecryptInit_ex(c, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) == 1 &&
              EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_GCM_SET_IVLEN, int(nonce.size()), nullptr) == 1 &&
              EVP_DecryptInit_ex(c, nullptr, nullptr, u(key), u(nonce)) == 1 &&
              EVP_DecryptUpdate(c, nullptr, &len, u(aad), int(aad.size())) == 1 &&
              EVP_DecryptUpdate(c, reinterpret_cast<unsigned char *>(out.data()), &len, u(ct), int(ct.size())) == 1;
    total = len;
    ok = ok && EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_GCM_SET_TAG, kTag, tag.data()) == 1;
    ok = ok && EVP_DecryptFinal_ex(c, reinterpret_cast<unsigned char *>(out.data()) + total, &len) == 1;
    total += len;
    EVP_CIPHER_CTX_free(c);
    if (!ok) return false;
    *plain = out.left(total);
    return true;
}

#else

bool deriveKeys(const QByteArray &, Keys *) { return false; }
QByteArray sign(const QByteArray &, const QByteArray &) { return {}; }
QByteArray seal(const QByteArray &, const QByteArray &, const QByteArray &, const QByteArray &) { return {}; }
bool open(const QByteArray &, const QByteArray &, const QByteArray &, const QByteArray &, QByteArray *) { return false; }

#endif

QByteArray header(uint8_t type, int bodyLen)
{
    QByteArray h(kHdr, '\0');
    h[0] = char(kMagic);
    h[1] = char(type);
    qToLittleEndian<quint16>(quint16(bodyLen), h.data() + 2);
    return h;
}

QList<QByteArray> RecordFramer::feed(const QByteArray &data)
{
    m_buf.append(data);
    QList<QByteArray> out;
    while (!m_buf.isEmpty()) {
        if (uint8_t(m_buf[0]) != kMagic) {
            qsizetype j = m_buf.indexOf(char(kMagic));
            if (j < 0) j = m_buf.size();
            m_garbage += j;
            m_buf.remove(0, j);
            continue;
        }
        if (m_buf.size() < kHdr) break;
        const uint8_t type = uint8_t(m_buf[1]);
        const int len = qFromLittleEndian<quint16>(m_buf.constData() + 2);
        if ((type != kData && type != kKeepalive) || len > kMaxBody) {
            ++m_garbage;
            m_buf.remove(0, 1);
            continue;
        }
        if (m_buf.size() < kHdr + len) break;
        out.append(m_buf.left(kHdr + len));
        m_buf.remove(0, kHdr + len);
    }
    return out;
}

bool ReplayWindow::accept(const QByteArray &sid, uint32_t ctr)
{
    W w = m_seen.value(sid);
    if (ctr > w.top) {
        const uint32_t shift = ctr - w.top;
        w.mask = shift < uint32_t(kReplayWindow) ? ((w.mask << shift) | 1u) : 1u;
        w.top = ctr;
    } else {
        const uint32_t off = w.top - ctr;
        if (off >= uint32_t(kReplayWindow) || ((w.mask >> off) & 1u)) return false;
        w.mask |= (1u << off);
    }
    m_seen.insert(sid, w);
    return true;
}

} // namespace corv::lte

namespace corv {

namespace {
constexpr int kBatchBytes = 800;          // MAVLink frames packed into one record...
constexpr int kBatchMs = 20;              // ...closed at this size or this long after the first frame
constexpr int kHandshakeTimeoutMs = 20000;
constexpr int kReconnectMs[] = {1000, 2000, 5000, 10000};
constexpr int kMaxPendingBytes = 8192;    // written before the module's first record

QString normFingerprint(const QString &fp) { return fp.toLower().remove(QRegularExpression(QStringLiteral("[^0-9a-f]"))); }
} // namespace

LteTransport::LteTransport(const Options &o, QObject *parent) : Transport(parent), m_opt(o)
{
    if (m_opt.port <= 0) m_opt.port = 5765;
    m_pin = normFingerprint(o.certSha256);
    m_statsTimer.setInterval(1000);
    connect(&m_statsTimer, &QTimer::timeout, this, &LteTransport::emitStatus);
    m_reconnectTimer.setSingleShot(true);
    connect(&m_reconnectTimer, &QTimer::timeout, this, &LteTransport::connectRelay);
    m_batchTimer.setSingleShot(true);
    m_batchTimer.setInterval(kBatchMs);
    connect(&m_batchTimer, &QTimer::timeout, this, &LteTransport::flush);
    m_handshakeTimer.setSingleShot(true);
    m_handshakeTimer.setInterval(kHandshakeTimeoutMs);
    connect(&m_handshakeTimer, &QTimer::timeout, this, [this] { fail(QStringLiteral("Relay handshake timeout"), false); });
}

LteTransport::~LteTransport() { close(); }

QString LteTransport::describe() const
{
    return QStringLiteral("MAVLink LTE module %1 via %2:%3").arg(m_opt.moduleId, m_opt.host).arg(m_opt.port);
}

bool LteTransport::open(QString *error)
{
    if (!lte::available()) {
        *error = QStringLiteral("This build has no crypto backend (OpenSSL): no cellular link");
        return false;
    }
    const QByteArray key = QByteArray::fromHex(m_opt.keyHex.trimmed().toLatin1());
    if (key.size() != 32 || m_opt.keyHex.trimmed().size() != 64) {
        *error = QStringLiteral("AES key must be 64 hex characters (256 bit)");
        return false;
    }
    if (m_opt.moduleId.isEmpty() || m_opt.moduleId.contains(QRegularExpression(QStringLiteral("\\s")))) {
        *error = QStringLiteral("Invalid module ID");
        return false;
    }
    if (m_opt.host.isEmpty()) {
        *error = QStringLiteral("Relay host missing");
        return false;
    }
    if (!lte::deriveKeys(key, &m_keys)) {
        *error = QStringLiteral("Key derivation failed");
        return false;
    }
    m_opt.keyHex.fill(QLatin1Char('0'));   // the key itself is not kept
    m_stopped = false;
    m_statsTimer.start();
    // Resolves on the first successful handshake; after that the link reconnects by itself
    m_firstPending = true;
    QEventLoop loop;
    connect(this, &LteTransport::status, &loop, [this, &loop] { if (!m_firstPending) loop.quit(); });
    connectRelay();
    if (m_firstPending) loop.exec();
    if (!m_firstOk) {
        *error = m_firstError;
        close();
        return false;
    }
    return true;
}

void LteTransport::close()
{
    m_stopped = true;
    m_reconnectTimer.stop();
    m_batchTimer.stop();
    m_statsTimer.stop();
    m_handshakeTimer.stop();
    m_batch.clear();
    if (m_sock) {
        m_sock->disconnect(this);
        m_sock->abort();
        m_sock->deleteLater();
        m_sock = nullptr;
    }
    if (m_state != QLatin1String("stopped") && m_state != QLatin1String("idle")) setState(QStringLiteral("stopped"));
}

void LteTransport::resetSession()
{
    m_framer = lte::RecordFramer();
    m_mySid = QByteArray(lte::kSid, '\0');   // new session id: the nonce never repeats
    QRandomGenerator::system()->generate(reinterpret_cast<quint32 *>(m_mySid.data()),
                                         reinterpret_cast<quint32 *>(m_mySid.data() + lte::kSid));
    m_myCtr = 0;
    m_seen = lte::ReplayWindow();
    m_airSid.clear();
    m_pending.clear();
    m_pendingLen = 0;
}

void LteTransport::connectRelay()
{
    resetSession();
    setState(QStringLiteral("connecting"));
    m_handshakeDone = false;
    m_lineBuf.clear();
    m_stage = 0;
    auto *s = new QSslSocket(this);
    m_sock = s;
    // The relay certificate is pinned by fingerprint (it is usually self-signed).
    // Without a pin the TLS session is still encrypted but the relay is not
    // authenticated; the E2E layer protects the content.
    s->setPeerVerifyMode(QSslSocket::VerifyNone);
    s->setProtocol(QSsl::TlsV1_2OrLater);
    s->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    s->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
    connect(s, &QSslSocket::sslErrors, s, [s](const QList<QSslError> &) { s->ignoreSslErrors(); });
    connect(s, &QSslSocket::encrypted, this, &LteTransport::onEncrypted);
    connect(s, &QSslSocket::readyRead, this, &LteTransport::onData);
    connect(s, &QSslSocket::errorOccurred, this, [this, s](QAbstractSocket::SocketError) {
        if (m_sock == s) fail(s->errorString(), false);
    });
    connect(s, &QSslSocket::disconnected, this, [this, s] {
        if (m_sock == s) fail(QStringLiteral("Relay closed the connection"), false);
    });
    m_handshakeTimer.start();
    s->connectToHostEncrypted(m_opt.host, quint16(m_opt.port));
}

void LteTransport::onEncrypted()
{
    const QSslCertificate cert = m_sock->peerCertificate();
    const QByteArray digest = cert.isNull() ? QByteArray() : cert.digest(QCryptographicHash::Sha256);
    const QString fp = QString::fromLatin1(digest.toHex(':').toUpper());
    m_tls = QVariantMap{{QStringLiteral("version"), m_sock->sessionProtocol() == QSsl::TlsV1_3 ? QStringLiteral("TLSv1.3") : QStringLiteral("TLSv1.2")},
                        {QStringLiteral("cipher"), m_sock->sessionCipher().name()},
                        {QStringLiteral("fingerprint"), fp},
                        {QStringLiteral("verified"), !m_pin.isEmpty() && normFingerprint(fp) == m_pin}};
    if (!m_pin.isEmpty() && normFingerprint(fp) != m_pin) {
        fail(QStringLiteral("Relay certificate does not match the pinned fingerprint"), true);
        return;
    }
    setState(QStringLiteral("handshake"));
}

void LteTransport::onData()
{
    if (!m_sock) return;
    const QByteArray data = m_sock->readAll();
    m_bytesRx += data.size();
    m_lastRx = wallMs();
    if (m_handshakeDone) {
        onRecords(data);
        return;
    }
    m_lineBuf.append(data);
    qsizetype nl;
    while (!m_handshakeDone && (nl = m_lineBuf.indexOf('\n')) >= 0) {
        const QString line = QString::fromLatin1(m_lineBuf.left(nl)).trimmed();
        m_lineBuf.remove(0, nl + 1);
        if (m_stage == 0) {
            static const QRegularExpression banner(QStringLiteral("^CRV2-GCS ([0-9a-fA-F]{32})$"));
            const auto m = banner.match(line);
            if (!m.hasMatch()) {
                fail(QStringLiteral("Unexpected relay banner: ") + line, true);
                return;
            }
            const QByteArray challenge = QByteArray::fromHex(m.captured(1).toLatin1());
            const QByteArray sig = lte::sign(m_keys.seed, QByteArray("CRV3-GCS-AUTH ") + challenge + m_keys.pub + m_opt.moduleId.toUtf8());
            rawWrite("CRV3 " + m_opt.moduleId.toUtf8() + ' ' + m_keys.pub.toHex() + ' ' + sig.toHex() + '\n');
            m_stage = 1;
        } else {
            m_handshakeTimer.stop();
            if (!line.startsWith(QLatin1String("OK"))) {
                // the relay accepts any valid signature: a refusal means it does not
                // speak CRV3 (not updated) or is full, and retrying cannot help
                fail(line.startsWith(QLatin1String("ERR")) ? QStringLiteral("Relay refused the key signature (relay without CRV3 access?)")
                                                           : QStringLiteral("Relay: ") + line,
                     true);
                return;
            }
            m_handshakeDone = true;
            m_airOnline = line.contains(QLatin1String("air=online"));
            m_backoff = 0;
            m_lastError.clear();
            if (m_firstPending) {
                m_firstPending = false;
                m_firstOk = true;
            }
            setState(QStringLiteral("online"));
            if (!m_lineBuf.isEmpty()) {
                const QByteArray rest = m_lineBuf;
                m_lineBuf.clear();
                onRecords(rest);
            }
        }
    }
}

void LteTransport::onRecords(const QByteArray &data)
{
    for (const QByteArray &rec : m_framer.feed(data)) {
        if (uint8_t(rec[1]) != lte::kData) continue;   // keepalives only go to the module
        const QByteArray plain = openRecord(rec);
        if (!plain.isEmpty()) emit received(plain);
    }
}

QByteArray LteTransport::openRecord(const QByteArray &rec)
{
    if (rec.size() < lte::kHdr + lte::kSid + 4 + lte::kTag) {
        ++m_authFail;
        return {};
    }
    const QByteArray hdr = rec.left(lte::kHdr);
    const QByteArray nonce = rec.mid(lte::kHdr, lte::kSid + 4);
    QByteArray plain;
    if (!lte::open(m_keys.up, nonce, hdr, rec.mid(lte::kHdr + lte::kSid + 4), &plain)) {
        ++m_authFail;
        return {};
    }
    const QByteArray sid = nonce.left(lte::kSid);
    const uint32_t ctr = qFromBigEndian<quint32>(nonce.constData() + lte::kSid);
    if (!m_seen.accept(sid, ctr)) {
        ++m_replay;
        return {};
    }
    const bool firstSession = m_airSid.isEmpty();
    m_airSid = sid;   // commands are bound to this session
    if (firstSession && m_pendingLen) {   // what the GCS wrote while waiting
        const QByteArray queued = m_pending.join();
        m_pending.clear();
        m_pendingLen = 0;
        for (qsizetype off = 0; off < queued.size(); off += lte::kMaxPlain) rawWrite(sealRecord(queued.mid(off, lte::kMaxPlain)));
    }
    m_vehicleSession = QString::fromLatin1(sid.toHex());
    m_airOnline = true;
    ++m_recordsIn;
    return plain;
}

void LteTransport::fail(const QString &message, bool fatal)
{
    m_lastError = message;
    if (fatal) m_fatal = true;
    m_handshakeTimer.stop();
    if (m_firstPending) {   // first attempt: report to the caller, no retry loop
        m_firstPending = false;
        m_firstOk = false;
        m_firstError = message;
        m_fatal = true;
    }
    if (m_sock) {
        QSslSocket *s = m_sock;
        m_sock = nullptr;
        s->disconnect(this);
        s->abort();
        s->deleteLater();
    }
    scheduleReconnect();
}

void LteTransport::scheduleReconnect()
{
    if (m_stopped) {
        emitStatus();
        return;
    }
    if (m_fatal) {
        m_fatal = false;
        m_stopped = true;
        m_statsTimer.stop();
        setState(QStringLiteral("error"));
        return;
    }
    const int delay = kReconnectMs[std::min<int>(m_backoff++, int(std::size(kReconnectMs)) - 1)];
    ++m_reconnects;
    setState(QStringLiteral("reconnecting"));
    m_reconnectTimer.start(delay);
}

// ── send ────────────────────────────────────────────────────────────────────

qint64 LteTransport::write(const QByteArray &buf)
{
    if (m_stopped || !m_sock) return 0;
    m_batch.append(buf);
    if (m_batch.size() >= kBatchBytes) flush();
    else if (!m_batchTimer.isActive()) m_batchTimer.start();
    return buf.size();
}

void LteTransport::flush()
{
    m_batchTimer.stop();
    if (m_batch.isEmpty()) return;
    const QByteArray plain = m_batch;
    m_batch.clear();
    if (m_airSid.isEmpty()) {   // no telemetry yet: module session unknown
        if (m_pendingLen + plain.size() > kMaxPendingBytes) {
            ++m_droppedNoSession;
            return;
        }
        m_pending.append(plain);
        m_pendingLen += int(plain.size());
        return;
    }
    for (qsizetype off = 0; off < plain.size(); off += lte::kMaxPlain) rawWrite(sealRecord(plain.mid(off, lte::kMaxPlain)));
}

QByteArray LteTransport::sealRecord(const QByteArray &plain)
{
    ++m_myCtr;
    QByteArray nonce = m_mySid;
    nonce.resize(lte::kSid + 4);
    qToBigEndian<quint32>(m_myCtr, nonce.data() + lte::kSid);
    const QByteArray hdr = lte::header(lte::kData, lte::kSid + 4 + int(plain.size()) + lte::kTag);
    const QByteArray ct = lte::seal(m_keys.down, nonce, hdr + m_airSid, plain);
    ++m_recordsOut;
    return hdr + nonce + ct;
}

void LteTransport::rawWrite(const QByteArray &buf)
{
    if (!m_sock) return;
    m_bytesTx += buf.size();
    m_sock->write(buf);
}

// ── status ──────────────────────────────────────────────────────────────────

void LteTransport::setState(const QString &state)
{
    if (state != m_state) {
        // state changes only, never the key (logLteStatus of main-mavlink.js)
        const QString text = QStringLiteral("%1 (module=%2 relay=%3:%4 keyFingerprint=%5 airOnline=%6%7)")
                                 .arg(state, m_opt.moduleId, m_opt.host)
                                 .arg(m_opt.port)
                                 .arg(m_keys.fingerprint, m_airOnline ? QStringLiteral("true") : QStringLiteral("false"),
                                      m_lastError.isEmpty() ? QString() : QStringLiteral(" lastError=") + m_lastError);
        if (state.contains(QRegularExpression(QStringLiteral("error|fail|reject|denied")))) LOG_W("lte") << text;
        else LOG_I("lte") << text;
    }
    m_state = state;
    emitStatus();
}

void LteTransport::emitStatus()
{
    emit status(QVariantMap{{QStringLiteral("state"), m_state},
                            {QStringLiteral("moduleId"), m_opt.moduleId},
                            {QStringLiteral("relay"), QStringLiteral("%1:%2").arg(m_opt.host).arg(m_opt.port)},
                            {QStringLiteral("keyFingerprint"), m_keys.fingerprint},
                            {QStringLiteral("relayIdentity"), QString::fromLatin1(m_keys.pub.left(4).toHex())},
                            {QStringLiteral("airOnline"), m_airOnline},
                            {QStringLiteral("vehicleSession"), m_vehicleSession},
                            {QStringLiteral("tls"), m_tls},
                            {QStringLiteral("lastError"), m_lastError},
                            {QStringLiteral("bytesRx"), m_bytesRx},
                            {QStringLiteral("bytesTx"), m_bytesTx},
                            {QStringLiteral("recordsIn"), m_recordsIn},
                            {QStringLiteral("recordsOut"), m_recordsOut},
                            {QStringLiteral("authFail"), m_authFail},
                            {QStringLiteral("replay"), m_replay},
                            {QStringLiteral("droppedNoSession"), m_droppedNoSession},
                            {QStringLiteral("reconnects"), m_reconnects},
                            {QStringLiteral("lastRxAge"), m_lastRx ? QVariant(wallMs() - m_lastRx) : QVariant()}});
}

} // namespace corv
