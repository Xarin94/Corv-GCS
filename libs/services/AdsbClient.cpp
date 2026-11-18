#include "services/AdsbClient.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "vehicle/VehicleHub.h"
#include "vehicle/VehicleState.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QUrl>
#include <QUrlQuery>
#include <algorithm>
#include <cmath>

namespace corv {

namespace {

constexpr int kPollIntervalMs = 30000;     // ADSB_POLL_INTERVAL
constexpr int kStartDelayMs = 3000;
constexpr std::int64_t kFetchCooldownMs = 10000;   // OpenSky rate limit
constexpr std::int64_t kMaxAgeMs = 60000;  // ADSB_MAX_AGE: stale entries go after 60 s
constexpr int kTimeoutMs = 15000;
constexpr double kMarginDeg = 0.5;         // ~50 km at mid-latitudes

/// When the reported position was measured (ms). OpenSky's time_position is
/// often several seconds older than the fetch; the 3D view dead-reckons from
/// it. Kept only when it is plausible for this clock (not in the future, not
/// older than the purge age), otherwise the reception time.
std::int64_t positionTime(const QJsonValue &timePositionS, std::int64_t received)
{
    if (!timePositionS.isDouble()) return received;
    const double t = timePositionS.toDouble() * 1000;
    return std::isfinite(t) && t <= double(received) && double(received) - t < double(kMaxAgeMs) ? std::int64_t(t) : received;
}

} // namespace

AdsbClient::AdsbClient(VehicleHub *hub, QObject *parent) : QObject(parent), m_hub(hub)
{
    m_startTimer.setSingleShot(true);
    m_startTimer.setInterval(kStartDelayMs);
    connect(&m_startTimer, &QTimer::timeout, this, [this] {
        poll();
        m_pollTimer.start();
    });
    m_pollTimer.setInterval(kPollIntervalMs);
    connect(&m_pollTimer, &QTimer::timeout, this, &AdsbClient::poll);
}

void AdsbClient::setEnabled(bool on)
{
    if (on == m_enabled) return;
    m_enabled = on;
    if (on) {
        if (!m_startTimer.isActive() && !m_pollTimer.isActive()) m_startTimer.start();
    } else {
        m_startTimer.stop();
        m_pollTimer.stop();
    }
}

void AdsbClient::poll()
{
    if (!m_enabled || m_inFlight) return;
    const std::int64_t now = wallMs();
    if (now - m_lastFetch < kFetchCooldownMs) return;   // "Rate limited, wait N s"

    const auto s = m_hub->snapshot();
    QUrlQuery q;
    q.addQueryItem(QStringLiteral("lamin"), QString::number(s->lat - kMarginDeg, 'g', 12));
    q.addQueryItem(QStringLiteral("lomin"), QString::number(s->lon - kMarginDeg, 'g', 12));
    q.addQueryItem(QStringLiteral("lamax"), QString::number(s->lat + kMarginDeg, 'g', 12));
    q.addQueryItem(QStringLiteral("lomax"), QString::number(s->lon + kMarginDeg, 'g', 12));
    QUrl url(QStringLiteral("https://opensky-network.org/api/states/all"));
    url.setQuery(q);
    QNetworkRequest req(url);
    req.setTransferTimeout(kTimeoutMs);
    m_inFlight = m_net.get(req);
    connect(m_inFlight, &QNetworkReply::finished, this, [this, reply = m_inFlight] { onReply(reply); });
}

QList<TrafficEntry> AdsbClient::parseStates(const QByteArray &json, std::int64_t received, QString *error)
{
    QList<TrafficEntry> out;
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(json, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        if (error) *error = QStringLiteral("Invalid JSON from OpenSky");
        return out;
    }
    // OpenSky state vector columns:
    // 0=icao24, 1=callsign, 2=origin_country, 3=time_position, 5=longitude,
    // 6=latitude, 7=baro_altitude, 8=on_ground, 9=velocity, 10=true_track,
    // 11=vertical_rate, 13=geo_altitude
    const QJsonArray states = doc.object().value(QStringLiteral("states")).toArray();
    for (const QJsonValue &v : states) {
        const QJsonArray a = v.toArray();
        if (a.size() < 12 || !a.at(5).isDouble() || !a.at(6).isDouble()) continue;
        TrafficEntry t;
        t.icao24 = a.at(0).toString();
        t.callsign = a.at(1).toString().trimmed();
        t.lon = a.at(5).toDouble();
        t.lat = a.at(6).toDouble();
        t.alt = a.at(7).isDouble() ? a.at(7).toDouble() : a.size() > 13 ? a.at(13).toDouble(0) : 0;
        t.velocity = a.at(9).toDouble(0);
        t.heading = a.at(10).toDouble(0);
        t.vertRate = a.at(11).toDouble(0);
        t.onGround = a.at(8).toBool(false);
        t.ts = received;
        t.posTs = positionTime(a.at(3), received);
        out.append(t);
    }
    return out;
}

void AdsbClient::onReply(QNetworkReply *reply)
{
    reply->deleteLater();
    if (reply == m_inFlight) m_inFlight = nullptr;
    m_lastFetch = wallMs();

    QString error;
    const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    QList<TrafficEntry> fetched;
    if (reply->error() == QNetworkReply::OperationCanceledError || reply->error() == QNetworkReply::TimeoutError)
        error = QStringLiteral("OpenSky timeout");
    else if (reply->error() != QNetworkReply::NoError && status == 0)
        error = reply->errorString();
    else if (status != 200)
        error = QStringLiteral("OpenSky HTTP %1").arg(status);
    else
        fetched = parseStates(reply->readAll(), m_lastFetch, &error);

    if (!error.isEmpty()) {
        // Silently retried at the next interval; logged at most once a minute
        const int skipped = DebugLog::throttle(QStringLiteral("adsb-fetch"), 60000);
        if (skipped >= 0)
            LOG_W("adsb") << "OpenSky fetch failed:" << error
                          << (skipped ? QStringLiteral("(+%1 more failures)").arg(skipped) : QString());
        return;
    }
    if (!m_enabled) return;   // switched off while the request was in flight: drop it
    // Merge (replace by icao24, add new), then purge what is not heard for 60 s
    m_hub->mutate([fetched](VehicleState &s) {
        if (!s.adsbEnabled) return;
        for (const TrafficEntry &ac : fetched) {
            auto it = std::find_if(s.traffic.begin(), s.traffic.end(), [&](const TrafficEntry &t) { return t.icao24 == ac.icao24; });
            if (it != s.traffic.end()) *it = ac;
            else s.traffic.push_back(ac);
        }
        const std::int64_t now = wallMs();
        s.traffic.erase(std::remove_if(s.traffic.begin(), s.traffic.end(), [&](const TrafficEntry &t) { return now - t.ts >= kMaxAgeMs; }),
                        s.traffic.end());
    });
}

} // namespace corv
