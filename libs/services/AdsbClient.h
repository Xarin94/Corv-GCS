// AdsbClient.h - ADS-B traffic from the OpenSky Network (js/adsb/ADSBManager.js,
// adsbPoll / startADSBPolling of js/main.js and the 'adsb-fetch' handler of the
// Electron main process).
//
// While SHOW ADS-B TRAFFIC is on: a first request 3 s after it is switched on,
// then one every 30 s (never two within OpenSky's 10 s rate limit), for the box
// of ±0.5° around the vehicle. The aircraft are merged into VehicleState::traffic
// by ICAO address on the link thread, next to the ADSB_VEHICLE reports the
// autopilot forwards; StateMapper purges whatever is not heard for 60 s. A
// failed request is retried at the next interval, logged at most once a minute.
// GUI thread.
#pragma once

#include <QNetworkAccessManager>
#include <QObject>
#include <QTimer>
#include <cstdint>

class QNetworkReply;

namespace corv {

class VehicleHub;
struct TrafficEntry;

class AdsbClient : public QObject
{
    Q_OBJECT
public:
    explicit AdsbClient(VehicleHub *hub, QObject *parent = nullptr);

    /// toggleADSB: start or stop the polling (the hub drops the traffic itself).
    void setEnabled(bool on);
    bool enabled() const { return m_enabled; }

    /// One OpenSky state vector array (the "states" of /api/states/all) as
    /// traffic entries received at `received` (ms since the epoch). Tests.
    static QList<TrafficEntry> parseStates(const QByteArray &json, std::int64_t received, QString *error = nullptr);

private:
    void poll();
    void onReply(QNetworkReply *reply);

    VehicleHub *m_hub;
    QNetworkAccessManager m_net;
    QTimer m_startTimer;   // the 3 s delay before the first poll
    QTimer m_pollTimer;
    bool m_enabled = false;
    std::int64_t m_lastFetch = 0;
    QNetworkReply *m_inFlight = nullptr;
};

} // namespace corv
