// SceneOrigin.h - where the 3D world metres count from (port of js/core/SceneOrigin.js).
//
// The GPU holds positions in float32: 10 000 km from the origin they are 1 m
// apart, and the trail and the route are drawn in steps. So the origin goes
// where the vehicle is:
//  - every data stream that opens takes its first valid position (again after
//    it is closed and reopened: the vehicle may already be flying and
//    HOME_POSITION may never come);
//  - a HOME_POSITION moves it to that home, unless the origin is already
//    within 1 km of it. ArduPlane moves its home to where it stands every few
//    seconds while disarmed: each move rebuilt the terrain (RELOADING MAP) for
//    a shift of centimetres that float32 never notices.
// Owned by the link thread (StateMapper); the epoch in VehicleState tells the
// renderer and the map to rebuild what they hold in world metres.
#pragma once

#include "core/Geo.h"

#include <functional>

namespace corv {

class SceneOrigin
{
public:
    LatLon origin() const { return m_origin; }
    int epoch() const { return m_epoch; }

    /// Move the origin. True when it moved.
    bool set(double lat, double lon);
    /// A data stream opened: its next valid position becomes the origin.
    void awaitFix() { m_awaitingFix = true; }
    /// A valid position from the stream: the first one after it opened moves the origin there.
    bool fix(double lat, double lon);
    /// HOME_POSITION from the vehicle: the origin goes to that home when it is
    /// more than kHomeKeepM away (or the stream has had no position yet).
    void home(double lat, double lon);

    static constexpr double kHomeKeepM = 1000;

    std::function<void(LatLon)> onMoved;

private:
    LatLon m_origin = kDefaultOrigin;
    int m_epoch = 0;
    bool m_awaitingFix = false;
};

} // namespace corv
