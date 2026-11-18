// RosPoints.h - points out of ROS sensor messages, sampled and georeferenced
// (port of js/ros/RosPoints.js).
//
// Three message types cover the sensors this is for:
//   sensor_msgs/PointCloud2  3D LiDAR (livox_ros_driver2, velodyne, ouster),
//                            a multibeam sonar, a SLAM map (FAST-LIO, MAVROS)
//   sensor_msgs/LaserScan    2D scanner, push-broom LiDAR, a multibeam swath
//   sensor_msgs/Range        single beam: echo sounder (Ping1D), altimeter
// ROS 2 type names (sensor_msgs/msg/PointCloud2) are the same types.
//
// Not every point is used: at most `max` per message, picked with a uniform
// stride from a random start, so a 20 000-point scan costs what `max` points
// cost and successive scans sample different points. Range limits apply in
// the sensor frame; NaN, +-Inf and the (0, 0, 0) "no return" points are
// dropped.
//
// Time of each beam. A LaserScan with a time_increment was not measured at
// one instant: beam i was measured i x time_increment after the first. For a
// 2D LiDAR that is a few ms over a sweep; for a mechanical sector-scanning
// sonar, one beam per ping, it is seconds - the vehicle moves metres in a
// message. decodePoints gives each sampled beam its time from the first
// (seconds) so the client can place it with the pose of its own time.
//
// rosbridge sends uint8[] (PointCloud2.data) as base64 in JSON and as a CBOR
// byte string; float32[] (LaserScan.ranges) as a JSON array (non-finite values
// become null) or a CBOR typed array (RFC 8746 tag 85). Both are read here;
// a JSON message is first turned into CBOR values (QCborValue::fromJsonValue).
//
// Frames (REP-103/105). Points in a sensor frame (x forward, y left, z up;
// a Range or LaserScan measures along x) are georeferenced through the vehicle
// pose with the chain LidarCore uses for the Livox:
//   p_frd  = (x, -y, -z)                  REP-103 FLU -> body FRD
//   p_body = R_mount * p_frd + lever      mount attitude vs the autopilot IMU
//   p_ned  = R_att * p_body               ATTITUDE
//   ENU    = vehicle ENU + (e, n, -d)
// Points in a world-fixed frame (map, odom: ENU from the EKF origin, as MAVROS
// publishes it; *_ned frames are NED) are placed at that origin.
#pragma once

#include <QCborMap>
#include <QCborValue>
#include <QString>
#include <array>
#include <limits>

namespace corv::rospts {

/// "cloud" | "scan" | "range" for a ROS 1 or ROS 2 type name, empty if unsupported.
QString kindOf(const QString &type);

/// Frame handling for a header.frame_id: "world-enu", "world-ned" or "sensor" (forced: auto | world | sensor).
QString frameOf(const QString &frameId, const QString &forced = QStringLiteral("auto"));

/// A number of a CBOR / JSON message field (integer or double), NaN if it is none.
double num(const QCborValue &v);

struct Decoded {
    int count = 0;      // points written
    int total = 0;      // points the message carried
    double span = 0;    // seconds from its first beam to its last (0: all measured at once)
};

/// Decode the sampled points of one message into out (x, y, z in the message
/// frame, room for outCap points) and, for a LaserScan with a time_increment,
/// each point's time after the first beam into times (seconds; may be null).
Decoded decodePoints(const QString &kind, const QCborMap &msg, float *out, int outCap, int max, double rMin = 0,
                     double rMax = std::numeric_limits<double>::infinity(), float *times = nullptr);

struct Despiked {
    int count = 0, removed = 0;
    bool ordered = false;
};

/// Spikes in a scan profile, dropped in place. Points in the order the sensor
/// sent them - a sonar's beams across its swath, a LaserScan's sweep, a lidar's
/// rings - form a profile: a real surface is continuous from one beam to the
/// next but at a few edges, while aeration under the hull, a fish or a particle
/// is one beam, or a few, far off its neighbours. A point whose range is more
/// than max(ABS, REL x range) off the median of the 2*HALF + 1 points around
/// it is dropped: the median keeps edges (a wall, a wreck's side) and drops
/// what is narrower than HALF beams.
///
/// Only on ordered profiles: in a cloud without order (or sampled too sparsely
/// to keep it) the points around one are not its neighbours, and nothing is
/// dropped. A profile is ordered when the median step from one point to the
/// next is small against the spread of its ranges.
Despiked despike(float *pts, int count, int half = 4, double absM = 0.5, double rel = 0.08, float *times = nullptr);

struct AttSample {
    double t = 0;                     // ms on the samples' clock
    double roll = 0, pitch = 0, yaw = 0;
    double p = 0, q = 0, r = 0;       // body rates, rad/s
};
struct PosSample {
    double t = 0;
    double lat = 0, lon = 0, alt = 0;
    double vn = 0, ve = 0, vd = 0;    // m/s
};
struct Pose {
    double lat = 0, lon = 0, alt = 0, roll = 0, pitch = 0, yaw = 0;
};

/// Pose at time t (ms), projected from the last attitude sample with its body
/// rates (ZYX Euler kinematics, 20 ms steps)
///   phi' = p + (q sin phi + r cos phi) tan theta,  theta' = q cos phi - r sin phi,  psi' = (q sin phi + r cos phi) / cos theta
/// and from the last position sample with its NED velocity (m/s; lat/lon at
/// 111 320 m per degree, the app's convention). k: autopilot seconds per second
/// of the samples' clock (a SITL at speedup 20: k = 20).
Pose projectPose(const AttSample &a, const PosSample &q, double t, double k = 1);

/// ZYX Euler -> rotation matrix (row-major 3x3) mapping the rotated frame into
/// its parent: body -> NED for the attitude, sensor -> body for the mount.
std::array<double, 9> eulerToMatrix(double roll, double pitch, double yaw);

/// Local metric frame at an anchor. Spherical (111 320 m per degree, longitude
/// scaled by the anchor's latitude): the convention of latLonToMeters() and of
/// the relative navigation's synthetic coordinates, so metres here are metres
/// in the 3D view in both modes.
struct Anchor {
    double lat = 0, lon = 0, alt = 0, mPerLat = 111320, mPerLon = 111320;
};
Anchor makeAnchor(double lat, double lon, double alt);
/// ENU of a lat/lon/alt from the anchor.
std::array<double, 3> toEnu(const Anchor &a, double lat, double lon, double alt);

/// Sensor-frame points (FLU) -> ENU from the anchor: Rm mount (sensor -> body),
/// lever IMU -> sensor (m, FRD), Ra attitude (body -> NED), veh the vehicle ENU.
void sensorToEnu(const float *src, int count, const std::array<double, 9> &Rm, const std::array<double, 3> &lever,
                 const std::array<double, 9> &Ra, const std::array<double, 3> &veh, double *out);

/// World-frame points -> ENU from the anchor: ned the frame is NED (x north, y east, z down), else ENU; org its origin's ENU.
void worldToEnu(const float *src, int count, bool ned, const std::array<double, 3> &org, double *out);

} // namespace corv::rospts
