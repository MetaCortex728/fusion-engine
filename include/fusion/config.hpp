// Per-sensor configuration loaded from YAML.
//
// File format (all fields required):
//
//   name: rgb                       # sensor (target) frame name
//   resolution: [640, 480]          # width, height in pixels
//   intrinsics:
//     K: [[fx, s, cx], [0, fy, cy], [0, 0, 1]]
//   distortion: [k1, k2, p1, p2, k3]   # radial-tangential, see Distortion
//   extrinsics:                     # T_<name>_<source>
//     source: lidar                 # frame the transform maps points from
//     rotation: [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]]
//     translation: [x, y, z]        # metres
//   time_offset_ns: 0               # integer nanoseconds
//
// Thermal cameras add an optional section that says how raw values become
// degrees Celsius. `encoding` is optional and defaults to `planck`, which
// needs the four Planck constants of PlanckParams (finite, R > 0, B > 0):
//
//   thermal:
//     encoding: planck              # optional, the default
//     planck: {R: 380747, B: 1428, F: 1, O: -88.539}
//
// `linear` is for cameras that deliver temperature directly. It needs the
// LinearParams (finite, scale != 0) with T = raw * scale + offset, here
// centi-kelvin:
//
//   thermal:
//     encoding: linear
//     linear: {scale: 0.01, offset: -273.15}
//
// The parameter section of the other encoding must not be present. A config
// without a `thermal` section, such as an RGB camera's, leaves
// SensorConfig::thermal empty.
//
// The extrinsics map a point from the `source` frame into this sensor's
// frame: p_sensor = R * p_source + t. For a camera with source `lidar` this
// is T_camera_lidar. The time offset is added to this sensor's timestamps to
// express them on the reference clock: t_reference = t_sensor + time_offset_ns.
#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion/geometry.hpp"
#include "fusion/thermal.hpp"

namespace fusion {

struct SensorConfig {
    std::string name;
    int width = 0;
    int height = 0;
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    Distortion distortion;
    std::string source_frame;
    /// T_sensor_source: maps points from `source_frame` into this sensor's frame.
    Eigen::Isometry3d T_sensor_source = Eigen::Isometry3d::Identity();
    std::int64_t time_offset_ns = 0;
    /// Present only for thermal cameras: the raw-to-Celsius conversion.
    std::optional<ThermalCalibration> thermal;
};

/// Thrown for an unreadable file, invalid YAML, a missing field, a value of
/// the wrong type or shape, or an invalid value. The message names the file
/// and the field, e.g. "rgb.yaml: extrinsics.rotation: ...".
class ConfigError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// Loads and validates a sensor config file. Throws ConfigError.
SensorConfig load_sensor_config(const std::string& path);

/// Parses and validates sensor config YAML text. `origin` names the source in
/// error messages. Throws ConfigError.
SensorConfig parse_sensor_config(const std::string& yaml_text,
                                 const std::string& origin = "<string>");

/// Camera model for the resolution, K, and distortion of a config.
CameraModel make_camera_model(const SensorConfig& config);

}  // namespace fusion
