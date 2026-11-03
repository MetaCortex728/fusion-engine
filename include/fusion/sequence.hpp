// On-disk sequence format (part of the fusion_io library).
//
// A sequence is a directory with a manifest, `sequence.yaml`, that names the
// camera config files, an optional trajectory, and one list file per sensor.
// Dataset adapters convert into this format; the engine never needs to know
// about a dataset. See docs/sequence-format.md for the full description.
//
//   format_version: 1
//   trajectory: trajectory.csv      # optional: timestamp_ns,x,y,z,qw,qx,qy,qz
//   lidar:
//     name: lidar                   # optional, default "lidar"
//     list: lidar.csv               # timestamp_ns,path  (.bin scans)
//     time_offset_ns: 0             # optional, default 0
//     T_body_lidar:                 # required with a trajectory
//       rotation: [[...], [...], [...]]
//       translation: [x, y, z]
//   cameras:                        # optional; each camera optional
//     rgb:
//       config: rgb.yaml            # sensor config (fusion/config.hpp)
//       list: rgb.csv               # timestamp_ns,path  (8-bit PNG)
//       tau_ns: 25000000            # optional association threshold
//     thermal:
//       config: thermal.yaml
//       list: thermal.csv           # timestamp_ns,path  (16-bit PNG)
//
// All paths are relative to the manifest's directory (absolute paths are
// also accepted). Timestamps are integer nanoseconds on each sensor's own
// clock; the sensor's time_offset_ns moves them onto the reference clock.
// Trajectory timestamps are on the reference clock.
#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion/aligned_frame.hpp"
#include "fusion/config.hpp"
#include "fusion/image_io.hpp"
#include "fusion/pose.hpp"
#include "fusion/thermal.hpp"
#include "fusion/time.hpp"

namespace fusion {

/// Thrown for any problem with a sequence: a missing file, a malformed
/// manifest or list file, unsorted or duplicate timestamps, a LiDAR file of
/// the wrong size, or an image whose size does not match its camera config.
/// The message starts with the offending file (and line, where there is
/// one).
class SequenceError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// One entry of a list file.
struct StampedPath {
    TimeNs t_ns = 0;   ///< sensor clock
    std::string path;  ///< resolved against the manifest's directory
};

struct CameraStream {
    std::string config_path;
    SensorConfig config;
    std::string list_path;
    std::vector<StampedPath> frames;  ///< strictly increasing times
    std::optional<TimeNs> tau_ns;     ///< from the manifest, if given
};

struct Sequence {
    std::string manifest_path;
    std::string lidar_name = "lidar";
    std::string lidar_list_path;
    std::vector<StampedPath> lidar;  ///< strictly increasing times
    TimeNs lidar_time_offset_ns = 0;
    Eigen::Isometry3d T_body_lidar = Eigen::Isometry3d::Identity();
    std::string trajectory_path;     ///< empty without a trajectory
    std::optional<Trajectory> trajectory;
    std::optional<CameraStream> rgb;
    std::optional<CameraStream> thermal;
};

/// Reads and validates the manifest, the camera configs, every list file and
/// the trajectory, and checks that every listed data file exists. Data files
/// themselves are read later, one frame at a time. Throws SequenceError
/// (camera config problems are reported as SequenceError too, with the
/// ConfigError message).
Sequence load_sequence(const std::string& manifest_path);

/// Reads a `timestamp_ns,path` list file. Relative paths are resolved against
/// `base_dir`. Throws SequenceError for a missing file, a bad header or row,
/// unsorted or duplicate timestamps, or a listed file that does not exist.
std::vector<StampedPath> read_list_csv(const std::string& path, const std::string& base_dir);

/// Reads a `timestamp_ns,x,y,z,qw,qx,qy,qz` trajectory (T_world_body).
/// Throws SequenceError for a missing file, a bad header or row, unsorted or
/// duplicate timestamps, or an invalid pose.
Trajectory read_trajectory_csv(const std::string& path);

/// Reads a LiDAR scan: little-endian float32 records x,y,z,intensity
/// (16 bytes per point), in the LiDAR frame. `t_ns` is stored as the scan
/// time unchanged. Throws SequenceError for a missing file, a size that is
/// not a multiple of 16 bytes, or a non-finite value.
LidarScan read_lidar_bin(const std::string& path, TimeNs t_ns);

/// Reads an RGB frame or thermal frame and checks its size against the
/// camera config. Throws SequenceError (also for ImageError problems).
RgbImage read_rgb_frame(const std::string& path, const SensorConfig& config);
RawThermalImage read_thermal_frame(const std::string& path, const SensorConfig& config);

/// Writers for adapters and the synthetic generator. Each throws
/// SequenceError if the file cannot be written.
void write_list_csv(const std::string& path, const std::vector<StampedPath>& entries);
void write_trajectory_csv(const std::string& path, const std::vector<StampedPose>& poses);
/// Writes points and intensities as float32 records (values are rounded to
/// float32). Throws std::invalid_argument if the counts differ.
void write_lidar_bin(const std::string& path, const std::vector<Eigen::Vector3d>& points,
                     const std::vector<float>& intensity);

/// Scan `index` of the sequence, with its time moved to the reference clock.
LidarScan load_lidar_scan(const Sequence& sequence, std::size_t index);

/// Association threshold for a camera: the manifest's tau_ns if given,
/// otherwise default_tau_ns() of the median LiDAR and camera periods. Throws
/// SequenceError if a stream has fewer than two stamps and no tau_ns is set.
TimeNs camera_tau_ns(const Sequence& sequence, const CameraStream& camera);

/// Associates every LiDAR scan with the camera's frames on the reference
/// clock, using camera_tau_ns(). One entry per scan.
std::vector<Association> associate_camera(const Sequence& sequence, const CameraStream& camera);

}  // namespace fusion
