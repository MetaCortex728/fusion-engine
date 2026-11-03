// Aligned frame: the engine's fused result for one reference time.
//
// The reference is one LiDAR scan taken at t_l. Each camera (RGB, thermal)
// contributes the image that time association picked for t_l, if any. For
// every accepted camera the scan is moved to that camera's capture time with
// scan-level motion compensation (when a trajectory covers both times) and
// projected into the camera with the z-buffered projection, giving a sparse
// depth map and, per point, the pixel, depth and, for thermal, the raw count.
//
// The frame owns all of its data. Building one reads its inputs and never
// modifies them; there is no global state.
#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion/config.hpp"
#include "fusion/depth_map.hpp"
#include "fusion/geometry.hpp"
#include "fusion/pose.hpp"
#include "fusion/thermal.hpp"
#include "fusion/time.hpp"

namespace fusion {

/// One LiDAR scan with a single (scan-level) time.
struct LidarScan {
    TimeNs t_ns = 0;                       ///< scan time on the reference clock
    std::vector<Eigen::Vector3d> points;   ///< LiDAR frame, metres
    std::vector<float> intensity;          ///< one value per point
};

/// One camera's input for a reference time.
struct CameraInput {
    /// Camera config. Its T_sensor_source is used as T_camera_lidar.
    const SensorConfig* config = nullptr;
    /// The association for this reference time, from associate(): its
    /// t_ref_ns must equal the scan's time. Only an accepted association is
    /// projected.
    Association association;
    /// Thermal cameras: the associated raw image, required when accepted.
    /// Ignored for the RGB camera.
    const RawThermalImage* thermal_image = nullptr;
};

/// Everything needed to build one aligned frame. Pointers are not owned and
/// only read during build_aligned_frame().
struct FrameInputs {
    const LidarScan* scan = nullptr;
    /// Optional body trajectory (T_world_body). Without it, or when it does
    /// not cover both the scan time and a camera's time, the scan is
    /// projected into that camera uncompensated.
    const Trajectory* trajectory = nullptr;
    /// LiDAR mounting on the body, used for compensation.
    Eigen::Isometry3d T_body_lidar = Eigen::Isometry3d::Identity();
    std::optional<CameraInput> rgb;
    std::optional<CameraInput> thermal;
};

/// One scan point as seen by one camera.
struct PointInCamera {
    /// Pixel and depth of the (compensated) point, or nullopt when it is not
    /// visible: behind the camera, outside the image, or beyond the
    /// distortion model's valid radius.
    std::optional<Projection> projection;
    /// True if this point holds the depth map's value at its pixel: it is the
    /// nearest of the points landing there (the first one on a tie).
    bool nearest = false;
    /// Thermal camera only: the raw count at the point's pixel.
    std::optional<std::uint16_t> raw;
};

/// The result for one camera.
struct CameraFrame {
    std::string name;          ///< SensorConfig::name
    Association association;   ///< delta_t_ns and accepted flag
    /// True if the scan was moved to the camera's time before projecting.
    bool compensated = false;
    /// Sparse depth map over the camera's pixel grid. Present only when the
    /// association was accepted.
    std::optional<SparseDepthMap> depth;
    /// One entry per scan point when accepted, empty otherwise.
    std::vector<PointInCamera> points;
    /// Number of scan points with a projection.
    std::size_t projected_count = 0;
};

struct AlignedFrame {
    TimeNs t_ref_ns = 0;                  ///< the scan's time, reference clock
    std::vector<Eigen::Vector3d> points;  ///< scan points at t_ref_ns, LiDAR frame
    std::vector<float> intensity;
    /// Present when the corresponding camera input was given, whether or not
    /// its association was accepted.
    std::optional<CameraFrame> rgb;
    std::optional<CameraFrame> thermal;
};

/// Builds the aligned frame. Throws std::invalid_argument when the scan is
/// missing, the intensity count differs from the point count, a camera has
/// no config, a camera's association is for another reference time, or an
/// accepted thermal camera has no image or one of the wrong size.
AlignedFrame build_aligned_frame(const FrameInputs& inputs);

/// Writes the per-point output as CSV with the header
///   point,x,y,z,intensity,
///   rgb_u,rgb_v,rgb_depth,rgb_nearest,
///   thermal_u,thermal_v,thermal_depth,thermal_raw,thermal_nearest
/// (one line) and one row per scan point. A camera's fields are empty when
/// the camera is absent, its association was rejected, or the point is not
/// visible in it. Numbers use the shortest text that reads back exactly.
void write_aligned_frame_csv(std::ostream& out, const AlignedFrame& frame);

/// Same, to a file. Throws std::runtime_error if it cannot be written.
void write_aligned_frame_csv(const std::string& path, const AlignedFrame& frame);

}  // namespace fusion
