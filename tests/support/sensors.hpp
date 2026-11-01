// Virtual pinhole cameras and spinning LiDAR for the synthetic scene.
// Test-only: not part of the fusion library's public API.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "rigid.hpp"
#include "scene.hpp"

namespace fusion::testing {

// Rotation that maps directions in a camera optical frame (x right, y down,
// z forward) into a body frame (x forward, y left, z up): R_body_optical.
Mat3 R_body_optical();

struct PinholeIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int width = 0;
    int height = 0;
};

struct Ray {
    Vec3 origin;
    Vec3 direction;  // unit
};

// Ideal pinhole camera with no distortion. The camera frame is the optical
// frame: x right, y down, z along the optical axis.
struct PinholeCamera {
    PinholeIntrinsics K;
    Rigid3 T_world_camera;

    // Unit ray through pixel coordinates (u, v), in the camera frame.
    // Pixel centres sit at integer coordinates.
    Vec3 pixel_direction_camera(double u, double v) const;

    // The same ray in the world frame.
    Ray pixel_ray_world(double u, double v) const;

    // Z-depth image (row-major, width * height) of the scene seen from this
    // camera. Pixels with no hit within max_distance hold 0.
    std::vector<double> render_depth(const Scene& scene, double max_distance) const;
};

// Ground-truth RGB and thermal cameras at different poses, both looking at
// the reference scene's box on +x.
PinholeCamera make_rgb_camera();
PinholeCamera make_thermal_camera();

struct LidarPoint {
    Vec3 p_lidar;      // point in the LiDAR frame
    double range = 0;  // measured range, metres
    int ring = 0;      // index into LidarConfig::elevations_rad
    int column = 0;    // azimuth step index
};

// Spinning LiDAR. The LiDAR frame has x forward, y left, z up; azimuth is
// measured from +x towards +y and elevation from the xy-plane towards +z.
struct LidarConfig {
    std::vector<double> elevations_rad;
    double azimuth_step_rad = 0.0;
    double min_range = 0.0;
    double max_range = 0.0;
    // Standard deviation of Gaussian noise added to each range. Zero gives
    // exact ranges.
    double range_noise_stddev = 0.0;
    // Seed for the range noise generator. Same seed, same scan.
    std::uint64_t seed = 0;
    Rigid3 T_world_lidar;
};

// 16 beams from -15 to +15 degrees, 0.5 degree azimuth step, mounted 1.8 m
// above the ground at the world origin, no noise.
LidarConfig make_reference_lidar();

// One full revolution. Beams that hit nothing within [min_range, max_range]
// produce no point.
std::vector<LidarPoint> scan(const Scene& scene, const LidarConfig& config);

}  // namespace fusion::testing
