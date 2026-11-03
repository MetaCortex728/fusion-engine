// Synthetic sequence in the on-disk sequence format, built from the moving
// rig of the synthetic scene: LiDAR scans, RGB and thermal images, and the
// body trajectory. Test support: not part of the fusion library's API.
//
// Timing (reference clock), for N scans (1 <= N <= 5):
//   LiDAR    t_l(k) = rig.t_lidar_ns - 100 ms * (N - 1 - k), k = 0..N-1,
//            so the last scan is the rig's scan.
//   RGB      t_l(0) + 33 ms + 50 ms * j, j = 0..2N-1 (20 Hz), tau_ns 25 ms.
//            Scan 0 is 33 ms from its nearest image and is rejected; every
//            other scan matches the image 17 ms before it.
//   thermal  t_l(k) + 21 ms (10 Hz), except that the frame for k = 3 is
//            missing, so scan 3 is 79 ms from its nearest image and is
//            rejected by the default tau of 50 ms.
// Sensor clocks: LiDAR offset 0, RGB offset -2 ms, thermal offset +5 ms
// (t_reference = t_sensor + time_offset_ns).
// Each image is rendered from the camera's true pose at its reference time.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <fusion/aligned_frame.hpp>
#include <fusion/config.hpp>
#include <fusion/image_io.hpp>
#include <fusion/pose.hpp>
#include <fusion/thermal.hpp>

#include "scene.hpp"
#include "sensors.hpp"

namespace fusion::testing {

struct ExpectedMatch {
    std::size_t index = 0;          // camera frame index
    std::int64_t t_ns = 0;          // its reference-clock time
    std::int64_t delta_t_ns = 0;    // t_ns - t_l
    bool accepted = false;
};

struct SyntheticCamera {
    fusion::SensorConfig config;
    std::vector<std::int64_t> stamps_ref_ns;     // reference clock
    std::vector<std::int64_t> stamps_sensor_ns;  // as written to the list file
    std::int64_t tau_ns = 0;                     // effective association threshold
    bool tau_in_manifest = false;                // false: the default is expected
    std::vector<ExpectedMatch> expected;         // one per scan, brute force
};

struct SyntheticSequence {
    Scene scene;
    MovingRig rig;
    std::int64_t lidar_time_offset_ns = 0;
    std::vector<fusion::LidarScan> scans;        // double precision, reference clock
    std::vector<std::vector<int>> scan_surfaces;  // scene surface per point
    SyntheticCamera rgb;
    SyntheticCamera thermal;
    std::vector<fusion::RgbImage> rgb_images;
    std::vector<fusion::RawThermalImage> thermal_images;
    std::vector<fusion::StampedPose> poses;      // T_world_body, every 40 ms
};

// Builds the sequence in memory. Throws std::invalid_argument unless
// 1 <= scans <= 5 (more would drive the rig into a box).
SyntheticSequence make_synthetic_sequence(int scans = 5);

// Writes the sequence to `dir` (created if needed): sequence.yaml, rgb.yaml,
// thermal.yaml, lidar.csv, rgb.csv, thermal.csv, trajectory.csv and the data
// files under lidar/, rgb/ and thermal/. Returns the manifest path.
std::string write_synthetic_sequence(const SyntheticSequence& seq, const std::string& dir);

// The value a float32 file stores for a coordinate.
inline float as_float32(double v) { return static_cast<float>(v); }

}  // namespace fusion::testing
