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
//
// Thermal: the scene is the reference scene's geometry with an apparent
// temperature per surface (kSyntheticSurfaceCelsius; the sky where nothing is
// hit is kSyntheticSkyCelsius). Each temperature is converted to a raw count
// with celsius_to_raw() and the camera's Planck constants kSyntheticPlanck,
// so the camera's conversion reads it back to within half a count (under
// 0.01 °C for these temperatures).
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

// Planck constants of the synthetic thermal camera (T = B / ln(R / (S - O) +
// F) - 273.15). Illustrative, not the calibration of a specific camera, but
// physically reasonable for an uncooled LWIR microbolometer: B = c2 / lambda
// with c2 = 14388 um K gives an effective wavelength of 10.1 um, the middle
// of the 8-14 um band; F = 1 makes S - O proportional to the Planck radiance
// at that wavelength; and 10 to 60 °C maps to counts 2384 to 5222 (14-bit
// range), about 49 counts per kelvin (0.02 K per count) at 20 °C, finer
// than the 30-60 mK noise of such cameras.
inline constexpr fusion::PlanckParams kSyntheticPlanck{380747.0, 1428.0, 1.0, -88.539};

// Apparent temperature (°C) of each surface of the synthetic thermal scene,
// indexed by surface id: the ground (road), the warm box ahead on +x
// (2 x 2 x 2 m, vehicle-sized, e.g. a running engine), the wall on +y, the
// low block on -x and the floating block on -y.
inline constexpr double kSyntheticSurfaceCelsius[] = {18.0, 45.0, 22.0, 12.0, 26.0};
// Apparent temperature of the clear sky, which is cold in the LWIR band.
inline constexpr double kSyntheticSkyCelsius = -15.0;

// Raw count whose Planck temperature is `celsius`: the inverse
// S = R / (exp(B / (T + 273.15)) - F) + O, rounded to the nearest integer.
// Throws std::invalid_argument for invalid constants (fusion::validate) and
// std::domain_error when exp(B / T) - F is not positive or the count is not
// in [0, 65535].
std::uint16_t celsius_to_raw(double celsius, const fusion::PlanckParams& params);

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
