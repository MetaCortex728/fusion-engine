// Thermal camera images: raw counts, conversion to degrees Celsius, display
// rendering, and LiDAR projection into a thermal camera.
//
// Raw counts are the camera's 16-bit values and are never modified. Celsius
// and display images are computed from them on demand.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion/depth_map.hpp"
#include "fusion/geometry.hpp"

namespace fusion {

/// Raw 16-bit thermal image, row-major. Stores exactly the values it is given.
class RawThermalImage {
public:
    /// All pixels 0. Throws std::invalid_argument for a non-positive size.
    RawThermalImage(int width, int height);

    /// Takes `values` (row-major, width * height) unchanged. Throws
    /// std::invalid_argument for a non-positive size or a size mismatch.
    RawThermalImage(int width, int height, std::vector<std::uint16_t> values);

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }

    /// Raw value at (col, row). Throws std::out_of_range outside the image.
    std::uint16_t at(int col, int row) const;

    /// Sets the raw value at (col, row). Throws std::out_of_range outside
    /// the image.
    void set(int col, int row, std::uint16_t value);

    /// All raw values, row-major.
    const std::vector<std::uint16_t>& values() const noexcept { return values_; }

private:
    std::size_t index(int col, int row) const;

    int width_;
    int height_;
    std::vector<std::uint16_t> values_;
};

/// Per-camera Planck constants for converting a raw count S to temperature:
///
///   T = B / ln(R / (S - O) + F) - 273.15     (degrees Celsius)
///
/// Valid constants are finite with R > 0 and B > 0.
struct PlanckParams {
    double R = 0.0;
    double B = 0.0;
    double F = 0.0;
    double O = 0.0;
};

/// Throws std::invalid_argument unless all constants are finite, R > 0, and
/// B > 0.
void validate(const PlanckParams& params);

/// Converts a raw count to degrees Celsius with the Planck form above.
/// Throws std::invalid_argument for invalid constants (see validate()), and
/// std::domain_error when S == O, when the logarithm's argument
/// R / (S - O) + F is not positive (logarithm undefined), when it equals 1
/// (division by zero), or when the result is not above absolute zero.
double raw_to_celsius(std::uint16_t raw, const PlanckParams& params);

/// Display window in degrees Celsius: `level` is the centre and `width` the
/// span. Valid windows have finite values and width > 0.
struct WindowLevel {
    double level = 0.0;
    double width = 0.0;
};

/// v = clamp((T - (level - width / 2)) / width, 0, 1). Throws
/// std::invalid_argument for width <= 0 or any non-finite input.
double window_level(double celsius, const WindowLevel& window);

struct Rgb8 {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

inline bool operator==(const Rgb8& a, const Rgb8& b) {
    return a.r == b.r && a.g == b.g && a.b == b.b;
}

/// "Hot" colormap from black through red and yellow to white. v is clamped
/// to [0, 1]; each channel is round(255 * clamp(3 v - k, 0, 1)) with k = 0
/// for red, 1 for green, 2 for blue. Throws std::invalid_argument for NaN.
Rgb8 hot_colormap(double v);

/// 8-bit RGB display image, row-major.
struct DisplayImage {
    int width = 0;
    int height = 0;
    std::vector<Rgb8> pixels;

    const Rgb8& at(int col, int row) const {
        return pixels.at(static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
                         static_cast<std::size_t>(col));
    }
};

/// Renders raw counts for display: Celsius, then window/level, then the hot
/// colormap. The raw image is only read. Pixels whose raw value cannot be
/// converted (raw_to_celsius() throws std::domain_error) are drawn black.
/// Throws std::invalid_argument for invalid constants or window.
DisplayImage render_display(const RawThermalImage& raw, const PlanckParams& params,
                            const WindowLevel& window);

/// One LiDAR point projected into the thermal camera.
struct ThermalSample {
    std::size_t point_index = 0;  ///< index into the input point list
    Projection projection;        ///< pixel, depth, col, row
    std::uint16_t raw = 0;        ///< raw count at (col, row)
};

struct ThermalProjection {
    SparseDepthMap depth;                ///< nearest depth per thermal pixel
    std::vector<ThermalSample> samples;  ///< every point that projected, in input order
};

/// Projects LiDAR points into the thermal camera
/// (p_thermal = T_thermal_lidar * p_lidar) with project_point(), building the
/// sparse depth map and reading each point's raw value at its pixel. Points
/// rejected by CameraModel::project() are skipped. Throws
/// std::invalid_argument if the image size differs from the camera's.
ThermalProjection project_to_thermal(const CameraModel& camera,
                                     const Eigen::Isometry3d& T_thermal_lidar,
                                     const std::vector<Eigen::Vector3d>& points_lidar,
                                     const RawThermalImage& image);

}  // namespace fusion
