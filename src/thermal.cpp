#include "fusion/thermal.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace fusion {

namespace {

constexpr double kCelsiusOffset = 273.15;

std::size_t pixel_count(int width, int height) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("RawThermalImage: width and height must be positive");
    }
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

void validate_window(const WindowLevel& window) {
    if (!std::isfinite(window.level) || !std::isfinite(window.width)) {
        throw std::invalid_argument("window/level: level and width must be finite");
    }
    if (!(window.width > 0.0)) {
        throw std::invalid_argument("window/level: width must be positive");
    }
}

std::uint8_t channel(double x) {
    const double c = std::clamp(x, 0.0, 1.0);
    return static_cast<std::uint8_t>(std::lround(255.0 * c));
}

}  // namespace

RawThermalImage::RawThermalImage(int width, int height)
    : width_(width), height_(height), values_(pixel_count(width, height), 0) {}

RawThermalImage::RawThermalImage(int width, int height, std::vector<std::uint16_t> values)
    : width_(width), height_(height), values_(std::move(values)) {
    if (values_.size() != pixel_count(width, height)) {
        throw std::invalid_argument("RawThermalImage: expected " +
                                    std::to_string(pixel_count(width, height)) +
                                    " values, got " + std::to_string(values_.size()));
    }
}

std::size_t RawThermalImage::index(int col, int row) const {
    if (col < 0 || col >= width_ || row < 0 || row >= height_) {
        throw std::out_of_range("RawThermalImage: pixel outside the image");
    }
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(col);
}

std::uint16_t RawThermalImage::at(int col, int row) const { return values_[index(col, row)]; }

void RawThermalImage::set(int col, int row, std::uint16_t value) {
    values_[index(col, row)] = value;
}

void validate(const PlanckParams& p) {
    if (!std::isfinite(p.R) || !std::isfinite(p.B) || !std::isfinite(p.F) ||
        !std::isfinite(p.O)) {
        throw std::invalid_argument("Planck constants must be finite");
    }
    if (!(p.R > 0.0)) throw std::invalid_argument("Planck constant R must be positive");
    if (!(p.B > 0.0)) throw std::invalid_argument("Planck constant B must be positive");
}

double raw_to_celsius(std::uint16_t raw, const PlanckParams& p) {
    validate(p);
    const double s = static_cast<double>(raw);
    const double denom = s - p.O;
    if (denom == 0.0) {
        throw std::domain_error("raw_to_celsius: raw value equals O (division by zero)");
    }
    const double arg = p.R / denom + p.F;
    if (!(arg > 0.0)) {
        throw std::domain_error(
            "raw_to_celsius: R / (S - O) + F is not positive (logarithm undefined)");
    }
    const double log = std::log(arg);
    if (log == 0.0) {
        throw std::domain_error("raw_to_celsius: R / (S - O) + F equals 1 (division by zero)");
    }
    const double kelvin = p.B / log;
    if (!std::isfinite(kelvin) || !(kelvin > 0.0)) {
        throw std::domain_error("raw_to_celsius: result is not above absolute zero");
    }
    return kelvin - kCelsiusOffset;
}

double window_level(double celsius, const WindowLevel& window) {
    validate_window(window);
    if (!std::isfinite(celsius)) {
        throw std::invalid_argument("window/level: temperature must be finite");
    }
    const double low = window.level - window.width / 2.0;
    return std::clamp((celsius - low) / window.width, 0.0, 1.0);
}

Rgb8 hot_colormap(double v) {
    if (std::isnan(v)) throw std::invalid_argument("hot_colormap: value is NaN");
    const double x = 3.0 * std::clamp(v, 0.0, 1.0);
    return {channel(x), channel(x - 1.0), channel(x - 2.0)};
}

DisplayImage render_display(const RawThermalImage& raw, const PlanckParams& params,
                            const WindowLevel& window) {
    validate(params);
    validate_window(window);
    DisplayImage out;
    out.width = raw.width();
    out.height = raw.height();
    out.pixels.reserve(raw.values().size());
    for (const std::uint16_t s : raw.values()) {
        Rgb8 color;  // black when the raw value cannot be converted
        try {
            color = hot_colormap(window_level(raw_to_celsius(s, params), window));
        } catch (const std::domain_error&) {
            // Leave the pixel black.
        }
        out.pixels.push_back(color);
    }
    return out;
}

ThermalProjection project_to_thermal(const CameraModel& camera,
                                     const Eigen::Isometry3d& T_thermal_lidar,
                                     const std::vector<Eigen::Vector3d>& points_lidar,
                                     const RawThermalImage& image) {
    if (image.width() != camera.width() || image.height() != camera.height()) {
        throw std::invalid_argument("project_to_thermal: image size differs from the camera's");
    }
    ThermalProjection out{SparseDepthMap(camera.width(), camera.height()), {}};
    for (std::size_t i = 0; i < points_lidar.size(); ++i) {
        if (const auto proj = project_point(camera, T_thermal_lidar, points_lidar[i])) {
            out.depth.insert(proj->col, proj->row, proj->depth);
            out.samples.push_back({i, *proj, image.at(proj->col, proj->row)});
        }
    }
    return out;
}

}  // namespace fusion
