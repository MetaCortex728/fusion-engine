// Sparse depth map over a camera's pixel grid, filled by projecting points.
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion/geometry.hpp"

namespace fusion {

/// Depth per pixel for a width x height grid, where most pixels are empty.
/// Acts as a depth buffer: when several points land on one pixel, the
/// nearest (smallest positive depth) is kept; on a tie the first one stays.
class SparseDepthMap {
public:
    struct Entry {
        int col = 0;
        int row = 0;
        double depth = 0.0;  ///< z in the camera frame, metres
    };

    /// Throws std::invalid_argument for a non-positive size.
    SparseDepthMap(int width, int height);

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }

    /// Offers a depth for pixel (col, row). Returns true if it was stored,
    /// false if the pixel already holds a depth that is not farther, or if
    /// the pixel or depth is invalid (outside the grid, not finite, <= 0).
    bool insert(int col, int row, double depth);

    /// Depth at (col, row), or nullopt if empty or outside the grid.
    std::optional<double> at(int col, int row) const;

    /// Number of pixels holding a depth.
    std::size_t size() const noexcept { return filled_; }
    bool empty() const noexcept { return filled_ == 0; }

    /// Filled pixels in row-major order.
    std::vector<Entry> entries() const;

private:
    int width_;
    int height_;
    std::size_t filled_ = 0;
    std::vector<double> depth_;  // row-major, 0 = empty
};

/// Projects LiDAR points into the camera (p_camera = T_camera_lidar * p_lidar)
/// and keeps the nearest depth per pixel. Points rejected by
/// CameraModel::project() are skipped.
SparseDepthMap project_to_depth_map(const CameraModel& camera,
                                    const Eigen::Isometry3d& T_camera_lidar,
                                    const std::vector<Eigen::Vector3d>& points_lidar);

}  // namespace fusion
