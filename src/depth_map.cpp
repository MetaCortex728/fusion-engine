#include "fusion/depth_map.hpp"

#include <cmath>
#include <stdexcept>

namespace fusion {

SparseDepthMap::SparseDepthMap(int width, int height) : width_(width), height_(height) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("SparseDepthMap: width and height must be positive");
    }
    depth_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0.0);
}

bool SparseDepthMap::insert(int col, int row, double depth) {
    if (col < 0 || col >= width_ || row < 0 || row >= height_) return false;
    if (!std::isfinite(depth) || depth <= 0.0) return false;
    double& slot = depth_[static_cast<std::size_t>(row) * static_cast<std::size_t>(width_) +
                          static_cast<std::size_t>(col)];
    if (slot == 0.0) {
        ++filled_;
    } else if (!(depth < slot)) {
        return false;
    }
    slot = depth;
    return true;
}

std::optional<double> SparseDepthMap::at(int col, int row) const {
    if (col < 0 || col >= width_ || row < 0 || row >= height_) return std::nullopt;
    const double d = depth_[static_cast<std::size_t>(row) * static_cast<std::size_t>(width_) +
                            static_cast<std::size_t>(col)];
    if (d == 0.0) return std::nullopt;
    return d;
}

std::vector<SparseDepthMap::Entry> SparseDepthMap::entries() const {
    std::vector<Entry> out;
    out.reserve(filled_);
    std::size_t i = 0;
    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col, ++i) {
            if (depth_[i] != 0.0) out.push_back({col, row, depth_[i]});
        }
    }
    return out;
}

SparseDepthMap project_to_depth_map(const CameraModel& camera,
                                    const Eigen::Isometry3d& T_camera_lidar,
                                    const std::vector<Eigen::Vector3d>& points_lidar) {
    SparseDepthMap map(camera.width(), camera.height());
    for (const Eigen::Vector3d& p : points_lidar) {
        if (const auto proj = project_point(camera, T_camera_lidar, p)) {
            map.insert(proj->col, proj->row, proj->depth);
        }
    }
    return map;
}

}  // namespace fusion
