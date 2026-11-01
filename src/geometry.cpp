#include "fusion/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace fusion {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Derivative of r * radial(r) with respect to r, written in s = r^2.
double radial_slope(const Distortion& d, double s) {
    return 1.0 + s * (3.0 * d.k1 + s * (5.0 * d.k2 + s * 7.0 * d.k3));
}

// Smallest r > 0 where the radial distortion stops being monotonic, or
// infinity. The slope is a cubic in s = r^2 that starts at 1; scan s on a
// geometric grid up to 1e6 (r = 1000, about 89.94 degrees off axis), then
// bisect the first sign change.
double monotonic_radius(const Distortion& d) {
    if (d.k1 >= 0.0 && d.k2 >= 0.0 && d.k3 >= 0.0) return kInf;
    double lo = 0.0;
    double hi = 0.0;
    bool found = false;
    for (double s = 1e-8; s <= 1e6; s *= 1.01) {
        if (radial_slope(d, s) <= 0.0) {
            hi = s;
            found = true;
            break;
        }
        lo = s;
    }
    if (!found) return kInf;
    for (int i = 0; i < 200 && hi - lo > 1e-15 * hi; ++i) {
        const double mid = 0.5 * (lo + hi);
        (radial_slope(d, mid) > 0.0 ? lo : hi) = mid;
    }
    return std::sqrt(lo);
}

void require(bool ok, const std::string& what) {
    if (!ok) throw std::invalid_argument("CameraModel: " + what);
}

}  // namespace

CameraModel::CameraModel(int width, int height, const Eigen::Matrix3d& K,
                         const Distortion& distortion)
    : width_(width), height_(height), K_(K), distortion_(distortion), max_radius_(kInf) {
    require(width > 0 && height > 0, "width and height must be positive");
    require(K.allFinite(), "K must be finite");
    require(K(0, 0) > 0.0 && K(1, 1) > 0.0, "fx and fy must be positive");
    require(K(1, 0) == 0.0 && K(2, 0) == 0.0 && K(2, 1) == 0.0 && K(2, 2) == 1.0,
            "K must have the form [fx s cx; 0 fy cy; 0 0 1]");
    const Distortion& d = distortion;
    require(std::isfinite(d.k1) && std::isfinite(d.k2) && std::isfinite(d.p1) &&
                std::isfinite(d.p2) && std::isfinite(d.k3),
            "distortion coefficients must be finite");
    max_radius_ = monotonic_radius(d);
}

Eigen::Vector2d CameraModel::distort(const Eigen::Vector2d& n) const {
    const Distortion& d = distortion_;
    const double x = n.x();
    const double y = n.y();
    const double r2 = x * x + y * y;
    const double radial = 1.0 + r2 * (d.k1 + r2 * (d.k2 + r2 * d.k3));
    return {x * radial + 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * x * x),
            y * radial + d.p1 * (r2 + 2.0 * y * y) + 2.0 * d.p2 * x * y};
}

std::optional<Eigen::Vector2d> CameraModel::undistort(const Eigen::Vector2d& target) const {
    if (!target.allFinite()) return std::nullopt;
    if (distortion_.is_zero()) return target;
    const Distortion& d = distortion_;
    const double tol = 1e-15 * std::max(1.0, target.norm());
    Eigen::Vector2d n = target;
    bool converged = false;
    for (int i = 0; i < 50; ++i) {
        const Eigen::Vector2d residual = distort(n) - target;
        if (residual.norm() <= tol) {
            converged = true;
            break;
        }
        const double x = n.x();
        const double y = n.y();
        const double r2 = x * x + y * y;
        const double radial = 1.0 + r2 * (d.k1 + r2 * (d.k2 + r2 * d.k3));
        const double dradial = d.k1 + r2 * (2.0 * d.k2 + r2 * 3.0 * d.k3);  // d radial / d r2
        Eigen::Matrix2d J;
        J(0, 0) = radial + 2.0 * x * x * dradial + 2.0 * d.p1 * y + 6.0 * d.p2 * x;
        J(0, 1) = 2.0 * x * y * dradial + 2.0 * d.p1 * x + 2.0 * d.p2 * y;
        J(1, 0) = J(0, 1);
        J(1, 1) = radial + 2.0 * y * y * dradial + 6.0 * d.p1 * y + 2.0 * d.p2 * x;
        const double det = J.determinant();
        if (!std::isfinite(det) || det == 0.0) return std::nullopt;
        n -= J.inverse() * residual;
        if (!n.allFinite()) return std::nullopt;
    }
    // Accept a final residual a little above the stopping tolerance: the
    // last Newton step can stall at rounding level.
    if (!converged && (distort(n) - target).norm() > 1e3 * tol) return std::nullopt;
    if (n.norm() >= max_radius_) return std::nullopt;
    return n;
}

Eigen::Vector2d CameraModel::to_pixel(const Eigen::Vector2d& n) const {
    return {K_(0, 0) * n.x() + K_(0, 1) * n.y() + K_(0, 2), K_(1, 1) * n.y() + K_(1, 2)};
}

Eigen::Vector2d CameraModel::from_pixel(const Eigen::Vector2d& uv) const {
    const double y = (uv.y() - K_(1, 2)) / K_(1, 1);
    const double x = (uv.x() - K_(0, 2) - K_(0, 1) * y) / K_(0, 0);
    return {x, y};
}

std::optional<Projection> CameraModel::project(const Eigen::Vector3d& p) const {
    // Written as !(z > 0) so NaN is rejected too.
    if (!(p.z() > 0.0) || !p.allFinite()) return std::nullopt;
    const Eigen::Vector2d n(p.x() / p.z(), p.y() / p.z());
    if (!(n.norm() < max_radius_)) return std::nullopt;
    const Eigen::Vector2d uv = to_pixel(distort(n));
    const double u = uv.x();
    const double v = uv.y();
    // Also rejects NaN and infinity.
    if (!(u >= -0.5 && u < width_ - 0.5 && v >= -0.5 && v < height_ - 0.5)) {
        return std::nullopt;
    }
    Projection out;
    out.pixel = uv;
    out.depth = p.z();
    // Clamp guards the half-open upper edge against rounding in u + 0.5.
    out.col = std::min(static_cast<int>(std::floor(u + 0.5)), width_ - 1);
    out.row = std::min(static_cast<int>(std::floor(v + 0.5)), height_ - 1);
    return out;
}

std::optional<Projection> project_point(const CameraModel& camera,
                                        const Eigen::Isometry3d& T_camera_lidar,
                                        const Eigen::Vector3d& p_lidar) {
    return camera.project(T_camera_lidar * p_lidar);
}

}  // namespace fusion
