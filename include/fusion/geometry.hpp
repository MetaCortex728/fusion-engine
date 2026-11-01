// Pinhole camera with radial-tangential distortion, and projection of 3D
// points into pixels.
//
// Conventions:
// - The camera frame is the optical frame: x right, y down, z forward.
// - Pixel centres sit at integer coordinates. Pixel (col, row) covers
//   u in [col - 0.5, col + 0.5) and v in [row - 0.5, row + 0.5), so the image
//   spans u in [-0.5, width - 0.5) and v in [-0.5, height - 0.5).
// - Depth is the z coordinate in the camera frame (z-depth), not the range.
#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fusion {

/// Radial-tangential (Brown-Conrady) distortion coefficients, in the same
/// order and meaning as OpenCV's five-coefficient model.
///
/// For normalized undistorted coordinates (x, y) with r2 = x^2 + y^2:
///   radial = 1 + k1 r2 + k2 r2^2 + k3 r2^3
///   x_d = x radial + 2 p1 x y + p2 (r2 + 2 x^2)
///   y_d = y radial + p1 (r2 + 2 y^2) + 2 p2 x y
struct Distortion {
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;

    bool is_zero() const noexcept {
        return k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0 && k3 == 0.0;
    }
};

/// A point projected into the image.
struct Projection {
    Eigen::Vector2d pixel;  ///< sub-pixel (u, v)
    double depth = 0.0;     ///< z in the camera frame, metres, > 0
    int col = 0;            ///< pixel column containing `pixel`, in [0, width)
    int row = 0;            ///< pixel row containing `pixel`, in [0, height)
};

class CameraModel {
public:
    /// `K` is the 3x3 intrinsic matrix [fx s cx; 0 fy cy; 0 0 1] with
    /// fx, fy > 0. Throws std::invalid_argument for a non-positive size, a
    /// malformed K, or non-finite distortion coefficients.
    CameraModel(int width, int height, const Eigen::Matrix3d& K,
                const Distortion& distortion = {});

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }
    const Eigen::Matrix3d& K() const noexcept { return K_; }
    const Distortion& distortion() const noexcept { return distortion_; }

    /// Largest normalized radius sqrt(x^2 + y^2) for which the radial
    /// distortion is still monotonic. Points beyond it are rejected by
    /// project(), because there the model folds distant points back into
    /// the image. Infinity when the radial part is monotonic everywhere.
    double max_normalized_radius() const noexcept { return max_radius_; }

    /// Applies distortion to normalized undistorted coordinates.
    Eigen::Vector2d distort(const Eigen::Vector2d& normalized) const;

    /// Inverts distort() with Newton iterations. Returns nullopt if the
    /// iteration does not converge or the result lies beyond
    /// max_normalized_radius().
    std::optional<Eigen::Vector2d> undistort(const Eigen::Vector2d& distorted) const;

    /// Normalized (distorted) coordinates to pixel coordinates via K.
    Eigen::Vector2d to_pixel(const Eigen::Vector2d& distorted) const;

    /// Pixel coordinates to normalized (distorted) coordinates via K^-1.
    Eigen::Vector2d from_pixel(const Eigen::Vector2d& pixel) const;

    /// Projects a point given in the camera frame. Returns nullopt unless
    /// z > 0, the point lies within max_normalized_radius(), and the pixel
    /// lands inside the image.
    std::optional<Projection> project(const Eigen::Vector3d& p_camera) const;

private:
    int width_;
    int height_;
    Eigen::Matrix3d K_;
    Distortion distortion_;
    double max_radius_;
};

/// Projects a LiDAR point into the camera: p_camera = T_camera_lidar * p_lidar.
std::optional<Projection> project_point(const CameraModel& camera,
                                        const Eigen::Isometry3d& T_camera_lidar,
                                        const Eigen::Vector3d& p_lidar);

}  // namespace fusion
