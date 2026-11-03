// Timestamped poses and a time-sorted trajectory with interpolation.
//
// A pose is T_world_body: it maps points from the body frame into the world
// frame, p_world = R * p_body + position.
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion/time.hpp"

namespace fusion {

struct StampedPose {
    TimeNs t_ns = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    /// The pose as a rigid transform T_world_body.
    Eigen::Isometry3d T_world_body() const;
};

/// Rotation between unit quaternions q0 (alpha = 0) and q1 (alpha = 1),
/// along the shortest arc: if q0 . q1 < 0, q1 is replaced by -q1, which is
/// the same rotation. With theta = acos(q0 . q1):
///   q = (sin((1 - alpha) theta) q0 + sin(alpha theta) q1) / sin(theta)
/// For nearly equal quaternions this falls back to normalized linear
/// interpolation. Returns q0 exactly for alpha == 0 and q1 (sign as given)
/// for alpha == 1.
Eigen::Quaterniond slerp_shortest(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1,
                                  double alpha);

/// Poses sorted by strictly increasing time.
class Trajectory {
public:
    /// Throws std::invalid_argument if `poses` is empty, its times are not
    /// strictly increasing (unsorted or duplicate), a value is not finite, or
    /// an orientation has zero norm. Orientations are normalized. Throws
    /// std::overflow_error if neighbouring times are too far apart for their
    /// difference to fit in TimeNs.
    explicit Trajectory(std::vector<StampedPose> poses);

    const std::vector<StampedPose>& poses() const noexcept { return poses_; }
    std::size_t size() const noexcept { return poses_.size(); }
    TimeNs start_ns() const noexcept { return poses_.front().t_ns; }
    TimeNs end_ns() const noexcept { return poses_.back().t_ns; }

    /// True if start_ns() <= t_ns <= end_ns().
    bool contains(TimeNs t_ns) const noexcept;

    /// Pose at t_ns, between the neighbouring poses i and i + 1 with
    /// t_i <= t_ns <= t_{i+1}:
    ///   alpha = (t_ns - t_i) / (t_{i+1} - t_i)
    ///   position = p_i + alpha (p_{i+1} - p_i)
    ///   orientation = slerp_shortest(q_i, q_{i+1}, alpha)
    /// At a stored time the stored pose is returned exactly. Throws
    /// std::out_of_range outside [start_ns(), end_ns()]; it never
    /// extrapolates.
    StampedPose interpolate(TimeNs t_ns) const;

    /// interpolate(t_ns) as T_world_body(t).
    Eigen::Isometry3d T_world_body(TimeNs t_ns) const;

private:
    std::vector<StampedPose> poses_;
};

}  // namespace fusion
