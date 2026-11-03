#include "fusion/pose.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace fusion {

namespace {

// Above this |q0 . q1| the quaternions are so close that sin(theta) loses
// precision; normalized linear interpolation is accurate there.
constexpr double kNlerpDot = 1.0 - 1e-12;

bool is_finite(const StampedPose& p) {
    return p.position.allFinite() && p.orientation.coeffs().allFinite();
}

}  // namespace

Eigen::Isometry3d StampedPose::T_world_body() const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = orientation.toRotationMatrix();
    T.translation() = position;
    return T;
}

Eigen::Quaterniond slerp_shortest(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1,
                                  double alpha) {
    if (alpha == 0.0) return q0;
    if (alpha == 1.0) return q1;
    if (q0.coeffs() == q1.coeffs()) return q0;

    Eigen::Vector4d a = q0.coeffs();
    Eigen::Vector4d b = q1.coeffs();
    double dot = a.dot(b);
    if (dot < 0.0) {  // shortest arc: -q1 is the same rotation as q1
        b = -b;
        dot = -dot;
    }
    Eigen::Vector4d c;
    if (dot > kNlerpDot) {
        c = a + alpha * (b - a);
    } else {
        const double theta = std::acos(std::min(dot, 1.0));
        const double s = std::sin(theta);
        c = (std::sin((1.0 - alpha) * theta) / s) * a + (std::sin(alpha * theta) / s) * b;
    }
    c.normalize();
    Eigen::Quaterniond q;
    q.coeffs() = c;
    return q;
}

Trajectory::Trajectory(std::vector<StampedPose> poses) : poses_(std::move(poses)) {
    if (poses_.empty()) throw std::invalid_argument("fusion::Trajectory: no poses");
    for (std::size_t i = 0; i < poses_.size(); ++i) {
        StampedPose& p = poses_[i];
        if (!is_finite(p)) {
            throw std::invalid_argument("fusion::Trajectory: pose " + std::to_string(i) +
                                        " is not finite");
        }
        const double n = p.orientation.norm();
        if (n == 0.0) {
            throw std::invalid_argument("fusion::Trajectory: pose " + std::to_string(i) +
                                        " has a zero quaternion");
        }
        p.orientation.coeffs() /= n;
        if (i == 0) continue;
        const TimeNs prev = poses_[i - 1].t_ns;
        if (p.t_ns == prev) {
            throw std::invalid_argument("fusion::Trajectory: duplicate time at pose " +
                                        std::to_string(i));
        }
        if (p.t_ns < prev) {
            throw std::invalid_argument("fusion::Trajectory: times not sorted at pose " +
                                        std::to_string(i));
        }
        // Strictly increasing, so the difference is positive; check it fits.
        if (prev < 0 && p.t_ns > std::numeric_limits<TimeNs>::max() + prev) {
            throw std::overflow_error("fusion::Trajectory: interval at pose " +
                                      std::to_string(i) + " does not fit in int64 nanoseconds");
        }
    }
}

bool Trajectory::contains(TimeNs t_ns) const noexcept {
    return t_ns >= start_ns() && t_ns <= end_ns();
}

StampedPose Trajectory::interpolate(TimeNs t_ns) const {
    if (!contains(t_ns)) {
        throw std::out_of_range("fusion::Trajectory::interpolate: time " + std::to_string(t_ns) +
                                " ns is outside [" + std::to_string(start_ns()) + ", " +
                                std::to_string(end_ns()) + "]");
    }
    // First pose with t > t_ns; the segment is [it - 1, it].
    const auto it = std::upper_bound(
        poses_.begin(), poses_.end(), t_ns,
        [](TimeNs t, const StampedPose& p) { return t < p.t_ns; });
    const StampedPose& p0 = *(it - 1);
    if (p0.t_ns == t_ns || it == poses_.end()) return p0;
    const StampedPose& p1 = *it;

    const double alpha =
        static_cast<double>(t_ns - p0.t_ns) / static_cast<double>(p1.t_ns - p0.t_ns);
    StampedPose out;
    out.t_ns = t_ns;
    out.position = p0.position + alpha * (p1.position - p0.position);
    out.orientation = slerp_shortest(p0.orientation, p1.orientation, alpha);
    return out;
}

Eigen::Isometry3d Trajectory::T_world_body(TimeNs t_ns) const {
    return interpolate(t_ns).T_world_body();
}

}  // namespace fusion
