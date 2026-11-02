#include "fusion/lidar.hpp"

namespace fusion {

namespace {

bool same_pose(const StampedPose& a, const StampedPose& b) {
    return a.position == b.position && a.orientation.coeffs() == b.orientation.coeffs();
}

Eigen::Isometry3d motion_transform(const StampedPose& at_scan, const StampedPose& at_target,
                                   const Eigen::Isometry3d& T_body_lidar) {
    return T_body_lidar.inverse() * at_target.T_world_body().inverse() *
           at_scan.T_world_body() * T_body_lidar;
}

}  // namespace

Eigen::Isometry3d scan_motion_transform(const Trajectory& trajectory,
                                        const Eigen::Isometry3d& T_body_lidar,
                                        TimeNs t_scan_ns, TimeNs t_target_ns) {
    return motion_transform(trajectory.interpolate(t_scan_ns),
                            trajectory.interpolate(t_target_ns), T_body_lidar);
}

std::vector<Eigen::Vector3d> compensate_scan(const std::vector<Eigen::Vector3d>& points_lidar,
                                             const Trajectory& trajectory,
                                             const Eigen::Isometry3d& T_body_lidar,
                                             TimeNs t_scan_ns, TimeNs t_target_ns) {
    // Both lookups run first, so out-of-range times throw even when the
    // shortcut below would apply.
    const StampedPose at_scan = trajectory.interpolate(t_scan_ns);
    const StampedPose at_target = trajectory.interpolate(t_target_ns);
    if (t_scan_ns == t_target_ns || same_pose(at_scan, at_target)) return points_lidar;

    const Eigen::Isometry3d T = motion_transform(at_scan, at_target, T_body_lidar);
    std::vector<Eigen::Vector3d> out;
    out.reserve(points_lidar.size());
    for (const Eigen::Vector3d& p : points_lidar) out.push_back(T * p);
    return out;
}

}  // namespace fusion
