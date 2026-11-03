// Scan-level motion compensation of LiDAR points.
//
// A scan taken at t_l is moved to a camera time t_c using the body
// trajectory and the LiDAR mounting T_body_lidar:
//
//   p_L(t_c) = T_body_lidar^-1 * T_world_body(t_c)^-1 * T_world_body(t_l)
//              * T_body_lidar * p_L
//
// The result is still in the LiDAR frame, now as the LiDAR sits at t_c, so
// projecting it with the camera extrinsics T_camera_lidar works unchanged.
//
// The whole scan shares one time t_l (scan-level). Per-point deskew, which
// needs a time per point, is not implemented.
#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion/pose.hpp"
#include "fusion/time.hpp"

namespace fusion {

/// T_lidar(t_c)_lidar(t_l): maps points from the LiDAR frame at t_scan_ns
/// into the LiDAR frame at t_target_ns. Throws std::out_of_range if either
/// time lies outside the trajectory.
Eigen::Isometry3d scan_motion_transform(const Trajectory& trajectory,
                                        const Eigen::Isometry3d& T_body_lidar,
                                        TimeNs t_scan_ns, TimeNs t_target_ns);

/// Applies scan_motion_transform() to every point. Returns the points
/// unchanged (exact copies) when t_scan_ns == t_target_ns or when the
/// trajectory gives the same body pose at both times. Throws
/// std::out_of_range if either time lies outside the trajectory.
std::vector<Eigen::Vector3d> compensate_scan(const std::vector<Eigen::Vector3d>& points_lidar,
                                             const Trajectory& trajectory,
                                             const Eigen::Isometry3d& T_body_lidar,
                                             TimeNs t_scan_ns, TimeNs t_target_ns);

}  // namespace fusion
