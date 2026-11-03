// Scan-level motion compensation on the moving synthetic rig: a LiDAR scan
// at t_l is moved to the camera time t_c = t_l + 33 ms and checked against
// the camera's ray-cast depth at its true pose at t_c.
#include <doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fusion/geometry.hpp>
#include <fusion/lidar.hpp>
#include <fusion/pose.hpp>

#include "support/scene.hpp"
#include "support/sensors.hpp"

using namespace fusion::testing;

namespace {

constexpr double kMaxDistance = 100.0;
// Same relative depth tolerance as the RGB projection test.
constexpr double kDepthRelTol = 1e-9;
constexpr std::int64_t kMs = 1'000'000;

fusion::CameraModel model_of(const PinholeCamera& cam) {
    Eigen::Matrix3d K;
    K << cam.K.fx, 0.0, cam.K.cx,
         0.0, cam.K.fy, cam.K.cy,
         0.0, 0.0, 1.0;
    return fusion::CameraModel(cam.K.width, cam.K.height, K);
}

// Poses of the body sampled every `step_ns` over [from_ns, to_ns].
fusion::Trajectory sample_trajectory(const MovingBody& body, std::int64_t from_ns,
                                     std::int64_t to_ns, std::int64_t step_ns) {
    std::vector<fusion::StampedPose> poses;
    for (std::int64_t t = from_ns; t <= to_ns; t += step_ns) {
        const Rigid3 T = body.T_world_body(t);
        poses.push_back({t, T.translation(), Eigen::Quaterniond(T.linear())});
    }
    return fusion::Trajectory(std::move(poses));
}

std::vector<Eigen::Vector3d> points_of(const std::vector<LidarPoint>& scan_points) {
    std::vector<Eigen::Vector3d> out;
    out.reserve(scan_points.size());
    for (const LidarPoint& lp : scan_points) out.push_back(lp.p_lidar);
    return out;
}

struct DepthCheck {
    std::size_t projected = 0;
    std::size_t agree = 0;     // |camera depth - LiDAR depth| <= kDepthRelTol * depth
    std::size_t in_front = 0;  // camera sees farther than the point: impossible if aligned
    double median_abs_error = 0.0;  // metres; a ray that sees nothing counts as infinite
};

// Projects LiDAR-frame points into the camera and compares each projected
// depth with the depth the camera ray-casts through the same sub-pixel
// position. With `boxes_only`, only points whose beam hit a box count.
DepthCheck check_depth(const Scene& scene, const PinholeCamera& cam,
                       const Rigid3& T_camera_lidar,
                       const std::vector<Eigen::Vector3d>& points_lidar,
                       const std::vector<int>& surfaces, bool boxes_only) {
    const fusion::CameraModel model = model_of(cam);
    DepthCheck r;
    std::vector<double> errors;
    for (std::size_t i = 0; i < points_lidar.size(); ++i) {
        if (boxes_only && surfaces[i] < kFirstBoxSurface) continue;
        const auto proj = fusion::project_point(model, T_camera_lidar, points_lidar[i]);
        if (!proj) continue;
        const Vec3 d_cam = cam.pixel_direction_camera(proj->pixel.x(), proj->pixel.y());
        const Ray ray = cam.pixel_ray_world(proj->pixel.x(), proj->pixel.y());
        const auto hit = scene.raycast(ray.origin, ray.direction, kMaxDistance);
        ++r.projected;
        const double camera_depth =
            hit ? hit->distance * d_cam.z() : std::numeric_limits<double>::infinity();
        const double diff = camera_depth - proj->depth;
        errors.push_back(std::abs(diff));
        if (diff > kDepthRelTol * proj->depth) ++r.in_front;
        if (std::abs(diff) <= kDepthRelTol * proj->depth) ++r.agree;
    }
    if (!errors.empty()) {
        std::nth_element(errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2),
                         errors.end());
        r.median_abs_error = errors[errors.size() / 2];
    }
    return r;
}

}  // namespace

TEST_CASE("scan motion transform matches the rig's true relative LiDAR pose") {
    const MovingRig rig = make_moving_rig();
    const fusion::Trajectory traj = sample_trajectory(
        rig.body, rig.body.t0_ns, rig.body.t0_ns + 400 * kMs, 40 * kMs);
    // t_l = t0 + 107 ms and t_c = t0 + 140 ms fall in different segments,
    // neither on a sample.
    REQUIRE((rig.t_lidar_ns - rig.body.t0_ns) % (40 * kMs) != 0);
    REQUIRE((rig.t_camera_ns - rig.body.t0_ns) % (40 * kMs) != 0);

    const Rigid3 T_world_lidar_l = rig.lidar_at(rig.t_lidar_ns).T_world_lidar;
    const Rigid3 T_world_lidar_c = rig.lidar_at(rig.t_camera_ns).T_world_lidar;
    const Rigid3 expected = T_world_lidar_c.inverse() * T_world_lidar_l;
    const Eigen::Isometry3d T = fusion::scan_motion_transform(
        traj, rig.T_body_lidar, rig.t_lidar_ns, rig.t_camera_ns);
    CHECK((T.matrix() - expected.matrix()).cwiseAbs().maxCoeff() <= 1e-12);
    // The body really moved: 8 m/s over 33 ms is 0.264 m.
    CHECK(expected.translation().norm() > 0.25);
}

TEST_CASE("compensated scan depth agrees with the camera's ray-cast depth at t_c") {
    const Scene scene = Scene::reference();
    const MovingRig rig = make_moving_rig();
    const fusion::Trajectory traj = sample_trajectory(
        rig.body, rig.body.t0_ns, rig.body.t0_ns + 400 * kMs, 40 * kMs);

    // Instantaneous scan at t_l from the LiDAR's true pose; the camera at
    // its true pose at t_c is the ground truth.
    const std::vector<LidarPoint> lidar_scan = scan(scene, rig.lidar_at(rig.t_lidar_ns));
    const std::vector<Eigen::Vector3d> raw = points_of(lidar_scan);
    std::vector<int> surfaces;
    for (const LidarPoint& lp : lidar_scan) surfaces.push_back(lp.surface);
    const PinholeCamera camera = rig.camera_at(rig.t_camera_ns);
    const Rigid3 T_camera_lidar = rig.T_camera_lidar();

    const std::vector<Eigen::Vector3d> compensated = fusion::compensate_scan(
        raw, traj, rig.T_body_lidar, rig.t_lidar_ns, rig.t_camera_ns);
    REQUIRE(compensated.size() == raw.size());

    // The body moves in the ground plane (xy translation and yaw), which
    // maps the plane z = 0 onto itself, so stale ground points still lie on
    // the ground and many of them happen to agree. Box points show the
    // motion, so they are reported separately.
    const DepthCheck before = check_depth(scene, camera, T_camera_lidar, raw, surfaces, false);
    const DepthCheck after =
        check_depth(scene, camera, T_camera_lidar, compensated, surfaces, false);
    const DepthCheck before_box =
        check_depth(scene, camera, T_camera_lidar, raw, surfaces, true);
    const DepthCheck after_box =
        check_depth(scene, camera, T_camera_lidar, compensated, surfaces, true);

    const auto report = [](const std::string& what, const DepthCheck& c) {
        std::ostringstream msg;
        msg << what << ": " << c.agree << "/" << c.projected
            << " projected points agree within 1e-9 relative depth, median |depth error| = "
            << c.median_abs_error << " m, " << c.in_front
            << " land where the camera sees farther than the point";
        MESSAGE(msg.str());
    };
    report("before compensation, all points", before);
    report("after compensation,  all points", after);
    report("before compensation, box points", before_box);
    report("after compensation,  box points", after_box);

    // After: the same criteria as the static RGB projection test. The camera
    // never sees farther than a real surface point, and most points are
    // visible with matching depth; the rest are occluded from the camera.
    CHECK(after.projected > 200);
    CHECK(after.in_front == 0);
    CHECK(after.agree > after.projected * 8 / 10);
    CHECK(after.median_abs_error <= 1e-9);
    CHECK(after_box.projected > 200);
    CHECK(after_box.in_front == 0);
    CHECK(after_box.agree > after_box.projected * 8 / 10);
    CHECK(after_box.median_abs_error <= 1e-9);

    // Before: the scan is 33 ms stale, so the depths clearly disagree.
    CHECK(before.median_abs_error > 0.05);
    CHECK(before.median_abs_error > 1e6 * std::max(after.median_abs_error, 1e-15));
    CHECK(before_box.agree < before_box.projected / 10);
    CHECK(before_box.median_abs_error > 0.1);
}

TEST_CASE("compensation is the identity for t_c == t_l and for a static trajectory") {
    const Scene scene = Scene::reference();
    const MovingRig rig = make_moving_rig();
    const std::vector<Eigen::Vector3d> raw =
        points_of(scan(scene, rig.lidar_at(rig.t_lidar_ns)));
    REQUIRE(raw.size() > 1000);

    SUBCASE("same time on a moving trajectory") {
        const fusion::Trajectory traj = sample_trajectory(
            rig.body, rig.body.t0_ns, rig.body.t0_ns + 400 * kMs, 40 * kMs);
        const auto out = fusion::compensate_scan(raw, traj, rig.T_body_lidar,
                                                 rig.t_lidar_ns, rig.t_lidar_ns);
        CHECK(out == raw);
        const Eigen::Isometry3d T = fusion::scan_motion_transform(
            traj, rig.T_body_lidar, rig.t_lidar_ns, rig.t_lidar_ns);
        CHECK((T.matrix() - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff() <= 1e-15);
    }
    SUBCASE("static trajectory") {
        MovingBody still = rig.body;
        still.velocity = Vec3::Zero();
        still.yaw_rate = 0.0;
        const fusion::Trajectory traj =
            sample_trajectory(still, still.t0_ns, still.t0_ns + 400 * kMs, 40 * kMs);
        const auto out = fusion::compensate_scan(raw, traj, rig.T_body_lidar,
                                                 rig.t_lidar_ns, rig.t_camera_ns);
        CHECK(out == raw);
        const Eigen::Isometry3d T = fusion::scan_motion_transform(
            traj, rig.T_body_lidar, rig.t_lidar_ns, rig.t_camera_ns);
        CHECK((T.matrix() - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff() <= 1e-15);
    }
}

TEST_CASE("compensation rejects times outside the trajectory") {
    const MovingRig rig = make_moving_rig();
    const fusion::Trajectory traj = sample_trajectory(
        rig.body, rig.body.t0_ns, rig.body.t0_ns + 120 * kMs, 40 * kMs);
    const std::vector<Eigen::Vector3d> pts{{1.0, 2.0, 3.0}};
    // t_c = t0 + 140 ms is past the end at t0 + 120 ms.
    CHECK_THROWS_AS(fusion::compensate_scan(pts, traj, rig.T_body_lidar, rig.t_lidar_ns,
                                            rig.t_camera_ns),
                    std::out_of_range);
    CHECK_THROWS_AS(fusion::compensate_scan(pts, traj, rig.T_body_lidar,
                                            rig.body.t0_ns - 1, rig.t_lidar_ns),
                    std::out_of_range);
    // Out of range even when both times are equal.
    CHECK_THROWS_AS(fusion::compensate_scan(pts, traj, rig.T_body_lidar, rig.body.t0_ns - 1,
                                            rig.body.t0_ns - 1),
                    std::out_of_range);
    CHECK_THROWS_AS(fusion::scan_motion_transform(traj, rig.T_body_lidar, rig.t_lidar_ns,
                                                  rig.t_camera_ns),
                    std::out_of_range);
}
