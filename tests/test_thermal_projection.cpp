// LiDAR to thermal projection checked against the synthetic scene, whose
// surfaces each carry a raw thermal value.
#include <doctest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fusion/geometry.hpp>
#include <fusion/thermal.hpp>

#include "support/scene.hpp"
#include "support/sensors.hpp"

using namespace fusion::testing;

namespace {

fusion::CameraModel model_of(const PinholeCamera& cam) {
    Eigen::Matrix3d K;
    K << cam.K.fx, 0.0, cam.K.cx,
         0.0, cam.K.fy, cam.K.cy,
         0.0, 0.0, 1.0;
    return fusion::CameraModel(cam.K.width, cam.K.height, K);
}

constexpr double kMaxDistance = 100.0;
// Raw value of pixels that see nothing; no scene surface uses it.
constexpr std::uint16_t kBackground = 1234;
// Pixels within this many pixels (Chebyshev distance) of a change of surface
// in the virtual camera's surface image count as near an edge and are
// excluded from the raw value check.
constexpr int kEdgeMarginPx = 2;
// Same relative depth tolerance as the RGB projection test.
constexpr double kDepthTol = 1e-9;

struct Rig {
    std::string name;
    Scene scene;
    LidarConfig lidar;
    // Minimum share of projected points the camera sees, in tenths. The
    // random scene has 25 boxes and so more occlusion between the poses.
    std::size_t min_visible_tenths;
};

std::vector<Rig> rigs() {
    LidarConfig tilted = make_reference_lidar();
    tilted.T_world_lidar = make_rigid(rot_rpy(0.05, -0.03, 0.4), {0.3, -0.2, 1.7});
    LidarConfig shifted = make_reference_lidar();
    shifted.T_world_lidar = make_rigid(rot_rpy(0.0, 0.0, 0.2), {-0.5, 0.4, 1.9});
    return {{"reference scene, level LiDAR", Scene::reference(), make_reference_lidar(), 8},
            {"reference scene, tilted LiDAR", Scene::reference(), tilted, 8},
            {"random scene, shifted LiDAR", Scene::random(99, 25), shifted, 7}};
}

bool near_edge(const std::vector<int>& surfaces, int width, int height, int col, int row) {
    const int centre = surfaces[static_cast<std::size_t>(row * width + col)];
    for (int r = row - kEdgeMarginPx; r <= row + kEdgeMarginPx; ++r) {
        for (int c = col - kEdgeMarginPx; c <= col + kEdgeMarginPx; ++c) {
            if (c < 0 || c >= width || r < 0 || r >= height) return true;
            if (surfaces[static_cast<std::size_t>(r * width + c)] != centre) return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("synthetic thermal image holds each surface's raw value") {
    const Scene scene = Scene::reference();
    const PinholeCamera cam = make_thermal_camera();
    const auto surfaces = cam.render_surfaces(scene, kMaxDistance);
    const auto raw = cam.render_raw(scene, kMaxDistance, kBackground);
    REQUIRE(raw.size() == surfaces.size());
    std::set<int> seen;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (surfaces[i] == kNoSurface) {
            CHECK(raw[i] == kBackground);
        } else {
            CHECK(raw[i] == scene.raw_of(surfaces[i]));
            seen.insert(surfaces[i]);
        }
    }
    // The thermal camera looks at the box ahead, standing on the ground.
    CHECK(seen.count(kGroundSurface) == 1);
    CHECK(seen.count(kFirstBoxSurface) == 1);
    CHECK(scene.raw_of(kGroundSurface) == 14000);
    CHECK(scene.raw_of(kFirstBoxSurface) == 21000);
    CHECK(scene.raw_of(kFirstBoxSurface + 1) == 65535);
    CHECK(scene.raw_of(kFirstBoxSurface + 2) == 0);
    CHECK_THROWS_AS((void)scene.raw_of(kNoSurface), std::out_of_range);
}

TEST_CASE("LiDAR points read the raw value of their surface, away from edges") {
    const PinholeCamera cam = make_thermal_camera();
    const fusion::CameraModel model = model_of(cam);
    for (const Rig& rig : rigs()) {
        CAPTURE(rig.name);
        const Rigid3 T_thermal_lidar = cam.T_world_camera.inverse() * rig.lidar.T_world_lidar;
        const fusion::RawThermalImage image(cam.K.width, cam.K.height,
                                            cam.render_raw(rig.scene, kMaxDistance, kBackground));
        const auto surfaces = cam.render_surfaces(rig.scene, kMaxDistance);

        const std::vector<LidarPoint> scanned = scan(rig.scene, rig.lidar);
        std::vector<Eigen::Vector3d> points;
        for (const LidarPoint& p : scanned) points.push_back(p.p_lidar);

        const fusion::ThermalProjection proj =
            fusion::project_to_thermal(model, T_thermal_lidar, points, image);

        std::size_t checked = 0;
        std::size_t excluded_edge = 0;
        for (const fusion::ThermalSample& s : proj.samples) {
            const LidarPoint& lp = scanned[s.point_index];
            const auto& px = s.projection;
            CHECK(s.raw == image.at(px.col, px.row));
            // Only points the camera sees: its ray through the projected
            // position hits the same surface at the same depth.
            const Vec3 d_cam = cam.pixel_direction_camera(px.pixel.x(), px.pixel.y());
            const Ray ray = cam.pixel_ray_world(px.pixel.x(), px.pixel.y());
            const auto hit = rig.scene.raycast(ray.origin, ray.direction, kMaxDistance);
            REQUIRE(hit.has_value());
            if (std::abs(hit->distance * d_cam.z() - px.depth) > kDepthTol * px.depth) continue;
            if (near_edge(surfaces, cam.K.width, cam.K.height, px.col, px.row)) {
                ++excluded_edge;
                continue;
            }
            CHECK(s.raw == rig.scene.raw_of(lp.surface));
            ++checked;
        }
        CHECK(proj.samples.size() > 200);
        CHECK(checked > proj.samples.size() / 2);
        CHECK(excluded_edge < checked);
    }
}

TEST_CASE("thermal depth from the LiDAR agrees with the virtual thermal camera's depth") {
    const PinholeCamera cam = make_thermal_camera();
    const fusion::CameraModel model = model_of(cam);
    for (const Rig& rig : rigs()) {
        CAPTURE(rig.name);
        const Rigid3 T_thermal_lidar = cam.T_world_camera.inverse() * rig.lidar.T_world_lidar;
        const fusion::RawThermalImage image(cam.K.width, cam.K.height,
                                            cam.render_raw(rig.scene, kMaxDistance, kBackground));
        std::vector<Eigen::Vector3d> points;
        for (const LidarPoint& p : scan(rig.scene, rig.lidar)) points.push_back(p.p_lidar);

        const fusion::ThermalProjection proj =
            fusion::project_to_thermal(model, T_thermal_lidar, points, image);

        std::size_t visible = 0;
        std::map<std::pair<int, int>, double> nearest;
        for (const fusion::ThermalSample& s : proj.samples) {
            const auto& px = s.projection;
            const Vec3 d_cam = cam.pixel_direction_camera(px.pixel.x(), px.pixel.y());
            const Ray ray = cam.pixel_ray_world(px.pixel.x(), px.pixel.y());
            const auto hit = rig.scene.raycast(ray.origin, ray.direction, kMaxDistance);
            REQUIRE(hit.has_value());
            const double camera_depth = hit->distance * d_cam.z();
            // As for RGB: never farther than the LiDAR point, and equal when
            // the point is visible from the camera.
            CHECK(camera_depth <= px.depth + kDepthTol);
            if (std::abs(camera_depth - px.depth) <= kDepthTol * px.depth) ++visible;

            const auto key = std::make_pair(px.col, px.row);
            const auto it = nearest.find(key);
            if (it == nearest.end() || px.depth < it->second) nearest[key] = px.depth;
        }
        CHECK(proj.samples.size() > 200);
        CHECK(visible > proj.samples.size() * rig.min_visible_tenths / 10);

        // The depth map holds the nearest sample depth on each pixel.
        CHECK(proj.depth.width() == cam.K.width);
        CHECK(proj.depth.height() == cam.K.height);
        CHECK(proj.depth.size() == nearest.size());
        for (const auto& e : proj.depth.entries()) {
            const auto it = nearest.find({e.col, e.row});
            REQUIRE(it != nearest.end());
            CHECK(e.depth == it->second);
        }
    }
}

TEST_CASE("thermal projection rejects an image of the wrong size") {
    const PinholeCamera cam = make_thermal_camera();
    const fusion::CameraModel model = model_of(cam);
    const fusion::RawThermalImage image(cam.K.width - 1, cam.K.height);
    CHECK_THROWS_AS(fusion::project_to_thermal(model, Rigid3::Identity(), {}, image),
                    std::invalid_argument);
}
