#include <doctest.h>

#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include <fusion/depth_map.hpp>
#include <fusion/geometry.hpp>

#include "support/scene.hpp"
#include "support/sensors.hpp"

using fusion::SparseDepthMap;
using namespace fusion::testing;

namespace {

fusion::CameraModel model_of(const PinholeCamera& cam) {
    Eigen::Matrix3d K;
    K << cam.K.fx, 0.0, cam.K.cx,
         0.0, cam.K.fy, cam.K.cy,
         0.0, 0.0, 1.0;
    return fusion::CameraModel(cam.K.width, cam.K.height, K);
}

}  // namespace

TEST_CASE("depth buffer keeps the nearest depth per pixel") {
    SparseDepthMap map(4, 3);
    CHECK(map.empty());
    CHECK_FALSE(map.at(1, 2).has_value());

    CHECK(map.insert(1, 2, 5.0));
    CHECK_FALSE(map.insert(1, 2, 7.0));  // farther: ignored
    CHECK_FALSE(map.insert(1, 2, 5.0));  // tie: first one stays
    CHECK(map.insert(1, 2, 3.0));        // nearer: replaces
    REQUIRE(map.at(1, 2).has_value());
    CHECK(*map.at(1, 2) == 3.0);
    CHECK(map.size() == 1);

    CHECK(map.insert(3, 0, 2.5));
    CHECK(map.insert(0, 1, 9.0));
    const auto entries = map.entries();
    REQUIRE(entries.size() == 3);
    // Row-major order.
    CHECK((entries[0].col == 3 && entries[0].row == 0 && entries[0].depth == 2.5));
    CHECK((entries[1].col == 0 && entries[1].row == 1 && entries[1].depth == 9.0));
    CHECK((entries[2].col == 1 && entries[2].row == 2 && entries[2].depth == 3.0));
}

TEST_CASE("depth buffer rejects invalid pixels and depths") {
    SparseDepthMap map(4, 3);
    CHECK_FALSE(map.insert(-1, 0, 1.0));
    CHECK_FALSE(map.insert(4, 0, 1.0));
    CHECK_FALSE(map.insert(0, 3, 1.0));
    CHECK_FALSE(map.insert(0, 0, 0.0));
    CHECK_FALSE(map.insert(0, 0, -1.0));
    CHECK_FALSE(map.insert(0, 0, std::numeric_limits<double>::quiet_NaN()));
    CHECK_FALSE(map.insert(0, 0, std::numeric_limits<double>::infinity()));
    CHECK(map.empty());
    CHECK_FALSE(map.at(4, 0).has_value());
    CHECK_THROWS_AS(SparseDepthMap(0, 3), std::invalid_argument);
}

TEST_CASE("a point behind a box does not replace the nearer point on the same pixel") {
    const Scene scene = Scene::reference();
    const PinholeCamera cam = make_rgb_camera();
    const fusion::CameraModel model = model_of(cam);
    const LidarConfig lidar = make_reference_lidar();
    const Rigid3 T_camera_lidar = cam.T_world_camera.inverse() * lidar.T_world_lidar;
    const Rigid3 T_lidar_world = lidar.T_world_lidar.inverse();

    // Camera ray through pixel (400, 250) hits the front (x = 4) face of the
    // box ahead, which spans x in [4, 6].
    const Ray ray = cam.pixel_ray_world(400.0, 250.0);
    const auto hit = scene.raycast(ray.origin, ray.direction, 100.0);
    REQUIRE(hit.has_value());
    REQUIRE(hit->point.x() == doctest::Approx(4.0));
    const Vec3 near_world = hit->point;
    const Vec3 far_world = ray.origin + (hit->distance + 3.0) * ray.direction;
    REQUIRE(far_world.x() > 6.0);  // behind the box, hidden from the camera

    const Vec3 near_lidar = T_lidar_world * near_world;
    const Vec3 far_lidar = T_lidar_world * far_world;
    const auto near_proj = fusion::project_point(model, T_camera_lidar, near_lidar);
    const auto far_proj = fusion::project_point(model, T_camera_lidar, far_lidar);
    REQUIRE(near_proj.has_value());
    REQUIRE(far_proj.has_value());
    REQUIRE(near_proj->col == 400);
    REQUIRE(near_proj->row == 250);
    REQUIRE(far_proj->col == 400);
    REQUIRE(far_proj->row == 250);
    REQUIRE(far_proj->depth > near_proj->depth);

    for (const auto& points : {std::vector<Eigen::Vector3d>{far_lidar, near_lidar},
                               std::vector<Eigen::Vector3d>{near_lidar, far_lidar}}) {
        const SparseDepthMap map = fusion::project_to_depth_map(model, T_camera_lidar, points);
        CHECK(map.size() == 1);
        REQUIRE(map.at(400, 250).has_value());
        CHECK(*map.at(400, 250) == near_proj->depth);
    }
}

TEST_CASE("depth map from a LiDAR scan holds the minimum depth of the points on each pixel") {
    const Scene scene = Scene::random(99, 25);
    const PinholeCamera cam = make_thermal_camera();
    const fusion::CameraModel model = model_of(cam);
    LidarConfig lidar = make_reference_lidar();
    lidar.T_world_lidar = make_rigid(rot_rpy(0.0, 0.0, 0.2), {-0.5, 0.4, 1.9});
    const Rigid3 T_camera_lidar = cam.T_world_camera.inverse() * lidar.T_world_lidar;

    std::vector<Eigen::Vector3d> points;
    for (const LidarPoint& p : scan(scene, lidar)) points.push_back(p.p_lidar);
    // Add points hidden behind each scan point along the camera's line of
    // sight, so every pixel with a hit sees a nearer and a farther candidate.
    const Rigid3 T_lidar_camera = T_camera_lidar.inverse();
    const std::size_t scanned = points.size();
    for (std::size_t i = 0; i < scanned; ++i) {
        points.push_back(T_lidar_camera * (1.5 * (T_camera_lidar * points[i])));
    }

    std::map<std::pair<int, int>, double> expected;
    for (const Eigen::Vector3d& p : points) {
        if (const auto proj = fusion::project_point(model, T_camera_lidar, p)) {
            const auto key = std::make_pair(proj->col, proj->row);
            const auto it = expected.find(key);
            if (it == expected.end() || proj->depth < it->second) expected[key] = proj->depth;
        }
    }
    REQUIRE(expected.size() > 100);

    const SparseDepthMap map = fusion::project_to_depth_map(model, T_camera_lidar, points);
    CHECK(map.size() == expected.size());
    for (const auto& e : map.entries()) {
        const auto it = expected.find({e.col, e.row});
        REQUIRE(it != expected.end());
        CHECK(e.depth == it->second);
    }
}
