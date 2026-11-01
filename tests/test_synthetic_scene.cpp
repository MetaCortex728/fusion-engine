#include <doctest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "support/rigid.hpp"
#include "support/scene.hpp"
#include "support/sensors.hpp"

using namespace fusion::testing;

namespace {

bool same_bits(const std::vector<LidarPoint>& a, const std::vector<LidarPoint>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].p_lidar.x != b[i].p_lidar.x || a[i].p_lidar.y != b[i].p_lidar.y ||
            a[i].p_lidar.z != b[i].p_lidar.z || a[i].range != b[i].range ||
            a[i].ring != b[i].ring || a[i].column != b[i].column) {
            return false;
        }
    }
    return true;
}

void check_points_on_surfaces(const Scene& scene, const LidarConfig& lidar) {
    const auto points = scan(scene, lidar);
    REQUIRE(!points.empty());
    int off_ground = 0;
    for (const LidarPoint& p : points) {
        const Vec3 p_world = lidar.T_world_lidar * p.p_lidar;
        CHECK(scene.on_surface(p_world, 1e-9));
        if (std::abs(p_world.z - scene.ground_z()) > 1e-6) ++off_ground;
    }
    // Some points must land on boxes, not only on the ground.
    CHECK(off_ground > 0);
}

}  // namespace

TEST_CASE("ray hits a known box face at the hand-computed distance") {
    const Scene scene = Scene::reference();

    SUBCASE("straight along +x hits the x = 4 face of the box ahead") {
        const auto hit = scene.raycast({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, 100.0);
        REQUIRE(hit.has_value());
        CHECK(hit->distance == doctest::Approx(4.0).epsilon(1e-15));
        CHECK(hit->point.x == doctest::Approx(4.0));
        CHECK(hit->normal.x == -1.0);
    }

    SUBCASE("oblique ray hits the same face at sqrt(4^2 + 0.5^2)") {
        // Direction (4, 0.5, 0) reaches x = 4 at y = 0.5, inside the face.
        const Vec3 dir = normalized({4.0, 0.5, 0.0});
        const auto hit = scene.raycast({0.0, 0.0, 1.0}, dir, 100.0);
        REQUIRE(hit.has_value());
        CHECK(hit->distance == doctest::Approx(std::sqrt(16.25)).epsilon(1e-14));
        CHECK(hit->point.y == doctest::Approx(0.5));
    }

    SUBCASE("downward ray hits the ground plane before the box") {
        // From height 1.8 at 45 degrees down: ground at x = 1.8, before x = 4.
        const Vec3 dir = normalized({1.0, 0.0, -1.0});
        const auto hit = scene.raycast({0.0, 0.0, 1.8}, dir, 100.0);
        REQUIRE(hit.has_value());
        CHECK(hit->distance == doctest::Approx(1.8 * std::sqrt(2.0)).epsilon(1e-14));
        CHECK(hit->point.z == 0.0);
        CHECK(hit->normal.z == 1.0);
    }

    SUBCASE("ray above every box misses, and max distance is respected") {
        CHECK_FALSE(scene.raycast({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, 100.0).has_value());
        CHECK_FALSE(scene.raycast({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, 3.9).has_value());
    }
}

TEST_CASE("rigid transform round-trips with T_target_source naming") {
    const Rigid3 T_world_lidar{rot_rpy(0.1, -0.2, 0.7), {1.0, 2.0, 3.0}};
    const Vec3 p_lidar{0.5, -1.5, 2.0};
    const Vec3 p_back = T_world_lidar.inverse() * (T_world_lidar * p_lidar);
    CHECK(norm(p_back - p_lidar) < 1e-12);
}

TEST_CASE("RGB and thermal cameras sit at different poses and see the box") {
    const Scene scene = Scene::reference();
    const PinholeCamera rgb = make_rgb_camera();
    const PinholeCamera thermal = make_thermal_camera();
    CHECK(norm(rgb.T_world_camera.t - thermal.T_world_camera.t) > 0.1);

    // RGB optical axis points along world +x from (0, 0.1, 1.2): x = 4 face at 4 m.
    const Ray axis = rgb.pixel_ray_world(rgb.K.cx, rgb.K.cy);
    CHECK(axis.direction.x == doctest::Approx(1.0));
    const auto hit = scene.raycast(axis.origin, axis.direction, 100.0);
    REQUIRE(hit.has_value());
    CHECK(hit->distance == doctest::Approx(4.0));

    const auto depth = rgb.render_depth(scene, 100.0);
    REQUIRE(depth.size() == 640u * 480u);
    // Pixel (320, 240) is half a pixel from the principal point; still on the x = 4 face.
    CHECK(depth[240u * 640u + 320u] == doctest::Approx(4.0));

    const Ray t_axis = thermal.pixel_ray_world(thermal.K.cx, thermal.K.cy);
    const auto t_hit = scene.raycast(t_axis.origin, t_axis.direction, 100.0);
    REQUIRE(t_hit.has_value());
    CHECK(t_hit->normal.x == -1.0);
    CHECK(t_hit->point.x == doctest::Approx(4.0));
}

TEST_CASE("LiDAR points mapped back to the world lie on scene surfaces within 1e-9 m") {
    LidarConfig lidar = make_reference_lidar();

    SUBCASE("reference scene, level mount") {
        check_points_on_surfaces(Scene::reference(), lidar);
    }

    SUBCASE("reference scene, tilted and offset mount") {
        lidar.T_world_lidar = {rot_rpy(0.05, -0.03, 0.4), {0.3, -0.2, 1.7}};
        check_points_on_surfaces(Scene::reference(), lidar);
    }

    SUBCASE("seeded random scene") {
        check_points_on_surfaces(Scene::random(7, 12), lidar);
    }
}

TEST_CASE("same seed gives identical LiDAR points") {
    const Scene scene = Scene::random(1234, 10);
    LidarConfig lidar = make_reference_lidar();
    lidar.range_noise_stddev = 0.02;
    lidar.seed = 42;

    const auto first = scan(scene, lidar);
    const auto second = scan(Scene::random(1234, 10), lidar);
    REQUIRE(!first.empty());
    CHECK(same_bits(first, second));

    lidar.seed = 43;
    CHECK_FALSE(same_bits(first, scan(scene, lidar)));
}
