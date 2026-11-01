#include <doctest.h>

#include <cmath>

#include "support/rigid.hpp"
#include "support/scene.hpp"

using namespace fusion::testing;

namespace {

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
