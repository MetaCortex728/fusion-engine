#include <doctest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>

#include <fusion/geometry.hpp>

using fusion::CameraModel;
using fusion::Distortion;
using fusion::Projection;

namespace {

// fx = 400, fy = 380, cx = 320, cy = 240, 640 x 480.
Eigen::Matrix3d test_K() {
    Eigen::Matrix3d K;
    K << 400.0, 0.0, 320.0,
         0.0, 380.0, 240.0,
         0.0, 0.0, 1.0;
    return K;
}

// Typical mild barrel distortion. Its radial part is monotonic for every
// radius (1 - 0.84 s + 0.35 s^2 has no real root).
Distortion barrel() { return {-0.28, 0.07, 2e-4, -1e-4, 0.0}; }

}  // namespace

TEST_CASE("projection of known points matches hand computation") {
    SUBCASE("pinhole through a rotated and translated T_camera_lidar") {
        const CameraModel cam(640, 480, test_K());
        // LiDAR body frame (x forward, y left, z up) to camera optical frame
        // (x right, y down, z forward): camera = (-y, -z, x) + (0.1, -0.6, 0).
        Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
        T_camera_lidar.linear() << 0.0, -1.0, 0.0,
                                   0.0, 0.0, -1.0,
                                   1.0, 0.0, 0.0;
        T_camera_lidar.translation() << 0.1, -0.6, 0.0;
        // p_lidar = (10, 1, -0.5) -> p_camera = (-1 + 0.1, 0.5 - 0.6, 10) = (-0.9, -0.1, 10)
        // x_n = -0.09, y_n = -0.01
        // u = 400 * -0.09 + 320 = 284, v = 380 * -0.01 + 240 = 236.2
        const auto p = fusion::project_point(cam, T_camera_lidar, {10.0, 1.0, -0.5});
        REQUIRE(p.has_value());
        CHECK(p->pixel.x() == doctest::Approx(284.0).epsilon(1e-14));
        CHECK(p->pixel.y() == doctest::Approx(236.2).epsilon(1e-14));
        CHECK(p->depth == 10.0);
        CHECK(p->col == 284);
        CHECK(p->row == 236);
    }

    SUBCASE("radial-tangential distortion with all five coefficients") {
        const CameraModel cam(640, 480, test_K(), {-0.2, 0.05, 0.001, -0.002, 0.01});
        // p_camera = (1, 2, 10): x = 0.1, y = 0.2, r2 = 0.05
        // radial = 1 - 0.2 * 0.05 + 0.05 * 0.0025 + 0.01 * 0.000125 = 0.99012625
        // x_d = 0.1 * radial + 2 * 0.001 * 0.02 - 0.002 * (0.05 + 0.02) = 0.098912625
        // y_d = 0.2 * radial + 0.001 * (0.05 + 0.08) - 2 * 0.002 * 0.02 = 0.19807525
        // u = 400 * x_d + 320 = 359.56505, v = 380 * y_d + 240 = 315.268595
        const auto p = cam.project({1.0, 2.0, 10.0});
        REQUIRE(p.has_value());
        CHECK(p->pixel.x() == doctest::Approx(359.56505).epsilon(1e-13));
        CHECK(p->pixel.y() == doctest::Approx(315.268595).epsilon(1e-13));
        CHECK(p->depth == 10.0);
        CHECK(p->col == 360);
        CHECK(p->row == 315);
    }

    SUBCASE("skew enters u through K(0, 1)") {
        Eigen::Matrix3d K = test_K();
        K(0, 1) = 2.0;
        const CameraModel cam(640, 480, K);
        // x_n = 0.1, y_n = 0.2: u = 400 * 0.1 + 2 * 0.2 + 320 = 360.4, v = 380 * 0.2 + 240 = 316
        const auto p = cam.project({1.0, 2.0, 10.0});
        REQUIRE(p.has_value());
        CHECK(p->pixel.x() == doctest::Approx(360.4).epsilon(1e-14));
        CHECK(p->pixel.y() == doctest::Approx(316.0).epsilon(1e-14));
        CHECK((cam.from_pixel(p->pixel) - Eigen::Vector2d(0.1, 0.2)).norm() < 1e-15);
    }
}

TEST_CASE("zero distortion matches the plain pinhole formula") {
    const Eigen::Matrix3d K = test_K();
    const CameraModel cam(640, 480, K, Distortion{});
    std::mt19937_64 rng(2024);
    const auto uniform = [&rng](double lo, double hi) {
        return lo + (hi - lo) * static_cast<double>(rng() >> 11) * 0x1.0p-53;
    };
    int projected = 0;
    for (int i = 0; i < 2000; ++i) {
        const Eigen::Vector3d p(uniform(-8.0, 8.0), uniform(-6.0, 6.0), uniform(0.5, 20.0));
        const double u = K(0, 0) * p.x() / p.z() + K(0, 2);
        const double v = K(1, 1) * p.y() / p.z() + K(1, 2);
        const bool inside = u >= -0.5 && u < 639.5 && v >= -0.5 && v < 479.5;
        const auto proj = cam.project(p);
        REQUIRE(proj.has_value() == inside);
        if (!proj) continue;
        ++projected;
        CHECK(std::abs(proj->pixel.x() - u) <= 1e-12);
        CHECK(std::abs(proj->pixel.y() - v) <= 1e-12);
        CHECK(proj->depth == p.z());
    }
    CHECK(projected > 500);
    CHECK(cam.max_normalized_radius() == std::numeric_limits<double>::infinity());
}

TEST_CASE("distort and undistort round trip within 1e-9 px") {
    const CameraModel cam(640, 480, test_K(), barrel());

    SUBCASE("pixel -> undistort -> distort -> pixel over the whole image") {
        for (double v = -0.5; v < 479.5; v += 7.25) {
            for (double u = -0.5; u < 639.5; u += 7.25) {
                const Eigen::Vector2d uv(u, v);
                const auto n = cam.undistort(cam.from_pixel(uv));
                REQUIRE(n.has_value());
                const Eigen::Vector2d back = cam.to_pixel(cam.distort(*n));
                CHECK((back - uv).norm() <= 1e-9);
            }
        }
    }

    SUBCASE("normalized -> distort -> undistort -> normalized") {
        for (double y = -1.0; y <= 1.0; y += 0.05) {
            for (double x = -1.2; x <= 1.2; x += 0.05) {
                const Eigen::Vector2d n(x, y);
                const auto back = cam.undistort(cam.distort(n));
                REQUIRE(back.has_value());
                // 1e-12 in normalized units is 4e-10 px at fx = 400.
                CHECK((*back - n).norm() <= 1e-12);
            }
        }
    }
}

TEST_CASE("projection rejects points behind the camera and outside the image") {
    const CameraModel cam(640, 480, test_K());

    SUBCASE("behind or on the camera plane") {
        // Would land on the principal point if the sign of z were ignored.
        CHECK_FALSE(cam.project({0.0, 0.0, -5.0}).has_value());
        CHECK_FALSE(cam.project({0.0, 0.0, 0.0}).has_value());
        CHECK_FALSE(cam.project({0.1, 0.1, -1e-12}).has_value());
        CHECK(cam.project({0.0, 0.0, 5.0}).has_value());
    }

    SUBCASE("image edges are half-open at pixel centre +- 0.5") {
        // u = 400 * x / 10 + 320, so x = (u - 320) / 40.
        const auto at_u = [&cam](double u) {
            return cam.project({(u - 320.0) / 40.0, 0.0, 10.0});
        };
        REQUIRE(at_u(-0.4).has_value());
        CHECK(at_u(-0.4)->col == 0);
        REQUIRE(at_u(639.4).has_value());
        CHECK(at_u(639.4)->col == 639);
        CHECK_FALSE(at_u(-0.6).has_value());
        CHECK_FALSE(at_u(639.6).has_value());

        // v = 380 * y / 10 + 240, so y = (v - 240) / 38.
        const auto at_v = [&cam](double v) {
            return cam.project({0.0, (v - 240.0) / 38.0, 10.0});
        };
        REQUIRE(at_v(479.4).has_value());
        CHECK(at_v(479.4)->row == 479);
        CHECK_FALSE(at_v(-0.6).has_value());
        CHECK_FALSE(at_v(479.6).has_value());
    }

    SUBCASE("non-finite points") {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        CHECK_FALSE(cam.project({nan, 0.0, 1.0}).has_value());
        CHECK_FALSE(cam.project({0.0, 0.0, nan}).has_value());
        CHECK_FALSE(cam.project({inf, 0.0, 1.0}).has_value());
    }
}

TEST_CASE("strong radial distortion does not fold far off-axis points into the image") {
    // k1 = -0.5: d(r * radial)/dr = 1 - 1.5 r^2, monotonic up to r = sqrt(2/3).
    const CameraModel cam(640, 480, test_K(), {-0.5, 0.0, 0.0, 0.0, 0.0});
    CHECK(cam.max_normalized_radius() == doctest::Approx(std::sqrt(2.0 / 3.0)).epsilon(1e-12));

    // x_n = 1.5 (56 degrees off axis): x_d = 1.5 * (1 - 0.5 * 2.25) = -0.1875,
    // which would land at u = 245, inside the image, on the wrong side.
    const Eigen::Vector2d folded = cam.to_pixel(cam.distort({1.5, 0.0}));
    CHECK(folded.x() == doctest::Approx(245.0));
    CHECK_FALSE(cam.project({15.0, 0.0, 10.0}).has_value());

    // Inside the monotonic region the point is kept.
    CHECK(cam.project({5.0, 0.0, 10.0}).has_value());
}

TEST_CASE("camera model rejects invalid parameters") {
    CHECK_THROWS_AS(CameraModel(0, 480, test_K()), std::invalid_argument);
    CHECK_THROWS_AS(CameraModel(640, -1, test_K()), std::invalid_argument);
    Eigen::Matrix3d K = test_K();
    K(0, 0) = 0.0;
    CHECK_THROWS_AS(CameraModel(640, 480, K), std::invalid_argument);
    K = test_K();
    K(2, 2) = 2.0;
    CHECK_THROWS_AS(CameraModel(640, 480, K), std::invalid_argument);
    Distortion d;
    d.k2 = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(CameraModel(640, 480, test_K(), d), std::invalid_argument);
}
