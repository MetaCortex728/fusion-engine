// Projection checked against the synthetic scene and its virtual sensors.
#include <doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <fusion/geometry.hpp>

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

struct Rig {
    std::string name;
    PinholeCamera camera;
    LidarConfig lidar;
};

std::vector<Rig> rigs() {
    LidarConfig tilted = make_reference_lidar();
    tilted.T_world_lidar = make_rigid(rot_rpy(0.05, -0.03, 0.4), {0.3, -0.2, 1.7});
    return {{"rgb, level LiDAR", make_rgb_camera(), make_reference_lidar()},
            {"thermal, level LiDAR", make_thermal_camera(), make_reference_lidar()},
            {"thermal, tilted LiDAR", make_thermal_camera(), tilted}};
}

}  // namespace

TEST_CASE("surface points seen by the virtual camera project back to their pixel within 1e-6 px") {
    const Scene scene = Scene::reference();
    for (const Rig& rig : rigs()) {
        CAPTURE(rig.name);
        const PinholeCamera& cam = rig.camera;
        const fusion::CameraModel model = model_of(cam);
        const Rigid3 T_camera_lidar = cam.T_world_camera.inverse() * rig.lidar.T_world_lidar;
        const Rigid3 T_lidar_camera = T_camera_lidar.inverse();

        // Ground truth is defined where the camera ray hits the scene.
        const auto depth = cam.render_depth(scene, 100.0);
        const std::size_t w = static_cast<std::size_t>(cam.K.width);
        std::size_t checked = 0;
        double worst_px = 0.0;
        double worst_depth = 0.0;
        for (int row = 0; row < cam.K.height; row += 3) {
            for (int col = 0; col < cam.K.width; col += 3) {
                const double d =
                    depth[static_cast<std::size_t>(row) * w + static_cast<std::size_t>(col)];
                if (d <= 0.0) continue;
                const Vec3 p_camera = d * Vec3{(col - cam.K.cx) / cam.K.fx,
                                               (row - cam.K.cy) / cam.K.fy, 1.0};
                const Vec3 p_lidar = T_lidar_camera * p_camera;
                const auto proj = fusion::project_point(model, T_camera_lidar, p_lidar);
                REQUIRE(proj.has_value());
                worst_px = std::max(worst_px, (proj->pixel - Eigen::Vector2d(col, row)).norm());
                worst_depth = std::max(worst_depth, std::abs(proj->depth - d) / d);
                CHECK(proj->col == col);
                CHECK(proj->row == row);
                ++checked;
            }
        }
        CHECK(checked > 1000);
        CHECK(worst_px <= 1e-6);
        CHECK(worst_depth <= 1e-12);
    }
}

TEST_CASE("virtual LiDAR points project to pixels whose virtual-camera depth agrees") {
    const Scene scene = Scene::reference();
    for (const Rig& rig : rigs()) {
        CAPTURE(rig.name);
        const PinholeCamera& cam = rig.camera;
        const fusion::CameraModel model = model_of(cam);
        const Rigid3 T_camera_lidar = cam.T_world_camera.inverse() * rig.lidar.T_world_lidar;

        std::size_t projected = 0;
        std::size_t visible = 0;
        for (const LidarPoint& lp : scan(scene, rig.lidar)) {
            const auto proj = fusion::project_point(model, T_camera_lidar, lp.p_lidar);
            if (!proj) continue;
            ++projected;
            // Camera ray through the projected sub-pixel position.
            const Vec3 d_cam = cam.pixel_direction_camera(proj->pixel.x(), proj->pixel.y());
            const Ray ray = cam.pixel_ray_world(proj->pixel.x(), proj->pixel.y());
            const auto hit = scene.raycast(ray.origin, ray.direction, 100.0);
            REQUIRE(hit.has_value());
            const double camera_depth = hit->distance * d_cam.z();
            // The camera sees the nearest surface on that ray, so it is never
            // farther than a real surface point. If it is equally far, the
            // point is visible from the camera and the depths must agree.
            CHECK(camera_depth <= proj->depth + 1e-9);
            if (std::abs(camera_depth - proj->depth) <= 1e-9 * proj->depth) ++visible;
        }
        CHECK(projected > 200);
        // LiDAR and camera sit at different poses, so a few points are
        // occluded from the camera, but most are visible.
        CHECK(visible > projected * 8 / 10);
    }
}
