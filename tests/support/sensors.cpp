#include "sensors.hpp"

#include <cmath>
#include <cstddef>
#include <random>

namespace fusion::testing {

namespace {

constexpr double kPi = 3.14159265358979323846;

double deg(double d) { return d * kPi / 180.0; }

// Standard normal samples from raw 64-bit draws via Box-Muller, so the
// sequence does not depend on the standard library's distributions.
class Gaussian {
public:
    explicit Gaussian(std::uint64_t seed) : rng_(seed) {}

    double operator()() {
        if (has_spare_) {
            has_spare_ = false;
            return spare_;
        }
        // u1 in (0, 1], u2 in [0, 1).
        const double u1 = (static_cast<double>(rng_() >> 11) + 1.0) * 0x1.0p-53;
        const double u2 = static_cast<double>(rng_() >> 11) * 0x1.0p-53;
        const double r = std::sqrt(-2.0 * std::log(u1));
        spare_ = r * std::sin(2.0 * kPi * u2);
        has_spare_ = true;
        return r * std::cos(2.0 * kPi * u2);
    }

private:
    std::mt19937_64 rng_;
    double spare_ = 0.0;
    bool has_spare_ = false;
};

}  // namespace

Mat3 rot_rpy(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
            Eigen::AngleAxisd(roll, Vec3::UnitX()))
        .toRotationMatrix();
}

Rigid3 make_rigid(const Mat3& R, const Vec3& t) {
    Rigid3 T = Rigid3::Identity();
    T.linear() = R;
    T.translation() = t;
    return T;
}

Mat3 R_body_optical() {
    Mat3 r;
    r << 0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
         0.0, -1.0, 0.0;
    return r;
}

Vec3 PinholeCamera::pixel_direction_camera(double u, double v) const {
    return Vec3{(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0}.normalized();
}

Ray PinholeCamera::pixel_ray_world(double u, double v) const {
    return {T_world_camera.translation(), T_world_camera.linear() * pixel_direction_camera(u, v)};
}

std::vector<double> PinholeCamera::render_depth(const Scene& scene, double max_distance) const {
    const std::size_t w = static_cast<std::size_t>(K.width);
    const std::size_t h = static_cast<std::size_t>(K.height);
    std::vector<double> depth(w * h, 0.0);
    for (std::size_t row = 0; row < h; ++row) {
        for (std::size_t col = 0; col < w; ++col) {
            const Vec3 d_cam =
                pixel_direction_camera(static_cast<double>(col), static_cast<double>(row));
            const Ray ray{T_world_camera.translation(), T_world_camera.linear() * d_cam};
            if (auto hit = scene.raycast(ray.origin, ray.direction, max_distance)) {
                depth[row * w + col] = hit->distance * d_cam.z();
            }
        }
    }
    return depth;
}

std::vector<int> PinholeCamera::render_surfaces(const Scene& scene, double max_distance) const {
    const std::size_t w = static_cast<std::size_t>(K.width);
    const std::size_t h = static_cast<std::size_t>(K.height);
    std::vector<int> surfaces(w * h, kNoSurface);
    for (std::size_t row = 0; row < h; ++row) {
        for (std::size_t col = 0; col < w; ++col) {
            const Ray ray =
                pixel_ray_world(static_cast<double>(col), static_cast<double>(row));
            if (auto hit = scene.raycast(ray.origin, ray.direction, max_distance)) {
                surfaces[row * w + col] = hit->surface;
            }
        }
    }
    return surfaces;
}

std::vector<std::uint16_t> PinholeCamera::render_raw(const Scene& scene, double max_distance,
                                                     std::uint16_t background) const {
    const std::vector<int> surfaces = render_surfaces(scene, max_distance);
    std::vector<std::uint16_t> raw(surfaces.size(), background);
    for (std::size_t i = 0; i < surfaces.size(); ++i) {
        if (surfaces[i] != kNoSurface) raw[i] = scene.raw_of(surfaces[i]);
    }
    return raw;
}

PinholeCamera make_rgb_camera() {
    PinholeCamera cam;
    cam.K = {500.0, 500.0, 319.5, 239.5, 640, 480};
    // Body frame at (0, 0.1, 1.2) looking along +x.
    cam.T_world_camera = make_rigid(R_body_optical(), {0.0, 0.1, 1.2});
    return cam;
}

PinholeCamera make_thermal_camera() {
    PinholeCamera cam;
    cam.K = {320.0, 320.0, 159.5, 127.5, 320, 256};
    // Body frame at (0, -0.15, 1.25), yawed 3 degrees and pitched 2 degrees
    // down (positive pitch about +y tilts +x towards -z).
    const Mat3 R_world_body = rot_rpy(0.0, deg(2.0), deg(3.0));
    cam.T_world_camera = make_rigid(R_world_body * R_body_optical(), {0.0, -0.15, 1.25});
    return cam;
}

LidarConfig make_reference_lidar() {
    LidarConfig c;
    for (int i = 0; i < 16; ++i) {
        c.elevations_rad.push_back(deg(-15.0 + 2.0 * i));
    }
    c.azimuth_step_rad = deg(0.5);
    c.min_range = 0.5;
    c.max_range = 100.0;
    c.T_world_lidar = make_rigid(Mat3::Identity(), {0.0, 0.0, 1.8});
    return c;
}

std::vector<LidarPoint> scan(const Scene& scene, const LidarConfig& config) {
    std::vector<LidarPoint> points;
    const int columns = static_cast<int>(std::lround(2.0 * kPi / config.azimuth_step_rad));
    Gaussian noise(config.seed);
    for (int col = 0; col < columns; ++col) {
        const double az = config.azimuth_step_rad * col;
        for (std::size_t ring = 0; ring < config.elevations_rad.size(); ++ring) {
            const double el = config.elevations_rad[ring];
            const Vec3 d_lidar{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                               std::sin(el)};
            const Vec3 d_world = config.T_world_lidar.linear() * d_lidar;
            // Draw for every beam, hit or not, so one beam's outcome does
            // not shift the noise applied to the others.
            const double n = config.range_noise_stddev * noise();
            auto hit = scene.raycast(config.T_world_lidar.translation(), d_world, config.max_range);
            if (!hit || hit->distance < config.min_range) continue;
            const double range = hit->distance + n;
            points.push_back(
                {range * d_lidar, range, static_cast<int>(ring), col, hit->surface});
        }
    }
    return points;
}

Vec3 MovingBody::position(std::int64_t t_ns) const {
    const double dt = static_cast<double>(t_ns - t0_ns) * 1e-9;
    return position0 + dt * velocity;
}

double MovingBody::yaw(std::int64_t t_ns) const {
    const double dt = static_cast<double>(t_ns - t0_ns) * 1e-9;
    return yaw0 + dt * yaw_rate;
}

Rigid3 MovingBody::T_world_body(std::int64_t t_ns) const {
    return make_rigid(rot_rpy(0.0, 0.0, yaw(t_ns)), position(t_ns));
}

Rigid3 MovingRig::T_camera_lidar() const { return T_body_camera.inverse() * T_body_lidar; }

LidarConfig MovingRig::lidar_at(std::int64_t t_ns) const {
    LidarConfig c = lidar;
    c.T_world_lidar = body.T_world_body(t_ns) * T_body_lidar;
    return c;
}

PinholeCamera MovingRig::camera_at(std::int64_t t_ns) const {
    PinholeCamera cam;
    cam.K = camera_K;
    cam.T_world_camera = body.T_world_body(t_ns) * T_body_camera;
    return cam;
}

MovingRig make_moving_rig() {
    MovingRig rig;
    rig.body.t0_ns = 2'000'000'000;
    rig.body.position0 = {-1.0, 0.0, 0.0};
    rig.body.yaw0 = 0.02;
    rig.body.velocity = {std::sqrt(64.0 - 0.09), 0.3, 0.0};  // 8 m/s
    rig.body.yaw_rate = 0.6;
    rig.T_body_lidar = make_rigid(rot_rpy(0.01, -0.02, 0.03), {0.0, 0.0, 1.8});
    rig.T_body_camera = make_rigid(R_body_optical(), {0.2, 0.1, 1.2});
    rig.lidar = make_reference_lidar();
    rig.lidar.T_world_lidar = Rigid3::Identity();
    rig.camera_K = make_rgb_camera().K;
    rig.t_lidar_ns = rig.body.t0_ns + 107'000'000;
    rig.t_camera_ns = rig.t_lidar_ns + 33'000'000;
    return rig;
}

}  // namespace fusion::testing
