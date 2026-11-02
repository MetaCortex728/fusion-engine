// Deterministic synthetic scene used as ground truth in tests.
// Test-only: not part of the fusion library's public API.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace fusion::testing {

using Vec3 = Eigen::Vector3d;

// Axis-aligned box in the world frame, given by its min and max corners.
struct Box {
    Vec3 min;
    Vec3 max;
};

// Surface ids: kGroundSurface for the ground plane, kFirstBoxSurface + i for
// box i, kNoSurface for nothing.
inline constexpr int kNoSurface = -1;
inline constexpr int kGroundSurface = 0;
inline constexpr int kFirstBoxSurface = 1;

struct Hit {
    double distance = 0.0;  // along the unit ray direction, in metres
    Vec3 point;             // world frame
    Vec3 normal;            // world frame, unit, facing the ray origin side
    int surface = kNoSurface;  // set by Scene::raycast; raycast_box leaves kNoSurface
};

// Ray hits closer than this are ignored, so a ray does not re-hit the surface
// it starts on.
inline constexpr double kMinHitDistance = 1e-9;

class Scene {
public:
    // Ground plane z = ground_z (world frame), facing +z, with thermal raw
    // value `ground_raw`.
    explicit Scene(double ground_z = 0.0, std::uint16_t ground_raw = kDefaultGroundRaw)
        : ground_z_(ground_z), ground_raw_(ground_raw) {}

    // Every face of a box shares the box's thermal raw value.
    void add_box(const Box& box, std::uint16_t raw = kDefaultBoxRaw) {
        boxes_.push_back(box);
        box_raw_.push_back(raw);
    }
    const std::vector<Box>& boxes() const { return boxes_; }
    double ground_z() const { return ground_z_; }

    // Thermal raw value of a surface id. Throws std::out_of_range for
    // kNoSurface or an unknown id.
    std::uint16_t raw_of(int surface) const;

    static constexpr std::uint16_t kDefaultGroundRaw = 14000;
    static constexpr std::uint16_t kDefaultBoxRaw = 20000;

    // Nearest hit along origin + t * direction for t in (kMinHitDistance, max_distance].
    // `direction` must be unit length; both are in the world frame.
    std::optional<Hit> raycast(const Vec3& origin, const Vec3& direction,
                               double max_distance) const;

    // True if `p` (world frame) lies within `tol` of the ground plane or of
    // the surface of any box.
    bool on_surface(const Vec3& p, double tol) const;

    // Fixed reference scene: a ground plane at z = 0 and a few boxes, each
    // surface with a distinct raw value, including 0 and 65535.
    static Scene reference();

    // Ground plane at z = 0 plus `box_count` boxes with sizes and positions
    // drawn from a generator seeded with `seed`. Same seed, same scene.
    // Box i has raw value 15000 + 1000 * i (wrapping modulo 2^16), so the
    // geometry does not depend on the raw values.
    static Scene random(std::uint64_t seed, int box_count);

private:
    double ground_z_;
    std::uint16_t ground_raw_;
    std::vector<Box> boxes_;
    std::vector<std::uint16_t> box_raw_;
};

std::optional<Hit> raycast_box(const Box& box, const Vec3& origin, const Vec3& direction,
                               double max_distance);

}  // namespace fusion::testing
