// Deterministic synthetic scene used as ground truth in tests.
// Test-only: not part of the fusion library's public API.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "rigid.hpp"

namespace fusion::testing {

// Axis-aligned box in the world frame, given by its min and max corners.
struct Box {
    Vec3 min;
    Vec3 max;
};

struct Hit {
    double distance = 0.0;  // along the unit ray direction, in metres
    Vec3 point;             // world frame
    Vec3 normal;            // world frame, unit, facing the ray origin side
};

// Ray hits closer than this are ignored, so a ray does not re-hit the surface
// it starts on.
inline constexpr double kMinHitDistance = 1e-9;

class Scene {
public:
    // Ground plane z = ground_z (world frame), facing +z.
    explicit Scene(double ground_z = 0.0) : ground_z_(ground_z) {}

    void add_box(const Box& box) { boxes_.push_back(box); }
    const std::vector<Box>& boxes() const { return boxes_; }
    double ground_z() const { return ground_z_; }

    // Nearest hit along origin + t * direction for t in (kMinHitDistance, max_distance].
    // `direction` must be unit length; both are in the world frame.
    std::optional<Hit> raycast(const Vec3& origin, const Vec3& direction,
                               double max_distance) const;

    // True if `p` (world frame) lies within `tol` of the ground plane or of
    // the surface of any box.
    bool on_surface(const Vec3& p, double tol) const;

    // Fixed reference scene: a ground plane at z = 0 and a few boxes.
    static Scene reference();

    // Ground plane at z = 0 plus `box_count` boxes with sizes and positions
    // drawn from a generator seeded with `seed`. Same seed, same scene.
    static Scene random(std::uint64_t seed, int box_count);

private:
    double ground_z_;
    std::vector<Box> boxes_;
};

std::optional<Hit> raycast_box(const Box& box, const Vec3& origin, const Vec3& direction,
                               double max_distance);

}  // namespace fusion::testing
