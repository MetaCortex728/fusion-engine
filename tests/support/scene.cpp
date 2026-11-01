#include "scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace fusion::testing {

namespace {

double component(const Vec3& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

Vec3 axis_normal(int axis, double sign) {
    Vec3 n;
    if (axis == 0) n.x = sign;
    if (axis == 1) n.y = sign;
    if (axis == 2) n.z = sign;
    return n;
}

// Uniform double in [lo, hi) from raw 64-bit draws. Unlike
// std::uniform_real_distribution, this gives the same values with every
// standard library.
double uniform(std::mt19937_64& rng, double lo, double hi) {
    const double u = static_cast<double>(rng() >> 11) * 0x1.0p-53;
    return lo + (hi - lo) * u;
}

}  // namespace

std::optional<Hit> raycast_box(const Box& box, const Vec3& origin, const Vec3& direction,
                               double max_distance) {
    // Slab method, tracking which face bounds the entry and the exit.
    double t_near = -std::numeric_limits<double>::infinity();
    double t_far = std::numeric_limits<double>::infinity();
    int near_axis = -1, far_axis = -1;
    double near_sign = 0.0, far_sign = 0.0;

    for (int axis = 0; axis < 3; ++axis) {
        const double o = component(origin, axis);
        const double d = component(direction, axis);
        const double lo = component(box.min, axis);
        const double hi = component(box.max, axis);
        if (d == 0.0) {
            if (o < lo || o > hi) return std::nullopt;
            continue;
        }
        double t0 = (lo - o) / d;  // crossing the min face
        double t1 = (hi - o) / d;  // crossing the max face
        double s0 = -1.0, s1 = 1.0;
        if (t0 > t1) {
            std::swap(t0, t1);
            std::swap(s0, s1);
        }
        if (t0 > t_near) { t_near = t0; near_axis = axis; near_sign = s0; }
        if (t1 < t_far) { t_far = t1; far_axis = axis; far_sign = s1; }
        if (t_near > t_far) return std::nullopt;
    }

    double t = 0.0;
    Vec3 normal;
    if (t_near > kMinHitDistance) {
        t = t_near;
        normal = axis_normal(near_axis, near_sign);
    } else if (t_far > kMinHitDistance) {
        // Origin inside the box: hit the exit face, normal facing inwards.
        t = t_far;
        normal = axis_normal(far_axis, -far_sign);
    } else {
        return std::nullopt;
    }
    if (t > max_distance) return std::nullopt;
    return Hit{t, origin + t * direction, normal};
}

std::optional<Hit> Scene::raycast(const Vec3& origin, const Vec3& direction,
                                  double max_distance) const {
    std::optional<Hit> best;
    if (direction.z != 0.0) {
        const double t = (ground_z_ - origin.z) / direction.z;
        if (t > kMinHitDistance && t <= max_distance) {
            Vec3 point = origin + t * direction;
            point.z = ground_z_;  // exact on the plane
            const double side = origin.z >= ground_z_ ? 1.0 : -1.0;
            best = Hit{t, point, Vec3{0.0, 0.0, side}};
        }
    }
    for (const Box& box : boxes_) {
        const double limit = best ? best->distance : max_distance;
        if (auto hit = raycast_box(box, origin, direction, limit)) {
            if (!best || hit->distance < best->distance) best = hit;
        }
    }
    return best;
}

bool Scene::on_surface(const Vec3& p, double tol) const {
    if (std::abs(p.z - ground_z_) <= tol) return true;
    for (const Box& b : boxes_) {
        const bool inside_expanded =
            p.x >= b.min.x - tol && p.x <= b.max.x + tol &&
            p.y >= b.min.y - tol && p.y <= b.max.y + tol &&
            p.z >= b.min.z - tol && p.z <= b.max.z + tol;
        if (!inside_expanded) continue;
        for (int axis = 0; axis < 3; ++axis) {
            const double v = component(p, axis);
            if (std::abs(v - component(b.min, axis)) <= tol ||
                std::abs(v - component(b.max, axis)) <= tol) {
                return true;
            }
        }
    }
    return false;
}

Scene Scene::reference() {
    Scene s(0.0);
    s.add_box({{4.0, -1.0, 0.0}, {6.0, 1.0, 2.0}});     // ahead on +x
    s.add_box({{-2.0, 5.0, 0.0}, {2.0, 6.0, 3.0}});     // wall on +y
    s.add_box({{-7.0, -3.0, 0.0}, {-5.0, 3.0, 1.5}});   // low block on -x
    s.add_box({{1.0, -6.0, 0.5}, {3.0, -4.0, 2.5}});    // floating block on -y
    return s;
}

Scene Scene::random(std::uint64_t seed, int box_count) {
    std::mt19937_64 rng(seed);
    Scene s(0.0);
    for (int i = 0; i < box_count; ++i) {
        const double cx = uniform(rng, -15.0, 15.0);
        const double cy = uniform(rng, -15.0, 15.0);
        const double hx = uniform(rng, 0.25, 2.0);
        const double hy = uniform(rng, 0.25, 2.0);
        const double h = uniform(rng, 0.5, 4.0);
        s.add_box({{cx - hx, cy - hy, 0.0}, {cx + hx, cy + hy, h}});
    }
    return s;
}

}  // namespace fusion::testing
