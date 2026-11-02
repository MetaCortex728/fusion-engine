// Pose interpolation along a trajectory.
#include <doctest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <fusion/pose.hpp>

using fusion::StampedPose;
using fusion::TimeNs;
using fusion::Trajectory;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace {

constexpr double kPi = 3.14159265358979323846;

Quaterniond yaw(double angle) { return Quaterniond(AngleAxisd(angle, Vector3d::UnitZ())); }

// Angle of the rotation taking a to b, in [0, pi]; independent of the signs.
double angle_between(const Quaterniond& a, const Quaterniond& b) {
    return a.angularDistance(b);
}

}  // namespace

TEST_CASE("interpolation returns the stored poses exactly at both endpoints") {
    const Quaterniond q0(AngleAxisd(0.3, Vector3d(1, 2, 3).normalized()));
    const Quaterniond q1(AngleAxisd(-1.1, Vector3d(-2, 0.5, 1).normalized()));
    const Trajectory traj({{100, {1, 2, 3}, q0}, {300, {4, -5, 6}, q1}});
    CHECK(traj.start_ns() == 100);
    CHECK(traj.end_ns() == 300);

    const StampedPose a = traj.interpolate(100);
    CHECK(a.t_ns == 100);
    CHECK(a.position == Vector3d(1, 2, 3));
    CHECK(a.orientation.coeffs() == traj.poses()[0].orientation.coeffs());
    const StampedPose b = traj.interpolate(300);
    CHECK(b.t_ns == 300);
    CHECK(b.position == Vector3d(4, -5, 6));
    CHECK(b.orientation.coeffs() == traj.poses()[1].orientation.coeffs());
}

TEST_CASE("interpolation at the midpoint matches a hand-computed pose") {
    // Yaw 0 to 90 degrees, position (0, 0, 0) to (2, 4, -6) over 1 s.
    const Trajectory traj({{0, {0, 0, 0}, Quaterniond::Identity()},
                           {1'000'000'000, {2, 4, -6}, yaw(kPi / 2)}});
    const StampedPose m = traj.interpolate(500'000'000);
    CHECK(m.t_ns == 500'000'000);
    CHECK((m.position - Vector3d(1, 2, -3)).norm() <= 1e-15);
    // Halfway is yaw 45 degrees: q = (cos(pi/8), 0, 0, sin(pi/8)).
    const Quaterniond expected(std::cos(kPi / 8), 0.0, 0.0, std::sin(kPi / 8));
    CHECK((m.orientation.coeffs() - expected.coeffs()).norm() <= 1e-15);

    // As a transform: body x axis maps to (cos 45, sin 45, 0).
    const Eigen::Isometry3d T = traj.T_world_body(500'000'000);
    const Vector3d x = T.linear() * Vector3d::UnitX();
    CHECK((x - Vector3d(std::sqrt(0.5), std::sqrt(0.5), 0.0)).norm() <= 1e-15);
    CHECK((T * Vector3d::Zero() - Vector3d(1, 2, -3)).norm() <= 1e-15);
}

TEST_CASE("interpolation over unequal intervals uses each segment's own alpha") {
    // Segments of 10 ns and 30 ns, yaw rate differing between them.
    const Trajectory traj({{0, {0, 0, 0}, yaw(0.0)},
                           {10, {1, 0, 0}, yaw(0.2)},
                           {40, {1, 3, 0}, yaw(0.8)}});
    SUBCASE("first segment") {
        const StampedPose p = traj.interpolate(4);  // alpha = 0.4
        CHECK((p.position - Vector3d(0.4, 0, 0)).norm() <= 1e-15);
        CHECK(angle_between(p.orientation, yaw(0.08)) <= 1e-12);
    }
    SUBCASE("second segment") {
        const StampedPose p = traj.interpolate(19);  // alpha = 9 / 30 = 0.3
        CHECK((p.position - Vector3d(1, 0.9, 0)).norm() <= 1e-15);
        CHECK(angle_between(p.orientation, yaw(0.2 + 0.3 * 0.6)) <= 1e-12);
    }
    SUBCASE("interior sample") {
        const StampedPose p = traj.interpolate(10);
        CHECK(p.position == Vector3d(1, 0, 0));
        CHECK(p.orientation.coeffs() == traj.poses()[1].orientation.coeffs());
    }
}

TEST_CASE("slerp takes the shortest arc when the second quaternion is negated") {
    const Quaterniond q0 = yaw(0.2);
    const Quaterniond q1 = yaw(0.6);
    Quaterniond q1_neg;
    q1_neg.coeffs() = -q1.coeffs();
    REQUIRE(q0.coeffs().dot(q1_neg.coeffs()) < 0.0);

    const Trajectory pos({{0, {0, 0, 0}, q0}, {100, {0, 0, 0}, q1}});
    const Trajectory neg({{0, {0, 0, 0}, q0}, {100, {0, 0, 0}, q1_neg}});
    for (const TimeNs t : {10, 25, 50, 75, 99}) {
        CAPTURE(t);
        const double alpha = static_cast<double>(t) / 100.0;
        const Quaterniond a = pos.interpolate(t).orientation;
        const Quaterniond b = neg.interpolate(t).orientation;
        // Same rotation, through yaw 0.2 + 0.4 alpha, not the long way round.
        CHECK(angle_between(a, b) <= 1e-12);
        CHECK(angle_between(b, yaw(0.2 + 0.4 * alpha)) <= 1e-12);
        CHECK(std::abs(b.norm() - 1.0) <= 1e-15);
    }

    // Long arc check: yaw 0 to yaw 350 degrees goes back 10 degrees, not
    // forward 350.
    const Quaterniond far = yaw(350.0 * kPi / 180.0);
    const Quaterniond mid = fusion::slerp_shortest(yaw(0.0), far, 0.5);
    CHECK(angle_between(mid, yaw(-5.0 * kPi / 180.0)) <= 1e-12);
}

TEST_CASE("slerp of nearly equal quaternions stays accurate and unit length") {
    const Quaterniond q0 = yaw(1.0);
    const Quaterniond q1 = yaw(1.0 + 1e-9);
    const Quaterniond q = fusion::slerp_shortest(q0, q1, 0.5);
    CHECK(std::abs(q.norm() - 1.0) <= 1e-15);
    CHECK(angle_between(q, yaw(1.0 + 0.5e-9)) <= 1e-12);
    // Identical quaternions give the same quaternion back exactly.
    CHECK(fusion::slerp_shortest(q0, q0, 0.37).coeffs() == q0.coeffs());
}

TEST_CASE("interpolation rejects times outside the trajectory") {
    const Trajectory traj({{100, {0, 0, 0}, Quaterniond::Identity()},
                           {200, {1, 0, 0}, Quaterniond::Identity()}});
    CHECK(traj.contains(100));
    CHECK(traj.contains(200));
    CHECK_FALSE(traj.contains(99));
    CHECK_FALSE(traj.contains(201));
    CHECK_THROWS_AS(traj.interpolate(99), std::out_of_range);
    CHECK_THROWS_AS(traj.interpolate(201), std::out_of_range);
    CHECK_THROWS_AS(traj.T_world_body(std::numeric_limits<TimeNs>::min()), std::out_of_range);
    CHECK_THROWS_AS(traj.T_world_body(std::numeric_limits<TimeNs>::max()), std::out_of_range);

    // A single pose is valid only at its own time.
    const Trajectory one({{5, {1, 2, 3}, yaw(0.4)}});
    CHECK(one.interpolate(5).position == Vector3d(1, 2, 3));
    CHECK_THROWS_AS(one.interpolate(4), std::out_of_range);
    CHECK_THROWS_AS(one.interpolate(6), std::out_of_range);
}

TEST_CASE("trajectory rejects unsorted, duplicate, empty, and invalid poses") {
    const Quaterniond I = Quaterniond::Identity();
    CHECK_THROWS_AS(Trajectory({}), std::invalid_argument);
    CHECK_THROWS_AS(Trajectory({{0, {0, 0, 0}, I}, {20, {0, 0, 0}, I}, {10, {0, 0, 0}, I}}),
                    std::invalid_argument);
    CHECK_THROWS_AS(Trajectory({{0, {0, 0, 0}, I}, {10, {0, 0, 0}, I}, {10, {1, 0, 0}, I}}),
                    std::invalid_argument);
    CHECK_THROWS_AS(Trajectory({{0, {std::nan(""), 0, 0}, I}}), std::invalid_argument);
    CHECK_THROWS_AS(Trajectory({{0, {0, 0, 0}, Quaterniond(0, 0, 0, 0)}}),
                    std::invalid_argument);
    CHECK_THROWS_AS(Trajectory({{std::numeric_limits<TimeNs>::min(), {0, 0, 0}, I},
                                {std::numeric_limits<TimeNs>::max(), {0, 0, 0}, I}}),
                    std::overflow_error);

    // Orientations are normalized on construction.
    const Trajectory scaled({{0, {0, 0, 0}, Quaterniond(2, 0, 0, 0)}});
    CHECK(scaled.poses()[0].orientation.coeffs() == Quaterniond::Identity().coeffs());
}
