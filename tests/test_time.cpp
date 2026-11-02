// Time association: boundary cases and a jittered 10 Hz / 15 Hz sequence.
#include <doctest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fusion/config.hpp>
#include <fusion/time.hpp>

using fusion::Association;
using fusion::TimeNs;

namespace {

constexpr TimeNs kMs = 1'000'000;

}  // namespace

TEST_CASE("association accepts |delta_t| == tau and rejects tau + 1 ns") {
    const TimeNs tau = 5 * kMs;
    const auto r = fusion::associate({1000 * kMs, 2000 * kMs, 3000 * kMs, 4000 * kMs},
                                     {1000 * kMs + tau, 2000 * kMs - tau,
                                      3000 * kMs + tau + 1, 4000 * kMs - tau - 1},
                                     tau);
    REQUIRE(r.size() == 4);
    CHECK(r[0].delta_t_ns == tau);
    CHECK(r[0].accepted);
    CHECK(r[1].delta_t_ns == -tau);
    CHECK(r[1].accepted);
    CHECK(r[2].delta_t_ns == tau + 1);
    CHECK_FALSE(r[2].accepted);
    CHECK(r[3].delta_t_ns == -tau - 1);
    CHECK_FALSE(r[3].accepted);
    for (std::size_t i = 0; i < r.size(); ++i) {
        CHECK(r[i].match_index == i);
        CHECK(r[i].t_match_ns == r[i].t_ref_ns + r[i].delta_t_ns);
    }
}

TEST_CASE("association with tau == 0 accepts only exact matches") {
    const auto r = fusion::associate({10, 20}, {10, 21}, 0);
    REQUIRE(r.size() == 2);
    CHECK(r[0].accepted);
    CHECK(r[0].delta_t_ns == 0);
    CHECK_FALSE(r[1].accepted);
    CHECK(r[1].delta_t_ns == 1);
}

TEST_CASE("association picks the nearest measurement and returns rejected candidates") {
    const std::vector<TimeNs> other{0, 100, 200, 300};
    const auto r = fusion::associate({-50, 40, 160, 260, 1000}, other, 30);
    REQUIRE(r.size() == 5);
    CHECK(r[0].match_index == 0);  // before every measurement
    CHECK(r[0].delta_t_ns == 50);
    CHECK_FALSE(r[0].accepted);
    CHECK(r[1].match_index == 0);
    CHECK(r[1].delta_t_ns == -40);
    CHECK_FALSE(r[1].accepted);
    CHECK(r[2].match_index == 2);
    CHECK(r[2].delta_t_ns == 40);
    CHECK(r[3].match_index == 3);
    CHECK(r[3].delta_t_ns == 40);
    CHECK(r[4].match_index == 3);  // after every measurement
    CHECK(r[4].delta_t_ns == -700);
    CHECK_FALSE(r[4].accepted);
}

TEST_CASE("association ties go to the earlier measurement") {
    SUBCASE("halfway between two measurements") {
        const auto r = fusion::associate({150, 250}, {100, 200, 300}, 50);
        REQUIRE(r.size() == 2);
        CHECK(r[0].match_index == 0);
        CHECK(r[0].t_match_ns == 100);
        CHECK(r[0].delta_t_ns == -50);
        CHECK(r[0].accepted);
        CHECK(r[1].match_index == 1);
        CHECK(r[1].delta_t_ns == -50);
    }
    SUBCASE("halfway, odd gap: no tie, nearer wins") {
        const auto r = fusion::associate({150}, {100, 201}, 100);
        REQUIRE(r.size() == 1);
        CHECK(r[0].match_index == 0);
        const auto s = fusion::associate({151}, {100, 201}, 100);
        CHECK(s[0].match_index == 1);
    }
    SUBCASE("equal measurement times pick the first index") {
        const auto r = fusion::associate({100, 150, 500}, {100, 100, 200, 200, 200}, 50);
        REQUIRE(r.size() == 3);
        CHECK(r[0].match_index == 0);
        CHECK(r[1].match_index == 0);  // tie between 100 and 200
        CHECK(r[2].match_index == 2);  // after the end: first of the equal last times
    }
}

TEST_CASE("association of empty inputs is empty") {
    CHECK(fusion::associate({}, {}, 10).empty());
    CHECK(fusion::associate({}, {1, 2, 3}, 10).empty());
    CHECK(fusion::associate({1, 2, 3}, {}, 10).empty());
    std::ostringstream csv;
    fusion::write_association_csv(csv, {});
    CHECK(csv.str() == "t_ref_ns,t_match_ns,delta_t_ns,accepted\n");
}

TEST_CASE("association rejects bad input") {
    CHECK_THROWS_AS(fusion::associate({1}, {1}, -1), std::invalid_argument);
    CHECK_THROWS_AS(fusion::associate({2, 1}, {1}, 1), std::invalid_argument);
    CHECK_THROWS_AS(fusion::associate({1}, {3, 2}, 1), std::invalid_argument);
    constexpr TimeNs kMax = std::numeric_limits<TimeNs>::max();
    constexpr TimeNs kMin = std::numeric_limits<TimeNs>::min();
    CHECK_THROWS_AS(fusion::associate({kMin}, {kMax}, 1), std::overflow_error);
    CHECK_THROWS_AS(fusion::to_reference_time({kMax - 1}, 2), std::overflow_error);
    CHECK_THROWS_AS(fusion::to_reference_time({kMin + 1}, -2), std::overflow_error);
}

TEST_CASE("time offset moves sensor stamps onto the reference clock before association") {
    CHECK(fusion::to_reference_time({0, 10, -5}, 7) == std::vector<TimeNs>{7, 17, 2});

    // The camera clock runs 12 ms behind: its stamps are 12 ms early.
    const TimeNs offset = 12 * kMs;
    const std::vector<TimeNs> lidar{0, 100 * kMs, 200 * kMs};
    const std::vector<TimeNs> camera_sensor{-12 * kMs + 1 * kMs, 88 * kMs + 2 * kMs,
                                            188 * kMs - 3 * kMs};
    const TimeNs tau = 5 * kMs;

    const auto raw = fusion::associate(lidar, camera_sensor, tau);
    for (const Association& a : raw) CHECK_FALSE(a.accepted);

    const auto r = fusion::associate(lidar, 0, camera_sensor, offset, tau);
    REQUIRE(r.size() == 3);
    CHECK(r[0].delta_t_ns == 1 * kMs);
    CHECK(r[1].delta_t_ns == 2 * kMs);
    CHECK(r[2].delta_t_ns == -3 * kMs);
    CHECK(r[2].t_match_ns == 197 * kMs);  // reference clock, not sensor clock
    for (const Association& a : r) CHECK(a.accepted);

    // The offset read from a config is used the same way, on either side.
    const fusion::SensorConfig cfg = fusion::parse_sensor_config(
        "name: rgb\n"
        "resolution: [640, 480]\n"
        "intrinsics: {K: [[500, 0, 319.5], [0, 500, 239.5], [0, 0, 1]]}\n"
        "distortion: [0, 0, 0, 0, 0]\n"
        "extrinsics:\n"
        "  source: lidar\n"
        "  rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]\n"
        "  translation: [0, 0, 0]\n"
        "time_offset_ns: 12000000\n");
    REQUIRE(cfg.time_offset_ns == offset);
    const auto c = fusion::associate(lidar, 0, camera_sensor, cfg.time_offset_ns, tau);
    REQUIRE(c.size() == 3);
    for (std::size_t i = 0; i < c.size(); ++i) CHECK(c[i].delta_t_ns == r[i].delta_t_ns);

    // A reference offset shifts the reference times and every delta_t.
    const auto s = fusion::associate(lidar, 1 * kMs, camera_sensor, offset, tau);
    CHECK(s[0].t_ref_ns == 1 * kMs);
    CHECK(s[0].delta_t_ns == 0);
}

TEST_CASE("default tau is half the slower sensor's period") {
    CHECK(fusion::default_tau_ns(100 * kMs, 66'666'667) == 50 * kMs);
    CHECK(fusion::default_tau_ns(66'666'667, 100 * kMs) == 50 * kMs);
    CHECK(fusion::default_tau_ns(3, 3) == 1);
    CHECK_THROWS_AS(fusion::default_tau_ns(0, 10), std::invalid_argument);
    CHECK_THROWS_AS(fusion::default_tau_ns(10, -1), std::invalid_argument);
}

TEST_CASE("CSV rows hold t_ref_ns,t_match_ns,delta_t_ns,accepted") {
    const auto r = fusion::associate({100, 200}, {90, 260}, 20);
    std::ostringstream csv;
    fusion::write_association_csv(csv, r);
    CHECK(csv.str() ==
          "t_ref_ns,t_match_ns,delta_t_ns,accepted\n"
          "100,90,-10,1\n"
          "200,260,60,0\n");
    CHECK_THROWS_AS(fusion::write_association_csv(
                        std::string(FUSION_TEST_OUTPUT_DIR) + "/no/such/dir/a.csv", r),
                    std::runtime_error);
}

// 10 Hz LiDAR (reference) against a 15 Hz camera, each stamp jittered by a
// seeded uniform integer in [-J, J], and the camera clock 7 ms behind with
// the matching time_offset_ns.
//
// Hand-derived bounds. Without jitter the LiDAR fires at k * 100 ms and the
// camera at m * 200/3 ms. For even k the LiDAR time is a camera time
// (k * 100 = (3k/2) * 200/3), so |delta_t| <= 2J with jitter. For odd k it
// sits halfway between two camera times, 100/3 ms from each. With jitter
// both candidates are 100/3 ms + e, with |e| <= 2J, so the nearer one has
// |delta_t| in [100/3 - 2J, 100/3 + 2J]. Moreover, the mean of the two
// distances is 100/3 + (j_after - j_before) / 2, so the nearer one is at
// most 100/3 + J. Integer camera stamps (floor of m * 2e8 / 3 ns) move each
// bound by at most 1 ns.
//
// With N scans (N even), half fall in each group, so
//   mean |delta_t| in [(100/3 - 2J) / 2, (2J + 100/3 + J) / 2].
// Every |delta_t| <= 100/3 + J < tau = 50 ms, so all are accepted.
TEST_CASE("10 Hz LiDAR against 15 Hz camera with seeded jitter: delta-t CSV statistics") {
    constexpr TimeNs J = 2 * kMs;
    constexpr TimeNs kOffset = 7 * kMs;
    constexpr int kScans = 200;  // 20 s
    constexpr TimeNs kStart = 1'000'000'000;  // 1 s, so jittered times stay positive

    std::mt19937_64 rng(20261102);
    const auto jitter = [&rng]() {
        return static_cast<TimeNs>(rng() % static_cast<std::uint64_t>(2 * J + 1)) - J;
    };

    std::vector<TimeNs> lidar;
    for (int k = 0; k < kScans; ++k) lidar.push_back(kStart + k * 100 * kMs + jitter());
    std::vector<TimeNs> camera_sensor;
    // Two extra frames on each side so every scan has camera frames around it.
    for (int m = -2; m < kScans * 3 / 2 + 2; ++m) {
        camera_sensor.push_back(kStart + m * 200'000'000LL / 3 + jitter() - kOffset);
    }

    const TimeNs tau = fusion::default_tau_ns(100 * kMs, 1'000'000'000 / 15);
    CHECK(tau == 50 * kMs);
    const auto r = fusion::associate(lidar, 0, camera_sensor, kOffset, tau);
    REQUIRE(r.size() == static_cast<std::size_t>(kScans));

    const std::string path = std::string(FUSION_TEST_OUTPUT_DIR) + "/association_10hz_15hz.csv";
    fusion::write_association_csv(path, r);

    // Read the statistics back from the CSV.
    std::ifstream in(path);
    REQUIRE(in.good());
    std::string line;
    std::getline(in, line);
    CHECK(line == "t_ref_ns,t_match_ns,delta_t_ns,accepted");
    std::vector<TimeNs> deltas;
    int accepted = 0;
    while (std::getline(in, line)) {
        std::istringstream row(line);
        std::string t_ref, t_match, delta, acc;
        std::getline(row, t_ref, ',');
        std::getline(row, t_match, ',');
        std::getline(row, delta, ',');
        std::getline(row, acc, ',');
        const TimeNs d = std::stoll(delta);
        CHECK(std::stoll(t_match) - std::stoll(t_ref) == d);
        deltas.push_back(d);
        accepted += acc == "1" ? 1 : 0;
    }
    REQUIRE(deltas.size() == static_cast<std::size_t>(kScans));
    CHECK(accepted == kScans);

    constexpr TimeNs third = 100 * kMs / 3;  // 33 333 333 ns
    int near_zero = 0;
    int near_third = 0;
    TimeNs max_abs = 0;
    double sum_abs = 0.0;
    for (std::size_t k = 0; k < deltas.size(); ++k) {
        const TimeNs a = std::llabs(deltas[k]);
        max_abs = std::max(max_abs, a);
        sum_abs += static_cast<double>(a);
        if (k % 2 == 0) {
            CHECK(a <= 2 * J + 1);
            ++near_zero;
        } else {
            CHECK(a >= third - 2 * J - 1);
            CHECK(a <= third + J + 1);
            ++near_third;
        }
    }
    CHECK(near_zero == kScans / 2);
    CHECK(near_third == kScans / 2);
    CHECK(max_abs <= third + J + 1);
    CHECK(max_abs < tau);
    const double mean_abs_ms = sum_abs / kScans / 1e6;
    const double lo_ms = (100.0 / 3.0 - 2.0 * 2.0) / 2.0;
    const double hi_ms = (2.0 * 2.0 + 100.0 / 3.0 + 2.0) / 2.0;
    MESSAGE("10 Hz / 15 Hz: mean |delta_t| = " << mean_abs_ms << " ms, bounds [" << lo_ms
                                               << ", " << hi_ms << "] ms, max |delta_t| = "
                                               << static_cast<double>(max_abs) / 1e6 << " ms");
    CHECK(mean_abs_ms >= lo_ms - 1e-6);
    CHECK(mean_abs_ms <= hi_ms + 1e-6);
}
