// AlignedFrame on the synthetic moving sequence: association, compensation,
// projection, and the per-point output, checked against the generator and
// against ray-cast ground truth.
#include <doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fusion/aligned_frame.hpp>
#include <fusion/sequence.hpp>
#include <fusion/time.hpp>

#include "support/scene.hpp"
#include "support/sensors.hpp"
#include "support/synthetic_sequence.hpp"

using namespace fusion::testing;

namespace {

constexpr double kMaxDistance = 100.0;
// Same relative depth tolerance as the RGB projection and compensation tests.
constexpr double kDepthRelTol = 1e-9;

struct DepthCheck {
    std::size_t projected = 0;
    std::size_t agree = 0;
    std::size_t in_front = 0;
    double median_abs_error = 0.0;
};

// Compares every projected point's depth with the depth the camera, at its
// true pose, ray-casts through the same sub-pixel position.
// With `surfaces`, only points whose beam hit a box count.
DepthCheck check_depth(const Scene& scene, const PinholeCamera& cam, const fusion::CameraFrame& cf,
                       double rel_tol, const std::vector<int>* surfaces = nullptr) {
    DepthCheck r;
    std::vector<double> errors;
    for (std::size_t i = 0; i < cf.points.size(); ++i) {
        const fusion::PointInCamera& p = cf.points[i];
        if (!p.projection) continue;
        if (surfaces != nullptr && (*surfaces)[i] < kFirstBoxSurface) continue;
        const auto& proj = *p.projection;
        const Vec3 d_cam = cam.pixel_direction_camera(proj.pixel.x(), proj.pixel.y());
        const Ray ray = cam.pixel_ray_world(proj.pixel.x(), proj.pixel.y());
        const auto hit = scene.raycast(ray.origin, ray.direction, kMaxDistance);
        ++r.projected;
        const double camera_depth =
            hit ? hit->distance * d_cam.z() : std::numeric_limits<double>::infinity();
        const double diff = camera_depth - proj.depth;
        errors.push_back(std::abs(diff));
        if (diff > rel_tol * proj.depth) ++r.in_front;
        if (std::abs(diff) <= rel_tol * proj.depth) ++r.agree;
    }
    if (!errors.empty()) {
        std::nth_element(errors.begin(),
                         errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2),
                         errors.end());
        r.median_abs_error = errors[errors.size() / 2];
    }
    return r;
}

void report(const std::string& what, std::size_t k, bool thermal, const DepthCheck& c) {
    std::ostringstream msg;
    msg << what << " scan " << k << (thermal ? " thermal: " : " rgb:     ") << c.agree << "/"
        << c.projected << " agree, median |depth error| = " << c.median_abs_error << " m";
    MESSAGE(msg.str());
}

std::vector<std::int64_t> lidar_sensor_stamps(const SyntheticSequence& s) {
    std::vector<std::int64_t> out;
    for (const auto& scan : s.scans) out.push_back(scan.t_ns - s.lidar_time_offset_ns);
    return out;
}

std::vector<fusion::Association> associations(const SyntheticSequence& s,
                                              const SyntheticCamera& cam) {
    return fusion::associate(lidar_sensor_stamps(s), s.lidar_time_offset_ns, cam.stamps_sensor_ns,
                             cam.config.time_offset_ns, cam.tau_ns);
}

void check_matches_generator(const std::vector<fusion::Association>& got,
                             const std::vector<ExpectedMatch>& want) {
    REQUIRE(got.size() == want.size());
    for (std::size_t k = 0; k < got.size(); ++k) {
        CAPTURE(k);
        CHECK(got[k].match_index == want[k].index);
        CHECK(got[k].t_match_ns == want[k].t_ns);
        CHECK(got[k].delta_t_ns == want[k].delta_t_ns);
        CHECK(got[k].accepted == want[k].accepted);
    }
}

// Everything build_aligned_frame() needs for scan k of an in-memory sequence.
struct Inputs {
    std::vector<fusion::Association> rgb;
    std::vector<fusion::Association> thermal;
    fusion::Trajectory trajectory;

    explicit Inputs(const SyntheticSequence& s)
        : rgb(associations(s, s.rgb)), thermal(associations(s, s.thermal)), trajectory(s.poses) {}

    fusion::FrameInputs at(const SyntheticSequence& s, std::size_t k, bool with_trajectory) const {
        fusion::FrameInputs in;
        in.scan = &s.scans[k];
        in.trajectory = with_trajectory ? &trajectory : nullptr;
        in.T_body_lidar = s.rig.T_body_lidar;
        in.rgb = fusion::CameraInput{&s.rgb.config, rgb[k], nullptr};
        const fusion::RawThermalImage* img =
            thermal[k].accepted ? &s.thermal_images[thermal[k].match_index] : nullptr;
        in.thermal = fusion::CameraInput{&s.thermal.config, thermal[k], img};
        return in;
    }
};

const SyntheticSequence& sequence() {
    static const SyntheticSequence s = make_synthetic_sequence(5);
    return s;
}

}  // namespace

TEST_CASE("synthetic sequence: association matches the generator") {
    const SyntheticSequence& s = sequence();
    REQUIRE(s.scans.size() == 5);
    const Inputs in(s);
    check_matches_generator(in.rgb, s.rgb.expected);
    check_matches_generator(in.thermal, s.thermal.expected);
    // The generator's design: RGB rejects scan 0 (+33 ms), thermal rejects
    // scan 3 (its frame is missing, -79 ms); the rest are accepted.
    CHECK_FALSE(s.rgb.expected[0].accepted);
    CHECK(s.rgb.expected[0].delta_t_ns == 33'000'000);
    CHECK(s.rgb.expected[2].delta_t_ns == -17'000'000);
    CHECK_FALSE(s.thermal.expected[3].accepted);
    CHECK(s.thermal.expected[3].delta_t_ns == -79'000'000);
    CHECK(s.thermal.expected[1].delta_t_ns == 21'000'000);
}

TEST_CASE("aligned frame: compensated depth agrees with ray-cast ground truth") {
    const SyntheticSequence& s = sequence();
    const Inputs in(s);
    std::size_t rgb_frames = 0, thermal_frames = 0, raw_checked = 0;
    for (std::size_t k = 0; k < s.scans.size(); ++k) {
        CAPTURE(k);
        const fusion::AlignedFrame f = fusion::build_aligned_frame(in.at(s, k, true));
        CHECK(f.t_ref_ns == s.scans[k].t_ns);
        CHECK(f.points == s.scans[k].points);
        CHECK(f.intensity == s.scans[k].intensity);
        REQUIRE(f.rgb);
        REQUIRE(f.thermal);
        CHECK(f.rgb->name == "rgb");
        CHECK(f.thermal->name == "thermal");
        CHECK(f.rgb->association.delta_t_ns == s.rgb.expected[k].delta_t_ns);
        CHECK(f.rgb->association.accepted == s.rgb.expected[k].accepted);
        CHECK(f.thermal->association.delta_t_ns == s.thermal.expected[k].delta_t_ns);
        CHECK(f.thermal->association.accepted == s.thermal.expected[k].accepted);

        for (const auto* cf : {&*f.rgb, &*f.thermal}) {
            const bool thermal = cf == &*f.thermal;
            CAPTURE(thermal);
            if (!cf->association.accepted) {
                CHECK_FALSE(cf->depth);
                CHECK(cf->points.empty());
                CHECK(cf->projected_count == 0);
                CHECK_FALSE(cf->compensated);
                continue;
            }
            (thermal ? thermal_frames : rgb_frames) += 1;
            CHECK(cf->compensated);
            REQUIRE(cf->depth);
            REQUIRE(cf->points.size() == s.scans[k].points.size());

            // Per-point bookkeeping is consistent with the depth map.
            std::size_t projected = 0, nearest = 0;
            for (const auto& p : cf->points) {
                if (!p.projection) {
                    CHECK_FALSE(p.nearest);
                    CHECK_FALSE(p.raw);
                    continue;
                }
                ++projected;
                CHECK(p.raw.has_value() == thermal);
                const auto d = cf->depth->at(p.projection->col, p.projection->row);
                REQUIRE(d);
                CHECK(*d <= p.projection->depth);
                if (p.nearest) {
                    ++nearest;
                    CHECK(*d == p.projection->depth);
                }
            }
            CHECK(projected == cf->projected_count);
            CHECK(nearest == cf->depth->size());

            const PinholeCamera cam =
                thermal ? s.rig.thermal_at(cf->association.t_match_ns)
                        : s.rig.camera_at(cf->association.t_match_ns);
            const DepthCheck c = check_depth(s.scene, cam, *cf, kDepthRelTol);
            report("in memory, 1e-9:", k, thermal, c);
            CHECK(c.projected > 200);
            CHECK(c.in_front == 0);
            CHECK(c.agree > c.projected * 8 / 10);
            CHECK(c.median_abs_error <= 1e-9);

            if (thermal) {
                // Visible points away from surface boundaries read their
                // surface's raw value.
                const std::vector<int> surf = cam.render_surfaces(s.scene, kMaxDistance);
                const int w = cam.K.width, h = cam.K.height;
                const auto surface_at = [&](int col, int row) {
                    return surf[static_cast<std::size_t>(row) * static_cast<std::size_t>(w) +
                                static_cast<std::size_t>(col)];
                };
                for (std::size_t i = 0; i < cf->points.size(); ++i) {
                    const auto& p = cf->points[i];
                    if (!p.projection) continue;
                    const auto& pr = *p.projection;
                    const Vec3 d_cam = cam.pixel_direction_camera(pr.pixel.x(), pr.pixel.y());
                    const Ray ray = cam.pixel_ray_world(pr.pixel.x(), pr.pixel.y());
                    const auto hit = s.scene.raycast(ray.origin, ray.direction, kMaxDistance);
                    if (!hit || std::abs(hit->distance * d_cam.z() - pr.depth) >
                                    kDepthRelTol * pr.depth) {
                        continue;  // occluded from the camera
                    }
                    bool uniform = true;
                    for (int dr = -2; dr <= 2 && uniform; ++dr) {
                        for (int dc = -2; dc <= 2 && uniform; ++dc) {
                            const int col = pr.col + dc, row = pr.row + dr;
                            uniform = col >= 0 && row >= 0 && col < w && row < h &&
                                      surface_at(col, row) == s.scan_surfaces[k][i];
                        }
                    }
                    if (!uniform) continue;
                    ++raw_checked;
                    CHECK(*p.raw == s.scene.raw_of(s.scan_surfaces[k][i]));
                }
            }
        }
    }
    MESSAGE("thermal raw values checked: " << raw_checked);
    CHECK(rgb_frames == 4);
    CHECK(thermal_frames == 4);
    CHECK(raw_checked > 1000);
}

TEST_CASE("aligned frame: without a trajectory the scan is not compensated") {
    const SyntheticSequence& s = sequence();
    const Inputs in(s);
    const fusion::AlignedFrame f = fusion::build_aligned_frame(in.at(s, 2, false));
    REQUIRE(f.rgb);
    REQUIRE(f.rgb->association.accepted);
    CHECK_FALSE(f.rgb->compensated);
    // The scan is 17 ms stale at 8 m/s, so the depths of box points clearly
    // disagree. (Ground points barely change: the planar motion keeps the
    // ground plane in place.)
    const DepthCheck c = check_depth(s.scene, s.rig.camera_at(f.rgb->association.t_match_ns),
                                     *f.rgb, kDepthRelTol, &s.scan_surfaces[2]);
    CHECK(c.projected > 200);
    CHECK(c.median_abs_error > 0.05);
    CHECK(c.agree < c.projected / 10);

    // A trajectory that does not cover the camera time behaves the same.
    // Poses are every 40 ms; this one ends 20 ms after the scan.
    std::vector<fusion::StampedPose> short_poses;
    for (const auto& p : s.poses) {
        if (p.t_ns <= s.scans[2].t_ns + 20'000'000) short_poses.push_back(p);
    }
    REQUIRE(short_poses.back().t_ns == s.scans[2].t_ns + 20'000'000);
    const fusion::Trajectory short_traj(short_poses);
    fusion::FrameInputs fi = in.at(s, 2, true);
    fi.trajectory = &short_traj;
    REQUIRE(f.thermal);
    const fusion::AlignedFrame g = fusion::build_aligned_frame(fi);
    CHECK(g.rgb->compensated);       // t_c = t_l - 17 ms is covered
    CHECK_FALSE(g.thermal->compensated);  // t_c = t_l + 21 ms is not
}

TEST_CASE("aligned frame: inputs are not modified and the frame owns its data") {
    const SyntheticSequence& s = sequence();
    const Inputs in(s);
    fusion::LidarScan scan = s.scans[1];
    fusion::RawThermalImage img = s.thermal_images[in.thermal[1].match_index];
    const fusion::LidarScan scan_before = scan;
    const std::vector<std::uint16_t> img_before = img.values();
    fusion::FrameInputs fi = in.at(s, 1, true);
    fi.scan = &scan;
    fi.thermal->thermal_image = &img;

    fusion::AlignedFrame f = fusion::build_aligned_frame(fi);
    CHECK(scan.points == scan_before.points);
    CHECK(scan.intensity == scan_before.intensity);
    CHECK(scan.t_ns == scan_before.t_ns);
    CHECK(img.values() == img_before);

    // Destroying the inputs leaves the frame intact.
    const auto first = f.points.front();
    const auto raw = f.thermal->points;
    scan = fusion::LidarScan{};
    img = fusion::RawThermalImage(1, 1);
    CHECK(f.points.front() == first);
    CHECK(f.thermal->points.size() == raw.size());

    // Building twice gives the same result.
    fi = in.at(s, 1, true);
    const fusion::AlignedFrame a = fusion::build_aligned_frame(fi);
    const fusion::AlignedFrame b = fusion::build_aligned_frame(fi);
    std::ostringstream ca, cb;
    fusion::write_aligned_frame_csv(ca, a);
    fusion::write_aligned_frame_csv(cb, b);
    CHECK(ca.str() == cb.str());
}

TEST_CASE("aligned frame: invalid inputs") {
    const SyntheticSequence& s = sequence();
    const Inputs in(s);

    fusion::FrameInputs fi = in.at(s, 1, true);
    fi.scan = nullptr;
    CHECK_THROWS_AS(fusion::build_aligned_frame(fi), std::invalid_argument);

    fusion::LidarScan bad = s.scans[1];
    bad.intensity.pop_back();
    fi = in.at(s, 1, true);
    fi.scan = &bad;
    CHECK_THROWS_WITH_AS(fusion::build_aligned_frame(fi), doctest::Contains("intensities"),
                         std::invalid_argument);

    fi = in.at(s, 1, true);
    fi.rgb->association = in.rgb[2];  // for another scan
    CHECK_THROWS_WITH_AS(fusion::build_aligned_frame(fi), doctest::Contains("reference time"),
                         std::invalid_argument);

    fi = in.at(s, 1, true);
    fi.thermal->thermal_image = nullptr;
    CHECK_THROWS_WITH_AS(fusion::build_aligned_frame(fi), doctest::Contains("no raw image"),
                         std::invalid_argument);

    fi = in.at(s, 1, true);
    const fusion::RawThermalImage wrong(10, 10);
    fi.thermal->thermal_image = &wrong;
    CHECK_THROWS_AS(fusion::build_aligned_frame(fi), std::invalid_argument);

    fi = in.at(s, 1, true);
    fi.rgb->config = nullptr;
    CHECK_THROWS_WITH_AS(fusion::build_aligned_frame(fi), doctest::Contains("no config"),
                         std::invalid_argument);
}

TEST_CASE("aligned frame: nearest point wins a shared pixel, first on a tie") {
    fusion::SensorConfig cfg;
    cfg.name = "cam";
    cfg.width = 4;
    cfg.height = 4;
    cfg.K << 1.0, 0.0, 1.5, 0.0, 1.0, 1.5, 0.0, 0.0, 1.0;
    cfg.source_frame = "lidar";
    // Camera frame = LiDAR frame. Points 0..2 lie on the optical axis, so
    // they land on u = v = 1.5, which is pixel (2, 2); point 3 is behind.
    fusion::LidarScan scan;
    scan.t_ns = 100;
    scan.points = {{0, 0, 3}, {0, 0, 2}, {0, 0, 2}, {0, 0, -1}};
    scan.intensity = {1, 2, 3, 4};
    fusion::FrameInputs fi;
    fi.scan = &scan;
    fusion::Association a;
    a.t_ref_ns = 100;
    a.t_match_ns = 100;
    a.accepted = true;
    fi.rgb = fusion::CameraInput{&cfg, a, nullptr};
    const fusion::AlignedFrame f = fusion::build_aligned_frame(fi);
    REQUIRE(f.rgb);
    CHECK_FALSE(f.thermal);
    CHECK(f.rgb->projected_count == 3);
    CHECK_FALSE(f.rgb->points[0].nearest);
    CHECK(f.rgb->points[1].nearest);
    CHECK_FALSE(f.rgb->points[2].nearest);
    CHECK_FALSE(f.rgb->points[3].projection);
    CHECK(f.rgb->depth->size() == 1);
    CHECK(*f.rgb->depth->at(2, 2) == 2.0);

    std::ostringstream csv;
    fusion::write_aligned_frame_csv(csv, f);
    CHECK(csv.str() ==
          "point,x,y,z,intensity,rgb_u,rgb_v,rgb_depth,rgb_nearest,"
          "thermal_u,thermal_v,thermal_depth,thermal_raw,thermal_nearest\n"
          "0,0,0,3,1,1.5,1.5,3,0,,,,,\n"
          "1,0,0,2,2,1.5,1.5,2,1,,,,,\n"
          "2,0,0,2,3,1.5,1.5,2,0,,,,,\n"
          "3,0,0,-1,4,,,,,,,,,\n");
}

TEST_CASE("aligned frame from the on-disk sequence agrees within float32 precision") {
    const SyntheticSequence& s = sequence();
    const std::string dir = std::string(FUSION_TEST_OUTPUT_DIR) + "/aligned_frame_disk";
    std::filesystem::remove_all(dir);
    const fusion::Sequence seq = fusion::load_sequence(write_synthetic_sequence(s, dir));
    REQUIRE(seq.rgb);
    REQUIRE(seq.thermal);
    REQUIRE(seq.trajectory);
    const auto rgb = fusion::associate_camera(seq, *seq.rgb);
    const auto thermal = fusion::associate_camera(seq, *seq.thermal);
    check_matches_generator(rgb, s.rgb.expected);
    check_matches_generator(thermal, s.thermal.expected);

    // The file stores float32 coordinates, so each point moves by up to
    // about 2^-24 of its coordinates (under a micrometre here). Seen at
    // grazing angles that becomes a larger depth difference, so the check
    // uses a relative tolerance of 1e-5 instead of 1e-9.
    constexpr double kFloat32RelTol = 1e-5;
    for (std::size_t k = 0; k < seq.lidar.size(); ++k) {
        CAPTURE(k);
        const fusion::LidarScan scan = fusion::load_lidar_scan(seq, k);
        std::optional<fusion::RawThermalImage> img;
        if (thermal[k].accepted) {
            img = fusion::read_thermal_frame(seq.thermal->frames[thermal[k].match_index].path,
                                             seq.thermal->config);
        }
        fusion::FrameInputs fi;
        fi.scan = &scan;
        fi.trajectory = &*seq.trajectory;
        fi.T_body_lidar = seq.T_body_lidar;
        fi.rgb = fusion::CameraInput{&seq.rgb->config, rgb[k], nullptr};
        fi.thermal = fusion::CameraInput{&seq.thermal->config, thermal[k], img ? &*img : nullptr};
        const fusion::AlignedFrame f = fusion::build_aligned_frame(fi);
        for (const auto* cf : {&*f.rgb, &*f.thermal}) {
            if (!cf->association.accepted) continue;
            const bool is_thermal = cf == &*f.thermal;
            const PinholeCamera cam = is_thermal ? s.rig.thermal_at(cf->association.t_match_ns)
                                                 : s.rig.camera_at(cf->association.t_match_ns);
            const DepthCheck c = check_depth(s.scene, cam, *cf, kFloat32RelTol);
            report("from disk, 1e-5:", k, is_thermal, c);
            CAPTURE(is_thermal);
            CHECK(cf->compensated);
            CHECK(c.projected > 200);
            CHECK(c.in_front == 0);
            CHECK(c.agree > c.projected * 8 / 10);
            CHECK(c.median_abs_error <= 1e-6);
        }
    }
}
