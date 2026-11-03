// fuse_cli: runs the fusion engine over a whole sequence.
//
//   fuse_cli <sequence.yaml> --out <dir> [--frames N] [--overlay] [--viewer]
//            [--viewer-max-points N]
//
// For every LiDAR scan (the reference) it builds an AlignedFrame and writes
//   frame_NNNNNN.csv              per-point output (write_aligned_frame_csv)
//   overlay_rgb_NNNNNN.png        with --overlay: depth points over the RGB image
//   overlay_thermal_NNNNNN.png    with --overlay: depth points over the rendered
//                                 thermal image
// and at the end
//   association_rgb.csv, association_thermal.csv   time association per camera
//   summary.csv                   one row per frame
//   summary.json                  latency statistics
//   viewer/index.html             with --viewer: a static web page (see
//                                 viewer.hpp), plus its per-frame points
//                                 scripts (viewer/points/) and images
//
// Exit codes: 0 success, 1 a sequence or processing error, 2 bad arguments.
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include <fusion/aligned_frame.hpp>
#include <fusion/image_io.hpp>
#include <fusion/sequence.hpp>
#include <fusion/thermal.hpp>
#include <fusion/time.hpp>

#include "viewer.hpp"

namespace fs = std::filesystem;

namespace {

using Clock = std::chrono::steady_clock;

const char* kUsage =
    "usage: fuse_cli <sequence.yaml> --out <dir> [--frames N] [--overlay] [--viewer]\n"
    "                [--viewer-max-points N]\n"
    "\n"
    "  --out <dir>              output directory (created if missing)\n"
    "  --frames N               process only the first N LiDAR scans\n"
    "  --overlay                also write PNGs of the depth points over each camera image\n"
    "  --viewer                 also write viewer/index.html, a static web page\n"
    "  --viewer-max-points N    embed at most N points per camera and frame in the page\n"
    "                           (default 20000, deterministic subsampling above that)\n";

struct Args {
    std::string manifest;
    std::string out_dir;
    std::optional<std::size_t> frames;
    bool overlay = false;
    bool viewer = false;
    std::size_t viewer_max_points = fuse_cli::kDefaultViewerMaxPoints;
};

struct UsageError {
    std::string what;
};

struct HelpRequested {};

std::size_t positive(const std::string& option, const std::string& v) {
    std::size_t n = 0;
    const auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
    if (ec != std::errc() || ptr != v.data() + v.size() || n == 0) {
        throw UsageError{option + " needs a positive integer, got '" + v + "'"};
    }
    return n;
}

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            throw HelpRequested{};
        } else if (arg == "--out") {
            if (++i >= argc) throw UsageError{"--out needs a directory"};
            a.out_dir = argv[i];
        } else if (arg == "--frames") {
            if (++i >= argc) throw UsageError{"--frames needs a number"};
            a.frames = positive("--frames", argv[i]);
        } else if (arg == "--overlay") {
            a.overlay = true;
        } else if (arg == "--viewer") {
            a.viewer = true;
        } else if (arg == "--viewer-max-points") {
            if (++i >= argc) throw UsageError{"--viewer-max-points needs a number"};
            a.viewer_max_points = positive("--viewer-max-points", argv[i]);
        } else if (!arg.empty() && arg[0] == '-') {
            throw UsageError{"unknown option " + arg};
        } else if (a.manifest.empty()) {
            a.manifest = arg;
        } else {
            throw UsageError{"unexpected argument " + arg};
        }
    }
    if (a.manifest.empty()) throw UsageError{"missing <sequence.yaml>"};
    if (a.out_dir.empty()) throw UsageError{"missing --out <dir>"};
    return a;
}

double ms_since(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::string num(double v) {
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, r.ptr);
}

std::string frame_name(const std::string& prefix, std::size_t i, const std::string& ext) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%06zu", i);
    return prefix + buf + ext;
}

// Median (mean of the two middle values for an even count) and p95 by the
// nearest-rank method: the ceil(0.95 n)-th smallest value.
using fuse_cli::Stats;

Stats stats(std::vector<double> v) {
    Stats s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    s.median = n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    const auto rank = static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(n)));
    s.p95 = v[std::max<std::size_t>(rank, 1) - 1];
    return s;
}

// --- overlays -----------------------------------------------------------------

// Near points red, far points blue, through yellow, green and cyan.
fusion::Rgb8 depth_color(double t) {
    t = std::clamp(t, 0.0, 1.0);
    const double h = 4.0 * t;  // 0 red, 1 yellow, 2 green, 3 cyan, 4 blue
    const int k = std::min(static_cast<int>(h), 3);
    const double f = h - k;
    const auto c = [](double x) { return static_cast<std::uint8_t>(std::lround(255.0 * x)); };
    switch (k) {
        case 0: return {255, c(f), 0};
        case 1: return {c(1.0 - f), 255, 0};
        case 2: return {0, 255, c(f)};
        default: return {0, c(1.0 - f), 255};
    }
}

void draw_depth(fusion::RgbImage& img, const fusion::SparseDepthMap& depth) {
    std::vector<fusion::SparseDepthMap::Entry> e = depth.entries();
    if (e.empty()) return;
    double lo = std::numeric_limits<double>::infinity(), hi = 0.0;
    for (const auto& x : e) {
        lo = std::min(lo, x.depth);
        hi = std::max(hi, x.depth);
    }
    // Far first, so nearer points are drawn on top.
    std::sort(e.begin(), e.end(), [](const auto& a, const auto& b) { return a.depth > b.depth; });
    for (const auto& x : e) {
        const fusion::Rgb8 c = depth_color(hi > lo ? (x.depth - lo) / (hi - lo) : 0.0);
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                const int col = x.col + dc, row = x.row + dr;
                if (col < 0 || row < 0 || col >= img.width || row >= img.height) continue;
                img.pixels[static_cast<std::size_t>(row) * static_cast<std::size_t>(img.width) +
                           static_cast<std::size_t>(col)] = c;
            }
        }
    }
}

// Thermal image for display: with a configured conversion (Planck or linear),
// Celsius windowed to the image's range and the hot colormap; without, raw
// counts stretched to grey.
fusion::RgbImage render_thermal(const fusion::RawThermalImage& raw,
                                const fusion::SensorConfig& config) {
    if (config.thermal) {
        const fusion::ThermalCalibration& cal = *config.thermal;
        double lo = std::numeric_limits<double>::infinity();
        double hi = -lo;
        for (std::uint16_t v : raw.values()) {
            try {
                const double c = fusion::raw_to_celsius(v, cal);
                lo = std::min(lo, c);
                hi = std::max(hi, c);
            } catch (const std::domain_error&) {
            }
        }
        if (lo <= hi) {
            const fusion::WindowLevel w{0.5 * (lo + hi), std::max(hi - lo, 0.1)};
            return fusion::render_display(raw, cal, w);
        }
    }
    const auto [mn, mx] = std::minmax_element(raw.values().begin(), raw.values().end());
    fusion::RgbImage img{raw.width(), raw.height(), {}};
    img.pixels.reserve(raw.values().size());
    const double span = std::max(1.0, static_cast<double>(*mx) - static_cast<double>(*mn));
    for (std::uint16_t v : raw.values()) {
        const auto g = static_cast<std::uint8_t>(
            std::lround(255.0 * (static_cast<double>(v) - static_cast<double>(*mn)) / span));
        img.pixels.push_back({g, g, g});
    }
    return img;
}

// --- summary ------------------------------------------------------------------

struct FrameRow {
    std::size_t frame = 0;
    fusion::TimeNs t_ref_ns = 0;
    std::size_t points = 0;
    std::optional<fusion::CameraFrame> rgb;      // without the per-point data
    std::optional<fusion::CameraFrame> thermal;  // without the per-point data
    double read_ms = 0.0;
    double process_ms = 0.0;
    double write_ms = 0.0;
};

void put_camera(std::ostream& out, const std::optional<fusion::CameraFrame>& c) {
    if (!c) {
        out << ",,,,,,";
        return;
    }
    const fusion::Association& a = c->association;
    out << ',' << a.match_index << ',' << a.t_match_ns << ',' << a.delta_t_ns << ','
        << (a.accepted ? 1 : 0) << ',' << (c->compensated ? 1 : 0) << ',' << c->projected_count;
}

void write_summary_csv(const std::string& path, const std::vector<FrameRow>& rows) {
    std::ofstream out(path);
    out << "frame,t_ref_ns,points,"
           "rgb_index,rgb_t_ns,rgb_delta_t_ns,rgb_accepted,rgb_compensated,rgb_projected,"
           "thermal_index,thermal_t_ns,thermal_delta_t_ns,thermal_accepted,"
           "thermal_compensated,thermal_projected,read_ms,process_ms,write_ms\n";
    for (const FrameRow& r : rows) {
        out << r.frame << ',' << r.t_ref_ns << ',' << r.points;
        put_camera(out, r.rgb);
        put_camera(out, r.thermal);
        out << ',' << num(r.read_ms) << ',' << num(r.process_ms) << ',' << num(r.write_ms) << '\n';
    }
    out.flush();
    if (!out) throw std::runtime_error("cannot write " + path);
}

void write_summary_json(const std::string& path, const std::string& manifest,
                        const std::vector<FrameRow>& rows, const Stats& read, const Stats& process,
                        const Stats& write) {
    std::ofstream out(path);
    const auto stat = [&](const char* name, const Stats& s) {
        out << "  \"" << name << "\": {\"median\": " << num(s.median) << ", \"p95\": " << num(s.p95)
            << "}";
    };
    std::string escaped;
    for (char ch : manifest) {
        if (ch == '"' || ch == '\\') escaped += '\\';
        escaped += ch;
    }
    out << "{\n  \"sequence\": \"" << escaped << "\",\n  \"frames\": " << rows.size() << ",\n";
    stat("process_ms", process);
    out << ",\n";
    stat("read_ms", read);
    out << ",\n";
    stat("write_ms", write);
    out << "\n}\n";
    out.flush();
    if (!out) throw std::runtime_error("cannot write " + path);
}

// --- main loop ----------------------------------------------------------------

int run(const Args& args) {
    const fusion::Sequence seq = fusion::load_sequence(args.manifest);

    std::error_code ec;
    fs::create_directories(args.out_dir, ec);
    if (ec || !fs::is_directory(args.out_dir)) {
        throw std::runtime_error("cannot create output directory " + args.out_dir);
    }
    const auto out_path = [&](const std::string& name) {
        return (fs::path(args.out_dir) / name).string();
    };

    std::vector<fusion::Association> rgb_assoc, thermal_assoc;
    if (seq.rgb) {
        rgb_assoc = fusion::associate_camera(seq, *seq.rgb);
        fusion::write_association_csv(out_path("association_rgb.csv"), rgb_assoc);
    }
    if (seq.thermal) {
        thermal_assoc = fusion::associate_camera(seq, *seq.thermal);
        fusion::write_association_csv(out_path("association_thermal.csv"), thermal_assoc);
    }

    const std::size_t n = std::min(seq.lidar.size(), args.frames.value_or(seq.lidar.size()));
    std::optional<fuse_cli::ViewerWriter> viewer;
    if (args.viewer) viewer.emplace(args.out_dir, seq, args.manifest, args.viewer_max_points);

    std::vector<FrameRow> rows;
    std::size_t uncompensated = 0;
    for (std::size_t i = 0; i < n; ++i) {
        // Reading: the scan, the thermal image (needed for raw values) and,
        // for overlays and the viewer, the RGB image.
        auto t0 = Clock::now();
        const fusion::LidarScan scan = fusion::load_lidar_scan(seq, i);
        std::optional<fusion::RawThermalImage> thermal_img;
        std::optional<fusion::RgbImage> rgb_img;
        if (seq.thermal && thermal_assoc[i].accepted) {
            thermal_img = fusion::read_thermal_frame(
                seq.thermal->frames[thermal_assoc[i].match_index].path, seq.thermal->config);
        }
        if ((args.overlay || args.viewer) && seq.rgb && rgb_assoc[i].accepted) {
            rgb_img = fusion::read_rgb_frame(seq.rgb->frames[rgb_assoc[i].match_index].path,
                                             seq.rgb->config);
        }
        FrameRow row;
        row.read_ms = ms_since(t0);

        // Processing: association lookup, compensation, projection.
        t0 = Clock::now();
        fusion::FrameInputs in;
        in.scan = &scan;
        if (seq.trajectory) in.trajectory = &*seq.trajectory;
        in.T_body_lidar = seq.T_body_lidar;
        if (seq.rgb) in.rgb = fusion::CameraInput{&seq.rgb->config, rgb_assoc[i], nullptr};
        if (seq.thermal) {
            in.thermal = fusion::CameraInput{&seq.thermal->config, thermal_assoc[i],
                                             thermal_img ? &*thermal_img : nullptr};
        }
        const fusion::AlignedFrame frame = fusion::build_aligned_frame(in);
        row.process_ms = ms_since(t0);

        // Writing.
        t0 = Clock::now();
        fusion::write_aligned_frame_csv(out_path(frame_name("frame_", i, ".csv")), frame);
        std::optional<fusion::RgbImage> thermal_display;
        if ((args.overlay || args.viewer) && thermal_img) {
            thermal_display = render_thermal(*thermal_img, seq.thermal->config);
        }
        if (viewer) {
            // Before the overlays, which draw into the images.
            viewer->add_frame(i, frame, rgb_img ? &*rgb_img : nullptr,
                              thermal_display ? &*thermal_display : nullptr);
        }
        if (args.overlay) {
            if (rgb_img && frame.rgb && frame.rgb->depth) {
                draw_depth(*rgb_img, *frame.rgb->depth);
                fusion::write_rgb_png(out_path(frame_name("overlay_rgb_", i, ".png")), *rgb_img);
            }
            if (thermal_display && frame.thermal && frame.thermal->depth) {
                draw_depth(*thermal_display, *frame.thermal->depth);
                fusion::write_rgb_png(out_path(frame_name("overlay_thermal_", i, ".png")),
                                      *thermal_display);
            }
        }
        row.write_ms = ms_since(t0);

        row.frame = i;
        row.t_ref_ns = frame.t_ref_ns;
        row.points = frame.points.size();
        for (auto [src, dst] : {std::pair{&frame.rgb, &row.rgb}, {&frame.thermal, &row.thermal}}) {
            if (!*src) continue;
            fusion::CameraFrame c;
            c.name = (*src)->name;
            c.association = (*src)->association;
            c.compensated = (*src)->compensated;
            c.projected_count = (*src)->projected_count;
            if (c.association.accepted && !c.compensated) ++uncompensated;
            *dst = std::move(c);
        }
        rows.push_back(std::move(row));
    }

    std::vector<double> read, process, write;
    for (const FrameRow& r : rows) {
        read.push_back(r.read_ms);
        process.push_back(r.process_ms);
        write.push_back(r.write_ms);
    }
    const Stats rs = stats(read), ps = stats(process), ws = stats(write);
    write_summary_csv(out_path("summary.csv"), rows);
    write_summary_json(out_path("summary.json"), args.manifest, rows, rs, ps, ws);
    if (viewer) viewer->finish(read, process, write, rs, ps, ws);

    std::cout << "fuse_cli: " << rows.size() << " frames from " << args.manifest << " -> "
              << args.out_dir << "\n"
              << "  processing (excluding file I/O): median " << num(ps.median) << " ms, p95 "
              << num(ps.p95) << " ms\n"
              << "  reading:                         median " << num(rs.median) << " ms, p95 "
              << num(rs.p95) << " ms\n";
    if (viewer) std::cout << "  viewer: " << viewer->index_path() << "\n";
    if (uncompensated > 0) {
        std::cout << "  note: " << uncompensated
                  << " accepted camera images were projected without motion compensation "
                     "(no trajectory, or it does not cover both times)\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parse_args(argc, argv);
    } catch (const HelpRequested&) {
        std::cout << kUsage;
        return 0;
    } catch (const UsageError& e) {
        std::cerr << "fuse_cli: " << e.what << "\n\n" << kUsage;
        return 2;
    }
    try {
        return run(args);
    } catch (const std::exception& e) {
        std::cerr << "fuse_cli: error: " << e.what() << "\n";
        return 1;
    }
}
