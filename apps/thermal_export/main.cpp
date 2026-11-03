// thermal_export: converts every thermal frame of a sequence to degrees
// Celsius with the camera's configured encoding and renders it for display.
//
//   thermal_export <sequence.yaml> --out <dir> [--frames N]
//                  [--percentiles LO,HI] [--no-celsius] [--no-render]
//
// For thermal frame j (in list order) it writes
//   celsius_NNNNNN.f64   width * height float64 values, row-major,
//                        little-endian; NaN where the raw value cannot be
//                        converted
//   render_NNNNNN.png    render_display() with a per-image window from the
//                        LO-th to the HI-th percentile (default 1,99)
// and at the end
//   thermal_stats.csv    one row per frame: size, convertible pixels, min,
//                        max and mean Celsius, the window, pixels below and
//                        above it, black and white display pixels
//
// Percentiles use the nearest-rank method over the convertible pixels: the
// ceil(p / 100 * n)-th smallest value (the smallest for p = 0).
//
// Exit codes: 0 success, 1 a sequence or processing error, 2 bad arguments.
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <fusion/image_io.hpp>
#include <fusion/sequence.hpp>
#include <fusion/thermal.hpp>

namespace fs = std::filesystem;

namespace {

const char* kUsage =
    "usage: thermal_export <sequence.yaml> --out <dir> [--frames N]\n"
    "                      [--percentiles LO,HI] [--no-celsius] [--no-render]\n"
    "\n"
    "  --out <dir>          output directory (created if missing)\n"
    "  --frames N           process only the first N thermal frames\n"
    "  --percentiles LO,HI  display window from the LO-th to the HI-th percentile\n"
    "                       of each image (default 1,99; 0 <= LO < HI <= 100)\n"
    "  --no-celsius         do not write celsius_NNNNNN.f64\n"
    "  --no-render          do not write render_NNNNNN.png\n";

struct Args {
    std::string manifest;
    std::string out_dir;
    std::optional<std::size_t> frames;
    double p_lo = 1.0;
    double p_hi = 99.0;
    bool celsius = true;
    bool render = true;
};

struct UsageError {
    std::string what;
};

struct HelpRequested {};

double parse_double(const std::string& s, const std::string& what) {
    double v = 0.0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc() || ptr != s.data() + s.size() || !std::isfinite(v)) {
        throw UsageError{what + ": expected a number, got '" + s + "'"};
    }
    return v;
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
            const std::string v = argv[i];
            std::size_t n = 0;
            const auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
            if (ec != std::errc() || ptr != v.data() + v.size() || n == 0) {
                throw UsageError{"--frames needs a positive integer, got '" + v + "'"};
            }
            a.frames = n;
        } else if (arg == "--percentiles") {
            if (++i >= argc) throw UsageError{"--percentiles needs LO,HI"};
            const std::string v = argv[i];
            const auto comma = v.find(',');
            if (comma == std::string::npos) throw UsageError{"--percentiles needs LO,HI"};
            a.p_lo = parse_double(v.substr(0, comma), "--percentiles");
            a.p_hi = parse_double(v.substr(comma + 1), "--percentiles");
            if (!(a.p_lo >= 0.0 && a.p_lo < a.p_hi && a.p_hi <= 100.0)) {
                throw UsageError{"--percentiles needs 0 <= LO < HI <= 100"};
            }
        } else if (arg == "--no-celsius") {
            a.celsius = false;
        } else if (arg == "--no-render") {
            a.render = false;
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

std::string num(double v) {
    if (std::isnan(v)) return "";
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, r.ptr);
}

std::string frame_name(const std::string& prefix, std::size_t i, const std::string& ext) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%06zu", i);
    return prefix + buf + ext;
}

// Nearest rank on sorted values: the ceil(p / 100 * n)-th smallest.
double percentile(const std::vector<double>& sorted, double p) {
    const std::size_t n = sorted.size();
    const auto rank = static_cast<std::size_t>(std::ceil(p / 100.0 * static_cast<double>(n)));
    return sorted[std::min(std::max<std::size_t>(rank, 1), n) - 1];
}

// Little-endian on any host.
void write_f64(const std::string& path, const std::vector<double>& values) {
    std::ofstream out(path, std::ios::binary);
    std::vector<unsigned char> bytes(values.size() * 8);
    for (std::size_t i = 0; i < values.size(); ++i) {
        std::uint64_t u = 0;
        std::memcpy(&u, &values[i], 8);
        for (std::size_t k = 0; k < 8; ++k) bytes[i * 8 + k] = static_cast<unsigned char>(u >> (8 * k));
    }
    out.write(reinterpret_cast<const char*>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
    out.flush();
    if (!out) throw std::runtime_error("cannot write " + path);
}

int run(const Args& args) {
    const fusion::Sequence seq = fusion::load_sequence(args.manifest);
    if (!seq.thermal) throw std::runtime_error(args.manifest + ": the sequence has no thermal camera");
    const fusion::SensorConfig& config = seq.thermal->config;
    if (!config.thermal) {
        throw std::runtime_error(args.manifest + ": the thermal camera config has no 'thermal' "
                                 "section, so raw values cannot be converted");
    }
    const fusion::ThermalCalibration& cal = *config.thermal;
    fusion::validate(cal);

    std::error_code ec;
    fs::create_directories(args.out_dir, ec);
    if (ec || !fs::is_directory(args.out_dir)) {
        throw std::runtime_error("cannot create output directory " + args.out_dir);
    }
    const auto out_path = [&](const std::string& name) {
        return (fs::path(args.out_dir) / name).string();
    };

    std::ofstream stats(out_path("thermal_stats.csv"));
    stats << "frame,timestamp_ns,width,height,convertible,min_c,max_c,mean_c,window_lo_c,"
             "window_hi_c,below_window,above_window,display_black,display_white\n";

    const std::size_t n =
        std::min(seq.thermal->frames.size(), args.frames.value_or(seq.thermal->frames.size()));
    for (std::size_t j = 0; j < n; ++j) {
        const fusion::RawThermalImage raw = fusion::read_thermal_frame(seq.thermal->frames[j].path, config);
        std::vector<double> celsius;
        celsius.reserve(raw.values().size());
        std::vector<double> finite;
        finite.reserve(raw.values().size());
        double sum = 0.0;
        for (const std::uint16_t s : raw.values()) {
            double c = std::numeric_limits<double>::quiet_NaN();
            try {
                c = fusion::raw_to_celsius(s, cal);
            } catch (const std::domain_error&) {
            }
            celsius.push_back(c);
            if (std::isfinite(c)) {
                finite.push_back(c);
                sum += c;
            }
        }
        if (args.celsius) write_f64(out_path(frame_name("celsius_", j, ".f64")), celsius);

        const double nan = std::numeric_limits<double>::quiet_NaN();
        double mn = nan, mx = nan, mean = nan, lo = nan, hi = nan;
        std::size_t below = 0, above = 0, black = 0, white = 0;
        if (!finite.empty()) {
            std::sort(finite.begin(), finite.end());
            mn = finite.front();
            mx = finite.back();
            mean = sum / static_cast<double>(finite.size());
            lo = percentile(finite, args.p_lo);
            hi = percentile(finite, args.p_hi);
            for (const double c : finite) {
                if (c < lo) ++below;
                if (c > hi) ++above;
            }
            // A flat image still gets a valid, narrow window around its value.
            const fusion::WindowLevel window{0.5 * (lo + hi), std::max(hi - lo, 1e-6)};
            const fusion::DisplayImage display = fusion::render_display(raw, cal, window);
            fusion::RgbImage img{display.width, display.height, {}};
            img.pixels.reserve(display.pixels.size());
            for (const fusion::Rgb8& p : display.pixels) {
                if (p == fusion::Rgb8{0, 0, 0}) ++black;
                if (p == fusion::Rgb8{255, 255, 255}) ++white;
                img.pixels.push_back(p);
            }
            if (args.render) fusion::write_rgb_png(out_path(frame_name("render_", j, ".png")), img);
        }
        stats << j << ',' << seq.thermal->frames[j].t_ns << ',' << raw.width() << ','
              << raw.height() << ',' << finite.size() << ',' << num(mn) << ',' << num(mx) << ','
              << num(mean) << ',' << num(lo) << ',' << num(hi) << ',' << below << ',' << above
              << ',' << black << ',' << white << '\n';
    }
    stats.flush();
    if (!stats) throw std::runtime_error("cannot write " + out_path("thermal_stats.csv"));
    std::cout << "thermal_export: " << n << " thermal frames from " << args.manifest << " -> "
              << args.out_dir << "\n";
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
        std::cerr << "thermal_export: " << e.what << "\n\n" << kUsage;
        return 2;
    }
    try {
        return run(args);
    } catch (const std::exception& e) {
        std::cerr << "thermal_export: error: " << e.what() << "\n";
        return 1;
    }
}
