#include "viewer.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <system_error>

#include "viewer_page.hpp"

namespace fs = std::filesystem;

namespace fuse_cli {

namespace {

// Shortest text that reads back to the same double; JSON has no NaN or
// infinity, so those become null.
std::string num(double v) {
    if (!std::isfinite(v)) return "null";
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, r.ptr);
}

// Fixed decimals, for the per-point arrays (keeps the page small). "-0.00"
// becomes "0.00" so the output does not depend on the sign of tiny values.
std::string fixed(double v, int decimals) {
    if (!std::isfinite(v)) return "null";
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    std::string s(buf);
    if (s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos) s.erase(0, 1);
    return s;
}

// JSON string literal. '<', '>' and '&' are escaped too, so the text can
// never close the <script> element it is embedded in.
std::string json_string(const std::string& s) {
    std::string r = "\"";
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        switch (c) {
            case '"': r += "\\\""; break;
            case '\\': r += "\\\\"; break;
            case '\n': r += "\\n"; break;
            case '\r': r += "\\r"; break;
            case '\t': r += "\\t"; break;
            default:
                if (c < 0x20 || c == '<' || c == '>' || c == '&') {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    r += buf;
                } else {
                    r += ch;
                }
        }
    }
    return r + "\"";
}

std::string image_name(const char* prefix, std::size_t index) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%s_%06zu.png", prefix, index);
    return buf;
}

// Points file of the k-th frame of the page, relative to the page.
std::string points_file_name(std::size_t k) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "points/frame_%06zu.js", k);
    return buf;
}

std::string array(const std::vector<double>& v) {
    std::string s = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) s += ',';
        s += num(v[i]);
    }
    return s + "]";
}

std::string stats_json(const Stats& s) {
    return "{\"median\":" + num(s.median) + ",\"p95\":" + num(s.p95) + "}";
}

std::string range_json(double lo, double hi) {
    if (!(lo <= hi)) return "null";
    return "[" + num(lo) + "," + num(hi) + "]";
}

}  // namespace

std::vector<std::size_t> subsample_ranks(std::size_t m, std::size_t max_points) {
    std::vector<std::size_t> r;
    if (m <= max_points) {
        r.resize(m);
        for (std::size_t i = 0; i < m; ++i) r[i] = i;
        return r;
    }
    r.reserve(max_points);
    for (std::size_t k = 0; k < max_points; ++k) {
        // k * m < max_points * m; both are point counts, so the product of
        // two counts far below 2^32 fits in 64 bits.
        r.push_back(static_cast<std::size_t>(static_cast<std::uint64_t>(k) *
                                             static_cast<std::uint64_t>(m) /
                                             static_cast<std::uint64_t>(max_points)));
    }
    return r;
}

ViewerWriter::ViewerWriter(const std::string& out_dir, const fusion::Sequence& seq,
                           const std::string& manifest, std::size_t max_points)
    : dir_((fs::path(out_dir) / "viewer").string()),
      index_path_((fs::path(out_dir) / "viewer" / "index.html").string()),
      max_points_(max_points),
      depth_lo_(std::numeric_limits<double>::infinity()),
      depth_hi_(-std::numeric_limits<double>::infinity()),
      thermal_lo_(std::numeric_limits<double>::infinity()),
      thermal_hi_(-std::numeric_limits<double>::infinity()) {
    if (max_points_ == 0) throw std::invalid_argument("viewer: max_points must be positive");
    std::error_code ec;
    fs::create_directories(fs::path(dir_) / "points", ec);
    if (ec || !fs::is_directory(fs::path(dir_) / "points")) {
        throw std::runtime_error("cannot create viewer directory " + dir_);
    }
    out_.open(index_path_, std::ios::binary);
    if (!out_) throw std::runtime_error("cannot write " + index_path_);
    thermal_celsius_ = seq.thermal && seq.thermal->config.thermal.has_value();

    out_ << kViewerPageHead;
    out_ << "{\"format\":2,\"sequence\":" << json_string(manifest) << ",\"cameras\":{";
    bool first = true;
    for (const auto& [key, stream] : {std::pair{"rgb", &seq.rgb}, {"thermal", &seq.thermal}}) {
        if (!*stream) continue;
        const fusion::SensorConfig& c = (*stream)->config;
        out_ << (first ? "" : ",") << '"' << key << "\":{\"name\":" << json_string(c.name)
             << ",\"width\":" << c.width << ",\"height\":" << c.height << '}';
        first = false;
    }
    out_ << "},\"thermal_unit\":" << json_string(thermal_celsius_ ? "\xC2\xB0" "C" : "raw")
         << ",\"max_points_per_camera\":" << max_points_ << ",\"frames\":[\n";
}

void ViewerWriter::write_camera(const char* key, const std::optional<fusion::CameraFrame>& cam,
                                const fusion::AlignedFrame& frame, const fusion::RgbImage* image,
                                std::string& points_json) {
    out_ << ",\"" << key << "\":";
    points_json += std::string(points_json.empty() ? "{" : ",") + '"' + key + "\":";
    if (!cam) {
        out_ << "null";
        points_json += "null";
        return;
    }
    const fusion::Association& a = cam->association;
    std::string image_ref = "null";
    if (a.accepted && image) {
        const bool rgb = std::string(key) == "rgb";
        std::set<std::size_t>& written = rgb ? written_rgb_ : written_thermal_;
        const std::string name = image_name(key, a.match_index);
        if (written.insert(a.match_index).second) {
            fusion::write_rgb_png((fs::path(dir_) / name).string(), *image);
        }
        image_ref = json_string(name);
    }

    // Visible points in scan order, then the deterministic subsample.
    std::vector<std::size_t> visible;
    std::size_t hidden = 0;
    for (std::size_t i = 0; i < cam->points.size(); ++i) {
        const fusion::PointInCamera& p = cam->points[i];
        if (!p.projection) continue;
        visible.push_back(i);
        if (!p.nearest) ++hidden;
        depth_lo_ = std::min(depth_lo_, p.projection->depth);
        depth_hi_ = std::max(depth_hi_, p.projection->depth);
    }
    const std::vector<std::size_t> ranks = subsample_ranks(visible.size(), max_points_);

    // The point's thermal value (°C, or the raw count without a conversion),
    // when the thermal camera sees it.
    const auto thermal_value = [&](std::size_t i) -> std::optional<double> {
        if (!frame.thermal || i >= frame.thermal->points.size()) return std::nullopt;
        const fusion::PointInCamera& t = frame.thermal->points[i];
        if (thermal_celsius_) return t.celsius;
        if (t.raw) return static_cast<double>(*t.raw);
        return std::nullopt;
    };

    std::string idx, u, v, depth, nearest, thermal;
    for (std::size_t k = 0; k < ranks.size(); ++k) {
        const std::size_t i = visible[ranks[k]];
        const fusion::PointInCamera& p = cam->points[i];
        const char* sep = k ? "," : "";
        idx += sep + std::to_string(i);
        u += sep + fixed(p.projection->pixel.x(), 2);
        v += sep + fixed(p.projection->pixel.y(), 2);
        depth += sep + fixed(p.projection->depth, 3);
        nearest += sep + std::string(p.nearest ? "1" : "0");
        const std::optional<double> t = thermal_value(i);
        thermal += sep + (t ? fixed(*t, 2) : std::string("null"));
    }
    // The thermal range comes from the thermal camera's own points, so it
    // covers every value either view can show.
    if (std::string(key) == "thermal") {
        for (std::size_t i : visible) {
            if (const auto t = thermal_value(i)) {
                thermal_lo_ = std::min(thermal_lo_, *t);
                thermal_hi_ = std::max(thermal_hi_, *t);
            }
        }
    }

    out_ << "{\"name\":" << json_string(cam->name) << ",\"accepted\":" << (a.accepted ? "true" : "false")
         << ",\"match_index\":" << a.match_index << ",\"t_match_ns\":\"" << a.t_match_ns << '"'
         << ",\"delta_t_ns\":" << a.delta_t_ns
         << ",\"compensated\":" << (cam->compensated ? "true" : "false")
         << ",\"projected\":" << cam->projected_count << ",\"hidden\":" << hidden
         << ",\"embedded\":" << ranks.size()
         << ",\"subsampled\":" << (ranks.size() < visible.size() ? "true" : "false")
         << ",\"image\":" << image_ref << '}';
    points_json += "{\"i\":[" + idx + "],\"u\":[" + u + "],\"v\":[" + v + "],\"depth\":[" + depth +
                   "],\"nearest\":[" + nearest + "],\"thermal\":[" + thermal + "]}";
}

void ViewerWriter::add_frame(std::size_t index, const fusion::AlignedFrame& frame,
                             const fusion::RgbImage* rgb,
                             const fusion::RgbImage* thermal_display) {
    // Times as strings: epoch nanoseconds exceed the 2^53 integers a
    // JavaScript number holds exactly.
    const std::string points_name = points_file_name(frames_);
    out_ << (frames_ ? ",\n" : "") << "{\"index\":" << index << ",\"t_ref_ns\":\""
         << frame.t_ref_ns << "\",\"points\":" << frame.points.size()
         << ",\"points_file\":" << json_string(points_name);
    std::string points_json;
    write_camera("rgb", frame.rgb, frame, rgb, points_json);
    write_camera("thermal", frame.thermal, frame, thermal_display, points_json);
    out_ << '}';

    // The frame's points, as a script the page loads with a <script> element
    // when it shows the frame. The JSON holds only keys, numbers and null.
    const std::string points_path = (fs::path(dir_) / points_name).string();
    std::ofstream js(points_path, std::ios::binary);
    js << "fusionPoints(" << frames_ << "," << points_json << "});\n";
    js.close();
    if (!js) throw std::runtime_error("cannot write " + points_path);
    ++frames_;
    if (!out_) throw std::runtime_error("cannot write " + index_path_);
}

void ViewerWriter::finish(const std::vector<double>& read_ms, const std::vector<double>& process_ms,
                          const std::vector<double>& write_ms, const Stats& read,
                          const Stats& process, const Stats& write) {
    out_ << "\n],\"run\":{\"frames\":" << frames_ << ",\"process_ms\":" << stats_json(process)
         << ",\"read_ms\":" << stats_json(read) << ",\"write_ms\":" << stats_json(write)
         << ",\"depth_range\":" << range_json(depth_lo_, depth_hi_)
         << ",\"thermal_range\":" << range_json(thermal_lo_, thermal_hi_)
         << ",\"frame_ms\":{\"read\":" << array(read_ms) << ",\"process\":" << array(process_ms)
         << ",\"write\":" << array(write_ms) << "}}}";
    out_ << kViewerPageTail;
    out_.close();
    if (!out_) throw std::runtime_error("cannot write " + index_path_);
}

}  // namespace fuse_cli
