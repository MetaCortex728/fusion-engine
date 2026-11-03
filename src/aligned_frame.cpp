#include "fusion/aligned_frame.hpp"

#include <charconv>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <system_error>
#include <unordered_map>
#include <utility>

#include "fusion/lidar.hpp"

namespace fusion {

namespace {

// Moves the scan to the camera's time when the trajectory covers both
// times; otherwise returns the points unchanged.
std::pair<std::vector<Eigen::Vector3d>, bool> points_at(const FrameInputs& in, TimeNs t_camera_ns) {
    const LidarScan& scan = *in.scan;
    if (in.trajectory != nullptr && in.trajectory->contains(scan.t_ns) &&
        in.trajectory->contains(t_camera_ns)) {
        return {compensate_scan(scan.points, *in.trajectory, in.T_body_lidar, scan.t_ns,
                                t_camera_ns),
                true};
    }
    return {scan.points, false};
}

// Marks, for every projected point, whether it holds the depth map's value
// at its pixel. Follows SparseDepthMap: the smallest depth wins and on a tie
// the first point stays.
void mark_nearest(std::vector<PointInCamera>& points, int width) {
    std::unordered_map<std::size_t, std::size_t> owner;  // pixel -> point index
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& proj = points[i].projection;
        if (!proj) continue;
        const std::size_t pixel = static_cast<std::size_t>(proj->row) * static_cast<std::size_t>(width) +
                                  static_cast<std::size_t>(proj->col);
        const auto [it, inserted] = owner.emplace(pixel, i);
        if (!inserted && proj->depth < points[it->second].projection->depth) it->second = i;
    }
    for (const auto& [pixel, i] : owner) {
        (void)pixel;
        points[i].nearest = true;
    }
}

CameraFrame camera_frame(const FrameInputs& in, const CameraInput& cam, bool thermal) {
    const std::string which = thermal ? "thermal" : "rgb";
    if (cam.config == nullptr) {
        throw std::invalid_argument("build_aligned_frame: " + which + " camera has no config");
    }
    if (cam.association.t_ref_ns != in.scan->t_ns) {
        throw std::invalid_argument("build_aligned_frame: " + which +
                                    " association is for reference time " +
                                    std::to_string(cam.association.t_ref_ns) + ", the scan is at " +
                                    std::to_string(in.scan->t_ns));
    }
    CameraFrame out;
    out.name = cam.config->name;
    out.association = cam.association;
    if (!cam.association.accepted) return out;

    const CameraModel model = make_camera_model(*cam.config);
    const Eigen::Isometry3d& T_camera_lidar = cam.config->T_sensor_source;
    auto [points, compensated] = points_at(in, cam.association.t_match_ns);
    out.compensated = compensated;
    out.points.resize(points.size());

    if (thermal) {
        if (cam.thermal_image == nullptr) {
            throw std::invalid_argument(
                "build_aligned_frame: accepted thermal camera has no raw image");
        }
        // Throws std::invalid_argument for an image of the wrong size.
        ThermalProjection tp = project_to_thermal(model, T_camera_lidar, points, *cam.thermal_image);
        for (const ThermalSample& s : tp.samples) {
            out.points[s.point_index].projection = s.projection;
            out.points[s.point_index].raw = s.raw;
        }
        out.projected_count = tp.samples.size();
        out.depth = std::move(tp.depth);
    } else {
        SparseDepthMap depth(model.width(), model.height());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (auto proj = project_point(model, T_camera_lidar, points[i])) {
                depth.insert(proj->col, proj->row, proj->depth);
                out.points[i].projection = *proj;
                ++out.projected_count;
            }
        }
        out.depth = std::move(depth);
    }
    mark_nearest(out.points, model.width());
    return out;
}

template <typename T>
void put(std::ostream& out, T value) {
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), value);
    if (r.ec != std::errc()) throw std::runtime_error("write_aligned_frame_csv: number format");
    out.write(buf, r.ptr - buf);
}

void put_camera(std::ostream& out, const std::optional<CameraFrame>& cam, std::size_t i,
                bool thermal) {
    const PointInCamera* p = nullptr;
    if (cam && i < cam->points.size() && cam->points[i].projection) p = &cam->points[i];
    if (p == nullptr) {
        out << (thermal ? ",,,,," : ",,,,");
        return;
    }
    out << ',';
    put(out, p->projection->pixel.x());
    out << ',';
    put(out, p->projection->pixel.y());
    out << ',';
    put(out, p->projection->depth);
    if (thermal) {
        out << ',';
        if (p->raw) put(out, static_cast<unsigned>(*p->raw));
    }
    out << ',' << (p->nearest ? '1' : '0');
}

}  // namespace

AlignedFrame build_aligned_frame(const FrameInputs& inputs) {
    if (inputs.scan == nullptr) throw std::invalid_argument("build_aligned_frame: no scan");
    const LidarScan& scan = *inputs.scan;
    if (scan.intensity.size() != scan.points.size()) {
        throw std::invalid_argument("build_aligned_frame: " + std::to_string(scan.points.size()) +
                                    " points but " + std::to_string(scan.intensity.size()) +
                                    " intensities");
    }
    AlignedFrame frame;
    frame.t_ref_ns = scan.t_ns;
    frame.points = scan.points;
    frame.intensity = scan.intensity;
    if (inputs.rgb) frame.rgb = camera_frame(inputs, *inputs.rgb, false);
    if (inputs.thermal) frame.thermal = camera_frame(inputs, *inputs.thermal, true);
    return frame;
}

void write_aligned_frame_csv(std::ostream& out, const AlignedFrame& frame) {
    out << "point,x,y,z,intensity,rgb_u,rgb_v,rgb_depth,rgb_nearest,"
           "thermal_u,thermal_v,thermal_depth,thermal_raw,thermal_nearest\n";
    for (std::size_t i = 0; i < frame.points.size(); ++i) {
        put(out, i);
        for (int k = 0; k < 3; ++k) {
            out << ',';
            put(out, frame.points[i][k]);
        }
        out << ',';
        if (i < frame.intensity.size()) put(out, frame.intensity[i]);
        put_camera(out, frame.rgb, i, false);
        put_camera(out, frame.thermal, i, true);
        out << '\n';
    }
}

void write_aligned_frame_csv(const std::string& path, const AlignedFrame& frame) {
    std::ofstream file(path);
    if (!file) throw std::runtime_error("fusion::write_aligned_frame_csv: cannot open " + path);
    write_aligned_frame_csv(file, frame);
    file.flush();
    if (!file) throw std::runtime_error("fusion::write_aligned_frame_csv: cannot write " + path);
}

}  // namespace fusion
