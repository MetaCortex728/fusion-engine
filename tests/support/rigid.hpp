// Test-only 3D vector and rigid transform types for the synthetic scene.
//
// Eigen is the intended linear algebra dependency, but it was not available
// in the environment where this support code was written, so the tests use
// this minimal stand-in. It is not part of the fusion library's API.
#pragma once

#include <cmath>

namespace fusion::testing {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(double s, const Vec3& v) { return {s * v.x, s * v.y, s * v.z}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) { return (1.0 / norm(v)) * v; }

// Row-major 3x3 matrix.
struct Mat3 {
    double m[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

    static Mat3 identity() { return {}; }

    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
            }
        }
        return r;
    }

    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = m[j][i];
            }
        }
        return r;
    }
};

inline Mat3 rot_x(double a) {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[1][1] = c; r.m[1][2] = -s;
    r.m[2][1] = s; r.m[2][2] = c;
    return r;
}

inline Mat3 rot_y(double a) {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0][0] = c;  r.m[0][2] = s;
    r.m[2][0] = -s; r.m[2][2] = c;
    return r;
}

inline Mat3 rot_z(double a) {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0][0] = c; r.m[0][1] = -s;
    r.m[1][0] = s; r.m[1][1] = c;
    return r;
}

// Rotation from roll (about x), pitch (about y), yaw (about z): R = Rz * Ry * Rx.
inline Mat3 rot_rpy(double roll, double pitch, double yaw) {
    return rot_z(yaw) * rot_y(pitch) * rot_x(roll);
}

// Rigid transform. A variable named T_target_source maps a point expressed in
// the `source` frame into the `target` frame: p_target = T_target_source * p_source.
struct Rigid3 {
    Mat3 R;
    Vec3 t;

    static Rigid3 identity() { return {}; }

    Vec3 operator*(const Vec3& p) const { return R * p + t; }

    // T_a_c = T_a_b * T_b_c
    Rigid3 operator*(const Rigid3& o) const { return {R * o.R, R * o.t + t}; }

    // Rotation only, for directions.
    Vec3 rotate(const Vec3& v) const { return R * v; }

    // T_source_target from T_target_source.
    Rigid3 inverse() const {
        const Mat3 Rt = R.transposed();
        return {Rt, -1.0 * (Rt * t)};
    }
};

}  // namespace fusion::testing
