#pragma once

// =============================================================================
// PaveTypes.h - Core types for Pave
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

// Include TrussC math types
#include <tcMath.h>
#include <cmath>
#include <functional>
#include <vector>
#include <optional>
#include <variant>

namespace pave {

// Use TrussC Vec2 directly
using Vec2 = trussc::Vec2;

// Constant for float comparison
constexpr float EPSILON = 1e-6f;

// Approximate equality check
inline bool approx(float a, float b, float eps = EPSILON) {
    return std::abs(a - b) < eps;
}

inline bool approx(const Vec2& a, const Vec2& b, float eps = EPSILON) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps);
}

// =============================================================================
// Mat2d - 2D affine transformation matrix (2x3)
// Layout: [a, b, c, d, tx, ty]
// | a  c  tx |
// | b  d  ty |
// =============================================================================
struct Mat2d {
    float a = 1.0f;   // scale x / rotation cos
    float b = 0.0f;   // rotation sin / skew
    float c = 0.0f;   // rotation -sin / skew
    float d = 1.0f;   // scale y / rotation cos
    float tx = 0.0f;  // translation x
    float ty = 0.0f;  // translation y

    // Constructors
    Mat2d() = default;
    Mat2d(float a_, float b_, float c_, float d_, float tx_, float ty_)
        : a(a_), b(b_), c(c_), d(d_), tx(tx_), ty(ty_) {}

    // Identity matrix
    static Mat2d identity() { return Mat2d(); }

    // Create from translation
    static Mat2d fromTranslation(const Vec2& t) {
        return Mat2d(1, 0, 0, 1, t.x, t.y);
    }
    static Mat2d fromTranslation(float x, float y) {
        return Mat2d(1, 0, 0, 1, x, y);
    }

    // Create from rotation (degrees)
    static Mat2d rotation(float degrees, const Vec2& origin = Vec2(0, 0)) {
        float rad = trussc::deg2rad(degrees);
        float cos_a = std::cos(rad);
        float sin_a = std::sin(rad);
        // Rotate around origin
        float tx = origin.x - cos_a * origin.x + sin_a * origin.y;
        float ty = origin.y - sin_a * origin.x - cos_a * origin.y;
        return Mat2d(cos_a, sin_a, -sin_a, cos_a, tx, ty);
    }

    // Create from scale
    static Mat2d fromScale(const Vec2& s) {
        return Mat2d(s.x, 0, 0, s.y, 0, 0);
    }
    static Mat2d fromScale(float sx, float sy) {
        return Mat2d(sx, 0, 0, sy, 0, 0);
    }

    // TRS: Translate * Rotate * Scale
    // origin is optional (null = no translation)
    static Mat2d trs(const Vec2* origin, float rotationDeg, const Vec2& scale) {
        float rad = trussc::deg2rad(rotationDeg);
        float cos_a = std::cos(rad);
        float sin_a = std::sin(rad);

        Mat2d m;
        m.a = cos_a * scale.x;
        m.b = sin_a * scale.x;
        m.c = -sin_a * scale.y;
        m.d = cos_a * scale.y;

        if (origin) {
            m.tx = origin->x;
            m.ty = origin->y;
        } else {
            m.tx = 0;
            m.ty = 0;
        }

        return m;
    }

    // Transform a point
    Vec2 transform(const Vec2& p) const {
        return Vec2(
            a * p.x + c * p.y + tx,
            b * p.x + d * p.y + ty
        );
    }

    // Matrix multiplication
    Mat2d operator*(const Mat2d& other) const {
        return Mat2d(
            a * other.a + c * other.b,
            b * other.a + d * other.b,
            a * other.c + c * other.d,
            b * other.c + d * other.d,
            a * other.tx + c * other.ty + tx,
            b * other.tx + d * other.ty + ty
        );
    }

    // Determinant
    float det() const {
        return a * d - b * c;
    }

    // Inverse
    std::optional<Mat2d> invert() const {
        float det_val = det();
        if (std::abs(det_val) < EPSILON) return std::nullopt;

        float inv_det = 1.0f / det_val;
        return Mat2d(
            d * inv_det,
            -b * inv_det,
            -c * inv_det,
            a * inv_det,
            (c * ty - d * tx) * inv_det,
            (b * tx - a * ty) * inv_det
        );
    }

    // Scale the matrix
    static Mat2d scale(const Mat2d& m, const Vec2& s) {
        return Mat2d(
            m.a * s.x, m.b * s.x,
            m.c * s.y, m.d * s.y,
            m.tx, m.ty
        );
    }

    // Rotate the matrix (degrees)
    static Mat2d rotate(const Mat2d& m, float degrees) {
        float rad = trussc::deg2rad(degrees);
        float cos_a = std::cos(rad);
        float sin_a = std::sin(rad);
        return Mat2d(
            m.a * cos_a + m.c * sin_a,
            m.b * cos_a + m.d * sin_a,
            m.c * cos_a - m.a * sin_a,
            m.d * cos_a - m.b * sin_a,
            m.tx, m.ty
        );
    }
};

// Transform vector with Mat2d
inline Vec2 transformMat2d(const Vec2& p, const Mat2d& m) {
    return m.transform(p);
}

// =============================================================================
// Rect - Axis-aligned bounding box as [min, max] points
// =============================================================================
struct Rect {
    Vec2 min;
    Vec2 max;

    Rect() : min(0, 0), max(0, 0) {}
    Rect(const Vec2& min_, const Vec2& max_) : min(min_), max(max_) {}
    Rect(float x1, float y1, float x2, float y2) : min(x1, y1), max(x2, y2) {}

    float width() const { return max.x - min.x; }
    float height() const { return max.y - min.y; }
    Vec2 center() const { return Vec2((min.x + max.x) / 2, (min.y + max.y) / 2); }

    // Create from two points (auto-orders min/max)
    static Rect fromPoints(const Vec2& p1, const Vec2& p2) {
        return Rect(
            std::min(p1.x, p2.x), std::min(p1.y, p2.y),
            std::max(p1.x, p2.x), std::max(p1.y, p2.y)
        );
    }

    // Unite multiple rects
    static Rect unite(const std::vector<Rect>& rects) {
        if (rects.empty()) return Rect();

        Rect result = rects[0];
        for (size_t i = 1; i < rects.size(); i++) {
            result.min.x = std::min(result.min.x, rects[i].min.x);
            result.min.y = std::min(result.min.y, rects[i].min.y);
            result.max.x = std::max(result.max.x, rects[i].max.x);
            result.max.y = std::max(result.max.y, rects[i].max.y);
        }
        return result;
    }

    // Expand to include a point
    Rect& include(const Vec2& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        return *this;
    }
};

// =============================================================================
// Circle utilities
// =============================================================================
namespace Circle {
    struct CircleData {
        Vec2 center;
        float radius;
    };

    // Calculate circumcircle of three points
    // Returns nullopt if points are collinear
    inline std::optional<CircleData> circumcircle(const Vec2& p1, const Vec2& p2, const Vec2& p3) {
        float ax = p2.x - p1.x;
        float ay = p2.y - p1.y;
        float bx = p3.x - p1.x;
        float by = p3.y - p1.y;

        float d = 2.0f * (ax * by - ay * bx);
        if (std::abs(d) < EPSILON) return std::nullopt;

        float aSq = ax * ax + ay * ay;
        float bSq = bx * bx + by * by;

        float cx = (by * aSq - ay * bSq) / d + p1.x;
        float cy = (ax * bSq - bx * aSq) / d + p1.y;

        Vec2 center(cx, cy);
        float radius = center.distance(p1);

        return CircleData{center, radius};
    }
}

// =============================================================================
// Scalar utilities (matching pave's scalar namespace)
// =============================================================================
namespace scalar {
    inline float rad(float deg) { return trussc::deg2rad(deg); }
    inline float deg(float rad) { return trussc::rad2deg(rad); }

    inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
    inline float invlerp(float a, float b, float v) {
        return (std::abs(b - a) < EPSILON) ? 0.0f : (v - a) / (b - a);
    }

    inline float clamp(float v, float min, float max) {
        return std::max(min, std::min(max, v));
    }

    inline float mod(float a, float b) {
        float result = std::fmod(a, b);
        return (result < 0) ? result + b : result;
    }

    inline bool approx(float a, float b, float eps = EPSILON) {
        return std::abs(a - b) < eps;
    }

    inline float sin(float deg) { return std::sin(rad(deg)); }
    inline float cos(float deg) { return std::cos(rad(deg)); }
    inline float tan(float deg) { return std::tan(rad(deg)); }
    inline float atan(float y, float x) { return deg(std::atan2(y, x)); }
}

// =============================================================================
// Vec2 utilities (matching pave's vec2 namespace)
// =============================================================================
namespace vec2 {
    const Vec2 zero(0, 0);

    inline float distance(const Vec2& a, const Vec2& b) { return a.distance(b); }
    inline float dist(const Vec2& a, const Vec2& b) { return a.distance(b); }
    inline float len(const Vec2& v) { return v.length(); }

    inline Vec2 add(const Vec2& a, const Vec2& b) { return a + b; }
    inline Vec2 sub(const Vec2& a, const Vec2& b) { return a - b; }
    inline Vec2 scale(const Vec2& v, float s) { return v * s; }
    inline Vec2 normalize(const Vec2& v) { return v.normalized(); }

    inline Vec2 scaleAndAdd(const Vec2& a, const Vec2& b, float s) {
        return a + b * s;
    }

    inline float dot(const Vec2& a, const Vec2& b) { return a.dot(b); }
    inline float cross(const Vec2& a, const Vec2& b) { return a.cross(b); }

    inline Vec2 lerp(const Vec2& a, const Vec2& b, float t) { return a.lerp(b, t); }

    // Angle of vector (degrees)
    inline float angle(const Vec2& v) { return trussc::rad2deg(v.angle()); }

    // Angle between two vectors (degrees)
    inline float angle(const Vec2& a, const Vec2& b) {
        return trussc::rad2deg(a.angle(b));
    }

    // Create unit vector from angle (degrees)
    inline Vec2 direction(float degrees) {
        return Vec2::fromAngle(trussc::deg2rad(degrees));
    }

    inline Vec2 dir(float degrees, float length = 1.0f, const Vec2& origin = zero) {
        return origin + Vec2::fromAngle(trussc::deg2rad(degrees), length);
    }

    // Rotate 90 degrees clockwise (for Y-down coordinate)
    inline Vec2 rotate90(const Vec2& v) { return Vec2(v.y, -v.x); }

    // Rotate by degrees
    inline Vec2 rotate(const Vec2& v, float degrees) {
        return v.rotated(trussc::deg2rad(degrees));
    }

    inline Vec2 transformMat2d(const Vec2& v, const Mat2d& m) {
        return m.transform(v);
    }

    inline bool approx(const Vec2& a, const Vec2& b, float eps = EPSILON) {
        return pave::approx(a, b, eps);
    }
}

// =============================================================================
// Distort type - Function that returns local transformation matrix
// =============================================================================
using Distort = std::function<Mat2d(const Vec2&)>;

} // namespace pave
