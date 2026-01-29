#pragma once

// =============================================================================
// PaveCubicBezier.h - Cubic Bezier segment functions
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveData.h"
#include <array>

namespace pave {
namespace CubicBezier {

// =============================================================================
// Creation
// =============================================================================

inline Segment of(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point) {
    return Segment::cubic(start, c1, c2, point);
}

// Convert quadratic bezier to cubic bezier
inline Segment fromQuadratic(const Vec2& start, const Vec2& control, const Vec2& point) {
    Vec2 c1 = vec2::lerp(start, control, 2.0f / 3.0f);
    Vec2 c2 = vec2::lerp(point, control, 2.0f / 3.0f);
    return Segment::cubic(start, c1, c2, point);
}

// =============================================================================
// Parametric queries
// =============================================================================

// Point at time t using de Casteljau's algorithm
inline Vec2 pointAtTime(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point, float t) {
    float u = 1.0f - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;

    Vec2 p;
    p.x = uuu * start.x + 3.0f * uu * t * c1.x + 3.0f * u * tt * c2.x + ttt * point.x;
    p.y = uuu * start.y + 3.0f * uu * t * c1.y + 3.0f * u * tt * c2.y + ttt * point.y;
    return p;
}

inline Vec2 pointAtTime(const Segment& seg, float t) {
    return pointAtTime(seg.start, seg.control1, seg.control2, seg.point, t);
}

// Derivative at time t
inline Vec2 derivativeAtTime(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point, float t) {
    // Derivative coefficients
    float ax = 9.0f * (c1.x - c2.x) + 3.0f * (point.x - start.x);
    float bx = 6.0f * (start.x + c2.x) - 12.0f * c1.x;
    float cx = 3.0f * (c1.x - start.x);

    float ay = 9.0f * (c1.y - c2.y) + 3.0f * (point.y - start.y);
    float by = 6.0f * (start.y + c2.y) - 12.0f * c1.y;
    float cy = 3.0f * (c1.y - start.y);

    float dx = (ax * t + bx) * t + cx;
    float dy = (ay * t + by) * t + cy;

    return Vec2(dx, dy);
}

inline Vec2 derivativeAtTime(const Segment& seg, float t) {
    return derivativeAtTime(seg.start, seg.control1, seg.control2, seg.point, t);
}

// Tangent at time t (normalized derivative)
inline Vec2 tangentAtTime(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point, float t) {
    return vec2::normalize(derivativeAtTime(start, c1, c2, point, t));
}

inline Vec2 tangentAtTime(const Segment& seg, float t) {
    return tangentAtTime(seg.start, seg.control1, seg.control2, seg.point, t);
}

// Normal at time t (perpendicular to tangent)
inline Vec2 normalAtTime(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point, float t) {
    return vec2::rotate90(tangentAtTime(start, c1, c2, point, t));
}

inline Vec2 normalAtTime(const Segment& seg, float t) {
    return normalAtTime(seg.start, seg.control1, seg.control2, seg.point, t);
}

// =============================================================================
// Length calculation using Gaussian quadrature
// =============================================================================

namespace detail {
    // 5-point Gaussian quadrature
    constexpr float gaussWeights[5] = {
        0.5688888888888889f,
        0.4786286704993665f, 0.4786286704993665f,
        0.2369268850561891f, 0.2369268850561891f
    };
    constexpr float gaussAbscissae[5] = {
        0.0f,
        -0.5384693101056831f, 0.5384693101056831f,
        -0.9061798459386640f, 0.9061798459386640f
    };
}

inline float length(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point) {
    // Gaussian quadrature for arc length
    float sum = 0.0f;
    for (int i = 0; i < 5; i++) {
        float t = 0.5f * (1.0f + detail::gaussAbscissae[i]);
        Vec2 d = derivativeAtTime(start, c1, c2, point, t);
        sum += detail::gaussWeights[i] * vec2::len(d);
    }
    return sum * 0.5f;
}

inline float length(const Segment& seg) {
    return length(seg.start, seg.control1, seg.control2, seg.point);
}

// =============================================================================
// Bounding box
// =============================================================================

inline Rect bounds(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point) {
    // Start with endpoints
    Rect rect = Rect::fromPoints(start, point);

    // Find extrema by solving derivative = 0
    // For x: 3at^2 + 2bt + c = 0
    auto solveQuadratic = [](float a, float b, float c) -> std::pair<float, float> {
        if (std::abs(a) < EPSILON) {
            // Linear case
            if (std::abs(b) < EPSILON) return {-1, -1};
            float t = -c / b;
            return {t, -1};
        }
        float disc = b * b - 4.0f * a * c;
        if (disc < 0) return {-1, -1};
        float sqrtDisc = std::sqrt(disc);
        return {(-b + sqrtDisc) / (2.0f * a), (-b - sqrtDisc) / (2.0f * a)};
    };

    // X extrema
    {
        float a = 3.0f * (-start.x + 3.0f * c1.x - 3.0f * c2.x + point.x);
        float b = 6.0f * (start.x - 2.0f * c1.x + c2.x);
        float c = 3.0f * (c1.x - start.x);
        auto [t1, t2] = solveQuadratic(a, b, c);

        if (t1 > 0 && t1 < 1) {
            Vec2 p = pointAtTime(start, c1, c2, point, t1);
            rect.include(p);
        }
        if (t2 > 0 && t2 < 1) {
            Vec2 p = pointAtTime(start, c1, c2, point, t2);
            rect.include(p);
        }
    }

    // Y extrema
    {
        float a = 3.0f * (-start.y + 3.0f * c1.y - 3.0f * c2.y + point.y);
        float b = 6.0f * (start.y - 2.0f * c1.y + c2.y);
        float c = 3.0f * (c1.y - start.y);
        auto [t1, t2] = solveQuadratic(a, b, c);

        if (t1 > 0 && t1 < 1) {
            Vec2 p = pointAtTime(start, c1, c2, point, t1);
            rect.include(p);
        }
        if (t2 > 0 && t2 < 1) {
            Vec2 p = pointAtTime(start, c1, c2, point, t2);
            rect.include(p);
        }
    }

    return rect;
}

inline Rect bounds(const Segment& seg) {
    return bounds(seg.start, seg.control1, seg.control2, seg.point);
}

// =============================================================================
// Time conversion (unit/offset to time)
// =============================================================================

inline float toTime(const Segment& seg, const SegmentLocation& loc) {
    if (std::holds_alternative<float>(loc)) {
        return normalizeOffset(std::get<float>(loc), 1.0f);
    } else if (std::holds_alternative<UnitLoc>(loc)) {
        // Unit to time requires arc length parameterization
        // Approximate using binary search
        float targetUnit = normalizeOffset(std::get<UnitLoc>(loc).unit, 1.0f);
        float totalLen = length(seg);
        float targetLen = targetUnit * totalLen;

        float lo = 0.0f, hi = 1.0f;
        for (int i = 0; i < 16; i++) {
            float mid = (lo + hi) / 2.0f;
            // Calculate length from 0 to mid
            float sum = 0.0f;
            for (int j = 0; j < 5; j++) {
                float t = 0.5f * mid * (1.0f + detail::gaussAbscissae[j]);
                Vec2 d = derivativeAtTime(seg, t);
                sum += detail::gaussWeights[j] * vec2::len(d);
            }
            float lenAtMid = sum * 0.5f * mid;

            if (lenAtMid < targetLen) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return (lo + hi) / 2.0f;
    } else if (std::holds_alternative<TimeLoc>(loc)) {
        return normalizeOffset(std::get<TimeLoc>(loc).time, 1.0f);
    } else if (std::holds_alternative<OffsetLoc>(loc)) {
        float totalLen = length(seg);
        float targetLen = normalizeOffset(std::get<OffsetLoc>(loc).offset, totalLen);
        float targetUnit = (totalLen > 0) ? targetLen / totalLen : 0;
        return toTime(seg, UnitLoc(targetUnit));
    }
    return 0;
}

// Point at location
inline Vec2 pointAt(const Segment& seg, const SegmentLocation& loc) {
    float t = toTime(seg, loc);
    return pointAtTime(seg, t);
}

// Derivative at location
inline Vec2 derivative(const Segment& seg, const SegmentLocation& loc) {
    float t = toTime(seg, loc);
    return derivativeAtTime(seg, t);
}

// Tangent at location
inline Vec2 tangent(const Segment& seg, const SegmentLocation& loc) {
    float t = toTime(seg, loc);
    return tangentAtTime(seg, t);
}

// Normal at location
inline Vec2 normal(const Segment& seg, const SegmentLocation& loc) {
    float t = toTime(seg, loc);
    return normalAtTime(seg, t);
}

// Orientation matrix at location
inline Mat2d orientation(const Segment& seg, const SegmentLocation& loc) {
    Vec2 p = pointAt(seg, loc);
    Vec2 xAxis = tangent(seg, loc);
    Vec2 yAxis = vec2::rotate90(xAxis);
    return Mat2d(xAxis.x, xAxis.y, yAxis.x, yAxis.y, p.x, p.y);
}

// =============================================================================
// Splitting using de Casteljau's algorithm
// =============================================================================

// Split bezier at time t, returns [newC1, inControl, midPoint, outControl, newC2]
inline std::array<Vec2, 5> splitAtTime(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point, float t) {
    Vec2 newC1 = vec2::lerp(start, c1, t);
    Vec2 p12 = vec2::lerp(c1, c2, t);
    Vec2 newC2 = vec2::lerp(c2, point, t);

    Vec2 inControl = vec2::lerp(newC1, p12, t);
    Vec2 outControl = vec2::lerp(p12, newC2, t);

    Vec2 midPoint = vec2::lerp(inControl, outControl, t);

    return {newC1, inControl, midPoint, outControl, newC2};
}

// Trim between two times (must be 0 <= from <= to <= 1)
inline std::array<Vec2, 4> trimBetweenTimes(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& point,
                                             float from, float to) {
    Vec2 newStart, midC1, midC2;

    if (from == 0) {
        newStart = start;
        midC1 = c1;
        midC2 = c2;
    } else {
        auto [_, __, ns, mc1, mc2] = splitAtTime(start, c1, c2, point, from);
        newStart = ns;
        midC1 = mc1;
        midC2 = mc2;
    }

    Vec2 newC1, newC2, newEnd;

    if (to == 1) {
        newEnd = point;
        newC1 = midC1;
        newC2 = midC2;
    } else {
        float relTo = scalar::invlerp(from, 1.0f, to);
        auto [nc1, nc2, ne, _, __] = splitAtTime(newStart, midC1, midC2, point, relTo);
        newC1 = nc1;
        newC2 = nc2;
        newEnd = ne;
    }

    return {newStart, newC1, newC2, newEnd};
}

// =============================================================================
// Trim
// =============================================================================

inline Segment trim(const Segment& seg, const SegmentLocation& fromLoc, const SegmentLocation& toLoc) {
    float t0 = toTime(seg, fromLoc);
    float t1 = toTime(seg, toLoc);

    bool flip = t0 > t1;
    if (flip) std::swap(t0, t1);

    auto [newStart, newC1, newC2, newEnd] = trimBetweenTimes(
        seg.start, seg.control1, seg.control2, seg.point, t0, t1);

    if (flip) {
        std::swap(newStart, newEnd);
        std::swap(newC1, newC2);
    }

    return Segment::cubic(newStart, newC1, newC2, newEnd);
}

// =============================================================================
// Divide at times
// =============================================================================

inline std::vector<Vertex> divideAtTimes(const Segment& seg, const std::vector<float>& times) {
    std::vector<float> allTimes = {0.0f};
    allTimes.insert(allTimes.end(), times.begin(), times.end());
    allTimes.push_back(1.0f);

    std::vector<Vertex> vertices;

    for (size_t i = 0; i + 1 < allTimes.size(); i++) {
        auto [_, c1, c2, p] = trimBetweenTimes(
            seg.start, seg.control1, seg.control2, seg.point,
            allTimes[i], allTimes[i + 1]);
        vertices.push_back(Vertex::cubic(p, c1, c2));
    }

    return vertices;
}

// =============================================================================
// Predicates
// =============================================================================

inline bool isZero(const Segment& seg) {
    return vec2::approx(seg.start, seg.point) &&
           vec2::approx(seg.start, seg.control1) &&
           vec2::approx(seg.start, seg.control2);
}

inline bool isStraight(const Segment& seg) {
    if (isZero(seg)) return true;

    // Both handles at endpoints
    if (vec2::approx(seg.start, seg.control1) && vec2::approx(seg.point, seg.control2)) {
        return true;
    }

    // Zero-length line with handles
    if (vec2::approx(seg.start, seg.point)) {
        return true;
    }

    // Check if control points are on the line between start and end
    Vec2 sp = vec2::sub(seg.point, seg.start);
    Vec2 sc1 = vec2::sub(seg.control1, seg.start);
    Vec2 sc2 = vec2::sub(seg.control2, seg.start);

    float spLen = vec2::len(sp);

    // All control points must be on the line and within the segment
    float angle1 = vec2::angle(sp, sc1);
    float angle2 = vec2::angle(sp, sc2);

    return scalar::approx(angle1, 0) && scalar::approx(angle2, 0) &&
           vec2::len(sc1) < spLen && vec2::len(sc2) < spLen;
}

// =============================================================================
// Offset (simplified - returns approximation using multiple beziers)
// =============================================================================

// This is a simplified offset that doesn't handle all cases perfectly
// For proper offset, consider using a more sophisticated algorithm
inline Path offset(const Segment& seg, float distance) {
    // Simple approach: offset the control points and endpoints
    // This is not geometrically correct but works for small distances

    // Sample points and normals
    const int samples = 8;
    std::vector<Vec2> offsetPoints;

    for (int i = 0; i <= samples; i++) {
        float t = (float)i / samples;
        Vec2 p = pointAtTime(seg, t);
        Vec2 n = normalAtTime(seg, t);
        offsetPoints.push_back(p + n * distance);
    }

    // Fit cubic beziers to the offset points
    // Simplified: just return a polyline approximation
    Curve curve;
    for (const Vec2& p : offsetPoints) {
        curve.vertices.push_back(Vertex::line(p));
    }
    curve.closed = false;

    return Path({curve});
}

} // namespace CubicBezier
} // namespace pave
