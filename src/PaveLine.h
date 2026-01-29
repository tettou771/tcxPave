#pragma once

// =============================================================================
// PaveLine.h - Line segment functions
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveData.h"

namespace pave {
namespace Line {

// =============================================================================
// Properties
// =============================================================================

// Bounding box of line segment
inline Rect bounds(const Vec2& start, const Vec2& point) {
    return Rect::fromPoints(start, point);
}

inline Rect bounds(const Segment& seg) {
    return bounds(seg.start, seg.point);
}

// Length of line segment
inline float length(const Vec2& start, const Vec2& point) {
    return vec2::distance(start, point);
}

inline float length(const Segment& seg) {
    return length(seg.start, seg.point);
}

// =============================================================================
// Parametric queries
// =============================================================================

// Convert location to time [0, 1]
inline float toTime(const Vec2& start, const Vec2& point, const SegmentLocation& loc) {
    float t = 0;

    if (std::holds_alternative<float>(loc)) {
        t = std::get<float>(loc);
    } else if (std::holds_alternative<UnitLoc>(loc)) {
        t = std::get<UnitLoc>(loc).unit;
    } else if (std::holds_alternative<TimeLoc>(loc)) {
        t = std::get<TimeLoc>(loc).time;
    } else if (std::holds_alternative<OffsetLoc>(loc)) {
        float len = length(start, point);
        t = (len > 0) ? std::get<OffsetLoc>(loc).offset / len : 0;
    }

    return normalizeOffset(t, 1.0f);
}

inline float toTime(const Segment& seg, const SegmentLocation& loc) {
    return toTime(seg.start, seg.point, loc);
}

// Point at location
inline Vec2 pointAt(const Vec2& start, const Vec2& point, const SegmentLocation& loc) {
    float t = toTime(start, point, loc);
    return vec2::lerp(start, point, t);
}

inline Vec2 pointAt(const Segment& seg, const SegmentLocation& loc) {
    return pointAt(seg.start, seg.point, loc);
}

// Derivative (tangent direction, not normalized)
inline Vec2 derivative(const Vec2& start, const Vec2& point) {
    return vec2::sub(point, start);
}

inline Vec2 derivative(const Segment& seg) {
    return derivative(seg.start, seg.point);
}

// Tangent (normalized derivative)
inline Vec2 tangent(const Vec2& start, const Vec2& point) {
    return vec2::normalize(derivative(start, point));
}

inline Vec2 tangent(const Segment& seg) {
    return tangent(seg.start, seg.point);
}

// Normal (perpendicular to tangent, rotated 90 degrees clockwise in Y-down)
inline Vec2 normal(const Vec2& start, const Vec2& point) {
    return vec2::rotate90(tangent(start, point));
}

inline Vec2 normal(const Segment& seg) {
    return normal(seg.start, seg.point);
}

// Curvature (always 0 for lines)
constexpr float curvature = 0.0f;

// Orientation matrix at location
inline Mat2d orientation(const Vec2& start, const Vec2& point, const SegmentLocation& loc) {
    Vec2 p = pointAt(start, point, loc);
    Vec2 xAxis = tangent(start, point);
    Vec2 yAxis = vec2::rotate90(xAxis);
    return Mat2d(xAxis.x, xAxis.y, yAxis.x, yAxis.y, p.x, p.y);
}

inline Mat2d orientation(const Segment& seg, const SegmentLocation& loc) {
    return orientation(seg.start, seg.point, loc);
}

// =============================================================================
// Modifications
// =============================================================================

// Trim line segment between two locations
inline Segment trim(const Vec2& start, const Vec2& point,
                    const SegmentLocation& fromLoc, const SegmentLocation& toLoc) {
    float t0 = toTime(start, point, fromLoc);
    float t1 = toTime(start, point, toLoc);

    Vec2 newStart = vec2::lerp(start, point, t0);
    Vec2 newEnd = vec2::lerp(start, point, t1);

    return Segment::line(newStart, newEnd);
}

inline Segment trim(const Segment& seg,
                    const SegmentLocation& fromLoc, const SegmentLocation& toLoc) {
    return trim(seg.start, seg.point, fromLoc, toLoc);
}

// Divide line at multiple times
inline std::vector<Vertex> divideAtTimes(const Vec2& start, const Vec2& point,
                                          const std::vector<float>& times) {
    std::vector<Vertex> vertices;
    for (float t : times) {
        vertices.push_back(Vertex::line(vec2::lerp(start, point, t)));
    }
    vertices.push_back(Vertex::line(point));  // Always include end point
    return vertices;
}

inline std::vector<Vertex> divideAtTimes(const Segment& seg,
                                          const std::vector<float>& times) {
    return divideAtTimes(seg.start, seg.point, times);
}

// =============================================================================
// Predicates
// =============================================================================

// Check if line segment is zero-length
inline bool isZero(const Vec2& start, const Vec2& point) {
    return vec2::approx(start, point);
}

inline bool isZero(const Segment& seg) {
    return isZero(seg.start, seg.point);
}

// =============================================================================
// Offset
// =============================================================================

// Offset line by distance (perpendicular)
inline Path offset(const Vec2& start, const Vec2& point, float distance) {
    Vec2 n = normal(start, point);
    Vec2 offsetVec = n * distance;

    Curve curve;
    curve.vertices.push_back(Vertex::line(start + offsetVec));
    curve.vertices.push_back(Vertex::line(point + offsetVec));
    curve.closed = false;

    return Path({curve});
}

inline Path offset(const Segment& seg, float distance) {
    return offset(seg.start, seg.point, distance);
}

} // namespace Line
} // namespace pave
