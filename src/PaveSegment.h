#pragma once

// =============================================================================
// PaveSegment.h - Unified segment functions (dispatch to Line/Cubic/Arc)
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveLine.h"
#include "PaveCubicBezier.h"
#include "PaveArc.h"

namespace pave {
namespace SegmentFn {

// =============================================================================
// Length
// =============================================================================

inline float length(const Segment& seg) {
    switch (seg.command) {
        case Command::L: return Line::length(seg);
        case Command::C: return CubicBezier::length(seg);
        case Command::A: return Arc::length(seg);
    }
    return 0;
}

// =============================================================================
// Bounds
// =============================================================================

inline Rect bounds(const Segment& seg) {
    switch (seg.command) {
        case Command::L: return Line::bounds(seg);
        case Command::C: return CubicBezier::bounds(seg);
        case Command::A: return Arc::bounds(seg);
    }
    return Rect();
}

// =============================================================================
// Parametric queries
// =============================================================================

inline Vec2 pointAt(const Segment& seg, const SegmentLocation& loc) {
    switch (seg.command) {
        case Command::L: return Line::pointAt(seg, loc);
        case Command::C: return CubicBezier::pointAt(seg, loc);
        case Command::A: return Arc::pointAt(seg, loc);
    }
    return Vec2();
}

inline Vec2 derivative(const Segment& seg, const SegmentLocation& loc) {
    switch (seg.command) {
        case Command::L: return Line::derivative(seg);
        case Command::C: return CubicBezier::derivative(seg, loc);
        case Command::A: return Arc::derivative(seg, loc);
    }
    return Vec2();
}

inline Vec2 tangent(const Segment& seg, const SegmentLocation& loc) {
    Vec2 d = derivative(seg, loc);
    return vec2::normalize(d);
}

inline Vec2 normal(const Segment& seg, const SegmentLocation& loc) {
    return vec2::rotate90(tangent(seg, loc));
}

inline Mat2d orientation(const Segment& seg, const SegmentLocation& loc) {
    Vec2 p = pointAt(seg, loc);
    Vec2 xAxis = tangent(seg, loc);
    Vec2 yAxis = vec2::rotate90(xAxis);
    return Mat2d(xAxis.x, xAxis.y, yAxis.x, yAxis.y, p.x, p.y);
}

// =============================================================================
// Time conversion
// =============================================================================

inline float toTime(const Segment& seg, const SegmentLocation& loc) {
    switch (seg.command) {
        case Command::L: return Line::toTime(seg, loc);
        case Command::C: return CubicBezier::toTime(seg, loc);
        case Command::A: return Arc::toTime(seg, loc);
    }
    return 0;
}

// =============================================================================
// Trim
// =============================================================================

inline Segment trim(const Segment& seg, const SegmentLocation& from, const SegmentLocation& to) {
    switch (seg.command) {
        case Command::L: return Line::trim(seg, from, to);
        case Command::C: return CubicBezier::trim(seg, from, to);
        case Command::A: return Arc::trim(seg, from, to);
    }
    return seg;
}

// =============================================================================
// Divide at times
// =============================================================================

inline std::vector<Vertex> divideAtTimes(const Segment& seg, const std::vector<float>& times) {
    switch (seg.command) {
        case Command::L: return Line::divideAtTimes(seg, times);
        case Command::C: return CubicBezier::divideAtTimes(seg, times);
        case Command::A: return Arc::divideAtTimes(seg, times);
    }
    return {};
}

// =============================================================================
// Predicates
// =============================================================================

inline bool isZero(const Segment& seg) {
    switch (seg.command) {
        case Command::L: return Line::isZero(seg);
        case Command::C: return CubicBezier::isZero(seg);
        case Command::A: return Arc::isZero(seg);
    }
    return false;
}

inline bool isStraight(const Segment& seg) {
    switch (seg.command) {
        case Command::L: return true;
        case Command::C: return CubicBezier::isStraight(seg);
        case Command::A: return Arc::isStraight(seg);
    }
    return true;
}

// =============================================================================
// Offset
// =============================================================================

inline Path offset(const Segment& seg, float distance, float unarcAngle = 90.0f) {
    switch (seg.command) {
        case Command::L: return Line::offset(seg, distance);
        case Command::C: return CubicBezier::offset(seg, distance);
        case Command::A: return Arc::offset(seg, distance, unarcAngle);
    }
    return Path();
}

} // namespace SegmentFn
} // namespace pave
