#pragma once

// =============================================================================
// PaveCurve.h - Curve functions (collection of segments)
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveSegment.h"

namespace pave {
namespace CurveFn {

// =============================================================================
// Segment access
// =============================================================================

// Get number of segments in curve
inline int segmentCount(const Curve& curve) {
    if (curve.vertices.empty()) return 0;
    return curve.closed ? (int)curve.vertices.size() : (int)curve.vertices.size() - 1;
}

// Get all segments from curve
inline std::vector<Segment> segments(const Curve& curve) {
    std::vector<Segment> segs;
    if (curve.vertices.empty()) return segs;

    Vec2 start = curve.vertices[0].point;

    for (size_t i = 1; i < curve.vertices.size(); i++) {
        segs.push_back(Segment(start, curve.vertices[i]));
        start = curve.vertices[i].point;
    }

    if (curve.closed && curve.vertices.size() > 1) {
        segs.push_back(Segment(start, curve.vertices[0]));
    }

    return segs;
}

// Get segment at index
inline Segment segment(const Curve& curve, int index) {
    auto segs = segments(curve);
    index = normalizeIndex(index, (int)segs.size());
    if (index >= 0 && index < (int)segs.size()) {
        return segs[index];
    }
    return Segment();
}

// Get neighbor segment (with offset)
inline std::optional<Segment> neighborSegment(const Curve& curve, int segmentIndex, int offset) {
    int segCount = segmentCount(curve);
    int index = segmentIndex + offset;

    if (curve.closed) {
        index = (int)scalar::mod((float)index, (float)segCount);
    }

    if (index < 0 || index >= segCount) {
        return std::nullopt;
    }

    return segment(curve, index);
}

inline std::optional<Segment> nextSegment(const Curve& curve, int segmentIndex) {
    return neighborSegment(curve, segmentIndex, 1);
}

inline std::optional<Segment> prevSegment(const Curve& curve, int segmentIndex) {
    return neighborSegment(curve, segmentIndex, -1);
}

// =============================================================================
// Properties
// =============================================================================

inline float length(const Curve& curve) {
    float len = 0;
    for (const Segment& seg : segments(curve)) {
        len += SegmentFn::length(seg);
    }
    return len;
}

inline Rect bounds(const Curve& curve) {
    auto segs = segments(curve);
    if (segs.empty()) return Rect();

    std::vector<Rect> rects;
    for (const Segment& seg : segs) {
        rects.push_back(SegmentFn::bounds(seg));
    }
    return Rect::unite(rects);
}

// =============================================================================
// Location conversion
// =============================================================================

struct TimeCurveLocationResult {
    Segment segment;
    int segmentIndex;
    float time;
};

inline TimeCurveLocationResult toTime(const Curve& curve, const CurveLocation& loc) {
    auto segs = segments(curve);
    if (segs.empty()) return {Segment(), 0, 0};

    // Handle float (unit)
    if (std::holds_alternative<float>(loc)) {
        float unit = std::get<float>(loc);
        float curveLen = length(curve);
        float targetLen = normalizeOffset(unit, 1.0f) * curveLen;

        float accLen = 0;
        for (int i = 0; i < (int)segs.size(); i++) {
            float segLen = SegmentFn::length(segs[i]);
            if (accLen + segLen >= targetLen || i == (int)segs.size() - 1) {
                float segOffset = targetLen - accLen;
                float time = SegmentFn::toTime(segs[i], OffsetLoc(segOffset));
                return {segs[i], i, time};
            }
            accLen += segLen;
        }
    }

    // Handle UnitCurveLoc
    if (std::holds_alternative<UnitCurveLoc>(loc)) {
        auto uloc = std::get<UnitCurveLoc>(loc);
        if (uloc.segmentIndex) {
            int idx = normalizeIndex(*uloc.segmentIndex, (int)segs.size());
            float time = SegmentFn::toTime(segs[idx], uloc.unit);
            return {segs[idx], idx, time};
        }
        return toTime(curve, uloc.unit);
    }

    // Handle OffsetCurveLoc
    if (std::holds_alternative<OffsetCurveLoc>(loc)) {
        auto oloc = std::get<OffsetCurveLoc>(loc);
        if (oloc.segmentIndex) {
            int idx = normalizeIndex(*oloc.segmentIndex, (int)segs.size());
            float time = SegmentFn::toTime(segs[idx], OffsetLoc(oloc.offset));
            return {segs[idx], idx, time};
        }
        float curveLen = length(curve);
        float offset = normalizeOffset(oloc.offset, curveLen);
        return toTime(curve, offset / curveLen);
    }

    // Handle TimeCurveLoc
    if (std::holds_alternative<TimeCurveLoc>(loc)) {
        auto tloc = std::get<TimeCurveLoc>(loc);
        if (tloc.segmentIndex) {
            int idx = normalizeIndex(*tloc.segmentIndex, (int)segs.size());
            return {segs[idx], idx, normalizeOffset(tloc.time, 1.0f)};
        }
        float extendedTime = normalizeOffset(tloc.time, 1.0f) * segs.size();
        int segIdx = std::min((int)std::floor(extendedTime), (int)segs.size() - 1);
        float time = extendedTime - segIdx;
        return {segs[segIdx], segIdx, time};
    }

    return {Segment(), 0, 0};
}

// =============================================================================
// Parametric queries
// =============================================================================

inline Vec2 pointAt(const Curve& curve, const CurveLocation& loc) {
    auto [seg, segIdx, time] = toTime(curve, loc);
    return SegmentFn::pointAt(seg, TimeLoc(time));
}

inline Vec2 tangent(const Curve& curve, const CurveLocation& loc) {
    auto [seg, segIdx, time] = toTime(curve, loc);
    return SegmentFn::tangent(seg, TimeLoc(time));
}

inline Vec2 normal(const Curve& curve, const CurveLocation& loc) {
    auto [seg, segIdx, time] = toTime(curve, loc);
    return SegmentFn::normal(seg, TimeLoc(time));
}

inline Mat2d orientation(const Curve& curve, const CurveLocation& loc) {
    auto [seg, segIdx, time] = toTime(curve, loc);
    return SegmentFn::orientation(seg, TimeLoc(time));
}

// =============================================================================
// Modifications
// =============================================================================

inline Curve reverse(const Curve& curve) {
    std::vector<Vertex> vertices;
    auto segs = segments(curve);

    int numVertex = (int)curve.vertices.size();
    int iStart = curve.closed ? numVertex : numVertex - 1;

    for (int i = iStart; i >= 0; i--) {
        if (!curve.closed && i == numVertex - 1) {
            vertices.push_back(Vertex::line(curve.vertices[i].point));
            continue;
        }

        Vec2 point = curve.vertices[i % numVertex].point;
        const Vertex& nextVert = curve.vertices[(i + 1) % numVertex];

        if (nextVert.command == Command::L) {
            vertices.push_back(Vertex::line(point));
        } else if (nextVert.command == Command::C) {
            // Reverse control points
            vertices.push_back(Vertex::cubic(point, nextVert.control2, nextVert.control1));
        } else {
            // Arc - flip sweep flag
            vertices.push_back(Vertex::arc(point, nextVert.radii, nextVert.xAxisRotation,
                                           nextVert.largeArcFlag, !nextVert.sweepFlag));
        }
    }

    return Curve(vertices, curve.closed);
}

inline Curve close(const Curve& curve, bool fuse = true) {
    if (curve.vertices.empty()) return curve;

    if (fuse) {
        const Vec2& first = curve.vertices.front().point;
        const Vec2& last = curve.vertices.back().point;

        if (vec2::approx(first, last)) {
            std::vector<Vertex> vertices = {curve.vertices.back()};
            vertices.insert(vertices.end(), curve.vertices.begin() + 1, curve.vertices.end());
            return Curve(vertices, true);
        }
    }

    return Curve(curve.vertices, true);
}

// =============================================================================
// Predicates
// =============================================================================

inline bool isZero(const Curve& curve) {
    auto segs = segments(curve);
    if (segs.empty()) return true;
    for (const Segment& seg : segs) {
        if (!SegmentFn::isZero(seg)) return false;
    }
    return true;
}

} // namespace CurveFn
} // namespace pave
