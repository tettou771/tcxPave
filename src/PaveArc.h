#pragma once

// =============================================================================
// PaveArc.h - Elliptical arc segment functions
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveData.h"
#include <array>

namespace pave {
namespace Arc {

// =============================================================================
// Center parameterization - convert SVG arc to center form
// =============================================================================

struct CenterParams {
    Vec2 center;
    Vec2 radii;
    float startAngle;  // degrees
    float endAngle;    // degrees
    float xAxisRotation;  // degrees
    bool sweep;
};

// Convert SVG arc to center parameterization
inline CenterParams toCenterParams(const Segment& arc) {
    const Vec2& start = arc.start;
    const Vec2& point = arc.point;
    Vec2 radii = arc.radii;
    float xAxisRotation = arc.xAxisRotation;
    bool largeArcFlag = arc.largeArcFlag;
    bool sweepFlag = arc.sweepFlag;

    // Handle zero radii (treat as line)
    if (scalar::approx(radii.x, 0) || scalar::approx(radii.y, 0)) {
        Vec2 v = vec2::sub(start, point);
        float rot = vec2::angle(v);
        float rx = vec2::len(v) / 2.0f;
        return {
            vec2::lerp(start, point, 0.5f),
            Vec2(rx, 0),
            0, 180,
            rot,
            true
        };
    }

    float xAxisRotationRad = scalar::rad(xAxisRotation);
    float cosphi = std::cos(xAxisRotationRad);
    float sinphi = std::sin(xAxisRotationRad);

    // Step 1: Compute (x1', y1')
    float x1p = (cosphi * (start.x - point.x)) / 2.0f + (sinphi * (start.y - point.y)) / 2.0f;
    float y1p = (-sinphi * (start.x - point.x)) / 2.0f + (cosphi * (start.y - point.y)) / 2.0f;

    // Correct radii if too small
    float rx = std::abs(radii.x);
    float ry = std::abs(radii.y);
    float A = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry);
    if (A > 1) {
        rx *= std::sqrt(A);
        ry *= std::sqrt(A);
    }

    float rx2 = rx * rx;
    float ry2 = ry * ry;

    // Step 2: Compute (cx', cy')
    float n = rx2 * ry2 - rx2 * y1p * y1p - ry2 * x1p * x1p;
    float d = rx2 * y1p * y1p + ry2 * x1p * x1p;

    float sign = (largeArcFlag != sweepFlag) ? 1.0f : -1.0f;
    float sq = std::abs(n / d);
    float cxp = sign * std::sqrt(sq) * (rx * y1p) / ry;
    float cyp = sign * std::sqrt(sq) * (-ry * x1p) / rx;

    // Step 3: Compute (cx, cy)
    float cx = cosphi * cxp - sinphi * cyp + (start.x + point.x) / 2.0f;
    float cy = sinphi * cxp + cosphi * cyp + (start.y + point.y) / 2.0f;

    // Step 4: Compute angles
    float ax = (x1p - cxp) / rx;
    float ay = (y1p - cyp) / ry;
    float bx = (-x1p - cxp) / rx;
    float by = (-y1p - cyp) / ry;

    float startAngle = scalar::deg(std::atan2(ay, ax));
    float deltaAngle = scalar::deg(std::atan2(by, bx) - std::atan2(ay, ax));

    if (!sweepFlag && deltaAngle > 0) {
        deltaAngle -= 360.0f;
    } else if (sweepFlag && deltaAngle < 0) {
        deltaAngle += 360.0f;
    }

    float endAngle = startAngle + deltaAngle;

    return {
        Vec2(cx, cy),
        Vec2(rx, ry),
        startAngle,
        endAngle,
        xAxisRotation,
        deltaAngle > 0
    };
}

// =============================================================================
// Length
// =============================================================================

inline float ellipticArcLength(const Vec2& radii, float startAngle, float endAngle) {
    float rx = radii.x;
    float ry = radii.y;

    if (scalar::approx(rx, ry)) {
        // Circle
        return scalar::rad(std::abs(endAngle - startAngle)) * rx;
    }

    // Ellipse - numerical integration
    float angleBetween = std::abs(endAngle - startAngle);
    int steps = std::max(1, (int)std::ceil(angleBetween / 0.25f));

    float startRad = scalar::rad(startAngle);
    float endRad = scalar::rad(endAngle);
    float deltaRad = scalar::rad(angleBetween) / steps;

    float len = 0.0f;
    for (int i = 0; i < steps; i++) {
        float a = scalar::lerp(startRad, endRad, (i + 0.5f) / steps);
        len += deltaRad * std::sqrt(
            (rx * std::sin(a)) * (rx * std::sin(a)) +
            (ry * std::cos(a)) * (ry * std::cos(a))
        );
    }
    return len;
}

inline float length(const Segment& arc) {
    CenterParams cp = toCenterParams(arc);
    return ellipticArcLength(cp.radii, cp.startAngle, cp.endAngle);
}

// =============================================================================
// Time conversion
// =============================================================================

// For circles, unit == time. For ellipses, use binary search
inline float unitToTime(const Segment& arc, float unit) {
    CenterParams cp = toCenterParams(arc);

    // For circle
    if (scalar::approx(cp.radii.x, cp.radii.y)) {
        return normalizeOffset(unit, 1.0f);
    }

    // For ellipse - binary search
    float targetUnit = normalizeOffset(unit, 1.0f);
    float totalLen = length(arc);
    float targetLen = targetUnit * totalLen;

    float lo = 0.0f, hi = 1.0f;
    float time = 0.5f;

    for (int i = 0; i < 16; i++) {
        time = (lo + hi) / 2.0f;
        float midAngle = scalar::lerp(cp.startAngle, cp.endAngle, time);
        float lenAtTime = ellipticArcLength(cp.radii, cp.startAngle, midAngle);

        if (lenAtTime < targetLen) {
            lo = time;
        } else {
            hi = time;
        }

        if (scalar::approx(lenAtTime, targetLen)) break;
    }

    return time;
}

inline float toTime(const Segment& arc, const SegmentLocation& loc) {
    if (std::holds_alternative<float>(loc)) {
        return unitToTime(arc, std::get<float>(loc));
    } else if (std::holds_alternative<UnitLoc>(loc)) {
        return unitToTime(arc, std::get<UnitLoc>(loc).unit);
    } else if (std::holds_alternative<TimeLoc>(loc)) {
        return normalizeOffset(std::get<TimeLoc>(loc).time, 1.0f);
    } else if (std::holds_alternative<OffsetLoc>(loc)) {
        float totalLen = length(arc);
        float unit = (totalLen > 0) ? std::get<OffsetLoc>(loc).offset / totalLen : 0;
        return unitToTime(arc, unit);
    }
    return 0;
}

// =============================================================================
// Parametric queries
// =============================================================================

inline Vec2 pointAt(const Segment& arc, const SegmentLocation& loc) {
    float time = toTime(arc, loc);
    CenterParams cp = toCenterParams(arc);

    float angle = scalar::lerp(cp.startAngle, cp.endAngle, time);
    Mat2d xform = Mat2d::trs(&cp.center, cp.xAxisRotation, cp.radii);

    return transformMat2d(vec2::direction(angle), xform);
}

inline Vec2 derivative(const Segment& arc, const SegmentLocation& loc) {
    float time = toTime(arc, loc);
    CenterParams cp = toCenterParams(arc);

    float angle = scalar::lerp(cp.startAngle, cp.endAngle, time);
    float dirAngle = angle + 90.0f * (cp.sweep ? 1.0f : -1.0f);

    Mat2d xform = Mat2d::trs(nullptr, cp.xAxisRotation, cp.radii);
    return transformMat2d(vec2::direction(dirAngle), xform);
}

inline Vec2 tangent(const Segment& arc, const SegmentLocation& loc) {
    return vec2::normalize(derivative(arc, loc));
}

inline Vec2 normal(const Segment& arc, const SegmentLocation& loc) {
    return vec2::rotate90(tangent(arc, loc));
}

inline Mat2d orientation(const Segment& arc, const SegmentLocation& loc) {
    Vec2 p = pointAt(arc, loc);
    Vec2 xAxis = tangent(arc, loc);
    Vec2 yAxis = vec2::rotate90(xAxis);
    return Mat2d(xAxis.x, xAxis.y, yAxis.x, yAxis.y, p.x, p.y);
}

// =============================================================================
// Bounding box
// =============================================================================

inline Rect bounds(const Segment& arc) {
    CenterParams cp = toCenterParams(arc);

    Rect rect = Rect::fromPoints(arc.start, arc.point);

    // Check if arc crosses cardinal directions
    auto crossesAngle = [&cp](float angle) {
        float start = cp.startAngle;
        float end = cp.endAngle;
        if (start == end) return false;

        // Normalize angle to [-180, 180]
        angle = std::fmod(angle + 180.0f, 360.0f) - 180.0f;

        if (start < end) {
            if (start < angle) return angle < end;
            return angle < end - 360.0f;
        } else {
            if (angle < start) return end < angle;
            return angle < end + 360.0f;
        }
    };

    Mat2d xform = Mat2d::trs(&cp.center, cp.xAxisRotation, cp.radii);

    // Calculate extreme angles
    float sy = cp.radii.y / cp.radii.x;
    float angleAtXmax = -scalar::atan(sy * scalar::sin(cp.xAxisRotation), scalar::cos(cp.xAxisRotation));
    float angleAtXmin = angleAtXmax + 180.0f;
    float angleAtYmax = -scalar::atan(-sy * scalar::cos(cp.xAxisRotation), scalar::sin(cp.xAxisRotation));
    float angleAtYmin = angleAtYmax + 180.0f;

    if (crossesAngle(angleAtXmax)) {
        rect.include(transformMat2d(vec2::direction(angleAtXmax), xform));
    }
    if (crossesAngle(angleAtXmin)) {
        rect.include(transformMat2d(vec2::direction(angleAtXmin), xform));
    }
    if (crossesAngle(angleAtYmax)) {
        rect.include(transformMat2d(vec2::direction(angleAtYmax), xform));
    }
    if (crossesAngle(angleAtYmin)) {
        rect.include(transformMat2d(vec2::direction(angleAtYmin), xform));
    }

    return rect;
}

// =============================================================================
// Trim
// =============================================================================

inline Segment trim(const Segment& arc, const SegmentLocation& fromLoc, const SegmentLocation& toLoc) {
    float t0 = toTime(arc, fromLoc);
    float t1 = toTime(arc, toLoc);

    CenterParams cp = toCenterParams(arc);

    Mat2d xform = Mat2d::trs(&cp.center, cp.xAxisRotation, cp.radii);

    float startAngle = scalar::lerp(cp.startAngle, cp.endAngle, t0);
    float endAngle = scalar::lerp(cp.startAngle, cp.endAngle, t1);

    bool largeArc = std::abs(endAngle - startAngle) > 180.0f;
    bool sweep = cp.sweep;
    if (t0 > t1) sweep = !sweep;

    return Segment::arc(
        transformMat2d(vec2::direction(startAngle), xform),
        transformMat2d(vec2::direction(endAngle), xform),
        cp.radii,
        cp.xAxisRotation,
        largeArc,
        sweep
    );
}

// =============================================================================
// Divide at times
// =============================================================================

inline std::vector<Vertex> divideAtTimes(const Segment& arc, const std::vector<float>& times) {
    CenterParams cp = toCenterParams(arc);

    std::vector<float> allTimes = {0.0f};
    allTimes.insert(allTimes.end(), times.begin(), times.end());
    allTimes.push_back(1.0f);

    Mat2d xform = Mat2d::trs(&cp.center, cp.xAxisRotation, cp.radii);

    std::vector<Vertex> vertices;

    for (size_t i = 0; i + 1 < allTimes.size(); i++) {
        float startAngle = scalar::lerp(cp.startAngle, cp.endAngle, allTimes[i]);
        float endAngle = scalar::lerp(cp.startAngle, cp.endAngle, allTimes[i + 1]);

        bool largeArc = std::abs(endAngle - startAngle) > 180.0f;

        Vec2 point = transformMat2d(vec2::direction(endAngle), xform);

        vertices.push_back(Vertex::arc(point, cp.radii, cp.xAxisRotation, largeArc, cp.sweep));
    }

    return vertices;
}

// =============================================================================
// Predicates
// =============================================================================

inline bool isZero(const Segment& arc) {
    return vec2::approx(arc.start, arc.point);
}

inline bool isStraight(const Segment& arc) {
    if (isZero(arc)) return true;
    return scalar::approx(arc.radii.x, 0) || scalar::approx(arc.radii.y, 0);
}

// =============================================================================
// Approximate arc with cubic beziers
// =============================================================================

inline std::vector<Vertex> approximateByCubicBeziers(const Segment& arc, float maxAngle = 90.0f) {
    maxAngle = (maxAngle == 0) ? 90.0f : std::abs(maxAngle);

    CenterParams cp = toCenterParams(arc);

    float angleBetween = std::abs(cp.endAngle - cp.startAngle);
    int n = (int)std::ceil(angleBetween / maxAngle);
    float delta = (cp.endAngle - cp.startAngle) / n;

    Mat2d xform = Mat2d::trs(&cp.center, cp.xAxisRotation, cp.radii);

    std::vector<Vertex> beziers;

    for (int i = 0; i < n; i++) {
        float a0 = cp.startAngle + i * delta;
        float a1 = cp.startAngle + (i + 1) * delta;

        Vec2 p0 = vec2::direction(a0);
        Vec2 p1 = vec2::direction(a1);

        float handleLen = (4.0f / 3.0f) * std::tan(scalar::rad(a1 - a0) / 4.0f);

        float dir = cp.sweep ? 1.0f : -1.0f;
        Vec2 c1 = vec2::scaleAndAdd(p0, vec2::direction(a0 + 90.0f * dir), handleLen);
        Vec2 c2 = vec2::scaleAndAdd(p1, vec2::direction(a1 - 90.0f * dir), handleLen);

        beziers.push_back(Vertex::cubic(
            transformMat2d(p1, xform),
            transformMat2d(c1, xform),
            transformMat2d(c2, xform)
        ));
    }

    return beziers;
}

// =============================================================================
// Offset
// =============================================================================

// Offset an arc by a distance (positive = outward, negative = inward)
// For circular arcs, this simply changes the radius
inline Path offset(const Segment& arc, float distance, float unarcAngle = 90.0f) {
    // Get radii from segment
    float rx = arc.radii.x;
    float ry = arc.radii.y;

    CenterParams center = toCenterParams(arc);

    if (scalar::approx(rx, ry)) {
        // Circular arc - simply adjust radius
        float newRadius = rx + distance;
        if (newRadius <= 0) {
            return Path{}; // Collapsed to nothing
        }

        // Offset points along radial direction
        Vec2 startDir = (arc.start - center.center).normalized();
        Vec2 endDir = (arc.point - center.center).normalized();

        Vec2 newStart = center.center + startDir * newRadius;
        Vec2 newEnd = center.center + endDir * newRadius;

        Curve curve;
        curve.vertices.push_back(Vertex::line(newStart));
        curve.vertices.push_back(Vertex::arc(newEnd, Vec2(newRadius, newRadius),
                                             arc.xAxisRotation, arc.largeArcFlag, arc.sweepFlag));
        curve.closed = false;

        return Path{{curve}};
    }

    // For elliptical arcs, approximate via beziers then offset
    // Simplified implementation - just convert to beziers
    auto beziers = approximateByCubicBeziers(arc);

    Path result;
    Curve curve;
    curve.vertices.push_back(Vertex::line(arc.start));
    for (const auto& bez : beziers) {
        // Each vertex in beziers is a cubic bezier vertex
        curve.vertices.push_back(bez);
    }
    curve.closed = false;
    result.curves.push_back(curve);

    return result;
}

} // namespace Arc
} // namespace pave
