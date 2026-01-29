#pragma once

// =============================================================================
// PavePath.h - Path functions (collection of curves)
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveCurve.h"

namespace pave {
namespace PathFn {

// =============================================================================
// Primitives - Creating paths
// =============================================================================

// Empty path
inline Path empty() {
    return Path();
}

// Line segment
inline Path line(const Vec2& start, const Vec2& end) {
    Curve curve;
    curve.vertices.push_back(Vertex::line(start));
    curve.vertices.push_back(Vertex::line(end));
    curve.closed = false;
    return Path({curve});
}

// Rectangle
inline Path rectangle(const Vec2& start, const Vec2& end) {
    Curve curve;
    curve.vertices.push_back(Vertex::line(start));
    curve.vertices.push_back(Vertex::line(Vec2(end.x, start.y)));
    curve.vertices.push_back(Vertex::line(end));
    curve.vertices.push_back(Vertex::line(Vec2(start.x, end.y)));
    curve.closed = true;
    return Path({curve});
}

inline Path rect(const Vec2& start, const Vec2& end) {
    return rectangle(start, end);
}

inline Path rectFromCenter(const Vec2& center, const Vec2& size) {
    Vec2 halfSize = size * 0.5f;
    return rectangle(center - halfSize, center + halfSize);
}

// Circle
inline Path circle(const Vec2& center, float radius) {
    Vec2 radii(radius, radius);
    Vec2 right(center.x + radius, center.y);
    Vec2 left(center.x - radius, center.y);

    Curve curve;
    curve.vertices.push_back(Vertex::arc(right, radii, 0, false, true));
    curve.vertices.push_back(Vertex::arc(left, radii, 0, false, true));
    curve.closed = true;

    return Path({curve});
}

// Ellipse
inline Path ellipse(const Vec2& center, const Vec2& radius) {
    Vec2 right(center.x + radius.x, center.y);
    Vec2 left(center.x - radius.x, center.y);

    Curve curve;
    curve.vertices.push_back(Vertex::arc(right, radius, 0, false, true));
    curve.vertices.push_back(Vertex::arc(left, radius, 0, false, true));
    curve.closed = true;

    return Path({curve});
}

// Arc
inline Path arc(const Vec2& center, float radius, float startAngle, float endAngle) {
    Vec2 radii(radius, radius);
    bool sweep = endAngle > startAngle;
    float angleBetween = std::abs(endAngle - startAngle);

    // Break into segments if > 180 degrees
    float maxStep = 360.0f - 0.0001f;
    float step = std::min(maxStep, std::abs(angleBetween));

    std::vector<Vertex> vertices;
    vertices.push_back(Vertex::line(vec2::dir(startAngle, radius, center)));

    float currentAngle = startAngle;
    while (std::abs(endAngle - currentAngle) > 0.001f) {
        float nextAngle = currentAngle + (sweep ? step : -step);
        if ((sweep && nextAngle > endAngle) || (!sweep && nextAngle < endAngle)) {
            nextAngle = endAngle;
        }

        bool largeArc = std::abs(nextAngle - currentAngle) > 180.0f;
        vertices.push_back(Vertex::arc(
            vec2::dir(nextAngle, radius, center),
            radii, 0, largeArc, sweep
        ));

        currentAngle = nextAngle;
    }

    return Path({Curve(vertices, false)});
}

// Regular polygon
inline Path regularPolygon(const Vec2& center, float radius, int sides) {
    if (sides < 3) sides = 3;

    std::vector<Vertex> vertices;
    for (int i = 0; i < sides; i++) {
        float angle = 360.0f * i / sides - 90.0f;  // Start from top
        vertices.push_back(Vertex::line(vec2::dir(angle, radius, center)));
    }

    return Path({Curve(vertices, true)});
}

// Polyline (open)
inline Path polyline(const std::vector<Vec2>& points) {
    if (points.empty()) return Path();

    std::vector<Vertex> vertices;
    for (const Vec2& p : points) {
        vertices.push_back(Vertex::line(p));
    }

    return Path({Curve(vertices, false)});
}

// Polygon (closed)
inline Path polygon(const std::vector<Vec2>& points) {
    if (points.empty()) return Path();

    std::vector<Vertex> vertices;
    for (const Vec2& p : points) {
        vertices.push_back(Vertex::line(p));
    }

    return Path({Curve(vertices, true)});
}

// Cubic bezier
inline Path cubicBezier(const Vec2& start, const Vec2& c1, const Vec2& c2, const Vec2& end) {
    std::vector<Vertex> vertices;
    vertices.push_back(Vertex::line(start));
    vertices.push_back(Vertex::cubic(end, c1, c2));

    return Path({Curve(vertices, false)});
}

// Quadratic bezier
inline Path quadraticBezier(const Vec2& start, const Vec2& control, const Vec2& end) {
    // Convert to cubic bezier
    Vec2 c1 = vec2::lerp(start, control, 2.0f / 3.0f);
    Vec2 c2 = vec2::lerp(end, control, 2.0f / 3.0f);
    return cubicBezier(start, c1, c2, end);
}

// =============================================================================
// Properties
// =============================================================================

inline float length(const Path& path) {
    float len = 0;
    for (const Curve& curve : path.curves) {
        len += CurveFn::length(curve);
    }
    return len;
}

inline Rect bounds(const Path& path) {
    if (path.curves.empty()) return Rect();

    std::vector<Rect> rects;
    for (const Curve& curve : path.curves) {
        rects.push_back(CurveFn::bounds(curve));
    }
    return Rect::unite(rects);
}

// =============================================================================
// Location conversion
// =============================================================================

struct TimePathLocationResult {
    Segment segment;
    int curveIndex;
    int segmentIndex;
    float time;
};

inline TimePathLocationResult toTime(const Path& path, const PathLocation& loc) {
    if (path.curves.empty()) return {Segment(), 0, 0, 0};

    // Calculate curve lengths
    std::vector<float> curveLengths;
    float totalLength = 0;
    for (const Curve& curve : path.curves) {
        float len = CurveFn::length(curve);
        curveLengths.push_back(len);
        totalLength += len;
    }

    // Handle float (unit)
    if (std::holds_alternative<float>(loc)) {
        float unit = normalizeOffset(std::get<float>(loc), 1.0f);
        float targetLen = unit * totalLength;

        float accLen = 0;
        for (int ci = 0; ci < (int)path.curves.size(); ci++) {
            if (accLen + curveLengths[ci] >= targetLen || ci == (int)path.curves.size() - 1) {
                float curveOffset = targetLen - accLen;
                float curveUnit = (curveLengths[ci] > 0) ? curveOffset / curveLengths[ci] : 0;
                auto result = CurveFn::toTime(path.curves[ci], curveUnit);
                return {result.segment, ci, result.segmentIndex, result.time};
            }
            accLen += curveLengths[ci];
        }
    }

    // Handle UnitPathLoc, OffsetPathLoc, TimePathLoc similarly...
    // For simplicity, convert them to unit first
    float unit = 0;
    if (std::holds_alternative<UnitPathLoc>(loc)) {
        unit = std::get<UnitPathLoc>(loc).unit;
    } else if (std::holds_alternative<OffsetPathLoc>(loc)) {
        unit = (totalLength > 0) ? std::get<OffsetPathLoc>(loc).offset / totalLength : 0;
    } else if (std::holds_alternative<TimePathLoc>(loc)) {
        unit = std::get<TimePathLoc>(loc).time;
    }

    return toTime(path, unit);
}

// =============================================================================
// Parametric queries
// =============================================================================

inline Vec2 pointAt(const Path& path, const PathLocation& loc) {
    auto result = toTime(path, loc);
    return SegmentFn::pointAt(result.segment, TimeLoc(result.time));
}

inline Vec2 tangent(const Path& path, const PathLocation& loc) {
    auto result = toTime(path, loc);
    return SegmentFn::tangent(result.segment, TimeLoc(result.time));
}

inline Vec2 normal(const Path& path, const PathLocation& loc) {
    auto result = toTime(path, loc);
    return SegmentFn::normal(result.segment, TimeLoc(result.time));
}

inline Mat2d orientation(const Path& path, const PathLocation& loc) {
    auto result = toTime(path, loc);
    return SegmentFn::orientation(result.segment, TimeLoc(result.time));
}

// =============================================================================
// Transformations
// =============================================================================

// Transform path with matrix
inline Path transform(const Path& path, const Mat2d& matrix) {
    Path result;

    for (const Curve& curve : path.curves) {
        Curve newCurve;
        newCurve.closed = curve.closed;

        for (const Vertex& v : curve.vertices) {
            Vertex newV = v;
            newV.point = matrix.transform(v.point);

            if (v.command == Command::C) {
                newV.control1 = matrix.transform(v.control1);
                newV.control2 = matrix.transform(v.control2);
            } else if (v.command == Command::A) {
                // Arc transformation is more complex
                // For now, approximate - full implementation would need Arc::transform
                newV.radii = Vec2(v.radii.x * std::abs(matrix.a),
                                  v.radii.y * std::abs(matrix.d));
            }

            newCurve.vertices.push_back(newV);
        }

        result.curves.push_back(newCurve);
    }

    return result;
}

// Reverse path
inline Path reverse(const Path& path) {
    Path result;
    for (auto it = path.curves.rbegin(); it != path.curves.rend(); ++it) {
        result.curves.push_back(CurveFn::reverse(*it));
    }
    return result;
}

// Close all curves in path
inline Path close(const Path& path, bool fuse = true) {
    Path result;
    for (const Curve& curve : path.curves) {
        result.curves.push_back(CurveFn::close(curve, fuse));
    }
    return result;
}

// Join multiple paths into one
inline Path join(const std::vector<Path>& paths) {
    Path result;
    for (const Path& p : paths) {
        result.curves.insert(result.curves.end(), p.curves.begin(), p.curves.end());
    }
    return result;
}

// =============================================================================
// Flatten - Convert curves to line segments
// =============================================================================

inline Path flatten(const Path& path, float flatness = 1.0f) {
    Path result;

    for (const Curve& curve : path.curves) {
        Curve newCurve;
        newCurve.closed = curve.closed;

        auto segs = CurveFn::segments(curve);

        if (!segs.empty()) {
            newCurve.vertices.push_back(Vertex::line(segs[0].start));
        }

        for (const Segment& seg : segs) {
            if (seg.command == Command::L) {
                newCurve.vertices.push_back(Vertex::line(seg.point));
            } else {
                // Subdivide curves
                int steps = std::max(2, (int)(SegmentFn::length(seg) / flatness));
                for (int i = 1; i <= steps; i++) {
                    float t = (float)i / steps;
                    newCurve.vertices.push_back(Vertex::line(SegmentFn::pointAt(seg, TimeLoc(t))));
                }
            }
        }

        result.curves.push_back(newCurve);
    }

    return result;
}

// =============================================================================
// Resample - Uniform vertex spacing
// =============================================================================

inline Path resample(const Path& path, float interval) {
    Path result;

    for (const Curve& curve : path.curves) {
        float curveLen = CurveFn::length(curve);
        if (curveLen <= 0) continue;

        int steps = std::max(2, (int)(curveLen / interval));

        Curve newCurve;
        newCurve.closed = curve.closed;

        for (int i = 0; i <= steps; i++) {
            float t = (float)i / steps;
            newCurve.vertices.push_back(Vertex::line(CurveFn::pointAt(curve, t)));
        }

        result.curves.push_back(newCurve);
    }

    return result;
}

// =============================================================================
// SVG Output
// =============================================================================

inline std::string toSVGString(const Path& path, int precision = 4) {
    std::string d;

    auto formatNum = [precision](float n) -> std::string {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", precision, n);
        // Remove trailing zeros
        std::string s(buf);
        size_t dot = s.find('.');
        if (dot != std::string::npos) {
            size_t last = s.find_last_not_of('0');
            if (last > dot) {
                s = s.substr(0, last + 1);
            } else {
                s = s.substr(0, dot);
            }
        }
        return s;
    };

    for (const Curve& curve : path.curves) {
        if (curve.vertices.empty()) continue;

        // Move to first point
        d += "M" + formatNum(curve.vertices[0].point.x) + ","
                 + formatNum(curve.vertices[0].point.y);

        for (size_t i = 1; i < curve.vertices.size(); i++) {
            const Vertex& v = curve.vertices[i];

            if (v.command == Command::L) {
                d += "L" + formatNum(v.point.x) + "," + formatNum(v.point.y);
            } else if (v.command == Command::C) {
                d += "C" + formatNum(v.control1.x) + "," + formatNum(v.control1.y) + ","
                         + formatNum(v.control2.x) + "," + formatNum(v.control2.y) + ","
                         + formatNum(v.point.x) + "," + formatNum(v.point.y);
            } else if (v.command == Command::A) {
                d += "A" + formatNum(v.radii.x) + "," + formatNum(v.radii.y) + ","
                         + formatNum(v.xAxisRotation) + ","
                         + (v.largeArcFlag ? "1" : "0") + ","
                         + (v.sweepFlag ? "1" : "0") + ","
                         + formatNum(v.point.x) + "," + formatNum(v.point.y);
            }
        }

        if (curve.closed) {
            d += "Z";
        }
    }

    return d;
}

} // namespace PathFn
} // namespace pave
