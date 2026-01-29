#pragma once

// =============================================================================
// PaveData.h - Path data structures
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveTypes.h"
#include "PaveLocation.h"
#include <vector>
#include <array>

namespace pave {

// =============================================================================
// Command types - Line, CubicBezier, Arc
// =============================================================================

enum class Command {
    L,  // Line
    C,  // Cubic Bezier
    A   // Arc
};

// =============================================================================
// CommandArgsC - Arguments for cubic Bezier (C) command
// Two control points
// =============================================================================
struct CommandArgsC {
    Vec2 control1;
    Vec2 control2;

    CommandArgsC() = default;
    CommandArgsC(const Vec2& c1, const Vec2& c2) : control1(c1), control2(c2) {}
};

// =============================================================================
// CommandArgsA - Arguments for arc (A) command
// SVG elliptical arc parameters
// =============================================================================
struct CommandArgsA {
    Vec2 radii;          // rx, ry
    float xAxisRotation; // degrees
    bool largeArcFlag;
    bool sweepFlag;

    CommandArgsA() : radii(0, 0), xAxisRotation(0), largeArcFlag(false), sweepFlag(false) {}
    CommandArgsA(const Vec2& r, float rot, bool large, bool sweep)
        : radii(r), xAxisRotation(rot), largeArcFlag(large), sweepFlag(sweep) {}
};

// =============================================================================
// Vertex - End point with command from previous vertex
// =============================================================================

// Line vertex
struct VertexL {
    Vec2 point;
    static constexpr Command command = Command::L;

    VertexL() = default;
    explicit VertexL(const Vec2& p) : point(p) {}
    VertexL(float x, float y) : point(x, y) {}
};

// Cubic Bezier vertex
struct VertexC {
    Vec2 point;
    CommandArgsC args;
    static constexpr Command command = Command::C;

    VertexC() = default;
    VertexC(const Vec2& p, const Vec2& c1, const Vec2& c2)
        : point(p), args(c1, c2) {}
};

// Arc vertex
struct VertexA {
    Vec2 point;
    CommandArgsA args;
    static constexpr Command command = Command::A;

    VertexA() = default;
    VertexA(const Vec2& p, const Vec2& radii, float xRot, bool large, bool sweep)
        : point(p), args(radii, xRot, large, sweep) {}
};

// =============================================================================
// Vertex - Union type for any vertex
// =============================================================================
struct Vertex {
    Vec2 point;
    Command command = Command::L;

    // For cubic bezier (C)
    Vec2 control1;
    Vec2 control2;

    // For arc (A)
    Vec2 radii;
    float xAxisRotation = 0;
    bool largeArcFlag = false;
    bool sweepFlag = false;

    Vertex() = default;

    // Line vertex
    static Vertex line(const Vec2& p) {
        Vertex v;
        v.point = p;
        v.command = Command::L;
        return v;
    }

    // Cubic bezier vertex
    static Vertex cubic(const Vec2& p, const Vec2& c1, const Vec2& c2) {
        Vertex v;
        v.point = p;
        v.command = Command::C;
        v.control1 = c1;
        v.control2 = c2;
        return v;
    }

    // Arc vertex
    static Vertex arc(const Vec2& p, const Vec2& r, float xRot, bool large, bool sweep) {
        Vertex v;
        v.point = p;
        v.command = Command::A;
        v.radii = r;
        v.xAxisRotation = xRot;
        v.largeArcFlag = large;
        v.sweepFlag = sweep;
        return v;
    }

    // Check command type
    bool isLine() const { return command == Command::L; }
    bool isCubic() const { return command == Command::C; }
    bool isArc() const { return command == Command::A; }
};

// =============================================================================
// Segment - Vertex with start point (complete curve segment)
// =============================================================================
struct Segment {
    Vec2 start;
    Vec2 point;
    Command command = Command::L;

    // For cubic bezier (C)
    Vec2 control1;
    Vec2 control2;

    // For arc (A)
    Vec2 radii;
    float xAxisRotation = 0;
    bool largeArcFlag = false;
    bool sweepFlag = false;

    Segment() = default;

    // Create from start point and vertex
    Segment(const Vec2& s, const Vertex& v)
        : start(s), point(v.point), command(v.command),
          control1(v.control1), control2(v.control2),
          radii(v.radii), xAxisRotation(v.xAxisRotation),
          largeArcFlag(v.largeArcFlag), sweepFlag(v.sweepFlag) {}

    // Line segment
    static Segment line(const Vec2& s, const Vec2& e) {
        Segment seg;
        seg.start = s;
        seg.point = e;
        seg.command = Command::L;
        return seg;
    }

    // Cubic bezier segment
    static Segment cubic(const Vec2& s, const Vec2& c1, const Vec2& c2, const Vec2& e) {
        Segment seg;
        seg.start = s;
        seg.point = e;
        seg.command = Command::C;
        seg.control1 = c1;
        seg.control2 = c2;
        return seg;
    }

    // Arc segment
    static Segment arc(const Vec2& s, const Vec2& e, const Vec2& r, float xRot, bool large, bool sweep) {
        Segment seg;
        seg.start = s;
        seg.point = e;
        seg.command = Command::A;
        seg.radii = r;
        seg.xAxisRotation = xRot;
        seg.largeArcFlag = large;
        seg.sweepFlag = sweep;
        return seg;
    }

    // Convert to vertex (loses start point)
    Vertex toVertex() const {
        Vertex v;
        v.point = point;
        v.command = command;
        v.control1 = control1;
        v.control2 = control2;
        v.radii = radii;
        v.xAxisRotation = xAxisRotation;
        v.largeArcFlag = largeArcFlag;
        v.sweepFlag = sweepFlag;
        return v;
    }

    // Check command type
    bool isLine() const { return command == Command::L; }
    bool isCubic() const { return command == Command::C; }
    bool isArc() const { return command == Command::A; }
};

// =============================================================================
// Curve - Single open or closed stroke
// =============================================================================
struct Curve {
    std::vector<Vertex> vertices;
    bool closed = false;

    Curve() = default;
    Curve(const std::vector<Vertex>& verts, bool isClosed = false)
        : vertices(verts), closed(isClosed) {}

    // Number of segments in the curve
    int segmentCount() const {
        if (vertices.empty()) return 0;
        return closed ? (int)vertices.size() : (int)vertices.size() - 1;
    }

    // Check if curve is empty
    bool empty() const { return vertices.empty(); }
};

// =============================================================================
// Path - Collection of curves
// =============================================================================
struct Path {
    std::vector<Curve> curves;

    Path() = default;
    explicit Path(const std::vector<Curve>& c) : curves(c) {}
    explicit Path(const Curve& c) : curves({c}) {}

    // Check if path is empty
    bool empty() const {
        return curves.empty() || (curves.size() == 1 && curves[0].empty());
    }

    // Get total number of curves
    int curveCount() const { return (int)curves.size(); }

    // Static empty path
    static const Path& emptyPath() {
        static Path empty;
        return empty;
    }
};

// Type aliases for specific path types
using PathL = Path;  // Path with only line commands
using PathC = Path;  // Path with only cubic bezier commands
using PathA = Path;  // Path with only arc commands

} // namespace pave
