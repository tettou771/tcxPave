#pragma once

// =============================================================================
// Pave.h - Path manipulation toolkit for TrussC
// Ported from baku89's pave (https://github.com/baku89/pave)
//
// Pave is an immutable, functional API for manipulating SVG-like paths.
// It supports lines, cubic beziers, and elliptical arcs.
//
// Usage:
//   #include <TrussC.h>  // Must include TrussC first
//   #include <Pave.h>
//
//   using namespace pave;
//
//   // Create a circle
//   Path circle = PathFn::circle(Vec2(100, 100), 50);
//
//   // Get point at 25% along the path
//   Vec2 p = PathFn::pointAt(circle, 0.25f);
//
//   // Apply wave distortion
//   Path wavy = distort(circle, DistortFn::wave(10, 30));
//
// =============================================================================

// TrussC math types are required - included via PaveTypes.h

// Core types and utilities
#include "PaveTypes.h"
#include "PaveLocation.h"

// Data structures
#include "PaveData.h"

// Segment type implementations
#include "PaveLine.h"
#include "PaveCubicBezier.h"
#include "PaveArc.h"

// High-level APIs
#include "PaveSegment.h"
#include "PaveCurve.h"
#include "PavePath.h"
#include "PaveDistort.h"

// =============================================================================
// Convenience namespace aliases
// =============================================================================

namespace pave {

// Bring common functions into pave namespace for convenience
using namespace PathFn;

// Path type alias for compatibility
using P = Path;

// Quick access to primitive creation
inline Path line(const Vec2& a, const Vec2& b) { return PathFn::line(a, b); }
inline Path rect(const Vec2& a, const Vec2& b) { return PathFn::rect(a, b); }

} // namespace pave

// =============================================================================
// Integration with TrussC drawing
// =============================================================================

namespace pave {

// Draw a path using TrussC's drawing API
// Note: Requires TrussC.h to be included for tc::drawPath, etc.
#ifdef TRUSSC_GRAPHICS_INCLUDED

inline void draw(const Path& path) {
    // Convert to line segments and draw
    Path flat = PathFn::flatten(path, 2.0f);

    for (const Curve& curve : flat.curves) {
        if (curve.vertices.empty()) continue;

        tc::beginShape();
        for (const Vertex& v : curve.vertices) {
            tc::vertex(v.point.x, v.point.y);
        }
        if (curve.closed) {
            tc::endShape(true);
        } else {
            tc::endShape(false);
        }
    }
}

inline void stroke(const Path& path) {
    tc::noFill();
    draw(path);
}

inline void fill(const Path& path) {
    tc::fill();
    draw(path);
}

#endif // TRUSSC_GRAPHICS_INCLUDED

} // namespace pave
