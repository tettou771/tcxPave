#pragma once

// =============================================================================
// PaveLocation.h - Location types for Pave
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PaveTypes.h"
#include <variant>
#include <optional>

namespace pave {

// =============================================================================
// Segment Location - position on a single curve segment
// =============================================================================

// Unit location: [0, 1] normalized position along segment length
struct UnitLoc {
    float unit;
    explicit UnitLoc(float u) : unit(u) {}
};

// Offset location: distance in pixels from segment start
struct OffsetLoc {
    float offset;
    explicit OffsetLoc(float o) : offset(o) {}
};

// Time location: mathematical parameter t in [0, 1] for curve equation
struct TimeLoc {
    float time;
    explicit TimeLoc(float t) : time(t) {}
};

// SegmentLocation can be any of the above, or just a float (treated as unit)
using SegmentLocation = std::variant<float, UnitLoc, OffsetLoc, TimeLoc>;

// Helper to get time from SegmentLocation (segment-specific implementation needed)
inline float toTimeFromFloat(float loc) {
    return loc;  // float is treated as unit, which equals time for lines
}

// =============================================================================
// Curve Location - position on a curve (multiple segments)
// =============================================================================

struct CurveLocationBase {
    std::optional<int> segmentIndex;
};

struct UnitCurveLoc : CurveLocationBase {
    float unit;
    UnitCurveLoc(float u, std::optional<int> segIdx = std::nullopt)
        : unit(u) { segmentIndex = segIdx; }
};

struct OffsetCurveLoc : CurveLocationBase {
    float offset;
    OffsetCurveLoc(float o, std::optional<int> segIdx = std::nullopt)
        : offset(o) { segmentIndex = segIdx; }
};

struct TimeCurveLoc : CurveLocationBase {
    float time;
    TimeCurveLoc(float t, std::optional<int> segIdx = std::nullopt)
        : time(t) { segmentIndex = segIdx; }
};

using CurveLocation = std::variant<float, UnitCurveLoc, OffsetCurveLoc, TimeCurveLoc>;

// =============================================================================
// Path Location - position on a path (multiple curves)
// =============================================================================

struct PathLocationBase {
    std::optional<int> curveIndex;
    std::optional<int> segmentIndex;
};

struct UnitPathLoc : PathLocationBase {
    float unit;
    UnitPathLoc(float u, std::optional<int> curveIdx = std::nullopt,
                std::optional<int> segIdx = std::nullopt)
        : unit(u) { curveIndex = curveIdx; segmentIndex = segIdx; }
};

struct OffsetPathLoc : PathLocationBase {
    float offset;
    OffsetPathLoc(float o, std::optional<int> curveIdx = std::nullopt,
                  std::optional<int> segIdx = std::nullopt)
        : offset(o) { curveIndex = curveIdx; segmentIndex = segIdx; }
};

struct TimePathLoc : PathLocationBase {
    float time;
    TimePathLoc(float t, std::optional<int> curveIdx = std::nullopt,
                std::optional<int> segIdx = std::nullopt)
        : time(t) { curveIndex = curveIdx; segmentIndex = segIdx; }
};

using PathLocation = std::variant<float, UnitPathLoc, OffsetPathLoc, TimePathLoc>;

// =============================================================================
// Utility functions
// =============================================================================

// Normalize index (handle negative indices like Python)
inline int normalizeIndex(int index, int count) {
    if (count == 0) return 0;
    index = index % count;
    if (index < 0) index += count;
    return index;
}

// Normalize offset to [0, length] range, handling negative values
inline float normalizeOffset(float offset, float length) {
    if (length <= 0) return 0;
    offset = std::fmod(offset, length);
    if (offset < 0) offset += length;
    return offset;
}

} // namespace pave
