#pragma once

// =============================================================================
// PaveDistort.h - Distortion functions for paths
// Ported from baku89's pave (https://github.com/baku89/pave)
// =============================================================================

#include "PavePath.h"

namespace pave {
namespace DistortFn {

// =============================================================================
// Core distort function
// Distort = (Vec2) -> Mat2d (local transformation matrix)
// =============================================================================

// Create distort from a simple point transformer using numerical differentiation
// This computes the Jacobian matrix at each point
inline Distort fromPointTransformer(std::function<Vec2(const Vec2&)> fn) {
    constexpr float delta = 0.01f;

    return [fn, delta](const Vec2& p) -> Mat2d {
        Vec2 pt = fn(p);
        Vec2 deltaX = fn(Vec2(p.x + delta, p.y));
        Vec2 deltaY = fn(Vec2(p.x, p.y + delta));

        // Compute Jacobian via numerical differentiation
        float axisX_x = (deltaX.x - pt.x) / delta;
        float axisX_y = (deltaX.y - pt.y) / delta;
        float axisY_x = (deltaY.x - pt.x) / delta;
        float axisY_y = (deltaY.y - pt.y) / delta;

        return Mat2d(axisX_x, axisX_y, axisY_x, axisY_y, pt.x, pt.y);
    };
}

// =============================================================================
// Built-in distort functions
// =============================================================================

// Wave distortion
inline Distort wave(float amplitude, float width, float phase = 0,
                    float angle = 0, const Vec2& origin = Vec2(0, 0)) {
    Mat2d xform = Mat2d::rotation(angle, origin);
    auto invXform = xform.invert();

    return fromPointTransformer([=](const Vec2& p) -> Vec2 {
        Mat2d inv = invXform.value_or(Mat2d::identity());
        Vec2 pLocal = transformMat2d(p, inv);

        float theta = (pLocal.x / width + phase / 360.0f) * trussc::TAU;
        float yOffset = std::sin(theta) * amplitude;

        Vec2 pDistortedLocal(pLocal.x, pLocal.y + yOffset);
        return transformMat2d(pDistortedLocal, xform);
    });
}

// Twirl distortion
inline Distort twirl(const Vec2& center, float radius, float angle,
                     std::function<float(float)> ramp = nullptr) {
    if (!ramp) {
        ramp = [](float t) { return t; };
    }

    return fromPointTransformer([=](const Vec2& p) -> Vec2 {
        float dist = vec2::distance(center, p);

        if (dist > radius) {
            return p;
        }

        float theta = ramp(1.0f - dist / radius) * angle;
        Mat2d rotMat = Mat2d::rotation(theta, center);

        return transformMat2d(p, rotMat);
    });
}

// Bulge/Pinch distortion
inline Distort bulge(const Vec2& center, float radius, float amount) {
    return fromPointTransformer([=](const Vec2& p) -> Vec2 {
        Vec2 d = p - center;
        float dist = d.length();

        if (dist > radius || dist < 0.001f) {
            return p;
        }

        float factor = std::pow(dist / radius, amount);
        return center + d.normalized() * (factor * radius);
    });
}

// Noise distortion (requires external noise function)
inline Distort noise(std::function<float(float, float)> noiseFn,
                     float scale, float amount) {
    return fromPointTransformer([=](const Vec2& p) -> Vec2 {
        float nx = noiseFn(p.x * scale, p.y * scale);
        float ny = noiseFn(p.x * scale + 1000.0f, p.y * scale + 1000.0f);
        return Vec2(p.x + nx * amount, p.y + ny * amount);
    });
}

// =============================================================================
// Apply distort to path
// =============================================================================

// Distort a single vertex
inline Vertex distortVertex(const Vertex& v, const Distort& distort) {
    Vertex result = v;

    // Transform end point
    Mat2d m = distort(v.point);
    result.point = Vec2(m.tx, m.ty);

    // Transform control points for cubic bezier
    if (v.command == Command::C) {
        Mat2d m1 = distort(v.control1);
        Mat2d m2 = distort(v.control2);
        result.control1 = Vec2(m1.tx, m1.ty);
        result.control2 = Vec2(m2.tx, m2.ty);
    }

    return result;
}

// Distort a segment (subdivide for better approximation)
inline std::vector<Vertex> distortSegment(const Segment& seg, const Distort& distort,
                                           int subdivisions = 4) {
    std::vector<Vertex> result;

    if (seg.command == Command::L) {
        // For lines, just distort endpoints
        Mat2d m = distort(seg.point);
        result.push_back(Vertex::line(Vec2(m.tx, m.ty)));
    } else if (seg.command == Command::C) {
        // Subdivide and distort each sub-segment
        for (int i = 0; i < subdivisions; i++) {
            float t0 = (float)i / subdivisions;
            float t1 = (float)(i + 1) / subdivisions;

            auto [newStart, c1, c2, newEnd] = CubicBezier::trimBetweenTimes(
                seg.start, seg.control1, seg.control2, seg.point, t0, t1);

            // Distort control points using Jacobian
            Mat2d m0 = distort(newStart);
            Mat2d m1 = distort(c1);
            Mat2d m2 = distort(c2);
            Mat2d m3 = distort(newEnd);

            // Apply local transformation to control vectors
            Vec2 dc1 = c1 - newStart;
            Vec2 dc2 = c2 - newEnd;

            Vec2 newC1(m0.a * dc1.x + m0.c * dc1.y + m0.tx,
                       m0.b * dc1.x + m0.d * dc1.y + m0.ty);
            Vec2 newC2(m3.a * dc2.x + m3.c * dc2.y + m3.tx,
                       m3.b * dc2.x + m3.d * dc2.y + m3.ty);

            result.push_back(Vertex::cubic(Vec2(m3.tx, m3.ty), newC1, newC2));
        }
    } else if (seg.command == Command::A) {
        // Convert arc to cubic beziers first, then distort
        auto beziers = Arc::approximateByCubicBeziers(seg, 45.0f);
        for (const Vertex& bv : beziers) {
            Segment bseg;
            bseg.start = seg.start;  // Approximate
            bseg.point = bv.point;
            bseg.command = Command::C;
            bseg.control1 = bv.control1;
            bseg.control2 = bv.control2;

            auto distorted = distortSegment(bseg, distort, subdivisions);
            result.insert(result.end(), distorted.begin(), distorted.end());
        }
    }

    return result;
}

// Distort a curve
inline Curve distortCurve(const Curve& curve, const Distort& distort, int subdivisions = 4) {
    auto segs = CurveFn::segments(curve);
    if (segs.empty()) return curve;

    std::vector<Vertex> vertices;

    // First vertex (start point)
    Mat2d m0 = distort(segs[0].start);
    vertices.push_back(Vertex::line(Vec2(m0.tx, m0.ty)));

    // Distort each segment
    for (const Segment& seg : segs) {
        auto distorted = distortSegment(seg, distort, subdivisions);
        vertices.insert(vertices.end(), distorted.begin(), distorted.end());
    }

    return Curve(vertices, curve.closed);
}

// Distort a path
inline Path distortPath(const Path& path, const Distort& distort, int subdivisions = 4) {
    Path result;

    for (const Curve& curve : path.curves) {
        result.curves.push_back(distortCurve(curve, distort, subdivisions));
    }

    return result;
}

} // namespace DistortFn

// Convenience alias at namespace level
inline Path distort(const Path& path, const Distort& distort, int subdivisions = 4) {
    return DistortFn::distortPath(path, distort, subdivisions);
}

} // namespace pave
