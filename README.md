# tcxPave

Path manipulation toolkit for TrussC, ported from [baku89's pave](https://github.com/baku89/pave).

## Features

- **Immutable, functional API** for manipulating SVG-like paths
- Support for **Line**, **Cubic Bezier**, and **Elliptical Arc** segments
- **Parametric queries**: point, tangent, normal at any location
- **Distortion system** using Jacobian matrices for proper curve transformation
- Path primitives: circle, rectangle, arc, polygon, polyline, etc.
- Operations: flatten, resample, transform, reverse, join

## Installation

Place this addon in `TrussC/addons/tcxPave` and add to your project's `addons.make`:

```
tcxPave
```

## Usage

```cpp
#include <TrussC.h>
#include <Pave.h>

using namespace pave;

// Create paths
Path circle = PathFn::circle(Vec2(200, 200), 80);
Path rect = PathFn::rectangle(Vec2(100, 100), Vec2(300, 200));

// Query points on path
Vec2 point = PathFn::pointAt(circle, 0.25f);  // 25% along the path
Vec2 tangent = PathFn::tangent(circle, 0.25f);
Vec2 normal = PathFn::normal(circle, 0.25f);

// Apply distortion
Distort wave = DistortFn::wave(10.0f, 30.0f);  // amplitude, frequency
Path wavy = pave::distort(circle, wave);

Distort twirl = DistortFn::twirl(Vec2(200, 200), 100.0f, 45.0f);  // center, radius, angle
Path twisted = pave::distort(rect, twirl);

// Flatten to polyline for drawing
Path flat = PathFn::flatten(wavy, 2.0f);  // tolerance
```

## Drawing Paths

```cpp
void drawPath(const pave::Path& path) {
    pave::Path flat = PathFn::flatten(path, 2.0f);

    for (const pave::Curve& curve : flat.curves) {
        if (curve.vertices.empty()) continue;

        beginShape();
        for (const pave::Vertex& v : curve.vertices) {
            vertex(v.point.x, v.point.y);
        }
        endShape(curve.closed);
    }
}
```

## API Overview

### Path Primitives (PathFn namespace)
- `line(a, b)` - Line segment
- `rectangle(min, max)` - Rectangle
- `circle(center, radius)` - Circle
- `ellipse(center, radius)` - Ellipse
- `arc(center, radius, startAngle, endAngle)` - Arc
- `polygon(points)` - Closed polygon
- `polyline(points)` - Open polyline
- `regularPolygon(center, radius, sides)` - Regular polygon

### Path Queries
- `length(path)` - Total arc length
- `bounds(path)` - Bounding box
- `pointAt(path, t)` - Point at normalized position
- `tangent(path, t)` - Tangent vector
- `normal(path, t)` - Normal vector

### Path Operations
- `flatten(path, tolerance)` - Convert to line segments
- `resample(path, count)` - Uniform resampling
- `transform(path, matrix)` - Apply transformation
- `reverse(path)` - Reverse direction
- `join(paths)` - Join multiple paths

### Distortion Functions (DistortFn namespace)
- `wave(amplitude, frequency, phase)` - Wave distortion
- `twirl(center, radius, angle)` - Twirl/spiral distortion
- `bulge(center, radius, strength)` - Bulge/pinch distortion

## License

MIT License (same as original pave)

## Credits

Original TypeScript implementation by [baku89](https://github.com/baku89/pave).
