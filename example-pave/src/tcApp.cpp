#include "tcApp.h"
#include <TrussC.h>
#include <Pave.h>

using namespace pave;

void tcApp::setup() {
    // Create some test paths
    testCircle = PathFn::circle(Vec2(200, 200), 80);
    testRect = PathFn::rectangle(Vec2(350, 150), Vec2(550, 250));
    testArc = PathFn::arc(Vec2(650, 200), 60, 45, 315);

    // Create a polygon
    vector<Vec2> points = {
        Vec2(100, 400), Vec2(180, 350), Vec2(250, 420),
        Vec2(200, 500), Vec2(120, 480)
    };
    testPolygon = PathFn::polygon(points);
}

void tcApp::update() {
    time = getElapsedTimef();
}

void tcApp::draw() {
    clear(0.1f);

    // Draw original paths
    setColor(colors::white);
    noFill();

    drawPath(testCircle);
    drawPath(testRect);
    drawPath(testArc);
    drawPath(testPolygon);

    // Draw distorted circle
    float waveAmp = 10.0f * (1.0f + sin(time * 2.0f));
    Distort waveDistort = DistortFn::wave(waveAmp, 30.0f, time * 50.0f);
    pave::Path wavyCircle = pave::distort(testCircle, waveDistort);

    setColor(colors::skyBlue);
    drawPath(wavyCircle);

    // Draw distorted rectangle with twirl
    Vec2 twirlCenter(450, 200);
    float twirlAngle = sin(time) * 45.0f;
    Distort twirlDistort = DistortFn::twirl(twirlCenter, 150.0f, twirlAngle);
    pave::Path twirledRect = pave::distort(testRect, twirlDistort);

    setColor(colors::coral);
    drawPath(twirledRect);

    // Show point at location on polygon
    float t = fmod(time * 0.2f, 1.0f);
    Vec2 pointOnPath = PathFn::pointAt(testPolygon, t);
    Vec2 tangentVec = PathFn::tangent(testPolygon, t);
    Vec2 normalVec = PathFn::normal(testPolygon, t);

    setColor(colors::yellow);
    fill();
    drawCircle(pointOnPath, 8);

    // Draw tangent and normal
    noFill();
    setColor(colors::green);
    drawLine(pointOnPath, pointOnPath + tangentVec * 40.0f);

    setColor(colors::red);
    drawLine(pointOnPath, pointOnPath + normalVec * 40.0f);

    // Info text
    setColor(colors::white);
    drawBitmapString("Pave Example - Path Manipulation", 20, 30);
    drawBitmapString("Blue: Wave distortion on circle", 20, 550);
    drawBitmapString("Coral: Twirl distortion on rectangle", 20, 570);
}

void tcApp::drawPath(const pave::Path& path) {
    // Flatten path to polyline and draw
    pave::Path flat = PathFn::flatten(path, 2.0f);

    for (const pave::Curve& curve : flat.curves) {
        if (curve.vertices.empty()) continue;

        beginShape();
        for (const pave::Vertex& v : curve.vertices) {
            vertex(v.point.x, v.point.y);
        }
        if (curve.closed) {
            endShape(true);
        } else {
            endShape(false);
        }
    }
}

void tcApp::keyPressed(int key) {}
void tcApp::keyReleased(int key) {}
void tcApp::mousePressed(const MouseEventArgs& e) {}
void tcApp::mouseReleased(const MouseEventArgs& e) {}
void tcApp::mouseMoved(const MouseMoveEventArgs& e) {}
void tcApp::mouseDragged(const MouseDragEventArgs& e) {}
void tcApp::mouseScrolled(const ScrollEventArgs& e) {}
void tcApp::windowResized(int width, int height) {}
void tcApp::filesDropped(const vector<string>& files) {}
void tcApp::exit() {}
