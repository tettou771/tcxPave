#pragma once

#include <TrussC.h>
#include <Pave.h>

using namespace std;
using namespace tc;

class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void keyPressed(int key) override;
    void keyReleased(int key) override;

    void mousePressed(const MouseEventArgs& e) override;
    void mouseReleased(const MouseEventArgs& e) override;
    void mouseMoved(const MouseEventArgs& e) override;
    void mouseDragged(const MouseEventArgs& e) override;
    void mouseScrolled(const ScrollEventArgs& e) override;

    void windowResized(int width, int height) override;
    void filesDropped(const vector<string>& files) override;
    void exit() override;

private:
    void drawPath(const pave::Path& path);

    pave::Path testCircle;
    pave::Path testRect;
    pave::Path testArc;
    pave::Path testPolygon;

    float time = 0;
};
