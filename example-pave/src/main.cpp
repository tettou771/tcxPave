#include "tcApp.h"

int main() {
    WindowSettings settings;
    settings.setSize(800, 600);
    settings.setTitle("Pave Example");
    return runApp<tcApp>(settings);
}
