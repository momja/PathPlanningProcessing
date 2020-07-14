// Max Omdal 2020
boolean paused = false;


void setup() {
    size(1280, 960, P3D);
    cam.setPerspective();
    surface.setTitle("Path Planning [Max Omdal]");
}

void draw() {
    cam.update();
    background(50,60,50);
    lights();
    directionalLight(1, -1, -1, 255, 255, 255);
    if (!paused) {
        update(1.f/frameRate);
    }

    // Insert draw calls here
}

void keyPressed() {
    if (key == ' ') {
        paused = !paused;
    }
}

void update(float dt) {

}