class Camera {
    Vec3 camLocation = new Vec3(0,0,0);
    Vec3 camLookAt = new Vec3(0,0,0);
    Vec3 camUp = new Vec3(0,-1,0);
    float radius = 40;
    int slider = 200;
    float theta = 0;
    float fov = 55;
    float nearPlaneW = 1 + 1.f/3;
    float nearPlaneH = 1;
    float nearPlaneDist = 1;
    float farPlaneDist = 1000;

    public void update() {
        if (keyPressed) {
            if (keys[6]) {
                slider += 3;
            } else if (keys[7]) {
                slider -= 3;
            } else if (keys[4]) {
                theta -= 0.8;
            } else if (keys[5]) {
                theta += 0.8;
            }
        }
        camLocation.x = cos(radians(theta))*radius;
        camLocation.y = float(slider)/5;
        camLocation.z = sin(radians(theta))*radius;
        camLocation.add(camLookAt);
        camera(camLocation.x, camLocation.y, camLocation.z,
               camLookAt.x,   camLookAt.y,   camLookAt.z,
               camUp.x,       camUp.y,       camUp.z);
    }

    public void setPerspective() {
         perspective(radians(fov), nearPlaneW/nearPlaneH, nearPlaneDist, farPlaneDist);
    }
}