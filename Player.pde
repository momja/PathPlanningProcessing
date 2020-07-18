// Max Omdal 2020

class Player {
    Vec3 position;
    Vec2 position2D;
    float speed = 20;
    boolean[] batCallActive = {false, false, false};
    PImage hoverPointer;
    float hoverBounce = 0;
    ArrayList<Projectile> projectiles = new ArrayList<Projectile>();
    ArrayList<Probe> probes = new ArrayList<Probe>();

    public Player(Vec3 initialPosition) {
        this.position = initialPosition;
        this.position2D = new Vec2(position.x, position.z);
        hoverPointer = loadImage("hover_pointer.png");
    }

    public void update(float dt) {
        hoverBounce += 3*dt;
        for (int i = 0; i < projectiles.size(); i++) {
            Projectile projectile = projectiles.get(i);
            if (projectile.isActive) {
                projectile.update(dt);
            } else {
                projectiles.remove(i);
                i--;
            }
        }
        for (int i = 0; i < probes.size(); i++) {
            Probe probe = probes.get(i);
            if (probe.isActive) {
                probe.update(dt);
            } else {
                probes.remove(i);
                i--;
            }
        }
        if (batCallActive[0] || batCallActive[1] || batCallActive[2]) {
            return;
        }
        if (keyPressed) {
            Vec3 positionUpdate = new Vec3();
            if (keys[0]) {
                // Forwards
                positionUpdate.z += 1;
            } else if (keys[2]) {
                // Backwards
                positionUpdate.z -= 1;
            }
            if (keys[1]) {
                // Left
                positionUpdate.x -= 1;
            } else if (keys[3]) {
                // Right
                positionUpdate.x += 1;
            }
            if (positionUpdate.length() > 0) {
                // We then rotate the position update according to the camera
                positionUpdate.normalize();
                Vec3 camToPlayer = position.minus(cam.camLocation).normalized();
                Vec2 camToPlayer2D = new Vec2(camToPlayer.x, camToPlayer.z).normalized();
                float rotationAngle = -atan2(camToPlayer2D.y,camToPlayer2D.x)-3*PI/2;
                Vec3 rotatedPositionUpdate = new Vec3(cos(rotationAngle)*positionUpdate.x + sin(rotationAngle)*positionUpdate.z,
                                                      positionUpdate.y,
                                                      -sin(rotationAngle)*positionUpdate.x + cos(rotationAngle)*positionUpdate.z);

                position.add(rotatedPositionUpdate.times(dt*speed));
                position2D.x = position.x;
                position2D.y = position.z;
            }
        }
    }

    public void draw() {
        Cylinder body = new Cylinder(0.3,2, this.position.plus(new Vec3(0, 1, 0)));
        body.materialColor = new Vec3(255,10,10);
        body.draw();

        for (int i = 0; i < 3; i++) {
            if (batCallActive[i]) {
                Cylinder focusRing = new Cylinder(0.9, 0.1, new Vec3(goalPositions[i].x, 0.05, goalPositions[i].y));
                focusRing.materialColor = new Vec3(200,200,0);
                focusRing.draw();
            }
        }

        for (Projectile projectile : projectiles) {
            projectile.draw();
        }
        for (Probe probe : probes) {
            probe.draw();
        }

        push();
        noStroke();
        translate(position.x, position.y + 5 + 0.5*cos(hoverBounce), position.z);
        Vec3 camToPlayer = position.minus(cam.camLocation).normalized();
        Vec2 camToPlayer2D = new Vec2(camToPlayer.x, camToPlayer.z).normalized();
        float angle = atan2(camToPlayer2D.y,camToPlayer2D.x);
        rotateY(-angle-3*PI/2);
        shader(unlitShader);
        beginShape();
        textureMode(NORMAL);
        texture(hoverPointer);
        vertex(-1,1.5,0,0,0);
        vertex(1,1.5,0,1,0);
        vertex(1,-1.5,0,1,1);
        vertex(-1,-1.5,0,0,1);
        endShape();
        resetShader();
        pop();
    }

    public void throwProjectile(Vec3 direction) {
        Ray3 dirRay = new Ray3(this.position.plus(new Vec3(0,3,0)), direction);
        projectiles.add(new Projectile(dirRay));
    }

    public void launchProbe(Vec2 goalPos) {
        probes.add(new Probe(this.position2D, goalPos));
    }
}