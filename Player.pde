// Max Omdal 2020

class Player {
    Vec3 position;
    float speed = 10;
    boolean batCallActive = false;
    PImage hoverPointer;
    float hoverBounce = 0;

    public Player(Vec3 initialPosition) {
        this.position = initialPosition;
        hoverPointer = loadImage("hover_pointer.png");
    }

    public void update(float dt) {
        hoverBounce += 3*dt;
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
                position.add(positionUpdate.normalized().times(dt*speed));
            }
        }
    }

    public void draw() {
       Cylinder body = new Cylinder(0.3,2, this.position.plus(new Vec3(0, 1, 0)));
       body.materialColor = new Vec3(255,10,10);
       body.draw();

       if (batCallActive) {
           Cylinder focusRing = new Cylinder(0.9, 0.1, this.position.plus(new Vec3(0,0.05,0)));
           focusRing.materialColor = new Vec3(200,200,0);
           focusRing.draw();
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
}