// Max Omdal 2020

class Enemy extends Agent {
    PImage hoverPointer;
    float hoverBounce = 0;
    boolean freeToMove = false;
    PathTraversal pathMaker;
    PShape model;

    public Enemy(Vec3 initialPosition) {
        super();
        this.position = initialPosition;
        this.position2D = new Vec2(position.x, position.z);
        this.pathMaker = new PathTraversal();
        this.hoverPointer = loadImage("hover_pointer.png");
        this.model = loadShape("joker.obj");
    }

    public void startMove() {
        this.freeToMove = true;
        this.pathMaker.destinationReached = false;
        this.pathMaker.setPath(this.position2D, player.position2D);
    }

    public void update(float dt) {
        hoverBounce += 3*dt;

        if (freeToMove) {
            if (forces != null)
                this.velocity.add(forces.times(dt));
            this.position2D.add(this.velocity.times(dt));
            this.position.x = this.position2D.x;
            this.position.z = this.position2D.y;
            if (pathMaker.path != null && pathMaker.path.size() > 1) {
                this.pathMaker.updateMovementDir(position2D);
                this.goalVelocity = this.pathMaker.movementDir.times(speed);

                if (pathMaker.destinationReached) {
                    freeToMove = false;
                    this.velocity = new Vec2();
                }
            }
        }
    }

    public void draw() {
        Vec2 movementDir = velocity.normalized();
        if (freeToMove) {
            orientationAngle = atan2(movementDir.y, movementDir.x);
        } else {
            orientationAngle = 0;
        }
        // if (movementAngle > orientationAngle) {
        //     orientationAngle = orientationAngle + 0.1 * (2*PI - movementAngle - orientationAngle);
        // } else {
        //     orientationAngle = orientationAngle + 0.1 * (movementAngle - orientationAngle);
        // }
        if (this.model == null) {
            Cylinder body = new Cylinder(0.3,2, this.position.plus(new Vec3(0, 1, 0)));
            body.materialColor = new Vec3(255,255,10);
            rotateY(-orientationAngle-3*PI/2);
            body.draw();
        } else {
            push();
            translate(position.x, position.y, position.z);
            rotateY(-orientationAngle-3*PI/2);
            shape(this.model);
            pop();
        }

        push();
        noStroke();
        translate(position.x, position.y + 8 + 0.5*cos(hoverBounce), position.z);
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