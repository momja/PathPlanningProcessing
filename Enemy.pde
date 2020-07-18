// Max Omdal 2020

class Enemy extends Agent {
    PImage hoverPointer;
    float hoverBounce = 0;
    boolean freeToMove = false;
    PathTraversal pathMaker;
    PShape model;

    public Enemy(Vec3 initialPosition) {
        this.position = initialPosition;
        this.position2D = new Vec2(position.x, position.z);
        this.pathMaker = new PathTraversal();
        this.hoverPointer = loadImage("hover_pointer.png");
        this.model = loadShape("stanford_bunny.obj");
    }

    public void startMove() {
        this.freeToMove = true;
        this.pathMaker.destinationReached = false;
        this.pathMaker.setPath(this.position2D, player.position2D);
    }

    public void update(float dt) {
        hoverBounce += 3*dt;

        if (forces != null)
            this.velocity.add(forces.times(dt));
        this.position2D.add(this.velocity.times(dt));
        this.position.x = this.position2D.x;
        this.position.z = this.position2D.y;
        
        if (freeToMove) {
            if (pathMaker.path != null && pathMaker.path.size() > 1) {
                // TODO: use path to target to take forward movement
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
        Cylinder body = new Cylinder(0.3,2, this.position.plus(new Vec3(0, 1, 0)));
        body.materialColor = new Vec3(255,255,10);

        Vec2 movementDir = new Vec2(0,0);
        Vec2 nextNodePos = pathMaker.getNextNodeOnPath();
        if (nextNodePos != null) {
            movementDir = nextNodePos.minus(position2D).normalized();
        }
        float movementAngle = atan2(movementDir.y, movementDir.x);
        if (movementAngle - orientationAngle > PI) {
            orientationAngle = orientationAngle + 0.1 * (2*PI - movementAngle - orientationAngle);
        } else {
            orientationAngle = orientationAngle + 0.1 * (movementAngle - orientationAngle);
        }
        push();
        if (this.model == null) {
            rotateY(-orientationAngle-3*PI/2);
            body.draw();
        } else {
            translate(position.x, position.y, position.z);
            rotateY(-orientationAngle-3*PI/2);
            shape(this.model);
        }
        pop();

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