// Max Omdal 2020

class Probe extends Agent {
    PShape model = createShape(BOX,1,1,1);
    PathTraversal pathMaker;
    boolean isActive = true;

    public Probe(Vec2 startPos, Vec2 goalPos) {
        super();
        pathMaker = new PathTraversal();
        boolean successfulPath = pathMaker.setPath(startPos, goalPos);
        if (!successfulPath) {
            // If we can't get to the actual desired position, we will not send a probe
            this.isActive = false;
        }
        this.position2D = startPos;
        this.position = new Vec3(this.position2D.x, -5, this.position2D.y);
    }

    public void update(float dt) {
        if (forces != null)
            this.velocity.add(forces.times(dt));
        this.position2D.add(this.velocity.times(dt));
        this.position.x = this.position2D.x;
        this.position.z = this.position2D.y;

        if (pathMaker.path != null && pathMaker.path.size() > 1) {
            this.pathMaker.updateMovementDir(position2D);
            this.goalVelocity = this.pathMaker.movementDir.times(speed);

            if (pathMaker.destinationReached) {
                this.isActive = false;
                this.velocity = new Vec2();
            }
        }
    }

    public void draw() {
        if (!this.isActive) {
            return;
        }
        Vec2 movementDir = velocity.normalized();
        float movementAngle = atan2(movementDir.y, movementDir.x);
        // if (movementAngle > orientationAngle) {
        //     orientationAngle = orientationAngle + 0.1 * (2*PI - movementAngle - orientationAngle);
        // } else {
        //     orientationAngle = orientationAngle + 0.1 * (movementAngle - orientationAngle);
        // }
        orientationAngle = movementAngle;

        push();
        if (this.model == null) {
            point(position.x, position.y, position.z);
        } else {
            translate(position.x, position.y, position.z);
            rotateY(-orientationAngle-3*PI/2);
            shape(this.model);
        }
        pop();
        push();
        stroke(255,0,0);
        strokeWeight(1);
        if (debugMode) {
            Vec3 velDir = new Vec3(this.velocity.x*0.1 + this.position.x,
                                   this.position.y,
                                   this.velocity.y*0.1 + this.position.z);
            line(this.position.x, this.position.y, this.position.z, velDir.x, velDir.y, velDir.z);
        }
        pop();
    }
}