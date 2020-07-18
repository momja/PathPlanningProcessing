// Max Omdal 2020

class Probe extends Agent{
    PShape model = createShape(BOX,1,1,1);
    PathTraversal pathMaker;
    boolean isActive = true;

    public Probe(Vec2 startPos, Vec2 goalPos) {
        pathMaker = new PathTraversal();
        pathMaker.setPath(startPos, goalPos);
        this.position2D = startPos;
        this.position = new Vec3(this.position2D.x, -5, this.position2D.y);
    }

    public void update(float dt) {
        if (pathMaker.path != null && pathMaker.path.size() > 1) {
            // TODO: use path to target to take forward movement
            this.pathMaker.updateMovementDir(position2D);
            this.velocity = this.pathMaker.movementDir.times(speed);
            this.position2D.add(this.velocity.times(dt));
            this.position.x = this.position2D.x;
            this.position.z = this.position2D.y;
            if (pathMaker.destinationReached) {
                this.isActive = false;
            }
        }
    }

    public void draw() {
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
            point(position.x, position.y, position.z);
        } else {
            translate(position.x, position.y, position.z);
            rotateY(-orientationAngle-3*PI/2);
            shape(this.model);
        }
        pop();
    }
}