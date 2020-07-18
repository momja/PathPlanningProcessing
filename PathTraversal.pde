// Max Omdal 2020

class PathTraversal {
    int nextNodeIdx = 1;
    ArrayList<Integer> path;
    float distanceToNextNode = 0;
    Vec2 startPos;
    Vec2 goalPos;
    boolean destinationReached = false;
    Vec2 movementDir;

    public boolean setPath(Vec2 startPos, Vec2 goalPos) {
        this.startPos = new Vec2(startPos);
        this.goalPos = new Vec2(goalPos);
        this.path = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
        this.nextNodeIdx = 1;
        if (path == null || path.size() == 1) {
            return false;
        }
        this.movementDir = nodePos[path.get(1)].minus(startPos);
        this.distanceToNextNode = movementDir.length();
        this.movementDir.normalize();
        return true;
    }

    public Vec2 getNextNodeOnPath() {
        if (path != null) {
            if (nextNodeIdx == path.size() - 1)
                return this.goalPos;
            return nodePos[path.get(nextNodeIdx)];
        } else {
            return null;
        }
    }
    
    public void updateMovementDir(Vec2 curPos) {
        nodePos[numNodes] = startPos;
        nodePos[numNodes+1] = goalPos;
        Vec2 curPosToNode = nodePos[path.get(nextNodeIdx)].minus(curPos);

        // Check to see if we can shortcut any nodes
        for (int i = nextNodeIdx+1; i < path.size(); i++) {
            Vec2 node = nodePos[path.get(i)];
            Vec2 curPosToShortcutNode = node.minus(curPos);
            hitInfo intersect = rayCircleListIntersect(circlePos, circleRad, numObstacles, curPos, curPosToShortcutNode.normalized(), curPosToShortcutNode.length());
            if (!intersect.hit) {
                // Woohoo! We found a shortcut, let's take it
                nextNodeIdx = i;
                curPosToNode = nodePos[path.get(nextNodeIdx)].minus(curPos);
                // There could be a better shortcut, so we keep looking
            }
        }

        distanceToNextNode = curPosToNode.length();
        this.movementDir = curPosToNode.normalized();

        if (distanceToNextNode <= 0.5) {
            nextNodeIdx++;
            if (nextNodeIdx == path.size()) {
                // Destination reached
                nextNodeIdx = 1;
                distanceToNextNode = 0;
                path = null;
                startPos = new Vec2(goalPos);
                destinationReached = true;
                return;
            }
            distanceToNextNode = nodePos[path.get(nextNodeIdx)].minus(nodePos[path.get(nextNodeIdx-1)]).length();
        }
    }
}