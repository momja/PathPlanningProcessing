// Max Omdal 2020
import java.util.Arrays;

boolean paused = false;
Camera cam = new Camera();
int maxNumNodes = 2000;
int maxNumObstacles = 1000;
int maxNumRigidBodies = 1000;
int numObstacles = 100;
int numNodes = 800;
boolean debugMode = false;
boolean[] keys;
ArrayList<Integer>[] paths = new ArrayList[3];
float speed = 15;
Vec2[] posAlongPath = new Vec2[3];
int[] nextNodeIdx = new int[3];
float[] distanceToNextNode = new float[3];
int nextSpawnIndex = 0;

PShader unlitShader;

PShape[] rigidBodies = new PShape[maxNumRigidBodies];
Cylinder[] obstacles = new Cylinder[maxNumObstacles];
float[] circleRad = new float[maxNumObstacles];
Vec2[] circlePos = new Vec2[maxNumObstacles];
Vec2[] nodePos = new Vec2[maxNumNodes];
Vec2[] startPositions = new Vec2[3];
Vec2[] goalPositions = new Vec2[3];
ArrayList<Integer> jokerPath = new ArrayList<Integer>();
Enemy joker;
PImage groundTexture;
Player player;
Bats bats;

void setup() {
    size(1280, 960, P3D);
    cam.setPerspective();
    surface.setTitle("Path Planning [Max Omdal]");
    keys = new boolean[8];
    keys[0] = false; keys[1] = false; keys[2] = false; keys[3] = false;
    keys[4] = false; keys[5] = false; keys[6] = false; keys[7] = false;

    groundTexture = loadImage("concrete_floor.png");
    unlitShader = loadShader("unlit_frag.glsl", "unlit_vert.glsl");

    // Set up PRM
    createObstacles();
    generateRandomNodes(numNodes, circlePos, circleRad);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);

    Vec3[] spawnPoints = new Vec3[3];
    for (int i = 0; i < 3; i++) {
        distanceToNextNode[i] = 0;
        nextNodeIdx[i] = 1;
        startPositions[i] = nodePos[i];
        goalPositions[i] = nodePos[i];
        spawnPoints[i] = new Vec3(startPositions[i].x, 5, startPositions[i].y);
    }
    player = new Player(new Vec3(0, -5, 0));
    joker = new Enemy(new Vec3(nodePos[3].x, -5, nodePos[3].y));

    bats = new Bats(10);
    bats.spawnPoints = spawnPoints;
    bats.numTetherPoints = 3;
    bats.initialize();

    // Set up rigidbodies
    for (int i = 0; i < numObstacles; i++) {
        rigidBodies[i] = obstacles[i].mesh;
    }
}

void draw() {
    cam.update();
    background(50,60,50);
    lights();
    pointLight(100, 100, 255, 0, 1, 0);
    if (!paused) {
        update(1.f/frameRate);
    }

    // Ground plane
    textureMode(NORMAL);
    textureWrap(REPEAT);
    beginShape();
    texture(groundTexture);
    vertex(-100,-5,-100, 0, 0);
    vertex(-100,-5,100, 0, 10);
    vertex(100,-5,100, 10, 10);
    vertex(100,-5,-100, 10, 0);
    endShape();
    drawObstacles();
    player.draw();
    joker.draw();
    bats.drawBoids();

    if (debugMode) {
        // drawPRMGraph();
        pushStyle();
        stroke(255,0,0);
        strokeWeight(10);
        point(bats.tetherPoint.x, bats.tetherPoint.y, bats.tetherPoint.z);
        stroke(0,255,0);
        for (int i = 0; i < 3; i++) {
            point(goalPositions[i].x, 5, goalPositions[i].y);
        }
        stroke(0,0,255);
        for (int i = 0; i < 3; i++) {
            point(startPositions[i].x, 5, startPositions[i].y);
        }
        popStyle();
    }

}

void update(float dt) {
    player.update(dt);
    joker.update(dt);
    cam.camLookAt = player.position;
    for (int i = 0; i < paths.length; i++) {
        if (player.batCallActive[i] && paths[i] != null) {
            updatePositionAlongPath(dt, i);
        }
    }

    bats.updateBoidPositions(dt);
    // bats.checkForCollisions(rigidBodies, numObstacles);
}

void updatePositionAlongPath(float dt, int idx) {
    nodePos[numNodes] = startPositions[idx];
    nodePos[numNodes+1] = goalPositions[idx];
    Vec2 curPosToNode = nodePos[paths[idx].get(nextNodeIdx[idx])].minus(posAlongPath[idx]).normalized();
    posAlongPath[idx].add(curPosToNode.times(dt*speed));
    bats.tetherPoints[idx] = new Vec3(posAlongPath[idx].x, 5, posAlongPath[idx].y);

    // Check to see if we can shortcut to any further along nodes
    for (int i = nextNodeIdx[idx]+1; i < paths[idx].size(); i++) {
        Vec2 node = nodePos[paths[idx].get(i)];
        Vec2 curPosToShortcutNode = node.minus(posAlongPath[idx]);
        hitInfo intersect = rayCircleListIntersect(circlePos, circleRad, numObstacles, posAlongPath[idx], curPosToShortcutNode.normalized(), curPosToShortcutNode.length());
        if (!intersect.hit) {
            // Woohoo! We found a shortcut, let's take it
            nextNodeIdx[idx] = i;
            distanceToNextNode[idx] = curPosToShortcutNode.length();
            // We keep looking for any better shortcuts
        }
    }

    distanceToNextNode[idx] -= dt*speed;

    if (distanceToNextNode[idx] <= 0 || (distanceToNextNode[idx] <= 0.5 && nextNodeIdx[idx] == paths[idx].size() - 1)) {
        nextNodeIdx[idx]++;
        if (nextNodeIdx[idx] == paths[idx].size()) {
            // Destination reached
            nextNodeIdx[idx] = 1;
            distanceToNextNode[idx] = 0;
            paths[idx] = null;
            player.batCallActive[idx] = false;
            startPositions[idx] = new Vec2(goalPositions[idx]);
            return;
        }
        distanceToNextNode[idx] = nodePos[paths[idx].get(nextNodeIdx[idx])].minus(nodePos[paths[idx].get(nextNodeIdx[idx]-1)]).length();
    }
    push();
    stroke(255);
    strokeWeight(10);
    point(posAlongPath[idx].x, -3, posAlongPath[idx].y);
    pop();
}

void createObstacles() {
    for (int i = 0; i < numObstacles; i++) {
        float radius = random(3,5);
        float height = random(11,12);
        Vec3 pos = new Vec3(random(-80,80), -5 + height/2, random(-80,80));
        obstacles[i] = new Cylinder(radius, height, pos);
        circleRad[i] = radius;
        circlePos[i] = new Vec2(pos.x, pos.z);
    }
}

void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii) {
    for (int i = 0; i < numNodes; i++) {
        Vec2 randPos = new Vec2(random(-100,100), random(-100,100));
        boolean insideAnyCircle = pointInCircleList(circleCenters, circleRadii, numObstacles, randPos);

        while(insideAnyCircle) {
            randPos = new Vec2(random(-100,100), random(-100,100));
            insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
        }
        nodePos[i] = randPos;
    }
}

void drawObstacles() {
    for (int i = 0; i < numObstacles; i++) {
        obstacles[i].draw();
    }
}