// Max Omdal 2020
boolean paused = false;
Camera cam = new Camera();
int maxNumNodes = 1000;
int maxNumObstacles = 1000;
int numObstacles = 100;
int numNodes = 900;
boolean debugMode = false;
boolean[] keys;
ArrayList<Integer> path;
float speed = 10;
float distanceAlongPath = 0;

PShader unlitShader;

Cylinder[] obstacles = new Cylinder[maxNumObstacles];
float[] circleRad = new float[maxNumObstacles];
Vec2[] circlePos = new Vec2[maxNumObstacles];
Vec2[] nodePos = new Vec2[maxNumNodes];
Vec2 startPos;
Vec2 goalPos;
PImage groundTexture;
Player player;
BoidSystem bats;

void setup() {
    size(1280, 960, P3D);
    cam.setPerspective();
    surface.setTitle("Path Planning [Max Omdal]");
    keys = new boolean[4];
    keys[0] = false; keys[1] = false; keys[2] = false; keys[3] = false;
    bats = new BoidSystem(100);

    groundTexture = loadImage("concrete_floor.png");
    unlitShader = loadShader("unlit_frag.glsl", "unlit_vert.glsl");

    createObstacles();
    generateRandomNodes(numNodes, circlePos, circleRad);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
    startPos = nodePos[0];
    player = new Player(new Vec3(startPos.x, -5, startPos.y));

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

    if (debugMode) {
        drawPRMGraph();
    }

}

void keyPressed() {
    if (key == ' ')
        paused = !paused;
    if (key == 'p')
        debugMode = !debugMode;
    if (key=='w')
        keys[0]=true;
    if (key=='a')
        keys[1]=true;
    if (key=='s')
        keys[2]=true;
    if (key=='d')
        keys[3]=true;
    if (keyCode==ENTER) {
        player.batCallActive = true;
        goalPos = new Vec2(player.position.x, player.position.z);
        path = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
    }
}

void keyReleased() {
    if (key=='w')
        keys[0]=false;
    if (key=='a')
        keys[1]=false;
    if (key=='s')
        keys[2]=false;
    if (key=='d')
        keys[3]=false;
}

void update(float dt) {
    player.update(dt);
    if (player.batCallActive && path != null) {
        if (path.get(0) != -1) {
            distanceAlongPath += dt*speed;
            int[] pathArr = new int[path.size()];
            for (int i = 0; i < path.size(); i++) {
                pathArr[i] = path.get(i);
            }
            Vec2 newPos = positionAlongPath(distanceAlongPath, pathArr);
            if (newPos == null) {
                // Destination reached
                distanceAlongPath = 0;
                path = null;
                player.batCallActive = false;
                startPos = goalPos;
            } else {
                push();
                stroke(255);
                strokeWeight(10);
                pop();
            }
        }
    }
}

void createObstacles() {
    for (int i = 0; i < numObstacles; i++) {
        float radius = random(3,5);
        float height = random(5,8);
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