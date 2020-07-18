import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.Arrays; 
import java.util.Comparator; 
import java.util.PriorityQueue; 
import java.util.Collections; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class PathPlanningProcessing extends PApplet {

// Max Omdal 2020


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

public void setup() {
    
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

public void draw() {
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

public void update(float dt) {
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

public void updatePositionAlongPath(float dt, int idx) {
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

    if (distanceToNextNode[idx] <= 0 || (distanceToNextNode[idx] <= 0.5f && nextNodeIdx[idx] == paths[idx].size() - 1)) {
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

public void createObstacles() {
    for (int i = 0; i < numObstacles; i++) {
        float radius = random(3,5);
        float height = random(11,12);
        Vec3 pos = new Vec3(random(-80,80), -5 + height/2, random(-80,80));
        obstacles[i] = new Cylinder(radius, height, pos);
        circleRad[i] = radius;
        circlePos[i] = new Vec2(pos.x, pos.z);
    }
}

public void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii) {
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

public void drawObstacles() {
    for (int i = 0; i < numObstacles; i++) {
        obstacles[i].draw();
    }
}
// Max Omdal 2020

class Bats extends BoidSystem {
    // Bats have multiple tether points and tuned parameters

    Vec3[] tetherPoints;
    Vec3[] spawnPoints;
    int numTetherPoints;

    public Bats(int boidCount) {
        super(boidCount);
    }

    @Override
    public void initialize() {
        boids = new ArrayList<Boid>();
        this.tetherPoints = new Vec3[numTetherPoints];
        this.boundingBoxOrigin = new Vec3(0,0,0);
        this.maxSpeed = 200;
        this.visualDistance = 0.9f;
        this.influenceToTetherPoint = 1.5f;
        this.separation = 1.5f;
        this.boidSize = 5;

        for (int i = 0; i < numTetherPoints; i++) {
            this.tetherPoints[i] = spawnPoints[i];
        }

        setBoundingBox(boundingBoxOrigin, 200, 200, 200);
        initializePositionsWithManySpawns(spawnPoints, 1.f);
    }


    public void initializePositionsWithManySpawns(Vec3[] spawnPoints, float variance) {
        // Given the bounding box, we will randomly place our boids within this box
        for (int i = 0; i < boidCount; i++) {
            Vec3 spawnPoint = spawnPoints[floor(((float)i) / boidCount * tetherPoints.length)];
            Vec3 position = new Vec3(random(variance, variance),
                                     random(variance, variance)/2,
                                     random(variance, variance));
            position.plus(spawnPoint);
            Boid boid = new Boid();
            boid.coords = position;
            boid.previousCoord = position;
            boid.perchCountdown = 0.f;
            boid.id = i;

            Vec3 velocity = new Vec3(random(-1,1),random(-1,1),random(-1,1));
            velocity.normalize();
            velocity.mul(random(maxSpeed/2, maxSpeed));
            boid.velocity = velocity;

            boids.add(boid);
        }
    }

    @Override
    public Vec3 moveToTetherPoint(Boid boid) {
        Vec3 tether = tetherPoints[floor(((float)boid.id) / boidCount * tetherPoints.length)];
        return tether.minus(boid.coords).normalized().times(influenceToTetherPoint);
    }
}
public class BoidSystem {
    // private ArrayList<Vec3> boidCoords;
    // private ArrayList<Vec3> boidPrevCoords;
    // // private ArrayList<Quaternion> boidOrientations;
    // private ArrayList<Vec3> boidVelocities;
    // private ArrayList<Float> perchedBoidCountdowns;
    protected ArrayList<Boid> boids;
    protected float width, height, length;

    Vec3 boundingBoxOrigin = new Vec3(0,25,0);
    float minX, maxX, minY, maxY, minZ, maxZ;

    int boidCount = 100;
    float boidSize = 2;
    float maxSpeed = 100;
    float separation = 2;
    float visualDistance = 6;
    float influenceToCenter = 0.02f;
    Vec3 boidColor = new Vec3(0,0,0);
    PImage boidTexture = null;
    PShape boidModel = null;
    PShape boidPerchedModel = null;
    Vec3 tetherPoint = new Vec3(8,5,-4);
    float influenceToTetherPoint = 0.03f;
    Vec3 spawnPoint;
    OctreeTriangles octree;

    public BoidSystem(int boidCount) {
        this.boidCount = boidCount;
        OctantTris octreeBounds = new OctantTris(new Vec3(), new Vec3(200,200,200));
        this.octree = new OctreeTriangles(octreeBounds, 20);
    }

    public void initialize() {
        // boidCoords = new ArrayList<Vec3>();
        // boidPrevCoords = new ArrayList<Vec3>();
        // // boidOrientations = new ArrayList<Quaternion>();
        // boidVelocities = new ArrayList<Vec3>();
        // perchedBoidCountdowns = new ArrayList<Float>();
        boids = new ArrayList<Boid>();
        setBoundingBox(boundingBoxOrigin, 100, 60, 100);
        initializePositions(spawnPoint);
    }

    public void setBoundingBox(Vec3 origin, float width, float height, float length) {
        this.width = width;
        this.height = height;
        this.length = length;
        this.boundingBoxOrigin = origin;

        minX = origin.x - width/2;
        maxX = origin.x + width/2;
        minY = origin.y - height/2;
        maxY = origin.y + height/2;
        minZ = origin.z - length/2;
        maxZ = origin.z + length/2;
    }

    public void checkForCollisions(PShape[] rigidBodies, int numRigidBodies) {
        // Check each boid to see if it has collided with the collision meshes
        for (int i = 0; i < numRigidBodies; i++) {
            PShape rigidBody = rigidBodies[i];
            checkCollision(rigidBody);
        }
    }

    public void checkCollision(PShape rigidBody) {
        for (int i = 0; i < rigidBody.getChildCount(); i++) {
            PShape child = rigidBody.getChild(i);
            if (child.getChildCount() != 0) {
                checkCollision(child);
            } else {
                for (int v = 0; v < child.getVertexCount(); v+=3) {
                    int j = 0;
                    PVector v1 = child.getVertex(v);
                    PVector v2 = child.getVertex(v+1);
                    PVector v3 = child.getVertex(v+2);

                    Vec3 vert1 = new Vec3(v1.x, v1.y, v1.z);
                    Vec3 vert2 = new Vec3(v2.x, v2.y, v2.z);
                    Vec3 vert3 = new Vec3(v3.x, v3.y, v3.z);

                    Vec3 e1 = vert2.minus(vert1);
                    Vec3 e2 = vert3.minus(vert1);

                    while(j < boidCount) {
                        Boid boid = boids.get(j);
                        Vec3 boidPosition = boid.coords;
                        // TODO: Use Barycentric Coordinates to find if there is a collision with the surface
                        boolean collision = false;
                        Vec3 collisionPoint = new Vec3(0,0,0);

                        Vec3 rayOrigin = boidPosition;
                        Vec3 rayDirection = boidPosition.minus(boid.previousCoord);
                        float maxT = rayDirection.length();
                        rayDirection.normalize();

                        if (maxT < 0.00001f) {
                            j++;
                            continue;
                        }

                        rayDirection.normalize();

                        Vec3 surfaceNormal = cross(e1, e2).normalized();
                        // float x_0 = rayOrigin.x; float y_0 = rayOrigin.y; float z_0 = rayOrigin.z;
                        // float x_d = rayDirection.x; float y_d = rayDirection.y; float z_d = rayDirection.z;
                        float denominator = dot(surfaceNormal, rayDirection);
                        if (abs(denominator) <= 0.0001f) {
                            // No ray plane intersection exists
                            j++;
                            continue;
                        }

                        float D = dot(vert1, surfaceNormal);

                        float numerator = -(dot(surfaceNormal, rayOrigin) - D);

                        float t = numerator/denominator;

                        if (t < 0.00000001f) {
                            // Haven't hit yet
                            j++;
                            continue;
                        }
                        Vec3 p = rayOrigin.plus(rayDirection.times(t));
                        if (t < maxT && pointLiesOnTriangle(p, vert1, vert2, vert3, e1, e2)) {
                            boid.coords = p;
                            boid.velocity = reflect(boid.velocity, surfaceNormal);
                        }
                        j++;
                    }
                }
            }
        }
    }

    public void perchBoid(int idx, Vec3 p, Vec3 launchDir) {
        // Hold boid at current position for a ranged amount of time.
        // Then release the boid in the direction provided
        Boid boid = boids.get(idx);
        boid.coords = p;
        boid.perchCountdown = random(3,4);
        boid.velocity = launchDir.normalized().times(maxSpeed/3);
    }
    
    public void addBoid() {
        Vec3 position = new Vec3(random(minX, maxX),
                                     random(minY, maxY)/2,
                                     random(minZ, maxZ));
        Boid boid = new Boid();
        boid.coords = position;
        boid.previousCoord = position;
        boid.perchCountdown = 0.f;
        boid.id = boidCount;

        Vec3 velocity = new Vec3(random(-1,1),random(-1,1),random(-1,1));
        velocity.normalize();
        velocity.mul(random(maxSpeed/2, maxSpeed));
        boid.velocity = velocity;

        boids.add(boid);
        boidCount++;
    }

    public void loseBoid() {
        boids.remove(0);
        boidCount--;
    }

    public void initializePositions(Vec3 spawnPoint) {
        // Given the bounding box, we will randomly place our boids within this box
        for (int i = 0; i < boidCount; i++) {
            Vec3 position = new Vec3(random(minX, maxX),
                                     random(minY, maxY)/2,
                                     random(minZ, maxZ));
            position.plus(spawnPoint);
            Boid boid = new Boid();
            boid.coords = position;
            boid.previousCoord = position;
            boid.perchCountdown = 0.f;
            boid.id = i;

            Vec3 velocity = new Vec3(random(-1,1),random(-1,1),random(-1,1));
            velocity.normalize();
            velocity.mul(random(maxSpeed/2, maxSpeed));
            boid.velocity = velocity;

            boids.add(boid);
        }
    }

    public void updateBoidPositions(float dt) {
        Vec3 v1, v2, v3, v4;
        for (Boid boid : boids) {
            if(boid.perchCountdown > 0) {
                boid.perchCountdown -= dt;
                continue;
            }

            v1 = flyTowardsCenter(boid);
            v2 = keepDistance(boid);
            v3 = matchVelocity(boid);
            v4 = moveToTetherPoint(boid);

            // Velocity
            boid.velocity.add(v1.times(0.6f).plus(v2.times(0.6f)).plus(v3).plus(v4));
            boid.velocity.y *= 0.9f;
            boid.velocity.clamp(0, maxSpeed);
            // Position
            boid.previousCoord = new Vec3(boid.coords);
            boid.coords.add(boid.velocity.times(dt));

            boundPosition(boid);
        }
    }

    public void drawBoids() {
        if (boidTexture != null) {
            for (Boid boid : boids) {
                // Draw texture
                Vec3 pos = boid.coords;
                push();
                translate(pos.x, pos.y, pos.z);
                beginShape();
                texture(boidTexture);
                vertex(-boidSize/2,boidSize/2,0,0,0);
                vertex(boidSize/2,boidSize/2,0,boidTexture.width,0);
                vertex(boidSize/2,-boidSize/2,0,boidTexture.width,boidTexture.height);
                vertex(-boidSize/2,-boidSize/2,0,0,boidTexture.height);
                endShape();
                pop();
            }
        }
        else if (boidModel != null) {
            for (Boid boid : boids) {
                // Draw boid model
                Vec3 pos = boid.coords;
                if (pos.distanceTo(cam.camLocation) > 30) {
                    push();
                    stroke(boidColor.x, boidColor.y, boidColor.z);
                    strokeWeight(3);
                    point(pos.x, pos.y, pos.z);
                    pop();
                    continue;
                }
                Vec3 vel = boid.velocity.normalized();
                pushMatrix();
                translate(pos.x, pos.y, pos.z);
                Vec3 xz_vec = new Vec3(vel.x, 0, vel.z);
                xz_vec.normalize();
                float angle = acos(dot(xz_vec, new Vec3(0,0,1)));
                if (cross(xz_vec, new Vec3(0,0,1)).y > 0) {
                    rotateY(-angle-PI/2);
                } else {
                    rotateY(angle-PI/2);
                }
                if (boid.perchCountdown > 0) {
                    shape(boidPerchedModel);
                } else {
                    shape(boidModel);
                }
                popMatrix();
            }
        }
        else {
            for (Boid boid : boids) {
                // Draw a point
                Vec3 pos = boid.coords;
                push();
                stroke(boidColor.x, boidColor.y, boidColor.z);
                strokeWeight(boidSize);
                point(pos.x, pos.y, pos.z);
                pop();
            }
        }
        if (debugMode) {
            push();
            stroke(255, 0, 0);
            strokeWeight(1);
            for(Boid boid : boids) {
                Vec3 endPtVelDir = boid.coords.plus(boid.velocity.times(0.5f));
                line(boid.coords.x, boid.coords.y, boid.coords.z,
                     endPtVelDir.x, endPtVelDir.y, endPtVelDir.z);
            }
            pop();
        }
    }

    private void boundPosition(Boid boid) {
        Vec3 pos = boid.coords;
        Vec3 vel = boid.velocity;
        // Check for boids outside the bounding box
        if (pos.x > maxX) {
            pos.x = maxX;
            vel.x = -20;
        } else if (pos.x < minX) {
            pos.x = minX;
            vel.x = 20;
        }
        if (pos.y > maxY) {
            pos.y = maxY;
            vel.y = -20;
        } else if (pos.y < minY) {
            pos.y = minY;
            vel.y = 20;
        }
        if (pos.z > maxZ) {
            pos.z = maxZ;
            vel.z = -20;
        } else if (pos.z < minZ) {
            pos.z = minZ;
            vel.z = 20;
        }
    }

    private Vec3 flyTowardsCenter(Boid boid) {
        // given an index of a certain boid,
        // find the center of mass of nearby boids
        // and return a weight in that direction
        
        // Fly towards the center of mass excluding the current boid
        Vec3 perceivedCenterOfMass = new Vec3(0,0,0);
        int cnt = 0;
        int i = 0;
        for (Boid otherBoid : boids) {
            if (otherBoid.equals(boid) || otherBoid.perchCountdown > 0) {
                i++;
                continue;
            } else if (boid.coords.distanceTo(otherBoid.coords) < visualDistance) {
                perceivedCenterOfMass.add(otherBoid.coords);
                cnt++;
            }
            i++;
        }
        if (cnt > 0) {
            perceivedCenterOfMass.mul(1.f/(cnt));
        }
        return perceivedCenterOfMass.minus(boid.coords).times(influenceToCenter);
    }

    private Vec3 keepDistance(Boid boid) {
        // look at all the nearby boids and objects
        // and return a weight that maneuvers away from them
        Vec3 c = new Vec3(0,0,0);

        int i = 0;
        for (Boid otherBoid : boids) {
            if (otherBoid.equals(boid)) {
                i++;
                continue;
            }
            Vec3 betweenVec = otherBoid.coords.minus(boid.coords);
            if (betweenVec.length() < separation) {
                c.subtract(betweenVec);
            }
            i++;
        }
        return c;
    }

    private Vec3 matchVelocity(Boid boid) {
        // find the velocities of nearby boids and approximate it
        Vec3 curBoidVelocty = boid.velocity;
        Vec3 newPosition = new Vec3(0,0,0);
        int cnt = 0;
        int i = 0;
        for (Boid otherBoid : boids) {
            if (otherBoid.equals(boid) || otherBoid.perchCountdown > 0) {
                i++;
                continue;
            } else if (boid.coords.distanceTo(otherBoid.coords) < visualDistance) {
                newPosition.add(otherBoid.velocity);
                cnt++;
            }
            i++;
        }
        if (cnt > 0) {
            newPosition.mul(1.f/(cnt));
        }
        newPosition.subtract(curBoidVelocty);
        newPosition.mul(1.f/8);
        return newPosition;
    }

    public Vec3 moveToTetherPoint(Boid boid) {
        return tetherPoint.minus(boid.coords).normalized().times(influenceToTetherPoint);
    }

    public Vec3 noisyMovement() {
        return new Vec3(random(-maxSpeed/200, maxSpeed/200),
                        random(-maxSpeed/200, maxSpeed/200),
                        random(-maxSpeed/200, maxSpeed/200));
    }
}

public class Boid {
    Vec3 coords;
    Vec3 previousCoord;
    float perchCountdown;
    Vec3 velocity;
    int id;
}
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
                theta -= 0.8f;
            } else if (keys[5]) {
                theta += 0.8f;
            }
        }
        camLocation.x = cos(radians(theta))*radius;
        camLocation.y = PApplet.parseFloat(slider)/5;
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

/////////
// Point Intersection Tests
/////////

//Returns true if the point is inside a box
public boolean pointInBox(Vec2 boxTopLeft, float boxW, float boxH, Vec2 pointPos){
  Vec2 diff = pointPos.minus(boxTopLeft);
  if (diff.x > 0 && diff.x < boxW && diff.y > 0 && diff.y < boxH) return true;
  return false;
}

//Returns true if the point is inside a circle
public boolean pointInCircle(Vec2 center, float r, Vec2 pointPos){
  float dist = pointPos.distanceTo(center);
  if (dist < r+2){ //small safty factor
    return true;
  }
  return false;
}

//Returns true if the point is inside a list of circle
public boolean pointInCircleList(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos){
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (pointInCircle(center,r,pointPos)){
      return true;
    }
  }
  return false;
}


/////////
// Ray Intersection Tests
/////////

class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

public hitInfo rayBoxIntersect(Vec2 boxTopLeft, float boxW, float boxH, Vec2 ray_start, Vec2 ray_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.hit = true;
  
  float t_left_x, t_right_x, t_top_y, t_bot_y;
  t_left_x = (boxTopLeft.x - ray_start.x)/ray_dir.x;
  t_right_x = (boxTopLeft.x + boxW - ray_start.x)/ray_dir.x;
  t_top_y = (boxTopLeft.y - ray_start.y)/ray_dir.y;
  t_bot_y = (boxTopLeft.y + boxH - ray_start.y)/ray_dir.y;
  
  float t_max_x = max(t_left_x,t_right_x);
  float t_max_y = max(t_top_y,t_bot_y);
  float t_max = min(t_max_x,t_max_y); //When the ray exists the box
  
  float t_min_x = min(t_left_x,t_right_x);
  float t_min_y = min(t_top_y,t_bot_y);
  float t_min = max(t_min_x,t_min_y); //When the ray enters the box
  
  
  //The the box is behind the ray (negative t)
  if (t_max < 0){
    hit.hit = false;
    hit.t = t_max;
    return hit;
  }
  
  //The ray never hits the box
  if (t_min > t_max){
    hit.hit = false;
  }
  
  //The ray hits, but further out than max_t
  if (t_min > max_t){
    hit.hit = false;  }
  
  hit.t = t_min;
  return hit;
}

public hitInfo rayCircleIntersect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Length of l_dir (we normalized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r*r); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the length of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only need the first collision
      float t2 = (-b + sqrt(d))/(2*a); //Optimization: we only need the first collision
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      }
      else if (t1 < 0 && t2 > 0){
        hit.hit = true;
        hit.t = -1;
      }
      
    }
    
  return hit;
}

public hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii,  int numObstacles, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.t = max_t;
  for (int i = 0; i < numObstacles; i++){
    Vec2 center = centers[i];
    float r = radii[i];
    
    hitInfo circleHit = rayCircleIntersect(center, r, l_start, l_dir, hit.t);
    if (circleHit.t > 0 && circleHit.t < hit.t){
      hit.hit = true;
      hit.t = circleHit.t;
    }
    else if (circleHit.hit && circleHit.t < 0){
      hit.hit = true;
      hit.t = -1;
    }
  }
  return hit;
}
public class CollisionTrigger {
    boolean isActive = false;

    public void onCollision(Vec3 point, Vec3 normal) {
        isActive = true;
        return;
    }

    public CollisionTrigger copy() {
        return new CollisionTrigger();
    }

    public void update(float dt) {
    }

    public void draw() {
    }
}

public class SpawnEmitter extends CollisionTrigger {
    ParticleSystem emitter;

    public SpawnEmitter() {
        emitter = new ParticleSystem(100);
        emitter.streakLength = 0;
        emitter.particleLifespanMax = 0.3f;
        emitter.particleLifespanMin = 0.2f;
        emitter.birthRate = 10;
        emitter.emitterLifespan = 0.1f;
        emitter.r = 1.3f;
        emitter.particleSpeed = 20;
        emitter.speedRange = 10;
        emitter.particleDirection = new Vec3(0,1,0);
        emitter.particleDirectionRange = 0.1f;
        emitter.particleAcceleration = new Vec3(0,-500,0);
        emitter.particleAccelerationRange = 0.1f;
    }

    @Override
    public SpawnEmitter copy() {
        return new SpawnEmitter();
    }

    @Override
    public void onCollision(Vec3 point, Vec3 normal) {
        // TODO : Spawn a new emitter at the point of collision
        super.onCollision(point, normal);
        emitter.emitterPosition = point;
        emitter.particleDirection = normal;
    }

    @Override 
    public void update(float dt) {
        emitter.generateNewParticles(dt);
        emitter.updateParticlePositions(dt);
        emitter.updateParticleProperties(dt);
        emitter.removeDeadParticles();
        emitter.updateTriggers(dt);
    }

    @Override
    public void draw() {
        while (emitter.drawNextParticle()) {
        }
        emitter.drawTriggers();
        this.isActive = emitter.isActive;
    }
}

public class AnimateRaindropCollision extends CollisionTrigger {
    @Override
    public void onCollision(Vec3 point, Vec3 normal) {
        super.onCollision(point, normal);
    }
}

public class TriggerCollection {
    public ArrayList<CollisionTrigger> triggers;

    public TriggerCollection() {
        triggers = new ArrayList<CollisionTrigger>();
    }

    public void add(CollisionTrigger trigger) {
        triggers.add(trigger);
    }

    public void updateAllTriggers(float dt) {
        int i = 0;
        while (i < triggers.size()) {
            CollisionTrigger trigger = triggers.get(i);

            trigger.update(dt);

            if (!trigger.isActive) {
                // Remove any inactive triggers
                triggers.remove(i);
                i--;
            }
            i++;
        }
    }

    public void drawAllTriggers() {
        int i = 0;
        while (i < triggers.size()) {
            CollisionTrigger trigger = triggers.get(i);
            trigger.draw();
            i++;
        }
    }
}

public class AnimateExplosion extends CollisionTrigger {
    ParticleSystem explosion;

    public AnimateExplosion() {
        explosion = new ParticleSystem(100);
        explosion.streakLength = 0;
        explosion.particleLifespanMax = 1.0f;
        explosion.particleLifespanMin = 0.7f;
        explosion.birthRate = 200;
        explosion.emitterLifespan = 0.2f;
        explosion.r = 5;
        explosion.particleSpeed = 20;
        explosion.speedRange = 5;
        explosion.particleDirection = new Vec3(0,0,0);
        explosion.particleDirectionRange = 1;
        explosion.particleAcceleration = new Vec3();
    }

    @Override
    public AnimateExplosion copy() {
        return new AnimateExplosion();
    }

    @Override
    public void onCollision(Vec3 point, Vec3 normal) {
        super.onCollision(point, normal);
        explosion.emitterPosition = point;
    }

    @Override
    public void update(float dt) {
        explosion.generateNewParticles(dt);
        explosion.updateParticlePositions(dt);
        explosion.updateParticleProperties(dt);
        explosion.removeDeadParticles();
        explosion.updateTriggers(dt);
    }

    @Override
    public void draw() {
        while (explosion.drawNextParticle()) {
        }
        explosion.drawTriggers();
        this.isActive = explosion.isActive;
    }
}
// Max Omdal 2020

class Cylinder {
    float radius;
    float height;
    Vec3 center;
    float sides = 20;
    Vec3 materialColor = new Vec3(255,255,255);
    PShape mesh;

    public Cylinder(float radius, float height, Vec3 center) {
        this.radius = radius;
        this.height = height;
        this.center = center;
        calculateMesh();
    }

    public void calculateMesh() {
        float theta = 0;
        float stepSize = 2*PI/sides;
        this.mesh = createShape(GROUP);
        PShape barrel = createShape();
        barrel.beginShape(TRIANGLES);
        barrel.noStroke();
        barrel.fill(materialColor.x, materialColor.y, materialColor.z);
        // Draw the sides of the cylinder
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            Vec3 vertexPos = new Vec3(this.center.x + this.radius * cos(theta),
                                    this.center.y + this.height/2,
                                    this.center.z + this.radius * sin(theta));
            Vec3 vertexPos2 = new Vec3(this.center.x + this.radius * cos(theta+stepSize),
                                       this.center.y + this.height/2,
                                       this.center.z + this.radius * sin(theta+stepSize));
            Vec3 vecNormal = center.minus(vertexPos).normalized();
            // barrel.normal(vecNormal.x, vecNormal.y, vecNormal.z);

            barrel.vertex(vertexPos.x, vertexPos.y, vertexPos.z);
            barrel.vertex(vertexPos.x, vertexPos.y - this.height, vertexPos.z);
            barrel.vertex(vertexPos2.x, vertexPos2.y - this.height, vertexPos2.z);
            
            barrel.vertex(vertexPos2.x, vertexPos2.y - this.height, vertexPos2.z);
            barrel.vertex(vertexPos2.x, vertexPos2.y, vertexPos2.z);
            barrel.vertex(vertexPos.x, vertexPos.y, vertexPos.z);
        }
        barrel.endShape(CLOSE);
        
        // Draw the caps
        PShape cap1 = createShape();
        theta = 0;
        cap1.beginShape(TRIANGLES);
        cap1.noStroke();
        cap1.fill(materialColor.x, materialColor.y, materialColor.z);
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            cap1.vertex(this.center.x + this.radius * cos(theta),
                   this.center.y + this.height/2,
                   this.center.z + this.radius * sin(theta));
            cap1.vertex(this.center.x + this.radius * cos(theta+stepSize),
                        this.center.y + this.height/2,
                        this.center.z + this.radius * sin(theta+stepSize));
            cap1.vertex(this.center.x, this.center.y + this.height/2, this.center.z);
        }
        cap1.endShape();

        PShape cap2 = createShape();
        theta = 0;
        cap2.beginShape(TRIANGLES);
        cap2.noStroke();
        cap2.fill(materialColor.x, materialColor.y, materialColor.z);
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            cap2.vertex(this.center.x + this.radius * cos(theta),
                   this.center.y - this.height/2,
                   this.center.z + this.radius * sin(theta));
            cap2.vertex(this.center.x + this.radius * cos(theta+stepSize),
                        this.center.y - this.height/2,
                        this.center.z + this.radius * sin(theta+stepSize));
            cap2.vertex(this.center.x, this.center.y - this.height/2, this.center.z);
        }
        cap2.endShape();

        this.mesh.addChild(barrel);
        this.mesh.addChild(cap1);
        this.mesh.addChild(cap2);
    }

    public void draw() {
        shape(this.mesh);
    }
}
// Max Omdal 2020

class Enemy {
    Vec3 position;
    Vec2 position2D;
    float speed = 20;
    Vec2 velocity;
    float collisionRadius = 2;
    PImage hoverPointer;
    float hoverBounce = 0;
    boolean freeToMove = false;
    PathTraversal pathMaker;
    float orientationAngle = 0;
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

    public void avoidProbes(ArrayList<Probe> probes) {
        // Use time to collision to avoid intersection with projectile
        for (Probe probe : probes) {
            Vec3 posToProbe = probe.position.minus(this.position);
            float timeToCollision = posToProbe.length();
        }
    }

    public void update(float dt) {
        hoverBounce += 3*dt;
        
        if (freeToMove) {
            if (pathMaker.path != null && pathMaker.path.size() > 1) {
                // TODO: use path to target to take forward movement
                this.pathMaker.updateMovementDir(position2D);
                this.velocity = this.pathMaker.movementDir.times(speed);
                this.position2D.add(this.velocity.times(dt));
                this.position.x = this.position2D.x;
                this.position.z = this.position2D.y;
                if (pathMaker.destinationReached) {
                    freeToMove = false;
                }
            }
        }
    }

    public void draw() {
        Cylinder body = new Cylinder(0.3f,2, this.position.plus(new Vec3(0, 1, 0)));
        body.materialColor = new Vec3(255,255,10);

        Vec2 movementDir = new Vec2(0,0);
        Vec2 nextNodePos = pathMaker.getNextNodeOnPath();
        if (nextNodePos != null) {
            movementDir = nextNodePos.minus(position2D).normalized();
        }
        float movementAngle = atan2(movementDir.y, movementDir.x);
        if (movementAngle - orientationAngle > PI) {
            orientationAngle = orientationAngle + 0.05f * (2*PI - movementAngle - orientationAngle);
        } else {
            orientationAngle = orientationAngle + 0.05f * (movementAngle - orientationAngle);
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
        translate(position.x, position.y + 5 + 0.5f*cos(hoverBounce), position.z);
        Vec3 camToPlayer = position.minus(cam.camLocation).normalized();
        Vec2 camToPlayer2D = new Vec2(camToPlayer.x, camToPlayer.z).normalized();
        float angle = atan2(camToPlayer2D.y,camToPlayer2D.x);
        rotateY(-angle-3*PI/2);
        shader(unlitShader);
        beginShape();
        textureMode(NORMAL);
        texture(hoverPointer);
        vertex(-1,1.5f,0,0,0);
        vertex(1,1.5f,0,1,0);
        vertex(1,-1.5f,0,1,1);
        vertex(-1,-1.5f,0,0,1);
        endShape();
        resetShader();
        pop();
    }

}
class OctantPoints {
    Vec3 origin;
    Vec3 size;
    int capacity;
    float xmin, xmax, ymin, ymax, zmin, zmax;
    Vec3[] bounds = new Vec3[2];

    public OctantPoints(Vec3 origin, Vec3 size) {
        this.origin = origin;
        this.size = size;

        this.xmin = origin.x - size.x/2;
        this.xmax = origin.x + size.x/2;
        this.ymin = origin.y - size.y/2;
        this.ymax = origin.y + size.y/2;
        this.zmin = origin.z - size.z/2;
        this.zmax = origin.z + size.z/2;
        bounds[0] = new Vec3(xmin,ymin,zmin);
        bounds[1] = new Vec3(xmax,ymax,zmax);
    }

    public boolean contains(Vec3 p) {
        return (p.x >= xmin &&
                p.x < xmax &&
                p.y >= ymin &&
                p.y < ymax &&
                p.z >= zmin &&
                p.z < zmax);
    }

    public boolean rayIntersects(Ray3 ray) {
        // Credit goes to https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

        // 1.) Check if ray is contained inside the OctantPoints
        if (contains(ray.origin)) {
            return true;
        }
        // 2.) Check if ray passes through the oct
        float tmin, tmax, tymin, tymax, tzmin, tzmax;

        tmin = (bounds[ray.sign[0]].x - ray.origin.x) * ray.invDir.x;
        tmax = (bounds[1-ray.sign[0]].x - ray.origin.x) * ray.invDir.x;
        tymin = (bounds[ray.sign[1]].y - ray.origin.y) * ray.invDir.y;
        tymax = (bounds[1-ray.sign[1]].y - ray.origin.y) * ray.invDir.y;

        if ((tmin > tymax) || (tymin > tmax)) {
            return false;
        }
        if (tymin > tmin) {
            tmin = tymin;
        }
        if (tymax < tmax) {
            tmax = tymax;
        }

        tzmin = (bounds[ray.sign[2]].z - ray.origin.z) * ray.invDir.z; 
        tzmax = (bounds[1-ray.sign[2]].z - ray.origin.z) * ray.invDir.z; 
    
        if ((tmin > tzmax) || (tzmin > tmax)) {
            return false;
        }
        if (tzmin > tmin) {
            tmin = tzmin;
        }
        if (tzmax < tmax) {
            tmax = tzmax;
        }

        // if (tmax < 0.f) {
        //     return false;
        // }

        // if (ray.magnitude > 0 && ray.magnitude < tmax) {
        //     return false;
        // }

        return true; 
    }
}
class OctantTris {
    Vec3 origin;
    Vec3 size;
    int capacity;
    float xmin, xmax, ymin, ymax, zmin, zmax;
    Vec3[] bounds = new Vec3[2];

    public OctantTris(Vec3 origin, Vec3 size) {
        this.origin = origin;
        this.size = size;

        this.xmin = origin.x - size.x/2;
        this.xmax = origin.x + size.x/2;
        this.ymin = origin.y - size.y/2;
        this.ymax = origin.y + size.y/2;
        this.zmin = origin.z - size.z/2;
        this.zmax = origin.z + size.z/2;
        bounds[0] = new Vec3(xmin,ymin,zmin);
        bounds[1] = new Vec3(xmax,ymax,zmax);
    }

    public boolean contains(PShape tri) {
        for (int i = 0; i < 3; i++) {
            Vec3 p = new Vec3(tri.getVertex(i)); // Convert PVector to Vec3
            if (p.x >= xmin &&
                                   p.x < xmax &&
                                   p.y >= ymin &&
                                   p.y < ymax &&
                                   p.z >= zmin &&
                                   p.z < zmax) {
                                       return true;
                                   }
        }
        return false;
    }

    public boolean contains(Vec3 p) {
        return (p.x >= xmin &&
                p.x < xmax &&
                p.y >= ymin &&
                p.y < ymax &&
                p.z >= zmin &&
                p.z < zmax);
    }

    public boolean rayIntersects(Ray3 ray) {
        // Credit goes to https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

        // 1.) Check if ray is contained inside the OctantTris
        if (contains(ray.origin)) {
            return true;
        }
        // 2.) Check if ray passes through the oct
        float tmin, tmax, tymin, tymax, tzmin, tzmax;

        tmin = (bounds[ray.sign[0]].x - ray.origin.x) * ray.invDir.x;
        tmax = (bounds[1-ray.sign[0]].x - ray.origin.x) * ray.invDir.x;
        tymin = (bounds[ray.sign[1]].y - ray.origin.y) * ray.invDir.y;
        tymax = (bounds[1-ray.sign[1]].y - ray.origin.y) * ray.invDir.y;

        if ((tmin > tymax) || (tymin > tmax)) {
            return false;
        }
        if (tymin > tmin) {
            tmin = tymin;
        }
        if (tymax < tmax) {
            tmax = tymax;
        }

        tzmin = (bounds[ray.sign[2]].z - ray.origin.z) * ray.invDir.z; 
        tzmax = (bounds[1-ray.sign[2]].z - ray.origin.z) * ray.invDir.z; 
    
        if ((tmin > tzmax) || (tzmin > tmax)) {
            return false;
        }
        if (tzmin > tmin) {
            tmin = tzmin;
        }
        if (tzmax < tmax) {
            tmax = tzmax;
        }

        if (tmax < 0.f) {
            return false;
        }

        if (ray.magnitude > 0 && ray.magnitude < tmax) {
            return false;
        }

        return true; 
    }

    public void show() {
        push();
        noFill();
        strokeWeight(0.8f);
        stroke(255);
        translate(origin.x, origin.y, origin.z);
        box(size.x, size.y, size.z);
        pop();
    }
}
class OctreePoints {
    OctantPoints bounds;
    int capacity;
    ArrayList<Vec3> points;
    boolean divided = false;

    OctreePoints q111,q011,q001,q101,q110,q010,q000,q100;

    public OctreePoints(OctantPoints bounds, int capacity) {
        this.bounds = bounds;
        this.capacity = capacity;
        this.points = new ArrayList<Vec3>();
    }

    public void insert(Vec3 p) {
        if (!this.bounds.contains(p)) {
            return;
        }

        if (this.points.size() < this.capacity) {
            this.points.add(p);
        } else {
            if (!this.divided) {
                subdivide();
            }

            q111.insert(p);
            q011.insert(p);
            q001.insert(p);
            q101.insert(p);
            q110.insert(p);
            q010.insert(p);
            q000.insert(p);
            q100.insert(p);
        }
    }

    public void show() {
        strokeWeight(0.8f);
        stroke(255);
        noFill();
        pushMatrix();
        translate(bounds.origin.x, bounds.origin.y, bounds.origin.z);
        box(bounds.size.x, bounds.size.y, bounds.size.z);
        popMatrix();
        if (this.divided) {
            q111.show();
            q011.show();
            q001.show();
            q101.show();
            q110.show();
            q010.show();
            q000.show();
            q100.show();
        }
        pushStyle();
        strokeWeight(4);
        stroke(255,0,0);
        for (Vec3 p : points) {
            // point(p.x, p.y, p.z);
        }
        popStyle();
    }

    public ArrayList<Vec3> rayIntersectsOctants(Ray3 ray) {
        // Gets all octants that the ray is in and returns the points stored in those octants
        
        ArrayList<Vec3> pointsInOctants = new ArrayList<Vec3>();
        
        if (bounds.rayIntersects(ray)) {
            if (divided) {
                // Recursively divide
                pointsInOctants.addAll(q111.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q011.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q001.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q101.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q110.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q010.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q000.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q100.rayIntersectsOctants(ray));
            } else {
                // Just add the points in this oct
                pointsInOctants.addAll(points);
            }
        }

        return pointsInOctants;
    }

    private void subdivide() {
        Vec3 origin = bounds.origin;
        Vec3 size = bounds.size;
        Vec3 subdivideSize = size.times(0.5f);

        OctantPoints q111_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5f,0.5f,0.5f))), subdivideSize);
        q111 = new OctreePoints(q111_b, capacity);
        OctantPoints q011_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5f,0.5f,0.5f))), subdivideSize);
        q011 = new OctreePoints(q011_b, capacity);
        OctantPoints q001_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5f,-0.5f,0.5f))), subdivideSize);
        q001 = new OctreePoints(q001_b, capacity);
        OctantPoints q101_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5f,-0.5f,0.5f))), subdivideSize);
        q101 = new OctreePoints(q101_b, capacity);
        OctantPoints q110_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5f,0.5f,-0.5f))), subdivideSize);
        q110 = new OctreePoints(q110_b, capacity);
        OctantPoints q010_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5f,0.5f,-0.5f))), subdivideSize);
        q010 = new OctreePoints(q010_b, capacity);
        OctantPoints q000_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5f,-0.5f,-0.5f))), subdivideSize);
        q000 = new OctreePoints(q000_b, capacity);
        OctantPoints q100_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5f,-0.5f,-0.5f))), subdivideSize);
        q100 = new OctreePoints(q100_b, capacity);

        this.divided = true;
    }

}
class OctreeTriangles {
    OctantTris bounds;
    int capacity;
    ArrayList<PShape> tris;
    boolean divided = false;

    OctreeTriangles q111,q011,q001,q101,q110,q010,q000,q100;

    public OctreeTriangles(OctantTris bounds, int capacity) {
        this.bounds = bounds;
        this.capacity = capacity;
        this.tris = new ArrayList<PShape>();
    }

    public void insert(PShape tri) {
        if (!this.bounds.contains(tri)) {
            return;
        }

        if (!this.divided && this.tris.size() < this.capacity) {
            this.tris.add(tri);
        } else {
            if (!this.divided) {
                subdivide();
            }

            q111.insert(tri);
            q011.insert(tri);
            q001.insert(tri);
            q101.insert(tri);
            q110.insert(tri);
            q010.insert(tri);
            q000.insert(tri);
            q100.insert(tri);
        }
    }

    public void show(boolean includeTris) {
        bounds.show();
        if (this.divided) {
            q111.show(includeTris);
            q011.show(includeTris);
            q001.show(includeTris);
            q101.show(includeTris);
            q110.show(includeTris);
            q010.show(includeTris);
            q000.show(includeTris);
            q100.show(includeTris);
        }
        pushStyle();
        strokeWeight(4);
        stroke(255,0,0);
        if (includeTris) {
            for (PShape p : tris) {
                // shape(p);
            }
        }
        popStyle();
    }

    public ArrayList<PShape> rayIntersectsOctants(Ray3 ray) {
        // Gets all OctantTriss that the ray is in and returns the points stored in those OctantTriss
        ArrayList<PShape> pointsInOctantTris = new ArrayList<PShape>();
        
        if (bounds.rayIntersects(ray)) {
            if (divided) {
                // Recursively divide
                pointsInOctantTris.addAll(q111.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q011.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q001.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q101.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q110.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q010.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q000.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q100.rayIntersectsOctants(ray));
            } else {
                // Just add the points in this oct
                pointsInOctantTris.addAll(tris);
            }
        }

        return pointsInOctantTris;
    }

    private void subdivide() {
        Vec3 origin = bounds.origin;
        Vec3 size = bounds.size;
        Vec3 subdivideSize = size.times(0.5f);

        OctantTris q111_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5f,0.5f,0.5f))), subdivideSize);
        q111 = new OctreeTriangles(q111_b, capacity);
        OctantTris q011_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5f,0.5f,0.5f))), subdivideSize);
        q011 = new OctreeTriangles(q011_b, capacity);
        OctantTris q001_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5f,-0.5f,0.5f))), subdivideSize);
        q001 = new OctreeTriangles(q001_b, capacity);
        OctantTris q101_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5f,-0.5f,0.5f))), subdivideSize);
        q101 = new OctreeTriangles(q101_b, capacity);
        OctantTris q110_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5f,0.5f,-0.5f))), subdivideSize);
        q110 = new OctreeTriangles(q110_b, capacity);
        OctantTris q010_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5f,0.5f,-0.5f))), subdivideSize);
        q010 = new OctreeTriangles(q010_b, capacity);
        OctantTris q000_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5f,-0.5f,-0.5f))), subdivideSize);
        q000 = new OctreeTriangles(q000_b, capacity);
        OctantTris q100_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5f,-0.5f,-0.5f))), subdivideSize);
        q100 = new OctreeTriangles(q100_b, capacity);

        // Insert all leaf nodes in children now
        for (PShape tri : tris) {
            q111.insert(tri);
            q011.insert(tri);
            q001.insert(tri);
            q101.insert(tri);
            q110.insert(tri);
            q010.insert(tri);
            q000.insert(tri);
            q100.insert(tri);
        }

        tris = new ArrayList<PShape>();

        this.divided = true;
    }

}
//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of obstacles
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of position in the nodePos array will be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it use BFS which will not provide the shortest path
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment (don't assume 
// this example funcationality is correct and copy it's mistakes!).





//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node
ArrayList<Integer> nodesTouchingStart = new ArrayList<Integer>();
ArrayList<Integer> nodesTouchingGoal = new ArrayList<Integer>();

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
public void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//This is probably a bad idea and you shouldn't use it...
public int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

public ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  // int startID = closestNode(startPos, nodePos, numNodes);
  // int goalID = closestNode(goalPos, nodePos, numNodes);
  int startID = numNodes;
  int goalID = numNodes + 1;

  // Add start and end node to graph
  numNodes+=2;
  nodePos[startID] = startPos;
  nodePos[goalID] = goalPos;

  neighbors[startID] = new ArrayList<Integer>();
  neighbors[goalID] = new ArrayList<Integer>();
  nodesTouchingGoal = new ArrayList<Integer>();
  nodesTouchingStart = new ArrayList<Integer>();
  // Set up neighbors
  if (pointInCircleList(centers, radii, numObstacles, startPos) || pointInCircleList(centers, radii, numObstacles, goalPos)) {
    path.add(0,-1);
    return path;
  }
  for (int i = 0; i < numNodes; i++) {
    Vec2 startToNode = nodePos[i].minus(startPos);
    if (!rayCircleListIntersect(centers, radii, numObstacles, startPos, startToNode.normalized(), startToNode.length()).hit) {
      neighbors[i].add(startID);
      nodesTouchingStart.add(i);
      neighbors[startID].add(i);
    }
    Vec2 nodeToGoal = goalPos.minus(nodePos[i]);
    if (!rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], nodeToGoal.normalized(), nodeToGoal.length()).hit) {
      neighbors[i].add(goalID);
      nodesTouchingGoal.add(i);
      neighbors[goalID].add(i);
    }
  }

  path = runAStar(nodePos, numNodes, startID, goalID);

  // Reset neighbors
  for (int idx : nodesTouchingStart) {
    neighbors[idx].remove(neighbors[idx].size() - 1);
  }
  for (int idx : nodesTouchingGoal) {
    neighbors[idx].remove(neighbors[idx].size() - 1);
  }
  
  return path;
}

class NodeHeuristic {
  float distanceFromStart;
  float heuristicToGoal;
  int uid;
  boolean visited;
  NodeHeuristic parent;
  Vec2 pos;
}

class SortByPathLength implements Comparator<NodeHeuristic> {
  public int compare(NodeHeuristic a, NodeHeuristic b) {
    return (int)((a.heuristicToGoal + a.distanceFromStart) - (b.heuristicToGoal + b.distanceFromStart));
  }
}

//A*
public ArrayList<Integer> runAStar(Vec2[] nodePos, int numNodes, int startID, int goalID) {
  NodeHeuristic[] nodes = new NodeHeuristic[numNodes];
  Comparator<NodeHeuristic> comparator = new SortByPathLength();
  PriorityQueue<NodeHeuristic> fringeQueue = new PriorityQueue<NodeHeuristic>(10, comparator);
  ArrayList<Integer> path = new ArrayList(); // Will resolve to final path

  // 0.) Clear visit tags and parent pointers, and define all heuristics
  for (int i = 0; i < numNodes; i++) {
    visited[i] = false;
    parent[i] = -1;
    nodes[i] = new NodeHeuristic();
    nodes[i].distanceFromStart = Float.MAX_VALUE;
    nodes[i].heuristicToGoal = nodePos[i].minus(nodePos[goalID]).length();
    nodes[i].uid = i;
    nodes[i].visited = false;
    nodes[i].pos = nodePos[i];
  }

  // 0.1.) Special case is first pt.
  nodes[startID].distanceFromStart = 0;
  fringeQueue.add(nodes[startID]);

  NodeHeuristic currentNode = fringeQueue.peek();
  
  while (fringeQueue.size() > 0) {
    // 1.) Choose the path with shortest length + heuristic and update visited
    currentNode = fringeQueue.poll();

    if (currentNode.uid == goalID) {
      // We've found the goal, but we don't yet know if it is optimal. Regardless, we don't get neighbor
      return reconstructPath(currentNode);
    }

    for (int neighbor : neighbors[currentNode.uid]) {
      NodeHeuristic neighborNode = nodes[neighbor];
      // if (neighborNode.visited == false) {
        // neighborNode.visited = true;
        float newDistance = currentNode.distanceFromStart + currentNode.pos.distanceTo(neighborNode.pos);
        if (newDistance < neighborNode.distanceFromStart) {
          // We can safely ignore nodes that have a distance greater than the min path because the heuristic will always be a lower bound
          neighborNode.parent = currentNode;
          neighborNode.distanceFromStart = newDistance;
          fringeQueue.add(neighborNode);
        }
      // }
    }
  }

  path.add(0,-1);
  return path;
}

public ArrayList<Integer> reconstructPath(NodeHeuristic goal) {
  ArrayList<Integer> path = new ArrayList<Integer>();
  // Reconstruct path from current node
  NodeHeuristic curNode = goal;
  while (curNode != null) {
    path.add(curNode.uid);
    curNode = curNode.parent;
  }

  Collections.reverse(path);

  return path;
}

//BFS (Breadth First Search)
public ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  return path;
}


public void drawPRMGraph() {
    push();
    stroke(0,255,255);
    strokeWeight(5);
    for (int i = 0; i < numNodes; i++) {
        point(nodePos[i].x, -3, nodePos[i].y);
    }
    stroke(255,0,0);
    strokeWeight(0.3f);
    for (int i = 0; i < numNodes; i++) {
      Vec3 pos = new Vec3(nodePos[i].x, -3, nodePos[i].y);
      for (int neighbor : neighbors[i]) {
        Vec3 neighborPos = new Vec3(nodePos[neighbor].x, -3, nodePos[neighbor].y);
        line(pos.x, pos.y, pos.z, neighborPos.x, neighborPos.y, neighborPos.z);
      }
    }
    pop();
}

// Vec2 proportionAlongPath(float t, int[] path) {
//   float totalLength = 0;
//   for (int i : path) {
//     totalLength += nodePos[i].length();
//   }
//   float distanceAlongPath = totalLength*t;
//   float tmpLength = 0;
//   for (int i = 0; i < path.length - 1; i++) {
//     int idx1 = path[i];
//     int idx2 = path[i+1];
//     Vec2 btwnNodes = nodePos[idx2].minus(nodePos[idx1]);
//     tmpLength += btwnNodes.length();
//     if (tmpLength > distanceAlongPath) {
//       float segmentT = (distanceAlongPath - (tmpLength - btwnNodes.length())) / btwnNodes.length();
//       return btwnNodes.normalized().times(segmentT);
//     }
//   }
//   return null;
// }

public Vec2 positionAlongPath(float distanceTravelled, int[] path) {
  float totalLength = 0;
  for (int i = 0; i < path.length - 1; i++) {
    int idx1 = path[i];
    int idx2 = path[i+1];
    Vec2 btwnNodes = nodePos[idx2].minus(nodePos[idx1]);
    totalLength += btwnNodes.length();
    if (totalLength > distanceTravelled) {
      return nodePos[idx1].plus(btwnNodes.normalized().times(distanceTravelled - (totalLength - btwnNodes.length())));
    }
  }
  return null;
}

public class ParticleSystem {
  protected ArrayList<Vec3> particleCoords;
  protected ArrayList<Vec3> previousParticleCoords;
  protected ArrayList<Vec3> particleVelocities;
  protected ArrayList<Vec3> particleAccelerations;
  protected ArrayList<Float> particleLifespan;
  protected ArrayList<Vec3> particleColors;
  protected ArrayList<Float> particleRadii;
  protected ArrayList<Float> particleStreakLength;

  int partIdx = 0;
  int streakLength = 0;
  int particleCount = 0;
  int maxParticleCount;
  float particleLifespanMax = 5;
  float particleLifespanMin = 5;
  float emitterLifespan = -1;
  float emitterElapsedTime = 0;
  boolean isActive = true;
  float birthRate = 100;
  float r = 0.2f;
  float particleSpeed = 200;
  float speedRange = 50;
  Vec3 particleDirection = new Vec3(0,-1,0);
  float particleDirectionRange = 0.07f;
  Vec3 particleAcceleration = new Vec3(0,0,0);
  float particleAccelerationRange = 0;
  Vec3 emitterPosition = new Vec3(0,0,0);
  Vec3 particleColor = new Vec3(230,245,255);
  Vec3 particleColorRange = new Vec3(0,0,0);
  PImage particleTexture = null;
  CollisionTrigger collisionTrigger = null;
  TriggerCollection triggerCollection = new TriggerCollection();
  
  public ParticleSystem(int maxParticleCount) {
    particleCoords = new ArrayList<Vec3>();
    previousParticleCoords = new ArrayList<Vec3>();
    particleVelocities = new ArrayList<Vec3>();
    particleAccelerations = new ArrayList<Vec3>();
    particleLifespan = new ArrayList<Float>();
    particleColors = new ArrayList<Vec3>();
    particleRadii = new ArrayList<Float>();
    particleStreakLength = new ArrayList<Float>();
    this.maxParticleCount = maxParticleCount;
  }

  public void generateNewParticles(float dt) {
    // given the change in time, (assuming since the last particle generation)
    // generate more particles and add to the ArrayList of existing particles.
    // Initialize the new particles time, velocities, accelerations, lifespan etc.
    if (emitterLifespan > 0 && emitterElapsedTime >= emitterLifespan) {
      birthRate = 0;
      return;
    }
    emitterElapsedTime += dt;

    float newParticlesToGen = dt * birthRate;
    float decimal = newParticlesToGen - (float)(floor(newParticlesToGen));
    int stochasticNewParticles = floor(newParticlesToGen);
    if (decimal*100 > random(100)) {
      stochasticNewParticles++;
    }
    if (maxParticleCount < particleCount + stochasticNewParticles) {
      stochasticNewParticles = maxParticleCount - particleCount;
    }
    for(int i = 0; i < stochasticNewParticles; i++) {
      particleCoords.add(new Vec3(emitterPosition));
      previousParticleCoords.add(new Vec3(emitterPosition));
      Vec3 particleDir = new Vec3(particleDirection);
      Vec3 randomParticleDir = new Vec3(random(-particleDirectionRange,particleDirectionRange),
                                  random(-particleDirectionRange,particleDirectionRange),
                                  random(-particleDirectionRange,particleDirectionRange));
      particleDir.add(randomParticleDir);
      particleDir.normalize();
      particleVelocities.add(particleDir.times(particleSpeed + random(-speedRange, speedRange)));
      Vec3 particleAccel = new Vec3(particleAcceleration);
      Vec3 randomParticleAccel = new Vec3(random(-particleAccelerationRange,particleAccelerationRange),
                                          random(-particleAccelerationRange,particleAccelerationRange),
                                          random(-particleAccelerationRange,particleAccelerationRange));
      particleAccel.add(randomParticleAccel);
      particleAccelerations.add(particleAcceleration);
      float randLifespan = random(particleLifespanMin, particleLifespanMax);
      particleLifespan.add(randLifespan);
      particleColors.add(new Vec3(particleColor));
      particleRadii.add(r);
      float startingStreakLength = 0.0f;
      particleStreakLength.add(startingStreakLength);
      particleCount++;
    }
  }
  
  public void updateParticlePositions(float dt) {
    // using the particle velocities, acceleration and current position,
    // find its new position.
    for(int i = 0; i < particleCount; i++) {
      previousParticleCoords.set(i, new Vec3(particleCoords.get(i)));
      Vec3 vel = particleVelocities.get(i);
      Vec3 accel = particleAccelerations.get(i);
      Vec3 translation = vel.times(dt).plus(accel.times(0.5f).times(dt*dt));
      particleCoords.get(i).add(translation);
      vel.add(accel.times(dt));
      Float partStreak = particleStreakLength.get(i);
      if (partStreak < streakLength) {
        partStreak += translation.length();
      }
    }
  }
  
  public void checkParticlesForCollisions(PShape[] rigidBodies) {
    // see if any particles are intersecting anything. Move to outside that object
    // and recalculate velocity and acceleration

    if (collisionTrigger == null) {
      // If the collision trigger is null, we will ignore all rigidbodies
      return;
    }

    for (PShape rigidBody : rigidBodies) {
      int triCount = rigidBody.getChildCount();
      for (int i = 0; i < triCount; i++) {
        PShape triangle = rigidBody.getChild(i);
        int j = 0;
        while(j < particleCount) {
          Vec3 particlePosition = particleCoords.get(j);
          // TODO: Use Barycentric Coordinates to find if there is a collision with the surface
          boolean collision = false;
          Vec3 collisionPoint = new Vec3(0,0,0);

          Vec3 rayOrigin = particlePosition;
          Vec3 rayDirection = previousParticleCoords.get(j).minus(particlePosition);
          float maxT = rayDirection.length();

          if (maxT < 0.00001f) {
            j++;
            continue;
          }

          rayDirection.normalize();

          PVector v1 = triangle.getVertex(0);
          PVector v2 = triangle.getVertex(1);
          PVector v3 = triangle.getVertex(2);

          Vec3 vert1 = new Vec3(v1.x, v1.y, v1.z);
          Vec3 vert2 = new Vec3(v2.x, v2.y, v2.z);
          Vec3 vert3 = new Vec3(v3.x, v3.y, v3.z);

          Vec3 e1 = vert2.minus(vert1);
          Vec3 e2 = vert3.minus(vert1);

          Vec3 surfaceNormal = cross(e1, e2);
          // float x_0 = rayOrigin.x; float y_0 = rayOrigin.y; float z_0 = rayOrigin.z;
          // float x_d = rayDirection.x; float y_d = rayDirection.y; float z_d = rayDirection.z;
          float denominator = dot(surfaceNormal, rayDirection);
          if (abs(denominator) <= 0.0001f) {
            // No ray plane intersection exists
            j++;
            continue;
          }

          float D = dot(vert1, surfaceNormal);

          float numerator = -(dot(surfaceNormal, rayOrigin) - D);

          float t = numerator/denominator;

          if (t < 0) {
            // Haven't hit yet
            j++;
            continue;
          }
          
          Vec3 p = rayOrigin.plus(rayDirection.times(t));

          if (t < maxT && pointLiesOnTriangle(p, vert1, vert2, vert3, e1, e2)) {
            CollisionTrigger newTrig = collisionTrigger.copy();
            newTrig.onCollision(p, surfaceNormal.normalized());
            triggerCollection.add(newTrig);
            // remove particle
            removeParticleAtIndex(j);
            j--;
          }
          j++;
        }
      }
    }
  }
  
  public void updateParticleProperties(float dt) {
    // update the color, lifespan, etc of the particles.
    for(int i = 0; i < particleCount; i++) {
      particleLifespan.set(i, particleLifespan.get(i) - dt);
    }
  }
  
  public void removeDeadParticles() {
    // any particles that are older than the particle lifespan
    // should be popped off the list
    int i = 0;
    while(i < particleCount) {
      if(particleLifespan.get(i) < 0.0f) {
        removeParticleAtIndex(i);
        i--;
      }
      i++;
    }
    if (particleCount == 0 && emitterElapsedTime >= emitterLifespan) {
      isActive = true;
    }
  }

  public void removeParticleAtIndex(int i) {
    particleCoords.remove(i);
    previousParticleCoords.remove(i);
    particleVelocities.remove(i);
    particleAccelerations.remove(i);
    particleLifespan.remove(i);
    particleColors.remove(i);
    particleRadii.remove(i);
    particleStreakLength.remove(i);
    particleCount--;
  }

  
  public ArrayList<Vec3> getParticleCoords() {
    return particleCoords;
  }

  public ArrayList<Vec3> getParticleColors() {
    return particleColors;
  }
  
  public ArrayList<Float> getParticleRadii() {
    return particleRadii;
  }

  public void drawAllParticles() {
    hint(ENABLE_DEPTH_SORT);
    while (drawNextParticle()) {

    }
    hint(DISABLE_DEPTH_SORT);
  }

  public boolean drawNextParticle() {
    if (partIdx < particleCount) {
      push();
      stroke(particleColors.get(partIdx).x, particleColors.get(partIdx).y, particleColors.get(partIdx).z);
      strokeWeight(particleRadii.get(partIdx));
      Vec3 vel = new Vec3(particleVelocities.get(partIdx));
      float velMagnitude = vel.length();
      vel.normalize();
      line(particleCoords.get(partIdx).x,
         particleCoords.get(partIdx).y,
         particleCoords.get(partIdx).z,
         particleCoords.get(partIdx).x - streakLength*vel.x*velMagnitude/70,
         particleCoords.get(partIdx).y - streakLength*vel.y*velMagnitude/70,
         particleCoords.get(partIdx).z - streakLength*vel.z*velMagnitude/70);
      pop();
      if (particleTexture == null) {
        push();
        stroke(particleColors.get(partIdx).x, particleColors.get(partIdx).y, particleColors.get(partIdx).z);
        strokeWeight(particleRadii.get(partIdx));
        point(particleCoords.get(partIdx).x, particleCoords.get(partIdx).y, particleCoords.get(partIdx).z);
        pop();
      } else {
        push();
        noStroke();
        translate(particleCoords.get(partIdx).x, particleCoords.get(partIdx).y, particleCoords.get(partIdx).z);
        Vec3 camToParticle = particleCoords.get(partIdx).minus(cam.camLocation).normalized();
        Vec2 camToParticle2D = new Vec2(camToParticle.x, camToParticle.z).normalized();
        float angle = atan2(camToParticle2D.y,camToParticle2D.x);
        rotateY (-angle-3*PI/2);
        beginShape();
        texture(particleTexture);
        float width_2 = particleRadii.get(partIdx);
        vertex(-width_2,width_2,0,0,0);
        vertex(width_2,width_2,0,particleTexture.width,0);
        vertex(width_2,-width_2,0,particleTexture.width,particleTexture.height);
        vertex(-width_2,-width_2,0,0,particleTexture.height);
        endShape();
        pop();
      }
      partIdx++;
      return true;
    }
    else {
      partIdx = 0;
      return false;
    }
  }

  public void updateTriggers(float dt) {
    triggerCollection.updateAllTriggers(dt);
  }

  public void drawTriggers() {
    triggerCollection.drawAllTriggers();
  }
}


public class PlanarParticleSystem extends ParticleSystem {
  Vec3 emitterPlaneNormal = new Vec3(0,1,0);
  float emitterRadius = 10;

  public PlanarParticleSystem(int maxParticleCount) {
    super(maxParticleCount);
  }

  @Override
  public void generateNewParticles(float dt) {
    // given the change in time, (assuming since the last particle generation)
    // generate more particles and add to the ArrayList of existing particles.
    // Initialize the new particles time, velocities, accelerations, lifespan etc.
    if (emitterLifespan > 0 && emitterElapsedTime >= emitterLifespan) {
      birthRate = 0;
      isActive = false;
      return;
    }
    emitterElapsedTime += dt;
    float newParticlesToGen = dt * birthRate;
    float decimal = newParticlesToGen - (float)(floor(newParticlesToGen));
    int stochasticNewParticles = floor(newParticlesToGen);
    if (decimal*100 > random(100)) {
      stochasticNewParticles++;
    }
    if (maxParticleCount < particleCount + stochasticNewParticles) {
      stochasticNewParticles = maxParticleCount - particleCount;
    }
    for(int i = 0; i < stochasticNewParticles; i++) {
      float R = emitterRadius*sqrt(random(0,1));
      float theta = random(0,1) * 2 * PI;
      Vec3 radialPosition = new Vec3(R*cos(theta), 0, R*sin(theta));
      float d = dot(emitterPosition.times(-1), emitterPlaneNormal);
      float k = -(dot(emitterPlaneNormal, radialPosition) + d);
      Vec3 projPointOntoPlane = radialPosition.plus(emitterPlaneNormal.times(k));
      previousParticleCoords.add(emitterPosition.plus(projPointOntoPlane));
      particleCoords.add(emitterPosition.plus(projPointOntoPlane));
      Vec3 particleDir = new Vec3(particleDirection);
      Vec3 randomParticleDir = new Vec3(random(-particleDirectionRange,particleDirectionRange),
                                  random(-particleDirectionRange,particleDirectionRange),
                                  random(-particleDirectionRange,particleDirectionRange));
      particleDir.add(randomParticleDir);
      particleDir.normalize();
      particleVelocities.add(particleDir.times(particleSpeed + random(-speedRange, speedRange)));
      Vec3 particleAccel = new Vec3(particleAcceleration);
      Vec3 randomParticleAccel = new Vec3(random(-particleAccelerationRange,particleAccelerationRange),
                                          random(-particleAccelerationRange,particleAccelerationRange),
                                          random(-particleAccelerationRange,particleAccelerationRange));
      particleAccel.add(randomParticleAccel);
      particleAccelerations.add(particleAcceleration);
      float randLifespan = random(particleLifespanMin, particleLifespanMax);
      particleLifespan.add(randLifespan);
      particleColors.add(new Vec3(particleColor));
      particleRadii.add(r);
      float startingStreakLength = 0.0f;
      particleStreakLength.add(startingStreakLength);
      particleCount++;
    }
  }
}
// Max Omdal 2020

class PathTraversal {
    int nextNodeIdx = 1;
    ArrayList<Integer> path;
    float distanceToNextNode = 0;
    Vec2 startPos;
    Vec2 goalPos;
    boolean destinationReached = false;
    Vec2 movementDir;

    public void setPath(Vec2 startPos, Vec2 goalPos) {
        this.startPos = new Vec2(startPos);
        this.goalPos = new Vec2(goalPos);
        this.path = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
        this.nextNodeIdx = 1;
        this.movementDir = nodePos[path.get(1)].minus(startPos);
        this.distanceToNextNode = movementDir.length();
        this.movementDir.normalize();
    }

    public Vec2 getNextNodeOnPath() {
        if (path != null) {
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

        if (distanceToNextNode <= 0.5f) {
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
        Cylinder body = new Cylinder(0.3f,2, this.position.plus(new Vec3(0, 1, 0)));
        body.materialColor = new Vec3(255,10,10);
        body.draw();

        for (int i = 0; i < 3; i++) {
            if (batCallActive[i]) {
                Cylinder focusRing = new Cylinder(0.9f, 0.1f, new Vec3(goalPositions[i].x, 0.05f, goalPositions[i].y));
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
        translate(position.x, position.y + 5 + 0.5f*cos(hoverBounce), position.z);
        Vec3 camToPlayer = position.minus(cam.camLocation).normalized();
        Vec2 camToPlayer2D = new Vec2(camToPlayer.x, camToPlayer.z).normalized();
        float angle = atan2(camToPlayer2D.y,camToPlayer2D.x);
        rotateY(-angle-3*PI/2);
        shader(unlitShader);
        beginShape();
        textureMode(NORMAL);
        texture(hoverPointer);
        vertex(-1,1.5f,0,0,0);
        vertex(1,1.5f,0,1,0);
        vertex(1,-1.5f,0,1,1);
        vertex(-1,-1.5f,0,0,1);
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
// Max Omdal 2020

class Probe {
    Vec3 position;
    Vec2 position2D;
    float speed = 10;
    Vec2 velocity;
    float collisionRadius = 1.5f;
    PShape model = createShape(BOX,1,1,1);
    PathTraversal pathMaker;
    float orientationAngle = 0;
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
            orientationAngle = orientationAngle + 0.05f * (2*PI - movementAngle - orientationAngle);
        } else {
            orientationAngle = orientationAngle + 0.05f * (movementAngle - orientationAngle);
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
// Max Omdal 2020
class Projectile {
    float speed = 30;
    PShape model = createShape(SPHERE, 0.5f);
    Ray3 dir;
    float maxTime = 0.5f;
    float time = 0;
    CollisionTrigger collisionTrigger;
    boolean isActive = true;
    Vec3 position;

    public Projectile(Ray3 dir) {
        collisionTrigger = new AnimateExplosion();
        this.dir = dir;
        this.position = dir.origin;
    }

    public void update(float dt) {
        time += dt;
        if (time >= maxTime) {
            if (collisionTrigger.isActive) {
                collisionTrigger.update(dt);
            } else {
                collisionTrigger.onCollision(position, new Vec3());
            }
        }
        this.isActive = time < maxTime || collisionTrigger.isActive;
    }

    public void draw() {
        if (time >= maxTime) {
            collisionTrigger.draw();
        } else {
            if (model != null) {
                this.position = dir.pointAtTime(time*speed);
                push();
                translate(position.x, position.y, position.z);
                shape(model);
                pop();
            }
        }
    }
}
class Ray3 {
    Vec3 origin;
    Vec3 direction;
    float magnitude = -1;
    Vec3 invDir;
    int[] sign = new int[3];
    
    public Ray3(Vec3 origin, Vec3 dir, float magnitude) {
        this.origin = origin;
        this.direction = dir.normalized();
        this.magnitude = magnitude;

        invDir = new Vec3(1.f/dir.x, 1.f/dir.y, 1.f/dir.z);
        sign[0] = PApplet.parseInt(invDir.x < 0);
        sign[1] = PApplet.parseInt(invDir.y < 0);
        sign[2] = PApplet.parseInt(invDir.z < 0);
    }

    public Ray3(Vec3 origin, Vec3 dir) {
        this.origin = origin;
        this.direction = dir.normalized();

        invDir = new Vec3(1.f/dir.x, 1.f/dir.y, 1.f/dir.z);
        sign[0] = PApplet.parseInt(invDir.x < 0);
        sign[1] = PApplet.parseInt(invDir.y < 0);
        sign[2] = PApplet.parseInt(invDir.z < 0);
    }

    public void debugDraw() {
        pushStyle();
        stroke(255,255,0);
        strokeWeight(1);
        if (magnitude > 0) line(origin.x, origin.y, origin.z, origin.x+direction.x*magnitude, origin.y+direction.y*magnitude, origin.z+direction.z*magnitude);
        else line(origin.x, origin.y, origin.z, origin.x+direction.x*10, origin.y+direction.y*10, origin.z+direction.z*10);
        popStyle();
    }

    public Vec3 pointAtTime(float t) {
        return origin.plus(direction.times(t));
    }
}
public Vec3 testForCollisions(Vec3 origin, Vec3 dir, PShape[] rigidBodies) {
 // Check each boid to see if it has collided with the collision meshes
 Vec3 closestPoint = null;
 float tMin = Float.MAX_VALUE;
        for (PShape rigidBody : rigidBodies) {
        int triCount = rigidBody.getChildCount();
        for (int i = 0; i < triCount; i++) {
            PShape triangle = rigidBody.getChild(i);
            int j = 0;
            Vec3 boidPosition = origin;

            Vec3 rayOrigin = boidPosition;
            Vec3 rayDirection = dir;
            rayDirection.normalize();

            PVector v1 = triangle.getVertex(0);
            PVector v2 = triangle.getVertex(1);
            PVector v3 = triangle.getVertex(2);

            Vec3 vert1 = new Vec3(v1.x, v1.y, v1.z);
            Vec3 vert2 = new Vec3(v2.x, v2.y, v2.z);
            Vec3 vert3 = new Vec3(v3.x, v3.y, v3.z);

            Vec3 e1 = vert2.minus(vert1);
            Vec3 e2 = vert3.minus(vert1);

            Vec3 surfaceNormal = cross(e1, e2);
            // float x_0 = rayOrigin.x; float y_0 = rayOrigin.y; float z_0 = rayOrigin.z;
            // float x_d = rayDirection.x; float y_d = rayDirection.y; float z_d = rayDirection.z;
            float denominator = dot(surfaceNormal, rayDirection);
            if (abs(denominator) <= 0.0001f) {
                // No ray plane intersection exists
                continue;
            }

            float D = dot(vert1, surfaceNormal);

            float numerator = -(dot(surfaceNormal, rayOrigin) - D);

            float t = numerator/denominator;

            if (t < 0) {
                // Haven't hit yet
                continue;
            }
            
            Vec3 p = rayOrigin.plus(rayDirection.times(t));

            if (t < tMin && pointLiesOnTriangle(p, vert1, vert2, vert3, e1, e2)) {
                closestPoint = p;
                t = tMin;
            }
            
        }
        }
    return closestPoint;
}
public Ray3 getMouseCast() {
  Vec3 w = cam.camLookAt.minus(cam.camLocation).normalized();
  Vec3 u = cross(w, cam.camUp).normalized();
  Vec3 v = cross(u, w).normalized();

  w.mul(-1);

  float m3dx = map(mouseX, 0, width, -cam.nearPlaneW/2, cam.nearPlaneW/2);
  float m3dy = map(mouseY, 0, height, -cam.nearPlaneH/2, cam.nearPlaneH/2);
  float m3dz = -1;

  float m3dx_world = m3dx*u.x + m3dy*v.x + m3dz*w.x + cam.camLocation.x;
  float m3dy_world = m3dx*u.y + m3dy*v.y + m3dz*w.y + cam.camLocation.y;
  float m3dz_world = m3dx*u.z + m3dy*v.z + m3dz*w.z + cam.camLocation.z;

  Vec3 m_world = new Vec3(m3dx_world, m3dy_world, m3dz_world);
  Vec3 rayDir = m_world.minus(cam.camLocation);
  rayDir.normalize();
  return new Ray3(cam.camLocation, rayDir);
}

public void keyPressed() {
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
    if (keyCode==LEFT)
        keys[4]=true;
    if (keyCode==RIGHT)
        keys[5]=true;
    if (keyCode==UP)
        keys[6]=true;
    if (keyCode==DOWN)
        keys[7]=true;
    if (keyCode==ENTER) {
        // Set paths for bats
        for (int i = 0; i < paths.length; i++) {
            paths[i] = planPath(startPositions[i], goalPositions[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
            if (paths[i] != null && paths[i].size() > 1) {
                player.batCallActive[i] = true;
                distanceToNextNode[i] = nodePos[paths[i].get(1)].minus(startPositions[i]).length();
                posAlongPath[i] = new Vec2(nodePos[paths[i].get(0)]);
                bats.tetherPoints[i] = new Vec3(posAlongPath[i].x, 5, posAlongPath[i].y);
            }
        }
        // Set path for joker
        joker.startMove();
    }
    if (key=='q') {
        // Drop a new bat signalling device
        goalPositions[nextSpawnIndex] = new Vec2(player.position2D);
        nextSpawnIndex++;
        if (nextSpawnIndex > 2) nextSpawnIndex = 0;
    }
}

public void keyReleased() {
    if (key=='w')
        keys[0]=false;
    if (key=='a')
        keys[1]=false;
    if (key=='s')
        keys[2]=false;
    if (key=='d')
        keys[3]=false;
    if (keyCode==LEFT)
        keys[4]=false;
    if (keyCode==RIGHT)
        keys[5]=false;
    if (keyCode==UP)
        keys[6]=false;
    if (keyCode==DOWN)
        keys[7]=false;
}

public void mouseClicked() {
    Ray3 mouseCast = getMouseCast();
    Vec3 groundCenter = new Vec3(0,-5,0);
    Vec3 groundNormal = new Vec3(0,1,0);
    float groundCastIntersectTime = dot(groundCenter.minus(cam.camLocation), groundNormal)/dot(mouseCast.direction, groundNormal);
    Vec3 intersectPoint = mouseCast.pointAtTime(groundCastIntersectTime);
    if (mouseButton == RIGHT) {
        // Launch projectile
        player.throwProjectile(intersectPoint.minus(player.position));
    } else if (mouseButton == LEFT) {
        // Launch probe
        // Get Nearest node
        Vec2 intersectPoint2D = new Vec2(intersectPoint.x, intersectPoint.z);
        player.launchProbe(intersectPoint2D);
    }
}
//Vector Library [2D]
//CSCI 5611 Vector 3 Library [Incomplete]

//Instructions: Add 3D versions of all of the 2D vector functions
//              Vec3 must also support the cross product.
public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }

  public Vec2(Vec2 v) {
    this.x = v.x;
    this.y = v.y;
  }
  
  public String toString(){
    return "(" + x + ", " + y + ")";
  }
  
  public float length(){
    return sqrt(x*x + y*y);
  }

  public float lengthSqr() {
    return (x*x + y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(rhs.x+this.x, rhs.y+this.y);
  }
  
  public void add(Vec2 rhs){
    this.x += rhs.x;
    this.y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(this.x-rhs.x, this.y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    this.x -= rhs.x;
    this.y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(this.x*rhs, this.y*rhs);
  }
  
  public void mul(float rhs){
    this.x *= rhs;
    this.y *= rhs;
  }
  
  public void normalize(){
    float magnitude = this.length();
    this.x /= magnitude;
    this.y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = this.length();
    return new Vec2(this.x/magnitude, this.y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    return this.minus(rhs).length();
  }
}

public Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

public float interpolate(float a, float b, float t) {
   return a + (b - a)*t; 
}

public float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

public Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(dot(a, b));
}
//Vector Library [2D]
//CSCI 5611 Vector 3 Library [Incomplete]

//Instructions: Add 3D versions of all of the 2D vector functions
//              Vec3 must also support the cross product.
public class Vec3 {
  public float x, y, z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public Vec3(Vec3 copyVec) {
    this.x = copyVec.x;
    this.y = copyVec.y;
    this.z = copyVec.z;
  }

  public Vec3() {
    this.x = 0;
    this.y = 0;
    this.z = 0;
  }

  public Vec3(PVector v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
  }

  public boolean equals(Vec3 v) {
    return this.x == v.x && this.y == v.y && this.z == v.z;
  }
  
  public String toString(){
    return "(" + x + ", " + y + ", " + z + ")";
  }
  
  public float length(){
    if (x == 0 && y == 0 && z == 0) return 0.f;
    return sqrt(x*x + y*y + z*z);
  }
  
  public Vec3 plus(Vec3 rhs){
    return new Vec3(rhs.x+this.x, rhs.y+this.y, rhs.z+this.z);
  }
  
  public void add(Vec3 rhs){
    this.x += rhs.x;
    this.y += rhs.y;
    this.z += rhs.z;
  }
  
  public Vec3 minus(Vec3 rhs){
    return new Vec3(this.x-rhs.x, this.y-rhs.y, this.z-rhs.z);
  }
  
  public void subtract(Vec3 rhs){
    this.x -= rhs.x;
    this.y -= rhs.y;
    this.z -= rhs.z;
  }
  
  public Vec3 times(float rhs){
    return new Vec3(this.x*rhs, this.y*rhs, this.z*rhs);
  }

  public Vec3 times(Vec3 v) {
    return new Vec3(this.x*v.x, this.y*v.y, this.z*v.z);
  }
  
  public void mul(float rhs){
    this.x *= rhs;
    this.y *= rhs;
    this.z *= rhs;
  }
  
  public void normalize(){
    float magnitude = this.length();
    this.x /= magnitude;
    this.y /= magnitude;
    this.z /= magnitude;
  }
  
  public Vec3 normalized(){
    float magnitude = this.length();
    assert magnitude > 0 : "zero magnitude";
    return new Vec3(this.x/magnitude, this.y/magnitude, this.z/magnitude);
  }
  
  public float distanceTo(Vec3 rhs){
    return this.minus(rhs).length();
  }

  public void clamp(float minLength, float maxLength) {
    float curLength = length();
    if (curLength > maxLength) {
      this.normalize();
      this.mul(maxLength);
    } else if (curLength < minLength) {
      this.normalize();
      this.mul(minLength);
    }
  }
}

public Vec3 interpolate(Vec3 a, Vec3 b, float t){
  return a.plus((b.minus(a)).times(t));
}

public float dot(Vec3 a, Vec3 b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

public Vec3 cross(Vec3 a, Vec3 b){
  return new Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

public Vec3 projAB(Vec3 a, Vec3 b){
  return b.times(dot(a, b));
}

public Vec3 reflect(Vec3 d, Vec3 n) {
  Vec3 r = d.minus(n.times(dot(d,n.normalized())*2));
  return r;
}

public boolean pointLiesOnTriangle(Vec3 point, Vec3 vert1, Vec3 vert2, Vec3 vert3, Vec3 e1, Vec3 e2) {
  // See if point on a plane is within a triangle
  // Vec3 ep = point.minus(vert1);
  // float d11 = dot(e1,e1);
  // float d12 = dot(e1,e2);
  // float d22 = dot(e2,e2);
  // float dp1 = dot(ep,e1);
  // float dp2 = dot(ep,e2);
  // float D       = d11*d22 - d12*d12;
  // float D_beta  = d22*dp1 - d12*dp2;
  // float D_gamma = d11*dp2 - d12*dp1;
  // float beta = D_beta/D;
  // float gamma = D_gamma/D;
  // float alpha = 1 - beta + gamma;
  // print(alpha);
  // print(" ");
  // print(beta);
  // print(" ");
  // print(gamma);
  // println();
  // return (alpha > 0.0000001 && alpha < 1.0000001);

  // Source inspired by Scratchapixel:
  // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates

  Vec3 surfaceNormal = cross(e1, e2);
  Vec3 C;

  Vec3 edge0 = vert2.minus(vert1);
  Vec3 vp0 = point.minus(vert1);
  C = cross(edge0, vp0);
  if (dot(surfaceNormal, C) < 0) { return false; }

  Vec3 edge1 = vert3.minus(vert2);
  Vec3 vp1 = point.minus(vert2);
  C = cross(edge1, vp1);
  if (dot(surfaceNormal, C) < 0) { return false; }

  Vec3 edge2 = vert1.minus(vert3);
  Vec3 vp2 = point.minus(vert3);
  C = cross(edge2, vp2);
  if (dot(surfaceNormal, C) < 0) { return false; }

  return true;
}
  public void settings() {  size(1280, 960, P3D); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "PathPlanningProcessing" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
