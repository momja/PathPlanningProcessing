import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

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

public void setup() {
    
    cam.setPerspective();
    surface.setTitle("Path Planning [Max Omdal]");
    keys = new boolean[4];
    keys[0] = false; keys[1] = false; keys[2] = false; keys[3] = false;

    groundTexture = loadImage("concrete_floor.png");
    unlitShader = loadShader("unlit_frag.glsl", "unlit_vert.glsl");

    createObstacles();
    generateRandomNodes(numNodes, circlePos, circleRad);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
    startPos = nodePos[0];
    player = new Player(new Vec3(startPos.x, -5, startPos.y));

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

    if (debugMode) {
        drawPRMGraph();
    }

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
    if (keyCode==ENTER) {
        player.batCallActive = true;
        goalPos = new Vec2(player.position.x, player.position.z);
        path = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
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
}

public void update(float dt) {
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

public void createObstacles() {
    for (int i = 0; i < numObstacles; i++) {
        float radius = random(3,5);
        float height = random(5,8);
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
class Camera {
    Vec3 camLocation = new Vec3(0,0,0);
    Vec3 camLookAt = new Vec3(0,0,0);
    Vec3 camUp = new Vec3(0,-1,0);
    float radius = 40;
    int slider = 80;
    float theta = 0;
    float fov = 55;
    float nearPlaneW = 1 + 1.f/3;
    float nearPlaneH = 1;
    float nearPlaneDist = 1;
    float farPlaneDist = 1000;

    public void update() {
        if (keyPressed) {
            if (keyCode == UP) {
                slider++;
            } else if (keyCode == DOWN) {
                slider--;
            } else if (keyCode == LEFT) {
                theta -= 0.8f;
            } else if (keyCode == RIGHT) {
                theta += 0.8f;
            }
        }
        camLocation.x = cos(radians(theta))*radius;
        camLocation.y = PApplet.parseFloat(slider)/5;
        camLocation.z = sin(radians(theta))*radius;
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
// Max Omdal 2020

class Cylinder {
    float radius;
    float height;
    Vec3 center;
    float sides = 20;
    Vec3 materialColor = new Vec3(255,255,255);

    public Cylinder(float radius, float height, Vec3 center) {
        this.radius = radius;
        this.height = height;
        this.center = center;
    }

    public void draw() {
        float theta = 0;
        float stepSize = 2*PI/sides;
        push();
        noStroke();
        fill(materialColor.x, materialColor.y, materialColor.z);
        // Draw the sides of the cylinder
        beginShape(QUAD_STRIP);
        for (int i = 0; i < sides+1; i++) {
            theta += stepSize;
            Vec3 vertexPos = new Vec3(this.center.x + this.radius * cos(theta),
                                    this.center.y + this.height/2,
                                    this.center.z + this.radius * sin(theta));
            Vec3 vecNormal = center.minus(vertexPos).normalized();
            normal(vecNormal.x, vecNormal.y, vecNormal.z);
            vertex(vertexPos.x, vertexPos.y, vertexPos.z);
            vertex(vertexPos.x, vertexPos.y - this.height, vertexPos.z);
        }
        endShape(CLOSE);
        
        // Draw the caps
        theta = 0;
        beginShape();
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            vertex(this.center.x + this.radius * cos(theta),
                   this.center.y + this.height/2,
                   this.center.z + this.radius * sin(theta));
        }
        endShape();

        theta = 0;
        beginShape();
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            vertex(this.center.x + this.radius * cos(theta),
                   this.center.y - this.height/2,
                   this.center.z + this.radius * sin(theta));
        }
        endShape();

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
  // Set up neighbors
  if (pointInCircleList(centers, radii, numObstacles, startPos) || pointInCircleList(centers, radii, numObstacles, goalPos)) {
    path.add(0,-1);
    return path;
  }
  for (int i = 0; i < numNodes; i++) {
    Vec2 startToNode = nodePos[i].minus(startPos);
    if (!rayCircleListIntersect(centers, radii, numObstacles, startPos, startToNode.normalized(), startToNode.length()).hit) {
      neighbors[i].add(startID);
      neighbors[startID].add(i);
    }
    Vec2 nodeToGoal = goalPos.minus(nodePos[i]);
    if (!rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], nodeToGoal.normalized(), nodeToGoal.length()).hit) {
      neighbors[i].add(goalID);
      neighbors[goalID].add(i);
    }
  }

  path = runAStar(nodePos, numNodes, startID, goalID);
  
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
    strokeWeight(1);
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
       Cylinder body = new Cylinder(0.3f,2, this.position.plus(new Vec3(0, 1, 0)));
       body.materialColor = new Vec3(255,10,10);
       body.draw();

       if (batCallActive) {
           Cylinder focusRing = new Cylinder(0.9f, 0.1f, this.position.plus(new Vec3(0,0.05f,0)));
           focusRing.materialColor = new Vec3(200,200,0);
           focusRing.draw();
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
