public class BoidSystem {
    // private ArrayList<Vec3> boidCoords;
    // private ArrayList<Vec3> boidPrevCoords;
    // // private ArrayList<Quaternion> boidOrientations;
    // private ArrayList<Vec3> boidVelocities;
    // private ArrayList<Float> perchedBoidCountdowns;
    private ArrayList<Boid> boids;
    private float width, height, length;

    Vec3 boundingBoxOrigin = new Vec3(0,25,0);
    float minX, maxX, minY, maxY, minZ, maxZ;

    int boidCount = 100;
    float boidSize = 2;
    float maxSpeed = 100;
    float separation = 2;
    float visualDistance = 6;
    float influenceToCenter = 0.02;
    Vec3 boidColor = new Vec3(0,0,0);
    PImage boidTexture = null;
    PShape boidModel = null;
    PShape boidPerchedModel = null;
    Vec3 tetherPoint = new Vec3(8,5,-4);
    float influenceToTetherPoint = 0.03;

    public BoidSystem(int boidCount) {
        // boidCoords = new ArrayList<Vec3>();
        // boidPrevCoords = new ArrayList<Vec3>();
        // // boidOrientations = new ArrayList<Quaternion>();
        // boidVelocities = new ArrayList<Vec3>();
        // perchedBoidCountdowns = new ArrayList<Float>();
        boids = new ArrayList<Boid>();
        setBoundingBox(boundingBoxOrigin, 100, 60, 100);
        this.boidCount = boidCount;
        initializePositions();
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

    public void checkForCollisions(PShape[] rigidBodies) {
        // Check each boid to see if it has collided with the collision meshes
        for (PShape rigidBody : rigidBodies) {
        int triCount = rigidBody.getChildCount();
        for (int i = 0; i < triCount; i++) {
            PShape triangle = rigidBody.getChild(i);
            int j = 0;
            while(j < boidCount) {
                Boid boid = boids.get(j);
                Vec3 boidPosition = boid.coords;
                if (boid.perchCountdown > 0 || boidPosition.y > 5 || boidPosition.x > 20 || boidPosition.x < -20 || boidPosition.z > 20 || boidPosition.z < -20) {
                    j++;
                    continue;
                }
                // TODO: Use Barycentric Coordinates to find if there is a collision with the surface
                boolean collision = false;
                Vec3 collisionPoint = new Vec3(0,0,0);

                Vec3 rayOrigin = boidPosition;
                Vec3 rayDirection = boidPosition.minus(boid.previousCoord);
                float maxT = rayDirection.length();

                if (maxT < 0.00001) {
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
                if (abs(denominator) <= 0.0001) {
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
                    perchBoid(j, p, reflect(rayDirection, surfaceNormal));
                }
                j++;
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

    public void initializePositions() {
        // Given the bounding box, we will randomly place our boids within this box
        for (int i = 0; i < boidCount; i++) {
            Vec3 position = new Vec3(random(minX, maxX),
                                     random(minY, maxY)/2,
                                     random(minZ, maxZ));
            Boid boid = new Boid();
            boid.coords = position;
            boid.previousCoord = position;
            boid.perchCountdown = 0.f;

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
            boid.velocity.add(v1.times(0.6).plus(v2.times(0.6)).plus(v3).plus(v4));
            boid.velocity.y *= 0.9;
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
                if (pos.distanceTo(camLocation) > 30) {
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
                stroke(255);
                strokeWeight(boidSize);
                point(pos.x, pos.y, pos.z);
                pop();
            }
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
}