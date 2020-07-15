
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
  float r = 0.2;
  float particleSpeed = 200;
  float speedRange = 50;
  Vec3 particleDirection = new Vec3(0,-1,0);
  float particleDirectionRange = 0.07;
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
      Vec3 translation = vel.times(dt).plus(accel.times(0.5).times(dt*dt));
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
      if(particleLifespan.get(i) < 0.0) {
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
        stroke(particleColors.get(partIdx).x, particleColors.get(partIdx).y, particleColors.get(partIdx).z);
        strokeWeight(particleRadii.get(partIdx));
        point(particleCoords.get(partIdx).x, particleCoords.get(partIdx).y, particleCoords.get(partIdx).z);
      } else {
        push();
        noStroke();
        translate(particleCoords.get(partIdx).x, particleCoords.get(partIdx).y, particleCoords.get(partIdx).z);
        rotateY (-radians(theta+270));
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

  public void drawTriggers(float dt) {
    triggerCollection.drawAllTriggers(dt);
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
