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
        emitter.particleLifespanMax = 0.3;
        emitter.particleLifespanMin = 0.2;
        emitter.birthRate = 10;
        emitter.emitterLifespan = 0.1;
        emitter.r = 1.3;
        emitter.particleSpeed = 20;
        emitter.speedRange = 10;
        emitter.particleDirection = new Vec3(0,1,0);
        emitter.particleDirectionRange = 0.1;
        emitter.particleAcceleration = new Vec3(0,-500,0);
        emitter.particleAccelerationRange = 0.1;
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
        explosion.particleLifespanMax = 1.0;
        explosion.particleLifespanMin = 0.7;
        explosion.birthRate = 200;
        explosion.emitterLifespan = 0.2;
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