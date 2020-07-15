public class CollisionTrigger {
    boolean isActive = false;

    public void onCollision(Vec3 point, Vec3 normal) {
        isActive = true;
        return;
    }

    public CollisionTrigger copy() {
        return new CollisionTrigger();
    }

    public void draw(float dt) {
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

    public void draw(float dt) {
        emitter.generateNewParticles(dt);
        emitter.updateParticlePositions(dt);
        emitter.updateParticleProperties(dt);
        emitter.removeDeadParticles();
        while (emitter.drawNextParticle()) {
        }
        emitter.drawTriggers(dt);
        this.isActive = emitter.isActive;
    }
}

public class AnimateRaindropCollision extends CollisionTrigger {
    @Override
    public void onCollision(Vec3 point, Vec3 normal) {
        // TODO : Spawn a textured quad that animates through a series
        // of raindrop splash images
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

    public void drawAllTriggers(float dt) {
        int i = 0;
        while (i < triggers.size()) {
            CollisionTrigger trigger = triggers.get(i);

            trigger.draw(dt);

            if (!trigger.isActive) {
                // Remove any inactive triggers
                triggers.remove(i);
                i--;
            }
            i++;
        }
    }
}