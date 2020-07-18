// Max Omdal 2020
class Projectile {
    float speed = 30;
    PShape model = createShape(SPHERE, 0.5);
    Ray3 dir;
    float maxTime = 0.5;
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