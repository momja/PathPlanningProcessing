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
        this.visualDistance = 0.9;
        this.influenceToTetherPoint = 1.5;
        this.separation = 1.5;
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