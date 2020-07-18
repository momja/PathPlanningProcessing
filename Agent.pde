// Max Omdal 2020

class Agent {
    Vec3 position;
    Vec2 position2D;
    float speed = 20;
    Vec2 velocity;
    Vec2 goalVelocity;
    float goalForceStrength = 2;
    float collisionRadius = 2;
    float timeHorizon = 2;
    float maxForce = 10;
    Vec2 forces;
    PathTraversal pathMaker;
    float orientationAngle = 0;

    public Agent(Vec3 position) {
        this.position = position;
        this.position2D = new Vec2(position.x, position.z);
    }

    static void avoidAgents(ArrayList<Agent> agents) {
        Vec2[] finalForces = new Vec2[agents.size()];
        for (int i = 0; i < agents.size(); i++) {
            Agent a = agents.get(i);
            finalForces[i] = new Vec2();
            Vec2 totalForce = new Vec3();
            Vec2 forceToGoal = a.goalVelocity.minus(a.velocity).times(a.goalForceStrength);
            totalForce.add(a.forceToGoal);
            for (int j = 0; j < agents.size(); j++) {
                Agent b = agents.get(j);
                float t = ttc(a, b);
                Vec2 avoidanceDir = a.position2D.plus(a.velocity.times(t)).minus(b.position2D.plus(b.velocity.times(t)));
                if (avoidanceDir.length() != 0) {
                    avoidanceDir.normalize();
                }
                float mag;
                if (t <= a.timeHorizon)
                    mag = (a.timeHorizon-t)/(t+0.001);
                if (mag > a.maxForce)
                    mag = a.maxForce;
                Vec2 avoidanceForce = avoidanceDir.times(mag);
                finalForces[i].add(avoidanceForce);
            }
        }

        for (Agent agent : agents) {
            agent.forces
        }
    }

    static float ttc(Agent a, Agent b) {
        Vec2 w = a.position2D.minus(b.position2D);
        Vec2 v = b.velocity.minus(a.velocity);
        float a = dot(v,v);
        float b = -dot(w,v);
        float c = dot(w,w) - pow(b.collisionRadius+a.collisionRadius, 2);
        float discr = b*b - a*c
        if (discr <= 0) {
            return Float.MAX_VALUE;
        }
        Vec2 timeToCollision = (b - sqrt(discr))/a
        if (timeToCollision < 0) {
            return Float.MAX_VALUE;
        }
        return timeToCollision;
    }
}