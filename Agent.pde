// Max Omdal 2020

class Agent {
    Vec3 position;
    Vec2 position2D;
    float speed = 20;
    Vec2 velocity = new Vec2();
    Vec2 goalVelocity = new Vec2();
    float goalForceStrength = 5;
    float collisionRadius = 2;
    float timeHorizon = 4;
    float maxForce = 1000;
    Vec2 forces = new Vec2();
    PathTraversal pathMaker;
    float orientationAngle = 0;

    public Agent() {
    }
}

void avoidAgents(ArrayList<Agent> agents) {
    for (int i = 0; i < agents.size(); i++) {
        Agent a = agents.get(i);
        Vec2 forceToGoal = a.goalVelocity.minus(a.velocity).times(a.goalForceStrength);
        a.forces = forceToGoal;
        for (int j = 0; j < agents.size(); j++) {
            if (j == i) {
                continue;
            }
            Agent b = agents.get(j);
            float t = ttc(a, b);
            if (t == Float.MAX_VALUE) {
                continue;
            }
            Vec2 avoidanceDir = a.position2D.plus(a.velocity.times(t)).minus(b.position2D.plus(b.velocity.times(t)));
            if (avoidanceDir.length() != 0) {
                avoidanceDir.normalize();
            }
            float mag = 0;
            if (t >= 0 && t <= a.timeHorizon)
                mag = (a.timeHorizon-t)/(t+0.001);
            if (mag > a.maxForce)
                mag = a.maxForce;
            Vec2 avoidanceForce = avoidanceDir.times(mag);
            a.forces.add(avoidanceForce);
        }
    }
}

float ttc(Agent agent_1, Agent agent_2) {
    Vec2 w = agent_1.position2D.minus(agent_2.position2D);
    Vec2 v = agent_2.velocity.minus(agent_1.velocity);
    float a = dot(v,v);
    float b = dot(w,v);
    float c = dot(w,w) - pow(agent_2.collisionRadius+agent_1.collisionRadius, 2);
    if (c < 0) {
        return 0;
    }
    float discr = b*b - a*c;
    if (discr <= 0) {
        return Float.MAX_VALUE;
    }
    float timeToCollision = (b - sqrt(discr))/a;
    if (timeToCollision < 0) {
        return Float.MAX_VALUE;
    }
    return timeToCollision;
}